/*
 * FUXI touch: ST FTS (FTM5) on QUP1 SE4, SPI through QUP1 GPI DMA (polled).
 * Hardware facts and protocol: research/touch/hw-report.md; first proven in
 * the loader (shim/fuxi-probe/probe.c request 0x55). The loader votes the
 * RPMh rails (ldob12 vdd 1.8 V, ldom6 avdd 3.2 V) before the XNU handoff;
 * this driver does not talk to RPMh. SE4 is GSI-only (GENI_IF_DISABLE_RO),
 * so transfers use GPII 0 (TX channel 0, RX channel 1, one event ring), with
 * apps-smmu SMRs for streams 0xb6 (GPI) and 0xa3 (QUP1) routed to context
 * bank 4 (stage 1 off), as the loader does (research/touch/gpi-recipe.md,
 * ts-12). It takes the IC through its reset,
 * waits for the controller-ready event, enables multi-touch scanning and
 * polls the event FIFO while GPIO25 (IRQ, active low) is asserted.
 * No firmware upload: the FTS keeps its firmware in its own flash.
 *
 * Polling (0093): every 4 ms while a finger is down or the IRQ line was
 * seen recently, every 8 ms when idle and every 32 ms while the display is
 * off (AppleCLCD RequestPowerChange(0)); deadlines are absolute and use the
 * critical timer urgency, so no coalescing leeway is added. Touches are not
 * reported while the display is off (boot-arg fuxitouch_offsuppress=0 keeps
 * reporting). Boot-arg fuxitouch_poll_us sets the active period (default
 * 4000), fuxitouch_idle_us the idle one (default 8000).
 */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/errno.h>
#include <kern/clock.h>
#include <kern/thread.h>
#include <kern/sched_prim.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>
#include <vm/vm_kern.h>
#include <vm/pmap.h>
#include <arm/caches_internal.h>

extern vm_offset_t ml_io_map(vm_offset_t, vm_size_t);
extern kern_return_t thread_terminate(thread_t);
#include <sys/macan_input.h>
extern void redwood_touch_post(uint32_t type, uint32_t finger, uint32_t x, uint32_t y);
extern PE_state_t PE_state;
extern ppnum_t pmap_find_phys(pmap_t pmap, addr64_t va);
extern kern_return_t kernel_thread_start_priority(thread_continue_t continuation,
    void *parameter, integer_t priority, thread_t *new_thread);
static int sweet_fb_display_off;        /* no display-off gating on alor */
void alor_touch_init(void);

#define FT_SE_PA     0x00a90000u
/* GPI DMA instance that serves QUP1. The live device tree has
 *   /soc/qcom,gpi-dma@a00000   reg = <0x0 0xa00000 0x0 0x60000>
 * and the SPI master lists it as a supplier (a00000.qcom,gpi-dma).
 * This used to be 0xa10000 -- "ee base 0x9f0000 + 0x20000" -- which is not
 * what this board's DT says and lands inside the same 0x60000 window but at
 * the wrong register block. */
#define FT_GPI_PA    0x00a00000u
#define FT_GPI_SIZE  0x60000u
#define FT_SMMU_PA   0x15000000u
#define FT_GCC_PA    0x00100000u
/* Neo8 (alor/RMX8899) TLMM.
 *
 * GROUND TRUTH, read from the handset while Android was running with touch
 * fully working (so these values cannot be wrong):
 *
 *   /proc/iomem:
 *     0f000000-0fffffff : f000000.pinctrl pinctrl@f000000
 *
 *   /sys/devices/platform/soc/f000000.pinctrl/of_node/reg:
 *     00 00 00 00 0f 00 00 00  00 00 00 00 01 00 00 00
 *     = <0x0 0x0f000000 0x0 0x01000000>
 *   compatible = "qcom,alor-tlmm"
 *
 *   /sys/kernel/debug/pinctrl/f000000.pinctrl/pins:
 *     pin 0 (GPIO_0) ... pin 205 (UFS_RESET)   -> exactly 206 pins
 *   /sys/kernel/debug/gpio:
 *     gpiochip0: GPIOs 512-717, parent: platform/f000000.pinctrl
 *
 *   There is no offset property on the node, so pin n sits at
 *   f000000 + 0x1000*n. 206 pins end at 0x0f0cd000, comfortably inside the
 *   16 MiB window.
 *
 * This file previously flip-flopped between 0x0f000000 and 0x0f100000. The
 * 0x0f100000 claim came from a comment about the *downstream* pinctrl-canoe.ko
 * on SM8850, not from this board's DT, and it is contradicted twice over:
 * Android maps the resource at 0x0f000000, and the one capture where the
 * driver actually read these pads (logs/cap-105442, v73) was built on
 * 0x0f000000 and returned 48=0x1803 -- a valid pin CTL value (egpio_present
 * and egpio_enable set, pull-up). A hole would not return that.
 *
 * So: 0x0f000000 is the base. */
#define FT_TLMM_PA   0x0f000000u
#define FT_TLMM_SIZE 0x1000000u    /* the node's own reg size, all 206 pins */
#define FT_GPIO(n)   ((uint32_t)(n) * 0x1000u)
#define FT_PIN_MAX   206u          /* pin 205 is the last one that exists */
#define ALOR_PANEL_W 1272u
#define ALOR_PANEL_H 2772u
/* Neo8 touch pins (device tree, overlay fragment@53 focaltech@0):
 *   irq-gpio = <&tlmm 55 0x2002>   reset-gpio = <&tlmm 161 0x1>
 *   spi lanes gpio48..51 = qup1_se4_l0..l3, hardware mux function 1 */
#define FT_RESET     161u
#define FT_IRQ       55u
/* QUP1 SE4 lanes, live pinctrl confirms gpio48..51 = qup1_se4_l0..l3 */
#define FT_PIN_MISO  48u
#define FT_PIN_MOSI  49u
#define FT_PIN_CLK   50u
#define FT_PIN_CS    51u
#define FT_MAX       10u
#define FT_EV_READY  0x03u
#define FT_EV_ENTER  0x13u
#define FT_EV_MOTION 0x23u
#define FT_EV_LEAVE  0x33u
#define FT_TYPE_HOVER 0x05u        /* ftsSoftware.h TOUCH_TYPE_HOVER: not touching */
#define FT_PRIORITY  92            /* BASEPRI_PREEMPT */

static struct {
	vm_offset_t se, gcc, tlmm, gpi, smmu;
	vm_offset_t dma;
	uint64_t dma_pa;
	unsigned depth, level, wp[2], evrp;
	bool started;
	bool bb;                       /* transfer over bit-banged GPIO, not QUP */
	bool rails;                    /* L4B_E0 + L16B_E0 voted on */
	uint32_t active;
	int x[FT_MAX], y[FT_MAX];
	bool ic_reset;                 /* controller-ready event while sensing */
	/* Per-contact statistics (first finger down -> last finger up). */
	uint64_t c_start, c_last, c_poll_sum, c_poll_max;
	unsigned c_reports, c_polls, contacts;
} ft;
static int ft_chan;

static uint32_t
ft_rd(vm_offset_t base, uint32_t off)
{
	uint32_t v = *(volatile uint32_t *)(base + off);
	__asm__ volatile("dmb oshld" ::: "memory");
	return v;
}

static void
ft_wr(vm_offset_t base, uint32_t off, uint32_t v)
{
	__asm__ volatile("dmb oshst" ::: "memory");
	*(volatile uint32_t *)(base + off) = v;
	__asm__ volatile("dsb osh" ::: "memory");
}

static void
ft_sleep_until(uint64_t deadline)
{
	(void)assert_wait_deadline_with_leeway(&ft_chan, THREAD_UNINT,
	    TIMEOUT_URGENCY_SYS_CRITICAL, deadline, 0);
	thread_block(THREAD_CONTINUE_NULL);
}

static void
ft_sleep(unsigned ms)
{
	uint64_t deadline;
	clock_interval_to_deadline(ms, NSEC_PER_MSEC, &deadline);
	(void)assert_wait_deadline(&ft_chan, THREAD_UNINT, deadline);
	thread_block(THREAD_CONTINUE_NULL);
}

static volatile unsigned alor_stage, alor_done;
static unsigned alor_ws = 9;
static vm_offset_t alor_fb;
static uint32_t alor_fb_row;

static void
alor_blip_c(unsigned n, uint32_t colour)
{
	if (!alor_fb || n > 15) {
		return;
	}
	/* 64x64 on a 76-pixel pitch. The old 36x36 blocks sat under the console's
	 * own text and could not be resolved by a webcam, which made every
	 * framebuffer marker invisible in practice. */
	for (unsigned y = 0; y < 64; y++) {
		volatile uint32_t *p = (volatile uint32_t *)(alor_fb +
		    (vm_size_t)(60 + y) * alor_fb_row + (24 + 76 * n) * 4);
		for (unsigned x = 0; x < 64; x++) {
			p[x] = colour;
		}
	}
}

/* A 240x240 verdict square at the top right, green for success and red for
 * failure. The console font is far below what a webcam can resolve, so text on
 * the panel is unreadable from the host -- but a saturated block of colour is
 * unmistakable at this size. That makes the one fact this driver exists to
 * establish, "did the chip answer", visible without any log at all. */
static void
alor_verdict_c(uint32_t colour)
{
	if (!alor_fb) {
		return;
	}
	for (unsigned y = 0; y < 240; y++) {
		volatile uint32_t *p = (volatile uint32_t *)(alor_fb +
		    (vm_size_t)(60 + y) * alor_fb_row + 980 * 4);
		for (unsigned x = 0; x < 240; x++) {
			p[x] = colour;
		}
	}
}

static void
alor_verdict(bool ok)
{
	alor_verdict_c(ok ? 0x0000ff00u : 0x00ff0000u);
}

static void
alor_blip(unsigned n)
{
	alor_blip_c(n, 0x0000ff00u);
}

static void
alor_mark(unsigned n, bool ok)
{
	alor_blip_c(n, ok ? 0x0000ff00u : 0x00ff0000u);
}

static void
alor_fb_init(void)
{
	unsigned long base = PE_state.video.v_baseAddr;
	if (base && base < 0xffff000000000000ULL) {
		vm_size_t span = (vm_size_t)PE_state.video.v_rowBytes * 200u;
		if (span > (2u << 20)) {
			span = 2u << 20;
		}
		base = (unsigned long)ml_io_map((vm_offset_t)base, span);
	}
	if (base && PE_state.video.v_rowBytes) {
		alor_fb = (vm_offset_t)base;
		alor_fb_row = (uint32_t)PE_state.video.v_rowBytes;
	}
}

static void
alor_watchdog(void *arg, wait_result_t result)
{
	(void)arg;
	(void)result;
	ft_sleep(30000);
	if (!alor_done) {
		panic("ALOR TOUCH: stuck at stage %u", alor_stage);
	}
	(void)thread_terminate(current_thread());
	thread_block(THREAD_CONTINUE_NULL);
}

/* Panel -> HID units: inject.c divides by 65535, so scale the 1272x2772 panel. */
static uint32_t alor_shim_mask;
static void
alor_post(unsigned id, uint32_t type)
{
	int x = ft.x[id], y = ft.y[id];
	uint32_t nx, ny;
	x = x < 0 ? 0 : (x > (int)ALOR_PANEL_W - 1 ? (int)ALOR_PANEL_W - 1 : x);
	y = y < 0 ? 0 : (y > (int)ALOR_PANEL_H - 1 ? (int)ALOR_PANEL_H - 1 : y);
	nx = (uint32_t)x * 65535u / (ALOR_PANEL_W - 1);
	ny = (uint32_t)y * 65535u / (ALOR_PANEL_H - 1);
	redwood_touch_post(type, id, nx, ny);
}
static void
sweet_hid_touch(int finger, int down, int x, int y)
{
	if (finger < 0 || finger >= (int)FT_MAX) {
		return;
	}
	ft.x[finger] = x;
	ft.y[finger] = y;
	uint32_t bit = 1u << finger;
	if (down) {
		alor_post((unsigned)finger,
		    (alor_shim_mask & bit) ? REDWOOD_TOUCH_MOVE : REDWOOD_TOUCH_DOWN);
		alor_shim_mask |= bit;
	} else if (alor_shim_mask & bit) {
		alor_post((unsigned)finger, REDWOOD_TOUCH_UP);
		alor_shim_mask &= ~bit;
	}
}

/* TLMM CTL: pull [1:0] (1 down, 3 up), func [5:2], drive [8:6] = mA/2-1, OE [9]. */
static void
ft_gpio(unsigned n, unsigned func, unsigned pull, unsigned ma, int out)
{
	if (n >= FT_PIN_MAX) {
		printf("ALOR TOUCH: pin %u does not exist (max %u)\n", n, FT_PIN_MAX - 1);
		return;
	}
	if (out >= 0) {
		ft_wr(ft.tlmm, FT_GPIO(n) + 4, out ? 2u : 0u);
	}
	uint32_t old = ft_rd(ft.tlmm, FT_GPIO(n));
	uint32_t cfg = (old & ~(0x3ffu | 0x1000u)) | pull | (func << 2) |
	    ((ma / 2 - 1) << 6) | (out >= 0 ? 0x200u : 0u);
	if (old & 0x800u) {
		cfg |= 0x1000u;        /* egpio enable: without it the mux never reaches the pad */
	}
	ft_wr(ft.tlmm, FT_GPIO(n), cfg);
}

static bool
ft_on(uint32_t cbcr)
{
	/* Qualcomm GCC CBCR: bit31 = CLK_OFF, bit27 = CLK_IS_ON (hw status). */
	return !(ft_rd(ft.gcc, cbcr) & 0x80000000u);
}

/*
 * Read-only clock survey.
 *
 * Register truth for this SoC family (recorded in alor_touch_v79.c from
 * gcc-sm8750, and the only offsets any on-device run has ever confirmed):
 *
 *   QUP1 wrapper M_AHB  0x23140
 *   QUP1 wrapper S_AHB  0x23144
 *   QUP1 SE4 core       0x183cc
 *   vote register       0x52008   (bits 20/21/26)
 *
 * An earlier revision of this file used 0x18004 / 0x18008 / 0x185ec and then
 * *wrote* them. 0x18004 is SE0's clock and 0x185ec belongs to something else
 * entirely, so those writes poked unrelated GCC registers -- a plausible way
 * to hang the AHB. Nothing here writes any more: measured on the live handset
 * through debugfs clk_print_regs, all three QUP1 clocks are already on
 * (CBCR 0x08000000, CLK_OFF=0, IS_ON=1; SE4 at 120 MHz = GPLL0 240 MHz / 2).
 */
static int
ft_clock(void)
{
	uint32_t m_ahb = ft_rd(ft.gcc, 0x23140);
	uint32_t s_ahb = ft_rd(ft.gcc, 0x23144);
	uint32_t se4 = ft_rd(ft.gcc, 0x183cc);
	bool m_on = !(m_ahb & 0x80000000u);
	bool s_on = !(s_ahb & 0x80000000u);
	bool e_on = !(se4 & 0x80000000u);
	printf("ALOR TOUCH: clk qup1 m_ahb %u s_ahb %u se4 %u (%#x %#x %#x)\n",
	    m_on, s_on, e_on, m_ahb, s_ahb, se4);
	alor_mark(1, m_on);
	alor_mark(2, s_on);
	alor_mark(3, e_on);
	if (!m_on || !s_on || !e_on) {
		printf("ALOR TOUCH: a QUP1 clock is off; the loader must enable it\n");
		return EIO;
	}
	alor_mark(4, m_on);
	alor_mark(5, e_on);
	return 0;
}

/*
 * ---- touch rail power-up -------------------------------------------------
 *
 * The FT3683 has two supplies: L4B_E0 (VDDI) and L16B_E0 (AVDD). The live
 * device tree states both, in logs/dt-ft3683.txt:
 *     power,vddi_volt <0x1b7740 0x1b7740>   = 1,800,000 uV = 1800 mV
 *     power,avdd_volt <0x325aa0 0x325aa0>   = 3,300,000 uV = 3300 mV
 * That is what Android runs this panel at, so it is what this driver asks for.
 * The loader's own voter uses 2800 mV for L16B_E0 (logs/rails-060727/vote.txt),
 * the same regulator at a lower set point: likely enough for the part to
 * answer, but the device tree value is the one the vendor chose.
 *
 * Two ways to bring them up, and they are not equally safe:
 *   - alor_rails=0: the loader votes them with `oem rails <slot>` in firmware,
 *     where MMIO is known-good. The kernel then never touches the RSC.
 *   - alor_rails=2 (default): the kernel votes through macan_rpmh_vote(), which
 *     writes the apps RSC at 0x18920000. Every kernel-side write to that block
 *     on this board has ended the run.
 * Until this existed the chip was never powered at all -- every "one byte a3 ->
 * 00" / "id probe rc 5 id 00" in the archives came from an unpowered part,
 * which says nothing about the protocol.
 *
 * Evidence that no earlier run ever powered it (checked across all 30+
 * captures in logs/):
 *   - "about to vote", "through macan_rpmh_vote", "macan-power: L4B_E0" and
 *     "macan-power: L16B_E0" appear ZERO times.
 *   - every run that printed "step 12 ok" had "rails mode 0", and step 12 is
 *     gated as alor_step(12, rails_mode ? avdd_ok : true) -- i.e. it reports
 *     true unconditionally when rails are not being driven at all.
 *   - the single "rails mode 1" run (logs/cap-050147) stopped at "step 10 ok",
 *     before any ft_rail_on() call.
 *   - "ALOR LIVE ... rail=N" is alor_rail_bar, the pixel count of an on-screen
 *     progress bar, NOT a supply reading. It only ever said the cmd-db mapped.
 *
 * The loader's TrustZone IO-proxy path (SMC 0x02000501/0x02000502) is refused
 * on this handset -- "tz2: monitor refuses IO access, rails stay off" -- so the
 * vote goes through the kit's own RPMh voter, macan_power.c, which is already
 * used for WLAN on this SoC family and takes the same cmd-db at 0x81c60000.
 *
 * The cmd-db addresses below (0x40700 / 0x41700, slave 4) were read off this
 * handset in logs/cap-050147.
 */
extern int macan_rpmh_vote(const char *resource, uint32_t millivolts, uint32_t mode, uint32_t enable);

#define FT_RSC_BASE    0x18920000u
#define FT_CMDDB_BASE  0x81c60000u
#define FT_CMDDB_SIZE  0x20000u
#define RH_DB_MAGIC    0x0c0330dbu
#define RH_DB_HEADER   144u
#define RH_DB_ENTRY    24u
#define FT_VDDI_MV     1800u      /* vcc_1v8 = RPMh L4B_E0 */
#define FT_AVDD_MV     3300u      /* RPMh L16B_E0; DT power,avdd_volt = 0x325aa0 */

extern vm_offset_t ml_io_map_wcomb(vm_offset_t, vm_size_t);

static vm_offset_t rh_db;
static uint32_t ft_rail_vddi, ft_rail_avdd;

static uint16_t
rh_db16(uint32_t off)
{
	return (uint16_t)(ft_rd(rh_db, off & ~3u) >> (8u * (off & 3u)));
}

/* Walk the cmd-db slave tables for one resource name and return its VRM
 * address. Exactly one match is required; anything else is a hard no. */
static int
rh_lookup(const char *res, uint32_t *address)
{
	uint32_t key[2] = { 0, 0 };
	unsigned matches = 0;
	for (unsigned i = 0; i < 8 && res[i]; i++) {
		key[i / 4] |= (uint32_t)(uint8_t)res[i] << (8u * (i % 4));
	}
	for (unsigned i = 0; i < 8; i++) {
		uint32_t h = 8u + 16u * i;
		unsigned slave = rh_db16(h);
		uint32_t entries;
		unsigned count;
		if (!slave) {
			break;
		}
		entries = RH_DB_HEADER + rh_db16(h + 2);
		count = rh_db16(h + 6);
		if ((entries & 3u) || entries + count * RH_DB_ENTRY > FT_CMDDB_SIZE) {
			printf("ALOR TOUCH: cmd-db slave header %u malformed\n", i);
			return EINVAL;
		}
		for (unsigned j = 0; j < count; j++) {
			uint32_t e = entries + RH_DB_ENTRY * j;
			uint32_t addr;
			if (ft_rd(rh_db, e) != key[0] || ft_rd(rh_db, e + 4) != key[1]) {
				continue;
			}
			addr = ft_rd(rh_db, e + 16);
			printf("ALOR TOUCH: cmd-db %s addr=%#x slave=%u\n", res, addr, slave);
			if (slave == 4 && ((addr >> 16) & 7u) == 4 && !(addr & 15u)) {
				*address = addr;
				matches++;
			}
		}
	}
	if (matches != 1) {
		printf("ALOR TOUCH: cmd-db %s: %u VRM matches; not voting\n", res, matches);
		return ENODEV;
	}
	return 0;
}

static bool
ft_rails_map(void)
{
	uint32_t magic;
	rh_db = ml_io_map_wcomb(FT_CMDDB_BASE, FT_CMDDB_SIZE);
	if (!rh_db) {
		printf("ALOR TOUCH: cmd-db unmapped\n");
		return false;
	}
	magic = ft_rd(rh_db, 4);
	printf("ALOR TOUCH: cmd-db version=%#x magic=%#x\n", ft_rd(rh_db, 0), magic);
	if (magic != RH_DB_MAGIC) {
		return false;
	}
	return !rh_lookup("L4B_E0", &ft_rail_vddi) && !rh_lookup("L16B_E0", &ft_rail_avdd);
}

/* The alor LDOs list supported-modes 2 and 4; the kit's own touch driver votes
 * L4B_E0 with mode 7 and works, so all three are accepted. */
static uint32_t
ft_ldo_mode(void)
{
	int mode = 3;
	(void)PE_parse_boot_argn("wifi_ready", &mode, sizeof(mode));
	if (mode != 2 && mode != 4 && mode != 7) {
		mode = 3;
	}
	return (uint32_t)mode;
}

/* How far to go with the rails. A boot argument, because the steps carry very
 * different risk:
 *
 *   alor_rails=0   touch nothing. The loader already voted the rails with its
 *                  own `oem rails <slot>` command, which runs in firmware
 *                  where MMIO is known-good. The kernel must then never touch
 *                  the apps RSC at 0x18920000.
 *   alor_rails=1   read the cmd-db only (memory at 0x81c60000, no RSC). Prints
 *                  the VRM addresses so the lookup can be checked.
 *   alor_rails=2   read cmd-db, then vote both rails through macan_rpmh_vote(),
 *                  which writes the RSC. Default.
 *
 * Why 0 exists, and why it is now the recommended way: every kernel-side write
 * to 0x18920000 on this board has ended the run. The handoff notes record the
 * whole box going quiet within tens of seconds (the vote result, the per-second heartbeat and the loader's USB answers all vanish together), and v79 carries the same account for v83/v84, adding
 * that v84 died on a read-only peek before any vote went out. On 2026-11-12 a
 * run at rails mode 7 froze at the Apple boot screen and never returned control
 * to the loader, so its log was unreachable. Meanwhile the loader's own RAIL
 * implementation demonstrably works: logs/rails-060727/vote.txt shows
 *
 *   RAILS: slot 1 armed, triggering
 *   RAILS: slot 1 status 1 after 29 polls
 *   RAILS: cmd 0 addr 40700 status 10101 resp 0
 *   RAILS: L4B_E0 voted 1800 mV mode 3
 *   RAILS: L16B_E0 wants 2800 mV mode 3 at RPMh 41700
 *
 * i.e. both rails, commands completed, no kernel involved. That is the path
 * worth using: power from the loader, then let the kernel do nothing but
 * bit-bang the ID read.
 *
 * A cross-reference of every capture in logs/ is what forced this design. Across
 * 50+ archived runs, 17 captures contain a completed vote and dozens contain
 * driver output, but the intersection is EMPTY: no session ever had the rails
 * voted and the driver running at the same time. The reason is structural --
 * `oem rails` is refused while a kernel runs, and once `oem run` hands control
 * over, the loader stops servicing commands -- so the combination cannot happen
 * by accident; it has to be arranged. Until it is, every "id 00" in the archive
 * came from a chip that was never powered and tests nothing about the protocol.
 */
static int
ft_rails_stage(void)
{
	int stage = 2;
	(void)PE_parse_boot_argn("alor_rails", &stage, sizeof(stage));
	if (stage < 0 || stage > 2) {
		stage = 2;
	}
	return stage;
}

static bool
ft_rail_on(const char *name, uint32_t mv)
{
	int done = 0;
	printf("ALOR TOUCH: voting %s %u mV mode %u\n", name, mv, ft_ldo_mode());
	/* macan_rpmh_vote only uses ACTIVE TCS 0 and gives up when the GPU or the
	 * DSP holds it, so retry rather than lose the rail. */
	for (unsigned tries = 0; tries < 8 && !done; tries++) {
		done = macan_rpmh_vote(name, mv, ft_ldo_mode(), 1);
		if (!done) {
			ft_sleep(200);
		}
	}
	printf("ALOR TOUCH: %s %u mV -> %d\n", name, mv, done);
	return done != 0;
}

/* Returns true when the touch supplies are believed to be on.
 *
 * At stage 0 the loader already voted them, so "on" is an assumption this
 * driver cannot check -- and deliberately does not try to, because checking
 * would mean reading the RSC, which is the one access that kills the run. The
 * driver reports that it is assuming, and then proves or disproves it the only
 * way that matters: by reading the chip ID.
 *
 * At stage 2 "up" is checked two ways, because the vote's return value only
 * says the RPMh transaction completed -- it does not say the regulator came on:
 *   1. macan_rpmh_vote() reports success for each rail.
 *   2. The RPMh block is read back and must show a non-zero level for the
 *      resource. That is the only feedback the AP gets on this path.
 */
static bool
ft_rails_up(void)
{
	bool vddi, avdd;
	int stage = ft_rails_stage();
	if (stage == 0) {
		printf("ALOR TOUCH: alor_rails=0, the loader owns the rails; "
		    "kernel will not touch the RSC\n");
		return true;
	}
	if (!ft_rails_map()) {
		printf("ALOR TOUCH: cmd-db unusable; rails stay off\n");
		return false;
	}
	if (stage == 1) {
		printf("ALOR TOUCH: alor_rails=1, cmd-db only: L4B_E0 vrm %#x, "
		    "L16B_E0 vrm %#x; not voting, RSC untouched\n",
		    ft_rail_vddi, ft_rail_avdd);
		return false;
	}
	vddi = ft_rail_on("L4B_E0", FT_VDDI_MV);
	avdd = ft_rail_on("L16B_E0", FT_AVDD_MV);
	printf("ALOR TOUCH: rails L4B %u L16B %u (vrm %#x/%#x)\n", vddi, avdd,
	    ft_rail_vddi, ft_rail_avdd);
	if (vddi && avdd) {
		/* The HBP driver waits 80 ms after the rails are up before it pulses
		 * reset; give the supplies the same margin. */
		ft_sleep(80);
		printf("ALOR TOUCH: rails settled, chip should be powered\n");
	} else {
		printf("ALOR TOUCH: a rail failed to vote; the chip stays dark\n");
	}
	return vddi && avdd;
}

/* DMA page: event ring (4 KiB, 256 events), TX and RX rings (1 KiB, 64 TREs
 * each), then the TX and RX data buffers. Cacheable: clean before a doorbell,
 * invalidate before reading what the hardware wrote. */
#define FT_EV_OFF    0x0000u
#define FT_RING_OFF(k) (0x1000u + 0x400u * (k))
#define FT_TXB_OFF   0x1800u
#define FT_RXB_OFF   0x1c00u
#define FT_RING      64u
#define FT_EVRING    256u
#define GPI_CH(k)    (0x0000u + 0x80u * (k))
#define GPI_EV       0x1000u
#define GPI_CHDB(k)  (0x2000u + 8u * (k))
#define GPI_EVDB     0x2100u
#define GPI_GPII     0x3000u

static void
ft_dma_sync(uint32_t off, uint32_t len)
{
	flush_dcache64((addr64_t)(ft.dma_pa + off), len, TRUE);
}

static uint32_t
ft_state(uint32_t ctx)
{
	return (ft_rd(ft.gpi, ctx) >> 20) & 0xf;
}

static bool
ft_gpi_cmd(uint32_t reg, uint32_t v, uint32_t ctx, uint32_t want)
{
	ft_wr(ft.gpi, GPI_GPII + reg, v);
	for (unsigned i = 0; i < 50000; i++) {
		uint32_t st = ft_state(ctx);
		if (st == want) {
			ft_wr(ft.gpi, GPI_GPII + 0xa0, ft_rd(ft.gpi, GPI_GPII + 0x90));
			ft_wr(ft.gpi, GPI_GPII + 0xa4, ft_rd(ft.gpi, GPI_GPII + 0x94));
			return true;
		}
		if (st == 0xf) {
			break;
		}
		delay(1);
	}
	printf("FUXI TOUCH: GPI cmd %#x -> state %#x (want %u), error log %#x\n",
	    v, ft_state(ctx), want, ft_rd(ft.gpi, GPI_GPII + 0x200));
	return false;
}

/* Bus-masters without an SMR fault (sCR0.USFCFG) and the hypervisor turns an
 * S2CR bypass into fault: route GPI and QUP1 to CB4 like DWC3. Idempotent. */
static int
ft_smmu(void)
{
	static const uint16_t sid[] = {0xb6, 0xa3};
	uint32_t nsmr = ft_rd(ft.smmu, 0x20) & 0xff;
	for (unsigned j = 0; j < 2; j++) {
		unsigned i, freei = ~0u;
		for (i = 0; i < nsmr; i++) {
			uint32_t smr = ft_rd(ft.smmu, 0x800 + 4 * i);
			if (!(smr & 0x80000000u)) {
				if (freei == ~0u) {
					freei = i;
				}
				continue;
			}
			if (((sid[j] ^ smr) & ~(smr >> 16) & 0x7fff) == 0) {
				break;
			}
		}
		if (i < nsmr) {
			continue;
		}
		if (freei == ~0u) {
			return ENOSPC;
		}
		ft_wr(ft.smmu, 0xc00 + 4 * freei, 4);
		ft_wr(ft.smmu, 0x800 + 4 * freei, 0x80000000u | sid[j]);
		printf("FUXI TOUCH: SMR %u = sid %#x -> S2CR %#x\n", freei, sid[j],
		    ft_rd(ft.smmu, 0xc00 + 4 * freei));
	}
	return 0;
}

static int
ft_gpi_init(void)
{
	uint32_t lo = (uint32_t)ft.dma_pa, hi = (uint32_t)(ft.dma_pa >> 32);
	/* The loader leaves both channels started on its own rings: stop, reset, free. */
	for (unsigned k = 0; k < 2; k++) {
		if (ft_state(GPI_CH(k)) == 2) {
			(void)ft_gpi_cmd(0x08, (2u << 24) | k, GPI_CH(k), 3);
		}
		if (ft_state(GPI_CH(k))) {
			(void)ft_gpi_cmd(0x08, (9u << 24) | k, GPI_CH(k), 1);
			ft_wr(ft.gpi, GPI_GPII + 0x08, (10u << 24) | k);
			delay(1000);
		}
	}
	if (ft_state(GPI_EV)) {
		(void)ft_gpi_cmd(0x10, 9u << 24, GPI_EV, 1);
		ft_wr(ft.gpi, GPI_GPII + 0x10, 10u << 24);
		delay(1000);
	}
	bzero((void *)ft.dma, PAGE_SIZE);
	ft_dma_sync(0, PAGE_SIZE);
	ft_wr(ft.gpi, GPI_GPII + 0x88, 0x4f);
	ft_wr(ft.gpi, GPI_GPII + 0xb8, 1);
	ft_wr(ft.gpi, GPI_GPII + 0x98, 3);
	ft_wr(ft.gpi, GPI_GPII + 0x9c, 1);
	ft_wr(ft.gpi, GPI_GPII + 0x108, 1);
	ft_wr(ft.gpi, GPI_GPII + 0x120, 0xf);
	ft_wr(ft.gpi, GPI_GPII + 0x188, 0);
	ft_wr(ft.gpi, GPI_GPII + 0x18c, 0);
	ft_wr(ft.gpi, GPI_GPII + 0x400, 0);
	ft_wr(ft.gpi, GPI_GPII + 0x404, 0);
	ft_wr(ft.gpi, GPI_GPII + 0x180, 1);
	ft_wr(ft.gpi, GPI_GPII + 0x200, 0);
	if (!ft_gpi_cmd(0x10, 0, GPI_EV, 1)) {
		return EIO;
	}
	ft_wr(ft.gpi, GPI_EV + 0x00, 0x10010002u);
	ft_wr(ft.gpi, GPI_EV + 0x04, FT_EVRING * 16);
	ft_wr(ft.gpi, GPI_EV + 0x08, lo + FT_EV_OFF);
	ft_wr(ft.gpi, GPI_EV + 0x0c, hi);
	ft_wr(ft.gpi, GPI_EVDB + 4, hi);
	for (uint32_t o = 0x20; o <= 0x34; o += 4) {
		ft_wr(ft.gpi, GPI_EV + o, 0);
	}
	ft.evrp = 0;
	ft_wr(ft.gpi, GPI_EVDB, lo + FT_EV_OFF + FT_EVRING * 16 - 16);
	for (unsigned k = 0; k < 2; k++) {
		if (!ft_gpi_cmd(0x08, k, GPI_CH(k), 1)) {
			return EIO;
		}
		ft_wr(ft.gpi, GPI_CH(k) + 0x00, k ? 0x10000002u : 0x1000000au);
		ft_wr(ft.gpi, GPI_CH(k) + 0x04, FT_RING * 16);
		ft_wr(ft.gpi, GPI_CH(k) + 0x08, lo + FT_RING_OFF(k));
		ft_wr(ft.gpi, GPI_CH(k) + 0x0c, hi);
		ft_wr(ft.gpi, GPI_CHDB(k) + 4, hi);
		/* SCRATCH_0: pair, SPI protocol 1, SE 4. */
		ft_wr(ft.gpi, GPI_CH(k) + 0x60, k ? 0x00000014u : 0x00010014u);
		ft_wr(ft.gpi, GPI_CH(k) + 0x64, 0);
		ft_wr(ft.gpi, GPI_CH(k) + 0x68, 0);
		ft_wr(ft.gpi, GPI_CH(k) + 0x6c, 0);
		ft_wr(ft.gpi, GPI_CH(k) + 0x5c, 1);
		ft.wp[k] = 0;
	}
	for (unsigned k = 0; k < 2; k++) {
		if (!ft_gpi_cmd(0x08, (1u << 24) | k, GPI_CH(k), 2)) {
			return EIO;
		}
	}
	return 0;
}

static int
ft_se_init(void)
{
	uint32_t fw = ft_rd(ft.se, 0x068);
	if (((fw >> 8) & 0xff) != 1 || (ft_rd(ft.se, 0x040) & 1)) {
		printf("FUXI TOUCH: SE4 fw %#x status %#x: not idle SPI\n", fw, ft_rd(ft.se, 0x040));
		return ENODEV;
	}
	/* geni_se_select_mode(GENI_GPI_DMA): the CONFIG0 TRE sets mode, word
	 * size, packing and clock. */
	ft_wr(ft.se, 0xe18, 0);
	ft_wr(ft.se, 0x618, 0xffffffffu);
	ft_wr(ft.se, 0x648, 0xffffffffu);
	ft_wr(ft.se, 0xc44, 0xffffffffu);
	ft_wr(ft.se, 0xd44, 0xffffffffu);
	ft_wr(ft.se, 0x028, ft_rd(ft.se, 0x028) | 0x7f);
	ft_wr(ft.se, 0xe30, ft_rd(ft.se, 0xe30) | 0xf);
	ft_wr(ft.se, 0x024, 0x7f);
	ft_wr(ft.se, 0x020, 1);
	ft_wr(ft.se, 0xe1c, 0);
	ft_wr(ft.se, 0x614, ft_rd(ft.se, 0x614) & ~((1u << 0) | (1u << 26) | (1u << 27) | (1u << 30)));
	ft_wr(ft.se, 0x644, ft_rd(ft.se, 0x644) & ~1u);
	ft_wr(ft.se, 0x258, 1);
	/* SE_SPI_CLK_DIV (0x600): clk_div = SE_clk / (2 * sclk).
	 * The device tree asks for 12 MHz on the touch node (0xb71b00); the bus
	 * itself is 19 MHz (0x121eac0) and the master's ceiling is 50 MHz
	 * (0x02faf080, read from /sys/class/spi_master/spi4/of_node). The SE4
	 * source runs at 120 MHz (measured: gcc_qupv3_wrap1_s4_clk/clk_measure
	 * = 120000128), so 120e6 / (2 * 12e6) = 5.
	 * Without this the SE keeps its reset divider and clocks the panel far
	 * above what the FT3683 accepts. */
	ft_wr(ft.se, 0x600, 5);
	ft_wr(ft.se, 0xe18, ft_rd(ft.se, 0xe18) | 0xf);
	return 0;
}

static void
ft_tre(unsigned k, uint32_t d0, uint32_t d1, uint32_t d2, uint32_t d3)
{
	uint32_t off = FT_RING_OFF(k) + 16 * ft.wp[k];
	volatile uint32_t *t = (volatile uint32_t *)(ft.dma + off);
	t[0] = d0;
	t[1] = d1;
	t[2] = d2;
	t[3] = d3;
	ft_dma_sync(off, 16);
	ft.wp[k] = (ft.wp[k] + 1) % FT_RING;
}

static void
ft_ring_db(unsigned k)
{
	__asm__ volatile("dsb sy" ::: "memory");
	ft_wr(ft.gpi, GPI_CHDB(k), (uint32_t)ft.dma_pa + FT_RING_OFF(k) + 16 * ft.wp[k]);
}

/* Drain events until an end-of-transfer (code 2) from chid arrives. */
static int
ft_gpi_wait(unsigned chid, uint32_t *len)
{
	for (unsigned us = 0; us < 5000; us++) {
		uint32_t hw = ft_rd(ft.gpi, GPI_EV + 0x10);
		bool found = false, bad = false;
		while (hw != (uint32_t)ft.dma_pa + FT_EV_OFF + 16 * ft.evrp) {
			uint32_t off = FT_EV_OFF + 16 * ft.evrp;
			ft_dma_sync(off, 16);
			volatile uint32_t *e = (volatile uint32_t *)(ft.dma + off);
			uint32_t d2 = e[2], d3 = e[3];
			ft.evrp = (ft.evrp + 1) % FT_EVRING;
			unsigned code = d2 >> 24, type = (d3 >> 16) & 0xff, ch = d3 >> 24;
			if (code == 16 || type == 0xff || ch >= 2) {
				bad = true;
				printf("FUXI TOUCH: GPI event %#x %#x\n", d2, d3);
			} else if (type == 0x22 && ch == chid && code == 2) {
				found = true;
				*len = d2 & 0xffffff;
			}
		}
		ft_wr(ft.gpi, GPI_EVDB, (uint32_t)ft.dma_pa + FT_EV_OFF +
		    16 * ((ft.evrp + FT_EVRING - 1) % FT_EVRING));
		ft_wr(ft.gpi, GPI_GPII + 0xc0, 1);
		if (bad) {
			return EIO;
		}
		if (found) {
			return 0;
		}
		delay(1);
	}
	return ETIMEDOUT;
}

#define FT_C0()  ft_tre(0, 0x03000004u, 0, (ft.level << 16) | 2u, 0x00220001u)

static int
ft_tx(const uint8_t *tx, unsigned nt, bool frag)
{
	uint32_t len = 0;
	int err;
	memcpy((void *)(ft.dma + FT_TXB_OFF), tx, nt);
	ft_dma_sync(FT_TXB_OFF, nt);
	FT_C0();
	ft_tre(0, (frag ? (1u << 26) : 0) | 1u, 0, 0, 0x00200001u);
	ft_tre(0, (uint32_t)ft.dma_pa + FT_TXB_OFF, (uint32_t)(ft.dma_pa >> 32), nt, 0x00100200u);
	ft_ring_db(0);
	if ((err = ft_gpi_wait(0, &len))) {
		return err;
	}
	return len == nt ? 0 : EIO;
}

/* FTS read: opcode + address (CS held), then one dummy byte and n data bytes. */
static int
ft_read(const uint8_t *cmd, unsigned nc, uint8_t *out, unsigned n)
{
	uint32_t len = 0;
	int err;
	if (nc > 8 || n > 8 * 32) {
		return EINVAL;
	}
	if ((err = ft_tx(cmd, nc, true))) {
		return err;
	}
	ft_dma_sync(FT_RXB_OFF, n + 1);
	ft_tre(1, (uint32_t)ft.dma_pa + FT_RXB_OFF, (uint32_t)(ft.dma_pa >> 32), n + 1, 0x00100200u);
	ft_tre(0, 2u, 0, n + 1, 0x00200900u);
	ft_ring_db(1);
	ft_ring_db(0);
	if ((err = ft_gpi_wait(1, &len))) {
		return err;
	}
	ft_dma_sync(FT_RXB_OFF, n + 1);
	memcpy(out, (const void *)(ft.dma + FT_RXB_OFF + 1), n);
	return 0;
}

static uint64_t
ft_us(uint64_t abs)
{
	uint64_t ns;
	absolutetime_to_nanoseconds(abs, &ns);
	return ns / 1000;
}

/* One line per contact: how often iOS got a report and what a FIFO read costs. */
static void
ft_contact_end(void)
{
	uint64_t dur = ft_us(ft.c_last - ft.c_start);
	ft.contacts++;
	printf("FUXI TOUCH: contact %u: %u reports in %llu ms (%llu us apart), %u polls, "
	    "FIFO read avg %llu us max %llu us\n", ft.contacts, ft.c_reports, dur / 1000,
	    ft.c_reports > 1 ? dur / (ft.c_reports - 1) : 0ULL, ft.c_polls,
	    ft.c_polls ? ft_us(ft.c_poll_sum) / ft.c_polls : 0ULL, ft_us(ft.c_poll_max));
}

static void
ft_report(uint32_t mask)
{
	uint64_t now = mach_absolute_time();
	if (mask && !ft.active) {
		ft.c_start = now;
		ft.c_reports = ft.c_polls = 0;
		ft.c_poll_sum = ft.c_poll_max = 0;
	}
	for (unsigned id = 0; id < FT_MAX; id++) {
		uint32_t bit = 1u << id;
		if (mask & bit) {
			sweet_hid_touch((int)id, 1, ft.x[id], ft.y[id]);
		} else if (ft.active & bit) {
			sweet_hid_touch((int)id, 0, ft.x[id], ft.y[id]);
		}
	}
	if (mask || ft.active) {
		ft.c_reports++;
		ft.c_last = now;
	}
	bool ended = ft.active && !mask;
	ft.active = mask;
	if (ended) {
		ft_contact_end();
	}
}

/*
 * Drain the FIFO in two reads (as fts_event_handler): one event, whose
 * byte 7 holds the number of events still queued, then all of those at
 * once. With deliver false (display off) the events are consumed and
 * dropped.
 */
static int
ft_poll(bool deliver)
{
	static const uint8_t fifo[] = {0x87};
	uint8_t e[8 * 32];
	uint32_t mask = ft.active;
	bool changed = false;
	int err = ft_read(fifo, 1, e, 8);
	if (err) {
		return err;
	}
	if (e[0] == 0) {
		return 0;
	}
	unsigned n = 1 + (e[7] & 0x1f);
	if (n > 32) {
		n = 32;
	}
	if (n > 1 && (err = ft_read(fifo, 1, e + 8, 8 * (n - 1)))) {
		return err;
	}
	for (unsigned i = 0; i < n; i++) {
		const uint8_t *ev = e + 8 * i;
		if (ev[0] == 0) {
			break;
		}
		if (ev[0] == FT_EV_READY) {
			ft.ic_reset = true;
			continue;
		}
		unsigned id = ev[1] >> 4, type = ev[1] & 0xf;
		if ((ev[0] != FT_EV_ENTER && ev[0] != FT_EV_MOTION && ev[0] != FT_EV_LEAVE) ||
		    id >= FT_MAX || (type == 0 && ev[0] != FT_EV_LEAVE)) {
			continue;
		}
		/* Coordinates arrive at 10x the panel resolution. The panel on this
		 * handset is 1272x2772 (ALOR_PANEL_W/H); the 1080x2400 that used to be
		 * hard-coded here is the fuxi/macan panel and would clip the bottom
		 * and right edges. */
		int x = ((ev[3] << 8) | ev[2]) / 10, y = ((ev[5] << 8) | ev[4]) / 10;
		ft.x[id] = x > (int)ALOR_PANEL_W - 1 ? (int)ALOR_PANEL_W - 1 : x;
		ft.y[id] = y > (int)ALOR_PANEL_H - 1 ? (int)ALOR_PANEL_H - 1 : y;
		if (ev[0] == FT_EV_LEAVE || type == FT_TYPE_HOVER) {
			mask &= ~(1u << id);
		} else {
			mask |= 1u << id;
		}
		changed = true;
	}
	if (changed && deliver) {
		ft_report(mask);
	}
	return 0;
}

/* ---- FT3683 high-bandwidth protocol over the GPI DMA path above ----
 *
 * Frame layout, taken from the vendor binary rather than from guesswork. The
 * project's own disassembly of tp_ft3658u.ko (the same FocalTech family) is in
 * re/wr.txt and re/xfer.txt; the write path builds its header like this:
 *
 *     6e10  add  w19, w23, #9     ; total transfer = n + 9
 *     6e88  sub  w2, w23, #1      ; the count field is n - 1
 *     6e90  strb wzr, [x20, #1]   ; tx[1] = 0  (write opcode)
 *     6e98  strb w2,  [x20, #3]   ; tx[3] = count low
 *     6e9c  strb w8,  [x20]       ; tx[0] = register address
 *     6ea0  strb w9,  [x20, #2]   ; tx[2] = count high
 *     6ea8  add  x0, x20, #7      ; payload starts at tx[7]
 *     6eac  add  x1, x21, #1      ; from caller's buffer + 1
 *     6eb4  add  w25, w23, #6     ; when the count is zero, length = 4
 *     6f40  mov  w8, #0xa0        ; tx[3] & 0xa0 is the error test
 *     6f44  tst  w2, w8
 *
 * The vendor's n counts its own address byte, so its n - 1 is this driver's
 * count and its n + 6 is 7 + count: the two agree exactly. The read path asks
 * for n + 9 bytes -- seven header, n data, two CRC -- which is also what this
 * file does. So the framing was already right; what the disassembly settles is
 * that tx[1] is 0xa0 for a read and 0x00 for a write, that bytes 4..6 are
 * always zero, that the payload begins at offset 7, and that the error test is
 * the one used here.
 */
static uint16_t
alor_crc(const uint8_t *p, unsigned n)
{
	uint16_t c = 0xffff;
	while (n--) {
		c ^= *p++;
		for (unsigned b = 0; b < 8; b++) {
			c = (c & 1) ? (uint16_t)((c >> 1) ^ 0x8408) : (uint16_t)(c >> 1);
		}
	}
	return c;
}

static int
alor_spi_duplex_gpi(const uint8_t *tx, unsigned len, uint8_t *rx)
{
	uint32_t got = 0, got2 = 0;
	int err;
	if (len == 0 || len > 0x300) {
		return EINVAL;
	}
	memcpy((void *)(ft.dma + FT_TXB_OFF), tx, len);
	ft_dma_sync(FT_TXB_OFF, len);
	ft_dma_sync(FT_RXB_OFF, len);
	ft_tre(1, (uint32_t)ft.dma_pa + FT_RXB_OFF, (uint32_t)(ft.dma_pa >> 32), len, 0x00100200u);
	FT_C0();
	ft_tre(0, 3u, 0, len, 0x00200801u);
	ft_tre(0, (uint32_t)ft.dma_pa + FT_TXB_OFF, (uint32_t)(ft.dma_pa >> 32), len, 0x00100200u);
	ft_ring_db(1);
	ft_ring_db(0);
	if ((err = ft_gpi_wait(1, &got))) {
		return err;
	}
	(void)ft_gpi_wait(0, &got2);
	if (got != len) {
		return EIO;
	}
	ft_dma_sync(FT_RXB_OFF, len);
	memcpy(rx, (const void *)(ft.dma + FT_RXB_OFF), len);
	return 0;
}

/*
 * Pure GPIO bit-bang fallback, SPI mode 0 (CPOL=0, CPHA=0, MSB first).
 *
 * Why this exists: the GPI/GENI path needs the QUP1 SE4 clock, the apps-SMMU
 * SMRs and the GPII channel state all live, and every attempt to bring those
 * up from XNU on this board has hung the bus. The four lanes are ordinary
 * TLMM pads, so drive them directly instead: func 0, MOSI/CLK/CS as outputs,
 * MISO as input. This touches no QUP, no SMMU and no RSC register.
 *
 * Timings are deliberately slow (~1 us per half period, i.e. well under
 * 1 MHz). The FT3683 samples fine at that rate and the poll loop only needs a
 * handful of 90-byte transfers per frame.
 */
static void
ft_bb_delay(void)
{
	for (volatile int i = 0; i < 60; i++) {
	}
}

static void
ft_bb_pin(unsigned n, bool output, bool high)
{
	uint32_t old, cfg;
	if (n >= FT_PIN_MAX) {
		return;
	}
	if (output) {
		ft_wr(ft.tlmm, FT_GPIO(n) + 4, high ? 2u : 0u);
	}
	old = ft_rd(ft.tlmm, FT_GPIO(n));
	/* func 0, 4 mA, OE only when this pad is an output.
	 * Keep bit 12 (egpio enable) exactly as ft_gpio() does: without it the
	 * mux never reaches the pad on this controller.
	 *
	 * MISO needs a pull-up. With pull 0 a floating line reads back as noise
	 * whenever the panel is not actively driving it, which is exactly what an
	 * absent answer looks like: every bit reads 1 or 0 at random and the
	 * register comes back as garbage rather than as a clean 0x00. The stock
	 * value for these pads is 0x1803 (pull-up), and that is what the vendor
	 * pinctrl leaves behind too. */
	cfg = (old & ~0x3ffu) | (output ? 0x200u : 0u) | ((4u / 2u - 1u) << 6) |
	    (output ? 0u : 3u);
	/* Always set egpio_enable. The stock value for the SPI lanes is 0x0801 --
	 * egpio_present set (bit 11) but egpio_enable clear (bit 12) -- and on
	 * this controller the TLMM function only reaches the pad once bit 12 is
	 * set. ft_gpio() already does this for the QUP path; the bit-bang path
	 * used to copy whatever was there, which left CLK and CS (0x0801, bit 12
	 * clear) with an inert mux: the output register changed but the pad never
	 * moved, so no clock ever reached the panel. */
	if (old & 0x800u) {
		cfg |= 0x1000u;
	}
	ft_wr(ft.tlmm, FT_GPIO(n), cfg);
}

static int
alor_spi_duplex_bb(const uint8_t *tx, unsigned len, uint8_t *rx)
{
	if (len == 0 || len > 0x300) {
		return EINVAL;
	}
	/* CS asserted (active low), clock idles low */
	ft_wr(ft.tlmm, FT_GPIO(FT_PIN_CS) + 4, 0u);
	ft_bb_delay();
	for (unsigned i = 0; i < len; i++) {
		uint8_t in = 0;
		for (int bit = 7; bit >= 0; bit--) {
			/* MOSI set up while CLK is low (mode 0) */
			ft_wr(ft.tlmm, FT_GPIO(FT_PIN_MOSI) + 4, ((tx[i] >> bit) & 1u) ? 2u : 0u);
			ft_bb_delay();
			ft_wr(ft.tlmm, FT_GPIO(FT_PIN_CLK) + 4, 2u);
			ft_bb_delay();
			in = (uint8_t)((in << 1) | (ft_rd(ft.tlmm, FT_GPIO(FT_PIN_MISO) + 4) & 1u));
			ft_wr(ft.tlmm, FT_GPIO(FT_PIN_CLK) + 4, 0u);
			ft_bb_delay();
		}
		rx[i] = in;
	}
	ft_bb_delay();
	ft_wr(ft.tlmm, FT_GPIO(FT_PIN_CS) + 4, 2u);   /* CS released, idles high */
	ft_bb_delay();
	return 0;
}

static int
alor_spi_duplex(const uint8_t *tx, unsigned len, uint8_t *rx)
{
	if (ft.bb) {
		return alor_spi_duplex_bb(tx, len, rx);
	}
	return alor_spi_duplex_gpi(tx, len, rx);
}

static int
alor_reg_read(uint8_t addr, uint8_t *out, unsigned n)
{
	uint8_t tx[128], rx[128];
	if (n + 9 > sizeof(tx)) {
		return EINVAL;
	}
	for (unsigned attempt = 0; attempt < 3; attempt++) {
		memset(tx, 0, n + 9);
		tx[0] = addr;
		tx[1] = 0xa0;
		tx[2] = (uint8_t)(n >> 8);
		tx[3] = (uint8_t)n;
		memset(rx, 0, n + 9);
		int rc = alor_spi_duplex(tx, n + 9, rx);
		delay(150);
		if (rc || (rx[3] & 0xa0)) {
			continue;
		}
		uint16_t crc = (uint16_t)(rx[7 + n] | ((uint16_t)rx[8 + n] << 8));
		if (alor_crc(rx + 7, n) != crc) {
			continue;
		}
		memcpy(out, rx + 7, n);
		return 0;
	}
	return EIO;
}

static int
alor_reg_write(uint8_t addr, const uint8_t *data, unsigned n)
{
	uint8_t tx[128], rx[128];
	unsigned len = n ? 7 + n : 4;
	if (len > sizeof(tx)) {
		return EINVAL;
	}
	for (unsigned attempt = 0; attempt < 3; attempt++) {
		memset(tx, 0, len);
		tx[0] = addr;
		tx[1] = 0x00;
		tx[2] = (uint8_t)(n >> 8);
		tx[3] = (uint8_t)n;
		if (n) {
			memcpy(tx + 7, data, n);
		}
		memset(rx, 0, len);
		int rc = alor_spi_duplex(tx, len, rx);
		delay(150);
		if (rc || (rx[3] & 0xa0)) {
			continue;
		}
		return 0;
	}
	return EIO;
}

/* Contacts of a TIC frame at 0x01: v2 slots are 8 bytes from buf+4 with X/Y in
 * 1/16 pixel, v1 slots are 6 bytes from buf+2 in whole pixels. */
static unsigned
alor_frame_decode(const uint8_t *buf, int *ids, int *xs, int *ys)
{
	unsigned type = buf[1] >> 4, reported = buf[1] & 0xf, count = 0;
	if (reported > 10) {
		return 0;
	}
	if (type == 2) {
		if (!reported) {
			return 0;
		}
		for (unsigned i = 0; i < reported; i++) {
			if ((buf[4 + 8 * i + 2] >> 4) >= 10) {
				return 0;
			}
		}
		for (unsigned i = 0; i < reported; i++) {
			const uint8_t *p = buf + 4 + 8 * i;
			unsigned event = p[0] >> 6;
			if (event != 0 && event != 2) {
				continue;
			}
			unsigned x = ((p[0] & 0xf) << 12) | ((unsigned)p[1] << 4) | (p[4] >> 4);
			unsigned y = ((p[2] & 0xf) << 12) | ((unsigned)p[3] << 4) | (p[4] & 0xf);
			ids[count] = p[2] >> 4;
			xs[count] = (int)(x / 16u);
			ys[count] = (int)(y / 16u);
			count++;
		}
	} else if (type == 0) {
		for (unsigned i = 0; i < 10; i++) {
			const uint8_t *p = buf + 2 + 6 * i;
			unsigned id = p[2] >> 4, event = p[0] >> 6;
			if (id >= 10) {
				break;
			}
			if (event != 0 && event != 2) {
				continue;
			}
			if (!reported) {
				return 0;
			}
			ids[count] = (int)id;
			xs[count] = (int)(((p[0] & 0xf) << 8) | p[1]);
			ys[count] = (int)(((p[2] & 0xf) << 8) | p[3]);
			count++;
		}
	}
	return count;
}

static void
alor_fts_run(void)
{
	uint8_t id3 = 0, id9f = 0, fw6 = 0, mode = 0xff, frame[84];
	static const uint8_t zero = 0;
	int rc3 = -1, rc9f = -1, rcfw = -1, rc9e = -1;
	unsigned delay_s = 8, frames = 0, bad = 0, printed = 0, good = 0;
	uint32_t probe = 0;
	(void)PE_parse_boot_argn("macan_touch_probe", &probe, sizeof(probe));
	(void)PE_parse_boot_argn("alortouchdelay", &delay_s, sizeof(delay_s));
	printf("ALOR TOUCH: starting in %u s\n", delay_s);
	ft_sleep(delay_s * 1000u);
	alor_stage = 8;
	/* Neo8 pin config, straight from the device tree:
	 *   pmx_ts_int_active:   gpio55,  function gpio, pull-up, input, 8 mA
	 *   pmx_ts_reset_active: gpio161, function gpio, no pull, output-high, 8 mA
	 *   qupv3_se4_spi_*:     gpio48..51, qup1_se4_l0..l3 (func 1), bias-disable, 4 mA
	 *
	 * In bit-bang mode the same four pads are driven as plain GPIO (func 0):
	 * MISO is an input, MOSI/CLK/CS are outputs, CS idles high. */
	/* alor_pins_loader=1: the loader owns every pad on this chip.
	 *
	 * loader.c:756-762 records why. Reading pin 48 from the kernel took a
	 * 900E on 2026-11-09, and so did reading pin 55. The loader therefore
	 * muxes 48..51 to function 1, sets gpio55 as the IRQ input and releases
	 * reset through gpio161, all before the rails come up, precisely so that
	 * the kernel never has to map TLMM. Any ft_gpio() call here undoes that
	 * arrangement and walks straight back into the fault.
	 *
	 * With the flag set this whole block is skipped: the pads already hold the
	 * vendor settings the loader programmed. */
	if (!pins_loader) {
		ft_gpio(FT_IRQ, 0, 3, 8, -1);
		if (ft.bb) {
			ft_bb_pin(FT_PIN_MISO, false, false);
			ft_bb_pin(FT_PIN_MOSI, true, false);
			ft_bb_pin(FT_PIN_CLK, true, false);
			ft_bb_pin(FT_PIN_CS, true, true);
		} else {
			/* The four lanes stay owned by the QUP once muxed to func 1: the
			 * vendor pinctrl state sets no output-enable on them (the serial
			 * engine drives direction), so pass -1 rather than forcing OE. */
			ft_gpio(FT_PIN_MISO, 1, 0, 4, -1);
			ft_gpio(FT_PIN_MOSI, 1, 0, 4, -1);
			ft_gpio(FT_PIN_CLK, 1, 0, 4, -1);
			ft_gpio(FT_PIN_CS, 1, 0, 4, -1);
		}
	}
	alor_blip(5);
	/* Three permanent landmarks at the top of the panel, drawn as soon as the
	 * framebuffer is mapped and never overwritten:
	 *   left   blue   -- this build reaches the framebuffer at all
	 *   middle yellow -- the rails stage returned true
	 *   right  white  -- the TLMM mapping is present (bit-bang is possible)
	 * Without these, "nothing on screen" is ambiguous: it can mean the
	 * framebuffer write failed, the driver never ran, or the panel is dark. */
	alor_blip_c(10, 0x00ff0000u);
	alor_blip_c(11, 0x0000ffffu);
	alor_blip_c(12, 0x00ffffffu);
	if (alor_ws <= 6) {
		alor_done = 1;
		printf("ALOR TOUCH: stage %u done, holding\n", alor_ws);
		for (;;) {
			ft_sleep(5000);
		}
	}
	if (probe) {
		alor_done = 1;
		printf("ALOR TOUCH: probe mode: init done, no bus traffic\n");
		for (;;) {
			ft_sleep(5000);
		}
	}
	alor_stage = 9;
	/* Vendor timing, taken from the phone's own ft3683g driver: assert reset
	 * low, hold, release, then wait 300 ms before the first transfer. The
	 * reset pad has no pull in the device tree (pmx_ts_reset_active sets
	 * bias-disable), so pass pull 0 rather than the 3 used for IRQ. */
	if (!pins_loader) {
		ft_gpio(FT_RESET, 0, 0, 8, 0);
		ft_sleep(20);
		ft_gpio(FT_RESET, 0, 0, 8, 1);
		ft_sleep(300);
	} else {
		/* alor_touch_pins() in the loader already pulsed this pad and waited
		 * the vendor's 300 ms, so the chip is out of reset by the time the
		 * kernel runs. Touching gpio161 here would be a TLMM access. */
		printf("ALOR TOUCH: alor_pins_loader=1, reset was done by the loader\n");
	}
	alor_blip(6);
	if (alor_ws <= 7) {
		alor_done = 1;
		printf("ALOR TOUCH: stage %u done after reset, holding\n", alor_ws);
		for (;;) {
			ft_sleep(5000);
		}
	}
	alor_stage = 10;
	/* Two ID pairs are accepted on purpose.
	 *
	 * This file has always demanded 0xA3==0x36 / 0x9F==0x83, but the FocalTech
	 * path in goodix_touch.c -- the one FocalTech path in this tree that was
	 * tested against real hardware (Ace 6T, same SM8845) -- requires 0x56/0x62
	 * and logs it as "FT3680". Both cannot be right for the same panel, and no
	 * run has ever confirmed which pair this Neo8 answers with, because every
	 * capture read back 00/ff.
	 *
	 * Accepting both turns the next boot into the experiment: whatever the chip
	 * says, the driver either proceeds (a real ID) or reports the raw bytes
	 * (silence). Refusing one of them costs a whole boot to learn nothing. */
	bool id_a = false, id_b = false;
	for (unsigned t = 0; t < 5; t++) {
		rc3 = alor_reg_read(0xa3, &id3, 1);
		rc9f = alor_reg_read(0x9f, &id9f, 1);
		id_a = !rc3 && id3 == 0x36 && id9f == 0x83;
		id_b = !rc3 && id3 == 0x56 && id9f == 0x62;
		if (id_a || id_b) {
			break;
		}
		ft_sleep(200);
	}
	rcfw = alor_reg_read(0xa6, &fw6, 1);
	printf("ALOR TOUCH: FT3683 id %02x/%02x fw %02x rc %d/%d/%d%s\n", id3, id9f, fw6, rc3, rc9f, rcfw,
	    id_b && !id_a ? " (matched the 56/62 pair)" : "");
	/* Green for either accepted pair; red only when the chip is silent or
	 * answers something unrecognised. The old marker here was unconditional
	 * green, so a silent chip and a working one looked identical on the panel. */
	bool id_ok = id_a || id_b;
	alor_mark(7, id_ok);
	/* Redraw the verdict for ten seconds whatever it says. The kernel console
	 * shares this framebuffer and writes text straight over the blocks, so a
	 * single write can vanish under a later printf. Repainting is the only way
	 * to guarantee the colour is still there when someone looks at the panel. */
	for (unsigned r = 0; r < 100; r++) {
		alor_verdict(id_ok);
		ft_sleep(100);
	}
	alor_verdict(id_ok);
	if (alor_ws <= 8) {
		alor_done = 1;
		printf("ALOR TOUCH: stage %u done after id, holding\n", alor_ws);
		for (;;) {
			ft_sleep(5000);
		}
	}
	for (unsigned t = 0; t < 3; t++) {
		rc9e = alor_reg_write(0x9e, &zero, 1);
		ft_sleep(1);
		if (!rc9e) {
			rc9e = alor_reg_read(0x9e, &mode, 1);
		}
		if (!rc9e && mode == 0) {
			break;
		}
	}
	alor_blip(8);
	alor_stage = 11;
	printf("ALOR TOUCH: TIC 9E=%02x rc %d\n", mode, rc9e);
	alor_done = 1;
	alor_stage = 12;
	for (;;) {
		bool irq = !(ft_rd(ft.tlmm, FT_GPIO(FT_IRQ) + 4) & 1u);
		if (irq || (++frames & 7u) == 0) {
			memset(frame, 0, sizeof(frame));
			if (alor_reg_read(0x01, frame, sizeof(frame))) {
				bad++;
			} else if (frame[0] == 0xff && frame[1] == 0xff) {
				(void)alor_reg_write(0x9e, &zero, 1);   /* firmware restarted */
			} else {
				int ids[10], xs[10], ys[10];
				unsigned n = alor_frame_decode(frame, ids, xs, ys);
				uint32_t mask = 0;
				if (n) {
					good++;
				}
				for (unsigned i = 0; i < n; i++) {
					unsigned id = (unsigned)ids[i];
					if (id >= FT_MAX) {
						continue;
					}
					ft.x[id] = xs[i];
					ft.y[id] = ys[i];
					mask |= 1u << id;
				}
				for (unsigned id = 0; id < FT_MAX; id++) {
					uint32_t bit = 1u << id;
					if (mask & bit) {
						sweet_hid_touch((int)id, 1, ft.x[id], ft.y[id]);
					} else if (ft.active & bit) {
						sweet_hid_touch((int)id, 0, ft.x[id], ft.y[id]);
					}
				}
				ft.active = mask;
				if (printed < 6) {
					printed++;
					printf("ALOR TOUCH: frame %u type %u n %u hdr %02x %02x %02x %02x\n",
					    frames, frame[1] >> 4, n, frame[0], frame[1], frame[2], frame[3]);
				}
			}
			if ((frames & 255u) == 0) {
				printf("ALOR TOUCH: %u frames, %u bad, %u contacts\n", frames, bad, good);
			}
		}
		ft_sleep(8);
	}
}

static void
ft_thread(void *arg, wait_result_t result)
{
	static const uint8_t id_cmd[] = {0xfb, 0x20, 0x00, 0x00, 0x00}, fifo[] = {0x87};
	static const uint8_t sense[] = {0xa0, 0x00, 0x01};
	uint8_t ev[8] = {0}, chip[2] = {0};
	unsigned delay_s = 8, errors = 0;
	unsigned poll_us = 4000, idle_us = 8000, offsup = 1;
	bool irq_seen = false, was_off = false;
	(void)arg;
	(void)result;
	(void)PE_parse_boot_argn("fuxitouchdelay", &delay_s, sizeof(delay_s));
	(void)PE_parse_boot_argn("fuxitouch_poll_us", &poll_us, sizeof(poll_us));
	(void)PE_parse_boot_argn("fuxitouch_idle_us", &idle_us, sizeof(idle_us));
	(void)PE_parse_boot_argn("fuxitouch_offsuppress", &offsup, sizeof(offsup));
	alor_ws = 9;
	(void)PE_parse_boot_argn("wifi_step", &alor_ws, sizeof(alor_ws));
	if (alor_ws < 1 || alor_ws > 9) {
		alor_ws = 9;
	}
	poll_us = poll_us < 1000 ? 1000 : poll_us;
	idle_us = idle_us < poll_us ? poll_us : idle_us;
	/* alor_bb=1 transfers over bit-banged GPIO instead of the QUP/GPI path.
	 * That path needs the SE4 clock, the apps-SMMU SMRs and a live GPII
	 * channel, and bringing any of those up from XNU on this board has hung
	 * the bus every time. Bit-banging touches only the four TLMM pads. */
	uint32_t bb = 0;
	(void)PE_parse_boot_argn("alor_bb", &bb, sizeof(bb));
	/* Set by the runner: the loader has already programmed every pad on this
	 * chip, so this driver must not touch TLMM at all. See the comment at the
	 * pad block below for the 900E history behind it. */
	int pins_loader = 0;
	(void)PE_parse_boot_argn("alor_pins_loader", &pins_loader,
	    sizeof(pins_loader));
	ft.bb = bb != 0;
	printf("FUXI TOUCH: starting in %u s (poll %u us active, %u us idle, %s)\n", delay_s,
	    poll_us, idle_us, ft.bb ? "gpio bit-bang" : "QUP/GPI");
	alor_fb_init();
	alor_blip(0);
	ft_sleep(delay_s * 1000u);
	alor_stage = 1;
	ft.tlmm = ml_io_map(FT_TLMM_PA, FT_TLMM_SIZE);
	if (!ft.tlmm) {
		printf("ALOR TOUCH: cannot map TLMM\n");
		goto out;
	}
	if (ft.bb) {
		/* Bit-bang needs only TLMM -- no GCC, no SE, no GPI, no SMMU, no DMA.
		 * Power the chip first: without the rails every transfer reads 0x00
		 * regardless of how correct the protocol is. */
		alor_stage = 7;
		ft.rails = ft_rails_up();
		printf("ALOR TOUCH: bit-bang mode, TLMM mapped, no QUP/SMMU/DMA, rails %u\n",
		    ft.rails);
		alor_blip(4);
		alor_fts_run();
		goto out;
	}
	ft.gcc = ml_io_map(FT_GCC_PA, 0x1f0000);
	ft.se = ml_io_map(FT_SE_PA, 0x4000);
	ft.gpi = ml_io_map(FT_GPI_PA, FT_GPI_SIZE);
	ft.smmu = ml_io_map(FT_SMMU_PA, 0x1000);
	if (!ft.gcc || !ft.se || !ft.gpi || !ft.smmu) {
		goto out;
	}
	alor_fb_init();
	alor_blip(0);
	if (kmem_alloc_contig(kernel_map, &ft.dma, PAGE_SIZE, PAGE_MASK, 0, 0,
	    KMA_DATA | KMA_ZERO | KMA_PERMANENT, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		printf("FUXI TOUCH: no DMA memory\n");
		goto out;
	}
	ppnum_t pn = pmap_find_phys(kernel_pmap, (addr64_t)ft.dma);
	if (!pn) {
		goto out;
	}
	ft.dma_pa = ptoa_64(pn);
	alor_stage = 2;
	int ierr;
	if (alor_ws >= 2 && (ierr = ft_clock())) { alor_stage = 3; printf("ALOR TOUCH: clock %d\n", ierr); goto out; }
	alor_blip(1);
	/* Power the panel before anything talks to it. Every capture that read
	 * 0x00 from the chip also had rail=0; without this the protocol is moot. */
	alor_stage = 3;
	ft.rails = ft_rails_up();
	alor_blip(1);
	if (!ft.rails) {
		printf("ALOR TOUCH: rails failed; the chip cannot answer\n");
	}
	alor_stage = 4;
	if (alor_ws >= 3 && (ierr = ft_se_init())) { printf("ALOR TOUCH: se %d\n", ierr); goto out; }
	alor_blip(2);
	alor_stage = 5;
	if (alor_ws >= 4 && (ierr = ft_smmu())) { printf("ALOR TOUCH: smmu %d\n", ierr); goto out; }
	alor_blip(3);
	alor_stage = 6;
	if (alor_ws >= 5 && (ierr = ft_gpi_init())) { printf("ALOR TOUCH: gpi %d\n", ierr); goto out; }
	alor_blip(4);
	alor_stage = 7;
	printf("FUXI TOUCH: stages done up to %u, DMA page %#llx\n", alor_ws, ft.dma_pa);
	if (alor_ws <= 5) {
		alor_done = 1;
		for (;;) {
			ft_sleep(5000);
		}
	}
	alor_fts_run();
	goto out;
	/*
	 * IRQ is level low while the FIFO holds events. Check it every poll
	 * period (absolute deadlines, no timer leeway) and read the FIFO every
	 * 32 ms regardless in case the IC does not drive the line.
	 */
	uint64_t poll_abs, idle_abs, off_abs, recent_abs, sweep_abs;
	nanoseconds_to_absolutetime(poll_us * 1000ULL, &poll_abs);
	nanoseconds_to_absolutetime(idle_us * 1000ULL, &idle_abs);
	nanoseconds_to_absolutetime(32 * NSEC_PER_MSEC, &off_abs);
	nanoseconds_to_absolutetime(32 * NSEC_PER_MSEC, &sweep_abs);
	nanoseconds_to_absolutetime(150 * NSEC_PER_MSEC, &recent_abs);
	uint64_t next = mach_absolute_time(), last_irq = 0, last_read = 0;
	for (;;) {
		uint64_t now = mach_absolute_time();
		bool off = offsup && sweet_fb_display_off;
		if (off != was_off) {
			was_off = off;
			if (off && ft.active) {
				ft_report(0);   /* release fingers held when the display went off */
			}
		}
		bool irq = !(ft_rd(ft.tlmm, FT_GPIO(FT_IRQ) + 4) & 1u);
		if (irq) {
			last_irq = now;
			if (!irq_seen) {
				irq_seen = true;
				printf("FUXI TOUCH: IRQ line asserted\n");
			}
		}
		if (irq || ft.active || now - last_read >= sweep_abs) {
			last_read = now;
			int perr = ft_poll(!off);
			uint64_t took = mach_absolute_time() - now;
			if (ft.active) {
				ft.c_polls++;
				ft.c_poll_sum += took;
				ft.c_poll_max = took > ft.c_poll_max ? took : ft.c_poll_max;
			}
			if (perr) {
				errors++;
				if (errors == 1 || (errors % 100u) == 0) {
					printf("FUXI TOUCH: FIFO read error %d (%u in a row)\n", perr, errors);
				}
				/* Do not leave a finger stuck down while reads fail. */
				if (errors >= 2 && ft.active) {
					ft_report(0);
				}
				/* A timed-out transfer leaves TREs queued: reset the GPII rings. */
				if (errors == 3 || (errors % 50u) == 0) {
					int rerr = ft_gpi_init();
					printf("FUXI TOUCH: GPI re-init after %u errors -> %d\n", errors, rerr);
				}
			} else {
				if (errors >= 3) {
					printf("FUXI TOUCH: FIFO reads recovered after %u errors\n", errors);
				}
				errors = 0;
			}
			if (ft.ic_reset) {
				/* Controller-ready event while sensing: the IC reset itself. */
				ft.ic_reset = false;
				if (ft.active) {
					ft_report(0);
				}
				int serr = ft_tx(sense, 3, false);
				printf("FUXI TOUCH: controller reset event, sense on again -> %d\n", serr);
			}
		}
		uint64_t period = off ? off_abs :
		    (ft.active || now - last_irq < recent_abs) ? poll_abs : idle_abs;
		next += period;
		now = mach_absolute_time();
		if (next <= now) {
			next = now + period;
		}
		ft_sleep_until(next);
	}
out:
	printf("FUXI TOUCH: stopped\n");
	(void)thread_terminate(current_thread());
	thread_block(THREAD_CONTINUE_NULL);
}

static void
alor_swipe(void *arg, wait_result_t result)
{
	static const unsigned ys[6] = {2420u, 2000u, 1550u, 1100u, 700u, 380u};
	unsigned x;
	(void)arg;
	(void)result;
	ft_sleep(20000);
	x = 562u * 65535u / 1124u;
	for (unsigned round = 0; round < 3; round++) {
		printf("ALOR TOUCH: posting the unlock swipe %u\n", round);
		redwood_touch_post(REDWOOD_TOUCH_DOWN, 1, x, ys[0] * 65535u / 2435u);
		for (unsigned i = 1; i < 6; i++) {
			ft_sleep(40);
			redwood_touch_post(REDWOOD_TOUCH_MOVE, 1, x, ys[i] * 65535u / 2435u);
		}
		ft_sleep(40);
		redwood_touch_post(REDWOOD_TOUCH_UP, 1, x, ys[5] * 65535u / 2435u);
		ft_sleep(10000);
	}
	printf("ALOR TOUCH: swipes done\n");
	(void)thread_terminate(current_thread());
	thread_block(THREAD_CONTINUE_NULL);
}

void
alor_touch_init(void)
{
	int enabled = 0;
	thread_t thread;
	if (ft.started) {
		return;
	}
	ft.started = true;
	/* Any of the three switches arms the driver. run_ios.py's default is
	 * macan_touch=1 and the loader allowlist accepts all three, so the old
	 * gate on macan_touch_diag alone meant a normal `run` boot never even
	 * reached this function's thread start. */
	if (PE_parse_boot_argn("macan_touch_diag", &enabled, sizeof(enabled)) && enabled) {
		/* diag */
	} else if (PE_parse_boot_argn("macan_touch", &enabled, sizeof(enabled)) && enabled) {
		/* normal */
	} else if (PE_parse_boot_argn("macan_touch_probe", &enabled, sizeof(enabled)) && enabled) {
		/* probe */
	} else {
		printf("ALOR TOUCH: disabled\n");
		return;
	}
	if (kernel_thread_start_priority(ft_thread, NULL, FT_PRIORITY, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
	if (kernel_thread_start(alor_watchdog, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}
