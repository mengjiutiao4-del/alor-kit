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
extern uint64_t kvtophys(vm_offset_t va);
static char ft_dma_page[16384] __attribute__((aligned(4096)));
extern kern_return_t kernel_thread_start_priority(thread_continue_t continuation,
    void *parameter, integer_t priority, thread_t *new_thread);
static bool ft_no_pio;
static int sweet_fb_display_off;        /* no display-off gating on alor */
void alor_touch_init(void);

#define FT_SE_PA     0x00a90000u
#define FT_GPI_PA    0x00a10000u   /* GPII 0: ee base 0x9f0000 + 0x20000 */
#define FT_SMMU_PA   0x15000000u
#define FT_GCC_PA    0x00100000u
#define FT_TLMM_PA   0x0f000000u   /* live alor DT: pinctrl@f000000, reg size 16 MB */
#define FT_RSC_PA    0x18900000u
#define FT_CMDB_PA   0x81c60000u
#define FT_GPIO(n)   ((uint32_t)(n) * 0x1000u)
#define ALOR_PANEL_W 1272u
#define ALOR_PANEL_H 2772u
#define FT_RESET     161u          /* DTB reset-gpio = tlmm 0xa1 */
#define FT_IRQ       55u           /* DTB irq-gpio = tlmm 0x37 */
#define FT_VDDI      1800000u      /* vcc_1v8 = RPMh L4B_E0, microvolts */
#define FT_VDD       3300000u      /* vdd_2v8 = RPMh L16B_E0, microvolts */
#define FT_MAX       10u
#define FT_EV_READY  0x03u
#define FT_EV_ENTER  0x13u
#define FT_EV_MOTION 0x23u
#define FT_EV_LEAVE  0x33u
#define FT_TYPE_HOVER 0x05u        /* ftsSoftware.h TOUCH_TYPE_HOVER: not touching */
#define FT_PRIORITY  92            /* BASEPRI_PREEMPT */

static struct {
	vm_offset_t se, gcc, tlmm, tlmm_hi, gpi, smmu;
	vm_offset_t dma;
	uint64_t dma_pa;
	unsigned depth, level, wp[2], evrp;
	bool started;
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

static volatile unsigned alor_stage, alor_done, alor_frames;
static unsigned alor_ws = 9;
static vm_offset_t alor_fb;
static uint32_t alor_fb_row;

static uint32_t alor_col[27];
static int alor_top = -1;

static void
alor_blip_c(unsigned n, uint32_t colour)
{
	if (!alor_fb || n > 26) {
		return;
	}
	alor_col[n] = colour;
	if ((int)n > alor_top) {
		alor_top = (int)n;
	}
	for (unsigned y = 0; y < 36; y++) {
		volatile uint32_t *p = (volatile uint32_t *)(alor_fb +
		    (vm_size_t)(40 + y) * alor_fb_row + (20 + 44 * n) * 4);
		for (unsigned x = 0; x < 36; x++) {
			p[x] = colour;
		}
	}
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
alor_painter(void *arg, wait_result_t result)
{
	(void)arg;
	(void)result;
	for (;;) {
		ft_sleep(2000);
		if (!alor_fb) {
			continue;
		}
		for (int n = 0; n <= alor_top && n < 27; n++) {
			alor_blip_c(n, alor_col[n]);
		}
	}
}

static void
alor_watchdog(void *arg, wait_result_t result)
{
	(void)arg;
	(void)result;
	ft_sleep(60000);
	if (!alor_done) {
		printf("ALOR TOUCH: still at stage %u after 60 s\n", alor_stage);
	}
	ft_sleep(60000);
	if (!alor_done && !alor_frames) {
		/* Keep the kernel alive: a panic here leaves the phone in the
		 * Qualcomm crash dump, which needs a hand on the power key. */
		printf("ALOR TOUCH: no frame after 120 s, stage %u\n", alor_stage);
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
	/* Pins past 159 live above the window the low pins use. */
	vm_offset_t base = ft.tlmm;
	uint32_t off = FT_GPIO(n);
	if (n >= 160 && ft.tlmm_hi) {
		base = ft.tlmm_hi;
		off = FT_GPIO(n) - 0xa0000u;
	}
	if (out >= 0) {
		ft_wr(base, off + 4, out ? 2u : 0u);
	}
	uint32_t old = ft_rd(base, off);
	uint32_t cfg = (old & ~(0x3ffu | 0x1000u)) | pull | (func << 2) |
	    ((ma / 2 - 1) << 6) | (out >= 0 ? 0x200u : 0u);
	if (old & 0x800u) {
		cfg |= 0x1000u;        /* egpio enable: without it the mux never reaches the pad */
	}
	ft_wr(base, off, cfg);
}

static uint32_t
ft_pin_raw(unsigned n, unsigned off)
{
	vm_offset_t base = ft.tlmm;
	uint32_t o = FT_GPIO(n);
	if (n >= 160 && ft.tlmm_hi) {
		base = ft.tlmm_hi;
		o = FT_GPIO(n) - 0xa0000u;
	}
	return ft_rd(base, o + off);
}

static void
ft_pin_dump(const char *when)
{
	static const unsigned pins[6] = { 48, 49, 50, 51, 55, 161 };
	printf("ALOR TOUCH: pins %s:", when);
	for (unsigned i = 0; i < 6; i++) {
		printf(" %u=%#x/%#x", pins[i], ft_pin_raw(pins[i], 0), ft_pin_raw(pins[i], 4));
	}
	printf("\n");
}

static bool
ft_on(uint32_t cbcr)
{
	return !(ft_rd(ft.gcc, cbcr) & 0x80000000u);
}

/* RPMh command path, copied from macan's goodix driver: the touch rails are
 * not voted by the UEFI, so the 1.8 V (L4B_E0) and 3.3 V (L16B_E0) supplies
 * have to be switched on here. Voltages are in millivolts. */
#define FT_TCS(reg)        (0xd00u + (reg))
#define TCS_IRQ_ENABLE     0x00
#define TCS_IRQ_STATUS     0x04
#define TCS_IRQ_CLEAR      0x08
#define TCS_WAIT_FOR_COMPLETION 0x10
#define TCS_CONTROL        0x14
#define TCS_CONTROL_ENABLE (1u << 16)
#define TCS_CONTROL_TRIGGER (1u << 24)
#define TCS_STATUS         0x18
#define TCS_CMD_ENABLE     0x1c
#define TCS_CMD_MSGID(n)   (0x30 + 20 * (n))
#define TCS_CMD_ADDR(n)    (0x34 + 20 * (n))
#define TCS_CMD_DATA(n)    (0x38 + 20 * (n))
#define MSGID_WRITE_4_BYTES 0x10008
#define MSGID_RESPONSE      0x100
#define VRM_VOLTAGE 0
#define VRM_ENABLE  4
#define VRM_MODE    8
#define VRM_MODE_LDO_HPM 7

struct ft_cmd_db_slave {
	uint16_t id, header_offset, data_offset, count, version, reserved[3];
};
struct ft_cmd_db_header {
	uint32_t version;
	uint8_t magic[4];
	struct ft_cmd_db_slave slave[8];
	uint32_t checksum, reserved;
	uint8_t data[];
};
struct ft_cmd_db_entry {
	char id[8];
	uint32_t priority[2];
	uint32_t address;
	uint16_t length, offset;
};

static vm_offset_t ft_rsc, ft_cmdb;
extern vm_offset_t ml_io_map_wcomb(vm_offset_t, vm_size_t);

static uint32_t
ft_rpmh_address(const char *name)
{
	const struct ft_cmd_db_header *db = (const struct ft_cmd_db_header *)ft_cmdb;
	char id[8] = { 0 };
	if (!ft_cmdb || db->magic[0] != 0xdb || db->magic[1] != 0x30 ||
	    db->magic[2] != 0x03 || db->magic[3] != 0x0c) {
		printf("ALOR TOUCH: no RPMh command database\n");
		return 0;
	}
	strlcpy(id, name, sizeof(id));
	for (unsigned s = 0; s < 8; s++) {
		const struct ft_cmd_db_slave *slave = &db->slave[s];
		const struct ft_cmd_db_entry *entry =
		    (const struct ft_cmd_db_entry *)(db->data + slave->header_offset);
		if (slave->id == 0 || slave->header_offset >= 0x20000u - 0x1000u ||
		    slave->count > 512) {
			continue;
		}
		for (unsigned i = 0; i < slave->count; i++) {
			if (memcmp(entry[i].id, id, sizeof(id)) == 0) {
				return entry[i].address;
			}
		}
	}
	return 0;
}

static bool
ft_rpmh_wait(uint32_t reg, uint32_t mask, uint32_t value, const char *what)
{
	for (int i = 0; i < 2000; i++) {
		if ((ft_rd(ft_rsc, FT_TCS(reg)) & mask) == value) {
			return true;
		}
		delay(100);
	}
	printf("ALOR TOUCH: RPMh timeout on %s (%#x)\n", what,
	    ft_rd(ft_rsc, FT_TCS(reg)));
	return false;
}

static bool
ft_rpmh_write(const uint32_t *address, const uint32_t *data, unsigned count)
{
	uint32_t irq_enable = ft_rd(ft_rsc, FT_TCS(TCS_IRQ_ENABLE));
	bool done;
	for (int i = 0; ft_rd(ft_rsc, FT_TCS(TCS_STATUS)) == 0; i++) {
		if (i > 2000) {
			printf("ALOR TOUCH: RPMh command slots stay busy\n");
			return false;
		}
		delay(100);
	}
	ft_wr(ft_rsc, FT_TCS(TCS_CONTROL),
	    ft_rd(ft_rsc, FT_TCS(TCS_CONTROL)) & ~TCS_CONTROL_TRIGGER);
	ft_wr(ft_rsc, FT_TCS(TCS_CONTROL),
	    ft_rd(ft_rsc, FT_TCS(TCS_CONTROL)) & ~TCS_CONTROL_ENABLE);
	ft_wr(ft_rsc, FT_TCS(TCS_IRQ_CLEAR), 1);
	for (unsigned i = 0; i < count; i++) {
		ft_wr(ft_rsc, FT_TCS(TCS_CMD_MSGID(i)), MSGID_WRITE_4_BYTES | MSGID_RESPONSE);
		ft_wr(ft_rsc, FT_TCS(TCS_CMD_ADDR(i)), address[i]);
		ft_wr(ft_rsc, FT_TCS(TCS_CMD_DATA(i)), data[i]);
	}
	ft_wr(ft_rsc, FT_TCS(TCS_WAIT_FOR_COMPLETION), (1u << count) - 1);
	ft_wr(ft_rsc, FT_TCS(TCS_CMD_ENABLE), (1u << count) - 1);
	ft_wr(ft_rsc, FT_TCS(TCS_IRQ_ENABLE), irq_enable | 1);
	ft_wr(ft_rsc, FT_TCS(TCS_CONTROL), TCS_CONTROL_ENABLE);
	ft_wr(ft_rsc, FT_TCS(TCS_CONTROL), TCS_CONTROL_ENABLE | TCS_CONTROL_TRIGGER);
	done = ft_rpmh_wait(TCS_IRQ_STATUS, 1, 1, "the request");
	ft_wr(ft_rsc, FT_TCS(TCS_CONTROL),
	    ft_rd(ft_rsc, FT_TCS(TCS_CONTROL)) & ~TCS_CONTROL_TRIGGER);
	ft_wr(ft_rsc, FT_TCS(TCS_CONTROL),
	    ft_rd(ft_rsc, FT_TCS(TCS_CONTROL)) & ~TCS_CONTROL_ENABLE);
	ft_wr(ft_rsc, FT_TCS(TCS_CMD_ENABLE), 0);
	ft_wr(ft_rsc, FT_TCS(TCS_WAIT_FOR_COMPLETION), 0);
	ft_wr(ft_rsc, FT_TCS(TCS_IRQ_CLEAR), 1);
	ft_wr(ft_rsc, FT_TCS(TCS_IRQ_ENABLE), irq_enable);
	return done;
}

static bool
ft_power_on(void)
{
	ft_rsc = ml_io_map(FT_RSC_PA, 0x10000);
	ft_cmdb = ml_io_map_wcomb(FT_CMDB_PA, 0x20000);
	if (!ft_rsc || !ft_cmdb) {
		return false;
	}
	uint32_t vddi = ft_rpmh_address("L4B_E0");
	uint32_t vdd = ft_rpmh_address("L16B_E0");
	printf("ALOR TOUCH: rails L4B_E0 %#x L16B_E0 %#x\n", vddi, vdd);
	if (!vddi || !vdd) {
		return false;
	}
	/* The setpoints are already what Android programmed; only flip the
	 * enables, so a bad voltage unit can never collapse the rails. */
	uint32_t a1[2] = { vddi + VRM_ENABLE, vddi + VRM_MODE };
	uint32_t d1[2] = { 1, VRM_MODE_LDO_HPM };
	uint32_t a2[2] = { vdd + VRM_ENABLE, vdd + VRM_MODE };
	uint32_t d2[2] = { 1, VRM_MODE_LDO_HPM };
	if (!ft_rpmh_write(a1, d1, 2) || !ft_rpmh_write(a2, d2, 2)) {
		return false;
	}
	return true;
}

static int
ft_clock(void)
{
	bool m0, s0, e0, m1 = false, s1 = false, e1 = false, ack = true;
	/* Register truth from gcc-sm8750: wrapper AHBs at 0x23140/0x23144, SE4's
	 * clock at 0x183cc, all voted through 0x52008 bits 20/21/26. 0x18004 is
	 * SE0's clock and 0x185ec belongs to something else entirely. */
	m0 = !(ft_rd(ft.gcc, 0x23140) & 0x80000000u);
	s0 = !(ft_rd(ft.gcc, 0x23144) & 0x80000000u);
	e0 = !(ft_rd(ft.gcc, 0x183cc) & 0x80000000u);
	alor_mark(10, m0);
	alor_mark(11, s0);
	alor_mark(12, e0);
	printf("ALOR TOUCH: clock before m_ahb=%d s_ahb=%d se4=%d (%#x %#x %#x)\n", m0, s0, e0, ft_rd(ft.gcc, 0x23140), ft_rd(ft.gcc, 0x23144), ft_rd(ft.gcc, 0x183cc));
	uint32_t vote = ft_rd(ft.gcc, 0x52008);
	ft_wr(ft.gcc, 0x52008, vote | (0xfu << 18) | (1u << 26));
	if (!e0) {
		/* SE4 RCG on the XO branch: CFG source 0 divider 1, then the update handshake. */
		ft_wr(ft.gcc, 0x183e0, (ft_rd(ft.gcc, 0x183e0) & ~0x371fu) | 1u);
		ft_wr(ft.gcc, 0x183dc, ft_rd(ft.gcc, 0x183dc) | 1u);
		for (int i = 0; i < 500 && (ft_rd(ft.gcc, 0x183dc) & 1u); i++) {
			delay(10);
		}
		ack = !(ft_rd(ft.gcc, 0x183dc) & 1u);
	}
	alor_mark(13, ack);
	for (int i = 0; i < 2000; i++) {
		m1 = !(ft_rd(ft.gcc, 0x23140) & 0x80000000u);
		s1 = !(ft_rd(ft.gcc, 0x23144) & 0x80000000u);
		e1 = !(ft_rd(ft.gcc, 0x183cc) & 0x80000000u);
		if (m1 && s1 && e1) {
			break;
		}
		delay(10);
	}
	m1 = !(ft_rd(ft.gcc, 0x23140) & 0x80000000u);
	s1 = !(ft_rd(ft.gcc, 0x23144) & 0x80000000u);
	e1 = !(ft_rd(ft.gcc, 0x183cc) & 0x80000000u);
	if (!m1 || !s1 || !e1) {
		/* Second try: some supplies need a beat before the branch latches. */
		delay(5000);
		ft_wr(ft.gcc, 0x52008, ft_rd(ft.gcc, 0x52008) | (0xfu << 18) | (1u << 26));
		for (int i = 0; i < 2000; i++) {
			m1 = !(ft_rd(ft.gcc, 0x23140) & 0x80000000u);
			s1 = !(ft_rd(ft.gcc, 0x23144) & 0x80000000u);
			e1 = !(ft_rd(ft.gcc, 0x183cc) & 0x80000000u);
			if (m1 && s1 && e1) {
				break;
			}
			delay(10);
		}
	}
	m1 = !(ft_rd(ft.gcc, 0x23140) & 0x80000000u);
	s1 = !(ft_rd(ft.gcc, 0x23144) & 0x80000000u);
	e1 = !(ft_rd(ft.gcc, 0x183cc) & 0x80000000u);
	alor_mark(14, m1 && s1);
	alor_mark(15, e1);
	printf("ALOR TOUCH: clock after m_ahb=%d s_ahb=%d se4=%d ack=%d (%#x %#x %#x)\n", m1, s1, e1, ack, ft_rd(ft.gcc, 0x23140), ft_rd(ft.gcc, 0x23144), ft_rd(ft.gcc, 0x183cc));
	if (!m1 || !s1 || !e1) {
		return EIO;
	}
	return 0;
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
			/* The stream already exists (the firmware may have described
			 * it for translation); force bypass so our physical rings
			 * are used as they are. */
			ft_wr(ft.smmu, 0xc00 + 4 * i, 1u << 16);
			continue;
		}
		if (freei == ~0u) {
			return ENOSPC;
		}
		/* Our rings are addressed physically, and nothing here builds a
		 * translation table for context bank 4: bypass is the only safe
		 * setting (type 1 at S2CR bit 16). */
		ft_wr(ft.smmu, 0xc00 + 4 * freei, 1u << 16);
		ft_wr(ft.smmu, 0x800 + 4 * freei, 0x80000000u | sid[j]);
		printf("FUXI TOUCH: SMR %u = sid %#x -> S2CR %#x\n", freei, sid[j],
		    ft_rd(ft.smmu, 0xc00 + 4 * freei));
	}
	/* Read back: a stream match register that did not stick means the
	 * GPI can never fetch its rings. */
	for (unsigned j = 0; j < 2; j++) {
		unsigned i;
		for (i = 0; i < nsmr; i++) {
			uint32_t smr = ft_rd(ft.smmu, 0x800 + 4 * i);
			if ((smr & 0x80000000u) &&
			    ((sid[j] ^ smr) & ~(smr >> 16) & 0x7fff) == 0) {
				break;
			}
		}
		if (i == nsmr) {
			return EIO;
		}
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
	ft_no_pio = (ft_rd(ft.se, 0x064) & 1u) != 0;
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
		unsigned hops = 0;
		while (hw != (uint32_t)ft.dma_pa + FT_EV_OFF + 16 * ft.evrp && hops++ < FT_EVRING) {
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
		if (hops >= FT_EVRING && hw != (uint32_t)ft.dma_pa + FT_EV_OFF + 16 * ft.evrp) {
			return EIO;   /* the write pointer left our ring: do not spin here */
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
		/* Super-resolution coordinates, 10x the 1080x2400 panel. */
		int x = ((ev[3] << 8) | ev[2]) / 10, y = ((ev[5] << 8) | ev[4]) / 10;
		ft.x[id] = x > 1079 ? 1079 : x;
		ft.y[id] = y > 2399 ? 2399 : y;
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

/* ---- FT3683 high-bandwidth protocol over the GPI DMA path above ---- */
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


/* ---- GPIO bit-bang SPI ------------------------------------------------
 * The QUP1 SE4 engine wedges this interconnect the moment it is asked to
 * move data, and its FIFO interface is disabled. The HBP frames are tiny,
 * so clock them out by hand on the same four pads (48 MISO, 49 MOSI,
 * 50 CLK, 51 CS), exactly as macan's goodix driver falls back to.
 */
static unsigned ft_bb_miso = 48u, ft_bb_mosi = 49u;
static unsigned ft_bb_clk = 50u, ft_bb_cs = 51u;
#define FT_BB_MISO ft_bb_miso
#define FT_BB_MOSI ft_bb_mosi
#define FT_BB_CLK  ft_bb_clk
#define FT_BB_CS   ft_bb_cs

static void
ft_bb_pin(unsigned n, int out, int level)
{
	ft_gpio(n, 0, 0, 6, out >= 0 ? level : -1);
}

static void
ft_bb_init(void)
{
	ft_bb_pin(ft_bb_miso, -1, 0);
	ft_bb_pin(ft_bb_mosi, 0, 0);
	ft_bb_pin(ft_bb_clk, 0, 0);
	ft_bb_pin(ft_bb_cs, 0, 1);
}

static void
ft_bb_level(unsigned n, int level)
{
	ft_wr(ft.tlmm, FT_GPIO(n) + 4, level ? 2u : 0u);
}

static int
ft_bb_xfer(const uint8_t *tx, uint8_t *rx, unsigned len)
{
	/* Same shape as the macan fallback that is known to work on FT368x:
	 * no delays between edges, CS framed by a long idle gap. */
	ft_bb_level(FT_BB_CS, 0);
	for (unsigned i = 0; i < len; i++) {
		uint8_t out = tx ? tx[i] : 0, in = 0;
		for (int bit = 7; bit >= 0; bit--) {
			ft_bb_level(FT_BB_MOSI, (out >> bit) & 1);
			ft_bb_level(FT_BB_CLK, 1);
			in = (uint8_t)((in << 1) | (ft_rd(ft.tlmm, FT_GPIO(FT_BB_MISO) + 4) & 1u));
			ft_bb_level(FT_BB_CLK, 0);
		}
		if (rx) {
			rx[i] = in;
		}
	}
	ft_bb_level(FT_BB_CS, 1);
	delay(150);
	return 0;
}

/* ---- Hardware GENI SPI (QUP1 SE4), sequence lifted from macan's proven
 * goodix_touch.c, which runs on the same SoC family. ---- */
#define FT_QUP_PA      0x00ac0000u
#define G_FORCE_DEFAULT 0x020u
#define G_OUTPUT_CTRL   0x024u
#define G_CGC_CTRL      0x028u
#define G_STATUS        0x040u
#define G_SER_M_CLK     0x048u
#define G_IF_DISABLE    0x064u
#define G_FW_REV        0x068u
#define G_CLK_SEL       0x07cu
#define S_CPHA          0x224u
#define S_LOOPBACK      0x22cu
#define S_CPOL          0x230u
#define S_DEMUX_INV     0x24cu
#define S_DEMUX_SEL     0x250u
#define G_BYTE_GRAN     0x254u
#define G_DMA_MODE_EN   0x258u
#define S_TRANS_CFG     0x25cu
#define G_TX_PACK0      0x260u
#define G_TX_PACK1      0x264u
#define S_WORD_LEN      0x268u
#define S_TX_LEN        0x26cu
#define S_RX_LEN        0x270u
#define S_DELAY_CNT     0x278u
#define G_RX_PACK0      0x284u
#define G_RX_PACK1      0x288u
#define G_M_CMD0        0x600u
#define G_M_CMD_CTRL    0x604u
#define G_M_IRQ_STATUS  0x610u
#define G_M_IRQ_EN      0x614u
#define G_M_IRQ_CLEAR   0x618u
#define G_S_IRQ_CLEAR   0x648u
#define G_TX_FIFO       0x700u
#define G_RX_FIFO       0x780u
#define G_TX_FIFO_ST    0x800u
#define G_RX_FIFO_ST    0x804u
#define G_TX_WM         0x80cu
#define G_RX_WM         0x810u
#define G_RX_RFR_WM     0x814u
#define G_DMA_TX_CLR    0xc44u
#define G_DMA_RX_CLR    0xd44u
#define G_GSI_EVENT_EN  0xe18u
#define G_IRQ_EN        0xe1cu
#define G_HW_PARAM_0    0xe24u
#define G_DMA_GEN_CFG   0xe30u
#define G_M_CMD_DONE    (1u << 0)
#define G_M_CMD_CANCEL  (1u << 4)
#define G_M_CMD_ABORT   (1u << 5)
#define G_M_ERRORS      ((1u << 1) | (1u << 2) | (1u << 3) | (1u << 24) | (1u << 25) | (1u << 28) | (1u << 29))
#define G_SPI_FULL_DUPLEX (3u << 27)
#define G_U32(off)      ft_rd(ft.se, (off))
#define G_W32(off, v)   ft_wr(ft.se, (off), (v))

static uint32_t ft_fifo_depth;
static vm_offset_t ft_qup;
static bool ft_on_geni;

static int
ft_geni_init(uint32_t divider)
{
	if (((G_U32(G_FW_REV) >> 8) & 0xff) != 1) {
		return ENODEV;
	}
	if (G_U32(G_IF_DISABLE) & 1u) {
		return EBUSY;
	}
	ft_fifo_depth = (G_U32(G_HW_PARAM_0) >> 16) & 0x3f;
	if (ft_fifo_depth < 4) {
		return ENODEV;
	}
	G_W32(G_GSI_EVENT_EN, 0);
	G_W32(G_M_IRQ_CLEAR, 0xffffffffu);
	G_W32(G_S_IRQ_CLEAR, 0xffffffffu);
	G_W32(G_DMA_TX_CLR, 0xffffffffu);
	G_W32(G_DMA_RX_CLR, 0xffffffffu);
	G_W32(G_CGC_CTRL, G_U32(G_CGC_CTRL) | 0x7fu);
	G_W32(G_DMA_GEN_CFG, G_U32(G_DMA_GEN_CFG) | 0xfu);
	G_W32(G_OUTPUT_CTRL, 0x7fu);
	G_W32(G_FORCE_DEFAULT, 1);
	G_W32(G_DMA_MODE_EN, G_U32(G_DMA_MODE_EN) & ~1u);
	G_W32(G_RX_WM, ft_fifo_depth - 3);
	G_W32(G_RX_RFR_WM, ft_fifo_depth - 2);
	G_W32(G_TX_WM, 0);
	G_W32(G_IRQ_EN, G_U32(G_IRQ_EN) | 0xfu);
	G_W32(G_M_IRQ_EN, G_U32(G_M_IRQ_EN) | 0x7fc0007fu);
	G_W32(S_LOOPBACK, 0);
	G_W32(S_DEMUX_SEL, 0);
	G_W32(S_CPHA, 0);
	G_W32(S_CPOL, 0);
	G_W32(S_DEMUX_INV, 0);
	G_W32(S_TRANS_CFG, G_U32(S_TRANS_CFG) & ~2u);
	G_W32(S_DELAY_CNT, 0);
	G_W32(G_TX_PACK0, 0x0007f8fe);
	G_W32(G_TX_PACK1, 0x000ffefe);
	G_W32(G_RX_PACK0, 0x0007f8fe);
	G_W32(G_RX_PACK1, 0x000ffefe);
	G_W32(G_BYTE_GRAN, 0);
	G_W32(S_WORD_LEN, 8 - 4);
	G_W32(G_CLK_SEL, 0);
	G_W32(G_SER_M_CLK, ((divider ? divider : 1) << 4) | 1u);
	return 0;
}

static void
ft_geni_drain(uint8_t *rx, unsigned len, unsigned *received)
{
	uint32_t status = G_U32(G_RX_FIFO_ST), words = status & 0x01ffffffu, bytes = words * 4;
	if (status & 0x80000000u) {
		uint32_t valid = (status >> 28) & 7u;
		if (valid && valid < 4) {
			bytes -= 4 - valid;
		}
	}
	for (uint32_t i = 0; i < words; i++) {
		uint32_t word = G_U32(G_RX_FIFO);
		for (int j = 0; j < 4 && bytes; j++, bytes--) {
			if (*received < len) {
				if (rx) {
					rx[*received] = (uint8_t)(word >> (8 * j));
				}
				(*received)++;
			}
		}
	}
}

static void
ft_geni_recover(void)
{
	G_W32(G_M_CMD_CTRL, 1u << 2);
	for (int i = 0; i < 2000 && !(G_U32(G_M_IRQ_STATUS) & G_M_CMD_CANCEL); i++) {
		delay(1);
	}
	if (!(G_U32(G_M_IRQ_STATUS) & G_M_CMD_CANCEL)) {
		G_W32(G_M_CMD_CTRL, 1u << 1);
		for (int i = 0; i < 2000 && !(G_U32(G_M_IRQ_STATUS) & G_M_CMD_ABORT); i++) {
			delay(1);
		}
	}
	for (int i = 0; i < 64 && (G_U32(G_RX_FIFO_ST) & 0x01ffffff); i++) {
		(void)G_U32(G_RX_FIFO);
	}
	G_W32(G_M_IRQ_CLEAR, 0xffffffffu);
}

static int
ft_geni_xfer(const uint8_t *tx, uint8_t *rx, unsigned len)
{
	unsigned sent = 0, received = 0;
	uint32_t status = 0;
	for (int i = 0; G_U32(G_STATUS) & 1u; i++) {
		if (i > 2000) {
			ft_geni_recover();
			break;
		}
		delay(1);
	}
	G_W32(G_M_IRQ_CLEAR, 0xffffffffu);
	G_W32(S_TX_LEN, len);
	G_W32(S_RX_LEN, len);
	G_W32(G_M_CMD0, G_SPI_FULL_DUPLEX);
	for (int spin = 0; spin < 400000; spin++) {
		while (sent < len && (G_U32(G_TX_FIFO_ST) & 0x0fffffffu) + 1 < ft_fifo_depth) {
			uint32_t word = 0;
			for (int j = 0; j < 4 && sent < len; j++, sent++) {
				word |= (uint32_t)(tx ? tx[sent] : 0) << (8 * j);
			}
			G_W32(G_TX_FIFO, word);
		}
		status = G_U32(G_M_IRQ_STATUS);
		ft_geni_drain(rx, len, &received);
		if (status & G_M_ERRORS) {
			break;
		}
		if (status & G_M_CMD_DONE) {
			ft_geni_drain(rx, len, &received);
			G_W32(G_M_IRQ_CLEAR, status);
			return received == len ? 0 : EIO;
		}
		delay(1);
	}
	ft_geni_recover();
	return EIO;
}

static int
alor_spi_duplex(const uint8_t *tx, unsigned len, uint8_t *rx)
{
	if (len == 0 || len > 0x300) {
		return EINVAL;
	}
	if (ft_on_geni) {
		return ft_geni_xfer(tx, rx, len);
	}
	return ft_bb_xfer(tx, rx, len);
}

static int
ft_geni_bringup(uint32_t divider)
{
	printf("ALOR TOUCH: qup +0 %#x +4 %#x +8 %#x +c %#x +10 %#x +14 %#x +18 %#x +1c %#x\n",
	    ft_rd(ft_qup, 0x00), ft_rd(ft_qup, 0x04), ft_rd(ft_qup, 0x08), ft_rd(ft_qup, 0x0c),
	    ft_rd(ft_qup, 0x10), ft_rd(ft_qup, 0x14), ft_rd(ft_qup, 0x18), ft_rd(ft_qup, 0x1c));
	if (G_U32(G_IF_DISABLE) & 1u) {
		/* The wrapper decides between FIFO and GSI; Android ran the bus in
		 * DMA mode, so the FIFO interface has to be handed back first. */
		static const uint32_t rst[4] = { 0x0c, 0x08, 0x10, 0x14 };
		for (unsigned i = 0; i < 4; i++) {
			ft_wr(ft_qup, rst[i], 1);
			for (int j = 0; j < 200 && (ft_rd(ft_qup, rst[i]) & 1u); j++) {
				delay(1);
			}
			printf("ALOR TOUCH: qup reset %#x -> %#x if %#x\n", rst[i],
			    ft_rd(ft_qup, rst[i]), G_U32(G_IF_DISABLE));
			if (!(G_U32(G_IF_DISABLE) & 1u)) {
				break;
			}
		}
	}
	if (G_U32(G_IF_DISABLE) & 1u) {
		return EBUSY;
	}
	int rc = ft_geni_init(divider);
	if (rc) {
		return rc;
	}
	ft_gpio(48, 1, 0, 4, -1);
	ft_gpio(49, 1, 0, 4, -1);
	ft_gpio(50, 1, 0, 4, -1);
	ft_gpio(51, 1, 0, 4, -1);
	ft_on_geni = true;
	return 0;
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
	ft_gpio(FT_IRQ, 0, 3, 8, -1);
	ft_bb_init();
	alor_blip(20);
	alor_blip(26);
	/* Voting these rails from the kernel kills the handset even with the
	 * right units, so the power stays whatever Android left behind. */
	alor_mark(25, true);
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
	alor_blip(21);
	ft_pin_dump("as found");
	printf("ALOR TOUCH: se4 fw %#x if %#x hw %#x status %#x\n", ft_rd(ft.se, 0x068),
	    ft_rd(ft.se, 0x064), ft_rd(ft.se, 0xe24), ft_rd(ft.se, 0x040));
	if (alor_ws <= 7) {
		alor_done = 1;
		printf("ALOR TOUCH: stage %u done after reset, holding\n", alor_ws);
		for (;;) {
			ft_sleep(5000);
		}
	}
	alor_stage = 10;
	ft_bb_init();
	ft_pin_dump("after bb init");
	/* Prove the pin window is live: drive MOSI and read the pad back. */
	ft_bb_level(FT_BB_MOSI, 1);
	delay(2);
	unsigned lb1 = ft_rd(ft.tlmm, FT_GPIO(FT_BB_MOSI) + 4) & 1u;
	ft_bb_level(FT_BB_MOSI, 0);
	delay(2);
	unsigned lb0 = ft_rd(ft.tlmm, FT_GPIO(FT_BB_MOSI) + 4) & 1u;
	printf("ALOR TOUCH: mosi loopback %u/%u\n", lb1, lb0);
	alor_mark(20, lb1 && !lb0);
	rc3 = alor_reg_read(0xa3, &id3, 1);
	printf("ALOR TOUCH: id probe rc %d id %02x\n", rc3, id3);
	/* v16 leaves the reset line alone: prove the read path first, so a
	 * mis-mapped pin cannot take the phone down. */
	bool id_ok = !rc3 && id3 != 0 && id3 != 0xff;
	alor_mark(22, id_ok);
	if (!id_ok) {
		/* The four pads are muxed by pinctrl and the data pair is the
		 * only line order that is not fixed by the DT. */
		static const unsigned char map[3][4] = {
			{ 49, 48, 50, 51 },   /* MISO/MOSI swapped */
			{ 48, 50, 51, 49 },   /* CS on 49, CLK on 51 */
			{ 50, 49, 51, 48 },   /* CS on 48, MISO on 50 */
		};
		for (unsigned m = 0; m < 3 && !id_ok; m++) {
			ft_bb_miso = map[m][0];
			ft_bb_mosi = map[m][1];
			ft_bb_clk = map[m][2];
			ft_bb_cs = map[m][3];
			ft_bb_init();
			rc3 = alor_reg_read(0xa3, &id3, 1);
			printf("ALOR TOUCH: id map %u rc %d id %02x\n", m, rc3, id3);
			id_ok = !rc3 && id3 != 0 && id3 != 0xff;
		}
	}
	alor_mark(22, id_ok);
	if (0 && !id_ok) {
		/* v16 disables the GENI/QUP path: the wrapper reset is the one
		 * piece that could disturb hardware we do not own yet. */
		int grc = ft_geni_bringup(8);
		printf("ALOR TOUCH: geni bringup %d if %#x\n", grc, G_U32(G_IF_DISABLE));
		if (!grc) {
			static const uint32_t dv[6] = { 31, 16, 8, 4, 2, 1 };
			for (unsigned d = 0; d < 6 && !id_ok; d++) {
				ft_geni_init(dv[d]);
				rc3 = alor_reg_read(0xa3, &id3, 1);
				printf("ALOR TOUCH: geni div %u rc %d id %02x\n", dv[d], rc3, id3);
				id_ok = !rc3 && id3 != 0 && id3 != 0xff;
			}
		}
		if (id_ok) {
			alor_mark(22, true);
		} else {
			ft_on_geni = false;
		}
	}
	/* Put the pads back on the pins the DT names before polling. */
	if (!ft_on_geni) {
		ft_bb_miso = 48;
		ft_bb_mosi = 49;
		ft_bb_clk = 50;
		ft_bb_cs = 51;
		ft_bb_init();
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
	alor_mark(23, !rc9e && mode == 0);
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
					if (!good) {
						alor_mark(24, true);
					}
					alor_frames++;
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
	printf("FUXI TOUCH: starting in %u s (poll %u us active, %u us idle)\n", delay_s,
	    poll_us, idle_us);
	alor_fb_init();
	alor_blip(0);
	ft_sleep(delay_s * 1000u);
	alor_blip(1);
	alor_stage = 1;
	ft.gcc = ml_io_map(FT_GCC_PA, 0x1f0000);
	alor_blip(2);
	ft.tlmm = ml_io_map(FT_TLMM_PA, 0x40000);
	ft.tlmm_hi = ml_io_map(FT_TLMM_PA + 0xa0000u, 0x8000);
	alor_blip(3);
	ft.se = ml_io_map(FT_SE_PA, 0x4000);
	ft_qup = ml_io_map(FT_QUP_PA, 0x1000);
	alor_blip(4);
	ft.gpi = ml_io_map(FT_GPI_PA, 0x4000);
	alor_blip(5);
	ft.smmu = ml_io_map(FT_SMMU_PA, 0x1000);
	alor_blip(6);
	if (!ft.gcc || !ft.tlmm || !ft.se || !ft.gpi || !ft.smmu || !ft_qup) {
		goto out;
	}
	alor_blip(7);
	/* No vm allocation and no second ml_io_map here: both take the kernel
	 * map lock this early and can wedge the whole system. A static page is
	 * enough for the GPI rings. */
	alor_blip(8);
	ft.dma = (vm_offset_t)ft_dma_page;
	uint64_t dpa = kvtophys(ft.dma);
	if (!dpa) {
		printf("FUXI TOUCH: static DMA page has no physical address\n");
		alor_mark(9, false);
		goto out;
	}
	ft.dma_pa = dpa;
	alor_stage = 2;
	int ierr;
	alor_blip(9);
	if (alor_ws >= 2 && (ierr = ft_clock())) { alor_stage = 3; printf("ALOR TOUCH: clock %d\n", ierr); goto out; }
	alor_mark(16, true);
	alor_stage = 4;
	alor_mark(17, true);   /* bit-bang: no serial engine needed */
	alor_stage = 5;
	alor_mark(18, true);   /* bit-bang: no SMMU */
	alor_stage = 6;
	alor_mark(19, true);   /* bit-bang: no GPI */
	printf("ALOR TOUCH: geni_if_disable -> no_pio=%d\n", ft_no_pio);
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
	(void)PE_parse_boot_argn("macan_touch_diag", &enabled, sizeof(enabled));
	if (!enabled) {
		printf("ALOR TOUCH: disabled\n");
		return;
	}
	if (kernel_thread_start_priority(ft_thread, NULL, FT_PRIORITY, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
	if (kernel_thread_start(alor_painter, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
	if (kernel_thread_start(alor_watchdog, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}
