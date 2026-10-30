import sys
import re

NL = chr(10)

# Installed into loader/loader.c by the CI: vote the two FocalTech supply rails
# while the firmware is still in charge. It runs after usb_start() so a fault
# here still leaves the loader's USB gadget alive and the log readable, and it
# refuses any command-table walk that could leave the 128 KiB region.
CODE = r'''
#define RH_RSC   0x18920000u
#define RH_CMDB  0x81c60000u
#define RH_DB_SIZE 0x20000u
#define RH_TCS   0xd00u
#define RH_STRIDE 0x2a0u
#define RH_WAIT  0x20u
#define RH_CTL   0x24u
#define RH_STS   0x28u
#define RH_EN    0x2cu
#define RH_MID   0x34u
#define RH_AD    0x38u
#define RH_DA    0x3cu
#define RH_CSTAT 0x40u
#define RH_CSTRIDE 24u
static unsigned
rh_rd(unsigned a)
{
 return *(volatile unsigned *)a;
}
static void
rh_wt(unsigned a, unsigned v)
{
 unsigned i;
 *(volatile unsigned *)a = v;
 for (i = 0; i < 40; i++) {
  if (rh_rd(a) == v) {
   return;
  }
  BS->stall(50);
 }
 print("rails: %#x stays %#x want %#x\n", (uint64_t)a, (uint64_t)rh_rd(a), (uint64_t)v);
}
static uint64_t
rh_db16(uint64_t off)
{
 if (off + 4 > RH_DB_SIZE) {
  return 0;
 }
 return (uint64_t)((rh_rd((unsigned)(RH_CMDB + (off & ~3u))) >> (8u * (off & 3u))) & 0xffffu);
}
static unsigned
rh_vrm(const char *res)
{
 uint64_t key[2] = {0, 0};
 uint64_t i, j, addr = 0, hits = 0;
 for (i = 0; i < 8 && res[i]; i++) {
  key[i / 4] |= (uint64_t)(uint8_t)res[i] << (8u * (i % 4));
 }
 if (rh_rd(RH_CMDB + 4) != 0x0c0330dbu) {
  print("rails: cmd-db magic %#x\n", (uint64_t)rh_rd(RH_CMDB + 4));
  return 0;
 }
 for (i = 0; i < 8; i++) {
  uint64_t h = 8u + 16u * i, slave = rh_db16(h), entries, count;
  if (!slave) {
   break;
  }
  entries = 144u + rh_db16(h + 2);
  count = rh_db16(h + 6);
  if (!count || entries + count * 24u > RH_DB_SIZE) {
   print("rails: slave %d table %d for %d entries exceeds the table\n",
       (uint64_t)i, (uint64_t)entries, (uint64_t)count);
   continue;
  }
  for (j = 0; j < count; j++) {
   uint64_t e = entries + 24u * j;
   if (rh_db16(e) == (key[0] & 0xffffu) && (uint64_t)rh_rd((unsigned)(RH_CMDB + e)) == key[0] &&
       (uint64_t)rh_rd((unsigned)(RH_CMDB + e + 4)) == key[1]) {
    uint64_t a = rh_rd((unsigned)(RH_CMDB + e + 16));
    if (slave == 4 && ((a >> 16) & 7u) == 4 && !(a & 15u)) {
     addr = a;
     hits++;
    }
   }
  }
 }
 print("rails: %s -> %#x (%d matches)\n", (uint64_t)res, addr, hits);
 return hits == 1 ? (unsigned)addr : 0;
}
static void
rh_vote(unsigned addr, unsigned mv)
{
 unsigned t, b = 0, bit = 0, irq, c, done = 0;
 for (t = 0; t < 3; t++) {
  unsigned x = RH_TCS + RH_STRIDE * t;
  unsigned st = rh_rd(RH_RSC + x + RH_STS);
  if (st != 0 && st != 0xffffffffu && rh_rd(RH_RSC + x + RH_CTL) == 0 &&
      rh_rd(RH_RSC + x + RH_EN) == 0 && rh_rd(RH_RSC + x + RH_WAIT) == 0 &&
      !(rh_rd(RH_RSC + RH_TCS + 4) & (1u << t))) {
   b = RH_RSC + x;
   bit = 1u << t;
   break;
  }
 }
 if (!bit) {
  print("rails: no free ACTIVE TCS\n");
  return;
 }
 irq = rh_rd(RH_RSC + RH_TCS + 0);
 rh_wt(RH_RSC + RH_TCS + 0, irq & ~bit);
 rh_wt(b + RH_MID, 0x10108u);
 rh_wt(b + RH_AD, addr);
 rh_wt(b + RH_DA, mv);
 rh_wt(b + RH_CSTRIDE + RH_MID, 0x10108u);
 rh_wt(b + RH_CSTRIDE + RH_AD, addr + 8);
 rh_wt(b + RH_CSTRIDE + RH_DA, 7);
 rh_wt(b + 2u * RH_CSTRIDE + RH_MID, 0x10108u);
 rh_wt(b + 2u * RH_CSTRIDE + RH_AD, addr + 4);
 rh_wt(b + 2u * RH_CSTRIDE + RH_DA, 1);
 rh_wt(b + RH_WAIT, 7);
 rh_wt(b + RH_EN, 7);
 rh_wt(b + RH_CTL, 0);
 rh_wt(b + RH_CTL, 1u << 16);
 *(volatile unsigned *)(b + RH_CTL) = (1u << 16) | (1u << 24);
 for (c = 0; c < 2000 && !done; c++) {
  unsigned k, ok = 1;
  for (k = 0; k < 3; k++) {
   unsigned s = rh_rd(b + RH_CSTAT + RH_CSTRIDE * k);
   ok = ok && s != 0xffffffffu && (s & 0x10100u) == 0x10100u;
  }
  done = ok && rh_rd(b + RH_STS) != 0;
  BS->stall(50);
 }
 print("rails: vote %#x %d mV slot %d done %d\n", (uint64_t)addr, (uint64_t)mv, (uint64_t)bit,
     (uint64_t)done);
 rh_wt(b + RH_CTL, 1u << 16);
 rh_wt(b + RH_CTL, 0);
 rh_wt(b + RH_EN, 0);
 rh_wt(b + RH_WAIT, 0);
 *(volatile unsigned *)(RH_RSC + RH_TCS + 8) = bit;
 rh_wt(RH_RSC + RH_TCS + 0, irq);
}
static void
rails_on(void)
{
 unsigned vddi, avdd;
 print("rails: rsc id %#x cfg %#x magic %#x\n", (uint64_t)rh_rd(RH_RSC),
     (uint64_t)rh_rd(RH_RSC + 0x0c), (uint64_t)rh_rd(RH_CMDB + 4));
 vddi = rh_vrm("L4B_E0");
 avdd = rh_vrm("L16B_E0");
 if (!vddi || !avdd) {
  return;
 }
 rh_vote(vddi, 1800);
 rh_vote(avdd, 3300);
 print("rails: finished\n");
}
'''

def main():
    p = sys.argv[1]
    s = open(p, encoding='utf-8').read()
    decl_anchor = 'static void' + NL + 'reboot(void)'
    if decl_anchor not in s:
        raise SystemExit('ANCHOR MISS: reboot declaration')
    m = re.search(r'( *)if \(usb_started\) \{', s)
    if not m:
        raise SystemExit('ANCHOR MISS: usb_started guard')
    if 'rails_on(void)' in s:
        print('loader rails patch already present')
        return
    s = s.replace(decl_anchor, CODE + NL + decl_anchor, 1)
    # Right before the loader hands USB over to the command loop: the gadget is
    # up by then, so a fault here still leaves the log reachable.
    s = s.replace(m.group(0), m.group(1) + 'rails_on();' + NL + m.group(0), 1)
    open(p, 'w', encoding='utf-8', newline=NL).write(s)
    print('loader rails patch v2 applied (call after usb_start, bounds checked)')

main()
