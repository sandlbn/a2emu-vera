/*
 * Reinette II plus, a french Apple II emulator, using SDL2
 * and powered by puce6502 - a MOS 6502 cpu emulator by the same author
 * Last modified 21st of June 2021
 * Copyright (c) 2020 Arthur Ferreira
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


/* a2core.c -- the Apple II+ machine (memory, soft switches, language card, Disk ][,
 * block device, paddles, VERA card wiring) without any frontend.  Split out of
 * reinette.c so that the SDL frontend (reinette.c) and the headless runner
 * (a2run.c) share it.  Frontends talk to it through a2core.h. */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>   /* getenv/atoi/atof -- WITHOUT this they are implicitly declared
                        as returning int, the 64-bit char* is truncated, and the very
                        first A2_CYAREG=1 run SEGFAULTS. */
#include <stdint.h>

#include "puce6502.h"
#include "veracore.h"
#include "a2core.h"
#include "ym2151.h"

/* VERA (a2_vera card) mapped in a peripheral slot's I/O window.
   The misterblack1/a2_vera card lives at $Cn00-$Cn1F (slot n); we place it in
   slot 4 = $C400 (the demos are slot-agnostic and auto-detect it there). */
#ifndef VERA_SLOT_BASE               /* -DVERA_SLOT_BASE=0xC500 to test another slot */
#define VERA_SLOT_BASE 0xC400
#endif
/* Runtime VERA base (E4): A2_VERA_SLOT=<1-7> or a frontend sets it before running. */
uint16_t g_vera_base = VERA_SLOT_BASE;
vc_t g_vera;
int  g_vera_enabled = 0;
/* VERA IRQ -> 6502 /IRQ. The A2VERA wires VERA's IRQ to slot pin 30 through diode D1
   (always connected, per the card's author), so this is ON by default.
   A2_VERA_IRQ=0 models a board built without D1 (the polled-fallback test). */
int  g_vera_irq = 1;

/* VERA is clocked in 16 MHz "steps" (veracore's CPU_MHZ); the Apple II CPU runs at
   14.31818 MHz / 14 * 65/64 ~ 1,020,484 Hz on average.  Converting the cycles the CPU
   actually executed (with the remainder carried) puts a VERA frame at
   16e6 / 268,800 -> 59.52 Hz = ~17,145 CPU cycles, as on the real card.
   (Before E2 this was a fixed 504 steps per 32 cycles = 59.79 Hz.) */
#define A2_CPU_HZ       1020484u
#define VC_STEPS_PER_S 16000000u
static unsigned int     cvs_last  = 0;
static unsigned long long cvs_carry = 0;
static int              cvs_init  = 0;

/* Run ~32 CPU cycles, then advance VERA by the matching amount and present its IRQ
   line to the CPU. The line is level-triggered: while ISR & IEN is nonzero and the
   I flag is clear, the CPU keeps taking the IRQ (puce6502IRQ checks P.I). */
static int cvs_step1 = -1;               /* A2_STEP1=1: one instruction per step (tracing) */
void cpu_vera_step(void) {
	if (!cvs_init) { cvs_last = ticks; cvs_init = 1; }
	if (cvs_step1 < 0) cvs_step1 = getenv("A2_STEP1") ? 1 : 0;
	puce6502Exec(cvs_step1 ? 1 : 32);
	unsigned int d = ticks - cvs_last; cvs_last = ticks;      /* wrap-safe */
	unsigned long long num = (unsigned long long)d * VC_STEPS_PER_S + cvs_carry;
	int steps = (int)(num / A2_CPU_HZ); cvs_carry = num % A2_CPU_HZ;
	if (vc_tick(&g_vera, steps) && g_vera_irq) puce6502IRQ();
}

/* IIGS speed model . BOTH DEFAULT-OFF. g_cyareg_model = 0 -> $C036 is not modelled at all and... */
int     g_cyareg_model    = 0;
/*  verification models, BOTH DEFAULT-OFF (see softSwitches for the contracts):
   A2_SNESMAX=<slot 1-7> + A2_SNESMAX_BITS/_P2=<hex16> models the SNES MAX card;
   A2_SPEEDLOG=<path> traces $C036 writes and the CYAREG in force at each VERA access. */
int      g_snesmax_slot = 0;
uint16_t g_snesmax_bits = 0xFFFF, g_snesmax_p2 = 0xFFFF;
unsigned long g_snesmax_latches = 0, g_snesmax_clocks = 0, g_snesmax_reads = 0;
FILE    *g_speedlog       = NULL;
void speedlog_note(uint8_t kind, uint8_t val, unsigned int tick);
uint8_t g_cyareg          = 0;
int     g_gs_fast_permille = 2800;
extern int g_a2_speed_permille;

void vera_bus_init(void)
{
    { const char *e = getenv("A2_CYAREG");
      g_cyareg_model = e ? atoi(e) : 0;
      { const char *sm = getenv("A2_SNESMAX");
        g_snesmax_slot = sm ? atoi(sm) : 0;
        const char *sb = getenv("A2_SNESMAX_BITS");
        g_snesmax_bits = sb ? (uint16_t)strtoul(sb, NULL, 16) : 0xFFFF;
        const char *s2 = getenv("A2_SNESMAX_P2");
        g_snesmax_p2 = s2 ? (uint16_t)strtoul(s2, NULL, 16) : 0xFFFF;
        const char *sl = getenv("A2_SPEEDLOG");
        if (sl) g_speedlog = fopen(sl, "wb"); }
      e = getenv("A2_GS_MHZ");
      if (e) { double mhz = atof(e); if (mhz > 0.05) g_gs_fast_permille = (int)(mhz * 1000.0 + 0.5); }
      /* A2_GS_BOOTFAST=1 powers the machine on in FAST mode, which is what a real IIGS does
         (Control Panel "System Speed: Fast" is the factory default). This is the ONLY way to
         exercise / in an emulator: without it the boot runs at 1 MHz regardless and the
         1 MHz-forcing those sections added is untestable. Requires A2_CYAREG=1. */
      g_cyareg = 0; g_a2_speed_permille = 1000;
      if (g_cyareg_model && getenv("A2_GS_BOOTFAST") && atoi(getenv("A2_GS_BOOTFAST"))) {
          g_cyareg = 0x80; g_a2_speed_permille = g_gs_fast_permille;
      } }
    { const char *e = getenv("A2_VERA_IRQ"); g_vera_irq = e ? atoi(e) : 1;
      e = getenv("A2_VERA_SLOT"); if (e && atoi(e) >= 1 && atoi(e) <= 7) g_vera_base = 0xC000 + atoi(e) * 0x100; }
    vc_init(&g_vera);
    { const char *e = getenv("A2_YM"); ym_init(e ? atoi(e) : 1); }               // E14
    if (g_vera.framebuffer == NULL) {
        g_vera.framebuffer = calloc((size_t)VC_SCREEN_W * VC_SCREEN_H, sizeof(uint32_t));
    }
    g_vera_enabled = 1;
}

// memory layout
#define RAMSIZE  0xC000
#define ROMSTART 0xD000
#define ROMSIZE  0x3000
uint8_t ram[RAMSIZE];   // 48K of ram in $000-$BFFF
uint8_t rom[ROMSIZE];   // 12K of rom in $D000-$FFFF

// language card
#define LGCSTART 0xD000
#define LGCSIZE  0x3000
#define BK2START 0xD000
#define BK2SIZE  0x1000
uint8_t lgc[LGCSIZE];   // Language Card 12K in $D000-$FFFF
uint8_t bk2[BK2SIZE];   // bank 2 of Language Card 4K in $D000-$DFFF

// disk ][ prom
#define SL6START 0xC600
#define SL6SIZE  0x0100
uint8_t sl6[SL6SIZE];   // P5A disk ][ prom in slot 6


//================================================================ SOFT SWITCHES

uint8_t KBD   = 0;                                                              // $C000, $C010 ascii value of keyboard input
/* Any-Key-Down (AKD) emulation: on the IIe/IIc/IIGS, reading $C010 returns bit7=1
 * while a key is physically held (a LEVEL, independent of the auto-repeat strobe).
 * The II/II+ has no AKD ($C010 just clears the strobe). Off by default; a front-end
 * enables g_akd (e.g. bootwin --akd) and drives g_kbdDown from the held-key state. */
int     g_akd     = 0;                                                          // AKD emulation enabled
uint8_t g_kbdDown = 0;                                                          // bit7 = a key is currently held
bool TEXT  = true;                                                              // $C050 CLRTEXT  / $C051 SETTEXT
bool MIXED = false;                                                             // $C052 CLRMIXED / $C053 SETMIXED
bool PAGE2 = false;                                                             // $C054 PAGE2 off / $C055 PAGE2 on
bool HIRES = false;                                                             // $C056 GR       / $C057 HGR
bool LCWR  = true;                                                              // Language Card writable
bool LCRD  = false;   /* exposed for intercept feasibility check */                                                             // Language Card readable
bool LCBK2 = true;                                                              // Language Card bank 2 enabled
bool LCWFF = false;                                                             // Language Card pre-write flip flop


//============================================================ APPLE IIe (E13)
/* g_iie: 0 = Apple ][+ (default), 1 = IIe (unenhanced ROM, 6502), 2 = enhanced IIe
 * (65C02).  Model of the IIe MMU/IOU that software sees:
 *   $C000-$C00F writes: 80STORE, RAMRD, RAMWRT, INTCXROM, ALTZP, SLOTC3ROM, 80COL,
 *   ALTCHAR (off/on pairs); $C011-$C01F reads: their state in bit 7 (+ KBD bits 0-6),
 *   $C019 = VBL (bit 7 = 0 during vertical blank); $C000-$C00F reads = keyboard.
 *   Auxiliary 64 KB.  ALTZP (zero page, stack and language card from aux) is done by
 *   SWAPPING those 16.5 KB, so the CPU's RAM fast paths stay valid; RAMRD/RAMWRT/
 *   80STORE send RAM accesses through the slow path (g_memslow) and rebuild the
 *   fetch mirror.  Internal ROM $C100-$CFFF: all of it with INTCXROM, $C3xx unless
 *   SLOTC3ROM, and $C800-$CFFF always (no emulated card has a $C800 ROM).
 *   Reset (a2_reset) clears the switches and maps the LC to ROM, as the IIe does.
 * Without the extended 80-column card (g_iie_aux = 0, the default: plan A-D2 baseline)
 * there is no aux RAM: aux reads return $FF, aux writes are lost, ALTZP does nothing
 * (on a real machine it would map in floating bus) -- so aux-detection code (ProDOS's
 * /RAM) correctly finds none.
 * Not modelled: 80-column / double-res VIDEO (the frontend draws 40 columns),
 * snapshots of the IIe state, the IIe keyboard's auto-repeat. */
int      g_iie = 0;
int      g_iie_aux = 0;                /* 1 = extended 80-column card (aux 64 KB) fitted */
int      g_memslow = 0;                /* puce6502.c: RAM accesses take the slow path */
uint8_t  iiecx[0x1000];                /* internal ROM image for $C000-$CFFF */
static uint8_t aux[RAMSIZE], auxlgc[LGCSIZE], auxbk2[BK2SIZE];
bool S80STORE = false, RAMRD = false, RAMWRT = false, ALTZP = false;
bool INTCXROM = false, SLOTC3ROM = false, S80COL = false, ALTCHAR = false;

/* the bank a RAM access to `a` (< $C000) uses; ZP/stack come via the ALTZP swap */
static uint8_t noaux_rd[256], noaux_wr[256];   /* no aux card: reads $FF, writes lost */
static inline uint8_t *aux_at(uint16_t a, int write) {
	if (g_iie_aux) return &aux[a];
	return write ? &noaux_wr[a & 0xFF] : &noaux_rd[a & 0xFF];
}
static inline uint8_t *iie_bank(uint16_t a, int write) {
	if (S80STORE && ((a >= 0x0400 && a < 0x0800) || (HIRES && a >= 0x2000 && a < 0x4000)))
		return PAGE2 ? aux_at(a, write) : &ram[a];
	if (a < 0x0200) return &ram[a];
	return (write ? RAMWRT : RAMRD) ? aux_at(a, write) : &ram[a];
}
static void iie_update_memslow(void) { g_memslow = g_iie && (RAMRD || RAMWRT || S80STORE); }

static void swapbuf(uint8_t *a, uint8_t *b, size_t n) {
	while (n--) { uint8_t t = *a; *a++ = *b; *b++ = t; }
}
static void iie_set_altzp(bool on) {            /* swap ZP+stack and the LC with aux */
	if (!g_iie_aux) { ALTZP = false; return; }   /* no card: nothing to map in */
	if (on == ALTZP) return;
	swapbuf(ram, aux, 0x200);
	swapbuf(lgc, auxlgc, LGCSIZE);
	swapbuf(bk2, auxbk2, BK2SIZE);
	ALTZP = on;
}

//============================================================ FETCH-VIEW MIRROR
/* 64 KB execute-view of the address space for the 6502 core's instruction fetches:... */
uint8_t fetchMem[0x10000];

#ifdef __mc68000__
extern void jit_invalidate_all(void);            /* puce6502_jit.inc */
extern uint8_t jit_pgcode_g[256];
extern void jit_page_smc_g(unsigned page);
#define JIT_INVALIDATE() jit_invalidate_all()
#define JIT_PAGE_CHECK(ad) do { \
	if (jit_pgcode_g[(uint16_t)(ad) >> 8]) jit_page_smc_g((uint16_t)(ad) >> 8); \
} while (0)
#else
#define JIT_INVALIDATE() ((void)0)
#define JIT_PAGE_CHECK(ad) ((void)0)
#endif

void fetchSyncHigh(void) {                       /* remap $D000-$FFFF */
	JIT_INVALIDATE();                            /* translations read fetchMem */
	if (!LCRD) {
		memcpy(fetchMem + ROMSTART, rom, ROMSIZE);
	} else {
		memcpy(fetchMem + 0xD000, LCBK2 ? bk2 : lgc, 0x1000);
		memcpy(fetchMem + 0xE000, lgc + 0x1000, 0x2000);
	}
}

void fetchSyncRam(void) {                       /* $0000-$BFFF as reads see it */
	JIT_INVALIDATE();
	if (!g_iie) { memcpy(fetchMem, ram, RAMSIZE); return; }
	memset(noaux_rd, 0xFF, sizeof noaux_rd);
	for (unsigned p = 0; p < 0xC0; p++) memcpy(fetchMem + (p << 8), iie_bank((uint16_t)(p << 8), 0), 256);
}
void fetchSyncCx(void) {                        /* $C000-$CFFF: slot / internal ROMs */
	JIT_INVALIDATE();
	memset(fetchMem + 0xC000, 0xFF, 0x1000);
	memcpy(fetchMem + SL6START, sl6, SL6SIZE);
	if (g_hd_present) memcpy(fetchMem + SL7START, sl7, sizeof sl7);
	if (g_iie) {
		if (INTCXROM) memcpy(fetchMem + 0xC100, iiecx + 0x100, 0x700);
		else if (!SLOTC3ROM) memcpy(fetchMem + 0xC300, iiecx + 0x300, 0x100);
		memcpy(fetchMem + 0xC800, iiecx + 0x800, 0x800);
	}
}
void fetchSyncAll(void) {
	JIT_INVALIDATE();                            /* whole guest map replaced */
	fetchSyncRam();
	fetchSyncCx();
	fetchSyncHigh();
}


//====================================================================== PADDLES

uint8_t PB0 = 0;                                                                // $C061 Push Button 0 (bit 7) / Open Apple
uint8_t PB1 = 0;                                                                // $C062 Push Button 1 (bit 7) / Solid Apple
uint8_t PB2 = 0;                                                                // $C063 Push Button 2 (bit 7) / shift mod !!!
// --- game-port NES 4021 controller (default off; distinct from the $4016 NES-side pad) ---
// g_gpad bit7=A bit6=B bit5=Select bit4=Start bit3=Up bit2=Down bit1=Left bit0=Right (1=pressed)
// Wiring (matches a2vera/INPUT.md): LATCH=AN0 ($C058/9), CLOCK=AN1 ($C05A/B), DATA=PB2 ($C063, active low).
uint8_t g_gpad = 0;
int     g_gpad_en = 0;
static int g_gpad_idx = 0;                                                      // 4021 output bit index (0=A .. 7=Right)
// EDGE-accurate CD4021 model (2026): the register only latches on an AN0 high->low
// pulse and only shifts on an AN1 low->high pulse -- exactly what the hardware needs.
// Touching one side of a pair (as a broken reader does) produces no edge and no data,
// so this can no longer give a false pass to a shim that skips the second half.
// LIMITATION: this model shifts on the AN1 rising edge regardless of the CLOCK's IDLE
// LEVEL, so it CANNOT tell idle-low ($C05B then $C05A) from idle-high ($C05A then $C05B).
// The real adapter needs the clock (and latch) to idle HIGH -- the driver's default is
// cfg_pclk=1. A Total Replay shim MUST
// use the idle-HIGH order or it reads dead on metal (both games, hardware round 3).
int     g_gp_an0 = 0, g_gp_an1 = 0;                                       // current AN0/AN1 line levels
static uint8_t g_gpad_latch = 0;                                                 // buttons sampled at the latch pulse
float GCP[4] = { 127.0f, 127.0f, -1.0f, -1.0f };                                // GC Position 0 (left/up) .. 255 (right/down);
                                                                                // < 0 = nothing connected (the timer never runs out)
float GCC[4] = { 0.0f };                                                        // (unused since WP1.4; kept for the frontends' ABI)
int GCD[2] = { 0 };                                                             // GC0 and GC1 Directions (left/down or right/up)
int GCA[2] = { 0 };                                                             // GC0 and GC1 Action (push or release)
uint8_t GCActionSpeed = 8;                                                      // Game Controller speed at which it goes to the edges
uint8_t GCReleaseSpeed = 8;                                                     // Game Controller speed at which it returns to center
unsigned int GCCrigger;                                                        // $C070 the tick at which the GCs were reseted

/* Paddle timers (WP1.4): $C070 starts all four 558 timers; $C064+n reads bit 7 = 1 until
 * GCP[n] * 11 cycles have passed (the monitor's PREAD loop is 11 cycles per count, so
 * PREAD returns ~GCP[n], as on the real machine with a 150k pot).  The old model
 * (countdown GCP^2, minus the TOTAL elapsed time at every read) only gave PREAD-like
 * values for a reader that polls every 11 cycles. */
inline static void resetPaddles() {
	GCCrigger = ticks;                                                            // records the time this was done
}

inline static uint8_t readPaddle(int pdl) {
	if (GCP[pdl] < 0) return 0x80;                                                // no paddle: never times out
	return (unsigned int)(ticks - GCCrigger) < (unsigned int)(GCP[pdl] * 11.0f) ? 0x80 : 0;
}


//====================================================================== SPEAKER

/* $C030 toggles call the frontend's speaker hook (the audio device lives in the
   frontend; the headless runner leaves it NULL). */
void (*a2_speaker_hook)(void) = NULL;
bool muted = false;                                                             // mute/unmute switch (saved in snapshots)
static void playSound() { if (a2_speaker_hook) a2_speaker_hook(); }


//====================================================================== DISK ][

int curDrv = 0;
int g_disk_dbg = 0;                                                                 // Current Drive - only one can be enabled at a time

struct drive disk[2] = { 0 };                                                  // two disk ][ drive units (struct in a2core.h)

/* True while either Disk ][ motor is spinning (i.e. RWTS is loading) -- the
 * Mac frontend uses this to run the 6502 flat-out (skip the 60 Hz pace +
 * per-frame render) during disk loads, then drop back to 1x for play. */
int disk_motor_on(void) { return disk[0].motorOn || disk[1].motorOn; }


//================================================== ProDOS HARD DISK (slot 7)
/* A ProDOS block device in slot 7 (E6): firmware at $C700, I/O at $C0F0-$C0F9.
 * The ][+ Autostart ROM scans slots 7..1 for $Cn01=$20 $Cn03=$00 $Cn05=$03 and boots
 * the first hit -- it also requires $Cn07 = $3C -- so with an image inserted the machine
 * boots from slot 7 (the Disk ][ in slot 6 stays usable).  ProDOS tells a Disk ][ by
 * $CnFF = $00; ours is $2A and $Cn07 != $00 (not SmartPort), so ProDOS treats it as a
 * generic block device: status byte $CnFE, driver at $Cn00+$CnFF.
 * Boot: block 0 -> $0800, X = $70 (slot 7 * 16), JMP $0801 -- the ProDOS boot block
 * contract.  The driver takes the ProDOS ZP parameters ($42 cmd, $43 unit, $44/45
 * buffer, $46/47 block). */
static const uint8_t hd_fw[] = {
  /*$00*/ 0xA2,0x20, 0xA0,0x00, 0xA2,0x03, 0xC9,0x3C,        // LDX#$20;LDY#$00;LDX#$03;CMP#$3C
                                                            //   -> sig $20,$00,$03,$3C (Autostart)
  /*boot $08*/ 0xA9,0x01, 0x8D,0xF0,0xC0,                    // LDA #1(read); STA $C0F0
               0xA9,0x70, 0x8D,0xF1,0xC0,                    // LDA #$70(unit slot7); STA $C0F1
               0xA9,0x00, 0x8D,0xF2,0xC0, 0x8D,0xF3,0xC0,    // block 0
               0x8D,0xF4,0xC0, 0xA9,0x08, 0x8D,0xF5,0xC0,    // buffer $0800
               0x8D,0xF6,0xC0,                               // STA $C0F6 (execute)
               0xA2,0x70, 0x4C,0x01,0x08,                    // LDX #$70(slot7<<4); JMP $0801
  /*driver $2A*/ 0xA5,0x42, 0x8D,0xF0,0xC0,                  // cmd
                 0xA5,0x43, 0x8D,0xF1,0xC0,                  // unit
                 0xA5,0x46, 0x8D,0xF2,0xC0,                  // block lo
                 0xA5,0x47, 0x8D,0xF3,0xC0,                  // block hi
                 0xA5,0x44, 0x8D,0xF4,0xC0,                  // buf lo
                 0xA5,0x45, 0x8D,0xF5,0xC0,                  // buf hi
                 0x8D,0xF6,0xC0,                             // STA $C0F6 (execute)
                 0xAE,0xF8,0xC0, 0xAC,0xF9,0xC0,             // LDX blocks-lo; LDY blocks-hi (STATUS)
                 0xAD,0xF7,0xC0, 0x18, 0xF0,0x01, 0x38, 0x60 // LDA err; CLC; BEQ +1; SEC; RTS
};
uint8_t sl7[256];                       /* slot-7 firmware, mapped while an image is in */
int     g_hd_present = 0;
static long hd_offset = 0;              /* .2mg: data offset from its header */
static int  hd_ro = 0;                  /* image opened read-only: WRITE -> $2B (write protected) */
static FILE *hd_file = NULL;
static unsigned hd_blocks = 0;
static uint8_t  hd_cmd = 0, hd_unit = 0, hd_status = 0;
static uint16_t hd_block = 0, hd_buf = 0;

int insertHardDisk(char *filename) {
	hd_ro = 0;
	hd_file = fopen(filename, "r+b");                                             // try read/write
	if (!hd_file) { hd_file = fopen(filename, "rb"); hd_ro = 1; }                 // else read-only:
	                                                                              //  writes -> $2B
	if (!hd_file) return 0;
	fseek(hd_file, 0, SEEK_END);
	long sz = ftell(hd_file);
	hd_offset = 0;
	{ uint8_t h[64];                                                              // .2mg: skip its header
	  fseek(hd_file, 0, SEEK_SET);
	  if (sz > 64 && fread(h, 1, 64, hd_file) == 64 && !memcmp(h, "2IMG", 4)) {
		hd_offset = h[0x18] | (h[0x19] << 8) | ((long)h[0x1A] << 16) | ((long)h[0x1B] << 24);
		sz = h[0x1C] | (h[0x1D] << 8) | ((long)h[0x1E] << 16) | ((long)h[0x1F] << 24);
	  } }
	hd_blocks = (unsigned)(sz / 512);
	if (hd_blocks > 65535) hd_blocks = 65535;
	memset(sl7, 0, sizeof sl7);                                                  // install firmware in slot 7
	memcpy(sl7, hd_fw, sizeof hd_fw);
	sl7[0xFE] = 0x07;   // status byte: status+read+write, 1 volume, not removable
	sl7[0xFF] = 0x2A;   // driver entry offset within $C700 ($C72A)
	sl7[0xFC] = hd_blocks & 0xFF; sl7[0xFD] = hd_blocks >> 8;                    // block count
	g_hd_present = 1;
	memcpy(fetchMem + SL7START, sl7, sizeof sl7);
	return 1;
}

/* poke/peek that respect the main-RAM / language-card map (ProDOS may read a
 * block straight into the LC); $C0xx I/O is skipped. */
static void hd_poke(uint16_t a, uint8_t v) {
	if (a < RAMSIZE) { ram[a] = v; fetchMem[a] = v; JIT_PAGE_CHECK(a); }
	else if (a >= ROMSTART) {
		if (LCBK2 && a < 0xE000) bk2[a - BK2START] = v; else lgc[a - LGCSTART] = v;
		if (LCRD) fetchMem[a] = v;                     /* hd_poke writes the LCRD-visible bank */
	}
}
static uint8_t hd_peek(uint16_t a) {
	if (a < RAMSIZE) return ram[a];
	if (a >= ROMSTART) { if (LCBK2 && a < 0xE000) return bk2[a - BK2START]; return lgc[a - LGCSTART]; }
	return 0;
}
int g_hd_log = 0;                                                                // instrumentation: log block reads -> dest addr
__attribute__((constructor)) static void _hdlog_env(void) { if (getenv("A2_HDLOG")) g_hd_log = 1; }   /* A2_HDLOG=1 */
int g_kbd_log = 0;
__attribute__((constructor)) static void _kbdlog_env(void){ if(getenv("KBDLOG")) g_kbd_log = 1; }
int g_akd_reads = 0;                                                             // count of AKD reads that saw a key (for single-keystroke injection)
static void hd_exec(void) {                                                      // $C0F6 write -> do the op
	unsigned char blk[512]; int i;
	if (!hd_file) { hd_status = 0x28; return; }                                  // no device connected
	if (hd_cmd == 0) { hd_status = 0; return; }                                  // STATUS: ok (count via $C0F8/9)
	if (hd_block >= hd_blocks) { hd_status = 0x27; return; }                     // I/O error: block out of range
	fseek(hd_file, hd_offset + (long)hd_block * 512, SEEK_SET);
	if (hd_cmd == 1) {                                                           // READ
		if (fread(blk, 1, 512, hd_file) != 512) { hd_status = 0x27; return; }
		for (i = 0; i < 512; i++) hd_poke(hd_buf + i, blk[i]);
		if (g_hd_log) fprintf(stderr, "[hd] read blk %5u -> $%04X\n", hd_block, hd_buf);
		hd_status = 0;
	} else if (hd_cmd == 2 && hd_ro) {                                           // WRITE, protected
		hd_status = 0x2B;
	} else if (hd_cmd == 2) {                                                    // WRITE
		for (i = 0; i < 512; i++) blk[i] = hd_peek(hd_buf + i);
		if (g_hd_log) fprintf(stderr, "[hd] write blk %5u <- $%04X\n", hd_block, hd_buf);
		fwrite(blk, 1, 512, hd_file); fflush(hd_file);
		hd_status = 0;
	} else hd_status = 0;                                                        // FORMAT etc: pretend ok
}


/* DOS 3.3 6-and-2 write table + a .dsk (143360 B, DOS sector order) -> .nib
 * (35*6656 B nibble stream) nibblizer, so reinette can load raw sector images
 * directly (most Apple II archive disks are .dsk, not .nib). */
static const uint8_t gcr6[64] = {
	0x96,0x97,0x9A,0x9B,0x9D,0x9E,0x9F,0xA6,0xA7,0xAB,0xAC,0xAD,0xAE,0xAF,0xB2,0xB3,
	0xB4,0xB5,0xB6,0xB7,0xB9,0xBA,0xBB,0xBC,0xBD,0xBE,0xBF,0xCB,0xCD,0xCE,0xCF,0xD3,
	0xD6,0xD7,0xD9,0xDA,0xDB,0xDC,0xDD,0xDE,0xDF,0xE5,0xE6,0xE7,0xE9,0xEA,0xEB,0xEC,
	0xED,0xEE,0xEF,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,0xF9,0xFA,0xFB,0xFC,0xFD,0xFE,0xFF
};
static void nibblize_dsk(const uint8_t *dsk, uint8_t *nib) {
	static const uint8_t p2l[16] = {0,7,0xE,6,0xD,5,0xC,4,0xB,3,0xA,2,9,1,8,0xF};
	int track, ps, i;
	memset(nib, 0xFF, 35 * 0x1A00);                                            // sync-fill
	for (track = 0; track < 35; track++) {
		uint8_t *out = nib + track * 0x1A00;
		for (i = 0; i < 48; i++) *out++ = 0xFF;                                // track gap
		for (ps = 0; ps < 16; ps++) {
			const uint8_t *sd = dsk + track * 16 * 256 + p2l[ps] * 256;        // DOS skew
			uint8_t chk = 254 ^ track ^ ps, nb[342], last;
			*out++=0xD5; *out++=0xAA; *out++=0x96;                             // address prologue
			*out++=(254>>1)|0xAA; *out++=254|0xAA;                             // volume 254 (4-and-4)
			*out++=(track>>1)|0xAA; *out++=track|0xAA;
			*out++=(ps>>1)|0xAA; *out++=ps|0xAA;
			*out++=(chk>>1)|0xAA; *out++=chk|0xAA;
			*out++=0xDE; *out++=0xAA; *out++=0xEB;                             // address epilogue
			for (i = 0; i < 6; i++) *out++ = 0xFF;
			*out++=0xD5; *out++=0xAA; *out++=0xAD;                             // data prologue
			for (i = 0; i < 256; i++) nb[86 + i] = sd[i] >> 2;                 // 6-and-2 encode
			for (i = 0; i < 86; i++) nb[i] = 0;
			for (i = 0; i < 256; i++)
				nb[i % 86] |= (((sd[i] & 1) << 1) | ((sd[i] & 2) >> 1)) << (2 * (i / 86));
			last = 0;
			for (i = 0; i < 342; i++) { *out++ = gcr6[(nb[i] ^ last) & 0x3F]; last = nb[i]; }
			*out++ = gcr6[last & 0x3F];                                        // data checksum
			*out++=0xDE; *out++=0xAA; *out++=0xEB;                             // data epilogue
			for (i = 0; i < 27; i++) *out++ = 0xFF;
		}
	}
}

/* ProDOS-order (.po) <-> DOS-order (.dsk/.do) 140 KB images: ProDOS sector p of a
 * track is DOS logical sector po2dos[p] (the table tools/mk_prodos.py uses). */
static const uint8_t po2dos[16] = {0,14,13,12,11,10,9,8,7,6,5,4,3,2,1,15};
static void po_to_dos(const uint8_t *po, uint8_t *dos) {
	for (int t = 0; t < 35; t++)
		for (int p = 0; p < 16; p++) memcpy(dos + t * 4096 + po2dos[p] * 256, po + t * 4096 + p * 256, 256);
}
static void dos_to_po(const uint8_t *dos, uint8_t *po) {
	for (int t = 0; t < 35; t++)
		for (int p = 0; p < 16; p++) memcpy(po + t * 4096 + p * 256, dos + t * 4096 + po2dos[p] * 256, 256);
}

/* The inverse of nibblize_dsk: scan each track's nibble stream for address fields
 * (D5 AA 96, 4-and-4) followed by a data field (D5 AA AD, 6-and-2 + checksum) and
 * decode it into the DOS-order image.  Sectors that do not decode are left as they
 * are in `dsk`.  Returns the number of distinct sectors decoded (560 = all). */
int denibblize(const uint8_t *nib, uint8_t *dsk) {
	static const uint8_t p2l[16] = {0,7,0xE,6,0xD,5,0xC,4,0xB,3,0xA,2,9,1,8,0xF};
	uint8_t inv[256];
	int found = 0;
	memset(inv, 0xFF, sizeof inv);
	for (int i = 0; i < 64; i++) inv[gcr6[i]] = (uint8_t)i;
	for (int track = 0; track < 35; track++) {
		const uint8_t *tr = nib + track * 0x1A00;
		const int n = 0x1A00;
		unsigned got = 0;
		#define NB(k) tr[(i + (k)) % n]
		for (int i = 0; i < n; i++) {
			if (NB(0) != 0xD5 || NB(1) != 0xAA || NB(2) != 0x96) continue;
			int trk = ((NB(5) << 1) | 1) & NB(6), sec = ((NB(7) << 1) | 1) & NB(8);
			if (trk != track || sec > 15) continue;
			for (int j = 9; j < 80; j++) {
				if (NB(j) != 0xD5 || NB(j + 1) != 0xAA || NB(j + 2) != 0xAD) continue;
				uint8_t nb[342], last = 0; int ok = 1;
				for (int k = 0; k < 342; k++) {
					uint8_t v = inv[NB(j + 3 + k)];
					if (v == 0xFF) { ok = 0; break; }
					last ^= v; nb[k] = last;
				}
				if (ok && inv[NB(j + 3 + 342)] != last) ok = 0;          /* checksum */
				if (ok) {
					uint8_t *sd = dsk + track * 4096 + p2l[sec] * 256;
					for (int k = 0; k < 256; k++) {
						uint8_t a = nb[k % 86] >> (2 * (k / 86));
						sd[k] = (uint8_t)((nb[86 + k] << 2) | ((a & 1) << 1) | ((a & 2) >> 1));
					}
					if (!(got & (1u << sec))) { got |= 1u << sec; found++; }
				}
				break;
			}
		}
		#undef NB
	}
	return found;
}

/* ROTATIONAL DISK ][ TIMING MODEL (DEFAULT OFF) . */
int g_disktime = 0;                      /* A2_DISKTIME=1 */
static int      dt_init_done = 0;
static unsigned dt_ntrk   = 0;           /* nibbles per track in the timed layout   */
static unsigned dt_trev   = 0;           /* cycles per revolution                   */
static unsigned dt_pfx[6660];            /* cycle offset of nibble i within a rev   */
static unsigned dt_secpos[16];           /* nibble index of each sector's addr D5    */
static unsigned dt_t0     = 0;           /* tick at rotational origin               */
static unsigned dt_pos    = 0;           /* cached head position (nibble index)     */
static unsigned dt_delivered = 0xFFFFFFFFu; /* last position handed to the CPU      */
static unsigned dt_spinup = 0, dt_settle = 0, dt_motor_t = 0, dt_settle_t = 0;
static int      dt_slip   = 0, dt_slipn = 0;   /* A2_DISK_SLIP=n: drop every n-th step */
/* A2_DISK_NIBCYC: cycles per data nibble.  32 = the real 1 MHz Disk ][ (4 us/bit x 8).
 * MARGIN PROBE: lower it and the streaming loops have less time per nibble, so sweeping
 * it finds the exact cycle budget the RWTS survives -- and proves the model can FAIL. */
static unsigned dt_nibcyc = 32;
static FILE    *dt_log    = NULL;
static unsigned dt_frozen = 0;           /* rotational time while the motor is off  */
static int      dt_spinning = 0;
unsigned long   g_dt_missed = 0;         /* nibbles that passed while nobody looked */

static void dt_note(uint8_t kind, uint8_t a, uint8_t b, unsigned tick) {
	if (!dt_log) return;
	fputc(kind, dt_log); fputc(a, dt_log); fputc(b, dt_log); fputc(0, dt_log);
	fputc(tick & 0xFF, dt_log);         fputc((tick >> 8) & 0xFF, dt_log);
	fputc((tick >> 16) & 0xFF, dt_log); fputc((tick >> 24) & 0xFF, dt_log);
}

/* Re-nibblize with REAL DOS 3.3 gap sizes and build the per-nibble time map.
 * gap1 = 34 self-sync, gap2 = 6, gap3 = 14  ->  16 x 12,416 + 1,360 = 200,016 cyc. */
static void nibblize_dsk_timed(const uint8_t *dsk, uint8_t *nib) {
	static const uint8_t p2l[16] = {0,7,0xE,6,0xD,5,0xC,4,0xB,3,0xA,2,9,1,8,0xF};
	const int G1 = 34, G2 = 6, G3 = 14;
	uint8_t sync[6660];                        /* 1 = 10-bit self-sync byte */
	int track, ps, i, n = 0;
	memset(nib, 0xFF, 35 * 0x1A00);
	memset(sync, 0, sizeof sync);
	for (track = 0; track < 35; track++) {
		uint8_t *out = nib + track * 0x1A00;
		n = 0;
		for (i = 0; i < G1; i++) { *out++ = 0xFF; sync[n++] = 1; }
		for (ps = 0; ps < 16; ps++) {
			const uint8_t *sd = dsk + track * 16 * 256 + p2l[ps] * 256;
			uint8_t chk = 254 ^ track ^ ps, nb[342], last;
			if (track == 0) dt_secpos[ps] = n;
			*out++=0xD5; *out++=0xAA; *out++=0x96; n += 3;
			*out++=(254>>1)|0xAA; *out++=254|0xAA;                    n += 2;
			*out++=(track>>1)|0xAA; *out++=track|0xAA;                n += 2;
			*out++=(ps>>1)|0xAA; *out++=ps|0xAA;                      n += 2;
			*out++=(chk>>1)|0xAA; *out++=chk|0xAA;                    n += 2;
			*out++=0xDE; *out++=0xAA; *out++=0xEB;                    n += 3;
			for (i = 0; i < G2; i++) { *out++ = 0xFF; sync[n++] = 1; }
			*out++=0xD5; *out++=0xAA; *out++=0xAD;                    n += 3;
			for (i = 0; i < 256; i++) nb[86 + i] = sd[i] >> 2;
			for (i = 0; i < 86; i++) nb[i] = 0;
			for (i = 0; i < 256; i++)
				nb[i % 86] |= (((sd[i] & 1) << 1) | ((sd[i] & 2) >> 1)) << (2 * (i / 86));
			last = 0;
			for (i = 0; i < 342; i++) { *out++ = gcr6[(nb[i] ^ last) & 0x3F]; last = nb[i]; }
			*out++ = gcr6[last & 0x3F];                               n += 343;
			*out++=0xDE; *out++=0xAA; *out++=0xEB;                    n += 3;
			for (i = 0; i < G3; i++) { *out++ = 0xFF; sync[n++] = 1; }
		}
	}
	dt_ntrk = n;
	dt_pfx[0] = 0;
	for (i = 0; i < (int)dt_ntrk; i++) dt_pfx[i + 1] = dt_pfx[i] + (sync[i] ? dt_nibcyc + 8 : dt_nibcyc);
	dt_trev = dt_pfx[dt_ntrk];
}

void disktime_init(void) {
	const char *e;
	if (dt_init_done) return;
	dt_init_done = 1;
	e = getenv("A2_DISKTIME");  g_disktime = e ? atoi(e) : 0;
	e = getenv("A2_DISK_SPINUP"); dt_spinup = e ? (unsigned)strtoul(e, NULL, 0) : 0;
	e = getenv("A2_DISK_SETTLE"); dt_settle = e ? (unsigned)strtoul(e, NULL, 0) : 0;
	e = getenv("A2_DISK_SLIP");   dt_slip   = e ? atoi(e) : 0;
	e = getenv("A2_DISK_NIBCYC"); if (e) { dt_nibcyc = (unsigned)atoi(e); if (dt_nibcyc < 8) dt_nibcyc = 8; }
	e = getenv("A2_DISKLOG");     if (e && g_disktime) dt_log = fopen(e, "wb");
}

/* Current head position, given the cycle counter.  O(1) amortized: the common case
 * is a polling loop hammering $C0EC inside one nibble window. */
static unsigned dt_head(void) {
	unsigned t;
	if (!dt_spinning) return dt_pos;
	t = (ticks - dt_t0) % dt_trev;
	if (t < dt_pfx[dt_pos]) dt_pos = 0;                        /* wrapped */
	while (dt_pos + 1 < dt_ntrk && t >= dt_pfx[dt_pos + 1]) dt_pos++;
	return dt_pos;
}

static void dt_motor(int on) {
	if (!g_disktime) return;
	if (on && !dt_spinning) {
		dt_t0 = ticks - dt_frozen; dt_spinning = 1; dt_motor_t = ticks;
		dt_note(1, 0, 0, ticks);
	} else if (!on && dt_spinning) {
		dt_frozen = (ticks - dt_t0) % dt_trev; dt_spinning = 0;
		dt_note(2, 0, 0, ticks);
	}
}

void (*a2_title_hook)(void *wdo, const char *title) = NULL;

/* Image format from the file name: .po = ProDOS order, .nib = nibbles, anything
 * else 140 KB (.dsk/.do) = DOS order. */
int disk_fmt_of(const char *filename) {
	const char *e = strrchr(filename, '.');
	if (e && (!strcasecmp(e, ".po"))) return DISK_PO;
	if (e && (!strcasecmp(e, ".nib"))) return DISK_NIB;
	return DISK_DOS;
}

/* E6 write-back: every dirty, writable floppy goes back to its file in its own
 * format (.nib raw; .dsk/.do/.po decoded from the nibble stream, keeping any sector
 * that does not decode as it was in the file).  Returns 0 if a write failed. */
int a2_flush_disks(void) {
	int ok = 1;
	for (int d = 0; d < 2; d++) {
		if (!disk[d].dirty || disk[d].readOnly || !disk[d].filename[0]) continue;
		if (disk[d].fmt == DISK_NIB) { ok &= saveFloppy(d); disk[d].dirty = false; continue; }
		static uint8_t dsk[143360], tmp[143360];
		FILE *f = fopen(disk[d].filename, "rb");
		if (!f || fread(tmp, 1, sizeof tmp, f) != sizeof tmp) { if (f) fclose(f); ok = 0; continue; }
		fclose(f);
		if (disk[d].fmt == DISK_PO) po_to_dos(tmp, dsk); else memcpy(dsk, tmp, sizeof dsk);
		int n = denibblize(disk[d].data, dsk);
		if (n < 560) fprintf(stderr, "a2core: %s: only %d of 560 sectors decoded on write-back\n", disk[d].filename, n);
		if (disk[d].fmt == DISK_PO) { dos_to_po(dsk, tmp); memcpy(dsk, tmp, sizeof dsk); }
		f = fopen(disk[d].filename, "wb");
		if (!f || fwrite(dsk, 1, sizeof dsk, f) != sizeof dsk) ok = 0;
		if (f) fclose(f);
		disk[d].dirty = false;
	}
	return ok;
}

int insertFloppy(void *wdo, char *filename, int drv) {
	long sz;
	FILE *f;

	disktime_init();                                                              /*  */
	/* Force-zero the emulated RAM + language card and reset this drive's head
	 * state, in reinette.c itself -- don't rely on the loader zeroing BSS. */
	memset(ram, 0, sizeof ram);
	memset(lgc, 0, sizeof lgc);
	memset(bk2, 0, sizeof bk2);
	disk[drv].nibble = 0;
	disk[drv].track = 0;
	disk[drv].motorOn = false;
	disk[drv].writeMode = false;

	f = fopen(filename, "rb");
	if (!f) return 0;
	fseek(f, 0, SEEK_END); sz = ftell(f); fseek(f, 0, SEEK_SET);
	if (sz == 232960) {                                                        // .nib: load as-is
		if (fread(disk[drv].data, 1, 232960, f) != 232960) { fclose(f); return 0; }
	} else if (sz == 143360) {                                                 // .dsk/.do (DOS order) or .po
		static uint8_t dskbuf[143360], pobuf[143360];
		if (fread(dskbuf, 1, 143360, f) != 143360) { fclose(f); return 0; }
		if (disk_fmt_of(filename) == DISK_PO) {                                // ProDOS order -> DOS order
			memcpy(pobuf, dskbuf, sizeof pobuf);
			po_to_dos(pobuf, dskbuf);
		}
		if (g_disktime) nibblize_dsk_timed(dskbuf, disk[drv].data);            //  realistic gaps
		else            nibblize_dsk(dskbuf, disk[drv].data);
	} else { fclose(f); return 0; }                                           // unknown geometry
	fclose(f);

	sprintf(disk[drv].filename, "%s", filename);                                  // update disk filename record
	disk[drv].fmt = (sz == 232960) ? DISK_NIB : disk_fmt_of(filename);
	disk[drv].dirty = false;

	f = fopen(filename, "ab");                                                    // try to open the file in append binary mode
	if (f) {                                                                      // success, file is writable
		disk[drv].readOnly = false;                                                 // update the readOnly flag
		fclose(f);                                                                  // and close it untouched
	} else {
		disk[drv].readOnly = true;                                                  // f is NULL, no writable, no need to close it
	}
	char title[1000];                                                             // UPDATE WINDOW TITLE
	int i, a, b;

	i = a = 0;
	while (disk[0].filename[i] != 0)                                              // find start of filename for disk0
		if (disk[0].filename[i++] == '\\') a = i;
	i = b = 0;
	while (disk[1].filename[i] != 0)                                              // find start of filename for disk1
		if (disk[1].filename[i++] == '\\') b = i;

	sprintf(title, "Reinette ][+   D1: %s   D2: %s", disk[0].filename + a, disk[1].filename + b);
	if (a2_title_hook) a2_title_hook(wdo, title);                                 // updates window title

	return 1;
}


int saveFloppy(int drive) {
	if (!disk[drive].filename[0]) return 0;                                       // no file loaded into drive
	if (disk[drive].readOnly) return 0;                                           // file is read only write no aptempted

	FILE *f = fopen(disk[drive].filename, "wb");
	if (!f) return 0;                                                             // could not open the file in write overide binary

	if (fwrite(disk[drive].data, 1, 232960, f) != 232960) {                       // failed to write the full file (disk full ?)
		fclose(f);                                                                  // release the ressource
		return 0;
	}
	fclose(f);                                                                    // success, release the ressource
	return 1;
}


/* halfTrackPos is SNAPSHOT STATE: the stepper's absolute half-track position. */
int halfTrackPos[2] = { 0 };

void stepMotor(uint16_t address) {
	static bool phases[2][4] = { 0 };                                             // phases states (for both drives)
	static bool phasesB[2][4] = { 0 };                                            // phases states Before
	static bool phasesBB[2][4] = { 0 };                                           // phases states Before Before
	static int pIdx[2] = { 0 };                                                   // phase index (for both drives)
	static int pIdxB[2] = { 0 };                                                  // phase index Before

	address &= 7;
	int phase = address >> 1;

	phasesBB[curDrv][pIdxB[curDrv]] = phasesB[curDrv][pIdxB[curDrv]];
	phasesB[curDrv][pIdx[curDrv]]   = phases[curDrv][pIdx[curDrv]];
	pIdxB[curDrv] = pIdx[curDrv];
	pIdx[curDrv]  = phase;

	if (!(address & 1)) {                                                         // head not moving (PHASE x OFF)
		phases[curDrv][phase] = false;
		return;
	}

	if ((phasesBB[curDrv][(phase + 1) & 3]) && (--halfTrackPos[curDrv] < 0))      // head is moving in
		halfTrackPos[curDrv] = 0;

	if ((phasesBB[curDrv][(phase - 1) & 3]) && (++halfTrackPos[curDrv] > 140))    // head is moving out
		halfTrackPos[curDrv] = 140;

	phases[curDrv][phase] = true;                                                 // update track#
	{	int nt = (halfTrackPos[curDrv] + 1) / 2;
		/* : A2_DISK_SLIP=n drops every n-th half-track step, i.e. models a drive
		 * that UNDER-STEPS.  The head then sits on the wrong track, every address
		 * field mismatches, and only a real RECALIBRATE (step out against the track-0
		 * stop) can resynchronise -- which is exactly what the retry path must do. */
		if (dt_slip && nt != disk[curDrv].track) {
			if (++dt_slipn >= dt_slip) { dt_slipn = 0; halfTrackPos[curDrv] = disk[curDrv].track * 2; return; }
		}
		if (nt != disk[curDrv].track) { dt_settle_t = ticks; if (dt_log) dt_note(3, (uint8_t)nt, 0, ticks); }
		disk[curDrv].track = nt;
	}
}


static void setDrv(int drv) {
	disk[drv].motorOn = disk[!drv].motorOn || disk[drv].motorOn;                  // if any of the motors were ON
	disk[!drv].motorOn = false;                                                   // motor of the other drive is set to OFF
	curDrv = drv;                                                                 // set the current drive
}


//=========================================== RAM CARD ("Slinky") + DEBUG CARD
/* Apple II Memory Expansion Card model (plan A-D4, E5). Default OFF.
   $C0n0/1/2 = address bits 0-7 / 8-15 / 16-23 (read/write), $C0n3 = data,
   read or write, then the address auto-increments (24-bit wrap). Accesses use
   address & (size-1), i.e. the RAM mirrors above the card's size. Other offsets
   read $FF. NOTE: what the real card returns for the unused high address bits and
   for $C0n4-$C0nF is not modelled -- the game's detection must not rely on it. */
int      g_xmem_slot = 0;                /* 0 = no card */
uint32_t g_xmem_size = 0;                /* bytes (power of two) */
uint8_t *g_xmem      = NULL;
uint32_t g_xmem_addr = 0;
unsigned long g_xmem_reads = 0, g_xmem_writes = 0;

int xmem_init(unsigned kb, int slot) {
	if (!kb || slot < 1 || slot > 7 || (kb & (kb - 1))) return 0;   /* power of two KB */
	free(g_xmem);
	g_xmem_size = kb * 1024u;
	g_xmem = calloc(g_xmem_size, 1);
	if (!g_xmem) { g_xmem_slot = 0; return 0; }
	g_xmem_slot = slot; g_xmem_addr = 0;
	return 1;
}

static uint8_t xmem_io(uint16_t address, uint8_t value, bool WRT) {
	switch (address & 0x0F) {
	case 0: if (WRT) g_xmem_addr = (g_xmem_addr & 0xFFFF00u) | value;
	        return g_xmem_addr & 0xFF;
	case 1: if (WRT) g_xmem_addr = (g_xmem_addr & 0xFF00FFu) | ((uint32_t)value << 8);
	        return (g_xmem_addr >> 8) & 0xFF;
	case 2: if (WRT) g_xmem_addr = (g_xmem_addr & 0x00FFFFu) | ((uint32_t)value << 16);
	        return (g_xmem_addr >> 16) & 0xFF;
	case 3: { uint8_t *m = &g_xmem[g_xmem_addr & (g_xmem_size - 1)], v = *m;
	        if (WRT) { *m = value; g_xmem_writes++; } else g_xmem_reads++;
	        g_xmem_addr = (g_xmem_addr + 1) & 0xFFFFFFu;
	        return v; }
	default: return 0xFF;
	}
}

/* Debug card (plan A-D10, E7): emulator-only, in a slot's device-select window
   $C0n0-$C0nF (default slot 3 when a frontend enables it; A2_DEBUGCARD=<slot>).
     W $C0n0  char-out to stdout          W $C0n1  exit the emulator with this code
     W $C0n2  VERA screenshot             W $C0n3  VRAM dump
     R $C0n4  latch the 32-bit cycle counter, R $C0n4-7 = its bytes (LE)
     W $C0n8  profiler window (value 1 = start, 0 = stop)
     R $C0nF  = $A2 (ID: a debug card is present)
   Writes to 2,3,8.. go to the frontend's a2_dbg_hook. */
int      g_dbg_slot  = 0;
int      g_dbg_exit  = -1;               /* >= 0: the guest asked to exit */
uint32_t g_dbg_latch = 0;
void (*a2_dbg_hook)(int reg, uint8_t value) = NULL;

static uint8_t dbg_io(uint16_t address, uint8_t value, bool WRT) {
	int r = address & 0x0F;
	if (WRT) {
		if (r == 0) { putchar(value == 0x0D ? '\n' : value); if (value == 0x0D || value == 0x0A) fflush(stdout); }
		else if (r == 1) { fflush(stdout); if (g_dbg_exit < 0) g_dbg_exit = value; }   /* first exit wins: the
		                                                         runner stops at the end of the current step */
		else if (a2_dbg_hook) a2_dbg_hook(r, value);
		return 0;
	}
	if (r == 4) g_dbg_latch = ticks;
	if (r >= 4 && r <= 7) return (g_dbg_latch >> ((r - 4) * 8)) & 0xFF;
	if (r == 0x0F) return 0xA2;
	return 0;
}

__attribute__((constructor)) static void _cards_env(void) {
	const char *e = getenv("A2_XMEM");                /* <KB>[,<slot>] e.g. 512,5 */
	if (e) { unsigned kb = (unsigned)atoi(e); const char *c = strchr(e, ',');
	         xmem_init(kb, c ? atoi(c + 1) : 5); }
	e = getenv("A2_DEBUGCARD");
	if (e && atoi(e) >= 1 && atoi(e) <= 7) g_dbg_slot = atoi(e);
}


//========================================== MEMORY MAPPED SOFT SWITCHES HANDLER
// this function is called from readMem and writeMem
// it complements both functions when address is in page $C0
/* SNES MAX card model The SNES MAX is a slot card using /DEVICE SELECT only -- no slot ROM,... */
static uint16_t sm_sr1 = 0xFFFF, sm_sr2 = 0xFFFF;

/* $C036 / VERA speed trace A2_SPEEDLOG=<path> writes 6-byte LE records {u8 kind, u8 val, u32... */
void speedlog_note(uint8_t kind, uint8_t val, unsigned int tick) {
	if (!g_speedlog) return;
	fputc(kind, g_speedlog); fputc(val, g_speedlog);
	fputc(tick & 0xFF, g_speedlog);         fputc((tick >> 8) & 0xFF, g_speedlog);
	fputc((tick >> 16) & 0xFF, g_speedlog); fputc((tick >> 24) & 0xFF, g_speedlog);
}

uint8_t softSwitches(uint16_t address, uint8_t value, bool WRT) {
	static uint8_t dLatch = 0;                                                    // disk ][ I/O register
	if (g_snesmax_slot) {                                                         // SNES MAX (default-off)
		uint16_t smb = 0xC080 + g_snesmax_slot * 16;
		if (address == smb && WRT) {                                                // LATCH: sample both pads
			sm_sr1 = g_snesmax_bits; sm_sr2 = g_snesmax_p2; g_snesmax_latches++; return 0;
		}
		if (address == smb + 1 && WRT) {                                            // CLOCK: shift both
			sm_sr1 = (uint16_t)(sm_sr1 << 1) | 1; sm_sr2 = (uint16_t)(sm_sr2 << 1) | 1;
			g_snesmax_clocks++; return 0;
		}
		if (address == smb && !WRT) {                                               // DATA: MSB first
			g_snesmax_reads++;
			return (uint8_t)(((sm_sr1 & 0x8000) ? 0x80 : 0) | ((sm_sr2 & 0x8000) ? 0x40 : 0));
		}
	}
	if (g_xmem_slot && (address & 0xFFF0) == 0xC080 + g_xmem_slot * 16) return xmem_io(address, value, WRT);
	if (g_dbg_slot  && (address & 0xFFF0) == 0xC080 + g_dbg_slot  * 16) return dbg_io(address, value, WRT);
	if (g_iie && address <= 0xC01F && address != 0xC010) {
		if (address <= 0xC00F) {
			if (!WRT) return KBD;                                       /* reads = keyboard */
			bool on = address & 1;
			switch (address & 0x0E) {
			case 0x00: S80STORE = on; break;
			case 0x02: RAMRD = on; break;
			case 0x04: RAMWRT = on; break;
			case 0x06: INTCXROM = on; fetchSyncCx(); break;
			case 0x08: iie_set_altzp(on); fetchSyncAll(); break;
			case 0x0A: SLOTC3ROM = on; fetchSyncCx(); break;
			case 0x0C: S80COL = on; break;
			case 0x0E: ALTCHAR = on; break;
			}
			iie_update_memslow();
			fetchSyncRam();
			return 0;
		}
		if (!WRT && address >= 0xC011) {
			bool st = false;
			switch (address) {
			case 0xC011: st = LCBK2; break;
			case 0xC012: st = LCRD; break;
			case 0xC013: st = RAMRD; break;
			case 0xC014: st = RAMWRT; break;
			case 0xC015: st = INTCXROM; break;
			case 0xC016: st = ALTZP; break;
			case 0xC017: st = SLOTC3ROM; break;
			case 0xC018: st = S80STORE; break;
			case 0xC019: st = (ticks % 17030u) < 192u * 65u; break;     /* 0 during VBL */
			case 0xC01A: st = TEXT; break;
			case 0xC01B: st = MIXED; break;
			case 0xC01C: st = PAGE2; break;
			case 0xC01D: st = HIRES; break;
			case 0xC01E: st = ALTCHAR; break;
			case 0xC01F: st = S80COL; break;
			}
			return (uint8_t)((st ? 0x80 : 0) | (KBD & 0x7F));
		}
	}
	uint8_t iie_map_before = (uint8_t)(PAGE2 | (HIRES << 1));
	/* fetch-mirror: which bank is execute-visible at $D000-$FFFF right now.
	 * The LC cases below ($C080-$C08F) may change it; compare on the way out
	 * and remap fetchMem only on a REAL change (games hammering the same LC
	 * switch in a loop must not pay a 12 KB memcpy per access). */
	uint8_t oldMap = LCRD ? (LCBK2 ? 2 : 1) : 0;

	switch (address) {
  	case 0xC000:                                                    // KEYBOARD
  		if (g_kbd_log && (KBD & 0x80)) fprintf(stderr, "[kbd] $C000 read = $%02X\n", KBD);
  		return KBD;
  	case 0xC010:                                                               // KBDSTROBE (+ AKD on IIe+)
  		if (g_kbd_log && (KBD & 0x80)) { Puce6502State _s; puce6502GetState(&_s); fprintf(stderr, "[kbd] $C010 CLEARS $%02X from PC=$%04X\n", KBD, _s.pc); }
  		if (g_akd) { uint8_t akd = g_kbdDown | (KBD & 0x7F);
  			if (akd & 0x80) { g_akd_reads++;
  				if (g_kbd_log) fprintf(stderr, "[kbd] $C010 AKD = $%02X\n", akd); }
  			KBD &= 0x7F; return akd; }
  		KBD &= 0x7F; return KBD;

  	case 0xC020:                                                                // TAPEOUT (shall we listen it ? - try SAVE from applesoft)
  	case 0xC030:                                                                // SPEAKER
  	case 0xC033: playSound(); break;                                            // apple invader uses $C033 to output sound !

  	/* IIGS CYAREG $C036 A REAL register on a IIGS: bit7 = 1 -> 2. */
  	case 0xC036:
  		if (!g_cyareg_model) break;                                               // not modelled -> old behaviour
  		if (WRT) {
  			g_cyareg = value;
  			g_a2_speed_permille = (value & 0x80) ? g_gs_fast_permille : 1000;
  			speedlog_note(0, value, ticks);
  		}
  		return g_cyareg;

  	case 0xC050: TEXT  = false; break;                                          // Graphics
  	case 0xC051: TEXT  = true;  break;                                          // Text
  	case 0xC052: MIXED = false; break;                                          // Mixed off
  	case 0xC053: MIXED = true;  break;                                          // Mixed on
  	case 0xC054: PAGE2 = false; break;                                          // PAGE2 off
  	case 0xC055: PAGE2 = true;  break;                                          // PAGE2 on
  	case 0xC056: HIRES = false; break;                                          // HiRes off
  	case 0xC057: HIRES = true;  break;                                          // HiRes on

  	case 0xC058:                                                                // AN0 low
  		if (g_gpad_en) { if (g_gp_an0) { g_gpad_latch = g_gpad; g_gpad_idx = 0; } g_gp_an0 = 0; }
  		break;                                                                    //   LATCH only on high->low edge
  	case 0xC059: if (g_gpad_en) g_gp_an0 = 1; break;                            // AN0 high (arm the latch)
  	case 0xC05A: if (g_gpad_en) g_gp_an1 = 0; break;                            // AN1 low
  	case 0xC05B:                                                                // AN1 high
  		if (g_gpad_en) { if (!g_gp_an1 && g_gpad_idx < 8) g_gpad_idx++; g_gp_an1 = 1; }
  		break;                                                                    //   CLOCK/shift only on low->high edge

  	case 0xC061: return PB0;                                                    // Push Button 0
  	case 0xC062: return PB1;                                                    // Push Button 1
  	case 0xC063:                                                                // Push Button 2 / NES 4021 DATA
  		if (g_gpad_en) {
  			// serial out of the LATCHED value; past the 8 buttons the line reads low (NES tail)
  			int pressed = (g_gpad_idx < 8) ? ((g_gpad_latch >> (7 - g_gpad_idx)) & 1) : 1;
  			return pressed ? 0x00 : 0x80;                                         // active low: pressed -> data line 0
  		}
  		return PB2;
  	case 0xC064: return readPaddle(0);                                          // Paddle 0
  	case 0xC065: return readPaddle(1);                                          // Paddle 1
  	case 0xC066: return readPaddle(2);                                          // Paddle 2 (WP1.4)
  	case 0xC067: return readPaddle(3);                                          // Paddle 3

  	case 0xC070: resetPaddles(); break;                                         // paddle timer RST

    case 0xC080:                                                      // LANGUAGE CARD :
  	case 0xC084: LCBK2 = 1; LCRD = 1; LCWR = 0;      LCWFF = 0;    break;       // LC2RD
  	case 0xC081:
  	case 0xC085: LCBK2 = 1; LCRD = 0; LCWR |= LCWFF; LCWFF = !WRT; break;       // LC2WR
  	case 0xC082:
  	case 0xC086: LCBK2 = 1; LCRD = 0; LCWR = 0;      LCWFF = 0;    break;       // ROMONLY2
  	case 0xC083:
  	case 0xC087: LCBK2 = 1; LCRD = 1; LCWR |= LCWFF; LCWFF = !WRT; break;       // LC2RW
  	case 0xC088:
  	case 0xC08C: LCBK2 = 0; LCRD = 1; LCWR = 0;      LCWFF = 0;    break;       // LC1RD
  	case 0xC089:
  	case 0xC08D: LCBK2 = 0; LCRD = 0; LCWR |= LCWFF; LCWFF = !WRT; break;       // LC1WR
  	case 0xC08A:
  	case 0xC08E: LCBK2 = 0; LCRD = 0; LCWR = 0;      LCWFF = 0;    break;       // ROMONLY1
  	case 0xC08B:
  	case 0xC08F: LCBK2 = 0; LCRD = 1; LCWR |= LCWFF; LCWFF = !WRT; break;       // LC1RW

		case 0xC0E0:
		case 0xC0E1:
		case 0xC0E2:
		case 0xC0E3:
		case 0xC0E4:
    case 0xC0E5:
		case 0xC0E6:
		case 0xC0E7: stepMotor(address); break;                                     // MOVE DRIVE HEAD

  	case 0xCFFF:
  	case 0xC0E8: disk[curDrv].motorOn = false; dt_motor(0); break;              // MOTOROFF
  	case 0xC0E9: disk[curDrv].motorOn = true;  dt_motor(1); break;              // MOTORON

  	case 0xC0EA: setDrv(0); break;                                              // DRIVE0EN
  	case 0xC0EB: setDrv(1); break;                                              // DRIVE1EN

  	case 0xC0EC:                                                     // Shift Data Latch
  		if (g_disktime && !disk[curDrv].writeMode) {                              /*  rotational model */
  			unsigned p;
  			if (!disk[curDrv].motorOn) return 0;                                    /* head parked        */
  			if (dt_spinup && (unsigned)(ticks - dt_motor_t) < dt_spinup) return 0;  /* spindle spinning up */
  			if (dt_settle && (unsigned)(ticks - dt_settle_t) < dt_settle) return 0; /* head still settling */
  			p = dt_head();
  			if (p == dt_delivered) return 0;                        /* same byte still under the head:
  			                                                           the latch is NOT ready (bit 7 clear) */
  			if (dt_delivered != 0xFFFFFFFFu) {                      /* count nibbles that went by unread */
  				unsigned gap = (p >= dt_delivered) ? p - dt_delivered : p + dt_ntrk - dt_delivered;
  				if (gap > 1) g_dt_missed += gap - 1;
  			}
  			dt_delivered = p;
  			disk[curDrv].nibble = p;
  			dLatch = disk[curDrv].data[disk[curDrv].track * 0x1A00 + p];
  			if (dt_log && p == dt_secpos[0]) dt_note(0, disk[curDrv].track, 0, ticks);
  			else if (dt_log) { int s; for (s = 1; s < 16; s++) if (p == dt_secpos[s]) { dt_note(0, disk[curDrv].track, (uint8_t)s, ticks); break; } }
  			return dLatch;
  		}
  		if (disk[curDrv].writeMode) {                                           // writting
  			disk[curDrv].data[disk[curDrv].track*0x1A00+disk[curDrv].nibble]=dLatch;// good luck gcc
  			disk[curDrv].dirty = true;                                              // E6: write back on exit
  		}
  		else                                                                      // reading
  			dLatch=disk[curDrv].data[disk[curDrv].track*0x1A00+disk[curDrv].nibble];// easy peasy
  		{ static int dbgn=0; extern int g_disk_dbg; if(g_disk_dbg && dbgn<32){ fprintf(stderr,"[nib] drv=%d trk=%d nib=%d val=%02X motor=%d\n",curDrv,disk[curDrv].track,disk[curDrv].nibble,dLatch,disk[curDrv].motorOn); dbgn++; } }
  		disk[curDrv].nibble = (disk[curDrv].nibble + 1) % (g_disktime ? dt_ntrk : 0x1A00);   // turn floppy of 1 nibble
  		return dLatch;

  	case 0xC0ED: dLatch = value; break;                                         // Load Data Latch

  	case 0xC0EE:                                                                // latch for READ
  		disk[curDrv].writeMode = false;
  		return disk[curDrv].readOnly ? 0x80 : 0;                                  // check protection

  	case 0xC0EF:                                                                // latch for WRITE
  		if (g_disktime && dt_spinning && !disk[curDrv].writeMode)                 // timed: start on the nibble
  			disk[curDrv].nibble = (dt_head() + 1) % dt_ntrk;                        //   coming under the head, not
  		disk[curDrv].writeMode = true;                                            //   over the one already read
  		break;

  	/* --- slot-7 ProDOS hard-disk card I/O registers --- */
  	case 0xC0F0: if (WRT) hd_cmd   = value;                          break;     // command
  	case 0xC0F1: if (WRT) hd_unit  = value;                          break;     // unit
  	case 0xC0F2: if (WRT) hd_block = (hd_block & 0xFF00) | value;    break;     // block lo
  	case 0xC0F3: if (WRT) hd_block = (hd_block & 0x00FF) | (value << 8); break; // block hi
  	case 0xC0F4: if (WRT) hd_buf   = (hd_buf   & 0xFF00) | value;    break;     // buffer lo
  	case 0xC0F5: if (WRT) hd_buf   = (hd_buf   & 0x00FF) | (value << 8); break; // buffer hi
  	case 0xC0F6: if (WRT) hd_exec();                                 break;     // execute the block op
  	case 0xC0F7: return hd_status;                                              // error code (0 = ok)
  	case 0xC0F8: return hd_blocks & 0xFF;                                       // # blocks lo (STATUS)
  	case 0xC0F9: return (hd_blocks >> 8) & 0xFF;                                // # blocks hi
	}
	if ((LCRD ? (LCBK2 ? 2 : 1) : 0) != oldMap)
		fetchSyncHigh();                                                            // LC remap -> refresh fetch mirror
	if (g_iie && S80STORE && (uint8_t)(PAGE2 | (HIRES << 1)) != iie_map_before)
		fetchSyncRam();                                                             // 80STORE: PAGE2/HIRES move RAM
	return ticks % 0xFF;                                                          // catch all, gives a 'floating' value
}


//======================================================================= MEMORY
// these two functions are imported into puce6502.c

/* readMemSlow / writeMemSlow: out-of-line fallbacks for the puce6502 inline
 * fast paths (RAM + normal ROM are inlined in puce6502.c).  Reached only for
 * soft switches ($C0xx), slot-6 ROM, and language-card banking. */
/* $Cxxx access log -- diagnose the real-HW VERA-detection freeze. Records every
 * slot-window read/write ($C080-$CFFF; skips $C000-$C07F keyboard/main switches). */
int   g_cxxx_log = 0;
FILE *g_cxxx_fp  = NULL;
static inline void cxxx_note(char rw, uint16_t a, uint8_t v) {
	if (g_cxxx_log && g_cxxx_fp && a >= 0xC100 && a < 0xD000)
		fprintf(g_cxxx_fp, "%c $%04X = $%02X   (slot %d, ofs $%02X)\n",
		        rw, a, v, (a >> 8) & 0x0F, a & 0x1F);
}

static uint8_t readMemSlow_impl(uint16_t address);
uint8_t readMemSlow(uint16_t address) {
	uint8_t v = readMemSlow_impl(address);
	cxxx_note('R', address, v);
	return v;
}
static uint8_t readMemSlow_impl(uint16_t address) {
	if (address < RAMSIZE)
		return g_memslow ? *iie_bank(address, 0) : ram[address];                    // RAM (IIe banks)

	if (g_iie && address >= 0xC100 && address < 0xD000 &&                        // IIe internal ROM
	    (address >= 0xC800 || INTCXROM || ((address & 0xFF00) == 0xC300 && !SLOTC3ROM)))
		return iiecx[address - 0xC000];

	if (address >= ROMSTART) {
		if (!LCRD)
			return rom[address - ROMSTART];                                           // ROM

		if (LCBK2 && (address < 0xE000))
			return bk2[address - BK2START];                                           // BK2

		return lgc[address - LGCSTART];                                             // LC
	}

	if ((address & 0xFF00) == SL6START)
		return sl6[address - SL6START];                                             // disk][ or hard-disk card

	if (g_hd_present && (address & 0xFF00) == SL7START)
		return sl7[address - SL7START];                                             // slot-7 block device firmware

	if (g_vera_enabled && (address & 0xFFE0) == g_vera_base) {
		speedlog_note(1, g_cyareg, ticks);                                           // : speed at this access
		return vc_read(&g_vera, address & 0x1F);                                     // VERA card ($C400-$C41F)
	}
	if (g_vera_enabled && g_ym_enabled && (address & 0xFFFE) == g_vera_base + 0x20)
		return ym_read(address & 1);                                                 // A2VERA YM2151 ($Cn20-$Cn21, E14)

	if ((address & 0xF000) == 0xC000)
		return softSwitches(address, 0, false);                                     // Soft Switches

	return ticks & 0xFF;                                                          // catch all, gives a 'floating' value
}


void writeMemSlow(uint16_t address, uint8_t value) {
	cxxx_note('W', address, value);
	if (address < RAMSIZE) {
		if (g_memslow) {                                                            // IIe banks
			uint8_t *w = iie_bank(address, 1);
			*w = value;
			if (w == iie_bank(address, 0)) { fetchMem[address] = value; JIT_PAGE_CHECK(address); }
			return;
		}
		ram[address] = value;                                                       // RAM
		fetchMem[address] = value;
		return;
	}
	if (g_iie && INTCXROM && address >= 0xC100 && address < 0xC800) return;      // slot I/O-select inhibited

	if (LCWR && (address >= ROMSTART)) {
		if (LCBK2 && (address < 0xE000)) {
			bk2[address - BK2START] = value;                                          // BK2
			if (LCRD) { fetchMem[address] = value; JIT_PAGE_CHECK(address); }         // bank is execute-visible
			return;
		}
		lgc[address - LGCSTART] = value;                                            // LC
		if (LCRD) { fetchMem[address] = value;           /* this branch == the bank
		    LCRD reads at this address (bk2 was handled above), so mirror it */
			JIT_PAGE_CHECK(address); }
		return;
	}

	if (g_vera_enabled && (address & 0xFFE0) == g_vera_base) {
		speedlog_note(1, g_cyareg, ticks);                                           // : speed at this access
		vc_write(&g_vera, address & 0x1F, value);                                    // VERA card ($C400-$C41F)
		return;
	}
	if (g_vera_enabled && g_ym_enabled && (address & 0xFFFE) == g_vera_base + 0x20) {
		ym_write(address & 1, value);                                                // A2VERA YM2151 ($Cn20-$Cn21, E14)
		return;
	}

	if ((address & 0xF000) == 0xC000) {
		softSwitches(address, value, true);                                         // Soft Switches
		return;
	}
}


//=============================================================== RESET / MODEL
/* Reset as the machine does it: on the IIe the MMU clears the soft switches and the
 * language card goes back to "read ROM, write bank 2" (so RESET always runs the ROM's
 * handler, which then honours the $3F2-$3F4 soft-reset vector).  The ][+ keeps its
 * language-card state (a II+ with LC RAM mapped takes RESET through $FFFC in RAM). */
void a2_reset(void) {
	if (g_iie) {
		iie_set_altzp(false);
		S80STORE = RAMRD = RAMWRT = INTCXROM = SLOTC3ROM = S80COL = ALTCHAR = false;
		LCRD = false; LCBK2 = true; LCWR = true; LCWFF = false;
		iie_update_memslow();
	}
	fetchSyncAll();
	puce6502RST();
}

/* IIe ROMs: two 8 KB files from `dir`, found by part number in the file name
 * (enhanced: 342-0304 = $C000-$DFFF, 342-0303 = $E000-$FFFF; unenhanced: 342-0135 +
 * 342-0134).  Returns 1 and switches the machine to IIe mode (AKD keyboard on). */
#include <dirent.h>
static int read_part(const char *dir, const char *part, uint8_t *buf) {
	DIR *d = opendir(dir); struct dirent *e; int ok = 0;
	if (!d) return 0;
	while ((e = readdir(d))) {
		if (!strstr(e->d_name, part)) continue;
		char p[1200]; snprintf(p, sizeof p, "%s/%s", dir, e->d_name);
		FILE *f = fopen(p, "rb");
		if (f) { ok = fread(buf, 1, 0x2000, f) == 0x2000; fclose(f); }
		if (ok) break;
	}
	closedir(d);
	return ok;
}
int a2_load_iie_roms(const char *dir, int enhanced) {
	static uint8_t cd[0x2000], ef[0x2000];
	if (!read_part(dir, enhanced ? "342-0304" : "342-0135", cd)) return 0;
	if (!read_part(dir, enhanced ? "342-0303" : "342-0134", ef)) return 0;
	memcpy(iiecx, cd, 0x1000);                  /* $C000-$CFFF (first page unused) */
	memcpy(rom, cd + 0x1000, 0x1000);           /* $D000-$DFFF */
	memcpy(rom + 0x1000, ef, 0x2000);           /* $E000-$FFFF */
	g_iie = enhanced ? 2 : 1;
	g_akd = 1;
	return 1;
}

//================================================== SAVE / LOAD STATE (snapshot)
/* A snapshot is the full live machine state in a file, so a game resumes INSTANTLY instead of... */

int reinetteSaveState(const char *path) {
	FILE *f = fopen(path, "wb");
	if (!f) return -1;
	Puce6502State cpu; puce6502GetState(&cpu);
	unsigned int tk = ticks;
	uint8_t sw = (TEXT  ? 0x01 : 0) | (MIXED ? 0x02 : 0) | (PAGE2 ? 0x04 : 0)
	           | (HIRES ? 0x08 : 0) | (LCWR  ? 0x10 : 0) | (LCRD  ? 0x20 : 0)
	           | (LCBK2 ? 0x40 : 0) | (LCWFF ? 0x80 : 0);
	fwrite("R6S1", 1, 4, f);
	fputc((cpu.pc >> 8) & 0xFF, f); fputc(cpu.pc & 0xFF, f);          /* pc BE */
	fputc(cpu.a, f); fputc(cpu.x, f); fputc(cpu.y, f);
	fputc(cpu.sp, f); fputc(cpu.p, f); fputc(0, f);                   /* pad to 8 */
	fputc((tk >> 24) & 0xFF, f); fputc((tk >> 16) & 0xFF, f);         /* ticks BE */
	fputc((tk >> 8) & 0xFF, f);  fputc(tk & 0xFF, f);
	fwrite(ram, 1, RAMSIZE, f);
	fwrite(lgc, 1, LGCSIZE, f);
	fwrite(bk2, 1, BK2SIZE, f);
	fputc(sw, f); fputc(KBD, f); fputc((uint8_t)curDrv, f); fputc(muted ? 1 : 0, f);
	for (int d = 0; d < 2; d++) {
		fwrite(disk[d].filename, 1, sizeof disk[d].filename, f);
		fputc(disk[d].motorOn, f); fputc(disk[d].writeMode, f);
		fputc(disk[d].readOnly, f); fputc(disk[d].track, f);
		fputc(disk[d].nibble & 0xFF, f); fputc((disk[d].nibble >> 8) & 0xFF, f);
	}
	/* appended (R6S1-compatible): stepper half-track positions -- without
	 * them a resumed game's first head-step teleports the head (see
	 * halfTrackPos above) */
	fputc(halfTrackPos[0] & 0xFF, f); fputc(halfTrackPos[1] & 0xFF, f);
	fclose(f);
	return 0;
}

int reinetteLoadState(const char *path) {
	FILE *f = fopen(path, "rb");
	if (!f) return -1;
	char magic[4];
	if (fread(magic, 1, 4, f) != 4 || memcmp(magic, "R6S1", 4)) { fclose(f); return -1; }
	Puce6502State cpu; unsigned int tk;
	{
		uint8_t h[12];
		if (fread(h, 1, 12, f) != 12) { fclose(f); return -1; }
		cpu.pc = (h[0] << 8) | h[1];                                    /* pc BE */
		cpu.a = h[2]; cpu.x = h[3]; cpu.y = h[4]; cpu.sp = h[5]; cpu.p = h[6];
		tk = ((unsigned int)h[8] << 24) | ((unsigned int)h[9] << 16)    /* ticks BE */
		   | ((unsigned int)h[10] << 8) | h[11];
	}
	/* insertFloppy (the disk re-insert below) FORCE-ZEROES ram/lgc/bk2 for a clean boot -- so... */
	if (fseek(f, 16 + RAMSIZE + LGCSIZE + BK2SIZE, SEEK_SET)) { fclose(f); return -1; }
	int sw = fgetc(f);
	TEXT  = (sw & 0x01) != 0; MIXED = (sw & 0x02) != 0; PAGE2 = (sw & 0x04) != 0;
	HIRES = (sw & 0x08) != 0; LCWR  = (sw & 0x10) != 0; LCRD  = (sw & 0x20) != 0;
	LCBK2 = (sw & 0x40) != 0; LCWFF = (sw & 0x80) != 0;
	KBD = fgetc(f); curDrv = fgetc(f); muted = fgetc(f) != 0;
	for (int d = 0; d < 2; d++) {
		char fn[sizeof disk[d].filename];
		fread(fn, 1, sizeof fn, f);
		int mo = fgetc(f), wm = fgetc(f), ro = fgetc(f), tr = fgetc(f);
		int nlo = fgetc(f), nhi = fgetc(f);
		if (fn[0] && strcmp(disk[d].filename, fn) != 0) {
			/* Re-insert the snapshot's disk image so the resumed game can keep
			 * reading it.  Host-generated snapshots embed HOST paths -- if the
			 * exact path fails, retry by basename in the disk library dir. */
			if (!insertFloppy((void *)0, fn, d)) {
				char alt[sizeof fn]; const char *base = strrchr(fn, '/');
				int n = snprintf(alt, sizeof alt, "/usr/games/appleII/disk/%s",
				                 base ? base + 1 : fn);
				if (n >= 0 && (size_t)n < sizeof alt && insertFloppy((void *)0, alt, d))
					strcpy(fn, alt);   /* keep state consistent */
			}
		}
		if (fn[0]) {
			disk[d].track = (uint8_t) tr;
			disk[d].nibble = (uint16_t)(nlo | (nhi << 8));
			disk[d].motorOn = mo != 0; disk[d].writeMode = wm != 0;
			disk[d].readOnly = ro != 0;
		}
	}
	/* stepper half-track positions (appended field; old snapshots lack it --
	 * derive from the restored track so the head at least starts aligned) */
	{
		int h0 = fgetc(f), h1 = fgetc(f);
		halfTrackPos[0] = (h0 == EOF) ? disk[0].track * 2 : h0;
		halfTrackPos[1] = (h1 == EOF) ? disk[1].track * 2 : h1;
	}
	/* disks are in (any RAM wipe is done); NOW load the memory image */
	if (fseek(f, 16, SEEK_SET)) { fclose(f); return -1; }
	if (fread(ram, 1, RAMSIZE, f) != RAMSIZE) { fclose(f); return -1; }
	fread(lgc, 1, LGCSIZE, f);
	fread(bk2, 1, BK2SIZE, f);
	fclose(f);
	ticks = tk;
	puce6502SetState(&cpu);
	fetchSyncAll();                                  /* ram/lgc/bk2 + LC map just changed wholesale */
	return 0;
}


