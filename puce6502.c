/*
  puce6502 - MOS 6502 cpu emulator
  Last modified 21st of June 2021
  Copyright (c) 2018 Arthur Ferreira

  This version has been modified for reinette II plus, a french Apple II plus
  emulator using SDL2 (https://github.com/ArthurFerreira2/reinette-II-plus).

  Please download the latest version from
  https://github.com/ArthurFerreira2/puce6502

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/


// set to zero for 'normal' use
// or to 1 if you want to run the functionnal tests
#define _FUNCTIONNAL_TESTS 0

#include <stdint.h>
#include "puce6502.h"
#include <stdlib.h>   /* getenv/atoi/atexit --  phantom-read model (default OFF) */
#include <stdio.h>    /*  A2_PHANTOM_STATS */
#include <string.h>   /*  A2_CPU personality name compare */

#define CARRY 0x01
#define ZERO  0x02
#define INTR  0x04
#define DECIM 0x08
#define BREAK 0x10
#define UNDEF 0x20
#define OFLOW 0x40
#define SIGN  0x80

#if _FUNCTIONNAL_TESTS

	// for functionnal tests, see main()
	uint8_t RAM[65536];
	inline uint8_t readMem(uint16_t address) { return RAM[address]; }
	inline void writeMem(uint16_t address, uint8_t value) { RAM[address] = value; }
	#define fetch(a) readMem(a)   /* no fetch mirror in the test harness */

#else

	/* Machine memory (reinette. c). The 6502 core hits memory ~2-3x per instruction, so a JSR per... */
	extern uint8_t ram[];                  /* $0000-$BFFF */
	extern uint8_t rom[];                  /* $D000-$FFFF */
	extern uint8_t KBD;                    /* $C000 keyboard latch */
	extern bool    LCRD;                   /* language-card read (false => ROM mapped) */
	extern uint8_t readMemSlow(uint16_t address);
	extern void    writeMemSlow(uint16_t address, uint8_t value);
	extern uint8_t fetchMem[];             /* 64 KB execute-view mirror (reinette.c) */
	extern int     g_memslow;              /* IIe aux banking active: RAM via the slow path */

#if defined(__mc68000__) || defined(JIT_HOST_EMITTEST)
	/* JIT coherency: EVERY store path must invalidate translations of the written page, or stale... */
	extern uint8_t jit_pgcode_g[256];
	extern void jit_page_smc_g(unsigned page);
	#define JIT_STORE_CHECK(ad) do { \
		if (jit_pgcode_g[(uint16_t)(ad) >> 8]) jit_page_smc_g((uint16_t)(ad) >> 8); \
	} while (0)
#else
	#define JIT_STORE_CHECK(ad) ((void)0)
#endif

	/* Instruction fetch: ONE unconditional indexed load.  reinette.c keeps
	 * fetchMem[] coherent (RAM writes mirrored below, LC remaps + bulk loads
	 * resynced), so opcode/operand fetches skip the readMem compare chain
	 * entirely -- Applesoft executes from ROM, the WORST readMem path. */
	#define fetch(a) fetchMem[a]

	static inline uint8_t readMem(uint16_t address) {
		if (address < 0xC000 && !g_memslow) return ram[address];                  /* RAM */
		if (address == 0xC000) return KBD;                          /* keyboard:
		    99% of all $C0xx reads (Apple input loops poll LDA $C000/BPL) --
		    fast-path it inline to skip the readMemSlow+softSwitches call chain. */
		if (address >= 0xD000 && !LCRD) return rom[address - 0xD000];/* ROM */
		return readMemSlow(address);                                /* $C0xx / LC */
	}
	static inline void writeMem(uint16_t address, uint8_t value) {
		if (address < 0xC000 && !g_memslow) { ram[address] = value;
		                        fetchMem[address] = value;
		                        JIT_STORE_CHECK(address); return; } /* RAM (+ mirror) */
		writeMemSlow(address, value);                               /* $C0xx / LC */
	}

#endif


unsigned int ticks = 0;  // accumulated clock cycles (32-bit; wraps ~70 min, OK)


/*  PHANTOM (DUMMY) READ MODEL -- **DEFAULT OFF**, env `A2_PHANTOM=<mode>` Same discipline... */
#define PH_OFF   0
#define PH_NMOS  1
#define PH_65816 2
#define PH_65C02 3
int g_phantom = PH_OFF;          /* harnesses may set this directly */
/* Per-CLASS counters, so a test can assert the MECHANISM and not merely the answer ('s... */
unsigned long g_ph_w_izy = 0;    /* STA (zp),Y            -- ⭐ the  case */
unsigned long g_ph_w_absidx = 0; /* STA abs,X / abs,Y  (+ RMW abs,X index cycle) */
unsigned long g_ph_r_absidx = 0; /* LDA-class abs,X / abs,Y, page cross only */
unsigned long g_ph_r_izy = 0;    /* LDA-class (zp),Y,       page cross only */
unsigned long g_ph_rmw = 0;      /* RMW dummy write (NMOS) / second read (C02/816) */
static int g_phantom_env_done = 0;
static const char *g_phantom_statpath = NULL;

static void phantom_stats_dump(void) {
	FILE *f;
	if (!g_phantom_statpath) return;
	f = fopen(g_phantom_statpath, "w");
	if (!f) return;
	fprintf(f, "mode %d\nw_izy %lu\nw_absidx %lu\nr_absidx %lu\nr_izy %lu\nrmw %lu\n",
	        g_phantom, g_ph_w_izy, g_ph_w_absidx, g_ph_r_absidx, g_ph_r_izy, g_ph_rmw);
	fclose(f);
}

static void badop_init(void);          /* , defined just below */
static void cpu_init(void);            /*  CPU personality, ditto      */

void puce6502_phantom_init(void) {
	const char *e;
	if (g_phantom_env_done) return;
	g_phantom_env_done = 1;
	e = getenv("A2_PHANTOM");
	if (e) g_phantom = atoi(e);
	if (g_phantom < 0 || g_phantom > 3) g_phantom = PH_OFF;
	g_phantom_statpath = getenv("A2_PHANTOM_STATS");
	if (g_phantom_statpath) atexit(phantom_stats_dump);
	badop_init();
	cpu_init();
}

/*  65816-DIVERGENCE LOGGER -- **DEFAULT OFF**, env `A2_BADOP=<path>`. */
int g_badop = 0;                        /* harnesses may set this directly */
unsigned long g_badop_hits = 0;         /* total executions of a non-NMOS opcode */
#define BADOP_MAX 256
static struct { uint16_t pc; uint8_t op; unsigned long n; } g_badop_tbl[BADOP_MAX];
static int g_badop_used = 0;
static const char *g_badop_path = NULL;

void badop_record(uint16_t pc, uint8_t op) {
	int i;
	g_badop_hits++;
	for (i = 0; i < g_badop_used; i++)
		if (g_badop_tbl[i].pc == pc && g_badop_tbl[i].op == op) {
			g_badop_tbl[i].n++;
			return;
		}
	if (g_badop_used >= BADOP_MAX) return;
	g_badop_tbl[g_badop_used].pc = pc;
	g_badop_tbl[g_badop_used].op = op;
	g_badop_tbl[g_badop_used].n = 1;
	g_badop_used++;
}

static void badop_dump(void) {
	FILE *f;
	int i;
	if (!g_badop_path) return;
	f = fopen(g_badop_path, "w");
	if (!f) return;
	fprintf(f, "total %lu\ndistinct %d\n", g_badop_hits, g_badop_used);
	for (i = 0; i < g_badop_used; i++)
		fprintf(f, "%04X %02X %lu\n",
		        g_badop_tbl[i].pc, g_badop_tbl[i].op, g_badop_tbl[i].n);
	fclose(f);
}

static void badop_init(void) {
	g_badop_path = getenv("A2_BADOP");
	if (g_badop_path) { g_badop = 1; atexit(badop_dump); }
}

/*  CPU PERSONALITY MODEL -- **DEFAULT OFF**, env `A2_CPU=<name>` A2_CPU=off | nmos | 65c02... */
#define CPU_DEFAULT 0
#define CPU_NMOS    1
#define CPU_65C02   2
#define CPU_65816   3
int g_cpu = CPU_DEFAULT;              /* harnesses may set this directly */

int      g_cpu_trapped = 0;           /* 1 once the CPU has jammed */
uint16_t g_cpu_trap_pc = 0;
uint8_t  g_cpu_trap_op = 0;
const char *g_cpu_trap_why = NULL;
static const char *g_cpu_trap_path = NULL;

/* 65816 registers this core did not have.  Only the ones the modelled opcodes
 * touch; all zero at reset, which is what the IIGS firmware leaves for a
 * program running in emulation mode out of bank 0. */
uint8_t  g_816_B   = 0;               /* hidden high byte of C (XBA)      */
uint16_t g_816_D   = 0;               /* direct page                      */
uint8_t  g_816_DBR = 0;               /* data bank                        */
uint8_t  g_816_PBR = 0;               /* program bank                     */

void cpu_trap(uint16_t pc, uint8_t op, const char *why) {
	if (g_cpu_trapped) return;        /* first trap wins; it jams from here */
	g_cpu_trapped = 1;
	g_cpu_trap_pc = pc; g_cpu_trap_op = op; g_cpu_trap_why = why;
	fprintf(stderr, "A2_CPU TRAP: PC $%04X op $%02X -- %s\n", pc, op, why);
	if (g_cpu_trap_path) {
		FILE *f = fopen(g_cpu_trap_path, "w");
		if (f) { fprintf(f, "pc %04X\nop %02X\nwhy %s\n", pc, op, why); fclose(f); }
	}
}

static void cpu_init(void) {
	const char *e = getenv("A2_CPU");
	g_cpu_trap_path = getenv("A2_CPUTRAP");
	if (!e) return;
	if      (!strcmp(e, "nmos"))  g_cpu = CPU_NMOS;
	else if (!strcmp(e, "65c02") || !strcmp(e, "65C02")) g_cpu = CPU_65C02;
	else if (!strcmp(e, "65816")) g_cpu = CPU_65816;
	else                          g_cpu = CPU_DEFAULT;   /* "off" or garbage */
}

/* 65C02 (WDC W65C02S) encoding of every byte that reaches op_bad: high nibble = instruction... */
static const unsigned char c02_enc[256] = {
/*        0    1    2    3    4    5    6    7    8    9    A    B    C    D    E    F */
/*0*/  0x00,0x00,0x22,0x11,0x00,0x00,0x00,0x11,0x00,0x00,0x00,0x11,0x00,0x00,0x00,0x11,
/*1*/  0x00,0x00,0x00,0x11,0x00,0x00,0x00,0x11,0x00,0x00,0x00,0x11,0x00,0x00,0x00,0x11,
/*2*/  0x00,0x00,0x22,0x11,0x00,0x00,0x00,0x11,0x00,0x00,0x00,0x11,0x00,0x00,0x00,0x11,
/*3*/  0x00,0x00,0x00,0x11,0x00,0x00,0x00,0x11,0x00,0x00,0x00,0x11,0x00,0x00,0x00,0x11,
/*4*/  0x00,0x00,0x22,0x11,0x23,0x00,0x00,0x11,0x00,0x00,0x00,0x11,0x00,0x00,0x00,0x11,
/*5*/  0x00,0x00,0x00,0x11,0x24,0x00,0x00,0x11,0x00,0x00,0x00,0x11,0x38,0x00,0x00,0x11,
/*6*/  0x00,0x00,0x22,0x11,0x00,0x00,0x00,0x11,0x00,0x00,0x00,0x11,0x00,0x00,0x00,0x11,
/*7*/  0x00,0x00,0x00,0x11,0x00,0x00,0x00,0x11,0x00,0x00,0x00,0x11,0x00,0x00,0x00,0x11,
/*8*/  0x00,0x00,0x22,0x11,0x00,0x00,0x00,0x11,0x00,0x00,0x00,0x11,0x00,0x00,0x00,0x11,
/*9*/  0x00,0x00,0x00,0x11,0x00,0x00,0x00,0x11,0x00,0x00,0x00,0x11,0x00,0x00,0x00,0x11,
/*A*/  0x00,0x00,0x00,0x11,0x00,0x00,0x00,0x11,0x00,0x00,0x00,0x11,0x00,0x00,0x00,0x11,
/*B*/  0x00,0x00,0x00,0x11,0x00,0x00,0x00,0x11,0x00,0x00,0x00,0x11,0x00,0x00,0x00,0x11,
/*C*/  0x00,0x00,0x22,0x11,0x00,0x00,0x00,0x11,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x11,
/*D*/  0x00,0x00,0x00,0x11,0x24,0x00,0x00,0x11,0x00,0x00,0x00,0x00,0x34,0x00,0x00,0x11,
/*E*/  0x00,0x00,0x22,0x11,0x00,0x00,0x00,0x11,0x00,0x00,0x00,0x11,0x00,0x00,0x00,0x11,
/*F*/  0x00,0x00,0x00,0x11,0x24,0x00,0x00,0x11,0x00,0x00,0x00,0x11,0x34,0x00,0x00,0x11
};


static uint16_t PC;  //  Program Counter
static uint8_t A, X, Y, SP;  // Accumulator, X and y indexes and Stack Pointer
typedef union {
	uint8_t byte;
  /* Bitfield bit-ordering is endian-dependent. */
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
  struct {
    uint8_t S : 1;  // Sign            (bit 7)
    uint8_t V : 1;  // Overflow
    uint8_t U : 1;  // Undefined
    uint8_t B : 1;  // Break
    uint8_t D : 1;  // Decimal
    uint8_t I : 1;  // Interupt-disable
    uint8_t Z : 1;  // Zero
    uint8_t C : 1;  // Carry            (bit 0)
  };
#else
  struct {
    uint8_t C : 1;  // Carry            (bit 0)
    uint8_t Z : 1;  // Zero
    uint8_t I : 1;  // Interupt-disable
    uint8_t D : 1;  // Decimal
    uint8_t B : 1;  // Break
    uint8_t U : 1;  // Undefined
    uint8_t V : 1;  // Overflow
    uint8_t S : 1;  // Sign            (bit 7)
  };
#endif
} PRegT;
static PRegT P;  // Processor Status

void puce6502RST() {  // Reset
	puce6502_phantom_init();      /* : one-shot, env-driven, default OFF */
	PC = readMem(0xFFFC) | (readMem(0xFFFD) << 8);
	SP = 0xFD;
	P.I = 1;
	P.U = 1;
	ticks += 7;
}


void puce6502IRQ() {  // Interupt Request
	/* Three defects fixed 2026-08-12, all exposed by driving VERA's VSYNC IRQ: - the mask test... */
	if (P.I) return;                      // masked
	writeMem(0x100 + SP, (PC >> 8) & 0xFF);
	SP--;
	writeMem(0x100 + SP, PC & 0xFF);
	SP--;
	writeMem(0x100 + SP, P.byte & ~BREAK); // pushed with the PRE-entry I
	SP--;
	P.I = 1;
	PC = readMem(0xFFFE) | (readMem(0xFFFF) << 8);
	ticks += 7;
}


void puce6502NMI() {  // Non Maskable Interupt
	/* Same two defects the IRQ path had (fixed 2026-08-12): PC++ belongs to BRK, and P. */
	writeMem(0x100 + SP, (PC >> 8) & 0xFF);
	SP--;
	writeMem(0x100 + SP, PC & 0xFF);
	SP--;
	writeMem(0x100 + SP, P.byte & ~BREAK);
	SP--;
	P.I = 1;
	PC = readMem(0xFFFA) | (readMem(0xFFFB) << 8);
	ticks += 7;
}

void puce6502GetState(Puce6502State *s) {  // for save snapshots
	s->pc = PC; s->a = A; s->x = X; s->y = Y; s->sp = SP; s->p = P.byte;
}

void puce6502SetState(const Puce6502State *s) {  // for load snapshots
	PC = s->pc; A = s->a; X = s->x; Y = s->y; SP = s->sp; P.byte = s->p;
}


/* Addressing modes abreviations used in the comments down below : IMP : Implied or Implicit :... */

/* --- write-logger (diagnostic): record the unique PCs that write into the 32KB game
 * $4000-$4017) land on the relocated code that shares those port addresses -- this finds every
 * such code-corrupting writer. Enable g_wlog=1, then call wlog_dump(). Off = zero cost. */
#include <stdio.h>
int g_wlog = 0;
/* watched write range (default = the relocated game "ROM" image). Set g_wlog_lo/g_wlog_hi
 * to watch any other range, e.g. a single RAM cell like Block_Buffer_1+$A3. */
int g_wlog_lo = 0x0800, g_wlog_hi = 0x87FF;
/* NES $8000-$FFFF is ROM (writes are no-ops); the port relocated that code to $0800-$87FF
 * in RAM, so ANY write there corrupts code. Log every unique PC that writes the "ROM" space. */
static uint8_t  g_wlog_seen[0x10000];
static uint16_t g_wlog_tgt[0x10000];
static uint8_t  g_wlog_val[0x10000];

/* A2_RLOG: the READ half of the watched-range census wlog logs writes only. */
static uint8_t  g_rlog_seen[0x10000];
static uint16_t g_rlog_tgt[0x10000];
static unsigned long g_rlog_cnt[0x10000];
int g_rlog = 0;
static void rlog_dump(void) {
	const char *p = getenv("A2_RLOG");
	FILE *f; int pc, n = 0;
	if (!p || !*p) return;
	if (!(f = fopen(p, "w"))) return;
	fprintf(f, "# pc target count\n");
	for (pc = 0; pc < 0x10000; pc++) if (g_rlog_seen[pc]) {
		fprintf(f, "%04X %04X %lu\n", pc, g_rlog_tgt[pc], g_rlog_cnt[pc]);
		n++;
	}
	fprintf(f, "# distinct_reader_pcs %d\n", n);
	fclose(f);
}
__attribute__((constructor))
static void rlog_init(void) {
	const char *p = getenv("A2_RLOG");
	if (p && *p) { g_rlog = 1; atexit(rlog_dump); }
}
#define RLOG_NOTE(pc, ad) do { \
	if (!g_rlog_seen[(pc)]) { g_rlog_seen[(pc)] = 1; g_rlog_tgt[(pc)] = (ad); } \
	g_rlog_cnt[(pc)]++; } while (0)
static int      g_wlog_cnt[0x10000];
/*  OAM DIRTY-COVERAGE TRACKER -- DEFAULT OFF, zero cost when g_oam_track == 0. */
int g_oam_track = 0;
uint16_t g_oam_sel_lo = 0, g_oam_sel_hi = 0;
uint8_t g_oam_wr_all[64], g_oam_wr_sel[64];
uint16_t g_oam_lastpc[64];        /* : last GAME writer of each slot, for miss attribution */
/* : is the port's own blank_oam doing anything?  Count its OAM writes and how many
   actually CHANGE the byte.  If effective == 0 it is writing $F8 over $F8 every frame. */
long g_blank_wr = 0, g_blank_eff = 0;
long g_oam_selwrites = 0;   /* : OAM writes from the implant range, this frame */
static inline void oam_note(uint16_t pc, uint16_t tgt){
	int slot = (tgt - 0x0200) >> 2;
	/*  ONLY GAME-IMAGE writes can ever be marked by an implant. */
	if (pc < 0x0800 || pc > 0x87FF) return;
	g_oam_wr_all[slot] = 1; g_oam_lastpc[slot] = pc;
	if (pc >= g_oam_sel_lo && pc <= g_oam_sel_hi) { g_oam_wr_sel[slot] = 1; g_oam_selwrites++; }
}
int g_wlog_trace = 0;   /* 1 = print EVERY watched write as it happens (first-value-only
                         * summaries hide later values from the same PC) */
static inline void wlog_note(uint16_t pc, uint16_t tgt, uint8_t val){
	if(!g_wlog_seen[pc]){ g_wlog_seen[pc]=1; g_wlog_tgt[pc]=tgt; g_wlog_val[pc]=val; }
	g_wlog_cnt[pc]++;
	if(g_wlog_trace) printf("  WW PC $%04X -> $%04X = $%02X\n", pc, tgt, val);
}
void wlog_reset(void){ for(int i=0;i<0x10000;i++){ g_wlog_seen[i]=0; g_wlog_cnt[i]=0; } }
void wlog_dump(void){
	int n=0;
	for(int pc=0; pc<0x10000; pc++) if(g_wlog_seen[pc]){
		const char*w=(pc>=0x0800&&pc<=0x87FF)?"GAME":(pc>=0x9000&&pc<=0xA7FF)?"resid":"other";
		printf("  PC $%04X (%-5s orig $%04X) -> ROM $%04X (orig $%04X) val $%02X  x%d\n",
		       pc, w, (pc+0x7800)&0xFFFF, g_wlog_tgt[pc], (g_wlog_tgt[pc]+0x7800)&0xFFFF,
		       g_wlog_val[pc], g_wlog_cnt[pc]);
		n++;
	}
	printf("  %d unique PCs wrote into ROM space $0800-$87FF\n", n);
}

#if !_FUNCTIONNAL_TESTS
/* In-Exec memory access: same fast paths as the inline readMem/writeMem above, except the... */
#define readMem(a) (__extension__({ \
	uint16_t _ad = (a); uint8_t _v; \
	if (g_rlog && _ad >= g_wlog_lo && _ad <= g_wlog_hi) RLOG_NOTE(PC, _ad); \
	if (_ad < 0xC000 && !g_memslow) _v = ram[_ad]; \
	else if (_ad == 0xC000) _v = KBD; \
	else if (_ad >= 0xD000 && !LCRD) _v = rom[_ad - 0xD000]; \
	else { *gTICKS = ticks; _v = readMemSlow(_ad); } \
	_v; }))
#define writeMem(a, v) do { \
	uint16_t _ad = (a); uint8_t _vv = (v); \
	if (_ad < 0xC000 && !g_memslow) { \
		/*  THIS TEST MUST HAPPEN *BEFORE* THE STORE.  It was written after `ram[_ad] = _vv`
		   and therefore compared the byte with itself -- g_blank_eff could only ever be 0, and
		   it reported a confident, meaningless "0 of 6.8M writes are effective".  A VRAM anchor
		   diff refuted it.  Classic vacuous gate; see . */ \
		if (g_oam_track && _ad >= 0x0200 && _ad <= 0x02FF && (PC < 0x0800 || PC > 0x87FF)) { \
			g_blank_wr++; if (ram[_ad] != _vv) g_blank_eff++; } \
		ram[_ad] = _vv; fetchMem[_ad] = _vv; JIT_STORE_CHECK(_ad); \
		if (g_wlog && _ad >= g_wlog_lo && _ad <= g_wlog_hi) wlog_note(PC, _ad, _vv); \
		if (g_oam_track && _ad >= 0x0200 && _ad <= 0x02FF) oam_note(PC, _ad); } \
	else { *gTICKS = ticks; \
		/* LOG SOFT-SWITCH/CARD WRITES TOO. The watched-range test used to sit only in the `_ad <... */ \
		if (g_wlog && _ad >= g_wlog_lo && _ad <= g_wlog_hi) wlog_note(PC, _ad, _vv); \
		writeMemSlow(_ad, _vv); } \
} while (0)
#endif

/* Lazy status flags (Exec-local). The bitfield P union cost gcc a masked read-modify-write... */
#define PACK_P() ((unsigned char)((fNv & SIGN) | (fV ? OFLOW : 0) | UNDEF \
	| (fD ? DECIM : 0) | (fI ? INTR : 0) | (fZv ? 0 : ZERO) | fC))
#define UNPACK_P(v) do { unsigned char _p = (unsigned char)(v); fNv = _p; \
	fZv = (unsigned char)(~_p & ZERO); fC = (unsigned char)(_p & CARRY); \
	fV = (_p & OFLOW) != 0; fD = (_p & DECIM) != 0; fI = (_p & INTR) != 0; } while (0)

#ifdef PROF
int g_prof = 0;
unsigned int g_prof_pc_cyc[65536];
unsigned int g_prof_op_cyc[256];
unsigned int g_prof_op_cnt[256];
static unsigned int g_prof_pt;
static uint16_t g_prof_ppc;
static uint8_t  g_prof_pop;
/* attribute the just-finished instruction's cycle delta to the PC/opcode it ran at,
 * then latch the instruction about to run. Called at each dispatch. */
#define PROF_STEP(curPC,curOP,curT) do { if (g_prof) { \
    unsigned int _dc=(curT)-g_prof_pt; g_prof_pc_cyc[g_prof_ppc]+=_dc; \
    g_prof_op_cyc[g_prof_pop]+=_dc; g_prof_op_cnt[g_prof_pop]++; \
    g_prof_pt=(curT); g_prof_ppc=(curPC); g_prof_pop=(curOP); } } while(0)
#else
#define PROF_STEP(a,b,c)
#endif

/*  macros All five expand to NOTHING AT RUNTIME when g_phantom == 0 (one predictable... */
#define PH_UNCARRIED(base, idx) ((uint16_t)(((base) & 0xFF00) | (((base) + (idx)) & 0x00FF)))
#define PH_XPAGE(base, idx)     (((((base) & 0xFF) + (idx)) & 0x100) != 0)
#define PH_PC_DUMMY()           ((uint16_t)(PC - 1))
#define PH_TAKE(ctr, a)         do { (ctr)++; (void)readMem(a); } while (0)

/* ⭐⭐  CORRECTION -- THE 65816 ARM IS BUS-SILENT. */

/* STA abs,X / STA abs,Y.
 *   NMOS 6502:  real dummy read at the un-carried address.            [A, Sather]
 *   65C02:      destination when no page cross, else PC-1.            [A, Sather]
 *   65816:      NO BUS ACCESS -- internal cycle.                      [C, $9D/$99] */
#define PH_W_ABSIDX(base, idx) do { if (g_phantom && g_phantom != PH_65816) { \
		if (g_phantom == PH_65C02 && PH_XPAGE(base, idx)) PH_TAKE(g_ph_w_absidx, PH_PC_DUMMY()); \
		else PH_TAKE(g_ph_w_absidx, PH_UNCARRIED(base, idx)); } } while (0)

/* STA (zp),Y -- ⭐ THE  CASE, now REFUTED for the 65816.
 *   NMOS 6502:  real dummy read at the un-carried address.            [A, Sather]
 *   65C02:      re-addresses zp+1 instead.                            [A, Sather]
 *   65816:      NO BUS ACCESS -- internal cycle.                      [C, $91] */
#define PH_W_IZY(zp1, base) do { if (g_phantom && g_phantom != PH_65816) { \
		if (g_phantom == PH_65C02) PH_TAKE(g_ph_w_izy, (uint16_t)(uint8_t)(zp1)); \
		else PH_TAKE(g_ph_w_izy, PH_UNCARRIED(base, Y)); } } while (0)

/* LDA/LDX/LDY/ORA/AND/EOR/ADC/SBC/CMP abs,X / abs,Y -- page cross only.
 *   65816: the page-cross penalty cycle is INTERNAL, no bus access.   [C, $BD/$B9] */
#define PH_R_ABSIDX(base, idx) do { if (g_phantom && g_phantom != PH_65816 && PH_XPAGE(base, idx)) { \
		if (g_phantom == PH_65C02) PH_TAKE(g_ph_r_absidx, PH_PC_DUMMY()); \
		else PH_TAKE(g_ph_r_absidx, PH_UNCARRIED(base, idx)); } } while (0)

/* ... (zp),Y read forms -- same rule; the 65C02 address is INFERRED (zp+1).
 *   65816: internal, no bus access.                                   [C, $B1] */
#define PH_R_IZY(zp1, base) do { if (g_phantom && g_phantom != PH_65816 && PH_XPAGE(base, Y)) { \
		if (g_phantom == PH_65C02) PH_TAKE(g_ph_r_izy, (uint16_t)(uint8_t)(zp1)); \
		else PH_TAKE(g_ph_r_izy, PH_UNCARRIED(base, Y)); } } while (0)

/* Read-modify-write. NMOS 6502: writes the OLD value back before the new one. */
#define PH_RMW(addr, oldv) do { \
		if (g_phantom == PH_NMOS) { g_ph_rmw++; writeMem((addr), (oldv)); } \
		else if (g_phantom == PH_65C02) PH_TAKE(g_ph_rmw, addr); } while (0)

uint16_t puce6502Exec(unsigned int cycleCount) {
	register uint16_t address;
	register uint8_t  value8;
	register uint16_t value16;

	cycleCount += ticks;	// cycleCount becomes the targeted ticks value

	/* Registerize the CPU state for the whole batch: gcc otherwise keeps PC/A/X/Y/SP/P in MEMORY... */
	uint16_t *gPC = &PC;
	uint8_t  *gA = &A, *gX = &X, *gY = &Y, *gSP = &SP, *gP = &P.byte;
	unsigned int *gTICKS = &ticks;
	uint16_t entPC = PC;
	uint8_t  entA = A, entX = X, entY = Y, entSP = SP, entP = P.byte;
	unsigned int entTICKS = ticks;
	{
	register uint16_t PC = entPC;
	register uint8_t  A = entA, X = entX, Y = entY, SP = entSP;
	register unsigned int ticks = entTICKS;
	register unsigned char fZv, fNv, fC;  /* lazy flags: Z=(fZv==0), N=fNv bit7, C=0/1 */
	unsigned char fV, fD, fI;
	UNPACK_P(entP);

	/* wrap-safe "ticks < cycleCount": the signed difference stays small
	 * (~cycleCount per call), so it works correctly across the 32-bit wrap. */
	/* THREADED DISPATCH (computed goto): each handler ends with NEXT() --
	 * an inline tick-check + table jump.  Replaces the switch's bounds
	 * check, double zero-extend and loop back-jump (~1/3 of the per-
	 * instruction cost).  Wrap-safe tick compare as before. */
	static const void * const op_tbl[256] = {
		&&op_00, &&op_01, &&op_bad, &&op_bad, &&op_bad, &&op_05, &&op_06, &&op_bad,
		&&op_08, &&op_09, &&op_0A, &&op_bad, &&op_bad, &&op_0D, &&op_0E, &&op_bad,
		&&op_10, &&op_11, &&op_bad, &&op_bad, &&op_bad, &&op_15, &&op_16, &&op_bad,
		&&op_18, &&op_19, &&op_bad, &&op_bad, &&op_bad, &&op_1D, &&op_1E, &&op_bad,
		&&op_20, &&op_21, &&op_bad, &&op_bad, &&op_24, &&op_25, &&op_26, &&op_bad,
		&&op_28, &&op_29, &&op_2A, &&op_bad, &&op_2C, &&op_2D, &&op_2E, &&op_bad,
		&&op_30, &&op_31, &&op_bad, &&op_bad, &&op_bad, &&op_35, &&op_36, &&op_bad,
		&&op_38, &&op_39, &&op_bad, &&op_bad, &&op_bad, &&op_3D, &&op_3E, &&op_bad,
		&&op_40, &&op_41, &&op_bad, &&op_bad, &&op_bad, &&op_45, &&op_46, &&op_bad,
		&&op_48, &&op_49, &&op_4A, &&op_bad, &&op_4C, &&op_4D, &&op_4E, &&op_bad,
		&&op_50, &&op_51, &&op_bad, &&op_bad, &&op_bad, &&op_55, &&op_56, &&op_bad,
		&&op_58, &&op_59, &&op_bad, &&op_bad, &&op_bad, &&op_5D, &&op_5E, &&op_bad,
		&&op_60, &&op_61, &&op_bad, &&op_bad, &&op_bad, &&op_65, &&op_66, &&op_bad,
		&&op_68, &&op_69, &&op_6A, &&op_bad, &&op_6C, &&op_6D, &&op_6E, &&op_bad,
		&&op_70, &&op_71, &&op_bad, &&op_bad, &&op_bad, &&op_75, &&op_76, &&op_bad,
		&&op_78, &&op_79, &&op_bad, &&op_bad, &&op_bad, &&op_7D, &&op_7E, &&op_bad,
		&&op_bad, &&op_81, &&op_bad, &&op_bad, &&op_84, &&op_85, &&op_86, &&op_bad,
		&&op_88, &&op_bad, &&op_8A, &&op_bad, &&op_8C, &&op_8D, &&op_8E, &&op_bad,
		&&op_90, &&op_91, &&op_bad, &&op_bad, &&op_94, &&op_95, &&op_96, &&op_bad,
		&&op_98, &&op_99, &&op_9A, &&op_bad, &&op_bad, &&op_9D, &&op_bad, &&op_bad,
		&&op_A0, &&op_A1, &&op_A2, &&op_bad, &&op_A4, &&op_A5, &&op_A6, &&op_bad,
		&&op_A8, &&op_A9, &&op_AA, &&op_bad, &&op_AC, &&op_AD, &&op_AE, &&op_bad,
		&&op_B0, &&op_B1, &&op_bad, &&op_bad, &&op_B4, &&op_B5, &&op_B6, &&op_bad,
		&&op_B8, &&op_B9, &&op_BA, &&op_bad, &&op_BC, &&op_BD, &&op_BE, &&op_bad,
		&&op_C0, &&op_C1, &&op_bad, &&op_bad, &&op_C4, &&op_C5, &&op_C6, &&op_bad,
		&&op_C8, &&op_C9, &&op_CA, &&op_bad, &&op_CC, &&op_CD, &&op_CE, &&op_bad,
		&&op_D0, &&op_D1, &&op_bad, &&op_bad, &&op_bad, &&op_D5, &&op_D6, &&op_bad,
		&&op_D8, &&op_D9, &&op_bad, &&op_bad, &&op_bad, &&op_DD, &&op_DE, &&op_bad,
		&&op_E0, &&op_E1, &&op_bad, &&op_bad, &&op_E4, &&op_E5, &&op_E6, &&op_bad,
		&&op_E8, &&op_E9, &&op_EA, &&op_bad, &&op_EC, &&op_ED, &&op_EE, &&op_bad,
		&&op_F0, &&op_F1, &&op_bad, &&op_bad, &&op_bad, &&op_F5, &&op_F6, &&op_bad,
		&&op_F8, &&op_F9, &&op_bad, &&op_bad, &&op_bad, &&op_FD, &&op_FE, &&op_bad,
	};
#define NEXT() do { if ((int)(ticks - cycleCount) >= 0) goto exec_done; \
	{ uint8_t _nop = fetch(PC); PROF_STEP(PC, _nop, ticks); PC++; goto *op_tbl[_nop]; } } while (0)
	NEXT();

			op_00: ;  // IMP BRK
				PC++;
				writeMem(0x100 + SP, ((PC) >> 8) & 0xFF);
				SP--;
				writeMem(0x100 + SP, PC & 0xFF);
				SP--;
				writeMem(0x100 + SP, PACK_P() | BREAK);
				SP--;
				fI = (unsigned char)(1);
				fD = (unsigned char)(0);
				PC = readMem(0xFFFE) | (readMem(0xFFFF) << 8);
				ticks += 7;
			NEXT();

			op_01: ;  // IZX ORA
				value8 = fetch(PC) + X;
				PC++;
				address = readMem(value8);
				value8++;
				address |= readMem(value8) << 8;
				A |= readMem(address);
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 6;
			NEXT();

			op_05: ;  // ZPG ORA
				A |= readMem(fetch(PC));
				PC++;
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 3;
			NEXT();

			op_06: ;  // ZPG ASL
				address = fetch(PC);
				PC++;
				value16 = readMem(address) << 1;
				fC = (unsigned char)(value16 > 0xFF);
				value16 &= 0xFF;
				writeMem(address, value16);
				fZv = (unsigned char)(value16);
				fNv = (unsigned char)(value16);
				ticks += 5;
			NEXT();

			op_08: ;  // IMP PHP
				writeMem(0x100 + SP, PACK_P() | BREAK);
				SP--;
				ticks += 3;
			NEXT();

			op_09: ;  // IMM ORA
				A |= fetch(PC);
				PC++;
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 2;
			NEXT();

			op_0A: ;  // ACC ASL
				value16 = A << 1;
				fC = (unsigned char)(value16 > 0xFF);
				A = value16 & 0xFF;
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 2;
			NEXT();

			op_0D: ;  // ABS ORA
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				A |= readMem(address);
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 4;
			NEXT();

			op_0E: ;  // ABS ASL
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				value16 = readMem(address);
				PH_RMW(address, (uint8_t)value16);
				value16 <<= 1;
				fC = (unsigned char)(value16 > 0xFF);
				value16 &= 0xFF;
				writeMem(address, value16);
				fZv = (unsigned char)(value16);
				fNv = (unsigned char)(value16);
				ticks += 6;
			NEXT();

			op_10: ;  // REL BPL
				address = fetch(PC);
				PC++;
				if (!((fNv & SIGN) != 0)) {  // jump taken
					ticks++;
					if (address & SIGN)
						address |= 0xFF00;  // jump backward
					if (((PC & 0xFF) + address) & 0xFF00)  // page crossing
						ticks++;
					PC += address;
				}
				ticks += 2;
			NEXT();

			op_11: ;  // IZY ORA
				value8 = fetch(PC);
				PC++;
				address = readMem(value8);
				value8++;
				address |= readMem(value8) << 8;
				ticks += (((address & 0xFF) + Y) & 0xFF00) ? 6 : 5;  // page crossing
				PH_R_IZY(value8, address);
				address += Y;
				A |= readMem(address);
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
			NEXT();

			op_15: ;  // ZPX ORA
				A |= readMem((fetch(PC) + X) & 0xFF);
				PC++;
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 4;
			NEXT();

			op_16: ;  // ZPX ASL
				address = (fetch(PC) + X) & 0xFF;
				PC++;
				value16 = readMem(address) << 1;
				writeMem(address, value16 & 0xFF);
				fC = (unsigned char)(value16 > 0xFF);
				fZv = (unsigned char)(value16);
				fNv = (unsigned char)((value16 & 0xFF));
				ticks += 6;
			NEXT();

			op_18: ;  // IMP CLC
				fC = (unsigned char)(0);
				ticks += 2;
			NEXT();

			op_19: ;  // ABY ORA
				address = fetch(PC);
				PC++;
				ticks += ((address + Y) & 0xFF00) ? 5 : 4;  // page crossing
				address |= fetch(PC) << 8;
				PC++;
				PH_R_ABSIDX(address, Y);
				address += Y;
				A |= readMem(address);
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
			NEXT();

			op_1D: ;  // ABX ORA
				address = fetch(PC);
				PC++;
				ticks += ((address + X) & 0xFF00) ? 5 : 4;  // page crossing
				address |= fetch(PC) << 8;
				PC++;
				PH_R_ABSIDX(address, X);
				address += X;
				A |= readMem(address);
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
			NEXT();

			op_1E: ;  // ABX ASL
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				PH_W_ABSIDX(address, X);
				address += X;
				value16 = readMem(address);
				PH_RMW(address, (uint8_t)value16);
				value16 <<= 1;
				fC = (unsigned char)(value16 > 0xFF);
				value16 &= 0xFF;
				writeMem(address, value16);
				fZv = (unsigned char)(value16);
				fNv = (unsigned char)(value16);
				ticks += 7;
			NEXT();

			op_20: ;  // ABS JSR
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				writeMem(0x100 + SP, (PC >> 8) & 0xFF);
				SP--;
				writeMem(0x100 + SP, PC & 0xFF);
				SP--;
				PC = address;
				ticks += 6;
			NEXT();

			op_21: ;  // IZX AND
				value8 = fetch(PC) + X;
				PC++;
				address = readMem(value8);
				value8++;
				address |= readMem(value8) << 8;
				A &= readMem(address);
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 6;
			NEXT();

			op_24: ;  // ZPG BIT
				address = fetch(PC);
				PC++;
				value8 = readMem(address);
				fZv = (unsigned char)((A & value8));
				fNv = value8; fV = (value8 & OFLOW) != 0;  /* BIT: N,V from operand */
				ticks += 3;
			NEXT();

			op_25: ;  // ZPG AND
				A &= readMem(fetch(PC));
				PC++;
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 3;
			NEXT();

			op_26: ;  // ZPG ROL
				address = fetch(PC);
				PC++;
				value16 = (readMem(address) << 1) | fC;
				fC = (unsigned char)((value16 & 0x100) != 0);
				value16 &= 0xFF;
				writeMem(address, value16);
				fZv = (unsigned char)(value16);
				fNv = (unsigned char)(value16);
				ticks += 5;
			NEXT();

			op_28: ;  // IMP PLP
				SP++;
				UNPACK_P(readMem(0x100 + SP));
				ticks += 4;
			NEXT();

			op_29: ;  // IMM AND
				A &= fetch(PC);
				PC++;
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 2;
			NEXT();

			op_2A: ;  // ACC ROL
				value16 = (A << 1) | fC;
				fC = (unsigned char)((value16 & 0x100) != 0);
				A = value16 & 0xFF;
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 2;
			NEXT();

			op_2C: ;  // ABS BIT
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				value8 = readMem(address);
				fZv = (unsigned char)((A & value8));
				fNv = value8; fV = (value8 & OFLOW) != 0;  /* BIT: N,V from operand */
				ticks += 4;
			NEXT();

			op_2D: ;  // ABS AND
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				A &= readMem(address);
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 4;
			NEXT();

			op_2E: ;  // ABS ROL
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				value16 = readMem(address);
				PH_RMW(address, (uint8_t)value16);
				value16 = (value16 << 1) | fC;
				fC = (unsigned char)((value16 & 0x100) != 0);
				value16 &= 0xFF;
				writeMem(address, value16);
				fZv = (unsigned char)(value16);
				fNv = (unsigned char)(value16);
				ticks += 6;
			NEXT();

			op_30: ;  // REL BMI
				address = fetch(PC);
				PC++;
				if (((fNv & SIGN) != 0)) {  // branch taken
					ticks++;
					if (address & SIGN)
						address |= 0xFF00;  // jump backward
					if (((PC & 0xFF) + address) & 0xFF00)  // page crossing
						ticks++;
					PC += address;
				}
				ticks += 2;
			NEXT();

			op_31: ;  // IZY AND
				value8 = fetch(PC);
				PC++;
				address = readMem(value8);
				value8++;
				address |= readMem(value8) << 8;
				ticks += (((address & 0xFF) + Y) & 0xFF00) ? 6 : 5;  // page crossing
				PH_R_IZY(value8, address);
				address += Y;
				A &= readMem(address);
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
			NEXT();

			op_35: ;  // ZPX AND
				address = (fetch(PC) + X) & 0xFF;
				PC++;
				A &= readMem(address);
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 4;
			NEXT();

			op_36: ;  // ZPX ROL
				address = (fetch(PC) + X) & 0xFF;
				PC++;
				value16 = (readMem(address) << 1) | fC;
				fC = (unsigned char)(value16 > 0xFF);
				value16 &= 0xFF;
				writeMem(address, value16);
				fZv = (unsigned char)(value16);
				fNv = (unsigned char)(value16);
				ticks += 6;
			NEXT();

			op_38: ;  // IMP SEC
				fC = (unsigned char)(1);
				ticks += 2;
			NEXT();

			op_39: ;  // ABY AND
				address = fetch(PC);
				PC++;
				ticks += ((address + Y) & 0xFF00) ? 5 : 4;  // page crossing
				address |= fetch(PC) << 8;
				PC++;
				PH_R_ABSIDX(address, Y);
				address += Y;
				A &= readMem(address);
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
			NEXT();

			op_3D: ;  // ABX AND
				address = fetch(PC);
				PC++;
				ticks += ((address + X) & 0xFF00) ? 5 : 4;  // page crossing
				address |= fetch(PC) << 8;
				PC++;
				PH_R_ABSIDX(address, X);
				address += X;
				A &= readMem(address);
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
			NEXT();

			op_3E: ;  // ABX ROL
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				PH_W_ABSIDX(address, X);
				address += X;
				value16 = readMem(address);
				PH_RMW(address, (uint8_t)value16);
				value16 = (value16 << 1) | fC;
				fC = (unsigned char)(value16 > 0xFF);
				value16 &= 0xFF;
				writeMem(address, value16);
				fZv = (unsigned char)(value16);
				fNv = (unsigned char)(value16);
				ticks += 7;
			NEXT();

			op_40: ;  // IMP RTI
				SP++;
				UNPACK_P(readMem(0x100 + SP));
				SP++;
				PC = readMem(0x100 + SP);
				SP++;
				PC |= readMem(0x100 + SP) << 8;
				ticks += 6;
			NEXT();

			op_41: ;  // IZX EOR
				value8 = fetch(PC) + X;
				PC++;
				address = readMem(value8);
				value8++;
				address |= readMem(value8) << 8;
				A ^= readMem(address);
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 6;
			NEXT();

			op_45: ;  // ZPG EOR
				address = fetch(PC);
				PC++;
				A ^= readMem(address);
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 3;
			NEXT();

			op_46: ;  // ZPG LSR
				address = fetch(PC);
				PC++;
				value8 = readMem(address);
				fC = (unsigned char)((value8 & 1) != 0);
				value8 = value8 >> 1;
				writeMem(address, value8);
				fZv = (unsigned char)(value8);
				fNv = (unsigned char)(value8);
				ticks += 5;
			NEXT();

			op_48: ;  // IMP PHA
				writeMem(0x100 + SP, A);
				SP--;
				ticks += 3;
			NEXT();

			op_49: ;  // IMM EOR
				A ^= fetch(PC);
				PC++;
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 2;
			NEXT();

			op_4A: ;  // ACC LSR
				fC = (unsigned char)((A & 1) != 0);
				A = A >> 1;
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 2;
			NEXT();

			op_4C: ;  // ABS JMP
				PC = fetch(PC) | (fetch(PC + 1) << 8);
				ticks += 3;
			NEXT();

			op_4D: ;  // ABS EOR
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				A ^= readMem(address);
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 4;
			NEXT();

			op_4E: ;  // ABS LSR
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				value8 = readMem(address);
				PH_RMW(address, value8);
				fC = (unsigned char)((value8 & 1) != 0);
				value8 = value8 >> 1;
				writeMem(address, value8);
				fZv = (unsigned char)(value8);
				fNv = (unsigned char)(value8);
				ticks += 6;
			NEXT();

			op_50: ;  // REL BVC
				address = fetch(PC);
				PC++;
				if (!fV) {  // branch taken
					ticks++;
					if (address & SIGN)
						address |= 0xFF00;  // jump backward
					if (((PC & 0xFF) + address) & 0xFF00)  // page crossing
						ticks++;
					PC += address;
				}
				ticks += 2;
			NEXT();

			op_51: ;  // IZY EOR
				value8 = fetch(PC);
				PC++;
				address = readMem(value8);
				value8++;
				address |= readMem(value8) << 8;
				ticks += (((address & 0xFF) + Y) & 0xFF00) ? 6 : 5;  // page crossing
				PH_R_IZY(value8, address);
				A ^= readMem(address + Y);
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
			NEXT();

			op_55: ;  // ZPX EOR
				address = (fetch(PC) + X) & 0xFF;
				PC++;
				A ^= readMem(address);
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 4;
			NEXT();

			op_56: ;  // ZPX LSR
				address = (fetch(PC) + X) & 0xFF;
				PC++;
				value8 = readMem(address);
				fC = (unsigned char)((value8 & 1) != 0);
				value8 = value8 >> 1;
				writeMem(address, value8);
				fZv = (unsigned char)(value8);
				fNv = (unsigned char)(value8);
				ticks += 6;
			NEXT();

      op_58: ;  // IMP CLI
        fI = (unsigned char)(0);
        ticks += 2;
      NEXT();

			op_59: ;  // ABY EOR
				address = fetch(PC);
				PC++;
				ticks += ((address + Y) & 0xFF00) ? 5 : 4;  // page crossing
				address |= fetch(PC) << 8;
				PC++;
				PH_R_ABSIDX(address, Y);
				address += Y;
				A ^= readMem(address);
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
			NEXT();

			op_5D: ;  // ABX EOR
				address = fetch(PC);
				PC++;
				ticks += ((address + X) & 0xFF00) ? 5 : 4;  // page crossing
				address |= fetch(PC) << 8;
				PC++;
				PH_R_ABSIDX(address, X);
				address += X;
				A ^= readMem(address);
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
			NEXT();

			op_5E: ;  // ABX LSR
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				PH_W_ABSIDX(address, X);
				address += X;
				value8 = readMem(address);
				PH_RMW(address, value8);
				fC = (unsigned char)((value8 & 1) != 0);
				value8 = value8 >> 1;
				writeMem(address, value8);
				fZv = (unsigned char)(value8);
				fNv = (unsigned char)(value8);
				ticks += 7;
			NEXT();

			op_60: ;  // IMP RTS
				SP++;
				PC = readMem(0x100 + SP);
				SP++;
				PC |= readMem(0x100 + SP) << 8;
				PC++;
				ticks += 6;
			NEXT();

			op_61: ;  // IZX ADC
				value8 = fetch(PC) + X;
				PC++;
				address = readMem(value8);
				value8++;
				address |= readMem(value8) << 8;
				value8 = readMem(address);
				value16 = A + value8 + fC;
				fV = (unsigned char)(((value16 ^ A) & (value16 ^ value8) & 0x0080) != 0);
				if (fD)
					value16 += ((((value16 + 0x66) ^ A ^ value8) >> 3) & 0x22) * 3;
				fC = (unsigned char)(value16 > 0xFF);
				A = value16 & 0xFF;
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 6;
			NEXT();

			op_65: ;  // ZPG ADC
				address = fetch(PC);
				PC++;
				value8 = readMem(address);
				value16 = A + value8 + fC;
				fV = (unsigned char)(((value16 ^ A) & (value16 ^ value8) & 0x0080) != 0);
				if (fD)
					value16 += ((((value16 + 0x66) ^ A ^ value8) >> 3) & 0x22) * 3;
				fC = (unsigned char)(value16 > 0xFF);
				A = value16 & 0xFF;
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 3;
			NEXT();

			op_66: ;  // ZPG ROR
				address = fetch(PC);
				PC++;
				value8 = readMem(address);
				value16 = (value8 >> 1) | (fC << 7);
				fC = (unsigned char)((value8 & 0x1) != 0);
				value16 &= 0xFF;
				writeMem(address, value16);
				fZv = (unsigned char)(value16);
				fNv = (unsigned char)(value16);
				ticks += 5;
			NEXT();

			op_68: ;  // IMP PLA
				SP++;
				A = readMem(0x100 + SP);
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 4;
			NEXT();

			op_69: ;  // IMM ADC
				value8 = fetch(PC);
				PC++;
				value16 = A + value8 + fC;
				fV = (unsigned char)(((value16 ^ A) & (value16 ^ value8) & 0x0080) != 0);
				if (fD)
					value16 += ((((value16 + 0x66) ^ A ^ value8) >> 3) & 0x22) * 3;
				fC = (unsigned char)(value16 > 0xFF);
				A = value16 & 0xFF;
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 2;
			NEXT();

			op_6A: ;  // ACC ROR
				value16 = (A >> 1) | (fC << 7);
				fC = (unsigned char)((A & 0x1) != 0);
				A = value16 & 0xFF;
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 2;
			NEXT();

			op_6C: ;  // IND JMP
				address = fetch(PC) | fetch(PC + 1) << 8;
				PC = readMem(address) | (readMem(address + 1) << 8);
				ticks += 5;
			NEXT();

			op_6D: ;  // ABS ADC
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				value8 = readMem(address);
				value16 = A + value8 + fC;
				fV = (unsigned char)(((value16 ^ A) & (value16 ^ value8) & 0x0080) != 0);
				if (fD)
					value16 += ((((value16 + 0x66) ^ A ^ value8) >> 3) & 0x22) * 3;
				fC = (unsigned char)(value16 > 0xFF);
				A = value16 & 0xFF;
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 4;
			NEXT();

			op_6E: ;  // ABS ROR
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				value8 = readMem(address);
				PH_RMW(address, value8);
				value16 = (value8 >> 1) | (fC << 7);
				fC = (unsigned char)((value8 & 0x1) != 0);
				value16 = value16 & 0xFF;
				writeMem(address, value16);
				fZv = (unsigned char)(value16);
				fNv = (unsigned char)(value16);
				ticks += 6;
			NEXT();

			op_70: ;  // REL BVS
				address = fetch(PC);
				PC++;
				if (fV) {  // branch taken
					ticks++;
					if (((PC & 0xFF) + address) & 0xFF00)  // page crossing
						ticks++;
					if (address & SIGN)
						address |= 0xFF00;  // jump backward
					PC += address;
				}
				ticks += 2;
			NEXT();

			op_71: ;  // IZY ADC
				value8 = fetch(PC);
				PC++;
				address = readMem(value8);
				if ((address + Y) & 0xFF00)  // page crossing
					ticks++;
				value8++;
				address |= readMem(value8) << 8;
				PH_R_IZY(value8, address);
				address += Y;
				value8 = readMem(address);
				value16 = A + value8 + fC;
				fV = (unsigned char)(((value16 ^ A) & (value16 ^ value8) & 0x0080) != 0);
				if (fD)
					value16 += ((((value16 + 0x66) ^ A ^ value8) >> 3) & 0x22) * 3;
				fC = (unsigned char)(value16 > 0xFF);
				A = value16 & 0xFF;
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 5;
			NEXT();

			op_75: ;  // ZPX ADC
				address = (fetch(PC) + X) & 0xFF;
				PC++;
				value8 = readMem(address);
				value16 = A + value8 + fC;
				fV = (unsigned char)(((value16 ^ A) & (value16 ^ value8) & 0x0080) != 0);
				if (fD)
					value16 += ((((value16 + 0x66) ^ A ^ value8) >> 3) & 0x22) * 3;
				fC = (unsigned char)(value16 > 0xFF);
				A = value16 & 0xFF;
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 4;
			NEXT();

			op_76: ;  // ZPX ROR
				address = (fetch(PC) + X) & 0xFF;
				PC++;
				value8 = readMem(address);
				value16 = (value8 >> 1) | (fC << 7);
				fC = (unsigned char)((value8 & 0x1) != 0);
				value16 = value16 & 0xFF;
				writeMem(address, value16);
				fZv = (unsigned char)(value16);
				fNv = (unsigned char)(value16);
				ticks += 6;
			NEXT();

      op_78: ;  // IMP SEI
        fI = (unsigned char)(1);
        ticks += 2;
      NEXT();

			op_79: ;  // ABY ADC
				if ((fetch(PC) + Y) & 0xFF00)
					ticks++;
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				PH_R_ABSIDX(address, Y);
				address += Y;
				value8 = readMem(address);
				value16 = A + value8 + fC;
				fV = (unsigned char)(((value16 ^ A) & (value16 ^ value8) & 0x0080) != 0);
				if (fD)
					value16 += ((((value16 + 0x66) ^ A ^ value8) >> 3) & 0x22) * 3;
				fC = (unsigned char)(value16 > 0xFF);
				A = value16 & 0xFF;
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 4;
			NEXT();

			op_7D: ;  // ABX ADC
				if ((fetch(PC) + X) & 0xFF00)
					ticks++;
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				PH_R_ABSIDX(address, X);
				address += X;
				value8 = readMem(address);
				value16 = A + value8 + fC;
				fV = (unsigned char)(((value16 ^ A) & (value16 ^ value8) & 0x0080) != 0);
				if (fD)
					value16 += ((((value16 + 0x66) ^ A ^ value8) >> 3) & 0x22) * 3;
				fC = (unsigned char)(value16 > 0xFF);
				A = value16 & 0xFF;
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 4;
			NEXT();

			op_7E: ;  // ABX ROR
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				PH_W_ABSIDX(address, X);
				address += X;
				value8 = readMem(address);
				PH_RMW(address, value8);
				value16 = (value8 >> 1) | (fC << 7);
				fC = (unsigned char)((value8 & 0x1) != 0);                          // TBR
				value16 = value16 & 0xFF;
				writeMem(address, value16);
				fZv = (unsigned char)(value16);
				fNv = (unsigned char)(value16);
				ticks += 7;
			NEXT();

			op_81: ;  // IZX STA
				value8 = fetch(PC) + X;
				PC++;
				address = readMem(value8);
				value8++;
				address |= readMem(value8) << 8;
				writeMem(address, A);
				ticks += 6;
			NEXT();

			op_84: ;  // ZPG STY
				writeMem(fetch(PC), Y);
				PC++;
				ticks += 3;
			NEXT();

			op_85: ;  // ZPG STA
				writeMem(fetch(PC), A);
				PC++;
				ticks += 3;
			NEXT();

			op_86: ;  // ZPG STX
				writeMem(fetch(PC), X);
				PC++;
				ticks += 3;
			NEXT();

			op_88: ;  // IMP DEY
				Y--;
				fZv = (unsigned char)((Y & 0xFF));
				fNv = (unsigned char)((Y) & SIGN);
				ticks += 2;
			NEXT();

			op_8A: ;  // IMP TXA
				A = X;
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 2;
			NEXT();

			op_8C: ;  // ABS STY
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				writeMem(address, Y);
				ticks += 4;
			NEXT();

			op_8D: ;  // ABS STA
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				writeMem(address, A);
				ticks += 4;
			NEXT();

			op_8E: ;  // ABS STX
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				writeMem(address, X);
				ticks += 4;
			NEXT();

			op_90: ;  // REL BCC
				address = fetch(PC);
				PC++;
				if (!fC) {  // branch taken
					ticks++;
					if (((PC & 0xFF) + address) & 0xFF00)  // page crossing
						ticks++;
					if (address & SIGN)
						address |= 0xFF00;  // jump backward
					PC += address;
				}
				ticks += 2;
			NEXT();

			op_91: ;  // IZY STA
				value8 = fetch(PC);
				PC++;
				address = readMem(value8);
				value8++;
				address |= readMem(value8) << 8;
				PH_W_IZY(value8, address);
				address += Y;
				writeMem(address, A);
				ticks += 6;
			NEXT();

			op_94: ;  // ZPX STY
				address = (fetch(PC) + X) & 0xFF;
				PC++;
				writeMem(address, Y);
				ticks += 4;
			NEXT();

			op_95: ;  // ZPX STA
				writeMem((fetch(PC) + X) & 0xFF, A);
				PC++;
				ticks += 4;
			NEXT();

			op_96: ;  // ZPY STX
				writeMem((fetch(PC) + Y) & 0xFF, X);
				PC++;
				ticks += 4;
			NEXT();

			op_98: ;  // IMP TYA
				A = Y;
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 2;
			NEXT();

			op_99: ;  // ABY STA
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				PH_W_ABSIDX(address, Y);
				address += Y;
				writeMem(address, A);
				ticks += 5;
			NEXT();

			op_9A: ;  // IMP TXS
				SP = X;
				ticks += 2;
			NEXT();

			op_9D: ;  // ABX STA
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				PH_W_ABSIDX(address, X);
				address += X;
				writeMem(address, A);
				ticks += 5;
			NEXT();

			op_A0: ;  // IMM LDY
				Y = fetch(PC);
				PC++;
				fZv = (unsigned char)(Y);
				fNv = (unsigned char)(Y);
				ticks += 2;
			NEXT();

			op_A1: ;  // IZX LDA
				value8 = fetch(PC) + X;
				PC++;
				address = readMem(value8);
				value8++;
				address |= readMem(value8) << 8;
				A = readMem(address);
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 6;
			NEXT();

			op_A2: ;  // IMM LDX
				address = PC;
				PC++;
				X = readMem(address);
				fZv = (unsigned char)(X);
				fNv = (unsigned char)(X);
				ticks += 2;
			NEXT();

			op_A4: ;  // ZPG LDY
				Y = readMem(fetch(PC));
				PC++;
				fZv = (unsigned char)(Y);
				fNv = (unsigned char)(Y);
				ticks += 3;
			NEXT();

			op_A5: ;  // ZPG LDA
				A = readMem(fetch(PC));
				PC++;
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 3;
			NEXT();

			op_A6: ;  // ZPG LDX
				X = readMem(fetch(PC));
				PC++;
				fZv = (unsigned char)(X);
				fNv = (unsigned char)(X);
				ticks += 3;
			NEXT();

			op_A8: ;  // IMP TAY
				Y = A;
				fZv = (unsigned char)(Y);
				fNv = (unsigned char)(Y);
				ticks += 2;
			NEXT();

			op_A9: ;  // IMM LDA
				A = fetch(PC);
				PC++;
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 2;
			NEXT();

			op_AA: ;  // IMP TAX
				X = A;
				fZv = (unsigned char)(X);
				fNv = (unsigned char)(X);
				ticks += 2;
			NEXT();

			op_AC: ;  // ABS LDY
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				Y = readMem(address);
				fZv = (unsigned char)(Y);
				fNv = (unsigned char)(Y);
				ticks += 4;
			NEXT();

			op_AD: ;  // ABS LDA
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				A = readMem(address);
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 4;
			NEXT();

			op_AE: ;  // ABS LDX
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				X = readMem(address);
				fZv = (unsigned char)(X);
				fNv = (unsigned char)(X);
				ticks += 4;
			NEXT();

			op_B0: ;  // REL BCS
				address = fetch(PC);
				PC++;
				if (fC) {  // branch taken
					ticks++;
					if (address & SIGN)
						address |= 0xFF00;  // jump backward
					if (((PC & 0xFF) + address) & 0xFF00)  // page crossing
						ticks++;
					PC += address;
				}
				ticks += 2;
			NEXT();

			op_B1: ;  // IZY LDA
				value8 = fetch(PC);
				PC++;
				address = readMem(value8);
				value8++;
				address |= readMem(value8) << 8;
				PH_R_IZY(value8, address);
				A = readMem(address + Y);
				ticks += (((address & 0xFF) + Y) & 0xFF00) ? 6 : 5;  // page crossing
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
			NEXT();

			op_B4: ;  // ZPX LDY
				address = (fetch(PC) + X) & 0xFF;
				PC++;
				Y = readMem(address);
				fZv = (unsigned char)(Y);
				fNv = (unsigned char)(Y);
				ticks += 4;
			NEXT();

			op_B5: ;  // ZPX LDA
				address = (fetch(PC) + X) & 0xFF;
				PC++;
				A = readMem(address);
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 4;
			NEXT();

			op_B6: ;  // ZPY LDX
				address = (fetch(PC) + Y) & 0xFF;
				PC++;
				X = readMem(address);
				fZv = (unsigned char)(X);
				fNv = (unsigned char)(X);
				ticks += 4;
			NEXT();

      op_B8: ;  // IMP CLV
        fV = (unsigned char)(0);
        ticks += 2;
      NEXT();

			op_B9: ;  // ABY LDA
				address = fetch(PC);
				PC++;
				ticks += ((address + Y) & 0xFF00) ? 5 : 4;  // page crossing
				address |= fetch(PC) << 8;
				PC++;
				PH_R_ABSIDX(address, Y);
				address += Y;
				A = readMem(address);
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
			NEXT();

			op_BA: ;  // IMP TSX
				X = SP;
				fZv = (unsigned char)(X);
				fNv = (unsigned char)(X);
				ticks += 2;
			NEXT();

			op_BC: ;  // ABX LDY
				address = fetch(PC);
				PC++;
				ticks += ((address + X) & 0xFF00) ? 5 : 4;  // page crossing
				address |= fetch(PC) << 8;
				PC++;
				PH_R_ABSIDX(address, X);
				address += X;
				Y = readMem(address);
				fZv = (unsigned char)(Y);
				fNv = (unsigned char)(Y);
			NEXT();

			op_BD: ;  // ABX LDA
				address = fetch(PC);
				PC++;
				ticks += ((address + X) & 0xFF00) ? 5 : 4;  // page crossing
				address |= fetch(PC) << 8;
				PC++;
				PH_R_ABSIDX(address, X);
				address += X;
				A = readMem(address);
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
			NEXT();

			op_BE: ;  // ABY LDX
				address = fetch(PC);
				PC++;
				ticks += ((address + Y) & 0xFF00) ? 5 : 4;  // page crossing
				address |= fetch(PC) << 8;
				PC++;
				PH_R_ABSIDX(address, Y);
				address += Y;
				X = readMem(address);
				fZv = (unsigned char)(X);
				fNv = (unsigned char)(X);
			NEXT();

			op_C0: ;  // IMM CPY
				value8 = fetch(PC);
				PC++;
				fZv = (unsigned char)(((Y - value8) & 0xFF));
				fNv = (unsigned char)(((Y - value8)) & SIGN);
				fC = (unsigned char)((Y >= value8) != 0);
				ticks += 2;
			NEXT();

			op_C1: ;  // IZX CMP
				value8 = fetch(PC) + X;
				PC++;
				address = readMem(value8);
				value8++;
				address |= readMem(value8) << 8;
				value8 = readMem(address);
				fZv = (unsigned char)(((A - value8) & 0xFF));
				fNv = (unsigned char)(((A - value8)) & SIGN);
				fC = (unsigned char)((A >= value8) != 0);
				ticks += 6;
			NEXT();

			op_C4: ;  // ZPG CPY
				value8 = readMem(fetch(PC));
				PC++;
				fZv = (unsigned char)(((Y - value8) & 0xFF));
				fNv = (unsigned char)(((Y - value8)) & SIGN);
				fC = (unsigned char)((Y >= value8) != 0);
				ticks += 3;
			NEXT();

			op_C5: ;  // ZPG CMP
				value8 = readMem(fetch(PC));
				PC++;
				fZv = (unsigned char)(((A - value8) & 0xFF));
				fNv = (unsigned char)(((A - value8)) & SIGN);
				fC = (unsigned char)((A >= value8) != 0);
				ticks += 3;
			NEXT();

			op_C6: ;  // ZPG DEC
				address = fetch(PC);
				PC++;
				value8 = readMem(address);
				--value8;
				writeMem(address, value8);
				fZv = (unsigned char)(value8);
				fNv = (unsigned char)(value8);
				ticks += 5;
			NEXT();

			op_C8: ;  // IMP INY
				Y++;
				fZv = (unsigned char)(Y );
				fNv = (unsigned char)(Y);
				ticks += 2;
			NEXT();

			op_C9: ;  // IMM CMP
				value8 = fetch(PC);
				PC++;
				fZv = (unsigned char)(((A - value8) & 0xFF));
				fNv = (unsigned char)(((A - value8)) & SIGN);
				fC = (unsigned char)((A >= value8) != 0);
				ticks += 2;
			NEXT();

			op_CA: ;  // IMP DEX
			  X--;
				fZv = (unsigned char)((X & 0xFF));
				fNv = (unsigned char)(X);
				ticks += 2;
			NEXT();

			op_CC: ;  // ABS CPY
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				value8 = readMem(address);
				fZv = (unsigned char)(((Y - value8) & 0xFF));
				fNv = (unsigned char)(((Y - value8)) & SIGN);
				fC = (unsigned char)((Y >= value8) != 0);
				ticks += 4;
			NEXT();

			op_CD: ;  // ABS CMP
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				value8 = readMem(address);
				fZv = (unsigned char)(((A - value8) & 0xFF));
				fNv = (unsigned char)(((A - value8)) & SIGN);
				fC = (unsigned char)((A >= value8) != 0);
				ticks += 4;
			NEXT();

			op_CE: ;  // ABS DEC
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				value8 = readMem(address);
				PH_RMW(address, value8);
				value8--;
				writeMem(address, value8);
				fZv = (unsigned char)(value8);
				fNv = (unsigned char)(value8);
				ticks += 6;
			NEXT();

			op_D0: ;  // REL BNE
				address = fetch(PC);
				PC++;
				if (!(fZv == 0)) {  // branch taken
					ticks++;
					if (address & SIGN)
						address |= 0xFF00;  // jump backward
					if (((PC & 0xFF) + address) & 0xFF00)  // page crossing
						ticks++;
					PC += address;
				}
				ticks += 2;
			NEXT();

			op_D1: ;  // IZY CMP
				value8 = fetch(PC);
				PC++;
				address = readMem(value8);
				ticks += ((address + Y) & 0xFF00) ? 6 : 5;  // page crossing
				value8++;
				address |= readMem(value8) << 8;
				PH_R_IZY(value8, address);
				address += Y;
				value8 = readMem(address);
				fZv = (unsigned char)(((A - value8) & 0xFF));
				fNv = (unsigned char)(((A - value8)) & SIGN);
				fC = (unsigned char)((A >= value8) != 0);
			NEXT();

			op_D5: ;  // ZPX CMP
				address = (fetch(PC) + X) & 0xFF;
				PC++;
				value8 = readMem(address);
				fZv = (unsigned char)(((A - value8) & 0xFF));
				fNv = (unsigned char)(((A - value8)) & SIGN);
				fC = (unsigned char)((A >= value8) != 0);
				ticks += 4;
			NEXT();

			op_D6: ;  // ZPX DEC
				address = (fetch(PC) + X) & 0xFF;
				PC++;
				value8 = readMem(address);
				value8--;
				writeMem(address, value8);
				fZv = (unsigned char)(value8);
				fNv = (unsigned char)(value8);
				ticks += 6;
			NEXT();

			op_D8: ;  // IMP CLD
				fD = (unsigned char)(0);
				ticks += 2;
			NEXT();

			op_D9: ;  // ABY CMP
				address = fetch(PC);
				PC++;
				ticks += ((address + Y) & 0xFF00) ? 5 : 4;  // page crossing
				address |= fetch(PC) << 8;
				PC++;
				PH_R_ABSIDX(address, Y);
				address += Y;
				value8 = readMem(address);
				fZv = (unsigned char)(((A - value8) & 0xFF));
				fNv = (unsigned char)(((A - value8)) & SIGN);
				fC = (unsigned char)((A >= value8) != 0);
			NEXT();

			op_DD: ;  // ABX CMP
				address = fetch(PC);
				PC++;
				ticks += ((address + X) & 0xFF00) ? 5 : 4;  // page crossing
				address |= fetch(PC) << 8;
				PC++;
				PH_R_ABSIDX(address, X);
				address += X;
				value8 = readMem(address);
				fZv = (unsigned char)(((A - value8) & 0xFF));
				fNv = (unsigned char)(((A - value8)) & SIGN);
				fC = (unsigned char)((A >= value8) != 0);
			NEXT();

			op_DE: ;  // ABX DEC
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				PH_W_ABSIDX(address, X);
				address += X;
				value8 = readMem(address);
				PH_RMW(address, value8);
				value8--;
				writeMem(address, value8);
				fZv = (unsigned char)(value8);
				fNv = (unsigned char)((value8) & SIGN);
				ticks += 7;
			NEXT();

			op_E0: ;  // IMM CPX
				value8 = fetch(PC);
				PC++;
				fZv = (unsigned char)(((X - value8) & 0xFF));
				fNv = (unsigned char)(((X - value8)) & SIGN);
				fC = (unsigned char)((X >= value8) != 0);
				ticks += 2;
			NEXT();

			op_E1: ;  // IZX SBC
				value8 = fetch(PC) + X;
				PC++;
				address = readMem(value8);
				value8++;
				address |= readMem(value8) << 8;
				value8 = readMem(address);
				value8 ^= 0xFF;
				if (fD)
					value8 -= 0x0066;
				value16 = A + value8 + fC;
				fV = (unsigned char)(((value16 ^ A) & (value16 ^ value8) & 0x0080) != 0);
				if (fD)
					value16 += ((((value16 + 0x66) ^ A ^ value8) >> 3) & 0x22) * 3;
				fC = (unsigned char)(value16 > 0xFF);
				A = value16 & 0xFF;
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 6;
			NEXT();

			op_E4: ;  // ZPG CPX
				value8 = readMem(fetch(PC));
				PC++;
				fZv = (unsigned char)(((X - value8) & 0xFF));
				fNv = (unsigned char)(((X - value8)) & SIGN);
				fC = (unsigned char)((X >= value8) != 0);
				ticks += 3;
			NEXT();

			op_E5: ;  // ZPG SBC
				value8 = readMem(fetch(PC));
				PC++;
				value8 ^= 0xFF;
				if (fD)
					value8 -= 0x0066;
				value16 = A + value8 + fC;
				fV = (unsigned char)(((value16 ^ A) & (value16 ^ value8) & 0x0080) != 0);
				if (fD)
					value16 += ((((value16 + 0x66) ^ A ^ value8) >> 3) & 0x22) * 3;
				fC = (unsigned char)(value16 > 0xFF);
				A = value16 & 0xFF;
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 3;
			NEXT();

			op_E6: ;  // ZPG INC
				address = fetch(PC);
				PC++;
				value8 = readMem(address);
				value8++;
				writeMem(address, value8);
				fZv = (unsigned char)(value8);
				fNv = (unsigned char)(value8);
				ticks += 5;
			NEXT();

			op_E8: ;  // IMP INX
				X++;
				fZv = (unsigned char)(X);
				fNv = (unsigned char)(X);
				ticks += 2;
			NEXT();

			op_E9: ;  // IMM SBC
				value8 = fetch(PC);
				PC++;
				value8 ^= 0xFF;
				if (fD)
					value8 -= 0x0066;
				value16 = A + value8 + (fC);
				fV = (unsigned char)(((value16 ^ A) & (value16 ^ value8) & 0x0080) != 0);
				if (fD)
					value16 += ((((value16 + 0x66) ^ A ^ value8) >> 3) & 0x22) * 3;
				fC = (unsigned char)(value16 > 0xFF);
				A = value16 & 0xFF;
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 2;
			NEXT();

      op_EA: ;  // IMP NOP
        ticks += 2;
      NEXT();

			op_EC: ;  // ABS CPX
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				value8 = readMem(address);
				fZv = (unsigned char)(((X - value8) & 0xFF));
				fNv = (unsigned char)(((X - value8)) & SIGN);
				fC = (unsigned char)((X >= value8) != 0);
				ticks += 4;
			NEXT();

			op_ED: ;  // ABS SBC
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				value8 = readMem(address);
				value8 ^= 0xFF;
				if (fD)
					value8 -= 0x0066;
				value16 = A + value8 + fC;
				fV = (unsigned char)(((value16 ^ A) & (value16 ^ value8) & 0x0080) != 0);
				if (fD)
					value16 += ((((value16 + 0x66) ^ A ^ value8) >> 3) & 0x22) * 3;
				fC = (unsigned char)(value16 > 0xFF);
				A = value16 & 0xFF;
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 4;
			NEXT();

			op_EE: ;  // ABS INC
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				value8 = readMem(address);
				PH_RMW(address, value8);
				value8++;
				writeMem(address, value8);
				fZv = (unsigned char)(value8);
				fNv = (unsigned char)(value8);
				ticks += 6;
			NEXT();

			op_F0: ;  // REL BEQ
				address = fetch(PC);
				PC++;
				if ((fZv == 0)) {  // branch taken
					ticks++;
					if (address & SIGN)
						address |= 0xFF00;  // jump backward
					if (((PC & 0xFF) + address) & 0xFF00)  // page crossing
						ticks++;
					PC += address;
				}
				ticks += 2;
			NEXT();

			op_F1: ;  // IZY SBC
				value8 = fetch(PC);
				PC++;
				address = readMem(value8);
				if ((address + Y) & 0xFF00)  // page crossing
					ticks++;
				value8++;
				address |= readMem(value8) << 8;
				PH_R_IZY(value8, address);
				address += Y;
				value8 = readMem(address);
				value8 ^= 0xFF;
				if (fD)
					value8 -= 0x0066;
				value16 = A + value8 + fC;
				fV = (unsigned char)(((value16 ^ A) & (value16 ^ value8) & 0x0080) != 0);
				if (fD)
					value16 += ((((value16 + 0x66) ^ A ^ value8) >> 3) & 0x22) * 3;
				fC = (unsigned char)(value16 > 0xFF);
				A = value16 & 0xFF;
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 5;
			NEXT();

			op_F5: ;  // ZPX SBC
				address = (fetch(PC) + X) & 0xFF;
				PC++;
				value8 = readMem(address);
				value8 ^= 0xFF;
				if (fD)
					value8 -= 0x0066;
				value16 = A + value8 + fC;
				fV = (unsigned char)(((value16 ^ A) & (value16 ^ value8) & 0x0080) != 0);
				if (fD)
					value16 += ((((value16 + 0x66) ^ A ^ value8) >> 3) & 0x22) * 3;
				fC = (unsigned char)(value16 > 0xFF);
				A = value16 & 0xFF;
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 4;
			NEXT();

			op_F6: ;  // ZPX INC
				address = (fetch(PC) + X) & 0xFF;
				PC++;
				value8 = readMem(address);
				value8++;
				writeMem(address, value8);
				fZv = (unsigned char)(value8);
				fNv = (unsigned char)(value8);
				ticks += 6;
			NEXT();

      op_F8: ;  // IMP SED
        fD = (unsigned char)(1);
        ticks += 2;
      NEXT();

			op_F9: ;  // ABY SBC
				address = fetch(PC);
				PC++;
				if ((address + Y) & 0xFF00)  // page crossing
					ticks++;
				address |= fetch(PC) << 8;
				PC++;
				PH_R_ABSIDX(address, Y);
				address += Y;
				value8 = readMem(address);
				value8 ^= 0xFF;
				if (fD)
					value8 -= 0x0066;
				value16 = A + value8 + fC;
				fV = (unsigned char)(((value16 ^ A) & (value16 ^ value8) & 0x0080) != 0);
				if (fD)
					value16 += ((((value16 + 0x66) ^ A ^ value8) >> 3) & 0x22) * 3;
				fC = (unsigned char)(value16 > 0xFF);
				A = value16 & 0xFF;
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 4;
			NEXT();

			op_FD: ;  // ABX SBC
				address = fetch(PC);
				PC++;
				if ((address + X) & 0xFF00)  // page crossing
					ticks++;
				address |= fetch(PC) << 8;
				PC++;
				PH_R_ABSIDX(address, X);
				address += X;
				value8 = readMem(address);
				value8 ^= 0xFF;
				if (fD)
					value8 -= 0x0066;
				value16 = A + value8 + fC;
				fV = (unsigned char)(((value16 ^ A) & (value16 ^ value8) & 0x0080) != 0);
				if (fD)
					value16 += ((((value16 + 0x66) ^ A ^ value8) >> 3) & 0x22) * 3;
				fC = (unsigned char)( (value16 & 0xFF00) != 0);
				A = value16 & 0xFF;
				fZv = (unsigned char)(A);
				fNv = (unsigned char)(A);
				ticks += 4;
			NEXT();

			op_FE: ;  // ABX INC
				address = fetch(PC);
				PC++;
				address |= fetch(PC) << 8;
				PC++;
				PH_W_ABSIDX(address, X);
				address += X;
				value8 = readMem(address);
				PH_RMW(address, value8);
				value8++;
				writeMem(address, value8);
				fZv = (unsigned char)(value8);
				fNv = (unsigned char)(value8);
				ticks += 7;
			NEXT();

			op_bad: ;  // invalid / undocumented opcode
				/* : these are exactly the bytes whose 65C02 and 65816
				 * meanings diverge.  Log (PC, opcode) when A2_BADOP is set;
				 * default OFF, and op_bad is unreachable on a clean run. */
				if (g_badop) badop_record((uint16_t)(PC - 1), fetch(PC - 1));
				if (g_cpu == CPU_DEFAULT) { ticks += 2; NEXT(); }
				{
				/* ---  CPU PERSONALITY.  PC already points past the opcode. */
				uint16_t bpc = (uint16_t)(PC - 1);
				uint8_t  bop = fetch(bpc);
				uint16_t w16; uint8_t  b8;
				/* JAM: rewind to the offending PC and burn cycles forever, so
				 * the machine goes visibly static exactly like the hardware. */
#define CPU_JAM(why) do { cpu_trap(bpc, bop, (why)); PC = bpc; ticks += 2; NEXT(); } while (0)
#define PUSH8(v)  do { writeMem(0x100 + SP, (uint8_t)(v)); SP--; } while (0)
#define PULL8(d)  do { SP++; (d) = readMem(0x100 + SP); } while (0)
				if (g_cpu_trapped) { PC = bpc; ticks += 2; NEXT(); }

				if (g_cpu == CPU_65C02) {
					/* The 65C02 instructions the enhanced IIe uses (NCR/GTE 65C02:
					 * no Rockwell RMB/SMB/BBR/BBS).  Cycle counts per the data sheet,
					 * without the decimal-mode extra cycle. */
#define C02_ZPI() (b8 = fetch(PC), PC++, (uint16_t)(readMem(b8) | (readMem((uint8_t)(b8 + 1)) << 8)))
#define C02_ABS() (w16 = (uint16_t)(fetch(PC) | (fetch(PC + 1) << 8)), PC += 2, w16)
#define C02_NZ(r) do { fZv = (unsigned char)(r); fNv = (unsigned char)(r); } while (0)
					switch (bop) {
					case 0x80: { int8_t off = (int8_t)fetch(PC); PC++;              /* BRA */
					             PC = (uint16_t)(PC + off); ticks += 3; NEXT(); }
					case 0x64: writeMem(fetch(PC), 0); PC++; ticks += 3; NEXT();    /* STZ zp */
					case 0x74: writeMem((uint8_t)(fetch(PC) + X), 0); PC++; ticks += 4; NEXT();
					case 0x9C: writeMem(C02_ABS(), 0); ticks += 4; NEXT();          /* STZ abs */
					case 0x9E: writeMem((uint16_t)(C02_ABS() + X), 0); ticks += 5; NEXT();
					case 0xDA: PUSH8(X); ticks += 3; NEXT();                         /* PHX */
					case 0xFA: PULL8(X); C02_NZ(X); ticks += 4; NEXT();             /* PLX */
					case 0x5A: PUSH8(Y); ticks += 3; NEXT();                         /* PHY */
					case 0x7A: PULL8(Y); C02_NZ(Y); ticks += 4; NEXT();             /* PLY */
					case 0x1A: A++; C02_NZ(A); ticks += 2; NEXT();                   /* INC A */
					case 0x3A: A--; C02_NZ(A); ticks += 2; NEXT();                   /* DEC A */
					case 0x89: b8 = fetch(PC); PC++; fZv = (unsigned char)(A & b8);  /* BIT #: Z only */
					           ticks += 2; NEXT();
					case 0x34: w16 = (uint8_t)(fetch(PC) + X); PC++; goto c02_bit;  /* BIT zp,X */
					case 0x3C: w16 = (uint16_t)(C02_ABS() + X); goto c02_bit;       /* BIT abs,X */
					c02_bit:   b8 = readMem(w16); fZv = (unsigned char)(A & b8); fNv = b8;
					           fV = (b8 & OFLOW) != 0; ticks += 4; NEXT();
					case 0x04: w16 = fetch(PC); PC++; b8 = readMem(w16);             /* TSB zp */
					           fZv = (unsigned char)(A & b8); writeMem(w16, b8 | A); ticks += 5; NEXT();
					case 0x0C: (void)C02_ABS(); b8 = readMem(w16);                        /* TSB abs */
					           fZv = (unsigned char)(A & b8); writeMem(w16, b8 | A); ticks += 6; NEXT();
					case 0x14: w16 = fetch(PC); PC++; b8 = readMem(w16);             /* TRB zp */
					           fZv = (unsigned char)(A & b8); writeMem(w16, b8 & ~A); ticks += 5; NEXT();
					case 0x1C: (void)C02_ABS(); b8 = readMem(w16);                        /* TRB abs */
					           fZv = (unsigned char)(A & b8); writeMem(w16, b8 & ~A); ticks += 6; NEXT();
					case 0x7C: w16 = (uint16_t)(C02_ABS() + X);                       /* JMP (abs,X) */
					           PC = (uint16_t)(readMem(w16) | (readMem((uint16_t)(w16 + 1)) << 8));
					           ticks += 6; NEXT();
					case 0x12: A |= readMem(C02_ZPI()); C02_NZ(A); ticks += 5; NEXT();   /* ORA (zp) */
					case 0x32: A &= readMem(C02_ZPI()); C02_NZ(A); ticks += 5; NEXT();   /* AND (zp) */
					case 0x52: A ^= readMem(C02_ZPI()); C02_NZ(A); ticks += 5; NEXT();   /* EOR (zp) */
					case 0x92: writeMem(C02_ZPI(), A); ticks += 5; NEXT();               /* STA (zp) */
					case 0xB2: A = readMem(C02_ZPI()); C02_NZ(A); ticks += 5; NEXT();    /* LDA (zp) */
					case 0xD2: value8 = readMem(C02_ZPI());                              /* CMP (zp) */
					           fZv = (unsigned char)((A - value8) & 0xFF);
					           fNv = (unsigned char)((A - value8) & SIGN);
					           fC = (unsigned char)(A >= value8); ticks += 5; NEXT();
					case 0x72: value8 = readMem(C02_ZPI());                              /* ADC (zp) */
					           value16 = A + value8 + fC;
					           fV = (unsigned char)(((value16 ^ A) & (value16 ^ value8) & 0x0080) != 0);
					           if (fD) value16 += ((((value16 + 0x66) ^ A ^ value8) >> 3) & 0x22) * 3;
					           fC = (unsigned char)(value16 > 0xFF); A = value16 & 0xFF;
					           C02_NZ(A); ticks += 5; NEXT();
					case 0xF2: value8 = readMem(C02_ZPI()) ^ 0xFF;                       /* SBC (zp) */
					           if (fD) value8 -= 0x0066;
					           value16 = A + value8 + fC;
					           fV = (unsigned char)(((value16 ^ A) & (value16 ^ value8) & 0x0080) != 0);
					           if (fD) value16 += ((((value16 + 0x66) ^ A ^ value8) >> 3) & 0x22) * 3;
					           fC = (unsigned char)(value16 > 0xFF); A = value16 & 0xFF;
					           C02_NZ(A); ticks += 5; NEXT();
					default: break;
					}
#undef C02_ZPI
#undef C02_ABS
#undef C02_NZ
					b8 = c02_enc[bop];
					if (!b8) CPU_JAM("65C02 instruction (not a NOP) is not modelled");
					PC = (uint16_t)(PC + (b8 >> 4) - 1);
					ticks += (b8 & 0x0F);
					NEXT();
				}
				if (g_cpu == CPU_NMOS) {
					if ((bop & 0x0F) == 0x02) CPU_JAM("NMOS 6502 KIL/JAM");
					CPU_JAM("NMOS undocumented opcode is not modelled");
				}
				/* ------------------------------------------ CPU_65816, E = 1 */
				switch (bop) {
				case 0x42:                       /* WDM #imm -- true 2-byte NOP */
					PC++; ticks += 2; NEXT();
				case 0xEB:                       /* XBA */
					b8 = A; A = g_816_B; g_816_B = b8;
					fZv = A; fNv = A; ticks += 3; NEXT();
				case 0x8B:                       /* PHB */
					PUSH8(g_816_DBR); ticks += 3; NEXT();
				case 0xAB:                       /* PLB */
					PULL8(g_816_DBR);
					fZv = g_816_DBR; fNv = g_816_DBR; ticks += 4;
					if (g_816_DBR) CPU_JAM("PLB set DBR != 0; this core has no banks");
					NEXT();
				case 0x0B:                       /* PHD */
					PUSH8(g_816_D >> 8); PUSH8(g_816_D & 0xFF);
					ticks += 4; NEXT();
				case 0x2B:                       /* PLD */
					PULL8(b8); g_816_D = b8; PULL8(b8); g_816_D |= (uint16_t)b8 << 8;
					fZv = (unsigned char)(g_816_D | (g_816_D >> 8));
					fNv = (unsigned char)(g_816_D >> 8);
					ticks += 5;
					if (g_816_D) CPU_JAM("PLD relocated the direct page; not modelled");
					NEXT();
				case 0x82:                       /* BRL rel16 */
					w16 = (uint16_t)(fetch(PC) | (fetch((uint16_t)(PC + 1)) << 8));
					PC = (uint16_t)(PC + 2 + (int16_t)w16);
					ticks += 4; NEXT();
				case 0x62:                       /* PER rel16 */
					w16 = (uint16_t)(fetch(PC) | (fetch((uint16_t)(PC + 1)) << 8));
					PC += 2;
					w16 = (uint16_t)(PC + (int16_t)w16);
					PUSH8(w16 >> 8); PUSH8(w16 & 0xFF);
					ticks += 6; NEXT();
				case 0x5C:                       /* JML/JMP long */
					w16 = (uint16_t)(fetch(PC) | (fetch((uint16_t)(PC + 1)) << 8));
					b8  = fetch((uint16_t)(PC + 2));
					ticks += 4;
					if (b8) CPU_JAM("JMP long to a non-zero bank; this core has no banks");
					PC = w16; NEXT();
				case 0xDC:                       /* JML [abs] -- pointer in BANK 0 */
					w16 = (uint16_t)(fetch(PC) | (fetch((uint16_t)(PC + 1)) << 8));
					b8  = readMem((uint16_t)(w16 + 2));
					ticks += 6;
					if (b8) CPU_JAM("JMP [abs] to a non-zero bank; this core has no banks");
					PC = (uint16_t)(readMem(w16) | (readMem((uint16_t)(w16 + 1)) << 8));
					NEXT();
				case 0xFC:                       /* ⭐ JSR (abs,X) -- THE $B258 BUG */
					w16 = (uint16_t)(fetch(PC) | (fetch((uint16_t)(PC + 1)) << 8));
					PC++;                        /* return addr = last byte of insn */
					PUSH8(PC >> 8); PUSH8(PC & 0xFF);
					w16 = (uint16_t)(w16 + X);   /* PBR:(abs+X); PBR is 0 here */
					PC = (uint16_t)(readMem(w16) | (readMem((uint16_t)(w16 + 1)) << 8));
					ticks += 8; NEXT();
				case 0x02:                       /* ⭐ COP #imm -- vectors via $FFF4 */
					PC++;                        /* skip the signature byte */
					PUSH8(PC >> 8); PUSH8(PC & 0xFF);
					PUSH8(PACK_P());             /* measured: P pushed UNCHANGED */
					fI = 1; fD = 0;
					PC = (uint16_t)(readMem(0xFFF4) | (readMem(0xFFF5) << 8));
					ticks += 7; NEXT();
				case 0x44: CPU_JAM("65816 MVP block move -- needs banks; trapped by design");
				case 0x54: CPU_JAM("65816 MVN block move -- needs banks; trapped by design");
				default:
					if (c02_enc[bop])
						CPU_JAM("65816 opcode (a NOP only on the 65C02) is not modelled");
					CPU_JAM("65816/65C02 instruction is not modelled");
				}
#undef CPU_JAM
#undef PUSH8
#undef PULL8
				}
			NEXT();
	exec_done: ;
	/* sync the registerized state back to the real (shadowed) globals */
	*gPC = PC; *gA = A; *gX = X; *gY = Y; *gSP = SP; *gP = PACK_P();
	*gTICKS = ticks;
	return PC;
	}
}

/* back to the plain inline wrappers -- the debug helpers below (dasm,
 * printRegs) and puce6502.h users run outside Exec, where the global
 * `ticks` is always current and gTICKS doesn't exist. */
#undef readMem
#undef writeMem



// the code below was used during developpment for test and debug
// and is not required for normal operation

#include <stdio.h>

static const char* mn[256] = {
	"BRK","ORA","UND","UND","UND","ORA","ASL","UND","PHP","ORA","ASL","UND","UND","ORA","ASL","UND",
  "BPL","ORA","UND","UND","UND","ORA","ASL","UND","CLC","ORA","UND","UND","UND","ORA","ASL","UND",
  "JSR","AND","UND","UND","BIT","AND","ROL","UND","PLP","AND","ROL","UND","BIT","AND","ROL","UND",
  "BMI","AND","UND","UND","UND","AND","ROL","UND","SEC","AND","UND","UND","UND","AND","ROL","UND",
  "RTI","EOR","UND","UND","UND","EOR","LSR","UND","PHA","EOR","LSR","UND","JMP","EOR","LSR","UND",
  "BVC","EOR","UND","UND","UND","EOR","LSR","UND","CLI","EOR","UND","UND","UND","EOR","LSR","UND",
  "RTS","ADC","UND","UND","UND","ADC","ROR","UND","PLA","ADC","ROR","UND","JMP","ADC","ROR","UND",
  "BVS","ADC","UND","UND","UND","ADC","ROR","UND","SEI","ADC","UND","UND","UND","ADC","ROR","UND",
  "UND","STA","UND","UND","STY","STA","STX","UND","DEY","UND","TXA","UND","STY","STA","STX","UND",
  "BCC","STA","UND","UND","STY","STA","STX","UND","TYA","STA","TXS","UND","UND","STA","UND","UND",
  "LDY","LDA","LDX","UND","LDY","LDA","LDX","UND","TAY","LDA","TAX","UND","LDY","LDA","LDX","UND",
  "BCS","LDA","UND","UND","LDY","LDA","LDX","UND","CLV","LDA","TSX","UND","LDY","LDA","LDX","UND",
  "CPY","CMP","UND","UND","CPY","CMP","DEC","UND","INY","CMP","DEX","UND","CPY","CMP","DEC","UND",
  "BNE","CMP","UND","UND","UND","CMP","DEC","UND","CLD","CMP","UND","UND","UND","CMP","DEC","UND",
  "CPX","SBC","UND","UND","CPX","SBC","INC","UND","INX","SBC","NOP","UND","CPX","SBC","INC","UND",
  "BEQ","SBC","UND","UND","UND","SBC","INC","UND","SED","SBC","UND","UND","UND","SBC","INC","UND"
};

static const int am[256] = {
   0x0 , 0xC , 0x0 , 0x0 , 0x0 , 0x3 , 0x3 , 0x0 , 0x0 , 0x2 , 0x1 , 0x0 , 0x0 , 0x7 , 0x7 , 0x0 ,
   0x6 , 0xD , 0x0 , 0x0 , 0x0 , 0x4 , 0x4 , 0x0 , 0x0 , 0x9 , 0x0 , 0x0 , 0x0 , 0x8 , 0x8 , 0x0 ,
   0x7 , 0xC , 0x0 , 0x0 , 0x3 , 0x3 , 0x3 , 0x0 , 0x0 , 0x2 , 0x1 , 0x0 , 0x7 , 0x7 , 0x7 , 0x0 ,
   0x6 , 0xD , 0x0 , 0x0 , 0x0 , 0x4 , 0x4 , 0x0 , 0x0 , 0x9 , 0x0 , 0x0 , 0x0 , 0x8 , 0x8 , 0x0 ,
   0x0 , 0xC , 0x0 , 0x0 , 0x0 , 0x3 , 0x3 , 0x0 , 0x0 , 0x2 , 0x1 , 0x0 , 0x7 , 0x7 , 0x7 , 0x0 ,
   0x6 , 0xD , 0x0 , 0x0 , 0x0 , 0x4 , 0x4 , 0x0 , 0x0 , 0x9 , 0x0 , 0x0 , 0x0 , 0x8 , 0x8 , 0x0 ,
   0x0 , 0xC , 0x0 , 0x0 , 0x0 , 0x3 , 0x3 , 0x0 , 0x0 , 0x2 , 0x1 , 0x0 , 0xA , 0x7 , 0x7 , 0x0 ,
   0x6 , 0xD , 0x0 , 0x0 , 0x0 , 0x4 , 0x4 , 0x0 , 0x0 , 0x9 , 0x0 , 0x0 , 0x0 , 0x8 , 0x8 , 0x0 ,
   0x0 , 0xC , 0x0 , 0x0 , 0x3 , 0x3 , 0x3 , 0x0 , 0x0 , 0x0 , 0x0 , 0x0 , 0x7 , 0x7 , 0x7 , 0x0 ,
   0x6 , 0xD , 0x0 , 0x0 , 0x4 , 0x4 , 0x5 , 0x0 , 0x0 , 0x9 , 0x0 , 0x0 , 0x0 , 0x8 , 0x0 , 0x0 ,
   0x2 , 0xC , 0x2 , 0x0 , 0x3 , 0x3 , 0x3 , 0x0 , 0x0 , 0x2 , 0x0 , 0x0 , 0x7 , 0x7 , 0x7 , 0x0 ,
   0x6 , 0xD , 0x0 , 0x0 , 0x4 , 0x4 , 0x5 , 0x0 , 0x0 , 0x9 , 0x0 , 0x0 , 0x8 , 0x8 , 0x9 , 0x0 ,
   0x2 , 0xC , 0x0 , 0x0 , 0x3 , 0x3 , 0x3 , 0x0 , 0x0 , 0x2 , 0x0 , 0x0 , 0x7 , 0x7 , 0x7 , 0x0 ,
   0x6 , 0xD , 0x0 , 0x0 , 0x0 , 0x4 , 0x4 , 0x0 , 0x0 , 0x9 , 0x0 , 0x0 , 0x0 , 0x8 , 0x8 , 0x0 ,
   0x2 , 0xC , 0x0 , 0x0 , 0x3 , 0x3 , 0x3 , 0x0 , 0x0 , 0x2 , 0x0 , 0x0 , 0x7 , 0x7 , 0x7 , 0x0 ,
   0x6 , 0xD , 0x0 , 0x0 , 0x0 , 0x4 , 0x4 , 0x0 , 0x0 , 0x9 , 0x0 , 0x0 , 0x0 , 0x8 , 0x8 , 0x0
 };

void dasm(uint16_t address) {

  uint8_t op = readMem(address);
  uint8_t b1 = readMem((address + 1) & 0xFFFF);
  uint8_t b2 = readMem((address + 2) & 0xFFFF);

  printf("%04X %02X ", address, op);

  switch(am[op]) {
    case 0x0: printf("       %s          ",              mn[op]      ); break;  // implied
    case 0x1: printf("       %s A        ",              mn[op]      ); break;  // accumulator
    case 0x2: printf("%02X     %s #$%02X     ",    b1,   mn[op],b1   ); break;  // immediate
    case 0x3: printf("%02X     %s $%02X      ",    b1,   mn[op],b1   ); break;  // zero page
    case 0x4: printf("%02X     %s $%02X,X    ",    b1,   mn[op],b1   ); break;  // zero page, X indexed
    case 0x5: printf("%02X     %s $%02X,Y    ",    b1,   mn[op],b1   ); break;  // zero page, Y indexed
    case 0x6: printf("%02X     %s $%02X      ",    b1,   mn[op],b1   ); break;  // relative
    case 0xC: printf("%02X     %s ($%02X,X)  ",    b1,   mn[op],b1   ); break;  // X indexed, indirect
    case 0xD: printf("%02X     %s ($%02X),Y  ",    b1,   mn[op],b1   ); break;  // indirect, Y indexed
    case 0x7: printf("%02X%02X   %s $%02X%02X    ",b1,b2,mn[op],b2,b1); break;  // absolute
    case 0x8: printf("%02X%02X   %s $%02X%02X,X  ",b1,b2,mn[op],b2,b1); break;  // absolute, X indexed
    case 0x9: printf("%02X%02X   %s $%02X%02X,Y  ",b1,b2,mn[op],b2,b1); break;  // absolute, Y indexed
    case 0xA: printf("%02X%02X   %s ($%02X%02X)  ",b1,b2,mn[op],b2,b1); break;  // indirect
  }
}

void printRegs() {
  printf("A=%02X  X=%02X  Y=%02X  S=%02X  *S=%02X  %c%c%c%c%c%c%c%c", \
	A, X, Y, SP, readMem(0x100 + SP), \
	P.S?'N':'-', P.V?'V':'-', P.U?'U':'.', P.B?'B':'-', \
	P.D?'D':'-', P.I?'I':'-', P.Z?'Z':'-', P.C?'C':'-');
}

void setPC(uint16_t address) {
	PC = address;
}

uint16_t getPC(){
	return PC;
}



#if _FUNCTIONNAL_TESTS

		// 6502 functonnal tests
		// using Klaus Dormann's functonnal tests published at :
		// https://github.com/Klaus2m5/6502_65C02_functional_tests

		int main(int argc, char* argv[]){

			char *filename = "6502_functional_test.bin";
			FILE *f = fopen(filename, "rb");
		  if (!f || fread(RAM, 1, 65536, f) != 65536) {
		    printf("ERROR : can't load %s\n", filename);
		    return(0);
		  }
		  fclose(f);

			puce6502RST();  // reset the CPU
			PC = 0x400;  // set Program Counter to start of code

			unsigned long long int oldticks = 0;
			uint16_t oldPC = PC, newPC = PC;  // to detect the BNE $FE when an error occurs

			// while(1) {
			// 	dasm(newPC);
			// 	printf("  ");
			// 	newPC = puce6502Exec(1);
			// 	printRegs();
			// 	printf("   Cycles: %llu   Total: %llu\n", ticks - oldticks, ticks);
			// 	oldticks = ticks;
			//
			//   if (newPC == 0x3469){  // 6502_functional_test SUCCESS
			// 		printf("\nReached end of 6502_functional_test @ %04X : SUCCESS !\n", newPC);
			// 	  break;
			// 	}
			//
			// 	if (newPC == oldPC ) {
			// 		printf("\n\nLoop detected @ %04X - Press ENTER to proceed with next test or CTRL<C> to stop\n\n", newPC);
			// 		return(-1);
			// 		getchar();
			// 		PC = newPC + 2;
			// 	}
			// 	oldPC = newPC;
			// }

		  // Benchmark : replace the above while loop by this one
			  while(puce6502Exec(100) != 0x3469);
			  printf("%llu\n", ticks);
		  // and use the time utility to avaluate the speed the emulated 65C02

			return(0);
		}

#endif


/* test results : ## Using 6502_functional_test. */

