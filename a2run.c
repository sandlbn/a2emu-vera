/* a2run.c -- headless Apple II+ / VERA runner for test gates (plan E1, E2, E4, E5,
 * E7, E11).  No SDL: the machine is a2core.c + puce6502.c + veracore.c.
 *
 *   a2run [options] [disk1.dsk]
 *     --rom DIR             directory with the ROMs (default: <exe dir>/rom): appleII+.rom,
 *                           diskII.rom, the IIe ROMs (found by part number, see a2core.c)
 *     --model M             ii+ (default) | iie (unenhanced IIe, 6502) | iiee (enhanced IIe,
 *                           65C02; phantom model 65c02 unless --phantom says otherwise)
 *     --aux 0|64            IIe: extended 80-column card with 64 KB aux RAM (default 0 = none,
 *                           the "non-extended" IIe: plan A-D2 baseline)
 *     --disk1 F --disk2 F   Disk ][ images (.dsk/.do DOS order, .po ProDOS order, .nib), slot 6;
 *                           written back to their files at exit (E6) unless --no-writeback
 *     --hd F                ProDOS block device in slot 7 (.po/.hdv/.2mg, up to 32 MB): boots first
 *     --load FILE@ADDR      copy FILE into RAM at ADDR (hex, $ or 0x optional); repeatable
 *     --pc ADDR             start the CPU at ADDR (I=1, SP=$FF) instead of the ROM reset
 *     --cycles N            stop after N CPU cycles (exit 124)
 *     --frames N            stop after N VERA frames (exit 124)
 *     --vera-slot N         VERA card in slot N (1-7, default 4; 0 = no VERA card)
 *     --vera-irq on|off     VERA IRQ -> /IRQ (default on = A2VERA with D1)
 *     --phantom off|nmos|65c02   6502 dummy-read model (default nmos)
 *     --xmem KB             RAM card ("Slinky") of KB kilobytes (power of two)
 *     --xmem-slot N         its slot (default 5)
 *     --xmem-image F        preload the RAM card from F;  --xmem-dump F  save it at exit
 *     --debug-slot N        debug card slot (default 3, 0 = none)
 *     --shot-prefix P       debug-card screenshots -> P000.png, P001.png ... (default "shot")
 *     --shot-full           screenshots at VERA's 640x480 output (default 320x240 = every 2nd pixel)
 *     --shot-delay N        a debug-card shot is taken N frame ends after the request: 0 = the
 *                           last completed frame (now), 1 = the end of the current frame,
 *                           2 = after the next full frame (default; a frame half-drawn with
 *                           the old state is never shot)
 *     --shot-exit F         screenshot of the last VERA frame at exit
 *     --vramdump F          VERA state at exit (x16emu -dump V layout, for veracheck.py)
 *     --keys TEXT           type TEXT (\n = RETURN, ~ = 2 s pause) once the machine has run
 *     --keys-at N           ... starting at VERA frame N (default 60)
 *     --input FILE          scripted input per VERA frame (keys, buttons, paddles, game-port
 *                           pad, SNES MAX pads): format in a2run_input.c (E8)
 *     --snesmax SLOT        SNES MAX card in SLOT (1-7; default: env A2_SNESMAX, else none)
 *     --text                print the Apple text page 1 at exit
 *     --no-writeback        never write floppy changes back to the image files
 *     --wav FILE            the VERA PSG + PCM mix as a WAV (48,828 Hz stereo 16-bit; E9,
 *                           a2run_audio.c)
 *     --psglog FILE         the PSG registers after every frame that changed them
 *     --psgvlog FILE        the same, sampled at each VSYNC before its IRQ runs (the
 *                           state after the previous IRQ's work; a2run_audio.c)
 *     --ym on|off           the A2VERA's YM2151 at VERA base + $20 (E14; default on,
 *                           env A2_YM); its audio is in the --wav mix
 *     --ymlog FILE          every YM2151 data write: "<frame> <reg> <val>" (hex)
 *     --pctrace N           print the last N program counters (one per ~32-cycle step) at exit
 *     --stop-pc ADDR        stop as soon as a step ends at ADDR (exit 125; with --pctrace)
 *     --zpmin ADDR[,LO[,TOP]]  the lowest 16-bit value (>= LO) at ZP ADDR, from the first
 *                           time it equals TOP (e.g. cc65's c_sp:
 *                           the C stack depth), sampled every step; printed at exit
 *     --quiet               no summary line on stderr
 * Exit code: the value the guest writes to the debug card's $C0n1, 124 on a limit,
 * 2 on a setup error.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "puce6502.h"
#include "veracore.h"
#include "a2core.h"
#include "a2run_audio.h"
#include "a2run_input.h"
#include "ym2151.h"

extern int g_phantom, g_cpu;             /* puce6502.c */

//=========================================================== PNG (stored deflate)

static uint32_t crc_tab[256];
static void crc_init(void) {
	for (uint32_t n = 0; n < 256; n++) {
		uint32_t c = n;
		for (int k = 0; k < 8; k++) c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
		crc_tab[n] = c;
	}
}
static uint32_t crc_upd(uint32_t c, const uint8_t *b, size_t n) {
	while (n--) c = crc_tab[(c ^ *b++) & 0xFF] ^ (c >> 8);
	return c;
}
static void be32(uint8_t *p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }
static void png_chunk(FILE *f, const char *type, const uint8_t *data, uint32_t len) {
	uint8_t hdr[8]; be32(hdr, len); memcpy(hdr + 4, type, 4);
	fwrite(hdr, 1, 8, f); if (len) fwrite(data, 1, len, f);
	uint32_t c = crc_upd(0xFFFFFFFFu, (const uint8_t *)type, 4);
	c = crc_upd(c, data, len) ^ 0xFFFFFFFFu;
	uint8_t t[4]; be32(t, c); fwrite(t, 1, 4, f);
}
/* rgb = 0x00RRGGBB pixels */
static int png_write(const char *path, int w, int h, const uint32_t *rgb, int stride) {
	FILE *f = fopen(path, "wb");
	if (!f) return 0;
	static const uint8_t sig[8] = { 0x89, 'P', 'N', 'G', 13, 10, 26, 10 };
	fwrite(sig, 1, 8, f);
	uint8_t ihdr[13]; be32(ihdr, w); be32(ihdr + 4, h);
	ihdr[8] = 8; ihdr[9] = 2; ihdr[10] = ihdr[11] = ihdr[12] = 0;
	png_chunk(f, "IHDR", ihdr, 13);
	size_t raw_len = (size_t)h * (1 + 3 * w);
	uint8_t *raw = malloc(raw_len), *p = raw;
	for (int y = 0; y < h; y++) {
		*p++ = 0;
		for (int x = 0; x < w; x++) {
			uint32_t v = rgb[(size_t)y * stride + x];
			*p++ = v >> 16; *p++ = v >> 8; *p++ = v;
		}
	}
	size_t nblk = (raw_len + 65534) / 65535;
	size_t zlen = 2 + raw_len + nblk * 5 + 4;
	uint8_t *z = malloc(zlen), *q = z;
	*q++ = 0x78; *q++ = 0x01;
	uint32_t a = 1, b = 0;
	for (size_t i = 0; i < raw_len; i++) { a = (a + raw[i]) % 65521; b = (b + a) % 65521; }
	for (size_t off = 0; off < raw_len; off += 65535) {
		size_t n = raw_len - off < 65535 ? raw_len - off : 65535;
		*q++ = (off + n == raw_len); *q++ = n & 0xFF; *q++ = n >> 8; *q++ = ~n & 0xFF; *q++ = (~n >> 8) & 0xFF;
		memcpy(q, raw + off, n); q += n;
	}
	be32(q, (b << 16) | a); q += 4;
	png_chunk(f, "IDAT", z, (uint32_t)(q - z));
	png_chunk(f, "IEND", NULL, 0);
	free(raw); free(z);
	return fclose(f) == 0;
}

//=================================================================== the runner

static const char *shot_prefix = "shot";
static int shot_full = 0, shot_n = 0, shot_pending = 0, shot_delay = 2;
static const char *vramdump_path = NULL;

static int vera_shot(const char *path) {
	vc_render_frame(&g_vera);
	if (shot_full) return png_write(path, VC_SCREEN_W, VC_SCREEN_H, g_vera.framebuffer, VC_SCREEN_W);
	static uint32_t half[(VC_SCREEN_W / 2) * (VC_SCREEN_H / 2)];
	for (int y = 0; y < VC_SCREEN_H / 2; y++)
		for (int x = 0; x < VC_SCREEN_W / 2; x++)
			half[y * (VC_SCREEN_W / 2) + x] = g_vera.framebuffer[(2 * y) * VC_SCREEN_W + 2 * x] & 0xFFFFFF;
	return png_write(path, VC_SCREEN_W / 2, VC_SCREEN_H / 2, half, VC_SCREEN_W / 2);
}

static void dbg_hook(int reg, uint8_t value) {
	if (reg == 2) {
		if (shot_delay == 0) {                              /* now: the last completed frame */
			char p[1100]; snprintf(p, sizeof p, "%s%03d.png", shot_prefix, shot_n++);
			if (!vera_shot(p)) fprintf(stderr, "a2run: screenshot %s failed\n", p);
		} else shot_pending = shot_delay;               /* at the Nth frame end */
	}
	else if (reg == 3) {
		const char *p = vramdump_path ? vramdump_path : "vram.bin";
		if (!vc_dump_state(p)) fprintf(stderr, "a2run: VRAM dump to %s failed\n", p);
	}
	else if (reg == 8) { /* profiler window: E10 */ (void)value; }
}

static unsigned parse_num(const char *s) {
	if (*s == '$') return (unsigned)strtoul(s + 1, NULL, 16);
	return (unsigned)strtoul(s, NULL, 0);
}
static unsigned parse_addr(const char *s) {                /* addresses are hex */
	if (*s == '$') s++;
	else if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) s += 2;
	return (unsigned)strtoul(s, NULL, 16);
}

static int load_file(const char *spec) {
	char path[1024]; const char *at = strrchr(spec, '@');
	if (!at || (size_t)(at - spec) >= sizeof path) { fprintf(stderr, "a2run: --load needs FILE@ADDR\n"); return 0; }
	memcpy(path, spec, at - spec); path[at - spec] = 0;
	unsigned addr = parse_addr(at + 1);
	FILE *f = fopen(path, "rb");
	if (!f) { fprintf(stderr, "a2run: cannot open %s\n", path); return 0; }
	static uint8_t buf[0x10000];
	size_t n = fread(buf, 1, sizeof buf, f); fclose(f);
	if (addr + n > RAMSIZE) { fprintf(stderr, "a2run: %s does not fit below $C000 at $%04X\n", path, addr); return 0; }
	memcpy(ram + addr, buf, n);
	return 1;
}

static void text_dump(void) {
	for (int row = 0; row < 24; row++) {
		unsigned base = 0x400 + (row & 7) * 0x80 + (row >> 3) * 0x28;
		char line[41];
		for (int c = 0; c < 40; c++) {
			uint8_t ch = ram[base + c] & 0x7F;
			if (ch < 0x20) ch += 0x40;                    /* inverse/flash upper case */
			line[c] = (char)ch;
		}
		line[40] = 0;
		printf("|%s|\n", line);
	}
}

static int read_rom(const char *dir, const char *name, uint8_t *dst, size_t len) {
	char p[1200]; snprintf(p, sizeof p, "%s/%s", dir, name);
	FILE *f = fopen(p, "rb");
	if (!f) { fprintf(stderr, "a2run: cannot open %s\n", p); return 0; }
	size_t n = fread(dst, 1, len, f); fclose(f);
	if (n != len) { fprintf(stderr, "a2run: %s must be %zu bytes\n", p, len); return 0; }
	return 1;
}

int main(int argc, char **argv) {
	char romdir[1024];
	{ const char *sl = strrchr(argv[0], '/');
	  if (sl) snprintf(romdir, sizeof romdir, "%.*s/rom", (int)(sl - argv[0]), argv[0]);
	  else snprintf(romdir, sizeof romdir, "rom"); }
	const char *hd = NULL; int writeback = 1;
	const char *disk1 = NULL, *disk2 = NULL, *xmem_img = NULL, *xmem_dump = NULL;
	const char *shot_exit = NULL, *keys = NULL, *loads[32]; int nloads = 0;
	long pc = -1; unsigned long long max_cycles = 0; long max_frames = 0;
	int vera_slot = 0, vslot_given = 0, vera_irq = -1, phantom = 1, xmem_kb = 0, xmem_slot = 5, dbg_slot = 3;
	int pctrace = 0; long stop_pc = -1;
	static uint16_t pcring[4096]; unsigned pcpos = 0;
	int ym_opt = -1; FILE *ymlog = NULL; int zpmin = -1; unsigned zpmin_v = 0xFFFF, zpmin_lo = 0x0800, zpmin_top = 0; int zpmin_arm = 1;
	const char *zpenv = getenv("A2_ZPMIN");             /* --zpmin default (gates) */
	int keys_at = 60, want_text = 0, quiet = 0, model = 0, phantom_given = 0, aux_kb = 0;
	const char *input = NULL; int snesmax = -1;

	for (int i = 1; i < argc; i++) {
		const char *a = argv[i];
		#define ARG (i + 1 < argc ? argv[++i] : (fprintf(stderr, "a2run: %s needs a value\n", a), exit(2), ""))
		if      (!strcmp(a, "--rom"))         snprintf(romdir, sizeof romdir, "%s", ARG);
		else if (!strcmp(a, "--disk1"))       disk1 = ARG;
		else if (!strcmp(a, "--disk2"))       disk2 = ARG;
		else if (!strcmp(a, "--hd"))          hd = ARG;
		else if (!strcmp(a, "--no-writeback")) writeback = 0;
		else if (!strcmp(a, "--load"))        { if (nloads < 32) loads[nloads++] = ARG; }
		else if (!strcmp(a, "--pc"))          pc = parse_addr(ARG);
		else if (!strcmp(a, "--cycles"))      max_cycles = strtoull(ARG, NULL, 0);
		else if (!strcmp(a, "--frames"))      max_frames = atol(ARG);
		else if (!strcmp(a, "--vera-slot"))   { vera_slot = atoi(ARG); vslot_given = 1; }
		else if (!strcmp(a, "--vera-irq"))    vera_irq = strcmp(ARG, "off") != 0;
		else if (!strcmp(a, "--phantom"))     { const char *m = ARG; phantom = !strcmp(m, "off") ? 0 : !strcmp(m, "65c02") ? 3 : 1; phantom_given = 1; }
		else if (!strcmp(a, "--aux"))         aux_kb = atoi(ARG);
		else if (!strcmp(a, "--model"))       { const char *m = ARG; model = !strcmp(m, "iie") ? 1 : !strcmp(m, "iiee") ? 2 : 0;
		                                        if (!model && strcmp(m, "ii+")) { fprintf(stderr, "a2run: --model ii+|iie|iiee\n"); return 2; } }
		else if (!strcmp(a, "--xmem"))        xmem_kb = atoi(ARG);
		else if (!strcmp(a, "--xmem-slot"))   xmem_slot = atoi(ARG);
		else if (!strcmp(a, "--xmem-image"))  xmem_img = ARG;
		else if (!strcmp(a, "--xmem-dump"))   xmem_dump = ARG;
		else if (!strcmp(a, "--debug-slot"))  dbg_slot = atoi(ARG);
		else if (!strcmp(a, "--shot-prefix")) shot_prefix = ARG;
		else if (!strcmp(a, "--shot-full"))   shot_full = 1;
		else if (!strcmp(a, "--shot-delay"))  shot_delay = atoi(ARG);
		else if (!strcmp(a, "--shot-exit"))   shot_exit = ARG;
		else if (!strcmp(a, "--vramdump"))    vramdump_path = ARG;
		else if (!strcmp(a, "--keys"))        keys = ARG;
		else if (!strcmp(a, "--keys-at"))     keys_at = atoi(ARG);
		else if (!strcmp(a, "--text"))        want_text = 1;
		else if (!strcmp(a, "--input"))       input = ARG;
		else if (!strcmp(a, "--snesmax"))     snesmax = atoi(ARG);
		else if (!strcmp(a, "--quiet"))       quiet = 1;
		else if (!strcmp(a, "--ym"))          { const char *m = ARG; ym_opt = !strcmp(m, "on") ? 1 : !strcmp(m, "off") ? 0 : -2;
		                                        if (ym_opt == -2) { fprintf(stderr, "a2run: --ym on|off\n"); return 2; } }
		else if (!strcmp(a, "--zpmin"))       { char *e; zpmin = (int)strtol(ARG, &e, 0) & 0xFF; if (*e == ',') zpmin_lo = (unsigned)strtol(e + 1, &e, 0);
		                                        if (*e == ',') { zpmin_top = (unsigned)strtol(e + 1, NULL, 0); zpmin_arm = 0; } }
		else if (!strcmp(a, "--ymlog"))       { if (!(ymlog = fopen(ARG, "w"))) { fprintf(stderr, "a2run: cannot write %s\n", argv[i]); return 2; } }
		else if (!strcmp(a, "--stop-pc"))     stop_pc = parse_addr(ARG);
		else if (!strcmp(a, "--pctrace"))     { pctrace = atoi(ARG); if (pctrace > 4096) pctrace = 4096; }
		else if (!strcmp(a, "--wav"))         { if (!a2audio_open_wav(ARG)) { fprintf(stderr, "a2run: cannot write %s\n", argv[i]); return 2; } }
		else if (!strcmp(a, "--psgvlog"))     { if (!a2audio_open_psgvlog(ARG)) { fprintf(stderr, "a2run: cannot write %s\n", argv[i]); return 2; } }
		else if (!strcmp(a, "--psglog"))      { if (!a2audio_open_psglog(ARG)) { fprintf(stderr, "a2run: cannot write %s\n", argv[i]); return 2; } }
		else if (a[0] != '-' && !disk1)       disk1 = a;
		else { fprintf(stderr, "a2run: unknown option %s (see the header of a2run.c)\n", a); return 2; }
	}
	(void)parse_num;

	crc_init();
	if (!read_rom(romdir, "appleII+.rom", rom, ROMSIZE) || !read_rom(romdir, "diskII.rom", sl6, SL6SIZE)) return 2;

	if (model && !a2_load_iie_roms(romdir, model == 2)) {
		fprintf(stderr, "a2run: IIe ROMs not found in %s (342-030x / 342-013x)\n", romdir); return 2; }
	if (model == 2) { g_cpu = 2; if (!phantom_given) phantom = 3; }   /* CPU_65C02, PH_65C02 */
	g_iie_aux = model && aux_kb >= 64;
	if (zpmin < 0 && zpenv) {
		char *e; zpmin = (int)strtol(zpenv, &e, 0) & 0xFF;
		if (*e == ',') zpmin_lo = (unsigned)strtol(e + 1, &e, 0);
		if (*e == ',') { zpmin_top = (unsigned)strtol(e + 1, NULL, 0); zpmin_arm = 0; }
	}
	vera_bus_init();                                  /* env defaults first, then the flags win */
	if (vera_slot >= 1 && vera_slot <= 7) g_vera_base = 0xC000 + vera_slot * 0x100;
	if (vera_slot == 0 && vslot_given) g_vera_enabled = 0;
	if (vera_irq >= 0) g_vera_irq = vera_irq;
	if (ym_opt >= 0) ym_init(ym_opt);
	if (ymlog) ym_log_open(ymlog);
	g_phantom = phantom;
	if (snesmax >= 0) g_snesmax_slot = snesmax;
	if (input && !a2in_load(input)) return 2;
	if (dbg_slot) g_dbg_slot = dbg_slot;
	a2_dbg_hook = dbg_hook;
	if (xmem_kb && !xmem_init(xmem_kb, xmem_slot)) { fprintf(stderr, "a2run: bad --xmem %d / slot %d\n", xmem_kb, xmem_slot); return 2; }
	if (xmem_img) {
		if (!g_xmem_slot) { fprintf(stderr, "a2run: --xmem-image needs --xmem\n"); return 2; }
		FILE *f = fopen(xmem_img, "rb");
		if (!f) { fprintf(stderr, "a2run: cannot open %s\n", xmem_img); return 2; }
		size_t n = fread(g_xmem, 1, g_xmem_size, f); fclose(f); (void)n;
	}

	if (disk1 && !insertFloppy(NULL, (char *)disk1, 0)) { fprintf(stderr, "a2run: bad disk image %s\n", disk1); return 2; }
	if (disk2 && !insertFloppy(NULL, (char *)disk2, 1)) { fprintf(stderr, "a2run: bad disk image %s\n", disk2); return 2; }
	if (hd && !insertHardDisk((char *)hd)) { fprintf(stderr, "a2run: cannot open %s\n", hd); return 2; }
	for (int i = 0; i < nloads; i++) if (!load_file(loads[i])) return 2;   /* after insertFloppy (it clears RAM) */

	a2_reset();
	if (pc >= 0) {
		Puce6502State s = { (uint16_t)pc, 0, 0, 0, 0xFF, 0x34 };
		puce6502SetState(&s);
	}

	unsigned long long cycles = 0;
	unsigned int t0 = ticks;
	long frames = 0; int last_frame = g_vera.frame_count;
	const char *kp = keys; int key_hold = 0;
	int code = 124;
	if (input) a2in_frame(0);

	unsigned long vs_last = vc_vsync_raised; long vs_n = 0;
	for (;;) {
		cpu_vera_step();
		if (vc_vsync_raised != vs_last) { vs_last = vc_vsync_raised; a2audio_vsync(++vs_n); }
		if (pctrace || stop_pc >= 0) {
			Puce6502State ps; puce6502GetState(&ps);
			pcring[pcpos++ & 4095] = ps.pc;
			if (ps.pc == stop_pc) { code = 125; break; }
		}
		unsigned int t = ticks; cycles += (unsigned int)(t - t0); t0 = t;
		if (zpmin >= 0) { unsigned v = ram[zpmin] | ram[(zpmin + 1) & 0xFF] << 8; if (v == zpmin_top) zpmin_arm = 1;
		                  if (zpmin_arm && v >= zpmin_lo && v < zpmin_v) zpmin_v = v; }
		if (g_vera.frame_count != last_frame) {           /* a VERA frame just completed */
			last_frame = g_vera.frame_count; frames++;
			ym_log_frame(frames);
			a2audio_frame(frames);
			if (input) a2in_frame(frames);
			if (shot_pending && --shot_pending == 0) {
				char p[1100]; snprintf(p, sizeof p, "%s%03d.png", shot_prefix, shot_n++);
				if (!vera_shot(p)) fprintf(stderr, "a2run: screenshot %s failed\n", p);
			}
			if (kp && *kp && frames >= keys_at && --key_hold <= 0 && !(KBD & 0x80)) {
				char ch = *kp++;
				if (ch == '~') key_hold = 120;
				else { KBD = (ch == '\n' ? 0x8D : ((uint8_t)ch & 0x7F) | 0x80); key_hold = 3; }
			}
			if (max_frames && frames >= max_frames) break;
		}
		if (g_dbg_exit >= 0 && !shot_pending) { code = g_dbg_exit; break; }   /* a pending shot completes first */
		if (max_cycles && cycles >= max_cycles) break;
	}
	fflush(stdout);
	a2audio_close();
	if (ymlog) fclose(ymlog);

	if (shot_pending) {                               /* a limit hit while a shot was pending */
		char p[1100]; snprintf(p, sizeof p, "%s%03d.png", shot_prefix, shot_n++);
		if (!vera_shot(p)) fprintf(stderr, "a2run: screenshot %s failed\n", p);
	}
	if (shot_exit && !vera_shot(shot_exit)) fprintf(stderr, "a2run: screenshot %s failed\n", shot_exit);
	if (vramdump_path && !vc_dump_state(vramdump_path)) fprintf(stderr, "a2run: VRAM dump failed\n");
	if (xmem_dump && g_xmem_slot) {
		FILE *f = fopen(xmem_dump, "wb");
		if (f) { fwrite(g_xmem, 1, g_xmem_size, f); fclose(f); }
	}
	if (writeback && !a2_flush_disks()) fprintf(stderr, "a2run: floppy write-back failed\n");
	if (want_text) text_dump();
	if (pctrace) {
		fprintf(stderr, "PCTRACE (oldest first):");
		for (int k = pctrace; k > 0; k--) fprintf(stderr, " %04X", pcring[(pcpos - k) & 4095]);
		fprintf(stderr, "\n");
	}
	if (!quiet) {
		Puce6502State s; puce6502GetState(&s);
		fprintf(stderr, "A2RUN %s code=%d cycles=%llu frames=%ld pc=$%04X\n",
		        code == 124 && g_dbg_exit < 0 ? "LIMIT" : "EXIT", code, cycles, frames, s.pc);
		if (zpmin >= 0) fprintf(stderr, "A2RUN ZPMIN $%02X = $%04X\n", zpmin, zpmin_v);
		if (g_ym_writes || g_ym_busy_writes)
			fprintf(stderr, "A2RUN YM writes=%lu busy=%lu\n", g_ym_writes, g_ym_busy_writes);
	}
	return code;
}
