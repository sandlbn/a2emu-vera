/*
 * vera.c -- Commander X16 VERA video/audio module emulation
 *
 * Complete port of the X16 emulator VERA implementation for the
 * Agfa 9000PS emulator. VERA replaces VIA #2 at 0x04000020-0x0400003F.
 *
 * Original source: x16-emulator (https://github.com/X16Community/x16-emulator)
 * Copyright (c) 2019 Michael Steil
 * Copyright (c) 2020 Frank van den Hoef
 * All rights reserved. License: 2-clause BSD
 *
 * Includes: video rendering (layers, sprites, compositor), FX extensions,
 * PSG synthesizer (16 channels), PCM audio with FIFO, SPI with SD card.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
int g_vera_reset_writes = 0;
typedef struct { uint16_t pc; uint8_t a,x,y,sp,p; } Puce6502State;
void puce6502GetState(Puce6502State*);
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <limits.h>
#include <math.h>

#include "veracore.h"
#include "sdcard.h"

/* ================================================================
 * Build configuration
 * ================================================================ */

#define VC_VERSION_MAJOR  47
#define VC_VERSION_MINOR  0
#define VC_VERSION_PATCH  2

/* VRAM address ranges */
#define ADDR_VRAM_START     0x00000
#define ADDR_VRAM_END       0x20000
#define ADDR_PSG_START      0x1F9C0
#define ADDR_PSG_END        0x1FA00
#define ADDR_PALETTE_START  0x1FA00
#define ADDR_PALETTE_END    0x1FC00
#define ADDR_SPRDATA_START  0x1FC00
#define ADDR_SPRDATA_END    0x20000

#define NUM_SPRITES 128

/* VGA timing */
#define SCAN_HEIGHT     525
#define PIXEL_FREQ      25.0f

#define VGA_SCAN_WIDTH  800
#define VGA_Y_OFFSET    0

/* NTSC timing */
#define NTSC_HALF_SCAN_WIDTH 794
#define NTSC_X_OFFSET        270
#define NTSC_Y_OFFSET_LOW    42
#define NTSC_Y_OFFSET_HIGH   568
/* composite: next-line strobe (pixel clocks), line counter restarts (half-lines) */
#define NTSC_NEXT_LINE       235
#define NTSC_FRAME_START_1   42
#define NTSC_FRAME_START_2   568
#define TITLE_SAFE_X         0.067f
#define TITLE_SAFE_Y         0.05f

/* Visible area */
#define SCREEN_WIDTH    640
#define SCREEN_HEIGHT   480

/* Rendering optimization */
#define LAYER_PIXELS_PER_ITERATION 8

#define MAX(a,b) ((a) > (b) ? (a) : (b))

/* ================================================================
 * PSG (Programmable Sound Generator) — 16 channels
 * From vera_psg.c, Copyright (c) 2020 Frank van den Hoef, 2-clause BSD
 * ================================================================ */

enum psg_waveform {
    WF_PULSE = 0,
    WF_SAWTOOTH,
    WF_TRIANGLE,
    WF_NOISE,
};

struct psg_channel {
    uint16_t freq;
    uint16_t volume;
    bool     left, right;
    uint8_t  pw;
    uint8_t  waveform;
    uint16_t noiseval;
    uint32_t phase;
};

static struct psg_channel psg_channels[16];

static uint16_t psg_volume_lut[64] = {
      0,                                           4,   8,  12,
     16,  17,  18,  20,  21,  22,  23,  25,  26,  28,  30,  31,
     33,  35,  37,  40,  42,  45,  47,  50,  53,  56,  60,  63,
     67,  71,  75,  80,  85,  90,  95, 101, 107, 113, 120, 127,
    135, 143, 151, 160, 170, 180, 191, 202, 214, 227, 241, 255,
    270, 286, 303, 321, 341, 361, 382, 405, 429, 455, 482, 511
};

static uint16_t psg_noise_state;

static void
psg_reset(void)
{
    memset(psg_channels, 0, sizeof(psg_channels));
    psg_noise_state = 1;
}

uint8_t vc_psg_peak[16];
uint8_t vc_dbg_psg_peak(int ch){ return vc_psg_peak[ch & 15]; }
static void audio_render(void);
static void
psg_writereg(uint8_t reg, uint8_t val)
{
    audio_render();
    reg &= 0x3f;
    int ch  = reg / 4;
    int idx = reg & 3;

    switch (idx) {
        case 0: psg_channels[ch].freq = (psg_channels[ch].freq & 0xFF00) | val; break;
        case 1: psg_channels[ch].freq = (psg_channels[ch].freq & 0x00FF) | (val << 8); break;
        case 2:
            psg_channels[ch].right  = (val & 0x80) != 0;
            psg_channels[ch].left   = (val & 0x40) != 0;
            psg_channels[ch].volume = psg_volume_lut[val & 0x3F];
            /* End-of-run volume cannot distinguish "silent" from "played and
               stopped" -- the same single-sample trap that made L0 look off on
               eight titles. Latch the peak volume each voice ever carried. */
            if ((val & 0x3F) > vc_psg_peak[ch]) vc_psg_peak[ch] = val & 0x3F;
            break;
        case 3:
            psg_channels[ch].pw       = val & 0x3F;
            psg_channels[ch].waveform = val >> 6;
            break;
    }
}

static void
psg_render(int16_t *buf, unsigned num_samples)
{
    while (num_samples--) {
        int16_t l = 0;
        int16_t r = 0;

        for (int i = 0; i < 16; i++) {
            psg_noise_state = (psg_noise_state << 1) |
                (((psg_noise_state >> 1) ^ (psg_noise_state >> 2) ^
                  (psg_noise_state >> 4) ^ (psg_noise_state >> 15)) & 1);

            struct psg_channel *ch = &psg_channels[i];

            uint32_t new_phase = (ch->left || ch->right) ? ((ch->phase + ch->freq) & 0x1FFFF) : 0;
            if ((ch->phase & 0x10000) && !(new_phase & 0x10000)) {
                ch->noiseval = (psg_noise_state >> 1) & 0x3F;
            }
            ch->phase = new_phase;

            uint32_t v = 0;
            switch (ch->waveform) {
                case WF_PULSE:    v = ((ch->phase >> 10) > ch->pw) ? 0 : 0x3F; break;
                case WF_SAWTOOTH: v = (ch->phase >> 11) ^ ((ch->pw ^ 0x3f) & 0x3f); break;
                case WF_TRIANGLE: v = ((ch->phase & 0x10000) ? (~(ch->phase >> 10) & 0x3F) : ((ch->phase >> 10) & 0x3F)) ^ ((ch->pw ^ 0x3f) & 0x3f); break;
                case WF_NOISE:    v = ch->noiseval; break;
            }
            int16_t sv = (v ^ 0x20);
            if (sv & 0x20) sv |= 0xFFC0;

            int16_t val = sv * ch->volume;
            if (ch->left)  l += val >> 3;
            if (ch->right) r += val >> 3;
        }

        *buf++ = l;
        *buf++ = r;
    }
}

/* ================================================================
 * PCM Audio — FIFO-based sample playback
 * From vera_pcm.c, Copyright (c) 2020 Frank van den Hoef, 2-clause BSD
 * ================================================================ */

static uint8_t  pcm_fifo[4096];
static unsigned pcm_fifo_wridx;
static unsigned pcm_fifo_rdidx;
static unsigned pcm_fifo_cnt;

static uint8_t pcm_ctrl;
static uint8_t pcm_rate;
static uint8_t pcm_rate_reg;    /* as written, reads back unclamped */
static uint8_t pcm_loop;

static uint8_t pcm_volume_lut[16] = {0, 1, 2, 3, 4, 5, 6, 8, 11, 14, 18, 23, 30, 38, 49, 64};

static int16_t pcm_cur_l, pcm_cur_r;
static uint8_t pcm_phase;

static void pcm_fifo_reset(void)
{
    pcm_fifo_wridx = 0;
    pcm_fifo_rdidx = 0;
    pcm_fifo_cnt   = 0;
}

static void pcm_fifo_restart(void)
{
    pcm_fifo_rdidx = 0;
    pcm_fifo_cnt = pcm_fifo_wridx;
}

static void pcm_reset(void)
{
    pcm_fifo_reset();
    pcm_ctrl  = 0;
    pcm_rate  = 0;
    pcm_rate_reg = 0;
    pcm_cur_l = 0;
    pcm_cur_r = 0;
    pcm_phase = 0;
}

static void pcm_write_ctrl(uint8_t val)
{
    if ((val & 0xc0) == 0xc0) {
        pcm_loop = true;
    } else {
        pcm_loop = false;
        if (val & 0x80) pcm_fifo_reset();
    }
    if (val & 0x40) pcm_fifo_restart();
    pcm_ctrl = val & 0x3F;
}

static uint8_t pcm_read_ctrl(void)
{
    uint8_t result = pcm_ctrl;
    if (pcm_fifo_cnt == sizeof(pcm_fifo) - 1) result |= 0x80;
    if (pcm_fifo_cnt == 0) result |= 0x40;
    return result;
}

static void pcm_write_rate(uint8_t val)
{
    pcm_rate_reg = val;
    pcm_rate = (val > 128) ? (256 - val) : val;
}

static uint8_t pcm_read_rate(void)
{
    return pcm_rate_reg;
}

static int pcm_total_writes = 0;

static void pcm_write_fifo(uint8_t val)
{
    pcm_total_writes++;
    if (pcm_fifo_cnt < sizeof(pcm_fifo) - 1) {
        pcm_fifo[pcm_fifo_wridx++] = val;
        if (pcm_fifo_wridx == sizeof(pcm_fifo)) pcm_fifo_wridx = 0;
        pcm_fifo_cnt++;
    }
}

static uint8_t pcm_read_fifo(void)
{
    if (pcm_fifo_cnt == 0) return 0;
    uint8_t result = pcm_fifo[pcm_fifo_rdidx++];
    if (pcm_fifo_rdidx == sizeof(pcm_fifo)) pcm_fifo_rdidx = 0;
    pcm_fifo_cnt--;
    return result;
}

static bool pcm_is_fifo_almost_empty(void)
{
    return pcm_fifo_cnt < 1024;
}

extern unsigned int ticks;

#define AUD_RING 8192
#define AUD_LAG  4

static int16_t  aud_ring[AUD_RING][2];
static unsigned aud_rd, aud_wr, aud_last;
static unsigned long long aud_acc;

static void audio_sample(int16_t *out)
{
    int16_t psg[2];
    psg_render(psg, 1);

    if (pcm_rate > 0) {
        pcm_phase += pcm_rate;
        if (pcm_phase >= 128) {
            pcm_phase -= 128;
            if (pcm_fifo_cnt > 0) {
                switch (pcm_ctrl & 0x30) {
                    case 0x00:
                        pcm_cur_l = pcm_cur_r = (int8_t)pcm_read_fifo() * 256;
                        break;
                    case 0x10:
                        pcm_cur_l = (int8_t)pcm_read_fifo() * 256;
                        pcm_cur_r = (int8_t)pcm_read_fifo() * 256;
                        break;
                    case 0x20:
                        pcm_cur_l  = pcm_read_fifo();
                        pcm_cur_l |= pcm_read_fifo() << 8;
                        pcm_cur_r  = pcm_cur_l;
                        break;
                    case 0x30:
                        pcm_cur_l  = pcm_read_fifo();
                        pcm_cur_l |= pcm_read_fifo() << 8;
                        pcm_cur_r  = pcm_read_fifo();
                        pcm_cur_r |= pcm_read_fifo() << 8;
                        break;
                }
            } else if (pcm_loop) {
                pcm_fifo_restart();
            }
        }
    }

    int vol = pcm_volume_lut[pcm_ctrl & 0x0F];
    int l = psg[0] + ((pcm_cur_l * vol) >> 6);
    int r = psg[1] + ((pcm_cur_r * vol) >> 6);
    out[0] = l < -32768 ? -32768 : l > 32767 ? 32767 : l;
    out[1] = r < -32768 ? -32768 : r > 32767 ? 32767 : r;
}

/* render the samples owed up to the current CPU cycle (25 MHz / 512 per sample) */
static void audio_render(void)
{
    unsigned d = ticks - aud_last;
    aud_last = ticks;
    aud_acc += (unsigned long long)d * 48828125ull;
    while (aud_acc >= 1020484000ull) {
        aud_acc -= 1020484000ull;
        if (aud_wr - aud_rd >= AUD_RING) aud_rd++;
        audio_sample(aud_ring[aud_wr++ % AUD_RING]);
    }
}

static void audio_reset(void)
{
    memset(aud_ring, 0, sizeof(aud_ring));
    aud_rd = 0;
    aud_wr = AUD_LAG;
    aud_last = ticks;
    aud_acc = 0;
}

/* vc_audio_render — the PSG + PCM samples up to now, 48,828 Hz stereo */
void vc_audio_render(int16_t *buf, int num_samples)
{
    audio_render();
    for (int i = 0; i < num_samples; i++) {
        unsigned k = (aud_wr - aud_rd > 0 ? aud_rd++ : aud_wr - 1) % AUD_RING;
        buf[i * 2 + 0] = aud_ring[k][0];
        buf[i * 2 + 1] = aud_ring[k][1];
    }
    if (aud_wr - aud_rd > 48828 / 30) aud_rd = aud_wr - AUD_LAG;
}

/* ================================================================
 * SPI master and SD card slot
 * From vera_spi.c, Copyright (c) 2019 Michael Steil, 2-clause BSD
 * ================================================================ */

/* a byte takes 8 SPI clocks: 12.5 MHz is under one CPU cycle, 390 kHz about 21 */
#define SPI_CYCLES_FAST 1
#define SPI_CYCLES_SLOW 21

static bool     spi_ss;
static bool     spi_busy;
static bool     spi_autotx;
static bool     spi_slow;
static uint8_t  spi_sending_byte;
static uint8_t  spi_received_byte;
static unsigned spi_start;

static void vera_spi_init(void)
{
    if (spi_ss) sdcard_select(false);
    spi_ss = false;
    spi_busy = false;
    spi_autotx = false;
    spi_slow = false;
    spi_received_byte = 0xff;
}

static void spi_start_byte(uint8_t b)
{
    spi_sending_byte = b;
    spi_busy = true;
    spi_start = ticks;
}

static void spi_update(void)
{
    if (spi_busy && ticks - spi_start >= (spi_slow ? SPI_CYCLES_SLOW : SPI_CYCLES_FAST)) {
        spi_busy = false;
        spi_received_byte = sdcard_attached ? sdcard_handle(spi_sending_byte) : 0xff;
    }
}

static uint8_t vera_spi_read(uint8_t reg)
{
    spi_update();
    switch (reg) {
        case 0: {
            uint8_t b = spi_received_byte;
            if (spi_autotx && !spi_busy) spi_start_byte(0xff);
            return b;
        }
        case 1:
            return (spi_busy << 7) | (spi_autotx << 2) | (spi_slow << 1) | spi_ss;
    }
    return 0;
}

static void vera_spi_write(uint8_t reg, uint8_t value)
{
    spi_update();
    switch (reg) {
        case 0:
            if (!spi_busy) spi_start_byte(value);
            break;
        case 1:
            if (spi_ss != (value & 1)) {
                spi_ss = value & 1;
                sdcard_select(spi_ss);
            }
            spi_slow = !!(value & 2);
            spi_autotx = !!(value & 4);
            break;
    }
}

/* ================================================================
 * VERA Video Core — global state (matches X16 emulator structure)
 * ================================================================ */

static uint8_t video_ram[0x20000];
static uint8_t palette[256 * 2];
static uint8_t sprite_data[128][8];

/* I/O registers */
static uint32_t io_addr[2];
static uint8_t  io_rddata[2];
static uint8_t  io_inc[2];
static uint8_t  io_addrsel;
static uint8_t  io_dcsel;

static uint8_t ien;
unsigned long vc_vsync_raised = 0;   /* how often the core actually raises vsync */
/* IEN write trace: a game write landing on $C406 disarms VSYNC for good and
   starves the port of frames -- icehockey gets 7 NMIs where volguard gets
   1575 (tick 17). Counting them makes that visible. */
int vc_ien_writes; unsigned char vc_ien_last; int vc_ien_disarms;
static uint8_t isr;
static uint16_t irq_line;

static uint8_t reg_layer[2][7];

#define COMPOSER_SLOTS (4*64)
static uint8_t reg_composer[COMPOSER_SLOTS];
static uint8_t prev_reg_composer[2][COMPOSER_SLOTS];
/* read-only debug accessor: DC_VIDEO (composer reg 0) as the device currently holds it */
uint8_t vc_dbg_dc_video(void) { return reg_composer[0]; }
/* ADDRSEL/DCSEL readback. $C405 bit0 selects whether $C400-$C402 program ADDR0
   or ADDR1, while DATA0 always uses ADDR0 -- so a stray ADDRSEL=1 makes every
   resident write program one pointer and write through another, silently. */
unsigned long vc_wr_bucket[16];
unsigned long vc_wr_nz[16];
int vc_l0_ever = 0;
int vc_dbg_l0_ever(void) { return vc_l0_ever; }
uint8_t vc_dbg_ctrl(void) { return (uint8_t)((io_dcsel << 1) | io_addrsel); }
/* Layer register readback for headless verification: the port programs L0 via
   $C40D-$C413 and there was no way to confirm what actually landed. */
uint8_t vc_dbg_layer(int l, int r) { return reg_layer[l & 1][r % 7]; }

static uint8_t layer_line[2][SCREEN_WIDTH];
static uint8_t sprite_line_col[SCREEN_WIDTH];
static uint8_t sprite_line_z[SCREEN_WIDTH];
static uint8_t sprite_line_mask[SCREEN_WIDTH];
static uint8_t sprite_line_collisions;
static bool layer_line_enable[2];
static bool old_layer_line_enable[2];
static bool old_sprite_line_enable;
static bool sprite_line_enable;

/* ---- FX registers ---- */
static uint8_t  fx_addr1_mode;
static uint32_t fx_x_pixel_increment;
static uint32_t fx_y_pixel_increment;
static uint32_t fx_x_pixel_position;
static uint32_t fx_y_pixel_position;
static uint16_t fx_poly_fill_length;
static uint32_t fx_affine_tile_base;
static uint32_t fx_affine_map_base;
static uint8_t  fx_affine_map_size;
static bool fx_4bit_mode;
static bool fx_16bit_hop;
static bool fx_cache_byte_cycling;
static bool fx_cache_fill;
static bool fx_cache_write;
static bool fx_trans_writes;
static bool fx_2bit_poly;
static bool fx_2bit_poking;
static bool fx_cache_increment_mode;
static bool fx_cache_nibble_index;
static uint8_t fx_cache_byte_index;
static bool fx_multiplier;
static bool fx_subtract;
static bool fx_affine_clip;
static uint8_t fx_16bit_hop_align;
static bool fx_nibble_bit[2];
static bool fx_nibble_incr[2];
static uint8_t fx_cache[4];
static int32_t fx_mult_accumulator;

static const uint8_t vera_version_string[] = {
    'V', VC_VERSION_MAJOR, VC_VERSION_MINOR, VC_VERSION_PATCH
};

/* Scan position state */
static float    vga_scan_pos_x;
static uint16_t vga_scan_pos_y;
int vc_norender = 0;             /* set to skip the scanline renderer (measurements): vsync still fires, framebuffer isn't drawn */
static float    ntsc_half_cnt;
static uint16_t ntsc_scan_pos_y;
static uint16_t ntsc_y_counter, ntsc_y_counter_prev;  /* composer line counter, steps by 2 */
static bool     ntsc_field;
static int      frame_count;

static uint8_t framebuffer[SCREEN_WIDTH * SCREEN_HEIGHT * 4];

/* Pointer to the vc_t struct's framebuffer (set during tick/render) */
static vc_t *active_vera;

/* ================================================================
 * Default palette (256 entries, 12-bit RGB)
 * ================================================================ */

static const uint16_t default_palette[] = {
0x000,0xfff,0x800,0xafe,0xc4c,0x0c5,0x00a,0xee7,0xd85,0x640,0xf77,0x333,0x777,0xaf6,0x08f,0xbbb,
0x000,0x111,0x222,0x333,0x444,0x555,0x666,0x777,0x888,0x999,0xaaa,0xbbb,0xccc,0xddd,0xeee,0xfff,
0x211,0x433,0x644,0x866,0xa88,0xc99,0xfbb,0x211,0x422,0x633,0x844,0xa55,0xc66,0xf77,0x200,0x411,
0x611,0x822,0xa22,0xc33,0xf33,0x200,0x400,0x600,0x800,0xa00,0xc00,0xf00,0x221,0x443,0x664,0x886,
0xaa8,0xcc9,0xfeb,0x211,0x432,0x653,0x874,0xa95,0xcb6,0xfd7,0x210,0x431,0x651,0x862,0xa82,0xca3,
0xfc3,0x210,0x430,0x640,0x860,0xa80,0xc90,0xfb0,0x121,0x343,0x564,0x786,0x9a8,0xbc9,0xdfb,0x121,
0x342,0x463,0x684,0x8a5,0x9c6,0xbf7,0x120,0x241,0x461,0x582,0x6a2,0x8c3,0x9f3,0x120,0x240,0x360,
0x480,0x5a0,0x6c0,0x7f0,0x121,0x343,0x465,0x686,0x8a8,0x9ca,0xbfc,0x121,0x242,0x364,0x485,0x5a6,
0x6c8,0x7f9,0x020,0x141,0x162,0x283,0x2a4,0x3c5,0x3f6,0x020,0x041,0x061,0x082,0x0a2,0x0c3,0x0f3,
0x122,0x344,0x466,0x688,0x8aa,0x9cc,0xbff,0x122,0x244,0x366,0x488,0x5aa,0x6cc,0x7ff,0x022,0x144,
0x166,0x288,0x2aa,0x3cc,0x3ff,0x022,0x044,0x066,0x088,0x0aa,0x0cc,0x0ff,0x112,0x334,0x456,0x668,
0x88a,0x9ac,0xbcf,0x112,0x224,0x346,0x458,0x56a,0x68c,0x79f,0x002,0x114,0x126,0x238,0x24a,0x35c,
0x36f,0x002,0x014,0x016,0x028,0x02a,0x03c,0x03f,0x112,0x334,0x546,0x768,0x98a,0xb9c,0xdbf,0x112,
0x324,0x436,0x648,0x85a,0x96c,0xb7f,0x102,0x214,0x416,0x528,0x62a,0x83c,0x93f,0x102,0x204,0x306,
0x408,0x50a,0x60c,0x70f,0x212,0x434,0x646,0x868,0xa8a,0xc9c,0xfbe,0x211,0x423,0x635,0x847,0xa59,
0xc6b,0xf7d,0x201,0x413,0x615,0x826,0xa28,0xc3a,0xf3c,0x201,0x403,0x604,0x806,0xa08,0xc09,0xf0b
};

/* ================================================================
 * Palette lookup cache
 * ================================================================ */

struct video_palette {
    uint32_t entries[256];
    bool dirty;
};

static struct video_palette video_palette;

static void
refresh_palette(void)
{
    const uint8_t out_mode = reg_composer[0] & 3;
    const bool chroma_disable = ((reg_composer[0] & 0x07) == 6);
    for (int i = 0; i < 256; ++i) {
        uint8_t r, g, b;
        if (out_mode == 0) {
            r = 0; g = 0; b = 255;
        } else {
            uint16_t entry = palette[i * 2] | (palette[i * 2 + 1] << 8);
            r = ((entry >> 8) & 0xf) << 4 | ((entry >> 8) & 0xf);
            g = ((entry >> 4) & 0xf) << 4 | ((entry >> 4) & 0xf);
            b = (entry & 0xf) << 4 | (entry & 0xf);
            if (chroma_disable) {
                r = g = b = (r + b + g) / 3;
            }
        }
        /* BGRA format for SDL_PIXELFORMAT_RGB888 */
        video_palette.entries[i] = (uint32_t)(r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
    }
    video_palette.dirty = false;
}

/* ================================================================
 * Layer properties
 * ================================================================ */

struct video_layer_properties {
    uint8_t  color_depth;
    uint32_t map_base;
    uint32_t tile_base;

    bool text_mode;
    bool text_mode_256c;
    bool tile_mode;
    bool bitmap_mode;

    uint16_t hscroll;
    uint16_t vscroll;

    uint8_t  mapw_log2;
    uint8_t  maph_log2;
    uint16_t tilew;
    uint16_t tileh;
    uint8_t  tilew_log2;
    uint8_t  tileh_log2;

    uint16_t mapw_max;
    uint16_t maph_max;
    uint16_t tilew_max;
    uint16_t tileh_max;
    uint16_t layerw_max;
    uint16_t layerh_max;

    uint8_t  tile_size_log2;

    int min_eff_x;
    int max_eff_x;

    uint8_t bits_per_pixel;
    uint8_t first_color_pos;
    uint8_t color_mask;
    uint8_t color_fields_max;
};

#define NUM_LAYERS 2
static struct video_layer_properties layer_properties[NUM_LAYERS];
static struct video_layer_properties prev_layer_properties[2][NUM_LAYERS];

static int
calc_layer_eff_x(const struct video_layer_properties *props, const int x)
{
    return (x + props->hscroll) & (props->layerw_max);
}

static int
calc_layer_eff_y(const struct video_layer_properties *props, const int y)
{
    return (y + props->vscroll) & (props->layerh_max);
}

static uint32_t
calc_layer_map_addr_base2(const struct video_layer_properties *props, const int eff_x, const int eff_y)
{
    return props->map_base + ((((eff_y >> props->tileh_log2) << props->mapw_log2) + (eff_x >> props->tilew_log2)) << 1);
}

static void
refresh_layer_properties(const uint8_t layer)
{
    struct video_layer_properties *props = &layer_properties[layer];

    uint16_t prev_layerw_max = props->layerw_max;
    uint16_t prev_hscroll = props->hscroll;

    props->color_depth    = reg_layer[layer][0] & 0x3;
    props->map_base       = reg_layer[layer][1] << 9;
    props->tile_base      = (reg_layer[layer][2] & 0xFC) << 9;
    props->bitmap_mode    = (reg_layer[layer][0] & 0x4) != 0;
    props->text_mode      = (props->color_depth == 0) && !props->bitmap_mode;
    props->text_mode_256c = (reg_layer[layer][0] & 8) != 0;
    props->tile_mode      = !props->bitmap_mode && !props->text_mode;

    if (!props->bitmap_mode) {
        props->hscroll = reg_layer[layer][3] | (reg_layer[layer][4] & 0xf) << 8;
        props->vscroll = reg_layer[layer][5] | (reg_layer[layer][6] & 0xf) << 8;
    } else {
        props->hscroll = 0;
        props->vscroll = 0;
    }

    uint16_t mapw = 0;
    uint16_t maph = 0;
    props->tilew = 0;
    props->tileh = 0;

    if (props->tile_mode || props->text_mode) {
        props->mapw_log2 = 5 + ((reg_layer[layer][0] >> 4) & 3);
        props->maph_log2 = 5 + ((reg_layer[layer][0] >> 6) & 3);
        mapw = 1 << props->mapw_log2;
        maph = 1 << props->maph_log2;

        props->tilew_log2 = 3 + (reg_layer[layer][2] & 1);
        props->tileh_log2 = 3 + ((reg_layer[layer][2] >> 1) & 1);
        props->tilew      = 1 << props->tilew_log2;
        props->tileh      = 1 << props->tileh_log2;
    } else if (props->bitmap_mode) {
        props->tilew = (reg_layer[layer][2] & 1) ? 640 : 320;
        props->tileh = SCREEN_HEIGHT;
        mapw = 1;
        maph = 1;
    }

    props->mapw_max   = mapw - 1;
    props->maph_max   = maph - 1;
    props->tilew_max  = props->tilew - 1;
    props->tileh_max  = props->tileh - 1;
    props->layerw_max = (mapw * props->tilew) - 1;
    props->layerh_max = (maph * props->tileh) - 1;

    if (prev_layerw_max != props->layerw_max || prev_hscroll != props->hscroll) {
        int min_eff_x = INT_MAX;
        int max_eff_x = INT_MIN;
        for (int x = 0; x < SCREEN_WIDTH; ++x) {
            int eff_x = calc_layer_eff_x(props, x);
            if (eff_x < min_eff_x) min_eff_x = eff_x;
            if (eff_x > max_eff_x) max_eff_x = eff_x;
        }
        props->min_eff_x = min_eff_x;
        props->max_eff_x = max_eff_x;
    }

    props->bits_per_pixel  = 1 << props->color_depth;
    props->tile_size_log2  = props->tilew_log2 + props->tileh_log2 + props->color_depth - 3;
    props->first_color_pos = 8 - props->bits_per_pixel;
    props->color_mask      = (1 << props->bits_per_pixel) - 1;
    props->color_fields_max = (8 >> props->color_depth) - 1;
}

/* ================================================================
 * Sprite properties
 * ================================================================ */

struct video_sprite_properties {
    int8_t   sprite_zdepth;
    uint8_t  sprite_collision_mask;
    int16_t  sprite_x;
    int16_t  sprite_y;
    uint8_t  sprite_width_log2;
    uint8_t  sprite_height_log2;
    uint8_t  sprite_width;
    uint8_t  sprite_height;
    bool     hflip;
    bool     vflip;
    uint8_t  color_mode;
    uint32_t sprite_address;
    uint16_t palette_offset;
};

static struct video_sprite_properties sprite_properties[128];

static void
refresh_sprite_properties(const uint16_t sprite)
{
    struct video_sprite_properties *props = &sprite_properties[sprite];

    props->sprite_zdepth         = (sprite_data[sprite][6] >> 2) & 3;
    props->sprite_collision_mask = sprite_data[sprite][6] & 0xf0;

    props->sprite_x = sprite_data[sprite][2] | (sprite_data[sprite][3] & 3) << 8;
    props->sprite_y = sprite_data[sprite][4] | (sprite_data[sprite][5] & 3) << 8;
    props->sprite_width_log2  = (((sprite_data[sprite][7] >> 4) & 3) + 3);
    props->sprite_height_log2 = ((sprite_data[sprite][7] >> 6) + 3);
    props->sprite_width       = 1 << props->sprite_width_log2;
    props->sprite_height      = 1 << props->sprite_height_log2;

    if (props->sprite_x >= 0x400 - props->sprite_width)
        props->sprite_x -= 0x400;
    if (props->sprite_y >= 0x400 - props->sprite_height)
        props->sprite_y -= 0x400;

    props->hflip = sprite_data[sprite][6] & 1;
    props->vflip = (sprite_data[sprite][6] >> 1) & 1;

    props->color_mode     = (sprite_data[sprite][1] >> 7) & 1;
    props->sprite_address = sprite_data[sprite][0] << 5 | (sprite_data[sprite][1] & 0xf) << 13;
    props->palette_offset = (sprite_data[sprite][7] & 0x0f) << 4;
}

/* ================================================================
 * VRAM access helpers
 * ================================================================ */

static uint8_t
video_space_read(uint32_t address)
{
    return video_ram[address & 0x1FFFF];
}

static void
video_space_read_range(uint8_t *dest, uint32_t address, uint32_t size)
{
    if (address >= ADDR_VRAM_START && (address + size) <= ADDR_VRAM_END) {
        memcpy(dest, &video_ram[address], size);
    } else {
        for (uint32_t i = 0; i < size; ++i)
            *dest++ = video_space_read(address + i);
    }
}

static void
video_space_write(uint32_t address, uint8_t value)
{
    video_ram[address & 0x1FFFF] = value;

    if (address >= ADDR_PSG_START && address < ADDR_PSG_END) {
        psg_writereg(address & 0x3f, value);
    } else if (address >= ADDR_PALETTE_START && address < ADDR_PALETTE_END) {
        palette[address & 0x1ff] = value;
        video_palette.dirty = true;
    } else if (address >= ADDR_SPRDATA_START && address < ADDR_SPRDATA_END) {
        sprite_data[(address >> 3) & 0x7f][address & 0x7] = value;
        refresh_sprite_properties((address >> 3) & 0x7f);
    }
}

static void
fx_video_space_write(uint32_t address, bool nibble, uint8_t value)
{
    if (fx_4bit_mode) {
        if (nibble) {
            if (!fx_trans_writes || (value & 0x0f) > 0)
                video_ram[address & 0x1FFFF] = (video_ram[address & 0x1FFFF] & 0xf0) | (value & 0x0f);
        } else {
            if (!fx_trans_writes || (value & 0xf0) > 0)
                video_ram[address & 0x1FFFF] = (video_ram[address & 0x1FFFF] & 0x0f) | (value & 0xf0);
        }
    } else {
        if (!fx_trans_writes || value > 0)
            video_ram[address & 0x1FFFF] = value;
    }
    /* Log tilemap writes (char+attr pairs in first 16KB) */
    if ((address & 0x1FFFF) < 0x4000) {
        static FILE *tilelog = NULL;
        static int tcount = 0;
        if (!tilelog) tilelog = fopen("/tmp/vera_tile.txt", "w");
        if (tilelog && tcount < 50000) {
            int row = (address & 0x1FFFF) / 256;
            int col = ((address & 0x1FFFF) % 256) / 2;
            int is_attr = (address & 1);
            fprintf(tilelog, "[%02d,%02d] %s=0x%02X addr=%05X\n", row, col, is_attr ? "ATR" : "CHR", value, address & 0x1FFFF);
            tcount++;
            fflush(tilelog);
        }
    }
    if (address >= ADDR_PSG_START && address < ADDR_PSG_END) {
        psg_writereg(address & 0x3f, value);
    } else if (address >= ADDR_PALETTE_START && address < ADDR_PALETTE_END) {
        palette[address & 0x1ff] = value;
        video_palette.dirty = true;
    } else if (address >= ADDR_SPRDATA_START && address < ADDR_SPRDATA_END) {
        sprite_data[(address >> 3) & 0x7f][address & 0x7] = value;
        refresh_sprite_properties((address >> 3) & 0x7f);
    }
}

static void
fx_vram_cache_write(uint32_t address, uint8_t value, uint8_t mask)
{
    if (!fx_trans_writes || value > 0) {
        switch (mask) {
            case 0: video_ram[address & 0x1FFFF] = value; break;
            case 1: video_ram[address & 0x1FFFF] = (video_ram[address & 0x1FFFF] & 0x0f) | (value & 0xf0); break;
            case 2: video_ram[address & 0x1FFFF] = (video_ram[address & 0x1FFFF] & 0xf0) | (value & 0x0f); break;
            case 3: break; /* do nothing */
        }
    }
}

/* ================================================================
 * Address auto-increment logic (with FX extensions)
 * ================================================================ */

static const int increments[32] = {
    0,   0,
    1,   -1,
    2,   -2,
    4,   -4,
    8,   -8,
    16,  -16,
    32,  -32,
    64,  -64,
    128, -128,
    256, -256,
    512, -512,
    40,  -40,
    80,  -80,
    160, -160,
    320, -320,
    640, -640,
};

static void fx_affine_prefetch(void);

static uint32_t
get_and_inc_address(uint8_t sel, bool write)
{
    uint32_t address = io_addr[sel];
    int16_t incr = increments[io_inc[sel]];

    /* DATA1 reads in polygon mode don't step while 2-bit poke mode is armed */
    if (sel == 1 && fx_addr1_mode == 2 && fx_2bit_poking && !write)
        return address;

    if (fx_4bit_mode && fx_nibble_incr[sel] && !incr) {
        if (fx_nibble_bit[sel]) {
            if ((io_inc[sel] & 1) == 0) io_addr[sel] += 1;
            fx_nibble_bit[sel] = 0;
        } else {
            if (io_inc[sel] & 1) io_addr[sel] -= 1;
            fx_nibble_bit[sel] = 1;
        }
    }

    if (sel == 1 && fx_16bit_hop) {
        if (incr == 4) {
            incr = (fx_16bit_hop_align == (address & 0x3)) ? 1 : 3;
        } else if (incr == 320) {
            incr = (fx_16bit_hop_align == (address & 0x3)) ? 1 : 319;
        }
    }

    io_addr[sel] += incr;

    if (sel == 1 && fx_addr1_mode == 1) { /* FX line draw mode */
        fx_x_pixel_position += fx_x_pixel_increment;
        if (fx_x_pixel_position & 0x10000) {
            fx_x_pixel_position &= ~0x10000;
            if (fx_4bit_mode && fx_nibble_incr[0]) {
                if (fx_nibble_bit[1]) {
                    if ((io_inc[0] & 1) == 0) io_addr[1] += 1;
                    fx_nibble_bit[1] = 0;
                } else {
                    if (io_inc[0] & 1) io_addr[1] -= 1;
                    fx_nibble_bit[1] = 1;
                }
            }
            io_addr[1] += increments[io_inc[0]];
        }
    } else if (fx_addr1_mode == 2 && write == false) { /* FX polygon fill mode */
        fx_x_pixel_position += fx_x_pixel_increment;
        fx_y_pixel_position += fx_y_pixel_increment;
        fx_poly_fill_length = ((int32_t)fx_y_pixel_position >> 16) - ((int32_t)fx_x_pixel_position >> 16);
        if (sel == 0 && fx_cache_byte_cycling && !fx_cache_fill)
            fx_cache_byte_index = (fx_cache_byte_index + 1) & 3;
        if (sel == 1) {
            if (fx_4bit_mode) {
                io_addr[1] = io_addr[0] + (fx_x_pixel_position >> 17);
                fx_nibble_bit[1] = (fx_x_pixel_position >> 16) & 1;
            } else {
                io_addr[1] = io_addr[0] + (fx_x_pixel_position >> 16);
            }
        }
    } else if (sel == 1 && fx_addr1_mode == 3 && write == false) { /* FX affine mode */
        fx_x_pixel_position += fx_x_pixel_increment;
        fx_y_pixel_position += fx_y_pixel_increment;
    }
    io_addr[0] &= 0x1ffff;
    io_addr[1] &= 0x1ffff;
    return address;
}

static void
fx_affine_prefetch(void)
{
    if (fx_addr1_mode != 3) return;

    uint32_t address;
    uint8_t affine_x_tile = (fx_x_pixel_position >> 19) & 0xff;
    uint8_t affine_y_tile = (fx_y_pixel_position >> 19) & 0xff;
    uint8_t affine_x_sub_tile = (fx_x_pixel_position >> 16) & 0x07;
    uint8_t affine_y_sub_tile = (fx_y_pixel_position >> 16) & 0x07;

    if (!fx_affine_clip) {
        affine_x_tile &= fx_affine_map_size - 1;
        affine_y_tile &= fx_affine_map_size - 1;
    }

    if (affine_x_tile >= fx_affine_map_size || affine_y_tile >= fx_affine_map_size) {
        address = fx_affine_tile_base + (affine_y_sub_tile << (3 - fx_4bit_mode)) + (affine_x_sub_tile >> (uint8_t)fx_4bit_mode);
        fx_nibble_bit[1] = (affine_x_sub_tile & 1) >> (1 - fx_4bit_mode);
    } else {
        address = fx_affine_map_base + (affine_y_tile * fx_affine_map_size) + affine_x_tile;
        uint8_t affine_tile_idx = video_space_read(address);
        address = fx_affine_tile_base + (affine_tile_idx << (6 - fx_4bit_mode));
        address += (affine_y_sub_tile << (3 - fx_4bit_mode)) + (affine_x_sub_tile >> (uint8_t)fx_4bit_mode);
        fx_nibble_bit[1] = (affine_x_sub_tile & 1) >> (1 - fx_4bit_mode);
    }
    io_addr[1] = address;
    io_rddata[1] = video_space_read(address);
}

/* ================================================================
 * DC value read (FX register readback)
 * ================================================================ */

/* the hardware computes it from the current X/Y positions all the time */
static void
update_fill_length(void)
{
    fx_poly_fill_length = (((int32_t)fx_y_pixel_position >> 16) - ((int32_t)fx_x_pixel_position >> 16)) & 0x3ff;
}

static uint8_t
video_get_dc_value(uint8_t reg)
{
    switch (reg & 0x1F) {
        case 0x00: case 0x01: case 0x02: case 0x03:
        case 0x04: case 0x05: case 0x06: case 0x07:
        case 0x08: case 0x09: case 0x0a:
        case 0x0c: case 0x0d: case 0x0e: case 0x0f:
            return reg_composer[reg];
        case 0x0b:
            return reg_composer[reg] & 0x3f;

        case 0x10: return (fx_x_pixel_position >> 16) & 0xff;
        case 0x11: return ((fx_x_pixel_position >> 24) & 0x07) | (fx_x_pixel_position & 0x80);
        case 0x12: return (fx_y_pixel_position >> 16) & 0xff;
        case 0x13: return ((fx_y_pixel_position >> 24) & 0x07) | (fx_y_pixel_position & 0x80);
        case 0x14: return (fx_x_pixel_position >> 8) & 0xff;
        case 0x15: return (fx_y_pixel_position >> 8) & 0xff;
        case 0x16: /* DCSEL=5, FX_POLY_FILL_L */
            update_fill_length();
            if (fx_poly_fill_length >= 768) {
                return ((fx_2bit_poly && fx_addr1_mode == 2) ? 0x00 : 0x80);
            }
            if (fx_4bit_mode) {
                if (fx_2bit_poly && fx_addr1_mode == 2) {
                    return ((fx_y_pixel_position & 0x00008000) >> 8) |
                        ((fx_x_pixel_position >> 11) & 0x60) |
                        ((fx_x_pixel_position >> 14) & 0x10) |
                        ((fx_poly_fill_length & 0x0007) << 1) |
                        ((fx_x_pixel_position & 0x00008000) >> 15);
                } else {
                    return ((!!(fx_poly_fill_length & 0xfff8)) << 7) |
                        ((fx_x_pixel_position >> 11) & 0x60) |
                        ((fx_x_pixel_position >> 14) & 0x10) |
                        ((fx_poly_fill_length & 0x0007) << 1);
                }
            } else {
                return ((!!(fx_poly_fill_length & 0xfff0)) << 7) |
                    ((fx_x_pixel_position >> 11) & 0x60) |
                    ((fx_poly_fill_length & 0x000f) << 1);
            }
        case 0x17:
            update_fill_length();
            return ((fx_poly_fill_length & 0x03f8) >> 2);
        case 0x18: return fx_cache[0];
        case 0x19: return fx_cache[1];
        case 0x1a: return fx_cache[2];
        case 0x1b: return fx_cache[3];
        default: break;
    }
    return vera_version_string[reg % 4];
}

/* ================================================================
 * Video reset
 * ================================================================ */

static void
video_reset(void)
{
    memset(io_addr, 0, sizeof(io_addr));
    memset(io_inc, 0, sizeof(io_inc));
    io_addrsel = 0;
    io_dcsel = 0;
    io_rddata[0] = 0;
    io_rddata[1] = 0;

    ien = 0;
    isr = 0;
    irq_line = 0;

    memset(reg_layer, 0, sizeof(reg_layer));

    memset(reg_composer, 0, sizeof(reg_composer));
    reg_composer[1] = 128; /* hscale = 1.0 */
    reg_composer[2] = 128; /* vscale = 1.0 */
    reg_composer[5] = 640 >> 2;
    reg_composer[7] = 480 >> 1;

    /* FX registers */
    fx_addr1_mode = 0;
    fx_x_pixel_position = 0x8000;
    fx_y_pixel_position = 0x8000;
    fx_x_pixel_increment = 0;
    fx_y_pixel_increment = 0;
    fx_cache_write = false;
    fx_cache_fill = false;
    fx_4bit_mode = false;
    fx_16bit_hop = false;
    fx_subtract = false;
    fx_cache_byte_cycling = false;
    fx_trans_writes = false;
    fx_multiplier = false;
    fx_mult_accumulator = 0;
    fx_2bit_poly = false;
    fx_2bit_poking = false;
    fx_cache_nibble_index = 0;
    fx_cache_byte_index = 0;
    fx_cache_increment_mode = 0;
    memset(fx_cache, 0, sizeof(fx_cache));
    fx_16bit_hop_align = 0;
    fx_nibble_bit[0] = false;
    fx_nibble_bit[1] = false;
    fx_nibble_incr[0] = false;
    fx_nibble_incr[1] = false;
    fx_poly_fill_length = 0;
    fx_affine_tile_base = 0;
    fx_affine_map_base = 0;
    fx_affine_map_size = 2;
    fx_affine_clip = false;

    memset(sprite_data, 0, sizeof(sprite_data));

    /* Copy palette */
    for (int i = 0; i < 256; i++) {
        palette[i * 2 + 0] = default_palette[i] & 0xff;
        palette[i * 2 + 1] = default_palette[i] >> 8;
    }
    refresh_palette();

    /* Fill VRAM with random data (matches real hardware) */
    for (int i = 0; i < 128 * 1024; i++)
        video_ram[i] = rand();

    sprite_line_collisions = 0;

    vga_scan_pos_x = 0;
    vga_scan_pos_y = 0;
    ntsc_half_cnt = 0;
    ntsc_scan_pos_y = 0;
    ntsc_y_counter = 0;
    ntsc_y_counter_prev = 0;

    psg_reset();
    pcm_reset();
    vera_spi_init();
}

/* ================================================================
 * Rendering: expand 4bpp data
 * ================================================================ */

static void
expand_4bpp_data(uint8_t *dst, const uint8_t *src, int dst_size)
{
    while (dst_size >= 2) {
        *dst++ = (*src) >> 4;
        *dst++ = (*src) & 0xf;
        ++src;
        dst_size -= 2;
    }
}

/* ================================================================
 * Rendering: sprites
 * ================================================================ */

static void
render_sprite_line(const uint16_t y)
{
    memset(sprite_line_col, 0, SCREEN_WIDTH);
    memset(sprite_line_z, 0, SCREEN_WIDTH);
    memset(sprite_line_mask, 0, SCREEN_WIDTH);

    uint16_t sprite_budget = 800 + 1;
    for (int i = 0; i < NUM_SPRITES; i++) {
        sprite_budget--;
        if (sprite_budget == 0) break;
        const struct video_sprite_properties *props = &sprite_properties[i];

        if (props->sprite_zdepth == 0) continue;
        if (y < props->sprite_y || y >= props->sprite_y + props->sprite_height) continue;

        const uint16_t eff_sy = props->vflip ? ((props->sprite_height - 1) - (y - props->sprite_y)) : (y - props->sprite_y);

        int16_t       eff_sx      = (props->hflip ? (props->sprite_width - 1) : 0);
        const int16_t eff_sx_incr = props->hflip ? -1 : 1;

        const uint8_t *bitmap_data = video_ram + props->sprite_address + (eff_sy << (props->sprite_width_log2 - (1 - props->color_mode)));

        uint8_t unpacked_sprite_line[64];
        const uint16_t width = (props->sprite_width < 64 ? props->sprite_width : 64);
        const uint8_t vram_fetch_mask = ((2 - props->color_mode) << 2) - 1;
        if (props->color_mode == 0) {
            expand_4bpp_data(unpacked_sprite_line, bitmap_data, width);
        } else {
            memcpy(unpacked_sprite_line, bitmap_data, width);
        }

        for (uint16_t sx = 0; sx < props->sprite_width; ++sx) {
            const uint16_t line_x = props->sprite_x + sx;
            if (line_x >= SCREEN_WIDTH) {
                eff_sx += eff_sx_incr;
                continue;
            }

            if (!(sx & vram_fetch_mask)) {
                sprite_budget--;
                if (sprite_budget == 0) break;
            }
            sprite_budget--;
            if (sprite_budget == 0) break;

            uint8_t col_index = unpacked_sprite_line[eff_sx];
            eff_sx += eff_sx_incr;

            if (col_index > 0) {
                sprite_line_collisions |= sprite_line_mask[line_x] & props->sprite_collision_mask;

                /* a sprite that loses on Z (equal Z too) isn't written, nor is its mask */
                if (props->sprite_zdepth > sprite_line_z[line_x]) {
                    sprite_line_mask[line_x] |= props->sprite_collision_mask;
                    if (col_index < 16)
                        col_index += props->palette_offset;
                    sprite_line_col[line_x] = col_index;
                    sprite_line_z[line_x] = props->sprite_zdepth;
                }
            }
        }
        if (sprite_budget == 0) break;  /* out of time mid-sprite: the rest of the line is too */
    }
}

/* ================================================================
 * Rendering: text layer
 * ================================================================ */

static void
render_layer_line_text(uint8_t layer, uint16_t y)
{
    const struct video_layer_properties *props = &prev_layer_properties[1][layer];
    const struct video_layer_properties *props0 = &prev_layer_properties[0][layer];

    const uint8_t max_pixels_per_byte = (8 >> props->color_depth) - 1;
    const int     eff_y               = calc_layer_eff_y(props0, y);
    const int     yy                  = eff_y & props->tileh_max;
    const uint32_t y_add = (yy << props->tilew_log2) >> 3;

    const uint32_t map_addr_begin = calc_layer_map_addr_base2(props, props->min_eff_x, eff_y);
    const uint32_t map_addr_end   = calc_layer_map_addr_base2(props, props->max_eff_x, eff_y);
    const int      size           = (map_addr_end - map_addr_begin) + 2;

    uint8_t tile_bytes[512];
    video_space_read_range(tile_bytes, map_addr_begin, size);

    uint32_t tile_start;
    uint8_t  fg_color;
    uint8_t  bg_color;
    uint8_t  s;
    uint8_t  color_shift;

    {
        const int eff_x = calc_layer_eff_x(props, 0);
        const int xx    = eff_x & props->tilew_max;
        const uint32_t map_addr = calc_layer_map_addr_base2(props, eff_x, eff_y) - map_addr_begin;
        const uint8_t tile_index = tile_bytes[map_addr];
        const uint8_t byte1      = tile_bytes[map_addr + 1];

        if (!props->text_mode_256c) {
            fg_color = byte1 & 15;
            bg_color = byte1 >> 4;
        } else {
            fg_color = byte1;
            bg_color = 0;
        }

        tile_start = tile_index << props->tile_size_log2;

        const uint16_t x_add       = xx >> 3;
        const uint32_t tile_offset = tile_start + y_add + x_add;

        s           = video_space_read(props->tile_base + tile_offset);
        color_shift = max_pixels_per_byte - (xx & 0x7);
    }

    for (int x = 0; x < SCREEN_WIDTH; x++) {
        const int eff_x = calc_layer_eff_x(props, x);
        const int xx = eff_x & props->tilew_max;

        if ((eff_x & 0x7) == 0) {
            if ((eff_x & props->tilew_max) == 0) {
                const uint32_t map_addr = calc_layer_map_addr_base2(props, eff_x, eff_y) - map_addr_begin;
                const uint8_t tile_index = tile_bytes[map_addr];
                const uint8_t byte1      = tile_bytes[map_addr + 1];

                if (!props->text_mode_256c) {
                    fg_color = byte1 & 15;
                    bg_color = byte1 >> 4;
                } else {
                    fg_color = byte1;
                    bg_color = 0;
                }
                tile_start = tile_index << props->tile_size_log2;
            }

            const uint16_t x_add       = xx >> 3;
            const uint32_t tile_offset = tile_start + y_add + x_add;

            s           = video_space_read(props->tile_base + tile_offset);
            color_shift = max_pixels_per_byte;
        }

        const uint8_t col_index = (s >> color_shift) & 1;
        --color_shift;
        layer_line[layer][x] = col_index ? fg_color : bg_color;
    }
}

/* ================================================================
 * Rendering: tile layer
 * ================================================================ */

static void
render_layer_line_tile(uint8_t layer, uint16_t y)
{
    const struct video_layer_properties *props = &prev_layer_properties[1][layer];
    const struct video_layer_properties *props0 = &prev_layer_properties[0][layer];

    const uint8_t max_pixels_per_byte = (8 >> props->color_depth) - 1;
    const int     eff_y               = calc_layer_eff_y(props0, y);
    const uint8_t yy                  = eff_y & props->tileh_max;
    const uint8_t yy_flip             = yy ^ props->tileh_max;
    const uint32_t y_add              = (yy << ((props->tilew_log2 + props->color_depth - 3) & 31));
    const uint32_t y_add_flip         = (yy_flip << ((props->tilew_log2 + props->color_depth - 3) & 31));

    const uint32_t map_addr_begin = calc_layer_map_addr_base2(props, props->min_eff_x, eff_y);
    const uint32_t map_addr_end   = calc_layer_map_addr_base2(props, props->max_eff_x, eff_y);
    const int      size           = (map_addr_end - map_addr_begin) + 2;

    uint8_t tile_bytes[512];
    video_space_read_range(tile_bytes, map_addr_begin, size);

    uint8_t  palette_offset;
    bool     vflip;
    bool     hflip;
    uint32_t tile_start;
    uint8_t  s;
    uint8_t  color_shift;
    int8_t   color_shift_incr;

    {
        const int eff_x = calc_layer_eff_x(props, 0);
        const uint32_t map_addr = calc_layer_map_addr_base2(props, eff_x, eff_y) - map_addr_begin;
        const uint8_t byte0 = tile_bytes[map_addr];
        const uint8_t byte1 = tile_bytes[map_addr + 1];

        vflip = (byte1 >> 3) & 1;
        hflip = (byte1 >> 2) & 1;
        palette_offset = byte1 & 0xf0;

        const uint16_t tile_index = byte0 | ((byte1 & 3) << 8);
        tile_start = tile_index << props->tile_size_log2;

        color_shift_incr = hflip ? props->bits_per_pixel : -props->bits_per_pixel;

        int xx = eff_x & props->tilew_max;
        if (hflip) {
            xx = xx ^ (props->tilew_max);
            color_shift = 0;
        } else {
            color_shift = props->first_color_pos;
        }

        uint16_t x_add       = (xx << props->color_depth) >> 3;
        uint32_t tile_offset = tile_start + (vflip ? y_add_flip : y_add) + x_add;

        s = video_space_read(props->tile_base + tile_offset);
    }

    for (int x = 0; x < SCREEN_WIDTH; x++) {
        const int eff_x = calc_layer_eff_x(props, x);

        if ((eff_x & max_pixels_per_byte) == 0) {
            if ((eff_x & props->tilew_max) == 0) {
                const uint32_t map_addr = calc_layer_map_addr_base2(props, eff_x, eff_y) - map_addr_begin;
                const uint8_t byte0 = tile_bytes[map_addr];
                const uint8_t byte1 = tile_bytes[map_addr + 1];

                vflip = (byte1 >> 3) & 1;
                hflip = (byte1 >> 2) & 1;
                palette_offset = byte1 & 0xf0;

                const uint16_t tile_index = byte0 | ((byte1 & 3) << 8);
                tile_start = tile_index << props->tile_size_log2;

                color_shift_incr = hflip ? props->bits_per_pixel : -props->bits_per_pixel;
            }

            int xx = eff_x & props->tilew_max;
            if (hflip) {
                xx = xx ^ (props->tilew_max);
                color_shift = 0;
            } else {
                color_shift = props->first_color_pos;
            }

            const uint16_t x_add       = (xx << props->color_depth) >> 3;
            const uint32_t tile_offset = tile_start + (vflip ? y_add_flip : y_add) + x_add;

            s = video_space_read(props->tile_base + tile_offset);
        }

        uint8_t col_index = (s >> color_shift) & props->color_mask;
        color_shift += color_shift_incr;

        if (col_index > 0 && col_index < 16) {
            col_index += palette_offset;
            if (props->text_mode_256c) col_index |= 0x80;
        }
        layer_line[layer][x] = col_index;
    }
}

/* ================================================================
 * Rendering: bitmap layer
 * ================================================================ */

static void
render_layer_line_bitmap(uint8_t layer, uint16_t y)
{
    const struct video_layer_properties *props = &prev_layer_properties[1][layer];

    int yy = y % props->tileh;
    uint32_t y_add = (yy * props->tilew * props->bits_per_pixel) >> 3;

    for (int x = 0; x < SCREEN_WIDTH; x++) {
        int xx = x % props->tilew;

        uint8_t palette_offset = reg_layer[layer][4] & 0xf;

        uint16_t x_add = (xx * props->bits_per_pixel) >> 3;
        uint32_t tile_offset = y_add + x_add;
        uint8_t s = video_space_read(props->tile_base + tile_offset);

        uint8_t col_index = (s >> (props->first_color_pos - ((xx & props->color_fields_max) << props->color_depth))) & props->color_mask;

        if (col_index > 0 && col_index < 16) {
            col_index += palette_offset << 4;
            if (props->text_mode_256c) col_index |= 0x80;
        }
        layer_line[layer][x] = col_index;
    }
}

/* ================================================================
 * Rendering: line compositor
 * ================================================================ */

static uint8_t
calculate_line_col_index(uint8_t spr_zindex, uint8_t spr_col_index, uint8_t l1_col_index, uint8_t l2_col_index)
{
    uint8_t col_index = 0;
    switch (spr_zindex) {
        case 3: col_index = spr_col_index ? spr_col_index : (l2_col_index ? l2_col_index : l1_col_index); break;
        case 2: col_index = l2_col_index ? l2_col_index : (spr_col_index ? spr_col_index : l1_col_index); break;
        case 1: col_index = l2_col_index ? l2_col_index : (l1_col_index ? l1_col_index : spr_col_index); break;
        case 0: col_index = l2_col_index ? l2_col_index : l1_col_index; break;
    }
    return col_index;
}

static void
render_line(uint16_t y, float scan_pos_x)
{
    static uint16_t y_prev;
    static uint16_t s_pos_x_p;
    static uint32_t eff_y_fp;
    static uint32_t eff_x_fp;
    static uint8_t col_line[SCREEN_WIDTH];

    uint8_t dc_video = reg_composer[0];
    uint16_t vstart = reg_composer[6] << 1;
    uint16_t vstop = reg_composer[7] << 1;

    if (y != y_prev) {
        y_prev = y;
        s_pos_x_p = 0;

        memcpy(prev_reg_composer[1], prev_reg_composer[0], sizeof(*reg_composer) * COMPOSER_SLOTS);
        memcpy(prev_reg_composer[0], reg_composer, sizeof(*reg_composer) * COMPOSER_SLOTS);

        memcpy(prev_layer_properties[1], prev_layer_properties[0], sizeof(*layer_properties) * NUM_LAYERS);
        memcpy(prev_layer_properties[0], layer_properties, sizeof(*layer_properties) * NUM_LAYERS);

        if ((dc_video & 3) > 1) { /* 480i or 240p */
            if ((y >> 1) == 0) {
                eff_y_fp = y * (prev_reg_composer[1][2] << 9);
            } else if (((y & 0xfffe) >= vstart) && ((y & 0xfffe) < vstop)) {
                eff_y_fp += (prev_reg_composer[1][2] << 10);
            }
        } else {
            if (y == 0) {
                eff_y_fp = 0;
            } else if ((y >= vstart) && (y < vstop)) {
                eff_y_fp += (prev_reg_composer[1][2] << 9);
            }
        }
    }

    if ((dc_video & 8) && (dc_video & 3) > 1)
        y &= 0xfffe;

    if (video_palette.dirty)
        refresh_palette();

    if (y >= SCREEN_HEIGHT)
        return;

    uint16_t s_pos_x = (uint16_t)roundf(scan_pos_x);
    if (s_pos_x > SCREEN_WIDTH)
        s_pos_x = SCREEN_WIDTH;

    if (s_pos_x_p == 0)
        eff_x_fp = 0;

    uint8_t out_mode = reg_composer[0] & 3;
    uint8_t border_color = reg_composer[3];
    uint16_t hstart = reg_composer[4] << 2;
    uint16_t hstop = reg_composer[5] << 2;

    uint16_t eff_y = (eff_y_fp >> 16);
    if (eff_y >= 480) eff_y = 480 - (y & 1);

    layer_line_enable[0] = dc_video & 0x10;
    layer_line_enable[1] = dc_video & 0x20;
    sprite_line_enable   = dc_video & 0x40;

    for (uint8_t layer = 0; layer < 2; layer++) {
        if (!layer_line_enable[layer] && old_layer_line_enable[layer]) {
            for (uint16_t i = s_pos_x_p; i < SCREEN_WIDTH; i++)
                layer_line[layer][i] = 0;
        }
        if (s_pos_x_p == 0)
            old_layer_line_enable[layer] = layer_line_enable[layer];
    }

    if (!sprite_line_enable && old_sprite_line_enable) {
        for (uint16_t i = s_pos_x_p; i < SCREEN_WIDTH; i++) {
            sprite_line_col[i] = 0;
            sprite_line_z[i] = 0;
            sprite_line_mask[i] = 0;
        }
    }
    if (s_pos_x_p == 0)
        old_sprite_line_enable = sprite_line_enable;

    if (sprite_line_enable)
        render_sprite_line(eff_y);

    if (layer_line_enable[0]) {
        if (prev_layer_properties[1][0].text_mode)
            render_layer_line_text(0, eff_y);
        else if (prev_layer_properties[1][0].bitmap_mode)
            render_layer_line_bitmap(0, eff_y);
        else
            render_layer_line_tile(0, eff_y);
    }
    if (layer_line_enable[1]) {
        if (prev_layer_properties[1][1].text_mode)
            render_layer_line_text(1, eff_y);
        else if (prev_layer_properties[1][1].bitmap_mode)
            render_layer_line_bitmap(1, eff_y);
        else
            render_layer_line_tile(1, eff_y);
    }

    if (out_mode != 0) {
        if (y < vstart || y >= vstop) {
            uint32_t border_fill = border_color;
            border_fill = border_fill | (border_fill << 8);
            border_fill = border_fill | (border_fill << 16);
            memset(col_line, border_fill, SCREEN_WIDTH);
        } else {
            hstart = hstart < 640 ? hstart : 640;
            hstop = hstop < 640 ? hstop : 640;

            for (uint16_t x = s_pos_x_p; x < hstart && x < s_pos_x; ++x)
                col_line[x] = border_color;

            const uint32_t scale = reg_composer[1];
            for (uint16_t x = MAX(hstart, s_pos_x_p); x < hstop && x < s_pos_x; ++x) {
                uint16_t eff_x = eff_x_fp >> 16;
                col_line[x] = (eff_x < SCREEN_WIDTH) ? calculate_line_col_index(sprite_line_z[eff_x], sprite_line_col[eff_x], layer_line[0][eff_x], layer_line[1][eff_x]) : 0;
                eff_x_fp += (scale << 9);
            }
            for (uint16_t x = hstop; x < s_pos_x; ++x)
                col_line[x] = border_color;
        }
    }

    /* Color lookup into framebuffer */
    uint32_t *framebuffer4_begin = ((uint32_t *)framebuffer) + (y * SCREEN_WIDTH) + s_pos_x_p;
    {
        uint32_t *framebuffer4 = framebuffer4_begin;
        for (uint16_t x = s_pos_x_p; x < s_pos_x; x++)
            *framebuffer4++ = video_palette.entries[col_line[x]];
    }

    /* NTSC overscan dimming */
    if (out_mode == 2) {
        uint32_t *framebuffer4 = framebuffer4_begin;
        for (uint16_t x = s_pos_x_p; x < s_pos_x; x++) {
            if (x < SCREEN_WIDTH * TITLE_SAFE_X ||
                x > SCREEN_WIDTH * (1 - TITLE_SAFE_X) ||
                y < SCREEN_HEIGHT * TITLE_SAFE_Y ||
                y > SCREEN_HEIGHT * (1 - TITLE_SAFE_Y)) {
                *framebuffer4 &= 0x00fcfcfc;
                *framebuffer4 >>= 2;
            }
            framebuffer4++;
        }
    }

    s_pos_x_p = s_pos_x;
}

/* ================================================================
 * ISR / collision update
 * ================================================================ */

static void
vsync_isr(void)
{
    if (sprite_line_collisions != 0) isr |= 4;
    isr = (isr & 0xf) | sprite_line_collisions;
    sprite_line_collisions = 0;
    isr |= 1; vc_vsync_raised++; /* VSYNC IRQ */
}

static void
update_isr_and_coll(uint16_t y, uint16_t compare)
{
    if (y == SCREEN_HEIGHT)
        vsync_isr();
    if (y == compare)
        isr |= 2; /* LINE IRQ */
}

/* composite LINE irq compares the counter before it steps, bit 0 ignored */
static void
ntsc_next_line(bool ntsc_mode)
{
    if (ntsc_mode && (ntsc_y_counter >> 1) == (irq_line >> 1))
        isr |= 2;
    ntsc_y_counter_prev = ntsc_y_counter;
    ntsc_y_counter = (ntsc_y_counter + 2) & 0x3ff;
    if (ntsc_scan_pos_y == NTSC_FRAME_START_1 || ntsc_scan_pos_y == NTSC_FRAME_START_2) {
        ntsc_field = ntsc_scan_pos_y == NTSC_FRAME_START_2;
        ntsc_y_counter = ntsc_field;
        if (ntsc_mode)
            reg_composer[0] = (reg_composer[0] & 0x7f) | (ntsc_field ? 0x80 : 0);
    }
}

/* ================================================================
 * Video step — scanline timing
 * ================================================================ */

/* Approximate 68020 cycles per VGA scanline at 16 MHz:
 * 16,000,000 / (60 * 525) ~ 508 */
#define CPU_MHZ 16.0f

static bool
video_step_internal(float steps)
{
    uint16_t y = 0;
    bool ntsc_mode = reg_composer[0] & 2;
    bool new_frame = false;

    vga_scan_pos_x += PIXEL_FREQ * steps / CPU_MHZ;
    /* while, not if: a call may cover more than one line (a2run feeds the cycles the
       CPU really ran, often > 800 pixels); with `if` the surplus piled up and VERA
       ran at one line per call (17,325 instead of 17,145 CPU cycles per frame). */
    while (vga_scan_pos_x > VGA_SCAN_WIDTH) {
        vga_scan_pos_x -= VGA_SCAN_WIDTH;
        if (!ntsc_mode && !vc_norender)
            render_line(vga_scan_pos_y - VGA_Y_OFFSET, VGA_SCAN_WIDTH);
        vga_scan_pos_y++;
        if (vga_scan_pos_y == SCAN_HEIGHT) {
            vga_scan_pos_y = 0;
            if (!ntsc_mode) {
                /* the composite timing keeps running, the field bit follows it */
                reg_composer[0] = (reg_composer[0] & 0x7f) | (ntsc_field ? 0x80 : 0);
                new_frame = true;
                frame_count++;
            }
        }
        if (!ntsc_mode)
            update_isr_and_coll(vga_scan_pos_y - VGA_Y_OFFSET, irq_line);
    }

    float ntsc_from = ntsc_half_cnt;
    uint16_t field_len = (reg_composer[0] & 8) ? SCAN_HEIGHT + 1 : SCAN_HEIGHT;  /* 240p: 263 lines */
    ntsc_half_cnt += PIXEL_FREQ * steps / CPU_MHZ;
    for (;;) {   /* one half-line per pass, see above */
        float to = ntsc_half_cnt > NTSC_HALF_SCAN_WIDTH ? NTSC_HALF_SCAN_WIDTH : ntsc_half_cnt;
        if (!(ntsc_scan_pos_y & 1) && ntsc_from < NTSC_NEXT_LINE && to >= NTSC_NEXT_LINE)
            ntsc_next_line(ntsc_mode);
        if (ntsc_half_cnt <= NTSC_HALF_SCAN_WIDTH)
            break;
        ntsc_half_cnt -= NTSC_HALF_SCAN_WIDTH;
        ntsc_from = 0;
        if (ntsc_mode) {
            if (ntsc_scan_pos_y < SCAN_HEIGHT) {
                y = ntsc_scan_pos_y - NTSC_Y_OFFSET_LOW;
                if ((y & 1) == 0)
                    render_line(y, NTSC_HALF_SCAN_WIDTH);
            } else {
                y = ntsc_scan_pos_y - NTSC_Y_OFFSET_HIGH;
                if ((y & 1) == 0)
                    render_line(y | 1, NTSC_HALF_SCAN_WIDTH);
            }
        }
        ntsc_scan_pos_y++;
        if (ntsc_scan_pos_y == field_len) {
            if (ntsc_mode) {
                vsync_isr();
                new_frame = true;
                frame_count++;
            }
        }
        if (ntsc_scan_pos_y >= field_len * 2) {
            ntsc_scan_pos_y = 0;
            if (ntsc_mode) {
                vsync_isr();
                new_frame = true;
                frame_count++;
            }
        }
    }

    return new_frame;
}

unsigned long vc_isr_reads = 0, vc_isr_set = 0, vc_isr_acks = 0;   /* $2002-shim diagnostics */
static bool
video_get_irq_out(void)
{
    uint8_t tmp_isr = isr | (pcm_is_fifo_almost_empty() ? 8 : 0);
    return (tmp_isr & ien) != 0;
}

/* ================================================================
 * Register read
 * ================================================================ */

static uint8_t
video_read_reg(uint8_t reg)
{
    bool ntsc_mode = reg_composer[0] & 2;
    uint16_t scanline;
    if (ntsc_mode)
        scanline = ntsc_y_counter_prev >= 512 ? 511 : ntsc_y_counter & 0x1ff;
    else if (vga_scan_pos_y >= 512)
        scanline = 511;
    else
        scanline = (vga_scan_pos_y + 1) & 0x1ff;  /* the counter has moved on as line L starts */

    switch (reg & 0x1F) {
        case 0x00: return io_addr[io_addrsel] & 0xff;
        case 0x01: return (io_addr[io_addrsel] >> 8) & 0xff;
        case 0x02: return (io_addr[io_addrsel] >> 16) | (fx_nibble_bit[io_addrsel] << 1) | (fx_nibble_incr[io_addrsel] << 2) | (io_inc[io_addrsel] << 3);
        case 0x03:
        case 0x04: {
            bool addr_nibble = fx_nibble_bit[reg - 3];
            uint32_t address = get_and_inc_address(reg - 3, false);
            uint8_t value = io_rddata[reg - 3];

            if (reg == 4 && fx_addr1_mode == 3)
                fx_affine_prefetch();
            else
                io_rddata[reg - 3] = video_space_read(io_addr[reg - 3]);

            if (fx_cache_fill) {
                if (fx_4bit_mode) {
                    uint8_t nibble_read = (addr_nibble ? ((value & 0x0f) << 4) : (value & 0xf0));
                    if (fx_cache_nibble_index) {
                        fx_cache[fx_cache_byte_index] = (fx_cache[fx_cache_byte_index] & 0xf0) | (nibble_read >> 4);
                        fx_cache_nibble_index = 0;
                        fx_cache_byte_index = ((fx_cache_byte_index + 1) & 0x3);
                    } else {
                        fx_cache[fx_cache_byte_index] = (fx_cache[fx_cache_byte_index] & 0x0f) | (nibble_read);
                        fx_cache_nibble_index = 1;
                    }
                } else {
                    fx_cache[fx_cache_byte_index] = value;
                    if (fx_cache_increment_mode)
                        fx_cache_byte_index = (fx_cache_byte_index & 0x2) | ((fx_cache_byte_index + 1) & 0x1);
                    else
                        fx_cache_byte_index = ((fx_cache_byte_index + 1) & 0x3);
                }
            }
            (void)address;
            return value;
        }
        case 0x05: return (io_dcsel << 1) | io_addrsel;
        case 0x06: return ((irq_line & 0x100) >> 1) | ((scanline & 0x100) >> 2) | (ien & 0xF);
        case 0x07: { audio_render(); uint8_t rv = isr | (pcm_is_fifo_almost_empty() ? 8 : 0);
                     vc_isr_reads++; if (rv & 1) vc_isr_set++;
                     return rv; }
        case 0x08: return scanline & 0xFF;

        case 0x09:
        case 0x0A:
        case 0x0B:
        case 0x0C: {
            int i = reg - 0x09 + (io_dcsel << 2);
            switch (i) {
                case 0x00: case 0x01: case 0x02: case 0x03:
                case 0x04: case 0x05: case 0x06: case 0x07:
                case 0x08:
                case 0x16: case 0x17:
                    return video_get_dc_value(i);
                case 0x18:
                    fx_mult_accumulator = 0;
                    break;
                case 0x19: {
                    int32_t m_result = (int16_t)((fx_cache[1] << 8) | fx_cache[0]) * (int16_t)((fx_cache[3] << 8) | fx_cache[2]);
                    if (fx_subtract)
                        fx_mult_accumulator -= m_result;
                    else
                        fx_mult_accumulator += m_result;
                    break;
                }
                default:
                    break;
            }
            return vera_version_string[i % 4];
        }

        case 0x11: case 0x13:   /* the high scroll bytes are 4 bits */
            return reg_layer[0][reg - 0x0D] & 0x0f;
        case 0x0D: case 0x0E: case 0x0F:
        case 0x10: case 0x12:
            return reg_layer[0][reg - 0x0D];

        case 0x18: case 0x1A:
            return reg_layer[1][reg - 0x14] & 0x0f;
        case 0x14: case 0x15: case 0x16:
        case 0x17: case 0x19:
            return reg_layer[1][reg - 0x14];

        case 0x1B: audio_render(); return pcm_read_ctrl();
        case 0x1C: return pcm_read_rate();
        case 0x1D: return 0;

        case 0x1E:
        case 0x1F: return vera_spi_read(reg & 1);
    }
    return 0;
}

/* ================================================================
 * Register write
 * ================================================================ */

static void
video_write_reg(uint8_t reg, uint8_t value)
{
    switch (reg & 0x1F) {
        case 0x00:
            if (fx_2bit_poly && fx_4bit_mode && fx_addr1_mode == 2 && io_addrsel == 1) {
                fx_2bit_poking = true;
                io_addr[1] = (io_addr[1] & 0x1fffc) | (value & 0x3);
            } else {
                io_addr[io_addrsel] = (io_addr[io_addrsel] & 0x1ff00) | value;
                if (fx_16bit_hop && io_addrsel == 1)
                    fx_16bit_hop_align = value & 3;
            }
            io_rddata[io_addrsel] = video_space_read(io_addr[io_addrsel]);
            break;
        case 0x01:
            io_addr[io_addrsel] = (io_addr[io_addrsel] & 0x100ff) | (value << 8);
            io_rddata[io_addrsel] = video_space_read(io_addr[io_addrsel]);
            break;
        case 0x02:
            io_addr[io_addrsel] = (io_addr[io_addrsel] & 0x0ffff) | ((value & 0x1) << 16);
            fx_nibble_bit[io_addrsel] = (value >> 1) & 0x1;
            fx_nibble_incr[io_addrsel] = (value >> 2) & 0x1;
            io_inc[io_addrsel] = value >> 3;
            io_rddata[io_addrsel] = video_space_read(io_addr[io_addrsel]);
            break;
        case 0x03:
        case 0x04: {
            /* vc_dbg_wr: census of every DATA port write by target region, so
               "the routine ran but VRAM is zero" can be attributed. */
            { extern unsigned long vc_wr_bucket[16];
unsigned long vc_wr_nz[16];
              uint32_t wa = io_addr[reg - 3] & 0x1FFFF;
              vc_wr_bucket[wa >> 13]++;
              /* separate NON-ZERO writes: a game that clears the nametable
                 every frame writes constantly and leaves VRAM empty, which
                 looks identical to writing nothing unless the value is
                 counted too. */
              { extern unsigned long vc_wr_nz[16];
                if (value) vc_wr_nz[wa >> 13]++; } }
            /* Log DATA0/DATA1 writes to tilemap area */
            {
                int sel = reg - 3;
                uint32_t a = io_addr[sel] & 0x1FFFF;
                if (a < 0x4000) {
                    static FILE *dlog = NULL;
                    static int dc = 0;
                    if (!dlog) dlog = fopen("/tmp/vera_data.txt", "w");
                    if (dlog && dc < 100000) {
                        fprintf(dlog, "DATA%d addr=%05X val=%02X '%c' inc=%d\n",
                            sel, a, value, (value >= 0x20 && value < 0x7F) ? value : '.', increments[io_inc[sel]]);
                        dc++;
                        fflush(dlog);
                    }
                }
            }
            /* poke mode only takes a DATA1 write in polygon mode; DATA0 writes are normal */
            if (fx_2bit_poking && fx_addr1_mode == 2 && reg == 4) {
                fx_2bit_poking = false;
                uint8_t mask = value >> 6;
                switch (mask) {
                    case 0x00: video_ram[io_addr[1] & 0x1FFFF] = (fx_cache[fx_cache_byte_index] & 0xc0) | (io_rddata[1] & 0x3f); break;
                    case 0x01: video_ram[io_addr[1] & 0x1FFFF] = (fx_cache[fx_cache_byte_index] & 0x30) | (io_rddata[1] & 0xcf); break;
                    case 0x02: video_ram[io_addr[1] & 0x1FFFF] = (fx_cache[fx_cache_byte_index] & 0x0c) | (io_rddata[1] & 0xf3); break;
                    case 0x03: video_ram[io_addr[1] & 0x1FFFF] = (fx_cache[fx_cache_byte_index] & 0x03) | (io_rddata[1] & 0xfc); break;
                }
                break;
            }

            bool nibble = fx_nibble_bit[reg - 3];
            uint32_t address = get_and_inc_address(reg - 3, true);

            uint8_t wrdata_to_use;
            uint8_t ram_wrdata[4];
            uint8_t nibble_mask[4];
            uint8_t cache_to_use[4];

            if (fx_multiplier) {
                int32_t m_result = (int16_t)((fx_cache[1] << 8) | fx_cache[0]) * (int16_t)((fx_cache[3] << 8) | fx_cache[2]);
                if (fx_subtract)
                    m_result = fx_mult_accumulator - m_result;
                else
                    m_result = fx_mult_accumulator + m_result;
                cache_to_use[0] = (m_result) & 0xff;
                cache_to_use[1] = (m_result >> 8) & 0xff;
                cache_to_use[2] = (m_result >> 16) & 0xff;
                cache_to_use[3] = (m_result >> 24) & 0xff;
            } else {
                memcpy(cache_to_use, fx_cache, sizeof(fx_cache));
            }

            if (fx_cache_byte_cycling)
                wrdata_to_use = fx_cache[fx_cache_byte_index];
            else
                wrdata_to_use = value;

            if (fx_cache_write && !fx_cache_byte_cycling) {
                ram_wrdata[0] = cache_to_use[0];
                ram_wrdata[1] = cache_to_use[1];
                ram_wrdata[2] = cache_to_use[2];
                ram_wrdata[3] = cache_to_use[3];
            } else {
                ram_wrdata[0] = wrdata_to_use;
                ram_wrdata[1] = wrdata_to_use;
                ram_wrdata[2] = wrdata_to_use;
                ram_wrdata[3] = wrdata_to_use;
            }

            if (address >= ADDR_PSG_START) {
                /* FX write modes only apply to VRAM: PSG, palette and sprites get the plain byte */
                video_space_write(address, value);
            } else if (fx_cache_write) {
                address &= 0x1fffc;
                if (fx_trans_writes) {
                    if (fx_4bit_mode) {
                        nibble_mask[0] = (((ram_wrdata[0] & 0xf0) == 0) << 1) | ((ram_wrdata[0] & 0x0f) == 0);
                        nibble_mask[1] = (((ram_wrdata[1] & 0xf0) == 0) << 1) | ((ram_wrdata[1] & 0x0f) == 0);
                        nibble_mask[2] = (((ram_wrdata[2] & 0xf0) == 0) << 1) | ((ram_wrdata[2] & 0x0f) == 0);
                        nibble_mask[3] = (((ram_wrdata[3] & 0xf0) == 0) << 1) | ((ram_wrdata[3] & 0x0f) == 0);
                    } else {
                        nibble_mask[0] = (ram_wrdata[0] != 0) ? 0 : 3;
                        nibble_mask[1] = (ram_wrdata[1] != 0) ? 0 : 3;
                        nibble_mask[2] = (ram_wrdata[2] != 0) ? 0 : 3;
                        nibble_mask[3] = (ram_wrdata[3] != 0) ? 0 : 3;
                    }
                } else {
                    nibble_mask[0] = value & 0x3;
                    nibble_mask[1] = (value >> 2) & 0x3;
                    nibble_mask[2] = (value >> 4) & 0x3;
                    nibble_mask[3] = (value >> 6) & 0x3;
                }
                fx_vram_cache_write(address + 0, ram_wrdata[0], nibble_mask[0]);
                fx_vram_cache_write(address + 1, ram_wrdata[1], nibble_mask[1]);
                fx_vram_cache_write(address + 2, ram_wrdata[2], nibble_mask[2]);
                fx_vram_cache_write(address + 3, ram_wrdata[3], nibble_mask[3]);
            } else {
                fx_video_space_write(address, nibble, wrdata_to_use);
            }

            io_rddata[reg - 3] = video_space_read(io_addr[reg - 3]);
            break;
        }
        case 0x05:
            if (value & 0x80)
                video_reset();
            io_dcsel = (value >> 1) & 0x3f;
            io_addrsel = value & 1;
            break;
        case 0x06:
            irq_line = (irq_line & 0xFF) | ((value >> 7) << 8);
            ien = value & 0xF;
            vc_ien_writes++; vc_ien_last = ien; if(!(ien & 1)) vc_ien_disarms++;
            break;
        case 0x07:
            isr &= value ^ 0xff; vc_isr_acks++;
            break;
        case 0x08:
            irq_line = (irq_line & 0x100) | value;
            break;

        case 0x09: case 0x0A: case 0x0B: case 0x0C: {
            int i = reg - 0x09 + (io_dcsel << 2);
            /* DC_VIDEO is sampled once at end of run, which cannot distinguish
               "never enabled layer 0" from "enabled it, then crashed with it
               off". Latch whether L0 (bit 4) was EVER set. */
            if (i == 0 && (value & 0x10)) vc_l0_ever = 1;
            if (i == 0) {
                if (((reg_composer[0] & 0x8) == 0 && (value & 0x8)) ||
                    ((reg_composer[0] & 0x3) == 1 && (value & 0x3) > 1 && (value & 0x8))) {
                    memset(framebuffer, 0x00, SCREEN_WIDTH * SCREEN_HEIGHT * 4);
                }
                /* one line counter: going to composite it carries on from the VGA line */
                if ((reg_composer[0] & 2) == 0 && (value & 2)) {
                    ntsc_y_counter = (vga_scan_pos_y + 1) & 0x3ff;
                    ntsc_y_counter_prev = vga_scan_pos_y;
                }
                reg_composer[0] = (reg_composer[0] & ~0x7f) | (value & 0x7f);
                video_palette.dirty = true;
            } else {
                reg_composer[i] = value;
            }

            switch (i) {
                case 0x08:
                    fx_addr1_mode = value & 0x03;
                    fx_4bit_mode = (value & 0x04) >> 2;
                    fx_16bit_hop = (value & 0x08) >> 3;
                    fx_cache_byte_cycling = (value & 0x10) >> 4;
                    fx_cache_fill = (value & 0x20) >> 5;
                    fx_cache_write = (value & 0x40) >> 6;
                    fx_trans_writes = (value & 0x80) >> 7;
                    break;
                case 0x09:
                    fx_affine_tile_base = (value & 0xfc) << 9;
                    fx_affine_clip = (value & 0x02) >> 1;
                    fx_2bit_poly = (value & 0x01);
                    break;
                case 0x0a:
                    fx_affine_map_base = (value & 0xfc) << 9;
                    fx_affine_map_size = 2 << ((value & 0x03) << 1);
                    break;
                case 0x0b:
                    fx_cache_increment_mode = value & 0x01;
                    fx_cache_nibble_index = (value & 0x02) >> 1;
                    fx_cache_byte_index = (value & 0x0c) >> 2;
                    fx_multiplier = (value & 0x10) >> 4;
                    fx_subtract = (value & 0x20) >> 5;
                    if (value & 0x40) {
                        int32_t m_result = (int16_t)((fx_cache[1] << 8) | fx_cache[0]) * (int16_t)((fx_cache[3] << 8) | fx_cache[2]);
                        if (fx_subtract)
                            fx_mult_accumulator -= m_result;
                        else
                            fx_mult_accumulator += m_result;
                    }
                    if (value & 0x80)
                        fx_mult_accumulator = 0;
                    break;
                case 0x0c:
                    fx_x_pixel_increment = ((((reg_composer[0x0d] & 0x7f) << 15) + (reg_composer[0x0c] << 7))
                        | ((reg_composer[0x0d] & 0x40) ? 0xffc00000 : 0))
                        << 5*(!!(reg_composer[0x0d] & 0x80));
                    break;
                case 0x0d:
                    fx_x_pixel_increment = ((((reg_composer[0x0d] & 0x7f) << 15) + (reg_composer[0x0c] << 7))
                        | ((reg_composer[0x0d] & 0x40) ? 0xffc00000 : 0))
                        << 5*(!!(reg_composer[0x0d] & 0x80));
                    if (fx_addr1_mode == 1 || fx_addr1_mode == 2)
                        fx_x_pixel_position = (fx_x_pixel_position & 0x07ff0000) | 0x00008000;
                    break;
                case 0x0e:
                    fx_y_pixel_increment = ((((reg_composer[0x0f] & 0x7f) << 15) + (reg_composer[0x0e] << 7))
                        | ((reg_composer[0x0f] & 0x40) ? 0xffc00000 : 0))
                        << 5*(!!(reg_composer[0x0f] & 0x80));
                    break;
                case 0x0f:
                    fx_y_pixel_increment = ((((reg_composer[0x0f] & 0x7f) << 15) + (reg_composer[0x0e] << 7))
                        | ((reg_composer[0x0f] & 0x40) ? 0xffc00000 : 0))
                        << 5*(!!(reg_composer[0x0f] & 0x80));
                    if (fx_addr1_mode == 1 || fx_addr1_mode == 2)
                        fx_y_pixel_position = (fx_y_pixel_position & 0x07ff0000) | 0x00008000;
                    break;
                case 0x10:
                    fx_x_pixel_position = (fx_x_pixel_position & 0x0700ff80) | (value << 16);
                    fx_affine_prefetch();
                    break;
                case 0x11:
                    fx_x_pixel_position = (fx_x_pixel_position & 0x00ffff00) | ((value & 0x7) << 24) | (value & 0x80);
                    fx_affine_prefetch();
                    break;
                case 0x12:
                    fx_y_pixel_position = (fx_y_pixel_position & 0x0700ff80) | (value << 16);
                    fx_affine_prefetch();
                    break;
                case 0x13:
                    fx_y_pixel_position = (fx_y_pixel_position & 0x00ffff00) | ((value & 0x7) << 24) | (value & 0x80);
                    fx_affine_prefetch();
                    break;
                case 0x14:
                    fx_x_pixel_position = (fx_x_pixel_position & 0x07ff0080) | (value << 8);
                    break;
                case 0x15:
                    fx_y_pixel_position = (fx_y_pixel_position & 0x07ff0080) | (value << 8);
                    break;
                case 0x18: fx_cache[0] = value; break;
                case 0x19: fx_cache[1] = value; break;
                case 0x1a: fx_cache[2] = value; break;
                case 0x1b: fx_cache[3] = value; break;
            }
            break;
        }

        case 0x0D: case 0x0E: case 0x0F:
        case 0x10: case 0x11: case 0x12: case 0x13:
            reg_layer[0][reg - 0x0D] = value;
            refresh_layer_properties(0);
            break;

        case 0x14: case 0x15: case 0x16:
        case 0x17: case 0x18: case 0x19: case 0x1A:
            reg_layer[1][reg - 0x14] = value;
            refresh_layer_properties(1);
            break;

        case 0x1B: audio_render(); pcm_write_ctrl(value); break;
        case 0x1C: audio_render(); pcm_write_rate(value); break;
        case 0x1D: audio_render(); pcm_write_fifo(value); break;

        case 0x1E:
        case 0x1F:
            vera_spi_write(reg & 1, value);
            break;
    }
}

/* ================================================================
 * Public API — interface expected by agfa9000.c
 * ================================================================ */

void vc_init(vc_t *v)
{
    memset(v, 0, sizeof(*v));
    active_vera = v;
    video_reset();
    audio_reset();

    /* Set output mode to VGA with layer 0 enabled */
    reg_composer[0] = 0x21; /* VGA output + L0 enable */

    refresh_palette();

    const char *sd = getenv("A2_SDCARD");
    if (sd) sdcard_attach(sd);
}

uint8_t vc_read(vc_t *v, int reg)
{
    (void)v;
    return video_read_reg(reg & 0x1F);
}

void vc_write(vc_t *v, int reg, uint8_t val)
{
    (void)v;
    /* REGWATCH=<hex reg>: log every write to one VERA register with the PC of the writing... */
    /* g_vera_reset_pc_want: veracore cannot name the storing instruction (the
       in-write PC is unreliable, tick 253/254). Raise a flag instead and let
       boottest's single-step loop -- which knows the PC BEFORE each instruction
       -- report it. */
    if ((reg & 0x1F) == 0x05 && (val & 0x80)) g_vera_reset_writes++;
    { static int rw = -2;
      if (rw == -2) { const char *e = getenv("REGWATCH"); rw = e ? (int)strtol(e,0,16) : -1; }
      if (rw >= 0 && (rw == 0xFF || (reg & 0x1F) == rw)) {
          Puce6502State st; puce6502GetState(&st);
          printf("REGW %02X <- %02X  PC=$%04X\n", reg & 0x1F, val, st.pc);
      } }
    /* VRAMLOG=<n>: dump VRAM writes. MUST track ADDRSEL (CTRL bit0): regs 0-2 program ADDR0 or... */
    { static int vl_n = -1, vl_seen = 0, vl_skip = -1, vl_sel = 0;
      static unsigned vl_a[2] = {0,0};
      static int vl_step[2] = {0,0}, vl_nz = 0;
      static unsigned vl_lo = 0, vl_hi = 0xFFFFF;
      static const int vl_inc[16] = {0,1,2,4,8,16,32,64,128,256,512,40,80,160,320,640};
      if (vl_n < 0) { const char *e = getenv("VRAMLOG"); vl_n = e ? atoi(e) : 0;
                      e = getenv("VRAMLOGSKIP"); vl_skip = e ? atoi(e) : 0;
                      e = getenv("VRAMLOGLO"); vl_lo = e ? (unsigned)strtoul(e,0,16) : 0;
                      e = getenv("VRAMLOGHI"); vl_hi = e ? (unsigned)strtoul(e,0,16) : 0xFFFFF;
                      e = getenv("VRAMLOGNZ"); vl_nz = e ? atoi(e) : 0; }
      int r = reg & 0x1F;
      if (vl_n) {
          if (r == 5) vl_sel = val & 1;
          else if (r == 0) vl_a[vl_sel] = (vl_a[vl_sel] & 0xFFF00) | val;
          else if (r == 1) vl_a[vl_sel] = (vl_a[vl_sel] & 0xF00FF) | ((unsigned)val << 8);
          else if (r == 2) { vl_a[vl_sel] = (vl_a[vl_sel] & 0x0FFFF) | ((unsigned)(val & 0x0F) << 16);
                             vl_step[vl_sel] = vl_inc[val >> 4];
                             /* VRAMLOGADDR=1: report every ADDR0 (re)programming. */
                             { static int la = -1;
                               if (la == -1) { const char *e = getenv("VRAMLOGADDR"); la = e ? atoi(e) : 0; }
                               if (la && vl_seen < vl_n)
                                   printf("VRAMA%d %05X inc=%d\n", vl_sel, vl_a[vl_sel], vl_step[vl_sel]); } }
          else if (r == 3 || r == 4) { int p = (r == 4);
              /* Range-filter INSIDE the logger. Filtering the OUTPUT instead
                 lets the clear and CHR streams eat the whole budget before the
                 interesting writes happen -- which made the tilemap look like
                 it was never written at all. */
              if (vl_a[p] >= vl_lo && vl_a[p] <= vl_hi && (val || !vl_nz)) {
                  if (vl_skip > 0) vl_skip--;
                  else if (vl_seen < vl_n) { printf("VRAMW%d %05X %02X\n", p, vl_a[p], val); vl_seen++; } }
              vl_a[p] = (vl_a[p] + vl_step[p]) & 0xFFFFF; }
      } }
    video_write_reg(reg & 0x1F, val);
}

/* IIGS CPU-SPEED SCALING Every harness drives time as "puce6502Exec(N cpu cycles);... */
int g_a2_speed_permille = 1000;
static float vc_speed_carry = 0.0f;

int vc_tick(vc_t *v, int cycles)
{
    float vcyc = (float)cycles;
    if (g_a2_speed_permille != 1000) {
        float scaled = vcyc * 1000.0f / (float)g_a2_speed_permille + vc_speed_carry;
        float whole  = (float)(int)scaled;
        vc_speed_carry = scaled - whole;
        vcyc = whole;
    }
    bool new_frame = video_step_internal(vcyc);

    if (new_frame) {
        v->frame_count = frame_count;
    }

    audio_render();
    v->irq_out = video_get_irq_out();
    return v->irq_out;
}

void vc_render_frame(vc_t *v)
{
    if (!v->framebuffer) return;

    /* Copy internal framebuffer (BGRA uint8_t[]) to external uint32_t[] */
    memcpy(v->framebuffer, framebuffer, SCREEN_WIDTH * SCREEN_HEIGHT * 4);
}

int vc_irq_active(vc_t *v)
{
    return v->irq_out;
}

/* Raise the VSYNC IRQ flag (ISR bit0) directly, WITHOUT advancing the scanline engine. */
void vc_vsync_irq(vc_t *v)
{
    (void)v;
    isr |= 1;
}

void vera_dump_vram(const char *path)
{
    FILE *f = fopen(path, "wb");
    if (f) {
        fwrite(video_ram, 1, sizeof(video_ram), f);
        fclose(f);
        fprintf(stderr, "VRAM dumped to %s (%zu bytes)\n", path, sizeof(video_ram));
    }
}

/* expose internal VRAM to the VICE device shell (monitor tap). */
uint8_t *vc_vram(void) { return video_ram; }

/* expose the internal palette array (separate from VRAM $1FA00) so the
   NES->VERA setup can load a palette without driving the DATA port byte-by-byte.
   Caller must mark it dirty via vc_palette_dirty(). */
uint8_t *vc_palette(void) { return palette; }
void vc_palette_dirty(void) { video_palette.dirty = true; }

/* write a PSG register (reg = voice*4 + field) straight into the synth
   state, bypassing the DATA-port path. */
void vc_psg_write(uint8_t reg, uint8_t val) { psg_writereg(reg & 0x3f, val); }

/* set one sprite's 8 attribute bytes (bypasses the DATA-port path, so
   also refreshes the cached sprite properties). */
void vc_set_sprite(int idx, const uint8_t *b8)
{
    int i;
    if (idx < 0 || idx >= 128) return;
    for (i = 0; i < 8; i++) sprite_data[idx][i] = b8[i];
    refresh_sprite_properties((uint16_t)idx);
}

/* x16emu `-dump V` layout (what ~/Projects/BlastGridC64/tools/veracheck.py reads):
   VRAM[128K] + composer[256] + palette[512] + layer regs[2][7] + sprites[128][8]
   = 132,878 bytes.  The composer block is zero-padded to 256 bytes. */
int vc_dump_state(const char *path)
{
    uint8_t comp[256] = { 0 };
    FILE *f = fopen(path, "wb");
    if (!f) return 0;
    memcpy(comp, reg_composer, sizeof reg_composer < sizeof comp ? sizeof reg_composer : sizeof comp);
    int ok = fwrite(video_ram, 1, sizeof video_ram, f) == sizeof video_ram
          && fwrite(comp, 1, sizeof comp, f) == sizeof comp
          && fwrite(palette, 1, sizeof palette, f) == sizeof palette
          && fwrite(reg_layer, 1, sizeof reg_layer, f) == sizeof reg_layer
          && fwrite(sprite_data, 1, sizeof sprite_data, f) == sizeof sprite_data;
    return fclose(f) == 0 && ok;
}
