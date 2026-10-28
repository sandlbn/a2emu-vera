/* vera. h -- Commander X16 VERA video/audio module emulation Ported from the Commander X16... */

#ifndef VC_H
#define VC_H

#include <stdint.h>
#include <stdbool.h>

/* VRAM size */
#define VC_VRAM_SIZE  (128 * 1024)

/* Display dimensions */
#define VC_SCREEN_W   640
#define VC_SCREEN_H   480

/* Register offsets (from base address) */
#define VC_ADDR_L       0x00
#define VC_ADDR_M       0x01
#define VC_ADDR_H       0x02
#define VC_DATA0        0x03
#define VC_DATA1        0x04
#define VC_CTRL         0x05
#define VC_IEN          0x06
#define VC_ISR          0x07
#define VC_IRQLINE_L    0x08
#define VC_DC_VIDEO     0x09
#define VC_DC_HSCALE    0x0A
#define VC_DC_VSCALE    0x0B
#define VC_DC_BORDER    0x0C
#define VC_L0_CONFIG    0x0D
#define VC_L0_MAPBASE   0x0E
#define VC_L0_TILEBASE  0x0F
#define VC_L0_HSCROLL_L 0x10
#define VC_L0_HSCROLL_H 0x11
#define VC_L0_VSCROLL_L 0x12
#define VC_L0_VSCROLL_H 0x13
#define VC_L1_CONFIG    0x14
#define VC_L1_MAPBASE   0x15
#define VC_L1_TILEBASE  0x16
#define VC_L1_HSCROLL_L 0x17
#define VC_L1_HSCROLL_H 0x18
#define VC_L1_VSCROLL_L 0x19
#define VC_L1_VSCROLL_H 0x1A
#define VC_AUDIO_CTRL   0x1B
#define VC_AUDIO_RATE   0x1C
#define VC_AUDIO_DATA   0x1D
#define VC_SPI_DATA     0x1E
#define VC_SPI_CTRL     0x1F

/* IEN/ISR bits */
#define VC_IRQ_VSYNC  0x01
#define VC_IRQ_LINE   0x02
#define VC_IRQ_SPRCOL 0x04
#define VC_IRQ_AFLOW  0x08

typedef struct vc_s {
    /* Framebuffer for SDL display (BGRA / SDL_PIXELFORMAT_RGB888) */
    uint32_t *framebuffer;

    /* Frame counter (incremented on vsync) */
    int frame_count;

    /* IRQ output state */
    int irq_out;
} vc_t;

/* Initialize VERA to power-on state */
void vc_init(vc_t *v);

/* Read a VERA register ($00-$1F) */
uint8_t vc_read(vc_t *v, int reg);

/* Write a VERA register ($00-$1F) */
void vc_write(vc_t *v, int reg, uint8_t val);

/* Advance by N CPU cycles. Generates scanlines, vsync IRQs.
 * Returns nonzero if IRQ line should be asserted. */
int vc_tick(vc_t *v, int cycles);

/* Render current frame to framebuffer (call once per vsync) */
void vc_render_frame(vc_t *v);

/* Check if IRQ is active */
int vc_irq_active(vc_t *v);

/* Raise the VSYNC IRQ (ISR bit0) directly, decoupled from the scanline engine.
   Call once per emulated frame; run 17030 CPU cycles/frame separately so Apple II
   speed is independent of VERA video timing. */
void vc_vsync_irq(vc_t *v);
extern unsigned long vc_vsync_raised;   /* VSYNCs the core has raised (a2run --psgvlog) */

/* internal 128KB VRAM pointer for the monitor tap */
uint8_t *vc_vram(void);
uint8_t *vc_palette(void);
void vc_palette_dirty(void);
void vc_set_sprite(int idx, const uint8_t *b8);
void vc_psg_write(uint8_t reg, uint8_t val);

/* Write the VERA state in x16emu -dump V layout (132,878 B); 1 = ok. */
int vc_dump_state(const char *path);

/* Mix PSG + PCM into an interleaved stereo int16 buffer (num_samples frames). */
void vc_audio_render(int16_t *buf, int num_samples);

#endif
