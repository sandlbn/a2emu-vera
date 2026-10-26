/* ym2151.h -- the A2VERA's YM2151 (OPM), plan E14.
 *
 * The A2VERA keeps the X16 register layout: the YM sits right after the VERA's
 * 32 registers, so at $Cn20 (address) / $Cn21 (data; read = status, bit 7 busy,
 * bits 0-1 timer A/B overflow).  Clock 3,579,545 Hz as on the X16.
 *
 * Time is the CPU cycle counter (`ticks`): the busy flag lasts 64 YM clocks
 * (~18 CPU cycles) after a data write, the timers count in YM clocks, and the
 * audio is generated lazily (one sample = 64 YM clocks, 55,930 Hz) only while a
 * consumer is attached (ym_audio_on).  A data write while busy is dropped and
 * counted (g_ym_busy_writes) with a stderr warning, like x16emu does.
 *
 * Env: A2_YM=0 removes the chip (an A2VERA without the FM part).
 * ymfm (BSD-3, extern/ymfm) is the core x16emu uses. */
#ifndef YM2151_H
#define YM2151_H
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

extern int      g_ym_enabled;              /* 1 = the chip answers at g_ym_base */
extern unsigned long g_ym_writes, g_ym_busy_writes;

void    ym_init(int enabled);              /* reset; enabled = 0 -> floating bus */
uint8_t ym_read(int reg);                  /* reg 0/1: status */
void    ym_write(int reg, uint8_t value);  /* reg 0 = address latch, 1 = data */
void    ym_audio_on(void);                 /* start generating samples from now */
/* add n stereo samples at `rate` Hz, ending at the current CPU cycle, into buf */
void    ym_mix(int16_t *buf, int n, double rate);
void    ym_log_open(FILE *f);              /* "<frame> <reg> <val>" per data write */
void    ym_log_frame(long frame);          /* the frame stamped on the next writes */

#ifdef __cplusplus
}
#endif
#endif
