/* a2run_audio.h -- a2run's audio capture (plan E9): the VERA PSG + PCM mix
 * (vc_audio_render) to a WAV file, and a per-frame PSG register log. */
#ifndef A2RUN_AUDIO_H
#define A2RUN_AUDIO_H

int  a2audio_open_wav(const char *path);     /* 1 = ok */
int  a2audio_open_psglog(const char *path);  /* 1 = ok */
int  a2audio_open_psgvlog(const char *path); /* --psgvlog: sampled at each VSYNC */
void a2audio_vsync(long n);                  /* a VSYNC was just raised (before its IRQ) */
void a2audio_frame(long frame);              /* once per completed VERA frame */
void a2audio_close(void);                    /* finish the files (WAV sizes) */

#endif
