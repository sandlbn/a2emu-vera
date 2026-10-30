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


/* reinette.c -- the SDL2 frontend (windows, keyboard, joystick, speaker audio).
 * The machine itself is in a2core.c. */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#ifdef MAC_FRONTEND
#include "sdl_stub.h"
#else
#include <SDL2/SDL.h>
#endif

#include "puce6502.h"
#include "veracore.h"
#include "a2core.h"
#include "ym2151.h"


//====================================================================== SPEAKER

#define audioBufferSize 4096                                                    // found to be large enought
Sint8 audioBuffer[2][audioBufferSize] = { 0 };                                  // see in main() for more details
SDL_AudioDeviceID audioDevice;

static void sdl_playSound(void) {
#ifndef MAC_FRONTEND   /* POSIniX/Mac: speaker audio deferred -- avoids pulling
                        * 64-bit float libgcc helpers into the text-mode port */
	static unsigned int lastTick = 0;
	static double frac = 0;
	static bool SPKR = false;                                                     // $C030 Speaker toggle

	if (!muted) {
		SPKR = !SPKR;                                                               // toggle speaker state
		double len = (ticks - lastTick) * (96000.0 / 1020484.0) + frac;
		lastTick = ticks;
		if (len > audioBufferSize) { frac = 0; return; }                            // long gap: queue nothing
		Uint32 length = (Uint32)len;
		frac = len - length;
		if (SDL_GetQueuedAudioSize(audioDevice) < 96000 / 10)
			SDL_QueueAudio(audioDevice, audioBuffer[SPKR], length);
	}
#endif
}



//==================================================== VERA PSG + PCM AUDIO (E9)
/* A second SDL device for the card: vc_audio_render at VERA's native 48,828 Hz,
   S16 stereo (SDL converts to what the hardware takes; the OS mixes it with the
   speaker device).  Rendered after every emulated chunk, as many samples as the
   CPU cycles it ran (1,020,484 Hz) are worth; queued unless more than ~0.1 s is
   already waiting (then dropped, so the sound never lags the picture). */
static SDL_AudioDeviceID veraAudio;

static void vera_audio_open(void) {
	SDL_AudioSpec want = { 48828, AUDIO_S16SYS, 2, 0, 1024, 0, 0, NULL, NULL };
	SDL_AudioSpec have;
	veraAudio = SDL_OpenAudioDevice(NULL, 0, &want, &have, 0);
	if (getenv("A2_FPS"))
		fprintf(stderr, "a2emu: card audio %d Hz %d ch fmt %04X (asked 48828 Hz)\n", have.freq, have.channels, have.format);
	if (veraAudio) { SDL_PauseAudioDevice(veraAudio, 0); ym_audio_on(); }
}

/* A2_WAV=FILE: also write the card's stereo stream (PSG + PCM + YM2151) to a WAV,
   16-bit stereo at 48,828 Hz, closed at exit (the header is fixed up then) */
static FILE *wav_out;
static uint32_t wav_n;
static void wav_hdr(void) {
	uint32_t data = wav_n * 4u, rate = 48828u;
	uint8_t h[44] = { 'R','I','F','F', 0,0,0,0, 'W','A','V','E', 'f','m','t',' ', 16,0,0,0, 1,0, 2,0,
	                  0,0,0,0, 0,0,0,0, 4,0, 16,0, 'd','a','t','a', 0,0,0,0 };
	uint32_t v[4] = { 36u + data, rate, rate * 4u, data };
	int at[4] = { 4, 24, 28, 40 };
	for (int k = 0; k < 4; k++) for (int b = 0; b < 4; b++) h[at[k] + b] = (uint8_t)(v[k] >> (8 * b));
	fseek(wav_out, 0, SEEK_SET);
	fwrite(h, 1, 44, wav_out);
	fseek(wav_out, 0, SEEK_END);
}
static void wav_open(void) {
	const char *p = getenv("A2_WAV");
	if (!p || !(wav_out = fopen(p, "wb"))) return;
	wav_hdr();
	ym_audio_on();                                            /* (also without a device) */
}
static void wav_close(void) {
	if (!wav_out) return;
	wav_hdr();
	fclose(wav_out);
	wav_out = NULL;
}

static void vera_audio_run(void) {
	static unsigned int last;
	static int init;
	static unsigned long long acc;
	static int16_t buf[2 * 4096];
	if (!init) { last = ticks; init = 1; }
	unsigned int d = ticks - last; last = ticks;
	acc += (unsigned long long)d * 48828125ull;               /* samples x 1000 */
	unsigned long long n = acc / 1020484000ull;
	acc %= 1020484000ull;
	if (n > 4096) n = 4096;
	if (!n || (!veraAudio && !wav_out)) return;
	vc_audio_render(buf, (int)n);
	ym_mix(buf, (int)n, 48828.125);                           /* + the YM2151 (E14) */
	if (wav_out && !muted) {                                   /* A2_WAV (little-endian host) */
		fwrite(buf, 4, (size_t)n, wav_out);
		wav_n += (uint32_t)n;
	}
	if (!veraAudio) return;
	/* the main loop's pacing keeps the queue near AUDIO_TARGET (the sound card is the
	   clock); only a big backlog (after warp / a pause) is dropped */
	if (!muted && SDL_GetQueuedAudioSize(veraAudio) < 48828u * 4u / 10u)
		SDL_QueueAudio(veraAudio, buf, (Uint32)n * 4u);
}

/* audio-locked pacing: queued samples the loop aims for (~60 ms: 3.5 loops) */
#define AUDIO_TARGET 3000u
static int audio_queued(void) {             /* samples waiting, -1 = no audio device */
	return veraAudio && !muted ? (int)(SDL_GetQueuedAudioSize(veraAudio) / 4u) : -1;
}

static void sdl_title(void *wdo, const char *title) { SDL_SetWindowTitle((SDL_Window *)wdo, title); }


//========================================================== PROGRAM ENTRY POINT

#ifndef MAC_FRONTEND   /* POSIniX/Mac: SDL main() disabled; see reinette_mac.c */
int main(int argc, char *argv[]) {


	//========================================================= SDL INITIALIZATION

	int zoom = 2;
	uint8_t tries = 0;                                                            // for disk ][ speed-up access
	SDL_Event event;
	SDL_bool running = true, paused = false, ctrl = false, shift = false, alt = false;

	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) < 0) {
		printf("failed to initialize SDL2 : %s", SDL_GetError());
		return -1;
	}

	SDL_Window *wdo = SDL_CreateWindow("Reinette ][+", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 280 * zoom, 192 * zoom, SDL_WINDOW_OPENGL);
	SDL_Renderer *rdr = SDL_CreateRenderer(wdo, -1, SDL_RENDERER_ACCELERATED);   /* paced by the wall clock below, not vsync */
	SDL_SetRenderDrawBlendMode(rdr, SDL_BLENDMODE_NONE);                          // SDL_BLENDMODE_BLEND);
	SDL_EventState(SDL_DROPFILE, SDL_ENABLE);                                     // ask SDL2 to read dropfile events
	SDL_RenderSetScale(rdr, zoom, zoom);
	SDL_Surface *sshot;                                                           // used later for the screenshots

	/* VERA card output: a second 640x480 window. The Apple II window stays the
	   text console; works for any disk that drives the card. */
	a2_speaker_hook = sdl_playSound;                                              // frontend hooks into a2core
	a2_title_hook   = sdl_title;
	vera_bus_init();                                                              // g_vera up, responds at $C400-$C41F
	SDL_Window   *vwdo = SDL_CreateWindow("VERA (a2_vera)", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 640, 480, SDL_WINDOW_SHOWN);
	SDL_Renderer *vrdr = SDL_CreateRenderer(vwdo, -1, SDL_RENDERER_SOFTWARE);
	SDL_Texture  *vtex = SDL_CreateTexture(vrdr, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, 640, 480);
	static uint32_t veraFB[640*480];


	//=================================================== SDL AUDIO INITIALIZATION

	/* two output streams, named for the mixer / a recorder: the Apple speaker (mono
	   clicks) and the A2VERA card (VERA PSG + PCM + YM2151, stereo: record this one).
	   A2_SPEAKER=0 leaves the speaker stream out, so only the card's stereo shows. */
	SDL_SetHint(SDL_HINT_APP_NAME, "a2emu");
	const char *spk = getenv("A2_SPEAKER");
	if (!spk || atoi(spk)) {
		SDL_SetHint(SDL_HINT_AUDIO_DEVICE_STREAM_NAME, "Apple II speaker (mono)");
		SDL_AudioSpec desired = { 96000, AUDIO_S8, 1, 0, 4096, 0, 0, NULL, NULL };
		audioDevice = SDL_OpenAudioDevice(NULL, 0, &desired, NULL, SDL_FALSE);    // get the audio device ID
		SDL_PauseAudioDevice(audioDevice, muted);                                 // unmute it (muted is false)
	}
	SDL_SetHint(SDL_HINT_AUDIO_DEVICE_STREAM_NAME, "A2VERA: VERA PSG + YM2151 (stereo)");
	vera_audio_open();                                                            // E9: the VERA PSG + PCM
	wav_open();                                                                   // A2_WAV=FILE
	uint8_t volume = 4;

	for (int i = 0; i < audioBufferSize; i++) {                                   // two audio buffers,
		audioBuffer[true][i] = volume;                                              // one used when SPKR is true
		audioBuffer[false][i] = -volume;                                            // the other when SPKR is false
	}


	//===================================== VARIABLES USED IN THE VIDEO PRODUCTION

	int TextCache[24][40] = { 0 };
	int LoResCache[24][40] = { 0 };
	int HiResCache[192][40] = { 0 };                                              // check which Hi-Res 7 dots needs redraw
	uint8_t previousBit[192][40] = { 0 };                                         // the last bit value of the byte before.

	enum characterAttribute { A_NORMAL, A_INVERSE, A_FLASH } glyphAttr;           // character attribute in TEXT
	uint8_t flashCycle = 0;                                                       // TEXT cursor flashes at 2Hz

	SDL_Rect drvRect[2] = { { 272, 188, 4, 4 }, { 276, 188, 4, 4 } };             // disk drive status squares
	SDL_Rect pixelGR = { 0, 0, 7, 4 };                                            // a block in LoRes
	SDL_Rect dstRect = { 0, 0, 7, 8 };                                            // the dst character in rdr
	SDL_Rect charRects[128];                                                      // the src from the norm and rev textures
	for (int c = 0; c < 128; c++) {                                               // index of the array = ascii code
		charRects[c].x = 7 * c;
		charRects[c].y = 0;
		charRects[c].w = 7;
		charRects[c].h = 8;
	}

	const int color[16][3] = {                                                    // the 16 low res colors
		{ 0,   0,   0	  }, { 226, 57,  86  }, { 28,  116, 205 }, { 126, 110, 173 },
		{ 31,  129, 128 }, { 137, 130, 122 }, { 86,  168, 228 }, { 144, 178, 223 },
		{ 151, 88,  34	}, { 234, 108, 21  }, { 158, 151, 143 }, { 255, 206, 240 },
		{ 144, 192, 49	}, { 255, 253, 166 }, { 159, 210, 213 }, { 255, 255, 255 }
	};

	const int hcolor[16][3] = {                                                   // the high res colors (2 light levels)
		{ 0,   0,   0   }, { 144, 192, 49  }, { 126, 110, 173 }, { 255, 255, 255 },
		{ 0,   0,   0   }, { 234, 108, 21  }, { 86,  168, 228 }, { 255, 255, 255 },
		{ 0,   0,   0   }, { 63,  55,	 86  }, { 72,  96,  25	}, { 255, 255, 255 },
		{ 0,   0,   0   }, { 43,  84,	 114 }, { 117, 54,  10	}, { 255, 255, 255 }
	};

	const int offsetGR[24] = {                                                    // helper for TEXT and GR video generation
		0x0000, 0x0080, 0x0100, 0x0180, 0x0200, 0x0280, 0x0300, 0x0380,             // lines 0-7
		0x0028, 0x00A8, 0x0128, 0x01A8, 0x0228, 0x02A8, 0x0328, 0x03A8,             // lines 8-15
		0x0050, 0x00D0, 0x0150, 0x01D0, 0x0250, 0x02D0, 0x0350, 0x03D0              // lines 16-23
	};

	const int offsetHGR[192] = {                                                  // helper for HGR video generation
		0x0000, 0x0400, 0x0800, 0x0C00, 0x1000, 0x1400, 0x1800, 0x1C00,             // lines 0-7
		0x0080, 0x0480, 0x0880, 0x0C80, 0x1080, 0x1480, 0x1880, 0x1C80,             // lines 8-15
		0x0100, 0x0500, 0x0900, 0x0D00, 0x1100, 0x1500, 0x1900, 0x1D00,             // lines 16-23
		0x0180, 0x0580, 0x0980, 0x0D80, 0x1180, 0x1580, 0x1980, 0x1D80,
		0x0200, 0x0600, 0x0A00, 0x0E00, 0x1200, 0x1600, 0x1A00, 0x1E00,
		0x0280, 0x0680, 0x0A80, 0x0E80, 0x1280, 0x1680, 0x1A80, 0x1E80,
		0x0300, 0x0700, 0x0B00, 0x0F00, 0x1300, 0x1700, 0x1B00, 0x1F00,
		0x0380, 0x0780, 0x0B80, 0x0F80, 0x1380, 0x1780, 0x1B80, 0x1F80,
		0x0028, 0x0428, 0x0828, 0x0C28, 0x1028, 0x1428, 0x1828, 0x1C28,
		0x00A8, 0x04A8, 0x08A8, 0x0CA8, 0x10A8, 0x14A8, 0x18A8, 0x1CA8,
		0x0128, 0x0528, 0x0928, 0x0D28, 0x1128, 0x1528, 0x1928, 0x1D28,
		0x01A8, 0x05A8, 0x09A8, 0x0DA8, 0x11A8, 0x15A8, 0x19A8, 0x1DA8,
		0x0228, 0x0628, 0x0A28, 0x0E28, 0x1228, 0x1628, 0x1A28, 0x1E28,
		0x02A8, 0x06A8, 0x0AA8, 0x0EA8, 0x12A8, 0x16A8, 0x1AA8, 0x1EA8,
		0x0328, 0x0728, 0x0B28, 0x0F28, 0x1328, 0x1728, 0x1B28, 0x1F28,
		0x03A8, 0x07A8, 0x0BA8, 0x0FA8, 0x13A8, 0x17A8, 0x1BA8, 0x1FA8,
		0x0050, 0x0450, 0x0850, 0x0C50, 0x1050, 0x1450, 0x1850, 0x1C50,
		0x00D0, 0x04D0, 0x08D0, 0x0CD0, 0x10D0, 0x14D0, 0x18D0, 0x1CD0,
		0x0150, 0x0550, 0x0950, 0x0D50, 0x1150, 0x1550, 0x1950, 0x1D50,
		0x01D0, 0x05D0, 0x09D0, 0x0DD0, 0x11D0, 0x15D0, 0x19D0, 0x1DD0,
		0x0250, 0x0650, 0x0A50, 0x0E50, 0x1250, 0x1650, 0x1A50, 0x1E50,
		0x02D0, 0x06D0, 0x0AD0, 0x0ED0, 0x12D0, 0x16D0, 0x1AD0, 0x1ED0,             // lines 168-183
		0x0350, 0x0750, 0x0B50, 0x0F50, 0x1350, 0x1750, 0x1B50, 0x1F50,             // lines 176-183
		0x03D0, 0x07D0, 0x0BD0, 0x0FD0, 0x13D0, 0x17D0, 0x1BD0, 0x1FD0              // lines 184-191
	};


	//================================= LOAD NORMAL AND REVERSE CHARACTERS BITMAPS

	char workDir[1000];                                                           // find the working directory
	int workDirSize = 0, i = 0;
	while (argv[0][i] != '\0') {
		workDir[i] = argv[0][i];
		if (argv[0][++i] == '\\') workDirSize = i + 1;                              // find the last '/' if any
	}

	SDL_Surface *tmpSurface;
	workDir[workDirSize] = 0;
	tmpSurface = SDL_LoadBMP(strncat(workDir, "assets/font-normal.bmp", 23));     // load the normal font
	SDL_Texture *normCharTexture = SDL_CreateTextureFromSurface(rdr, tmpSurface);
	SDL_FreeSurface(tmpSurface);

	workDir[workDirSize] = 0;
	tmpSurface = SDL_LoadBMP(strncat(workDir, "assets/font-reverse.bmp", 24));    // load the reverse font
	SDL_Texture *revCharTexture = SDL_CreateTextureFromSurface(rdr, tmpSurface);
	SDL_FreeSurface(tmpSurface);


	//================================================================== LOAD ROMS

	workDir[workDirSize] = 0;
	FILE *f = fopen(strncat(workDir, "rom/appleII+.rom", 17), "rb");              // load the Apple II+ ROM
	if (!f) {
		SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Fatal error", "Could not locate appleII+.rom in the rom folder", NULL);
		return 1;                                                                   // exit
	}
	if (fread(rom, 1, ROMSIZE, f) != ROMSIZE) {                                   // the file is too small
		SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Fatal error", "appleII+.rom should be exactly 12 KB", NULL);
		return 1;                                                                   // exit
	}
	fclose(f);

	workDir[workDirSize] = 0;
	f = fopen(strncat(workDir, "rom/diskII.rom", 15), "rb");                      // load the P5A disk ][ PROM
	if (!f) {
		SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Fatal error", "Could not locate diskII.rom in the rom folder", NULL);
		return 1;                                                                   // exit
	}
	if (fread(sl6, 1, 256, f) != 256) {                                           // file too small
		SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Fatal error", "diskII.rom should be exactly 256 bytes", NULL);
		return 1;                                                                   // exit
	}
	fclose(f);


	//========================================================== VM INITIALIZATION

	{ /* A2_MODEL=iie | iiee: Apple IIe (E13) with the IIe ROMs from rom/ (by part number) */
	  extern int g_cpu;
	  const char *model = getenv("A2_MODEL");
	  if (model && (!strcmp(model, "iie") || !strcmp(model, "iiee"))) {
		char romdir[1100];
		snprintf(romdir, sizeof romdir, "%.*srom", workDirSize, workDir);
		if (!a2_load_iie_roms(romdir, !strcmp(model, "iiee"))) {
			SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Fatal error", "IIe ROMs (342-030x / 342-013x) not found in the rom folder", NULL);
			return 1;
		}
		if (!strcmp(model, "iiee")) g_cpu = 2;                                    // 65C02
		{ const char *ax = getenv("A2_AUX"); g_iie_aux = ax && atoi(ax) >= 64; }  // A2_AUX=64: extended 80-col card
		SDL_SetWindowTitle(wdo, !strcmp(model, "iiee") ? "Reinette //e (enhanced)" : "Reinette //e");
	  } }

	if (argc > 1) {                                                               // floppy, or a hard-disk image
		FILE *hf = fopen(argv[1], "rb"); long hsz = 0;                              //   (> 140 KB, not .nib) -> slot 7
		if (hf) { fseek(hf, 0, SEEK_END); hsz = ftell(hf); fclose(hf); }
		if (hsz > 143360 && hsz != 232960) insertHardDisk(argv[1]);
		else insertFloppy(wdo, argv[1], 0);
	}
	if (argc > 2) insertFloppy(wdo, argv[2], 1);                                  // second floppy -> drive 2

	// reset the machine (IIe: soft switches + LC to ROM)
	a2_reset();

	// dirty hack, fix soon... if I understand why
	ram[0x4D] = 0xAA;   // Joust crashes if this memory location equals zero
	ram[0xD0] = 0xAA;   // Planetoids won't work if this memory location equals zero
	fetchSyncAll();     // ROM/sl6/ram all loaded -> build the execute-view mirror


	//================================================================== MAIN LOOP

	const char *rv_shotenv = getenv("REINETTE_SHOT");   /* headless verify: BMP after N frames */
	int rv_shotat = rv_shotenv ? atoi(rv_shotenv) : 0, rv_framen = 0;
	const char *rv_type = getenv("REINETTE_TYPE");      /* auto-type a command sequence */
	int rv_typeat = getenv("REINETTE_TYPEAT") ? atoi(getenv("REINETTE_TYPEAT")) : 900;
	int rv_ti = 0;                                      /* index into rv_type */

	/* real-time pacing: one loop = 17050 CPU cycles = 16.71 ms at 1,020,484 Hz (the rate
	   the VERA and the audio are timed by, a2core.c A2_CPU_HZ). Vsync alone is not a clock
	   (compositors may not block, displays may run at 120/144 Hz), so the loop sleeps to
	   the wall clock, trimmed by the audio queue: the sound card's clock wins, so the
	   samples never pile up or run dry (no dropped chunks, no wobble). F9 toggles warp. */
	const uint64_t pf_hz = SDL_GetPerformanceFrequency();
	const uint64_t pf_frame = (uint64_t)((double)pf_hz * 17050.0 / 1020484.0);
	uint64_t pf_next = SDL_GetPerformanceCounter() + pf_frame;
	bool warp = getenv("A2_WARP") != NULL;

	while (running) {

		if (!paused) {                                                              // the apple II is clocked at 1023000.0 Hhz
			/* chunk the frame so VERA is ticked alongside the CPU (504 steps/32 cyc =
			   one 268800-step VERA frame per displayed frame). Programs that pace off
			   VERA vsync ($C407 / ISR) need this interleave -- one tick/frame would hang
			   them. Same calibration as the headless harnesses. */
			for (int vc = 0; vc < 17050; vc += 32) cpu_vera_step();
			while (disk[curDrv].motorOn && ++tries)                                   // until motor is off or i reaches 255+1=0
				for (int vs = 0; vs < 5000; vs += 32) cpu_vera_step();  // speed up drive access -- tick VERA too (its raster must keep running or the scanline-based VERA detect fails)
			vera_audio_run();                                                         // E9
		}


		//=============================================================== USER INPUT

		while (SDL_PollEvent(&event)) {
			alt   = SDL_GetModState() & KMOD_ALT   ? true : false;
			ctrl  = SDL_GetModState() & KMOD_CTRL  ? true : false;
			shift = SDL_GetModState() & KMOD_SHIFT ? true : false;
			PB0 = alt   ? 0xFF : 0x00;                                                // update push button 0
			PB1 = ctrl  ? 0xFF : 0x00;                                                // update push button 1
			PB2 = shift ? 0xFF : 0x00;                                                // update push button 2

			if (event.type == SDL_QUIT) running = false;                              // WM sent TERM signal

			// if (event.type == SDL_WINDOWEVENT) {                                      // pause if the window loses focus
			//   if (event.window.event == SDL_WINDOWEVENT_FOCUS_LOST)
			//     paused = true;
			//   else if (event.window.event == SDL_WINDOWEVENT_FOCUS_GAINED)
			//     paused = false;
			// }

			if (event.type == SDL_DROPFILE) {                                         // user dropped a file
				char *filename = event.drop.file;                                       // get full pathname
				if (!insertFloppy(wdo, filename, alt))                                  // if ALT is pressed : drv 1 else drv 0
					SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Load", "Not a valid nib file", NULL);
				SDL_free(filename);                                                     // free filename memory
				paused = false;                                                         // might already be the case

				if (!(alt || ctrl)) {                                                   // if ALT or CTRL were not pressed
					ram[0x3F4] = 0;                                                       // unset the Power-UP byte
					a2_reset();                                                           // do a cold reset
					memset(ram, 0, sizeof(ram));
					fetchSyncAll();                                                       // ram wiped -> rebuild mirror
				}
			}

			if (event.type == SDL_KEYDOWN) {                                          // a key has been pressed
				switch (event.key.keysym.sym) {

				// EMULATOR CONTROLS :

				case SDLK_F1:                                                           // SAVES
					if (ctrl) {
						if (saveFloppy(0))
							SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_INFORMATION, "Save", "\nDisk 1 saved back to file\n", NULL);
						else
							SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Save", "\nTError while saving Disk 1\n", NULL);
					} else if (alt) {
						if (saveFloppy(1))
							SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_INFORMATION, "Save", "\nDisk 2 saved back to file\n", NULL);
						else
							SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Save", "\nError while saving Disk 2\n", NULL);
					} else {
						SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_WARNING, "Save", "CTRL-F1 to save D1\nALT-F1 to save D2\n", NULL);
					}
				break;


				case SDLK_F2: {                                                         // SCREENSHOTS
					sshot = SDL_GetWindowSurface(wdo);
					SDL_RenderReadPixels(rdr, NULL, SDL_GetWindowPixelFormat(wdo), sshot->pixels, sshot->pitch);
					workDir[workDirSize] = 0;
					int i = -1, a = 0, b = 0;
					while (disk[0].filename[++i] != '\0') {
						if (disk[0].filename[i] == '\\') a = i;
						if (disk[0].filename[i] == '.') b = i;
					}
					strncat(workDir, "screenshots\\", 14);
					if (a != b)
						strncat(workDir, disk[0].filename + a, b - a);
					else
						strncat(workDir, "no disk", 10);
					strncat(workDir, ".bmp", 5);
					SDL_SaveBMP(sshot, workDir);
					SDL_FreeSurface(sshot);
					}
				break;

				case SDLK_F3:                                                           // PASTE text from clipboard
					if (SDL_HasClipboardText()) {
						char *clipboardText = SDL_GetClipboardText();
						int c = 0;
						while (clipboardText[c]) {                                          // all chars until ascii NUL
							KBD = clipboardText[c++] | 0x80;                                  // set bit7
							if (KBD == 0x8A) KBD = 0x8D;                                      // translate Line Feed to Carriage Ret
							puce6502Exec(400000);                                             // give cpu (and applesoft) some cycles to process each char
						}
						SDL_free(clipboardText);                                            // release the ressource
					}
				break;

				case SDLK_F4:                                                           // VOLUME
					if (shift && (volume < 120)) volume++;                                // increase volume
					if (ctrl && (volume > 0)) volume--;                                   // decrease volume
					if (!ctrl && !shift) muted = !muted;                                  // toggle mute / unmute
					for (int i = 0; i < audioBufferSize; i++) {                           // update the audio buffers,
						audioBuffer[true][i] = volume;                                      // one used when SPKR is true
						audioBuffer[false][i] = -volume;                                    // the other when SPKR is false
					}
				break;

				case SDLK_F5:                                                           // JOYSTICK Release Speed
					if (shift && (GCReleaseSpeed < 127)) GCReleaseSpeed += 2;             // increase Release Speed
					if (ctrl && (GCReleaseSpeed > 1)) GCReleaseSpeed -= 2;                // decrease Release Speed
					if (!ctrl && !shift) GCReleaseSpeed = 8;                              // reset Release Speed to 8
				break;

				case SDLK_F6:                                                           // JOYSTICK Action Speed
					if (shift && (GCActionSpeed < 127)) GCActionSpeed += 2;               // increase Action Speed
					if (ctrl && (GCActionSpeed > 1)) GCActionSpeed -= 2;                  // decrease Action Speed
					if (!ctrl && !shift) GCActionSpeed = 8;                               // reset Action Speed to 8
				break;

				case SDLK_F7:                                                           // ZOOM
					if (shift && (zoom < 8)) zoom++;                                      // zoom in
					if (ctrl && (zoom > 1)) zoom--;                                       // zoom out
					if (!ctrl && !shift) zoom = 2;                                        // reset zoom to 2
					SDL_SetWindowSize(wdo, 280 * zoom, 192 * zoom);                       // update window size
					SDL_RenderSetScale(rdr, zoom, zoom);                                  // update renderer size
				break;

				case SDLK_F9: warp = !warp; break;                                       // toggle warp speed
				case SDLK_F10: paused = !paused; break;                                  // toggle pause

				case SDLK_F11: a2_reset(); break;                                       // simulate a reset

				case SDLK_F12:                                                          // help box
					SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_INFORMATION, "Help",
						  "\tReinette ][ plus v0.4.8\n"
							"\n"
              "ctrl F1\twrites the changes of the floppy in drive 0\n"
              "alt F1\twrites the changes of the floppy in drive 1\n"
              "\n"
							"F2\tsave a screenshot into the screenshots directory\n"
              "F3\tpaste text from clipboard\n"
              "\n"
							"F4\tmute / un-mute sound\n"
              "shift F4\tincrease volume\n"
              "ctrl F4\tdecrease volume\n"
              "\n"
							"F5\treset joystick release speed\n"
              "shift F5\tincrease joystick release speed\n"
              "crtl F5\tdecrease joystick release speed\n"
              "\n"
							"F6\treset joystick action speed\n"
              "shift F6\tincrease joystick action speed\n"
              "crtl F6\tdecrease joystick action speed\n"
              "\n"
							"F7\treset zoom to 2:1\n"
              "shift F7\tincrease zoom up to 8:1\n"
              "ctrl F7\tdecrease zoom down to 1:1\n"
							"\n"
              "F9\twarp speed on / off\n"
              "F10\tpause / un-pause the emulator\n"
							"F11\treset\n"
              "\n"
							"F12\tthis help\n"
              "\n"
							"More information at github.com/ArthurFerreira2\n", NULL);
				break;

				// EMULATED KEYS :

				case SDLK_a:            KBD = ctrl ? 0x81: 0xC1;   break;               // a
				case SDLK_b:            KBD = ctrl ? 0x82: 0xC2;   break;               // b STX
				case SDLK_c:            KBD = ctrl ? 0x83: 0xC3;   break;               // c ETX
				case SDLK_d:            KBD = ctrl ? 0x84: 0xC4;   break;               // d EOT
				case SDLK_e:            KBD = ctrl ? 0x85: 0xC5;   break;               // e
				case SDLK_f:            KBD = ctrl ? 0x86: 0xC6;   break;               // f ACK
				case SDLK_g:            KBD = ctrl ? 0x87: 0xC7;   break;               // g BELL
				case SDLK_h:            KBD = ctrl ? 0x88: 0xC8;   break;               // h BS
				case SDLK_i:            KBD = ctrl ? 0x89: 0xC9;   break;               // i HTAB
				case SDLK_j:            KBD = ctrl ? 0x8A: 0xCA;   break;               // j LF
				case SDLK_k:            KBD = ctrl ? 0x8B: 0xCB;   break;               // k VTAB
				case SDLK_l:            KBD = ctrl ? 0x8C: 0xCC;   break;               // l FF
				case SDLK_m:            KBD = ctrl ? shift ? 0x9D: 0x8D: 0xCD; break;   // m CR ]
				case SDLK_n:            KBD = ctrl ? shift ? 0x9E: 0x8E: 0xCE; break;   // n ^
				case SDLK_o:            KBD = ctrl ? 0x8F: 0xCF;   break;               // o
				case SDLK_p:            KBD = ctrl ? shift ? 0x80: 0x90: 0xD0; break;   // p @
				case SDLK_q:            KBD = ctrl ? 0x91: 0xD1;   break;               // q
				case SDLK_r:            KBD = ctrl ? 0x92: 0xD2;   break;               // r
				case SDLK_s:            KBD = ctrl ? 0x93: 0xD3;   break;               // s ESC
				case SDLK_t:            KBD = ctrl ? 0x94: 0xD4;   break;               // t
				case SDLK_u:            KBD = ctrl ? 0x95: 0xD5;   break;               // u NAK
				case SDLK_v:            KBD = ctrl ? 0x96: 0xD6;   break;               // v
				case SDLK_w:            KBD = ctrl ? 0x97: 0xD7;   break;               // w
				case SDLK_x:            KBD = ctrl ? 0x98: 0xD8;   break;               // x CANCEL
				case SDLK_y:            KBD = ctrl ? 0x99: 0xD9;   break;               // y
				case SDLK_z:            KBD = ctrl ? 0x9A: 0xDA;   break;               // z
				case SDLK_LEFTBRACKET:  KBD = ctrl ? 0x9B: 0xDB;   break;               // [ {
				case SDLK_BACKSLASH:    KBD = ctrl ? 0x9C: 0xDC;   break;               // \ |
				case SDLK_RIGHTBRACKET: KBD = ctrl ? 0x9D: 0xDD;   break;               // ] }
				case SDLK_BACKSPACE:    KBD = ctrl ? 0xDF: 0x88;   break;               // BS
				case SDLK_0:            KBD = shift? 0xA9: 0xB0;   break;               // 0 )
				case SDLK_1:            KBD = shift? 0xA1: 0xB1;   break;               // 1 !
				case SDLK_2:            KBD = shift? 0xC0: 0xB2;   break;               // 2
				case SDLK_3:            KBD = shift? 0xA3: 0xB3;   break;               // 3 #
				case SDLK_4:            KBD = shift? 0xA4: 0xB4;   break;               // 4 $
				case SDLK_5:            KBD = shift? 0xA5: 0xB5;   break;               // 5 %
				case SDLK_6:            KBD = shift? 0xDE: 0xB6;   break;               // 6 ^
				case SDLK_7:            KBD = shift? 0xA6: 0xB7;   break;               // 7 &
				case SDLK_8:            KBD = shift? 0xAA: 0xB8;   break;               // 8 *
				case SDLK_9:            KBD = shift? 0xA8: 0xB9;   break;               // 9 (
				case SDLK_QUOTE:        KBD = shift? 0xA2: 0xA7;   break;               // ' "
				case SDLK_EQUALS:       KBD = shift? 0xAB: 0xBD;   break;               // = +
				case SDLK_SEMICOLON:    KBD = shift? 0xBA: 0xBB;   break;               // ; :
				case SDLK_COMMA:        KBD = shift? 0xBC: 0xAC;   break;               // , <
				case SDLK_PERIOD:       KBD = shift? 0xBE: 0xAE;   break;               // . >
				case SDLK_SLASH:        KBD = shift? 0xBF: 0xAF;   break;               // / ?
				case SDLK_MINUS:        KBD = shift? 0xDF: 0xAD;   break;               // - _
				case SDLK_BACKQUOTE:    KBD = shift? 0xFE: 0xE0;   break;               // ` ~
				case SDLK_LEFT:         KBD = 0x88;                break;               // BS
				case SDLK_RIGHT:        KBD = 0x95;                break;               // NAK
				case SDLK_UP:           KBD = 0x8B;                break;               // up arrow (VT, ctrl-K on II+)
				case SDLK_DOWN:         KBD = 0x8A;                break;               // down arrow (LF, ctrl-J on II+)
				case SDLK_SPACE:        KBD = 0xA0;                break;
				case SDLK_ESCAPE:       KBD = 0x9B;                break;               // ESC
				case SDLK_RETURN:       KBD = 0x8D;                break;               // CR

				// EMULATED JOYSTICK :

				case SDLK_KP_1:         GCD[0] = -1; GCA[0] = 1;   break;               // pdl0 <-
				case SDLK_KP_3:         GCD[0] = 1;  GCA[0] = 1;   break;               // pdl0 ->
				case SDLK_KP_5:         GCD[1] = -1; GCA[1] = 1;   break;               // pdl1 <-
				case SDLK_KP_2:         GCD[1] = 1;  GCA[1] = 1;   break;               // pdl1 ->
				}
			}

			if (g_akd && (event.type == SDL_KEYDOWN || event.type == SDL_KEYUP)) {   // IIe any-key-down
				int nk; const Uint8 *ks = SDL_GetKeyboardState(&nk); int down = 0;
				for (int k = 0; k < nk; k++)
					if (ks[k] && !(k >= SDL_SCANCODE_LCTRL && k <= SDL_SCANCODE_RGUI)) { down = 1; break; }
				g_kbdDown = down ? 0x80 : 0;
			}

			if (event.type == SDL_KEYUP) {
				switch (event.key.keysym.sym) {
				case SDLK_KP_1:         GCD[0] = 1;  GCA[0] = 0;   break;               // pdl0 ->
				case SDLK_KP_3:         GCD[0] = -1; GCA[0] = 0;   break;               // pdl0 <-
				case SDLK_KP_5:         GCD[1] = 1;  GCA[1] = 0;   break;               // pdl1 ->
				case SDLK_KP_2:         GCD[1] = -1; GCA[1] = 0;   break;               // pdl1 <-
				}
			}
		}

		for (int pdl = 0; pdl < 2; pdl++) {                                         // update the two paddles positions
			if (GCA[pdl]) {                                                           // actively pushing the stick
				GCP[pdl] += GCD[pdl] * GCActionSpeed;
				if (GCP[pdl] > 255) GCP[pdl] = 255;
				if (GCP[pdl] < 0)   GCP[pdl] = 0;
			} else {                                                                  // the stick is return back to center
				GCP[pdl] += GCD[pdl] * GCReleaseSpeed;
				if (GCD[pdl] == 1  && GCP[pdl] > 127) GCP[pdl] = 127;
				if (GCD[pdl] == -1 && GCP[pdl] < 127) GCP[pdl] = 127;
			}
		}


		//============================================================= VIDEO OUTPUT

		// HIGH RES GRAPHICS
		if (!TEXT && HIRES) {
			uint16_t word;
			uint8_t bits[16], bit, pbit, colorSet, even;
			uint16_t vRamBase = 0x2000 + PAGE2 * 0x2000;
			uint8_t lastLine = MIXED ? 160 : 192;
			uint8_t colorIdx = 0;                                                     // to index the color arrays

			for (int line = 0; line < lastLine; line++) {                             // for every line
				for (int col = 0; col < 40; col += 2) {                                 // for every 7 horizontal dots
					int x = col * 7;
					even = 0;

					word = (uint16_t)(ram[vRamBase + offsetHGR[line] + col + 1]) << 8;    // store the two next bytes into 'word'
					word +=           ram[vRamBase + offsetHGR[line] + col];              // in reverse order

					if (HiResCache[line][col] != word || !flashCycle) {                   // check if this group of 7 dots need a redraw

						for (bit=0; bit < 16; bit++)                                        // store all bits 'word' into 'bits'
							bits[bit] = (word >> bit) & 1;
						colorSet = bits[7] * 4;                                             // select the right color set
						pbit = previousBit[line][col];                                      // the bit value of the left dot
						bit = 0;                                                            // starting at 1st bit of 1st byte

						while (bit < 15) {                                                  // until we reach bit7 of 2nd byte
							if (bit == 7) {                                                   // moving into the second byte
								colorSet = bits[15] * 4;                                        // update the color set
								bit++;                                                          // skip bit 7
							}
							colorIdx = even + colorSet + (bits[bit] << 1) + (pbit);
							SDL_SetRenderDrawColor(rdr, hcolor[colorIdx][0], hcolor[colorIdx][1], hcolor[colorIdx][2], SDL_ALPHA_OPAQUE);
							SDL_RenderDrawPoint(rdr, x++, line);
							pbit = bits[bit++];                                               // proceed to the next pixel
							even = even ? 0 : 8;                                              // one pixel every two is darker
						}

						HiResCache[line][col] = word;                                       // update the video cache
						if ((col < 37) && (previousBit[line][col + 2] != pbit)) {           // check color franging effect on the dot after
							previousBit[line][col + 2] = pbit;                                // set pbit and clear the
							HiResCache[line][col + 2] = -1;                                   // video cache for next dot
						}
					}                                                                     // if (HiResCache[line][col] ...
				}
			}
		}

		// lOW RES GRAPHICS
		else if (!TEXT) {                                                           // and not in HIRES
			uint16_t vRamBase = 0x400 + PAGE2 * 0x0400;
			uint8_t lastLine = MIXED ? 20 : 24;
			uint8_t glyph;                                                            // 2 blocks in GR
			uint8_t colorIdx = 0;                                                     // to index the color arrays

			for (int col = 0; col < 40; col++) {                                      // for each column
				pixelGR.x = col * 7;
				for (int line = 0; line < lastLine; line++) {                           // for each row
					pixelGR.y = line * 8;                                                 // first block

					glyph = ram[vRamBase + offsetGR[line] + col];                         // read video memory
					if (LoResCache[line][col] != glyph || !flashCycle) {
						LoResCache[line][col] = glyph;

						colorIdx = glyph & 0x0F;                                              // first nibble
						SDL_SetRenderDrawColor(rdr, color[colorIdx][0], color[colorIdx][1], color[colorIdx][2], SDL_ALPHA_OPAQUE);
						SDL_RenderFillRect(rdr, &pixelGR);

						pixelGR.y += 4;                                                       // second block
						colorIdx = (glyph & 0xF0) >> 4;                                       // second nibble
						SDL_SetRenderDrawColor(rdr, color[colorIdx][0], color[colorIdx][1], color[colorIdx][2], SDL_ALPHA_OPAQUE);
						SDL_RenderFillRect(rdr, &pixelGR);
					}
				}
			}
		}

		// TEXT 40 COLUMNS
		if (TEXT || MIXED) {                                                        // not Full Graphics
			uint16_t vRamBase = 0x400 +PAGE2 * 0x0400;
			uint8_t firstLine = TEXT ? 0 : 20;
			uint8_t glyph;                                                            // a TEXT character

			for (int col = 0; col < 40; col++) {                                      // for each column
				dstRect.x = col * 7;
				for (int line = firstLine; line < 24; line++) {                         // for each row
					dstRect.y = line * 8;

					glyph = ram[vRamBase + offsetGR[line] + col];                         // read video memory
					if (glyph > 0x7F) glyphAttr = A_NORMAL;                               // is NORMAL ?
					else if (glyph < 0x40) glyphAttr = A_INVERSE;                         // is INVERSE ?
					else glyphAttr = A_FLASH;                                             // it's FLASH !

					if (glyphAttr==A_FLASH || TextCache[line][col]!=glyph || !flashCycle){
						TextCache[line][col] = glyph;

						glyph &= 0x7F;                                                        // unset bit 7
						if (glyph > 0x5F) glyph &= 0x3F;                                      // shifts to match
						if (glyph < 0x20) glyph |= 0x40;                                      // the ASCII codes

						if (glyphAttr==A_NORMAL || (glyphAttr==A_FLASH && flashCycle<15))
							SDL_RenderCopy(rdr, normCharTexture, &charRects[glyph], &dstRect);
						else
							SDL_RenderCopy(rdr, revCharTexture, &charRects[glyph], &dstRect);
					}
				}
			}
		}


		//====================================================== DISPLAY DISK STATUS

		if (disk[curDrv].motorOn) {                                                 // drive is active
			if (disk[curDrv].writeMode)
				SDL_SetRenderDrawColor(rdr, 255, 0, 0, 85);                             // red for writes
			else
				SDL_SetRenderDrawColor(rdr, 0, 255, 0, 85);                             // green for reads
			SDL_RenderFillRect(rdr, &drvRect[curDrv]);                                // square actually
		}


		//========================================================= SDL RENDER FRAME

		if (++flashCycle == 30)                                                     // increase cursor flash cycle
			flashCycle = 0;                                                           // reset to zero every half second
		SDL_RenderPresent(rdr);                                                     // swap buffers

		/* --- composite VERA's current VRAM+palette and show it --- */
		vc_render_frame(&g_vera);
		for (int vk = 0; vk < 640*480; vk++) veraFB[vk] = 0xFF000000u | g_vera.framebuffer[vk];
		SDL_UpdateTexture(vtex, NULL, veraFB, 640*4);
		SDL_RenderClear(vrdr);
		SDL_RenderCopy(vrdr, vtex, NULL, NULL);
		SDL_RenderPresent(vrdr);

		rv_framen++;
		/* auto-type: after rv_typeat frames, feed one char every 12 frames into $C000
		   (bit7 set). '~' in the string -> a longer pause (wait for a program to load). */
		if (rv_type && rv_framen >= rv_typeat && rv_type[rv_ti]) {
			static int rv_hold = 0;
			if (--rv_hold <= 0) {
				char ch = rv_type[rv_ti++];
				if (ch == '~')      { rv_hold = 240; }               /* ~2s pause */
				else { KBD = (ch == '\n' ? 0x8D : (uint8_t)(ch & 0x7F) | 0x80); rv_hold = 12; }
			}
		}

		if (rv_shotat && rv_framen >= rv_shotat) {          /* verify: dump both renderers */
			SDL_Surface *sa = SDL_CreateRGBSurfaceWithFormat(0, 280*zoom, 192*zoom, 32, SDL_PIXELFORMAT_ARGB8888);
			SDL_RenderReadPixels(rdr, NULL, SDL_PIXELFORMAT_ARGB8888, sa->pixels, sa->pitch);
			SDL_SaveBMP(sa, "/tmp/rv_apple.bmp"); SDL_FreeSurface(sa);
			SDL_Surface *sv = SDL_CreateRGBSurfaceWithFormat(0, 640, 480, 32, SDL_PIXELFORMAT_ARGB8888);
			SDL_RenderReadPixels(vrdr, NULL, SDL_PIXELFORMAT_ARGB8888, sv->pixels, sv->pitch);
			SDL_SaveBMP(sv, "/tmp/rv_vera.bmp"); SDL_FreeSurface(sv);
			running = false;
		}

		if (getenv("A2_FPS")) {                             /* A2_FPS=1: emulated clock and VERA frame rate */
			static uint64_t fps_t0 = 0; static unsigned fps_c0, fps_v0;
			uint64_t t = SDL_GetPerformanceCounter();
			if (!fps_t0) { fps_t0 = t; fps_c0 = ticks; fps_v0 = (unsigned)vc_vsync_raised; }
			if (t - fps_t0 >= pf_hz) {
				double sec = (double)(t - fps_t0) / (double)pf_hz;
				fprintf(stderr, "a2emu: %.3f MHz, %.2f VERA frames/s (%s), audio queue %d samples\n",
				        (double)(ticks - fps_c0) / sec / 1e6, (double)((unsigned)vc_vsync_raised - fps_v0) / sec,
				        warp ? "warp" : "real time", audio_queued());
				fps_t0 = t; fps_c0 = ticks; fps_v0 = (unsigned)vc_vsync_raised;
			}
		}
		if (!warp && !rv_shotat) {                          /* real-time pacing */
			int q = audio_queued();                         /* trim by the audio queue: */
			if (q >= 0) {                                   /* 1/8 of the error per loop, */
				int64_t e = ((int64_t)q - (int64_t)AUDIO_TARGET) * (int64_t)pf_hz / 48828 / 8;
				int64_t lim = (int64_t)pf_frame / 4;        /* at most +-25 % of a loop */
				if (e > lim) e = lim;
				if (e < -lim) e = -lim;
				pf_next = (uint64_t)((int64_t)pf_next + e);
			}
			uint64_t now = SDL_GetPerformanceCounter();
			if (now < pf_next) {
				uint64_t ms = (pf_next - now) * 1000 / pf_hz;
				if (ms > 1) SDL_Delay((Uint32)(ms - 1));
				while (SDL_GetPerformanceCounter() < pf_next) {}
				pf_next += pf_frame;
			} else if (now - pf_next > 4 * pf_frame) {
				pf_next = now + pf_frame;                       /* fell far behind: resync, don't sprint */
			} else {
				pf_next += pf_frame;
			}
		}
	}                                                                             // while (running)


	//================================================ RELEASE RESSOURSES AND EXIT

	a2_flush_disks();                                                             // E6: floppy changes back to their files

	SDL_AudioQuit();
	wav_close();
	SDL_Quit();
	return 0;
}
#endif /* MAC_FRONTEND */

