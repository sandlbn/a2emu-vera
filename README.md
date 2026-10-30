# a2emu — Apple II + A2VERA emulator

Reinette II plus (Apple ][+ / IIe, 65C02) with an A2VERA card: VERA video, PSG and the YM2151 at VERA base + $20.

- `make` builds `a2emu` (SDL2 window + audio) and `a2run` (headless test runner); `make windows` builds `a2emu.exe`.
- Put the Apple ROMs in `rom/` (`appleII+.rom`, `diskII.rom`, IIe 342-0135/0134 or 342-0303/0304); they are not included.
- Run: `./a2emu --model iie disk.po` (from this directory); `iiee` = enhanced IIe. `./a2emu --help` lists the options.
- Keys: F9 warp, F10 pause, F11 reset, F12 help.
- Options and the environment variables they set: `--sdcard card.img` (`A2_SDCARD`) puts a raw image in the VERA SD slot, `--wav out.wav` (`A2_WAV`) records the stereo card audio, `--no-speaker` (`A2_SPEAKER=0`) drops the speaker stream, `--fps` (`A2_FPS=1`) prints the emulated clock and frame rate.
- `a2run --help` lists the headless options (`--hd`, `--frames`, `--wav`, `--ymlog`, `--ym off`, ...).
- YM2151 core: ymfm (BSD-3, `extern/ymfm`).

A2VERA code: @fletto666 
YMFM integration: @sandlbn
