# a2emu — Apple II + A2VERA emulator

Reinette II plus (Apple ][+ / IIe, 65C02) with an A2VERA card: VERA video, PSG and the YM2151 at VERA base + $20.

- `make` builds `a2emu` (SDL2 window + audio) and `a2run` (headless test runner); `make windows` builds `a2emu.exe`.
- Put the Apple ROMs in `rom/` (`appleII+.rom`, `diskII.rom`, IIe 342-0135/0134 or 342-0303/0304); they are not included.
- ROM source: the images used during development came from the Asimov archive mirror,
  <https://mirrors.apple2.org.za/ftp.apple.asimov.net/emulators/rom_images/>
  (`apple_ii+_rom.zip` -- its F800-FFFF replaced by `Apple II+ - 341-0020 - Applesoft BASIC Autostart Monitor F800 - 2716.bin` from the collection below: the image in the zip has the IRQ vector patched to $FF59 instead of $FA40, so IRQs never reach $03FE -- `Apple Disk II 16 Sector Interface Card ROM P5 - 341-0027.bin-with-D4-D7 data bits swapped.bin`
  as `diskII.rom` -- it must start with `A2 20 A0 00`: the file without that suffix is a raw chip
  dump with scrambled lines and never boots a disk -- and the IIe CPU and video ROMs from
  `APPLE Computer and Peripheral Card Roms Collection.zip`).
  **The Apple ROMs are copyrighted by Apple: downloading them is not legal unless you actually own
  the original hardware.** If you do, dumping the chips of your own machine is the clean way to get them.
- Run: `./a2emu --model iie disk.po` (from this directory); `iiee` = enhanced IIe. `./a2emu --help` lists the options.
- `make install` copies binaries, `assets/` and `rom/` to `~/.local/share/a2emu-vera` and puts `a2emu`/`a2run`/`a2fs` in `~/bin` (override with `DATADIR=`/`BINDIR=`); `make uninstall` removes them but keeps the installed `rom/`.
- Keys: Ctrl+F1 / Alt+F1 save the floppy in drive 1 / 2 back to its file, F2 screenshot, F3 paste the clipboard as keystrokes,
  Shift/Ctrl+F4 volume, Shift/Ctrl+F5/F6 joystick release/action speed, Shift/Ctrl+F7 zoom,
  F9 warp, F10 pause, F11 reset, F12 help.
- Options and the environment variables they set: `--sdcard card.img` (`A2_SDCARD`) puts a raw image in the VERA SD slot, `--wav out.wav` (`A2_WAV`) records the stereo card audio, `--no-speaker` (`A2_SPEAKER=0`) drops the speaker stream, `--fps` (`A2_FPS=1`) prints the emulated clock and frame rate.
- `make install` also installs `a2fs` and the SDK (`a2fs sdk` prints where), for cc65 programs
  booted straight from a Disk II floppy, no DOS:
  - `a2fs mkimage prog.dsk prog.bin [NAME=FILE ...]`: bootable image (`sdk/boot.s` loads the
    program at $0900 with the Disk II PROM's sector reader) with files in A2FS, a minimal file
    system (directory on track 17, contiguous files with a CRC-16); `a2fs ls|get|put|rm` work on
    images, `a2fs --help` has the layout.
  - link with `sdk/a2emu-vera.cfg`; for files on the Apple add `sdk/a2fs.c sdk/rwts.s` and
    `#include "a2fs.h"` (`fs_start`, `fs_mount`, `fs_read`, `fs_write`, `blk_read`, `blk_write`).
    `rwts.s` reads and writes with the real drive's timing.
  - in a project: `SDK := $(shell a2fs sdk)`, then
    `cl65 -t apple2 -C $(SDK)/a2emu-vera.cfg -I $(SDK) -Osir -Cl -o prog.bin main.c $(SDK)/a2fs.c $(SDK)/rwts.s`
    and `a2fs mkimage prog.dsk prog.bin DATA=data.bin`.
- `examples/hello` (`make run`) and `examples/fstest` (`make test`: reads two files, writes a
  third and reads it back, then checks it from the host) use the SDK from this tree.
- The header of `a2run.c` lists the headless options (`--model`, `--aux`, `--hd`, `--frames`, `--keys`,
  `--input`, `--wav`, `--ymlog`, `--ym off`, ...).
- YM2151 at VERA base + $20 (`--ym off` / `A2_YM=0` removes it).  YM2151 core: ymfm (BSD-3, `extern/ymfm`).

A2VERA code: @fletto666 
YMFM integration: @sandlbn
