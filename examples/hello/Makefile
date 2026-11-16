# hello.dsk: a cc65 C program on a self-booting Disk II image, no DOS needed.
#   make        -> hello.dsk
#   make run    -> boot it in a2emu (enhanced IIe; MODEL=plus for the ][+)
# Outside this tree, with the emulator installed: SDK := $(shell a2fs sdk), A2FS = a2fs
CL65  ?= cl65
SDK   ?= ../../sdk
A2FS  ?= python3 ../../tools/a2fs.py
MODEL ?= iiee

all: hello.dsk

hello.bin: hello.c $(SDK)/a2emu-vera.cfg
	$(CL65) -t apple2 -C $(SDK)/a2emu-vera.cfg -Osir -Cl -o $@ hello.c

$(SDK)/boot.sector: $(SDK)/boot.s
	$(MAKE) -C ../.. sdk/boot.sector

hello.dsk: hello.bin $(SDK)/boot.sector
	$(A2FS) mkimage $@ hello.bin

run: hello.dsk
	A2_MODEL=$(if $(filter plus,$(MODEL)),,$(MODEL)) a2emu hello.dsk

clean:
	rm -f *.o *.bin hello.dsk

.PHONY: all run clean
