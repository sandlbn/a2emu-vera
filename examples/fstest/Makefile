# fstest.dsk: A2FS read/write test on a self-booting Disk II image, no DOS needed.
#   make        -> fstest.dsk (with DATA4K and DATA8K)
#   make test   -> run it headless in a2run (DISKTIME=1: timed disk), then check OUT3K
#                  from the host with a2fs
#   make run    -> boot it in a2emu (enhanced IIe; MODEL=plus for the ][+)
# Outside this tree, with the emulator installed: SDK := $(shell a2fs sdk), A2FS = a2fs
CL65  ?= cl65
SDK   ?= ../../sdk
A2FS  ?= python3 ../../tools/a2fs.py
MODEL ?= iiee
# a2run disk model: 1 = rotational timing (like a real drive), 0 = simple
DISKTIME ?= 1
# -Cl: static locals, the fastest here
COPT  ?= -Osir -Cl
SRC    = fstest.c $(SDK)/a2fs.c $(SDK)/rwts.s

all: fstest.dsk

fstest.bin: $(SRC) $(SDK)/a2fs.h $(SDK)/a2emu-vera.cfg
	$(CL65) -t apple2 -C $(SDK)/a2emu-vera.cfg -I $(SDK) $(COPT) -m fstest.map -o $@ $(SRC)

$(SDK)/boot.sector: $(SDK)/boot.s
	$(MAKE) -C ../.. sdk/boot.sector

data4k.bin data8k.bin:
	python3 -c "import random; random.seed('$@'); \
	  open('$@', 'wb').write(bytes(random.randrange(256) for _ in range($(if $(findstring 4k,$@),4096,8192))))"

fstest.dsk: fstest.bin $(SDK)/boot.sector data4k.bin data8k.bin
	$(A2FS) mkimage $@ fstest.bin DATA4K=data4k.bin DATA8K=data8k.bin

# a2run writes the floppy back at exit: work on a copy, then read OUT3K on the host
test: fstest.dsk
	cp fstest.dsk test.dsk
	A2_DISKTIME=$(DISKTIME) a2run --model $(MODEL) --disk1 test.dsk --frames 3000 --text --quiet | grep -v '^|  *|$$'
	$(A2FS) ls test.dsk
	$(A2FS) get test.dsk OUT3K out3k.bin

run: fstest.dsk
	A2_MODEL=$(if $(filter plus,$(MODEL)),,$(MODEL)) a2emu fstest.dsk

clean:
	rm -f *.o *.bin *.map fstest.dsk test.dsk $(SDK)/*.o

.PHONY: all test run clean
