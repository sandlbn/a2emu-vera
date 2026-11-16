#!/usr/bin/env python3
"""a2fs -- self-booting Apple II floppy images and their A2FS file system.

  a2fs mkimage OUT.dsk PROGRAM.bin [NAME=FILE ...] [--load ADDR] [--himem ADDR]
        bootable image: sdk/boot.sector loads PROGRAM (cc65, linked with
        sdk/a2emu-vera.cfg) at ADDR (default $0900) and runs it; FILEs go in A2FS
  a2fs ls IMAGE.dsk                       list the files
  a2fs get IMAGE.dsk NAME [OUTFILE]       extract a file (fails on a CRC mismatch)
  a2fs put IMAGE.dsk NAME=FILE ...        add or replace files
  a2fs rm IMAGE.dsk NAME ...              remove files (their blocks are not reused)
  a2fs sdk                                print the SDK directory (for Makefiles)

A2FS, written on the Apple by sdk/rwts.s + sdk/a2fs.c:
  block n = track n // 16, physical sector n % 16 (256 bytes)
  block 0         boot sector, blocks 1.. the program it loads
  block 272       directory (track 17, sector 0):
                    0  "A2FS"            4  next free block (u16 LE)
                    16 10 entries of 24 bytes: name[16] (NUL padded),
                       start block, length in bytes, CRC-16/CCITT-FALSE, spare (u16 LE)
  blocks 288..    file data, each file contiguous
The .dsk is in DOS sector order, as the emulator and floppy emulators expect.
"""
import argparse
import os
import struct
import sys

P2L = [0, 7, 14, 6, 13, 5, 12, 4, 11, 3, 10, 2, 9, 1, 8, 15]   # physical -> DOS logical
SIZE = 35 * 16 * 256
NBLOCKS = 35 * 16
DIRBLK, FIRSTDATA, NENTRIES, NAMELEN = 272, 288, 10, 16
ENTRY = struct.Struct('<16sHHHH')

# make install writes the installed SDK path here; from the source tree it is ../sdk
_INSTALLED_SDK = '@DATADIR@/sdk'
SDK = os.environ.get('A2SDK') or (
    _INSTALLED_SDK if not _INSTALLED_SDK.startswith('@')
    else os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'sdk'))


def die(msg):
    sys.exit(f'a2fs: {msg}')


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc


def nblocks(length):
    return (length + 255) // 256


def off(blk):
    track, sector = divmod(blk, 16)
    return track * 4096 + P2L[sector] * 256


def write_blocks(img, blk, data):
    for i in range(0, len(data), 256):
        o = off(blk + i // 256)
        chunk = data[i:i + 256]
        img[o:o + 256] = chunk + bytes(256 - len(chunk))


def read_blocks(img, blk, length):
    out = bytearray()
    while len(out) < length:
        o = off(blk)
        out += img[o:o + 256]
        blk += 1
    return bytes(out[:length])


class Image:
    def __init__(self, path):
        self.path = path
        self.img = bytearray(open(path, 'rb').read())
        if len(self.img) != SIZE:
            die(f'{path}: not a 140 KB .dsk image')
        self.dir = bytearray(read_blocks(self.img, DIRBLK, 256))
        if self.dir[:4] != b'A2FS':
            die(f'{path}: no A2FS directory')

    def entries(self):
        """(slot, name, start, length, crc) of the used entries"""
        for i in range(NENTRIES):
            name, start, length, crc, _ = ENTRY.unpack_from(self.dir, 16 + 24 * i)
            if name[0]:
                yield i, name.rstrip(b'\0').decode(), start, length, crc

    def find(self, name):
        return next((e for e in self.entries() if e[1] == name), None)

    @property
    def next_free(self):
        return struct.unpack_from('<H', self.dir, 4)[0]

    def put(self, name, data):
        if not 0 < len(name) <= NAMELEN:
            die(f'{name}: names are 1 to {NAMELEN} characters')
        e = self.find(name)
        if e and nblocks(len(data)) <= nblocks(e[3]):
            slot, start = e[0], e[2]                        # fits where it is
        else:
            slot = e[0] if e else next((i for i in range(NENTRIES)
                                        if not self.dir[16 + 24 * i]), None)
            if slot is None:
                die(f'{name}: directory full ({NENTRIES} files)')
            start = self.next_free
            if start + nblocks(len(data)) > NBLOCKS:
                die(f'{name}: disk full')
            struct.pack_into('<H', self.dir, 4, start + nblocks(len(data)))
        ENTRY.pack_into(self.dir, 16 + 24 * slot, name.encode(), start, len(data), crc16(data), 0)
        write_blocks(self.img, start, data)

    def rm(self, name):
        e = self.find(name)
        if not e:
            die(f'{name}: not found')
        self.dir[16 + 24 * e[0]:16 + 24 * (e[0] + 1)] = bytes(24)

    def save(self):
        write_blocks(self.img, DIRBLK, bytes(self.dir))
        open(self.path, 'wb').write(self.img)


def file_spec(spec):
    if '=' not in spec:
        die(f'{spec}: expected NAME=FILE')
    name, path = spec.split('=', 1)
    return name, open(path, 'rb').read()


def addr(text):
    return int(text.replace('$', '0x'), 0)


def cmd_mkimage(a):
    boot_path = a.boot or os.path.join(SDK, 'boot.sector')
    try:
        boot = bytearray(open(boot_path, 'rb').read())
    except OSError:
        die(f'{boot_path}: missing (make install, or make sdk/boot.sector in the source tree)')
    if len(boot) > 256 or boot[1] != 0x4C:
        die(f'{boot_path}: not the sdk boot sector')
    prog = open(a.program, 'rb').read()
    n = nblocks(len(prog))
    if a.load & 0xFF or a.load < 0x0900:
        die('--load must be page aligned and at least $0900 (the boot sector is at $0800)')
    if a.load + len(prog) > a.himem:
        die(f'{a.program}: {len(prog)} bytes at ${a.load:04X} go past HIMEM ${a.himem:04X}')
    if 1 + n > DIRBLK:
        die(f'{a.program}: too large for the space before the directory')
    struct.pack_into('<BHH', boot, 4, n, a.load, a.himem)       # boot.s parameter block
    img = bytearray(SIZE)
    write_blocks(img, 0, bytes(boot))
    write_blocks(img, 1, prog)
    d = bytearray(256)
    d[:4] = b'A2FS'
    struct.pack_into('<H', d, 4, FIRSTDATA)
    write_blocks(img, DIRBLK, bytes(d))
    open(a.out, 'wb').write(img)
    im = Image(a.out)
    for spec in a.files:
        im.put(*file_spec(spec))
    im.save()


def cmd_ls(a):
    im = Image(a.image)
    for _, name, start, length, crc in im.entries():
        print(f'{name:16} block {start:3}  {length:5} bytes  crc {crc:04X}')
    print(f'{NBLOCKS - im.next_free} blocks free')


def cmd_get(a):
    im = Image(a.image)
    e = im.find(a.name) or die(f'{a.name}: not found')
    data = read_blocks(im.img, e[2], e[3])
    open(a.outfile or a.name, 'wb').write(data)
    if crc16(data) != e[4]:
        die(f'{a.name}: crc {crc16(data):04X}, the directory says {e[4]:04X}')
    print(f'{a.name}: {e[3]} bytes, crc {e[4]:04X} ok')


def cmd_put(a):
    im = Image(a.image)
    for spec in a.files:
        im.put(*file_spec(spec))
    im.save()


def cmd_rm(a):
    im = Image(a.image)
    for name in a.names:
        im.rm(name)
    im.save()


def main():
    p = argparse.ArgumentParser(prog='a2fs', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('mkimage', help='bootable image with a program and files')
    s.add_argument('out')
    s.add_argument('program')
    s.add_argument('files', nargs='*', metavar='NAME=FILE')
    s.add_argument('--load', type=addr, default=0x0900, help='load address (default $0900)')
    s.add_argument('--himem', type=addr, default=0x9400, help='HIMEM / C stack top (default $9400)')
    s.add_argument('--boot', help='boot sector (default: the SDK one)')
    s.set_defaults(fn=cmd_mkimage)
    s = sub.add_parser('ls', help='list files')
    s.add_argument('image')
    s.set_defaults(fn=cmd_ls)
    s = sub.add_parser('get', help='extract a file')
    s.add_argument('image')
    s.add_argument('name')
    s.add_argument('outfile', nargs='?')
    s.set_defaults(fn=cmd_get)
    s = sub.add_parser('put', help='add or replace files')
    s.add_argument('image')
    s.add_argument('files', nargs='+', metavar='NAME=FILE')
    s.set_defaults(fn=cmd_put)
    s = sub.add_parser('rm', help='remove files')
    s.add_argument('image')
    s.add_argument('names', nargs='+', metavar='NAME')
    s.set_defaults(fn=cmd_rm)
    s = sub.add_parser('sdk', help='print the SDK directory')
    s.set_defaults(fn=lambda a: print(os.path.realpath(SDK)))
    a = p.parse_args()
    a.fn(a)


if __name__ == '__main__':
    main()
