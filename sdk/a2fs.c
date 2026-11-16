/* a2fs.c -- 6-and-2 sector encoding over rwts.s, and the A2FS file system:
 * a directory block at track 17 sector 0 and contiguous files (layout in fs.py). */
#include <string.h>
#include "a2fs.h"

#define NIBBUF ((unsigned char *)0x9400)   /* rwts.s */

extern unsigned char rw_track, rw_sector;
void __fastcall__ rw_init(void);
void __fastcall__ rw_off(void);
unsigned char __fastcall__ rw_rawread(void);
unsigned char __fastcall__ rw_rawwrite(void);

static const unsigned char gcr6[64] = {
    0x96,0x97,0x9A,0x9B,0x9D,0x9E,0x9F,0xA6,0xA7,0xAB,0xAC,0xAD,0xAE,0xAF,0xB2,0xB3,
    0xB4,0xB5,0xB6,0xB7,0xB9,0xBA,0xBB,0xBC,0xBD,0xBE,0xBF,0xCB,0xCD,0xCE,0xCF,0xD3,
    0xD6,0xD7,0xD9,0xDA,0xDB,0xDC,0xDD,0xDE,0xDF,0xE5,0xE6,0xE7,0xE9,0xEA,0xEB,0xEC,
    0xED,0xEE,0xEF,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,0xF9,0xFA,0xFB,0xFC,0xFD,0xFE,0xFF
};
static unsigned char inv[256];             /* disk nibble -> 6 bits, $FF = invalid */
static unsigned char nb[342];              /* 86 bytes of low bit pairs, 256 of high 6 bits */

/* the two low bits of each byte go to nb[0..85], swapped (DOS 3.3 order) */
#define SWAP2(b) ((((b) & 1) << 1) | (((b) & 2) >> 1))

static void encode(const unsigned char *sd)
{
    unsigned char *p = NIBBUF;
    unsigned char last = 0, k = 0, shift = 0;
    unsigned i;

    memset(nb, 0, 86);
    for (i = 0; i < 256; ++i) {
        nb[86 + i] = sd[i] >> 2;
        nb[k] |= SWAP2(sd[i]) << shift;
        if (++k == 86) { k = 0; shift += 2; }
    }
    *p++ = 0xD5; *p++ = 0xAA; *p++ = 0xAD;
    for (i = 0; i < 342; ++i) {
        *p++ = gcr6[(nb[i] ^ last) & 0x3F];
        last = nb[i];
    }
    *p++ = gcr6[last];                     /* checksum */
    *p++ = 0xDE; *p++ = 0xAA; *p++ = 0xEB; *p = 0xFF;
}

static int decode(unsigned char *sd)
{
    unsigned char last = 0, v, a, k = 0, shift = 0;
    unsigned i;

    for (i = 0; i < 342; ++i) {
        if ((v = inv[NIBBUF[i]]) == 0xFF) return 1;
        nb[i] = last ^= v;
    }
    if (inv[NIBBUF[342]] != last) return 1;
    for (i = 0; i < 256; ++i) {
        a = nb[k] >> shift;
        sd[i] = (nb[86 + i] << 2) | SWAP2(a);
        if (++k == 86) { k = 0; shift += 2; }
    }
    return 0;
}

static void setblk(unsigned blk)
{
    rw_track = blk >> 4;
    rw_sector = blk & 15;
}

int blk_read(unsigned blk, unsigned char *buf)
{
    unsigned char tries;

    setblk(blk);
    for (tries = 0; tries < 8; ++tries)
        if (rw_rawread() == 0 && decode(buf) == 0)
            return 0;
    return 1;
}

int blk_write(unsigned blk, const unsigned char *buf)
{
    setblk(blk);
    encode(buf);
    return rw_rawwrite();
}

/*---------------------------------------------------------------- the file system */

struct entry {                             /* 24 bytes, little endian like fs.py */
    char     name[A2FS_NAMELEN];
    unsigned start, len, crc, spare;
};
static unsigned char dir[256];
#define MAGIC    ((char *)dir)             /* "A2FS" */
#define NEXTFREE (*(unsigned *)(dir + 4))
#define ENTRY(i) ((struct entry *)(dir + 16) + (i))
#define NENTRIES 10

void fs_start(void)
{
    unsigned i;

    memset(inv, 0xFF, sizeof inv);
    for (i = 0; i < 64; ++i) inv[gcr6[i]] = i;
    rw_init();
}

void fs_stop(void)
{
    rw_off();
}

int fs_mount(void)
{
    if (blk_read(A2FS_DIRBLK, dir)) return 1;
    return memcmp(MAGIC, "A2FS", 4) != 0;
}

static struct entry *find(const char *name)
{
    unsigned char i;

    for (i = 0; i < NENTRIES; ++i)
        if (ENTRY(i)->name[0] && strncmp(ENTRY(i)->name, name, A2FS_NAMELEN) == 0)
            return ENTRY(i);
    return 0;
}

unsigned crc16(const void *buf, unsigned len)
{
    const unsigned char *p = buf;
    unsigned crc = 0xFFFF;
    unsigned char x;

    while (len--) {                        /* a byte at a time, no table */
        x = (crc >> 8) ^ *p++;
        x ^= x >> 4;
        crc = (crc << 8) ^ ((unsigned)x << 12) ^ ((unsigned)x << 5) ^ x;
    }
    return crc;
}

int fs_read(const char *name, void *buf, unsigned max, unsigned *len, unsigned *crc)
{
    static unsigned char blk[256];
    struct entry *e = find(name);
    unsigned char *p = buf;
    unsigned b, left, n;

    if (!e || e->len > max) return 1;
    *len = e->len;
    *crc = e->crc;
    for (b = e->start, left = e->len; left; ++b, left -= n, p += n) {
        n = left < 256 ? left : 256;
        if (blk_read(b, blk)) return 1;
        memcpy(p, blk, n);
    }
    return 0;
}

int fs_write(const char *name, const void *buf, unsigned len)
{
    static unsigned char blk[256];
    struct entry *e = find(name);
    const unsigned char *p = buf;
    unsigned nblk = (len + 255) >> 8, b, left, n;
    unsigned char i;

    if (!e || nblk > ((e->len + 255) >> 8)) {     /* new, or no longer fits: allocate */
        if (!e)
            for (i = 0; i < NENTRIES && !e; ++i)
                if (!ENTRY(i)->name[0]) e = ENTRY(i);
        if (!e || NEXTFREE + nblk > 35 * 16) return 1;
        memset(e->name, 0, A2FS_NAMELEN);
        strncpy(e->name, name, A2FS_NAMELEN);
        e->start = NEXTFREE;
        NEXTFREE += nblk;
    }
    for (b = e->start, left = len; left; ++b, left -= n, p += n) {
        n = left < 256 ? left : 256;
        memset(blk, 0, sizeof blk);
        memcpy(blk, p, n);
        if (blk_write(b, blk)) return 1;
    }
    e->len = len;
    e->crc = crc16(buf, len);
    return blk_write(A2FS_DIRBLK, dir);
}
