/* fstest.c -- A2FS read/write test: read DATA4K and DATA8K and check their CRC
 * against the directory, then write OUT3K, read it back and compare. */
#include <conio.h>
#include <string.h>
#include "a2fs.h"

static unsigned char buf4[4096], buf8[8192], out[3072], back[3072];
static unsigned char failed;

static void result(unsigned char ok)
{
    cputs(ok ? "OK\r\n" : "ERROR\r\n");
    if (!ok) failed = 1;
}

static void check(const char *name, unsigned char *buf, unsigned size)
{
    unsigned len, crc, got;

    cprintf("%-7s", name);
    if (fs_read(name, buf, size, &len, &crc)) {
        cputs("read ");
        result(0);
        return;
    }
    got = crc16(buf, len);
    cprintf("%5u bytes crc %04X/%04X ", len, got, crc);
    result(len == size && got == crc);
}

int main(void)
{
    unsigned i, x = 0xACE1, len, crc;

    clrscr();
    cputs("A2FS read/write test\r\n\r\n");
    fs_start();
    cputs("mount  ");
    if (fs_mount()) {
        result(0);
        goto done;
    }
    result(1);

    check("DATA4K", buf4, sizeof buf4);
    check("DATA8K", buf8, sizeof buf8);

    for (i = 0; i < sizeof out; ++i) {      /* 16-bit Galois LFSR */
        x = x & 1 ? (x >> 1) ^ 0xB400 : x >> 1;
        out[i] = x;
    }
    cprintf("write  OUT3K %u bytes crc %04X ", sizeof out, crc16(out, sizeof out));
    result(fs_write("OUT3K", out, sizeof out) == 0);

    memset(back, 0, sizeof back);
    cputs("reread ");
    if (fs_read("OUT3K", back, sizeof back, &len, &crc)) {
        result(0);
    } else {
        unsigned got = crc16(back, len);
        cprintf("OUT3K %u bytes crc %04X ", len, got);
        result(len == sizeof out && got == crc && memcmp(out, back, sizeof out) == 0);
    }

done:
    fs_stop();
    cputs(failed ? "\r\nTEST FAILED\r\n" : "\r\nTEST PASSED\r\n");
    for (;;)
        ;
}
