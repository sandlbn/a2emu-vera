/* a2fs.h -- a minimal file system on a Disk II floppy (see fs.py for the layout) */
#ifndef A2FS_H
#define A2FS_H

/* blocks are 256-byte physical sectors: block = track * 16 + sector */
#define A2FS_DIRBLK  272     /* track 17, sector 0 */
#define A2FS_NAMELEN 16

int blk_read(unsigned blk, unsigned char *buf);           /* 0 = ok */
int blk_write(unsigned blk, const unsigned char *buf);    /* 0 = ok */

void     fs_start(void);     /* motor on, recalibrate */
void     fs_stop(void);      /* motor off */
int      fs_mount(void);     /* 0 = ok */
/* read a whole file (at most max bytes): 0 = ok; *len and *crc are the directory's */
int      fs_read(const char *name, void *buf, unsigned max, unsigned *len, unsigned *crc);
/* create or replace a file, storing its CRC in the directory: 0 = ok */
int      fs_write(const char *name, const void *buf, unsigned len);
unsigned crc16(const void *buf, unsigned len);            /* CRC-16/CCITT-FALSE */

#endif
