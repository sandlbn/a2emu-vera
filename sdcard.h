#ifndef SDCARD_H
#define SDCARD_H

#include <stdbool.h>
#include <stdint.h>

extern bool sdcard_attached;

bool sdcard_attach(const char *path);
void sdcard_detach(void);
void sdcard_select(bool select);
uint8_t sdcard_handle(uint8_t inbyte);

#endif
