// SD card in SPI mode, backed by a raw image file.
// From the Commander X16 emulator's sdcard.c
// Copyright (c) 2019 Michael Steil, (c) 2020 Frank van den Hoef, 2-clause BSD
#define _FILE_OFFSET_BITS 64

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "sdcard.h"

#ifdef _MSC_VER
#define fseeko _fseeki64
#define ftello _ftelli64
#endif

enum {
	CMD0   = 0,         // GO_IDLE_STATE
	CMD8   = 8,         // SEND_IF_COND
	CMD9   = 9,         // SEND_CSD
	CMD12  = 12,        // STOP_TRANSMISSION
	CMD13  = 13,        // SEND_STATUS
	CMD16  = 16,        // SET_BLOCKLEN
	CMD17  = 17,        // READ_SINGLE_BLOCK
	CMD18  = 18,        // READ_MULTIPLE_BLOCK
	CMD24  = 24,        // WRITE_BLOCK
	CMD55  = 55,        // APP_CMD
	CMD58  = 58,        // READ_OCR
	ACMD41 = 0x80 | 41, // SD_SEND_OP_COND
};

static FILE *card;
static int64_t card_size;
bool sdcard_attached;

static uint8_t rxbuf[3 + 512];
static int rxbuf_idx;
static uint32_t lba;
static uint8_t last_cmd;
static bool is_acmd;
static bool is_idle = true;
static bool is_initialized;
static bool multiblock;
static bool selected;

static const uint8_t *response;
static int response_length;
static int response_counter;

bool sdcard_attach(const char *path)
{
	sdcard_detach();
	card = fopen(path, "r+b");
	if (!card) {
		fprintf(stderr, "sdcard: can't open %s\n", path);
		return false;
	}
	fseeko(card, 0, SEEK_END);
	card_size = ftello(card);
	sdcard_attached = true;
	is_initialized = false;
	return true;
}

void sdcard_detach(void)
{
	if (card) fclose(card);
	card = NULL;
	sdcard_attached = false;
}

void sdcard_select(bool select)
{
	selected = select;
	rxbuf_idx = 0;
}

static void set_response(const uint8_t *r, int len)
{
	response = r;
	response_length = len;
}

static void set_response_csd(void)
{
	static uint8_t rr[] = {
		0xff, 0xff,       // dummy
		0x00,             // R1
		0xff,             // dummy
		0xfe,             // start block
		0x40, 0x0e, 0x00, 0x32, 0x5b, 0x59, 0x00,
		0x00, 0x00, 0x00, // C_SIZE
		0x7f, 0x80, 0x0a, 0x40, 0x00, 0x01
	};
	uint64_t c_size = (card_size >> 19) - 1;  // CSD v2: (C_SIZE + 1) * 512K
	rr[12] = (c_size >> 16) & 0x3f;
	rr[13] = (c_size >> 8) & 0xff;
	rr[14] = c_size & 0xff;
	set_response(rr, sizeof rr);
}

static void set_response_r1(void)
{
	static uint8_t r1;
	r1 = is_idle;
	set_response(&r1, 1);
}

static void set_response_r2(void)
{
	static const uint8_t ok[] = {0x00, 0x00}, idle[] = {0x1f, 0xff};
	set_response(is_initialized ? ok : idle, 2);
}

// data token + 512 bytes + CRC, or a one-byte error token
static int load_block(uint8_t *dest)
{
	if ((int64_t)lba * 512 >= card_size) {
		dest[0] = 0x08;  // out of range
		return 1;
	}
	dest[0] = 0xfe;
	fseeko(card, (int64_t)lba * 512, SEEK_SET);
	if (fread(&dest[1], 1, 512, card) != 512)
		fprintf(stderr, "sdcard: short read at LBA %u\n", lba);
	return 1 + 512 + 2;
}

static uint32_t cmd_arg(void)
{
	return (uint32_t)rxbuf[1] << 24 | rxbuf[2] << 16 | rxbuf[3] << 8 | rxbuf[4];
}

static void command(void)
{
	static const uint8_t r3[] = {0xc0, 0xff, 0x80, 0x00};
	static const uint8_t r7[] = {0x01, 0x00, 0x00, 0x01, 0xaa};
	static const uint8_t bad_lba[] = {0x00, 0x08};
	static uint8_t block[1 + 1 + 512 + 2];

	rxbuf[0] &= 0x3f;
	if (is_acmd) {
		rxbuf[0] |= 0x80;
		is_acmd = false;
	}
	last_cmd = rxbuf[0];

	switch (rxbuf[0]) {
	case CMD0:
		is_idle = true;
		set_response_r1();
		break;
	case CMD8:
		set_response(r7, sizeof r7);
		break;
	case CMD9:
		set_response_csd();
		break;
	case ACMD41:
		is_idle = false;
		is_initialized = true;
		set_response_r1();
		break;
	case CMD12:
		multiblock = false;
		set_response_r1();
		break;
	case CMD13:
		set_response_r2();
		break;
	case CMD18:
		multiblock = true;
		/* fall through */
	case CMD17:
		lba = cmd_arg();
		block[0] = 0;  // R1
		set_response(block, 1 + load_block(&block[1]));
		if (response_length == 2) multiblock = false;
		break;
	case CMD24:
		lba = cmd_arg();
		if ((int64_t)lba * 512 >= card_size)
			set_response(bad_lba, sizeof bad_lba);
		else
			set_response_r1();
		break;
	case CMD55:
		is_acmd = true;
		set_response_r1();
		break;
	case CMD58:
		set_response(r3, sizeof r3);
		break;
	default:  // CMD16 and anything else: just R1
		set_response_r1();
		break;
	}
	response_counter = 0;
}

static void next_response_byte(uint8_t *out)
{
	static uint8_t next[1 + 512 + 2];

	*out = response[response_counter++];
	if (response_counter < response_length) return;
	if (multiblock) {
		lba++;
		response_length = load_block(next);
		if (response_length == 1) multiblock = false;
		response = next;
		response_counter = 0;
	} else
		response = NULL;
}

uint8_t sdcard_handle(uint8_t inbyte)
{
	uint8_t out = 0xff;

	if (!selected || !card) return 0xff;

	if (rxbuf_idx == 0 && inbyte == 0xff) {
		if (response) next_response_byte(&out);
		return out;
	}

	rxbuf[rxbuf_idx++] = inbyte;
	if ((rxbuf[0] & 0xc0) == 0x40 && rxbuf_idx == 6) {
		rxbuf_idx = 0;
		command();
	} else if (rxbuf_idx == 515) {  // start token + 512 bytes + CRC
		rxbuf_idx = 0;
		if (last_cmd == CMD24 && rxbuf[0] == 0xfe && (int64_t)lba * 512 < card_size) {
			fseeko(card, (int64_t)lba * 512, SEEK_SET);
			if (fwrite(rxbuf + 1, 1, 512, card) != 512)
				fprintf(stderr, "sdcard: short write at LBA %u\n", lba);
			fflush(card);
		}
	}
	return out;
}
