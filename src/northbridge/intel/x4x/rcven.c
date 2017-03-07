/*
 * This file is part of the coreboot project.
 *
 * Copyright (C) 2015 Damien Zammit <damien@zamaudio.com>
 * Copyright (C) 2017 Arthur Heymans <arthur@aheymans.xyz>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <arch/io.h>
#include <console/console.h>
#include <delay.h>
#include "iomap.h"
#include "x4x.h"

#define MAX_COARSE 15
#define DQS_HIGH 1
#define DQS_LOW 0

typedef struct {
	u8 medium;
	u8 coarse;
	u8 pi;
	u8 tap;
} rec_timing_t;

static inline void barrier(void)
{
	asm volatile("mfence":::);
}

static u8 sampledqs(u16 sample_offset, u32 addr)
{
	volatile u32 strobe;

	MCHBAR8(0x5d8) = MCHBAR8(0x5d8) & ~0x2;
	udelay(2);
	MCHBAR8(0x5d8) = MCHBAR8(0x5d8) | 0x2;
	udelay(2);
	MCHBAR8(0x9d8) = MCHBAR8(0x9d8) & ~0x2;
	udelay(2);
	MCHBAR8(0x9d8) = MCHBAR8(0x9d8) | 0x2;
	udelay(2);
	barrier();
	strobe = read32((u32 *)addr);
	barrier();
	return (MCHBAR32(sample_offset) & 0x40) >> 6;
}

static void program_timing(rec_timing_t *timing, u8 channel, u8 lane)
{
	u32 reg32;
	u16 reg16;
	u8 reg8;

	printk(RAM_SPEW, "      Programming timings:"
		"Coarse: %d, Medium: %d, TAP: %d, PI: %d\n",
		timing->coarse, timing->medium, timing->tap, timing->pi);

	reg32 = MCHBAR32(0x400 * channel + 0x248);
	reg32 &= ~0xf0000;
	reg32 |= timing->coarse << 16;
	MCHBAR32(0x400 * channel + 0x248) = reg32;

	reg16 = MCHBAR16(0x400 * channel + 0x58c);
	reg16 &= ~(3 << (lane * 2));
	reg16 |= timing->medium << (lane * 2);
	MCHBAR16(0x400 * channel + 0x58c) = reg16;

	reg8 = MCHBAR8(0x400 * channel + 0x560 + lane * 4);
	reg8 &= ~0xf;
	reg8 |= timing->tap;
	MCHBAR8(0x400 * channel + 0x560 + lane * 4) = reg8;

	reg8 = MCHBAR8(0x400 * channel + 0x560 + lane * 4);
	reg8 &= ~0x70;
	reg8 |= timing->pi << 4;
	MCHBAR8(0x400 * channel + 0x560 + lane * 4) = reg8;
}

static void increase_medium(rec_timing_t *timing)
{
	if (timing->medium < 3) {
		timing->medium++;
	} else {
		timing->medium = 0;
		timing->coarse++;
	}
}

static void decrease_medium(rec_timing_t *timing)
{
	if (timing->medium == 0) {
		timing->medium = 3;
		timing->coarse--;
	} else {
		timing->medium--;
	}
}

static void increase_tap(rec_timing_t *timing)
{
	if (timing->tap == 15) {
		timing->tap = 0;
		increase_medium(timing);
	} else {
		timing->tap++;
	}
}

static void decrease_tap(rec_timing_t *timing)
{
	if (timing->tap == 0) {
		timing->tap = 15;
		decrease_medium(timing);
	} else {
		timing->tap--;
	}
}

static int decr_coarse_low(u32 sample_offset, u8 channel, u8 lane, u32 addr,
			rec_timing_t *timing)
{
	printk(BIOS_DEBUG, "  Decreasing coarse until high to low transition is found\n");
	while (sampledqs(sample_offset, addr) != DQS_LOW) {
		if (timing->coarse == 0) {
			printk(BIOS_DEBUG, "Couldn't find DQS-high 0 indicator, halt\n");
			return -1;
		}
		timing->coarse--;
		program_timing(timing, channel, lane);
	}
	printk(BIOS_DEBUG, "    DQS low at coarse=%d medium=%d\n",
		timing->coarse, timing->medium);
	return 0;
}

static int fine_search_dqs_high(u32 sample_offset, u8 channel, u8 lane, u32 addr,
				rec_timing_t *timing)
{
	printk(BIOS_DEBUG, "  Increasing TAP until low to high transition is found\n");
	while (sampledqs(sample_offset, addr) != DQS_HIGH) {
		increase_tap(timing);
		if (timing->coarse > MAX_COARSE) {
			printk(BIOS_DEBUG, "Could not find DQS-high on fine search.\n");
			return -1;
		}
		program_timing(timing, channel, lane);
	}
	return 0;
}

static int find_dqs_low(u32 sample_offset, u8 channel, u8 lane, u32 addr,
			rec_timing_t *timing)
{
	/* Look for DQS low, using quarter steps. */
	printk(BIOS_DEBUG, "  Increasing medium until DQS LOW is found\n");
	while (sampledqs(sample_offset, addr) != DQS_LOW) {
		if (timing->coarse > MAX_COARSE) {
			printk(BIOS_DEBUG, "Coarse > 15: DQS tuning failed, halt\n");
			return -1;
		}
		increase_medium(timing);
		program_timing(timing, channel, lane);
	}
	printk(BIOS_DEBUG, "    DQS low at coarse=%d medium=%d\n",
		timing->coarse, timing->medium);
	return 0;
}
static int find_dqs_high(u32 sample_offset, u8 channel, u8 lane, u32 addr,
				rec_timing_t *timing)
{
	/* Look for DQS high, using quarter steps. */
	printk(BIOS_DEBUG, "  Increasing medium until DQS HIGH is found\n");
	while (sampledqs(sample_offset, addr) != DQS_HIGH) {
		increase_medium(timing);
		program_timing(timing, channel, lane);
		if (timing->coarse > MAX_COARSE) {
			printk(BIOS_DEBUG, "Coarse > 16: DQS tuning failed, halt\n");
			return -1;
		}
	}
	printk(BIOS_DEBUG, "    DQS high at coarse=%d medium=%d\n",
		timing->coarse, timing->medium);
	return 0;
}

static int find_dqs_edge_lowhigh(u32 sample_offset, u8 channel, u8 lane, u32 addr,
				rec_timing_t *timing)
{
	/* Coarsely look for DQS high. */
	if (find_dqs_high(sample_offset, channel, lane, addr, timing))
		return -1;

	/* Go back and perform finer search. */
	decrease_medium(timing);
	program_timing(timing, channel, lane);
	if (fine_search_dqs_high(sample_offset, channel, lane, addr, timing) < 0)
		return -1;

	return 0;
}

static int find_preamble(u32 sample_offset, u8 channel, u8 lane, u32 addr,
			rec_timing_t *timing)
{
	/* Add a quarter step */
	increase_medium(timing);
	program_timing(timing, channel, lane);
	/* Verify we are at high */
	if (sampledqs(sample_offset, addr) != DQS_HIGH) {
		printk(BIOS_DEBUG, "Not at DQS high, doh\n");
		return -1;
	}

	/* Decrease coarse until LOW is found */
	if (decr_coarse_low(sample_offset, channel, lane, addr, timing))
		return -1;
	return 0;
}

static int calibrate_receive_enable(u8 channel, u8 lane,
				u32 addr, rec_timing_t *timing)
{
	u32 sample_offset = 0x400 * channel + 0x561 + (lane << 2);

	program_timing(timing, channel, lane);
	/* Set receive enable bit */
	MCHBAR16(0x400 * channel + 0x588) = (MCHBAR16(0x400 * channel + 0x588)
				& ~(3 << (lane * 2))) | (1 << (lane * 2));

	if (find_dqs_low(sample_offset, channel, lane, addr, timing))
		return -1;

	/* Advance beyond previous high to low transition. */
	if (timing->coarse == MAX_COARSE && timing->medium == 3)
 		/* A finer search could be implemented */
		printk(BIOS_DEBUG, "Cannot increase medium further");
	else
		increase_medium(timing);
	program_timing(timing, channel, lane);

	if (find_dqs_edge_lowhigh(sample_offset, channel, lane, addr, timing))
		return -1;

	/* Go back on fine search */
	decrease_tap(timing);
	timing->pi = 3;
	program_timing(timing, channel, lane);

	if (find_preamble(sample_offset, channel, lane, addr, timing))
		return -1;

	if (find_dqs_edge_lowhigh(sample_offset, channel, lane, addr, timing))
		return -1;
	decrease_tap(timing);
	timing->pi = 7;
	program_timing(timing, channel, lane);

	/* Unset receive enable bit */
	MCHBAR16(0x400 * channel + 0x588) = MCHBAR16(0x400 * channel + 0x588) &
		~(3 << (lane * 2));
	return 0;
}

void rcven(const struct sysinfo *s)
{
	int i;
	u8 channel, lane, reg8;
	u32 addr;
	rec_timing_t timing[8];
	u8 mincoarse;

	MCHBAR8(0x5d8) = MCHBAR8(0x5d8) & ~0xc;
	MCHBAR8(0x9d8) = MCHBAR8(0x9d8) & ~0xc;
	MCHBAR8(0x5dc) = MCHBAR8(0x5dc) & ~0x80;
	FOR_EACH_POPULATED_CHANNEL(s->dimms, channel) {
		addr = (channel << 29);
		mincoarse = 0xff;
		for (i = 0; i < RANKS_PER_CHANNEL &&
			     !RANK_IS_POPULATED(s->dimms, channel, i); i++)
			addr += 128 * MiB;
		for (lane = 0; lane < 8; lane++) {
			printk(BIOS_DEBUG, "Channel %d, Lane %d addr=0x%08x\n",
				channel, lane, addr);
			timing[lane].coarse = (s->selected_timings.CAS - 1);
			switch (lane) {
			default:
			case 0:
			case 1:
				timing[lane].medium = 0;
				break;
			case 2:
			case 3:
				timing[lane].medium = 1;
				break;
			case 4:
			case 5:
				timing[lane].medium = 2;
				break;
			case 6:
			case 7:
				timing[lane].medium = 3;
				break;
			}
			timing[lane].tap = 0;
			timing[lane].pi = 0;

			if (calibrate_receive_enable(channel, lane, addr,
							&timing[lane]))
				die("Receive enable calibration failed\n");
			if (mincoarse > timing[lane].coarse)
				mincoarse = timing[lane].coarse;
		}
		printk(BIOS_DEBUG, "Found min coarse value = %d\n", mincoarse);
		printk(BIOS_DEBUG, "Receive enable, final timings:\n");
		/* Normalise coarse */
		for (lane = 0; lane < 8; lane++) {
			if (timing[lane].coarse == 0)
				reg8 = 0;
			else
				reg8 = timing[lane].coarse - mincoarse;
			printk(BIOS_DEBUG, "ch %d lane %d: coarse offset: %d;"
				"medium: %d; tap: %d\n",
				channel, lane, reg8, timing[lane].medium,
				timing[lane].tap);
			MCHBAR16(0x400 * channel + 0x5fa) &=
				~(3 << (lane * 2)) | (reg8 << (lane * 2));
		}
		/* simply use timing[0] to program mincoarse */
		timing[0].coarse = mincoarse;
		program_timing(&timing[0], channel, 0);
	}
}
