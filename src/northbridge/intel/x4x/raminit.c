/*
 * This file is part of the coreboot project.
 *
 * Copyright (C) 2015 Damien Zammit <damien@zamaudio.com>
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
#include <cbmem.h>
#include <console/console.h>
#include <cpu/x86/cache.h>
#include <cpu/x86/mtrr.h>
#include <delay.h>
#include <halt.h>
#include <lib.h>
#include "iomap.h"
#include <southbridge/intel/i82801gx/i82801gx.h> /* smbus_read_byte */
#include "x4x.h"
#include <pc80/mc146818rtc.h>
#include <spd.h>
#include <string.h>
#include <device/dram/ddr2.h>

static inline int spd_read_byte(unsigned int device, unsigned int address)
{
	return smbus_read_byte(device, address);
}

struct abs_timings {
	u32 min_tclk;
	u32 min_tRAS;
	u32 min_tRP;
	u32 min_tRCD;
	u32 min_tWR;
	u32 min_tRFC;
	u32 min_tWTR;
	u32 min_tRRD;
	u32 min_tRTP;
	u32 tCLK_cas[TOTAL_DIMMS][8];
	u32 cas_supported;
};

static void select_cas_dramfreq_ddr2(struct sysinfo *s,
				struct abs_timings *saved_timings)
{
	u8 selected_cas;
	u8 cas_mask = SPD_CAS_LATENCY_DDR2_5 | SPD_CAS_LATENCY_DDR2_6;
	u32 common_min_tclk[8];
	u32 common_tCLK;

	cas_mask &= saved_timings->cas_supported;
	get_common_min_tclk(cas_mask, TOTAL_DIMMS, common_min_tclk,
			saved_timings->tCLK_cas);
	/* On this northbridge the highest DDR2 frequency is 800MHz */
	common_tCLK = get_common_freq_cas(cas_mask, common_min_tclk,
					&selected_cas, TCK_400MHZ);

	if (common_tCLK == 0)
		die("Could not find common memory frequency and CAS\n");

	s->selected_timings.CAS = selected_cas;
	s->selected_timings.tclk = common_tCLK;

	switch (s->selected_timings.tclk) {
	case TCK_200MHZ:
	case TCK_266MHZ:
		/* FIXME: this works on vendor BIOS */
		die("Selected dran frequency not supported\n");
	case TCK_333MHZ:
		s->selected_timings.mem_clk = MEM_CLOCK_667MHz;
		break;
	case TCK_400MHZ:
		s->selected_timings.mem_clk = MEM_CLOCK_800MHz;
		break;
	}

	switch (s->selected_timings.tclk) {
	case TCK_200MHZ:
		s->selected_timings.mem_clk = MEM_CLOCK_400MHz; break;
	case TCK_266MHZ:
		s->selected_timings.mem_clk = MEM_CLOCK_533MHz; break;
	case TCK_333MHZ:
		s->selected_timings.mem_clk = MEM_CLOCK_667MHz; break;
	case TCK_400MHZ:
		s->selected_timings.mem_clk = MEM_CLOCK_800MHz; break;
	}
}

static void mchinfo_ddr2(struct sysinfo *s)
{
	const u32 eax = cpuid_ext(0x04, 0).eax;
	printk(BIOS_WARNING, "%d CPU cores\n", ((eax >> 26) & 0x3f) + 1);

	u32 capid = pci_read_config16(PCI_DEV(0, 0, 0), 0xe8);
	if (!(capid & (1<<(79-64))))
		printk(BIOS_WARNING, "iTPM enabled\n");

	capid = pci_read_config32(PCI_DEV(0, 0, 0), 0xe4);
	if (!(capid & (1<<(57-32))))
		printk(BIOS_WARNING, "ME enabled\n");

	if (!(capid & (1<<(56-32))))
		printk(BIOS_WARNING, "AMT enabled\n");

	s->max_ddr2_mhz = 800; // All chipsets in x4x support up to 800MHz DDR2
	printk(BIOS_WARNING, "Capable of DDR2 of %d MHz or lower\n", s->max_ddr2_mhz);

	if (!(capid & (1<<(48-32))))
		printk(BIOS_WARNING, "VT-d enabled\n");
}

static int ddr2_save_dimminfo(u8 dimm_idx, u8 *raw_spd,
		struct abs_timings *saved_timings, struct sysinfo *s)
{
	struct dimm_attr_st decoded_dimm;
	int i;

	if (spd_decode_ddr2(&decoded_dimm, raw_spd) != SPD_STATUS_OK)
		return 1;

	if (IS_ENABLED(CONFIG_DEBUG_RAM_SETUP))
		dram_print_spd_ddr2(&decoded_dimm);

	if (!(decoded_dimm.width & (0x8 | 0x10))) {
		printk(BIOS_ERR, "DIMM%d Unsupported width: x%d. Disabling dimm\n",
			dimm_idx, s->dimms[dimm_idx].width);
		return 1;
	}
	s->dimms[dimm_idx].width = (decoded_dimm.width >> 3) - 1;
	/*
	 * This boils down to:
	 * "Except for the x16 configuration, all DDR2 devices have a
	 * 1KB page size. For the x16 configuration, the page size is 2KB
	 * for all densities except the 256Mb device, which has a 1KB page size."
	 * Micron, 'TN-47-16 Designing for High-Density DDR2 Memory'
	 */
	 s->dimms[dimm_idx].page_size = s->dimms[dimm_idx].width *
		 (1 << decoded_dimm.col_bits);

	switch (decoded_dimm.banks) {
	case 4:
		s->dimms[dimm_idx].n_banks = 0;
		break;
	case 8:
		s->dimms[dimm_idx].n_banks = 1;
		break;
	default:
		printk(BIOS_ERR, "DIMM%d Unsupported #banks: x%d. Disabling dimm\n",
			 dimm_idx, decoded_dimm.banks);
		return 1;
	}

	s->dimms[dimm_idx].ranks = decoded_dimm.ranks;
	s->dimms[dimm_idx].rows = decoded_dimm.row_bits;
	s->dimms[dimm_idx].cols = decoded_dimm.col_bits;

	saved_timings->cas_supported &= decoded_dimm.cas_supported;

	saved_timings->min_tRAS =
		MAX(saved_timings->min_tRAS, decoded_dimm.tRAS);
	saved_timings->min_tRP =
		MAX(saved_timings->min_tRP, decoded_dimm.tRP);
	saved_timings->min_tRCD =
		MAX(saved_timings->min_tRCD, decoded_dimm.tRCD);
	saved_timings->min_tWR =
		MAX(saved_timings->min_tWR, decoded_dimm.tWR);
	saved_timings->min_tRFC =
		MAX(saved_timings->min_tRFC, decoded_dimm.tRFC);
	saved_timings->min_tWTR =
		MAX(saved_timings->min_tWTR, decoded_dimm.tWTR);
	saved_timings->min_tRRD =
		MAX(saved_timings->min_tRRD, decoded_dimm.tRRD);
	saved_timings->min_tRTP =
		MAX(saved_timings->min_tRTP, decoded_dimm.tRTP);
	for (i = 0; i < 8; i++) {
		saved_timings->tCLK_cas[dimm_idx][i] =
			decoded_dimm.cycle_time[i];
	}
	return 0;
}

static void select_discrete_timings(struct sysinfo *s,
				struct abs_timings *timings)
{
	s->selected_timings.tRAS = DIV_ROUND_UP(timings->min_tRAS,
						s->selected_timings.tclk);
	s->selected_timings.tRP = DIV_ROUND_UP(timings->min_tRP,
						s->selected_timings.tclk);
	s->selected_timings.tRCD = DIV_ROUND_UP(timings->min_tRCD,
						s->selected_timings.tclk);
	s->selected_timings.tWR = DIV_ROUND_UP(timings->min_tWR,
						s->selected_timings.tclk);
	s->selected_timings.tRFC = DIV_ROUND_UP(timings->min_tRFC,
						s->selected_timings.tclk);
	s->selected_timings.tWTR = DIV_ROUND_UP(timings->min_tWTR,
						s->selected_timings.tclk);
	s->selected_timings.tRRD = DIV_ROUND_UP(timings->min_tRRD,
						s->selected_timings.tclk);
	s->selected_timings.tRTP = DIV_ROUND_UP(timings->min_tRTP,
						s->selected_timings.tclk);
}
static void print_selected_timings(struct sysinfo *s)
{
	printk(BIOS_DEBUG, "Selected timings:\n");
	printk(BIOS_DEBUG, "\tFSB:  %dMHz\n", fsb2mhz(s->selected_timings.fsb_clk));
	printk(BIOS_DEBUG, "\tDDR:  %dMHz\n", ddr2mhz(s->selected_timings.mem_clk));

	printk(BIOS_DEBUG, "\tCAS:  %d\n", s->selected_timings.CAS);
	printk(BIOS_DEBUG, "\ttRAS: %d\n", s->selected_timings.tRAS);
	printk(BIOS_DEBUG, "\ttRP:  %d\n", s->selected_timings.tRP);
	printk(BIOS_DEBUG, "\ttRCD: %d\n", s->selected_timings.tRCD);
	printk(BIOS_DEBUG, "\ttWR:  %d\n", s->selected_timings.tWR);
	printk(BIOS_DEBUG, "\ttRFC: %d\n", s->selected_timings.tRFC);
	printk(BIOS_DEBUG, "\ttWTR: %d\n", s->selected_timings.tWTR);
	printk(BIOS_DEBUG, "\ttRRD: %d\n", s->selected_timings.tRRD);
	printk(BIOS_DEBUG, "\ttRTP: %d\n", s->selected_timings.tRTP);
}

static void find_fsb_speed(struct sysinfo *s)
{
	switch (MCHBAR32(0xc00) & 0x7) {
	case 0x0:
		s->max_fsb = FSB_CLOCK_1066MHz;
		break;
	case 0x2:
		s->max_fsb = FSB_CLOCK_800MHz;
		break;
	case 0x4:
		s->max_fsb = FSB_CLOCK_1333MHz;
		break;
	default:
		s->max_fsb = FSB_CLOCK_800MHz;
		printk(BIOS_WARNING, "Can't detect FSB, setting 800MHz\n");
		break;
	}
	s->selected_timings.fsb_clk = s->max_fsb;
}

static void decode_spd_select_timings(struct sysinfo *s)
{
	unsigned int device;
	u8 dram_type_mask = (1 << DDR2) | (1 << DDR3);
	u8 dimm_mask = 0;
	u8 raw_spd[128];
	int i;
	struct abs_timings saved_timings = { };
	saved_timings.cas_supported = (u32)-1;

	FOR_EACH_DIMM(i) {
		s->dimms[i].card_type = RAW_CARD_POPULATED;
		device = s->spd_map[i];
		if (!device) {
			s->dimms[i].card_type = RAW_CARD_UNPOPULATED;
			continue;
		}
		switch (spd_read_byte(s->spd_map[i], SPD_MEMORY_TYPE)) {
		case DDR2SPD:
			dram_type_mask &= 1 << DDR2;
			s->spd_type = DDR2;
			break;
		case DDR3SPD:
			dram_type_mask &= 1 << DDR3;
			s->spd_type = DDR3;
			break;
		default:
			s->dimms[i].card_type = RAW_CARD_UNPOPULATED;
			continue;
		}
		if (!dram_type_mask)
			die("Mixing up dimm types is not supported\n");
		if (s->spd_type == DDR2){
			i2c_block_read(device, 0, 64, raw_spd);
			if (ddr2_save_dimminfo(i, raw_spd, &saved_timings, s)) {
				/* something in decoded SPD was unsupported */
				s->dimms[i].card_type = RAW_CARD_UNPOPULATED;
				continue;
			}
		} else { /* DDR3: not implemented so don't decode */
			die("DDR3 support is not implemented\n");
		}
		dimm_mask = (1 << i);
	}
	if (!dimm_mask)
		die("No memory installed.\n");
	if (s->spd_type == DDR2)
		select_cas_dramfreq_ddr2(s, &saved_timings);
	select_discrete_timings(s, &saved_timings);
}

static void find_dimm_config(struct sysinfo *s)
{
	int chan, i;

	FOR_EACH_POPULATED_CHANNEL(s->dimms, chan) {
		FOR_EACH_POPULATED_DIMM_IN_CHANNEL(s->dimms, chan, i) {
			int dimm_config;
			if (s->dimms[i].ranks == 1) {
				if (s->dimms[i].width == 0)	/* x8 */
					dimm_config = 1;
				else				/* x16 */
					dimm_config = 3;
			} else {
				if (s->dimms[i].width == 0)	/* x8 */
					dimm_config = 2;
				else
					die("Dual-rank x16 not supported\n");
			}
			s->dimm_config[chan] |=
				dimm_config << (i % DIMMS_PER_CHANNEL) * 2;
		}
		printk(BIOS_DEBUG, "  Config[CH%d] : %d\n", chan, s->dimm_config[chan]);
	}

}

static void checkreset_ddr2(int boot_path)
{
	u8 pmcon2;
	u32 pmsts;

	if (boot_path >= 1) {
		pmsts = MCHBAR32(PMSTS_MCHBAR);
		if (!(pmsts & 1))
			printk(BIOS_DEBUG,
				"Channel 0 possibly not in self refresh\n");
		if (!(pmsts & 2))
			printk(BIOS_DEBUG,
				"Channel 1 possibly not in self refresh\n");
	}

	pmcon2 = pci_read_config8(PCI_DEV(0, 0x1f, 0), 0xa2);

	if (pmcon2 & 0x80) {
		pmcon2 &= ~0x80;
		pci_write_config8(PCI_DEV(0, 0x1f, 0), 0xa2, pmcon2);

		/* do magic 0xf0 thing. */
		u8 reg8 = pci_read_config8(PCI_DEV(0, 0, 0), 0xf0);
		pci_write_config8(PCI_DEV(0, 0, 0), 0xf0, reg8 & ~(1 << 2));
		reg8 = pci_read_config8(PCI_DEV(0, 0, 0), 0xf0);
		pci_write_config8(PCI_DEV(0, 0, 0), 0xf0, reg8 |  (1 << 2));

		printk(BIOS_DEBUG, "Reset...\n");
		outb(0x6, 0xcf9);
		asm ("hlt");
	}
	pmcon2 |= 0x80;
	pci_write_config8(PCI_DEV(0, 0x1f, 0), 0xa2, pmcon2);
}

/**
 * @param boot_path: 0 = normal, 1 = reset, 2 = resume from s3
 */
void sdram_initialize(int boot_path, const u8 *spd_map)
{
	struct sysinfo s;
	u8 reg8;

	printk(BIOS_DEBUG, "Setting up RAM controller.\n");

	pci_write_config8(PCI_DEV(0, 0, 0), 0xdf, 0xff);

	memset(&s, 0, sizeof(struct sysinfo));

	s.boot_path = boot_path;
	s.spd_map[0] = spd_map[0];
	s.spd_map[1] = spd_map[1];
	s.spd_map[2] = spd_map[2];
	s.spd_map[3] = spd_map[3];

	checkreset_ddr2(s.boot_path);

	/* Detect dimms per channel */
	reg8 = pci_read_config8(PCI_DEV(0, 0, 0), 0xe9);
	printk(BIOS_DEBUG, "Dimms per channel: %d\n", (reg8 & 0x10) ? 1 : 2);

	mchinfo_ddr2(&s);

	find_fsb_speed(&s);
	decode_spd_select_timings(&s);
	print_selected_timings(&s);
	find_dimm_config(&s);

	switch (s.spd_type) {
	case DDR2:
		raminit_ddr2(&s);
		break;
	case DDR3:
		// FIXME Add: raminit_ddr3(&s);
		break;
	default:
		die("Unknown DDR type\n");
		break;
	}

	reg8 = pci_read_config8(PCI_DEV(0, 0x1f, 0), 0xa2);
	pci_write_config8(PCI_DEV(0, 0x1f, 0), 0xa2, reg8 & ~0x80);

	reg8 = pci_read_config8(PCI_DEV(0, 0, 0), 0xf4);
	pci_write_config8(PCI_DEV(0, 0, 0), 0xf4, reg8 | 1);
	printk(BIOS_DEBUG, "RAM initialization finished.\n");
}
