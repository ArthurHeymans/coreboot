/* SPDX-License-Identifier: GPL-2.0-only */

#include <bootstate.h>
#include <stdint.h>
#include <stdbool.h>
#include <SilCommon.h>
#include <Sil-api.h> // needed above ApobCmn.h
#include <ApobCmn.h>
#include <device/device.h>
#include <xPRF-api.h>

#include "opensil.h"

static const char *hole_info_type(MEMORY_HOLE_TYPES type)
{
	const struct hole_type {
		MEMORY_HOLE_TYPES type;
		const char *string;
	} types[] = {
		{UMA, "UMA"},
		{MMIO, "MMIO"},
		{PrivilegedDRAM, "PrivilegedDRAM"},
		{Reserved1TbRemap, "Reserved1TbRemap"},
		{ReservedSLink, "ReservedSLink"},
		{ReservedSLinkAlignment, "ReservedSLinkAlignment"},
		{ReservedDrtm, "ReservedDrtm"},
		{ReservedCvip, "ReservedCvip"},
		{ReservedSmuFeatures, "ReservedSmuFeatures"},
		{ReservedFwtpm, "ReservedFwtpm"},
		{ReservedMpioC20, "ReservedMpioC20"},
		{ReservedNbif, "ReservedNbif"},
		{ReservedCxl, "ReservedCxl"},
		{ReservedCxlAlignment, "ReservedCxlAlignment"},
		{ReservedCpuTmr, "ReservedCpuTmr"},
		{ReservedRasEinj, "ReservedRasEinj"},
		{MaxMemoryHoleTypes, "MaxMemoryHoleTypes"},
	};

	int i;
	for (i = 0; i < ARRAY_SIZE(types); i++)
		if (type == types[i].type)
			break;
	if (i == ARRAY_SIZE(types))
		return "Unknown type";
	return types[i].string;
}

static void print_memory_holes(void *unused)
{
	uint64_t top_of_mem;
	uint32_t n_holes;
	MEMORY_HOLE_DESCRIPTOR *hole_info;
        SIL_STATUS status = xPrfGetSystemMemoryMap(&n_holes, &top_of_mem, (void **)&hole_info);
	SIL_STATUS_report("xPrfGetSystemMemoryMap", status);
	if (status != SilPass)
		return;

	printk(BIOS_DEBUG, "The following holes are reported in APOB\n");
	for (int hole = 0; hole < n_holes; hole++) {
		printk(BIOS_DEBUG, "  Base: 0x%016llx, Size: 0x%016llx, Type: %02d:%s\n",
		       hole_info[hole].Base, hole_info[hole].Size, hole_info[hole].Type,
		       hole_info_type(hole_info[hole].Type));
	}
}

BOOT_STATE_INIT_ENTRY(BS_DEV_RESOURCES, BS_ON_ENTRY, print_memory_holes, NULL);
