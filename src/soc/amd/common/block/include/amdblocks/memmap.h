/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef AMD_BLOCK_MEMMAP_H
#define AMD_BLOCK_MEMMAP_H

#include <device/device.h>
#include <stdint.h>
#include <symbols.h>

DECLARE_REGION(early_reserved_dram)

/*
 * Called in romstage by non-CAR SoCs that don't use FSP. The PSP has already trained the
 * memory, so this only needs to set up CBMEM and do any SoC specific work that depends on
 * it, like parsing the APOB. The default initializes an empty CBMEM.
 */
void soc_romstage_init(void);

void memmap_stash_early_dram_usage(void);

/* report SoC memory map up to cbmem_top */
void read_lower_soc_memmap_resources(struct device *dev, unsigned long *idx);

void fsp_get_smm_region(uintptr_t *start, size_t *size);

#endif /* AMD_BLOCK_MEMMAP_H */
