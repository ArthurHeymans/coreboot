/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/iomap.h>
#include <amdblocks/memmap.h>
#include <amdblocks/root_complex.h>
#include <device/device.h>
#include <device/resource.h>
#include <soc/native.h>

void read_soc_memmap_resources(struct device *dev, unsigned long *idx)
{
	/* Low RAM, early firmware workspace, CBMEM and the relocated TSEG. */
	read_lower_soc_memmap_resources(dev, idx);

	mmio_range(dev, (*idx)++, IOMMU_RESERVED_MMIO_BASE, IOMMU_RESERVED_MMIO_SIZE);

	/* BIOS entry 0x61 places APOB separately, below the early workspace. */
	reserved_ram_range(dev, (*idx)++, CONFIG_PSP_APOB_DRAM_ADDRESS,
			   CONFIG_PSP_APOB_DRAM_SIZE);

	sp3_native_add_memmap(dev, idx);
}
