/* SPDX-License-Identifier: GPL-2.0-only */

#include <acpi/acpigen_pci.h>
#include <amdblocks/acpi.h>
#include <amdblocks/data_fabric.h>
#include <amdblocks/root_complex.h>
#include <console/console.h>
#include <device/device.h>
#include <device/pci.h>
#include <soc/iomap.h>
#include <soc/native.h>
#include <soc/pci_devs.h>
#include <soc/southbridge.h>
#include "chip.h"

static void soc_init(void *chip_info)
{
	default_dev_ops_root.write_acpi_tables = soc_acpi_write_tables;

	sp3_native_silicon_init();

	data_fabric_print_mmio_conf();

	fch_init(chip_info);
}

static void soc_final(void *chip_info)
{
	fch_final(chip_info);
}

struct chip_operations soc_amd_sp3_ops = {
	.name = "AMD SP3 SOC",
	.init = soc_init,
	.final = soc_final
};
