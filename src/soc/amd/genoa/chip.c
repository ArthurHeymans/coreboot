/* SPDX-License-Identifier: GPL-2.0-only */

#include <device/device.h>
#include <soc/southbridge.h>
#include <soc/acpi.h>

static void soc_init(void *chip_info)
{
	// IDEA: Call opensil from here instead of using a hook?
	default_dev_ops_root.write_acpi_tables = hest_create;
}

static void soc_final(void *chip_info)
{

}

struct chip_operations soc_amd_genoa_ops = {
	CHIP_NAME("AMD Genoa SoC")
	.init = soc_init,
	.final = soc_final,
};
