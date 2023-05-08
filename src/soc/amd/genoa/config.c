/* SPDX-License-Identifier: GPL-2.0-only */

#include <device/device.h>
#include <amdblocks/chip.h>
#include <device/device.h>
#include "chip.h"

const struct soc_amd_common_config *soc_get_common_config(void)
{
	const struct device *cpu_cluster = dev_find_path(NULL, DEVICE_PATH_CPU_CLUSTER);
	assert(cpu_cluster);
	const struct soc_amd_genoa_config *cfg = cpu_cluster->chip_info;
	return &cfg->common_config;
}
