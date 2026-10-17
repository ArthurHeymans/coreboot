/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __ZEPPELIN_CHIP_H__
#define __ZEPPELIN_CHIP_H__

#include <amdblocks/chip.h>
#include <types.h>

struct soc_amd_zeppelin_config {
	struct soc_amd_common_config common_config;
};

typedef struct soc_amd_zeppelin_config soc_amd_config_t;

#endif /* __ZEPPELIN_CHIP_H__ */
