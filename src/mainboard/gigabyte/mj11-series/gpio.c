/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/gpio.h>
#include <soc/gpio.h>
#include <types.h>

#include "mainboard.h"

/*
 * The vendor CPM GPIO table ($A10). The values are IOMUX functions, not
 * named functions: the Zeppelin IOMUX assignments are not known. Rows that
 * only select a function leave the pad configuration at its default.
 */
static const struct soc_amd_gpio gpio_table[] = {
	PAD_CFG_STRUCT(5, 0, GPIO_OUTPUT_OUT_HIGH),
	PAD_CFG_STRUCT(6, 0, GPIO_OUTPUT_OUT_HIGH),
	PAD_CFG_STRUCT(9, 1, 0),
	PAD_CFG_STRUCT(10, 2, 0),
	PAD_CFG_STRUCT(17, 1, 0),
	PAD_CFG_STRUCT(23, 1, GPIO_OUTPUT_OUT_HIGH),
	PAD_CFG_STRUCT(28, 0, 0),
	PAD_CFG_STRUCT(40, 1, 0),
	PAD_CFG_STRUCT(42, 1, GPIO_OUTPUT_OUT_HIGH),
	PAD_CFG_STRUCT(70, 0, 0),
	PAD_CFG_STRUCT(76, 1, 0),
	PAD_CFG_STRUCT(88, 2, GPIO_OUTPUT_OUT_HIGH | GPIO_PULLUP_ENABLE),
	PAD_CFG_STRUCT(117, 0, 0),
	PAD_CFG_STRUCT(119, 2, 0),
	PAD_CFG_STRUCT(120, 0, 0),
	PAD_CFG_STRUCT(121, 0, 0),
	PAD_CFG_STRUCT(122, 0, 0),
	PAD_CFG_STRUCT(133, 0, 0),
	PAD_CFG_STRUCT(135, 2, 0),
	PAD_CFG_STRUCT(138, 0, 0),
	PAD_CFG_STRUCT(140, 2, 0),
	PAD_CFG_STRUCT(141, 1, 0),
	PAD_CFG_STRUCT(142, 2, 0),
	PAD_CFG_STRUCT(143, 1, 0),
	PAD_CFG_STRUCT(144, 1, 0),
	PAD_CFG_STRUCT(145, 2, 0),
	PAD_CFG_STRUCT(146, 2, 0),
	PAD_CFG_STRUCT(151, 1, 0),
	PAD_CFG_STRUCT(152, 1, 0),
};

void mainboard_program_gpios(void)
{
	gpio_configure_pads(gpio_table, ARRAY_SIZE(gpio_table));
}
