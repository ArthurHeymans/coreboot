/* SPDX-License-Identifier: GPL-2.0-only */

#include <soc/platform_descriptors.h>
#include <types.h>

/*
 * The vendor CPM DXIO template ($A26) with the Setup defaults applied:
 * SlimSAS as PCIe, both I210s on and the x16 slot not bifurcated (the vendor
 * "Auto" reads a riser ID first and falls back to x16). The vendor sets
 * "turn off unused lanes" on every engine and leaves the link speed at its
 * maximum.
 */
#define MJ11_PCIE(_start, _end, _dev, _fn) {	\
	.engine_type = PCIE_ENGINE,		\
	.start_logical_lane = _start,		\
	.end_logical_lane = _end,		\
	.gpio_group_id = 1,			\
	.port_present = true,			\
	.device_number = _dev,			\
	.function_number = _fn,			\
	.link_speed_capability = GEN_MAX,	\
	.turn_off_unused_lanes = true,		\
}

static const dxio_descriptor mj11_dxio_descriptors[] = {
	{ /* 4x SATA */
		.engine_type = SATA_ENGINE,
		.start_logical_lane = 0,
		.end_logical_lane = 3,
		.gpio_group_id = 1,
		.channel_type = SATA_CHANNEL_OTHER,
		.turn_off_unused_lanes = true,
	},
	MJ11_PCIE(4, 7, 1, 2),		/* SlimSAS */
	MJ11_PCIE(8, 11, 1, 3),		/* M.2 */
	MJ11_PCIE(12, 13, 1, 4),	/* I210 #1 */
	MJ11_PCIE(14, 14, 1, 5),	/* I210 #2 */
	MJ11_PCIE(15, 15, 1, 1),	/* AST2500 */
	MJ11_PCIE(31, 16, 3, 1),	/* x16 slot, lanes reversed */
};

void mainboard_get_dxio_descriptors(const dxio_descriptor **dxio, size_t *count)
{
	*dxio = mj11_dxio_descriptors;
	*count = ARRAY_SIZE(mj11_dxio_descriptors);
}
