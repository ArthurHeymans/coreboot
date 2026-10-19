/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef MJ11_VARIANT_DXIO_H
#define MJ11_VARIANT_DXIO_H

#include <soc/platform_descriptors.h>
#include <types.h>

/*
 * The vendor CPM DXIO template ($A26) with the Setup defaults applied:
 * SlimSAS as PCIe and both I210s on. The vendor sets "turn off unused lanes"
 * on every engine and leaves the link speed at its maximum. The x16 slot
 * (lanes 31..16, reversed) is split according to the riser, as the vendor
 * "Auto" bifurcation does.
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

#define MJ11_FIXED_ENGINES						\
	{ /* 4x SATA */							\
		.engine_type = SATA_ENGINE,				\
		.start_logical_lane = 0,				\
		.end_logical_lane = 3,					\
		.gpio_group_id = 1,					\
		.channel_type = SATA_CHANNEL_OTHER,			\
		.turn_off_unused_lanes = true,				\
	},								\
	MJ11_PCIE(4, 7, 1, 2),		/* SlimSAS */			\
	MJ11_PCIE(8, 11, 1, 3),		/* M.2 */			\
	MJ11_PCIE(12, 13, 1, 4),	/* I210 #1 */			\
	MJ11_PCIE(14, 14, 1, 5),	/* I210 #2 */			\
	MJ11_PCIE(15, 15, 1, 1)		/* AST2500 */

/*
 * The DXIO descriptors of the variant's vendor CPM template ($A26), used when
 * no riser selects a bifurcation of the x16 slot.
 */
void variant_default_dxio(const dxio_descriptor **dxio, size_t *count);

#endif /* MJ11_VARIANT_DXIO_H */
