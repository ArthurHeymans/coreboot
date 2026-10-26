/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_SP3_PLATFORM_DESCRIPTORS_H
#define AMD_SP3_PLATFORM_DESCRIPTORS_H

#include <types.h>

#define PCIE_ENGINE 1
#define SATA_ENGINE 3
#define GEN_MAX 0
#define GEN1 1
#define GEN2 2
#define GEN3 3
#define GEN4 4

struct sp3_phy_parameter {
	uint8_t id;
	uint8_t value;
};

typedef struct {
	uint8_t engine_type;
	uint8_t start_logical_lane; /* Board PHY lanes, including reversal. */
	uint8_t end_logical_lane;
	uint8_t device_number;
	uint8_t function_number;
	uint8_t link_speed_capability;
	bool port_present;
	const struct sp3_phy_parameter *tuning;
	size_t tuning_count;
} dxio_descriptor;

void mainboard_get_dxio_descriptors(unsigned int socket, const dxio_descriptor **dxio,
				    size_t *count);

#endif /* AMD_SP3_PLATFORM_DESCRIPTORS_H */
