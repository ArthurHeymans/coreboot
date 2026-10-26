/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef SOC_AMD_SP3_TOPOLOGY_H
#define SOC_AMD_SP3_TOPOLOGY_H

#include <soc/identity.h>

#define SP3_MAX_SOCKETS		2
#define SP3_MAX_NODES		8
#define SP3_MAX_ROOT_BRIDGES	8
#define SP3_DF_DIRECT		0xff

struct sp3_node {
	uint8_t socket;
	uint8_t die;
	uint8_t device;
};

struct sp3_root_bridge {
	uint8_t socket;
	uint8_t index;		/* Physical die (DF2) or NBIO quadrant (DF3). */
	uint16_t fabric_id;
	uint8_t bus_base;
	uint8_t bus_limit;
};

struct sp3_topology {
	uint8_t sockets;
	uint8_t node_count;
	uint8_t root_bridge_count;
	uint8_t primary;		/* Index in root_bridges, not the physical quadrant. */
	struct sp3_node nodes[SP3_MAX_NODES];
	/* Physical socket/die/quadrant order; never reorder for PCI enumeration. */
	struct sp3_root_bridge root_bridges[SP3_MAX_ROOT_BRIDGES];
	/* Indices into root_bridges, sorted by root bus (AGESA logical order). */
	uint8_t logical[SP3_MAX_ROOT_BRIDGES];
};

enum sp3_topology_error {
	SP3_TOPOLOGY_OK,
	SP3_TOPOLOGY_ARGUMENT,
	SP3_TOPOLOGY_IO,
	SP3_TOPOLOGY_NO_DIES,
	SP3_TOPOLOGY_COUNT,
	SP3_TOPOLOGY_NO_PRIMARY,
	SP3_TOPOLOGY_BUS_MAP,
};

/* Discovery runs on the BSP before parallel use of the DF index/data pair. */
typedef bool (*sp3_df_reader)(void *context, uint8_t node, uint8_t instance,
			      uint8_t function, uint16_t reg, uint32_t *value);

/*
 * ioms_present[socket]: Milan fuse-filtered quadrant mask, supplied separately
 * for each socket. Rome has all four quadrants. Naples has one NBIO per die.
 * No register writes; output is unchanged on error.
 */
enum sp3_topology_error sp3_discover_topology(enum sp3_generation generation,
	sp3_df_reader read, void *context, const uint8_t ioms_present[SP3_MAX_SOCKETS],
	struct sp3_topology *topology);

#endif /* SOC_AMD_SP3_TOPOLOGY_H */
