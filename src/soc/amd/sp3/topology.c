/* SPDX-License-Identifier: GPL-2.0-only */

#include <soc/topology.h>
#include <stddef.h>

/* DF2/DF3 topology contracts: Documentation/soc/amd/sp3.md. */
#define DF_SYSTEM_CFG		0x200
#define DF_COMPONENT_COUNT	0x204
#define DF_CFG_ADDR_CTL		0x084
#define DF_CFG_MAP(i)		(0x0a0 + 4 * (i))
#define DF3_CCM0		0x10
#define DF3_IOMS0		0x18
#define DF3_FCH_IOMS		3

static enum sp3_topology_error add_root(struct sp3_topology *t, uint8_t socket,
	uint8_t index, uint16_t fabric_id, uint8_t base, const uint32_t maps[8], bool df3)
{
	const uint32_t dst_mask = df3 ? 0x3ff : 0xff;
	struct sp3_root_bridge root = {
		.socket = socket,
		.index = index,
		.fabric_id = fabric_id,
		.bus_base = base,
	};
	unsigned int matches = 0;

	for (unsigned int i = 0; i < 8; i++) {
		const uint32_t map = maps[i];

		if ((map & 3) != 3 || ((map >> 4) & dst_mask) != fabric_id)
			continue;
		if (((map >> 16) & 0xff) != base || (map >> 24) < base)
			return SP3_TOPOLOGY_BUS_MAP;
		root.bus_limit = map >> 24;
		matches++;
	}
	if (matches != 1)
		return SP3_TOPOLOGY_BUS_MAP;

	for (unsigned int i = 0; i < t->root_bridge_count; i++) {
		const struct sp3_root_bridge *other = &t->root_bridges[i];

		if (root.bus_base <= other->bus_limit && other->bus_base <= root.bus_limit)
			return SP3_TOPOLOGY_BUS_MAP;
	}
	if (t->root_bridge_count == SP3_MAX_ROOT_BRIDGES)
		return SP3_TOPOLOGY_COUNT;
	if (socket == 0 && index == (df3 ? DF3_FCH_IOMS : 0))
		t->primary = t->root_bridge_count;
	t->root_bridges[t->root_bridge_count++] = root;
	return SP3_TOPOLOGY_OK;
}

static void logical_order(struct sp3_topology *t)
{
	for (unsigned int i = 0; i < t->root_bridge_count; i++) {
		unsigned int pos = i;

		while (pos && t->root_bridges[t->logical[pos - 1]].bus_base >
		       t->root_bridges[i].bus_base) {
			t->logical[pos] = t->logical[pos - 1];
			pos--;
		}
		t->logical[pos] = i;
	}
}

enum sp3_topology_error sp3_discover_topology(enum sp3_generation generation,
	sp3_df_reader read, void *context, const uint8_t ioms_present[SP3_MAX_SOCKETS],
	struct sp3_topology *topology)
{
	struct sp3_topology found = { .primary = SP3_MAX_ROOT_BRIDGES };
	uint32_t config, count, maps[8];
	bool df3;

	if (!read || !topology)
		return SP3_TOPOLOGY_ARGUMENT;
	switch (generation) {
	case SP3_NAPLES:
		df3 = false;
		break;
	case SP3_ROME:
		df3 = true;
		break;
	case SP3_MILAN:
		if (!ioms_present)
			return SP3_TOPOLOGY_ARGUMENT;
		df3 = true;
		break;
	default:
		return SP3_TOPOLOGY_ARGUMENT;
	}

	if (!read(context, 0, SP3_DF_DIRECT, 1, DF_SYSTEM_CFG, &config) ||
	    !read(context, 0, SP3_DF_DIRECT, 1, DF_COMPONENT_COUNT, &count))
		return SP3_TOPOLOGY_IO;
	found.sockets = ((config >> (df3 ? 27 : 5)) & 1) + 1;
	for (unsigned int i = 0; i < 8; i++) {
		if (!read(context, 0, df3 ? DF3_CCM0 : SP3_DF_DIRECT,
			  0, DF_CFG_MAP(i), &maps[i]))
			return SP3_TOPOLOGY_IO;
	}

	for (unsigned int socket = 0; socket < found.sockets; socket++) {
		const uint8_t first_node = df3 ? socket : 4 * socket;
		uint8_t present;

		if (df3) {
			present = generation == SP3_MILAN ? ioms_present[socket] : 0xf;
			if (present & ~0xf)
				return SP3_TOPOLOGY_ARGUMENT;
			if (!present)
				return SP3_TOPOLOGY_NO_DIES;
			found.nodes[found.node_count++] = (struct sp3_node) {
				.socket = socket, .device = 0x18 + first_node,
			};
		} else {
			if (!read(context, first_node, SP3_DF_DIRECT, 1, DF_SYSTEM_CFG, &config))
				return SP3_TOPOLOGY_IO;
			present = config & 0xf;
			/* The donor uses ctz(present + 1): only a contiguous die prefix. */
			if (!present || (present & (present + 1)))
				return SP3_TOPOLOGY_NO_DIES;
		}

		for (unsigned int index = 0; index < 4; index++) {
			const uint8_t node = df3 ? socket : first_node + index;
			const uint16_t fabric_id = df3 ? (socket << 5) + DF3_IOMS0 + index :
				(socket << 7) | (index << 5) | 4;
			uint32_t base;
			enum sp3_topology_error error;

			if (!(present & (1 << index)))
				continue;
			if (!df3)
				found.nodes[found.node_count++] = (struct sp3_node) {
					.socket = socket, .die = index, .device = 0x18 + node,
				};
			if (!read(context, node, df3 ? DF3_IOMS0 + index : SP3_DF_DIRECT,
				  0, DF_CFG_ADDR_CTL, &base))
				return SP3_TOPOLOGY_IO;
			error = add_root(&found, socket, index, fabric_id, base & 0xff, maps, df3);
			if (error != SP3_TOPOLOGY_OK)
				return error;
		}
	}
	if (found.primary == SP3_MAX_ROOT_BRIDGES)
		return SP3_TOPOLOGY_NO_PRIMARY;
	if ((df3 ? (count >> 16) & 0xff : count & 0xff) != found.root_bridge_count)
		return SP3_TOPOLOGY_COUNT;

	logical_order(&found);
	*topology = found;
	return SP3_TOPOLOGY_OK;
}
