/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/smn.h>
#include <console/console.h>
#include <device/pci_ops.h>
#include <soc/smu.h>
#include <timer.h>
#include "topology.h"

#define DF_FN(node, fn) PCI_DEV(0, 0x18 + (node), fn)

static void select_instance(unsigned int node, uint8_t fn, uint16_t reg, uint8_t instance)
{
	pci_s_write_config32(DF_FN(node, 4), 0x5c,
		instance << 16 | (fn & 7) << 11 | (reg & 0x7fc) | 1);
}

uint32_t sp3_df_node_read32(unsigned int node, uint8_t fn, uint16_t reg, uint8_t instance)
{
	if (instance == SP3_DF_DIRECT)
		return pci_s_read_config32(DF_FN(node, fn), reg);
	select_instance(node, fn, reg, instance);
	return pci_s_read_config32(DF_FN(node, 4), 0x98);
}

void sp3_df_node_write32(unsigned int node, uint8_t fn, uint16_t reg, uint8_t instance,
			uint32_t value)
{
	if (instance == SP3_DF_DIRECT) {
		pci_s_write_config32(DF_FN(node, fn), reg, value);
		return;
	}
	select_instance(node, fn, reg, instance);
	pci_s_write_config32(DF_FN(node, 4), 0x98, value);
}

static bool read_df(void *unused, uint8_t node, uint8_t instance, uint8_t fn,
		    uint16_t reg, uint32_t *value)
{
	*value = sp3_df_node_read32(node, fn, reg, instance);
	return true;
}

const struct sp3_topology *sp3_topology(void)
{
	static struct sp3_topology topology;
	static bool valid;

	if (!valid) {
		enum sp3_topology_error error = sp3_discover_topology(SP3_ROME,
			read_df, NULL, NULL, &topology);

		if (error != SP3_TOPOLOGY_OK)
			die("SP3 DF topology error %u\n", error);
		/* S8030 is a one-socket board; do not silently omit xGMI on a 2P board. */
		if (topology.sockets != 1)
			die("S8030 firmware requires one socket\n");
		valid = true;
		for (unsigned int i = 0; i < topology.root_bridge_count; i++) {
			const struct sp3_root_bridge *r = &topology.root_bridges[topology.logical[i]];

			printk(BIOS_INFO, "SP3 domain %u: socket %u NBIO %u fabric %03x buses %02x-%02x\n",
			       i, r->socket, r->index, r->fabric_id, r->bus_base, r->bus_limit);
		}
	}
	return &topology;
}

unsigned int sp3_die_count(void)
{
	return sp3_topology()->root_bridge_count;
}

unsigned int sp3_socket_count(void)
{
	return sp3_topology()->sockets;
}

unsigned int sp3_nbio_index(unsigned int rb)
{
	const struct sp3_topology *t = sp3_topology();

	if (rb >= t->root_bridge_count)
		die("Invalid SP3 root bridge\n");
	return t->root_bridges[t->logical[rb]].index;
}

uint8_t sp3_die_bus(unsigned int rb)
{
	const struct sp3_topology *t = sp3_topology();

	return t->root_bridges[t->logical[rb]].bus_base;
}

uint32_t sp3_smn_read32(unsigned int rb, uint32_t reg)
{
	pci_devfn_t root = PCI_DEV(sp3_die_bus(rb), 0, 0);

	pci_s_write_config32(root, SMN_INDEX_ADDR, reg);
	return pci_s_read_config32(root, SMN_DATA_ADDR);
}

void sp3_smn_write32(unsigned int rb, uint32_t reg, uint32_t value)
{
	pci_devfn_t root = PCI_DEV(sp3_die_bus(rb), 0, 0);

	pci_s_write_config32(root, SMN_INDEX_ADDR, reg);
	pci_s_write_config32(root, SMN_DATA_ADDR, value);
}

uint32_t sp3_df_read32(unsigned int rb, uint8_t fn, uint16_t reg, uint8_t instance)
{
	const struct sp3_topology *t = sp3_topology();

	return sp3_df_node_read32(t->root_bridges[t->logical[rb]].socket, fn, reg, instance);
}

void sp3_df_write32(unsigned int rb, uint8_t fn, uint16_t reg, uint8_t instance,
		    uint32_t value)
{
	const struct sp3_topology *t = sp3_topology();

	sp3_df_node_write32(t->root_bridges[t->logical[rb]].socket, fn, reg, instance, value);
}

enum cb_err sp3_smu_send(unsigned int rb, uint32_t message, struct smu_payload *payload)
{
	struct stopwatch sw;
	uint32_t response;

	/* V11 does not require the V9 pre-request busy poll. DXIO may take seconds. */
	sp3_smn_write32(rb, SMN_SMU_MESG_RESP, 0);
	for (unsigned int i = 0; i < SMU_NUM_ARGS; i++)
		sp3_smn_write32(rb, SMN_SMU_MESG_ARG(i), payload->msg[i]);
	sp3_smn_write32(rb, SMN_SMU_MESG_ID, message);
	stopwatch_init_msecs_expire(&sw, 30000);
	while (!(response = sp3_smn_read32(rb, SMN_SMU_MESG_RESP))) {
		if (stopwatch_expired(&sw))
			break;
	}
	if (response != 1) {
		printk(BIOS_ERR, "SP3 SMU message %x failed: %x\n", message, response);
		return CB_ERR;
	}
	for (unsigned int i = 0; i < SMU_NUM_ARGS; i++)
		payload->msg[i] = sp3_smn_read32(rb, SMN_SMU_MESG_ARG(i));
	return CB_SUCCESS;
}
