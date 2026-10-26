/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_SP3_NATIVE_TOPOLOGY_H
#define AMD_SP3_NATIVE_TOPOLOGY_H

#include <amdblocks/smu.h>
#include <types.h>
#include <soc/topology.h>

const struct sp3_topology *sp3_topology(void);
unsigned int sp3_nbio_index(unsigned int rb);
unsigned int sp3_socket_count(void);
uint32_t sp3_df_node_read32(unsigned int node, uint8_t fn, uint16_t reg, uint8_t instance);
void sp3_df_node_write32(unsigned int node, uint8_t fn, uint16_t reg, uint8_t instance,
			uint32_t value);

static inline uint32_t sp3_nbio_address(unsigned int rb, uint32_t address)
{
	return address + (sp3_nbio_index(rb) << 20);
}

/*
 * Rome has one DF and SMU per socket, with four NBIO root bridges. The
 * historical die-named helpers below index logical PCI root bridges, not
 * CCDs or sockets. Each root's D0F0 provides SMN access; NBIO addresses also
 * need the physical quadrant offset from sp3_nbio_address().
 */
#define SP3_MAX_DIES	4

/* Logical PCI root bridges in the system. */
unsigned int sp3_die_count(void);

/* Per-root non-PCI pool: PSP +0, SMU +1 MiB, IOMMU +2 MiB, IOAPIC +7 MiB. */
#define SP3_NONPCI_MMIO_SIZE	(8 * MiB)
#define SP3_NONPCI_PSP		0
#define SP3_NONPCI_IOAPIC		(7 * MiB)
uint32_t sp3_nonpci_mmio_base(unsigned int die);

/* Root bus of a logical root bridge. */
uint8_t sp3_die_bus(unsigned int die);
/* Live IOMS fabric ID of a logical root bridge. */
static inline uint16_t sp3_ioms_fabric_id(unsigned int rb)
{
	const struct sp3_topology *t = sp3_topology();

	return t->root_bridges[t->logical[rb]].fabric_id;
}

uint32_t sp3_smn_read32(unsigned int die, uint32_t reg);
void sp3_smn_write32(unsigned int die, uint32_t reg, uint32_t value);
static inline void sp3_smn_clrsetbits32(unsigned int die, uint32_t reg, uint32_t clear,
					      uint32_t set)
{
	sp3_smn_write32(die, reg, (sp3_smn_read32(die, reg) & ~clear) | set);
}

/*
 * DF accesses through a logical root bridge's socket. SP3_DF_DIRECT selects
 * broadcast access; other instance IDs select FICAA/FICAD indirect access.
 */
#define SP3_DF_DIRECT	0xff
uint32_t sp3_df_read32(unsigned int die, uint8_t fn, uint16_t reg, uint8_t instance);
void sp3_df_write32(unsigned int die, uint8_t fn, uint16_t reg, uint8_t instance,
			 uint32_t value);
static inline void sp3_df_clrsetbits32(unsigned int die, uint8_t fn, uint16_t reg,
					     uint8_t instance, uint32_t clear, uint32_t set)
{
	sp3_df_write32(die, fn, reg, instance,
			    (sp3_df_read32(die, fn, reg, instance) & ~clear) | set);
}

/* Send to the root bridge's socket SMU; payload receives returned arguments. */
enum cb_err sp3_smu_send(unsigned int die, uint32_t message, struct smu_payload *payload);

#endif /* AMD_SP3_NATIVE_TOPOLOGY_H */
