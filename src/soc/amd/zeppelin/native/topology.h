/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_ZEPPELIN_NATIVE_TOPOLOGY_H
#define AMD_ZEPPELIN_NATIVE_TOPOLOGY_H

#include <amdblocks/smu.h>
#include <types.h>

/*
 * Zeppelin has one Data Fabric, NBIO, SMU and FCH per die. Every die has its
 * own DF PCI device on bus 0 (0x18 + die) and its own root bus; the SMN
 * addresses are the same on every die and are reached through D0F0 0xb8/0xbc
 * of that die's root bus.
 */
#define ZEPPELIN_MAX_DIES	2

/* Dies in the system, from the DF system configuration (1 on SP4r2, 2 on SP4) */
unsigned int zeppelin_die_count(void);
/* Root bus of a die */
uint8_t zeppelin_die_bus(unsigned int die);
/* Fabric ID of the IOMS (the PCIe root) of a die */
static inline uint8_t zeppelin_ioms_fabric_id(unsigned int die)
{
	return die << 5 | 4;
}

uint32_t zeppelin_smn_read32(unsigned int die, uint32_t reg);
void zeppelin_smn_write32(unsigned int die, uint32_t reg, uint32_t value);
static inline void zeppelin_smn_clrsetbits32(unsigned int die, uint32_t reg, uint32_t clear,
					      uint32_t set)
{
	zeppelin_smn_write32(die, reg, (zeppelin_smn_read32(die, reg) & ~clear) | set);
}

/*
 * DF accesses on one die. instance == ZEPPELIN_DF_DIRECT accesses the die's
 * own register (what AGESA calls broadcast), any other value one fabric
 * instance through FICAA/FICAD of that die.
 */
#define ZEPPELIN_DF_DIRECT	0xff
uint32_t zeppelin_df_read32(unsigned int die, uint8_t fn, uint16_t reg, uint8_t instance);
void zeppelin_df_write32(unsigned int die, uint8_t fn, uint16_t reg, uint8_t instance,
			 uint32_t value);
static inline void zeppelin_df_clrsetbits32(unsigned int die, uint8_t fn, uint16_t reg,
					     uint8_t instance, uint32_t clear, uint32_t set)
{
	zeppelin_df_write32(die, fn, reg, instance,
			    (zeppelin_df_read32(die, fn, reg, instance) & ~clear) | set);
}

/* Send a message to the SMU of a die. The payload gets the returned arguments. */
enum cb_err zeppelin_smu_send(unsigned int die, uint32_t message, struct smu_payload *payload);

#endif /* AMD_ZEPPELIN_NATIVE_TOPOLOGY_H */
