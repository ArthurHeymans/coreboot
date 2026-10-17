/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/smn.h>
#include <console/console.h>
#include <device/pci_ops.h>
#include <soc/smu.h>
#include <soc/soc_util.h>
#include <timer.h>
#include <types.h>

#include "topology.h"

#define DF_DEV(die)		PCI_DEV(0, 0x18 + (die), 0)
#define DF_FN(die, fn)		PCI_DEV(0, 0x18 + (die), fn)
#define DF_F0_BUS_BASE		0x84
#define DF_F1_SYSTEM_CFG	0x200
#define DF_F1_SYSTEM_NODES	0x204
#define DF_F4_FICAA		0x5c
#define DF_F4_FICAD_LO		0x98

#define SMU_TIMEOUT_MS		10000

unsigned int zeppelin_die_count(void)
{
	const unsigned int dies = pci_s_read_config32(DF_FN(0, 1), DF_F1_SYSTEM_NODES) & 0xff;
	const enum zeppelin_package pkg = zeppelin_package();

	/* The topology is live, but only one socket with the package's die count is known */
	if ((pkg == ZEPPELIN_PKG_SP4R2 && dies != 1) ||
	    (pkg == ZEPPELIN_PKG_SP4 && dies != 2) ||
	    (pkg != ZEPPELIN_PKG_SP4R2 && pkg != ZEPPELIN_PKG_SP4))
		die("Unsupported topology: package %u with %u dies\n", pkg, dies);

	return dies;
}

uint8_t zeppelin_die_bus(unsigned int die)
{
	return pci_s_read_config32(DF_DEV(die), DF_F0_BUS_BASE) & 0xff;
}

static pci_devfn_t die_root(unsigned int die)
{
	return die ? PCI_DEV(zeppelin_die_bus(die), 0, 0) : PCI_DEV(0, 0, 0);
}

uint32_t zeppelin_smn_read32(unsigned int die, uint32_t reg)
{
	const pci_devfn_t root = die_root(die);

	pci_s_write_config32(root, SMN_INDEX_ADDR, reg);
	return pci_s_read_config32(root, SMN_DATA_ADDR);
}

void zeppelin_smn_write32(unsigned int die, uint32_t reg, uint32_t value)
{
	const pci_devfn_t root = die_root(die);

	pci_s_write_config32(root, SMN_INDEX_ADDR, reg);
	pci_s_write_config32(root, SMN_DATA_ADDR, value);
}

/* FICAA: [0] instance access, [10:2] register, [13:11] function, [23:16] instance */
static void df_set_indirect(unsigned int die, uint8_t fn, uint16_t reg, uint8_t instance)
{
	pci_s_write_config32(DF_FN(die, 4), DF_F4_FICAA,
			   (uint32_t)instance << 16 | (fn & 7) << 11 | (reg & 0x7fc) | 1);
}

uint32_t zeppelin_df_read32(unsigned int die, uint8_t fn, uint16_t reg, uint8_t instance)
{
	if (instance == ZEPPELIN_DF_DIRECT)
		return pci_s_read_config32(DF_FN(die, fn), reg);

	df_set_indirect(die, fn, reg, instance);
	return pci_s_read_config32(DF_FN(die, 4), DF_F4_FICAD_LO);
}

void zeppelin_df_write32(unsigned int die, uint8_t fn, uint16_t reg, uint8_t instance,
			 uint32_t value)
{
	if (instance == ZEPPELIN_DF_DIRECT) {
		pci_s_write_config32(DF_FN(die, fn), reg, value);
		return;
	}

	df_set_indirect(die, fn, reg, instance);
	pci_s_write_config32(DF_FN(die, 4), DF_F4_FICAD_LO, value);
}

/* The vendor firmware polls without a timeout; time out instead of hanging */
static uint32_t smu_wait(unsigned int die)
{
	struct stopwatch sw;
	uint32_t response;

	stopwatch_init_msecs_expire(&sw, SMU_TIMEOUT_MS);
	while (!(response = zeppelin_smn_read32(die, SMN_SMU_MESG_RESP))) {
		if (stopwatch_expired(&sw))
			return 0;
	}
	return response;
}

enum cb_err zeppelin_smu_send(unsigned int die, uint32_t message, struct smu_payload *payload)
{
	uint32_t response;

	if (!smu_wait(die)) {
		printk(BIOS_ERR, "SMU %u busy before message 0x%x\n", die, message);
		return CB_ERR;
	}

	zeppelin_smn_write32(die, SMN_SMU_MESG_RESP, 0);
	for (unsigned int i = 0; i < SMU_NUM_ARGS; i++)
		zeppelin_smn_write32(die, SMN_SMU_MESG_ARG(i), payload->msg[i]);
	zeppelin_smn_write32(die, SMN_SMU_MESG_ID, message);

	response = smu_wait(die);
	if (response != SMU_MESG_RESP_OK) {
		printk(BIOS_ERR, "SMU %u message 0x%x failed: response 0x%x\n", die, message,
		       response);
		return CB_ERR;
	}

	for (unsigned int i = 0; i < SMU_NUM_ARGS; i++)
		payload->msg[i] = zeppelin_smn_read32(die, SMN_SMU_MESG_ARG(i));
	return CB_SUCCESS;
}
