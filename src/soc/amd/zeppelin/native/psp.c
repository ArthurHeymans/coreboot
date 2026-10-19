/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/psp.h>
#include <console/console.h>
#include <types.h>

#include "../../common/block/psp/psp_def.h"
#include "native.h"
#include "nbio.h"
#include "topology.h"

/* PSP mailbox command that locks the DF registers (vendor LockDFReg) */
#define MBOX_BIOS_CMD_LOCK_DF_REG	0x1b

static uint32_t psp_bar_of_die[ZEPPELIN_MAX_DIES];

/*
 * Map the 1 MiB PSP register window of every die at the start of the die's
 * non-PCI MMIO pool. The vendor firmware maps die 0's in PEI and the other
 * dies' in DXE, enabled and locked (|0x101). A locked BAR gets reported as a
 * fixed resource by the root complex code.
 */
void zeppelin_psp_bar_init(void)
{
	for (unsigned int die = 0; die < zeppelin_die_count(); die++) {
		const uint32_t base = zeppelin_nonpci_mmio_base(die) + ZEPPELIN_NONPCI_PSP;
		const uint32_t bar = zeppelin_smn_read32(die, PSP_BASE_ADDR_LO);
		const uint32_t want = base | IOHC_MMIO_LOCK | IOHC_MMIO_EN;

		if ((bar & (0xfff00000 | IOHC_MMIO_LOCK | IOHC_MMIO_EN)) == want)
			continue;
		if (bar & IOHC_MMIO_LOCK)
			printk(BIOS_ERR, "PSP BAR of die %u is locked at 0x%x\n", die, bar);

		zeppelin_smn_write32(die, PSP_BASE_ADDR_HI, 0);
		zeppelin_smn_write32(die, PSP_BASE_ADDR_LO, want);
	}
}

/* The APs can't use SMN in parallel, so the BSP reads the BARs before MP init */
void zeppelin_psp_cache_bars(void)
{
	for (unsigned int die = 0; die < zeppelin_die_count(); die++) {
		const uint32_t bar = zeppelin_smn_read32(die, PSP_BASE_ADDR_LO);

		psp_bar_of_die[die] = (bar & IOHC_MMIO_EN) ? bar & 0xfff00000 : 0;
	}
}

uint32_t zeppelin_psp_bar(unsigned int die)
{
	return die < ARRAY_SIZE(psp_bar_of_die) ? psp_bar_of_die[die] : 0;
}

/* After this the DF registers can't be changed anymore */
void zeppelin_psp_lock_df(void)
{
	struct mbox_default_buffer buffer = {
		.header = {
			.size = sizeof(buffer)
		}
	};

	printk(BIOS_DEBUG, "PSP: Lock the DF registers... ");
	if (send_psp_command(MBOX_BIOS_CMD_LOCK_DF_REG, &buffer)) {
		printk(BIOS_DEBUG, "Fail\n");
		return;
	}
	printk(BIOS_DEBUG, "OK\n");
}
