/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/psp.h>
#include <amdblocks/smn.h>
#include <console/console.h>
#include <soc/iomap.h>
#include <types.h>

#include "../../common/block/psp/psp_def.h"
#include "native.h"
#include "nbio.h"

/* PSP mailbox command that locks the DF registers (vendor LockDFReg) */
#define MBOX_BIOS_CMD_LOCK_DF_REG	0x1b

/*
 * Map the 1 MiB PSP register window of die 0. The vendor firmware enables and
 * locks it (|0x101) in the same register. A locked BAR gets reported as a fixed
 * resource by the root complex code.
 */
void zeppelin_psp_bar_init(void)
{
	const uint32_t bar = smn_read32(PSP_BASE_ADDR_LO);

	if ((bar & 0xfff00000) == PSP_MMIO_BASE_ADDRESS && (bar & IOHC_MMIO_EN) &&
	    (bar & IOHC_MMIO_LOCK))
		return;

	smn_write32(PSP_BASE_ADDR_HI, 0);
	smn_write32(PSP_BASE_ADDR_LO, PSP_MMIO_BASE_ADDRESS | IOHC_MMIO_LOCK | IOHC_MMIO_EN);
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
