/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/smn.h>
#include <console/console.h>
#include <device/device.h>
#include <static.h>
#include <timer.h>
#include <types.h>

#include "nbio.h"

#define ACP_TIMEOUT_US	1000

static enum cb_err acp_wait(uint32_t reg, uint32_t mask, uint32_t expected)
{
	uint32_t value;

	/* All ones means that the ACP doesn't respond, so there is no point in waiting */
	wait_us(ACP_TIMEOUT_US,
		((value = smn_read32(reg)) & mask) == expected || value == UINT32_MAX);
	if (value != UINT32_MAX && (value & mask) == expected)
		return CB_SUCCESS;

	printk(BIOS_ERR, "ACP timeout: SMN %08x=%08x mask=%08x expected=%08x\n",
	       reg, value, mask, expected);
	return CB_ERR;
}

/* Power off the ACP when it's disabled in the devicetree and hide its PCI function. */
enum cb_err picasso_acp_power_off(void)
{
	if (is_dev_enabled(DEV_PTR(acp)))
		return CB_SUCCESS;

	/* The ACP needs to be powered and out of reset to shut down its memories. */
	smn_clrsetbits32(ACP_PGFSM_CONTROL, 0, ACP_PGFSM_POWER_ON);
	if (acp_wait(ACP_PGFSM_STATUS, ACP_PGFSM_STATUS_MASK, 0))
		return CB_ERR;
	smn_clrsetbits32(ACP_CONTROL, 0, ACP_CLOCK_ENABLE);
	smn_clrsetbits32(ACP_SOFT_RESET, 0, ACP_SOFT_RESET_AUDIO);
	if (acp_wait(ACP_SOFT_RESET, ACP_SOFT_RESET_AUDIO_DONE, ACP_SOFT_RESET_AUDIO_DONE))
		return CB_ERR;
	smn_write32(ACP_SOFT_RESET, 0);
	if (acp_wait(ACP_SOFT_RESET, ACP_SOFT_RESET_AUDIO | ACP_SOFT_RESET_AUDIO_DONE, 0))
		return CB_ERR;

	smn_clrsetbits32(ACP_DYNAMIC_CG_MASTER_CONTROL, 0xffff, 0x1ff);
	smn_clrsetbits32(ACP_PGMEM_CTRL, ACP_PGMEM_SHUTDOWN_MASK, ACP_PGMEM_SHUTDOWN_MASK);
	if (acp_wait(ACP_MEM_PG_STS, ACP_MEM_PG_STS_MASK, 0))
		return CB_ERR;
	smn_write32(ACP_DYNAMIC_CG_MASTER_CONTROL, 0);
	smn_write32(ACP_CONTROL, 0);

	smn_clrsetbits32(NBIF_FUNC_STRAP(NBIF_FUNC_ACP), NBIF_FUNC_STRAP_FUNC_EN, 0);
	return CB_SUCCESS;
}
