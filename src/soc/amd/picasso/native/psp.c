/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/smn.h>
#include <soc/iomap.h>
#include <types.h>

#include "native.h"
#include "nbio.h"

/*
 * Map the PSP MMIO registers right above the SMU ones. Lock the BAR, so it gets reported as
 * a fixed resource by the root complex code.
 */
void picasso_psp_bar_init(void)
{
	const uint32_t bar = smn_read32(PSP_BASE_ADDR_LO);

	if ((bar & 0xfff00000) == PSP_MMIO_BASE_ADDRESS && (bar & IOHC_MMIO_EN) &&
	    (bar & IOHC_MMIO_LOCK))
		return;

	smn_write32(PSP_BASE_ADDR_LO,
		    PSP_MMIO_BASE_ADDRESS | IOHC_MMIO_LOCK | IOHC_MMIO_EN);
	smn_write32(PSP_BASE_ADDR_HI, 0);
}
