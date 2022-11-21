/* SPDX-License-Identifier: GPL-2.0-only */

#include <cbmem.h>
#include <console/console.h>
#include <vendorcode/amd/opensil/console.h>
#include <xSIM-api.h>
#include <xPRF-api.h>

uintptr_t cbmem_top_chipset(void)
{
	SilDebugSetup(HostDebugService);
	uintptr_t top_mem = xPrfGetLowUsableDramAddress();
	printk(BIOS_DEBUG, "xPrfGetLowUsableDramAddress: 0x%lx\n", top_mem);
	top_mem -= CONFIG_SMM_TSEG_SIZE;
	if (CONFIG_SMM_TSEG_SIZE)
		top_mem = ALIGN_DOWN(top_mem, CONFIG_SMM_TSEG_SIZE);
	/*
	 * This MSR has an 8M granularity.
	 * TSEG also needs to be aligned to its size so account for potentially ill aligned TOP_MEM.
	 */
	return top_mem;
}
