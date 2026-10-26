/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/psp.h>
#include <cpu/amd/msr.h>
#include <cpu/x86/msr.h>
#include <console/console.h>
#include "../../common/block/psp/psp_def.h"
#include "native.h"
#include "topology.h"

static uint32_t psp_base;

void sp3_psp_bar_init(void)
{
	unsigned int rb;

	for (rb = 0; rb < sp3_die_count(); rb++)
		if (sp3_nbio_index(rb) == 0)
			break;
	if (rb == sp3_die_count())
		die("SP3 PSP IOHC0 is absent\n");
	uint32_t bar = sp3_smn_read32(rb, 0x13b102e0);
	if (!(bar & 1)) {
		bar = sp3_nonpci_mmio_base(rb) | 0x101;
		sp3_smn_write32(rb, 0x13b102e4, 0);
		sp3_smn_write32(rb, 0x13b102e0, bar);
	}
	psp_base = bar & 0xfff00000;
	/* PSP_ADDR is write-once; only initialize the BSP here. APs do it once each. */
	msr_write(PSP_ADDR_MSR, psp_base);
}

void sp3_psp_cache_bars(void)
{
	psp_base = sp3_smn_read32(0, 0x13b102e0) & 0xfff00000;
}

uint32_t sp3_psp_bar(unsigned int unused)
{
	return psp_base;
}

void sp3_psp_lock_df(void)
{
	struct mbox_default_buffer buffer = { .header = { .size = sizeof(buffer) } };

	if (send_psp_command(0x1b, &buffer))
		printk(BIOS_ERR, "PSP DF lock failed\n");
}
