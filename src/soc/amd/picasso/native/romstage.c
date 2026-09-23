/* SPDX-License-Identifier: GPL-2.0-only */

#include <acpi/acpi.h>
#include <amdblocks/apob_cache.h>
#include <amdblocks/memmap.h>
#include <cbmem.h>
#include <cf9_reset.h>
#include <commonlib/helpers.h>
#include <console/console.h>
#include <cpu/amd/mtrr.h>
#include <romstage_handoff.h>
#include <soc/platform_descriptors.h>
#include <types.h>

#include "apob.h"
#include "native.h"
#include "nbio.h"

static uintptr_t native_cbmem_top;

/* CBMEM goes below TSEG, which is at the top of the DRAM below 4 GiB */
static uintptr_t calculate_cbmem_top(const void *apob)
{
	uintptr_t top = get_top_of_mem_below_4gb();
	const uint32_t tolm = picasso_apob_tolm(apob);

	if (tolm && tolm < top)
		top = tolm;
	if (top <= CONFIG_SMM_TSEG_SIZE)
		die("Invalid top of memory\n");

	return ALIGN_DOWN(top - CONFIG_SMM_TSEG_SIZE, 8 * MiB);
}

uintptr_t cbmem_top_chipset(void)
{
	if (!native_cbmem_top)
		native_cbmem_top = calculate_cbmem_top(soc_fill_apob_cache());
	return native_cbmem_top;
}

static void __noreturn fatal(bool s3_resume, const char *msg)
{
	/* A reset gives the PSP a chance to do a cold boot instead of the failing resume */
	if (s3_resume) {
		printk(BIOS_ERR, "%s\n", msg);
		full_reset();
	}
	die("%s\n", msg);
}

/* The PSP has trained the memory and reported the results in the APOB. */
void soc_romstage_init(void)
{
	struct picasso_apob_hole holes[APOB_MAX_HOLES];
	const bool s3_resume = acpi_is_wakeup_s3();
	uint64_t top = 0;
	void *apob;
	int hole_count;

	mb_pre_silicon_init();

	if (CONFIG(SOC_AMD_COMMON_BLOCK_APOB_NV_DISABLE)) {
		if (s3_resume)
			fatal(true, "S3 resume requires the non-volatile APOB");
		apob = (void *)(uintptr_t)CONFIG_PSP_APOB_DRAM_ADDRESS;
	} else {
		apob = soc_fill_apob_cache();
	}
	if (!picasso_apob_is_valid(apob))
		fatal(s3_resume, "Invalid APOB");

	hole_count = picasso_apob_memory_map(apob, &top, holes);
	if (hole_count < 0)
		fatal(s3_resume, "Invalid APOB memory map");
	printk(BIOS_INFO, "APOB: memory clock %u MHz, top of memory 0x%llx\n",
	       picasso_apob_memclk(apob), top);
	for (int i = 0; i < hole_count; i++)
		printk(BIOS_DEBUG, "APOB hole %d: type %u, base 0x%llx, size 0x%llx\n",
		       i, holes[i].type, holes[i].base, holes[i].size);

	native_cbmem_top = calculate_cbmem_top(apob);
	if (cbmem_recovery(s3_resume))
		fatal(s3_resume, "Failed to set up CBMEM");
	if (romstage_handoff_init(s3_resume))
		fatal(s3_resume, "Failed to set up the romstage handoff");

	/* The PSP restores the DF, CCX and NBIO state on S3 resume */
	if (s3_resume) {
		picasso_psp_bar_init();
		if (picasso_pcie_topology_restore())
			fatal(true, "Failed to restore the PCIe topology");
		return;
	}

	picasso_native_early_init();
}

void picasso_native_early_init(void)
{
	printk(BIOS_INFO, "Native silicon initialization (romstage)\n");

	picasso_df_mmio_init();
	picasso_df_early_init();

	if (picasso_nbio_early_init())
		die("NBIO initialization failed\n");

	picasso_psp_bar_init();
	picasso_ccx_bsp_init();
	picasso_ccx_mca_init();
	if (picasso_apply_cpu_topology_policy())
		die("Failed to apply the CPU topology settings\n");

	picasso_smu_print_name();
	picasso_nbio_tom_init();
}
