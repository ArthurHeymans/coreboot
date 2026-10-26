/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/apob_cache.h>
#include <amdblocks/memmap.h>
#include <cbmem.h>
#include <commonlib/helpers.h>
#include <console/console.h>
#include <cpu/amd/mtrr.h>
#include <cpu/x86/msr.h>
#include <romstage_handoff.h>
#include <soc/soc_util.h>
#include <types.h>

#include "apob.h"
#include "fch.h"
#include "native.h"
#include "nbio.h"
#include "smu.h"
#include "topology.h"

void __weak mainboard_fch_reset_done(void) {}

/* CBMEM goes below TSEG, which is at the top of the DRAM below 4 GiB */
static uintptr_t calculate_cbmem_top(void)
{
	uintptr_t top = get_top_of_mem_below_4gb();
	const uint32_t tolm = sp3_apob_tolm(sp3_apob());

	if (tolm && tolm < top)
		top = tolm;
	if (top <= CONFIG_SMM_TSEG_SIZE)
		die("Invalid top of memory\n");

	return ALIGN_DOWN(top - CONFIG_SMM_TSEG_SIZE, 8 * MiB);
}

uintptr_t cbmem_top_chipset(void)
{
	static uintptr_t top;

	if (!top)
		top = calculate_cbmem_top();
	return top;
}

/* The PSP has trained the memory and reported the results in the APOB. */
void soc_romstage_init(void)
{
	struct sp3_apob_hole holes[APOB_MAX_HOLES];
	uint64_t top = 0;
	int hole_count;

	sp3_print_cpu();
	/* The vendor SEC sets TOP_MEM to 4 GiB when it is 0; it is expected from the PSP */
	if (!rdmsr(TOP_MEM_MSR).lo)
		die("TOP_MEM wasn't set up by the PSP\n");
	if (!sp3_is_supported())
		die("Unsupported CPU\n");
	printk(BIOS_INFO, "SP3: %u root bridges, %u socket(s)\n",
	       sp3_die_count(), sp3_socket_count());

	const struct apob_base_header *header = sp3_apob();
	printk(BIOS_INFO, "APOB: signature %08x version %x size %x first %x\n",
	       header->signature, header->version, header->size, header->offset_of_first_entry);
	if (!sp3_apob_is_valid(sp3_apob()))
		die("Invalid APOB\n");
	hole_count = sp3_apob_memory_map(sp3_apob(), &top, holes);
	if (hole_count < 0)
		die("Invalid APOB memory map\n");
	printk(BIOS_INFO, "APOB: memory clock %u MHz, top of memory 0x%llx\n",
	       sp3_apob_memclk(sp3_apob()), top);
	for (int i = 0; i < hole_count; i++)
		printk(BIOS_DEBUG, "APOB hole %d: type %u, base 0x%llx, size 0x%llx\n",
		       i, holes[i].type, holes[i].base, holes[i].size);

	/* The fixed early DRAM and the APOB must not be reserved by the PSP */
	for (int i = 0; i < hole_count; i++) {
		const uint64_t end = holes[i].base + holes[i].size;

		if ((holes[i].base < CONFIG_ROMSTAGE_ADDR + CONFIG_ROMSTAGE_SIZE &&
		     end > CONFIG_EARLY_RESERVED_DRAM_BASE) ||
		    (holes[i].base < CONFIG_PSP_APOB_DRAM_ADDRESS + CONFIG_PSP_APOB_DRAM_SIZE &&
		     end > CONFIG_PSP_APOB_DRAM_ADDRESS))
			die("APOB hole %d overlaps the early DRAM or the APOB\n", i);
	}

	if (cbmem_recovery(false))
		die("Failed to set up CBMEM\n");
	if (romstage_handoff_init(false))
		die("Failed to set up the romstage handoff\n");

	printk(BIOS_INFO, "Native silicon initialization (romstage)\n");

	/* AmdSocSp3RmPei and AmdFabricSspPei. */
	sp3_df_mmio_init();
	sp3_df_early_init();
	sp3_df_scrub_init();

	/* Huashan reset tasks, then the board's CPM GPIO/LPC setup. */
	sp3_fch_reset_init();
	sp3_fch_sata_reset(0);
	mainboard_fch_reset_done();

	/* Rome NBIO base and CCX PEI phases. */
	sp3_nbio_early_init();
	sp3_psp_bar_init();
	sp3_ccx_bsp_init();
	sp3_smu_print_name();
	sp3_nbio_tom_init();
}
