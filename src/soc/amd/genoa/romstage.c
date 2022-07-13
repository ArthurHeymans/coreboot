/* SPDX-License-Identifier: GPL-2.0-only */

#include <arch/cpu.h>
#include <arch/stages.h>
#include <cbmem.h>
#include <console/console.h>
#include <program_loading.h>
#include <romstage_common.h>
#include <stdint.h>
#include <timestamp.h>
#include <cpu/amd/mtrr.h>

void *cbmem_top_chipset(void)
{
	/* This SOC has no graphic device so we can just use TOP_MEM */
	uintptr_t top_mem = rdmsr(TOP_MEM).lo;
	top_mem -= CONFIG_SMM_TSEG_SIZE;
	/*
	 * This MSR has an 8M granularity.
	 * TSEG also needs to be aligned to its size so account for potentially ill aligned TOP_MEM.
	 */
	return (void *)ALIGN_DOWN(top_mem, MAX(8 * MiB, CONFIG_SMM_TSEG_SIZE));
}

void __noreturn romstage_main(void)
{
	post_code(0x40);

	cbmem_initialize_empty();
	run_ramstage();
}
