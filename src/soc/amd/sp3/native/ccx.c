/* SPDX-License-Identifier: GPL-2.0-only */

/*
 * Rome CCX initialization from AmdCcxVhPei/Dxe (SP3 report CCX_SP3).
 */

#include <amdblocks/amd_pci_mmconf.h>
#include <amdblocks/cpu.h>
#include <amdblocks/msr_zen.h>
#include <amdblocks/psp.h>
#include <arch/cpu.h>
#include <console/console.h>
#include <cpu/amd/msr.h>
#include <cpu/amd/mtrr.h>
#include <cpu/x86/cache.h>
#include <cpu/x86/lapic.h>
#include <cpu/x86/msr.h>
#include <smp/spinlock.h>
#include <soc/soc_util.h>
#include <types.h>

#include "ccx_tables.h"
#include "native.h"
#include "smu.h"

#define CSTATE_BASE_ADDR_MSR		0xc0010073
#define  CSTATE_IO_BASE			0x813	/* vendor DXE PCD 0x14d */
#define CPU_WDT_CFG_MSR			0xc0010074
#define  CPU_WDT_2047_1_31MS		0x9
#define CSTATE_CONFIG_MSR		0xc0010296
#define  CSTATE_CONFIG_CC6		0x484848
#define MCA_INTR_CFG_MSR		0xc0000410
#define DPM_CFG_MSR			0xc0011074
#define  DPM_CFG_WEIGHTS_VALID		(1ULL << 63)
#define DPM_WAC_INDEX_MSR		0xc0011076
#define DPM_WAC_DATA_MSR		0xc0011077

/* MCA_CONFIG fields */
#define MCA_CONFIG_DEFERRED_LOGGING_SUPPORTED	BIT(2)
#define MCA_CONFIG_MCAX_ENABLE			(1ULL << 32)
#define MCA_CONFIG_LOG_DEFERRED_IN_MCA_STAT	(1ULL << 34)

static bool primary_thread_of_core(void)
{
	const unsigned int threads = ((cpuid_ebx(0x8000001e) >> 8) & 0xff) + 1;

	return ((cpuid_ebx(1) >> 24) % threads) == 0;
}

/* Initial APIC ID; APOB describes the logical CCD/CCX/core/thread mapping. */
static unsigned int initial_apicid(void)
{
	return cpuid_ebx(1) >> 24;
}

/* The reset register table of the vendor CCX code */
static void ccx_apply_msr_table(void)
{
	const uint8_t rev = sp3_revision();
	const bool core_primary = primary_thread_of_core();
	const unsigned int mask = ((cpuid_ebx(0x8000001e) >> 8) & 0xff) ? 7 : 3;
	const bool complex_primary = (initial_apicid() & mask) == 0;

	for (size_t i = 0; i < sp3_ccx_msr_table_count; i++) {
		const struct sp3_ccx_msr *m = &sp3_ccx_msr_table[i];

		if (m->rev && !(m->rev & rev))
			continue;
		if (m->scope == CCX_SCOPE_CORE && !core_primary)
			continue;
		if (m->scope == CCX_SCOPE_COMPLEX && !complex_primary)
			continue;
		msr_unset_and_set(m->msr, m->clear, m->set);
	}
}

void sp3_ccx_bsp_init(void)
{
	ccx_apply_msr_table();
	/* This build does not install encrypted mappings; keep every thread consistent. */
	msr_unset(SYSCFG_MSR, SYSCFG_MSR_SMEE);
}

#if ENV_RAMSTAGE
static void ccx_mca_init(void)
{
	const uint64_t mcg_cap = msr_read(IA32_MCG_CAP);
	const unsigned int bank_count = mcg_cap & 0xff;

	for (unsigned int bank = 0; bank < bank_count; bank++) {
		const uint64_t config = msr_read(MCAX_CONFIG_MSR(bank));
		uint64_t value = config & ~MCA_CONFIG_MCAX_ENABLE;

		if (config & BIT(1))
			value &= ~(1ULL << 33);
		if (bank_count <= 32)
			value |= MCA_CONFIG_MCAX_ENABLE;
		if (config & MCA_CONFIG_DEFERRED_LOGGING_SUPPORTED)
			value |= MCA_CONFIG_LOG_DEFERRED_IN_MCA_STAT;
		msr_write(MCAX_CONFIG_MSR(bank), value);

	}

	/* L3 and fabric masks, once per CCD. */
	if ((initial_apicid() & (((cpuid_ebx(0x8000001e) >> 8) & 0xff) ? 0xf : 7)) == 0) {
		for (unsigned int bank = 7; bank <= 14; bank++)
			msr_set(MCA_CTL_MASK_MSR(bank), BIT(7));
		msr_set(MCA_CTL_MASK_MSR(0x19), 0x3c0);
	}
}

static uint64_t cac_weights[SMU_CAC_WEIGHT_COUNT];
static bool cac_weights_valid;

/* Runs before MP init: the APs can't talk to the SMU in parallel */
void sp3_ccx_get_cac_weights(void)
{
	cac_weights_valid = sp3_smu_cac_weights(cac_weights) == CB_SUCCESS;
	if (!cac_weights_valid)
		printk(BIOS_ERR, "Failed to read the CAC weights\n");
}

static void ccx_load_cac_weights(void)
{
	const uint64_t saved = msr_read(DPM_CFG_MSR);

	msr_write(DPM_CFG_MSR, 0);
	for (unsigned int i = 0; i < SMU_CAC_WEIGHT_COUNT; i++) {
		msr_write(DPM_WAC_INDEX_MSR, i);
		msr_write(DPM_WAC_DATA_MSR, cac_weights[i]);
	}
	msr_write(DPM_CFG_MSR, saved | DPM_CFG_WEIGHTS_VALID);
}

DECLARE_SPIN_LOCK(ccx_cpu_init_lock);

void sp3_native_cpu_init(void)
{
	const bool core_primary = primary_thread_of_core();

	/* The SMT siblings share some of the MSRs, so don't run this in parallel. */
	spin_lock(&ccx_cpu_init_lock);

	ccx_apply_msr_table();

	/* The APs start with the reset value of MMIO_CONF_BASE */
	enable_pci_mmconf();

	/* Socket-wide on Rome; the BSP initialized this write-once MSR in romstage. */
	if (!(msr_read(0x1b) & BIT(8)) && sp3_psp_bar(0))
		msr_write(PSP_ADDR_MSR, sp3_psp_bar(0));

	/* MTRRs: memory above 4 GiB is write-back */
	if (msr_read(SYSCFG_MSR) & SYSCFG_MSR_TOM2En) {
		disable_cache();
		msr_set(SYSCFG_MSR, SYSCFG_MSR_TOM2WB);
		enable_cache();
	}

	ccx_mca_init();

	if (core_primary) {
		if (cac_weights_valid)
			ccx_load_cac_weights();
		msr_write(CSTATE_CONFIG_MSR, CSTATE_CONFIG_CC6);
		msr_set(BU_CFG_MSR, COMBINE_CR0_CD);
	}
	msr_write(CSTATE_BASE_ADDR_MSR, CSTATE_IO_BASE);
	msr_write(CPU_WDT_CFG_MSR, CPU_WDT_2047_1_31MS);
	/* Rewrite P0 so the new settings take effect */
	msr_write(PSTATE_0_MSR, msr_read(PSTATE_0_MSR));

	/*
	 * Memory encryption isn't set up, so leave SME disabled. With SMEE set coreboot would
	 * need to account for the encryption bit when setting up the MTRRs.
	 */
	msr_unset(SYSCFG_MSR, SYSCFG_MSR_SMEE);

	spin_unlock(&ccx_cpu_init_lock);
}
#endif
