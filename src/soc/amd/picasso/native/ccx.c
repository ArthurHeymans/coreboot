/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/amd_pci_mmconf.h>
#include <amdblocks/cpu.h>
#include <amdblocks/msr_zen.h>
#include <amdblocks/psp.h>
#include <arch/cpu.h>
#include <cpu/amd/msr.h>
#include <cpu/amd/mtrr.h>
#include <cpu/x86/cache.h>
#include <cpu/x86/lapic_def.h>
#include <cpu/x86/mp.h>
#include <cpu/x86/msr.h>
#include <smp/spinlock.h>
#include <types.h>

#include "native.h"

#define HWCR_EFF_FREQ_READ_ONLY_LOCK	BIT(27)
#define CPU_WDT_CFG_MSR			0xc0010074
#define MCA_INTR_CFG_MSR		0xc0000410
#define CSTATE_CONFIG_MSR		0xc0010296
#define DPM_CFG_MSR			0xc0011074
#define DPM_WAC_INDEX_MSR		0xc0011076
#define DPM_WAC_DATA_MSR		0xc0011077
#define DPM_WAC_COUNT			21

/* MCA_CONFIG fields */
#define MCA_CONFIG_DEFERRED_LOGGING_SUPPORTED	BIT(2)
#define MCA_CONFIG_MCAX_ENABLE			(1ULL << 32)
#define MCA_CONFIG_LOG_DEFERRED_IN_MCA_STAT	(1ULL << 34)
#define MCA_CONFIG_DEFERRED_INT_TYPE_APIC	(1ULL << 37)

struct msr_update {
	uint32_t index;
	uint64_t clear;
	uint64_t set;
};

static const struct msr_update reset_msr_table[] = {
	{ LAPIC_BASE_MSR, 0, LAPIC_BASE_MSR_ENABLE },
	{ HWCR_MSR, 0, HWCR_EFF_FREQ_READ_ONLY_LOCK },
	/* Don't report IBS and FMA4 in CPUID */
	{ CPU_ID_EXT_FEATURES_MSR, (1ULL << 48) | (1ULL << 42), 0 },
	{ 0xc00110dc, 0x0000000000028000, 0 },
	/* Enable the CPU watchdog timer with a timeout of 2047 * 1.31 ms */
	{ CPU_WDT_CFG_MSR, 0, 0x9 },
	{ MCAX_CONFIG_MSR(0), (1ULL << 33),
	  MCA_CONFIG_MCAX_ENABLE | MCA_CONFIG_LOG_DEFERRED_IN_MCA_STAT |
	  MCA_CONFIG_DEFERRED_INT_TYPE_APIC },
	{ MCAX_CONFIG_MSR(1), (1ULL << 33),
	  MCA_CONFIG_MCAX_ENABLE | MCA_CONFIG_LOG_DEFERRED_IN_MCA_STAT |
	  MCA_CONFIG_DEFERRED_INT_TYPE_APIC },
	{ MCAX_CONFIG_MSR(2), (1ULL << 33),
	  MCA_CONFIG_MCAX_ENABLE | MCA_CONFIG_LOG_DEFERRED_IN_MCA_STAT |
	  MCA_CONFIG_DEFERRED_INT_TYPE_APIC },
	{ MCAX_CONFIG_MSR(3), (1ULL << 33),
	  MCA_CONFIG_MCAX_ENABLE | MCA_CONFIG_LOG_DEFERRED_IN_MCA_STAT |
	  MCA_CONFIG_DEFERRED_INT_TYPE_APIC },
	{ MCAX_CONFIG_MSR(5), (1ULL << 33),
	  MCA_CONFIG_MCAX_ENABLE | MCA_CONFIG_LOG_DEFERRED_IN_MCA_STAT |
	  MCA_CONFIG_DEFERRED_INT_TYPE_APIC },
	{ MCAX_CONFIG_MSR(6), (1ULL << 33),
	  MCA_CONFIG_MCAX_ENABLE | MCA_CONFIG_LOG_DEFERRED_IN_MCA_STAT |
	  MCA_CONFIG_DEFERRED_INT_TYPE_APIC },
	/* Mask the IF SystemReadDataError, the FP and L2 HWA and an L3 error */
	{ MCA_CTL_MASK_MSR(1), 0, BIT(13) },
	{ MCA_CTL_MASK_MSR(6), 0, BIT(6) },
	{ MCA_CTL_MASK_MSR(2), 0, BIT(3) },
	{ MCA_CTL_MASK_MSR(7), 0, BIT(7) },
	{ LS_CFG_MSR, 0, 0x0000000000000100 },
	{ BU_CFG_MSR, 0, 0x0000200000000000 },
	{ FP_CFG_MSR, 0, 0x0000000000000014 },
	{ DE_CFG_MSR, 0, 0x0000000000026000 },
	{ BU_CFG2_MSR, 0, 0x0280000000008080 },
	{ EX_CFG_MSR, 0, 0x0168000000000000 },
	{ BU_CFG3_MSR, 0, 0x0000000000080000 },
	{ LS_CFG2_MSR, 0, 0x1000000100000000 },
	{ 0xc001102e, 0, 0x0000000000000001 },
	{ 0xc0011093, 0x0000000000058d8c, 0x0000000000480003 },
	{ 0xc0011097, 0x0000000000000008, 0x0000000000000c54 },
};

static void ccx_reset_msr_init(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(reset_msr_table); i++) {
		const struct msr_update *u = &reset_msr_table[i];

		msr_unset_and_set(u->index, u->clear, u->set);
	}

	msr_unset(DE_CFG_MSR, BIT(14));
	msr_unset(LS_CFG_MSR, BIT(28));
}

void picasso_ccx_bsp_init(void)
{
	ccx_reset_msr_init();

	/*
	 * Memory encryption isn't set up, so leave SME disabled. With SMEE set coreboot would
	 * need to account for the encryption bit when setting up the MTRRs.
	 */
	msr_unset(SYSCFG_MSR, SYSCFG_MSR_SMEE);
}

void picasso_ccx_mca_init(void)
{
	const uint64_t mcg_cap = msr_read(IA32_MCG_CAP);
	const unsigned int bank_count = mcg_cap & 0xff;

	for (unsigned int bank = 0; bank < bank_count; bank++) {
		const uint64_t config = msr_read(MCAX_CONFIG_MSR(bank));
		uint64_t value = config & ~MCA_CONFIG_MCAX_ENABLE;

		if (config & BIT(1))
			value &= ~(MCA_CONFIG_MCAX_ENABLE | (1ULL << 33));
		if (bank_count < 33)
			value |= MCA_CONFIG_MCAX_ENABLE;
		if (config & MCA_CONFIG_DEFERRED_LOGGING_SUPPORTED)
			value |= MCA_CONFIG_LOG_DEFERRED_IN_MCA_STAT;
		msr_write(MCAX_CONFIG_MSR(bank), value);

		/* Bank 15 doesn't have a threshold counter */
		if (bank != 15)
			msr_unset(MCAX_MISC0_MSR(bank), 0xff000000);
	}

	msr_unset(MCAX_MISC1_MSR(15), 0xff000000);
	msr_unset(MCAX_MISC1_MSR(16), 0xff000000);
	msr_set(MCA_INTR_CFG_MSR, BIT(3));
	msr_set(MCA_CTL_MASK_MSR(0), BIT(9) | BIT(10));	/* LS SystemReadDataErrorT0/T1 */
	msr_set(MCA_CTL_MASK_MSR(6), BIT(6));
	if (!((mcg_cap >> 4) & 0xf)) {
		for (unsigned int bank = 8; bank <= 15; bank++)
			msr_set(MCA_CTL_MASK_MSR(bank), BIT(7));
	}
}

#if ENV_RAMSTAGE
DECLARE_SPIN_LOCK(ccx_cpu_init_lock);

void picasso_native_cpu_init(void)
{
	uint64_t psp_base;
	uint64_t value;

	/* The SMT siblings share some of the MSRs, so don't run this in parallel. */
	spin_lock(&ccx_cpu_init_lock);

	if (cpu_index() != 0)
		ccx_reset_msr_init();

	/* The APs start with the reset value of MMIO_CONF_BASE */
	enable_pci_mmconf();

	/* RDRAND needs the PSP MMIO base in PSP_ADDR_MSR */
	psp_base = get_psp_mmio_base();
	if (psp_base)
		msr_write(PSP_ADDR_MSR, psp_base);

	/* The MTRRs were set up with TOM2 as write-back */
	disable_cache();
	msr_set(SYSCFG_MSR, SYSCFG_MSR_TOM2En | SYSCFG_MSR_TOM2WB);
	enable_cache();

	picasso_ccx_mca_init();

	/* Clear the weights on the primary thread of each core */
	if ((cpuid_ebx(1) >> 24) % get_threads_per_core() == 0) {
		value = msr_read(DPM_CFG_MSR);
		msr_write(DPM_CFG_MSR, 0);
		for (unsigned int index = 0; index < DPM_WAC_COUNT; index++) {
			msr_write(DPM_WAC_INDEX_MSR, index);
			msr_write(DPM_WAC_DATA_MSR, 0);
		}
		msr_write(DPM_CFG_MSR, value | 0x9000000000000000ULL);
	}

	msr_set(DE_CFG_MSR, BIT(3));
	msr_set(LS_CFG_MSR, BIT(13) | BIT(15));
	msr_set(BU_CFG_MSR, COMBINE_CR0_CD);

	/* Enable CC6 */
	msr_write(CSTATE_CONFIG_MSR, 0x0000000000484848ULL);
	msr_write(PSTATE_0_MSR, msr_read(PSTATE_0_MSR));

	msr_unset_and_set(BU_CFG3_MSR, BIT(2), BIT(3));
	msr_set(DC_CFG_MSR, BIT(13) | BIT(15));

	spin_unlock(&ccx_cpu_init_lock);
}
#endif
