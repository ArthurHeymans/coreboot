/* SPDX-License-Identifier: GPL-2.0-only */

#include <acpi/acpi.h>
#include <amdblocks/acpi.h>
#include <amdblocks/chip.h>
#include <amdblocks/cpu.h>
#include <device/device.h>
#include <soc/southbridge.h>

void acpi_fill_fadt(acpi_fadt_t *fadt)
{
	const struct soc_amd_common_config *cfg = soc_get_common_config();

	fadt->pm1a_evt_blk = ACPI_PM_EVT_BLK;
	fadt->pm1a_cnt_blk = ACPI_PM1_CNT_BLK;
	fadt->pm_tmr_blk = ACPI_PM_TMR_BLK;
	fadt->gpe0_blk = ACPI_GPE0_BLK;
	fadt->pm1_evt_len = 4;
	fadt->pm1_cnt_len = 2;
	fadt->pm_tmr_len = 4;
	fadt->gpe0_blk_len = 8;
	fill_fadt_extended_pm_io(fadt);
	fadt->iapc_boot_arch = cfg->fadt_boot_arch;
	fadt->flags |= ACPI_FADT_WBINVD | ACPI_FADT_C1_SUPPORTED |
		ACPI_FADT_SLEEP_BUTTON | ACPI_FADT_S4_RTC_WAKE | ACPI_FADT_32BIT_TIMER |
		ACPI_FADT_PCI_EXPRESS_WAKE | ACPI_FADT_PLATFORM_CLOCK | cfg->fadt_flags;
}

unsigned long soc_acpi_write_tables(const struct device *device, unsigned long current,
				    acpi_rsdp_t *rsdp)
{
	/* The pinned S8030 APCB selects NPS1. No synthetic per-NBIO NUMA domains. */
	return acpi_add_ivrs_table(current, rsdp);
}

static const acpi_cstate_t cstates[] = {
	{ .ctype = 1, .latency = 1, .power = 0 },
	{ .ctype = 2, .latency = 400, .power = 0 },
};

const acpi_cstate_t *get_cstate_config_data(size_t *size)
{
	*size = ARRAY_SIZE(cstates);
	return cstates;
}
