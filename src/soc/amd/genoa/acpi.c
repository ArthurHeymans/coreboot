/* SPDX-License-Identifier: GPL-2.0-only */

/* ACPI - create the Fixed ACPI Description Tables (FADT) */

#include <cpu/x86/smm.h>
#include <acpi/acpi.h>
#include <vendorcode/amd/opensil/opensil.h>
#include <soc/acpi.h>

unsigned long acpi_fill_madt(unsigned long current)
{
	// TODO
	return current;
}

void acpi_fill_fadt(acpi_fadt_t *fadt)
{
	fadt->sci_int = 9;		/* IRQ 09 - ACPI SCI */

	/* Fill in pm1_evt, pm1_cnt, pm_tmr, gpe0_blk from opensil input structure */
	opensil_fill_fadt(fadt);

	if (permanent_smi_handler()) {
		fadt->smi_cmd = APM_CNT;
		fadt->acpi_enable = APM_CNT_ACPI_ENABLE;
		fadt->acpi_disable = APM_CNT_ACPI_DISABLE;
	}

	fadt->pm1_evt_len = 4;	/* 32 bits */
	fadt->pm1_cnt_len = 2;	/* 16 bits */
	fadt->pm_tmr_len = 4;	/* 32 bits */
	fadt->gpe0_blk_len = 8;	/* 64 bits */

	fadt->p_lvl2_lat = ACPI_FADT_C2_NOT_SUPPORTED;
	fadt->p_lvl3_lat = ACPI_FADT_C3_NOT_SUPPORTED;
	fadt->duty_offset = 1;	/* CLK_VAL bits 3:1 */
	fadt->duty_width = 3;	/* CLK_VAL bits 3:1 */
	fadt->day_alrm = RTC_DATE_ALARM;
	fadt->mon_alrm = 0;
	fadt->iapc_boot_arch = ACPI_FADT_LEGACY_FREE; /* legacy free default */
	fadt->res2 = 0;		/* reserved, MUST be 0 ACPI 3.0 */
	fadt->flags |=	ACPI_FADT_WBINVD | /* See table 5-34 ACPI 6.3 spec */
			ACPI_FADT_C1_SUPPORTED |
			ACPI_FADT_S4_RTC_WAKE |
			ACPI_FADT_32BIT_TIMER |
			ACPI_FADT_PCI_EXPRESS_WAKE |
			ACPI_FADT_PLATFORM_CLOCK |
			ACPI_FADT_S4_RTC_VALID |
			ACPI_FADT_REMOTE_POWER_ON;
	// fadt->flags |= cfg->fadt_flags; /* additional board-specific flags */

	fadt->ARM_boot_arch = 0;	/* Must be zero if ACPI Revision <= 5.0 */

	fadt->x_firmware_ctl_l = 0;	/* set to 0 if firmware_ctrl is used */
	fadt->x_firmware_ctl_h = 0;

	fadt->x_pm1a_evt_blk.space_id = ACPI_ADDRESS_SPACE_IO;
	fadt->x_pm1a_evt_blk.bit_width = 32;
	fadt->x_pm1a_evt_blk.bit_offset = 0;
	fadt->x_pm1a_evt_blk.access_size = ACPI_ACCESS_SIZE_WORD_ACCESS;
	fadt->x_pm1a_evt_blk.addrh = 0x0;

	fadt->x_pm1a_cnt_blk.space_id = ACPI_ADDRESS_SPACE_IO;
	fadt->x_pm1a_cnt_blk.bit_width = 16;
	fadt->x_pm1a_cnt_blk.bit_offset = 0;
	fadt->x_pm1a_cnt_blk.access_size = ACPI_ACCESS_SIZE_WORD_ACCESS;
	fadt->x_pm1a_cnt_blk.addrh = 0x0;

	fadt->x_pm_tmr_blk.space_id = ACPI_ADDRESS_SPACE_IO;
	fadt->x_pm_tmr_blk.bit_width = 32;
	fadt->x_pm_tmr_blk.bit_offset = 0;
	fadt->x_pm_tmr_blk.access_size = ACPI_ACCESS_SIZE_DWORD_ACCESS;
	fadt->x_pm_tmr_blk.addrh = 0x0;

	fadt->x_gpe0_blk.space_id = ACPI_ADDRESS_SPACE_IO;
	fadt->x_gpe0_blk.bit_width = 64; /* EventStatus + Event Enable */
	fadt->x_gpe0_blk.bit_offset = 0;
	fadt->x_gpe0_blk.access_size = ACPI_ACCESS_SIZE_BYTE_ACCESS;
	fadt->x_gpe0_blk.addrh = 0x0;
}

void generate_cpu_entries(const struct device *device)
{
	// TODO
	return;
}
