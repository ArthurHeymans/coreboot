/* SPDX-License-Identifier: GPL-2.0-only */

/* ACPI - create the Fixed ACPI Description Tables (FADT) */

#include <acpi/acpi.h>
#include <acpi/acpigen.h>
#include <amdblocks/acpi.h>
#include <amdblocks/acpimmio.h>
#include <amdblocks/cppc.h>
#include <amdblocks/cpu.h>
#include <amdblocks/ioapic.h>
#include <amdblocks/smu.h>
#include <arch/ioapic.h>
#include <arch/smp/mpspec.h>
#include <cbmem.h>
#include <console/console.h>
#include <cpu/amd/msr.h>
#include <cpu/x86/smm.h>
#include <soc/acpi.h>
#include <soc/iomap.h>
#include <soc/msr.h>
#include <vendorcode/amd/opensil/opensil.h>

unsigned long acpi_fill_madt(unsigned long current)
{
	/* TODO: IOHUBs IOAPIC */

	return current;
}

void acpi_fill_fadt(acpi_fadt_t *fadt)
{
	/* Fill in pm1_evt, pm1_cnt, pm_tmr, gpe0_blk from opensil input structure */
	opensil_fill_fadt(fadt);

	fadt->pm1_evt_len = 4;	/* 32 bits */
	fadt->pm1_cnt_len = 2;	/* 16 bits */
	fadt->pm_tmr_len = 4;	/* 32 bits */
	fadt->gpe0_blk_len = 8;	/* 64 bits */

	fadt->iapc_boot_arch = ACPI_FADT_LEGACY_FREE; /* legacy free default */
	fadt->flags |=	ACPI_FADT_WBINVD | /* See table 5-34 ACPI 6.3 spec */
			ACPI_FADT_C1_SUPPORTED |
			ACPI_FADT_S4_RTC_WAKE |
			ACPI_FADT_32BIT_TIMER |
			ACPI_FADT_PCI_EXPRESS_WAKE |
			ACPI_FADT_PLATFORM_CLOCK |
			ACPI_FADT_S4_RTC_VALID |
			ACPI_FADT_REMOTE_POWER_ON;


	fadt->x_firmware_ctl_l = 0;	/* set to 0 if firmware_ctrl is used */
	fadt->x_firmware_ctl_h = 0;

	fill_fadt_extended_pm_regs(fadt);
}

static u64 hest_get_elog_addr(void)
{
	/* Reserve memory for Enhanced error logging */
	void *mem = cbmem_add(CBMEM_ID_ACPI_HEST, CONFIG_ERROR_LOG_BUFFER_SIZE);
	if (!mem) {
		printk(BIOS_ERR, "Unable to allocate HEST memory\n");
		return 0;
	}

	printk(BIOS_DEBUG, "HEST memory created: %p\n", mem);
	printk(BIOS_DEBUG, "elog_addr: %p, size:%x\n", mem,
		CONFIG_ERROR_LOG_BUFFER_SIZE);
	return *(unsigned long long int *)mem;
}

static u32 acpi_hest_add_ghes(void *current)
{
	ghes_record_t *rec = (ghes_record_t *)current;
	u32 size = sizeof(ghes_record_t);

	/* Fill GHES error source descriptor  */
	memset(rec, 0, size);
	rec->esd.type = HEST_GHES_DESC_TYPE;
	rec->esd.source_id = 0; /* 0 for MCE check exception source */
	rec->esd.enabled = 1;
	rec->esd.related_src_id = 0xffff;
	rec->esd.prealloc_erecords = 1;
	rec->esd.max_section_per_record = 0xf;
	rec->max_raw_data_length = GHES_MAX_RAW_DATA_LENGTH;

	/* Add error_status_address */
	rec->sts_addr.space_id = 0;
	rec->sts_addr.bit_width = 0x40;
	rec->sts_addr.bit_offset = 0;
	rec->sts_addr.access_size = QWORD_ACCESS;

	/* Add notification structure */
	rec->notify.type = NOTIFY_TYPE_SCI;
	rec->notify.length = sizeof(acpi_hest_hen_t);
	rec->err_sts_blk_len = GHEST_ERROR_STATUS_BLOCK_LENGTH;

	/* error status block entries start address */
	if (CONFIG(SOC_ACPI_HEST)) {
		rec->sts_addr.addr = hest_get_elog_addr();
	}

	return size;
}

static unsigned long acpi_fill_hest(acpi_hest_t *hest)
{
	acpi_header_t *header = &(hest->header);
	void *current;
	current = (void *)(hest);
	void *next = current;
	next = hest + 1;
	next += acpi_hest_add_ghes(next);
	hest->error_source_count += 1;
	header->length = next - current;
	return header->length;
}

unsigned long hest_create(const struct device *device, unsigned long current, struct acpi_rsdp *rsdp)
{
	acpi_hest_t *hest;
	current = ALIGN_UP(current, 8);
	hest = (acpi_hest_t *)current;
	acpi_write_hest(hest, acpi_fill_hest);
	acpi_add_table(rsdp, (void *)current);
	current += hest->header.length;
	return current;
}

const acpi_cstate_t cstate_cfg_table[] = {
	[0] = {
		.ctype = 1,
		.latency = 1,
		.power = 0,
	},
	[1] = {
		.ctype = 2,
		.latency = 0x12,
		.power = 0,
	},
};

const acpi_cstate_t *get_cstate_config_data(size_t *size)
{
	*size = ARRAY_SIZE(cstate_cfg_table);
	return cstate_cfg_table;
}
