/* SPDX-License-Identifier: GPL-2.0-only */

#include <acpi/acpi.h>
#include <amdblocks/acpimmio.h>
#include <amdblocks/acpimmio_map.h>
#include <amdblocks/smm.h>
#include <arch/io.h>
#include <arch/ioapic.h>
#include <device/mmio.h>
#include <soc/iomap.h>
#include <types.h>

#define PM_SLEEP_CTRL_EE	0xee
#define AOAC_REG_54		(ACPIMMIO_BASE(AOAC) + 0x94)	/* 0xfed81e94 */

#define IOAPIC_INDEX		0x00
#define IOAPIC_DATA		0x10
#define IOAPIC_PIN_ASSERT	0x20
#define  IOAPIC_RTE_MASKED	BIT(16)

/*
 * The Sx entry callbacks of the vendor FchTaishanSmmInit. PMxEE[1:0] = 3 at
 * S3/S4 entry; at S3, S4 and S5 entry bits 15:14 of 0xfed81e94 (cleared by the
 * reset tasks) get set again, 0x17 goes to the FCH IOAPIC register at 0x20 and
 * the low halves of all 24 redirection entries become 0x10000 (masked). The
 * 0xed reads are the vendor delays. The meaning of 0xfed81e94 and of the 0x17
 * write isn't known. The S3 IRQ table save and USB handshake aren't needed
 * without S3.
 */
void soc_smi_sleep_entry(uint8_t slp_typ)
{
	if (slp_typ == ACPI_S3 || slp_typ == ACPI_S4)
		pm_write8(PM_SLEEP_CTRL_EE, (pm_read8(PM_SLEEP_CTRL_EE) & ~0x3) | 0x3);

	write32p(AOAC_REG_54, read32p(AOAC_REG_54) | 0xc000);
	inb(0xed);
	write32p(IO_APIC_ADDR + IOAPIC_PIN_ASSERT, 0x17);
	inb(0xed);
	for (uint32_t index = 0x10; index < 0x40; index += 2) {
		write32p(IO_APIC_ADDR + IOAPIC_INDEX, index);
		write32p(IO_APIC_ADDR + IOAPIC_DATA, IOAPIC_RTE_MASKED);
	}
}
