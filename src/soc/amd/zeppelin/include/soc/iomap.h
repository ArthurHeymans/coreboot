/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_ZEPPELIN_IOMAP_H
#define AMD_ZEPPELIN_IOMAP_H

#if ENV_X86

/* MMIO Ranges */
/* IO_APIC_ADDR defined in arch/x86	0xfec00000 */
#define SPI_BASE_ADDRESS		0xfec10000
/*
 * The non-PCI MMIO (PSP BAR, NBIO IOAPIC) of every die is in the top 8 MiB of
 * the die's below-4G Data Fabric window, as in the vendor firmware; on die 0
 * that's 0xfe400000-0xfebfffff (native/data_fabric.c).
 */

/* FCH A-Link to AHB bridge and the AHB devices behind it */
#define ALINK_AHB_ADDRESS		0xfedc0000

#endif /* ENV_X86 */

#define FLASH_BASE_ADDR			((0xffffffff - CONFIG_ROM_SIZE) + 1)

/* I/O Ranges */
/* The vendor firmware uses the same layout at 0x800 */
#define ACPI_IO_BASE			0x800
#define  ACPI_PM_EVT_BLK	(ACPI_IO_BASE + 0x00)     /* 4 bytes */
#define  ACPI_PM1_STS		(ACPI_PM_EVT_BLK + 0x00)	  /* 2 bytes */
#define  ACPI_PM1_EN		(ACPI_PM_EVT_BLK + 0x02)	  /* 2 bytes */
#define  ACPI_PM1_CNT_BLK	(ACPI_IO_BASE + 0x04)     /* 2 bytes */
#define  ACPI_PM_TMR_BLK	(ACPI_IO_BASE + 0x08)     /* 4 bytes */
#define  ACPI_CSTATE_CONTROL	(ACPI_IO_BASE + 0x10)     /* 8 bytes */
/* doc says 0x14 for GPE0_BLK but 8 bytes from ACPI_CSTATE_CONTROL on are trapped in CPU core */
#define  ACPI_GPE0_BLK		(ACPI_IO_BASE + 0x20)     /* 8 bytes */
#define  ACPI_GPE0_STS		(ACPI_GPE0_BLK + 0x00)		  /* 4 bytes */
#define  ACPI_GPE0_EN		(ACPI_GPE0_BLK + 0x04)		  /* 4 bytes */
#define SMB_BASE_ADDR			0xb00
#define PM2_INDEX			0xcd0
#define PM2_DATA			0xcd1
#define BIOSRAM_INDEX			0xcd4
#define BIOSRAM_DATA			0xcd5
#define AB_INDX				0xcd8
#define AB_DATA				(AB_INDX + 4)

#endif /* AMD_ZEPPELIN_IOMAP_H */
