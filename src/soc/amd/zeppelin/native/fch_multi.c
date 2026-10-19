/* SPDX-License-Identifier: GPL-2.0-only */

/*
 * The FCH of the other dies of SP4 parts, following the vendor FchMultiFchPei
 * (ffdcb817) and FchMultiFchDxe (0348) with their defaults (sp4r2 report
 * FCH_TAISHAN section 4). These FCHs have no LPC, SPI, SMBus, HPET, RTC,
 * ACPI PM or SMI setup; their xHCI, SATA and xGbE are reached through the
 * SMN space of their die.
 */

#include <amdblocks/acpimmio_map.h>
#include <arch/io.h>
#include <console/console.h>
#include <device/device.h>
#include <device/mmio.h>
#include <device/pci_def.h>
#include <device/pci_ids.h>
#include <device/pci_ops.h>
#include <soc/iomap.h>
#include <soc/soc_util.h>
#include <static.h>
#include <types.h>

#include "fch.h"
#include "topology.h"

/* The vendor code allocates 8 KiB of the die's non-PCI pool; use the 64 KiB at +1 MiB */
#define FCH_ACPIMMIO_OFFSET	(1 * MiB)

#define PM_INDEX		0xcd6
#define PM_DATA			0xcd7
#define PM_ACPIMMIO_SEL		0xd5	/* [0]: decode the base in PMxD6 */
#define PM_ACPIMMIO_BASE	0xd6	/* 16 bits: base >> 16 */
#define PM_AB_REG_BAR		0xe0
#define ABCFG_10050		0xc0010050

#define DF_IO_BASE0		0xc0
#define DF_IO_LIMIT0		0xc4

uintptr_t zeppelin_fch_acpimmio(unsigned int die)
{
	if (!die)
		return AMD_SB_ACPI_MMIO_ADDR;
	return zeppelin_nonpci_mmio_base(die) + FCH_ACPIMMIO_OFFSET;
}

/* Legacy PM I/O of whichever FCH the DF routes I/O 0-0xfff to */
static void pm_io_write(uint8_t reg, uint8_t value)
{
	outb(reg, PM_INDEX);
	outb(value, PM_DATA);
}

static uint8_t pm_io_read(uint8_t reg)
{
	outb(reg, PM_INDEX);
	return inb(PM_DATA);
}

static void abcfg_clrsetbits(uint32_t reg, uint32_t clear, uint32_t set)
{
	uint32_t value;

	outl(reg, AB_INDX);
	value = inl(AB_DATA);
	outl(0, AB_INDX);
	outl(reg, AB_INDX);
	outl((value & ~clear) | set, AB_DATA);
	outl(0, AB_INDX);
}

static void mmio_clrsetbits8(uintptr_t addr, uint8_t clear, uint8_t set)
{
	write8p(addr, (read8p(addr) & ~clear) | set);
}

static void mmio_clrsetbits32(uintptr_t addr, uint32_t clear, uint32_t set)
{
	write32p(addr, (read32p(addr) & ~clear) | set);
}

/*
 * Vendor ffdcb7de, run while die 0's DF sends I/O 0-0xfff to the die: move the
 * die's ACPI MMIO block, then IOMUX, A-Link, clock generator and legacy SD
 * through it. Nothing may print here; the console is on die 0's LPC bus.
 */
static void fch_secondary_legacy(unsigned int die, uintptr_t mmio)
{
	const uint32_t base = mmio >> 16;

	pm_io_write(PM_ACPIMMIO_BASE, base & 0xff);
	pm_io_write(PM_ACPIMMIO_BASE + 1, base >> 8);
	pm_io_write(PM_ACPIMMIO_SEL, (pm_io_read(PM_ACPIMMIO_SEL) & 0xfc) | 1);

	/* SP4: IOMUX pin 2 function 1 */
	if (zeppelin_package() == ZEPPELIN_PKG_SP4)
		write8p(mmio + ACPIMMIO_IOMUX_BANK + 2, 1);

	/* A-Link index at 0xcd8 just for this write (low PMxE0 word only) */
	pm_io_write(PM_AB_REG_BAR, AB_INDX & 0xff);
	pm_io_write(PM_AB_REG_BAR + 1, AB_INDX >> 8);
	abcfg_clrsetbits(ABCFG_10050, BIT(2), 0);
	pm_io_write(PM_AB_REG_BAR, 0);
	pm_io_write(PM_AB_REG_BAR + 1, 0);

	/* Clock generator; these differ from die 0's values */
	mmio_clrsetbits32(mmio + 0xe10, 0x3, 0x1);
	mmio_clrsetbits32(mmio + 0xe10, 0x1ff0, 0x40);
	mmio_clrsetbits32(mmio + 0xe34, 0xff800000, 0x1e000000);
	mmio_clrsetbits32(mmio + 0xe14, 0x00ffff00, 0x3d700);
	mmio_clrsetbits32(mmio + 0xe18, 0xffff0000, 0x3e0000);
	mmio_clrsetbits32(mmio + 0xe1c, 0x1e000, 0x2000);
	mmio_clrsetbits32(mmio + 0xe1c, 0x1e0000, 0);
	mmio_clrsetbits32(mmio + 0xe1c, 0x0c000000, 0x8000000);
	mmio_clrsetbits32(mmio + 0xe1c, 0x1e00, 0x400);
	mmio_clrsetbits32(mmio + 0xe40, BIT(25), BIT(25));
	mmio_clrsetbits32(mmio + 0xe40, BIT(30), BIT(30));

	/* Legacy SD controller off: PMxD3[6], PMxE8[0], AOAC device 0x19 */
	mmio_clrsetbits8(mmio + ACPIMMIO_PMIO_BANK + 0xd3, BIT(6), 0);
	mmio_clrsetbits8(mmio + ACPIMMIO_PMIO_BANK + 0xe8, BIT(0), 0);
	mmio_clrsetbits8(mmio + ACPIMMIO_AOAC_BANK + 0x40 + 2 * 0x19, BIT(3), 0);
}

/* FchMultiFchPei: the legacy part, then the SATA reset task of the die */
void zeppelin_fch_multi_reset(void)
{
	for (unsigned int die = 1; die < zeppelin_die_count(); die++) {
		const uintptr_t mmio = zeppelin_fch_acpimmio(die);
		const pci_devfn_t df0 = PCI_DEV(0, 0x18, 0);
		const uint32_t io_base = pci_s_read_config32(df0, DF_IO_BASE0);
		const uint32_t io_limit = pci_s_read_config32(df0, DF_IO_LIMIT0);

		printk(BIOS_DEBUG, "FCH %u: ACPI MMIO at 0x%lx\n", die, mmio);

		/* Route I/O 0-0xfff to the die's IOMS */
		pci_s_write_config32(df0, DF_IO_BASE0, 3);
		pci_s_write_config32(df0, DF_IO_LIMIT0, zeppelin_ioms_fabric_id(die));
		fch_secondary_legacy(die, mmio);
		pci_s_write_config32(df0, DF_IO_BASE0, io_base);
		pci_s_write_config32(df0, DF_IO_LIMIT0, io_limit);

		/* Vendor PCD 0x8d: SATA enabled on every die */
		zeppelin_fch_sata_reset(die);
	}
}

#define NBIF_USB_STRAP		0x10134600
#define NBIF_SATA_STRAP		0x10234400
#define NBIF_XGBE_STRAP(n)	(0x10234800 + 0x200 * (n))
#define  NBIF_STRAP_ENABLE	BIT(28)
#define USB_14C			0x16d8014c
#define USB_RESET_CTRL		0x16d80100
#define SATA_SMN_CTRL		0x03100000
#define XGBE_PORT_CTRL(n)	(0x1631f030 + 4 * (n))

static const struct device *die_dev(unsigned int die, const struct device *die1)
{
	return die == 1 ? die1 : NULL;
}

/* af49 (FchTaishanDxe) and the xGbE part of FchMultiFchDxe: unstrapped ports off */
void zeppelin_fch_xgbe_init(unsigned int die)
{
	for (unsigned int n = 0; n < 4; n++) {
		if (!(zeppelin_smn_read32(die, NBIF_XGBE_STRAP(n)) & NBIF_STRAP_ENABLE))
			zeppelin_smn_clrsetbits32(die, XGBE_PORT_CTRL(n), 0, BIT(0));
	}
}

/* The USB part of FchMultiFchPei: odd die pairs (2, 3, 6, 7) are held in reset */
void zeppelin_fch_multi_usb(void)
{
	for (unsigned int die = 1; die < zeppelin_die_count(); die++) {
		zeppelin_smn_clrsetbits32(die, USB_14C, 0, BIT(31));
		if (die & 2) {
			zeppelin_smn_clrsetbits32(die, NBIF_USB_STRAP, NBIF_STRAP_ENABLE, 0);
			zeppelin_smn_clrsetbits32(die, USB_RESET_CTRL, 0, 0x7);
		} else if (is_dev_enabled(die_dev(die, DEV_PTR(die1_xhci_0)))) {
			zeppelin_fch_usb_init(die);
		} else {
			zeppelin_smn_clrsetbits32(die, NBIF_USB_STRAP, NBIF_STRAP_ENABLE, 0);
		}
	}
}

/* FchMultiFchDxe: SATA and xGbE of the dies whose internal bridge 8.1 exists */
void zeppelin_fch_multi_env(void)
{
	for (unsigned int die = 1; die < zeppelin_die_count(); die++) {
		const pci_devfn_t bridge = PCI_DEV(zeppelin_die_bus(die), 8, 1);

		if (pci_s_read_config16(bridge, PCI_VENDOR_ID) != PCI_VID_AMD) {
			zeppelin_smn_clrsetbits32(die, NBIF_SATA_STRAP, NBIF_STRAP_ENABLE, 0);
			for (unsigned int n = 0; n < 4; n++)
				zeppelin_smn_clrsetbits32(die, NBIF_XGBE_STRAP(n),
							  NBIF_STRAP_ENABLE, 0);
			continue;
		}

		if (!(zeppelin_smn_read32(die, NBIF_SATA_STRAP) & NBIF_STRAP_ENABLE)) {
			/* No SATA lanes on this die: switch the controller off */
			zeppelin_smn_clrsetbits32(die, SATA_SMN_CTRL, 0, BIT(0));
			zeppelin_smn_clrsetbits32(die, SATA_SMN_CTRL, 0, BIT(11));
		} else if (!is_dev_enabled(die_dev(die, DEV_PTR(die1_sata)))) {
			zeppelin_smn_clrsetbits32(die, NBIF_SATA_STRAP, NBIF_STRAP_ENABLE, 0);
		} else {
			zeppelin_fch_sata_init(die);
		}

		zeppelin_fch_xgbe_init(die);
	}
}

/* The SATA late task of the other dies (vendor SMM SW SMI 0xb4) */
void zeppelin_fch_multi_late(void)
{
	for (unsigned int die = 1; die < zeppelin_die_count(); die++) {
		const struct device *sata = die_dev(die, DEV_PTR(die1_sata));

		if (is_dev_enabled(sata) && sata->upstream)
			zeppelin_fch_sata_late(die, PCI_BDF(sata));
	}
}
