/* SPDX-License-Identifier: GPL-2.0-only */

/*
 * The SATA controller of a die (behind 8.1 of the die's root bus, function 2).
 * The vendor FchTaishan code programs it through its SMN aperture at
 * 0x03100000, with the AHCI registers at 0x03101000 (sp4r2 report
 * FCH_TAISHAN, tasks af47, af57 and af77, with the vendor policy defaults).
 */

#include <amdblocks/acpimmio.h>
#include <amdblocks/amd_pci_util.h>
#include <console/console.h>
#include <delay.h>
#include <device/pci_ops.h>
#include <timer.h>
#include <types.h>

#include "fch.h"
#include "fch_regs.h"
#include "topology.h"

#define SATA_PORT_COUNT		8

#define SATA_SMN_CTRL		SATA_SMN(0x0)
#define SATA_SMN_MSI		SATA_SMN(0x8)
#define AHCI_CAP		SATA_SMN(0x1000)
#define  AHCI_CAP_NP_MASK	0x1f
#define AHCI_PI			SATA_SMN(0x100c)
#define SATA_SMN_1024		SATA_SMN(0x1024)
#define SATA_SMN_10F4		SATA_SMN(0x10f4)	/* SGPIO */
#define SATA_SMN_10F8		SATA_SMN(0x10f8)
#define SATA_SMN_10FC		SATA_SMN(0x10fc)
#define SATA_SMN_MISC		SATA_SMN(0x1800)
#define  SATA_MISC_UNLOCK	BIT(0)
#define  SATA_MISC_BIT13	BIT(13)
#define  SATA_MISC_PORT_DIS(n)	BIT(16 + (n))
#define SATA_SMN_180C		SATA_SMN(0x180c)
#define SATA_SMN_1810		SATA_SMN(0x1810)
#define SATA_SMN_1814		SATA_SMN(0x1814)
#define SATA_SMN_PORT_CFG(n)	SATA_SMN(0x188c + (n) * 0x20)
#define AHCI_PORT(n, reg)	SATA_SMN(0x1100 + (n) * 0x80 + (reg))
#define  PX_IS			0x10
#define  PX_SSTS		0x28
#define  PX_SCTL		0x2c
#define  PX_SERR		0x30
#define  SSTS_DET_MASK		0xf
#define  SSTS_DET_PHY_OFFLINE	4
#define  SSTS_DET_PRESENT	3

/* NBIF straps of the SATA function */
#define NBIF_SATA_STRAP		0x10234400	/* [15:0] device ID, bit 28 enable */
#define NBIF_SATA_STRAP_0C	0x1023440c
#define NBIF_SATA_CLASS		0x10234434
#define NBIF_SATA_SSID		0x1024204c

#define SATA_AHCI_DEVICE_ID	0x7901
#define SATA_AHCI_CLASS		0x010601
#define SATA_SSID		0x10001458	/* Gigabyte, vendor PCD 0x15f */

#define PIRQ_IDX_MISC		0x08

/* Reset task: take the PHYs offline */
void zeppelin_fch_sata_reset(unsigned int die)
{
	for (unsigned int bit = 0; bit < 8; bit++)
		zeppelin_smn_clrsetbits32(die, SATA_SMN_RESET_STATUS, BIT(bit), 0);

	zeppelin_smn_clrsetbits32(die, SATA_SMN_CTRL, BIT(0), 0);
	for (unsigned int n = 0; n < SATA_PORT_COUNT; n++) {
		struct stopwatch sw;

		zeppelin_smn_clrsetbits32(die, AHCI_PORT(n, PX_SCTL), 0xf,
					  SSTS_DET_PHY_OFFLINE);
		/* The vendor code continues silently after 1 ms */
		stopwatch_init_usecs_expire(&sw, 1000);
		while ((zeppelin_smn_read32(die, AHCI_PORT(n, PX_SSTS)) & SSTS_DET_MASK) !=
		       SSTS_DET_PHY_OFFLINE && !stopwatch_expired(&sw))
			udelay(10);
	}
}

/* Env task: enable the ports and set up the controller as AHCI */
void zeppelin_fch_sata_init(unsigned int die)
{
	zeppelin_smn_clrsetbits32(die, SATA_SMN_MISC, SATA_MISC_UNLOCK, SATA_MISC_UNLOCK);

	/* Enable each port with a COMRESET */
	for (unsigned int n = 0; n < SATA_PORT_COUNT; n++) {
		zeppelin_smn_clrsetbits32(die, SATA_SMN_MISC, SATA_MISC_PORT_DIS(n), 0);
		zeppelin_smn_clrsetbits32(die, AHCI_PORT(n, PX_SCTL), 0xf, 1);
		mdelay(1);
		zeppelin_smn_clrsetbits32(die, AHCI_PORT(n, PX_SCTL), 0xf, 0);
	}

	zeppelin_smn_clrsetbits32(die, SATA_SMN_CTRL, BIT(1) | BIT(2) | BIT(3) | BIT(24),
				  BIT(2));
	zeppelin_smn_clrsetbits32(die, SATA_SMN_10FC, 0x08080000, 0x04000c02);
	zeppelin_smn_clrsetbits32(die, SATA_SMN_10F8, 0xff000000, 0xff000000);
	zeppelin_smn_clrsetbits32(die, SATA_SMN_1024, BIT(0), 0);
	zeppelin_smn_clrsetbits32(die, SATA_SMN_180C, 0x00800000, 0xf000006a);
	zeppelin_smn_clrsetbits32(die, SATA_SMN_1810, 0x0200c304, 0x0200c304);
	zeppelin_smn_clrsetbits32(die, SATA_SMN_1814, 0x400001e0, 0x400001e0);
	zeppelin_smn_clrsetbits32(die, SATA_SMN_MISC, SATA_MISC_BIT13, SATA_MISC_BIT13);
	zeppelin_smn_clrsetbits32(die, NBIF_SATA_STRAP_0C, BIT(18), BIT(18));
	/* No disabled ports */
	zeppelin_smn_clrsetbits32(die, SATA_SMN_MISC, 0x00ff0000, 0);

	/* No eSATA ports, no SGPIO */
	zeppelin_smn_clrsetbits32(die, SATA_SMN_10F8, 0x00ff00ff, 0);
	zeppelin_smn_clrsetbits32(die, SATA_SMN_10F8, BIT(20), 0);
	zeppelin_smn_clrsetbits32(die, SATA_SMN_10F4, BIT(4) | BIT(8) | BIT(9), 0);

	for (unsigned int n = 0; n < SATA_PORT_COUNT; n++) {
		zeppelin_smn_clrsetbits32(die, SATA_SMN_PORT_CFG(n), 0x700, n << 8);
		zeppelin_smn_write32(die, AHCI_PORT(n, PX_SERR), 0xffffffff);
		zeppelin_smn_write32(die, AHCI_PORT(n, PX_IS), 0);
	}

	zeppelin_smn_write32(die, NBIF_SATA_CLASS, SATA_AHCI_CLASS);
	zeppelin_smn_clrsetbits32(die, NBIF_SATA_STRAP, 0xffff, SATA_AHCI_DEVICE_ID);
	zeppelin_smn_write32(die, NBIF_SATA_SSID, SATA_SSID);

	/* The PCI IRQ index registers belong to the FCH of die 0 */
	if (die == 0)
		write_pci_int_idx(PIRQ_IDX_MISC, 0,
				  (read_pci_int_idx(PIRQ_IDX_MISC, 0) & 0xf) | 0xf0);

	zeppelin_smn_clrsetbits32(die, SATA_SMN_MISC, SATA_MISC_UNLOCK, 0);
}

/*
 * Late task (ReadyToBoot in the vendor firmware): disable the ports without a
 * device and report only the remaining ones, and copy the MSI capability of
 * the SATA function into the controller.
 */
void zeppelin_fch_sata_late(unsigned int die, pci_devfn_t sata)
{
	uint32_t pi = zeppelin_smn_read32(die, AHCI_PI);

	zeppelin_smn_clrsetbits32(die, SATA_SMN_MISC, SATA_MISC_UNLOCK, SATA_MISC_UNLOCK);
	for (unsigned int n = 0; n < SATA_PORT_COUNT; n++) {
		const uint32_t ssts = zeppelin_smn_read32(die, AHCI_PORT(n, PX_SSTS));

		if ((ssts & SSTS_DET_MASK) == SSTS_DET_PRESENT)
			continue;
		zeppelin_smn_clrsetbits32(die, SATA_SMN_MISC, 0, SATA_MISC_PORT_DIS(n));
		pi &= ~BIT(n);
	}
	if (!pi)
		pi = 1;
	zeppelin_smn_write32(die, AHCI_PI, pi);
	zeppelin_smn_clrsetbits32(die, AHCI_CAP, AHCI_CAP_NP_MASK, __builtin_popcount(pi) - 1);

	/* Vendor PCD 0x167: MSI capability multiple message capable */
	zeppelin_smn_clrsetbits32(die, SATA_SMN_MSI, 0x07000000,
			 ((pci_s_read_config32(sata, 0xa0) >> 17) & 7) << 24);
	zeppelin_smn_clrsetbits32(die, SATA_SMN_MISC, SATA_MISC_UNLOCK, 0);

	printk(BIOS_DEBUG, "SATA %u: ports implemented 0x%02x\n", die, pi);
}
