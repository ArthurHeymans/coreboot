/* SPDX-License-Identifier: GPL-2.0-only */

#include <stdint.h>
#include <commonlib/helpers.h>
#include <delay.h>
#include <device/pci_def.h>
#include <device/pci_ops.h>
#if CONFIG(SOUTHBRIDGE_INTEL_I82801GX)
#include <southbridge/intel/i82801gx/i82801gx.h> /* DEFAULT_PMBASE */
#else
#include <southbridge/intel/i82801jx/i82801jx.h> /* DEFAULT_PMBASE */
#endif
#include <option.h>
#include "x4x.h"
#include <console/console.h>

#define X4X_POLL_TIMEOUT_US 10000

void x4x_early_init(void)
{
	/* Setup MCHBAR. */
	pci_write_config32(HOST_BRIDGE, D0F0_MCHBAR_LO, CONFIG_FIXED_MCHBAR_MMIO_BASE | 1);

	/* Setup DMIBAR. */
	pci_write_config32(HOST_BRIDGE, D0F0_DMIBAR_LO, CONFIG_FIXED_DMIBAR_MMIO_BASE | 1);

	/* Setup EPBAR. */
	pci_write_config32(HOST_BRIDGE, D0F0_EPBAR_LO, CONFIG_FIXED_EPBAR_MMIO_BASE | 1);

	/* Setup HECIBAR */
	pci_write_config32(PCI_DEV(0, 3, 0), 0x10, DEFAULT_HECIBAR);

	/* Set C0000-FFFFF to access RAM on both reads and writes */
	pci_write_config8(HOST_BRIDGE, D0F0_PAM(0), 0x30);
	pci_write_config8(HOST_BRIDGE, D0F0_PAM(1), 0x33);
	pci_write_config8(HOST_BRIDGE, D0F0_PAM(2), 0x33);
	pci_write_config8(HOST_BRIDGE, D0F0_PAM(3), 0x33);
	pci_write_config8(HOST_BRIDGE, D0F0_PAM(4), 0x33);
	pci_write_config8(HOST_BRIDGE, D0F0_PAM(5), 0x33);
	pci_write_config8(HOST_BRIDGE, D0F0_PAM(6), 0x33);

	if (!(pci_read_config32(HOST_BRIDGE, D0F0_CAPID0 + 4) & (1 << (46 - 32)))) {
		/* Enable internal GFX */
		pci_write_config32(HOST_BRIDGE, D0F0_DEVEN, BOARD_DEVEN);

		/* Set preallocated IGD size from CMOS, or default to 64 MiB */
		u8 gfxsize = get_uint_option("gfx_uma_size", 6);
		if (gfxsize > 12)
			gfxsize = 6;
		/* Need at least 4M for cbmem_top alignment */
		else if (gfxsize < 1)
			gfxsize = 1;
		/* Set GTT size to 2+2M */
		pci_write_config16(HOST_BRIDGE, D0F0_GGC, 0x0b00 | (gfxsize + 1) << 4);
	} else { /* Does not feature internal graphics */
		pci_write_config32(HOST_BRIDGE, D0F0_DEVEN, D0EN | D1EN | PEG1EN);
		pci_write_config16(HOST_BRIDGE, D0F0_GGC, (1 << 1));
	}
}

#define PEG_DEV PCI_DEV(0, 1, 0)
#define PEG_LINK_RETRAIN_TIMEOUT_US 100000

/*
 * PEG link training based on vendor BIOS DMI/PEG PEIM.
 *
 * The vendor does explicit PEG link training with retry logic,
 * link speed negotiation (Gen1/Gen2), and PHY/AFE tuning.
 * Without this, coreboot relies on PCIe auto-enumeration and
 * silicon defaults.
 */

/* PEG PCIe capability offsets (capability base at D1:F0+0xa0) */
#define PEG_PCIE_CAP	0xa0
#define PEG_LCAP	(PEG_PCIE_CAP + PCI_EXP_LNKCAP)
#define PEG_LCTL	(PEG_PCIE_CAP + PCI_EXP_LNKCTL)
#define PEG_LSTS	(PEG_PCIE_CAP + PCI_EXP_LNKSTA)
#define PEG_LCTL2	(PEG_PCIE_CAP + 0x30)

static void peg_link_speed_config(u8 target_speed)
{
	u32 reg32;

	/*
	 * Vendor sequence: snapshot AFECFG, toggle bit 9 (speed change
	 * trigger), reconfigure speed/de-emphasis, then write back with
	 * bit 9 clear. The 16-bit write sets bit 9 in hardware briefly;
	 * the final 32-bit write completes the sequence with it cleared.
	 */
	reg32 = mchbar_read32(PEG_AFECFG);

	/* Set bit 9 to initiate speed change */
	mchbar_write16(PEG_AFECFG, (reg32 & 0xffff) | (1 << 9));

	/* Configure speed and de-emphasis settings */
	reg32 &= ~0x30;	/* clear bits [5:4] */
	reg32 |= 0x20;		/* set bit 5 (Gen2 de-emphasis) */

	if (target_speed >= 2) {
		/* Gen2 */
		reg32 = (reg32 & ~0x0b) | 0x05;
		reg32 = (reg32 & ~0x000f0000) | 0x00040000;
	} else {
		/* Gen1 */
		reg32 = (reg32 & ~0x0c) | 0x04;
		reg32 = (reg32 & ~0x000f0000) | 0x00030000;
	}

	/* Write back with bit 9 clear to complete speed change */
	reg32 &= ~(1 << 9);
	mchbar_write32(PEG_AFECFG, reg32);
}

static void peg_link_setup(void)
{
	/* Set PEG AFE clock distribution */
	mchbar_write32(PEG_AFE_CLK0, 0xbd000000);
	mchbar_write32(PEG_AFE_CLK1, 0x000000bd);
}

static int peg_link_wait(int timeout_us)
{
	/* Wait for link training to complete (bit 11 = 1 while training) */
	while (timeout_us-- > 0) {
		if (!(pci_read_config16(PEG_DEV, PEG_LSTS) & PCI_EXP_LNKSTA_LT))
			return 0; /* Link training complete */
		udelay(1);
	}
	return -1; /* Timeout */
}

static void init_peg(void)
{
	u32 deven;
	u16 lsts, lcap;
	u8 max_speed, current_speed, current_width;

	/* Check if PEG port is enabled */
	deven = pci_read_config32(HOST_BRIDGE, D0F0_DEVEN);
	if (!(deven & D1EN))
		return;

	/* MCHBAR PEG controller init: set bits [17:16] */
	mchbar_setbits32(0x4030, 3 << 16);

	printk(BIOS_DEBUG, "PEG: starting link training\n");

	/* Read link capabilities */
	lcap = pci_read_config16(PEG_DEV, PEG_LCAP);
	max_speed = lcap & PCI_EXP_LNKCAP_MLS;

	/* Set link speed target */
	if (max_speed >= 2) {
		/* Device supports Gen2 */
		pci_update_config16(PEG_DEV, PEG_LCTL2, ~0xf, 2);
	}

	/* Retrain link */
	pci_or_config16(PEG_DEV, PEG_LCTL, PCI_EXP_LNKCTL_RL);

	/* Wait for link training */
	if (peg_link_wait(PEG_LINK_RETRAIN_TIMEOUT_US) < 0) {
		printk(BIOS_WARNING, "PEG: link training timeout\n");
		/* Try Gen1 fallback */
		pci_update_config16(PEG_DEV, PEG_LCTL2, ~0xf, 1);
		pci_or_config16(PEG_DEV, PEG_LCTL, PCI_EXP_LNKCTL_RL);
		if (peg_link_wait(PEG_LINK_RETRAIN_TIMEOUT_US) < 0) {
			printk(BIOS_ERR, "PEG: link training failed\n");
			return;
		}
	}

	/* Read current link status */
	lsts = pci_read_config16(PEG_DEV, PEG_LSTS);
	current_speed = lsts & 0xf;
	current_width = (lsts >> 4) & 0x3f;
	if (!current_speed || !current_width) {
		printk(BIOS_DEBUG, "PEG: no active link\n");
		return;
	}

	printk(BIOS_DEBUG, "PEG: link up, Gen%d x%d\n", current_speed, current_width);

	/* Configure AFE for the negotiated speed */
	peg_link_speed_config(current_speed);

	/* Set up PEG clocking */
	peg_link_setup();

	/* Run SBI (sideband interface) sequence if ready */
	if (mchbar_read32(PEG_SBISTATUS) & (1 << 8)) {
		/* SBI sequence: clear bit 1, clear data, set bit 0, poll, set bit 1 */
		mchbar_clrbits32(PEG_SBIR, 0x2);
		mchbar_clrbits32(PEG_SBID, 0x888);
		mchbar_setbits32(PEG_SBIR, 1);

		/* Poll SBI completion (bit 0 clear) */
		int timeout = 10000;
		while ((mchbar_read32(PEG_SBIR) & 1) && timeout-- > 0)
			udelay(1);
		if (timeout <= 0)
			printk(BIOS_WARNING, "PEG: SBI command timeout\n");

		mchbar_clrbits32(PEG_SBID, 0x888);
		mchbar_setbits32(PEG_SBIR, 0x2);
	}

	printk(BIOS_DEBUG, "Done PEG init\n");
}

static void init_egress(void)
{
	static const u32 vc1_portarb[] = {
		0x01000001, 0x00040000, 0x00001000, 0x00000040,
		0x01000001, 0x00040000, 0x00001000, 0x00000040,
	};
	unsigned int i;
	int timeout;
	u32 reg32;

	/* VC0: TC0 only */
	epbar_write8(EPVC0RCTL, 1);
	epbar_write8(EPPVCCAP1, 1);

	switch (mchbar_read32(CLKCFG_MCHBAR) & CLKCFG_FSBCLK_MASK) {
	case 0x0:
		/* FSB 1066 */
		epbar_write32(EPVC1ITC, 0x0001a6db);
		break;
	case 0x2:
		/* FSB 800 */
		epbar_write32(EPVC1ITC, 0x00014514);
		break;
	default:
	case 0x4:
		/* FSB 1333 */
		epbar_write32(EPVC1ITC, 0x00022861);
		break;
	}
	epbar_write32(EPVC1MTS, 0x0a0a0a0a);
	epbar_clrsetbits8(EPPVCCTL, 7 << 1, 1 << 1);
	epbar_clrsetbits32(EPVC1RCAP, 0x7f << 16, 0x0a << 16);
	mchbar_setbits8(0x3c, 7);

	/* VC1: ID1, TC7 */
	reg32 = (epbar_read32(EPVC1RCTL) & ~(7 << 24)) | (1 << 24);
	reg32 = (reg32 & ~0xfe) | (1 << 7);
	epbar_write32(EPVC1RCTL, reg32);

	/* Init VC1 port arbitration table */
	for (i = 0; i < ARRAY_SIZE(vc1_portarb); i++)
		epbar_write32(EP_PORTARB(i), vc1_portarb[i]);

	/* Load table */
	reg32 = epbar_read32(EPVC1RCTL) | (1 << 16);
	epbar_write32(EPVC1RCTL, reg32);
	asm("nop");
	epbar_write32(EPVC1RCTL, reg32);

	/* Wait for table load */
	timeout = X4X_POLL_TIMEOUT_US;
	while ((epbar_read8(EPVC1RSTS) & (1 << 0)) != 0 && timeout-- > 0)
		;
	if (timeout <= 0)
		printk(BIOS_WARNING, "Egress VC1 table load timeout\n");

	/* VC1: enable */
	epbar_setbits32(EPVC1RCTL, 1 << 31);

	/* Wait for VC1 */
	timeout = X4X_POLL_TIMEOUT_US;
	while ((epbar_read8(EPVC1RSTS) & (1 << 1)) != 0 && timeout-- > 0)
		;
	if (timeout <= 0)
		printk(BIOS_WARNING, "Egress VC1 negotiation timeout\n");

	printk(BIOS_DEBUG, "Done Egress Port\n");
}

static void init_dmi(void)
{
	int timeout;
	u32 reg32;

	/* VCp: enable with ID 6 and TC mapping for ME traffic */
	dmibar_write32(DMIVCPRCTL, 0x86000000);

	/* DMI link config: bits[23:16] = 0x40 */
	dmibar_clrsetbits32(0x1b0, 0xff << 16, 0x40 << 16);

	/* Wait for DMI VCp negotiation */
	timeout = X4X_POLL_TIMEOUT_US;
	while ((dmibar_read8(DMIVCPRSTS) & VCPNP) && timeout-- > 0)
		udelay(1);
	if (timeout <= 0)
		printk(BIOS_WARNING, "DMI VCp negotiation timeout\n");

	/* Clear DMIBAR+0x204 bits [11:10] */
	dmibar_clrbits32(0x204, 3 << 10);

	/* DMI link config: clear bit 13, set bit 12 */
	dmibar_clrsetbits32(0x0f0, 1 << 13, 1 << 12);

	/* Set bit 0 of DMI link config */
	dmibar_setbits32(0x1b0, 1);

	/* Clear error status */
	dmibar_write32(DMIUESTS, 0xffffffff);
	dmibar_write32(DMICESTS, 0xffffffff);

	/* VC0: TC0 only */
	dmibar_write8(DMIVC0RCTL, 1);
	dmibar_write8(DMIPVCCAP1, 1);

	/* VC1: ID1, TC7 */
	reg32 = (dmibar_read32(DMIVC1RCTL) & ~(7 << 24)) | (1 << 24);
	reg32 = (reg32 & ~0xff) | 1 << 7;

	/* VC1: enable */
	reg32 |= 1 << 31;
	reg32 = (reg32 & ~(0x7 << 17)) | (0x4 << 17);

	dmibar_write32(DMIVC1RCTL, reg32);

	/* Set up VCs in southbridge RCBA */
	RCBA8(0x3022) &= ~1;

	reg32 = (0x5 << 28) | (1 << 6); /* PCIe x4 */
	RCBA32(0x2020) = (RCBA32(0x2020) & ~((0xf << 28) | (0x7 << 6))) | reg32;

	/* Assign VC1 id 1 */
	RCBA32(0x20) = (RCBA32(0x20) & ~(0x7 << 24)) | (1 << 24);

	/* Map TC7 to VC1 */
	RCBA8(0x20) &= 1;
	RCBA8(0x20) |= 1 << 7;

	/* Map TC0 to VC0 */
	RCBA8(0x14) &= 1;

	/* Init DMI VC1 port arbitration table */
	RCBA32(0x20) &= 0xfff1ffff;
	RCBA32(0x20) |= 1 << 19;

	RCBA32(0x30) = 0x0000000f;
	RCBA32(0x34) = 0x000f0000;
	RCBA32(0x38) = 0;
	RCBA32(0x3c) = 0x000000f0;
	RCBA32(0x40) = 0x0f000000;
	RCBA32(0x44) = 0;
	RCBA32(0x48) = 0x0000f000;
	RCBA32(0x4c) = 0;
	RCBA32(0x50) = 0x0000000f;
	RCBA32(0x54) = 0x000f0000;
	RCBA32(0x58) = 0;
	RCBA32(0x5c) = 0x000000f0;
	RCBA32(0x60) = 0x0f000000;
	RCBA32(0x64) = 0;
	RCBA32(0x68) = 0x0000f000;
	RCBA32(0x6c) = 0;

	RCBA32(0x20) |= 1 << 16;

	/* Enable VC1 */
	RCBA32(0x20) |= 1 << 31;

	/* Wait for VC1 */
	timeout = X4X_POLL_TIMEOUT_US;
	while ((RCBA8(0x26) & (1 << 1)) != 0 && timeout-- > 0)
		;
	if (timeout <= 0)
		printk(BIOS_WARNING, "RCBA VC1 negotiation timeout\n");

	/* Wait for table load */
	timeout = X4X_POLL_TIMEOUT_US;
	while ((RCBA8(0x26) & (1 << 0)) != 0 && timeout-- > 0)
		;
	if (timeout <= 0)
		printk(BIOS_WARNING, "RCBA VC1 table load timeout\n");

	/* ASPM on DMI link */
	RCBA16(0x1a8) &= ~0x3;
	/* FIXME: Do we need to read RCBA16(0x1a8)? */
	RCBA16(0x1a8);
	RCBA32(0x2010) = (RCBA32(0x2010) & ~(0x3 << 10)) | (1 << 10);
	/* FIXME: Do we need to read RCBA32(0x2010)? */
	RCBA32(0x2010);

	/* Set up VC1 max time */
	RCBA32(0x1c) = (RCBA32(0x1c) & ~0x7f0000) | 0x120000;

	timeout = X4X_POLL_TIMEOUT_US;
	while ((dmibar_read32(DMIVC1RSTS) & VC1NP) != 0 && timeout-- > 0)
		;
	if (timeout <= 0)
		printk(BIOS_WARNING, "DMI VC1 negotiation timeout\n");
	printk(BIOS_DEBUG, "Done DMI setup\n");

	/* ASPM on DMI */
	dmibar_clrbits32(0x200, 3 << 26);
	dmibar_clrsetbits16(0x210, 0xff7, 0x101);
	dmibar_clrbits32(DMILCTL, 3);
	dmibar_setbits32(DMILCTL, 3);
	/* FIXME: Do we need to read RCBA16(DMILCTL)? Probably not. */
	dmibar_read16(DMILCTL);
}

void x4x_late_init(void)
{
	init_egress();
	init_dmi();
	init_peg();
}
