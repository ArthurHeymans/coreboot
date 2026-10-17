/* SPDX-License-Identifier: GPL-2.0-only */

/*
 * FCH Taishan initialization of die 0, following the vendor FchTaishanPei/Dxe
 * reset, env and late tasks (sp4r2 report FCH_TAISHAN). The SATA and USB
 * controllers are in fch_sata.c and fch_usb.c.
 */

#include <amdblocks/acpimmio.h>
#include <amdblocks/acpimmio_map.h>
#include <amdblocks/smn.h>
#include <amdblocks/amd_pci_util.h>
#include <amdblocks/alink.h>
#include <arch/io.h>
#include <console/console.h>
#include <cpu/cpu.h>
#include <device/mmio.h>
#include <device/pci_ops.h>
#include <pc80/mc146818rtc.h>
#include <soc/amd_pci_int_defs.h>
#include <soc/iomap.h>
#include <soc/smi.h>
#include <soc/southbridge.h>
#include <types.h>

#include "fch.h"
#include "fch_regs.h"
#include "topology.h"

#define PM_AB_REG_BAR			0xe0
#define PM_CPU_CTRL			0x66	/* ACPI P_BLK, 0x810 */
#define PM_LEGACY_SD_CTRL		0xd3
#define PM_LEGACY_SD_ENABLE		0xe8

#define AOAC_D3_CTL(dev)		(ACPIMMIO_BASE(AOAC) + 0x40 + 2 * (dev))
#define  AOAC_PWR_ON_DEV		BIT(3)
#define AOAC_DEV_LEGACY_SD		0x19
#define AOAC_REG_54			(ACPIMMIO_BASE(AOAC) + 0x94)	/* 0xfed81e94 */

#define FCH_AL2AHB_BASE			ALINK_AHB_ADDRESS
#define FCH_LPC_DEV			PCI_DEV(0, 0x14, 3)
#define FCH_SMBUS_DEV			PCI_DEV(0, 0x14, 0)

#define SMBUS_AUTOPOLL_CTRL		0x14
#define ASF_BASE			0xb20

/* PCI IRQ routing indices of the miscellaneous settings */
#define PIRQ_IDX_MISC			0x08
#define PIRQ_IDX_MISC0			0x09
#define PIRQ_INDEX			0xc00
#define PIRQ_DATA			0xc01

struct acpimmio_rmw {
	uint8_t block;	/* 0: PM, 1: MISC, 2: SMI */
	uint8_t reg;
	uint8_t clear;
	uint8_t set;
};

#define PM(r, c, s)	{ 0, r, c, s }
#define MISC(r, c, s)	{ 1, r, c, s }
#define SMI(r, c, s)	{ 2, r, c, s }

/* The PCI IRQ routing index/data ports (amd_pci_util.c is ramstage only) */
static uint8_t pirq_read(uint8_t index)
{
	outb(index, PIRQ_INDEX);
	return inb(PIRQ_DATA);
}

static void pirq_write(uint8_t index, uint8_t value)
{
	outb(index, PIRQ_INDEX);
	outb(value, PIRQ_DATA);
}

static void apply_rmw_table(const struct acpimmio_rmw *t, size_t count)
{
	for (size_t i = 0; i < count; i++) {
		uint8_t v;

		switch (t[i].block) {
		case 0:
			v = pm_read8(t[i].reg);
			pm_write8(t[i].reg, (v & ~t[i].clear) | t[i].set);
			break;
		case 1:
			v = misc_read8(t[i].reg);
			misc_write8(t[i].reg, (v & ~t[i].clear) | t[i].set);
			break;
		default:
			v = smi_read8(t[i].reg);
			smi_write8(t[i].reg, (v & ~t[i].clear) | t[i].set);
			break;
		}
	}
}

/* ---- PEI reset tasks ---- */

/* pei_acpimmio_main of the vendor HwAcpi reset task */
static const struct acpimmio_rmw pei_acpimmio_main[] = {
	PM(0x5d, 0x03, 0x02), PM(0xd2, 0x30, 0x00),
	PM(0x47, 0x98, 0x88), PM(0x22, 0xff, 0xff), PM(0x23, 0x07, 0x07),
	PM(0x00, 0x08, 0x77), PM(0xec, 0x04, 0x04), PM(0x08, 0x01, 0x14),
	PM(0x09, 0x00, 0x01), PM(0x10, 0x03, 0x00), PM(0x54, 0xff, 0x50),
	PM(0x74, 0x09, 0x01), PM(0xc4, 0x11, 0x04), PM(0xc1, 0x07, 0x04),
	PM(0xc2, 0x40, 0x40), PM(0xbe, 0x20, 0x20), PM(0xc4, 0x04, 0x00),
	MISC(0x6e, 0x80, 0x80), MISC(0x6f, 0x08, 0x08),
};

static void fch_hwacpi_reset(void)
{
	/* ACPI PM blocks, SMI command port, PM2 control at 0xba */
	pm_write16(PM_EVT_BLK, ACPI_PM_EVT_BLK);
	pm_write16(PM1_CNT_BLK, ACPI_PM1_CNT_BLK);
	pm_write16(PM_TMR_BLK, ACPI_PM_TMR_BLK);
	pm_write16(PM_CPU_CTRL, ACPI_CSTATE_CONTROL);
	pm_write16(PM_GPE0_BLK, ACPI_GPE0_BLK);
	pm_write16(PM_ACPI_SMI_CMD, 0xb2);
	pm_write16(0x6e, 0x00ba);
	pm_write16(0x6c, 0xffff);
	pm_write32(PM_DECODE_EN, pm_read32(PM_DECODE_EN) & ~0x00180000);
	outb(0, SMB_BASE_ADDR + SMBUS_AUTOPOLL_CTRL);

	/* SMBus offset 0x12 through the ACPI MMIO alias */
	write8p(ACPIMMIO_BASE(SMBUS) + 0x12, 1);
	apply_rmw_table(pei_acpimmio_main, ARRAY_SIZE(pei_acpimmio_main));
	pm_write8(PM_DECODE_EN, pm_read8(PM_DECODE_EN) | BIT(7));	/* vendor PCD 0xa1 */

	/* RTC: no daylight saving, register A bit 4 clear */
	cmos_write_inner(cmos_read(RTC_CONTROL) & ~RTC_DST_EN, RTC_CONTROL);
	cmos_write_inner(cmos_read(RTC_FREQ_SELECT) & ~BIT(4), RTC_FREQ_SELECT);
	if ((cmos_read(RTC_VALID) & 0x3f) == 0x3f)
		cmos_write_inner(cmos_read(RTC_VALID) & 0x80, RTC_VALID);

	pirq_write(PIRQ_IDX_MISC, pirq_read(PIRQ_IDX_MISC) | 0x0a);
	pirq_write(PIRQ_IDX_MISC0, (pirq_read(PIRQ_IDX_MISC0) & 0x99) | 0x10);

	outb(0x70, ASF_BASE + 0x0e);
	outb(0x2f, ASF_BASE + 0x0a);

	if ((smi_read32(SMI_EVENT_ENABLE) & smi_read32(SMI_EVENT_STATUS)) ||
	    (inb(ACPI_PM1_STS + 1) & 5))
		outb(0x40, ACPI_PM1_STS + 1);

	/* Serial IRQ: bit 6 cleared (vendor PCD 0x95), then enabled */
	pm_write8(0x54, (pm_read8(0x54) & ~BIT(6)) | BIT(7));
}

void zeppelin_fch_reset_init(void)
{
	/* Prelim */
	pm_write8(0x0a, pm_read8(0x0a) | BIT(3));
	write32p(AOAC_REG_54, read32p(AOAC_REG_54) & ~0xc000);
	pm_write8(0x0a, pm_read8(0x0a) | BIT(4));
	pm_write8(0xc8, pm_read8(0xc8) & ~BIT(4));
	if (((cpuid_eax(0x80000001) >> 4) & 0xf) == 0)
		pm_write8(0x08, (pm_read8(0x08) & 0x9f) | 0x60);
	pm_write8(0xd2, pm_read8(0xd2) & ~BIT(6));
	pm_write8(0xc0, pm_read8(0xc0) | BIT(1));	/* clear the reset status */

	/* A-Link index/data at 0xcd8, kept enabled */
	pm_write32(PM_AB_REG_BAR, AB_INDX);

	fch_hwacpi_reset();

	/* LPC */
	pci_s_write_config8(FCH_LPC_DEV, 0x6c, 0);
	pci_s_write_config8(FCH_LPC_DEV, 0x7c, pci_s_read_config8(FCH_LPC_DEV, 0x7c) | 0x05);
	pci_s_write_config8(FCH_LPC_DEV, 0xba,
			    (pci_s_read_config8(FCH_LPC_DEV, 0xba) & ~0x60) | 0x60);
	pci_s_write_config8(FCH_LPC_DEV, 0xa4, pci_s_read_config8(FCH_LPC_DEV, 0xa4) | 0x01);
	pci_s_write_config8(FCH_LPC_DEV, 0xd1, pci_s_read_config8(FCH_LPC_DEV, 0xd1) | 0x60);
}

/* ---- DXE env tasks ---- */

static const struct acpimmio_rmw env_misc_table[] = {
	PM(0xcc, 0x07, 0x03), PM(0x74, 0xff, 0x17), PM(0x77, 0x20, 0x00),
	PM(0xba, 0x08, 0x08), PM(0xbc, 0x02, 0x02),
	/* SCI map: SCI source n -> GEVENT */
	SMI(0x41, 0xff, 0x01), SMI(0x43, 0xff, 0x03), SMI(0x44, 0xff, 0x04),
	SMI(0x45, 0xff, 0x05), SMI(0x46, 0xff, 0x06), SMI(0x57, 0xff, 0x17),
	SMI(0x48, 0xff, 0x18), SMI(0x65, 0xff, 0x1e), SMI(0x66, 0xff, 0x1f),
	SMI(0x70, 0xff, 0x09), SMI(0x79, 0xff, 0x0b), SMI(0x6c, 0xff, 0x0d),
	SMI(0x5d, 0xff, 0x10), SMI(0x5e, 0xff, 0x11),
	SMI(0x08, 0x18, 0x00), SMI(0x0e, 0x08, 0x08),
};

static const struct acpimmio_rmw env_hwacpi_table[] = {
	PM(0xed, 0x02, 0x02), PM(0xbb, 0x40, 0x00), PM(0x77, 0x0b, 0x08),
	PM(0x02, 0x04, 0x04), PM(0x81, 0x18, 0x18),
	PM(0x94, 0xff, 0x01), PM(0x83, 0x21, 0x20),
	MISC(0x6e, 0xa0, 0xa0),
};

/* Clock generator: the same values as Picasso, through the ACPI MMIO window */
static void fch_clock_init(void)
{
	misc_write32(0x10, misc_read32(0x10) & ~0x00001ff3);
	misc_write32(0x34, misc_read32(0x34) & 0x007fffff);
	misc_write32(0x14, misc_read32(0x14) & 0xff0000ff);
	misc_write32(0x18, misc_read32(0x18) & 0x0000ffff);
	misc_write32(0x1c, (misc_read32(0x1c) & ~0x0c1ffe00) |
		     4 << 13 | 3 << 17 | 3 << 26 | 1 << 9);
	misc_write32(0x40, misc_read32(0x40) | BIT(25) | BIT(30));
	misc_write32(0x10, misc_read32(0x10) | BIT(29));
	misc_write32(0x08, misc_read32(0x08) | BIT(0));
}

/* A-Link: the six table rows, then the policy settings of the vendor code */
static void fch_ab_init(void)
{
	static const struct {
		uint32_t reg;
		uint32_t clear;
		uint32_t set;
	} abcfg_table[] = {
		{ 0x00080, 0x00060000, 0x00060001 },
		{ 0x00090, 0x00200000, 0x00200000 },
		{ 0x00094, 0x00100000, 0x00100fee },
		{ 0x10054, 0x00ffffff, 0x000007ff },
		{ 0x10060, 0xfbffffff, 0x02000000 },
		{ 0x10090, 0x00010000, 0x00010000 },
		{ 0x10050, 0x00000004, 0x00000000 },
		{ 0x00054, 0x00ff0000, 0x00100000 },
		{ 0x10054, 0x00ff0000, 0x00100000 },
		{ 0x00054, 0x00000010, 0x00000010 },
		{ 0x00054, 0x01000000, 0x01000000 },
		{ 0x10054, 0x03000000, 0x03000000 },
		{ 0x00054, 0x00000005, 0x00000004 },
		{ 0x000bc, 0x00000010, 0x00000010 },
		{ 0x00058, 0xa0000000, 0xa0000000 },
		{ 0x00208, 0x00000280, 0x00000280 },
		{ 0x00208, 0x00408010, 0x00408010 },
		{ 0x00180, 0x00001f21, 0x00000000 },
		{ 0x00184, 0x00000004, 0x00000000 },
		{ 0x0022c, 0x00004800, 0x00000000 },
	};

	misc_write32(MISC_CLK_CNTL1, misc_read32(MISC_CLK_CNTL1) & ~BIT(1));
	pm_write32(PM_AB_REG_BAR, AB_INDX);
	for (size_t i = 0; i < ARRAY_SIZE(abcfg_table); i++)
		abcfg_reg(abcfg_table[i].reg, abcfg_table[i].clear, abcfg_table[i].set);

	misc_write8(0x68, misc_read8(0x68) & ~(BIT(1) | BIT(2)));
	misc_write8(0x2e, misc_read8(0x2e) | BIT(0) | BIT(1));
	pm_write8(0x06, pm_read8(0x06) | BIT(0));
}

void zeppelin_fch_env_init(void)
{
	fch_clock_init();
	apply_rmw_table(env_misc_table, ARRAY_SIZE(env_misc_table));

	/* HwAcpi */
	pci_s_write_config8(FCH_SMBUS_DEV, 0x10, 0);
	pci_s_write_config8(FCH_SMBUS_DEV, 0x11, 0);
	apply_rmw_table(env_hwacpi_table, ARRAY_SIZE(env_hwacpi_table));
	pm_write32(PM_DECODE_EN, (pm_read32(PM_DECODE_EN) | HPET_EN) & ~HPET_MSI_EN);
	write8p(FCH_AL2AHB_BASE + 0x10, read8p(FCH_AL2AHB_BASE + 0x10) | BIT(1));
	write8p(FCH_AL2AHB_BASE + 0x30, read8p(FCH_AL2AHB_BASE + 0x30) | BIT(1));
	write16p(FCH_AL2AHB_BASE + 0x20, 0xe400);	/* legacy UART decode: none enabled */

	/* LPC */
	pci_s_write_config8(FCH_LPC_DEV, 0x40,
			    (pci_s_read_config8(FCH_LPC_DEV, 0x40) & ~0x04) | 0x04);
	pci_s_write_config8(FCH_LPC_DEV, 0x78, pci_s_read_config8(FCH_LPC_DEV, 0x78) & ~0x03);
	pci_s_write_config8(FCH_LPC_DEV, 0xbb,
			    (pci_s_read_config8(FCH_LPC_DEV, 0xbb) & ~0x0c) | 0x0c);

	/* Legacy SD controller off */
	pm_write8(PM_LEGACY_SD_CTRL, pm_read8(PM_LEGACY_SD_CTRL) & ~BIT(6));
	pm_write8(PM_LEGACY_SD_ENABLE, pm_read8(PM_LEGACY_SD_ENABLE) & ~BIT(0));
	write8p(AOAC_D3_CTL(AOAC_DEV_LEGACY_SD),
		read8p(AOAC_D3_CTL(AOAC_DEV_LEGACY_SD)) & ~AOAC_PWR_ON_DEV);

	/* Gigabyte subsystem ID of the xHCI (vendor PCD 0x163) */
	smn_write32(0x1014304c, 0x10001458);

	fch_ab_init();
}

/* ---- DXE late tasks ---- */

/* IOAPIC routing of the internal devices (vendor table 9250) */
static const struct {
	uint8_t index;
	uint8_t irq;
} internal_irq_routing[] = {
	{ PIRQ_A, 0x10 }, { PIRQ_B, 0x11 }, { PIRQ_C, 0x12 }, { PIRQ_D, 0x13 },
	{ PIRQ_SD, 0x10 }, { PIRQ_SDIO, 0x10 }, { PIRQ_EMMC, 0x05 }, { PIRQ_GPIO, 0x07 },
	{ PIRQ_UART0, 0x03 }, { PIRQ_UART1, 0x04 }, { PIRQ_UART2, 0x03 }, { PIRQ_UART3, 0x04 },
	{ PIRQ_I2C0, 0x0a }, { PIRQ_I2C1, 0x0b }, { PIRQ_I2C2, 0x04 }, { PIRQ_I2C3, 0x06 },
	{ PIRQ_I2C4, 0x0e }, { PIRQ_I2C5, 0x0f },
};

void zeppelin_fch_late_init(void)
{
	pm_write32(0xa0, pm_read32(0xa0) & 0xc07f00a0);

	for (size_t i = 0; i < ARRAY_SIZE(internal_irq_routing); i++)
		write_pci_int_idx(internal_irq_routing[i].index, 1, internal_irq_routing[i].irq);
}
