/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/acpimmio.h>
#include <amdblocks/amd_pci_util.h>
#include <amdblocks/alink.h>
#include <amdblocks/aoac.h>
#include <amdblocks/smn.h>
#include <arch/io.h>
#include <cf9_reset.h>
#include <console/console.h>
#include <device/device.h>
#include <device/mmio.h>
#include <device/pci_ops.h>
#include <pc80/mc146818rtc.h>
#include <soc/amd_pci_int_defs.h>
#include <soc/aoac_defs.h>
#include <soc/iomap.h>
#include <soc/pci_devs.h>
#include <soc/smi.h>
#include <soc/southbridge.h>
#include <static.h>
#include <timer.h>
#include <types.h>

#include "../chip.h"
#include "fch.h"
#include "fch_regs.h"

#define  PM_ESPI_IRQ1_MASK		BIT(1)
#define  PM_ESPI_IRQ12_MASK		BIT(12)
#define PM_LEGACY_SD_CTRL		0xd3
#define  PM_LEGACY_SD_RESET		BIT(6)
#define PM_LEGACY_SD_ENABLE		0xe8
#define  PM_LEGACY_SD_EN		BIT(0)
#define PM_AB_REG_BAR			0xe0

/* LPC bridge registers */
#define LPC_REG_A4			0xa4	/* bit 0 isn't documented in the PPR */
#define LPC_CLK_CNTL			0xd0
#define  LPC_CLKRUN_EN			BIT(31)

#define SMBUS_AUTOPOLL_CTRL		0x14
#define SMBUS_ASF_STATUS		0x2a

/* ACPI PM1 status, bit 14 is the PCIe wake status */
#define ACPI_PM1_STS_PCIEXP_WAKE	(PCIEXPWAK_STS >> 8)	/* in the high byte */

/* FCH MISC registers accessed through SMN */
#define MISC_CG_PLL_CFG1		0x08
#define MISC_CG_PLL_CFG3		0x10
#define MISC_CG_PLL_CFG4		0x14
#define MISC_CG_PLL_CFG5		0x18
#define MISC_CG_PLL_CFG6		0x1c
#define MISC_CG_PLL_CFG8		0x34

/* SATA configuration registers accessed through the SATA controller's index/data pair */
#define SATA_INDEX			0xb8
#define SATA_DATA			0xbc
#define  SATA_RESET_ACTIVE		BIT(8)
#define  SATA_LINK_STATE_MASK		0xf
#define  SATA_LINK_STATE_ACTIVE		4
#define SATA_PORT_CTRL			0x0
#define SATA_PORT1_CTRL			0x10000
#define SATA_PORT_COUNT			2

/* eMMC configuration registers */
#define EMMC_CFG(reg)			(APU_EMMC_CONFIG_BASE + (reg))
#define EMMC_CFG_PHY_TIMING		0xa4
#define EMMC_CFG_INTERFACE		0xa8
#define EMMC_CFG_MODE			0xb0
#define EMMC_CFG_PHY_DRIVE		0xd0
#define EMMC_CFG_PRESET_VALUE(n)	(0xf0 + (n) * 2)
#define  EMMC_PRESET_DIVIDER_MASK	0x03ff
#define  EMMC_PRESET_STRENGTH_SHIFT	14
#define EMMC_CFG_MODE_CLEAR		0x116

void picasso_fch_ab_init(void)
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
		{ 0x00054, 0x00000001, 0x00000000 },
		{ 0x00054, 0x00000004, 0x00000004 },
		{ 0x00054, 0x00000001, 0x00000001 },
		{ 0x00054, 0x00000004, 0x00000000 },
		{ 0x000bc, 0x00000010, 0x00000010 },
		{ 0x00058, 0x20000000, 0x20000000 },
		{ 0x00058, 0x80000000, 0x80000000 },
		{ 0x00208, 0x00000280, 0x00000280 },
		{ 0x00208, 0x00408010, 0x00408010 },
		{ 0x00180, 0x00000001, 0x00000000 },
		{ 0x00184, 0x00000004, 0x00000000 },
		{ 0x00180, 0x00001f00, 0x00000000 },
		{ 0x00180, 0x00000020, 0x00000000 },
		{ 0x0022c, 0x00000800, 0x00000000 },
		{ 0x0022c, 0x00004000, 0x00000000 },
		{ 0x00228, 0x0000000a, 0x0000000a },
	};

	/* Enable the A-Link index/data I/O ports */
	pm_write32(PM_AB_REG_BAR, AB_INDX);
	misc_write32(MISC_CLK_CNTL1, misc_read32(MISC_CLK_CNTL1) & ~BIT(1));

	for (size_t i = 0; i < ARRAY_SIZE(abcfg_table); i++)
		abcfg_reg(abcfg_table[i].reg, abcfg_table[i].clear, abcfg_table[i].set);

	pm_write32(PM_AB_REG_BAR, 0);
}

/* IOAPIC routing of the internal devices */
static const struct {
	uint8_t index;
	uint8_t irq;
} internal_irq_routing[] = {
	{ PIRQ_A, 0x10 },
	{ PIRQ_B, 0x11 },
	{ PIRQ_C, 0x12 },
	{ PIRQ_D, 0x13 },
	{ PIRQ_SD, 0x10 },
	{ PIRQ_SDIO, 0x10 },
	{ PIRQ_EMMC, 0x05 },
	{ PIRQ_GPIO, 0x07 },
	{ PIRQ_I2C4, 0x03 },
	{ PIRQ_I2C5, 0x04 },
	{ PIRQ_UART2, 0x0f },
	{ PIRQ_UART3, 0x05 },
	{ PIRQ_I2C0, 0x0a },
	{ PIRQ_I2C1, 0x0b },
	{ PIRQ_I2C2, 0x0e },
	{ PIRQ_I2C3, 0x06 },
	{ PIRQ_UART0, 0x0e },
	{ PIRQ_UART1, 0x0f },
};

void picasso_fch_legacy_init(void)
{
	if (CONFIG(SOC_AMD_COMMON_BLOCK_USE_ESPI))
		pm_write32(PM_ESPI_INTR_CTRL, pm_read32(PM_ESPI_INTR_CTRL) &
			   ~(PM_ESPI_IRQ1_MASK | PM_ESPI_IRQ12_MASK));

	pm_io_write8(0x0a, pm_io_read8(0x0a) | BIT(3));
	pm_io_write8(0x0a, pm_io_read8(0x0a) | BIT(4));
	/* Clear the reset status */
	pm_io_write8(0xc0, pm_io_read8(0xc0) | BIT(1));

	outb(0, SMB_BASE_ADDR + SMBUS_AUTOPOLL_CTRL);

	/* Disable the RTC daylight saving and bit 4 of register A */
	cmos_write_inner(cmos_read(RTC_CONTROL) & ~RTC_DST_EN, RTC_CONTROL);
	cmos_write_inner(cmos_read(RTC_FREQ_SELECT) & ~BIT(4), RTC_FREQ_SELECT);

	write_pci_int_idx(PIRQ_MISC, 0, read_pci_int_idx(PIRQ_MISC, 0) | 0x0a);
	write_pci_int_idx(PIRQ_MISC0, 0, (read_pci_int_idx(PIRQ_MISC0, 0) & 0x99) | 0x10);

	outb(0x2f, SMB_BASE_ADDR + SMBUS_ASF_STATUS);

	if (smi_read32(SMI_EVENT_ENABLE) & smi_read32(SMI_EVENT_STATUS))
		outb(ACPI_PM1_STS_PCIEXP_WAKE, ACPI_PM1_STS + 1);
	if (inb(ACPI_PM1_STS + 1) & 5)
		outb(ACPI_PM1_STS_PCIEXP_WAKE, ACPI_PM1_STS + 1);

	for (size_t i = 0; i < ARRAY_SIZE(internal_irq_routing); i++)
		write_pci_int_idx(internal_irq_routing[i].index, 1, internal_irq_routing[i].irq);

	/* Put the legacy SD controller in reset and power it off when it's not used */
	if (!is_dev_enabled(DEV_PTR(sdhci))) {
		pm_write8(PM_LEGACY_SD_CTRL, pm_read8(PM_LEGACY_SD_CTRL) & ~PM_LEGACY_SD_RESET);
		pm_write8(PM_LEGACY_SD_ENABLE, pm_read8(PM_LEGACY_SD_ENABLE) & ~PM_LEGACY_SD_EN);
		aoac_write8(AOAC_DEV_D3_CTL(FCH_AOAC_DEV_LEGACY_SD),
			    aoac_read8(AOAC_DEV_D3_CTL(FCH_AOAC_DEV_LEGACY_SD)) &
			    ~FCH_AOAC_PWR_ON_DEV);
	}
}

void picasso_fch_lpc_init(void)
{
	pci_or_config32(SOC_LPC_DEV, LPC_REG_A4, BIT(0));
	/* Allow LPCCLK to be stopped with the CLKRUN# protocol */
	pci_or_config32(SOC_LPC_DEV, LPC_CLK_CNTL, LPC_CLKRUN_EN);
}

void picasso_fch_clock_init(void)
{
	smn_clrsetbits32(FCH_MISC_SMN(MISC_CG_PLL_CFG3), 0x00000003, 0);
	smn_clrsetbits32(FCH_MISC_SMN(MISC_CG_PLL_CFG3), 0x00001ff0, 0);
	smn_clrsetbits32(FCH_MISC_SMN(MISC_CG_PLL_CFG8), 0xff800000, 0);
	smn_clrsetbits32(FCH_MISC_SMN(MISC_CG_PLL_CFG4), 0x00ffff00, 0);
	smn_clrsetbits32(FCH_MISC_SMN(MISC_CG_PLL_CFG5), 0xffff0000, 0);
	smn_clrsetbits32(FCH_MISC_SMN(MISC_CG_PLL_CFG6), 0x0001e000, 0x00008000);
	smn_clrsetbits32(FCH_MISC_SMN(MISC_CG_PLL_CFG6), 0x001e0000, 0x00060000);
	smn_clrsetbits32(FCH_MISC_SMN(MISC_CG_PLL_CFG6), 0x0c000000, 0x0c000000);
	smn_clrsetbits32(FCH_MISC_SMN(MISC_CG_PLL_CFG6), 0x00001e00, 0x00000200);
	smn_clrsetbits32(FCH_MISC_SMN(MISC_CLK_CNTL1), BIT(25), BIT(25));
	smn_clrsetbits32(FCH_MISC_SMN(MISC_CLK_CNTL1), BIT(30), BIT(30));
	smn_clrsetbits32(FCH_MISC_SMN(MISC_CG_PLL_CFG3), BIT(29), BIT(29));
	smn_clrsetbits32(FCH_MISC_SMN(MISC_CG_PLL_CFG1), BIT(0), BIT(0));
	smn_clrsetbits32(FCH_MISC_SMN(MISC_CLK_CNTL1), BIT(30), BIT(30));
}

static uint32_t sata_read(struct device *dev, uint32_t index)
{
	pci_write_config32(dev, SATA_INDEX, index);
	return pci_read_config32(dev, SATA_DATA);
}

static void sata_write(struct device *dev, uint32_t index, uint32_t value)
{
	pci_write_config32(dev, SATA_INDEX, index);
	pci_write_config32(dev, SATA_DATA, value);
}

static void sata_clrsetbits(struct device *dev, uint32_t index, uint32_t clear, uint32_t set)
{
	sata_write(dev, index, (sata_read(dev, index) & ~clear) | set);
}

enum cb_err picasso_fch_sata_init(void)
{
	struct device *sata = DEV_PTR(sata);

	if (sata_read(sata, SATA_SMN_RESET_STATUS) & SATA_RESET_ACTIVE) {
		sata_clrsetbits(sata, SATA_PORT_CTRL, BIT(8) | BIT(11), 0);
		sata_clrsetbits(sata, SATA_PORT1_CTRL, BIT(16), BIT(16));
		outb(RST_CPU | SYS_RST, RST_CNT);
	}

	sata_clrsetbits(sata, SATA_PORT_CTRL, BIT(0), 0);
	sata_clrsetbits(sata, SATA_PORT_CTRL, BIT(1), 0);
	sata_clrsetbits(sata, SATA_PORT_CTRL, BIT(0), 0);

	for (unsigned int port = 0; port < SATA_PORT_COUNT; port++) {
		bool active;

		sata_clrsetbits(sata, 0x4, 0xf, 0x4);
		active = wait_us(100, (sata_read(sata, SATA_SMN_PORT_STATUS(port)) &
				       SATA_LINK_STATE_MASK) == SATA_LINK_STATE_ACTIVE);
		if (!active) {
			printk(BIOS_ERR, "SATA port %u link isn't active\n", port);
			return CB_ERR;
		}
	}

	return CB_SUCCESS;
}

/* eMMC mode settings for the timing modes of the devicetree */
struct emmc_mode {
	uint8_t type;
	uint8_t interface;
	uint8_t phy;
	uint8_t reference_mhz;
};

static const struct emmc_mode emmc_modes[] = {
	[SD_EMMC_DISABLE]		= { 0x00, 0x00, 0x00, 0x00 },
	[SD_EMMC_SD_LOW_SPEED]		= { 0x00, 0x20, 0x14, 0x3f },
	[SD_EMMC_SD_HIGH_SPEED]		= { 0x01, 0x20, 0x14, 0xc8 },
	[SD_EMMC_SD_UHS_I_SDR_50]	= { 0x02, 0x21, 0x14, 0xc8 },
	[SD_EMMC_SD_UHS_I_DDR_50]	= { 0x02, 0x24, 0x14, 0xc8 },
	[SD_EMMC_SD_UHS_I_SDR_104]	= { 0x02, 0x27, 0x14, 0xc8 },
	[SD_EMMC_EMMC_SDR_26]		= { 0x00, 0x08, 0x57, 0x3f },
	[SD_EMMC_EMMC_SDR_52]		= { 0x02, 0x08, 0x57, 0xc8 },
	[SD_EMMC_EMMC_DDR_104]		= { 0x02, 0x0c, 0x57, 0xc8 },
	[SD_EMMC_EMMC_HS200]		= { 0x02, 0x0b, 0x57, 0xc8 },
	[SD_EMMC_EMMC_HS400]		= { 0x02, 0x0f, 0x57, 0xc8 },
	[SD_EMMC_EMMC_HS300]		= { 0x02, 0x0f, 0x57, 0x96 },
};

/* Mode type 2 needs the SDHCI preset value registers to be programmed */
#define EMMC_MODE_PRESETS	2

static void emmc_clrsetbits32(uint16_t reg, uint32_t clear, uint32_t set)
{
	clrsetbits32((void *)(uintptr_t)EMMC_CFG(reg), clear, set);
}

/* Program the SDHCI preset value registers for the reference clock of the mode */
static void emmc_program_presets(const struct emmc_mode *mode)
{
	const struct soc_amd_picasso_config *config = config_of_soc();
	static const uint32_t target_25mhz[] = {
		0, 25000000, 50000000, 12000000, 25000000, 50000000, 104000000, 50000000, 0,
	};
	static const uint32_t target_26mhz[] = {
		0, 26000000, 52000000, 12000000, 26000000, 52000000, 200000000, 52000000,
		200000000,
	};
	const uint8_t strengths[] = {
		0, 0, 0, 0, 0,
		config->emmc_config.sdr50_driver_strength,
		config->emmc_config.sdr104_hs400_driver_strength,
		config->emmc_config.ddr50_driver_strength,
		config->emmc_config.sdr104_hs400_driver_strength,
	};
	const uint32_t *targets = (mode->phy & 0xc0) ? target_26mhz : target_25mhz;
	const uint32_t reference_hz = mode->reference_mhz * 1000000u;

	for (unsigned int i = 0; i < ARRAY_SIZE(strengths); i++) {
		const uintptr_t reg = EMMC_CFG(EMMC_CFG_PRESET_VALUE(i));
		const uint32_t target_hz = i == 0 ?
			config->emmc_config.init_khz_preset * 1000u : targets[i];
		uint32_t divider;
		uint16_t preset;

		if (i && !target_hz)
			continue;
		for (divider = 1; divider < 2 * EMMC_PRESET_DIVIDER_MASK; divider++)
			if (target_hz >= reference_hz / divider)
				break;

		preset = (read16p(reg) & ~EMMC_PRESET_DIVIDER_MASK) | (divider >> 1);
		write16p(reg, preset);
		preset = read16p(reg);
		preset &= ~(3 << EMMC_PRESET_STRENGTH_SHIFT);
		preset |= (strengths[i] & 3) << EMMC_PRESET_STRENGTH_SHIFT;
		write16p(reg, preset);
	}
}

void picasso_fch_emmc_init(void)
{
	const struct soc_amd_picasso_config *config = config_of_soc();
	unsigned int timing = config->emmc_config.timing;
	const struct emmc_mode *mode;

	if (!is_dev_enabled(DEV_PTR(emmc)) || timing >= ARRAY_SIZE(emmc_modes))
		timing = SD_EMMC_DISABLE;
	mode = &emmc_modes[timing];

	emmc_clrsetbits32(EMMC_CFG_MODE, 0xff000000, mode->type << 24);
	emmc_clrsetbits32(EMMC_CFG_INTERFACE, BIT(0), mode->interface & BIT(0));
	emmc_clrsetbits32(EMMC_CFG_INTERFACE, BIT(1), mode->interface & BIT(1));
	emmc_clrsetbits32(EMMC_CFG_INTERFACE, BIT(2), mode->interface & BIT(2));
	emmc_clrsetbits32(EMMC_CFG_PHY_DRIVE, BIT(17), ((mode->interface >> 3) & 1) << 17);
	emmc_clrsetbits32(EMMC_CFG_PHY_DRIVE, BIT(18), ((mode->interface >> 4) & 1) << 18);
	emmc_clrsetbits32(EMMC_CFG_INTERFACE, 0xc000, ((mode->interface >> 5) & 3) << 14);
	emmc_clrsetbits32(EMMC_CFG_INTERFACE, BIT(13), (mode->interface >> 7) << 13);
	emmc_clrsetbits32(EMMC_CFG_PHY_DRIVE, 0x0c000000, (mode->phy & 3) << 26);
	emmc_clrsetbits32(EMMC_CFG_PHY_TIMING, 0xc0000000, (uint32_t)(mode->phy >> 6) << 30);
	emmc_clrsetbits32(EMMC_CFG_PHY_TIMING, BIT(28), 0);
	emmc_clrsetbits32(EMMC_CFG_PHY_TIMING, 0x0000ff00, mode->reference_mhz << 8);
	write16p(EMMC_CFG(EMMC_CFG_MODE_CLEAR),
		 read16p(EMMC_CFG(EMMC_CFG_MODE_CLEAR)) & ~0x0030);

	if (mode->type == EMMC_MODE_PRESETS)
		emmc_program_presets(mode);
}
