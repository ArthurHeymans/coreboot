/* SPDX-License-Identifier: GPL-2.0-only */

/*
 * NBIO base initialization of the vendor AmdNbioBaseZPPei, per die (sp4r2
 * report NBIO_BASE_ZP and nbio-base-contracts.json).
 */

#include <console/console.h>
#include <cpu/amd/msr.h>
#include <cpu/amd/mtrr.h>
#include <cpu/x86/msr.h>
#include <device/pci_ops.h>
#include <soc/iomap.h>
#include <soc/soc_util.h>
#include <types.h>

#include "nbio.h"
#include "nbio_tables.h"
#include "topology.h"

#define NBIF_BUS_CFG		0x31404	/* bits 31:24: root bus of the die */
#define NBIF_REV_STRAP		0x1013460c
#define NBIF_PARITY_CTRL	0x39004
#define NBIF_HWINIT_LOCK	0x2358c

static void apply_table(unsigned int die, const struct zeppelin_nbio_reg *table, size_t count)
{
	for (size_t i = 0; i < count; i++) {
		const struct zeppelin_nbio_reg *r = &table[i];

		if (r->op == NBIO_OP_COPY)
			zeppelin_smn_write32(die, r->reg, zeppelin_smn_read32(die, r->clear));
		else if (r->clear == UINT32_MAX)
			zeppelin_smn_write32(die, r->reg, r->set);
		else
			zeppelin_smn_clrsetbits32(die, r->reg, r->clear, r->set);
	}
}

static void nbio_die_init(unsigned int die)
{
	const uint8_t bus = zeppelin_die_bus(die);
	const uint8_t rev = zeppelin_revision();

	for (unsigned int n = 0; n < NBIF_COUNT; n++)
		zeppelin_smn_clrsetbits32(die, NBIF_REG(n, NBIF_BUS_CFG), 0xff000000,
					  (uint32_t)bus << 24);

	apply_table(die, zeppelin_nbio_base_table, zeppelin_nbio_base_table_count);
	apply_table(die, zeppelin_nbio_iommu_table, zeppelin_nbio_iommu_table_count);

	/* ZP A0/B0/B1 only: strap bit 20 = vendor PCD 0xb9 (0) */
	if (rev & (ZEPPELIN_REV_A0 | ZEPPELIN_REV_B0 | ZEPPELIN_REV_B1))
		zeppelin_smn_clrsetbits32(die, NBIF_REV_STRAP, BIT(20), 0);

	zeppelin_smn_clrsetbits32(die, IOAPIC_FEATURES_ENABLE, 0, die ? 0x34 : 0x14);
	zeppelin_smn_write32(die, NB_BUS_NUM_CNTL, NB_BUS_NUM_CNTL_LAT_MODE | bus);

	/* Parity and poison handling */
	for (unsigned int n = 0; n < NBIF_COUNT; n++)
		zeppelin_smn_clrsetbits32(die, NBIF_REG(n, NBIF_PARITY_CTRL), 0, 0x77);
	zeppelin_smn_clrsetbits32(die, 0x0141f800, 0, 0x77);
	zeppelin_smn_clrsetbits32(die, 0x0151f800, 0, 0x77);
	zeppelin_smn_clrsetbits32(die, 0x0151f804, 0, 0x77);

	/* IOMMU RAS */
	zeppelin_smn_write32(die, 0x15704330, zeppelin_smn_read32(die, 0x13f00080));
	zeppelin_smn_write32(die, 0x1570401c, zeppelin_smn_read32(die, 0x0240001c));
	zeppelin_smn_clrsetbits32(die, 0x04000200, 0, BIT(16));
}

void zeppelin_nbio_early_init(void)
{
	for (unsigned int die = 0; die < zeppelin_die_count(); die++)
		nbio_die_init(die);
}

/* Tell the IOHC of each die about the DRAM map and the MMIO above 4 GiB */
void zeppelin_nbio_tom_init(void)
{
	const uint32_t tom = rdmsr(TOP_MEM_MSR).lo;
	uint64_t tom2 = 0;
	uint32_t mmio_base = 0x01000000;

	if (rdmsr(SYSCFG_MSR).lo & SYSCFG_MSR_TOM2En) {
		const msr_t msr = rdmsr(TOP_MEM2_MSR);

		tom2 = (uint64_t)msr.hi << 32 | msr.lo;
	}

	for (unsigned int die = 0; die < zeppelin_die_count(); die++) {
		const pci_devfn_t root = PCI_DEV(zeppelin_die_bus(die), 0, 0);
		uint32_t tom1_reg = pci_s_read_config32(root, NB_TOP_OF_DRAM_SLOT1);
		uint32_t lo = zeppelin_smn_read32(die, NB_LOWER_TOP_OF_DRAM2);
		uint32_t hi = zeppelin_smn_read32(die, NB_UPPER_TOP_OF_DRAM2);

		pci_s_write_config32(root, NB_TOP_OF_DRAM_SLOT1,
				     (tom1_reg & 0x007fffff) | (tom & 0xff800000));

		if (tom2 > 1ULL << 40) {
			/* TOM2 stops at the HyperTransport hole, the rest is above TOM3 */
			lo |= 0xff800001;
			hi |= 0x1ff;
			mmio_base = ((tom2 - 1) >> 16) + 1;
		} else if (tom2) {
			lo = (lo & 0x007ffffe) | ((uint32_t)tom2 & 0xff800000) | 1;
			hi = (hi & ~0x1ff) | ((tom2 >> 32) & 0xff);
		}
		zeppelin_smn_write32(die, NB_LOWER_TOP_OF_DRAM2, lo);
		zeppelin_smn_write32(die, NB_UPPER_TOP_OF_DRAM2, hi);

		zeppelin_smn_write32(die, NB_TOP_OF_DRAM3, 0x83ffffff);
		zeppelin_smn_write32(die, NB_MMIOBASE, mmio_base);
		zeppelin_smn_write32(die, NB_MMIOLIMIT, 0x7fffffff);
		zeppelin_smn_clrsetbits32(die, NB_PCI_CTRL, 0, NB_PCI_CTRL_MMIO_ENABLE);
	}

	printk(BIOS_DEBUG, "NBIO TOM 0x%08x TOM2 0x%llx\n", tom, tom2);
}

/*
 * The NBIO IOAPIC of die 0. The vendor firmware assigns it in DXE and sets
 * its ID to 0x81 + die; coreboot sets the ID when it registers the IOAPIC.
 */
void zeppelin_nbio_ioapic_init(void)
{
	smn_write32(IOAPIC_BASE_ADDR_HI, 0);
	smn_write32(IOAPIC_BASE_ADDR_LO, GNB_IO_APIC_ADDR & 0xffffff00);
	smn_write32(IOAPIC_BASE_ADDR_LO, (GNB_IO_APIC_ADDR & 0xffffff00) | IOHC_MMIO_EN);
}

static struct zeppelin_pcie_topology pcie_topology[ZEPPELIN_MAX_DIES];

struct zeppelin_pcie_topology *zeppelin_pcie_topology(unsigned int die)
{
	return &pcie_topology[die];
}
