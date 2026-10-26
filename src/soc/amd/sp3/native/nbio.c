/* SPDX-License-Identifier: GPL-2.0-only */

#include <console/console.h>
#include <cpu/amd/mtrr.h>
#include <cpu/x86/msr.h>
#include <device/pci_ops.h>
#include "nbio.h"
#include "nbio_tables.h"
#include "topology.h"

static void table(unsigned int rb, const struct sp3_nbio_reg *t, size_t count, bool socket)
{
	const uint32_t offset = socket ? 0 : sp3_nbio_index(rb) << 20;

	for (size_t i = 0; i < count; i++)
		sp3_smn_clrsetbits32(rb, t[i].reg + offset, t[i].clear, t[i].set);
}

void sp3_nbio_early_init(void)
{
	table(0, sp3_nbio_socket_table, sp3_nbio_socket_table_count, true);
	for (unsigned int rb = 0; rb < sp3_die_count(); rb++) {
		const uint32_t q = sp3_nbio_index(rb) << 20;
		const uint32_t bus = sp3_die_bus(rb);
		const uint32_t pool = sp3_nonpci_mmio_base(rb);
		const uint32_t sb = sp3_smn_read32(rb, 0x13b1007c + q);

		if (!rb) {
			sp3_smn_write32(rb, 0x15300024 + q, sb);
			sp3_smn_write32(rb, 0x13f0112c + q, sb);
		} else {
			sp3_smn_write32(rb, 0x13b1007c + q, 0);
		}
		table(rb, sp3_nbio_base_table, sp3_nbio_base_table_count, false);
		sp3_smn_clrsetbits32(rb, 0x14300000 + q, 0, rb ? 0x34 : 0x14);
		sp3_smn_write32(rb, 0x13b10044 + q, bus | 0x100);
		/* Rome sets NBIF1's three internal bridge bus/devfn straps. */
		for (unsigned int p = 0; p < 3; p++)
			sp3_smn_clrsetbits32(rb, 0x1053101c + q + 0x200 * p,
				0xffff0000, (0x28 + 0x20 * p) << 24 | bus << 16);
		/* Keep an SMU BAR left by ABL; otherwise allocate, enable and lock. */
		if (!sp3_smn_read32(rb, 0x13b102e8 + q) &&
		    !sp3_smn_read32(rb, 0x13b102ec + q)) {
			sp3_smn_write32(rb, 0x13b102ec + q, 0);
			sp3_smn_write32(rb, 0x13b102e8 + q, pool + MiB);
			sp3_smn_clrsetbits32(rb, 0x13b102e8 + q, 0, 3);
		}
		table(rb, sp3_nbio_iommu_table, sp3_nbio_iommu_table_count, false);
	}
}

void sp3_nbio_tom_init(void)
{
	const uint64_t tom = rdmsr(TOP_MEM_MSR).raw;
	const uint64_t tom2 = rdmsr(TOP_MEM2_MSR).raw;

	for (unsigned int rb = 0; rb < sp3_die_count(); rb++) {
		const uint32_t q = sp3_nbio_index(rb) << 20;
		const pci_devfn_t root = PCI_DEV(sp3_die_bus(rb), 0, 0);

		pci_s_write_config32(root, 0x90, (pci_s_read_config32(root, 0x90) & 0x7ffffe) |
			(tom & 0xff800000) | (tom == 1ULL << 32));
		if (tom2 >> 32 > 0x100) {
			sp3_smn_clrsetbits32(rb, 0x13b10064 + q, 0xff800001, 0xff800001);
			sp3_smn_clrsetbits32(rb, 0x13b10068 + q, 0x1ff, 0x1ff);
			sp3_smn_write32(rb, 0x13b10138 + q,
				0x80000000 | (((tom2 - 1) >> 22) & 0x3fffffff));
		} else if (tom2) {
			sp3_smn_clrsetbits32(rb, 0x13b10064 + q, 0xff800001,
				(tom2 & 0xff800000) | 1);
			sp3_smn_clrsetbits32(rb, 0x13b10068 + q, 0x1ff, tom2 >> 32);
		}
		sp3_smn_clrsetbits32(rb, 0x13b0004c + q, 0, BIT(23));
	}
}

void sp3_nbio_ioapic_init(void)
{
	for (unsigned int rb = 0; rb < sp3_die_count(); rb++) {
		const uint32_t q = sp3_nbio_index(rb) << 20;
		const uint32_t pool = sp3_nonpci_mmio_base(rb);

		sp3_smn_write32(rb, 0x13b102f4 + q, 0);
		sp3_smn_write32(rb, 0x13b102f0 + q, pool + 7 * MiB);
		sp3_smn_clrsetbits32(rb, 0x13b102f0 + q, 0, 1);
		sp3_smn_write32(rb, 0x02801000 + q, (0xf1 + sp3_nbio_index(rb)) << 24);
		/* IOMMU BAR, 512 KiB aligned and reserved from PCI allocation. */
		sp3_smn_write32(rb, 0x13f00048 + q, 0);
		sp3_smn_write32(rb, 0x13f00044 + q, pool + 2 * MiB);
	}
}

