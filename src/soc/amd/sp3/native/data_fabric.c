/* SPDX-License-Identifier: GPL-2.0-only */

/* Rome DF3 register layouts: SOC_DF_SP3 sections 3.1 and 3.6. */
#include <console/console.h>
#include <cpu/amd/mtrr.h>
#include <cpu/x86/msr.h>
#include <types.h>
#include "apob.h"
#include "native.h"
#include "topology.h"

#define DIRECT SP3_DF_DIRECT
#define MMIO_BASE(i) (0x200 + 16 * (i))
#define MMIO_LIMIT(i) (0x204 + 16 * (i))
#define MMIO_CTRL(i) (0x208 + 16 * (i))

static void df_modify(uint8_t fn, uint16_t reg, uint8_t instance, uint32_t clear, uint32_t set)
{
	sp3_df_node_write32(0, fn, reg, instance,
		(sp3_df_node_read32(0, fn, reg, instance) & ~clear) | set);
}

static void mmio(unsigned int pair, uint64_t base, uint64_t end, unsigned int rb)
{
	if (pair >= 16 || base >= end || (base & 0xffff) || (end & 0xffff))
		die("Invalid SP3 DF MMIO range\n");
	sp3_df_node_write32(0, 0, MMIO_CTRL(pair), DIRECT, 0);
	sp3_df_node_write32(0, 0, MMIO_BASE(pair), DIRECT, base >> 16);
	sp3_df_node_write32(0, 0, MMIO_LIMIT(pair), DIRECT, (end - 1) >> 16);
	sp3_df_node_write32(0, 0, MMIO_CTRL(pair), DIRECT,
		sp3_ioms_fabric_id(rb) << 4 | 3);
	printk(BIOS_DEBUG, "DF MMIO %u: %llx-%llx -> domain %u\n", pair, base, end - 1, rb);
}

void sp3_df_mmio_init(void)
{
	const struct sp3_topology *t = sp3_topology();
	const unsigned int n = t->root_bridge_count;
	const uint64_t tom = rdmsr(TOP_MEM_MSR).raw;
	const uint64_t tom2 = rdmsr(TOP_MEM2_MSR).raw;
	const uint64_t cfg = CONFIG_ECAM_MMCONF_BASE_ADDRESS;
	const uint64_t low_per = ALIGN_DOWN((cfg - tom) / n, 16 * MiB);
	const uint64_t high_base = MAX(tom2, 1ULL << 40);
	const uint64_t high_end = 0x7fd00000000ULL;
	const uint64_t high_per = ALIGN_DOWN((high_end - high_base) / n, 16 * MiB);

	if (tom >= cfg || low_per < 16 * MiB || high_base >= high_end)
		die("Insufficient SP3 DF MMIO space\n");
	for (unsigned int i = 0; i < 16; i++)
		sp3_df_node_write32(0, 0, MMIO_CTRL(i), DIRECT, 0);
	for (unsigned int rb = 0; rb < n; rb++) {
		const unsigned int physical = sp3_nbio_index(rb);
		const uint64_t base = tom + rb * low_per;
		const uint64_t end = rb == n - 1 ? cfg : base + low_per;
		const uint64_t high = high_base + rb * high_per;
		const uint32_t io_base = rb * 0x4000;
		const uint32_t io_end = rb == n - 1 ? 0x1ffffff : io_base + 0x3fff;

		mmio(2 * physical, base, end, rb);
		mmio(2 * physical + 1, high,
		     rb == n - 1 ? 1ULL << 48 : high + high_per, rb);
		sp3_df_node_write32(0, 0, 0xc0 + rb * 8, DIRECT, io_base | 3);
		sp3_df_node_write32(0, 0, 0xc4 + rb * 8, DIRECT,
			(io_end & 0x1fff000) | sp3_ioms_fabric_id(rb));
	}
	for (unsigned int i = n; i < 8; i++)
		sp3_df_node_write32(0, 0, 0xc0 + i * 8, DIRECT, 0);
	/* FCH root has a second window above the 256-bus ECAM aperture. */
	mmio(8, cfg + 256 * MiB, 0xfec00000, 0);
	mmio(9, 0xfed00000, 0xfed10000, 0);
	sp3_df_route_vga(0);
}

uint32_t sp3_nonpci_mmio_base(unsigned int rb)
{
	const unsigned int pair = rb ? 2 * sp3_nbio_index(rb) : 8;
	const uint64_t top = ((uint64_t)sp3_df_node_read32(0, 0, MMIO_LIMIT(pair), DIRECT) + 1) << 16;

	if (top > 4ULL * GiB || top < SP3_NONPCI_MMIO_SIZE)
		die("Invalid SP3 non-PCI MMIO pool\n");
	return top - SP3_NONPCI_MMIO_SIZE;
}

void sp3_df_early_init(void)
{
	static const uint8_t instances[] = {
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
		0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
		0x18, 0x19, 0x1a, 0x1b, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24,
	};

	sp3_df_node_write32(0, 0, 0x3f0, DIRECT, 1);
	sp3_df_node_write32(0, 2, 0x6c, DIRECT, 0x10040611);
	df_modify(2, 0x44, DIRECT, 0, 0x4000);
	const uint32_t cs = sp3_df_node_read32(0, 2, 0x48, DIRECT) | 0x868000f0;
	sp3_df_node_write32(0, 2, 0x48, DIRECT, cs);
	sp3_df_node_write32(0, 2, 0x48, 0xa, cs | 0x20000);
	sp3_df_node_write32(0, 4, 0xc8, DIRECT, 0x07020000);
	for (unsigned int i = 0; i <= 0x30; i++) {
		if ((i >= 0xc && i < 0x10) || i == 0x1c || i == 0x1d)
			continue;
		for (unsigned int reg = 0x1f0; reg <= 0x1f4; reg += 4) {
			if (sp3_df_node_read32(0, 6, reg, i))
				sp3_df_node_write32(0, 6, reg, i, 0);
		}
	}
	for (unsigned int i = 0; i < ARRAY_SIZE(instances); i++)
		df_modify(0, 0x3f8, instances[i], 0x77708, 0x66600);
}

void sp3_df_scrub_init(void)
{
	/* ABL supplies the region; preserve its address and use the donor's capped rate. */
	uint8_t dimms, channels;

	if (!sp3_apob_read8(sp3_apob(), 1, 0x11, 0, 0x30, 0, &dimms) ||
	    !sp3_apob_read8(sp3_apob(), 1, 0x11, 0, 0x31, 0, &channels) ||
	    !dimms || dimms > 2 || channels > 8)
		die("Missing APOB SPD channel map\n");
	for (unsigned int ch = 0; ch < channels; ch++) {
		bool present = false;

		for (unsigned int d = 0; d < dimms; d++) {
			uint8_t populated;

			if (!sp3_apob_read8(sp3_apob(), 1, 0x11, 0,
				0x38 + (ch * dimms + d) * 0x214, 0, &populated))
				die("Truncated APOB SPD records\n");
			present |= populated != 0;
		}
		df_modify(6, 0xb8, ch, present ? 0xf : 3, present ? 3 : 0);
		df_modify(2, 0x228, ch, 0x3e000000, BIT(24));
	}
	if (sp3_df_node_read32(0, 6, 0x44, DIRECT)) {
		df_modify(6, 0x44, DIRECT, 0xf, 3);
		df_modify(6, 0x40, DIRECT, 0, 3);
	}
}

void sp3_df_late_init(void)
{
	sp3_df_route_vga(0);
}

void sp3_df_route_vga(unsigned int rb)
{
	df_modify(0, 0x80, DIRECT, 0x3ff4, sp3_ioms_fabric_id(rb) << 4 | 1);
}

void sp3_df_disable_ext_cfg(void)
{
	df_modify(4, 0x44, DIRECT, 1, 0);
}
