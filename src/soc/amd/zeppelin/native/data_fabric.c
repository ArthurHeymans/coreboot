/* SPDX-License-Identifier: GPL-2.0-only */

/*
 * Zeppelin Data Fabric initialization. The sequences follow the vendor
 * AmdFabricZpPei/Dxe and the MMIO/IO distribution of AmdSocSp4ZpPei
 * (sp4r2 reports DF_ZP and SOC_DISPATCH). Each die has its own DF; what AGESA
 * calls a broadcast access is a direct access repeated on every die.
 */

#include <console/console.h>
#include <cpu/amd/msr.h>
#include <cpu/amd/mtrr.h>
#include <cpu/x86/msr.h>
#include <soc/soc_util.h>
#include <types.h>

#include "apob.h"
#include "native.h"
#include "topology.h"

#define DIRECT		ZEPPELIN_DF_DIRECT

/* Function 0 */
#define DF_F0_INSTANCE_COUNT	0x040
#define DF_F0_VGA_EN		0x080
#define DF_F0_IO_BASE(i)	(0x0c0 + 8 * (i))
#define DF_F0_IO_LIMIT(i)	(0x0c4 + 8 * (i))
#define DF_F0_MMIO_BASE(i)	(0x200 + 0x10 * (i))
#define DF_F0_MMIO_LIMIT(i)	(0x204 + 0x10 * (i))
#define DF_F0_MMIO_CTRL(i)	(0x208 + 0x10 * (i))
#define DF_F0_CLK_GATE		0x3f0
#define DF_F0_RAS_CTRL		0x3f8
/* Function 1 */
#define DF_F1_SYSTEM_CFG	0x200
/* Function 2 (CS) */
#define DF_F2_CS_CFG_68		0x068
#define DF_F2_CS_CFG_6C		0x06c
/* Function 3 and 4 (CCM, IOMS) */
#define DF_F3_IOMS_04C		0x04c
#define DF_F3_CCM_104		0x104
#define DF_F3_CCM_108		0x108
#define DF_F3_IOMS_300		0x300
#define DF_F4_EXT_CFG		0x044
#define DF_F4_CFG_0C0		0x0c0
#define DF_F4_CCM_0C4		0x0c4
#define DF_F4_CFG_0C8		0x0c8
/* Function 5 (PIE) */
#define DF_F5_PIE_CFG		0x104
#define  PIE_CFG_LINT_ERRATUM_1158	BIT(8)
#define  PIE_CFG_DIS_SYNC_FLOOD_PROP	BIT(19)
#define DF_F5_PIE_PWR_300	0x300
#define DF_F5_PIE_PWR_304	0x304
#define DF_F5_PIE_PWR_354	0x354
#define DF_F5_PIE_PWR_358	0x358
/* Function 6 */
#define DF_F6_ERR_STATUS_0	0x1f0
#define DF_F6_ERR_STATUS_1	0x1f4
#define DF_F6_CLK_GATE_DC	0x0dc

#define DF_INST_CCM0		2
#define DF_INST_CCM1		3
#define DF_INST_IOMS		4

#define DF_MMIO_PAIRS		8
#define DF_IO_PAIRS		8
#define ALIGN_16M		(~(uint64_t)0xffffff)

#define APOB_GROUP_MEMORY	1
#define APOB_TYPE_PIE_PWR	0xf

static void df_write_all(uint8_t fn, uint16_t reg, uint32_t value)
{
	for (unsigned int die = 0; die < zeppelin_die_count(); die++)
		zeppelin_df_write32(die, fn, reg, DIRECT, value);
}

/* ---- MMIO and I/O distribution (AmdSocSp4ZpPei, equal path) ---- */

struct df_window {
	unsigned int pair;
	uint64_t base;
	uint64_t size;
	unsigned int die;
};

struct df_plan {
	struct df_window mmio[DF_MMIO_PAIRS];
	unsigned int mmio_count;
	struct df_window io[DF_IO_PAIRS];
	unsigned int io_count;
};

static void plan_mmio(struct df_plan *plan, unsigned int pair, uint64_t base, uint64_t size,
		      unsigned int target)
{
	if (plan->mmio_count >= DF_MMIO_PAIRS || pair >= DF_MMIO_PAIRS)
		die("Too many DF MMIO windows\n");
	plan->mmio[plan->mmio_count++] = (struct df_window){ pair, base, size, target };
}

/*
 * One socket with n dies. The windows below 4 GiB split the space between TOM
 * and the PCIe configuration space and the space between the configuration
 * space and 0xfe000000 between the dies; die 0's window also covers up to
 * 0xfebfffff. Above 4 GiB, every die gets an equal part of the larger of
 * [TOM2, 0xfd_0000_0000) and [1 TiB, 0x7fd_0000_0000); the last die gets the
 * rest up to 2^48. A spare window routes the HPET range to die 0.
 */
static void df_plan(uint64_t tom, uint64_t tom2, uint64_t cfg_size, unsigned int n,
		    struct df_plan *plan)
{
	const uint64_t cfg_end = 0xf0000000 + cfg_size;
	const uint64_t above = 0xfe000000 - cfg_end;
	const uint64_t below = 0xf0000000 - tom;
	uint64_t total = above + below;
	bool above_small = false, below_small = false;
	unsigned int dies_above, left, below_used = 0, above_used = 0;
	uint64_t per, per_al, size, cursor, per_b_al = 0;
	int second = -1;

	if (cfg_end > 0xfe000000 || tom > 0xf0000000)
		die("Invalid DF MMIO inputs\n");

	if (n > 1) {
		if (above < (below / n) * 8 / 10) {
			total -= above;
			above_small = true;
		}
		if (below < (above / n) * 8 / 10) {
			total -= below;
			below_small = true;
		}
		if (above_small && below_small)
			die("No DF MMIO space for %u dies\n", n);
	}

	per = total / n;
	per_al = per & ALIGN_16M;
	if (above_small) {
		dies_above = 0;
		size = below - per_al * (n - 1);
		cursor = 0xf0000000;
	} else {
		dies_above = above / (per & 0xffffffff);
		if (above - per * dies_above > per / 2)
			dies_above++;
		dies_above = MAX(dies_above, 1);
		size = above - per_al * (dies_above - 1);
		cursor = 0xfe000000;
	}

	left = dies_above;
	for (unsigned int k = 0; k < n; k++) {
		uint64_t window;

		below_used |= BIT(k);
		if (left == 0) {
			if (below_small)
				die("DF MMIO: no below-4G space for die %u\n", k);
			if (k == dies_above) {
				const unsigned int m = n - k;

				per_b_al = (below / m) & ALIGN_16M;
				size = below - per_b_al * (m - 1);
				cursor = 0xf0000000;
			}
			if (k == dies_above + 1)
				size = per_b_al;
		} else {
			if (k == 1)
				size = per_al;
			left--;
		}
		cursor -= size;
		window = size;
		if (k == 0 && !above_small)
			window += 0xc00000;
		plan_mmio(plan, 2 * k, cursor, window, k);
	}

	if (n < 8 && ((above_small && above) || (below && (below_small || n == 1)))) {
		for (unsigned int d = 0; d < 8; d++) {
			if (!(below_used & BIT(d))) {
				second = d;
				break;
			}
		}
		if (above_small)
			plan_mmio(plan, 2 * second, cfg_end, 0xfec00000 - cfg_end, 0);
		else
			plan_mmio(plan, 2 * second, tom, below, 0);
	}

	/* Above 4 GiB, skipping the HyperTransport hole at 0xfd_0000_0000 */
	{
		const uint64_t tib = 1ULL << 40;
		uint64_t base = MAX(tom2, 4ULL * GiB);
		uint64_t limit = 0x7fdULL << 32;
		const uint64_t size1 = (base >> 32) < 0xfd ? (0xfdULL << 32) - base : 0;
		const uint64_t size2 = limit > tib ? limit - MAX(base, tib) : 0;

		if (!size1 && !size2)
			limit = 0;
		else if (size2 <= size1)
			limit = 0xfdULL << 32;
		else
			base = MAX(tom2, tib);

		if (limit > base) {
			const uint64_t span = limit - base;
			const uint64_t per_die = (span / n) & ALIGN_16M;

			for (unsigned int k = 0; k < n; k++) {
				const bool last = k == n - 1;
				const uint64_t sz = last ? span - per_die * (n - 1) : per_die;

				above_used |= BIT(k);
				if (sz >= 0x10000)
					plan_mmio(plan, 2 * k + 1, base,
						  last ? (1ULL << 48) - base : sz, k);
				base += sz;
			}
		}
	}

	/* HPET: the first unused below-4G pair, else the first unused above-4G pair */
	for (int d = 0; d < 8; d++) {
		if (!(below_used & BIT(d)) && d != second) {
			plan_mmio(plan, 2 * d, 0xfed00000, 0x10000, 0);
			break;
		}
		if ((below_used & BIT(d)) && !(above_used & BIT(d))) {
			plan_mmio(plan, 2 * d + 1, 0xfed00000, 0x10000, 0);
			break;
		}
	}

	/* I/O: die 0 gets an extra 4 KiB, the last die the rest of the 32 MiB space */
	{
		const uint32_t q = (0xf000 / n) & ~0xfffU;
		uint32_t io_cursor = 0;

		for (unsigned int k = 0; k < n; k++) {
			const uint32_t sz = k == n - 1 ? 0x2000000 - io_cursor :
				(k == 0 ? q + 0x1000 : q);

			plan->io[plan->io_count++] = (struct df_window){ k, io_cursor, sz, k };
			io_cursor += sz;
		}
	}
}

static void df_write_mmio_pair(const struct df_window *w)
{
	const uint32_t old = zeppelin_df_read32(0, 0, DF_F0_MMIO_CTRL(w->pair), DIRECT);
	const uint8_t dst = zeppelin_ioms_fabric_id(w->die);
	const uint32_t ctrl = (old & ~0xff0) | dst << 4 | 3;

	/* The vendor code keeps the component part of DstFabricID that the ABL left */
	if ((old & 3) && ((old >> 4) & 0x1f) != (dst & 0x1f))
		printk(BIOS_WARNING, "DF MMIO pair %u: replacing destination 0x%x\n", w->pair,
		       (old >> 4) & 0xff);

	df_write_all(0, DF_F0_MMIO_BASE(w->pair), (w->base + 0xffff) >> 16);
	df_write_all(0, DF_F0_MMIO_LIMIT(w->pair), (w->base + w->size - 1) >> 16);
	df_write_all(0, DF_F0_MMIO_CTRL(w->pair), ctrl);
}

static void df_write_io_pair(const struct df_window *w)
{
	const uint32_t old = zeppelin_df_read32(0, 0, DF_F0_IO_LIMIT(w->pair), DIRECT);
	const uint32_t end = w->base + w->size - 1;

	df_write_all(0, DF_F0_IO_BASE(w->pair), (w->base & 0x1fff000) | 3);
	df_write_all(0, DF_F0_IO_LIMIT(w->pair), (old & 0xfe000f00) |
		     zeppelin_ioms_fabric_id(w->die) | (end & 0x1fff000));
}

void zeppelin_df_mmio_init(void)
{
	const uint64_t tom = rdmsr(TOP_MEM_MSR).lo;
	const msr_t tom2 = rdmsr(TOP_MEM2_MSR);
	const unsigned int bus_bits = (rdmsr(MMIO_CONF_BASE).lo >> 2) & 0xf;
	struct df_plan plan = { 0 };

	df_plan(tom, (uint64_t)tom2.hi << 32 | tom2.lo, (1ULL << bus_bits) * MiB,
		zeppelin_die_count(), &plan);

	for (unsigned int i = 0; i < plan.mmio_count; i++) {
		const struct df_window *w = &plan.mmio[i];

		printk(BIOS_DEBUG, "DF MMIO pair %u: 0x%llx-0x%llx -> die %u\n", w->pair,
		       w->base, w->base + w->size - 1, w->die);
		df_write_mmio_pair(w);
	}
	for (unsigned int i = 0; i < plan.io_count; i++) {
		const struct df_window *w = &plan.io[i];

		printk(BIOS_DEBUG, "DF IO pair %u: 0x%llx-0x%llx -> die %u\n", w->pair,
		       w->base, w->base + w->size - 1, w->die);
		df_write_io_pair(w);
	}
}

/* ---- AmdFabricZpPei ---- */

static void df_clock_gating(unsigned int die, bool single_die)
{
	static const uint8_t instances[] = { 0, 1, 2, 3, 5, 12, 13, 14, 15, 16, 17, 18 };
	const bool gate = zeppelin_df_read32(die, 6, DF_F6_CLK_GATE_DC, DF_INST_IOMS) & 1;

	if (!gate && single_die) {
		zeppelin_df_write32(die, 0, DF_F0_CLK_GATE, DIRECT, 1);
		return;
	}

	for (size_t i = 0; i < ARRAY_SIZE(instances); i++)
		zeppelin_df_write32(die, 0, DF_F0_CLK_GATE, instances[i], 1);
	if (!gate)
		zeppelin_df_write32(die, 0, DF_F0_CLK_GATE, DF_INST_IOMS, 1);
}

static void df_ccm_ioms_init(unsigned int die, bool rev_a0)
{
	/* CS */
	zeppelin_df_write32(die, 2, DF_F2_CS_CFG_68, DIRECT, 0x00065019);
	zeppelin_df_write32(die, 2, DF_F2_CS_CFG_6C, DIRECT, 0x10040611);

	/* CCM */
	for (uint8_t inst = DF_INST_CCM0; inst <= DF_INST_CCM1; inst++)
		zeppelin_df_write32(die, 4, DF_F4_CCM_0C4, inst, 0x00020001);
	zeppelin_df_write32(die, 4, DF_F4_CFG_0C8, DIRECT, 0x01020000);
	for (uint8_t inst = DF_INST_CCM0; inst <= DF_INST_CCM1; inst++) {
		zeppelin_df_clrsetbits32(die, 4, DF_F4_CFG_0C0, inst, 0x3f000, 0);
		zeppelin_df_clrsetbits32(die, 3, DF_F3_CCM_104, inst, BIT(5),
					 BIT(28) | (rev_a0 ? BIT(5) : 0));
		zeppelin_df_clrsetbits32(die, 3, DF_F3_CCM_108, inst, 0, BIT(6));
	}

	/* IOMS */
	zeppelin_df_clrsetbits32(die, 3, DF_F3_IOMS_04C, DF_INST_IOMS, 0, BIT(5));
	zeppelin_df_clrsetbits32(die, 3, DF_F3_IOMS_300, DF_INST_IOMS, 0, BIT(8));
	zeppelin_df_clrsetbits32(die, 4, DF_F4_CFG_0C0, DF_INST_IOMS, 0x3f000,
				 rev_a0 ? 0x1e << 12 : 0);
}

static void df_pie_ras_init(unsigned int die)
{
	const unsigned int instances =
		zeppelin_df_read32(die, 0, DF_F0_INSTANCE_COUNT, DIRECT) & 0xff;

	/*
	 * Sync flood propagation stays enabled (vendor default). Bit 8 is the
	 * workaround for erratum 1158 (LINT0/LINT1 while the APIC is software
	 * disabled) from the revision guide; the vendor code doesn't set it.
	 */
	zeppelin_df_clrsetbits32(die, 5, DF_F5_PIE_CFG, DIRECT, PIE_CFG_DIS_SYNC_FLOOD_PROP,
				 PIE_CFG_LINT_ERRATUM_1158);

	/* Clear the error status left from before the reset */
	for (unsigned int inst = 0; inst < instances; inst++) {
		if (zeppelin_df_read32(die, 6, DF_F6_ERR_STATUS_0, inst))
			zeppelin_df_write32(die, 6, DF_F6_ERR_STATUS_0, inst, 0);
		if (zeppelin_df_read32(die, 6, DF_F6_ERR_STATUS_1, inst))
			zeppelin_df_write32(die, 6, DF_F6_ERR_STATUS_1, inst, 0);
	}

	for (uint8_t inst = 0; inst < 12; inst++)
		zeppelin_df_clrsetbits32(die, 0, DF_F0_RAS_CTRL, inst, BIT(3), 0);
}

void zeppelin_df_early_init(void)
{
	const unsigned int dies = zeppelin_die_count();
	const bool rev_a0 = zeppelin_revision() == ZEPPELIN_REV_A0;

	for (unsigned int die = 0; die < dies; die++) {
		df_clock_gating(die, dies == 1);
		df_ccm_ioms_init(die, rev_a0);
		df_pie_ras_init(die);
	}
	/*
	 * The vendor code also programs the DRAM scrubber here from the APOB
	 * channel population. It is not needed to boot and is not done yet.
	 */
}

/* ---- AmdFabricZpDxe ---- */

/* PIE power management, written at the DXE entry of the vendor code */
void zeppelin_df_late_init(void)
{
	const unsigned int dies = zeppelin_die_count();
	const unsigned int sockets =
		((zeppelin_df_read32(0, 1, DF_F1_SYSTEM_CFG, DIRECT) >> 5) & 1) + 1;
	const bool multi_die = dies / sockets > 1;
	const uint8_t rev = zeppelin_revision();
	uint8_t apob_flag = 0;

	/* APOB group 1 type 0xf, byte 0x130 (meaning unknown), APOB version 9 and up */
	zeppelin_apob_read8(zeppelin_apob(), APOB_GROUP_MEMORY, APOB_TYPE_PIE_PWR, 0x130,
			    9, &apob_flag);
	apob_flag = !!apob_flag;

	for (unsigned int die = 0; die < dies; die++) {
		uint32_t v;

		/* Bit 5 stays clear with C6 enabled (vendor PCD 0x13 = 1) */
		v = zeppelin_df_read32(die, 5, DF_F5_PIE_PWR_300, DIRECT);
		v &= ~(BIT(5) | BIT(12) | BIT(14) | BIT(15));
		v |= (apob_flag ? BIT(5) : 0) | BIT(13);
		if (!(dies > 1 && (rev & (ZEPPELIN_REV_A0 | ZEPPELIN_REV_B0 | ZEPPELIN_REV_B1))))
			v |= BIT(12);
		v |= (multi_die ? BIT(14) : 0) | (sockets > 1 ? BIT(15) : 0);
		zeppelin_df_write32(die, 5, DF_F5_PIE_PWR_300, DIRECT, v);

		v = zeppelin_df_read32(die, 5, DF_F5_PIE_PWR_358, DIRECT);
		v = (v & 0x0f000000) | 0x80e000e0 | (multi_die ? 0x0000e000 : 0);
		zeppelin_df_write32(die, 5, DF_F5_PIE_PWR_358, DIRECT, v);

		v = zeppelin_df_read32(die, 5, DF_F5_PIE_PWR_304, DIRECT);
		v = (v & ~0x420) | 0x3c0 | ((sockets == 1 || apob_flag) ? BIT(10) : 0) |
			(apob_flag ? BIT(5) : 0);
		zeppelin_df_write32(die, 5, DF_F5_PIE_PWR_304, DIRECT, v);

		if (dies > 1) {
			v = zeppelin_df_read32(die, 5, DF_F5_PIE_PWR_354, DIRECT);
			v = (v & 0xffff0000) | (sockets > 1 ? 0x7100 : 0) |
				(multi_die ? 0x46 : 0);
			zeppelin_df_write32(die, 5, DF_F5_PIE_PWR_354, DIRECT, v);
		}
	}

	/* Route the legacy VGA ranges to the IOMS of die 0 */
	for (unsigned int die = 0; die < dies; die++)
		zeppelin_df_clrsetbits32(die, 0, DF_F0_VGA_EN, DIRECT, 0xff4,
					 zeppelin_ioms_fabric_id(0) << 4 | 1);
}

/* ReadyToBoot in the vendor code: "Disable extended configuration access" */
void zeppelin_df_disable_ext_cfg(void)
{
	for (unsigned int die = 0; die < zeppelin_die_count(); die++)
		zeppelin_df_clrsetbits32(die, 4, DF_F4_EXT_CFG, DIRECT, BIT(0), 0);
}
