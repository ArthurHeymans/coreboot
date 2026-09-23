/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/data_fabric.h>
#include <arch/ioapic.h>
#include <console/console.h>
#include <cpu/amd/mtrr.h>
#include <soc/data_fabric.h>
#include <soc/soc_util.h>
#include <types.h>

#include "native.h"

#define DF_F0_SCRUB_COUNT		DF_REG_ID(0, 0x040)
#define DF_F0_SCRUB_CTRL		DF_REG_ID(0, 0x3f8)
#define  DF_F0_SCRUB_CTRL_BIT3		BIT(3)
#define DF_F6_ERR_STATUS_0		DF_REG_ID(6, 0x1f0)
#define DF_F6_ERR_STATUS_1		DF_REG_ID(6, 0x1f4)

#define DF_F5_CSTATE_CTRL		DF_REG_ID(5, 0x300)
#define  DF_F5_CSTATE_CTRL_BIT5		BIT(5)
#define  DF_F5_CSTATE_CTRL_BIT11	BIT(11)
#define  DF_F5_CSTATE_CTRL_PWRDN	0x3b00

/*
 * Registers without a public name: DF_F<function>_REG<offset>. The values written
 * to them are the ones the FSP programs.
 */
#define DF_F0_REG3F0	DF_REG_ID(0, 0x3f0)
#define DF_F2_REG040	DF_REG_ID(2, 0x040)
#define DF_F2_REG044	DF_REG_ID(2, 0x044)
#define DF_F2_REG048	DF_REG_ID(2, 0x048)
#define DF_F2_REG050	DF_REG_ID(2, 0x050)
#define DF_F2_REG068	DF_REG_ID(2, 0x068)
#define DF_F2_REG06C	DF_REG_ID(2, 0x06c)
#define DF_F3_REG04C	DF_REG_ID(3, 0x04c)
#define DF_F3_REG104	DF_REG_ID(3, 0x104)
#define DF_F3_REG108	DF_REG_ID(3, 0x108)
#define DF_F3_REG20C	DF_REG_ID(3, 0x20c)
#define DF_F3_REG300	DF_REG_ID(3, 0x300)
#define DF_F4_REG0C0	DF_REG_ID(4, 0x0c0)
#define DF_F4_REG0C4	DF_REG_ID(4, 0x0c4)
#define DF_F4_REG0C8	DF_REG_ID(4, 0x0c8)
#define DF_F5_REG338	DF_REG_ID(5, 0x338)
#define DF_F5_REG348	DF_REG_ID(5, 0x348)
#define DF_F5_REG34C	DF_REG_ID(5, 0x34c)
#define DF_F5_REG350	DF_REG_ID(5, 0x350)
#define DF_F5_REG360	DF_REG_ID(5, 0x360)
#define DF_F5_REG364	DF_REG_ID(5, 0x364)

/* Route the MMIO between TOM and the IOAPIC and all I/O ports to the IOMS */
void picasso_df_mmio_init(void)
{
	const uint32_t mmio_base = get_top_of_mem_below_4gb();
	const uint32_t mmio_limit = IO_APIC_ADDR - 1;
	union df_mmio_control ctrl;
	int free_mmio = -1;
	int free_io = -1;
	bool have_mmio = false;
	bool have_io = false;

	for (unsigned int i = 0; i < DF_MMIO_REG_SET_COUNT; i++) {
		uint64_t base, limit;

		ctrl.raw = data_fabric_broadcast_read32(DF_MMIO_CONTROL(i));
		if (!ctrl.re && !ctrl.we && free_mmio < 0)
			free_mmio = i;
		if (!ctrl.re || !ctrl.we || ctrl.np || ctrl.dst_fabric_id != IOMS0_FABRIC_ID)
			continue;
		base = (uint64_t)data_fabric_broadcast_read32(DF_MMIO_BASE(i)) << DF_MMIO_SHIFT;
		limit = (((uint64_t)data_fabric_broadcast_read32(DF_MMIO_LIMIT(i)) + 1) <<
			 DF_MMIO_SHIFT) - 1;
		if (base <= mmio_base && limit >= mmio_limit)
			have_mmio = true;
	}

	for (unsigned int i = 0; i < DF_IO_REG_COUNT; i++) {
		const union df_io_base base = {
			.raw = data_fabric_broadcast_read32(DF_IO_BASE(i)),
		};
		const union df_io_limit limit = {
			.raw = data_fabric_broadcast_read32(DF_IO_LIMIT(i)),
		};

		if (!base.re && !base.we && free_io < 0)
			free_io = i;
		if (base.re && base.we && limit.dst_fabric_id == IOMS0_FABRIC_ID &&
		    base.io_base == 0 && limit.io_limit >= (0xffff >> DF_IO_ADDR_SHIFT))
			have_io = true;
	}

	if (!have_mmio) {
		if (free_mmio < 0 || !mmio_base || mmio_base > mmio_limit)
			die("No Data Fabric MMIO window available\n");
		data_fabric_broadcast_write32(DF_MMIO_CONTROL(free_mmio), 0);
		data_fabric_broadcast_write32(DF_MMIO_BASE(free_mmio), mmio_base >> DF_MMIO_SHIFT);
		data_fabric_broadcast_write32(DF_MMIO_LIMIT(free_mmio), mmio_limit >> DF_MMIO_SHIFT);
		ctrl = (union df_mmio_control) {
			.re = 1,
			.we = 1,
			.dst_fabric_id = IOMS0_FABRIC_ID,
		};
		data_fabric_broadcast_write32(DF_MMIO_CONTROL(free_mmio), ctrl.raw);
	}

	if (!have_io) {
		const union df_io_base base = { .re = 1, .we = 1 };
		const union df_io_limit limit = {
			.dst_fabric_id = IOMS0_FABRIC_ID,
			.io_limit = 0xffff >> DF_IO_ADDR_SHIFT,
		};

		if (free_io < 0)
			die("No Data Fabric IO window available\n");
		data_fabric_broadcast_write32(DF_IO_LIMIT(free_io), limit.raw);
		data_fabric_broadcast_write32(DF_IO_BASE(free_io), base.raw);
	}

	/* Route the legacy VGA ranges to the IOMS for the VGA option ROM */
	data_fabric_broadcast_write32(D18F0_VGAEN, (IOMS0_FABRIC_ID << 4) | VGA_ADDR_ENABLE);
}

static void df_clear_error_status(uint8_t instance, uint16_t status_reg)
{
	if (data_fabric_read32(status_reg, instance))
		data_fabric_write32(status_reg, instance, 0);
}

/* Clear the error status left from before the reset */
static void df_clear_errors(void)
{
	const uint32_t instances = data_fabric_read32(DF_F0_SCRUB_COUNT, BROADCAST_FABRIC_ID) &
		0xff;

	for (uint32_t instance = 0; instance < instances; instance++) {
		df_clear_error_status(instance, DF_F6_ERR_STATUS_0);
		df_clear_error_status(instance, DF_F6_ERR_STATUS_1);
	}

	data_fabric_clrsetbits32(DF_F0_SCRUB_CTRL, BROADCAST_FABRIC_ID,
				 DF_F0_SCRUB_CTRL_BIT3, 0);
}

void picasso_df_early_init(void)
{
	unsigned int dram_ranges = 0;

	data_fabric_write32(DF_F0_REG3F0, BROADCAST_FABRIC_ID, 1);

	for (uint8_t instance = 0; instance < 2; instance++) {
		data_fabric_clrsetbits32(DF_F2_REG040, instance, 0, BIT(30));
		data_fabric_clrsetbits32(DF_F2_REG044, instance, BIT(26), 0);
		data_fabric_clrsetbits32(DF_F2_REG050, instance, 0, BIT(5));
		data_fabric_clrsetbits32(DF_F2_REG048, instance, 0, 0x8040);
	}
	data_fabric_write32(DF_F2_REG068, BROADCAST_FABRIC_ID, 0x65019);
	data_fabric_write32(DF_F2_REG06C, BROADCAST_FABRIC_ID, 0x14060619);

	for (uint8_t instance = 0; instance < 2; instance++) {
		if (!(data_fabric_read32(DF_DRAM_BASE0, instance) & DRAM_BASE_REG_VALID))
			data_fabric_clrsetbits32(DF_F2_REG048, instance, BIT(19), 0);
	}
	data_fabric_clrsetbits32(DF_F2_REG050, BROADCAST_FABRIC_ID, BIT(0), 0);

	data_fabric_write32(DF_F4_REG0C4, 2, 0x20001);
	data_fabric_write32(DF_F4_REG0C8, BROADCAST_FABRIC_ID, 0x1020000);
	data_fabric_clrsetbits32(DF_F3_REG104, 2, 0, BIT(28));
	data_fabric_clrsetbits32(DF_F3_REG108, 2, 0, BIT(6));
	data_fabric_write32(DF_F3_REG20C, BROADCAST_FABRIC_ID, 3);

	for (uint8_t instance = 0; instance < 2; instance++) {
		if ((data_fabric_read32(DF_DRAM_BASE0, instance) & DRAM_BASE_REG_VALID) ||
		    (data_fabric_read32(DF_DRAM_BASE0 + 8, instance) & DRAM_BASE_REG_VALID))
			dram_ranges++;
	}
	/* Programmed by the FSP depending on the number of populated DRAM channels */
	data_fabric_clrsetbits32(DF_F4_REG0C0, 6, 0xff0, (dram_ranges == 2 ? 0x48 : 0x30) << 4);

	data_fabric_clrsetbits32(DF_F3_REG04C, 9, 0, BIT(5));
	data_fabric_clrsetbits32(DF_F3_REG300, 9, 0, BIT(8));

	df_clear_errors();
}

void picasso_df_late_init(void)
{
	const bool raven2 = get_silicon_type() == SILICON_RV2;
	uint32_t value;

	if (get_silicon_type() == SILICON_UNKNOWN) {
		printk(BIOS_WARNING, "Unknown silicon, skipping the late DF setup\n");
		return;
	}

	value = data_fabric_broadcast_read32(DF_F5_CSTATE_CTRL);
	value &= ~DF_F5_CSTATE_CTRL_BIT5;
	value |= DF_F5_CSTATE_CTRL_PWRDN;
	if (raven2)
		value &= ~DF_F5_CSTATE_CTRL_BIT11;
	data_fabric_broadcast_write32(DF_F5_CSTATE_CTRL, value);

	data_fabric_clrsetbits32(DF_F5_REG338, BROADCAST_FABRIC_ID, 0, BIT(3));
	data_fabric_clrsetbits32(DF_F5_REG364, BROADCAST_FABRIC_ID, 0, BIT(1));
	data_fabric_clrsetbits32(DF_F5_REG348, BROADCAST_FABRIC_ID, 0x7f7f7f7f, 0x14171710);
	data_fabric_clrsetbits32(DF_F5_REG34C, BROADCAST_FABRIC_ID, 0x00ff0000, 0x00590000);
	data_fabric_clrsetbits32(DF_F5_REG360, BROADCAST_FABRIC_ID, BIT(21),
				 raven2 ? BIT(0) : 0);
	if (raven2)
		data_fabric_clrsetbits32(DF_F5_REG350, BROADCAST_FABRIC_ID, 0xff000000,
				0x0e000000);
}
