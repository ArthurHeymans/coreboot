/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/acpimmio.h>
#include <console/console.h>
#include <delay.h>
#include <device/i2c_simple.h>
#include <soc/i2c.h>
#include <soc/native.h>
#include <types.h>

#include "board_tables.h"
#include "mainboard.h"

#define VR_MUX_A	0x73
#define VR_MUX_B	0x71
#define VR_WRITE_TRIES	3

/* The vendor VR code waits 10 us after every successful VR register access */
static int vr_read(uint8_t addr, uint8_t reg, uint8_t *value)
{
	const int ret = i2c_readb(MJ11_I2C_BUS, addr, reg, value);

	if (!ret)
		udelay(10);
	return ret;
}

static int vr_write(uint8_t addr, uint8_t reg, uint8_t value)
{
	const int ret = i2c_writeb(MJ11_I2C_BUS, addr, reg, value);

	if (!ret)
		udelay(10);
	return ret;
}

static int vr_apply(const struct mj11_vr_image *vr)
{
	for (size_t i = 0; i < vr->count; i++) {
		const struct mj11_vr_reg *r = &vr->regs[i];
		const uint8_t want = r->value & r->mask;
		uint8_t value;
		int tries = 0;

		if (vr_read(vr->addr, r->reg, &value))
			return -1;
		while ((value & r->mask) != want) {
			if (tries++ == VR_WRITE_TRIES)
				return -1;
			if (vr_write(vr->addr, r->reg, want) ||
			    vr_read(vr->addr, r->reg, &value))
				return -1;
		}
	}
	return 0;
}

/*
 * GbtIRPower: load the register images of the two VR controllers on I2C3
 * behind the PCA954x at 0x71. The mux writes are byte writes to "register" 0,
 * as in the vendor code. Errors of the lock writes are ignored there, too.
 */
static void program_vrs(void)
{
	int ret = 0;

	i2c_writeb(MJ11_I2C_BUS, VR_MUX_A, 0, 0);
	for (size_t i = 0; i < mj11_vr_image_count && !ret; i++) {
		const struct mj11_vr_image *vr = &mj11_vr_images[i];

		if (i2c_writeb(MJ11_I2C_BUS, VR_MUX_B, 0, BIT(vr->mux_channel)))
			continue;
		vr_write(vr->addr, vr->lock_reg, 0);
		ret = vr_apply(vr);
		if (!ret)
			vr_write(vr->addr, vr->lock_reg, 7);
		printk(ret ? BIOS_ERR : BIOS_DEBUG, "VR 0x%02x: %s\n", vr->addr,
		       ret ? "programming failed" : "programmed");
	}
	i2c_writeb(MJ11_I2C_BUS, VR_MUX_B, 0, BIT(0));
}

static bool is_i2c_mux(uint8_t addr)
{
	return addr >= 0x70 && addr <= 0x77;	/* PCA954x */
}

/*
 * F09 GbtReDriverPei: ordered mux selection and external lane tuning. The
 * re-drivers sit on the riser and backplane of the G431 chassis; without them
 * the downstream mux doesn't answer. Skip the re-driver accesses behind a mux
 * that failed to switch instead of timing out on every one of them.
 */
static void program_ec1_redrivers(void)
{
	unsigned int failures = 0, skipped = 0;
	bool path_ok = true;

	for (size_t i = 0; i < mj11_ec1_redriver_i2c_count; i++) {
		const struct mj11_i2c_rmw *r = &mj11_ec1_redriver_i2c[i];
		const bool mux = is_i2c_mux(r->addr);
		uint8_t value;
		bool ok;

		if (!mux && !path_ok) {
			skipped++;
			continue;
		}
		ok = !i2c_readb(MJ11_I2C_BUS, r->addr, r->reg, &value) &&
		     !i2c_writeb(MJ11_I2C_BUS, r->addr, r->reg,
				 (value & r->and_mask) | r->or_mask);
		if (!ok)
			failures++;
		/* Only a channel selection (a nonzero value) opens a path */
		if (mux && r->or_mask)
			path_ok = ok;
	}
	if (failures || skipped)
		printk(BIOS_INFO, "EC1 external re-drivers: %u accesses failed, %u skipped\n",
		       failures, skipped);
}

void mainboard_fch_reset_done(void)
{
	/* I2C3 on GPIO19/20 */
	mj11_iomux_write(19, 1);
	mj11_iomux_write(20, 1);
	if (zeppelin_i2c_init(MJ11_I2C_BUS) == CB_SUCCESS) {
		program_vrs();
		if (CONFIG(BOARD_GIGABYTE_MJ11_EC1))
			program_ec1_redrivers();
	} else
		printk(BIOS_ERR, "I2C%u setup failed, VRs not programmed\n", MJ11_I2C_BUS);

	mj11_clear_pm1();
}
