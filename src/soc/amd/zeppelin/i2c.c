/* SPDX-License-Identifier: GPL-2.0-only */

#include <device/mmio.h>
#include <drivers/i2c/designware/dw_i2c.h>
#include <soc/i2c.h>
#include <types.h>

/*
 * The FCH DesignWare I2C controllers, numbered like the vendor AmdI2CMasterPei
 * instances. The vendor code doesn't power them up through AOAC or touch the
 * pads: it programs IC_CON = 0x63 (master, standard speed, restart), the SCL
 * counts below and IC_FS_SPKLEN = 7. 0x285 + 0x357 cycles per bit at 100 kHz
 * means a 150 MHz controller clock.
 */
static const uintptr_t i2c_base[ZEPPELIN_I2C_COUNT] = {
	0xfedc2000, 0xfedc3000, 0xfedc4000, 0xfedc5000, 0xfedc6000, 0xfedcb000,
};

static const struct dw_i2c_bus_config i2c_cfg = {
	.speed = I2C_SPEED_STANDARD,
	.speed_config[0] = {
		.speed = I2C_SPEED_STANDARD,
		.scl_hcnt = 0x285,
		.scl_lcnt = 0x357,
	},
};

#define IC_FS_SPKLEN	0xa0

uintptr_t dw_i2c_base_address(unsigned int bus)
{
	return bus < ARRAY_SIZE(i2c_base) ? i2c_base[bus] : 0;
}

const struct dw_i2c_bus_config *dw_i2c_get_soc_cfg(unsigned int bus)
{
	return bus < ARRAY_SIZE(i2c_base) ? &i2c_cfg : NULL;
}

enum cb_err zeppelin_i2c_init(unsigned int bus)
{
	if (dw_i2c_init(bus, &i2c_cfg) != CB_SUCCESS)
		return CB_ERR;

	/* The controller is still disabled here, so the spike length can be set */
	write32p(i2c_base[bus] + IC_FS_SPKLEN, 7);
	return CB_SUCCESS;
}
