/* SPDX-License-Identifier: GPL-2.0-only */

#include <console/console.h>
#include <device/i2c_simple.h>
#include <device/mmio.h>
#include <soc/i2c.h>
#include <soc/platform_descriptors.h>
#include <types.h>
#include <variant/dxio.h>

#include "mainboard.h"

/* Bifurcation modes of the vendor FUN_ffe08bf2 */
enum riser_mode {
	RISER_X4X4X4X4 = 0,
	RISER_X4X4 = 1,		/* lanes 23..16 unused */
	RISER_X8X4X4 = 2,
	RISER_X8X8 = 3,
	RISER_DEFAULT = 7,	/* enable engine 6, retaining the template's lanes */
	RISER_KEEP = 0xf,	/* retain the board-specific template */
};

static const dxio_descriptor mj11_x8x8[] = {
	MJ11_FIXED_ENGINES,
	MJ11_PCIE(31, 24, 3, 1),
	MJ11_PCIE(23, 16, 3, 2),
};

static const dxio_descriptor mj11_x8x4x4[] = {
	MJ11_FIXED_ENGINES,
	MJ11_PCIE(31, 24, 3, 1),
	MJ11_PCIE(23, 20, 3, 3),
	MJ11_PCIE(19, 16, 3, 4),
};

static const dxio_descriptor mj11_x4x4[] = {
	MJ11_FIXED_ENGINES,
	MJ11_PCIE(31, 28, 3, 1),
	MJ11_PCIE(27, 24, 3, 2),
};

static const dxio_descriptor mj11_x4x4x4x4[] = {
	MJ11_FIXED_ENGINES,
	MJ11_PCIE(31, 28, 3, 1),
	MJ11_PCIE(27, 24, 3, 2),
	MJ11_PCIE(23, 20, 3, 3),
	MJ11_PCIE(19, 16, 3, 4),
};

#define RISER_MUX		0x74	/* PCA9545 */
#define RISER_MUX_CHANNEL	BIT(2)
#define RISER_EEPROM		0x57
#define RISER_EEPROM_ID		0x11

/* FUN_ffe08ace: the ID byte of the riser EEPROM behind channel 2 of the mux */
static bool riser_id_from_eeprom(uint8_t *id)
{
	if (zeppelin_i2c_init(MJ11_I2C_BUS) != CB_SUCCESS ||
	    i2c_writeb(MJ11_I2C_BUS, RISER_MUX, 0, RISER_MUX_CHANNEL) ||
	    i2c_readb(MJ11_I2C_BUS, RISER_EEPROM, 1, id) ||	/* discarded */
	    i2c_readb(MJ11_I2C_BUS, RISER_EEPROM, RISER_EEPROM_ID, id))
		return false;

	if (*id == 1)
		*id = 2;
	else if (*id == 2)
		*id = 1;
	else if (*id == 4)
		*id = 7;
	return true;
}

static bool gpio_high(unsigned int pin)
{
	return read32p(MJ11_GPIO_BANK(pin)) & MJ11_GPIO_PIN_STS;
}

/* FUN_ffe08b3e: the riser ID straps; 0 means no riser */
static bool riser_id_from_gpios(uint8_t *id)
{
	*id = gpio_high(152) << 3 | gpio_high(151) << 2 | gpio_high(4) << 1 | gpio_high(3);
	return *id != 0;
}

/* FUN_ffe08ba8 */
static enum riser_mode riser_mode_from_id(uint8_t id)
{
	switch (id) {
	case 0: case 4: case 5: case 8: case 0xc: case 0xd:
		return RISER_X4X4X4X4;
	case 1: case 9: case 0x10:
		return RISER_X4X4;
	case 2: case 6: case 0xa: case 0xe: case 0x11:
		return RISER_X8X4X4;
	case 3: case 0xb:
		return RISER_X8X8;
	default:
		return RISER_KEEP;
	}
}

static enum riser_mode riser_mode(void)
{
	uint8_t id;

	if (riser_id_from_eeprom(&id)) {
		printk(BIOS_INFO, "Riser EEPROM ID 0x%02x\n", id);
		return riser_mode_from_id(id);
	}
	if (riser_id_from_gpios(&id)) {
		printk(BIOS_INFO, "Riser strap ID 0x%x\n", id);
		return riser_mode_from_id(id);
	}
	return RISER_DEFAULT;
}

/* SP4r2: one die */
void mainboard_get_dxio_descriptors(unsigned int die, const dxio_descriptor **dxio,
				    size_t *count)
{
	if (die) {
		*dxio = NULL;
		*count = 0;
		return;
	}

	switch (riser_mode()) {
	case RISER_X4X4X4X4:
		*dxio = mj11_x4x4x4x4;
		*count = ARRAY_SIZE(mj11_x4x4x4x4);
		break;
	case RISER_X4X4:
		*dxio = mj11_x4x4;
		*count = ARRAY_SIZE(mj11_x4x4);
		break;
	case RISER_X8X4X4:
		*dxio = mj11_x8x4x4;
		*count = ARRAY_SIZE(mj11_x8x4x4);
		break;
	case RISER_X8X8:
		*dxio = mj11_x8x8;
		*count = ARRAY_SIZE(mj11_x8x8);
		break;
	default:
		variant_default_dxio(dxio, count);
		break;
	}
}
