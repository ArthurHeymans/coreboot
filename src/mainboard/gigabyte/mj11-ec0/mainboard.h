/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef MAINBOARD_GIGABYTE_MJ11_EC0_H
#define MAINBOARD_GIGABYTE_MJ11_EC0_H

#include <amdblocks/acpi.h>
#include <amdblocks/acpimmio.h>
#include <amdblocks/acpimmio_map.h>
#include <device/mmio.h>
#include <types.h>

/* VR controllers, riser EEPROM and muxes */
#define MJ11_I2C_BUS	3

#define MJ11_BMC_KCS_PORT	0xca2

#define MJ11_GPIO_BANK(pin)	(ACPIMMIO_BASE(GPIO0) + 4 * (pin))
#define  MJ11_GPIO_PIN_STS	BIT(16)

void mainboard_program_gpios(void);

static inline void mj11_iomux_write(unsigned int pin, uint8_t function)
{
	write8p(ACPIMMIO_BASE(IOMUX) + pin, function);
}

/*
 * The PM1 write of the vendor AmdCpmOemInitPeim and SMBusSwitch: write back
 * all pending PM1 status bits plus 15:14 (all W1C) and clear PCIEXP_WAKE_DIS.
 */
static inline void mj11_clear_pm1(void)
{
	acpi_write16(MMIO_ACPI_PM1_STS, acpi_read16(MMIO_ACPI_PM1_STS) | 0xc000);
	acpi_write16(MMIO_ACPI_PM1_EN, acpi_read16(MMIO_ACPI_PM1_EN) & ~BIT(14));
}

#endif
