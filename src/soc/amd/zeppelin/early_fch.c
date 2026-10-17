/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/acpimmio.h>
#include <amdblocks/lpc.h>
#include <amdblocks/pmlib.h>
#include <amdblocks/smbus.h>
#include <amdblocks/spi.h>
#include <soc/southbridge.h>
#include <types.h>

/* Before console init */
void fch_pre_init(void)
{
	/* The GPIO and PM registers are accessed through the ACPI MMIO window */
	enable_acpimmio_decode_pm04();
	lpc_early_init();

	if (CONFIG(POST_IO) && (CONFIG_POST_IO_PORT == 0x80))
		lpc_enable_port80();

	/*
	 * The SPI read mode and speeds stay as the PSP left them. The vendor
	 * firmware doesn't change them either.
	 */
	fch_smbus_init();
	fch_enable_cf9_io();
	fch_enable_legacy_io();

	if (CONFIG(DISABLE_KEYBOARD_RESET_PIN))
		fch_disable_kb_rst();
}

/* After console init */
void fch_early_init(void)
{
	pm_set_power_failure_state();
	fch_print_pmxc0_status();
	show_spi_speeds_and_modes();

	if (CONFIG(DISABLE_SPI_FLASH_ROM_SHARING))
		lpc_disable_spi_rom_sharing();
}
