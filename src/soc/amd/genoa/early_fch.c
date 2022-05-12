/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/acpimmio.h>
#include <amdblocks/pmlib.h>
#include <soc/southbridge.h>
#include <soc/uart.h>

/* Before console init */
void fch_pre_init(void)
{
	/* Enable_acpimmio_decode_pm04 to enable the ACPIMMIO decode which is needed to access
	   the GPIO registers. */
	enable_acpimmio_decode_pm04();

	enable_aoac_devices();
	/*
	 * On reset Range_0 defaults to enabled. We want to start with a clean
	 * slate to not have things unexpectedly enabled.
	 */
	clear_uart_legacy_config();

	if (CONFIG(AMD_SOC_CONSOLE_UART))
		set_uart_config(CONFIG_UART_FOR_CONSOLE);

}

/* After console init */
void fch_early_init(void)
{

}
