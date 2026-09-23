/* SPDX-License-Identifier: GPL-2.0-only */

#include <acpi/acpi.h>
#include <console/console.h>
#include <device/device.h>
#include <soc/southbridge.h>
#include <static.h>
#include <types.h>

#include "fch.h"
#include "native.h"
#include "nbio.h"

static void silicon_resume(void)
{
	if (picasso_restore_cpu_topology_policy() != CB_SUCCESS)
		die("Failed to restore the CPU topology\n");

	picasso_nbio_tom_init();
	picasso_nbio_ioapic_init();
	if (picasso_dxio_init() != CB_SUCCESS)
		die("DXIO initialization failed\n");
	if (picasso_nbio_resume_init() != CB_SUCCESS)
		die("NBIO resume failed\n");
	if (picasso_fch_usb_init() != CB_SUCCESS)
		die("USB initialization failed\n");
}

void picasso_native_silicon_init(void)
{
	if (acpi_is_wakeup_s3()) {
		printk(BIOS_INFO, "Native silicon resume\n");
		silicon_resume();
		return;
	}

	printk(BIOS_INFO, "Native silicon initialization (ramstage)\n");

	/* Without a GPU the GFX and display audio devices only have a dummy function */
	if (!picasso_gfx_available()) {
		DEV_PTR(gfx)->enabled = false;
		DEV_PTR(gfx_hda)->enabled = false;
	}

	picasso_nbio_ioapic_init();
	if (picasso_dxio_init() != CB_SUCCESS)
		die("DXIO initialization failed\n");
	if (picasso_nbio_pre_enumeration_init() != CB_SUCCESS)
		die("NBIO initialization failed\n");

	picasso_fch_ab_init();
	fch_init_acpi_decode();
	picasso_fch_legacy_init();
	picasso_fch_lpc_init();
	picasso_fch_clock_init();

	if (is_dev_enabled(DEV_PTR(sata)) && picasso_fch_sata_init() != CB_SUCCESS)
		die("SATA initialization failed\n");

	if (picasso_fch_usb_init() != CB_SUCCESS)
		die("USB initialization failed\n");

	picasso_df_late_init();

	picasso_fch_emmc_init();
}
