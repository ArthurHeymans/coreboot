/* SPDX-License-Identifier: GPL-2.0-only */

#include <bootstate.h>
#include <console/console.h>
#include <soc/southbridge.h>
#include "fch.h"
#include "native.h"
#include "nbio.h"
#include "smu.h"

void __weak mainboard_dxio_training_done(void) {}

void sp3_native_silicon_init(void)
{
	printk(BIOS_INFO, "SP3 native silicon initialization\n");
	sp3_nbio_ioapic_init();
	sp3_psp_cache_bars();
	if (sp3_smu_init())
		die("SP3 SMU PPTable initialization failed\n");
	sp3_fch_usb_init(0);
	if (sp3_dxio_init())
		die("SP3 DXIO initialization failed\n");
	mainboard_dxio_training_done();
	sp3_ccx_get_cac_weights();
	sp3_df_late_init();
	fch_init_acpi_decode();
	sp3_fch_env_init();
	sp3_fch_sata_init(0);
}

static void ready_to_boot(void *unused)
{
	sp3_fch_sata_late(0, 0);
	sp3_fch_late_init();
	sp3_df_disable_ext_cfg();
	sp3_psp_lock_df();
}

BOOT_STATE_INIT_ENTRY(BS_PAYLOAD_LOAD, BS_ON_EXIT, ready_to_boot, NULL);
