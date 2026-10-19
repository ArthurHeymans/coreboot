/* SPDX-License-Identifier: GPL-2.0-only */

#include <bootstate.h>
#include <console/console.h>
#include <device/device.h>
#include <device/pci_ops.h>
#include <soc/southbridge.h>
#include <static.h>
#include <types.h>

#include "fch.h"
#include "native.h"
#include "nbio.h"
#include "smu.h"

void __weak mainboard_dxio_training_done(void) {}

void zeppelin_native_silicon_init(void)
{
	printk(BIOS_INFO, "Native silicon initialization (ramstage)\n");

	zeppelin_nbio_ioapic_init();
	zeppelin_psp_cache_bars();

	/* AmdNbioPcieZPPei */
	if (zeppelin_dxio_init() != CB_SUCCESS)
		die("DXIO initialization failed\n");
	mainboard_dxio_training_done();

	/* FchTaishanPei USB, then the DXE drivers */
	zeppelin_fch_usb_init(0);
	if (zeppelin_smu_init() != CB_SUCCESS)
		printk(BIOS_ERR, "SMU initialization failed\n");
	zeppelin_ccx_get_cac_weights();
	zeppelin_df_late_init();
	zeppelin_nbio_pre_enumeration_init();

	fch_init_acpi_decode();
	zeppelin_fch_env_init();
	if (is_dev_enabled(DEV_PTR(sata)))
		zeppelin_fch_sata_init(0);
}

/* ReadyToBoot in the vendor firmware */
static void zeppelin_ready_to_boot(void *unused)
{
	const struct device *sata = DEV_PTR(sata);

	if (is_dev_enabled(sata) && sata->upstream)
		zeppelin_fch_sata_late(0, PCI_BDF(sata));
	zeppelin_fch_late_init();
	zeppelin_df_disable_ext_cfg();
	zeppelin_psp_lock_df();
}

BOOT_STATE_INIT_ENTRY(BS_PAYLOAD_LOAD, BS_ON_EXIT, zeppelin_ready_to_boot, NULL);
