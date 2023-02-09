/* SPDX-License-Identifier: GPL-2.0-only */

#include <assert.h>
#include <bootstate.h>
#include <cbmem.h>
#include <cpu/cpu.h>
#include <Sil-api.h>
#include <SilCommon.h>
#include <xSIM-api.h>
#include <DF/RcManager-api.h>
#include <FCH/Common/FchCommonCfg.h>
#include <amdblocks/reset.h>
#include <soc/soc_chip.h>
#include "console.h"
#include "opensil.h"

void SIL_STATUS_report(const char *function, const int status)
{
	const int log_level = status == SilPass ? BIOS_DEBUG : BIOS_ERR;
	const char *error_string = "Unkown error";

	const struct error_string {
		SIL_STATUS status;
		const char *string;
	} errors[] = {
		{SilPass, "SilPass"},
		{SilUnsupportedHardware, "SilUnsupportedHardware"},
		{SilUnsupported, "SilUnsupported"},
		{SilInvalidParameter, "SilInvalidParameter"},
		{SilAborted, "SilAborted"},
		{SilOutOfResources, "SilOutOfResources"},
		{SilNotFound, "SilNotFound"},
		{SilOutOfBounds, "SilOutOfBounds"},
		{SilDeviceError, "SilDeviceError"},
		{SilResetRequestColdImm, "SilResetRequestColdImm"},
		{SilResetRequestColdDef, "SilResetRequestColdDef"},
		{SilResetRequestWarmImm, "SilResetRequestWarmImm"},
		{SilResetRequestWarmDef, "SilResetRequestWarmDef"},
	};

	int i;
	for (i = 0; i < ARRAY_SIZE(errors); i++) {
		if (errors[i].status == status)
			error_string = errors[i].string;
	}
	printk(log_level, "%s returned %d (%s)\n", function, status, error_string);
}

static void setup_rc_manager_default(void)
{
	DF4_RCMGR_INPUT_BLK *rc_mgr_input_block = SilFindStructure(SilId_RcManager,  0);
	rc_mgr_input_block->SetRcBasedOnNv = false;

	// Hacky: This should be done in opensil which knows about sockets and RbPerSocket. Not host
	rc_mgr_input_block->SocketNumber = 1;
	rc_mgr_input_block->RbsPerSocket = 4;
	rc_mgr_input_block->McptEnable = true;
	// This should be moved to opensil. It has functions for this already.
	rc_mgr_input_block->PciExpressBaseAddress = CONFIG_ECAM_MMCONF_BASE_ADDRESS;
	rc_mgr_input_block->BottomMmioReservedForPrimaryRb = 4ull * GiB - 32 * MiB;
	rc_mgr_input_block->MmioSizePerRbForNonPciDevice = 1 * MiB;
	rc_mgr_input_block->MmioAbove4GLimit = POWER_OF_2(cpu_phys_address_size());
	rc_mgr_input_block->Above4GMmioSizePerRbForNonPciDevice = 0;
	//rc_mgr_input_block->BmcSocket = 0;
	//rc_mgr_input_block->EarlyBmcLinkLaneNum = 134;
}

static void configure_usb(void)
{
	/* Use a device that always ought to be present on the first domain */
	struct device *dev = pcidev_on_root(0x18, 0);
	assert(dev);
	const config_t *soc_config = (config_t *)dev->chip_info;
	assert(soc_config);
	const struct soc_usb_config *usb = &soc_config->usb;

	FCHUSB_INPUT_BLK *fch_usb_data = SilFindStructure(SilId_FchUsb, 0);
	fch_usb_data->Xhci0Enable = usb->xhci0_enable;
	fch_usb_data->Xhci1Enable = usb->xhci1_enable;
	fch_usb_data->Xhci2Enable = usb->xhci2_enable;
	for (int i = 0; i < 2; i++) {
		memcpy(&fch_usb_data->XhciOCpinSelect[i].Usb20OcPin, &usb->usb2_oc_pins[i],
		       sizeof(fch_usb_data->XhciOCpinSelect[i].Usb20OcPin));
		memcpy(&fch_usb_data->XhciOCpinSelect[i].Usb31OcPin, &usb->usb3_oc_pins[i],
		       sizeof(fch_usb_data->XhciOCpinSelect[i].Usb31OcPin));
	}
	fch_usb_data->XhciOcPolarityCfgLow = usb->polarity_cfg_low;
	fch_usb_data->Usb3PortForceGen1 = usb->usb3_force_gen1.raw;

	//	memcpy(&fch_usb_data->OemUsbConfigurationTable, &usb_oem_data, sizeof(usb_oem_data));
	fch_usb_data->OemUsbConfigurationTable.Usb31PhyEnable = usb->usb31_phy_enable;
	if (usb->usb31_phy_enable)
		memcpy(&fch_usb_data->OemUsbConfigurationTable.Usb31PhyPort, usb->usb31_phy,
		       sizeof(fch_usb_data->OemUsbConfigurationTable.Usb31PhyPort));
	fch_usb_data->OemUsbConfigurationTable.Usb31PhyEnable = usb->s1_usb31_phy_enable;
	if (usb->s1_usb31_phy_enable)
		memcpy(&fch_usb_data->OemUsbConfigurationTable.S1Usb31PhyPort, usb->s1_usb31_phy,
		       sizeof(fch_usb_data->OemUsbConfigurationTable.S1Usb31PhyPort));

}

#define NUM_SATA_CONTROLLERS 4
/* TODO: move to chip configuration */
static void configure_sata(void)
{
	FCHSATA_INPUT_BLK *fch_sata_data = SilFindStructure(SilId_FchSata, 0);
	for (int i = 0; i < NUM_SATA_CONTROLLERS; i++) {
		fch_sata_data[i].SataEnable = true;
		fch_sata_data[i].SataMsiEnable = true;
		fch_sata_data[i].SataClass = SataAhci;
		/* TODO: check and set remaining SATA PCD values */
	}
}

static void setup_opensil(void *unused)
{
	const SIL_STATUS debug_ret = SilDebugSetup(HostDebugService);
	SIL_STATUS_report("SilDebugSetup", debug_ret);
	const size_t MemReq = xSimQueryMemoryRequirements();
	void *MemBuf = cbmem_add(CBMEM_ID_AMD_OPENSIL, MemReq);
	if (!MemBuf)
		die("%s: failed to alloc mem\n", __func__);
	/* We run all opensil timepoints in the same stage so using TP1 as argument is fine. */
	const SIL_STATUS assign_mem_ret = xSimAssignMemoryTp1(MemBuf, MemReq);
	SIL_STATUS_report("xSimAssignMemory", assign_mem_ret);

	/* TODO: Move into DF chip */
	setup_rc_manager_default();
	configure_usb();
	configure_sata();
}

BOOT_STATE_INIT_ENTRY(BS_DEV_INIT_CHIPS, BS_ON_ENTRY, setup_opensil, NULL);

static void tp1_opensil(void *timepoint)
{
	const SIL_STATUS ret = InitializeAMDSiTp1();
	SIL_STATUS_report("InitializeAMDSi Tp1", ret);
	if (ret == SilResetRequestColdImm || ret == SilResetRequestColdDef) {
		printk(BIOS_INFO, "OpenSil requested a cold reset");
		do_cold_reset();
	} else if (ret == SilResetRequestWarmImm || ret == SilResetRequestWarmDef) {
		printk(BIOS_INFO, "OpenSil requested a warm reset");
		do_warm_reset();
	}
}

BOOT_STATE_INIT_ENTRY(BS_DEV_INIT_CHIPS, BS_ON_EXIT, tp1_opensil, SIL_TP1);
