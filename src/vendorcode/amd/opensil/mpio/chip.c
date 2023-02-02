/* SPDX-License-Identifier: GPL-2.0-only */

#include <DF/RcManager-api.h>
#include <MPIO/MpioClass-api.h>
#include <MPIO/MpioStructs.h>
#include <device/device.h>
#include <device/pci_def.h>
#include "chip.h"

static void nbio_config(void)
{
	NBIOCLASS_INPUT_BLK *nbio_data = SilFindStructure(SilId_NbioClass, 0);
	nbio_data->CfgHdAudioEnable           = false;
	nbio_data->EsmEnableAllRootPorts      = false;
	nbio_data->EsmTargetSpeed             = 16;
	nbio_data->CfgRxMarginPersistenceMode = 1;
	nbio_data->CfgDxioFrequencyVetting    = false;
	nbio_data->CfgSkipPspMessage          = 1;
	nbio_data->CfgEarlyTrainTwoPcieLinks  = false;
	nbio_data->EarlyBmcLinkTraining       = true;
	nbio_data->EarlyBmcLinkSocket         = 0;
	nbio_data->EarlyBmcLinkLaneNum        = 134;
	nbio_data->EarlyBmcLinkDie            = 0;
	nbio_data->EdpcEnable                 = 0;
	nbio_data->PcieAerReportMechanism     = 2;
	nbio_data->SevSnpSupport              = false;
}

static void mpio_global_config(MPIOCLASS_INPUT_BLK *mpio_data)
{
	mpio_data->CfgDxioClockGating                  = 1;
	mpio_data->PcieDxioTimingControlEnable         = 0;
	mpio_data->PCIELinkReceiverDetectionPolling    = 0;
	mpio_data->PCIELinkResetToTrainingTime         = 0;
	mpio_data->PCIELinkL0Polling                   = 0;
	mpio_data->PCIeExactMatchEnable                = 0;
	mpio_data->DxioPhyValid                        = 1;
	mpio_data->DxioPhyProgramming                  = 1;
	mpio_data->CfgSkipPspMessage                   = 1;
	mpio_data->DxioSaveRestoreModes                = 0xff;
	mpio_data->AmdAllowCompliance                  = 0;
	mpio_data->AmdAllowCompliance                  = 0xff;
	mpio_data->SrisEnableMode                      = 0xff;
	mpio_data->SrisSkipInterval                    = 0;
	mpio_data->SrisSkpIntervalSel                  = 1;
	mpio_data->SrisCfgType                         = 0;
	mpio_data->SrisAutoDetectMode                  = 0xff;
	mpio_data->SrisAutodetectFactor                = 0;
	mpio_data->SrisLowerSkpOsGenSup                = 0;
	mpio_data->SrisLowerSkpOsRcvSup                = 0;
	mpio_data->AmdCxlOnAllPorts                    = 1;
	mpio_data->CxlCorrectableErrorLogging          = 1;
	mpio_data->CxlUnCorrectableErrorLogging        = 1;
	  // This is also available in Nbio. How to handle duplicate entries?
	mpio_data->CfgAEREnable                        = 1;
	mpio_data->CfgMcCapEnable                      = 0;
	mpio_data->CfgRcvErrEnable                     = 0;
	  // This is also available in Nbio. How to handle duplicate entries?
	mpio_data->EarlyBmcLinkTraining                = 1;
	//mpio_data->EarlyBmcLinkSocket                  = 0;
	// mpio_data->EarlyBmcLinkLaneNum                 = 0x86;
	// mpio_data->EarlyBmcLinkDie                     = 0;
	mpio_data->SurpriseDownFeature                 = 1;
	mpio_data->LcMultAutoSpdChgOnLastRateEnable    = 0;
	mpio_data->AmdRxMarginEnabled                  = 1;
	mpio_data->CfgPcieCVTestWA                     = 1;
	  // This is also available in Nbio. How to handle duplicate entries?
	mpio_data->CfgPcieAriSupport                   = 1;
	mpio_data->CfgNbioCTOtoSC                      = 0;
	mpio_data->CfgNbioCTOIgnoreError               = 1;
	mpio_data->CfgNbioSsid                         = 0;
	mpio_data->CfgIommuSsid                        = 0;
	mpio_data->CfgPspccpSsid                       = 0;
	mpio_data->CfgNtbccpSsid                       = 0;
	mpio_data->CfgNbifF0Ssid                       = 0;
	mpio_data->CfgNtbSsid                          = 0;
	mpio_data->AmdPcieSubsystemDeviceID            = 0x1453;
	mpio_data->AmdPcieSubsystemVendorID            = 0x1022;
	mpio_data->GppAtomicOps                        = 1;
	mpio_data->GfxAtomicOps                        = 1;
	mpio_data->AmdNbioReportEdbErrors              = 0;
	  // This is also available in Nbio. How to handle duplicate entries?
	mpio_data->OpnSpare                            = 0;
	mpio_data->AmdPreSilCtrl0                      = 0;
	mpio_data->MPIOAncDataSupport                  = 1;
	mpio_data->AfterResetDelay                     = 0;
	mpio_data->CfgEarlyLink                        = 0;
	mpio_data->AmdCfgExposeUnusedPciePorts         = 1; // Show all ports
	mpio_data->CfgForcePcieGenSpeed                = 0;
	mpio_data->CfgSataPhyTuning                    = 0;
	mpio_data->PcieLinkComplianceModeAllPorts      = 0;
	mpio_data->AmdMCTPEnable                       = 0;
	mpio_data->SbrBrokenLaneAvoidanceSup           = 1;
	mpio_data->AutoFullMarginSup                   = 1;
	  // A getter and setter, both are needed for this PCD.
	mpio_data->AmdPciePresetMask8GtAllPort         = 0xffffffff;
	  // A getter and setter, both are needed for this PCD.
	mpio_data->AmdPciePresetMask16GtAllPort        = 0xffffffff;
	  // A getter and setter, both are needed for this PCD.
	mpio_data->AmdPciePresetMask32GtAllPort        = 0xffffffff;
	mpio_data->PcieLinkAspmAllPort                 = 0xff;

	  // Is this needed? Ideally we only need to pass back the assigned value to the host.
	  // mpio_data->AmdMCTPMasterSeg               = PcdGet8(PcdAmdMCTPMasterSeg);

	  // Is this needed? Ideally we only need to pass back the assigned value to the host.
	  // mpio_data->AmdMCTPMasterID                = PcdGet16(PcdAmdMCTPMasterID);

	mpio_data->SyncHeaderByPass                    = 1;
	mpio_data->CxlTempGen5AdvertAltPtcl            = 0;

	/* TODO handle this differently on multisocket */
	mpio_data->PcieTopologyData.PlatformData[0].Flags = DESCRIPTOR_TERMINATE_LIST;
	mpio_data->PcieTopologyData.PlatformData[0].PciePortList = mpio_data->PcieTopologyData.PortList;

}

static void setup_bmc_lanes(uint8_t lane, uint8_t socket)
{
	DF4_RCMGR_INPUT_BLK *rc_mgr_input_block = SilFindStructure(SilId_RcManager,  0);
	rc_mgr_input_block->BmcSocket = socket;
	rc_mgr_input_block->EarlyBmcLinkLaneNum = lane;

	NBIOCLASS_INPUT_BLK *nbio_data = SilFindStructure(SilId_NbioClass, 0);
	nbio_data->EarlyBmcLinkSocket         = socket;
	nbio_data->EarlyBmcLinkLaneNum        = lane;
	nbio_data->EarlyBmcLinkDie            = 0; // TODO
}

static void per_device_config(MPIOCLASS_INPUT_BLK *mpio_data, struct device *dev,
			      struct vendorcode_amd_opensil_mpio_config *const config)
{
	const uint32_t df_id = dev->bus->dev->path.domain.domain;
	const uint32_t devfn = dev->path.pci.devfn;
	printk(BIOS_DEBUG, "Setting MPIO port for DF_ID 0x%x, PCI %d:%d\n",
	       df_id, PCI_SLOT(devfn), PCI_FUNC(devfn));

	if (config->bmc) {
		// TODO BMC setup of other parts of the code
		mpio_data->EarlyBmcLinkLaneNum = config->start_lane;
		setup_bmc_lanes(config->start_lane, 0); // TODO support multiple sockets
		return;
	}

	static int mpio_port = 0;
	MPIO_PORT_DESCRIPTOR port = {
		.Flags = DESCRIPTOR_TERMINATE_LIST,
		.EngineData = MPIO_ENGINE_DATA_INITIALIZER(MpioPcieEngine, config->start_lane,
							   config->start_lane + config->n_lanes - 1,
							   config->hotplug == HotplugDisabled ? 0 : 1,
							   config->gpio_group),
		.Port = MPIO_PORT_DATA_INITIALIZER_PCIE(MpioPortEnabled,
							PCI_SLOT(devfn),
							PCI_FUNC(devfn),
							config->hotplug,
							config->speed,
							0, // No backup PCIe speed
							config->aspm,
							config->aspm_l1_1,
							config->aspm_l1_2,
							config->clock_pm),
	};
	port.Port.AlwaysExpose = 1,
	mpio_data->PcieTopologyData.PortList[mpio_port] = port;
	/* Update TERMINATE list */
	if (mpio_port > 0)
		mpio_data->PcieTopologyData.PortList[mpio_port - 1].Flags = 0;
	mpio_port++;
}

static void mpio_config(void *const config)
{
	MPIOCLASS_INPUT_BLK *mpio_data = SilFindStructure(SilId_MpioClass, 0);
	mpio_global_config(mpio_data);
	nbio_config();

	/* Find all devices with this chip */
	for (struct device *dev = &dev_root; dev; dev = dev->next)
		if (dev->chip_ops->init == mpio_config)
			per_device_config(mpio_data, dev->bus->dev, dev->chip_info);
}

struct chip_operations vendorcode_amd_opensil_mpio_ops = {
	CHIP_NAME("AMD GENOA MPIO")
	.init = mpio_config,
};
