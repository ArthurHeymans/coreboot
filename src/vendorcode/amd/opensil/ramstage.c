/* SPDX-License-Identifier: GPL-2.0-only */

#include <bootstate.h>
#include <cbmem.h>
#include <cpu/cpu.h>
#include <Sil-api.h>
#include <SilCommon.h>
#include <xSIM-api.h>
#include <DF/RcManager-api.h>
#include <amdblocks/reset.h>
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

static DF4_FABRIC_IO_MANAGER io_rc_mgr;
static DF4_FABRIC_MMIO_MANAGER mmio_rc_mgr;

static void setup_rc_manager_default(void)
{
	DF4_RCMGR_INPUT_BLK *rc_mgr_input_block = xSimFindStructure(SilId_RcManager,  0);
	rc_mgr_input_block->IoRcMgr = &io_rc_mgr;
	rc_mgr_input_block->MmioRcMgr = &mmio_rc_mgr;
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
	rc_mgr_input_block->BmcSocket = 0;
	rc_mgr_input_block->EarlyBmcLinkLaneNum = 134;
}

static void setup_opensil(void)
{
	static bool done;

	if (done)
		return;

	const SIL_STATUS debug_ret = SilDebugSetup(HostDebugService);
	SIL_STATUS_report("SilDebugSetup", debug_ret);
	const size_t MemReq = xSimQueryMemoryRequirements();
	void *MemBuf = cbmem_add(CBMEM_ID_AMD_OPENSIL, MemReq);
	if (!MemBuf)
		die("%s: failed to alloc mem\n", __func__);
	/* We run all opensil timepoints in the same stage so using TP1 as argument is fine. */
	const SIL_STATUS assign_mem_ret = xSimAssignMemory(MemBuf, MemReq, SIL_TP1);
	SIL_STATUS_report("xSimAssignMemory", assign_mem_ret);
	done = true;

	setup_rc_manager_default();
}

static void tp1_opensil(void *timepoint)
{
	setup_opensil();

	const SIL_STATUS ret = InitializeAMDSi(SIL_TP1);
	SIL_STATUS_report("InitializeAMDSi", ret);
	if (ret == SilResetRequestColdImm || ret == SilResetRequestColdDef) {
		printk(BIOS_INFO, "OpenSil requested a cold reset");
		do_cold_reset();
	} else if (ret == SilResetRequestWarmImm || ret == SilResetRequestWarmDef) {
		printk(BIOS_INFO, "OpenSil requested a warm reset");
		do_warm_reset();
	}
}

BOOT_STATE_INIT_ENTRY(BS_PRE_DEVICE, BS_ON_EXIT, tp1_opensil, SIL_TP1);
