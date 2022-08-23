/* SPDX-License-Identifier: GPL-2.0-only */

#include <bootstate.h>
#include <cbmem.h>
#include <xSIM-api.h>
#include <vendorcode/amd/opensil/console.h>

static void SIL_STATUS_report(const char *function, const SIL_STATUS status)
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
	const SIL_STATUS assign_mem_ret =xSimAssignMemory(MemBuf, MemReq, SIL_TP1);
	SIL_STATUS_report("xSimAssignMemory", assign_mem_ret);
	done = true;
}

static void tp1_opensil(void *timepoint)
{
	setup_opensil();

	const SIL_STATUS ret = InitializeAMDSi();
	SIL_STATUS_report("InitializeAMDSi", ret);
}

BOOT_STATE_INIT_ENTRY(BS_PRE_DEVICE, BS_ON_EXIT, tp1_opensil, SIL_TP1);
