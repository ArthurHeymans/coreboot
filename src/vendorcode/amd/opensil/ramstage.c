#include <bootstate.h>
#include <cbmem.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <xSIM-api.h>

static void HostDebugService(size_t MsgLevel, const char *SilPrefix, const char *Message,
				 const char *Function, size_t Line, ...)
{
	/* Format the Prefix */
	char fmt_str[80];
	snprintf(fmt_str, sizeof(fmt_str), SilPrefix, (uintptr_t)Function, Line, " ");

	/* Write formatted data to buffed */
	char fmt_msg[80];
	va_list args;
	va_start(args, Line);
	vsnprintf(fmt_msg, sizeof(fmt_msg), Message, args);
	va_end(args);

	/*
	 * Combine prefix & message to a coreboot message
	 * TODO: transalte opensil message levels to coreboot ones.
	 */
	printk(BIOS_DEBUG, "L=%lu,%s%s", MsgLevel, fmt_str, fmt_msg);
}

static void setup_opensil(void)
{
	static bool done;

	if (done)
		return;

	SilDebugSetup(HostDebugService);
	const size_t MemReq = xSimQueryMemoryRequirements();
	void *MemBuf = cbmem_add(CBMEM_ID_AMD_OPENSIL, MemReq);
	if (!MemBuf)
		die("%s: failed to alloc mem\n", __func__);
	/* We run all opensil timepoints in the same stage so using TP1 as argument is fine. */
	xSimAssignMemory(MemBuf, MemReq, SIL_TP1);
	done = true;
}

static void tp1_opensil(void *timepoint)
{
	setup_opensil();

	InitializeAMDSi();
}

BOOT_STATE_INIT_ENTRY(BS_PRE_DEVICE, BS_ON_EXIT, tp1_opensil, SIL_TP1);
