#include <bootstate.h>
#include <cbmem.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

size_t xSimQueryMemoryRequirements(void);
int xSimAssignMemory(void *BaseAddress, size_t MemorySize);
int SilDebugSetup(void *HostDbgService);
int InitializeAMDSi(void);


static void testHostDebugService(size_t MsgLevel, const char *SilPrefix, const char *Message,
				 const char *Function, size_t Line, ...)
{
	char fmt_str[80];
	char prefix_fixup[80];
	strcpy(prefix_fixup, SilPrefix);
	prefix_fixup[10] = 's';
	snprintf(fmt_str, sizeof(fmt_str), prefix_fixup, (uintptr_t)Function, Line, " ");

	char fmt_msg[80];
	va_list args;
	va_start(args, Line);
	vsnprintf(fmt_msg, sizeof(fmt_msg), Message, args);
	va_end(args);

	printk(BIOS_DEBUG, "L=%lu,%s%s", MsgLevel, fmt_str, fmt_msg);
}

static void test_opensil(void *unused)
{
	size_t MemReq = xSimQueryMemoryRequirements();
	void *MemBuf = cbmem_add(0xabababa, MemReq);
	if (!MemBuf)
		die("%s: failed to alloc mem\n", __func__);
	xSimAssignMemory(MemBuf, MemReq);
	SilDebugSetup(testHostDebugService);
	InitializeAMDSi();
}

BOOT_STATE_INIT_ENTRY(BS_PRE_DEVICE, BS_ON_EXIT, test_opensil, NULL);
