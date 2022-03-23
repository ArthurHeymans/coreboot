/* SPDX-License-Identifier: GPL-2.0-only */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <console/console.h>
#include <vendorcode/amd/opensil/console.h>
#include <SilCommon.h>

static int translate_opensil_debug_level(size_t MsgLevel)
{
	switch (MsgLevel) {
	case SIL_TRACE_ERROR:
		return BIOS_ERR;
	case SIL_TRACE_WARNING:
		return BIOS_WARNING;
	case SIL_TRACE_ENTRY:
	case SIL_TRACE_EXIT:
		return BIOS_SPEW;
	case SIL_TRACE_INFO:
		return BIOS_DEBUG;
	default:
		return BIOS_NEVER;
	}
}

void HostDebugService(size_t MsgLevel, const char *SilPrefix, const char *Message,
				 const char *Function, size_t Line, ...)
{
	if (!CONFIG(OPENSIL_DEBUG_OUTPUT))
		return;

	/* Format the Prefix */
	char prefix[60];
	snprintf(prefix, sizeof(prefix), "%s%s:%d:", SilPrefix, (uintptr_t)Function, Line);

	/* Write formatted data to buffed */
	char msg[120];
	va_list args;
	va_start(args, Line);
	vsnprintf(msg, sizeof(msg), Message, args);
	va_end(args);

	const int loglevel = translate_opensil_debug_level(MsgLevel);
	printk(loglevel, "%s%s", CONFIG(OPENSIL_DEBUG_PREFIX) ? prefix : "", msg);
}
