/* SPDX-License-Identifier: GPL-2.0-only */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <console/console.h>
#include <vendorcode/amd/opensil/console.h>

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

	/*
	 * Combine prefix & message to a coreboot message
	 * TODO: translate opensil message levels to coreboot ones.
	 */
	printk(BIOS_DEBUG, "L=%lu,%s%s", MsgLevel, prefix, msg);
}
