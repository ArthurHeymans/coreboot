/* SPDX-License-Identifier: GPL-2.0-only */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <console/console.h>
#include <vendorcode/amd/opensil/console.h>

void HostDebugService(size_t MsgLevel, const char *SilPrefix, const char *Message,
				 const char *Function, size_t Line, ...)
{
	/* Format the Prefix */
	char fmt_str[60];
	snprintf(fmt_str, sizeof(fmt_str), SilPrefix, (uintptr_t)Function, Line, " ");

	/* Write formatted data to buffed */
	char fmt_msg[120];
	va_list args;
	va_start(args, Line);
	vsnprintf(fmt_msg, sizeof(fmt_msg), Message, args);
	va_end(args);

	/*
	 * Combine prefix & message to a coreboot message
	 * TODO: translate opensil message levels to coreboot ones.
	 */
	printk(BIOS_DEBUG, "L=%lu,%s%s", MsgLevel, fmt_str, fmt_msg);
}
