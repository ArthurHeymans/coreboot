/* SPDX-License-Identifier: GPL-2.0-only */

/* TODO: Global NVS */

Scope(\_SB) {
	/* global utility methods expected within the \_SB scope */
	#include <arch/x86/acpi/globutil.asl>

	#include <soc/amd/common/acpi/gpio_bank_lib.asl>

	#include <soc/amd/common/acpi/osc.asl>

	/* TODO: PIC/IO-APIC defs */

	#include <soc/amd/common/acpi/pci_int.asl>

	/* TODO: mmio */

	/* TODO: root bridges */
} /* End \_SB scope */

#include <soc/amd/common/acpi/alib.asl>

#include <soc/amd/common/acpi/platform.asl>

#include <soc/amd/common/acpi/sleepstates.asl>

#include <soc/amd/common/acpi/upep.asl>

#if CONFIG(SOC_AMD_COMMON_BLOCK_ACPI_DPTC)
#include <soc/amd/common/acpi/dptc.asl>
#endif

/*
 * Platform Notify
 *
 * This is called by soc/amd/common/acpi/platform.asl.
 */
Method (PNOT)
{
	/* Report AC/DC state to ALIB using WAL1() */
	\WAL1 ()
}
