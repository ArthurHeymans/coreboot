/* SPDX-License-Identifier: GPL-2.0-only */

/* TODO: Global NVS */

Scope(\_SB) {
	/* global utility methods expected within the \_SB scope */
	#include <arch/x86/acpi/globutil.asl>

	#include <soc/amd/common/acpi/gpio_bank_lib.asl>

	#include <soc/amd/common/acpi/osc.asl>

	#include "pci_int_defs.asl"

	#include <soc/amd/common/acpi/pci_int.asl>

	#include "mmio.asl"

#define ROOT_BRIDGE(socket, bus) \
	Device(S##socket##B##bus##) { \
		Name(_HID, EISAID("PNP0A08"))	/* PCI Express Root Bridge */ \
		Name(_CID, EISAID("PNP0A03"))	/* PCI Root Bridge */ \
		Method (_OSC, 4, NotSerialized) { \
			/* Check for proper PCI/PCIe UUID */ \
			If (Arg0 == ToUUID("33DB4D5B-1FF7-401C-9657-7441C03DD766")) \
			{ \
				/* Let OS control everything */ \
				Return(Arg3) \
			} Else { \
				CreateDWordField(Arg3, 0, CDW1) \
				CDW1 = CDW1 | 4	/* Unrecognized UUID, so set bit 2 to 1 */ \
				Return(Arg3) \
			} \
		} \
	} \

	ROOT_BRIDGE(0, 0)
	ROOT_BRIDGE(0, 1)
	ROOT_BRIDGE(0, 2)
	ROOT_BRIDGE(0, 3)
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
