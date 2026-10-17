/* SPDX-License-Identifier: GPL-2.0-only */

#include <soc/amd/common/acpi/pci_root.asl>
#include "globalnvs.asl"

#include "pnot.asl"

/* Contains the supported sleep states for this chipset */
#include <soc/amd/common/acpi/sleepstates.asl>

/* Contains _SWS methods */
#include <soc/amd/common/acpi/acpi_wake_source.asl>

/* System Bus */
Scope(\_SB) { /* Start \_SB scope */
	/* global utility methods expected within the \_SB scope */
	#include <arch/x86/acpi/globutil.asl>

	ROOT_BRIDGE(PCI0)

	Scope(PCI0) {
		/* Describe the AMD Northbridge */
		#include "northbridge.asl"
		#include "pci.asl"

		/* Describe the AMD Fusion Controller Hub */
		#include <soc/amd/common/acpi/lpc.asl>
		#include <soc/amd/common/acpi/platform.asl>
	}

	/* PCI IRQ mapping for the Southbridge */
	#include "pci_int_defs.asl"

	/* Describe PCI INT[A-H] for the Southbridge */
	#include <soc/amd/common/acpi/pci_int.asl>

	/* Describe the MMIO devices in the FCH */
	#include "mmio.asl"

	/* Add GPIO library */
	#include <soc/amd/common/acpi/gpio_bank_lib.asl>

} /* End \_SB scope */
