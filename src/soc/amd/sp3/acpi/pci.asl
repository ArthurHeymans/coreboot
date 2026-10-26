/* SPDX-License-Identifier: GPL-2.0-only */

#include <soc/amd/common/acpi/pci_template.asl>

/*
 * Keep name in sync with SSDT generator pcie_gpp_acpi_name()!
 * Currently the name is hex representation of dev->path.pci.devfn.
 */

/* Root complex */
Device (GNB)
{
	Name (_ADR, 0)
	Method (_STA, 0, NotSerialized)
	{
		Return (0x0F)
	}
}

/* PCIe GPP: core 0 (1.1-1.7, 2.1) and core 1 (3.1-3.7, 4.1) */
ACPI_PCI_DEV(GP09, 1, 1)
ACPI_PCI_DEV(GP0A, 1, 2)
ACPI_PCI_DEV(GP0B, 1, 3)
ACPI_PCI_DEV(GP0C, 1, 4)
ACPI_PCI_DEV(GP0D, 1, 5)
ACPI_PCI_DEV(GP0E, 1, 6)
ACPI_PCI_DEV(GP0F, 1, 7)
ACPI_PCI_DEV(GP11, 2, 1)
ACPI_PCI_DEV(GP19, 3, 1)
ACPI_PCI_DEV(GP1A, 3, 2)
ACPI_PCI_DEV(GP1B, 3, 3)
ACPI_PCI_DEV(GP1C, 3, 4)
ACPI_PCI_DEV(GP1D, 3, 5)
ACPI_PCI_DEV(GP1E, 3, 6)
ACPI_PCI_DEV(GP1F, 3, 7)
ACPI_PCI_DEV(GP21, 4, 1)

/* Internal NBIF bridges. */
ACPI_PCI_DEV(GP29, 5, 1)
ACPI_PCI_DEV(GP2A, 5, 2)
ACPI_PCI_DEV(GP39, 7, 1)
ACPI_PCI_DEV(GP41, 8, 1)
ACPI_PCI_DEV(GP42, 8, 2)
ACPI_PCI_DEV(GP43, 8, 3)
