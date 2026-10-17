/* SPDX-License-Identifier: GPL-2.0-or-later */

#include <amdblocks/amd_pci_util.h>
#include <commonlib/bsd/helpers.h>
#include <device/pci_def.h>
#include <soc/pci_devs.h>
#include <stddef.h>

/*
 * The IOAPIC bridge routing that the NBIO base table programs (SMN
 * 0x14300040 + 4 * bridge: irq << 16 | swizzle << 4 | group). The vendor
 * _PRT reads the hardware swizzle k as coreboot swizzle (4 - k) % 4.
 */
static const struct pci_routing_info pci_routing_table[] = {
	{ PCI_DEVFN(1, 1), 0, PCI_SWIZZLE_ABCD, 0x10 },
	{ PCI_DEVFN(1, 2), 1, PCI_SWIZZLE_ABCD, 0x11 },
	{ PCI_DEVFN(1, 3), 2, PCI_SWIZZLE_ABCD, 0x12 },
	{ PCI_DEVFN(1, 4), 3, PCI_SWIZZLE_ABCD, 0x13 },
	{ PCI_DEVFN(1, 5), 4, PCI_SWIZZLE_ABCD, 0x10 },
	{ PCI_DEVFN(1, 6), 5, PCI_SWIZZLE_ABCD, 0x11 },
	{ PCI_DEVFN(1, 7), 6, PCI_SWIZZLE_ABCD, 0x12 },
	{ PCI_DEVFN(2, 1), 7, PCI_SWIZZLE_ABCD, 0x13 },
	{ PCI_DEVFN(3, 1), 7, PCI_SWIZZLE_CDAB, 0x0c },
	{ PCI_DEVFN(3, 2), 6, PCI_SWIZZLE_CDAB, 0x0d },
	{ PCI_DEVFN(3, 3), 5, PCI_SWIZZLE_CDAB, 0x0e },
	{ PCI_DEVFN(3, 4), 4, PCI_SWIZZLE_CDAB, 0x0f },
	{ PCI_DEVFN(3, 5), 3, PCI_SWIZZLE_CDAB, 0x0c },
	{ PCI_DEVFN(3, 6), 2, PCI_SWIZZLE_CDAB, 0x0d },
	{ PCI_DEVFN(3, 7), 1, PCI_SWIZZLE_CDAB, 0x0e },
	{ PCI_DEVFN(4, 1), 0, PCI_SWIZZLE_CDAB, 0x0f },
	{ PCIE_GPP_A_DEVFN, 3, PCI_SWIZZLE_DABC, 0x14 },
	{ PCIE_GPP_B_DEVFN, 4, PCI_SWIZZLE_BCDA, 0x08 },
};

const struct pci_routing_info *get_pci_routing_table(size_t *entries)
{
	*entries = ARRAY_SIZE(pci_routing_table);
	return pci_routing_table;
}
