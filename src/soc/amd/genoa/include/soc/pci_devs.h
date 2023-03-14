/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_GENOA_PCI_DEVS_H
#define AMD_GENOA_PCI_DEVS_H


#include <device/pci_def.h>
#include <amdblocks/pci_devs.h>

/* GNB Root Complex */
#define GNB_DEV			0x0
#define GNB_FUNC		0
#define GNB_DEVFN		PCI_DEVFN(GNB_DEV, GNB_FUNC)
#define SOC_GNB_DEV		_SOC_DEV(GNB_DEV, GNB_FUNC)

#endif
