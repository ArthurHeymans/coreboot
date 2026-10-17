/* SPDX-License-Identifier: GPL-2.0-only */

/*
 * NBIO and PCIe settings of the vendor DXE drivers (AmdNbioBaseZPDxe,
 * AmdNbioPcieZPDxe), per die.
 */

#include <console/console.h>
#include <types.h>

#include "nbio.h"
#include "topology.h"

#define NBIF_HWINIT_LOCK	0x2358c

/*
 * Clock gating of the PCIe cores. The per-die wrapper features in the vendor
 * topology template enable ClkGating but not LclkGating.
 */
static void pcie_clock_gating(unsigned int die)
{
	for (unsigned int core = 0; core < PCIE_CORE_COUNT; core++) {
		zeppelin_smn_clrsetbits32(die, pcie_core_reg(core, PCIE_CONFIG_CNTL), 0, 0xf);
		zeppelin_smn_clrsetbits32(die, pcie_core_reg(core, CPM_CONTROL), 0x3c00, 0x81e6);
		zeppelin_smn_clrsetbits32(die, pcie_core_reg(core, PCIE_CNTL_18C), 0, 0x60);
	}
}

/* Before the PCI enumeration */
void zeppelin_nbio_pre_enumeration_init(void)
{
	for (unsigned int die = 0; die < zeppelin_die_count(); die++) {
		pcie_clock_gating(die);

		/* Lock the NBIF hardware init registers (straps, IDs) */
		for (unsigned int n = 0; n < NBIF_COUNT; n++)
			zeppelin_smn_clrsetbits32(die, NBIF_REG(n, NBIF_HWINIT_LOCK), 0, BIT(0));
	}
}
