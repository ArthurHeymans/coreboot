/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/root_complex.h>
#include <amdblocks/smn.h>
#include <device/device.h>
#include <types.h>

/*
 * SoCs with one SMN address space per root complex, like Zeppelin with one per
 * die, override this to reach the IOHC registers of the domain.
 */
__weak uint64_t domain_smn_read64(const struct device *domain, uint32_t reg)
{
	return smn_read64(reg);
}

void read_non_pci_resources(struct device *domain, unsigned long *idx)
{
	const uint32_t iohc_misc_base = get_iohc_misc_smn_base(domain);
	const struct non_pci_mmio_reg *regs;
	size_t reg_count;

	regs = get_iohc_non_pci_mmio_regs(&reg_count);

	for (size_t i = 0; i < reg_count; i++) {
		const uint32_t reg = iohc_misc_base + regs[i].iohc_misc_offset;
		const uint64_t reg64 = domain_smn_read64(domain, reg);
		/* only report enabled non-PCI MMIO regions */
		if (!(reg64 & IOHC_MMIO_EN))
			continue;

		const unsigned long res_idx = regs[i].res_idx == NON_PCI_RES_IDX_AUTO ?
						(*idx)++ : regs[i].res_idx;
		const uint64_t base = reg64 & regs[i].mask;
		mmio_range(domain, res_idx, base, regs[i].size);
	}
}
