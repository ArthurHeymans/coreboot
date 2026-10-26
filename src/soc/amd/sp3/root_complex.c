/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/ioapic.h>
#include <amdblocks/root_complex.h>
#include <device/device.h>
#include <device/pci.h>
#include "native/topology.h"

static const char *gnb_acpi_name(const struct device *dev)
{
	return "GNB";
}

struct device_operations sp3_root_complex_operations = {
	.read_resources = noop_read_resources,
	.set_resources = noop_set_resources,
	.enable_resources = pci_dev_enable_resources,
	.acpi_name = gnb_acpi_name,
};

static struct domain_iohc_info iohc_info[4] = {
	{ .fabric_id = 0x1b, .misc_smn_base = 0x13e10000 },
	{ .fabric_id = 0x1a, .misc_smn_base = 0x13d10000 },
	{ .fabric_id = 0x19, .misc_smn_base = 0x13c10000 },
	{ .fabric_id = 0x18, .misc_smn_base = 0x13b10000 },
};

#if ENV_RAMSTAGE
uint64_t domain_smn_read64(const struct device *domain, uint32_t reg)
{
	const unsigned int rb = dev_get_domain_id(domain);
	const uint64_t lo = sp3_smn_read32(rb, reg);

	return lo | (uint64_t)sp3_smn_read32(rb, reg + 4) << 32;
}
#endif

const struct domain_iohc_info *get_iohc_info(size_t *count)
{
#if ENV_RAMSTAGE
	*count = sp3_die_count();
	for (unsigned int rb = 0; rb < *count; rb++) {
		iohc_info[rb].fabric_id = sp3_ioms_fabric_id(rb);
		iohc_info[rb].misc_smn_base = sp3_nbio_address(rb, 0x13b10000);
	}
#else
	*count = ARRAY_SIZE(iohc_info);
#endif
	return iohc_info;
}

static const struct non_pci_mmio_reg non_pci_mmio[] = {
	{ IOHC_MISC_PSP_MMIO_REG, 0xfffffff00000ull, 1 * MiB, NON_PCI_RES_IDX_AUTO },
	{ IOHC_MISC_IOAPIC_MMIO_REG, 0xffffffffff00ull, 4 * KiB, IOMMU_IOAPIC_IDX },
	{ 0x2e8, 0xfffffff00000ull, 1 * MiB, NON_PCI_RES_IDX_AUTO },
};

const struct non_pci_mmio_reg *get_iohc_non_pci_mmio_regs(size_t *count)
{
	*count = ARRAY_SIZE(non_pci_mmio);
	return non_pci_mmio;
}
