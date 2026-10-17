/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/data_fabric.h>
#include <amdblocks/ioapic.h>
#include <amdblocks/root_complex.h>
#include <device/device.h>
#include <device/pci.h>
#include <stdint.h>
#include <soc/data_fabric.h>

static const char *gnb_acpi_name(const struct device *dev)
{
	return "GNB";
}

struct device_operations zeppelin_root_complex_operations = {
	/* The root complex has no PCI BARs implemented, so there's no need to call
	   pci_dev_read_resources for it */
	.read_resources		= noop_read_resources,
	.set_resources		= noop_set_resources,
	.enable_resources	= pci_dev_enable_resources,
	.acpi_name		= gnb_acpi_name,
};

/* The IOHC of die 0; SP4's second die is not described yet */
static const struct domain_iohc_info iohc_info[] = {
	[0] = {
		.fabric_id = IOMS0_FABRIC_ID,
		.misc_smn_base = SMN_IOHC_MISC_BASE_13B1,
	},
};

const struct domain_iohc_info *get_iohc_info(size_t *count)
{
	*count = ARRAY_SIZE(iohc_info);
	return iohc_info;
}

/* The non-PCI MMIO that the native code maps */
static const struct non_pci_mmio_reg non_pci_mmio[] = {
	{ IOHC_MISC_PSP_MMIO_REG, 0xfffffff00000ull, 1 * MiB, NON_PCI_RES_IDX_AUTO },
	/* The IOAPIC needs 256 byte alignment, but gets a 4 KiB aligned base */
	{ IOHC_MISC_IOAPIC_MMIO_REG, 0xffffffffff00ull, 4 * KiB, IOMMU_IOAPIC_IDX },
};

const struct non_pci_mmio_reg *get_iohc_non_pci_mmio_regs(size_t *count)
{
	*count = ARRAY_SIZE(non_pci_mmio);
	return non_pci_mmio;
}
