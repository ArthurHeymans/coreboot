/* SPDX-License-Identifier: GPL-2.0-only */

#include <acpi/acpigen.h>
#include <amdblocks/data_fabric.h>
#include <amdblocks/root_complex.h>
#include <device/device.h>
#include <device/pci.h>
#include <stdio.h>
#include <types.h>

#include "native/native.h"
#include "native/topology.h"

/*
 * PCI domain n is the root complex of die n. SP4 has two dies, SP4r2 one; the
 * DF bus, MMIO and I/O maps tell the common code which resources belong to
 * which die.
 */

static const char *const domain_acpi_names[ZEPPELIN_MAX_DIES] = { "PCI0", "PCI1" };

/*
 * Only mark the static devices of an absent die disabled. Their enable ops
 * would act on the hardware, and nothing behind the domain exists.
 */
static void mark_disabled(struct device *dev)
{
	dev->enabled = false;
	if (!dev->downstream)
		return;
	for (struct device *child = dev->downstream->children; child; child = child->sibling)
		mark_disabled(child);
}

static void zeppelin_domain_enable(struct device *domain)
{
	if (dev_get_domain_id(domain) >= zeppelin_die_count())
		mark_disabled(domain);
}

static void zeppelin_domain_set_resources(struct device *domain)
{
	/* The vendor code routes the legacy VGA ranges to the die with the VGA bus */
	if (domain->downstream->bridge_ctrl & PCI_BRIDGE_CTL_VGA)
		zeppelin_df_route_vga(dev_get_domain_id(domain));

	pci_domain_set_resources(domain);
}

static const char *zeppelin_domain_acpi_name(const struct device *domain)
{
	const unsigned int id = dev_get_domain_id(domain);

	return id < ARRAY_SIZE(domain_acpi_names) ? domain_acpi_names[id] : NULL;
}

static void zeppelin_domain_fill_ssdt(const struct device *domain)
{
	const unsigned int id = dev_get_domain_id(domain);

	amd_pci_domain_fill_ssdt(domain);

	acpigen_write_scope(acpi_device_path(domain));
	acpigen_write_name_integer("_UID", id);
	acpigen_pop_len();

	/* The DSDT has a root bridge for every possible die; hide the absent ones */
	if (id != 0)
		return;
	for (unsigned int die = zeppelin_die_count(); die < ZEPPELIN_MAX_DIES; die++) {
		char scope[16];

		snprintf(scope, sizeof(scope), "\\_SB.%s", domain_acpi_names[die]);
		acpigen_write_scope(scope);
		acpigen_write_STA(ACPI_STATUS_DEVICE_ALL_OFF);
		acpigen_pop_len();
	}
}

struct device_operations zeppelin_pci_domain_ops = {
	.read_resources	= amd_pci_domain_read_resources,
	.set_resources	= zeppelin_domain_set_resources,
	.scan_bus	= amd_pci_domain_scan_bus,
	.enable		= zeppelin_domain_enable,
	.init		= amd_pci_domain_init,
	.acpi_name	= zeppelin_domain_acpi_name,
	.acpi_fill_ssdt	= zeppelin_domain_fill_ssdt,
};
