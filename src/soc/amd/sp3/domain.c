/* SPDX-License-Identifier: GPL-2.0-only */

#include <acpi/acpigen.h>
#include <acpi/acpigen_pci.h>
#include <amdblocks/amd_pci_util.h>
#include <amdblocks/data_fabric.h>
#include <amdblocks/root_complex.h>
#include <device/device.h>
#include <device/pci.h>
#include <device/pci_ops.h>
#include <stdio.h>
#include <types.h>

#include "native/native.h"
#include "native/topology.h"

/*
 * Each PCI domain is a live NBIO root bridge, ordered by root bus number.
 * The DF maps describe its bus, MMIO and I/O windows.
 */

static const char *const domain_acpi_names[SP3_MAX_DIES] = { "PCI0", "PCI1", "PCI2", "PCI3" };

/*
 * Mark the static devices of an absent root bridge disabled. Their enable ops
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

static void sp3_domain_enable(struct device *domain)
{
	if (dev_get_domain_id(domain) >= sp3_die_count())
		mark_disabled(domain);
}

/* Not 0: the auto index of the common non-PCI resources */
#define NONPCI_POOL_IDX		0x20000200

/*
 * Keep each root bridge's PSP, SMU, IOMMU and IOAPIC pool out of PCI allocation.
 */
static void sp3_domain_read_resources(struct device *domain)
{
	const unsigned int die = dev_get_domain_id(domain);

	amd_pci_domain_read_resources(domain);
	mmio_range(domain, NONPCI_POOL_IDX, sp3_nonpci_mmio_base(die),
		   SP3_NONPCI_MMIO_SIZE);
}

static void sp3_domain_set_resources(struct device *domain)
{
	/* The vendor code routes the legacy VGA ranges to the die with the VGA bus */
	if (domain->downstream->bridge_ctrl & PCI_BRIDGE_CTL_VGA)
		sp3_df_route_vga(dev_get_domain_id(domain));

	pci_domain_set_resources(domain);
}

static const char *sp3_domain_acpi_name(const struct device *domain)
{
	const unsigned int id = dev_get_domain_id(domain);

	return id < ARRAY_SIZE(domain_acpi_names) ? domain_acpi_names[id] : NULL;
}

/* A root bus device with an INTx pin of its own */
struct root_irq {
	unsigned int slot;
	unsigned int pin;
	unsigned int gsi;
};

static void write_fch_prt_entries(void)
{
	static const char *const links[] = { "\\_SB.INTA", "\\_SB.INTB", "\\_SB.INTC",
					     "\\_SB.INTD" };

	/* IOMMU (0.2) and SMBus/LPC (14.0/14.3) */
	for (unsigned int slot = 0; slot <= 0x14; slot += 0x14)
		for (unsigned int pin = 0; pin < ARRAY_SIZE(links); pin++)
			acpigen_write_PRT_source_entry(slot, pin, links[pin], 0);
}

/*
 * The _PRT of a die's root bus. The FCH devices on die 0 interrupt through the
 * INTA-D links. Root bus bridges with an INTx pin of their own (the internal
 * bridges 7.1 and 8.1; the GPP bridges have none) interrupt at their bridge IRQ
 * of the die's NBIO IOAPIC, as the vendor DSDT has it. That IOAPIC has no PIC
 * route, so the PIC table leaves them out.
 */
static void sp3_write_root_prt(const struct device *domain)
{
	const bool fch = dev_get_domain_id(domain) == 0;
	struct root_irq bridges[8];
	size_t count = 0;

	for (struct device *dev = domain->downstream->children; dev; dev = dev->sibling) {
		const struct pci_routing_info *info;
		uint8_t pin;

		if (!dev->enabled || dev->path.type != DEVICE_PATH_PCI ||
		    (dev->hdr_type & 0x7f) != PCI_HEADER_TYPE_BRIDGE)
			continue;
		pin = pci_read_config8(dev, PCI_INTERRUPT_PIN);
		if (!pin || pin > 4 || count == ARRAY_SIZE(bridges))
			continue;
		info = get_pci_routing_info(dev->path.pci.devfn);
		if (!info)
			continue;
		bridges[count++] = (struct root_irq) {
			.slot = PCI_SLOT(dev->path.pci.devfn),
			.pin = pin - 1,
			.gsi = soc_get_gsi_base(dev) + info->bridge_irq,
		};
	}
	if (!fch && !count)
		return;

	acpigen_write_scope(acpi_device_path(domain));
	acpigen_write_method("_PRT", 0);

	acpigen_write_if();
	acpigen_emit_namestring("PICM");
	acpigen_emit_byte(RETURN_OP);
	acpigen_write_package((fch ? 8 : 0) + count);
	if (fch)
		write_fch_prt_entries();
	for (size_t i = 0; i < count; i++)
		acpigen_write_PRT_GSI_entry(bridges[i].slot, bridges[i].pin, bridges[i].gsi);
	acpigen_pop_len(); /* Package */

	acpigen_write_else();
	acpigen_emit_byte(RETURN_OP);
	acpigen_write_package(fch ? 8 : 0);
	if (fch)
		write_fch_prt_entries();
	acpigen_pop_len(); /* Package */
	acpigen_pop_len(); /* Else */

	acpigen_pop_len(); /* Method */
	acpigen_pop_len(); /* Scope */
}

static void sp3_domain_fill_ssdt(const struct device *domain)
{
	const unsigned int id = dev_get_domain_id(domain);

	amd_pci_domain_fill_ssdt(domain);
	sp3_write_root_prt(domain);

	acpigen_write_scope(acpi_device_path(domain));
	acpigen_write_name_integer("_UID", id);
	acpigen_pop_len();

	/* The DSDT has a root bridge for every possible die; hide the absent ones */
	if (id != 0)
		return;
	for (unsigned int die = sp3_die_count(); die < SP3_MAX_DIES; die++) {
		char scope[16];

		snprintf(scope, sizeof(scope), "\\_SB.%s", domain_acpi_names[die]);
		acpigen_write_scope(scope);
		acpigen_write_STA(ACPI_STATUS_DEVICE_ALL_OFF);
		acpigen_pop_len();
	}
}

struct device_operations sp3_pci_domain_ops = {
	.read_resources	= sp3_domain_read_resources,
	.set_resources	= sp3_domain_set_resources,
	.scan_bus	= amd_pci_domain_scan_bus,
	.enable		= sp3_domain_enable,
	.init		= amd_pci_domain_init,
	.acpi_name	= sp3_domain_acpi_name,
	.acpi_fill_ssdt	= sp3_domain_fill_ssdt,
};
