/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/acpi.h>
#include <amdblocks/amd_pci_util.h>
#include <device/device.h>
#include <soc/amd_pci_int_defs.h>
#include <soc/southbridge.h>
#include <types.h>

#include "mainboard.h"

/*
 * The FCH PCI interrupt routing at 0xc00/0xc01. The IOAPIC side matches the
 * vendor firmware; the vendor PIC values aren't known, so INTA-D are left for
 * the OS to route through the ACPI link devices.
 */
static const struct fch_irq_routing fch_irq_map[] = {
	{ PIRQ_A,	PIRQ_NC,	16 },
	{ PIRQ_B,	PIRQ_NC,	17 },
	{ PIRQ_C,	PIRQ_NC,	18 },
	{ PIRQ_D,	PIRQ_NC,	19 },
	{ PIRQ_SCI,	ACPI_SCI_IRQ,	ACPI_SCI_IRQ },
	{ PIRQ_GPIO,	7,		7 },
};

const struct fch_irq_routing *mb_get_fch_irq_mapping(size_t *length)
{
	*length = ARRAY_SIZE(fch_irq_map);
	return fch_irq_map;
}

static void mainboard_init(void *chip_info)
{
	mainboard_program_gpios();
}

struct chip_operations mainboard_ops = {
	.init = mainboard_init,
};
