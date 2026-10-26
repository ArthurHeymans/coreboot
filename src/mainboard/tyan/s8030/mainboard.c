/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/acpi.h>
#include <amdblocks/amd_pci_util.h>
#include <device/device.h>
#include <soc/amd_pci_int_defs.h>
#include <soc/southbridge.h>

static struct fch_irq_routing irq_map[] = {
	{ PIRQ_A, PIRQ_NC, 16 },
	{ PIRQ_B, PIRQ_NC, 17 },
	{ PIRQ_C, PIRQ_NC, 18 },
	{ PIRQ_D, PIRQ_NC, 19 },
	{ PIRQ_SCI, ACPI_SCI_IRQ, ACPI_SCI_IRQ },
	{ PIRQ_GPIO, 7, 7 },
	{ PIRQ_MISC, 0, 0 },
	{ PIRQ_MISC0, 0, 0 },
	{ PIRQ_MISC1, 0, 0 },
	{ PIRQ_MISC2, 0, 0 },
};

const struct fch_irq_routing *mb_get_fch_irq_mapping(size_t *length)
{
	/* Misc indices contain control bits, not IRQ numbers. Preserve timer routing. */
	for (size_t i = 0; i < ARRAY_SIZE(irq_map); i++) {
		const uint8_t index = irq_map[i].intr_index;

		if (index < PIRQ_MISC || index > PIRQ_MISC2)
			continue;
		irq_map[i].pic_irq_num = read_pci_int_idx(index, 0);
		irq_map[i].apic_irq_num = read_pci_int_idx(index, 1);
	}
	*length = ARRAY_SIZE(irq_map);
	return irq_map;
}

struct chip_operations mainboard_ops = {
	.name = "Tyan S8030",
};
