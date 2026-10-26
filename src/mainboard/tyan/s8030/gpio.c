/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/acpimmio.h>
#include <device/mmio.h>
#include <device/pci_ops.h>
#include <soc/native.h>
#include <types.h>

/* S8030 CPM $A10, observed table at Rome VA ffe46ac0. */
static const struct { uint16_t pin, setting; } pads[] = {
	{ 104, 0x8a8 },
	{ 105, 0x8a8 },
	{ 106, 0x8a8 },
	{ 107, 0x8a8 },
	{ 109, 0x888 },
	{ 74, 0x888 },
	{ 75, 0x888 },
	{ 21, 0x888 },
	{ 108, 0x8c8 },
	{ 88, 0x8a8 },
	{ 22, 0x8a8 },
	{ 87, 0x8a8 },
	{ 16, 0x8a8 },
	{ 17, 0x8a8 },
	{ 264, 0x8a8 },
	{ 265, 0x8a8 },
	{ 3, 0x881 },
	{ 6, 0x881 },
	{ 145, 0x888 },
	{ 146, 0x888 },
	{ 147, 0x888 },
	{ 148, 0x888 },
	{ 113, 0x888 },
	{ 114, 0x888 },
	{ 19, 0x888 },
	{ 20, 0x888 },
	{ 151, 0x888 },
	{ 152, 0x888 },
	{ 135, 0x8c8 },
	{ 137, 0x8a8 },
	{ 136, 0x8c8 },
	{ 138, 0x8a8 },
	{ 140, 0xaa2 },
	{ 144, 0x9a2 },
	{ 143, 0x9a1 },
	{ 89, 0xa82 },
	{ 86, 0x9a2 },
	{ 70, 0x882 },
	{ 29, 0x8a1 },
	{ 32, 0x8a1 },
	{ 258, 0x8a2 },
	{ 259, 0x8a1 },
	{ 10, 0xa88 },
	{ 40, 0x988 },
	{ 9, 0x988 },
	{ 23, 0x988 },
	{ 262, 0x988 },
	{ 263, 0x988 },
	{ 260, 0x988 },
	{ 261, 0x988 },
	{ 256, 0x880 },
	{ 257, 0x880 },
	{ 30, 0x882 },
	{ 31, 0x882 },
	{ 142, 0xa82 },
	{ 141, 0x982 },
	{ 42, 0x881 },
	{ 129, 0x9a2 },
	{ 0, 0x8a8 },
	{ 1, 0x8a8 },
	{ 2, 0x8a8 },
	{ 26, 0x8a8 },
	{ 266, 0x8a8 },
	{ 267, 0x8a8 },
	{ 27, 0x8a8 },
	{ 76, 0x888 },
	{ 118, 0x888 },
};

static void configure(unsigned int pin, uint16_t setting)
{
	const uintptr_t ctl = 0xfed80000 + (pin < 256 ? 0x1500 + 4 * pin :
		0x1200 + 4 * (pin - 256));
	const uintptr_t mux = 0xfed80000 + (pin < 256 ? 0xd00 + pin :
		0x12c0 + pin - 256);
	uint32_t value = read32p(ctl);

	if (setting & 0x800)
		write8p(mux, (setting >> 8) & 7);
	if (setting & 0x80) {
		value &= ~(BIT(20) | BIT(21));
		value |= setting & 0x20 ? BIT(20) : setting & 0x40 ? BIT(21) : 0;
	}
	if (!(setting & 8)) {
		value &= ~(BIT(23) | BIT(22));
		if (!(setting & 2))
			value |= BIT(23) | (setting & 1 ? BIT(22) : 0);
	}
	write32p(ctl, value);
}

void mainboard_fch_reset_done(void)
{
	const pci_devfn_t lpc = PCI_DEV(0, 0x14, 3);

	pci_s_write_config16(lpc, 0x66, 0xa00);
	pci_s_write_config16(lpc, 0x90, 0xca0);
	pci_s_write_config8(lpc, 0x74, (pci_s_read_config8(lpc, 0x74) & ~4) | 8);
	pci_s_write_config32(lpc, 0x48, pci_s_read_config32(lpc, 0x48) | 0x03000000);
	configure(144, 0x982);
	configure(140, 0xa82);
	configure(4, 0x8a1);
	configure(5, 0x8a1);
	for (size_t i = 0; i < ARRAY_SIZE(pads); i++)
		configure(pads[i].pin, pads[i].setting);
}
