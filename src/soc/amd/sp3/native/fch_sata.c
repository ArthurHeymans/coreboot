/* SPDX-License-Identifier: GPL-2.0-only */

#include <console/console.h>
#include <delay.h>
#include <device/mmio.h>
#include <types.h>
#include "fch.h"
#include "topology.h"

static const uint32_t phy[] = { 0x13c39404, 0x13c39804, 0x13d39404, 0x13d39804 };
static const uint32_t nbif[] = { 0x10635000, 0x10636000, 0x10735000, 0x10736000 };
static const uint8_t esp[] = { 0x3f, 0xff, 0 };

static void update(uint32_t address, uint32_t clear, uint32_t set)
{
	sp3_smn_clrsetbits32(0, address, clear, set);
}

void sp3_fch_sata_reset(unsigned int socket)
{
	static const uint32_t unused[] = { 0x13b39404, 0x13b39804, 0x13e39404, 0x13e39804 };

	for (unsigned int i = 0; i < ARRAY_SIZE(unused); i++)
		update(unused[i], 0, 7);
	for (unsigned int c = 0; c < 3; c++) {
		update(0x03100000 + c * MiB, 1, 0);
		update(phy[c], 7, 0);
	}
}

static bool sgpio(unsigned int c)
{
	const uint32_t base = 0x03100000 + c * MiB;
	const uint32_t a = base + 0x1000;
	static const uint16_t pins[2][4] = { { 9, 10, 23, 40 }, { 260, 261, 262, 263 } };
	static const uint32_t messages[3][5] = {
		{ 0x3f0000, 0xc00000, 0xfe, 1, 0x20 },
		{ 0xfffc0000, 0x30000, 0xfe, 1, 0xa0a0a0a0 },
		{ 0xffff0000, 0, 0xfd, 2, 0x800000 },
	};

	for (unsigned int i = 0; i < 4; i++) {
		const unsigned int pin = pins[c][i];
		const uintptr_t mux = 0xfed80000 + (pin < 256 ? 0xd00 + pin : 0x12c0 + pin - 256);
		const uintptr_t ctl = 0xfed80000 + (pin < 256 ? 0x1500 + 4 * pin :
			0x1200 + 4 * (pin - 256));

		write8p(mux, c == 0 && i == 1 ? 2 : 1);
		if (i != 3)
			write32p(ctl, (read32p(ctl) & ~BIT(21)) | BIT(20));
	}
	update(base, 0x01000007, 8);
	update(a + 0xfc, 0x80000, 0x0c100c02);
	for (unsigned int i = 0; i < 6; i++) {
		const uint32_t *m = messages[i % 3];

		update(a + 0x504, m[0], m[1]);
		update(a + 0x504, i % 3 == 0 ? 0xff3f0000 : m[0], m[1]);
		update(a + 0x508, m[2], m[3]);
		sp3_smn_write32(0, a + 0x50c, m[4] | (i == 3 ? 1 : 0));
		if (i % 3 == 2)
			sp3_smn_write32(0, a + 0x510, 0x0f0f3700);
		update(a + 0x20, 0, BIT(8));
		unsigned int polls;
		for (polls = 0; polls < 1000; polls++) {
			if (!(sp3_smn_read32(0, a + 0x20) & BIT(8)))
				break;
			udelay(10);
		}
		if (polls == 1000)
			return false;
	}
	return true;
}

void sp3_fch_sata_init(unsigned int socket)
{
	for (unsigned int c = 0; c < 3; c++) {
		const uint32_t base = 0x03100000 + c * MiB;
		const uint32_t a = base + 0x1000;

		for (unsigned int i = 0; i < 8; i++)
			update(base + 4, (2 << i) - 1, 0);
		update(a + 0x800, 0, 1);
		update(base, 0x0100000f, 0);
		update(a + 0xfc, 0x08080000, 0x04000c02);
		update(a + 0xf8, 0, 0xff000000);
		update(a + 0x24, 1, 0);
		update(a + 0x810, 0, 0x0200c304);
		update(a + 0x814, 0, 0x400001e0);
		update(a + 0x800, 0, 0x2001);
		update(nbif[c] + 0xc, 0, BIT(18));
		update(a + 0x800, 0xff0000, 0x2001);
		update(a + 0x804, 0, 0x400000);
		if (esp[c]) {
			update(a + 0xf8, esp[c], 0xff000000);
			update(a + 0xf8, ((~esp[c] & 0xff) << 16) | esp[c],
				esp[c] << 16 | 0xff000000);
			update(a + 0xfc, 0x08080000, 0x04100c02);
			update(a + 0x810, 0, 0x2200c304);
		} else {
			update(a + 0xf8, 0x00ff00ff, 0xff000000);
			update(a + 0xfc, 0x08180000, 0x04000c02);
			update(a + 0x810, BIT(29), 0x0200c304);
		}
		for (unsigned int p = 0; p < 8; p++) {
			sp3_smn_write32(0, a + 0x130 + 0x80 * p, 0xffffffff);
			const uint32_t status = a + 0x110 + 0x80 * p;
			sp3_smn_write32(0, status, sp3_smn_read32(0, status));
		}
		if (c < 2 && !sgpio(c))
			printk(BIOS_WARNING, "SATA%u SGPIO setup timeout\n", c);
		if (!c) {
			write8p(0xfed80d05, 0);
			write8p(0xfed80d06, 0);
			update(a + 0xf4, 0x310, 0);
			for (unsigned int p = 0; p < 8; p++)
				update(a + 0x88c + 0x20 * p, 0x700, p << 8);
		}
		sp3_smn_write32(0, nbif[c] + 0x34, 0x00010601);
		update(nbif[c], 0x86fe, 0x7901);
		update(a + 0x800, 0xff0001, 0x2000);
	}
	update(0x03400000, 0, 0x801);
	update(phy[3], 0, 7);
}

void sp3_fch_sata_late(unsigned int socket, pci_devfn_t unused)
{
	for (unsigned int c = 0; c < 3; c++) {
		const uint32_t a = 0x03101000 + c * MiB;
		uint32_t disabled = 0;

		update(a + 0x800, 0xff0000, 0x2001);
		for (unsigned int p = 0; p < 8; p++) {
			if ((esp[c] & BIT(p)) || (sp3_smn_read32(0, a + 0x128 + 0x80 * p) & 0xf) == 3)
				continue;
			update(a + 0x12c + 0x80 * p, 0xb, 4);
			disabled |= BIT(p);
			update(a + 0x800, 0xff0000, disabled << 16 | 0x2001);
			update(a + 0xc, disabled, 0);
		}
		uint32_t pi = sp3_smn_read32(0, a + 0xc) & 0xff;
		if (!pi) {
			update(a + 0xc, 0xfe, 1);
			pi = 1;
		}
		update(a, 0x1f, __builtin_popcount(pi) - 1);
		update(a + 0x800, 1, 0x2000);
	}
}
