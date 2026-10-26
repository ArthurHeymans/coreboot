/* SPDX-License-Identifier: GPL-2.0-only */

/* Rome DXIO and port setup, PCIE_DXIO_SP3 sections 3.3-3.10. */
#include <console/console.h>
#include <delay.h>
#include <device/pci_def.h>
#include <soc/platform_descriptors.h>
#include <string.h>
#include <timer.h>
#include "nbio.h"
#include "topology.h"

#define MAX_ENGINES 40
struct fw_engine {
	uint8_t type, flags, start, end;
	uint32_t groups;
	uint8_t channel, reserved9;
	uint16_t ancillary;
	uint32_t bdf, caps, caps_hi;
	uint8_t mac, port, returned[10], train, reserved[3];
} __packed;
_Static_assert(sizeof(struct fw_engine) == 0x28, "DXIO engine ABI");

struct engine_heap {
	uint8_t reserved[12];
	uint16_t count, reserved2;
	struct fw_engine engines[MAX_ENGINES];
	uint8_t tail[8];
} __packed;

struct lane_map {
	uint8_t nbio, core, phy, wire;
	bool reversed;
};
static const struct lane_map lanes[] = {
	{ 0, 0, 16, 16, true }, { 0, 1, 0, 42, false },
	{ 1, 0, 32, 58, false }, { 1, 1, 48, 0, true },
	{ 2, 0, 96, 114, false }, { 2, 1, 112, 90, true },
	{ 3, 0, 80, 74, true }, { 3, 1, 64, 130, false },
};
static const uint8_t defaults[18] = {
	0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x11,
	0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x21, 0x29, 0x2a,
};
struct mapped {
	const struct lane_map *lane;
	unsigned int rb;
	uint8_t port, devfn, bridge;
	bool valid;
};
static struct engine_heap heap __aligned(4096);
static uint32_t ancillary[512] __aligned(4096);
static struct mapped ports[MAX_ENGINES];
static const dxio_descriptor *board;
static size_t count, heap_size, ancillary_size;

static const struct lane_map *placement(const dxio_descriptor *d)
{
	const uint8_t lo = MIN(d->start_logical_lane, d->end_logical_lane);
	const uint8_t hi = MAX(d->start_logical_lane, d->end_logical_lane);

	for (size_t i = 0; i < ARRAY_SIZE(lanes); i++)
		if (lo >= lanes[i].phy && hi < lanes[i].phy + 16)
			return &lanes[i];
	die("DXIO PHY lanes %u-%u cross a core\n", lo, hi);
}

static uint8_t wire(const struct lane_map *l, uint8_t phy)
{
	return l->reversed ? l->wire + 15 - (phy - l->phy) : l->wire + phy - l->phy;
}

static uint32_t port_reg(const struct mapped *p, uint32_t reg)
{
	return 0x11140000 + (p->lane->nbio << 20) + (p->lane->core << 22) +
		(p->port << 12) + reg;
}

static uint32_t core_reg(const struct mapped *p, uint32_t reg)
{
	return 0x11180000 + (p->lane->nbio << 20) + (p->lane->core << 22) + reg;
}

static uint32_t cfg_reg(const struct mapped *p, uint32_t reg)
{
	return port_reg(p, reg) - 0x40000;
}

static void modify(uint32_t reg, uint32_t clear, uint32_t set)
{
	sp3_smn_clrsetbits32(0, reg, clear, set);
}

static void strap(const struct mapped *p, uint16_t index, uint32_t value)
{
	const uint32_t reg = 0x090467ac + (p->lane->nbio << 12) + (p->lane->core << 14);

	sp3_smn_write32(0, reg, 0xfffe0000 | index);
	sp3_smn_write32(0, reg + 4, value);
}

static enum cb_err raw(uint32_t op, uint32_t args[6])
{
	struct smu_payload p;

	memcpy(p.msg, args, sizeof(p.msg));
	p.msg[0] = op;
	if (sp3_smu_send(0, 8, &p))
		return CB_ERR;
	asm volatile ("" ::: "memory");
	memcpy(args, p.msg, sizeof(p.msg));
	return CB_SUCCESS;
}

static enum cb_err rpc(uint32_t op, uint32_t a, uint32_t b, uint32_t c, uint32_t d, uint32_t e)
{
	uint32_t args[6] = { op, a, b, c, d, e };

	if (raw(op, args))
		return CB_ERR;
	printk(BIOS_DEBUG, "DXIO op %x: %x %x %x %x %x %x\n", op,
	       args[0], args[1], args[2], args[3], args[4], args[5]);
	if ((args[0] & 0xffffff00) != 0x300 && (args[0] & 0xff) != 3) {
		printk(BIOS_ERR, "DXIO op %x failed: %x\n", op, args[0]);
		return CB_ERR;
	}
	return CB_SUCCESS;
}

static enum cb_err get_engines(void)
{
	if (rpc(0x14, 0, (uintptr_t)&heap, heap_size / 4, 0, 0))
		return CB_ERR;
	if (heap.count != count) {
		printk(BIOS_ERR, "DXIO changed engine count %u/%zu\n", heap.count, count);
		return CB_ERR;
	}
	return CB_SUCCESS;
}

static void build_heaps(void)
{
	size_t a = 1;
	const uint32_t fuse = sp3_smn_read32(0, 0x5d698);

	memset(&heap, 0, sizeof(heap));
	memset(ancillary, 0, sizeof(ancillary));
	for (unsigned int i = 0; i < count; i++) {
		const dxio_descriptor *d = &board[i];
		struct fw_engine *e = &heap.engines[i];
		const struct lane_map *l = placement(d);
		const uint8_t lo = wire(l, MIN(d->start_logical_lane, d->end_logical_lane));
		const uint8_t hi = wire(l, MAX(d->start_logical_lane, d->end_logical_lane));

		e->type = d->engine_type;
		e->start = MIN(lo, hi);
		e->end = MAX(lo, hi);
		e->groups = 1;
		if (d->engine_type == PCIE_ENGINE) {
			unsigned int speed = d->link_speed_capability;
			const bool reversed = (d->start_logical_lane > d->end_logical_lane) ^ l->reversed;

			if (!speed || speed > 2) {
				if (!(fuse & 0x1400))
					speed = 2;
				else
					e->caps_hi = 7; /* EQ override, preset search. */
			}
			e->bdf = d->start_logical_lane << 8 | d->end_logical_lane;
			e->caps = d->port_present | (reversed << 3) | (speed << 4) | 0x180000;
		}
		if (d->tuning_count) {
			const size_t record = a++;

			e->ancillary = record;
			for (size_t j = 0; j < d->tuning_count; j++) {
				const uint8_t id = d->tuning[j].id;
				const uint8_t low = id & 0x3f;
				const uint8_t code = low == 0x12 ? 0x14 : low == 0x13 ? 0x15 : low;

				if (!(low <= 2 || low == 0x12 || low == 0x13))
					die("Unsupported SATA tuning code\n");
				for (unsigned int lane = 0; lane <= e->end - e->start; lane++) {
					if (a + 1 >= ARRAY_SIZE(ancillary))
						die("DXIO ancillary heap overflow\n");
					ancillary[a++] = code | (id >> 6) << 7 | lane << 10 |
						d->tuning[j].value << 16;
				}
			}
			ancillary[record] = 0x203 | (a - record) << 16;
		}
	}
	heap.count = count;
	heap_size = 0x18 + count * 0x28;
	if (a > 1) {
		ancillary[0] = a << 16;
		ancillary_size = (a + 1) * 4;
	}
}

static enum cb_err mapped(void)
{
	uint8_t requested[4][18] = { 0 };
	uint8_t remap[4][18];

	for (unsigned int i = 0; i < count; i++) {
		const struct fw_engine *e = &heap.engines[i];
		const dxio_descriptor *d = &board[i];
		struct mapped *p = &ports[i];
		const struct lane_map *l = placement(d);
		const uint8_t w = MIN(wire(l, d->start_logical_lane), wire(l, d->end_logical_lane));

		if (e->type != PCIE_ENGINE)
			continue;
		if (e->start != w || e->port >= 8)
			return CB_ERR;
		p->lane = l;
		p->port = e->port;
		p->valid = true;
		for (p->rb = 0; p->rb < sp3_die_count(); p->rb++)
			if (sp3_nbio_index(p->rb) == l->nbio)
				break;
		if (p->rb == sp3_die_count())
			return CB_ERR;
		const uint8_t devfn = PCI_DEVFN(d->device_number, d->function_number);
		const unsigned int index = l->core * 8 + p->port;
		if (requested[l->nbio][index])
			return CB_ERR;
		requested[l->nbio][index] = devfn;
	}
	for (unsigned int n = 0; n < 4; n++) {
		unsigned int next = 0;

		for (unsigned int i = 0; i < 18; i++) {
			uint8_t value = requested[n][i];

			if (!value) {
				bool used;

				do {
					if (next == 18)
						return CB_ERR;
					value = defaults[next++];
					used = false;
					for (unsigned int j = 0; j < 18; j++)
						used |= requested[n][j] == value;
				} while (used);
			}
			remap[n][i] = value;
			modify(0x13b100b8 + (n << 20) + 4 * i, 0xff, value);
		}
	}
	for (unsigned int i = 0; i < count; i++) {
		struct mapped *p = &ports[i];

		if (!p->valid)
			continue;
		p->devfn = remap[p->lane->nbio][p->lane->core * 8 + p->port];
		for (p->bridge = 0; p->bridge < 18; p->bridge++)
			if (defaults[p->bridge] == p->devfn)
				break;
		if (p->bridge == 18)
			return CB_ERR;
		static const uint8_t eq[] = { 3, 3, 7, 7, 7, 5 };
		for (unsigned int j = 0; j < ARRAY_SIZE(eq); j++)
			strap(p, 0x9f + j, eq[j]);
		modify(port_reg(p, 0x2d8), BIT(16) | BIT(11) | 0x300,
		       heap.engines[i].caps_hi & 4 ? 0x300 : 0);
		modify(port_reg(p, 0x374), BIT(11) | BIT(8) | 3, 0);
		const unsigned int width = MAX(board[i].start_logical_lane, board[i].end_logical_lane) -
			MIN(board[i].start_logical_lane, board[i].end_logical_lane) + 1;
		if (width == 1 || width == 4)
			strap(p, 0xf9 + 0x61 * p->port, 2);
		modify(port_reg(p, 0x390), 0, BIT(13));
		modify(port_reg(p, 0x80), 0, BIT(20));
	}
	return CB_SUCCESS;
}

static void configured(void)
{
	for (unsigned int i = 0; i < count; i++) {
		const struct mapped *p = &ports[i];

		if (!p->valid)
			continue;
		const uint32_t dpc = cfg_reg(p, 0x388);
		sp3_smn_write32(0, dpc, sp3_smn_read32(0, dpc));
		modify(port_reg(p, 0x80), BIT(15), 0);
		modify(core_reg(p, 0x4d4), 0, BIT(2));
		modify(core_reg(p, 0x4d8), 0xfff80, 0xc97 << 7);
		modify(core_reg(p, 0x4dc), 0xf80000, 0xf << 19);
		modify(core_reg(p, 0x80), 0, BIT(29) | BIT(31));
		modify(core_reg(p, 0x18c), 0xf, p->lane->core + 2);
	}
}

static bool paused;

static enum cb_err run(bool first_pass)
{
	struct stopwatch sw;

	stopwatch_init_msecs_expire(&sw, 60000);
	while (!stopwatch_expired(&sw)) {
		uint32_t args[6] = { 0 };

		if (raw(9, args))
			return CB_ERR;
		const uint32_t result = (args[0] & 0xffffff00) == 0x300 ? 3 : args[0] & 0xff;
		const uint32_t kind = args[1] & 0xff;

		printk(BIOS_DEBUG, "DXIO reply %x %x %x %x %x %x\n",
		       args[0], args[1], args[2], args[3], args[4], args[5]);
		if (result != 3 || (kind == 2 && args[2] == 0xe)) {
			printk(BIOS_ERR, "DXIO state error: %x %x %x\n", args[0], args[1], args[2]);
			return CB_ERR;
		}
		if (kind == 0 || (kind == 2 && args[2] == 0xd))
			return CB_SUCCESS;
		if (kind == 2) {
			printk(BIOS_DEBUG, "DXIO state %x\n", args[2]);
			if (args[2] == 0xf && (get_engines() || mapped()))
				return CB_ERR;
			if (args[2] == 5) {
				configured();
				if (first_pass) {
					paused = true;
					return CB_SUCCESS;
				}
			}
		}
		if (kind == 4) {
			static const uint16_t pins[] = { 0x1568, 0x156c, 0x1228, 0x122c };

			for (unsigned int j = 0; j < ARRAY_SIZE(pins); j++)
				sp3_smn_write32(0, 0x02d01000 + pins[j], 0xc40000);
			mdelay(100);
		}
		if (rpc(0x308, 0, 0, 0, 0, 0))
			return CB_ERR;
	}
	printk(BIOS_ERR, "DXIO training timeout\n");
	return CB_ERR;
}

static void finish(void)
{
	for (unsigned int rb = 0; rb < sp3_die_count(); rb++) {
		const uint32_t q = sp3_nbio_index(rb) << 20;

		for (unsigned int j = 0; j < 18; j++)
			modify(0x13b31004 + q + 0x400 * j, 0x40005, 0x40005);
		/* Every IOHC learns routes in physical root-bridge order. */
		const struct sp3_topology *t = sp3_topology();
		for (unsigned int j = 0; j < t->root_bridge_count; j++) {
			const struct sp3_root_bridge *r = &t->root_bridges[j];

			sp3_smn_write32(0, 0x13b10190 + q + j * 0x14,
				r->bus_limit << 8 | r->bus_base |
				(r->index != sp3_nbio_index(rb) ? BIT(31) : 0));
		}
	}
	for (unsigned int i = 0; i < count; i++) {
		const struct mapped *p = &ports[i];

		if (!p->valid)
			continue;
		const uint32_t state = sp3_smn_read32(0, port_reg(p, 0x294));
		const bool trained = (state & 0x3f) >= 0x10 && (state & 0x3f) < 0x1c;
		printk(BIOS_INFO, "PCIe %02x:%02x.%x PHY %u-%u: %s (LTSSM %08x)\n",
		       sp3_die_bus(p->rb), PCI_SLOT(p->devfn), PCI_FUNC(p->devfn),
		       board[i].start_logical_lane, board[i].end_logical_lane,
		       trained ? "trained" : "empty", state);
		modify(core_reg(p, 0x2c0), 0, BIT(18) | BIT(20));
		modify(core_reg(p, 0x40), BIT(17), BIT(21));
		modify(core_reg(p, 0x70), 0x6000, 0x4000);
		modify(core_reg(p, 0x100), 0, BIT(14) | BIT(6));
		if (!trained)
			continue;
		modify(0x13b31004 + (p->lane->nbio << 20) + 0x400 * p->bridge, 0x40005, 0x40000);
		modify(port_reg(p, 0x84), 0xffff, sp3_die_bus(p->rb) << 8 | p->devfn);
		modify(cfg_reg(p, 0x80), 0xf, 6);
		modify(port_reg(p, 0x280), 0, BIT(23));
		modify(port_reg(p, 0x288), 0, BIT(19));
		modify(port_reg(p, 0x2c4), 0x180c000, 0x1004000);
		modify(port_reg(p, 8), 0, BIT(15));
	}
	modify(0x1053a008, 0, BIT(8) | BIT(16));
	modify(0x10535000, 0, BIT(28));
	modify(0x10536000, 0, BIT(28));
}

enum cb_err sp3_dxio_init(void)
{
	mainboard_get_dxio_descriptors(0, &board, &count);
	if (!count || count > MAX_ENGINES)
		return CB_ERR;
	build_heaps();
	if (rpc(0x2d, 1, 0, 0, 0, 0) || rpc(0x13, 0, 0, 0, 0, 0) ||
	    rpc(0, 0, 0, 0, 0, 0) || rpc(0x3a, 1, 3, 0, 0, 0) ||
	    rpc(0x22, 0x24, 0, 0, 0, 0) || rpc(0x22, 0xd, 1, 0, 0, 0) ||
	    rpc(0x22, 0xe, 1, 0, 0, 0) || rpc(0x22, 0xf, 1, 0, 0, 0) ||
	    rpc(0x22, 0x10, 1, 0, 0, 0) || rpc(0x22, 0x14, 1, 0, 0, 0) ||
	    rpc(0x24, 0, 0, 0, 0, 0))
		return CB_ERR;
	if (ancillary_size && rpc(0x23, 0, (uintptr_t)ancillary, ancillary_size / 4, 0, 6))
		return CB_ERR;
	if (rpc(0x23, 0, 0, 0, 1, 2) || rpc(0x23, 0, 0, 0, 1, 5) ||
	    rpc(0x23, 0, (uintptr_t)&heap, heap_size / 4, 0, 3) ||
	    rpc(0x3a, 1, 0, 9, 1, 0) || rpc(0x307, 0, 0, 0, 0, 0) || run(true))
		return CB_ERR;
	if (paused && (rpc(0x308, 0, 0, 0, 0, 0) || run(false)))
		return CB_ERR;
	if (get_engines())
		return CB_ERR;
	finish();
	return CB_SUCCESS;
}
