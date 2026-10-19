/* SPDX-License-Identifier: GPL-2.0-only */

/*
 * DXIO (PCIe/SATA lane) training and the PCIe setup around it, following the
 * default path of the vendor AmdNbioPcieZPPei (sp4r2 report NBIO_PCIE_SMU_ZP
 * section 3). The DXIO firmware runs on the SMU of each die and is reached
 * through SMU message 0x0f with the DXIO operation in the first argument.
 */

#include <console/console.h>
#include <device/pci_def.h>
#include <device/pci_ops.h>
#include <soc/smu.h>
#include <soc/soc_util.h>
#include <string.h>
#include <timer.h>
#include <types.h>

#include "nbio.h"
#include "smu.h"
#include "topology.h"

enum dxio_op {
	DXIO_OP_GET_VERSION = 0x00,
	DXIO_OP_GET_SM_STATE = 0x09,
	DXIO_OP_INIT = 0x13,
	DXIO_OP_GET_ENGINE_CONFIG = 0x14,
	DXIO_OP_SET_VAR = 0x22,
	DXIO_OP_LOAD_DATA = 0x23,
	DXIO_OP_LOAD_CAPS = 0x24,
	DXIO_OP_CLEANUP_EARLY = 0x2d,
	DXIO_OP_POST_TRAINING = 0x34,
	DXIO_OP_SM_START = 0x307,
	DXIO_OP_SM_RESUME = 0x308,
};

#define DXIO_RESULT_ERROR		2
#define DXIO_RESULT_OK			3

#define DXIO_VAR_RET_AFTER_MAP		0x0e
#define DXIO_VAR_RET_AFTER_CONF		0x0f
#define DXIO_VAR_ANCILLARY_V1		0x10

#define DXIO_DATA_MACPCS		2
#define DXIO_DATA_ENGINE_CONFIG		3
#define DXIO_DATA_GPIO			5

#define DXIO_EVENT_NONE			0
#define DXIO_EVENT_RESUME		1
#define DXIO_EVENT_SM			2
#define DXIO_EVENT_PERST		4

#define DXIO_SM_CONFIGURED		0x05
#define DXIO_SM_DONE			0x0d
#define DXIO_SM_MAPPED			0x0f

#define DXIO_TIMEOUT_MS			60000

/* Engine record of the platform descriptor that gets passed to the DXIO firmware */
struct dxio_engine {
	uint8_t type;
	uint8_t hotplug;
	uint8_t start_lane;	/* die-relative; lanes above 15 are shifted by 4 */
	uint8_t end_lane;
	uint8_t gpio_group;	/* always 1 */
	uint8_t gpio_group2;	/* the descriptor's GPIO group */
	uint8_t reserved_06[2];
	uint8_t sata_channel;
	uint8_t reserved_09[3];
	uint32_t devfn;
	uint32_t port_flags;
	uint8_t reserved_14[5];
	uint8_t port;		/* port of its PCIe core, assigned by the DXIO firmware */
	uint8_t reserved_1a[14];
} __packed;

_Static_assert(sizeof(struct dxio_engine) == 0x28, "Unexpected DXIO engine size");

#define DXIO_PORT_PRESENT		BIT(0)
#define DXIO_PORT_LANE_REVERSAL		BIT(3)
#define DXIO_PORT_SPEED_SHIFT		4
#define DXIO_PORT_FLAG_19		BIT(19)
#define DXIO_PORT_TURN_OFF_UNUSED	BIT(20)

#define DXIO_ENGINE_MAX			16

struct dxio_engine_config {
	uint8_t reserved_00[0x0c];
	uint16_t engine_count;
	uint8_t reserved_0e[2];
	struct dxio_engine engines[DXIO_ENGINE_MAX];
	uint8_t reserved_tail[8];
} __packed __aligned(16);

/* Default devfn of the PCIe ports of a die, also the order of the bridge control registers */
static const uint8_t port_default_devfn[PCIE_PORT_COUNT] = {
	PCI_DEVFN(1, 1), PCI_DEVFN(1, 2), PCI_DEVFN(1, 3), PCI_DEVFN(1, 4),
	PCI_DEVFN(1, 5), PCI_DEVFN(1, 6), PCI_DEVFN(1, 7), PCI_DEVFN(2, 1),
	PCI_DEVFN(3, 1), PCI_DEVFN(3, 2), PCI_DEVFN(3, 3), PCI_DEVFN(3, 4),
	PCI_DEVFN(3, 5), PCI_DEVFN(3, 6), PCI_DEVFN(3, 7), PCI_DEVFN(4, 1),
};

/* State of the training of one die */
struct dxio_die {
	unsigned int die;
	const dxio_descriptor *dxio;
	size_t count;
	struct dxio_engine_config *config;
	size_t config_size;
	uint8_t devfn_map[PCIE_PORT_COUNT];	/* devfn of each port */
};

static enum cb_err dxio_rpc(unsigned int die, uint32_t op, uint32_t args[SMU_NUM_ARGS])
{
	struct smu_payload payload;

	memcpy(payload.msg, args, sizeof(payload.msg));
	payload.msg[0] = op;
	if (zeppelin_smu_send(die, SMC_MSG_DXIO_SERVICE, &payload)) {
		printk(BIOS_ERR, "DXIO operation 0x%x on die %u failed\n", op, die);
		return CB_ERR;
	}
	memcpy(args, payload.msg, sizeof(payload.msg));
	printk(BIOS_SPEW, "DXIO %u op 0x%x: %08x %08x %08x %08x %08x %08x\n", die, op,
	       args[0], args[1], args[2], args[3], args[4], args[5]);
	return CB_SUCCESS;
}

static enum cb_err dxio_simple_rpc(unsigned int die, uint32_t op)
{
	uint32_t args[SMU_NUM_ARGS] = { 0 };

	return dxio_rpc(die, op, args);
}

static enum cb_err dxio_set_var(unsigned int die, uint32_t var, uint32_t value)
{
	uint32_t args[SMU_NUM_ARGS] = { 0, var, value };

	return dxio_rpc(die, DXIO_OP_SET_VAR, args);
}

static enum cb_err dxio_load_data(unsigned int die, uint32_t type, const void *data,
				  size_t size)
{
	uint32_t args[SMU_NUM_ARGS] = { 0, 0, (uintptr_t)data, size / sizeof(uint32_t), 0,
					type };

	return dxio_rpc(die, DXIO_OP_LOAD_DATA, args);
}

/* Tell the DXIO firmware that there is no data of the given type */
static enum cb_err dxio_load_empty(unsigned int die, uint32_t type)
{
	uint32_t args[SMU_NUM_ARGS] = { 0, 0, 0, 0, 1, type };

	return dxio_rpc(die, DXIO_OP_LOAD_DATA, args);
}

static enum cb_err dxio_get_engine_config(struct dxio_die *d)
{
	uint32_t args[SMU_NUM_ARGS] = { 0, 0, (uintptr_t)d->config,
					d->config_size / sizeof(uint32_t) };

	memset(d->config, 0, d->config_size);
	if (dxio_rpc(d->die, DXIO_OP_GET_ENGINE_CONFIG, args))
		return CB_ERR;
	if (d->config->engine_count > DXIO_ENGINE_MAX) {
		printk(BIOS_ERR, "Invalid DXIO engine count %u\n", d->config->engine_count);
		return CB_ERR;
	}
	return CB_SUCCESS;
}

/* ---- Platform descriptor ---- */

static bool descriptor_active(const dxio_descriptor *e)
{
	return e->engine_type == PCIE_ENGINE || e->engine_type == SATA_ENGINE;
}

/* Die lane of a wire lane; the DXIO lane numbering skips four lanes after lane 15 */
static uint8_t wire_to_die_lane(uint8_t lane)
{
	return lane > 15 ? lane - 4 : lane;
}

static uint8_t die_to_wire_lane(uint8_t lane)
{
	return lane > 15 ? lane + 4 : lane;
}

static uint32_t engine_speed(dxio_link_speed_cap speed)
{
	/* Single-step training with the descriptor speed (vendor PCD 0xe6 = 0) */
	return speed < GEN_INVALID ? speed : GEN_MAX;
}

static void build_engine_config(struct dxio_die *d)
{
	struct dxio_engine_config *config = d->config;
	size_t engines = 0;

	memset(config, 0, sizeof(*config));
	for (size_t i = 0; i < d->count && engines < DXIO_ENGINE_MAX; i++) {
		const dxio_descriptor *desc = &d->dxio[i];
		const uint8_t start = MIN(desc->start_logical_lane, desc->end_logical_lane);
		const uint8_t end = MAX(desc->start_logical_lane, desc->end_logical_lane);
		struct dxio_engine *e;

		if (!descriptor_active(desc))
			continue;

		e = &config->engines[engines++];
		e->type = desc->engine_type;
		e->hotplug = desc->link_hotplug != HOTPLUG_DISABLED;
		e->start_lane = die_to_wire_lane(start);
		e->end_lane = die_to_wire_lane(end);
		e->gpio_group = 1;
		e->gpio_group2 = desc->gpio_group_id;
		if (desc->engine_type == SATA_ENGINE) {
			e->sata_channel = desc->channel_type;
			continue;
		}

		e->devfn = PCI_DEVFN(desc->device_number, desc->function_number);
		e->port_flags = (desc->port_present ? DXIO_PORT_PRESENT : 0) |
			engine_speed(desc->link_speed_capability) << DXIO_PORT_SPEED_SHIFT |
			(desc->end_logical_lane < desc->start_logical_lane ?
			 DXIO_PORT_LANE_REVERSAL : 0) |
			(desc->link_hotplug == HOTPLUG_SERVER_SSD ? HOTPLUG_SERVER :
			 desc->link_hotplug & 7) << 8 |
			DXIO_PORT_FLAG_19 |
			/* The vendor code sets this on every engine */
			DXIO_PORT_TURN_OFF_UNUSED;
	}
	config->engine_count = engines;
	d->config_size = offsetof(struct dxio_engine_config, engines) +
		engines * sizeof(struct dxio_engine) + sizeof(config->reserved_tail);
}

static bool die_has_sata(const struct dxio_die *d)
{
	for (size_t i = 0; i < d->count; i++)
		if (d->dxio[i].engine_type == SATA_ENGINE)
			return true;
	return false;
}

/* ---- Port mapping ---- */

static const dxio_descriptor *find_descriptor(const struct dxio_die *d, uint8_t start,
					      uint8_t end)
{
	for (size_t i = 0; i < d->count; i++) {
		const dxio_descriptor *desc = &d->dxio[i];

		if (descriptor_active(desc) &&
		    MIN(desc->start_logical_lane, desc->end_logical_lane) <= start &&
		    MAX(desc->start_logical_lane, desc->end_logical_lane) >= end)
			return desc;
	}
	return NULL;
}

/* The DXIO firmware numbers the ports per PCIe core */
static unsigned int engine_port(const struct dxio_engine *e)
{
	return (wire_to_die_lane(e->start_lane) >= 16 ? PCIE_PORTS_PER_CORE : 0) +
		(e->port & 7);
}

static int devfn_index(uint8_t devfn)
{
	for (int i = 0; i < PCIE_PORT_COUNT; i++)
		if (port_default_devfn[i] == devfn)
			return i;
	return -1;
}

static enum cb_err dxio_ports_mapped(struct dxio_die *d)
{
	struct zeppelin_pcie_topology *topology = zeppelin_pcie_topology(d->die);
	bool used[PCIE_PORT_COUNT] = { false };
	unsigned int next = 0;

	memset(d->devfn_map, 0, sizeof(d->devfn_map));
	for (size_t i = 0; i < d->config->engine_count; i++) {
		const struct dxio_engine *e = &d->config->engines[i];
		const uint8_t start = wire_to_die_lane(e->start_lane);
		const uint8_t end = wire_to_die_lane(e->end_lane);
		const dxio_descriptor *desc;
		struct zeppelin_pcie_port *port;
		unsigned int p;
		uint8_t devfn;
		int idx;

		if (e->type != PCIE_ENGINE)
			continue;
		desc = find_descriptor(d, start, end);
		p = engine_port(e);
		devfn = PCI_DEVFN(desc ? desc->device_number : 0,
				  desc ? desc->function_number : 0);
		idx = devfn_index(devfn);
		if (!desc || idx < 0 || used[idx]) {
			printk(BIOS_ERR, "DXIO: can't map lanes %u-%u (port %u)\n", start, end, p);
			return CB_ERR;
		}

		used[idx] = true;
		d->devfn_map[p] = devfn;
		port = &topology->ports[p];
		*port = (struct zeppelin_pcie_port) {
			.start_lane = start,
			.end_lane = end,
			.device = desc->device_number,
			.function = desc->function_number,
			.link_speed = desc->link_speed_capability,
			.hotplug = desc->link_hotplug,
			.allocated = true,
		};
	}

	/* The other ports get the remaining default devfns in order */
	for (unsigned int p = 0; p < PCIE_PORT_COUNT; p++) {
		if (d->devfn_map[p])
			continue;
		while (used[next])
			next++;
		used[next] = true;
		d->devfn_map[p] = port_default_devfn[next];
	}

	for (unsigned int p = 0; p < PCIE_PORT_COUNT; p++) {
		zeppelin_smn_clrsetbits32(d->die, NB_PROG_DEVICE_REMAP(p), 0xff, d->devfn_map[p]);
		printk(BIOS_DEBUG, "DXIO %u: port %u -> %02x.%x\n", d->die, p,
		       PCI_SLOT(d->devfn_map[p]), PCI_FUNC(d->devfn_map[p]));
	}

	return CB_SUCCESS;
}

static pci_devfn_t port_pci_dev(unsigned int die, const struct zeppelin_pcie_port *port)
{
	return PCI_DEV(zeppelin_die_bus(die), port->device, port->function);
}

/* State 5: the ports are configured but not trained yet (vendor ffdeb04b/ffdeb296) */
static void dxio_ports_configured(struct dxio_die *d)
{
	const struct zeppelin_pcie_topology *topology = zeppelin_pcie_topology(d->die);

	for (unsigned int p = 0; p < PCIE_PORT_COUNT; p++) {
		const struct zeppelin_pcie_port *port = &topology->ports[p];
		uint16_t eq;
		uint32_t value;

		if (!port->allocated)
			continue;

		/* LC_CNTL6 bit 0 = vendor PCD 0xe0 (0), bit 2 = PCD 0xe1 (1) */
		zeppelin_smn_clrsetbits32(d->die, pcie_port_reg(p, PCIE_LC_CNTL6), BIT(0) | BIT(2),
					  BIT(2));
		/* LC_CNTL7 [20:13] = vendor PCD 0xe5 (0) */
		zeppelin_smn_clrsetbits32(d->die, pcie_port_reg(p, PCIE_LC_CNTL7), 0x1fe000, 0);

		/*
		 * Lane equalization control of the secondary PCIe capability for ports
		 * that may train at gen 3: TX preset 7 and RX hint 2 in both directions.
		 */
		if (port->link_speed != GEN_MAX && port->link_speed != GEN3)
			continue;
		eq = pci_s_read_config16(port_pci_dev(d->die, port), 0x27c);
		value = ((eq & 0x8080) | 0x2727) * 0x10001;
		for (unsigned int i = 0; i < 8; i++)
			pci_s_write_config32(port_pci_dev(d->die, port), 0x27c + 4 * i, value);
	}
}

/*
 * PERST request of the DXIO firmware. The vendor code answers both assert and
 * deassert by giving GPIO26 (PCIE_RST_L) back to its native function on the
 * FCH of that die, through the SMN alias of the FCH ACPI MMIO (0x02d01000).
 */
static void dxio_perst(unsigned int die)
{
	zeppelin_smn_write32(die, 0x02d02568, 0x00d20000);	/* GPIO26 control */
	zeppelin_smn_clrsetbits32(die, 0x02d01d18, 0xff0000, 0);	/* IOMUX pin 26 */
}

/* Link status of a port after the training (vendor ffde7de3) */
static bool port_trained(unsigned int die, unsigned int p)
{
	const unsigned int core = pcie_port_core(p);

	if (p % PCIE_PORTS_PER_CORE == 0)
		return !(zeppelin_smn_read32(die, pcie_port_reg(p, PCIE_LC_PORT_2DC)) & BIT(28));
	return !(zeppelin_smn_read32(die, pcie_core_reg(core, PCIE_LINK_STATUS)) &
		 BIT(p % PCIE_PORTS_PER_CORE));
}

static enum cb_err dxio_training_done(struct dxio_die *d)
{
	struct zeppelin_pcie_topology *topology = zeppelin_pcie_topology(d->die);

	for (size_t i = 0; i < d->config->engine_count; i++) {
		const struct dxio_engine *e = &d->config->engines[i];
		struct zeppelin_pcie_port *port;
		unsigned int p;

		if (e->type != PCIE_ENGINE)
			continue;
		p = engine_port(e);
		port = &topology->ports[p];
		port->trained = (e->port_flags & DXIO_PORT_PRESENT) && port_trained(d->die, p);
		printk(BIOS_INFO, "PCIe %u:%02x.%x lanes %u-%u: %s\n", d->die, port->device,
		       port->function, port->start_lane, port->end_lane,
		       port->trained ? "trained" : "not trained");
	}

	return CB_SUCCESS;
}

static enum cb_err dxio_event_loop(struct dxio_die *d)
{
	struct stopwatch sw;

	stopwatch_init_msecs_expire(&sw, DXIO_TIMEOUT_MS);
	if (dxio_simple_rpc(d->die, DXIO_OP_SM_START))
		return CB_ERR;

	while (1) {
		uint32_t args[SMU_NUM_ARGS] = { 0 };
		uint8_t result, type, state;

		if (stopwatch_expired(&sw)) {
			printk(BIOS_ERR, "DXIO timeout\n");
			return CB_ERR;
		}

		if (dxio_rpc(d->die, DXIO_OP_GET_SM_STATE, args))
			return CB_ERR;
		result = args[0] & 0xff;
		type = args[1] & 0xff;
		state = args[2] & 0xff;

		if (result == DXIO_RESULT_ERROR) {
			if (type == DXIO_EVENT_SM) {
				printk(BIOS_ERR, "DXIO state machine error, state 0x%x\n", state);
				return CB_ERR;
			}
			printk(BIOS_WARNING, "DXIO error event 0x%x\n", type);
		} else if (result != DXIO_RESULT_OK) {
			printk(BIOS_ERR, "DXIO request didn't return: 0x%x\n", result);
			return CB_ERR;
		} else if (type == DXIO_EVENT_NONE) {
			return CB_SUCCESS;
		} else if (type == DXIO_EVENT_SM) {
			printk(BIOS_DEBUG, "DXIO %u state 0x%02x\n", d->die, state);
			if (state == DXIO_SM_DONE)
				return CB_SUCCESS;
			if (state == DXIO_SM_CONFIGURED)
				dxio_ports_configured(d);
			if (state == DXIO_SM_MAPPED &&
			    (dxio_get_engine_config(d) || dxio_ports_mapped(d)))
				return CB_ERR;
		} else if (type == DXIO_EVENT_PERST) {
			printk(BIOS_DEBUG, "DXIO %u: PERST %s\n", d->die,
			       state ? "deassert" : "assert");
			dxio_perst(d->die);
		} else if (type != DXIO_EVENT_RESUME) {
			printk(BIOS_WARNING, "DXIO: unknown event 0x%x\n", type);
		}

		if (dxio_simple_rpc(d->die, DXIO_OP_SM_RESUME))
			return CB_ERR;
	}
}

static enum cb_err dxio_train(struct dxio_die *d)
{
	uint32_t args[SMU_NUM_ARGS] = { 0 };

	if ((uintptr_t)d->config > UINT32_MAX)
		return CB_ERR;

	if (dxio_rpc(d->die, DXIO_OP_GET_VERSION, args) ||
	    (args[0] & 0xff) != DXIO_RESULT_OK) {
		printk(BIOS_ERR, "DXIO %u: unexpected version response 0x%x\n", d->die, args[0]);
		return CB_ERR;
	}

	if (dxio_simple_rpc(d->die, DXIO_OP_INIT) ||
	    dxio_set_var(d->die, DXIO_VAR_RET_AFTER_MAP, 1) ||
	    dxio_set_var(d->die, DXIO_VAR_RET_AFTER_CONF, 1) ||
	    dxio_set_var(d->die, DXIO_VAR_ANCILLARY_V1, 1) ||
	    dxio_simple_rpc(d->die, DXIO_OP_LOAD_CAPS) ||
	    /* No ancillary data: the descriptors have no tuning records */
	    dxio_load_empty(d->die, DXIO_DATA_MACPCS) ||
	    dxio_load_empty(d->die, DXIO_DATA_GPIO) ||
	    dxio_load_data(d->die, DXIO_DATA_ENGINE_CONFIG, d->config, d->config_size))
		return CB_ERR;

	if (dxio_event_loop(d) || dxio_get_engine_config(d))
		return CB_ERR;

	return dxio_training_done(d);
}

/* ---- Before and after the training ---- */

static void strap_write(unsigned int die, unsigned int core, uint32_t index, uint32_t value)
{
	zeppelin_smn_write32(die, PCIE_STRAP_INDEX(core), index);
	zeppelin_smn_write32(die, PCIE_STRAP_DATA(core), value);
}

static uint32_t strap_read(unsigned int die, unsigned int core, uint32_t index)
{
	zeppelin_smn_write32(die, PCIE_STRAP_INDEX(core), index);
	return zeppelin_smn_read32(die, PCIE_STRAP_DATA(core));
}

static void pcie_pre_training_init(unsigned int die)
{
	/* TX_CNTL bit 15 */
	for (unsigned int p = 0; p < PCIE_PORT_COUNT; p++)
		zeppelin_smn_clrsetbits32(die, pcie_port_reg(p, PCIE_TX_CNTL), BIT(15), 0);

	/* ACS (vendor CBS "ACS Enable", default enabled) */
	for (unsigned int core = 0; core < PCIE_CORE_COUNT; core++) {
		strap_write(die, core, 0xfffe0002, 1);
		for (uint32_t index = 0xfffe0069; index <= 0xfffe006e; index++)
			strap_write(die, core, index, 1);
		strap_write(die, core, 0xfffe0068, 1);
	}
	for (unsigned int n = 0; n < NBIF_COUNT; n++) {
		zeppelin_smn_clrsetbits32(die, NBIF_REG(n, 0x31000), 0x1c, 0x1c);
		for (unsigned int fn = 0; fn < 8; fn++)
			zeppelin_smn_clrsetbits32(die, NBIF_REG(n, 0x34008 + 0x200 * fn),
						  0x130000, 0x130000);
	}
}

/* Enable the SATA function of the NBIF */
static void nbif_sata_enable(unsigned int die)
{
	zeppelin_smn_clrsetbits32(die, 0x10234400, BIT(28), BIT(28));
	zeppelin_smn_clrsetbits32(die, 0x1023a008, BIT(2), BIT(2));
}

/* Gigabyte subsystem ID of the NBIO functions and the PCIe ports (vendor PCDs) */
#define NBIO_SSID		0x10001458
static void nbio_set_ssids(unsigned int die)
{
	static const uint32_t regs[] = {
		0x13b00050, 0x13f00078, 0x1014204c, 0x1024104c,
		0x1024404c, 0x1024504c, 0x1024604c, 0x1024704c,
		0x1014004c, 0x1024004c, 0x1034004c,
		0x10131004, 0x10231004, 0x10331004,
		0x1014104c, 0x1034104c, 0x1024304c,
	};

	for (size_t i = 0; i < ARRAY_SIZE(regs); i++)
		zeppelin_smn_write32(die, regs[i], NBIO_SSID);
	for (unsigned int core = 0; core < PCIE_CORE_COUNT; core++) {
		strap_write(die, core, 0xfffe0083, NBIO_SSID >> 16);
		strap_write(die, core, 0xfffe0082, NBIO_SSID & 0xffff);
	}
}

/* PcieInterfaceAfterDxioDoneSetting of the vendor code */
static void pcie_interface_after_dxio(unsigned int die)
{
	const struct zeppelin_pcie_topology *topology = zeppelin_pcie_topology(die);

	for (unsigned int core = 0; core < PCIE_CORE_COUNT; core++) {
		/* PLL off in L1, if the strap allows it */
		const uint32_t pll = strap_read(die, core, 0xfffe0084);

		for (unsigned int i = 0; i < PCIE_PORTS_PER_CORE; i++) {
			const unsigned int p = core * PCIE_PORTS_PER_CORE + i;
			const struct zeppelin_pcie_port *port = &topology->ports[p];

			if (port->allocated) {
				const pci_devfn_t dev = port_pci_dev(die, port);

				/* Completion timeout and "slot implemented" */
				pci_s_write_config32(dev, 0x80,
					(pci_s_read_config32(dev, 0x80) & ~0x9) | 0x6);
				pci_s_write_config32(dev, 0x58,
					pci_s_read_config32(dev, 0x58) | BIT(24));
			}
			if (pll == 0 || pll == 5)
				zeppelin_smn_clrsetbits32(die, pcie_port_reg(p, PCIE_LC_CNTL6),
							  BIT(21), BIT(21));
		}

		/* STRAP_F0 bit 18 = vendor PCD 0xd1/0xd2 (1), bits 14:13 = 3 (PCD 0xdf) */
		zeppelin_smn_clrsetbits32(die, pcie_core_reg(core, PCIE_STRAP_F0), BIT(18), BIT(18));
		zeppelin_smn_clrsetbits32(die, pcie_core_reg(core, PCIE_STRAP_F0), 0x6000, 0x6000);

		for (unsigned int i = 0; i < PCIE_PORTS_PER_CORE; i++) {
			const unsigned int p = core * PCIE_PORTS_PER_CORE + i;

			zeppelin_smn_clrsetbits32(die, pcie_port_reg(p, PCIE_PORT_304), 0x6, 0x6);
			zeppelin_smn_clrsetbits32(die, pcie_port_cfg(p, 0x80), 0xf, 0x6);
			zeppelin_smn_clrsetbits32(die, pcie_port_reg(p, PCIE_LC_CNTL), BIT(23),
						  BIT(23));
			zeppelin_smn_clrsetbits32(die, pcie_port_reg(p, PCIE_LC_LINK_WIDTH_CNTL),
						  BIT(19), BIT(19));
			zeppelin_smn_clrsetbits32(die, pcie_port_reg(p, PCIE_LC_CNTL2), 0xc000,
						  0x4000);
			zeppelin_smn_clrsetbits32(die, pcie_port_reg(p, PCIE_PORT_008), BIT(15),
						  BIT(15));
		}

		zeppelin_smn_clrsetbits32(die, pcie_core_reg(core, PCIE_CNTL), BIT(17), 0);
		/* P_CNTL bit 6 = !vendor PCD 0xde (0), bits 15:14 = 1 */
		zeppelin_smn_clrsetbits32(die, pcie_core_reg(core, PCIE_P_CNTL), BIT(6), BIT(6));
		zeppelin_smn_clrsetbits32(die, pcie_core_reg(core, PCIE_P_CNTL), 0xc000, 0x4000);
	}
}

static void pcie_post_training_init(unsigned int die)
{
	const struct zeppelin_pcie_topology *topology = zeppelin_pcie_topology(die);
	uint32_t args[SMU_NUM_ARGS] = { 0, 0, 0xf, 0 };	/* 0xf: one socket */

	/* Hide all bridges, then show the ones with a link */
	for (unsigned int i = 0; i < PCIE_PORT_COUNT; i++)
		zeppelin_smn_clrsetbits32(die, IOHC_BRIDGE_CNTL(i), IOHC_BRIDGE_CNTL_MASK,
					  IOHC_BRIDGE_CNTL_HIDE);
	for (unsigned int p = 0; p < PCIE_PORT_COUNT; p++) {
		const struct zeppelin_pcie_port *port = &topology->ports[p];
		const int idx = devfn_index(PCI_DEVFN(port->device, port->function));

		if (!port->allocated || idx < 0)
			continue;
		if (port->trained || (port->hotplug != HOTPLUG_DISABLED &&
				      port->hotplug != HOTPLUG_INBOARD))
			zeppelin_smn_clrsetbits32(die, IOHC_BRIDGE_CNTL(idx),
						  IOHC_BRIDGE_CNTL_MASK, IOHC_BRIDGE_CNTL_ENABLE);
	}

	/* Not on AM4: meaning unknown, the argument depends on the socket count */
	dxio_rpc(die, DXIO_OP_POST_TRAINING, args);

	pcie_interface_after_dxio(die);
	nbio_set_ssids(die);
}

/* Bus routing between the root bridges: each IOHC learns the bus range of every die */
static void pcie_rb_bus_routing(unsigned int dies)
{
	for (unsigned int die = 0; die < dies; die++) {
		for (unsigned int rb = 0; rb < dies; rb++) {
			const uint8_t base = zeppelin_die_bus(rb);
			const uint8_t limit = rb + 1 < dies ? zeppelin_die_bus(rb + 1) - 1 :
				CONFIG_ECAM_MMCONF_BUS_NUMBER - 1;

			zeppelin_smn_write32(die, IOHC_RB_BUS_ROUTE(rb), limit << 8 | base |
					     (rb != die ? BIT(31) : 0));
		}
		/* Vendor PCD 0x1d (ARI support), default enabled */
		zeppelin_smn_clrsetbits32(die, IOHC_FEATURE_CNTL, IOHC_FEATURE_CNTL_BIT22,
					  IOHC_FEATURE_CNTL_BIT22);
	}
}

enum cb_err zeppelin_dxio_init(void)
{
	/* The DXIO firmware accesses these with DMA */
	static struct dxio_engine_config configs[ZEPPELIN_MAX_DIES];
	const unsigned int dies = zeppelin_die_count();

	if (zeppelin_smu_test_message(0))
		return CB_ERR;
	for (unsigned int die = 0; die < dies; die++)
		pcie_pre_training_init(die);

	for (unsigned int die = 0; die < dies; die++) {
		struct dxio_die d = {
			.die = die,
			.config = &configs[die],
		};

		mainboard_get_dxio_descriptors(die, &d.dxio, &d.count);
		build_engine_config(&d);
		if (die_has_sata(&d))
			nbif_sata_enable(die);

		/* Vendor PCD 0xcd/0xce/0xcf select die 0 for the early init cleanup */
		if (die == 0 && dxio_simple_rpc(die, DXIO_OP_CLEANUP_EARLY))
			return CB_ERR;

		if (!d.config->engine_count)
			continue;
		printk(BIOS_DEBUG, "DXIO %u: %u engines\n", die, d.config->engine_count);
		if (dxio_train(&d))
			return CB_ERR;
	}

	for (unsigned int die = 0; die < dies; die++)
		pcie_post_training_init(die);
	pcie_rb_bus_routing(dies);

	return CB_SUCCESS;
}
