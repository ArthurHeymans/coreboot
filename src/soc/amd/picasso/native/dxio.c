/* SPDX-License-Identifier: GPL-2.0-only */

#include <acpi/acpi.h>
#include <amdblocks/smn.h>
#include <amdblocks/smu.h>
#include <console/console.h>
#include <device/device.h>
#include <device/pci_def.h>
#include <device/pci_ids.h>
#include <device/pci_ops.h>
#include <soc/platform_descriptors.h>
#include <soc/smu.h>
#include <soc/soc_util.h>
#include <static.h>
#include <string.h>
#include <timer.h>
#include <types.h>

#include "../chip.h"
#include "fch_regs.h"
#include "native.h"
#include "nbio.h"
#include "smu.h"

/* Maximum number of DXIO descriptors, which is what the DXIO firmware can handle */
#define DXIO_DESCRIPTOR_COUNT	8

/* The GPP bridges 0..6 are remapped to one of the devfns 1.1 to 1.7 */
#define DXIO_REMAP_PORT_COUNT	PCIE_GPP_PORT_COUNT
#define DXIO_REMAP_DEVFN_FIRST	0x09
#define DXIO_REMAP_DEVFN_LAST	0x0f

/* Link speed that gets requested from the DXIO firmware */
static uint8_t picasso_dxio_link_speed(dxio_link_speed_cap requested,
				       bool safe_mode, bool balanced_policy,
				       bool s3_resume)
{
	if (safe_mode)
		return GEN1;
	if (balanced_policy)
		return s3_resume ? GEN2 : GEN1;
	return requested;
}

static bool picasso_dxio_ranges_overlap(uint8_t first_a, uint8_t last_a,
					uint8_t first_b, uint8_t last_b)
{
	const uint8_t low_a = first_a < last_a ? first_a : last_a;
	const uint8_t high_a = first_a > last_a ? first_a : last_a;
	const uint8_t low_b = first_b < last_b ? first_b : last_b;
	const uint8_t high_b = first_b > last_b ? first_b : last_b;

	return low_a <= high_b && low_b <= high_a;
}

static bool picasso_dxio_devfn_used(const uint8_t map[DXIO_REMAP_PORT_COUNT],
				    uint8_t devfn)
{
	for (size_t i = 0; i < DXIO_REMAP_PORT_COUNT; i++)
		if (map[i] == devfn)
			return true;
	return false;
}

/*
 * Assign the devfn requested by the board to a port. map[] has an entry for every port,
 * but only the first DXIO_REMAP_PORT_COUNT entries are remappable and checked for conflicts.
 */
static bool picasso_dxio_claim_devfn(uint8_t map[], uint8_t port, uint8_t devfn)
{
	if (devfn < DXIO_REMAP_DEVFN_FIRST || devfn > DXIO_REMAP_DEVFN_LAST ||
	    picasso_dxio_devfn_used(map, devfn))
		return false;
	map[port] = devfn;
	return true;
}

/* Assign the unused devfns to the ports that didn't get one assigned */
static bool picasso_dxio_complete_devfn_map(uint8_t map[])
{
	for (size_t port = 0; port < DXIO_REMAP_PORT_COUNT; port++) {
		if (map[port])
			continue;
		for (uint8_t devfn = DXIO_REMAP_DEVFN_FIRST; devfn <= DXIO_REMAP_DEVFN_LAST;
		     devfn++) {
			if (!picasso_dxio_devfn_used(map, devfn)) {
				map[port] = devfn;
				break;
			}
		}
		if (!map[port])
			return false;
	}
	return true;
}

/*
 * The DXIO firmware on the SMU configures the PHYs and trains the PCIe and SATA links. It is
 * controlled with the SMC_MSG_DXIO_SERVICE mailbox message. The first argument is the
 * operation, the other arguments are operation specific. On return the first argument has the
 * result of the operation.
 */
enum dxio_op {
	DXIO_OP_GET_VERSION = 0x00,
	DXIO_OP_GET_SM_STATE = 0x09,
	DXIO_OP_INIT = 0x13,
	DXIO_OP_GET_ENGINE_CONFIG = 0x14,
	DXIO_OP_SET_VAR = 0x22,
	DXIO_OP_LOAD_DATA = 0x23,
	DXIO_OP_LOAD_CAPS = 0x24,
	DXIO_OP_SM_START = 0x307,
	DXIO_OP_SM_RESUME = 0x308,
};

#define DXIO_RESULT_ERROR		2
#define DXIO_RESULT_OK			3

/* DXIO_OP_SET_VAR variables */
#define DXIO_VAR_SKIP_PSP		0x0d
#define DXIO_VAR_RET_AFTER_MAP		0x0e
#define DXIO_VAR_RET_AFTER_CONF		0x0f
#define DXIO_VAR_ANCILLARY_V1		0x10
#define DXIO_VAR_MAP_EXACT_MATCH	0x12

/* DXIO_OP_LOAD_DATA types */
#define DXIO_DATA_MACPCS		2
#define DXIO_DATA_ENGINE_CONFIG		3
#define DXIO_DATA_GPIO			5
#define DXIO_DATA_ANCILLARY		6

/* DXIO_OP_GET_SM_STATE returns the type of the pending event and the state */
#define DXIO_EVENT_NONE			0
#define DXIO_EVENT_SM			2

enum dxio_sm_state {
	DXIO_SM_CONFIGURED = 0x05,
	DXIO_SM_DONE = 0x0d,
	DXIO_SM_MAPPED = 0x0f,
};

/* A lot of links might need to get trained, so allow for some time */
#define DXIO_TIMEOUT_MS			60000

/* Engine configuration that gets passed to the DXIO firmware */
struct dxio_engine {
	uint8_t type;
	uint8_t hotplug;
	uint8_t start_lane;
	uint8_t end_lane;
	uint8_t gpio_group;
	uint8_t reserved_05[3];
	uint8_t sata_channel;
	uint8_t reserved_09[3];
	uint32_t devfn;
	uint8_t port_flags;
	uint8_t hotplug_type;
	uint8_t port_flags2;
	uint8_t reserved_13[6];
	uint8_t port;		/* hardware port number assigned by the DXIO firmware */
	uint8_t reserved_1a[14];
} __packed;

#define  DXIO_PORT_PRESENT		BIT(0)
#define  DXIO_PORT_LANE_REVERSAL	BIT(3)
#define  DXIO_PORT_SPEED_SHIFT		4
#define  DXIO_PORT2_BIT3		BIT(3)
#define  DXIO_PORT2_TURN_OFF_UNUSED	BIT(4)

_Static_assert(sizeof(struct dxio_engine) == 0x28, "Unexpected DXIO engine size");

struct dxio_engine_config {
	uint8_t reserved_00[0x0c];
	uint16_t engine_count;
	uint8_t reserved_0e[2];
	struct dxio_engine engines[DXIO_DESCRIPTOR_COUNT];
	uint8_t reserved_tail[8];
} __packed __aligned(16);

/* Ancillary data with the PCIe speed power policy */
static const uint32_t dxio_ancillary_pspp[] __aligned(16) = {
	0x00030000, 0x00020104, 0x600ededf, 0x00000000,
};

/* Default devfn of the PCIe ports */
static const uint8_t pcie_port_default_devfn[PCIE_PORT_COUNT] = {
	PCI_DEVFN(1, 1), PCI_DEVFN(1, 2), PCI_DEVFN(1, 3), PCI_DEVFN(1, 4),
	PCI_DEVFN(1, 5), PCI_DEVFN(1, 6), PCI_DEVFN(1, 7), PCI_DEVFN(2, 1),
};

static enum cb_err dxio_rpc(uint32_t op, uint32_t args[SMU_NUM_ARGS])
{
	struct smu_payload payload;

	memcpy(payload.msg, args, sizeof(payload.msg));
	payload.msg[0] = op;
	if (send_smu_message(SMC_MSG_DXIO_SERVICE, &payload)) {
		printk(BIOS_ERR, "DXIO operation 0x%x failed\n", op);
		return CB_ERR;
	}
	memcpy(args, payload.msg, sizeof(payload.msg));
	printk(BIOS_SPEW, "DXIO op 0x%x: %08x %08x %08x %08x %08x %08x\n", op,
	       args[0], args[1], args[2], args[3], args[4], args[5]);
	return CB_SUCCESS;
}

static enum cb_err dxio_simple_rpc(uint32_t op)
{
	uint32_t args[SMU_NUM_ARGS] = { 0 };

	return dxio_rpc(op, args);
}

static enum cb_err dxio_set_var(uint32_t var, uint32_t value)
{
	uint32_t args[SMU_NUM_ARGS] = { 0, var, value };

	return dxio_rpc(DXIO_OP_SET_VAR, args);
}

static enum cb_err dxio_load_data(uint32_t type, const void *data, size_t size)
{
	uint32_t args[SMU_NUM_ARGS] = { 0, 0, (uintptr_t)data, size / sizeof(uint32_t), 0, type };

	return dxio_rpc(DXIO_OP_LOAD_DATA, args);
}

/* Tell the DXIO firmware that there is no data of the given type */
static enum cb_err dxio_load_empty(uint32_t type)
{
	uint32_t args[SMU_NUM_ARGS] = { 0, 0, 0, 0, 1, type };

	return dxio_rpc(DXIO_OP_LOAD_DATA, args);
}

static enum cb_err dxio_get_engine_config(struct dxio_engine_config *config, size_t size)
{
	uint32_t args[SMU_NUM_ARGS] = { 0, 0, (uintptr_t)config, size / sizeof(uint32_t) };

	memset(config, 0, size);
	if (dxio_rpc(DXIO_OP_GET_ENGINE_CONFIG, args))
		return CB_ERR;

	if (size < offsetof(struct dxio_engine_config, engines) + sizeof(config->reserved_tail) ||
	    config->engine_count > (size - offsetof(struct dxio_engine_config, engines) -
				    sizeof(config->reserved_tail)) / sizeof(struct dxio_engine)) {
		printk(BIOS_ERR, "Invalid DXIO engine count %u\n", config->engine_count);
		return CB_ERR;
	}

	return CB_SUCCESS;
}

static bool descriptor_active(const dxio_descriptor *d)
{
	return d->engine_type != UNUSED_ENGINE;
}

static enum cb_err validate_descriptors(const dxio_descriptor *dxio,
					size_t count)
{
	if ((!dxio && count) || count > DXIO_DESCRIPTOR_COUNT)
		return CB_ERR;

	for (size_t i = 0; i < count; i++) {
		const dxio_descriptor *d = &dxio[i];

		if (!descriptor_active(d))
			continue;
		/* Only the engine types, lanes and features that are used by boards */
		if (d->engine_type != PCIE_ENGINE && d->engine_type != SATA_ENGINE)
			return CB_ERR;
		if (d->link_hotplug > HOTPLUG_SERVER_SSD || d->channel_type > SATA_CHANNEL_LONG ||
		    d->start_logical_lane >= PCIE_LANE_COUNT ||
		    d->end_logical_lane >= PCIE_LANE_COUNT || d->link_safe_mode || d->sb_link)
			return CB_ERR;
		for (size_t j = 0; j < i; j++) {
			if (descriptor_active(&dxio[j]) &&
			    picasso_dxio_ranges_overlap(d->start_logical_lane,
					d->end_logical_lane, dxio[j].start_logical_lane,
					dxio[j].end_logical_lane))
				return CB_ERR;
		}
	}
	return CB_SUCCESS;
}

static void build_engine_config(struct dxio_engine_config *config,
				const dxio_descriptor *dxio, size_t count,
				bool balanced_pspp)
{
	size_t engines = 0;

	memset(config, 0, sizeof(*config));
	for (size_t i = 0; i < count; i++) {
		const dxio_descriptor *d = &dxio[i];
		struct dxio_engine *e;

		if (!descriptor_active(d))
			continue;

		e = &config->engines[engines++];
		e->type = d->engine_type;
		e->hotplug = d->link_hotplug != HOTPLUG_DISABLED;
		e->start_lane = MIN(d->start_logical_lane, d->end_logical_lane);
		e->end_lane = MAX(d->start_logical_lane, d->end_logical_lane);
		e->gpio_group = 1;
		if (d->engine_type == SATA_ENGINE)
			e->sata_channel = d->channel_type;
		if (d->engine_type != PCIE_ENGINE)
			continue;

		e->devfn = PCI_DEVFN(d->device_number, d->function_number);
		e->port_flags = (d->port_present ? DXIO_PORT_PRESENT : 0) |
			picasso_dxio_link_speed(d->link_speed_capability, false,
						balanced_pspp, acpi_is_wakeup_s3())
			<< DXIO_PORT_SPEED_SHIFT;
		if (d->end_logical_lane < d->start_logical_lane)
			e->port_flags |= DXIO_PORT_LANE_REVERSAL;
		e->hotplug_type = d->link_hotplug == HOTPLUG_SERVER_SSD ?
			HOTPLUG_SERVER : d->link_hotplug;
		e->port_flags2 = DXIO_PORT2_BIT3 |
			(d->turn_off_unused_lanes ? DXIO_PORT2_TURN_OFF_UNUSED : 0);
	}
	config->engine_count = engines;
}

/* Returns the first descriptor that contains the lanes of the engine */
static const dxio_descriptor *find_descriptor(
	const struct dxio_engine *e, const dxio_descriptor *dxio,
	size_t count)
{
	for (size_t i = 0; i < count; i++) {
		if (e->start_lane <= dxio[i].start_logical_lane &&
		    e->end_lane >= dxio[i].end_logical_lane)
			return &dxio[i];
	}
	return NULL;
}

static struct picasso_pcie_port *find_port(struct picasso_pcie_topology *topology,
					   const struct dxio_engine *e)
{
	return e->port < PCIE_PORT_COUNT ? &topology->ports[e->port] : NULL;
}

/*
 * After mapping the engines to the PCIe ports, record the port configuration and remap the
 * GPP bridges to the devfns that the board requested.
 */
static enum cb_err dxio_ports_mapped(const struct dxio_engine_config *config,
				     const dxio_descriptor *dxio,
				     size_t count)
{
	struct picasso_pcie_topology topology = *picasso_pcie_topology();
	uint8_t devfn_map[PCIE_PORT_COUNT] = { 0 };

	for (size_t i = 0; i < config->engine_count; i++) {
		const struct dxio_engine *e = &config->engines[i];
		const dxio_descriptor *d;
		struct picasso_pcie_port *port;
		uint8_t devfn;

		if (e->type != PCIE_ENGINE)
			continue;
		d = find_descriptor(e, dxio, count);
		port = find_port(&topology, e);
		if (!d || !port)
			return CB_ERR;

		port->allocated = true;
		port->device = d->device_number;
		port->function = d->function_number;
		if (!port->device && !port->function) {
			port->device = PCI_SLOT(port->default_devfn);
			port->function = PCI_FUNC(port->default_devfn);
		}
		port->aspm = d->link_aspm;
		port->hotplug = d->link_hotplug;
		port->clk_req = d->clk_req;
		port->l1_1 = d->link_aspm_L1_1;
		port->l1_2 = d->link_aspm_L1_2;
		port->link_speed = d->link_speed_capability;

		devfn = PCI_DEVFN(port->device, port->function);
		if (devfn >= DXIO_REMAP_DEVFN_FIRST && devfn <= DXIO_REMAP_DEVFN_LAST)
			port->devfn_index = devfn - DXIO_REMAP_DEVFN_FIRST;
		if (!picasso_dxio_claim_devfn(devfn_map, port->port, devfn))
			printk(BIOS_WARNING, "Can't remap PCIe port %u to devfn %02x\n",
			       port->port, devfn);
	}

	if (!picasso_dxio_complete_devfn_map(devfn_map))
		return CB_ERR;

	*picasso_pcie_topology() = topology;

	for (unsigned int i = 0; i < DXIO_REMAP_PORT_COUNT; i++) {
		smn_clrsetbits32(NB_PROG_DEVICE_REMAP(i), 0xff, devfn_map[i]);
		smn_clrsetbits32(pcie_port_reg(i, PCIE_TX_REQUESTER_ID), 0x0000fff8,
				 devfn_map[i] & 0xf8);
		smn_clrsetbits32(pcie_port_reg(i, 0x008), 0x7, devfn_map[i] & 0x7);
	}

	return CB_SUCCESS;
}

/* Equalization presets for the ports that may train at gen 3 */
#define DXIO_GEN3_EQ_PRESET	0x1bb7

static void dxio_ports_configured(void)
{
	struct picasso_pcie_topology *topology = picasso_pcie_topology();

	for (int i = PCIE_PORT_COUNT - 1; i >= 0; i--) {
		const struct picasso_pcie_port *port = &topology->ports[i];
		const pci_devfn_t dev = PCI_DEV(0, port->device, port->function);
		uint16_t preset = 0;
		uint16_t eq;

		if (!port->allocated)
			continue;

		if (port->link_speed == GEN_MAX ||
		    port->link_speed == GEN3)
			preset = DXIO_GEN3_EQ_PRESET;

		/* Lane equalization control of the secondary PCIe extended capability */
		eq = pci_s_read_config16(dev, 0x27c) & 0x8080;
		eq |= preset & 0x7f;
		eq |= (((preset >> 8 & 7) * 2) | (preset >> 7 & 1) | (preset >> 7 & 0x70)) << 8;
		for (uint16_t reg = 0x27c; reg <= 0x298; reg += 4)
			pci_s_write_config32(dev, reg, eq * 0x10001u);

		smn_write32(PCIE_PHY_INDEX, 0xfffe0088);
		smn_write32(PCIE_PHY_DATA, preset >> 4 & 7);
		smn_write32(PCIE_PHY_INDEX, 0xfffe0089);
		smn_write32(PCIE_PHY_DATA, preset >> 11 & 7);
		smn_write32(PCIE_PHY_INDEX, 0xfffe008a);
		smn_write32(PCIE_PHY_DATA, preset & 0xf);
		smn_write32(PCIE_PHY_INDEX, 0xfffe008b);
		smn_write32(PCIE_PHY_DATA, ((preset >> 8 & 7) << 1) | (preset >> 7 & 1));
	}
}

static enum cb_err dxio_training_done(const struct dxio_engine_config *config,
				      const dxio_descriptor *dxio,
				      size_t count)
{
	struct picasso_pcie_topology *topology = picasso_pcie_topology();
	const uint32_t inhibit = smn_read32(pcie_core_reg(PCIE_TRAINING_INHIBIT));

	for (size_t i = 0; i < config->engine_count; i++) {
		const struct dxio_engine *e = &config->engines[i];
		struct picasso_pcie_port *port;

		if (e->type != PCIE_ENGINE)
			continue;
		port = find_port(topology, e);
		if (!find_descriptor(e, dxio, count) || !port)
			return CB_ERR;

		port->trained = (e->port_flags & DXIO_PORT_PRESENT) &&
			!(inhibit & BIT(port->port));
		printk(BIOS_DEBUG, "PCIe port %u: %s, LC state %08x\n", port->port,
		       port->trained ? "trained" : "not trained",
		       smn_read32(pcie_port_reg(port->port, PCIE_LC_STATE0)));
	}

	return picasso_pcie_topology_save();
}

static enum cb_err dxio_train(const dxio_descriptor *dxio, size_t count)
{
	const struct soc_amd_picasso_config *config = config_of_soc();
	/* The DXIO firmware accesses these with DMA */
	static struct dxio_engine_config engine_config;
	uint32_t args[SMU_NUM_ARGS];
	struct stopwatch sw;
	size_t config_size;

	build_engine_config(&engine_config, dxio, count,
			    config->pspp_policy == DXIO_PSPP_BALANCED);
	config_size = offsetof(struct dxio_engine_config, engines) +
		engine_config.engine_count * sizeof(struct dxio_engine) +
		sizeof(engine_config.reserved_tail);

	if ((uintptr_t)&engine_config > UINT32_MAX)
		return CB_ERR;

	memset(args, 0, sizeof(args));
	if (dxio_rpc(DXIO_OP_GET_VERSION, args) || (args[0] & 0xff) != DXIO_RESULT_OK)
		return CB_ERR;

	if (dxio_simple_rpc(DXIO_OP_INIT))
		return CB_ERR;

	/* Stop after the mapping and the configuration to handle the events below */
	if (dxio_set_var(DXIO_VAR_RET_AFTER_MAP, 1) ||
	    dxio_set_var(DXIO_VAR_RET_AFTER_CONF, 1) ||
	    dxio_set_var(DXIO_VAR_ANCILLARY_V1, 1))
		return CB_ERR;
	if (get_socket_type() == SOCKET_AM4 && dxio_set_var(DXIO_VAR_SKIP_PSP, 0))
		return CB_ERR;
	if (dxio_set_var(DXIO_VAR_MAP_EXACT_MATCH,
			 !!(picasso_native_silicon_id() & SILICON_IS_MYSTERY_MEAT)))
		return CB_ERR;

	if (dxio_simple_rpc(DXIO_OP_LOAD_CAPS))
		return CB_ERR;
	if (config->pspp_policy != DXIO_PSPP_DISABLED &&
	    dxio_load_data(DXIO_DATA_ANCILLARY, dxio_ancillary_pspp, sizeof(dxio_ancillary_pspp)))
		return CB_ERR;
	if (dxio_load_empty(DXIO_DATA_MACPCS) || dxio_load_empty(DXIO_DATA_GPIO))
		return CB_ERR;
	if (dxio_load_data(DXIO_DATA_ENGINE_CONFIG, &engine_config, config_size))
		return CB_ERR;

	stopwatch_init_msecs_expire(&sw, DXIO_TIMEOUT_MS);
	if (dxio_simple_rpc(DXIO_OP_SM_START))
		return CB_ERR;

	while (1) {
		uint32_t result, type, state;

		if (stopwatch_expired(&sw)) {
			printk(BIOS_ERR, "DXIO timeout\n");
			return CB_ERR;
		}

		memset(args, 0, sizeof(args));
		if (dxio_rpc(DXIO_OP_GET_SM_STATE, args))
			return CB_ERR;
		result = args[0] & 0xff;
		type = args[1] & 0xff;
		state = args[2] & 0xff;

		if (result != DXIO_RESULT_OK && result != DXIO_RESULT_ERROR)
			return CB_ERR;
		if (result == DXIO_RESULT_ERROR && type == DXIO_EVENT_SM) {
			printk(BIOS_ERR, "DXIO state machine error\n");
			return CB_ERR;
		}
		if (result == DXIO_RESULT_OK) {
			if (type == DXIO_EVENT_NONE ||
			    (type == DXIO_EVENT_SM && state == DXIO_SM_DONE))
				break;
			if (type == DXIO_EVENT_SM && state == DXIO_SM_CONFIGURED)
				dxio_ports_configured();
			if (type == DXIO_EVENT_SM && state == DXIO_SM_MAPPED) {
				if (dxio_get_engine_config(&engine_config, config_size) ||
				    dxio_ports_mapped(&engine_config, dxio, count))
					return CB_ERR;
			}
		}

		printk(BIOS_DEBUG, "DXIO state 0x%02x\n", state);
		if (dxio_simple_rpc(DXIO_OP_SM_RESUME))
			return CB_ERR;
	}

	if (dxio_get_engine_config(&engine_config, config_size))
		return CB_ERR;

	return dxio_training_done(&engine_config, dxio, count);
}

static void init_pcie_topology(void)
{
	struct picasso_pcie_topology *topology = picasso_pcie_topology();

	memset(topology, 0, sizeof(*topology));
	for (unsigned int i = 0; i < PCIE_PORT_COUNT; i++) {
		topology->ports[i].port = i;
		topology->ports[i].default_devfn = pcie_port_default_devfn[i];
		topology->ports[i].devfn_index = i;
	}
}

static void pcie_pre_training_init(bool hda_enabled)
{
	/* Disable functions 9 and 10 of the internal bus and configure the HDA function */
	smn_clrsetbits32(NBIF_FUNC_STRAP(9), NBIF_FUNC_STRAP_FUNC_EN, 0);
	smn_clrsetbits32(NBIF_INTR_LINE_ENABLE, BIT(9), 0);
	smn_clrsetbits32(NBIF_FUNC_STRAP(10), NBIF_FUNC_STRAP_FUNC_EN, 0);
	smn_clrsetbits32(NBIF_INTR_LINE_ENABLE, BIT(10), 0);
	smn_clrsetbits32(NBIF_FUNC_STRAP(NBIF_FUNC_HDA), NBIF_FUNC_STRAP_FUNC_EN,
			 hda_enabled ? NBIF_FUNC_STRAP_FUNC_EN : 0);
	smn_clrsetbits32(NBIF_INTR_LINE_ENABLE, BIT(NBIF_FUNC_HDA),
			 hda_enabled ? BIT(NBIF_FUNC_HDA) : 0);

	/* Enable bus mastering of the GFX and GFX audio functions and clear their status */
	smn_write32(NBIF_GFX_COMMAND, (smn_read32(NBIF_GFX_COMMAND) & 0xff) | PCI_COMMAND_MASTER);
	smn_write32(NBIF_GFX_AUDIO_COMMAND,
		    (smn_read32(NBIF_GFX_AUDIO_COMMAND) & 0xff) | PCI_COMMAND_MASTER);

	for (int port = PCIE_PORT_COUNT - 1; port >= 0; port--) {
		smn_clrsetbits32(pcie_port_reg(port, PCIE_TX_CNTL), BIT(15), 0);
		smn_clrsetbits32(pcie_port_reg(port, PCIE_LC_LINK_WIDTH_CNTL), 0, 0x600000);
	}

	smn_write32(PCIE_PHY_INDEX, 0xfffe0002);
	smn_write32(PCIE_PHY_DATA, 1);
	for (uint32_t index = 0xfffe0069; index < 0xfffe006f; index++) {
		smn_write32(PCIE_PHY_INDEX, index);
		smn_write32(PCIE_PHY_DATA, 1);
	}
	smn_write32(PCIE_PHY_INDEX, 0xfffe0068);
	smn_write32(PCIE_PHY_DATA, 1);

	/* NBIF error reporting and ACS */
	smn_clrsetbits32(0x10131200, 0x1c, 0x1c);
	for (unsigned int func = 8; func <= 10; func++)
		smn_clrsetbits32(NBIF_FUNC_STRAP(func) + 8, 0x130000, 0x130000);
	if (picasso_gfx_available()) {
		smn_clrsetbits32(0x1012349c, BIT(2), BIT(2));
		smn_clrsetbits32(0x101234c8, BIT(17), BIT(17));
	}

	for (uint32_t index = 0xfffe00c5; index != 0xfffe02fc; index += 0x51) {
		smn_write32(PCIE_PHY_INDEX, index);
		smn_write32(PCIE_PHY_DATA, 7);
	}
	smn_clrsetbits32(0x11a0b014, 0xf0, 0x40);

	smn_clrsetbits32(NBIF_FUNC_STRAP(8), NBIF_FUNC_STRAP_FUNC_EN, 0);
	smn_clrsetbits32(NBIF_INTR_LINE_ENABLE, BIT(8), 0);
}

/* PCI Express capability registers of the GPP bridges in the port configuration space */
#define PCIE_CFG_CAP_HEADER		0x058
#define  PCIE_CAP_SLOT_IMPLEMENTED	BIT(24)
#define PCIE_CFG_ROOT_CTRL		0x074
#define  PCIE_ROOT_CTRL_PME_INT_EN	BIT(3)

static void pcie_post_training_init(void)
{
	const struct picasso_pcie_topology *topology = picasso_pcie_topology();

	/* Hide the GPP bridges without a link or hotplug */
	for (unsigned int i = 0; i < PCIE_GPP_PORT_COUNT; i++)
		smn_clrsetbits32(IOHC_BRIDGE_CNTL(i), IOHC_BRIDGE_CNTL_ENABLE_MASK,
				 IOHC_BRIDGE_CNTL_ENABLE_MASK);
	for (int i = PCIE_PORT_COUNT - 1; i >= 0; i--) {
		const struct picasso_pcie_port *port = &topology->ports[i];

		if (port->trained || (port->hotplug & ~HOTPLUG_INBOARD))
			smn_clrsetbits32(IOHC_BRIDGE_CNTL(port->devfn_index),
					 IOHC_BRIDGE_CNTL_ENABLE_MASK, BIT(18));
	}

	smn_clrsetbits32(SATA_SMN(0x1800), BIT(3), BIT(3));
	smn_clrsetbits32(SATA_SMN(0x1800), BIT(3), 0);

	for (int port = PCIE_PORT_COUNT - 1; port >= 0; port--)
		smn_clrsetbits32(pcie_port_reg(port, PCIE_TX_CNTL), BIT(15), 0);

	for (int i = PCIE_PORT_COUNT - 1; i >= 0; i--) {
		if (topology->ports[i].allocated)
			smn_clrsetbits32(pcie_port_cfg(i, PCIE_CFG_CAP_HEADER), 0,
					 PCIE_CAP_SLOT_IMPLEMENTED);
		smn_clrsetbits32(pcie_port_reg(i, PCIE_LC_CNTL), BIT(23), 0);
		smn_clrsetbits32(pcie_port_reg(i, PCIE_LC_CNTL4), 0x10310, 0x100);
		smn_clrsetbits32(pcie_port_reg(i, 0x2f0), 0x07000000, 0);
		smn_clrsetbits32(pcie_port_reg(i, PCIE_LC_FORCE_COEFF), BIT(19), BIT(19));
		smn_clrsetbits32(pcie_port_reg(i, PCIE_LC_CNTL6), 0x15, 0x14);
		smn_clrsetbits32(pcie_port_reg(i, PCIE_LC_CNTL6), PCIE_LC_CNTL6_L1_ALLOW, 0);
		if (topology->ports[i].hotplug == HOTPLUG_ENHANCED)
			smn_clrsetbits32(pcie_port_cfg(i, PCIE_CFG_ROOT_CTRL), 0,
					 PCIE_ROOT_CTRL_PME_INT_EN);
	}

	smn_clrsetbits32(pcie_core_reg(PCIE_STRAP_F0), 0, BIT(18));

	for (unsigned int i = 0; i < PCIE_GPP_PORT_COUNT; i++) {
		smn_clrsetbits32(pcie_port_reg(i, 0x304), 0x6, 0x6);
		smn_clrsetbits32(pcie_port_cfg(i, 0x080), 0xf, 0x6);
		smn_clrsetbits32(pcie_port_reg(i, PCIE_LC_CNTL), BIT(23), BIT(23));
		smn_clrsetbits32(pcie_port_reg(i, PCIE_LC_LINK_WIDTH_CNTL), BIT(19), BIT(19));
		smn_clrsetbits32(pcie_port_reg(i, PCIE_LC_CNTL2), 0xc000, 0x4000);
		smn_clrsetbits32(pcie_port_reg(i, 0x008), BIT(15), BIT(15));
	}

	smn_clrsetbits32(pcie_core_reg(PCIE_CNTL), BIT(17), 0);
	smn_clrsetbits32(pcie_core_reg(PCIE_P_CNTL), 0, BIT(6));
	smn_clrsetbits32(pcie_core_reg(PCIE_P_CNTL), BIT(14), BIT(14));

	/* PCI IDs of the PCIe ports */
	smn_write32(PCIE_PHY_INDEX, 0xfffe0083);
	smn_write32(PCIE_PHY_DATA, 0x1453);
	smn_write32(PCIE_PHY_INDEX, 0xfffe0082);
	smn_write32(PCIE_PHY_DATA, PCI_VID_AMD);
}

enum cb_err picasso_dxio_init(void)
{
	const dxio_descriptor *dxio = NULL;
	const ddi_descriptor *ddi = NULL;
	const bool hda_enabled = is_dev_enabled(DEV_PTR(hda));
	size_t dxio_count = 0;
	size_t ddi_count = 0;

	mainboard_get_dxio_ddi_descriptors(&dxio, &dxio_count, &ddi, &ddi_count);
	if (validate_descriptors(dxio, dxio_count)) {
		printk(BIOS_ERR, "Unsupported DXIO descriptors\n");
		return CB_ERR;
	}

	/* The ACP driver sets up the audio pins shared with the HDA controller */
	if (hda_enabled && !is_dev_enabled(DEV_PTR(acp))) {
		printk(BIOS_ERR, "HDA requires the ACP to be enabled\n");
		return CB_ERR;
	}

	init_pcie_topology();

	if (picasso_smu_test_message())
		return CB_ERR;

	pcie_pre_training_init(hda_enabled);

	if (picasso_acp_power_off())
		return CB_ERR;

	if (dxio_train(dxio, dxio_count)) {
		printk(BIOS_ERR, "DXIO training failed\n");
		return CB_ERR;
	}

	pcie_post_training_init();

	return CB_SUCCESS;
}
