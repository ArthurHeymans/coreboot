/* SPDX-License-Identifier: GPL-2.0-only */

#include <acpi/acpi.h>
#include <amdblocks/smn.h>
#include <amdblocks/smu.h>
#include <bootstate.h>
#include <console/console.h>
#include <device/device.h>
#include <device/pci.h>
#include <device/pci_ids.h>
#include <device/pci_ops.h>
#include <soc/platform_descriptors.h>
#include <soc/smu.h>
#include <soc/soc_util.h>
#include <static.h>
#include <stdlib.h>
#include <string.h>
#include <types.h>

#include "../chip.h"
#include "fch_regs.h"
#include "native.h"
#include "nbio.h"
#include "smu.h"

/* CPM_CONTROL fields */
#define LCLK_DYN_GATE_ENABLE		BIT(0)
#define TXCLK_DYN_GATE_ENABLE		BIT(1)
#define TXCLK_PERM_GATE_ENABLE		BIT(2)
#define TXCLK_REGS_GATE_ENABLE		BIT(6)
#define LCLK_DYN_GATE_LATENCY_MASK	(3 << 9)
#define LCLK_DYN_GATE_LATENCY(x)	((x) << 9)
#define TXCLK_PERM_GATE_LATENCY_MASK	(3 << 12)
#define TXCLK_PERM_GATE_LATENCY(x)	((x) << 12)
#define TXCLK_REGS_GATE_LATENCY		BIT(14)
#define LCLK_GATE_TXCLK_FREE		BIT(16)
#define CLKREQB_UNGATE_TXCLK_ENABLE	BIT(24)
#define LCLK_GATE_ALLOW_IN_L1		BIT(25)

/* PCIE_CONFIG_CNTL fields */
#define CI_MAX_READ_REQUEST_SIZE_MODE	BIT(20)
#define CI_PRIV_MAX_READ_REQUEST_SIZE(x) ((x) << 21)
#define CI_MAX_READ_REQUEST_MASK	(0xf << 20)

#define PCIE_CNTL_HWINIT_WR_LOCK	BIT(0)

/*
 * Some of the SMU requests return a status other than the standard OK even when the request
 * got processed. Only treat timeouts as an error for those.
 */
static enum cb_err smu_request_payload(enum smu_message_id id, struct smu_payload *payload)
{
	const int32_t response = send_smu_message_raw(id, payload);

	if (response == SMU_MESG_RESP_TIMEOUT) {
		printk(BIOS_ERR, "SMU message 0x%x timed out\n", id);
		return CB_ERR;
	}
	if (response != SMU_MESG_RESP_OK)
		printk(BIOS_DEBUG, "SMU message 0x%x response 0x%08x\n", id, response);
	return CB_SUCCESS;
}

static enum cb_err smu_request(enum smu_message_id id, uint32_t arg0, uint32_t arg1)
{
	struct smu_payload payload = { .msg = { arg0, arg1 } };

	return smu_request_payload(id, &payload);
}

static void build_smu_power_policy(struct picasso_smu_power_policy *policy)
{
	const struct soc_amd_picasso_config *config = config_of_soc();
	/* Only use the board's PPT limits when all of them are set */
	const bool custom_ppt = config->slow_ppt_limit_mW && config->fast_ppt_limit_mW &&
		config->slow_ppt_time_constant_s && config->stapm_time_constant_s;

	const struct picasso_smu_telemetry telemetry = picasso_get_smu_telemetry();

	*policy = (struct picasso_smu_power_policy) {
		.version = SMU_POWER_POLICY_VERSION,
		.stapm_limit_mw = config->sustained_power_limit_mW,
		.fast_ppt_limit_mw = custom_ppt ? config->fast_ppt_limit_mW : 1,
		.slow_ppt_limit_mw = custom_ppt ? config->slow_ppt_limit_mW : 0,
		.slow_ppt_time_constant_s = custom_ppt ? config->slow_ppt_time_constant_s : 28,
		.stapm_time_constant_s = custom_ppt ? config->stapm_time_constant_s : 0,
		.thermctl_limit_c = config->thermctl_limit_degreeC,
		.vrm_current_limit_ma = config->vrm_current_limit_mA,
		.vrm_maximum_current_limit_ma = config->vrm_maximum_current_limit_mA,
		.vrm_soc_current_limit_ma = config->vrm_soc_current_limit_mA,
		.vrm_soc_maximum_current_limit_ma = config->vrm_soc_maximum_current_limit_mA,
		.psi0_current_limit_ma = config->psi0_current_limit_mA,
		.psi0_soc_current_limit_ma = config->psi0_soc_current_limit_mA,
		.prochot_l_deassertion_ramp_time_ms = config->prochot_l_deassertion_ramp_time_ms,
		.system_config = config->system_config,
		.gfx_dldo_psm_margin = 5,
		.core_dldo_bypass = config->core_dldo_bypass,
		.sb_tsi_alert_comparator_mode_enable = config->sb_tsi_alert_comparator_mode_en,
		.vddcr_vdd_voltage_margin_mv = config->vddcr_vdd_voltage_margin_mV,
		.vddcr_soc_voltage_margin_mv = config->vddcr_soc_voltage_margin_mV,
		.use_ccx_freq_curve_fuses = 1,
		.use_gfx_freq_curve_fuses = 1,
		.use_cac_sidd_fuses = 1,
		.telemetry_vddcr_vdd_slope_ma = telemetry.vdd_slope,
		.telemetry_vddcr_vdd_offset = telemetry.vdd_offset,
		.telemetry_vddcr_soc_slope_ma = telemetry.soc_slope,
		.telemetry_vddcr_soc_offset = telemetry.soc_offset,
		.aclk_dpm0_freq_400mhz = config->aclk_dpm0_freq_400MHz,
		.min_soc_vid_offset = config->min_soc_vid_offset,
	};

	printk(BIOS_INFO, "SMU power policy: PPT=%u/%u STAPM=%u/%u PSI=%u/%u "
	       "telemetry=%u/%d,%u/%d system=%u ACLK400=%u\n",
	       policy->fast_ppt_limit_mw, policy->slow_ppt_limit_mw,
	       policy->stapm_limit_mw, policy->stapm_time_constant_s,
	       policy->psi0_current_limit_ma, policy->psi0_soc_current_limit_ma,
	       policy->telemetry_vddcr_vdd_slope_ma, telemetry.vdd_offset,
	       policy->telemetry_vddcr_soc_slope_ma, telemetry.soc_offset,
	       policy->system_config, policy->aclk_dpm0_freq_400mhz);
}

static enum cb_err apply_smu_power_policy(void)
{
	/*
	 * The SMU uses a page for the policy and a scratch page. It keeps using them, so they
	 * are never freed. The ramstage heap is in the reserved CBMEM region.
	 */
	uint8_t *pages = memalign(4 * KiB, 8 * KiB);
	uint8_t *scratch = pages;
	struct picasso_smu_power_policy *policy = (void *)(pages + 4 * KiB);
	struct smu_payload payload = { 0 };

	if (!pages || (uintptr_t)pages > UINT32_MAX - 8 * KiB)
		return CB_ERR;

	memset(pages, 0, 8 * KiB);
	build_smu_power_policy(policy);

	/* The transfer request takes the arguments returned by the address request */
	payload.msg[0] = (uintptr_t)policy;
	if (smu_request_payload(SMC_MSG_SET_POLICY_ADDRESS, &payload))
		return CB_ERR;
	payload.msg[0] = 0;
	if (smu_request_payload(SMC_MSG_TRANSFER_POLICY, &payload))
		return CB_ERR;

	if (smu_request(SMC_MSG_FINALIZE_POLICY, 0, 0) ||
	    smu_request(SMC_MSG_SET_SCRATCH_ADDRESS, (uintptr_t)scratch, 0) ||
	    smu_request(SMC_MSG_TRANSFER_SCRATCH, 0, 0))
		return CB_ERR;

	return CB_SUCCESS;
}

/* Allow L1 and set up the L1 substates and the CLKREQ# routing of the PCIe ports. */
static void pcie_port_power_management(const struct picasso_pcie_topology *topology)
{
	uint8_t requested[PCIE_PORT_COUNT], clocks[PCIE_PORT_COUNT];
	uint32_t clkreq_map = 0;

	smn_clrsetbits32(pcie_core_reg(CPM_CONTROL),
			 LCLK_GATE_TXCLK_FREE | LCLK_DYN_GATE_LATENCY_MASK |
			 TXCLK_PERM_GATE_LATENCY_MASK,
			 LCLK_DYN_GATE_ENABLE | LCLK_DYN_GATE_LATENCY(1) |
			 TXCLK_DYN_GATE_ENABLE | TXCLK_PERM_GATE_ENABLE |
			 TXCLK_REGS_GATE_ENABLE | TXCLK_PERM_GATE_LATENCY(1) |
			 TXCLK_REGS_GATE_LATENCY | CLKREQB_UNGATE_TXCLK_ENABLE |
			 LCLK_GATE_ALLOW_IN_L1);
	smn_clrsetbits32(pcie_core_reg(PCIE_CONFIG_CNTL), CI_MAX_READ_REQUEST_MASK,
			 CI_MAX_READ_REQUEST_SIZE_MODE | CI_PRIV_MAX_READ_REQUEST_SIZE(1));
	smn_clrsetbits32(pcie_core_reg(0x18c), 0, 0x4060);

	for (unsigned int port = 0; port < PCIE_GPP_PORT_COUNT; port++)
		smn_clrsetbits32(pcie_port_reg(port, PCIE_ERR_CNTL), 0, ERR_REPORTING_DIS);

	for (int i = PCIE_PORT_COUNT - 1; i >= 0; i--) {
		const struct picasso_pcie_port *port = &topology->ports[i];

		if (port->allocated && port->aspm == ASPM_L1)
			smn_clrsetbits32(pcie_port_reg(i, PCIE_LC_CNTL6), 0,
					 PCIE_LC_CNTL6_L1_ALLOW);
	}

	for (int i = PCIE_PORT_COUNT - 1; i >= 0; i--) {
		const struct picasso_pcie_port *port = &topology->ports[i];

		if (!port->allocated || !(port->l1_1 || port->l1_2))
			continue;
		if (port->trained) {
			smn_clrsetbits32(pcie_port_reg(i, PCIE_LC_CNTL6), 0,
					 PCIE_LC_CNTL6_L1_ALLOW);
			if (port->l1_2) {
				for (unsigned int j = 0; j < PCIE_PORT_COUNT; j++)
					smn_clrsetbits32(pcie_port_reg(j, 0x304), 0, BIT(5));
			}
		} else if (!port->hotplug) {
			/* Enable the L1 substates in the L1 PM Substates Control 1 register */
			smn_clrsetbits32(pcie_port_cfg(i, PCIE_PORT_L1SS_CTL1), 0xf, 0xf);
			smn_clrsetbits32(pcie_port_reg(i, PCIE_LC_CNTL6), 0,
					 PCIE_LC_CNTL6_L1_ALLOW);
		}
	}

	/*
	 * Route the CLKREQ# pins to the ports that use L1 substates. clocks[] is the CLKREQ#
	 * pin of each port. When two ports use the same pin, the lower port number wins.
	 */
	for (unsigned int i = 0; i < PCIE_PORT_COUNT; i++) {
		requested[i] = 0xff;
		clocks[i] = i;
	}
	for (int i = PCIE_PORT_COUNT - 1; i >= 0; i--) {
		const struct picasso_pcie_port *port = &topology->ports[i];

		if (port->allocated && port->trained && (port->l1_1 || port->l1_2) &&
		    port->clk_req >= CLK_REQ0 && port->clk_req <= CLK_REQ7)
			requested[port->clk_req - CLK_REQ0] = i;
	}
	for (unsigned int port = 0; port < PCIE_PORT_COUNT; port++) {
		for (unsigned int pin = 0; pin < PCIE_PORT_COUNT; pin++) {
			uint8_t clock;

			if (requested[pin] != port)
				continue;
			clock = clocks[port];
			clocks[port] = clocks[pin];
			clocks[pin] = clock;
			requested[pin] = requested[port];
			requested[port] = 0xff;
		}
	}
	for (unsigned int port = 0; port < PCIE_PORT_COUNT; port++)
		clkreq_map |= (uint32_t)clocks[port] << (port * 4);
	smn_write32(pcie_core_reg(PCIE_CLKREQ_MAP), clkreq_map);

	smn_clrsetbits32(FCH_MISC_SMN(0xb0), BIT(8), 0);
	smn_clrsetbits32(FCH_MISC_SMN(0xb0), BIT(24), 0);
	smn_clrsetbits32(FCH_MISC_SMN(0xb4), BIT(25), 0);
	smn_clrsetbits32(FCH_MISC_SMN(0xb4), BIT(24), 0);
}

enum cb_err picasso_nbio_resume_init(void)
{
	if (picasso_pcie_topology_restore())
		return CB_ERR;

	pcie_port_power_management(picasso_pcie_topology());
	return CB_SUCCESS;
}

enum cb_err picasso_nbio_pre_enumeration_init(void)
{
	const struct picasso_pcie_topology *topology = picasso_pcie_topology();

	if (!topology->valid)
		return CB_ERR;

	pcie_port_power_management(topology);

	smn_clrsetbits32(0x11a0b7c0, 0x3, 1);

	if (smu_request(SMC_MSG_SET_POLICY_VERSION, SMU_POWER_POLICY_VERSION, 0))
		return CB_ERR;

	return apply_smu_power_policy();
}

static uint16_t fused_chip_id(void)
{
	return ((smn_read32(SMU_FUSE_CHIP_ID_HIGH) << 5) |
		(smn_read32(SMU_FUSE_CHIP_ID_LOW) >> 27)) & 0xffff;
}

/* Notify the SMU that all CPUs are initialized */
enum cb_err picasso_nbio_ccx_done(void)
{
	const uint16_t chip_id = fused_chip_id();
	const bool gfx_available = picasso_gfx_available();
	uint32_t features = 0xf7fffffd;

	if (chip_id == 0x62e && smu_request(SMC_MSG_35, 0x0e, 0))
		return CB_ERR;
	if (chip_id == 0x633 && smu_request(SMC_MSG_35, 0x08, 0))
		return CB_ERR;

	if (!gfx_available)
		features &= ~BIT(20);
	if (smu_request(SMC_MSG_ENABLE_FEATURES, features, 0x000008f7) ||
	    smu_request(SMC_MSG_0E, 0, 0))
		return CB_ERR;
	if (!gfx_available && smu_request(SMC_MSG_53, 0, 0))
		return CB_ERR;
	if (smu_request(SMC_MSG_3A, 184, 0) ||
	    smu_request(SMC_MSG_42, 0, 0) ||
	    smu_request(SMC_MSG_32, 0, 0) ||
	    smu_request(SMC_MSG_ENABLE_FEATURES, 0, BIT(8)))
		return CB_ERR;

	return CB_SUCCESS;
}

/* Last step of the silicon initialization, after the graphics initialization */
enum cb_err picasso_nbio_post_silicon_init(void)
{
	struct smu_payload payload = { 0 };

	smn_clrsetbits32(IOHC_BRIDGE_CNTL_NBIF1, 0, 0x5);
	smn_clrsetbits32(NBIF_FUNC_STRAP(8), NBIF_FUNC_STRAP_FUNC_EN, 0);
	smn_clrsetbits32(SATA_SMN_BASE, 0, BIT(0));
	smn_clrsetbits32(SATA_SMN_BASE, 0, BIT(11));

	if (get_silicon_type() != SILICON_PCO && get_silicon_type() != SILICON_RV2)
		return CB_SUCCESS;

	return send_smu_message(SMC_MSG_11, &payload);
}

static void pcie_after_enumeration(void *unused)
{
	/* Some PCIe root ports need a different link control setting */
	for (const struct device *dev = all_devices; dev; dev = dev->next) {
		if (!dev->enabled || dev->path.type != DEVICE_PATH_PCI ||
		    pci_read_config16(dev, PCI_VENDOR_ID) != PCI_VID_AMD ||
		    pci_read_config16(dev, PCI_DEVICE_ID) != 0x1c5c)
			continue;

		const uint32_t reg = pcie_port_reg(PCI_FUNC(dev->path.pci.devfn), PCIE_LC_CNTL);
		const uint32_t value = smn_read32(reg);
		smn_write32(reg, (value & 0x00ffffff) | ((((value >> 24) & 0xe9) | 0x14) << 24));
	}

	smn_clrsetbits32(pcie_core_reg(PCIE_CNTL), PCIE_CNTL_HWINIT_WR_LOCK, 0);
}

BOOT_STATE_INIT_ENTRY(BS_DEV_ENABLE, BS_ON_ENTRY, pcie_after_enumeration, NULL);
