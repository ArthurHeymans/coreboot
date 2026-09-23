/* SPDX-License-Identifier: GPL-2.0-only */

#include <acpi/acpi.h>
#include <amdblocks/smn.h>
#include <amdblocks/smu.h>
#include <device/mmio.h>
#include <soc/smu.h>
#include <soc/soc_util.h>
#include <static.h>
#include <string.h>
#include <types.h>

#include "../chip.h"
#include "fch.h"
#include "fch_regs.h"
#include "native.h"

/*
 * The USB controllers and PHYs are configured by the SMU. SMC_MSG_USB_CONFIG_UPDATE updates
 * the bits in mask of an SMN register of a group with data.
 */
#define USB_GROUP_CONTROLLER0		0x01000001
#define USB_GROUP_PHY			0x01000002
#define USB_GROUP_CONTROLLER1		0x01000003
#define USB_GROUP_TAIL			0x8200000f

/* USB controller SMN registers */
#define USB0_OC_MAP0			0x16d80120
#define USB0_OC_MAP1			0x16d80124
#define USB0_PORT_DISABLE		0x16d80128
#define USB0_FORCE_GEN1			0x16d80130
#define USB0_PORT_CFG			0x16d80104
#define USB0_PORT_CTRL			0x16d8010c
#define USB0_PHY_CFG			0x16d80114
#define USB1_OC_MAP0			0x16f80120
#define USB1_PORT_DISABLE		0x16f80128
#define USB1_FORCE_GEN1			0x16f80130
#define USB1_PORT_CFG			0x16f80104
#define USB1_PORT_CTRL			0x16f8010c
#define USB1_PHY_CFG			0x16f80114
#define USB0_SPARSE_MODE		0x16c0c12c
#define USB1_SPARSE_MODE		0x16e0c12c
#define USB0_PORT_GEN1(port)		(0x16c0d024 + (port) * 0x80)
#define USB2_PHY_TUNE(port)		(0x16d1c000 + (port) * 0x400)
#define  USB2_PHY_TUNE_MASK		0xf3bf7007
#define USB1_USB2_PHY_TUNE_REV3		0x16f0c000
#define USB1_USB2_PHY_TUNE		0x16f1c000
#define USB3_PHY_RX_EQ(port)		(0x16d2c064 + (port) * 0x400)
#define USB3_PHY_CTRL0			0x16d2003c
#define USB3_PHY_CTRL1			0x16d4003c

#define USB_PHY_REG_BASE		0xfedc6700

struct usb_settings {
	struct usb2_phy_tune usb2_phy[USB_PORT_COUNT];
	bool single_controller;
	bool package3;
	bool force_gen1;
	uint32_t usb3_port_disable;
	uint32_t usb2_port_disable;
	uint32_t oc_map;
	uint8_t usb3_force_gen1;
};

static enum cb_err usb_config(uint32_t reg, uint32_t mask, uint32_t data, uint32_t group)
{
	struct smu_payload payload = { .msg = { reg, mask, data, group } };

	return send_smu_message(SMC_MSG_USB_CONFIG_UPDATE, &payload);
}

static uint32_t usb2_tune_value(const struct usb2_phy_tune *t)
{
	return (t->com_pds_tune & 7) | ((uint32_t)(t->sq_rx_tune & 7) << 12) |
		((uint32_t)(t->tx_fsls_tune & 0xf) << 16) |
		((uint32_t)(t->tx_pre_emp_amp_tune & 3) << 20) |
		((uint32_t)(t->tx_pre_emp_pulse_tune & 1) << 23) |
		((uint32_t)(t->tx_rise_tune & 3) << 24) |
		((uint32_t)t->tx_vref_tune << 28);
}

static uint32_t usb2_tune_value_controller1(const struct usb2_phy_tune *t)
{
	return ((uint32_t)(t->com_pds_tune & 7) << 23) |
		((uint32_t)(t->sq_rx_tune & 7) << 20) |
		((uint32_t)(t->tx_fsls_tune & 0xf) << 16) |
		((uint32_t)(t->tx_pre_emp_amp_tune & 3) << 10) |
		((uint32_t)(t->tx_pre_emp_pulse_tune & 1) << 12) |
		((uint32_t)(t->tx_rise_tune & 3) << 6) |
		(t->tx_vref_tune & 0xf) |
		((uint32_t)(t->tx_hsxv_tune & 3) << 14) |
		((uint32_t)(t->tx_res_tune & 3) << 8);
}

static uint32_t usb2_crossing_value(const struct usb2_phy_tune *t)
{
	return ((t->tx_res_tune & 3) << 2) | (t->tx_hsxv_tune & 3);
}

/* On package 3 the USB3 ports 1, 2 and 3 are wired as 3, 1 and 2 */
static uint8_t usb3_port_mask(const struct usb_settings *s, uint8_t mask)
{
	mask &= 0xf;
	if (!s->package3)
		return mask;
	return (mask & 1) | ((mask << 2) & 8) | ((mask >> 1) & 6);
}

static uint32_t usb_oc_map0(const struct usb_settings *s)
{
	if (s->single_controller) {
		const uint32_t map = (s->oc_map & 0xffffff) * 0x1000001u;

		return s->package3 ? (map | (s->oc_map << 20)) & 0xf0000000 : map;
	}
	return (s->oc_map & 0xffff) * 0x10001u;
}

static uint32_t usb_oc_map1(const struct usb_settings *s)
{
	if (s->single_controller) {
		const uint32_t map = (s->oc_map >> 8) & 0xff;

		return s->package3 ? map | ((s->oc_map >> 12) & 0xf) | (s->oc_map & 0xf0) : map;
	}
	return ((s->oc_map >> 16) & 0xff) | ((s->oc_map >> 8) & 0xf00);
}

static enum cb_err usb_force_gen1(const struct usb_settings *s)
{
	if (get_socket_type() == SOCKET_AM4 && !s->single_controller &&
	    usb_config(USB1_FORCE_GEN1, 0xf, 1, USB_GROUP_CONTROLLER0))
		return CB_ERR;

	if (!s->force_gen1)
		return CB_SUCCESS;

	if (usb_config(USB0_FORCE_GEN1, 0xf0000, 0xf0000, USB_GROUP_CONTROLLER0))
		return CB_ERR;
	for (unsigned int port = 0; port < 4; port++)
		if (usb_config(USB0_PORT_GEN1(port), 0x400, 0x400, USB_GROUP_CONTROLLER1))
			return CB_ERR;
	return CB_SUCCESS;
}

static enum cb_err usb_controller_setup(const struct usb_settings *s)
{
	const uint32_t usb3 = s->usb3_port_disable;
	const uint32_t usb2 = s->usb2_port_disable;

	if (s->single_controller) {
		return usb_config(USB0_PORT_DISABLE, 0xf003f,
				  (uint32_t)usb3_port_mask(s, usb3) << 16 | (usb2 & 0x3f),
				  USB_GROUP_CONTROLLER0) ||
			usb_config(USB0_OC_MAP0, 0xffffffff, usb_oc_map0(s),
				   USB_GROUP_CONTROLLER0) ||
			usb_config(USB0_OC_MAP1, 0xff, usb_oc_map1(s), USB_GROUP_CONTROLLER0) ||
			usb_config(USB0_SPARSE_MODE, 0x20000, 0, USB_GROUP_CONTROLLER1);
	}

	return usb_config(USB0_PORT_DISABLE, 0xf000f, ((usb3 & 0xf) << 16) | (usb2 & 0xf),
			  USB_GROUP_CONTROLLER0) ||
		usb_config(USB1_PORT_DISABLE, 0xf000f, ((usb3 & 0x10) << 12) | ((usb2 >> 4) & 3),
			   USB_GROUP_CONTROLLER0) ||
		usb_config(USB0_OC_MAP0, 0xffffffff, usb_oc_map0(s), USB_GROUP_CONTROLLER0) ||
		usb_config(USB1_OC_MAP0, 0xfff, usb_oc_map1(s), USB_GROUP_CONTROLLER0) ||
		usb_config(USB0_SPARSE_MODE, 0x20000, 0, USB_GROUP_CONTROLLER1) ||
		usb_config(USB1_SPARSE_MODE, 0x20000, 0, USB_GROUP_CONTROLLER1);
}

static enum cb_err usb3_phy_override(const struct usb_settings *s)
{
	const struct soc_amd_picasso_config *cfg = config_of_soc();

	for (unsigned int i = 0; i < RV2_USB3_PORT_COUNT; i++) {
		/* On package 3 the PHYs 1, 2 and 3 are used by the ports 2, 3 and 1 */
		const unsigned int port = s->package3 && i ? (i == 3 ? 1 : i + 1) : i;
		const struct usb3_phy_tune *tune = &cfg->usb3_phy_tune_params[port];

		if (usb_config(USB3_PHY_RX_EQ(i), 0x1f,
			       ((tune->rx_eq_delta_iq_ovrd_en & 1) << 4) |
			       (tune->rx_eq_delta_iq_ovrd_val & 0xf), USB_GROUP_PHY))
			return CB_ERR;
	}

	return usb_config(USB3_PHY_CTRL0, 0x3ff, (cfg->usb3_rx_vref_ctrl & 0x1f) |
			  ((cfg->usb3_rx_vref_ctrl_en & 1) << 5) |
			  ((cfg->usb_3_tx_vboost_lvl & 7) << 6) |
			  ((cfg->usb_3_tx_vboost_lvl_en & 1) << 9), USB_GROUP_PHY) ||
		usb_config(USB3_PHY_CTRL1, 0x3ff, (cfg->usb_3_rx_vref_ctrl_x & 0x1f) |
			   ((cfg->usb_3_rx_vref_ctrl_en_x & 1) << 5) |
			   ((cfg->usb_3_tx_vboost_lvl_x & 7) << 6) |
			   ((cfg->usb_3_tx_vboost_lvl_en_x & 1) << 9), USB_GROUP_PHY);
}

static enum cb_err usb_phy_setup(const struct usb_settings *s)
{
	const struct soc_amd_picasso_config *cfg = config_of_soc();
	const unsigned int usb2_ports = s->single_controller ? 6 : 4;
	const uint8_t gen1_ports = usb3_port_mask(s, s->usb3_force_gen1);

	for (unsigned int i = 0; i < usb2_ports; i++) {
		if (usb_config(USB2_PHY_TUNE(i), USB2_PHY_TUNE_MASK,
			       usb2_tune_value(&s->usb2_phy[i]), USB_GROUP_CONTROLLER0) ||
		    usb_config(USB2_PHY_TUNE(i) + 4, 0xf, usb2_crossing_value(&s->usb2_phy[i]),
			       USB_GROUP_CONTROLLER0))
			return CB_ERR;
	}

	if (s->single_controller && cfg->usb3_phy_override &&
	    usb3_phy_override(s))
		return CB_ERR;

	if (!s->single_controller) {
		if (usb_config(USB1_USB2_PHY_TUNE_REV3, 0x03ffdfcf,
			       usb2_tune_value_controller1(&s->usb2_phy[4]),
			       USB_GROUP_CONTROLLER0) ||
		    usb_config(USB1_USB2_PHY_TUNE, USB2_PHY_TUNE_MASK,
			       usb2_tune_value(&s->usb2_phy[5]), USB_GROUP_CONTROLLER0) ||
		    usb_config(USB1_USB2_PHY_TUNE + 4, 0xf,
			       usb2_crossing_value(&s->usb2_phy[5]), USB_GROUP_CONTROLLER0) ||
		    usb_config(USB0_PHY_CFG, 0xffffffff, 0, USB_GROUP_CONTROLLER0))
			return CB_ERR;
	}
	if (usb_config(s->single_controller ? USB0_PHY_CFG : USB1_PHY_CFG, 0xffffffff, 0,
		       USB_GROUP_CONTROLLER0))
		return CB_ERR;

	if (gen1_ports) {
		if (usb_config(USB0_FORCE_GEN1, 0xf0000, (uint32_t)gen1_ports << 16,
			       USB_GROUP_CONTROLLER0))
			return CB_ERR;
		for (unsigned int port = 0; port < 4; port++)
			if ((gen1_ports & BIT(port)) &&
			    usb_config(USB0_PORT_GEN1(port), 0x400, 0x400, USB_GROUP_CONTROLLER1))
				return CB_ERR;
	}

	return CB_SUCCESS;
}

static enum cb_err usb_phy_tail(void)
{
	if (usb_config(0, 0, 0, USB_GROUP_TAIL))
		return CB_ERR;
	for (unsigned int port = 0; port < 4; port++) {
		if (usb_config(0x16d2c19c + port * 0x400, 0xffff, 0x1ff, USB_GROUP_PHY) ||
		    usb_config(0x16d2c1ac + port * 0x400, 0xffff, 0x80, USB_GROUP_PHY))
			return CB_ERR;
	}
	for (unsigned int port = 0; port < 4; port++)
		if (usb_config(0x16d2415c + port * 0x400, 0x7000, 0x3000, USB_GROUP_PHY))
			return CB_ERR;
	return CB_SUCCESS;
}

static void usb_early_setup(bool s3)
{
	static const uint32_t values[] = {
		0x10100, 0x9393, 0x303, 0, 0x10100, 0x9393, 0x303, 0,
	};

	for (unsigned int i = 0; i < ARRAY_SIZE(values); i++) {
		const uintptr_t reg = USB_PHY_REG_BASE + i * 4;
		const uint32_t keep = (i == 0 || i == 4) ? 0xff : 0;

		write32p(reg, (read32p(reg) & keep) | values[i]);
	}
	if (!s3)
		smn_write32(USB0_PORT_CFG, 0x33);
	smn_write32(USB0_PORT_CTRL, smn_read32(USB0_PORT_CTRL) & ~BIT(6));
	if (!s3)
		smn_write32(USB1_PORT_CFG, 0x33);
	smn_write32(USB1_PORT_CTRL, smn_read32(USB1_PORT_CTRL) & ~BIT(6));
}

static void usb_settings_init(struct usb_settings *s)
{
	static const struct usb2_phy_tune default_usb2_phy = {
		.com_pds_tune = 3,
		.sq_rx_tune = 3,
		.tx_fsls_tune = 3,
		.tx_pre_emp_amp_tune = 3,
		.tx_rise_tune = 1,
		.tx_vref_tune = 6,
		.tx_hsxv_tune = 3,
		.tx_res_tune = 1,
	};
	static const struct usb2_phy_tune default_usb2_phy_port4_5 = {
		.com_pds_tune = 3,
		.sq_rx_tune = 3,
		.tx_fsls_tune = 3,
		.tx_pre_emp_amp_tune = 2,
		.tx_rise_tune = 1,
		.tx_vref_tune = 5,
		.tx_hsxv_tune = 3,
		.tx_res_tune = 1,
	};
	const struct soc_amd_picasso_config *cfg = config_of_soc();

	memset(s, 0, sizeof(*s));
	/* Raven2 has only one USB controller */
	s->single_controller = get_silicon_type() == SILICON_RV2;
	s->package3 = SILICON_ID_PACKAGE(picasso_native_silicon_id()) == 3;
	s->usb3_force_gen1 = cfg->usb3_port_force_gen1.usb3_port_force_gen1_en;

	for (unsigned int port = 0; port < USB_PORT_COUNT; port++) {
		if (cfg->has_usb2_phy_tune_params)
			s->usb2_phy[port] = cfg->usb_2_port_tune_params[port];
		else
			s->usb2_phy[port] = port < 4 ? default_usb2_phy :
				default_usb2_phy_port4_5;
		s->oc_map |= (uint32_t)(cfg->usb_port_overcurrent_pin[port] & 0xf) << (port * 4);
	}

	/* The AM4 package doesn't have USB2 ports 0 and 5 and USB3 port 0 */
	if (get_socket_type() == SOCKET_AM4) {
		s->force_gen1 = true;
		s->usb2_port_disable |= 0x21;
		if (!s->single_controller)
			s->usb3_port_disable |= 1;
	}
}

enum cb_err picasso_fch_usb_init(void)
{
	const bool s3 = acpi_is_wakeup_s3();
	struct smu_payload payload = { 0 };
	struct usb_settings settings;

	usb_settings_init(&settings);

	smn_write32(SATA_SMN_RESET_STATUS, 0);
	smn_clrsetbits32(SATA_SMN_BASE, 0, BIT(0));
	smn_clrsetbits32(SATA_SMN_BASE, 0, BIT(11));

	if (send_smu_message(SMC_MSG_USB_DISABLE, &payload))
		return CB_ERR;

	usb_early_setup(s3);

	if (usb_force_gen1(&settings) || usb_controller_setup(&settings) ||
	    usb_phy_setup(&settings) || usb_phy_tail())
		return CB_ERR;

	memset(&payload, 0, sizeof(payload));
	return send_smu_message(s3 ? SMC_MSG_USB_PHY_COMPLETE_RESUME : SMC_MSG_USB_PHY_COMPLETE,
				&payload);
}
