/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_COMMON_ALIB_H
#define AMD_COMMON_ALIB_H

#define ALIB_FUNCTION_REPORT_AC_DC_STATE		0x1
#define ALIB_FUNCTION_DYNAMIC_POWER_THERMAL_CONFIG	0xc
#define ALIB_FUNCTION_HOLD_PCIE_LINK_TRAINING		0x12
#define ALIB_FUNCTION_RELEASE_PCIE_LINK_TRAINING	0x13

/* Parameter IDs for ALIB_FUNCTION_DYNAMIC_POWER_THERMAL_CONFIG, shared with AML. */
#define ALIB_DPTC_THERMAL_CONTROL_LIMIT_ID		0x3
#define ALIB_DPTC_SUSTAINED_POWER_LIMIT_ID		0x5
#define ALIB_DPTC_FAST_PPT_LIMIT_ID			0x6
#define ALIB_DPTC_SLOW_PPT_LIMIT_ID			0x7
#define ALIB_DPTC_SLOW_PPT_TIME_CONSTANT_ID		0x8
#define ALIB_DPTC_PROCHOT_L_DEASSERTION_RAMP_TIME_ID	0x9
#define ALIB_DPTC_VRM_CURRENT_LIMIT_ID			0xb
#define ALIB_DPTC_VRM_MAXIMUM_CURRENT_LIMIT		0xc
/* Picasso: SetVrmSocCurrentLimit (0xe) is not implemented in ALIB. */
#define ALIB_DPTC_VRM_SOC_CURRENT_LIMIT_ID		0xe

#define ALIB_DPTC_STT_ALPHA_APU				0x20
#define ALIB_DPTC_STT_SKIN_TEMPERATURE_LIMIT_APU_ID	0x22
#define ALIB_DPTC_STT_M1_ID				0x26
#define ALIB_DPTC_STT_M2_ID				0x27
#define ALIB_DPTC_STT_C_APU_ID				0x2c
#define ALIB_DPTC_STT_MIN_LIMIT_ID				0x2e

#ifndef __ACPI__

struct alib_dptc_param {
	uint8_t id;
	uint32_t value;
} __packed;

void acpigen_write_alib_dptc_default(uint8_t *default_param, size_t default_param_len);
void acpigen_write_alib_dptc_no_battery(uint8_t *no_battery_param, size_t no_battery_param_len);
void acpigen_write_alib_dptc_tablet(uint8_t *tablet_param, size_t tablet_param_len);
void acpigen_write_alib_dptc_thermal_B(uint8_t *thermal_param_B, size_t thermal_param_B_len);
void acpigen_write_alib_dptc_thermal_C(uint8_t *thermal_param_C, size_t thermal_param_C_len);
void acpigen_write_alib_dptc_thermal_D(uint8_t *thermal_param_D, size_t thermal_param_D_len);
void acpigen_write_alib_dptc_thermal_E(uint8_t *thermal_param_E, size_t thermal_param_E_len);
void acpigen_write_alib_dptc_thermal_F(uint8_t *thermal_param_F, size_t thermal_param_F_len);
void acpigen_write_alib_hold_link_training(const struct device *dev);
void acpigen_write_alib_release_link_training(const struct device *dev);
#endif /* !__ACPI__ */

#endif /* AMD_COMMON_ALIB_H */
