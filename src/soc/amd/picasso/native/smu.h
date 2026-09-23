/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_PICASSO_NATIVE_SMU_H
#define AMD_PICASSO_NATIVE_SMU_H

#include <amdblocks/smu.h>
#include <types.h>

/* SMU thread enable and topology registers (SMN) */
#define SMU_THREAD_ENABLE			0x0005a818
#define SMU_THREAD_CONFIG			0x0005a81c
#define  SMU_THREAD_CONFIG_CORE_COUNT_MASK	0x0000000f
#define  SMU_THREAD_CONFIG_COMPLEX_COUNT_SHIFT	4
#define  SMU_THREAD_CONFIG_COMPLEX_COUNT_MASK	0x000000f0
#define  SMU_THREAD_CONFIG_NOT_SMT		BIT(8)
#define  SMU_THREAD_CONFIG_DISABLE_SMT		BIT(9)
#define SMU_DOWNCORE_CONTROL			0x0005a860

/* Fuse registers (SMN) */
#define SMU_FUSE_OPN				0x0005d5c0
#define SMU_FUSE_CHIP_ID_LOW			0x0005d5b8
#define SMU_FUSE_CHIP_ID_HIGH			0x0005d5bc

/*
 * Power policy table that gets transferred to the SMU with SMC_MSG_SET_POLICY_ADDRESS and
 * SMC_MSG_TRANSFER_POLICY. The version of this layout is 0x13.
 */
#define SMU_POWER_POLICY_VERSION		0x13

struct picasso_smu_power_policy {
	uint32_t version;
	uint32_t stapm_limit_mw;
	uint32_t fast_ppt_limit_mw;
	uint32_t slow_ppt_limit_mw;
	uint32_t slow_ppt_time_constant_s;
	uint8_t reserved_014[3];
	uint8_t aa_mode_enable;
	uint32_t stapm_time_constant_s;
	uint32_t thermctl_limit_c;
	uint32_t vrm_current_limit_ma;
	uint32_t vrm_maximum_current_limit_ma;
	uint32_t vrm_soc_current_limit_ma;
	uint32_t vrm_soc_maximum_current_limit_ma;
	uint32_t psi0_current_limit_ma;
	uint32_t psi0_soc_current_limit_ma;
	uint32_t prochot_l_deassertion_ramp_time_ms;
	uint8_t system_config;
	uint8_t reserved_03d[0x10];
	uint8_t unknown_04d;
	uint8_t reserved_04e[0x0a];
	uint8_t core_dldo_bypass;
	uint8_t reserved_059[6];
	uint8_t sb_tsi_alert_comparator_mode_enable;
	uint32_t vddcr_vdd_voltage_margin_mv;
	uint32_t vddcr_soc_voltage_margin_mv;
	uint8_t reserved_068[4];
	uint8_t unknown_06c[3];
	uint8_t reserved_06f[0x6d];
	uint32_t telemetry_vddcr_vdd_slope_ma;
	uint32_t telemetry_vddcr_vdd_offset;
	uint32_t telemetry_vddcr_soc_slope_ma;
	uint32_t telemetry_vddcr_soc_offset;
	uint8_t reserved_0ec[0x21];
	uint8_t aclk_dpm0_freq_400mhz;
	uint8_t min_soc_vid_offset;
	uint8_t reserved_10f[0x35];
} __packed;

check_member(picasso_smu_power_policy, aa_mode_enable, 0x017);
check_member(picasso_smu_power_policy, system_config, 0x03c);
check_member(picasso_smu_power_policy, core_dldo_bypass, 0x058);
check_member(picasso_smu_power_policy, telemetry_vddcr_vdd_slope_ma, 0x0dc);
check_member(picasso_smu_power_policy, aclk_dpm0_freq_400mhz, 0x10d);
check_member(picasso_smu_power_policy, min_soc_vid_offset, 0x10e);
_Static_assert(sizeof(struct picasso_smu_power_policy) == 0x144,
	       "Unexpected SMU power policy size");

/* Send SMC_MSG_TEST, which also makes the SMU pick up the thread enable/downcore settings */
enum cb_err picasso_smu_test_message(void);

#endif /* AMD_PICASSO_NATIVE_SMU_H */
