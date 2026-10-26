/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_SP3_NATIVE_SMU_H
#define AMD_SP3_NATIVE_SMU_H

#include <types.h>

#define SMU_CAC_WEIGHT_COUNT 21

enum cb_err sp3_smu_test_message(unsigned int rb);
unsigned int sp3_thread_count(void);
enum cb_err sp3_smu_launch_aps(unsigned int ap_count);
void sp3_smu_print_name(void);
enum cb_err sp3_smu_init(void);
enum cb_err sp3_smu_aps_ready(void);
enum cb_err sp3_smu_ccx_done(void);
enum cb_err sp3_smu_cac_weights(uint64_t weights[SMU_CAC_WEIGHT_COUNT]);

#endif /* AMD_SP3_NATIVE_SMU_H */
