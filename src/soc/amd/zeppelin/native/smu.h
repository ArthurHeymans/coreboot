/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_ZEPPELIN_NATIVE_SMU_H
#define AMD_ZEPPELIN_NATIVE_SMU_H

#include <types.h>

/* SMU thread enable and topology registers (SMN, per die) */
#define SMU_THREAD_ENABLE			0x0005a818
#define SMU_THREAD_CONFIG			0x0005a81c
#define  SMU_THREAD_CONFIG_CORE_COUNT_MASK	0x0000000f
#define  SMU_THREAD_CONFIG_COMPLEX_COUNT_SHIFT	4
#define  SMU_THREAD_CONFIG_COMPLEX_COUNT_MASK	0x000000f0
#define  SMU_THREAD_CONFIG_NOT_SMT		BIT(8)

#define SMU_CAC_WEIGHT_COUNT			21

struct zeppelin_cpu_topology {
	unsigned int core_count;	/* per core complex */
	unsigned int complex_count;
	unsigned int threads_per_core;
};

/* Send SMC_MSG_TEST, which also makes the SMU pick up the ThreadEn settings */
enum cb_err zeppelin_smu_test_message(unsigned int die);
void zeppelin_get_cpu_topology(unsigned int die, struct zeppelin_cpu_topology *topology);
unsigned int zeppelin_thread_count(void);
enum cb_err zeppelin_smu_launch_aps(unsigned int ap_count);
void zeppelin_smu_print_name(void);
enum cb_err zeppelin_smu_init(void);
enum cb_err zeppelin_smu_ccx_done(void);
enum cb_err zeppelin_smu_cac_weights(uint64_t weights[SMU_CAC_WEIGHT_COUNT]);

#endif /* AMD_ZEPPELIN_NATIVE_SMU_H */
