/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/smu.h>
#include <console/console.h>
#include <soc/smu.h>
#include <soc/soc_util.h>
#include <string.h>
#include <types.h>

#include "native.h"
#include "smu.h"
#include "topology.h"

/* SMU features enabled on SP4 and SP4r2 (vendor PCD 0xbb) */
#define SMU_FEATURES_SP4		0xe73d
#define  SMU_FEATURE_CORE_CSTATES	BIT(8)

#define SMU_INTERFACE_VERSION		7

enum cb_err zeppelin_smu_test_message(unsigned int die)
{
	struct smu_payload payload = { .msg = { 0xaa55aa55 } };

	return zeppelin_smu_send(die, SMC_MSG_TEST, &payload);
}

static enum cb_err smu_send_arg(unsigned int die, uint32_t message, uint32_t arg)
{
	struct smu_payload payload = { .msg = { arg } };

	return zeppelin_smu_send(die, message, &payload);
}

void zeppelin_get_cpu_topology(unsigned int die, struct zeppelin_cpu_topology *topology)
{
	const uint32_t thread_config = zeppelin_smn_read32(die, SMU_THREAD_CONFIG);

	topology->core_count = (thread_config & SMU_THREAD_CONFIG_CORE_COUNT_MASK) + 1;
	topology->complex_count = ((thread_config & SMU_THREAD_CONFIG_COMPLEX_COUNT_MASK) >>
				   SMU_THREAD_CONFIG_COMPLEX_COUNT_SHIFT) + 1;
	topology->threads_per_core = (thread_config & SMU_THREAD_CONFIG_NOT_SMT) ? 1 : 2;
}

unsigned int zeppelin_thread_count(void)
{
	unsigned int threads = 0;

	for (unsigned int die = 0; die < zeppelin_die_count(); die++) {
		struct zeppelin_cpu_topology t;

		zeppelin_get_cpu_topology(die, &t);
		threads += t.complex_count * t.core_count * t.threads_per_core;
	}
	return threads;
}

/*
 * The SMU releases a thread from reset when its ThreadEn bit (complex * 8 + core *
 * threads + thread) gets set and message 1 is sent. Thread 0 of die 0 is the BSP.
 * Software can only set ThreadEn bits, not clear them.
 */
enum cb_err zeppelin_smu_launch_aps(unsigned int ap_count)
{
	unsigned int started = 0;

	for (unsigned int die = 0; die < zeppelin_die_count(); die++) {
		struct zeppelin_cpu_topology t;
		uint32_t enabled;

		zeppelin_get_cpu_topology(die, &t);
		enabled = zeppelin_smn_read32(die, SMU_THREAD_ENABLE);

		for (unsigned int complex = 0; complex < t.complex_count; complex++) {
			for (unsigned int thread = 0;
			     thread < t.core_count * t.threads_per_core; thread++) {
				const uint32_t bit = BIT(complex * 8 + thread);

				if (!die && !complex && !thread)
					continue;
				if (started == ap_count)
					return CB_SUCCESS;
				started++;
				if (enabled & bit)
					continue;

				enabled |= bit;
				zeppelin_smn_write32(die, SMU_THREAD_ENABLE, enabled);
				if (zeppelin_smu_test_message(die))
					return CB_ERR;
			}
		}
	}

	if (started != ap_count) {
		printk(BIOS_ERR, "Can only start %u of %u APs\n", started, ap_count);
		return CB_ERR;
	}

	return CB_SUCCESS;
}

void zeppelin_smu_print_name(void)
{
	char name[12 * sizeof(uint32_t)];

	for (unsigned int i = 0; i < 12; i++) {
		struct smu_payload payload = { .msg = { i } };

		if (zeppelin_smu_send(0, SMC_MSG_GET_NAME_STRING, &payload))
			return;
		memcpy(&name[i * sizeof(uint32_t)], &payload.msg[0], sizeof(uint32_t));
	}
	name[sizeof(name) - 1] = '\0';

	printk(BIOS_INFO, "CPU: %s\n", name);
}

/*
 * The SMU DXE driver of the vendor firmware, per die. The BIOS policy table is
 * all zeros: every field defaults to 0 there, which selects the fused values.
 */
enum cb_err zeppelin_smu_init(void)
{
	/* The SMU reads it with DMA */
	static uint8_t policy_table[0x7c] __aligned(64);

	memset(policy_table, 0, sizeof(policy_table));

	for (unsigned int die = 0; die < zeppelin_die_count(); die++) {
		const uint8_t bus = zeppelin_die_bus(die);
		struct smu_payload version = { .msg = { SMU_INTERFACE_VERSION } };

		/* The vendor code asserts (AGESA event 0xa2411310) and goes on */
		if (zeppelin_smu_send(die, SMC_MSG_GET_VERSION, &version))
			return CB_ERR;
		if (version.msg[0] != SMU_INTERFACE_VERSION)
			printk(BIOS_ERR, "SMU %u: interface version %u, expected %u\n", die,
			       version.msg[0], SMU_INTERFACE_VERSION);

		if (smu_send_arg(die, SMC_MSG_SET_POLICY_ADDRESS_LO, (uintptr_t)policy_table) ||
		    smu_send_arg(die, SMC_MSG_SET_POLICY_ADDRESS_HI, 0) ||
		    smu_send_arg(die, SMC_MSG_TRANSFER_POLICY, 0) ||
		    smu_send_arg(die, SMC_MSG_3B, bus) ||
		    smu_send_arg(die, SMC_MSG_3D, bus))
			return CB_ERR;
	}

	return CB_SUCCESS;
}

/*
 * The vendor SMU driver's MP-services callback ("Callback Ac Btc"). AC BTC,
 * which runs a workload on every AP with messages 0x23/0x24/0x25, is off
 * (vendor PCD 0x1bf = 0). DC BTC is on for SP4 and SP4r2 (the SMU driver sets
 * PCD 0x1be): message 0x25 with argument 0 to die 0. Then message 8 goes to
 * every die with the CORE_CSTATES bit of the feature mask.
 */
enum cb_err zeppelin_smu_aps_ready(void)
{
	if (smu_send_arg(0, SMC_MSG_DC_BTC, 0))
		return CB_ERR;

	for (unsigned int die = 0; die < zeppelin_die_count(); die++) {
		if (smu_send_arg(die, SMC_MSG_FINAL, SMU_FEATURES_SP4 & SMU_FEATURE_CORE_CSTATES))
			return CB_ERR;
	}

	return CB_SUCCESS;
}

/* CallbackAfterCcxDxeDone of the vendor SMU driver, per die */
enum cb_err zeppelin_smu_ccx_done(void)
{
	for (unsigned int die = 0; die < zeppelin_die_count(); die++) {
		if (smu_send_arg(die, SMC_MSG_CCX_DONE, 0) ||
		    smu_send_arg(die, SMC_MSG_ENABLE_FEATURES, SMU_FEATURES_SP4) ||
		    smu_send_arg(die, SMC_MSG_38, 0) ||
		    smu_send_arg(die, SMC_MSG_39, 0))
			return CB_ERR;
	}

	return CB_SUCCESS;
}

/* The CAC weights that the CCX code loads into the DPM weight MSRs */
enum cb_err zeppelin_smu_cac_weights(uint64_t weights[SMU_CAC_WEIGHT_COUNT])
{
	for (unsigned int i = 0; i < SMU_CAC_WEIGHT_COUNT; i++) {
		struct smu_payload payload = { .msg = { i } };

		if (zeppelin_smu_send(0, SMC_MSG_GET_CAC_WEIGHT, &payload))
			return CB_ERR;
		weights[i] = (uint64_t)payload.msg[0] << 32 | payload.msg[1];
	}

	return CB_SUCCESS;
}
