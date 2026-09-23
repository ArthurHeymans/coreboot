/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/smn.h>
#include <amdblocks/smu.h>
#include <console/console.h>
#include <cf9_reset.h>
#include <soc/smu.h>
#include <static.h>
#include <string.h>
#include <types.h>

#include "../chip.h"
#include "native.h"
#include "smu.h"

enum cb_err picasso_smu_test_message(void)
{
	struct smu_payload payload = { .msg = { 0xaa55aa55 } };

	return send_smu_message(SMC_MSG_TEST, &payload);
}

static enum cb_err downcore_policy(unsigned int mode, unsigned int *cores, uint32_t *mask)
{
	switch (mode) {
	case DOWNCORE_1:
		*cores = 1;
		*mask = 0xfe;
		return CB_SUCCESS;
	case DOWNCORE_2:
		*cores = 2;
		*mask = 0xfc;
		return CB_SUCCESS;
	case DOWNCORE_3:
		*cores = 3;
		*mask = 0xf8;
		return CB_SUCCESS;
	default:
		return CB_ERR;
	}
}

/* CPU topology after applying the devicetree downcore and SMT settings */
void picasso_get_cpu_topology(struct picasso_cpu_topology *topology)
{
	const struct soc_amd_picasso_config *config = config_of_soc();
	const uint32_t thread_config = smn_read32(SMU_THREAD_CONFIG);
	unsigned int requested_cores;
	uint32_t disable_mask;

	topology->core_count = (thread_config & SMU_THREAD_CONFIG_CORE_COUNT_MASK) + 1;
	topology->complex_count = ((thread_config & SMU_THREAD_CONFIG_COMPLEX_COUNT_MASK) >>
				   SMU_THREAD_CONFIG_COMPLEX_COUNT_SHIFT) + 1;
	topology->threads_per_core = (thread_config & SMU_THREAD_CONFIG_NOT_SMT) ? 1 : 2;
	if (config->downcore_mode != DOWNCORE_AUTO &&
	    downcore_policy(config->downcore_mode, &requested_cores, &disable_mask) ==
	    CB_SUCCESS && requested_cores < topology->core_count)
		topology->core_count = requested_cores;
	if (config->smt_disable)
		topology->threads_per_core = 1;
}

/* The downcore and SMT settings get lost in S3, so program them again on resume. */
enum cb_err picasso_restore_cpu_topology_policy(void)
{
	const struct soc_amd_picasso_config *config = config_of_soc();
	unsigned int requested_cores;
	uint32_t disable_mask;

	if (config->downcore_mode != DOWNCORE_AUTO) {
		if (downcore_policy(config->downcore_mode, &requested_cores, &disable_mask))
			return CB_ERR;
		smn_write32(SMU_DOWNCORE_CONTROL, disable_mask);
		if (picasso_smu_test_message())
			return CB_ERR;
	}
	if (config->smt_disable) {
		smn_write32(SMU_THREAD_CONFIG,
			    smn_read32(SMU_THREAD_CONFIG) | SMU_THREAD_CONFIG_DISABLE_SMT);
		if (picasso_smu_test_message())
			return CB_ERR;
	}
	return CB_SUCCESS;
}

/*
 * Apply the devicetree downcore and SMT settings. The new topology only takes effect after a
 * reset, so this resets the system when the topology needs to change.
 */
enum cb_err picasso_apply_cpu_topology_policy(void)
{
	const struct soc_amd_picasso_config *config = config_of_soc();
	const uint32_t thread_config = smn_read32(SMU_THREAD_CONFIG);
	const unsigned int current_cores = (thread_config & SMU_THREAD_CONFIG_CORE_COUNT_MASK) + 1;
	const unsigned int threads_per_core =
		(thread_config & SMU_THREAD_CONFIG_NOT_SMT) ? 1 : 2;
	unsigned int requested_cores = 0;
	uint32_t disable_mask = 0;
	bool reset_needed = false;

	if (config->downcore_mode == DOWNCORE_AUTO && !config->smt_disable)
		return CB_SUCCESS;

	if (config->downcore_mode != DOWNCORE_AUTO) {
		if (downcore_policy(config->downcore_mode, &requested_cores, &disable_mask))
			return CB_ERR;
		if (current_cores < requested_cores) {
			printk(BIOS_ERR, "Can't enable disabled cores without a cold boot: "
			       "current=%u requested=%u\n", current_cores, requested_cores);
			return CB_ERR;
		}
		if (current_cores > requested_cores) {
			smn_write32(SMU_DOWNCORE_CONTROL, disable_mask);
			if (picasso_smu_test_message())
				return CB_ERR;
			reset_needed = true;
		}
	}

	if (config->smt_disable && threads_per_core > 1) {
		smn_write32(SMU_THREAD_CONFIG,
			    smn_read32(SMU_THREAD_CONFIG) | SMU_THREAD_CONFIG_DISABLE_SMT);
		if (picasso_smu_test_message())
			return CB_ERR;
		reset_needed = true;
	}

	if (reset_needed) {
		printk(BIOS_INFO, "Resetting to apply the new CPU topology\n");
		system_reset();
	}

	return CB_SUCCESS;
}

/*
 * The SMU releases the APs from reset when their ThreadEn bit gets set. Thread 0 of complex 0
 * is the BSP, which is already running. Software can only set ThreadEn bits, not clear them.
 */
enum cb_err picasso_smu_launch_aps(unsigned int ap_count)
{
	struct picasso_cpu_topology topology;
	unsigned int threads_per_complex;
	unsigned int started = 0;

	picasso_get_cpu_topology(&topology);
	threads_per_complex = topology.core_count * topology.threads_per_core;

	for (unsigned int complex = 0; complex < topology.complex_count; complex++) {
		for (unsigned int thread = 0; thread < threads_per_complex; thread++) {
			const unsigned int bit = complex * 8 + thread;
			uint32_t enabled;

			if ((!complex && !thread) || bit >= 16)
				continue;
			if (started == ap_count)
				return CB_SUCCESS;

			enabled = smn_read32(SMU_THREAD_ENABLE);
			if (!(enabled & BIT(bit))) {
				smn_write32(SMU_THREAD_ENABLE, enabled | BIT(bit));
				if (picasso_smu_test_message())
					return CB_ERR;
			}
			started++;
		}
	}

	if (started != ap_count) {
		printk(BIOS_ERR, "Can only start %u of %u APs\n", started, ap_count);
		return CB_ERR;
	}

	return CB_SUCCESS;
}

void picasso_smu_print_name(void)
{
	char name[12 * sizeof(uint32_t)];

	for (unsigned int i = 0; i < 12; i++) {
		struct smu_payload payload = { .msg = { i } };

		if (send_smu_message(SMC_MSG_GET_NAME_STRING, &payload))
			return;
		memcpy(&name[i * sizeof(uint32_t)], &payload.msg[0], sizeof(uint32_t));
	}
	name[sizeof(name) - 1] = '\0';

	printk(BIOS_INFO, "CPU: %s\n", name);
}
