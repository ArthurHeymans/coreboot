/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/cpu.h>
#include <cpu/cpu.h>
#include <device/device.h>
#include <soc/cpu.h>
#include <soc/native.h>

/* One SP3 socket, up to 64 cores and two threads per core. */
static_assert(CONFIG_MAX_CPUS == 128, "Do not override MAX_CPUS. To reduce the number of "
	"available cores, use the downcore_mode and disable_smt devicetree settings instead.");

static void sp3_cpu_init(struct device *dev)
{
	amd_cpu_init(dev);
	sp3_native_cpu_init();
}

static struct device_operations cpu_dev_ops = {
	.init = sp3_cpu_init,
};

static struct cpu_device_id cpu_table[] = {
	{ X86_VENDOR_AMD, SP3_A0_CPUID, CPUID_ALL_STEPPINGS_MASK },
	{ X86_VENDOR_AMD, SP3_B0_CPUID, CPUID_ALL_STEPPINGS_MASK },
	CPU_TABLE_END
};

static const struct cpu_driver model_17 __cpu_driver = {
	.ops      = &cpu_dev_ops,
	.id_table = cpu_table,
};
