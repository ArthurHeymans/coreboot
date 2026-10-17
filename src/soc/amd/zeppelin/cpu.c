/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/cpu.h>
#include <cpu/cpu.h>
#include <device/device.h>
#include <soc/cpu.h>
#include <soc/native.h>

/* Two dies with up to 8 cores and 2 threads each (SP4) */
static_assert(CONFIG_MAX_CPUS == 32, "Do not override MAX_CPUS. To reduce the number of "
	"available cores, use the downcore_mode and disable_smt devicetree settings instead.");

static void zeppelin_cpu_init(struct device *dev)
{
	amd_cpu_init(dev);
	zeppelin_native_cpu_init();
}

static struct device_operations cpu_dev_ops = {
	.init = zeppelin_cpu_init,
};

static struct cpu_device_id cpu_table[] = {
	{ X86_VENDOR_AMD, ZEPPELIN_A0_CPUID, CPUID_ALL_STEPPINGS_MASK },
	{ X86_VENDOR_AMD, ZEPPELIN_B0_CPUID, CPUID_ALL_STEPPINGS_MASK },
	CPU_TABLE_END
};

static const struct cpu_driver model_17 __cpu_driver = {
	.ops      = &cpu_dev_ops,
	.id_table = cpu_table,
};
