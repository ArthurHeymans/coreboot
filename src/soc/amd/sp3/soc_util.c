/* SPDX-License-Identifier: GPL-2.0-only */

#include <arch/cpu.h>
#include <console/console.h>
#include <soc/identity.h>
#include <soc/soc_util.h>

static struct sp3_identity identity(void)
{
	struct sp3_identity id;

	if (!sp3_identify(cpuid_eax(0x80000001), cpuid_ebx(0x80000001), &id))
		die("Unknown SP3 processor/revision\n");
	return id;
}

enum sp3_package sp3_package(void)
{
	return cpuid_ebx(0x80000001) >> 28;
}

uint8_t sp3_revision(void)
{
	const struct sp3_identity id = identity();

	return id.revision == SP3_A0 ? 1 : 0x10 << (id.revision - SP3_B0);
}

bool sp3_is_supported(void)
{
	/* The initial S8030 image uses the bench's Rome-only PSP/ABL package. */
	return identity().generation == SP3_ROME;
}

void sp3_print_cpu(void)
{
	printk(BIOS_INFO, "SP3 CPUID %08x, revision mask %02x\n",
	       cpuid_eax(0x80000001), sp3_revision());
}
