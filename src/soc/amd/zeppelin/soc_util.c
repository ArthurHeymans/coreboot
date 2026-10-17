/* SPDX-License-Identifier: GPL-2.0-only */

#include <arch/cpu.h>
#include <console/console.h>
#include <soc/soc_util.h>
#include <types.h>

#define PKG_TYPE_SHIFT		28
#define PKG_TYPE_MASK		(0xf << PKG_TYPE_SHIFT)

/* Fn8000_0001 EAX with the stepping and the low model nibble masked */
#define ZEPPELIN_FAMILY_MASK	0x0fff0f00
#define ZEPPELIN_FAMILY		0x00800f00

enum zeppelin_package zeppelin_package(void)
{
	return (cpuid_ebx(0x80000001) & PKG_TYPE_MASK) >> PKG_TYPE_SHIFT;
}

uint8_t zeppelin_revision(void)
{
	const uint32_t eax = cpuid_eax(0x80000001);
	const uint8_t model = (eax >> 4) & 0xf;
	const uint8_t stepping = eax & 0xf;

	if ((eax & ZEPPELIN_FAMILY_MASK) != ZEPPELIN_FAMILY)
		return 0;

	/*
	 * The vendor firmware treats an unknown stepping as B2; fail closed
	 * instead.
	 */
	if (model == 0 && stepping == 0)
		return ZEPPELIN_REV_A0;
	if (model == 1 && stepping == 0)
		return ZEPPELIN_REV_B0;
	if (model == 1 && stepping == 1)
		return ZEPPELIN_REV_B1;
	if (model == 1 && stepping == 2)
		return ZEPPELIN_REV_B2;
	return 0;
}

bool zeppelin_is_supported(void)
{
	const enum zeppelin_package pkg = zeppelin_package();

	return zeppelin_revision() &&
		(pkg == ZEPPELIN_PKG_SP4 || pkg == ZEPPELIN_PKG_SP4R2);
}

void zeppelin_print_cpu(void)
{
	static const char *const rev_names[] = { "A0", "B0", "B1", "B2" };
	const uint8_t rev = zeppelin_revision();
	const enum zeppelin_package pkg = zeppelin_package();

	printk(BIOS_INFO, "Zeppelin %s, package %s\n",
	       rev ? rev_names[__builtin_ctz(rev)] : "unknown",
	       pkg == ZEPPELIN_PKG_SP4 ? "SP4" :
	       pkg == ZEPPELIN_PKG_SP4R2 ? "SP4r2" : "unsupported");
}
