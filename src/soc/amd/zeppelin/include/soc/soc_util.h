/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_ZEPPELIN_SOC_UTIL_H
#define AMD_ZEPPELIN_SOC_UTIL_H

#include <types.h>

/* CPUID Fn8000_0001 EBX[31:28] */
enum zeppelin_package {
	ZEPPELIN_PKG_SP4 = 1,	/* EPYC Embedded 3000, two dies */
	ZEPPELIN_PKG_AM4 = 2,
	ZEPPELIN_PKG_SP4R2 = 3,	/* EPYC Embedded 3000, one die */
	ZEPPELIN_PKG_SP3 = 4,
};

/* Logical revision bits as used by the vendor firmware's register tables */
#define ZEPPELIN_REV_A0		BIT(0)
#define ZEPPELIN_REV_B0		BIT(1)
#define ZEPPELIN_REV_B1		BIT(2)
#define ZEPPELIN_REV_B2		BIT(3)

enum zeppelin_package zeppelin_package(void);

/* Logical revision of the CPU, or 0 for an unknown model/stepping */
uint8_t zeppelin_revision(void);

/*
 * Only SP4 and SP4r2 parts of family 17h models 00h-0Fh are supported. The
 * vendor firmware installs nothing on other packages.
 */
bool zeppelin_is_supported(void);

void zeppelin_print_cpu(void);

#endif /* AMD_ZEPPELIN_SOC_UTIL_H */
