/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_SP3_SOC_UTIL_H
#define AMD_SP3_SOC_UTIL_H

#include <types.h>

/* CPUID Fn8000_0001 EBX[31:28] */
enum sp3_package {
	SP3_PKG_SP4 = 1,	/* EPYC Embedded 3000, two dies */
	SP3_PKG_AM4 = 2,
	SP3_PKG_SP4R2 = 3,	/* EPYC Embedded 3000, one die */
	SP3_PKG_SP3 = 4,
};

/* Rome logical revision bits used by the vendor register tables. */
#define SP3_REV_A0		BIT(0)
#define SP3_REV_B0		BIT(4)
#define SP3_REV_B1		BIT(5)
#define SP3_REV_B2		BIT(6)

enum sp3_package sp3_package(void);

/* Logical revision of the CPU, or 0 for an unknown model/stepping */
uint8_t sp3_revision(void);

/* Only Rome SP3 parts (family 17h, model 31h) are supported by this port. */
bool sp3_is_supported(void);

void sp3_print_cpu(void);

#endif /* AMD_SP3_SOC_UTIL_H */
