/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_ZEPPELIN_CPU_H
#define AMD_ZEPPELIN_CPU_H

/* Family 17h models 00h-0Fh: ZP-A0 is model 0, ZP-B0/B1/B2 are model 1 */
#define ZEPPELIN_A0_CPUID		CPUID_FROM_FMS(0x17, 0x00, 0)
#define ZEPPELIN_B0_CPUID		CPUID_FROM_FMS(0x17, 0x01, 0)

#endif /* AMD_ZEPPELIN_CPU_H */
