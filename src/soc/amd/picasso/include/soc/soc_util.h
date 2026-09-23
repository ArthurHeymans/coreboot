/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_PICASSO_SOC_UTIL_H
#define AMD_PICASSO_SOC_UTIL_H

#include <types.h>

enum socket_type {
	SOCKET_FP5 = 0,
	SOCKET_AM4 = 2,
	SOCKET_FT5 = 3,
};

enum silicon_type {
	SILICON_RV1,
	SILICON_PCO,
	SILICON_RV2,
	SILICON_UNKNOWN,
};

enum soc_type {
	SOC_PICASSO,
	SOC_DALI,
	SOC_POLLOCK,
	SOC_UNKNOWN,
};

/* Bits of the silicon ID reported by the SMU */
#define SILICON_IS_MYSTERY_MEAT	(1 << 31)
#define SILICON_IS_RV2		(1 << 30)

enum socket_type get_socket_type(void);
enum silicon_type get_silicon_type(void);
enum soc_type get_soc_type(void);

/* Silicon ID reported by the FSP in its misc data HOB, or 0 on error */
uint32_t picasso_fsp_silicon_id(void);

void print_socket_type(void);
void print_silicon_type(void);
void print_soc_type(void);

/* function to determine the connectivity feature set */
bool soc_is_reduced_io_sku(void);

/* CPUID-only test; unlike soc_is_raven2(), this does not consult the silicon ID. */
bool soc_cpuid_is_raven2(void);

/* function to determine the iGPU type */
bool soc_is_raven2(void);

#endif /* AMD_PICASSO_SOC_UTIL_H */
