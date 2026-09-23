/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_PICASSO_NATIVE_H
#define AMD_PICASSO_NATIVE_H

#include <device/device.h>
#include <stdint.h>

/*
 * Entry points of the native silicon initialization (SOC_AMD_PICASSO_NATIVE) that get
 * called from the code shared with the FSP path.
 */

/* Ramstage silicon initialization before MP init and PCI enumeration */
void picasso_native_silicon_init(void);
/* Per-CPU initialization, runs on every CPU */
void picasso_native_cpu_init(void);
/* Report the DRAM above cbmem_top() */
void picasso_native_add_memmap(struct device *dev, unsigned long *idx);
/* Silicon ID reported by the SMU, 0 when not available yet */
uint32_t picasso_native_silicon_id(void);

#endif /* AMD_PICASSO_NATIVE_H */
