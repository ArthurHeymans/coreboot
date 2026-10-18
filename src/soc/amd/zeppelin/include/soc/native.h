/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_ZEPPELIN_NATIVE_H
#define AMD_ZEPPELIN_NATIVE_H

#include <device/device.h>
#include <stdint.h>

/* Entry points of the native silicon initialization */

/* Ramstage silicon initialization before MP init and PCI enumeration */
void zeppelin_native_silicon_init(void);
/* Per-CPU initialization, runs on every CPU */
void zeppelin_native_cpu_init(void);
/* Report the DRAM above cbmem_top() */
void zeppelin_native_add_memmap(struct device *dev, unsigned long *idx);

/*
 * Optional mainboard hooks at the points where the vendor board PEIMs run:
 * after the FCH reset tasks (romstage, before NBIO and CCX init) and after
 * DXIO link training (ramstage, before PCI enumeration).
 */
void mainboard_fch_reset_done(void);
void mainboard_dxio_training_done(void);

#endif /* AMD_ZEPPELIN_NATIVE_H */
