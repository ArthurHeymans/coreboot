/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_SP3_NATIVE_NBIO_H
#define AMD_SP3_NATIVE_NBIO_H

#include <types.h>

void sp3_nbio_early_init(void);
void sp3_nbio_tom_init(void);
void sp3_nbio_ioapic_init(void);
void sp3_nbio_pre_enumeration_init(void);
enum cb_err sp3_dxio_init(void);

#endif /* AMD_SP3_NATIVE_NBIO_H */
