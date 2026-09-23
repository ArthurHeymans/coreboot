/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_PICASSO_NATIVE_FCH_H
#define AMD_PICASSO_NATIVE_FCH_H

#include <types.h>

void picasso_fch_ab_init(void);
void picasso_fch_legacy_init(void);
void picasso_fch_lpc_init(void);
void picasso_fch_clock_init(void);
enum cb_err picasso_fch_sata_init(void);
void picasso_fch_emmc_init(void);
enum cb_err picasso_fch_usb_init(void);

#endif /* AMD_PICASSO_NATIVE_FCH_H */
