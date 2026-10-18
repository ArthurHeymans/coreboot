/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_ZEPPELIN_NATIVE_INTERNAL_H
#define AMD_ZEPPELIN_NATIVE_INTERNAL_H

#include <soc/native.h>
#include <types.h>

/* psp.c */
void zeppelin_psp_bar_init(void);
void zeppelin_psp_lock_df(void);

/* data_fabric.c */
void zeppelin_df_mmio_init(void);
void zeppelin_df_early_init(void);
void zeppelin_df_scrub_init(void);
void zeppelin_df_late_init(void);
void zeppelin_df_disable_ext_cfg(void);

/* ccx.c */
void zeppelin_ccx_bsp_init(void);
void zeppelin_ccx_get_cac_weights(void);

#endif /* AMD_ZEPPELIN_NATIVE_INTERNAL_H */
