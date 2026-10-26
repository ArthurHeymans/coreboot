/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_SP3_NATIVE_INTERNAL_H
#define AMD_SP3_NATIVE_INTERNAL_H

#include <soc/native.h>
#include <types.h>

/* psp.c */
void sp3_psp_bar_init(void);
void sp3_psp_lock_df(void);
void sp3_psp_cache_bars(void);
uint32_t sp3_psp_bar(unsigned int die);

/* data_fabric.c */
void sp3_df_mmio_init(void);
void sp3_df_early_init(void);
void sp3_df_scrub_init(void);
void sp3_df_late_init(void);
void sp3_df_disable_ext_cfg(void);
void sp3_df_route_vga(unsigned int die);

/* ccx.c */
void sp3_ccx_bsp_init(void);
void sp3_ccx_get_cac_weights(void);

#endif /* AMD_SP3_NATIVE_INTERNAL_H */
