/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_PICASSO_NATIVE_INTERNAL_H
#define AMD_PICASSO_NATIVE_INTERNAL_H

#include <soc/native.h>
#include <types.h>

/* The silicon ID packs the package type in bits [19:16] */
#define SILICON_ID_PACKAGE(id)	(((id) >> 16) & 0xf)
#define SILICON_ID_NO_GFX	BIT(16)

/* romstage.c */
void picasso_native_early_init(void);

/* psp.c */
void picasso_psp_bar_init(void);

/* data_fabric.c */
void picasso_df_mmio_init(void);
void picasso_df_early_init(void);
void picasso_df_late_init(void);

/* ccx.c */
void picasso_ccx_bsp_init(void);
void picasso_ccx_mca_init(void);

/* smu.c */
struct picasso_cpu_topology {
	unsigned int core_count;
	unsigned int complex_count;
	unsigned int threads_per_core;
};

void picasso_get_cpu_topology(struct picasso_cpu_topology *topology);
enum cb_err picasso_apply_cpu_topology_policy(void);
enum cb_err picasso_restore_cpu_topology_policy(void);
enum cb_err picasso_smu_launch_aps(unsigned int ap_count);
void picasso_smu_print_name(void);

#endif /* AMD_PICASSO_NATIVE_INTERNAL_H */
