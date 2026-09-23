/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_PICASSO_NATIVE_APOB_H
#define AMD_PICASSO_NATIVE_APOB_H

#include <types.h>

/* Maximum number of memory holes in the APOB system memory map */
#define APOB_MAX_HOLES		18

/* The APOB reports the address ranges that aren't usable RAM as holes */
enum apob_hole_type {
	APOB_HOLE_UMA = 0,
	APOB_HOLE_MMIO = 1,
	APOB_HOLE_PRIVILEGED_DRAM = 2,
};

struct picasso_apob_hole {
	uint64_t base;
	uint64_t size;
	uint32_t type;
};

bool picasso_apob_is_valid(const void *apob);
/* Memory clock in MHz, 0 if not reported */
uint16_t picasso_apob_memclk(const void *apob);
/* Returns the number of holes or -1 on error. *top is the top of system memory. */
int picasso_apob_memory_map(const void *apob, uint64_t *top,
			    struct picasso_apob_hole holes[APOB_MAX_HOLES]);
/* Top of the DRAM below 4 GiB, 0 on error */
uint32_t picasso_apob_tolm(const void *apob);

#endif /* AMD_PICASSO_NATIVE_APOB_H */
