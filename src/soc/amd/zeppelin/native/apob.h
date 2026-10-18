/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_ZEPPELIN_NATIVE_APOB_H
#define AMD_ZEPPELIN_NATIVE_APOB_H

#include <types.h>

/* Maximum number of memory holes in the APOB system memory map */
#define APOB_MAX_HOLES		18
#define APOB_ANY_INSTANCE	UINT32_MAX

/* The APOB reports the address ranges that aren't usable RAM as holes */
enum apob_hole_type {
	APOB_HOLE_UMA = 0,
	APOB_HOLE_MMIO = 1,
	APOB_HOLE_PRIVILEGED_DRAM = 2,
};

struct zeppelin_apob_hole {
	uint64_t base;
	uint64_t size;
	uint32_t type;
};

/* The APOB that the PSP placed in DRAM */
static inline const void *zeppelin_apob(void)
{
	return (const void *)(uintptr_t)CONFIG_PSP_APOB_DRAM_ADDRESS;
}

bool zeppelin_apob_is_valid(const void *apob);
/* Memory clock in MHz, 0 if not reported */
uint16_t zeppelin_apob_memclk(const void *apob);
/* Returns the number of holes or -1 on error. *top is the top of system memory. */
int zeppelin_apob_memory_map(const void *apob, uint64_t *top,
			    struct zeppelin_apob_hole holes[APOB_MAX_HOLES]);
/*
 * Byte at offset (from the start of the entry, header included) of the first
 * entry of a group, type and instance (or APOB_ANY_INSTANCE). Returns false if
 * there is no such entry or the APOB is older than min_version.
 */
bool zeppelin_apob_read8(const void *apob, uint32_t group, uint32_t type, uint32_t instance,
			 size_t offset, uint32_t min_version, uint8_t *value);
/* Top of the DRAM below 4 GiB, 0 on error */
uint32_t zeppelin_apob_tolm(const void *apob);

#endif /* AMD_ZEPPELIN_NATIVE_APOB_H */
