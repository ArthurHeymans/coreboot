/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_ZEPPELIN_NUMA_H
#define AMD_ZEPPELIN_NUMA_H

#include <types.h>

struct zeppelin_dram_range {
	uint64_t base;
	uint64_t size;
	unsigned int domain;
};

/* APIC IDs are below 0x80 (2 sockets of 4 dies); the CRAT sibling maps use 256 bits */
#define ZEPPELIN_APIC_ID_MAP_SIZE	32

unsigned int zeppelin_numa_domain_count(void);
unsigned int zeppelin_numa_domain_of_apic(unsigned int apic_id);
size_t zeppelin_dram_ranges(struct zeppelin_dram_range *ranges, size_t max);
unsigned int zeppelin_cpu_apic_ids(uint8_t map[ZEPPELIN_APIC_ID_MAP_SIZE]);

static inline bool zeppelin_apic_present(const uint8_t *map, unsigned int id)
{
	return map[id / 8] & (1 << (id % 8));
}

#endif /* AMD_ZEPPELIN_NUMA_H */
