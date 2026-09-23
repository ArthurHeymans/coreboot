/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/apob_cache.h>
#include <cbmem.h>
#include <commonlib/helpers.h>
#include <cpu/amd/mtrr.h>
#include <device/device.h>
#include <soc/native.h>
#include <types.h>

#include "apob.h"

/* Report the DRAM above cbmem_top() using the APOB memory map */
void picasso_native_add_memmap(struct device *dev, unsigned long *idx)
{
	struct picasso_apob_hole holes[APOB_MAX_HOLES];
	const uintptr_t cbmem_base = cbmem_top();
	const uintptr_t low_top = get_top_of_mem_below_4gb();
	uint64_t cursor = 4ULL * GiB;
	uint64_t top;
	int count;

	/* TSEG and the UMA region */
	if (low_top > cbmem_base)
		reserved_ram_from_to(dev, (*idx)++, cbmem_base, low_top);

	count = picasso_apob_memory_map(soc_fill_apob_cache(), &top, holes);
	if (count < 0)
		die("Invalid APOB memory map\n");

	/* Sort the holes by address */
	for (int i = 1; i < count; i++) {
		const struct picasso_apob_hole hole = holes[i];
		int j = i;

		for (; j > 0 && holes[j - 1].base > hole.base; j--)
			holes[j] = holes[j - 1];
		holes[j] = hole;
	}

	for (int i = 0; i < count && cursor < top; i++) {
		const uint64_t start = MAX(holes[i].base, cursor);
		const uint64_t end = MIN(holes[i].base + holes[i].size, top);

		if (end <= start)
			continue;
		if (start > cursor)
			ram_from_to(dev, (*idx)++, cursor, start);
		reserved_ram_from_to(dev, (*idx)++, start, end);
		cursor = end;
	}

	if (cursor < top)
		ram_from_to(dev, (*idx)++, cursor, top);
}
