/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/apob_cache.h>
#include <commonlib/helpers.h>
#include <string.h>
#include <types.h>

#include "apob.h"

#define APOB_GROUP_MEMORY	1
#define APOB_GROUP_FABRIC	9

#define APOB_TYPE_MEMORY_CONFIG	2
#define APOB_TYPE_SYSTEM_MAP	9

struct apob_entry_header {
	uint32_t group;
	uint32_t type;
	uint32_t instance;
	uint32_t size;		/* including this header */
	uint8_t hmac[32];
} __packed;

/* APOB_GROUP_MEMORY, APOB_TYPE_MEMORY_CONFIG */
struct apob_memory_config {
	uint32_t boot_media_type;
	uint16_t memclk_mhz;
} __packed;

/* APOB_GROUP_FABRIC, APOB_TYPE_SYSTEM_MAP */
struct apob_system_map {
	uint64_t top_of_system_memory;
	uint32_t hole_count;
	uint32_t reserved;
	struct {
		uint64_t base;
		uint64_t size;
		uint32_t type;
		uint32_t reserved;
	} __packed holes[];
} __packed;

bool picasso_apob_is_valid(const void *apob)
{
	const struct apob_base_header *header = apob;

	return header && header->signature == APOB_SIGNATURE &&
		header->size >= sizeof(*header) &&
		header->size <= CONFIG_PSP_APOB_DRAM_SIZE &&
		header->offset_of_first_entry >= sizeof(*header) &&
		header->offset_of_first_entry <= header->size &&
		header->size - header->offset_of_first_entry >=
			sizeof(struct apob_entry_header);
}

/* Returns the payload of the first entry of the given group and type */
static const void *apob_find(const void *apob, uint32_t group, uint32_t type, size_t *size)
{
	const struct apob_base_header *header = apob;
	const uint8_t *current;
	const uint8_t *end;

	if (!picasso_apob_is_valid(apob))
		return NULL;

	current = (const uint8_t *)apob + header->offset_of_first_entry;
	end = (const uint8_t *)apob + header->size;
	while ((size_t)(end - current) >= sizeof(struct apob_entry_header)) {
		const struct apob_entry_header *entry = (const void *)current;

		if (entry->size < sizeof(*entry) || entry->size > (size_t)(end - current))
			return NULL;
		if (entry->group == group && entry->type == type) {
			*size = entry->size - sizeof(*entry);
			return current + sizeof(*entry);
		}
		current += entry->size;
	}

	return NULL;
}

uint16_t picasso_apob_memclk(const void *apob)
{
	const struct apob_memory_config *config;
	size_t size;

	config = apob_find(apob, APOB_GROUP_MEMORY, APOB_TYPE_MEMORY_CONFIG, &size);
	if (!config || size < sizeof(*config))
		return 0;

	return config->memclk_mhz;
}

int picasso_apob_memory_map(const void *apob, uint64_t *top,
			    struct picasso_apob_hole holes[APOB_MAX_HOLES])
{
	const struct apob_system_map *map;
	size_t size;

	map = apob_find(apob, APOB_GROUP_FABRIC, APOB_TYPE_SYSTEM_MAP, &size);
	if (!map || size < sizeof(*map) || map->hole_count > APOB_MAX_HOLES ||
	    map->hole_count > (size - sizeof(*map)) / sizeof(map->holes[0]))
		return -1;

	*top = map->top_of_system_memory;
	for (size_t i = 0; i < map->hole_count; i++) {
		holes[i] = (struct picasso_apob_hole) {
			.base = map->holes[i].base,
			.size = map->holes[i].size,
			.type = map->holes[i].type,
		};
		if (holes[i].base > UINT64_MAX - holes[i].size)
			return -1;
	}

	return map->hole_count;
}

uint32_t picasso_apob_tolm(const void *apob)
{
	struct picasso_apob_hole holes[APOB_MAX_HOLES];
	uint64_t tolm = 4ULL * GiB;
	uint64_t top;
	int count;

	count = picasso_apob_memory_map(apob, &top, holes);
	if (count < 0)
		return 0;
	tolm = MIN(tolm, top);

	/* The lowest hole below 4 GiB that ends above 2 GiB starts the MMIO/UMA region */
	for (int i = 0; i < count; i++) {
		const uint64_t end = holes[i].base + holes[i].size;

		if (end <= tolm && end > 2ULL * GiB)
			tolm = MIN(tolm, holes[i].base);
	}

	return tolm;
}
