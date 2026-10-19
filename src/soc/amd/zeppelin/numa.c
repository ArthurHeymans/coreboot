/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/data_fabric.h>
#include <commonlib/helpers.h>
#include <device/device.h>
#include <soc/data_fabric.h>
#include <soc/numa.h>
#include <string.h>
#include <types.h>

#include "native/topology.h"

/*
 * NUMA domains as the vendor AmdFabricZpDxe reports them: one per die, unless
 * a DRAM map pair interleaves dies (DramLimit[11:10]); the interleaved dies
 * then share one domain. With at most two dies that is all of them.
 */
static bool dies_interleaved(void)
{
	for (unsigned int i = 0; i < ZEPPELIN_NUM_DRAM_REG; i++) {
		const uint32_t base = data_fabric_read32(DF_DRAM_BASE(i), IOMS0_FABRIC_ID);

		if (!(base & DRAM_BASE_REG_VALID))
			continue;
		if (data_fabric_read32(DF_DRAM_LIMIT(i), IOMS0_FABRIC_ID) &
		    DRAM_LIMIT_INTLV_NUM_DIE)
			return true;
	}
	return false;
}

unsigned int zeppelin_numa_domain_count(void)
{
	return dies_interleaved() ? 1 : zeppelin_die_count();
}

static unsigned int domain_of_die(unsigned int die)
{
	return dies_interleaved() ? 0 : die;
}

/* APIC ID: socket << 6 | die << 4 | CCX << 3 | core * threads + thread */
unsigned int zeppelin_numa_domain_of_apic(unsigned int apic_id)
{
	return domain_of_die((apic_id >> 4) & 3);
}

static size_t add_range(struct zeppelin_dram_range *r, size_t n, size_t max, uint64_t base,
			uint64_t size, unsigned int domain)
{
	if (n < max && size)
		r[n++] = (struct zeppelin_dram_range){ base, size, domain };
	return n;
}

/*
 * The DRAM ranges of the DF DRAM map (256 MiB granularity), split like the
 * vendor SRAT: [0, 640 KiB), then from 1 MiB, then up to the DRAM hole and the
 * rest from 4 GiB. The destination die is DramLimit[6:5].
 */
size_t zeppelin_dram_ranges(struct zeppelin_dram_range *ranges, size_t max)
{
	size_t n = 0;

	for (unsigned int i = 0; i < ZEPPELIN_NUM_DRAM_REG; i++) {
		const uint32_t base_reg = data_fabric_read32(DF_DRAM_BASE(i), IOMS0_FABRIC_ID);
		uint32_t limit_reg;
		unsigned int domain;
		uint64_t base, size;

		if (!(base_reg & DRAM_BASE_REG_VALID))
			continue;
		limit_reg = data_fabric_read32(DF_DRAM_LIMIT(i), IOMS0_FABRIC_ID);
		domain = domain_of_die((limit_reg >> 5) & 3);
		base = (uint64_t)(base_reg >> DRAM_BASE_ADDR_SHFT) << 28;
		size = ((uint64_t)(limit_reg >> DRAM_LIMIT_ADDR_SHFT) + 1 -
			(base_reg >> DRAM_BASE_ADDR_SHFT)) << 28;

		if (!base) {
			n = add_range(ranges, n, max, 0, 0xa0000, domain);
			base = 1 * MiB;
			size -= 1 * MiB;
		}
		if (base_reg & DRAM_BASE_HOLE_EN) {
			const uint64_t hole =
				data_fabric_read32(DF_DRAM_HOLE_CTL, IOMS0_FABRIC_ID) &
				DRAM_HOLE_CTL_BASE;

			n = add_range(ranges, n, max, base, hole - base, domain);
			base = 4ULL * GiB;
			size = ((uint64_t)(limit_reg >> DRAM_LIMIT_ADDR_SHFT) + 1 - 0x10) << 28;
		}
		n = add_range(ranges, n, max, base, size, domain);
	}

	return n;
}

/* Fills a bitmap of the APIC IDs of the enabled CPUs; returns the CPU count */
unsigned int zeppelin_cpu_apic_ids(uint8_t map[ZEPPELIN_APIC_ID_MAP_SIZE])
{
	unsigned int count = 0;

	memset(map, 0, ZEPPELIN_APIC_ID_MAP_SIZE);
	for (struct device *cpu = all_devices; cpu; cpu = cpu->next) {
		const unsigned int id = cpu->path.apic.apic_id;

		if (cpu->path.type != DEVICE_PATH_APIC || !cpu->enabled ||
		    id >= 8 * ZEPPELIN_APIC_ID_MAP_SIZE)
			continue;
		map[id / 8] |= BIT(id % 8);
		count++;
	}
	return count;
}
