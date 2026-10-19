/* SPDX-License-Identifier: GPL-2.0-only */

#include <bootstate.h>
#include <cbmem.h>
#include <console/console.h>
#include <cpu/x86/mtrr.h>
#include <cpu/x86/tsc.h>
#include <decomp_probe.h>
#include <string.h>
#include <timestamp.h>

#define DECOMP_PROBE_SCRATCH_ID 0x50434544 /* 'DECP' */
#define DECOMP_PROBE_RECORDS_ID 0x52434544 /* 'DECR' */
#define MAX_RECORDS 8

/* Kept in CBMEM so ramstage can print postcar's results again late in boot. */
struct decomp_record {
	char what[24];
	uint32_t algo, in, out, split;
	uint64_t read_ticks, decode_ticks;
};

struct decomp_records {
	uint32_t count;
	struct decomp_record r[MAX_RECORDS];
};

static uint64_t t_start, t_read;
static bool in_ram;

void decomp_probe_start(void)
{
	in_ram = false;
	t_start = timestamp_get();
	t_read = t_start;
}

void decomp_probe_input_in_ram(void)
{
	in_ram = true;
	t_read = timestamp_get();
}

const void *decomp_probe_input(const void *src, size_t n)
{
	static void *scratch;

	if (!CONFIG(DECOMP_PROBE_STAGE_IN_RAM) || !ENV_HAS_CBMEM || in_ram)
		return src;
	if (!scratch)
		scratch = cbmem_add(DECOMP_PROBE_SCRATCH_ID, CONFIG_DECOMP_PROBE_SCRATCH_SIZE);
	if (!scratch || n > CONFIG_DECOMP_PROBE_SCRATCH_SIZE) {
		printk(BIOS_ERR, "DECOMP_PROBE: no scratch for %zu bytes\n", n);
		return src;
	}
	memcpy(scratch, src, n);
	t_read = timestamp_get();
	return scratch;
}

static void print_record(const struct decomp_record *r)
{
	printk(BIOS_NOTICE, "DECOMP_PROBE %s algo=%u in=%u out=%u mode=%s "
	       "read_ticks=%llu decode_ticks=%llu total_ticks=%llu\n",
	       r->what, r->algo, r->in, r->out, r->split ? "split" : "direct",
	       r->read_ticks, r->decode_ticks, r->read_ticks + r->decode_ticks);
}

void decomp_probe_report(const char *what, unsigned int algo, size_t in, size_t out)
{
	const uint64_t t_end = timestamp_get();
	struct decomp_record r = {
		.algo = algo, .in = in, .out = out,
		.split = CONFIG(DECOMP_PROBE_STAGE_IN_RAM),
		.read_ticks = t_read - t_start,
		.decode_ticks = t_end - t_read,
	};

	strncpy(r.what, what, sizeof(r.what) - 1);
	print_record(&r);

	if (!ENV_HAS_CBMEM)
		return;
	struct decomp_records *recs = cbmem_find(DECOMP_PROBE_RECORDS_ID);
	if (!recs) {
		recs = cbmem_add(DECOMP_PROBE_RECORDS_ID, sizeof(*recs));
		if (!recs)
			return;
		recs->count = 0;
	}
	if (recs->count < MAX_RECORDS)
		recs->r[recs->count++] = r;
}

#if ENV_RAMSTAGE
static void decomp_probe_dump(void *unused)
{
	const struct decomp_records *recs = cbmem_find(DECOMP_PROBE_RECORDS_ID);
	const uint64_t now = timestamp_get();

	printk(BIOS_NOTICE, "DECOMP_PROBE_SUMMARY begin tsc_mhz=%lu\n", tsc_freq_mhz());
	for (uint32_t i = 0; recs && i < recs->count; i++)
		print_record(&recs->r[i]);
	const struct timestamp_table *ts = cbmem_find(CBMEM_ID_TIMESTAMP);
	for (uint32_t i = 0; ts && i < ts->num_entries; i++)
		printk(BIOS_NOTICE, "DECOMP_PROBE_TS %u %lld\n", ts->entries[i].entry_id,
		       (long long)ts->entries[i].entry_stamp);
	if (ts)
		printk(BIOS_NOTICE, "DECOMP_PROBE_TS 0 %lld payload boot\n",
		       (long long)(now - ts->base_time));
	printk(BIOS_NOTICE, "DECOMP_PROBE_SUMMARY end\n");
}

BOOT_STATE_INIT_ENTRY(BS_PAYLOAD_BOOT, BS_ON_ENTRY, decomp_probe_dump, NULL);

#if CONFIG(DECOMP_PROBE_CACHE_ROM)
/* BSP only: the payload is loaded on the BSP. */
static int rom_mtrr = -1;

static void decomp_probe_cache_rom(void *unused)
{
	rom_mtrr = acquire_and_configure_mtrr(CACHE_ROM_BASE, CACHE_ROM_SIZE,
					      MTRR_TYPE_WRPROT);
	printk(BIOS_INFO, "DECOMP_PROBE: ROM cached WP with MTRR %d\n", rom_mtrr);
}

static void decomp_probe_uncache_rom(void *unused)
{
	if (rom_mtrr >= 0)
		clear_var_mtrr(rom_mtrr);
}

BOOT_STATE_INIT_ENTRY(BS_PAYLOAD_LOAD, BS_ON_ENTRY, decomp_probe_cache_rom, NULL);
BOOT_STATE_INIT_ENTRY(BS_PAYLOAD_BOOT, BS_ON_ENTRY, decomp_probe_uncache_rom, NULL);
#endif
#endif
