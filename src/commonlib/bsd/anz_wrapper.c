/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only */

#include <commonlib/bsd/compression.h>
#include <stdint.h>

/*
 * coreboot builds x86 code with -mno-sse. The SSE2 symbol decoder is 3x
 * faster than the scalar one; CR4.OSFXSR is set in the bootblock when the
 * CPU selects SSE.
 */
#if CONFIG(ANZ_SSE2)
#pragma GCC target("sse2")
/* Built -fhosted so memcpy() is a builtin; skip _mm_malloc, which needs libc. */
#define _MM_MALLOC_H_INCLUDED
#endif

#if CONFIG(DECOMP_INPUT_PREFETCH)
/* -mno-sse hides __builtin_prefetch; the LZMA decoder uses the same asm. */
#define ANZ_PREFETCH(p) asm volatile("prefetchnta %0" :: "m" (*(const uint8_t *)(p)))
#endif

/* Without POPCNT (e.g. Bonnell Atoms) GCC calls libgcc's table-driven version. */
static inline unsigned int anz_popcount(unsigned int x)
{
	x -= (x >> 1) & 0x55555555;
	x = (x & 0x33333333) + ((x >> 2) & 0x33333333);
	return (((x + (x >> 4)) & 0x0f0f0f0f) * 0x01010101) >> 24;
}
#define __builtin_popcount(x) anz_popcount(x)

#define ANZ_DECODER_ONLY
#include "anz/anz.c"

#if ENV_RAMSTAGE
#include <stdlib.h>
#endif

size_t uanzn(const void *src, size_t srcn, void *dst, size_t dstn)
{
#if ENV_RAMSTAGE
	/* From the heap: in BSS the models count against the TSEG stage cache. */
	static void *work;

	if (!work)
		work = memalign(16, ANZ_WORK_SIZE);
	if (!work)
		return 0;
#else
	static uint64_t work[ANZ_WORK_SIZE / sizeof(uint64_t)] __aligned(16);
#endif

	return anz_decode(src, srcn, dst, dstn, work);
}
