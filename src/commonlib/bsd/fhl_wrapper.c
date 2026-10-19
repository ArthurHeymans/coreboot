/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only */

#include <commonlib/bsd/compression.h>
#include <stdint.h>

#if CONFIG(DECOMP_INPUT_PREFETCH)
/* -mno-sse hides __builtin_prefetch; the LZMA decoder uses the same asm. */
#define FHL_PREFETCH(p) asm volatile("prefetchnta %0" :: "m" (*(const uint8_t *)(p)))
#endif

#define FHL_DECODER_ONLY
#include "fhl/fhl.c"

size_t ufhln(const void *src, size_t srcn, void *dst, size_t dstn)
{
	static uint64_t work[FHL_WORK_SIZE / sizeof(uint64_t)];

	return fhl_decode(src, srcn, dst, dstn, work);
}
