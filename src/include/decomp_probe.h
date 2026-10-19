/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef DECOMP_PROBE_H
#define DECOMP_PROBE_H

#include <stdbool.h>
#include <stddef.h>

/*
 * Experiment instrumentation: time how long it takes to load a compressed
 * ramstage or payload segment, split into reading the compressed bytes and
 * decoding them when DECOMP_PROBE_STAGE_IN_RAM is set.
 */
#if CONFIG(DECOMP_PROBE)
void decomp_probe_start(void);
/* The compressed input has been copied to RAM by the caller (LZ4 in place). */
void decomp_probe_input_in_ram(void);
/* Returns the pointer to decode from: a RAM copy with DECOMP_PROBE_STAGE_IN_RAM. */
const void *decomp_probe_input(const void *src, size_t n);
void decomp_probe_report(const char *what, unsigned int algo, size_t in, size_t out);
#else
static inline void decomp_probe_start(void) {}
static inline void decomp_probe_input_in_ram(void) {}
static inline const void *decomp_probe_input(const void *src, size_t n) { return src; }
static inline void decomp_probe_report(const char *what, unsigned int algo, size_t in,
				       size_t out) {}
#endif

#endif
