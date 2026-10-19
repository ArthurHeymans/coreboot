/* SPDX-License-Identifier: MIT */
#ifndef ANZ_H
#define ANZ_H

#include <stddef.h>
#include <stdint.h>

/* Bytes of 16-byte aligned work memory anz_decode() needs for its models. */
#define ANZ_WORK_SIZE (83 * 1024)

/*
 * Decompress the ANZ stream src[0..n) into dst[0..cap). Returns the
 * decompressed size, or 0 if the header is invalid or the output does not
 * fit. The decoder trusts the stream beyond its header: feed it only streams
 * you produced or verified (e.g. by hash).
 */
size_t anz_decode(const void *src, size_t n, void *dst, size_t cap, void *work);

/* Compress in[0..n) into out (n * 2 + 4096 bytes) with up to 'threads'
 * threads; lclo 0..6 (6 is best on firmware). Returns the size. */
size_t anz_compress(const uint8_t *in, uint32_t n, unsigned lclo, unsigned threads, uint8_t *out);

/* Encoder knobs. Every parse setting costs a full FHL parse, so they are the
 * encoder's whole run time: 'settings' 1..3 is the number of parse settings
 * tried (3 by default; 2 is within 0.06% on firmware), 'passes' is the FHL
 * price-feedback passes per setting (4 by default, 1 costs 0.3%), and
 * 'depth' is the hash-chain depth of the shared match finder (256 by default;
 * 64 costs 0.6%). The match finder runs on up to 'threads' threads too,
 * unless 'tree' picks FHL's serial binary-tree match finder (faster on one
 * core, finds more); 'greedy' seeds the prices with a greedy parse instead
 * of a flat-priced optimal one. */
struct anz_opts {
	unsigned lclo, threads, passes, depth, settings;
	int tree, greedy;
};
size_t anz_compress_opts(const uint8_t *in, uint32_t n, const struct anz_opts *o, uint8_t *out);

#endif
