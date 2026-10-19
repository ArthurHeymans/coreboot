/* SPDX-License-Identifier: MIT */
#ifndef FHL_H
#define FHL_H

#include <stddef.h>
#include <stdint.h>

/* Bytes of 8-byte aligned work memory fhl_decode() needs for its tables. */
#define FHL_WORK_SIZE (45 * 1024)

/*
 * Decompress the FHL stream src[0..n) into dst[0..cap). Returns the
 * decompressed size, or 0 if the header is invalid or the output does not
 * fit. The whole output buffer is used during decoding (literals are staged
 * in its tail), so FHL cannot decompress in place.
 *
 * The decoder trusts the stream beyond its header: feed it only streams you
 * produced or verified (e.g. by hash), as with any unchecked LZ decoder.
 */
size_t fhl_decode(const void *src, size_t n, void *dst, size_t cap, void *work);

/* Encoder-side parser used by ANZ; see fhl.c. */
struct fhl_seq { uint32_t L, M, ds, d; };
struct fhl_mf;
struct fhl_mf *fhl_matches(const uint8_t *in, uint32_t n);
/* Same, with the search depth (fhl_matches uses 256) on up to 'threads'
 * threads (the same matches for any count), and flags: FHL_MF_TREE for the
 * serial binary-tree match finder rather than hash chains, FHL_MF_GREEDY to
 * seed fhl_parse()'s prices with a greedy parse rather than a flat-priced
 * optimal one. */
#define FHL_MF_TREE 1
#define FHL_MF_GREEDY 2
struct fhl_mf *fhl_matches_mt(const uint8_t *in, uint32_t n, unsigned depth, unsigned threads,
			      unsigned flags);
void fhl_matches_free(struct fhl_mf *m);
size_t fhl_parse(const uint8_t *in, uint32_t n, const struct fhl_mf *m, unsigned K, unsigned Kf,
		 struct fhl_seq *out);
/* Same, with the number of price-feedback passes (fhl_parse uses 4). */
size_t fhl_parse_passes(const uint8_t *in, uint32_t n, const struct fhl_mf *m, unsigned K,
			unsigned Kf, int passes, struct fhl_seq *out);

#endif
