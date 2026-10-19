/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only */

#ifndef _COMMONLIB_COMPRESSION_H_
#define _COMMONLIB_COMPRESSION_H_

#include <stddef.h>

/* Decompresses an LZ4F image (multiple LZ4 blocks with frame header) from src
 * to dst, ensuring that it doesn't read more than srcn bytes and doesn't write
 * more than dstn. Buffer sizes must stay below 2GB. Can decompress files loaded
 * to the end of a buffer in-place, as long as buffer is larger than the final
 * output size. (Usually just a few bytes, but may be up to (8 + dstn/255) in
 * worst case. Will reliably return an error if buffer was too small.)
 * Returns amount of decompressed bytes, or 0 on error.
 */
size_t ulz4fn(const void *src, size_t srcn, void *dst, size_t dstn);

/* Same as ulz4fn() but does not perform any bounds checks. */
size_t ulz4f(const void *src, void *dst);

/* Decompresses ZSTD image */
size_t uzstdn(const void *src, size_t srcn, void *dst, size_t dstn);

/* Decompresses an LZSA2 stream image (multiple frames with a header/footer)
 * from src to dst, ensuring that it doesn't read more than srcn bytes and
 * doesn't write more than dstn. Back references may reach up to 64 KiB into
 * already decompressed output, so dst must be a single contiguous buffer.
 * Returns amount of decompressed bytes, or 0 on error.
 */
size_t ulzsa2n(const void *src, size_t srcn, void *dst, size_t dstn);

/* Decompress FHL and ANZ streams. The decoders trust the stream beyond its
   header. FHL stages literals in the tail of dst, so neither decompresses in
   place. Return the decompressed size, or 0 on error. */
size_t ufhln(const void *src, size_t srcn, void *dst, size_t dstn);
size_t uanzn(const void *src, size_t srcn, void *dst, size_t dstn);

#endif	/* _COMMONLIB_COMPRESSION_H_ */
