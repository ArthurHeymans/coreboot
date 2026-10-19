/* SPDX-License-Identifier: GPL-2.0-only */

/*
 * cbfstool interface to the LZSA2 encoder.
 *
 * The encoder implementation in this directory is the LZSA reference
 * implementation by Emmanuel Marty, used with the "prefer ratio" parse to
 * produce the standard LZSA2 stream format. The matching decompressor lives in
 * src/commonlib/bsd/lzsa2.c and is shared with the firmware build.
 */

#include "../common.h"

#include "lib.h"
#include "shrink_inmem.h"

/**
 * Compress a buffer with LZSA2.
 *
 * @param in a pointer to the buffer to compress
 * @param in_len the length in bytes
 * @param out a pointer to a buffer of at least in_len bytes
 * @param out_len a pointer to the compressed length of in
 *
 * Returns 0 on success, -1 if the input cannot be compressed into in_len bytes
 * (in which case the caller falls back to an uncompressed file).
 */
int do_lzsa2_compress(char *in, int in_len, char *out, int *out_len)
{
	size_t written;

	if (in_len <= 0) {
		ERROR("LZSA2: Input length is zero.\n");
		return -1;
	}

	/* cbfstool gives us exactly in_len bytes of scratch, so a stream that
	 * would not shrink the file has to be reported as a failure. A min match
	 * size of 0 selects the encoder's default (2, what the reference lzsa
	 * tool uses for its ratio-focused mode). */
	written = lzsa_compress_inmem((unsigned char *)in, (unsigned char *)out,
				      (size_t)in_len, (size_t)in_len,
				      LZSA_FLAG_FAVOR_RATIO, 0, 2);
	if (written == (size_t)-1) {
		ERROR("LZSA2: compression failed.\n");
		return -1;
	}

	*out_len = (int)written;
	return 0;
}
