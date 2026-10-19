/* SPDX-License-Identifier: GPL-2.0-only */
/* FHL encoder for cbfstool: the vendored codec with its own CLI's defaults. */

#define FHL_LIB
#include "fhl/fhl.c"

size_t fhl_compress(const uint8_t *in, uint32_t n, uint8_t *out);

/* out needs n * 2 + 65536 bytes. */
size_t fhl_compress(const uint8_t *in, uint32_t n, uint8_t *out)
{
	const struct copts o = { .K = 0, .passes = 1, .depth = 256, .threads = 5, .tree = 1 };
	return compress(in, n, out, &o);
}
