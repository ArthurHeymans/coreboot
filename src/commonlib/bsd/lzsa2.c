/* SPDX-License-Identifier: MIT */

/*
 * LZSA2 decompressor for coreboot.
 *
 * Decompresses an LZSA2 "stream" image (3-byte header, then a sequence of
 * frames, each holding up to 64 KiB of decompressed data, terminated by a
 * 3-byte zero footer) as produced by cbfstool's LZSA2 encoder.
 *
 * The decoder is deliberately small and stateless: it needs no scratch memory
 * beyond the destination buffer, which makes it usable for pre-RAM stages.
 * Back references may reach up to 64 KiB into already decompressed output, so
 * the whole destination buffer must be contiguous.
 */

#include <commonlib/bsd/compiler.h>
#include <commonlib/bsd/compression.h>
#include <commonlib/bsd/sysincludes.h>
#include <stdint.h>
#include <string.h>

#define LZSA2_BLOCK_SIZE	65536
#define LZSA2_FRAME_MASK	0x7fffff
#define LZSA2_FRAME_UNCOMPRESSED 0x800000

/* One nibble of scratch state: the nibble byte and whether its low half is
 * still pending. */
struct nibble_state {
	uint8_t byte;
	int pending;
};

/* Returns the next nibble, or -1 if the input was exhausted. */
static __always_inline int lzsa2_nibble(const uint8_t **in, const uint8_t *in_end,
					struct nibble_state *ns)
{
	if (ns->pending == 0) {
		if (*in >= in_end)
			return -1;
		ns->byte = *(*in)++;
		ns->pending = 1;
		return ns->byte >> 4;
	}

	ns->pending = 0;
	return ns->byte & 0x0f;
}

/* Decodes an extended literal or match length, adding it to *len. A value of
 * 0 is the raw-block end-of-data marker and never appears in a stream. */
static __always_inline int lzsa2_length(const uint8_t **in, const uint8_t *in_end,
					struct nibble_state *ns, uint32_t *len)
{
	int nib = lzsa2_nibble(in, in_end, ns);

	if (nib < 0)
		return -1;
	*len += nib;
	if (nib != 15)
		return 0;

	if (*in >= in_end)
		return -1;
	*len += *(*in)++;

	if (*len == 257) {
		if (*in + 2 > in_end)
			return -1;
		uint16_t value;
		__builtin_memcpy(&value, *in, sizeof(value));
		*len = le16toh(value);
		*in += 2;
	} else if (*len == 256) {
		*len = 0;
	}

	return 0;
}

static __always_inline void lzsa2_copy8(void *dst, const void *src)
{
	uint64_t value;

	/* Load before storing: the eight-byte ranges may overlap for short matches. */
	__builtin_memcpy(&value, src, sizeof(value));
	__builtin_memcpy(dst, &value, sizeof(value));
}

/* Decodes a single LZSA2 block from *in up to in_end into *out.
 * Returns 0 on success, -1 on malformed input. */
static int lzsa2_block(const uint8_t **in, const uint8_t *in_end,
		       uint8_t **out, uint8_t *out_end, uint8_t *out_start)
{
	struct nibble_state ns = { 0 };
	uint32_t offset = 0;

	while (*in < in_end) {
		uint8_t token = *(*in)++;
		uint32_t literals = (token >> 3) & 3;

		if (literals != 3) {
			/* 0..2 literals: over-copy 4 bytes into the headroom, the
			 * extra bytes are overwritten by later commands. */
			if (*in + 4 <= in_end && *out + 4 <= out_end) {
				uint32_t value;
				__builtin_memcpy(&value, *in, sizeof(value));
				__builtin_memcpy(*out, &value, sizeof(value));
			} else if (literals) {
				if (*in + literals > in_end || *out + literals > out_end)
					return -1;
				memcpy(*out, *in, literals);
			}
		} else {
			if (lzsa2_length(in, in_end, &ns, &literals) < 0)
				return -1;
			if (*in + literals > in_end || *out + literals > out_end)
				return -1;
			memcpy(*out, *in, literals);
		}
		*in += literals;
		*out += literals;

		if (*in >= in_end)	/* last command has literals only */
			break;

		switch (token & 0xc0) {
		case 0x00: {
			int v = lzsa2_nibble(in, in_end, &ns);
			if (v < 0)
				return -1;
			offset = ((((uint32_t)v << 1) | ((token >> 5) & 1)) ^ 0x1e) + 1;
			break;
		}
		case 0x40:
			if (*in >= in_end)
				return -1;
			offset = (((uint32_t)*(*in)++ | ((uint32_t)((token >> 5) & 1) << 8)) ^ 0x0ff) + 1;
			break;
		case 0x80: {
			int v = lzsa2_nibble(in, in_end, &ns);
			if (v < 0 || *in >= in_end)
				return -1;
			offset = ((uint32_t)*(*in)++ | ((uint32_t)v << 9) |
				  ((uint32_t)((token >> 5) & 1) << 8)) ^ 0x1eff;
			offset += 513;
			break;
		}
		default:
			if ((token & 0x20) == 0) {
				if (*in + 2 > in_end)
					return -1;
				uint16_t value;
				__builtin_memcpy(&value, *in, sizeof(value));
				offset = (be16toh(value) ^ 0xffff) + 1;
				*in += 2;
			} /* else: repeat previous offset */
			break;
		}

		uint32_t length = token & 0x07;
		if (length == 7) {
			length = 9;
			if (lzsa2_length(in, in_end, &ns, &length) < 0)
				return -1;
			if (length == 0)
				return -1;
		} else {
			length += 2;
		}

		if (offset == 0 || offset > (uint32_t)(*out - out_start))
			return -1;
		if (*out + length > out_end)
			return -1;

		const uint8_t *src = *out - offset;
		uint8_t *dst = *out;

		if (offset >= length && length <= 8 && out_end - dst >= 8) {
			lzsa2_copy8(dst, src);
		} else {
			size_t done = 0;

			/* Each chunk must finish before a subsequent overlapping chunk reads it. */
			if (offset >= 16) {
				while (done + 16 <= length) {
					lzsa2_copy8(dst + done, src + done);
					lzsa2_copy8(dst + done + 8, src + done + 8);
					done += 16;
				}
			}
			while (done < length) {
				dst[done] = src[done];
				done++;
			}
		}
		*out += length;
	}

	return 0;
}

size_t ulzsa2n(const void *src, size_t srcn, void *dst, size_t dstn)
{
	const uint8_t *in = src;
	const uint8_t *in_end = in + srcn;
	uint8_t *out = dst;
	uint8_t *out_start = dst;
	uint8_t *out_end = out + dstn;

	if (srcn < 4 || in[0] != 0x7b || in[1] != 0x9e || (in[2] & 0xe0) != 0x20)
		return 0;
	in += 3;

	while (in + 3 <= in_end) {
		uint32_t frame = (uint32_t)in[0] | ((uint32_t)in[1] << 8) |
				 ((uint32_t)in[2] << 16);
		uint32_t size = frame & LZSA2_FRAME_MASK;

		in += 3;
		if (size == 0)	/* footer */
			break;
		if (in + size > in_end)
			return 0;

		if (frame & LZSA2_FRAME_UNCOMPRESSED) {
			if (out + size > out_end)
				return 0;
			memcpy(out, in, size);
			out += size;
			in += size;
		} else {
			if (lzsa2_block(&in, in + size, &out, out_end, out_start) < 0)
				return 0;
		}
	}

	return (size_t)(out - out_start);
}
