/* SPDX-License-Identifier: GPL-2.0-only */

#include <commonlib/bsd/compression.h>
#include <string.h>
#include <tests/test.h>
#include <types.h>

/* One literal run, a match with a 16-bit offset, and a final literal. */
static size_t make_stream(uint8_t *stream, size_t offset, size_t length)
{
	size_t pos = 6;

	stream[0] = 0x7b;
	stream[1] = 0x9e;
	stream[2] = 0x20;
	stream[pos++] = 0xd8 | MIN(length - 2, 7);
	stream[pos++] = 0xf0 | (length < 9 ? 0 : MIN(length - 9, 15));
	stream[pos++] = 14; /* 3 + 15 + 14 = 32 literals. */
	for (size_t i = 0; i < 32; i++)
		stream[pos++] = i;
	stream[pos++] = (0x10000 - offset) >> 8;
	stream[pos++] = (0x10000 - offset) & 0xff;
	if (length >= 256) {
		stream[pos++] = 233; /* Extended 16-bit match length. */
		stream[pos++] = length & 0xff;
		stream[pos++] = length >> 8;
	} else if (length >= 24) {
		stream[pos++] = length - 24;
	}
	stream[pos++] = 0x08;
	stream[pos++] = '!';
	stream[3] = pos - 6;
	stream[4] = 0;
	stream[5] = 0;
	memset(stream + pos, 0, 3);
	return pos + 3;
}

static void test_lzsa2_matches(void **state)
{
	static const size_t offsets[] = { 1, 2, 7, 8, 15, 16, 17, 32 };
	static const size_t lengths[] = { 2, 7, 8, 9, 16, 17, 24, 33, 64, 257 };
	uint8_t input[64];
	uint8_t expected[300];
	uint8_t output[320];

	for (size_t o = 0; o < ARRAY_SIZE(offsets); o++) {
		for (size_t l = 0; l < ARRAY_SIZE(lengths); l++) {
			size_t offset = offsets[o];
			size_t length = lengths[l];
			size_t size = 32 + length + 1;
			size_t in_size = make_stream(input + 1, offset, length);

			for (size_t i = 0; i < 32; i++)
				expected[i] = i;
			for (size_t i = 32; i < size - 1; i++)
				expected[i] = expected[i - offset];
			expected[size - 1] = '!';

			/* Exercise unaligned buffers, with and without over-copy headroom. */
			for (size_t extra = 0; extra <= 8; extra += 8) {
				memset(output, 0xa5, sizeof(output));
				assert_int_equal(ulzsa2n(input + 1, in_size, output + 1,
							size + extra), size);
				assert_memory_equal(output + 1, expected, size);
				assert_int_equal(output[0], 0xa5);
				assert_int_equal(output[1 + size + extra], 0xa5);
			}
			assert_int_equal(ulzsa2n(input + 1, in_size, output + 1, size - 1), 0);
		}
	}
}

static void test_lzsa2_truncated_offset(void **state)
{
	uint8_t input[64];
	uint8_t output[64];

	make_stream(input, 16, 8);
	/* End the frame after only the first byte of the two-byte offset. */
	input[3] = 36;
	memset(input + 42, 0, 3);
	assert_int_equal(ulzsa2n(input, 45, output, sizeof(output)), 0);
}

int main(void)
{
	const struct CMUnitTest tests[] = {
		cmocka_unit_test(test_lzsa2_matches),
		cmocka_unit_test(test_lzsa2_truncated_offset),
	};

	return cb_run_group_tests(tests, NULL, NULL);
}
