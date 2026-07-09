/* SPDX-License-Identifier: GPL-2.0-only */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "../include/tests/test.h"

#include "../../src/drivers/amd/atombios/dcn10.h"

#define DCN_REG(reg) (((0x34c0 + (reg)) * 4))
#define DCHUBP_CNTL 0x0566
#define DCSURF_ADDRESS 0x057d
#define DCSURF_ADDRESS_HIGH 0x057e
#define VM_TLB_CONTROL 0x05b5
#define DCSURF_EARLIEST 0x0597
#define VBLANK_PARAMETERS_0 0x05bc
#define SEC_VIEWPORT_DIMENSION 0x0561
#define LB_DATA_FORMAT 0x0c7d
#define LB_MEMORY_CONTROL 0x0c7e
#define BNS_VALUES_R 0x0cbe
#define HDR_MULT_COEF 0x0d2b
#define SCL_MODE 0x0c64
#define DCSURF_EARLIEST_HIGH 0x0598
#define FB_BASE 0x0493
#define FB_TOP 0x0494
#define MPCC0_STATUS 0x1634
#define INPUT_CLOCK 0x1acd
#define OTG_CONTROL 0x1b41
#define OTG_FRAME_COUNT 0x1b4c
#define OTG_CLOCK 0x1b88
#define OTG_LOCK 0x1b8d
#define OTG_BLANK 0x1b42
#define HUBP_COMMIT_DISABLE_MASK ((1U << 0) | (1U << 2) | (1U << 12))

struct mmio_write {
	uint32_t reg;
	uint32_t value;
};

#define MMIO_TRACE_CAPACITY 256

static uint8_t *fake_mmio;
static struct mmio_write mmio_trace[MMIO_TRACE_CAPACITY];
static size_t mmio_trace_count;

void dcn10_test_mmio_write(uint32_t reg, uint32_t value)
{
	assert_true(mmio_trace_count < MMIO_TRACE_CAPACITY);
	mmio_trace[mmio_trace_count++] = (struct mmio_write){ reg, value };
}

static size_t find_write(size_t start, uint32_t reg, uint32_t set, uint32_t clear)
{
	size_t i;

	for (i = start; i < mmio_trace_count; i++) {
		if (mmio_trace[i].reg == reg &&
		    (mmio_trace[i].value & set) == set &&
		    !(mmio_trace[i].value & clear))
			return i;
	}
	return mmio_trace_count;
}

static void assert_commit_write_order(void)
{
	size_t high, low, hubp, unlock, unblank;

	high = find_write(0, DCSURF_ADDRESS_HIGH, 0xf4, 0);
	low = find_write(high + 1, DCSURF_ADDRESS, 0, ~0U);
	hubp = find_write(low + 1, DCHUBP_CNTL, 0,
			  HUBP_COMMIT_DISABLE_MASK);
	unlock = find_write(hubp + 1, OTG_LOCK, 0, 1);
	unblank = find_write(unlock + 1, OTG_BLANK, 0, 0x00010100);
	assert_true(high < low);
	assert_true(low < hubp);
	assert_true(hubp < unlock);
	assert_true(unlock < unblank);
	assert_true(unblank < mmio_trace_count);
}

static uint32_t *fake_reg(uint32_t reg)
{
	return (uint32_t *)(fake_mmio + DCN_REG(reg));
}

void udelay(unsigned int usecs)
{
	uint32_t hubp;

	(void)usecs;
	if (!fake_mmio)
		return;
	if ((*fake_reg(INPUT_CLOCK) & 3) == 3)
		*fake_reg(INPUT_CLOCK) |= 1U << 2;
	if ((*fake_reg(OTG_CLOCK) & 3) == 3)
		*fake_reg(OTG_CLOCK) |= 1U << 8;
	if (*fake_reg(OTG_CONTROL) & 1) {
		*fake_reg(OTG_CONTROL) |= 1U << 16;
		(*fake_reg(OTG_FRAME_COUNT))++;
	}
	if (*fake_reg(OTG_LOCK) & 1)
		*fake_reg(OTG_LOCK) |= 1U << 8;
	else
		*fake_reg(OTG_LOCK) &= ~(1U << 8);
	hubp = *fake_reg(DCHUBP_CNTL) | (1U << 1);
	if (hubp & 1) {
		hubp |= 1U << 3;
	} else {
		hubp &= ~(1U << 3);
		*fake_reg(DCSURF_EARLIEST) = *fake_reg(DCSURF_ADDRESS);
		*fake_reg(DCSURF_EARLIEST_HIGH) = *fake_reg(DCSURF_ADDRESS_HIGH);
	}
	*fake_reg(DCHUBP_CNTL) = hubp;
}

void mdelay(unsigned int msecs)
{
	while (msecs--)
		udelay(1000);
}

static const struct dcn10_timing vilboz_mode = {
	.h_active = 1366,
	.h_blank = 160,
	.h_front_porch = 48,
	.h_sync_width = 32,
	.v_active = 768,
	.v_blank = 25,
	.v_front_porch = 3,
	.v_sync_width = 5,
	.pixel_clock_khz = 72700,
	.ref_clock_khz = 48000,
	.disp_clock_khz = 600000,
};

static void test_vilboz_plane_calculation(void **state)
{
	struct dcn10_plane_regs regs;

	(void)state;
	assert_int_equal(dcn10_calculate_plane(&vilboz_mode, 1408, &regs), 0);
	assert_int_equal(regs.expansion_mode, 0x56);
	assert_int_equal(regs.request_size, 0x055d1b70);
	assert_int_equal(regs.blank_offset_0, 0x00160049);
	assert_int_equal(regs.blank_offset_1, 0x00000c59);
	assert_int_equal(regs.dst_dimensions, 0x0003ef89);
	assert_int_equal(regs.ref_freq_to_pix_freq, 0x0005482f);
	assert_int_equal(regs.prefetch_settings, 0x2d080000);
	assert_int_equal(regs.vblank_parameters_0, 0x301);
	assert_int_equal(regs.per_line_delivery, 0x385);
	assert_int_equal(regs.surf0_ttu_cntl0, 0x0800a3d1);
	assert_int_equal(regs.global_ttu_cntl, 0xe0000150);
	assert_int_equal(regs.vstartup, 13);
	assert_int_equal(regs.vupdate, 0x0082017e);
	assert_int_equal(regs.vready, 184);
	assert_int_equal(regs.watermark_urgent, 192);
	assert_int_equal(regs.watermark_sr_enter, 912);
}

static void test_rejects_unsupported_plane(void **state)
{
	struct dcn10_plane_regs regs;
	struct dcn10_timing invalid = vilboz_mode;

	(void)state;
	assert_int_equal(dcn10_calculate_plane(&vilboz_mode, 1366, &regs), -1);
	invalid.h_sync_width = 2;
	assert_int_equal(dcn10_calculate_plane(&invalid, 1408, &regs), -1);
	invalid = vilboz_mode;
	invalid.h_front_porch = invalid.h_blank;
	assert_int_equal(dcn10_calculate_plane(&invalid, 1408, &regs), -1);
	invalid = vilboz_mode;
	invalid.pixel_clock_khz = 1000000;
	assert_int_equal(dcn10_calculate_plane(&invalid, 1408, &regs), -1);
}

static void test_local_framebuffer_bounds(void **state)
{
	(void)state;
	assert_true(dcn10_surface_in_local_fb(0x000000f400000000ULL,
					       4325376, 0xf400, 0xf4ff));
	assert_false(dcn10_surface_in_local_fb(0xe0000000, 4325376,
						0xf400, 0xf4ff));
	assert_false(dcn10_surface_in_local_fb(0x000000f4fff00000ULL,
						4325376, 0xf400, 0xf4ff));
}

static void test_native_sequence_and_fail_closed(void **state)
{
	uint8_t *framebuffer;
	uint32_t pitch;

	(void)state;
	fake_mmio = calloc(1, 0x20000);
	framebuffer = malloc(1408 * 4 * 768);
	assert_non_null(fake_mmio);
	assert_non_null(framebuffer);
	*fake_reg(FB_BASE) = 0xf400;
	*fake_reg(FB_TOP) = 0xf4ff;
	*fake_reg(MPCC0_STATUS) = 1;

	assert_int_equal(dcn10_cold_init(fake_mmio, 4), 0);
	assert_int_equal(dcn10_program_stream(fake_mmio, &vilboz_mode), 0);
	mmio_trace_count = 0;
	assert_int_equal(dcn10_program_scanout(fake_mmio, 0x000000f400000000ULL,
						 framebuffer, 1408 * 4 * 768,
						 &vilboz_mode, &pitch), 0);
	assert_int_equal(pitch, 1408 * 4);
	assert_int_equal(*fake_reg(DCSURF_ADDRESS_HIGH), 0xf4);
	assert_int_equal(*fake_reg(DCSURF_ADDRESS), 0);
	assert_false(*fake_reg(DCHUBP_CNTL) & ((1U << 0) | (1U << 2) | (1U << 3)));
	assert_int_equal(*fake_reg(VBLANK_PARAMETERS_0), 0x301);
	assert_int_equal(*fake_reg(BNS_VALUES_R), 0x20000000);
	assert_int_equal(*fake_reg(HDR_MULT_COEF), 0x1f000);
	assert_int_equal(*fake_reg(LB_DATA_FORMAT), 0x1003);
	assert_int_equal(*fake_reg(LB_MEMORY_CONTROL), 0x3f01);
	assert_int_equal(*fake_reg(SCL_MODE) & 7, 0);
	assert_int_equal(*fake_reg(SEC_VIEWPORT_DIMENSION),
			 ((uint32_t)vilboz_mode.v_active << 16) |
			 vilboz_mode.h_active);
	assert_true(find_write(0, VM_TLB_CONTROL, 0, ~0U) < mmio_trace_count);
	assert_commit_write_order();

	/* An address outside the ASIC_Init local aperture fails before MMIO commit. */
	assert_int_equal(dcn10_program_scanout(fake_mmio, 0x000000e000000000ULL,
						 framebuffer, 1408 * 4 * 768,
						 &vilboz_mode, &pitch), -1);
	free(framebuffer);
	free(fake_mmio);
	fake_mmio = NULL;
}

int main(void)
{
	const struct CMUnitTest tests[] = {
		cmocka_unit_test(test_vilboz_plane_calculation),
		cmocka_unit_test(test_rejects_unsupported_plane),
		cmocka_unit_test(test_local_framebuffer_bounds),
		cmocka_unit_test(test_native_sequence_and_fail_closed),
	};

	return cmocka_run_group_tests_name("dcn10-test", tests, NULL, NULL);
}
