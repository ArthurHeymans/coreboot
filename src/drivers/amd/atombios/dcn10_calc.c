/* SPDX-License-Identifier: GPL-2.0-only */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "dcn10.h"

#define DCN10_DEEP_SLEEP_DCFCLK_KHZ	8000
#define DCN10_URGENT_LATENCY_US		4
#define DCN10_PREFETCH_CALC_US		3

static uint32_t div_round_up_u64(uint64_t value, uint32_t divisor)
{
	return (value + divisor - 1) / divisor;
}

static uint32_t watermark_cycles(uint32_t ns, uint32_t ref_clock_khz)
{
	uint64_t cycles = (uint64_t)ns * ref_clock_khz / 1000000;

	return cycles > 0x1fffff ? 0x1fffff : cycles;
}

int dcn10_calculate_plane(const struct dcn10_timing *timing,
			  uint32_t pitch_pixels,
			  struct dcn10_plane_regs *regs)
{
	uint32_t h_total, v_total, h_blank_end, v_blank_start, v_blank_end;
	uint32_t vupdate_offset, vupdate_width, vready_dpp, vready_dcf;
	uint32_t vready, vstartup, delay_pixels, setup_pixels, wait_pixels;
	uint32_t prefetch_quarters, line_delivery, requests_per_line;
	uint64_t ratio, prefetch_num;

	if (!timing || !regs || !timing->h_active || !timing->v_active ||
	    timing->h_blank < 32 || timing->v_blank < 3 ||
	    timing->h_sync_width < 4 || !timing->v_sync_width ||
	    !timing->pixel_clock_khz || !timing->ref_clock_khz ||
	    !timing->disp_clock_khz || pitch_pixels < timing->h_active ||
	    (pitch_pixels & 63))
		return -1;

	h_total = timing->h_active + timing->h_blank;
	v_total = timing->v_active + timing->v_blank;
	if (h_total > 0x4000 || v_total > 0x4000 ||
	    timing->h_front_porch + timing->h_sync_width > timing->h_blank ||
	    timing->v_front_porch + timing->v_sync_width > timing->v_blank ||
	    (uint64_t)h_total * 1000 <
		(uint64_t)(DCN10_URGENT_LATENCY_US + DCN10_PREFETCH_CALC_US) *
		timing->pixel_clock_khz)
		return -1;

	h_blank_end = h_total - timing->h_front_porch - timing->h_active;
	v_blank_start = v_total - timing->v_front_porch;
	v_blank_end = v_blank_start - timing->v_active;
	vstartup = timing->v_blank > 13 ? 13 : timing->v_blank - 1;
	vupdate_offset = (h_total + 3) / 4;
	vupdate_width = div_round_up_u64(14ULL * timing->pixel_clock_khz,
					 DCN10_DEEP_SLEEP_DCFCLK_KHZ) +
		div_round_up_u64(12ULL * timing->pixel_clock_khz,
				 timing->disp_clock_khz);
	vready_dpp = div_round_up_u64(150ULL * timing->pixel_clock_khz,
				      timing->disp_clock_khz);
	vready_dcf = div_round_up_u64(20ULL * timing->pixel_clock_khz,
				      DCN10_DEEP_SLEEP_DCFCLK_KHZ) +
		div_round_up_u64(10ULL * timing->pixel_clock_khz,
				 timing->disp_clock_khz);
	vready = vready_dpp > vready_dcf ? vready_dpp : vready_dcf;

	ratio = ((uint64_t)timing->ref_clock_khz << 19) /
		timing->pixel_clock_khz;
	delay_pixels = (132ULL * timing->pixel_clock_khz) /
		timing->disp_clock_khz;
	setup_pixels = vupdate_offset + vupdate_width + vready + delay_pixels;
	wait_pixels = ((DCN10_URGENT_LATENCY_US + DCN10_PREFETCH_CALC_US) *
		       timing->pixel_clock_khz) / 1000;
	setup_pixels += wait_pixels;
	prefetch_num = ((uint64_t)((vstartup - 1) * 8 + 1) * h_total);
	if (prefetch_num <= 8ULL * setup_pixels)
		return -1;
	prefetch_quarters = (prefetch_num - 8ULL * setup_pixels) /
		(2ULL * h_total);
	if (prefetch_quarters < 8 || prefetch_quarters > 0xff)
		return -1;

	line_delivery = ((uint64_t)timing->ref_clock_khz *
			 timing->h_active) / timing->pixel_clock_khz;
	requests_per_line = (pitch_pixels * 4 + 255) / 256;
	if (!requests_per_line)
		return -1;

	memset(regs, 0, sizeof(*regs));
	regs->expansion_mode = 2 | (1U << 2) | (1U << 4) | (1U << 6);
	regs->request_size = (7U << 4) | (3U << 8) | (3U << 11) |
		(1U << 16) | (3U << 18) | (5U << 20) | (5U << 24);
	regs->blank_offset_0 = ((ratio * h_blank_end) >> 19) |
		(v_blank_end << 16);
	regs->blank_offset_1 = v_blank_start * 4 +
		((uint64_t)(DCN10_URGENT_LATENCY_US + DCN10_PREFETCH_CALC_US) *
		 timing->pixel_clock_khz * 4 / (1000ULL * h_total));
	regs->dst_dimensions = (ratio * h_total * 256) >> 19;
	regs->dst_after_scaler = (ratio * delay_pixels) >> 19;
	regs->ref_freq_to_pix_freq = ratio;
	regs->prefetch_settings = (prefetch_quarters << 24) | (1U << 19);
	/* No VM/DCC: Linux DML reserves 1/4 line for VM and 3/4 for row fetch. */
	regs->vblank_parameters_0 = 1U | (3U << 8);
	regs->per_line_delivery_pre = line_delivery;
	regs->per_line_delivery = line_delivery;
	regs->ttu_qos_wm = (((uint64_t)4 * h_total * timing->ref_clock_khz /
			      timing->pixel_clock_khz) & 0x3fff) << 16;
	regs->surf0_ttu_cntl0 = (((uint64_t)line_delivery << 10) /
				 requests_per_line) | (8U << 24);
	regs->surf0_ttu_cntl1 = regs->surf0_ttu_cntl0 & 0x00ffffff;
	regs->global_ttu_cntl =
		((DCN10_URGENT_LATENCY_US + DCN10_PREFETCH_CALC_US) *
		 timing->ref_clock_khz / 1000) | (14U << 28);
	regs->vstartup = vstartup;
	regs->vupdate = vupdate_offset | (vupdate_width << 16);
	regs->vready = vready;
	regs->watermark_urgent = watermark_cycles(4000, timing->ref_clock_khz);
	regs->watermark_pte_meta = regs->watermark_urgent;
	regs->watermark_sr_enter = watermark_cycles(19000, timing->ref_clock_khz);
	regs->watermark_sr_exit = watermark_cycles(17000, timing->ref_clock_khz);
	regs->watermark_pstate = watermark_cycles(21000, timing->ref_clock_khz);
	regs->arb_sat_level = 60 * (timing->ref_clock_khz / 1000);
	return 0;
}

int dcn10_surface_in_local_fb(uint64_t address, size_t size,
			      uint32_t fb_base, uint32_t fb_top)
{
	uint64_t base = (uint64_t)(fb_base & 0xffffff) << 24;
	uint64_t top = ((uint64_t)(fb_top & 0xffffff) + 1) << 24;

	if (!size || fb_top < fb_base || address < base || address >= top)
		return 0;
	return size <= top - address;
}
