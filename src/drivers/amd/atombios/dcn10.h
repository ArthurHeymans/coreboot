/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_ATOMBIOS_DCN10_H
#define AMD_ATOMBIOS_DCN10_H

#include <stddef.h>
#include <stdint.h>

struct dcn10_timing {
	uint16_t h_active;
	uint16_t h_blank;
	uint16_t h_front_porch;
	uint16_t h_sync_width;
	uint16_t v_active;
	uint16_t v_blank;
	uint16_t v_front_porch;
	uint16_t v_sync_width;
	uint32_t pixel_clock_khz;
	uint32_t ref_clock_khz;
	uint32_t disp_clock_khz;
	uint8_t hsync_positive;
	uint8_t vsync_positive;
};

/* Fixed DCN1 state for one linear, non-DCC, non-VM ARGB8888 plane. */
struct dcn10_plane_regs {
	uint32_t expansion_mode;
	uint32_t request_size;
	uint32_t request_size_c;
	uint32_t blank_offset_0;
	uint32_t blank_offset_1;
	uint32_t dst_dimensions;
	uint32_t dst_after_scaler;
	uint32_t ref_freq_to_pix_freq;
	uint32_t prefetch_settings;
	uint32_t vblank_parameters_0;
	uint32_t per_line_delivery_pre;
	uint32_t per_line_delivery;
	uint32_t ttu_qos_wm;
	uint32_t surf0_ttu_cntl0;
	uint32_t surf0_ttu_cntl1;
	uint32_t global_ttu_cntl;
	uint32_t vstartup;
	uint32_t vupdate;
	uint32_t vready;
	uint32_t watermark_urgent;
	uint32_t watermark_pte_meta;
	uint32_t watermark_sr_enter;
	uint32_t watermark_sr_exit;
	uint32_t watermark_pstate;
	uint32_t arb_sat_level;
};

int dcn10_calculate_plane(const struct dcn10_timing *timing,
			  uint32_t pitch_pixels,
			  struct dcn10_plane_regs *regs);
int dcn10_surface_in_local_fb(uint64_t address, size_t size,
			      uint32_t fb_base, uint32_t fb_top);
void dcn10_disable_vga(void *mmio_base);
int dcn10_cold_init(void *mmio_base, uint8_t pipe_count);
int dcn10_program_stream(void *mmio_base, const struct dcn10_timing *timing);
int dcn10_program_scanout(void *mmio_base, uint64_t gpu_address,
			  void *cpu_address, size_t aperture_size,
			  const struct dcn10_timing *timing,
			  uint32_t *pitch_bytes);

#endif /* AMD_ATOMBIOS_DCN10_H */
