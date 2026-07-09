/* SPDX-License-Identifier: GPL-2.0-only */

#include <console/console.h>
#include <delay.h>
#include <device/mmio.h>
#include <stddef.h>
#include <stdint.h>

#include "dcn10.h"

/* Raven/Picasso IP segment bases, from Linux vega10_ip_offset.h. */
#define DCN10_SEG1_BASE	0x00c0
#define DCN10_SEG2_BASE	0x34c0
#define GC9_SEG0_BASE	0x2000
#define DCN10_REG1(reg)	((DCN10_SEG1_BASE + (reg)) * 4)
#define DCN10_REG2(reg)	((DCN10_SEG2_BASE + (reg)) * 4)
#define GC9_REG0(reg)	((GC9_SEG0_BASE + (reg)) * 4)

#define DCCG_GATE_DISABLE_CNTL		0x0074
#define DCCG_GATE_DISABLE_CNTL2		0x007c
#define D1VGA_CONTROL			0x000c
#define D2VGA_CONTROL			0x000e
#define VGA_TEST_CONTROL		0x0015
#define D3VGA_CONTROL			0x0038
#define D4VGA_CONTROL			0x0039
#define GB_ADDR_CONFIG			0x063e

#define DOMAIN0_PG_CONFIG		0x008a
#define DOMAIN0_PG_STATUS		0x008b
#define DOMAIN1_PG_CONFIG		0x008c
#define DOMAIN1_PG_STATUS		0x008d
#define DC_IP_REQUEST_CNTL		0x00ad
#define DCHUBBUB_SDPIF_FB_BASE		0x0493
#define DCHUBBUB_SDPIF_FB_TOP		0x0494
#define DCHUBBUB_SDPIF_FB_OFFSET	0x0495
#define DCHUBBUB_SDPIF_AGP_BOT		0x0496
#define DCHUBBUB_SDPIF_AGP_TOP		0x0497
#define DCHUBBUB_SDPIF_AGP_BASE	0x0498
#define DCHUBBUB_ARB_DF_REQ_OUTSTAND	0x0505
#define DCHUBBUB_ARB_SAT_LEVEL		0x0506
#define DCHUBBUB_ARB_DRAM_STATE_CNTL	0x0508
#define DCHUBBUB_ARB_WM_A		0x0509
#define DCFCLK_CNTL			0x0530
#define VTG_CONTROL			0x0528

#define DCSURF_SURFACE_CONFIG		0x0559
#define DCSURF_ADDR_CONFIG		0x055a
#define DCSURF_TILING_CONFIG		0x055b
#define DCSURF_PRI_VIEWPORT_START	0x055c
#define DCSURF_PRI_VIEWPORT_DIMENSION	0x055d
#define DCSURF_PRI_VIEWPORT_START_C	0x055e
#define DCSURF_PRI_VIEWPORT_DIMENSION_C	0x055f
#define DCSURF_SEC_VIEWPORT_START	0x0560
#define DCSURF_SEC_VIEWPORT_DIMENSION	0x0561
#define DCSURF_SEC_VIEWPORT_START_C	0x0562
#define DCSURF_SEC_VIEWPORT_DIMENSION_C	0x0563
#define DCHUBP_REQ_SIZE_CONFIG		0x0564
#define DCHUBP_REQ_SIZE_CONFIG_C	0x0565
#define DCHUBP_CNTL			0x0566
#define HUBP_CLK_CNTL			0x0567
#define DCSURF_SURFACE_PITCH		0x057b
#define DCSURF_PRIMARY_ADDRESS		0x057d
#define DCSURF_PRIMARY_ADDRESS_HIGH	0x057e
#define DCSURF_SURFACE_CONTROL		0x058d
#define DCSURF_FLIP_CONTROL		0x058e
#define DCSURF_EARLIEST_INUSE		0x0597
#define DCSURF_EARLIEST_INUSE_HIGH	0x0598
#define DCN_EXPANSION_MODE		0x059b
#define DCN_TTU_QOS_WM			0x059c
#define DCN_GLOBAL_TTU_CNTL		0x059d
#define DCN_SURF0_TTU_CNTL0		0x059e
#define DCN_SURF0_TTU_CNTL1		0x059f
#define DCN_SURF1_TTU_CNTL0		0x05a0
#define DCN_SURF1_TTU_CNTL1		0x05a1
#define DCN_CUR0_TTU_CNTL0		0x05a2
#define DCN_CUR0_TTU_CNTL1		0x05a3
#define DCN_VM_MX_L1_TLB_CNTL		0x05b5
#define BLANK_OFFSET_0			0x05b6
#define BLANK_OFFSET_1			0x05b7
#define DST_DIMENSIONS			0x05b8
#define DST_AFTER_SCALER		0x05b9
#define PREFETCH_SETTINGS		0x05ba
#define PREFETCH_SETTINGS_C		0x05bb
#define VBLANK_PARAMETERS_0		0x05bc
#define VBLANK_PARAMETERS_1		0x05bd
#define VBLANK_PARAMETERS_2		0x05be
#define VBLANK_PARAMETERS_3		0x05bf
#define VBLANK_PARAMETERS_4		0x05c0
#define NOM_PARAMETERS_0		0x05c1
#define NOM_PARAMETERS_7		0x05c8
#define PER_LINE_DELIVERY_PRE		0x05c9
#define PER_LINE_DELIVERY		0x05ca
#define REF_FREQ_TO_PIX_FREQ		0x05cc
#define HUBPRET_CONTROL			0x05e0
#define HUBP_CURSOR_CONTROL		0x05ec

#define DPP_CONTROL			0x0c3d
#define CNVC_SURFACE_PIXEL_FORMAT	0x0c47
#define CNVC_FORMAT_CONTROL		0x0c48
#define CNVC_CURSOR0_CONTROL		0x0c58
#define DSCL_SCL_MODE			0x0c64
#define DSCL_CONTROL			0x0c66
#define DSCL_RECOUT_START		0x0c7a
#define DSCL_RECOUT_SIZE		0x0c7b
#define DSCL_MPC_SIZE			0x0c7c
#define DSCL_AUTOCAL			0x0c75
#define DSCL_LB_DATA_FORMAT		0x0c7d
#define DSCL_LB_MEMORY_CTRL		0x0c7e
#define CM_IGAM_CONTROL		0x0c9f
#define CM_ICSC_CONTROL		0x0ca9
#define CM_GAMUT_REMAP_CONTROL		0x0cb0
#define CM_OCSC_CONTROL		0x0cb7
#define CM_BNS_VALUES_R		0x0cbe
#define CM_BNS_VALUES_G		0x0cbf
#define CM_BNS_VALUES_B		0x0cc0
#define CM_RGAM_CONTROL		0x0ced
#define CM_HDR_MULT_COEF		0x0d2b

#define MPCC0_TOP_SEL			0x1630
#define MPCC0_BOT_SEL			0x1631
#define MPCC0_OPP_ID			0x1632
#define MPCC0_CONTROL			0x1633
#define MPCC0_STATUS			0x1634
#define MPCC0_UPDATE_LOCK_SEL		0x1635
#define MPCC_STRIDE			0x001b
#define MPC_OUT0_MUX			0x172f
#define OPP_PIPE_CONTROL		0x188c
#define FMT_CONTROL			0x1840
#define FMT_BIT_DEPTH_CONTROL		0x1841
#define FMT_CLAMP_CNTL			0x1848
#define OPTC_DATA_SOURCE_SELECT		0x1acb
#define OPTC_INPUT_CLOCK_CONTROL	0x1acd
#define OTG_H_TOTAL			0x1b2a
#define OTG_H_BLANK_START_END		0x1b2b
#define OTG_H_SYNC_A			0x1b2c
#define OTG_H_SYNC_A_CNTL		0x1b2d
#define OTG_H_TIMING_CNTL		0x1b2e
#define OTG_V_TOTAL			0x1b2f
#define OTG_V_TOTAL_MIN			0x1b30
#define OTG_V_TOTAL_MAX			0x1b31
#define OTG_V_BLANK_START_END		0x1b36
#define OTG_V_SYNC_A			0x1b37
#define OTG_V_SYNC_A_CNTL		0x1b38
#define OTG_CONTROL			0x1b41
#define OTG_BLANK_CONTROL		0x1b42
#define OTG_INTERLACE_CONTROL		0x1b44
#define OTG_STATUS			0x1b49
#define OTG_STATUS_FRAME_COUNT		0x1b4c
#define OTG_DOUBLE_BUFFER_CONTROL	0x1b5b
#define OTG_CLOCK_CONTROL		0x1b88
#define OTG_VSTARTUP_PARAM		0x1b89
#define OTG_VUPDATE_PARAM		0x1b8a
#define OTG_VREADY_PARAM		0x1b8b
#define OTG_MASTER_UPDATE_LOCK		0x1b8d
#define OTG_GLOBAL_CONTROL0		0x1b92

#define HUBP_BLANK_EN			(1U << 0)
#define HUBP_NO_OUTSTANDING_REQ	(1U << 1)
#define HUBP_DISABLE			(1U << 2)
#define HUBP_IN_BLANK			(1U << 3)
#define HUBP_VTG_SEL_MASK		(0xfU << 4)
#define HUBP_TTU_DISABLE		(1U << 12)
#define HUBP_UNDERFLOW_STATUS		(7U << 28)
#define HUBP_CLOCK_ENABLE		(1U << 0)
#define DPP_CLOCK_ENABLE		(1U << 4)
#define OPP_PIPE_CLOCK_ENABLE		(1U << 0)
#define OTG_CURRENT_MASTER_EN		(1U << 16)
#define LB_PIXEL_DEPTH_36BPP		3U
#define LB_PIXEL_REDUCE_MODE		(1U << 12)
#define LB_MEMORY_CONFIG_1		1U
#define LB_MAX_PARTITIONS		(63U << 8)
#define CM_BNS_SCALE_UNITY		(0x2000U << 16)
#define CM_HDR_MULT_ONE		0x1f000U
#define DSCL_MODE_SCALING_444_BYPASS	0U

#ifdef DCN10_TEST_TRACE
void dcn10_test_mmio_write(uint32_t reg, uint32_t value);
#endif

static uint32_t dcn10_read(void *mmio, uint32_t reg)
{
	return read32((uint8_t *)mmio + DCN10_REG2(reg));
}

static void dcn10_write(void *mmio, uint32_t reg, uint32_t value)
{
	write32((uint8_t *)mmio + DCN10_REG2(reg), value);
#ifdef DCN10_TEST_TRACE
	dcn10_test_mmio_write(reg, value);
#endif
}

static void dcn10_update(void *mmio, uint32_t reg, uint32_t mask, uint32_t value)
{
	uint32_t old = dcn10_read(mmio, reg);

	dcn10_write(mmio, reg, (old & ~mask) | (value & mask));
}

static int dcn10_wait(void *mmio, uint32_t reg, uint32_t mask,
		      uint32_t value, unsigned int timeout_us)
{
	while (timeout_us--) {
		if ((dcn10_read(mmio, reg) & mask) == value)
			return 0;
		udelay(1);
	}
	return -1;
}

void dcn10_disable_vga(void *mmio_base)
{
	uint32_t vga_mode;

	vga_mode = read32((uint8_t *)mmio_base + DCN10_REG1(D1VGA_CONTROL)) |
		read32((uint8_t *)mmio_base + DCN10_REG1(D2VGA_CONTROL)) |
		read32((uint8_t *)mmio_base + DCN10_REG1(D3VGA_CONTROL)) |
		read32((uint8_t *)mmio_base + DCN10_REG1(D4VGA_CONTROL));
	if (!(vga_mode & 1))
		return;

	write32((uint8_t *)mmio_base + DCN10_REG1(D1VGA_CONTROL), 0);
	write32((uint8_t *)mmio_base + DCN10_REG1(D2VGA_CONTROL), 0);
	write32((uint8_t *)mmio_base + DCN10_REG1(D3VGA_CONTROL), 0);
	write32((uint8_t *)mmio_base + DCN10_REG1(D4VGA_CONTROL), 0);
	vga_mode = read32((uint8_t *)mmio_base + DCN10_REG1(VGA_TEST_CONTROL));
	write32((uint8_t *)mmio_base + DCN10_REG1(VGA_TEST_CONTROL),
		vga_mode | 0x101);
}

static void dcn10_reset_mpc(void *mmio, unsigned int pipe_count)
{
	unsigned int i;

	for (i = 0; i < pipe_count; i++) {
		uint32_t base = MPCC0_TOP_SEL + i * MPCC_STRIDE;

		dcn10_update(mmio, base, 0xf, 0xf);
		dcn10_update(mmio, base + 1, 0xf, 0xf);
		dcn10_update(mmio, base + 2, 0xf, 0xf);
		dcn10_update(mmio, base + 3, 0x3, 0);
		dcn10_update(mmio, base + 5, 0xf, 0xf);
		dcn10_update(mmio, MPC_OUT0_MUX + i, 0xf, 0xf);
	}
}

int dcn10_cold_init(void *mmio_base, uint8_t pipe_count)
{
	if (!mmio_base || !pipe_count || pipe_count > 4)
		return -1;

	write32((uint8_t *)mmio_base + DCN10_REG1(DCCG_GATE_DISABLE_CNTL), 0);
	write32((uint8_t *)mmio_base + DCN10_REG1(DCCG_GATE_DISABLE_CNTL2), 0);
	dcn10_update(mmio_base, DCFCLK_CNTL, 1U << 31, 0);
	dcn10_reset_mpc(mmio_base, pipe_count);
	dcn10_update(mmio_base, DCHUBP_CNTL,
		     HUBP_BLANK_EN | HUBP_DISABLE | HUBP_TTU_DISABLE,
		     HUBP_BLANK_EN | HUBP_DISABLE | HUBP_TTU_DISABLE);
	return 0;
}

static int dcn10_power_on_plane(void *mmio)
{
	dcn10_update(mmio, DC_IP_REQUEST_CNTL, 1, 1);
	dcn10_update(mmio, DOMAIN1_PG_CONFIG, 1U << 8, 0);
	if (dcn10_wait(mmio, DOMAIN1_PG_STATUS, 3U << 30, 0, 1000))
		goto fail;
	dcn10_update(mmio, DOMAIN0_PG_CONFIG, 1U << 8, 0);
	if (dcn10_wait(mmio, DOMAIN0_PG_STATUS, 3U << 30, 0, 1000))
		goto fail;
	dcn10_update(mmio, DC_IP_REQUEST_CNTL, 1, 0);
	return 0;
fail:
	dcn10_update(mmio, DC_IP_REQUEST_CNTL, 1, 0);
	return -1;
}

static int dcn10_lock(void *mmio, int lock)
{
	dcn10_update(mmio, OTG_GLOBAL_CONTROL0, 0x0e000000, 0);
	dcn10_update(mmio, OTG_MASTER_UPDATE_LOCK, 1, lock ? 1 : 0);
	return dcn10_wait(mmio, OTG_MASTER_UPDATE_LOCK, 1U << 8,
			  lock ? 1U << 8 : 0, 1000);
}

int dcn10_program_stream(void *mmio, const struct dcn10_timing *timing)
{
	struct dcn10_plane_regs calc;
	uint32_t h_total, v_total, h_blank_start, h_blank_end;
	uint32_t v_blank_start, v_blank_end, frame;

	if (dcn10_calculate_plane(timing,
				   (timing->h_active + 63U) & ~63U, &calc))
		return -1;
	h_total = timing->h_active + timing->h_blank;
	v_total = timing->v_active + timing->v_blank;
	h_blank_start = h_total - timing->h_front_porch;
	h_blank_end = h_blank_start - timing->h_active;
	v_blank_start = v_total - timing->v_front_porch;
	v_blank_end = v_blank_start - timing->v_active;

	/* optc1_enable_optc_clock() and tg_init(), still transmitting blank data. */
	dcn10_update(mmio, OPTC_INPUT_CLOCK_CONTROL, 0x3, 0x3);
	if (dcn10_wait(mmio, OPTC_INPUT_CLOCK_CONTROL, 1U << 2, 1U << 2, 1000))
		return -1;
	dcn10_update(mmio, OTG_CLOCK_CONTROL, 0x3, 0x3);
	if (dcn10_wait(mmio, OTG_CLOCK_CONTROL, 1U << 8, 1U << 8, 1000))
		return -1;
	dcn10_update(mmio, OTG_BLANK_CONTROL, 0x00010100, 0x00000100);
	dcn10_update(mmio, VTG_CONTROL, 0x80000000, 0);
	dcn10_update(mmio, OTG_DOUBLE_BUFFER_CONTROL, 0x00010100, 0x00000100);

	dcn10_write(mmio, OTG_H_TOTAL, h_total - 1);
	dcn10_write(mmio, OTG_H_SYNC_A, (uint32_t)timing->h_sync_width << 16);
	dcn10_write(mmio, OTG_H_BLANK_START_END,
		    h_blank_start | (h_blank_end << 16));
	dcn10_update(mmio, OTG_H_SYNC_A_CNTL, 1,
		     timing->hsync_positive ? 0 : 1);
	dcn10_write(mmio, OTG_H_TIMING_CNTL, 0);
	dcn10_write(mmio, OTG_V_TOTAL, v_total - 1);
	dcn10_write(mmio, OTG_V_TOTAL_MIN, v_total - 1);
	dcn10_write(mmio, OTG_V_TOTAL_MAX, v_total - 1);
	dcn10_write(mmio, OTG_V_SYNC_A, (uint32_t)timing->v_sync_width << 16);
	dcn10_write(mmio, OTG_V_BLANK_START_END,
		    v_blank_start | (v_blank_end << 16));
	dcn10_update(mmio, OTG_V_SYNC_A_CNTL, 1,
		     timing->vsync_positive ? 0 : 1);
	dcn10_write(mmio, OTG_INTERLACE_CONTROL, 0);
	dcn10_write(mmio, OTG_VSTARTUP_PARAM, calc.vstartup);
	dcn10_write(mmio, OTG_VUPDATE_PARAM, calc.vupdate);
	dcn10_write(mmio, OTG_VREADY_PARAM, calc.vready);
	dcn10_update(mmio, OTG_CONTROL, 0x7000, 1U << 12);
	dcn10_update(mmio, VTG_CONTROL, 0x3fff8000,
		     (v_total - timing->v_front_porch) << 15);
	dcn10_update(mmio, OPTC_DATA_SOURCE_SELECT, 0x700, 0);
	dcn10_update(mmio, VTG_CONTROL, 0x80000000, 0x80000000);
	dcn10_update(mmio, OTG_CONTROL, 0x301, 0x301);
	if (dcn10_wait(mmio, OTG_CONTROL, OTG_CURRENT_MASTER_EN,
			 OTG_CURRENT_MASTER_EN, 20000))
		return -1;
	frame = dcn10_read(mmio, OTG_STATUS_FRAME_COUNT);
	mdelay(20);
	if (frame == dcn10_read(mmio, OTG_STATUS_FRAME_COUNT))
		return -1;
	return 0;
}

static int dcn10_addr_config(void *mmio, uint32_t *config)
{
	uint32_t gb = read32((uint8_t *)mmio + GC9_REG0(GB_ADDR_CONFIG));
	uint32_t pipe_interleave = (gb >> 3) & 0x7;

	/* DCSURF has a two-bit PIPE_INTERLEAVE encoding on DCN1. */
	if (pipe_interleave > 3)
		return -1;
	*config = (gb & 0x7) | (((gb >> 12) & 0x7) << 3) |
		(pipe_interleave << 6) | (((gb >> 19) & 0x3) << 8) |
		(((gb >> 26) & 0x3) << 10) | (((gb >> 6) & 0x3) << 12);
	return 0;
}

static void dcn10_program_watermarks(void *mmio,
				     const struct dcn10_plane_regs *calc)
{
	unsigned int i;

	for (i = 0; i < 4; i++) {
		uint32_t reg = DCHUBBUB_ARB_WM_A + 5 * i;

		dcn10_write(mmio, reg, calc->watermark_urgent);
		dcn10_write(mmio, reg + 1, calc->watermark_pte_meta);
		dcn10_write(mmio, reg + 2, calc->watermark_sr_enter);
		dcn10_write(mmio, reg + 3, calc->watermark_sr_exit);
		dcn10_write(mmio, reg + 4, calc->watermark_pstate);
	}
	dcn10_write(mmio, DCHUBBUB_ARB_SAT_LEVEL, calc->arb_sat_level);
	dcn10_update(mmio, DCHUBBUB_ARB_DF_REQ_OUTSTAND, 0x01ff0000, 68U << 16);
	/* Firmware is static: prohibit self-refresh and memory pstate changes. */
	dcn10_update(mmio, DCHUBBUB_ARB_DRAM_STATE_CNTL, 0x33, 0x22);
}

static void dcn10_program_deadlines(void *mmio,
				    const struct dcn10_plane_regs *calc)
{
	unsigned int reg;

	dcn10_write(mmio, DCN_EXPANSION_MODE, calc->expansion_mode);
	dcn10_write(mmio, DCHUBP_REQ_SIZE_CONFIG, calc->request_size);
	dcn10_write(mmio, DCHUBP_REQ_SIZE_CONFIG_C, 0);
	dcn10_write(mmio, BLANK_OFFSET_0, calc->blank_offset_0);
	dcn10_write(mmio, BLANK_OFFSET_1, calc->blank_offset_1);
	dcn10_write(mmio, DST_DIMENSIONS, calc->dst_dimensions);
	dcn10_write(mmio, DST_AFTER_SCALER, calc->dst_after_scaler);
	dcn10_write(mmio, REF_FREQ_TO_PIX_FREQ, calc->ref_freq_to_pix_freq);
	dcn10_write(mmio, PREFETCH_SETTINGS, calc->prefetch_settings);
	dcn10_write(mmio, PREFETCH_SETTINGS_C, 0);
	dcn10_write(mmio, VBLANK_PARAMETERS_0, calc->vblank_parameters_0);
	/*
	 * VBLANK1/2 and NOM0..3 schedule DPTE; VBLANK3/4 and NOM4..7
	 * schedule DCC metadata. L1 TLB and DCC are disabled, respectively.
	 */
	for (reg = VBLANK_PARAMETERS_1; reg <= VBLANK_PARAMETERS_4; reg++)
		dcn10_write(mmio, reg, 0);
	for (reg = NOM_PARAMETERS_0; reg <= NOM_PARAMETERS_7; reg++)
		dcn10_write(mmio, reg, 0);
	dcn10_write(mmio, PER_LINE_DELIVERY_PRE, calc->per_line_delivery_pre);
	dcn10_write(mmio, PER_LINE_DELIVERY, calc->per_line_delivery);
	dcn10_write(mmio, DCN_TTU_QOS_WM, calc->ttu_qos_wm);
	dcn10_write(mmio, DCN_SURF0_TTU_CNTL0, calc->surf0_ttu_cntl0);
	dcn10_write(mmio, DCN_SURF0_TTU_CNTL1, calc->surf0_ttu_cntl1);
	dcn10_write(mmio, DCN_SURF1_TTU_CNTL0, 0);
	dcn10_write(mmio, DCN_SURF1_TTU_CNTL1, 0);
	dcn10_write(mmio, DCN_CUR0_TTU_CNTL0, 0);
	dcn10_write(mmio, DCN_CUR0_TTU_CNTL1, 0);
	dcn10_write(mmio, DCN_GLOBAL_TTU_CNTL, calc->global_ttu_cntl);
}

static void dcn10_fill_test_pattern(void *address, uint32_t pitch,
				    uint16_t width, uint16_t height)
{
	static const uint32_t colors[] = {
		0xffff0000, 0xffffff00, 0xff00ff00,
		0xff00ffff, 0xff0000ff, 0xffff00ff,
	};
	uint16_t x, y;

	for (y = 0; y < height; y++) {
		uint32_t *row = (uint32_t *)((uint8_t *)address + y * pitch);

		for (x = 0; x < width; x++)
			row[x] = colors[(x * 6U) / width];
	}
}

int dcn10_program_scanout(void *mmio, uint64_t gpu_address,
			  void *cpu_address, size_t aperture_size,
			  const struct dcn10_timing *timing,
			  uint32_t *pitch_bytes)
{
	struct dcn10_plane_regs calc;
	uint32_t pitch_pixels, pitch, dimensions, hubp_cntl;
	uint32_t fb_base, fb_top, addr_config;
	unsigned int i;

	if (!mmio || !cpu_address || !gpu_address || !timing || !pitch_bytes ||
	    (gpu_address & 0xff))
		return -1;
	pitch_pixels = (timing->h_active + 63U) & ~63U;
	pitch = pitch_pixels * 4;
	if (dcn10_calculate_plane(timing, pitch_pixels, &calc) ||
	    aperture_size < (size_t)pitch * timing->v_active)
		return -1;
	fb_base = dcn10_read(mmio, DCHUBBUB_SDPIF_FB_BASE) & 0xffffff;
	fb_top = dcn10_read(mmio, DCHUBBUB_SDPIF_FB_TOP) & 0xffffff;
	if (!dcn10_surface_in_local_fb(gpu_address,
					(size_t)pitch * timing->v_active,
					fb_base, fb_top)) {
		printk(BIOS_ERR,
		       "ATOMBIOS: DCN1 surface outside local FB (%06x-%06x)\n",
		       fb_base, fb_top);
		return -1;
	}
	if (dcn10_addr_config(mmio, &addr_config)) {
		printk(BIOS_ERR, "ATOMBIOS: unsupported GFX9 address topology\n");
		return -1;
	}

	/* Linux FRAME_BUFFER_MODE_LOCAL_ONLY; preserve VBIOS FB base/top/offset. */
	dcn10_write(mmio, DCHUBBUB_SDPIF_AGP_BASE, 0);
	dcn10_write(mmio, DCHUBBUB_SDPIF_AGP_BOT, 0x03ffff);
	dcn10_write(mmio, DCHUBBUB_SDPIF_AGP_TOP, 0);
	dcn10_program_watermarks(mmio, &calc);

	if (dcn10_lock(mmio, 1))
		return -1;
	if (dcn10_power_on_plane(mmio))
		goto fail_unlock;
	hubp_cntl = dcn10_read(mmio, DCHUBP_CNTL);
	hubp_cntl &= ~HUBP_VTG_SEL_MASK;
	hubp_cntl |= HUBP_BLANK_EN | HUBP_TTU_DISABLE;
	dcn10_write(mmio, DCHUBP_CNTL, hubp_cntl);
	if (dcn10_wait(mmio, DCHUBP_CNTL, HUBP_NO_OUTSTANDING_REQ,
			 HUBP_NO_OUTSTANDING_REQ, 1000))
		goto fail_unlock;

	dcn10_update(mmio, HUBP_CLK_CNTL, HUBP_CLOCK_ENABLE, HUBP_CLOCK_ENABLE);
	dcn10_update(mmio, DPP_CONTROL, DPP_CLOCK_ENABLE, DPP_CLOCK_ENABLE);
	dcn10_update(mmio, OPP_PIPE_CONTROL,
		     OPP_PIPE_CLOCK_ENABLE, OPP_PIPE_CLOCK_ENABLE);
	/* Physical local-FB addresses: disable page-table and metadata requests. */
	dcn10_write(mmio, DCN_VM_MX_L1_TLB_CNTL, 0);
	dcn10_write(mmio, DCSURF_SURFACE_CONTROL, 0);
	dcn10_program_deadlines(mmio, &calc);

	/* Complete fixed RGB bypass state. */
	dcn10_write(mmio, CNVC_SURFACE_PIXEL_FORMAT, 8);
	dcn10_write(mmio, CNVC_FORMAT_CONTROL, 1U << 8);
	dcn10_write(mmio, CM_IGAM_CONTROL, 2U << 26);
	dcn10_write(mmio, CM_ICSC_CONTROL, 0);
	dcn10_write(mmio, CM_GAMUT_REMAP_CONTROL, 0);
	dcn10_write(mmio, CM_OCSC_CONTROL, 0);
	dcn10_write(mmio, CM_RGAM_CONTROL, 0);
	dcn10_write(mmio, CM_HDR_MULT_COEF, CM_HDR_MULT_ONE);
	dcn10_write(mmio, CM_BNS_VALUES_R, CM_BNS_SCALE_UNITY);
	dcn10_write(mmio, CM_BNS_VALUES_G, CM_BNS_SCALE_UNITY);
	dcn10_write(mmio, CM_BNS_VALUES_B, CM_BNS_SCALE_UNITY);
	dcn10_write(mmio, CNVC_CURSOR0_CONTROL, 0);
	dcn10_write(mmio, HUBP_CURSOR_CONTROL, 0);
	dcn10_write(mmio, DSCL_AUTOCAL, 0);
	dcn10_write(mmio, DSCL_CONTROL, 0);
	dcn10_write(mmio, DSCL_RECOUT_START, 0);
	dimensions = ((uint32_t)timing->v_active << 16) | timing->h_active;
	dcn10_write(mmio, DSCL_RECOUT_SIZE, dimensions);
	dcn10_write(mmio, DSCL_MPC_SIZE, dimensions);
	dcn10_write(mmio, DSCL_LB_DATA_FORMAT,
		    LB_PIXEL_DEPTH_36BPP | LB_PIXEL_REDUCE_MODE);
	dcn10_write(mmio, DSCL_LB_MEMORY_CTRL,
		    LB_MEMORY_CONFIG_1 | LB_MAX_PARTITIONS);
	dcn10_update(mmio, DSCL_SCL_MODE, 0x7,
		     DSCL_MODE_SCALING_444_BYPASS);

	/* Known disconnected MPCC0, then DPP0 -> MPCC0 -> OPP0. */
	if (dcn10_wait(mmio, MPCC0_STATUS, 1, 1, 1000))
		goto fail_unlock;
	dcn10_update(mmio, MPCC0_BOT_SEL, 0xf, 0xf);
	dcn10_write(mmio, MPCC0_CONTROL, 2 | (0xffU << 16) | (0xffU << 24));
	dcn10_update(mmio, MPCC0_TOP_SEL, 0xf, 0);
	dcn10_update(mmio, MPCC0_OPP_ID, 0xf, 0);
	dcn10_update(mmio, MPCC0_UPDATE_LOCK_SEL, 0xf, 0);
	dcn10_update(mmio, MPC_OUT0_MUX, 0xf, 0);
	/* Linux leaves DCN1 FMT defaults for 8-bpc RGB; clear stale dither/clamp. */
	dcn10_write(mmio, FMT_CONTROL, 0);
	dcn10_write(mmio, FMT_BIT_DEPTH_CONTROL, 0);
	dcn10_write(mmio, FMT_CLAMP_CNTL, 0);

	dcn10_write(mmio, DCSURF_ADDR_CONFIG, addr_config);
	dcn10_write(mmio, DCSURF_TILING_CONFIG, 0);
	dcn10_update(mmio, DCSURF_SURFACE_CONFIG, 0x0000077f, 8);
	dcn10_write(mmio, DCSURF_SURFACE_PITCH, pitch_pixels - 1);
	dcn10_write(mmio, DCSURF_PRI_VIEWPORT_START, 0);
	dcn10_write(mmio, DCSURF_PRI_VIEWPORT_DIMENSION, dimensions);
	dcn10_write(mmio, DCSURF_PRI_VIEWPORT_START_C, 0);
	dcn10_write(mmio, DCSURF_PRI_VIEWPORT_DIMENSION_C, 0);
	dcn10_write(mmio, DCSURF_SEC_VIEWPORT_START, 0);
	dcn10_write(mmio, DCSURF_SEC_VIEWPORT_DIMENSION, dimensions);
	dcn10_write(mmio, DCSURF_SEC_VIEWPORT_START_C, 0);
	dcn10_write(mmio, DCSURF_SEC_VIEWPORT_DIMENSION_C, 0);
	dcn10_update(mmio, DCSURF_FLIP_CONTROL, 0x7, 0);
	dcn10_update(mmio, HUBPRET_CONTROL, 0x00f00fff, 0x00e00000);

	dcn10_fill_test_pattern(cpu_address, pitch,
				timing->h_active, timing->v_active);
	dcn10_write(mmio, DCSURF_PRIMARY_ADDRESS_HIGH, gpu_address >> 32);
	dcn10_write(mmio, DCSURF_PRIMARY_ADDRESS, gpu_address);
	hubp_cntl = dcn10_read(mmio, DCHUBP_CNTL);
	hubp_cntl &= ~(HUBP_BLANK_EN | HUBP_DISABLE | HUBP_TTU_DISABLE |
		       HUBP_VTG_SEL_MASK | HUBP_UNDERFLOW_STATUS);
	dcn10_write(mmio, DCHUBP_CNTL, hubp_cntl);
	if (dcn10_lock(mmio, 0))
		return -1;

	/* Plane is committed; now permit stream pixels. */
	dcn10_update(mmio, OTG_BLANK_CONTROL, 0x00010100, 0);
	for (i = 0; i < 50; i++) {
		uint64_t earliest;

		mdelay(1);
		earliest = ((uint64_t)dcn10_read(mmio, DCSURF_EARLIEST_INUSE_HIGH) << 32) |
			dcn10_read(mmio, DCSURF_EARLIEST_INUSE);
		if (!(dcn10_read(mmio, DCHUBP_CNTL) & HUBP_IN_BLANK) &&
		    earliest == gpu_address)
			break;
	}
	if (i == 50 || (dcn10_read(mmio, DCHUBP_CNTL) & HUBP_UNDERFLOW_STATUS)) {
		printk(BIOS_ERR,
		       "ATOMBIOS: DCN1 plane commit failed cntl=0x%08x earliest=0x%08x%08x\n",
		       dcn10_read(mmio, DCHUBP_CNTL),
		       dcn10_read(mmio, DCSURF_EARLIEST_INUSE_HIGH),
		       dcn10_read(mmio, DCSURF_EARLIEST_INUSE));
		dcn10_update(mmio, OTG_BLANK_CONTROL, 0x00010100, 0x00000100);
		dcn10_update(mmio, DCHUBP_CNTL,
			     HUBP_BLANK_EN | HUBP_TTU_DISABLE,
			     HUBP_BLANK_EN | HUBP_TTU_DISABLE);
		return -1;
	}
	*pitch_bytes = pitch;
	printk(BIOS_INFO,
	       "ATOMBIOS: DCN1 native plane active addr=0x%016llx pitch=%u fb=%06x-%06x off=%06x\n",
	       (unsigned long long)gpu_address, pitch, fb_base, fb_top,
	       dcn10_read(mmio, DCHUBBUB_SDPIF_FB_OFFSET) & 0xffffff);
	return 0;

fail_unlock:
	dcn10_lock(mmio, 0);
	return -1;
}
