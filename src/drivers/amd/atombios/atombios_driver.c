/* SPDX-License-Identifier: GPL-2.0-only */

/*
 * AMD AtomBIOS modesetting driver for coreboot.
 *
 * Executes ATOM command tables from the VBIOS to initialize AMD GPUs
 * and set up a linear framebuffer, without running the 16-bit option ROM.
 *
 * The ATOM interpreter (atom.c) is MIT-licensed, ported from the Linux
 * kernel amdgpu driver (original author: Stanislaw Skowronek).
 */

#include <cbfs.h>
#include <console/console.h>
#include <delay.h>
#include <device/device.h>
#include <device/mmio.h>
#include <device/pci.h>
#include <device/pci_ids.h>
#include <device/pci_ops.h>
#include <device/resource.h>
#include <edid.h>
#include <framebuffer_info.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "atom.h"
#include "atom-bits.h"
#include "atomfirmware.h"

/*
 * AMD GPU MMIO BAR indices:
 *   BAR 0 (or 2 on some): framebuffer aperture (VRAM)
 *   BAR 2 (or 5 on some): MMIO register space
 * For discrete GPUs, the MMIO BAR is typically PCI_BASE_ADDRESS_2 or
 * PCI_BASE_ADDRESS_5. We try BAR5 first (common for GCN+), then BAR2.
 */
#define AMD_GPU_MMIO_BAR	PCI_BASE_ADDRESS_5
#define AMD_GPU_FB_BAR		PCI_BASE_ADDRESS_0

/* Scratch memory for ATOM FB workspace (in dwords) */
#define ATOM_SCRATCH_SIZE_DWORDS	256

/* Default framebuffer settings */
#define DEFAULT_BPP	32
#define BYTES_PER_PIXEL	(DEFAULT_BPP / 8)
#define DEFAULT_DCE_DISPCLK_10KHZ 60000

/*
 * ATOM command and data table indices.
 *
 * The master command/data table layout differs between atombios.h (v1,
 * pre-Vega) and atomfirmware.h (v2, Vega+). The ATOM bytecode interpreter
 * is format-agnostic, but command table indices differ because the v2
 * master table has a different field order.
 *
 * We detect the format at runtime from the master table header revision
 * and store the resolved indices in an atombios_cmd_table_idx struct.
 */

/* v1 index: offsetof into ATOM_MASTER_LIST_OF_COMMAND_TABLES */
#define CMD_IDX_V1(field) \
	(offsetof(ATOM_MASTER_LIST_OF_COMMAND_TABLES, field) / sizeof(USHORT))

/* v2 index: offsetof into atom_master_list_of_command_functions_v2_1 */
#define CMD_IDX_V2(field) \
	(offsetof(struct atom_master_list_of_command_functions_v2_1, field) / sizeof(uint16_t))

/* v1 data index: offsetof into ATOM_MASTER_LIST_OF_DATA_TABLES */
#define DATA_IDX_V1(field) \
	(offsetof(ATOM_MASTER_LIST_OF_DATA_TABLES, field) / sizeof(USHORT))

/* v2 data index: offsetof into atom_master_list_of_data_tables_v2_1 */
#define DATA_IDX_V2(field) \
	(offsetof(struct atom_master_list_of_data_tables_v2_1, field) / sizeof(uint16_t))

struct atombios_cmd_indices {
	int asic_init;
	int set_pixel_clock;
	int adjust_display_pll;
	int set_dce_clock;
	int set_crtc_using_dtd;
	int enable_crtc;
	int blank_crtc;
	int select_crtc_source;
	int enable_disp_power_gating;
	int enable_scaler;
	int set_crtc_overscan;
	int enable_yuv;
	int dig_encoder_ctrl;
	int dig1_xmtr_ctrl;
	int tmds1_encoder_ctrl;
	int tmds2_encoder_ctrl;
	int tmds1_output_ctrl;
	int tmds2_output_ctrl;
	int dac1_encoder_ctrl;	/* -1 if not available (v2) */
	int dac2_encoder_ctrl;	/* -1 if not available (v2) */
	int dac_load_detect;	/* -1 if not available (v2) */
	int process_i2c;
	int process_aux;
	int data_object_header;
	int data_gpio_i2c;
	int data_gpio_pin;
	int data_supported_devices;
	int data_firmware_info;
	bool is_v2;
};

static void atombios_init_cmd_indices_v1(struct atombios_cmd_indices *idx)
{
	idx->is_v2 = false;
	idx->asic_init              = CMD_IDX_V1(ASIC_Init);
	idx->set_pixel_clock        = CMD_IDX_V1(SetPixelClock);
	idx->adjust_display_pll     = CMD_IDX_V1(AdjustDisplayPll);
	idx->set_dce_clock          = CMD_IDX_V1(SetDCEClock);
	idx->set_crtc_using_dtd     = CMD_IDX_V1(SetCRTC_UsingDTDTiming);
	idx->enable_crtc            = CMD_IDX_V1(EnableCRTC);
	idx->blank_crtc             = CMD_IDX_V1(BlankCRTC);
	idx->select_crtc_source     = CMD_IDX_V1(SelectCRTC_Source);
	idx->enable_disp_power_gating = CMD_IDX_V1(EnableDispPowerGating);
	idx->enable_scaler         = CMD_IDX_V1(EnableScaler);
	idx->set_crtc_overscan     = CMD_IDX_V1(SetCRTC_OverScan);
	idx->enable_yuv            = CMD_IDX_V1(EnableYUV);
	idx->dig_encoder_ctrl       = CMD_IDX_V1(DIGxEncoderControl);
	idx->dig1_xmtr_ctrl         = CMD_IDX_V1(DIG1TransmitterControl);
	idx->tmds1_encoder_ctrl     = CMD_IDX_V1(TMDSAEncoderControl);
	idx->tmds2_encoder_ctrl     = CMD_IDX_V1(LVTMAEncoderControl);
	idx->tmds1_output_ctrl      = CMD_IDX_V1(TMDSAOutputControl);
	idx->tmds2_output_ctrl      = CMD_IDX_V1(LVTMAOutputControl);
	idx->dac1_encoder_ctrl      = CMD_IDX_V1(DAC1EncoderControl);
	idx->dac2_encoder_ctrl      = CMD_IDX_V1(DAC2EncoderControl);
	idx->dac_load_detect        = CMD_IDX_V1(DAC_LoadDetection);
	idx->process_i2c            = CMD_IDX_V1(ProcessI2cChannelTransaction);
	idx->process_aux            = CMD_IDX_V1(ProcessAuxChannelTransaction);
	idx->data_object_header     = DATA_IDX_V1(Object_Header);
	idx->data_gpio_i2c          = DATA_IDX_V1(GPIO_I2C_Info);
	idx->data_gpio_pin          = DATA_IDX_V1(GPIO_Pin_LUT);
	idx->data_supported_devices = DATA_IDX_V1(SupportedDevicesInfo);
	idx->data_firmware_info     = DATA_IDX_V1(FirmwareInfo);
}

static void atombios_init_cmd_indices_v2(struct atombios_cmd_indices *idx)
{
	idx->is_v2 = true;
	idx->asic_init              = CMD_IDX_V2(asic_init);
	idx->set_pixel_clock        = CMD_IDX_V2(setpixelclock);
	idx->adjust_display_pll     = -1;
	idx->set_dce_clock          = CMD_IDX_V2(setdceclock);
	idx->set_crtc_using_dtd     = CMD_IDX_V2(setcrtc_usingdtdtiming);
	idx->enable_crtc            = CMD_IDX_V2(enablecrtc);
	idx->blank_crtc             = CMD_IDX_V2(blankcrtc);
	idx->select_crtc_source     = CMD_IDX_V2(selectcrtc_source);
	idx->enable_disp_power_gating = CMD_IDX_V2(enabledisppowergating);
	idx->enable_scaler         = -1;
	idx->set_crtc_overscan     = -1;
	idx->enable_yuv            = -1;
	idx->dig_encoder_ctrl       = CMD_IDX_V2(digxencodercontrol);
	idx->dig1_xmtr_ctrl         = CMD_IDX_V2(dig1transmittercontrol);
	idx->tmds1_encoder_ctrl     = -1;
	idx->tmds2_encoder_ctrl     = -1;
	idx->tmds1_output_ctrl      = -1;
	idx->tmds2_output_ctrl      = -1;
	idx->dac1_encoder_ctrl      = -1; /* No DAC on Vega+ */
	idx->dac2_encoder_ctrl      = -1;
	idx->dac_load_detect        = -1;
	idx->process_i2c            = CMD_IDX_V2(processi2cchanneltransaction);
	idx->process_aux            = CMD_IDX_V2(processauxchanneltransaction);
	idx->data_object_header     = DATA_IDX_V2(displayobjectinfo);
	idx->data_gpio_i2c          = DATA_IDX_V2(gpio_pin_lut);
	idx->data_gpio_pin          = DATA_IDX_V2(gpio_pin_lut);
	idx->data_supported_devices = -1;
	idx->data_firmware_info     = DATA_IDX_V2(firmwareinfo);
}

/*
 * Detect whether the VBIOS uses v1 (atombios.h) or v2 (atomfirmware.h)
 * master table format by reading the format_revision of the master
 * command table header.
 */
static bool atombios_detect_v2(struct atom_context *ctx)
{
	/* The master command table starts with a 4-byte common header.
	 * format_revision is at byte offset 2. v1 has revision 1.x,
	 * v2 has revision 2.x. */
	uint8_t frev = get_u8(ctx->bios, ctx->cmd_table + 2);
	return frev >= 2;
}

/* Global command index table, initialized during atombios_init() */
static struct atombios_cmd_indices cmd_idx;

/* Maximum EDID block size */
#define EDID_BLOCK_SIZE		128

/* I2C limits for ProcessI2cChannelTransaction */
#define ATOM_MAX_HW_I2C_WRITE	3
#define ATOM_MAX_HW_I2C_READ	255

/* DDC slave addresses */
#define DDC_EDID_ADDRESS	0x50

/* ---- DPCD register addresses ---- */
#define DP_DPCD_REV		0x000
#define DP_MAX_LINK_RATE	0x001
#define DP_MAX_LANE_COUNT	0x002
#define DP_MAX_DOWNSPREAD	0x003
#define DP_RECEIVER_CAP_SIZE	0x00F

#define DP_LINK_BW_SET		0x100
#define DP_LANE_COUNT_SET	0x101
#define DP_TRAINING_PATTERN_SET	0x102
#define DP_TRAINING_LANE0_SET	0x103

#define DP_DOWNSPREAD_CTRL	0x107
#define DP_EDP_CONFIGURATION_SET 0x10A

#define DP_LANE0_1_STATUS	0x202
#define DP_LANE2_3_STATUS	0x203
#define DP_LANE_ALIGN_STATUS_UPDATED 0x204
#define DP_ADJUST_REQUEST_LANE0_1 0x206
#define DP_ADJUST_REQUEST_LANE2_3 0x207

#define DP_SET_POWER		0x600

/* DP link bandwidth codes */
#define DP_LINK_BW_1_62		0x06
#define DP_LINK_BW_2_7		0x0a
#define DP_LINK_BW_5_4		0x14
#define DP_LINK_BW_8_1		0x1e

/* DP training patterns */
#define DP_TRAINING_PATTERN_DISABLE	0x00
#define DP_TRAINING_PATTERN_1		0x01
#define DP_TRAINING_PATTERN_2		0x02
#define DP_TRAINING_PATTERN_3		0x03

/* DP lane status bits */
#define DP_LANE_CR_DONE			(1 << 0)
#define DP_LANE_CHANNEL_EQ_DONE		(1 << 1)
#define DP_LANE_SYMBOL_LOCKED		(1 << 2)
#define DP_CHANNEL_EQ_BITS		(DP_LANE_CR_DONE | DP_LANE_CHANNEL_EQ_DONE | DP_LANE_SYMBOL_LOCKED)
#define DP_INTERLANE_ALIGN_DONE		(1 << 0)

/* DP lane training bits */
#define DP_TRAIN_VOLTAGE_SWING_MASK	0x03
#define DP_TRAIN_PRE_EMPHASIS_MASK	0x18
#define DP_TRAIN_PRE_EMPHASIS_SHIFT	3
#define DP_TRAIN_MAX_SWING_REACHED	(1 << 2)
#define DP_TRAIN_MAX_PRE_REACHED	(1 << 5)

/* DP misc */
#define DP_SPREAD_AMP_0_5		0x10
#define DP_LANE_COUNT_ENHANCED_FRAME_EN	(1 << 7)
#define DP_TPS3_SUPPORTED		(1 << 6)

/* DP power states */
#define DP_SET_POWER_D0			0x01
#define DP_SET_POWER_D3			0x02

/* DP AUX request types */
#define DP_AUX_NATIVE_WRITE		0x8
#define DP_AUX_NATIVE_READ		0x9
#define DP_AUX_HEADER_SIZE		4

/* DP link status size (0x202-0x207) */
#define DP_LINK_STATUS_SIZE		6

static uint32_t dp_link_bw_code_to_clock_khz(uint8_t link_bw)
{
	return (uint32_t)link_bw * 27000;
}

static uint16_t dp_link_bw_code_to_clock_10khz(uint8_t link_bw)
{
	return (uint16_t)((uint32_t)link_bw * 2700);
}

/* ---- Display path info discovered from ATOM tables ---- */

struct atombios_display_path {
	uint8_t  connector_type;	/* 1=HDMI, 2=DP/eDP, 3=DVI, 4=VGA, 5=DVI-I */
	uint8_t  encoder_obj_id;	/* ENCODER_OBJECT_ID_* */
	uint8_t  encoder_type;		/* GRAPH_OBJECT_ENUM_ID */
	uint8_t  i2c_line;		/* ATOM I2C line ID for DDC */
	uint8_t  i2c_valid;		/* DDC line was found in connector records */
	uint8_t  hpd_id;		/* normalized HPD pin (0-5, ATOMBIOS_HPD_NONE if none) */
	uint8_t  dig_encoder;		/* DIG index (0=DIG1, ...) for digital */
	uint8_t  phy_id;		/* UNIPHY ID (0=A, 1=B, ...) for digital */
	uint8_t  encoder_mode;		/* ATOM_ENCODER_MODE_* */
	uint8_t  dp_lane_count;		/* negotiated DP lane count */
	uint8_t  dp_link_bw;		/* DPCD DP_LINK_BW_* code */
	uint16_t connector_obj_id;	/* Full connector object ID */
	uint8_t  is_dac;		/* 1 if DAC encoder (VGA/CRT), 0 if digital */
	uint8_t  is_legacy_tmds;	/* 1 for pre-UNIPHY TMDS/LVTMA encoders */
	uint8_t  dac_type;		/* ATOM_DAC_A or ATOM_DAC_B */
	uint16_t device_tag;		/* ATOM_DEVICE_*_SUPPORT bit */
};

struct atombios_i2c_bus_rec {
	uint32_t mask_clk_reg;
	uint32_t mask_data_reg;
	uint32_t en_clk_reg;
	uint32_t en_data_reg;
	uint32_t y_clk_reg;
	uint32_t y_data_reg;
	uint32_t a_clk_reg;
	uint32_t a_data_reg;
	uint32_t mask_clk_mask;
	uint32_t mask_data_mask;
	uint32_t en_clk_mask;
	uint32_t en_data_mask;
	uint32_t y_clk_mask;
	uint32_t y_data_mask;
	uint32_t a_clk_mask;
	uint32_t a_data_mask;
	uint8_t i2c_id;
	uint8_t hw_capable;
	uint8_t valid;
};

enum atombios_mc_reg_layout {
	ATOMBIOS_MC_R600,
	ATOMBIOS_MC_R700,
};

enum atombios_scanout_layout {
	ATOMBIOS_SCANOUT_AVIVO,
	ATOMBIOS_SCANOUT_EVERGREEN,
};

enum atombios_si_pipe_config {
	ATOMBIOS_SI_PIPE_NONE = 0,
	ATOMBIOS_SI_PIPE_P4_8X16 = 4,
	ATOMBIOS_SI_PIPE_P8_32X32_8X16 = 10,
};

struct atombios_gpu_profile {
	enum atombios_mc_reg_layout mc;
	enum atombios_scanout_layout scanout;
	enum atombios_si_pipe_config si_pipe;
	uint8_t dispclk_ppll;
	uint8_t pixel_clock_ppll;
	bool use_set_dce_clock;
	bool use_combo_phy_pixel_clock;
	bool use_cik_desktop_height;
	bool use_tile_mode_pipe_config;
	bool config_memsize_in_mb;
	bool has_display;
	bool supported;
};

/* Forward declarations for functions used by DP link training */
static int atombios_dig_encoder_setup(struct atom_context *ctx,
				      const struct atombios_display_path *path,
				      uint16_t pixel_clock_10khz,
				      uint8_t action, uint8_t bpc);
static int atombios_transmitter_control(struct atom_context *ctx,
					const struct atombios_display_path *path,
					uint16_t pixel_clock_10khz,
					uint8_t action,
					uint8_t lane_num, uint8_t lane_set);

static struct atombios_gpu_profile atombios_get_gpu_profile(uint16_t device_id)
{
	struct atombios_gpu_profile profile = {
		.mc = ATOMBIOS_MC_R700,
		.scanout = ATOMBIOS_SCANOUT_EVERGREEN,
		.si_pipe = ATOMBIOS_SI_PIPE_NONE,
		.dispclk_ppll = ATOM_DCPLL,
		.pixel_clock_ppll = ATOM_PPLL1,
		.use_set_dce_clock = false,
		.use_combo_phy_pixel_clock = false,
		.use_cik_desktop_height = false,
		.use_tile_mode_pipe_config = false,
		.config_memsize_in_mb = true,
		.has_display = true,
		.supported = true,
	};

	switch (device_id) {
	/* R600/RV6xx/RS780/RS880 use R600 MC registers and AVIVO scanout. */
	case 0x9400: case 0x9401: case 0x9402: case 0x9403:
	case 0x9405: case 0x940A: case 0x940B: case 0x940F:
	case 0x94C0: case 0x94C1: case 0x94C3: case 0x94C4:
	case 0x94C5: case 0x94C6: case 0x94C7: case 0x94C8:
	case 0x94C9: case 0x94CB: case 0x94CC: case 0x94CD:
	case 0x9580: case 0x9581: case 0x9583: case 0x9586:
	case 0x9587: case 0x9588: case 0x9589: case 0x958A:
	case 0x958B: case 0x958C: case 0x958D: case 0x958E: case 0x958F:
	case 0x95C0: case 0x95C2: case 0x95C4: case 0x95C5:
	case 0x95C6: case 0x95C7: case 0x95C9: case 0x95CC:
	case 0x95CD: case 0x95CE: case 0x95CF:
	case 0x9590: case 0x9591: case 0x9593: case 0x9595:
	case 0x9596: case 0x9597: case 0x9598: case 0x9599: case 0x959B:
	case 0x9500: case 0x9501: case 0x9504: case 0x9505:
	case 0x9506: case 0x9507: case 0x9508: case 0x9509:
	case 0x950F: case 0x9511: case 0x9515: case 0x9517: case 0x9519:
	case 0x9610: case 0x9611: case 0x9612: case 0x9613:
	case 0x9614: case 0x9615: case 0x9616:
	case 0x9710: case 0x9711: case 0x9712: case 0x9713:
	case 0x9714: case 0x9715:
		profile.mc = ATOMBIOS_MC_R600;
		profile.scanout = ATOMBIOS_SCANOUT_AVIVO;
		profile.config_memsize_in_mb = false;
		break;

	/* RV7xx uses the R700 MC location but keeps the AVIVO display registers. */
	case 0x9440: case 0x9441: case 0x9442: case 0x9443:
	case 0x9444: case 0x9446: case 0x944A: case 0x944B:
	case 0x944C: case 0x944E: case 0x9450: case 0x9452:
	case 0x9456: case 0x945A: case 0x945B: case 0x945E:
	case 0x9460: case 0x9462: case 0x946A: case 0x946B:
	case 0x947A: case 0x947B:
	case 0x9480: case 0x9487: case 0x9488: case 0x9489:
	case 0x948A: case 0x948F: case 0x9490: case 0x9491:
	case 0x9495: case 0x9498: case 0x949C: case 0x949E: case 0x949F:
	case 0x9540: case 0x9541: case 0x9542: case 0x954E:
	case 0x954F: case 0x9552: case 0x9553: case 0x9555:
	case 0x9557: case 0x955F:
	case 0x94A0: case 0x94A1: case 0x94A3: case 0x94B1:
	case 0x94B3: case 0x94B4: case 0x94B5: case 0x94B9:
		profile.scanout = ATOMBIOS_SCANOUT_AVIVO;
		profile.config_memsize_in_mb = false;
		break;

	/* Palm/Sumo report the VRAM size register in bytes, not MiB. */
	case 0x9802: case 0x9803: case 0x9804: case 0x9805:
	case 0x9806: case 0x9807: case 0x9808: case 0x9809: case 0x980A:
	case 0x9640: case 0x9641: case 0x9642: case 0x9643:
	case 0x9644: case 0x9645: case 0x9647: case 0x9648:
	case 0x9649: case 0x964A: case 0x964B: case 0x964C:
	case 0x964E: case 0x964F:
		profile.config_memsize_in_mb = false;
		break;

	/* Southern Islands needs the SI pipe field in GRPH_CONTROL. */
	case 0x6780: case 0x6784: case 0x6788: case 0x678A:
	case 0x6790: case 0x6791: case 0x6792: case 0x6798:
	case 0x6799: case 0x679A: case 0x679B: case 0x679E: case 0x679F:
	case 0x6800: case 0x6801: case 0x6802: case 0x6806:
	case 0x6808: case 0x6809: case 0x6810: case 0x6811:
	case 0x6816: case 0x6817: case 0x6818: case 0x6819:
		profile.si_pipe = ATOMBIOS_SI_PIPE_P8_32X32_8X16;
		profile.dispclk_ppll = ATOM_PPLL0;
		break;
	case 0x6820: case 0x6821: case 0x6822: case 0x6823:
	case 0x6824: case 0x6825: case 0x6826: case 0x6827:
	case 0x6828: case 0x6829: case 0x682A: case 0x682B:
	case 0x682C: case 0x682D: case 0x682F: case 0x6830:
	case 0x6831: case 0x6835: case 0x6837: case 0x6838:
	case 0x6839: case 0x683B: case 0x683D: case 0x683F:
	case 0x6600: case 0x6601: case 0x6602: case 0x6603:
	case 0x6604: case 0x6605: case 0x6606: case 0x6607:
	case 0x6608: case 0x6610: case 0x6611: case 0x6613:
	case 0x6617: case 0x6620: case 0x6621: case 0x6623: case 0x6631:
		profile.si_pipe = ATOMBIOS_SI_PIPE_P4_8X16;
		profile.dispclk_ppll = ATOM_PPLL0;
		break;
	case 0x6660: case 0x6663: case 0x6664: case 0x6665:
	case 0x6667: case 0x666F:
		profile.has_display = false;
		profile.supported = false;
		break;

	/* CIK/DCE8 and VI/DCE10 use the CIK desktop-height register. */
	case 0x6640: case 0x6641: case 0x6646: case 0x6647: case 0x6649:
	case 0x6650: case 0x6651: case 0x6658: case 0x665C: case 0x665D: case 0x665F:
	case 0x1304: case 0x1305: case 0x1306: case 0x1307: case 0x1309: case 0x130A: case 0x130B:
	case 0x130C: case 0x130D: case 0x130E: case 0x130F: case 0x1310: case 0x1311: case 0x1312:
	case 0x1313: case 0x1315: case 0x1316: case 0x1317: case 0x1318: case 0x131B: case 0x131C: case 0x131D:
		profile.si_pipe = ATOMBIOS_SI_PIPE_P4_8X16;
		profile.dispclk_ppll = ATOM_EXT_PLL1;
		profile.pixel_clock_ppll = ATOM_PPLL2;
		profile.use_cik_desktop_height = true;
		profile.use_tile_mode_pipe_config = true;
		break;
	case 0x67A0: case 0x67A1: case 0x67A2: case 0x67A8: case 0x67A9: case 0x67AA:
	case 0x67B0: case 0x67B1: case 0x67B8: case 0x67B9: case 0x67BA: case 0x67BE:
	case 0x6920: case 0x6921: case 0x6928: case 0x6929: case 0x692B: case 0x692F:
	case 0x6930: case 0x6938: case 0x6939: case 0x7300: case 0x730F:
		profile.si_pipe = ATOMBIOS_SI_PIPE_P8_32X32_8X16;
		profile.dispclk_ppll = ATOM_EXT_PLL1;
		profile.pixel_clock_ppll = ATOM_PPLL2;
		profile.use_cik_desktop_height = true;
		profile.use_tile_mode_pipe_config = true;
		break;
	case 0x9830: case 0x9831: case 0x9832: case 0x9833: case 0x9834: case 0x9835: case 0x9836: case 0x9837:
	case 0x9838: case 0x9839: case 0x983A: case 0x983B: case 0x983C: case 0x983D: case 0x983E: case 0x983F:
	case 0x9850: case 0x9851: case 0x9852: case 0x9853: case 0x9854: case 0x9855: case 0x9856: case 0x9857:
	case 0x9858: case 0x9859: case 0x985A: case 0x985B: case 0x985C: case 0x985D: case 0x985E: case 0x985F:
		profile.dispclk_ppll = ATOM_EXT_PLL1;
		profile.pixel_clock_ppll = ATOM_PPLL2;
		profile.use_cik_desktop_height = true;
		profile.use_tile_mode_pipe_config = true;
		break;
	/* DCE11.0 uses the DCE clock manager but still programs DISPCLK via
	 * SetPixelClock, with DFS selected as ATOM_EXT_PLL1. */
	case 0x9870: case 0x9874: case 0x9875: case 0x9876: case 0x9877:
	case 0x98E4:
		profile.dispclk_ppll = ATOM_EXT_PLL1;
		profile.pixel_clock_ppll = ATOM_PPLL0;
		profile.use_cik_desktop_height = true;
		profile.use_tile_mode_pipe_config = true;
		break;
	case 0x67C0: case 0x67C1: case 0x67C2: case 0x67C4: case 0x67C7: case 0x67C8: case 0x67C9:
	case 0x67CA: case 0x67CC: case 0x67CF: case 0x67D0: case 0x67DF: case 0x6FDF:
	case 0x694C: case 0x694E: case 0x694F:
		profile.si_pipe = ATOMBIOS_SI_PIPE_P8_32X32_8X16;
		profile.dispclk_ppll = ATOM_GCK_DFS;
		profile.pixel_clock_ppll = ATOM_COMBOPHY_PLL0;
		profile.use_set_dce_clock = true;
		profile.use_combo_phy_pixel_clock = true;
		profile.use_cik_desktop_height = true;
		profile.use_tile_mode_pipe_config = true;
		break;
	case 0x67E0: case 0x67E1: case 0x67E3: case 0x67E7: case 0x67E8: case 0x67E9:
	case 0x67EB: case 0x67EF: case 0x67FF:
	case 0x6980: case 0x6981: case 0x6985: case 0x6986: case 0x6987: case 0x6995: case 0x6997: case 0x699F:
		profile.si_pipe = ATOMBIOS_SI_PIPE_P4_8X16;
		profile.dispclk_ppll = ATOM_GCK_DFS;
		profile.pixel_clock_ppll = ATOM_COMBOPHY_PLL0;
		profile.use_set_dce_clock = true;
		profile.use_combo_phy_pixel_clock = true;
		profile.use_cik_desktop_height = true;
		profile.use_tile_mode_pipe_config = true;
		break;
	/* Topaz has no DCE in Linux amdgpu. */
	case 0x6900: case 0x6901: case 0x6902: case 0x6903: case 0x6907:
		profile.has_display = false;
		profile.supported = false;
		break;
	}

	return profile;
}

/* ---- card_info MMIO callbacks ---- */

static void cail_reg_write(struct card_info *info, uint32_t reg, uint32_t val)
{
	write32((uint8_t *)info->mmio_base + reg * 4, val);
}

static uint32_t cail_reg_read(struct card_info *info, uint32_t reg)
{
	return read32((uint8_t *)info->mmio_base + reg * 4);
}

#define R600_BIOS_0_SCRATCH	0x1724
#define R600_BIOS_2_SCRATCH	0x172c
#define R600_BIOS_3_SCRATCH	0x1730
#define R600_BIOS_6_SCRATCH	0x173c

#define ATOMBIOS_HPD_NONE	0xff
#define AVIVO_DC_GPIO_HPD_A	0x7e94
#define EVERGREEN_DC_GPIO_HPD_A	0x64b4
#define SI_DC_GPIO_HPD_A	0x65b4

#define VGA_HDP_CONTROL			0x0328
#define VGA_MEMORY_DISABLE		(1 << 4)
#define R600_MC_VM_FB_LOCATION		0x2180
#define R600_MC_VM_SYSTEM_APERTURE_LOW_ADDR 0x2190
#define R600_MC_VM_SYSTEM_APERTURE_HIGH_ADDR 0x2194
#define R600_MC_VM_SYSTEM_APERTURE_DEFAULT_ADDR 0x2198
#define R700_MC_VM_FB_LOCATION		0x2024
#define R700_MC_VM_SYSTEM_APERTURE_LOW_ADDR 0x2034
#define R700_MC_VM_SYSTEM_APERTURE_HIGH_ADDR 0x2038
#define R700_MC_VM_SYSTEM_APERTURE_DEFAULT_ADDR 0x203c
#define HDP_NONSURFACE_BASE		0x2c04
#define HDP_NONSURFACE_INFO		0x2c08
#define HDP_NONSURFACE_INFO_32BPP	(2 << 7)
#define HDP_NONSURFACE_INFO_EG_LINEAR	(1 << 30)
#define HDP_NONSURFACE_SIZE		0x2c0c
#define MC_SHARED_BLACKOUT_CNTL		0x20ac
#define BLACKOUT_MODE_MASK		0x00000007
#define ATOMBIOS_CONFIG_MEMSIZE		0x5428
#define BIF_FB_EN			0x5490
#define FB_READ_EN			(1 << 0)
#define FB_WRITE_EN			(1 << 1)
#define HDP_MEM_COHERENCY_FLUSH_CNTL	0x5480

#define AVIVO_D1VGA_CONTROL			0x0330
#define AVIVO_D1CRTC_UPDATE_LOCK		0x60e8
#define AVIVO_D1MODE_MASTER_UPDATE_MODE		0x60e4
#define AVIVO_D1GRPH_ENABLE			0x6100
#define AVIVO_D1GRPH_CONTROL			0x6104
#define AVIVO_D1GRPH_LUT_SEL			0x6108
#define R600_D1GRPH_SWAP_CONTROL		0x610c
#define AVIVO_D1GRPH_PRIMARY_SURFACE_ADDRESS	0x6110
#define AVIVO_D1GRPH_SECONDARY_SURFACE_ADDRESS	0x6118
#define AVIVO_D1GRPH_PITCH			0x6120
#define AVIVO_D1GRPH_SURFACE_OFFSET_X		0x6124
#define AVIVO_D1GRPH_SURFACE_OFFSET_Y		0x6128
#define AVIVO_D1GRPH_X_START			0x612c
#define AVIVO_D1GRPH_Y_START			0x6130
#define AVIVO_D1GRPH_X_END			0x6134
#define AVIVO_D1GRPH_Y_END			0x6138
#define AVIVO_D1GRPH_FLIP_CONTROL		0x6148
#define AVIVO_D1MODE_DESKTOP_HEIGHT		0x652c
#define AVIVO_D1MODE_VIEWPORT_START		0x6580
#define AVIVO_D1MODE_VIEWPORT_SIZE		0x6584

#define AVIVO_D1GRPH_CONTROL_DEPTH_32BPP	(2 << 0)
#define AVIVO_D1GRPH_CONTROL_32BPP_ARGB8888	(0 << 8)
#define R600_D1GRPH_SWAP_ENDIAN_NONE		0

#define AVIVO_DC_LUT_RW_SELECT			0x6480
#define AVIVO_DC_LUT_RW_MODE			0x6484
#define AVIVO_DC_LUT_RW_INDEX			0x6488
#define AVIVO_DC_LUT_WRITE_EN_MASK		0x648c
#define AVIVO_DC_LUT_30_COLOR			0x6494
#define AVIVO_DC_LUTA_CONTROL			0x64c0
#define AVIVO_DC_LUTA_BLACK_OFFSET_BLUE		0x64c4
#define AVIVO_DC_LUTA_BLACK_OFFSET_GREEN	0x64c8
#define AVIVO_DC_LUTA_BLACK_OFFSET_RED		0x64cc
#define AVIVO_DC_LUTA_WHITE_OFFSET_BLUE		0x64d0
#define AVIVO_DC_LUTA_WHITE_OFFSET_GREEN	0x64d4
#define AVIVO_DC_LUTA_WHITE_OFFSET_RED		0x64d8

#define EVERGREEN_GRPH_ENABLE			0x6800
#define EVERGREEN_GRPH_CONTROL			0x6804
#define EVERGREEN_GRPH_LUT_10BIT_BYPASS_CONTROL 0x6808
#define EVERGREEN_GRPH_SWAP_CONTROL		0x680c
#define EVERGREEN_GRPH_PRIMARY_SURFACE_ADDRESS	0x6810
#define EVERGREEN_GRPH_SECONDARY_SURFACE_ADDRESS 0x6814
#define EVERGREEN_GRPH_PITCH			0x6818
#define EVERGREEN_GRPH_PRIMARY_SURFACE_ADDRESS_HIGH 0x681c
#define EVERGREEN_GRPH_SECONDARY_SURFACE_ADDRESS_HIGH 0x6820
#define EVERGREEN_GRPH_SURFACE_OFFSET_X		0x6824
#define EVERGREEN_GRPH_SURFACE_OFFSET_Y		0x6828
#define EVERGREEN_GRPH_X_START			0x682c
#define EVERGREEN_GRPH_Y_START			0x6830
#define EVERGREEN_GRPH_X_END			0x6834
#define EVERGREEN_GRPH_Y_END			0x6838
#define EVERGREEN_GRPH_FLIP_CONTROL		0x6848
#define EVERGREEN_DESKTOP_HEIGHT		0x6b04
#define CIK_LB_DESKTOP_HEIGHT			0x6b0c
#define EVERGREEN_VIEWPORT_START		0x6d70
#define EVERGREEN_VIEWPORT_SIZE			0x6d74
#define EVERGREEN_CRTC_CONTROL			0x6e70
#define EVERGREEN_CRTC_DISP_READ_REQUEST_DISABLE (1 << 24)
#define EVERGREEN_CRTC_BLANK_CONTROL		0x6e74
#define EVERGREEN_CRTC_BLANK_DATA_EN		(1 << 8)
#define EVERGREEN_CRTC_UPDATE_LOCK		0x6ed4
#define EVERGREEN_MASTER_UPDATE_LOCK		0x6ef4
#define EVERGREEN_MASTER_UPDATE_MODE		0x6ef8
#define GB_TILE_MODE0				0x9910
#define GB_TILE_MODE_SCANOUT_INDEX		10
#define GB_TILE_MODE_PIPE_CONFIG_SHIFT		6
#define GB_TILE_MODE_PIPE_CONFIG_MASK		0x1f
#define EVERGREEN_GRPH_DEPTH_32BPP		(2 << 0)
#define EVERGREEN_GRPH_FORMAT_ARGB8888		(0 << 8)
#define EVERGREEN_GRPH_ARRAY_LINEAR_GENERAL	(0 << 20)
#define EVERGREEN_GRPH_PIPE_CONFIG(x)		(((x) & 0x1f) << 24)
#define EVERGREEN_GRPH_SURFACE_ADDRESS_MASK	0xffffff00
#define EVERGREEN_GRPH_ENDIAN_NONE		0
#define EVERGREEN_DC_LUT_RW_MODE		0x69e0
#define EVERGREEN_DC_LUT_RW_INDEX		0x69e4
#define EVERGREEN_DC_LUT_30_COLOR		0x69f0
#define EVERGREEN_DC_LUT_WRITE_EN_MASK		0x69f8
#define EVERGREEN_DC_LUT_CONTROL		0x6a00
#define EVERGREEN_DC_LUT_BLACK_OFFSET_BLUE	0x6a04
#define EVERGREEN_DC_LUT_BLACK_OFFSET_GREEN	0x6a08
#define EVERGREEN_DC_LUT_BLACK_OFFSET_RED	0x6a0c
#define EVERGREEN_DC_LUT_WHITE_OFFSET_BLUE	0x6a10
#define EVERGREEN_DC_LUT_WHITE_OFFSET_GREEN	0x6a14
#define EVERGREEN_DC_LUT_WHITE_OFFSET_RED	0x6a18
#define EVERGREEN_PRESCALE_GRPH_CONTROL		0x68b4
#define EVERGREEN_PRESCALE_OVL_CONTROL		0x68c4
#define EVERGREEN_INPUT_CSC_CONTROL		0x68d4
#define EVERGREEN_OUTPUT_CSC_CONTROL		0x68f0
#define EVERGREEN_DEGAMMA_CONTROL		0x6960
#define EVERGREEN_GAMUT_REMAP_CONTROL		0x6964
#define EVERGREEN_REGAMMA_CONTROL		0x6a80
#define EVERGREEN_PRESCALE_BYPASS		(1 << 4)

static uint32_t atombios_scratch_read(struct atom_context *ctx, uint32_t reg)
{
	return read32((uint8_t *)ctx->card->mmio_base + reg);
}

static void atombios_scratch_write(struct atom_context *ctx, uint32_t reg,
					   uint32_t val)
{
	write32((uint8_t *)ctx->card->mmio_base + reg, val);
}

static uint32_t atombios_s0_connected_bit(uint16_t device_tag)
{
	switch (device_tag) {
	case ATOM_DEVICE_CRT1_SUPPORT: return ATOM_S0_CRT1_COLOR;
	case ATOM_DEVICE_CRT2_SUPPORT: return ATOM_S0_CRT2_COLOR;
	case ATOM_DEVICE_LCD1_SUPPORT: return ATOM_S0_LCD1;
	case ATOM_DEVICE_LCD2_SUPPORT: return ATOM_S0_LCD2;
	case ATOM_DEVICE_DFP1_SUPPORT: return ATOM_S0_DFP1;
	case ATOM_DEVICE_DFP2_SUPPORT: return ATOM_S0_DFP2;
	case ATOM_DEVICE_DFP3_SUPPORT: return ATOM_S0_DFP3;
	case ATOM_DEVICE_DFP4_SUPPORT: return ATOM_S0_DFP4;
	case ATOM_DEVICE_DFP5_SUPPORT: return ATOM_S0_DFP5;
	case ATOM_DEVICE_DFP6_SUPPORT: return ATOM_S0_DFP6;
	default: return 0;
	}
}

static uint32_t atombios_s2_dpms_bit(uint16_t device_tag)
{
	switch (device_tag) {
	case ATOM_DEVICE_CRT1_SUPPORT: return 0x00010000;
	case ATOM_DEVICE_LCD1_SUPPORT: return 0x00020000;
	case ATOM_DEVICE_TV1_SUPPORT:  return 0x00040000;
	case ATOM_DEVICE_DFP1_SUPPORT: return 0x00080000;
	case ATOM_DEVICE_CRT2_SUPPORT: return 0x00100000;
	case ATOM_DEVICE_LCD2_SUPPORT: return 0x00200000;
	case ATOM_DEVICE_DFP6_SUPPORT: return 0x00400000;
	case ATOM_DEVICE_DFP2_SUPPORT: return 0x00800000;
	case ATOM_DEVICE_CV_SUPPORT:   return 0x01000000;
	case ATOM_DEVICE_DFP3_SUPPORT: return 0x02000000;
	case ATOM_DEVICE_DFP4_SUPPORT: return 0x04000000;
	case ATOM_DEVICE_DFP5_SUPPORT: return 0x08000000;
	default: return 0;
	}
}

static uint32_t atombios_s3_active_bit(uint16_t device_tag)
{
	switch (device_tag) {
	case ATOM_DEVICE_CRT1_SUPPORT: return ATOM_S3_CRT1_ACTIVE;
	case ATOM_DEVICE_LCD1_SUPPORT: return ATOM_S3_LCD1_ACTIVE;
	case ATOM_DEVICE_TV1_SUPPORT:  return ATOM_S3_TV1_ACTIVE;
	case ATOM_DEVICE_DFP1_SUPPORT: return ATOM_S3_DFP1_ACTIVE;
	case ATOM_DEVICE_CRT2_SUPPORT: return ATOM_S3_CRT2_ACTIVE;
	case ATOM_DEVICE_LCD2_SUPPORT: return ATOM_S3_LCD2_ACTIVE;
	case ATOM_DEVICE_DFP6_SUPPORT: return ATOM_S3_DFP6_ACTIVE;
	case ATOM_DEVICE_DFP2_SUPPORT: return ATOM_S3_DFP2_ACTIVE;
	case ATOM_DEVICE_CV_SUPPORT:   return ATOM_S3_CV_ACTIVE;
	case ATOM_DEVICE_DFP3_SUPPORT: return ATOM_S3_DFP3_ACTIVE;
	case ATOM_DEVICE_DFP4_SUPPORT: return ATOM_S3_DFP4_ACTIVE;
	case ATOM_DEVICE_DFP5_SUPPORT: return ATOM_S3_DFP5_ACTIVE;
	default: return 0;
	}
}

static uint32_t atombios_s3_crtc_shift(uint16_t device_tag)
{
	switch (device_tag) {
	case ATOM_DEVICE_CRT1_SUPPORT: return 16;
	case ATOM_DEVICE_LCD1_SUPPORT: return 17;
	case ATOM_DEVICE_TV1_SUPPORT:  return 18;
	case ATOM_DEVICE_DFP1_SUPPORT: return 19;
	case ATOM_DEVICE_CRT2_SUPPORT: return 20;
	case ATOM_DEVICE_LCD2_SUPPORT: return 21;
	case ATOM_DEVICE_DFP6_SUPPORT: return 22;
	case ATOM_DEVICE_DFP2_SUPPORT: return 23;
	case ATOM_DEVICE_CV_SUPPORT:   return 24;
	case ATOM_DEVICE_DFP3_SUPPORT: return 25;
	case ATOM_DEVICE_DFP4_SUPPORT: return 26;
	case ATOM_DEVICE_DFP5_SUPPORT: return 27;
	default: return 0;
	}
}

static uint32_t atombios_s6_acc_req_bit(uint16_t device_tag)
{
	switch (device_tag) {
	case ATOM_DEVICE_CRT1_SUPPORT: return ATOM_S6_ACC_REQ_CRT1;
	case ATOM_DEVICE_LCD1_SUPPORT: return ATOM_S6_ACC_REQ_LCD1;
	case ATOM_DEVICE_TV1_SUPPORT:  return ATOM_S6_ACC_REQ_TV1;
	case ATOM_DEVICE_DFP1_SUPPORT: return ATOM_S6_ACC_REQ_DFP1;
	case ATOM_DEVICE_CRT2_SUPPORT: return ATOM_S6_ACC_REQ_CRT2;
	case ATOM_DEVICE_LCD2_SUPPORT: return ATOM_S6_ACC_REQ_LCD2;
	case ATOM_DEVICE_DFP6_SUPPORT: return ATOM_S6_ACC_REQ_DFP6;
	case ATOM_DEVICE_DFP2_SUPPORT: return ATOM_S6_ACC_REQ_DFP2;
	case ATOM_DEVICE_CV_SUPPORT:   return ATOM_S6_ACC_REQ_CV;
	case ATOM_DEVICE_DFP3_SUPPORT: return ATOM_S6_ACC_REQ_DFP3;
	case ATOM_DEVICE_DFP4_SUPPORT: return ATOM_S6_ACC_REQ_DFP4;
	case ATOM_DEVICE_DFP5_SUPPORT: return ATOM_S6_ACC_REQ_DFP5;
	default: return 0;
	}
}

static uint8_t atombios_object_id(uint16_t object_id)
{
	return (object_id & OBJECT_ID_MASK) >> OBJECT_ID_SHIFT;
}

static uint32_t atombios_mmio_read(struct atom_context *ctx, uint32_t reg)
{
	return read32((uint8_t *)ctx->card->mmio_base + reg);
}

static void atombios_mmio_write(struct atom_context *ctx, uint32_t reg,
				       uint32_t val)
{
	write32((uint8_t *)ctx->card->mmio_base + reg, val);
}

static void atombios_mmio_write8(struct atom_context *ctx, uint32_t reg,
					uint8_t val)
{
	write8((uint8_t *)ctx->card->mmio_base + reg, val);
}

static void atombios_load_identity_lut(struct atom_context *ctx, int crtc_id)
{
	int i;

	printk(BIOS_DEBUG, "ATOMBIOS: loading identity CRTC LUT\n");

	atombios_mmio_write(ctx, AVIVO_DC_LUTA_CONTROL, 0);
	atombios_mmio_write(ctx, AVIVO_DC_LUTA_BLACK_OFFSET_BLUE, 0);
	atombios_mmio_write(ctx, AVIVO_DC_LUTA_BLACK_OFFSET_GREEN, 0);
	atombios_mmio_write(ctx, AVIVO_DC_LUTA_BLACK_OFFSET_RED, 0);
	atombios_mmio_write(ctx, AVIVO_DC_LUTA_WHITE_OFFSET_BLUE, 0xffff);
	atombios_mmio_write(ctx, AVIVO_DC_LUTA_WHITE_OFFSET_GREEN, 0xffff);
	atombios_mmio_write(ctx, AVIVO_DC_LUTA_WHITE_OFFSET_RED, 0xffff);

	atombios_mmio_write(ctx, AVIVO_DC_LUT_RW_SELECT, crtc_id);
	atombios_mmio_write(ctx, AVIVO_DC_LUT_RW_MODE, 0);
	atombios_mmio_write(ctx, AVIVO_DC_LUT_WRITE_EN_MASK, 0x0000003f);
	atombios_mmio_write8(ctx, AVIVO_DC_LUT_RW_INDEX, 0);

	for (i = 0; i < 256; i++) {
		uint16_t v = (i << 8) | i;
		uint32_t color = ((v & 0xffc0) << 14) |
			((v & 0xffc0) << 4) | (v >> 6);

		atombios_mmio_write(ctx, AVIVO_DC_LUT_30_COLOR, color);
	}

	atombios_mmio_write(ctx, AVIVO_D1GRPH_LUT_SEL,
			   (atombios_mmio_read(ctx, AVIVO_D1GRPH_LUT_SEL) & ~1) |
			   (crtc_id & 1));
}

static uint64_t atombios_vram_size(struct atom_context *ctx,
					   const struct atombios_gpu_profile *profile)
{
	uint32_t raw_size = atombios_mmio_read(ctx, ATOMBIOS_CONFIG_MEMSIZE);
	uint64_t size = raw_size;

	/* R600/R700 report bytes. Evergreen and newer discrete GPUs report MiB. */
	if (profile->config_memsize_in_mb && raw_size != 0xffffffff)
		size *= 1024ULL * 1024ULL;

	if (!size || raw_size == 0xffffffff)
		size = 256ULL * 1024ULL * 1024ULL;
	return size;
}

static uint32_t atombios_mc_fb_location_reg(const struct atombios_gpu_profile *profile)
{
	return profile->mc == ATOMBIOS_MC_R600 ? R600_MC_VM_FB_LOCATION : R700_MC_VM_FB_LOCATION;
}

static void atombios_mc_regs(const struct atombios_gpu_profile *profile,
				     uint32_t *fb_location, uint32_t *low,
				     uint32_t *high, uint32_t *default_addr)
{
	if (profile->mc == ATOMBIOS_MC_R600) {
		*fb_location = R600_MC_VM_FB_LOCATION;
		*low = R600_MC_VM_SYSTEM_APERTURE_LOW_ADDR;
		*high = R600_MC_VM_SYSTEM_APERTURE_HIGH_ADDR;
		*default_addr = R600_MC_VM_SYSTEM_APERTURE_DEFAULT_ADDR;
	} else {
		*fb_location = R700_MC_VM_FB_LOCATION;
		*low = R700_MC_VM_SYSTEM_APERTURE_LOW_ADDR;
		*high = R700_MC_VM_SYSTEM_APERTURE_HIGH_ADDR;
		*default_addr = R700_MC_VM_SYSTEM_APERTURE_DEFAULT_ADDR;
	}
}

static void atombios_program_mc_aperture(struct atom_context *ctx,
					 const struct atombios_gpu_profile *profile)
{
	uint32_t fb_location_reg, aperture_low_reg, aperture_high_reg, default_addr_reg;
	uint32_t old_fb;
	uint64_t vram_size = atombios_vram_size(ctx, profile);
	uint64_t vram_start = 0;
	uint64_t vram_end = vram_size - 1;
	uint32_t fb_location;

	atombios_mc_regs(profile, &fb_location_reg, &aperture_low_reg,
			 &aperture_high_reg, &default_addr_reg);
	old_fb = atombios_mmio_read(ctx, fb_location_reg);

	/* Match Linux MC programming for discrete PCIe cards: GPU VRAM
	 * addresses start at 0, while the CPU accesses VRAM through PCI BAR0. */
	fb_location = ((vram_end >> 24) & 0xffff) << 16;
	fb_location |= (vram_start >> 24) & 0xffff;

	atombios_mmio_write(ctx, VGA_HDP_CONTROL, VGA_MEMORY_DISABLE);
	atombios_mmio_write(ctx, aperture_low_reg, vram_start >> 12);
	atombios_mmio_write(ctx, aperture_high_reg, vram_end >> 12);
	atombios_mmio_write(ctx, default_addr_reg, 0);
	atombios_mmio_write(ctx, fb_location_reg, fb_location);
	atombios_mmio_write(ctx, HDP_NONSURFACE_BASE, vram_start >> 8);
	atombios_mmio_write(ctx, HDP_NONSURFACE_INFO, HDP_NONSURFACE_INFO_32BPP |
		(profile->scanout == ATOMBIOS_SCANOUT_EVERGREEN ?
		 HDP_NONSURFACE_INFO_EG_LINEAR : 0));
	atombios_mmio_write(ctx, HDP_NONSURFACE_SIZE, 0x3fffffff);

	if (profile->scanout == ATOMBIOS_SCANOUT_EVERGREEN) {
		uint32_t blackout = atombios_mmio_read(ctx, MC_SHARED_BLACKOUT_CNTL);

		atombios_mmio_write(ctx, MC_SHARED_BLACKOUT_CNTL,
				   blackout & ~BLACKOUT_MODE_MASK);
		atombios_mmio_write(ctx, BIF_FB_EN, FB_READ_EN | FB_WRITE_EN);
	}

	printk(BIOS_INFO,
	       "ATOMBIOS: MC VRAM %llu MiB layout=%s old_fb=0x%08x new_fb=0x%08x hdp_base=0x%08x hdp_info=0x%08x\n",
	       (unsigned long long)(vram_size / (1024ULL * 1024ULL)),
	       profile->mc == ATOMBIOS_MC_R600 ? "R600" : "R700+",
	       old_fb, fb_location, atombios_mmio_read(ctx, HDP_NONSURFACE_BASE),
	       atombios_mmio_read(ctx, HDP_NONSURFACE_INFO));
}

static void atombios_load_evergreen_identity_lut(struct atom_context *ctx)
{
	int i;

	printk(BIOS_DEBUG, "ATOMBIOS: loading DCE4+ identity CRTC LUT\n");

	/* Match Linux's DCE color setup: scanout pixels are already RGB,
	 * so bypass input/output CSC, prescale, degamma, gamut remap, and
	 * regamma.  Some VBIOS tables leave output CSC in a YCbCr mode,
	 * which makes black scanout data appear green. */
	atombios_mmio_write(ctx, EVERGREEN_INPUT_CSC_CONTROL, 0);
	atombios_mmio_write(ctx, EVERGREEN_OUTPUT_CSC_CONTROL, 0);
	atombios_mmio_write(ctx, EVERGREEN_PRESCALE_GRPH_CONTROL,
			   EVERGREEN_PRESCALE_BYPASS);
	atombios_mmio_write(ctx, EVERGREEN_PRESCALE_OVL_CONTROL,
			   EVERGREEN_PRESCALE_BYPASS);
	atombios_mmio_write(ctx, EVERGREEN_DEGAMMA_CONTROL, 0);
	atombios_mmio_write(ctx, EVERGREEN_GAMUT_REMAP_CONTROL, 0);
	atombios_mmio_write(ctx, EVERGREEN_REGAMMA_CONTROL, 0);

	atombios_mmio_write(ctx, EVERGREEN_DC_LUT_CONTROL, 0);
	atombios_mmio_write(ctx, EVERGREEN_DC_LUT_BLACK_OFFSET_BLUE, 0);
	atombios_mmio_write(ctx, EVERGREEN_DC_LUT_BLACK_OFFSET_GREEN, 0);
	atombios_mmio_write(ctx, EVERGREEN_DC_LUT_BLACK_OFFSET_RED, 0);
	atombios_mmio_write(ctx, EVERGREEN_DC_LUT_WHITE_OFFSET_BLUE, 0xffff);
	atombios_mmio_write(ctx, EVERGREEN_DC_LUT_WHITE_OFFSET_GREEN, 0xffff);
	atombios_mmio_write(ctx, EVERGREEN_DC_LUT_WHITE_OFFSET_RED, 0xffff);
	atombios_mmio_write(ctx, EVERGREEN_DC_LUT_RW_MODE, 0);
	atombios_mmio_write(ctx, EVERGREEN_DC_LUT_WRITE_EN_MASK, 0x0000003f);
	atombios_mmio_write(ctx, EVERGREEN_DC_LUT_RW_INDEX, 0);

	for (i = 0; i < 256; i++) {
		uint16_t v = (i << 8) | i;
		uint32_t color = ((v & 0xffc0) << 14) |
			((v & 0xffc0) << 4) | (v >> 6);

		atombios_mmio_write(ctx, EVERGREEN_DC_LUT_30_COLOR, color);
	}
}

static void atombios_program_avivo_framebuffer(struct atom_context *ctx,
					       const struct atombios_gpu_profile *profile,
					       resource_t fb_bar,
					       const struct edid_mode *mode)
{
	uint32_t width = mode->ha;
	uint32_t height = mode->va;
	uint32_t pitch_pixels = width;
	uint32_t fb_format = AVIVO_D1GRPH_CONTROL_DEPTH_32BPP |
		AVIVO_D1GRPH_CONTROL_32BPP_ARGB8888;
	uint32_t surface_addr = 0;

	atombios_program_mc_aperture(ctx, profile);

	printk(BIOS_INFO,
	       "ATOMBIOS: programming AVIVO scanout %ux%u pitch=%u gpu_addr=0x%08x cpu_addr=0x%llx mc_fb=0x%08x\n",
	       width, height, pitch_pixels, surface_addr,
	       (unsigned long long)fb_bar,
	       atombios_mmio_read(ctx, atombios_mc_fb_location_reg(profile)));

	memset((void *)(uintptr_t)fb_bar, 0, width * height * BYTES_PER_PIXEL);
	atombios_mmio_write(ctx, HDP_MEM_COHERENCY_FLUSH_CNTL, 1);

	atombios_mmio_write(ctx, AVIVO_D1VGA_CONTROL, 0);
	atombios_mmio_write(ctx, AVIVO_D1CRTC_UPDATE_LOCK, 1);
	atombios_mmio_write(ctx, AVIVO_D1GRPH_FLIP_CONTROL, 0);
	atombios_mmio_write(ctx, AVIVO_D1GRPH_PRIMARY_SURFACE_ADDRESS, surface_addr);
	atombios_mmio_write(ctx, AVIVO_D1GRPH_SECONDARY_SURFACE_ADDRESS, surface_addr);
	atombios_mmio_write(ctx, AVIVO_D1GRPH_CONTROL, fb_format);
	atombios_mmio_write(ctx, R600_D1GRPH_SWAP_CONTROL, R600_D1GRPH_SWAP_ENDIAN_NONE);
	atombios_load_identity_lut(ctx, 0);
	atombios_mmio_write(ctx, AVIVO_D1GRPH_SURFACE_OFFSET_X, 0);
	atombios_mmio_write(ctx, AVIVO_D1GRPH_SURFACE_OFFSET_Y, 0);
	atombios_mmio_write(ctx, AVIVO_D1GRPH_X_START, 0);
	atombios_mmio_write(ctx, AVIVO_D1GRPH_Y_START, 0);
	atombios_mmio_write(ctx, AVIVO_D1GRPH_X_END, width);
	atombios_mmio_write(ctx, AVIVO_D1GRPH_Y_END, height);
	atombios_mmio_write(ctx, AVIVO_D1GRPH_PITCH, pitch_pixels);
	atombios_mmio_write(ctx, AVIVO_D1GRPH_ENABLE, 1);
	atombios_mmio_write(ctx, AVIVO_D1MODE_DESKTOP_HEIGHT, height);
	atombios_mmio_write(ctx, AVIVO_D1MODE_VIEWPORT_START, 0);
	atombios_mmio_write(ctx, AVIVO_D1MODE_VIEWPORT_SIZE,
			   (width << 16) | ((height + 1) & ~1u));
	atombios_mmio_write(ctx, AVIVO_D1MODE_MASTER_UPDATE_MODE, 3);
	atombios_mmio_write(ctx, AVIVO_D1CRTC_UPDATE_LOCK, 0);
}

static void atombios_program_evergreen_framebuffer(struct atom_context *ctx,
						   const struct atombios_gpu_profile *profile,
						   resource_t fb_bar,
						   const struct edid_mode *mode)
{
	uint32_t width = mode->ha;
	uint32_t height = mode->va;
	uint32_t pitch_pixels = width;
	uint32_t surface_addr = 0;
	uint32_t pipe_config = profile->si_pipe;
	uint32_t fb_format = EVERGREEN_GRPH_DEPTH_32BPP |
		EVERGREEN_GRPH_FORMAT_ARGB8888 |
		EVERGREEN_GRPH_ARRAY_LINEAR_GENERAL;

	if (profile->use_tile_mode_pipe_config) {
		uint32_t tile_mode = atombios_mmio_read(ctx, GB_TILE_MODE0 +
			GB_TILE_MODE_SCANOUT_INDEX * sizeof(uint32_t));
		uint32_t reg_pipe = (tile_mode >> GB_TILE_MODE_PIPE_CONFIG_SHIFT) &
			GB_TILE_MODE_PIPE_CONFIG_MASK;

		if (reg_pipe)
			pipe_config = reg_pipe;
	}

	if (pipe_config != ATOMBIOS_SI_PIPE_NONE)
		fb_format |= EVERGREEN_GRPH_PIPE_CONFIG(pipe_config);

	atombios_program_mc_aperture(ctx, profile);

	printk(BIOS_INFO,
	       "ATOMBIOS: programming DCE4+ scanout %ux%u pitch=%u gpu_addr=0x%08x cpu_addr=0x%llx mc_fb=0x%08x pipe=%d\n",
	       width, height, pitch_pixels, surface_addr,
	       (unsigned long long)fb_bar,
	       atombios_mmio_read(ctx, atombios_mc_fb_location_reg(profile)),
	       pipe_config);

	memset((void *)(uintptr_t)fb_bar, 0, width * height * BYTES_PER_PIXEL);
	atombios_mmio_write(ctx, HDP_MEM_COHERENCY_FLUSH_CNTL, 1);

	atombios_mmio_write(ctx, AVIVO_D1VGA_CONTROL, 0);
	atombios_mmio_write(ctx, EVERGREEN_MASTER_UPDATE_LOCK, 1);
	atombios_mmio_write(ctx, EVERGREEN_GRPH_FLIP_CONTROL, 0);
	atombios_mmio_write(ctx, EVERGREEN_GRPH_PRIMARY_SURFACE_ADDRESS_HIGH, 0);
	atombios_mmio_write(ctx, EVERGREEN_GRPH_SECONDARY_SURFACE_ADDRESS_HIGH, 0);
	atombios_mmio_write(ctx, EVERGREEN_GRPH_PRIMARY_SURFACE_ADDRESS,
			   surface_addr & EVERGREEN_GRPH_SURFACE_ADDRESS_MASK);
	atombios_mmio_write(ctx, EVERGREEN_GRPH_SECONDARY_SURFACE_ADDRESS,
			   surface_addr & EVERGREEN_GRPH_SURFACE_ADDRESS_MASK);
	atombios_mmio_write(ctx, EVERGREEN_GRPH_CONTROL, fb_format);
	atombios_mmio_write(ctx, EVERGREEN_GRPH_SWAP_CONTROL, EVERGREEN_GRPH_ENDIAN_NONE);
	atombios_load_evergreen_identity_lut(ctx);
	atombios_mmio_write(ctx, EVERGREEN_GRPH_SURFACE_OFFSET_X, 0);
	atombios_mmio_write(ctx, EVERGREEN_GRPH_SURFACE_OFFSET_Y, 0);
	atombios_mmio_write(ctx, EVERGREEN_GRPH_X_START, 0);
	atombios_mmio_write(ctx, EVERGREEN_GRPH_Y_START, 0);
	atombios_mmio_write(ctx, EVERGREEN_GRPH_X_END, width);
	atombios_mmio_write(ctx, EVERGREEN_GRPH_Y_END, height);
	atombios_mmio_write(ctx, EVERGREEN_GRPH_PITCH, pitch_pixels);
	atombios_mmio_write(ctx, EVERGREEN_GRPH_ENABLE, 1);
	atombios_mmio_write(ctx, profile->use_cik_desktop_height ?
			   CIK_LB_DESKTOP_HEIGHT : EVERGREEN_DESKTOP_HEIGHT, height);
	atombios_mmio_write(ctx, EVERGREEN_VIEWPORT_START, 0);
	atombios_mmio_write(ctx, EVERGREEN_VIEWPORT_SIZE,
			   (width << 16) | ((height + 1) & ~1u));
	atombios_mmio_write(ctx, EVERGREEN_MASTER_UPDATE_MODE, 0);
	atombios_mmio_write(ctx, EVERGREEN_MASTER_UPDATE_LOCK, 0);
}

static void atombios_program_framebuffer(struct atom_context *ctx,
					 const struct atombios_gpu_profile *profile,
					 resource_t fb_bar,
					 const struct edid_mode *mode)
{
	if (profile->scanout == ATOMBIOS_SCANOUT_AVIVO)
		atombios_program_avivo_framebuffer(ctx, profile, fb_bar, mode);
	else
		atombios_program_evergreen_framebuffer(ctx, profile, fb_bar, mode);
}

static void atombios_evergreen_unblank_scanout(struct atom_context *ctx,
					       const struct atombios_gpu_profile *profile)
{
	uint32_t crtc_control;
	uint32_t blank_control;

	if (profile->scanout != ATOMBIOS_SCANOUT_EVERGREEN)
		return;

	crtc_control = atombios_mmio_read(ctx, EVERGREEN_CRTC_CONTROL);
	blank_control = atombios_mmio_read(ctx, EVERGREEN_CRTC_BLANK_CONTROL);

	atombios_mmio_write(ctx, EVERGREEN_CRTC_UPDATE_LOCK, 1);
	atombios_mmio_write(ctx, EVERGREEN_CRTC_BLANK_CONTROL,
			   blank_control & ~EVERGREEN_CRTC_BLANK_DATA_EN);
	atombios_mmio_write(ctx, EVERGREEN_CRTC_CONTROL,
			   crtc_control & ~EVERGREEN_CRTC_DISP_READ_REQUEST_DISABLE);
	atombios_mmio_write(ctx, EVERGREEN_CRTC_UPDATE_LOCK, 0);

	printk(BIOS_DEBUG,
	       "ATOMBIOS: CRTC ctrl=0x%08x blank=0x%08x after unblank\n",
	       atombios_mmio_read(ctx, EVERGREEN_CRTC_CONTROL),
	       atombios_mmio_read(ctx, EVERGREEN_CRTC_BLANK_CONTROL));
}

static void atombios_output_lock(struct atom_context *ctx, bool lock)
{
	uint32_t scratch = atombios_scratch_read(ctx, R600_BIOS_6_SCRATCH);

	if (lock) {
		scratch |= ATOM_S6_CRITICAL_STATE;
		scratch &= ~ATOM_S6_ACC_MODE;
	} else {
		scratch &= ~ATOM_S6_CRITICAL_STATE;
		scratch |= ATOM_S6_ACC_MODE;
	}

	atombios_scratch_write(ctx, R600_BIOS_6_SCRATCH, scratch);
}

static void atombios_update_scratch_for_path(struct atom_context *ctx,
					     const struct atombios_display_path *path,
					     int crtc_id, bool connected, bool dpms_on)
{
	uint32_t s0 = atombios_scratch_read(ctx, R600_BIOS_0_SCRATCH);
	uint32_t s2 = atombios_scratch_read(ctx, R600_BIOS_2_SCRATCH);
	uint32_t s3 = atombios_scratch_read(ctx, R600_BIOS_3_SCRATCH);
	uint32_t s6 = atombios_scratch_read(ctx, R600_BIOS_6_SCRATCH);
	uint32_t s0_bit = atombios_s0_connected_bit(path->device_tag);
	uint32_t s2_bit = atombios_s2_dpms_bit(path->device_tag);
	uint32_t s3_bit = atombios_s3_active_bit(path->device_tag);
	uint32_t s6_bit = atombios_s6_acc_req_bit(path->device_tag);
	uint32_t shift = atombios_s3_crtc_shift(path->device_tag);

	if (connected) {
		s0 |= s0_bit;
		s3 |= s3_bit;
		s6 |= s6_bit;
	} else {
		s0 &= ~s0_bit;
		s3 &= ~s3_bit;
		s6 &= ~s6_bit;
	}

	if (dpms_on)
		s2 &= ~s2_bit;
	else
		s2 |= s2_bit;

	if (shift) {
		s3 &= ~(1u << shift);
		s3 |= (uint32_t)crtc_id << shift;
	}

	atombios_scratch_write(ctx, R600_BIOS_0_SCRATCH, s0);
	atombios_scratch_write(ctx, R600_BIOS_2_SCRATCH, s2);
	atombios_scratch_write(ctx, R600_BIOS_3_SCRATCH, s3);
	atombios_scratch_write(ctx, R600_BIOS_6_SCRATCH, s6);
}

#define RV515_MC_IND_INDEX	0x70
#define RV515_MC_IND_DATA	0x74
#define RV515_MC_IND_WR_EN	(1 << 24)

static void cail_mc_write(struct card_info *info, uint32_t reg, uint32_t val)
{
	write32((uint8_t *)info->mmio_base + RV515_MC_IND_INDEX,
		0xff0000 | (reg & 0xffff) | RV515_MC_IND_WR_EN);
	write32((uint8_t *)info->mmio_base + RV515_MC_IND_DATA, val);
	write32((uint8_t *)info->mmio_base + RV515_MC_IND_INDEX, 0);
}

static uint32_t cail_mc_read(struct card_info *info, uint32_t reg)
{
	uint32_t val;

	write32((uint8_t *)info->mmio_base + RV515_MC_IND_INDEX,
		0x7f0000 | (reg & 0xffff));
	val = read32((uint8_t *)info->mmio_base + RV515_MC_IND_DATA);
	write32((uint8_t *)info->mmio_base + RV515_MC_IND_INDEX, 0);
	return val;
}

#define RADEON_CLOCK_CNTL_DATA	0x0c
#define RADEON_CLOCK_CNTL_INDEX	0x08
#define RADEON_PLL_WR_EN	(1 << 7)

static void cail_pll_write(struct card_info *info, uint32_t reg, uint32_t val)
{
	write8((uint8_t *)info->mmio_base + RADEON_CLOCK_CNTL_INDEX,
	       (reg & 0x3f) | RADEON_PLL_WR_EN);
	(void)read32((uint8_t *)info->mmio_base + RADEON_CLOCK_CNTL_DATA);
	write32((uint8_t *)info->mmio_base + RADEON_CLOCK_CNTL_DATA, val);
}

static uint32_t cail_pll_read(struct card_info *info, uint32_t reg)
{
	write8((uint8_t *)info->mmio_base + RADEON_CLOCK_CNTL_INDEX,
	       reg & 0x3f);
	return read32((uint8_t *)info->mmio_base + RADEON_CLOCK_CNTL_DATA);
}

/* ---- VBIOS loading ---- */

/*
 * Try to load VBIOS from CBFS first (pciVVVV,DDDD.rom), then fall back
 * to reading from the PCI expansion ROM BAR.
 */
static void *load_vbios(struct device *dev)
{
	void *rom = NULL;
	size_t rom_size = 0;
	char cbfs_name[24];
	uint16_t vendor = dev->vendor;
	uint16_t device_id = dev->device;
	uint16_t rom_size_field;

	snprintf(cbfs_name, sizeof(cbfs_name), "pci%04x,%04x.rom",
		 vendor, device_id);

	rom = cbfs_map(cbfs_name, &rom_size);
	if (rom) {
		printk(BIOS_INFO, "ATOMBIOS: loaded VBIOS from CBFS: %s (%zu bytes)\n",
		       cbfs_name, rom_size);
		return rom;
	}

	/* Try PCI expansion ROM BAR */
	uint32_t rom_bar = pci_read_config32(dev, PCI_ROM_ADDRESS);
	if (!(rom_bar & 1)) {
		/* Enable ROM decode */
		pci_write_config32(dev, PCI_ROM_ADDRESS, rom_bar | 1);
		rom_bar = pci_read_config32(dev, PCI_ROM_ADDRESS);
	}
	rom_bar &= PCI_ROM_ADDRESS_MASK;
	if (!rom_bar) {
		printk(BIOS_ERR, "ATOMBIOS: no VBIOS found in CBFS or ROM BAR\n");
		return NULL;
	}

	/* Validate ROM magic */
	if (read16((void *)(uintptr_t)rom_bar) != 0xAA55) {
		printk(BIOS_ERR, "ATOMBIOS: ROM BAR has no valid BIOS signature\n");
		pci_write_config32(dev, PCI_ROM_ADDRESS, rom_bar & ~1u);
		return NULL;
	}

	/* Read ROM size from byte 2 (in 512-byte units) */
	rom_size_field = read8((void *)(uintptr_t)(rom_bar + 2));
	rom_size = rom_size_field * 512;
	if (rom_size == 0 || rom_size > 512 * 1024)
		rom_size = 128 * 1024; /* sane default */

	/* Copy to RAM so we can disable the ROM BAR */
	rom = malloc(rom_size);
	if (!rom) {
		printk(BIOS_ERR, "ATOMBIOS: failed to allocate %zu bytes for VBIOS\n",
		       rom_size);
		return NULL;
	}
	memcpy(rom, (void *)(uintptr_t)rom_bar, rom_size);

	/* Disable ROM BAR */
	pci_write_config32(dev, PCI_ROM_ADDRESS, rom_bar & ~1u);

	printk(BIOS_INFO, "ATOMBIOS: loaded VBIOS from PCI ROM BAR (%zu bytes)\n",
	       rom_size);
	return rom;
}

/* ---- ATOM table execution helpers ---- */

static int atombios_exec(struct atom_context *ctx, int index, uint32_t *params,
			 int params_size)
{
	uint8_t frev, crev;

	if (!atom_parse_cmd_header(ctx, index, &frev, &crev)) {
		printk(BIOS_DEBUG, "ATOMBIOS: command table %d not present\n", index);
		return -1;
	}

	printk(BIOS_DEBUG, "ATOMBIOS: executing table %d (frev=%d crev=%d)\n",
	       index, frev, crev);
	return atom_execute_table(ctx, index, params, params_size);
}

/* ---- Linux-style GPIO DDC bit banging ---- */

static void atombios_gpio_i2c_lookup_quirks(ATOM_GPIO_I2C_ASSIGMENT *gpio,
					    uint8_t index)
{
	/* Linux radeon has this DCE3 quirk for bad GPIO_I2C_Info data. */
	if (index == 4 && gpio->usClkMaskRegisterIndex == 0x1fda &&
	    gpio->sucI2cId.ucAccess == 0x94)
		gpio->sucI2cId.ucAccess = 0x14;
}

static void atombios_i2c_bus_from_gpio(struct atombios_i2c_bus_rec *bus,
				       const ATOM_GPIO_I2C_ASSIGMENT *gpio)
{
	memset(bus, 0, sizeof(*bus));

	bus->mask_clk_reg = gpio->usClkMaskRegisterIndex * 4;
	bus->mask_data_reg = gpio->usDataMaskRegisterIndex * 4;
	bus->en_clk_reg = gpio->usClkEnRegisterIndex * 4;
	bus->en_data_reg = gpio->usDataEnRegisterIndex * 4;
	bus->y_clk_reg = gpio->usClkY_RegisterIndex * 4;
	bus->y_data_reg = gpio->usDataY_RegisterIndex * 4;
	bus->a_clk_reg = gpio->usClkA_RegisterIndex * 4;
	bus->a_data_reg = gpio->usDataA_RegisterIndex * 4;
	bus->mask_clk_mask = 1u << gpio->ucClkMaskShift;
	bus->mask_data_mask = 1u << gpio->ucDataMaskShift;
	bus->en_clk_mask = 1u << gpio->ucClkEnShift;
	bus->en_data_mask = 1u << gpio->ucDataEnShift;
	bus->y_clk_mask = 1u << gpio->ucClkY_Shift;
	bus->y_data_mask = 1u << gpio->ucDataY_Shift;
	bus->a_clk_mask = 1u << gpio->ucClkA_Shift;
	bus->a_data_mask = 1u << gpio->ucDataA_Shift;
	bus->i2c_id = gpio->sucI2cId.ucAccess;
	bus->hw_capable = !!(gpio->sucI2cId.ucAccess &
		ATOM_DEVICE_I2C_HARDWARE_CAP_MASK);
	bus->valid = !!bus->mask_clk_reg;
}

static int atombios_lookup_i2c_gpio(struct atom_context *ctx, uint8_t i2c_id,
				    struct atombios_i2c_bus_rec *bus)
{
	ATOM_GPIO_I2C_INFO *i2c_info;
	ATOM_GPIO_I2C_ASSIGMENT *gpio;
	uint16_t data_offset, size;
	int num_indices;
	int i;

	memset(bus, 0, sizeof(*bus));

	if (!atom_parse_data_header(ctx, cmd_idx.data_gpio_i2c, &size, NULL,
				    NULL, &data_offset))
		return -1;

	i2c_info = (ATOM_GPIO_I2C_INFO *)((uint8_t *)ctx->bios + data_offset);
	num_indices = (size - sizeof(ATOM_COMMON_TABLE_HEADER)) /
		sizeof(ATOM_GPIO_I2C_ASSIGMENT);
	gpio = &i2c_info->asGPIO_Info[0];

	for (i = 0; i < num_indices; i++) {
		atombios_gpio_i2c_lookup_quirks(gpio, i);
		if (gpio->sucI2cId.ucAccess == i2c_id) {
			atombios_i2c_bus_from_gpio(bus, gpio);
			return bus->valid ? 0 : -1;
		}
		gpio = (ATOM_GPIO_I2C_ASSIGMENT *)
			((uint8_t *)gpio + sizeof(ATOM_GPIO_I2C_ASSIGMENT));
	}

	return -1;
}

static void atombios_gpio_i2c_pre_xfer(struct atom_context *ctx,
				       const struct atombios_i2c_bus_rec *bus)
{
	uint32_t temp;

	/* Linux switches DCE3 hardware-capable pads to DDC/GPIO mode. */
	if (bus->hw_capable) {
		temp = atombios_mmio_read(ctx, bus->mask_clk_reg);
		temp &= ~(1u << 16);
		atombios_mmio_write(ctx, bus->mask_clk_reg, temp);
	}

	/* Clear output values. */
	temp = atombios_mmio_read(ctx, bus->a_clk_reg) & ~bus->a_clk_mask;
	atombios_mmio_write(ctx, bus->a_clk_reg, temp);
	temp = atombios_mmio_read(ctx, bus->a_data_reg) & ~bus->a_data_mask;
	atombios_mmio_write(ctx, bus->a_data_reg, temp);

	/* Release pins, then mask them for software use. */
	temp = atombios_mmio_read(ctx, bus->en_clk_reg) & ~bus->en_clk_mask;
	atombios_mmio_write(ctx, bus->en_clk_reg, temp);
	temp = atombios_mmio_read(ctx, bus->en_data_reg) & ~bus->en_data_mask;
	atombios_mmio_write(ctx, bus->en_data_reg, temp);
	temp = atombios_mmio_read(ctx, bus->mask_clk_reg) | bus->mask_clk_mask;
	atombios_mmio_write(ctx, bus->mask_clk_reg, temp);
	(void)atombios_mmio_read(ctx, bus->mask_clk_reg);
	temp = atombios_mmio_read(ctx, bus->mask_data_reg) | bus->mask_data_mask;
	atombios_mmio_write(ctx, bus->mask_data_reg, temp);
	(void)atombios_mmio_read(ctx, bus->mask_data_reg);
}

static void atombios_gpio_i2c_post_xfer(struct atom_context *ctx,
					const struct atombios_i2c_bus_rec *bus)
{
	uint32_t temp;

	temp = atombios_mmio_read(ctx, bus->mask_clk_reg) & ~bus->mask_clk_mask;
	atombios_mmio_write(ctx, bus->mask_clk_reg, temp);
	(void)atombios_mmio_read(ctx, bus->mask_clk_reg);
	temp = atombios_mmio_read(ctx, bus->mask_data_reg) & ~bus->mask_data_mask;
	atombios_mmio_write(ctx, bus->mask_data_reg, temp);
	(void)atombios_mmio_read(ctx, bus->mask_data_reg);
}

static int atombios_gpio_i2c_get_clock(struct atom_context *ctx,
				       const struct atombios_i2c_bus_rec *bus)
{
	return !!(atombios_mmio_read(ctx, bus->y_clk_reg) & bus->y_clk_mask);
}

static int atombios_gpio_i2c_get_data(struct atom_context *ctx,
				      const struct atombios_i2c_bus_rec *bus)
{
	return !!(atombios_mmio_read(ctx, bus->y_data_reg) & bus->y_data_mask);
}

static void atombios_gpio_i2c_set_clock(struct atom_context *ctx,
					const struct atombios_i2c_bus_rec *bus,
					int clock)
{
	uint32_t val;

	val = atombios_mmio_read(ctx, bus->en_clk_reg) & ~bus->en_clk_mask;
	if (!clock)
		val |= bus->en_clk_mask;
	atombios_mmio_write(ctx, bus->en_clk_reg, val);
}

static void atombios_gpio_i2c_set_data(struct atom_context *ctx,
				       const struct atombios_i2c_bus_rec *bus,
				       int data)
{
	uint32_t val;

	val = atombios_mmio_read(ctx, bus->en_data_reg) & ~bus->en_data_mask;
	if (!data)
		val |= bus->en_data_mask;
	atombios_mmio_write(ctx, bus->en_data_reg, val);
}

static void atombios_gpio_i2c_delay(void)
{
	udelay(10);
}

static int atombios_gpio_i2c_wait_clock_high(struct atom_context *ctx,
					     const struct atombios_i2c_bus_rec *bus)
{
	int i;

	atombios_gpio_i2c_set_clock(ctx, bus, 1);
	for (i = 0; i < 100; i++) {
		atombios_gpio_i2c_delay();
		if (atombios_gpio_i2c_get_clock(ctx, bus))
			return 0;
	}
	return -1;
}

static int atombios_gpio_i2c_start(struct atom_context *ctx,
				   const struct atombios_i2c_bus_rec *bus)
{
	atombios_gpio_i2c_set_data(ctx, bus, 1);
	if (atombios_gpio_i2c_wait_clock_high(ctx, bus))
		return -1;
	atombios_gpio_i2c_set_data(ctx, bus, 0);
	atombios_gpio_i2c_delay();
	atombios_gpio_i2c_set_clock(ctx, bus, 0);
	atombios_gpio_i2c_delay();
	return 0;
}

static void atombios_gpio_i2c_stop(struct atom_context *ctx,
				  const struct atombios_i2c_bus_rec *bus)
{
	atombios_gpio_i2c_set_data(ctx, bus, 0);
	atombios_gpio_i2c_delay();
	atombios_gpio_i2c_wait_clock_high(ctx, bus);
	atombios_gpio_i2c_set_data(ctx, bus, 1);
	atombios_gpio_i2c_delay();
}

static int atombios_gpio_i2c_write_byte(struct atom_context *ctx,
					const struct atombios_i2c_bus_rec *bus,
					uint8_t val)
{
	int i;
	int ack;

	for (i = 7; i >= 0; i--) {
		atombios_gpio_i2c_set_data(ctx, bus, !!(val & (1u << i)));
		atombios_gpio_i2c_delay();
		if (atombios_gpio_i2c_wait_clock_high(ctx, bus))
			return -1;
		atombios_gpio_i2c_set_clock(ctx, bus, 0);
		atombios_gpio_i2c_delay();
	}

	atombios_gpio_i2c_set_data(ctx, bus, 1);
	atombios_gpio_i2c_delay();
	if (atombios_gpio_i2c_wait_clock_high(ctx, bus))
		return -1;
	ack = !atombios_gpio_i2c_get_data(ctx, bus);
	atombios_gpio_i2c_set_clock(ctx, bus, 0);
	atombios_gpio_i2c_delay();

	return ack ? 0 : -1;
}

static int atombios_gpio_i2c_read_byte(struct atom_context *ctx,
				       const struct atombios_i2c_bus_rec *bus,
				       uint8_t *val, int last)
{
	uint8_t data = 0;
	int i;

	atombios_gpio_i2c_set_data(ctx, bus, 1);
	for (i = 7; i >= 0; i--) {
		if (atombios_gpio_i2c_wait_clock_high(ctx, bus))
			return -1;
		if (atombios_gpio_i2c_get_data(ctx, bus))
			data |= 1u << i;
		atombios_gpio_i2c_set_clock(ctx, bus, 0);
		atombios_gpio_i2c_delay();
	}

	/* ACK every byte except the last one. */
	atombios_gpio_i2c_set_data(ctx, bus, last ? 1 : 0);
	atombios_gpio_i2c_delay();
	if (atombios_gpio_i2c_wait_clock_high(ctx, bus))
		return -1;
	atombios_gpio_i2c_set_clock(ctx, bus, 0);
	atombios_gpio_i2c_set_data(ctx, bus, 1);
	atombios_gpio_i2c_delay();

	*val = data;
	return 0;
}

static int atombios_gpio_i2c_read(struct atom_context *ctx, uint8_t i2c_line,
				  uint8_t slave_addr, uint8_t reg_offset,
				  uint8_t *buf, uint8_t len)
{
	struct atombios_i2c_bus_rec bus;
	int ret = -1;
	int i;

	if (atombios_lookup_i2c_gpio(ctx, i2c_line, &bus))
		return -1;

	printk(BIOS_DEBUG,
	       "ATOMBIOS: GPIO DDC line=0x%02x clk=0x%04x/%08x data=0x%04x/%08x hw=%d\n",
	       i2c_line, bus.en_clk_reg, bus.en_clk_mask, bus.en_data_reg,
	       bus.en_data_mask, bus.hw_capable);

	atombios_gpio_i2c_pre_xfer(ctx, &bus);

	if (atombios_gpio_i2c_start(ctx, &bus))
		goto out;
	if (atombios_gpio_i2c_write_byte(ctx, &bus, slave_addr << 1))
		goto stop;
	if (atombios_gpio_i2c_write_byte(ctx, &bus, reg_offset))
		goto stop;
	if (atombios_gpio_i2c_start(ctx, &bus))
		goto stop;
	if (atombios_gpio_i2c_write_byte(ctx, &bus, (slave_addr << 1) | 1))
		goto stop;
	for (i = 0; i < len; i++) {
		if (atombios_gpio_i2c_read_byte(ctx, &bus, &buf[i], i == len - 1))
			goto stop;
	}
	ret = 0;

stop:
	atombios_gpio_i2c_stop(ctx, &bus);
out:
	atombios_gpio_i2c_post_xfer(ctx, &bus);
	return ret;
}

/* ---- EDID reading via ATOM I2C ---- */

/*
 * Read a single I2C transaction via the ProcessI2cChannelTransaction
 * ATOM command table. This is used for DDC/EDID reading.
 *
 * Returns 0 on success, -1 on failure.
 */
static int atombios_i2c_write_offset(struct atom_context *ctx, uint8_t i2c_line,
				     uint8_t slave_addr, uint8_t reg_offset)
{
	PROCESS_I2C_CHANNEL_TRANSACTION_PARAMETERS args;
	int ret;

	memset(&args, 0, sizeof(args));
	args.ucI2CSpeed = 50; /* 50 kHz */
	args.ucRegIndex = reg_offset;
	args.lpI2CDataOut = 0;
	args.ucFlag = HW_I2C_WRITE;
	args.ucTransBytes = 0;
	args.ucSlaveAddr = slave_addr << 1;
	args.ucLineNumber = i2c_line;

	ret = atom_execute_table(ctx, cmd_idx.process_i2c, (uint32_t *)&args,
				 sizeof(args));
	if (ret || args.ucStatus != HW_ASSISTED_I2C_STATUS_SUCCESS) {
		printk(BIOS_DEBUG,
		       "ATOMBIOS: ATOM I2C offset write failed "
		       "(line=0x%02x addr=0x%02x status=%d)\n",
		       i2c_line, slave_addr, args.ucStatus);
		return -1;
	}

	return 0;
}

static int atombios_i2c_read_chunk(struct atom_context *ctx, uint8_t i2c_line,
				   uint8_t slave_addr, uint8_t reg_offset,
				   uint8_t *buf, uint8_t len)
{
	PROCESS_I2C_CHANNEL_TRANSACTION_PARAMETERS args;
	int ret;

	if (len > ATOM_MAX_HW_I2C_READ)
		len = ATOM_MAX_HW_I2C_READ;

	memset(&args, 0, sizeof(args));
	args.ucI2CSpeed = 50; /* 50 kHz */
	args.ucRegIndex = reg_offset;
	args.lpI2CDataOut = 0;
	args.ucFlag = HW_I2C_READ;
	args.ucTransBytes = len;
	args.ucSlaveAddr = slave_addr << 1;
	args.ucLineNumber = i2c_line;

	ret = atom_execute_table(ctx, cmd_idx.process_i2c, (uint32_t *)&args,
				 sizeof(args));
	if (ret || args.ucStatus != HW_ASSISTED_I2C_STATUS_SUCCESS) {
		printk(BIOS_DEBUG,
		       "ATOMBIOS: ATOM I2C read failed "
		       "(line=0x%02x addr=0x%02x status=%d)\n",
		       i2c_line, slave_addr, args.ucStatus);
		return -1;
	}

	memcpy(buf, ctx->scratch, len);
	return 0;
}

static int atombios_i2c_read(struct atom_context *ctx, uint8_t i2c_line,
			     uint8_t slave_addr, uint8_t reg_offset,
			     uint8_t *buf, uint8_t len)
{
	if (!atombios_i2c_write_offset(ctx, i2c_line, slave_addr, reg_offset))
		return atombios_i2c_read_chunk(ctx, i2c_line, slave_addr, 0, buf, len);

	/* Some older VBIOS tables combine the offset and read in one call. */
	return atombios_i2c_read_chunk(ctx, i2c_line, slave_addr, reg_offset,
				     buf, len);
}

/*
 * Read EDID from a display via ATOM I2C.
 *
 * Reads one 128-byte EDID block from DDC slave address 0x50.
 * Returns 0 on success, -1 on failure.
 */
static int atombios_read_edid(struct atom_context *ctx, uint8_t i2c_line,
			      uint8_t *edid_buf)
{
	int offset;
	int chunk;
	int ret;

	printk(BIOS_DEBUG, "ATOMBIOS: reading EDID on I2C line 0x%02x\n", i2c_line);

	/*
	 * Read EDID in chunks. The ProcessI2cChannelTransaction table
	 * can read up to 255 bytes, but some implementations are limited.
	 * Use 16-byte chunks for maximum compatibility.
	 */
	for (offset = 0; offset < EDID_BLOCK_SIZE; offset += chunk) {
		chunk = EDID_BLOCK_SIZE - offset;
		if (chunk > 16)
			chunk = 16;

		ret = atombios_gpio_i2c_read(ctx, i2c_line, DDC_EDID_ADDRESS,
					     (uint8_t)offset,
					     edid_buf + offset, (uint8_t)chunk);
		if (ret) {
			printk(BIOS_DEBUG,
			       "ATOMBIOS: GPIO DDC failed, trying ATOM I2C\n");
			ret = atombios_i2c_read(ctx, i2c_line, DDC_EDID_ADDRESS,
						(uint8_t)offset, edid_buf + offset,
						(uint8_t)chunk);
		}
		if (ret)
			return -1;
	}

	/* Validate EDID header: 00 FF FF FF FF FF FF 00 */
	if (edid_buf[0] != 0x00 || edid_buf[1] != 0xFF ||
	    edid_buf[6] != 0xFF || edid_buf[7] != 0x00) {
		printk(BIOS_DEBUG,
		       "ATOMBIOS: EDID header invalid on I2C line 0x%02x: "
		       "%02x %02x %02x %02x %02x %02x %02x %02x\n",
		       i2c_line, edid_buf[0], edid_buf[1], edid_buf[2],
		       edid_buf[3], edid_buf[4], edid_buf[5], edid_buf[6],
		       edid_buf[7]);
		return -1;
	}

	printk(BIOS_INFO, "ATOMBIOS: EDID read successfully on I2C line 0x%02x\n",
	       i2c_line);
	return 0;
}

/* ---- DP AUX channel via ProcessAuxChannelTransaction ---- */

/*
 * Perform a DP AUX channel transaction via the ATOM command table.
 *
 * The ATOM table uses the scratch register area for data:
 *   scratch+4:  AUX request bytes (4-byte header + write payload)
 *   scratch+20: AUX reply data
 *
 * Returns number of reply bytes on success, -1 on failure.
 */
static int atombios_dp_aux_transfer(struct atom_context *ctx,
				    uint8_t aux_id, uint8_t hpd_id,
				    uint8_t request, uint32_t address,
				    uint8_t *send, uint8_t send_len,
				    uint8_t *recv, uint8_t recv_size)
{
	PROCESS_AUX_CHANNEL_TRANSACTION_PARAMETERS_V2 args;
	uint8_t *base;
	uint8_t tx_buf[20];
	int tx_size;
	int recv_bytes;

	/* The ATOM AUX request buffer is 16 bytes including the 4-byte header. */
	if (send_len > 12)
		return -1;

	/* Build 4-byte AUX request header */
	tx_buf[0] = address & 0xFF;
	tx_buf[1] = (address >> 8) & 0xFF;
	tx_buf[2] = (request << 4) | ((address >> 16) & 0xF);
	tx_buf[3] = send_len ? (send_len - 1) : 0;

	if (request == DP_AUX_NATIVE_WRITE && send_len > 0) {
		tx_size = DP_AUX_HEADER_SIZE + send_len;
		tx_buf[3] |= (tx_size << 4);
		memcpy(tx_buf + DP_AUX_HEADER_SIZE, send, send_len);
	} else {
		tx_size = DP_AUX_HEADER_SIZE;
		tx_buf[3] |= (tx_size << 4);
	}

	/* Copy request into scratch area at offset +4 (DWORD 1) */
	base = (uint8_t *)(ctx->scratch + 1);
	memcpy(base, tx_buf, tx_size);

	memset(&args, 0, sizeof(args));
	args.lpAuxRequest = 0 + 4;   /* byte offset in scratch for request */
	args.lpDataOut = 16 + 4;     /* byte offset in scratch for reply */
	args.ucDataOutLen = 0;
	args.ucChannelID = aux_id;
	args.ucDelay = 0;
	args.ucHPD_ID = hpd_id <= 5 ? hpd_id : 0;

	if (atom_execute_table(ctx, cmd_idx.process_aux,
			       (uint32_t *)&args,
			       sizeof(args))) {
		return -1;
	}

	/* Check reply status: 0=ok, 1=timeout, 2=flags, 3=error */
	if (args.ucReplyStatus != 0) {
		printk(BIOS_DEBUG, "ATOMBIOS: AUX reply status %d\n",
		       args.ucReplyStatus);
		return -1;
	}

	recv_bytes = args.ucDataOutLen;
	if (recv && recv_size && recv_bytes > 0) {
		if (recv_bytes > recv_size)
			recv_bytes = recv_size;
		memcpy(recv, base + 16, recv_bytes);
	}

	return recv_bytes;
}

/* Convenience wrappers for DP AUX native read/write */

static int dp_aux_native_write(struct atom_context *ctx,
			       uint8_t aux_id, uint8_t hpd_id,
			       uint32_t address, uint8_t *data, uint8_t len)
{
	return atombios_dp_aux_transfer(ctx, aux_id, hpd_id,
					DP_AUX_NATIVE_WRITE, address,
					data, len, NULL, 0);
}

static int dp_aux_native_read(struct atom_context *ctx,
			      uint8_t aux_id, uint8_t hpd_id,
			      uint32_t address, uint8_t *data, uint8_t len)
{
	return atombios_dp_aux_transfer(ctx, aux_id, hpd_id,
					DP_AUX_NATIVE_READ, address,
					NULL, 0, data, len);
}

static int dp_dpcd_writeb(struct atom_context *ctx,
			  uint8_t aux_id, uint8_t hpd_id,
			  uint32_t address, uint8_t val)
{
	return dp_aux_native_write(ctx, aux_id, hpd_id, address, &val, 1);
}

/* ---- DP link status helpers ---- */

static uint8_t dp_get_lane_status(const uint8_t link_status[DP_LINK_STATUS_SIZE],
				  int lane)
{
	int idx = (lane >> 1); /* 0 for lanes 0,1; 1 for lanes 2,3 */
	int shift = (lane & 1) * 4;
	return (link_status[idx] >> shift) & 0xF;
}

static bool dp_clock_recovery_ok(const uint8_t link_status[DP_LINK_STATUS_SIZE],
				 int lane_count)
{
	int lane;
	for (lane = 0; lane < lane_count; lane++) {
		if (!(dp_get_lane_status(link_status, lane) & DP_LANE_CR_DONE))
			return false;
	}
	return true;
}

static bool dp_channel_eq_ok(const uint8_t link_status[DP_LINK_STATUS_SIZE],
			     int lane_count)
{
	int lane;
	/* Check interlane alignment */
	if (!(link_status[DP_LANE_ALIGN_STATUS_UPDATED - DP_LANE0_1_STATUS]
	      & DP_INTERLANE_ALIGN_DONE))
		return false;
	for (lane = 0; lane < lane_count; lane++) {
		if ((dp_get_lane_status(link_status, lane) & DP_CHANNEL_EQ_BITS)
		    != DP_CHANNEL_EQ_BITS)
			return false;
	}
	return true;
}

/*
 * Extract voltage swing and pre-emphasis adjustment requests from the sink.
 * DPCD registers 0x206-0x207 contain the requested settings.
 */
static void dp_get_adjust_train(const uint8_t link_status[DP_LINK_STATUS_SIZE],
				int lane_count, uint8_t train_set[4])
{
	uint8_t voltage = 0;
	uint8_t pre_emph = 0;
	int lane;

	for (lane = 0; lane < lane_count; lane++) {
		int idx = (lane >> 1) + (DP_ADJUST_REQUEST_LANE0_1 - DP_LANE0_1_STATUS);
		int shift = (lane & 1) * 4;
		uint8_t adj_req = (link_status[idx] >> shift) & 0xF;
		uint8_t lane_voltage = adj_req & DP_TRAIN_VOLTAGE_SWING_MASK;
		uint8_t lane_pre_emph = (adj_req & 0xc) << 1;

		if (lane_voltage > voltage)
			voltage = lane_voltage;
		if (lane_pre_emph > pre_emph)
			pre_emph = lane_pre_emph;
	}

	if (voltage == DP_TRAIN_VOLTAGE_SWING_MASK)
		voltage |= DP_TRAIN_MAX_SWING_REACHED;
	if (pre_emph == DP_TRAIN_PRE_EMPHASIS_MASK)
		pre_emph |= DP_TRAIN_MAX_PRE_REACHED;

	for (lane = 0; lane < 4; lane++)
		train_set[lane] = voltage | pre_emph;
}

static int atombios_dp_get_link_config(const uint8_t dpcd[DP_RECEIVER_CAP_SIZE],
					       uint32_t pixel_clock_khz,
					       uint8_t *lane_count,
					       uint8_t *link_bw)
{
	static const uint8_t link_bw_codes[] = {
		DP_LINK_BW_1_62,
		DP_LINK_BW_2_7,
		DP_LINK_BW_5_4,
	};
	uint8_t max_link_bw = dpcd[DP_MAX_LINK_RATE];
	uint8_t max_lanes = dpcd[DP_MAX_LANE_COUNT] & 0x1f;
	int i;

	if (max_lanes > 4)
		max_lanes = 4;
	if (max_lanes == 0)
		max_lanes = 1;

	for (i = 0; i < ARRAY_SIZE(link_bw_codes); i++) {
		uint8_t bw = link_bw_codes[i];
		uint32_t link_clock_khz;
		uint8_t lanes;

		if (bw > max_link_bw)
			continue;

		link_clock_khz = dp_link_bw_code_to_clock_khz(bw);
		for (lanes = 1; lanes <= max_lanes; lanes <<= 1) {
			/* 8b/10b link coding, 24 bpp framebuffer. */
			uint32_t max_pixel_clock = (lanes * link_clock_khz * 8) / 24;

			if (max_pixel_clock >= pixel_clock_khz) {
				*lane_count = lanes;
				*link_bw = bw;
				return 0;
			}
		}
	}

	return -1;
}

/* ---- DP Link Training ---- */

struct dp_link_train_info {
	struct atom_context *ctx;
	const struct atombios_display_path *path;
	uint8_t aux_id;
	uint8_t hpd_id;
	uint8_t dpcd[DP_RECEIVER_CAP_SIZE];
	uint8_t link_status[DP_LINK_STATUS_SIZE];
	uint8_t train_set[4];
	uint8_t dp_lane_count;
	uint8_t dp_link_bw;
	uint16_t pixel_clock_10khz;
	bool tp3_supported;
};

static void dp_set_training_pattern(struct dp_link_train_info *info, int pattern)
{
	int encoder_cmd = 0;

	switch (pattern) {
	case DP_TRAINING_PATTERN_1:
		encoder_cmd = ATOM_ENCODER_CMD_DP_LINK_TRAINING_PATTERN1;
		break;
	case DP_TRAINING_PATTERN_2:
		encoder_cmd = ATOM_ENCODER_CMD_DP_LINK_TRAINING_PATTERN2;
		break;
	case DP_TRAINING_PATTERN_3:
		encoder_cmd = ATOM_ENCODER_CMD_DP_LINK_TRAINING_PATTERN3;
		break;
	}

	/* Set pattern on source */
	atombios_dig_encoder_setup(info->ctx, info->path,
				   info->pixel_clock_10khz,
				   encoder_cmd, PANEL_8BIT_PER_COLOR);
	/* Set pattern on sink */
	dp_dpcd_writeb(info->ctx, info->aux_id, info->hpd_id,
		       DP_TRAINING_PATTERN_SET, pattern);
}

static void dp_update_vs_emph(struct dp_link_train_info *info)
{
	/* Set voltage swing and pre-emphasis on source transmitter. */
	atombios_transmitter_control(info->ctx, info->path,
				     info->pixel_clock_10khz,
				     ATOM_TRANSMITTER_ACTION_SETUP_VSEMPH,
				     info->dp_lane_count, info->train_set[0]);

	/* Set on sink (write lane training registers) */
	dp_aux_native_write(info->ctx, info->aux_id, info->hpd_id,
			    DP_TRAINING_LANE0_SET,
			    info->train_set, info->dp_lane_count);
}

static int dp_link_train_init(struct dp_link_train_info *info)
{
	uint8_t tmp;

	/* Power up DP 1.1+ sinks. */
	if (info->dpcd[DP_DPCD_REV] >= 0x11) {
		dp_dpcd_writeb(info->ctx, info->aux_id, info->hpd_id,
			       DP_SET_POWER, DP_SET_POWER_D0);
		mdelay(1);
	}

	/* Enable downspread if supported */
	if (info->dpcd[DP_MAX_DOWNSPREAD] & 0x01)
		dp_dpcd_writeb(info->ctx, info->aux_id, info->hpd_id,
			       DP_DOWNSPREAD_CTRL, DP_SPREAD_AMP_0_5);
	else
		dp_dpcd_writeb(info->ctx, info->aux_id, info->hpd_id,
			       DP_DOWNSPREAD_CTRL, 0);

	/* Set lane count (with enhanced framing if supported) */
	tmp = info->dp_lane_count;
	if (info->dpcd[DP_MAX_LANE_COUNT] & (1 << 7)) /* enhanced frame cap */
		tmp |= DP_LANE_COUNT_ENHANCED_FRAME_EN;
	dp_dpcd_writeb(info->ctx, info->aux_id, info->hpd_id,
		       DP_LANE_COUNT_SET, tmp);

	/* Set link bandwidth */
	dp_dpcd_writeb(info->ctx, info->aux_id, info->hpd_id,
		       DP_LINK_BW_SET, info->dp_link_bw);

	/* Start training on source encoder */
	atombios_dig_encoder_setup(info->ctx, info->path,
				   info->pixel_clock_10khz,
				   ATOM_ENCODER_CMD_DP_LINK_TRAINING_START,
				   PANEL_8BIT_PER_COLOR);

	/* Disable training pattern on sink */
	dp_dpcd_writeb(info->ctx, info->aux_id, info->hpd_id,
		       DP_TRAINING_PATTERN_SET, DP_TRAINING_PATTERN_DISABLE);

	return 0;
}

static int dp_link_train_cr(struct dp_link_train_info *info)
{
	uint8_t voltage;
	int tries = 0;
	int i;

	/* Set training pattern 1 */
	dp_set_training_pattern(info, DP_TRAINING_PATTERN_1);
	memset(info->train_set, 0, sizeof(info->train_set));
	dp_update_vs_emph(info);
	udelay(400);

	voltage = 0xFF;
	while (1) {
		/* Wait for clock recovery (100us per DP spec) */
		udelay(100);

		/* Read link status */
		if (dp_aux_native_read(info->ctx, info->aux_id, info->hpd_id,
				       DP_LANE0_1_STATUS, info->link_status,
				       DP_LINK_STATUS_SIZE) < 0) {
			printk(BIOS_ERR, "ATOMBIOS: DP link status read failed\n");
			return -1;
		}

		if (dp_clock_recovery_ok(info->link_status, info->dp_lane_count)) {
			printk(BIOS_INFO, "ATOMBIOS: DP clock recovery OK\n");
			return 0;
		}

		/* Check if all lanes hit max swing */
		for (i = 0; i < info->dp_lane_count; i++) {
			if (!(info->train_set[i] & DP_TRAIN_MAX_SWING_REACHED))
				break;
		}
		if (i == info->dp_lane_count) {
			printk(BIOS_ERR, "ATOMBIOS: DP CR failed: max voltage reached\n");
			return -1;
		}

		/* Check for no progress */
		if ((info->train_set[0] & DP_TRAIN_VOLTAGE_SWING_MASK) == voltage) {
			if (++tries >= 5) {
				printk(BIOS_ERR, "ATOMBIOS: DP CR failed: 5 tries\n");
				return -1;
			}
		} else {
			tries = 0;
		}
		voltage = info->train_set[0] & DP_TRAIN_VOLTAGE_SWING_MASK;

		/* Adjust based on sink request */
		dp_get_adjust_train(info->link_status, info->dp_lane_count,
				    info->train_set);
		dp_update_vs_emph(info);
	}
}

static int dp_link_train_ce(struct dp_link_train_info *info)
{
	int tries = 0;

	/* Set training pattern 3 (preferred) or 2 */
	if (info->tp3_supported)
		dp_set_training_pattern(info, DP_TRAINING_PATTERN_3);
	else
		dp_set_training_pattern(info, DP_TRAINING_PATTERN_2);

	while (1) {
		/* Wait for channel equalization (400us per DP spec) */
		udelay(400);

		if (dp_aux_native_read(info->ctx, info->aux_id, info->hpd_id,
				       DP_LANE0_1_STATUS, info->link_status,
				       DP_LINK_STATUS_SIZE) < 0) {
			printk(BIOS_ERR, "ATOMBIOS: DP link status read failed\n");
			return -1;
		}

		if (dp_channel_eq_ok(info->link_status, info->dp_lane_count)) {
			printk(BIOS_INFO, "ATOMBIOS: DP channel equalization OK\n");
			return 0;
		}

		if (++tries > 5) {
			printk(BIOS_ERR, "ATOMBIOS: DP channel EQ failed: 5 tries\n");
			return -1;
		}

		dp_get_adjust_train(info->link_status, info->dp_lane_count,
				    info->train_set);
		dp_update_vs_emph(info);
	}
}

static void dp_link_train_finish(struct dp_link_train_info *info)
{
	udelay(400);

	/* Disable training pattern on sink */
	dp_dpcd_writeb(info->ctx, info->aux_id, info->hpd_id,
		       DP_TRAINING_PATTERN_SET, DP_TRAINING_PATTERN_DISABLE);

	/* Complete training on source */
	atombios_dig_encoder_setup(info->ctx, info->path,
				   info->pixel_clock_10khz,
				   ATOM_ENCODER_CMD_DP_LINK_TRAINING_COMPLETE,
				   PANEL_8BIT_PER_COLOR);
}

/*
 * Full DP link training sequence.
 * Returns 0 on success, -1 on failure.
 */
static int atombios_dp_link_train(struct atom_context *ctx,
				  const struct atombios_display_path *path,
				  uint16_t pixel_clock_10khz,
				  uint8_t aux_id, uint8_t hpd_id)
{
	struct dp_link_train_info info;
	struct atombios_display_path train_path = *path;
	int ret;

	memset(&info, 0, sizeof(info));
	info.ctx = ctx;
	info.path = &train_path;
	info.aux_id = aux_id;
	info.hpd_id = hpd_id;
	info.pixel_clock_10khz = pixel_clock_10khz;

	/* Read receiver capabilities (DPCD 0x000-0x00E) */
	if (dp_aux_native_read(ctx, aux_id, hpd_id, DP_DPCD_REV,
			       info.dpcd, DP_RECEIVER_CAP_SIZE) < 0) {
		printk(BIOS_ERR, "ATOMBIOS: failed to read DP receiver caps\n");
		return -1;
	}

	printk(BIOS_INFO, "ATOMBIOS: DP DPCD rev=%d.%d max_rate=0x%02x max_lanes=%d\n",
	       info.dpcd[0] >> 4, info.dpcd[0] & 0xF,
	       info.dpcd[DP_MAX_LINK_RATE],
	       info.dpcd[DP_MAX_LANE_COUNT] & 0x1F);

	if (path->dp_link_bw && path->dp_lane_count) {
		info.dp_link_bw = path->dp_link_bw;
		info.dp_lane_count = path->dp_lane_count;
	} else if (atombios_dp_get_link_config(info.dpcd,
					       (uint32_t)pixel_clock_10khz * 10,
					       &info.dp_lane_count,
					       &info.dp_link_bw)) {
		printk(BIOS_WARNING,
		       "ATOMBIOS: failed to choose DP link config, using sink maximum\n");
		info.dp_link_bw = info.dpcd[DP_MAX_LINK_RATE];
		info.dp_lane_count = info.dpcd[DP_MAX_LANE_COUNT] & 0x1F;
		if (info.dp_lane_count > 4)
			info.dp_lane_count = 4;
	}
	if (!info.dp_lane_count)
		info.dp_lane_count = 1;
	if (!info.dp_link_bw)
		info.dp_link_bw = DP_LINK_BW_1_62;
	train_path.dp_lane_count = info.dp_lane_count;
	train_path.dp_link_bw = info.dp_link_bw;

	/* Only use TPS3 on HBR2 links; older sources cannot generate it. */
	info.tp3_supported = info.dp_link_bw >= DP_LINK_BW_5_4 &&
		!!(info.dpcd[DP_MAX_LANE_COUNT] & DP_TPS3_SUPPORTED);

	/* Init */
	ret = dp_link_train_init(&info);
	if (ret)
		goto done;

	/* Phase 1: Clock Recovery */
	ret = dp_link_train_cr(&info);
	if (ret)
		goto done;

	/* Phase 2: Channel Equalization */
	ret = dp_link_train_ce(&info);

done:
	/* Always finish training (even on failure) */
	dp_link_train_finish(&info);

	if (ret == 0) {
		/* Enable video on source */
		atombios_dig_encoder_setup(ctx, info.path, pixel_clock_10khz,
					   ATOM_ENCODER_CMD_DP_VIDEO_ON,
					   PANEL_8BIT_PER_COLOR);
		printk(BIOS_INFO, "ATOMBIOS: DP link training succeeded (%d lanes @ 0x%02x)\n",
		       info.dp_lane_count, info.dp_link_bw);
	} else {
		printk(BIOS_ERR, "ATOMBIOS: DP link training FAILED\n");
	}

	return ret;
}

static void atombios_dp_prepare_link(struct atom_context *ctx,
					     struct atombios_display_path *path,
					     uint32_t pixel_clock_khz)
{
	uint8_t dpcd[DP_RECEIVER_CAP_SIZE];

	if (path->encoder_mode != ATOM_ENCODER_MODE_DP || !path->i2c_valid)
		return;

	if (dp_aux_native_read(ctx, path->i2c_line, path->hpd_id, DP_DPCD_REV,
			       dpcd, sizeof(dpcd)) < 0) {
		printk(BIOS_WARNING, "ATOMBIOS: failed to read DP DPCD caps\n");
		return;
	}

	if (atombios_dp_get_link_config(dpcd, pixel_clock_khz,
				       &path->dp_lane_count, &path->dp_link_bw)) {
		printk(BIOS_WARNING,
		       "ATOMBIOS: no DP link config for %u kHz pixel clock\n",
		       pixel_clock_khz);
		return;
	}

	printk(BIOS_INFO, "ATOMBIOS: selected DP link %d lanes @ 0x%02x\n",
	       path->dp_lane_count, path->dp_link_bw);
}

/* ---- eDP panel power control ---- */

/*
 * Power on/off an eDP panel via the transmitter control table.
 */
static int atombios_edp_panel_power(struct atom_context *ctx,
				    const struct atombios_display_path *path,
				    int power_on)
{
	uint8_t action = power_on ? ATOM_TRANSMITTER_ACTION_POWER_ON
				  : ATOM_TRANSMITTER_ACTION_POWER_OFF;

	printk(BIOS_INFO, "ATOMBIOS: eDP panel power %s\n",
	       power_on ? "ON" : "OFF");

	return atombios_transmitter_control(ctx, path, 0, action, 0, 0);
}

/* ---- SetPixelClock ---- */

struct atombios_pll_params {
	uint32_t reference_freq;
	uint32_t pll_out_min;
	uint32_t pll_out_max;
};

static void atombios_get_pll_params(struct atom_context *ctx,
					    struct atombios_pll_params *pll)
{
	uint16_t data_offset;
	uint8_t frev, crev;
	ATOM_FIRMWARE_INFO *info;

	pll->reference_freq = 2700;
	pll->pll_out_min = 64800;
	pll->pll_out_max = 200000;

	if (!atom_parse_data_header(ctx, cmd_idx.data_firmware_info,
				    NULL, &frev, &crev, &data_offset))
		return;

	info = (ATOM_FIRMWARE_INFO *)((uint8_t *)ctx->bios + data_offset);
	if (info->usReferenceClock)
		pll->reference_freq = info->usReferenceClock;
	if (info->ulMaxPixelClockPLL_Output)
		pll->pll_out_max = info->ulMaxPixelClockPLL_Output;

	if (frev < 2 && crev < 2) {
		if (info->usMinPixelClockPLL_Output)
			pll->pll_out_min = info->usMinPixelClockPLL_Output;
	} else {
		ATOM_FIRMWARE_INFO_V1_2 *info12 = (ATOM_FIRMWARE_INFO_V1_2 *)info;
		if (info12->ulMinPixelClockPLL_Output)
			pll->pll_out_min = info12->ulMinPixelClockPLL_Output;
	}
}

static uint32_t atombios_abs_diff_u32(uint32_t a, uint32_t b)
{
	return a > b ? a - b : b - a;
}

static uint32_t atombios_gcd_u32(uint32_t a, uint32_t b)
{
	while (b) {
		uint32_t t = b;
		b = a % b;
		a = t;
	}
	return a ? a : 1;
}

static uint32_t atombios_div_round_up(uint32_t n, uint32_t d)
{
	return (n + d - 1) / d;
}

static uint32_t atombios_div_round_closest(uint64_t n, uint32_t d)
{
	return (uint32_t)((n + d / 2) / d);
}

static uint32_t atombios_clamp_u32(uint32_t v, uint32_t min, uint32_t max)
{
	if (v < min)
		return min;
	if (v > max)
		return max;
	return v;
}

static void atombios_avivo_reduce_ratio(uint32_t *nom, uint32_t *den,
					uint32_t nom_min, uint32_t den_min)
{
	uint32_t tmp = atombios_gcd_u32(*nom, *den);

	*nom /= tmp;
	*den /= tmp;

	if (*nom < nom_min) {
		tmp = atombios_div_round_up(nom_min, *nom);
		*nom *= tmp;
		*den *= tmp;
	}

	if (*den < den_min) {
		tmp = atombios_div_round_up(den_min, *den);
		*nom *= tmp;
		*den *= tmp;
	}
}

static void atombios_avivo_get_fb_ref_div(uint32_t nom, uint32_t den,
					  uint32_t post_div,
					  uint32_t fb_div_max,
					  uint32_t ref_div_max,
					  uint32_t *fb_div,
					  uint32_t *ref_div)
{
	ref_div_max = atombios_clamp_u32(100 / post_div, 1, ref_div_max);
	*ref_div = atombios_clamp_u32(den / post_div, 1, ref_div_max);
	*fb_div = atombios_div_round_closest((uint64_t)nom * *ref_div * post_div,
					      den);

	if (*fb_div > fb_div_max) {
		*ref_div = (*ref_div * fb_div_max) / *fb_div;
		*fb_div = fb_div_max;
	}
}

static void atombios_compute_avivo_pll(struct atom_context *ctx,
					       uint32_t target_clock,
					       uint32_t *dot_clock,
					       uint32_t *fb_div,
					       uint32_t *frac_fb_div,
					       uint32_t *ref_div,
					       uint32_t *post_div)
{
	struct atombios_pll_params pll;
	uint32_t fb_div_min = 4;
	uint32_t fb_div_max = 0x7ff;
	uint32_t ref_div_min = 2;
	uint32_t ref_div_max = 7;
	uint32_t post_min, post_max, post;
	uint32_t post_best, diff_best = 0xffffffff;
	uint32_t nom, den;

	atombios_get_pll_params(ctx, &pll);

	post_min = pll.pll_out_min / target_clock;
	if (target_clock * post_min < pll.pll_out_min)
		post_min++;
	if (post_min < 2)
		post_min = 2;

	post_max = pll.pll_out_max / target_clock;
	if (target_clock * post_max > pll.pll_out_max)
		post_max--;
	if (post_max > 0x7f)
		post_max = 0x7f;
	if (post_max < post_min)
		post_max = post_min;

	/* Match Linux radeon_compute_pll_avivo() flags used on RV630:
	 * RADEON_PLL_PREFER_LOW_REF_DIV and RADEON_PLL_PREFER_MINM_OVER_MAXP. */
	nom = target_clock;
	den = pll.reference_freq;
	atombios_avivo_reduce_ratio(&nom, &den, fb_div_min, post_min);

	post_best = post_min;
	for (post = post_min; post <= post_max; post++) {
		uint32_t fb, ref, actual, diff;

		atombios_avivo_get_fb_ref_div(nom, den, post, fb_div_max,
					      ref_div_max, &fb, &ref);
		actual = (pll.reference_freq * fb) / (ref * post);
		diff = atombios_abs_diff_u32(target_clock, actual);
		if (diff < diff_best) {
			post_best = post;
			diff_best = diff;
		}
	}

	*post_div = post_best;
	atombios_avivo_get_fb_ref_div(nom, den, *post_div, fb_div_max,
				      ref_div_max, fb_div, ref_div);
	atombios_avivo_reduce_ratio(fb_div, ref_div, fb_div_min, ref_div_min);

	*frac_fb_div = 0;
	*dot_clock = (pll.reference_freq * *fb_div) / (*ref_div * *post_div);

	printk(BIOS_DEBUG,
	       "ATOMBIOS: PLL ref=%u min=%u max=%u target=%u actual=%u fb=%u refdiv=%u post=%u\n",
	       pll.reference_freq, pll.pll_out_min, pll.pll_out_max,
	       target_clock, *dot_clock, *fb_div, *ref_div, *post_div);
}

static uint16_t atombios_adjust_display_pll(struct atom_context *ctx,
					    const struct atombios_display_path *path,
					    uint16_t pixel_clock_10khz)
{
	uint8_t frev, crev;
	union {
		ADJUST_DISPLAY_PLL_PS_ALLOCATION v1;
		ADJUST_DISPLAY_PLL_PS_ALLOCATION_V3 v3;
	} args;
	uint16_t adjusted_clock = pixel_clock_10khz;

	/* Our DP path does not track the trained link clock yet. */
	if (path->encoder_mode == ATOM_ENCODER_MODE_DP)
		return adjusted_clock;

	if (cmd_idx.adjust_display_pll < 0)
		return adjusted_clock;
	if (!atom_parse_cmd_header(ctx, cmd_idx.adjust_display_pll, &frev, &crev))
		return adjusted_clock;

	memset(&args, 0, sizeof(args));

	switch (crev) {
	case 1:
	case 2:
		args.v1.usPixelClock = pixel_clock_10khz;
		args.v1.ucTransmitterID = path->encoder_obj_id;
		args.v1.ucEncodeMode = path->encoder_mode;
		if (atom_execute_table(ctx, cmd_idx.adjust_display_pll,
				       (uint32_t *)&args, sizeof(args)) == 0)
			adjusted_clock = args.v1.usPixelClock;
		break;
	case 3:
	default:
		args.v3.sInput.usPixelClock = pixel_clock_10khz;
		args.v3.sInput.ucTransmitterID = path->encoder_obj_id;
		args.v3.sInput.ucEncodeMode = path->encoder_mode;
		if (path->encoder_mode == ATOM_ENCODER_MODE_DVI &&
		    pixel_clock_10khz > 16500)
			args.v3.sInput.ucDispPllConfig |= DISPPLL_CONFIG_DUAL_LINK;
		if (atom_execute_table(ctx, cmd_idx.adjust_display_pll,
				       (uint32_t *)&args, sizeof(args)) == 0 &&
		    args.v3.sOutput.ulDispPllFreq)
			adjusted_clock = args.v3.sOutput.ulDispPllFreq;
		break;
	}

	if (adjusted_clock != pixel_clock_10khz)
		printk(BIOS_DEBUG,
		       "ATOMBIOS: AdjustDisplayPll %u -> %u *10kHz\n",
		       pixel_clock_10khz, adjusted_clock);

	return adjusted_clock;
}

/*
 * Program the pixel clock PLL via the SetPixelClock ATOM command table.
 * Linux computes PLL dividers before calling SetPixelClock on table
 * revisions 1.1 through 1.3; do the same here for older AVIVO chips.
 */
static int atombios_set_pixel_clock(struct atom_context *ctx,
				    const struct atombios_gpu_profile *profile,
				    const struct atombios_display_path *path,
				    uint16_t pixel_clock_10khz,
				    int crtc_id)
{
	uint8_t frev, crev;
	union {
		PIXEL_CLOCK_PARAMETERS v1;
		PIXEL_CLOCK_PARAMETERS_V2 v2;
		PIXEL_CLOCK_PARAMETERS_V3 v3;
		PIXEL_CLOCK_PARAMETERS_V5 v5;
		PIXEL_CLOCK_PARAMETERS_V6 v6;
		PIXEL_CLOCK_PARAMETERS_V7 v7;
	} args;
	uint8_t encoder_id;
	uint8_t pll_id = profile->pixel_clock_ppll;
	uint16_t adjusted_clock_10khz;
	uint32_t dot_clock, fb_div, frac_fb_div, ref_div, post_div;

	if (profile->use_combo_phy_pixel_clock && path->phy_id < 6)
		pll_id = ATOM_COMBOPHY_PLL0 + path->phy_id;

	if (!atom_parse_cmd_header(ctx, cmd_idx.set_pixel_clock, &frev, &crev))
		return -1;

	memset(&args, 0, sizeof(args));

	/* Linux passes radeon_encoder->encoder_id here. */
	encoder_id = path->encoder_obj_id;
	adjusted_clock_10khz = atombios_adjust_display_pll(ctx, path, pixel_clock_10khz);
	atombios_compute_avivo_pll(ctx, adjusted_clock_10khz, &dot_clock,
				 &fb_div, &frac_fb_div, &ref_div, &post_div);

	printk(BIOS_DEBUG,
	       "ATOMBIOS: SetPixelClock frev=%d crev=%d clock=%d*10kHz crtc=%d ppll=%u\n",
	       frev, crev, adjusted_clock_10khz, crtc_id, pll_id);

	switch (crev) {
	case 1:
		args.v1.usPixelClock = adjusted_clock_10khz;
		args.v1.usRefDiv = ref_div;
		args.v1.usFbDiv = fb_div;
		args.v1.ucFracFbDiv = frac_fb_div;
		args.v1.ucPostDiv = post_div;
		args.v1.ucPpll = pll_id;
		args.v1.ucCRTC = crtc_id;
		args.v1.ucRefDivSrc = 1;
		break;
	case 2:
		args.v2.usPixelClock = adjusted_clock_10khz;
		args.v2.usRefDiv = ref_div;
		args.v2.usFbDiv = fb_div;
		args.v2.ucFracFbDiv = frac_fb_div;
		args.v2.ucPostDiv = post_div;
		args.v2.ucPpll = pll_id;
		args.v2.ucCRTC = crtc_id;
		args.v2.ucRefDivSrc = 1;
		break;
	case 3:
		args.v3.usPixelClock = adjusted_clock_10khz;
		args.v3.usRefDiv = ref_div;
		args.v3.usFbDiv = fb_div;
		args.v3.ucFracFbDiv = frac_fb_div;
		args.v3.ucPostDiv = post_div;
		args.v3.ucPpll = pll_id;
		args.v3.ucTransmitterId = encoder_id;
		args.v3.ucEncoderMode = path->encoder_mode;
		args.v3.ucMiscInfo = crtc_id == ATOM_CRTC2 ?
			PIXEL_CLOCK_MISC_CRTC_SEL_CRTC2 :
			PIXEL_CLOCK_MISC_CRTC_SEL_CRTC1;
		break;
	case 5:
		args.v5.ucCRTC = crtc_id;
		args.v5.usPixelClock = adjusted_clock_10khz;
		args.v5.ucRefDiv = ref_div;
		args.v5.usFbDiv = fb_div;
		args.v5.ulFbDivDecFrac = frac_fb_div * 100000;
		args.v5.ucPostDiv = post_div;
		args.v5.ucPpll = pll_id;
		args.v5.ucTransmitterID = encoder_id;
		args.v5.ucEncoderMode = path->encoder_mode;
		args.v5.ucMiscInfo = 0;
		break;
	case 6:
		args.v6.ulDispEngClkFreq = ((uint32_t)crtc_id << 24) |
			adjusted_clock_10khz;
		args.v6.ucRefDiv = ref_div;
		args.v6.usFbDiv = fb_div;
		args.v6.ulFbDivDecFrac = frac_fb_div * 100000;
		args.v6.ucPostDiv = post_div;
		args.v6.ucPpll = pll_id;
		args.v6.ucTransmitterID = encoder_id;
		args.v6.ucEncoderMode = path->encoder_mode;
		args.v6.ucMiscInfo = 0;
		break;
	case 7:
	default:
		args.v7.ulPixelClock = (uint32_t)adjusted_clock_10khz * 100;
		args.v7.ucCRTC = crtc_id;
		args.v7.ucPpll = pll_id;
		args.v7.ucTransmitterID = encoder_id;
		args.v7.ucEncoderMode = path->encoder_mode;
		args.v7.ucMiscInfo = 0;
		if (path->encoder_mode == ATOM_ENCODER_MODE_DVI &&
		    adjusted_clock_10khz > 16500)
			args.v7.ucMiscInfo |= PIXEL_CLOCK_V7_MISC_DVI_DUALLINK_EN;
		break;
	}

	return atom_execute_table(ctx, cmd_idx.set_pixel_clock,
				  (uint32_t *)&args,
				  sizeof(args));
}

static int atombios_set_dce_clock(struct atom_context *ctx,
					  const struct atombios_gpu_profile *profile)
{
	uint8_t frev, crev;
	union {
		SET_DCE_CLOCK_PS_ALLOCATION_V1_1 v1;
		SET_DCE_CLOCK_PS_ALLOCATION_V2_1 v2;
	} args;

	if (!atom_parse_cmd_header(ctx, cmd_idx.set_dce_clock, &frev, &crev))
		return -1;

	memset(&args, 0, sizeof(args));

	if (frev == 1 && crev == 1) {
		args.v1.asParam.ulDISPClkFreq = DEFAULT_DCE_DISPCLK_10KHZ;
		printk(BIOS_DEBUG,
		       "ATOMBIOS: SetDCEClock dispclk=%u*10kHz crev=%u\n",
		       DEFAULT_DCE_DISPCLK_10KHZ, crev);
		return atom_execute_table(ctx, cmd_idx.set_dce_clock,
					  (uint32_t *)&args, sizeof(args));
	}

	if (frev == 2 && crev == 1) {
		int ret;

		args.v2.asParam.ulDCEClkFreq = DEFAULT_DCE_DISPCLK_10KHZ;
		args.v2.asParam.ucDCEClkType = DCE_CLOCK_TYPE_DISPCLK;
		args.v2.asParam.ucDCEClkSrc = profile->dispclk_ppll;
		printk(BIOS_DEBUG,
		       "ATOMBIOS: SetDCEClock dispclk=%u*10kHz src=%u crev=%u\n",
		       DEFAULT_DCE_DISPCLK_10KHZ, profile->dispclk_ppll, crev);
		ret = atom_execute_table(ctx, cmd_idx.set_dce_clock,
					 (uint32_t *)&args, sizeof(args));

		memset(&args, 0, sizeof(args));
		args.v2.asParam.ucDCEClkType = DCE_CLOCK_TYPE_DPREFCLK;
		args.v2.asParam.ucDCEClkSrc = profile->dispclk_ppll;
		atom_execute_table(ctx, cmd_idx.set_dce_clock,
				   (uint32_t *)&args, sizeof(args));
		return ret;
	}

	return 0;
}

static int atombios_set_disp_eng_clock(struct atom_context *ctx,
					       const struct atombios_gpu_profile *profile)
{
	uint8_t frev, crev;
	union {
		PIXEL_CLOCK_PARAMETERS_V5 v5;
		PIXEL_CLOCK_PARAMETERS_V6 v6;
	} args;

	if (profile->scanout != ATOMBIOS_SCANOUT_EVERGREEN)
		return 0;

	if (profile->use_set_dce_clock)
		return atombios_set_dce_clock(ctx, profile);

	if (!atom_parse_cmd_header(ctx, cmd_idx.set_pixel_clock, &frev, &crev))
		return -1;

	memset(&args, 0, sizeof(args));

	/* Match Linux radeon/amdgpu disp_eng_pll_init(). DCE4+ needs the
	 * display engine clock programmed before the CRTC pixel clock, but the
	 * PLL id differs by generation. */
	switch (crev) {
	case 5:
		args.v5.ucCRTC = ATOM_CRTC_INVALID;
		args.v5.usPixelClock = DEFAULT_DCE_DISPCLK_10KHZ;
		args.v5.ucPpll = ATOM_DCPLL;
		break;
	case 6:
		args.v6.ulDispEngClkFreq = DEFAULT_DCE_DISPCLK_10KHZ;
		args.v6.ucPpll = profile->dispclk_ppll;
		break;
	default:
		return 0;
	}

	printk(BIOS_DEBUG,
	       "ATOMBIOS: SetPixelClock dispclk=%u*10kHz ppll=%u crev=%u\n",
	       DEFAULT_DCE_DISPCLK_10KHZ,
	       crev == 6 ? profile->dispclk_ppll : ATOM_DCPLL, crev);

	return atom_execute_table(ctx, cmd_idx.set_pixel_clock,
				  (uint32_t *)&args, sizeof(args));
}

/* ---- DAC Encoder Control (VGA/CRT) ---- */

/*
 * Enable or disable a DAC encoder for CRT/VGA output.
 *
 * This calls DAC1EncoderControl or DAC2EncoderControl depending on which
 * DAC is associated with the display path. The DAC standard is always
 * ATOM_DAC1_PS2 (VGA/PC monitor).
 */
static int atombios_dac_encoder_setup(struct atom_context *ctx,
				      const struct atombios_display_path *path,
				      uint16_t pixel_clock_10khz,
				      uint8_t action)
{
	DAC_ENCODER_CONTROL_PARAMETERS args;
	int index;

	memset(&args, 0, sizeof(args));

	if (path->dac_type == ATOM_DAC_A)
		index = cmd_idx.dac1_encoder_ctrl;
	else
		index = cmd_idx.dac2_encoder_ctrl;

	args.usPixelClock = pixel_clock_10khz;
	args.ucDacStandard = ATOM_DAC1_PS2;
	args.ucAction = action;

	printk(BIOS_DEBUG, "ATOMBIOS: DAC%c encoder %s (clock=%d*10kHz)\n",
	       path->dac_type == ATOM_DAC_A ? 'A' : 'B',
	       action == ATOM_ENABLE ? "enable" : "disable",
	       pixel_clock_10khz);

	return atom_execute_table(ctx, index, (uint32_t *)&args,
				  sizeof(args));
}

/* ---- DAC Load Detection (VGA/CRT) ---- */

/*
 * Perform DAC load detection to check if a CRT/VGA display is connected.
 *
 * This calls the DAC_LoadDetection ATOM command table, which physically
 * drives the DAC outputs and measures the load. A connected CRT monitor
 * presents a resistive load that the DAC can detect.
 *
 * Returns 0 on success (table executed), -1 on failure.
 * The actual detection result is in the BIOS scratch registers.
 */
static int atombios_dac_load_detect(struct atom_context *ctx,
				    const struct atombios_display_path *path)
{
	DAC_LOAD_DETECTION_PS_ALLOCATION args;

	memset(&args, 0, sizeof(args));

	args.sDacload.ucDacType = path->dac_type;
	args.sDacload.usDeviceID = ATOM_DEVICE_CRT1_SUPPORT;
	args.sDacload.ucMisc = 0;

	printk(BIOS_DEBUG, "ATOMBIOS: DAC%c load detection\n",
	       path->dac_type == ATOM_DAC_A ? 'A' : 'B');

	return atom_execute_table(ctx, cmd_idx.dac_load_detect,
				  (uint32_t *)&args,
				  sizeof(args));
}

static int atombios_device_tag_index(uint16_t device_tag)
{
	int i;

	for (i = 0; i < ATOM_MAX_SUPPORTED_DEVICE; i++) {
		if (device_tag & (1 << i))
			return i;
	}

	return -1;
}

static void atombios_apply_supported_device_info(struct atom_context *ctx,
						 struct atombios_display_path *paths,
						 int count)
{
	ATOM_SUPPORTED_DEVICES_INFO_2d1 *supported;
	uint16_t data_offset, device_support;
	uint8_t frev, crev;
	int max_device;
	int i;

	if (cmd_idx.data_supported_devices < 0)
		return;

	if (!atom_parse_data_header(ctx, cmd_idx.data_supported_devices,
				    NULL, &frev, &crev, &data_offset))
		return;

	supported = (ATOM_SUPPORTED_DEVICES_INFO_2d1 *)
		((uint8_t *)ctx->bios + data_offset);
	device_support = supported->usDeviceSupport;
	max_device = (frev > 1) ? ATOM_MAX_SUPPORTED_DEVICE : ATOM_MAX_SUPPORTED_DEVICE_INFO;

	printk(BIOS_INFO,
	       "ATOMBIOS: SupportedDevicesInfo frev=%d crev=%d support=0x%04x\n",
	       frev, crev, device_support);

	for (i = 0; i < count; i++) {
		ATOM_CONNECTOR_INFO_I2C *conn;
		uint8_t conn_type;
		uint8_t dac;
		int dev_index = atombios_device_tag_index(paths[i].device_tag);

		if (dev_index < 0 || dev_index >= max_device)
			continue;
		if (!(device_support & (1 << dev_index)))
			continue;

		conn = &supported->asConnInfo[dev_index];
		conn_type = conn->sucConnectorInfo.sbfAccess.bfConnectorType;
		dac = conn->sucConnectorInfo.sbfAccess.bfAssociatedDAC;

		if (!paths[i].i2c_valid) {
			paths[i].i2c_line = conn->sucI2cId.ucAccess;
			paths[i].i2c_valid = 1;
		}

		/* Linux uses SupportedDevicesInfo as an old-table fallback, not
		 * as an override for Object_Header-derived connector paths. */
		if (paths[i].connector_type != 0)
			goto log_supported_device;

		switch (conn_type) {
		case 1: /* VGA */
		case 4: /* DVI-A */
			paths[i].connector_type = 4;
			paths[i].encoder_mode = ATOM_ENCODER_MODE_CRT;
			paths[i].is_dac = 1;
			paths[i].dac_type = (dac == 2) ? ATOM_DAC_B : ATOM_DAC_A;
			break;
		case 2: /* DVI-I */
		case 3: /* DVI-D */
		case 8: /* digital link */
			paths[i].connector_type = 3;
			paths[i].encoder_mode = ATOM_ENCODER_MODE_DVI;
			break;
		case 7: /* LVDS */
			paths[i].connector_type = 2;
			paths[i].encoder_mode = ATOM_ENCODER_MODE_LVDS;
			break;
		case 0x0a: /* HDMI type A */
		case 0x0b: /* HDMI type B */
			paths[i].connector_type = 1;
			/* Linux drives HDMI as DVI/TMDS when audio/infoframes are off. */
			paths[i].encoder_mode = ATOM_ENCODER_MODE_DVI;
			break;
		default:
			break;
		}

log_supported_device:
		printk(BIOS_INFO,
		       "ATOMBIOS: supported dev %d tag=0x%04x conn_type=0x%x i2c=0x%02x dac=%d\n",
		       dev_index, paths[i].device_tag, conn_type, paths[i].i2c_line, dac);
	}
}

static uint8_t atombios_hpd_from_gpio(struct atom_context *ctx, uint8_t gpio_id)
{
	ATOM_GPIO_PIN_LUT *gpio_info;
	ATOM_GPIO_PIN_ASSIGNMENT *pin;
	uint16_t data_offset, size;
	int num_indices;
	int i;

	if (!atom_parse_data_header(ctx, cmd_idx.data_gpio_pin, &size, NULL,
				    NULL, &data_offset))
		return ATOMBIOS_HPD_NONE;

	gpio_info = (ATOM_GPIO_PIN_LUT *)((uint8_t *)ctx->bios + data_offset);
	num_indices = (size - sizeof(ATOM_COMMON_TABLE_HEADER)) /
		sizeof(ATOM_GPIO_PIN_ASSIGNMENT);
	pin = &gpio_info->asGPIO_Pin[0];

	for (i = 0; i < num_indices; i++) {
		uint32_t reg;
		uint32_t mask;

		if (pin->ucGPIO_ID != gpio_id) {
			pin = (ATOM_GPIO_PIN_ASSIGNMENT *)
				((uint8_t *)pin + sizeof(ATOM_GPIO_PIN_ASSIGNMENT));
			continue;
		}

		reg = pin->usGpioPin_AIndex * 4;
		mask = 1u << pin->ucGpioPinBitShift;
		if (reg != AVIVO_DC_GPIO_HPD_A &&
		    reg != EVERGREEN_DC_GPIO_HPD_A && reg != SI_DC_GPIO_HPD_A)
			return ATOMBIOS_HPD_NONE;

		switch (mask) {
		case (1 << 0): return 0;
		case (1 << 8): return 1;
		case (1 << 16): return 2;
		case (1 << 24): return 3;
		case (1 << 26): return 4;
		case (1 << 28): return 5;
		default: return ATOMBIOS_HPD_NONE;
		}
	}

	return ATOMBIOS_HPD_NONE;
}

static void atombios_read_connector_records(struct atom_context *ctx,
					    ATOM_OBJECT_HEADER *obj_hdr,
					    uint16_t data_offset,
					    struct atombios_display_path *path)
{
	ATOM_OBJECT_TABLE *con_obj;
	uint8_t *base = (uint8_t *)ctx->bios;
	int i;

	if (!obj_hdr->usConnectorObjectTableOffset)
		return;

	con_obj = (ATOM_OBJECT_TABLE *)
		(base + data_offset + obj_hdr->usConnectorObjectTableOffset);

	for (i = 0; i < con_obj->ucNumberOfObjects; i++) {
		ATOM_COMMON_RECORD_HEADER *record;

		if (con_obj->asObjects[i].usObjectID != path->connector_obj_id)
			continue;
		if (!con_obj->asObjects[i].usRecordOffset)
			return;

		record = (ATOM_COMMON_RECORD_HEADER *)
			(base + data_offset + con_obj->asObjects[i].usRecordOffset);

		while (record->ucRecordSize > 0 &&
		       record->ucRecordType > 0 &&
		       record->ucRecordType <= ATOM_MAX_OBJECT_RECORD_NUMBER) {
			switch (record->ucRecordType) {
			case ATOM_I2C_RECORD_TYPE: {
				ATOM_I2C_RECORD *i2c_record = (ATOM_I2C_RECORD *)record;
				ATOM_I2C_ID_CONFIG_ACCESS *i2c_id =
					(ATOM_I2C_ID_CONFIG_ACCESS *)&i2c_record->sucI2cId;
				path->i2c_line = i2c_id->ucAccess;
				path->i2c_valid = 1;
				break;
			}
			case ATOM_HPD_INT_RECORD_TYPE: {
				ATOM_HPD_INT_RECORD *hpd_record = (ATOM_HPD_INT_RECORD *)record;
				path->hpd_id = atombios_hpd_from_gpio(ctx,
						hpd_record->ucHPDIntGPIOID);
				break;
			}
			}

			record = (ATOM_COMMON_RECORD_HEADER *)
				((uint8_t *)record + record->ucRecordSize);
		}
		return;
	}
}

/* ---- Display path discovery from ATOM object tables ---- */

/*
 * Parse the ATOM Object_Header data table to discover display paths.
 * Each path maps a connector to an encoder chain.
 *
 * Returns the number of display paths found, or 0 on error.
 */
static int atombios_discover_display_paths(struct atom_context *ctx,
					   struct atombios_display_path *paths,
					   int max_paths)
{
	uint16_t data_offset;
	uint8_t frev, crev;
	int count = 0;
	ATOM_OBJECT_HEADER *obj_hdr;
	ATOM_DISPLAY_OBJECT_PATH_TABLE *path_tbl;
	uint8_t *base;
	int i;
	uint16_t path_offset;

	if (!atom_parse_data_header(ctx, cmd_idx.data_object_header,
				    NULL, &frev, &crev, &data_offset)) {
		printk(BIOS_WARNING, "ATOMBIOS: Object_Header table not found\n");
		return 0;
	}

	base = (uint8_t *)ctx->bios;
	obj_hdr = (ATOM_OBJECT_HEADER *)(base + data_offset);

	if (!obj_hdr->usDisplayPathTableOffset) {
		printk(BIOS_WARNING, "ATOMBIOS: no display path table\n");
		return 0;
	}

	path_tbl = (ATOM_DISPLAY_OBJECT_PATH_TABLE *)
		   (base + data_offset + obj_hdr->usDisplayPathTableOffset);

	printk(BIOS_INFO, "ATOMBIOS: found %d display paths (version %d)\n",
	       path_tbl->ucNumOfDispPath, path_tbl->ucVersion);

	/* Walk each display path */
	path_offset = (uint16_t)((uint8_t *)path_tbl->asDispPath - base);
	for (i = 0; i < path_tbl->ucNumOfDispPath && count < max_paths; i++) {
		ATOM_DISPLAY_OBJECT_PATH *disp_path;
		uint16_t conn_obj_id, enc_obj_id;
		uint8_t obj_type, obj_id, enc_id;
		int j;

		disp_path = (ATOM_DISPLAY_OBJECT_PATH *)(base + path_offset);

		conn_obj_id = disp_path->usConnObjectId;
		obj_type = (conn_obj_id & OBJECT_TYPE_MASK) >> OBJECT_TYPE_SHIFT;

		if (obj_type != GRAPH_OBJECT_TYPE_CONNECTOR ||
		    !(obj_hdr->usDeviceSupport & disp_path->usDeviceTag)) {
			/* Skip to next path using usSize */
			path_offset += disp_path->usSize;
			continue;
		}

		obj_id = (conn_obj_id & OBJECT_ID_MASK) >> OBJECT_ID_SHIFT;

		paths[count].connector_obj_id = conn_obj_id;
		paths[count].device_tag = disp_path->usDeviceTag;
		paths[count].hpd_id = ATOMBIOS_HPD_NONE;

		/* Map ATOM connector object ID to a simple type */
		switch (obj_id) {
		case CONNECTOR_OBJECT_ID_HDMI_TYPE_A:
			paths[count].connector_type = 1; /* HDMI */
			/* Linux drives HDMI as DVI/TMDS when audio/infoframes are off. */
			paths[count].encoder_mode = ATOM_ENCODER_MODE_DVI;
			break;
		case CONNECTOR_OBJECT_ID_DISPLAYPORT:
		case CONNECTOR_OBJECT_ID_eDP:
			paths[count].connector_type = 2; /* DP/eDP */
			paths[count].encoder_mode = ATOM_ENCODER_MODE_DP;
			break;
		case CONNECTOR_OBJECT_ID_LVDS:
		case CONNECTOR_OBJECT_ID_LVDS_eDP:
			paths[count].connector_type = 2; /* internal panel */
			paths[count].encoder_mode = ATOM_ENCODER_MODE_LVDS;
			break;
		case CONNECTOR_OBJECT_ID_SINGLE_LINK_DVI_D:
		case CONNECTOR_OBJECT_ID_DUAL_LINK_DVI_D:
		case CONNECTOR_OBJECT_ID_SINGLE_LINK_DVI_I:
		case CONNECTOR_OBJECT_ID_DUAL_LINK_DVI_I:
			paths[count].connector_type = 3; /* DVI */
			paths[count].encoder_mode = ATOM_ENCODER_MODE_DVI;
			break;
		case CONNECTOR_OBJECT_ID_VGA:
			paths[count].connector_type = 4; /* VGA */
			paths[count].encoder_mode = ATOM_ENCODER_MODE_CRT;
			break;
		default:
			paths[count].connector_type = 0;
			paths[count].encoder_mode = ATOM_ENCODER_MODE_DVI;
			break;
		}

		/* Walk graphic objects in the path to find encoder */
		for (j = 0; j < (disp_path->usSize - 8) / 2; j++) {
			enc_obj_id = disp_path->usGraphicObjIds[j];
			obj_type = (enc_obj_id & OBJECT_TYPE_MASK) >> OBJECT_TYPE_SHIFT;

			if (obj_type != GRAPH_OBJECT_TYPE_ENCODER)
				continue;

			enc_id = (enc_obj_id & OBJECT_ID_MASK) >> OBJECT_ID_SHIFT;
			paths[count].encoder_obj_id = enc_id;

			/* Determine encoder type from encoder object ID */
			switch (enc_id) {
			case ENCODER_OBJECT_ID_INTERNAL_LVDS:
				paths[count].encoder_mode = ATOM_ENCODER_MODE_LVDS;
				break;
			case ENCODER_OBJECT_ID_INTERNAL_TMDS1:
			case ENCODER_OBJECT_ID_INTERNAL_TMDS2:
			case ENCODER_OBJECT_ID_INTERNAL_KLDSCP_TMDS1:
				paths[count].is_legacy_tmds = 1;
				paths[count].encoder_mode = ATOM_ENCODER_MODE_DVI;
				break;
			case ENCODER_OBJECT_ID_INTERNAL_KLDSCP_DVO1:
				paths[count].encoder_mode = ATOM_ENCODER_MODE_DVI;
				break;
			case ENCODER_OBJECT_ID_INTERNAL_LVTM1:
				paths[count].is_legacy_tmds = 1;
				paths[count].encoder_mode = ATOM_ENCODER_MODE_DVI;
				break;
			case ENCODER_OBJECT_ID_INTERNAL_KLDSCP_LVTMA:
				paths[count].dig_encoder = 1; /* DIG2 on DCE3 */
				paths[count].phy_id = 1;      /* LVTMA */
				if (paths[count].connector_type == 2)
					paths[count].encoder_mode = ATOM_ENCODER_MODE_LVDS;
				break;
			case ENCODER_OBJECT_ID_INTERNAL_UNIPHY:
				paths[count].dig_encoder = 0; /* DIG1 */
				paths[count].phy_id = 0;      /* UNIPHY_A */
				break;
			case ENCODER_OBJECT_ID_INTERNAL_UNIPHY1:
				paths[count].dig_encoder = 2; /* DIG3 */
				paths[count].phy_id = 2;      /* UNIPHY_C */
				break;
			case ENCODER_OBJECT_ID_INTERNAL_UNIPHY2:
				paths[count].dig_encoder = 4; /* DIG5 */
				paths[count].phy_id = 4;      /* UNIPHY_E */
				break;
			case ENCODER_OBJECT_ID_INTERNAL_UNIPHY3:
				paths[count].dig_encoder = 6; /* DIG7 */
				paths[count].phy_id = 6;      /* UNIPHY_G */
				break;
			case ENCODER_OBJECT_ID_INTERNAL_KLDSCP_DAC1:
				paths[count].is_dac = 1;
				paths[count].dac_type = ATOM_DAC_A;
				paths[count].encoder_mode = ATOM_ENCODER_MODE_CRT;
				break;
			case ENCODER_OBJECT_ID_INTERNAL_KLDSCP_DAC2:
				paths[count].is_dac = 1;
				paths[count].dac_type = ATOM_DAC_B;
				paths[count].encoder_mode = ATOM_ENCODER_MODE_CRT;
				break;
			default:
				paths[count].dig_encoder = 0;
				paths[count].phy_id = 0;
				break;
			}

			/* For DIG encoders, check link B (ENUM_ID_2) */
			if (!paths[count].is_dac &&
			    ((enc_obj_id & ENUM_ID_MASK) >> ENUM_ID_SHIFT) == GRAPH_OBJECT_ENUM_ID2) {
				paths[count].dig_encoder += 1;
				paths[count].phy_id += 1;
			}
			break; /* use first encoder found */
		}

		atombios_read_connector_records(ctx, obj_hdr, data_offset, &paths[count]);

		count++;
		path_offset += disp_path->usSize;
	}

	atombios_apply_supported_device_info(ctx, paths, count);

	/* Look up I2C lines from GPIO_I2C_Info table */
	if (atom_parse_data_header(ctx, cmd_idx.data_gpio_i2c,
				   NULL, &frev, &crev, &data_offset)) {
		ATOM_GPIO_I2C_INFO *i2c_info;
		i2c_info = (ATOM_GPIO_I2C_INFO *)(base + data_offset);

		/* Use GPIO_I2C_Info as a fallback if connector records had no DDC. */
		for (i = 0; i < count; i++) {
			if (paths[i].i2c_valid)
				continue;
			if (i < ATOM_MAX_SUPPORTED_DEVICE) {
				paths[i].i2c_line =
					i2c_info->asGPIO_Info[i].sucI2cId.ucAccess;
				paths[i].i2c_valid = 1;
			}
		}
	}

	for (i = 0; i < count; i++) {
		if (paths[i].is_dac)
			printk(BIOS_INFO,
			       "ATOMBIOS: path %d: tag=0x%04x conn=0x%04x type=%d i2c=0x%02x DAC-%c mode=CRT\n",
			       i, paths[i].device_tag, paths[i].connector_obj_id,
			       paths[i].connector_type, paths[i].i2c_line,
			       paths[i].dac_type == ATOM_DAC_A ? 'A' : 'B');
		else if (paths[i].is_legacy_tmds)
			printk(BIOS_INFO,
			       "ATOMBIOS: path %d: tag=0x%04x conn=0x%04x type=%d "
			       "i2c=0x%02x enc=0x%02x legacy TMDS mode=%d\n",
			       i, paths[i].device_tag, paths[i].connector_obj_id,
			       paths[i].connector_type, paths[i].i2c_line,
			       paths[i].encoder_obj_id, paths[i].encoder_mode);
		else
			printk(BIOS_INFO,
			       "ATOMBIOS: path %d: tag=0x%04x conn=0x%04x type=%d "
			       "i2c=0x%02x enc=0x%02x dig=%d phy=%d mode=%d\n",
			       i, paths[i].device_tag, paths[i].connector_obj_id,
			       paths[i].connector_type, paths[i].i2c_line,
			       paths[i].encoder_obj_id, paths[i].dig_encoder,
			       paths[i].phy_id, paths[i].encoder_mode);
	}

	return count;
}


/* ---- Legacy TMDS/LVTMA Encoder Control ---- */

static int atombios_legacy_tmds_encoder_index(const struct atombios_display_path *path)
{
	switch (path->encoder_obj_id) {
	case ENCODER_OBJECT_ID_INTERNAL_TMDS1:
	case ENCODER_OBJECT_ID_INTERNAL_KLDSCP_TMDS1:
		return cmd_idx.tmds1_encoder_ctrl;
	case ENCODER_OBJECT_ID_INTERNAL_TMDS2:
	case ENCODER_OBJECT_ID_INTERNAL_LVTM1:
		return cmd_idx.tmds2_encoder_ctrl;
	default:
		return -1;
	}
}

static int atombios_legacy_tmds_output_index(const struct atombios_display_path *path)
{
	switch (path->encoder_obj_id) {
	case ENCODER_OBJECT_ID_INTERNAL_TMDS1:
	case ENCODER_OBJECT_ID_INTERNAL_KLDSCP_TMDS1:
		return cmd_idx.tmds1_output_ctrl;
	case ENCODER_OBJECT_ID_INTERNAL_TMDS2:
	case ENCODER_OBJECT_ID_INTERNAL_LVTM1:
		return cmd_idx.tmds2_output_ctrl;
	default:
		return -1;
	}
}

static int atombios_legacy_tmds_encoder_setup(struct atom_context *ctx,
					      const struct atombios_display_path *path,
					      uint16_t pixel_clock_10khz,
					      uint8_t action)
{
	uint8_t frev, crev;
	int index = atombios_legacy_tmds_encoder_index(path);
	union {
		LVDS_ENCODER_CONTROL_PARAMETERS v1;
		LVDS_ENCODER_CONTROL_PARAMETERS_V2 v2;
	} args;

	if (index < 0)
		return -1;
	if (!atom_parse_cmd_header(ctx, index, &frev, &crev))
		return -1;

	memset(&args, 0, sizeof(args));
	args.v1.usPixelClock = pixel_clock_10khz;
	args.v1.ucAction = action;
	args.v1.ucMisc = ATOM_PANEL_MISC_888RGB;
	if (pixel_clock_10khz > 16500)
		args.v1.ucMisc |= PANEL_ENCODER_MISC_DUAL;

	printk(BIOS_DEBUG,
	       "ATOMBIOS: legacy TMDS encoder setup table=%d frev=%d crev=%d action=%d clock=%d*10kHz\n",
	       index, frev, crev, action, pixel_clock_10khz);

	return atom_execute_table(ctx, index, (uint32_t *)&args, sizeof(args));
}

static int atombios_legacy_tmds_output_control(struct atom_context *ctx,
					       const struct atombios_display_path *path,
					       uint8_t action)
{
	DISPLAY_DEVICE_OUTPUT_CONTROL_PS_ALLOCATION args;
	int index = atombios_legacy_tmds_output_index(path);

	if (index < 0)
		return -1;

	memset(&args, 0, sizeof(args));
	args.ucAction = action;

	printk(BIOS_DEBUG, "ATOMBIOS: legacy TMDS output table=%d action=%d\n",
	       index, action);

	return atom_execute_table(ctx, index, (uint32_t *)&args, sizeof(args));
}

/* ---- DIG Encoder Control ---- */

/*
 * Configure the DIG encoder block for the specified display mode.
 *
 * This calls the DIGxEncoderControl ATOM command table which programs
 * the DIG encoder for DP, HDMI, DVI, or LVDS mode.
 */
static int atombios_dig_encoder_setup(struct atom_context *ctx,
				      const struct atombios_display_path *path,
				      uint16_t pixel_clock_10khz,
				      uint8_t action, uint8_t bpc)
{
	uint8_t frev, crev;
	uint8_t lane_num;
	union {
		DIG_ENCODER_CONTROL_PARAMETERS      v1;
		DIG_ENCODER_CONTROL_PARAMETERS_V2   v2;
		DIG_ENCODER_CONTROL_PARAMETERS_V3   v3;
		DIG_ENCODER_CONTROL_PARAMETERS_V4   v4;
		DIG_ENCODER_CONTROL_PARAMETERS_V5   v5;
	} args;

	if (!atom_parse_cmd_header(ctx, cmd_idx.dig_encoder_ctrl, &frev, &crev))
		return -1;

	memset(&args, 0, sizeof(args));

	if (path->encoder_mode == ATOM_ENCODER_MODE_DP && path->dp_lane_count)
		lane_num = path->dp_lane_count;
	else if (path->encoder_mode == ATOM_ENCODER_MODE_DVI && pixel_clock_10khz > 16500)
		lane_num = 8;
	else
		lane_num = 4;

	printk(BIOS_DEBUG,
	       "ATOMBIOS: DIG encoder setup frev=%d crev=%d action=0x%02x "
	       "mode=%d lanes=%d link_bw=0x%02x\n",
	       frev, crev, action, path->encoder_mode, lane_num, path->dp_link_bw);

	switch (crev) {
	case 1:
		args.v1.ucAction = action;
		args.v1.usPixelClock = pixel_clock_10khz;
		args.v1.ucEncoderMode = path->encoder_mode;
		args.v1.ucLaneNum = lane_num;
		if (path->phy_id / 2 == 1)
			args.v1.ucConfig |= ATOM_ENCODER_CONFIG_TRANSMITTER2;
		else if (path->phy_id / 2 == 2)
			args.v1.ucConfig |= 0x10;
		if (path->dig_encoder & 1)
			args.v1.ucConfig |= ATOM_ENCODER_CONFIG_LINKB;
		if (path->encoder_mode == ATOM_ENCODER_MODE_DP &&
		    path->dp_link_bw >= DP_LINK_BW_2_7)
			args.v1.ucConfig |= ATOM_ENCODER_CONFIG_DPLINKRATE_2_70GHZ;
		break;
	case 2:
		args.v2.ucAction = action;
		args.v2.usPixelClock = pixel_clock_10khz;
		args.v2.ucEncoderMode = path->encoder_mode;
		args.v2.ucLaneNum = lane_num;
		args.v2.acConfig.ucLinkSel = path->dig_encoder & 1;
		args.v2.acConfig.ucTransmitterSel = path->phy_id / 2;
		if (path->encoder_mode == ATOM_ENCODER_MODE_DP &&
		    path->dp_link_bw >= DP_LINK_BW_2_7)
			args.v2.acConfig.ucDPLinkRate = 1;
		break;
	case 3:
		args.v3.ucAction = action;
		args.v3.usPixelClock = pixel_clock_10khz;
		args.v3.ucEncoderMode = path->encoder_mode;
		args.v3.ucLaneNum = lane_num;
		args.v3.ucBitPerColor = bpc;
		args.v3.acConfig.ucDigSel = path->dig_encoder;
		if (path->encoder_mode == ATOM_ENCODER_MODE_DP &&
		    path->dp_link_bw >= DP_LINK_BW_2_7)
			args.v3.acConfig.ucDPLinkRate = 1;
		break;
	case 4:
		args.v4.ucAction = action;
		args.v4.usPixelClock = pixel_clock_10khz;
		args.v4.ucEncoderMode = path->encoder_mode;
		args.v4.ucLaneNum = lane_num;
		args.v4.ucBitPerColor = bpc;
		args.v4.ucHPD_ID = path->hpd_id <= 5 ? path->hpd_id + 1 : 0;
		args.v4.acConfig.ucDigSel = path->dig_encoder;
		if (path->encoder_mode == ATOM_ENCODER_MODE_DP) {
			if (path->dp_link_bw >= DP_LINK_BW_5_4)
				args.v4.acConfig.ucDPLinkRate =
					ATOM_ENCODER_CONFIG_V4_DPLINKRATE_5_40GHZ;
			else if (path->dp_link_bw >= DP_LINK_BW_2_7)
				args.v4.acConfig.ucDPLinkRate =
					ATOM_ENCODER_CONFIG_V4_DPLINKRATE_2_70GHZ;
			else
				args.v4.acConfig.ucDPLinkRate =
					ATOM_ENCODER_CONFIG_V4_DPLINKRATE_1_62GHZ;
		}
		break;
	case 5:
	default:
		if (action == ATOM_ENCODER_CMD_STREAM_SETUP) {
			args.v5.asStreamParam.ucDigId = path->dig_encoder;
			args.v5.asStreamParam.ucAction = action;
			args.v5.asStreamParam.ucDigMode = path->encoder_mode;
			args.v5.asStreamParam.ucLaneNum = lane_num;
			args.v5.asStreamParam.ulPixelClock = pixel_clock_10khz;
			args.v5.asStreamParam.ucBitPerColor = bpc;
			if (path->encoder_mode == ATOM_ENCODER_MODE_DP)
				args.v5.asStreamParam.ucLinkRateIn270Mhz = path->dp_link_bw;
		} else {
			args.v5.asCmdParam.ucDigId = path->dig_encoder;
			args.v5.asCmdParam.ucAction = action;
		}
		break;
	}

	return atom_execute_table(ctx, cmd_idx.dig_encoder_ctrl,
				  (uint32_t *)&args,
				  sizeof(args));
}

/* ---- UNIPHY Transmitter Control ---- */

/*
 * Control the UNIPHY transmitter (physical link layer).
 *
 * Actions: INIT, ENABLE, DISABLE, SETUP_VSEMPH, POWER_ON/OFF.
 * This programs the PHY for the target link rate and drives the
 * physical signals.
 */
static int atombios_transmitter_control(struct atom_context *ctx,
					const struct atombios_display_path *path,
					uint16_t pixel_clock_10khz,
					uint8_t action,
					uint8_t lane_num, uint8_t lane_set)
{
	uint8_t frev, crev;
	uint8_t transmitter_lane_num;
	uint16_t sym_clock_10khz;
	union {
		DIG_TRANSMITTER_CONTROL_PARAMETERS       v1;
		DIG_TRANSMITTER_CONTROL_PARAMETERS_V2    v2;
		DIG_TRANSMITTER_CONTROL_PARAMETERS_V3    v3;
		DIG_TRANSMITTER_CONTROL_PARAMETERS_V4    v4;
		DIG_TRANSMITTER_CONTROL_PARAMETERS_V1_5  v5;
		DIG_TRANSMITTER_CONTROL_PARAMETERS_V1_6  v6;
	} args;

	if (!atom_parse_cmd_header(ctx, cmd_idx.dig1_xmtr_ctrl, &frev, &crev))
		return -1;

	memset(&args, 0, sizeof(args));

	transmitter_lane_num = lane_num;
	if (!transmitter_lane_num) {
		if (path->encoder_mode == ATOM_ENCODER_MODE_DP && path->dp_lane_count)
			transmitter_lane_num = path->dp_lane_count;
		else if (path->encoder_mode == ATOM_ENCODER_MODE_DVI &&
			 pixel_clock_10khz > 16500)
			transmitter_lane_num = 8;
		else
			transmitter_lane_num = 4;
	}

	sym_clock_10khz = pixel_clock_10khz;
	if (path->encoder_mode == ATOM_ENCODER_MODE_DP && path->dp_link_bw)
		sym_clock_10khz = dp_link_bw_code_to_clock_10khz(path->dp_link_bw);

	printk(BIOS_DEBUG,
	       "ATOMBIOS: transmitter ctrl frev=%d crev=%d action=%d "
	       "phy=%d lanes=%d symclk=%d*10kHz\n",
	       frev, crev, action, path->phy_id, transmitter_lane_num,
	       sym_clock_10khz);

	switch (crev) {
	case 1:
		args.v1.ucAction = action;
		if (action == ATOM_TRANSMITTER_ACTION_SETUP_VSEMPH) {
			args.v1.asMode.ucLaneSel = transmitter_lane_num;
			args.v1.asMode.ucLaneSet = lane_set;
		} else if (action == ATOM_TRANSMITTER_ACTION_INIT) {
			args.v1.usInitInfo = path->connector_obj_id;
		} else {
			args.v1.usPixelClock = sym_clock_10khz;
		}
		if (path->encoder_mode == ATOM_ENCODER_MODE_HDMI ||
		    path->encoder_mode == ATOM_ENCODER_MODE_DP)
			args.v1.ucConfig |= (1 << 1);
		break;
	case 2:
		args.v2.ucAction = action;
		if (action == ATOM_TRANSMITTER_ACTION_SETUP_VSEMPH) {
			args.v2.asMode.ucLaneSel = transmitter_lane_num;
			args.v2.asMode.ucLaneSet = lane_set;
		} else if (action == ATOM_TRANSMITTER_ACTION_INIT) {
			args.v2.usInitInfo = path->connector_obj_id;
		} else {
			args.v2.usPixelClock = sym_clock_10khz;
		}
		args.v2.acConfig.ucEncoderSel = (path->dig_encoder & 1);
		args.v2.acConfig.ucTransmitterSel = path->phy_id / 2;
		if (path->encoder_mode == ATOM_ENCODER_MODE_DP)
			args.v2.acConfig.fDPConnector = 1;
		if (path->encoder_mode == ATOM_ENCODER_MODE_HDMI ||
		    path->encoder_mode == ATOM_ENCODER_MODE_DP)
			args.v2.acConfig.fCoherentMode = 1;
		break;
	case 3:
		args.v3.ucAction = action;
		if (action == ATOM_TRANSMITTER_ACTION_SETUP_VSEMPH) {
			args.v3.asMode.ucLaneSel = transmitter_lane_num;
			args.v3.asMode.ucLaneSet = lane_set;
		} else if (action == ATOM_TRANSMITTER_ACTION_INIT) {
			args.v3.usInitInfo = path->connector_obj_id;
		} else {
			args.v3.usPixelClock = sym_clock_10khz;
		}
		args.v3.ucLaneNum = transmitter_lane_num;
		args.v3.acConfig.ucEncoderSel = (path->dig_encoder & 1);
		args.v3.acConfig.ucTransmitterSel = path->phy_id / 2;
		if (path->encoder_mode == ATOM_ENCODER_MODE_HDMI ||
		    path->encoder_mode == ATOM_ENCODER_MODE_DP)
			args.v3.acConfig.fCoherentMode = 1;
		break;
	case 4:
		args.v4.ucAction = action;
		if (action == ATOM_TRANSMITTER_ACTION_SETUP_VSEMPH) {
			args.v4.asMode.ucLaneSel = transmitter_lane_num;
			args.v4.asMode.ucLaneSet = lane_set;
		} else if (action == ATOM_TRANSMITTER_ACTION_INIT) {
			args.v4.usInitInfo = path->connector_obj_id;
		} else {
			args.v4.usPixelClock = sym_clock_10khz;
		}
		args.v4.ucLaneNum = transmitter_lane_num;
		args.v4.acConfig.ucEncoderSel = (path->dig_encoder & 1);
		args.v4.acConfig.ucTransmitterSel = path->phy_id / 2;
		if (path->encoder_mode == ATOM_ENCODER_MODE_HDMI ||
		    path->encoder_mode == ATOM_ENCODER_MODE_DP)
			args.v4.acConfig.fCoherentMode = 1;
		break;
	case 5:
		args.v5.ucAction = action;
		args.v5.usSymClock = sym_clock_10khz;
		args.v5.ucPhyId = path->phy_id;
		args.v5.ucLaneNum = transmitter_lane_num;
		args.v5.ucConnObjId = atombios_object_id(path->connector_obj_id);
		args.v5.ucDigMode = path->encoder_mode;
		args.v5.ucDigEncoderSel = (1 << path->dig_encoder);
		args.v5.ucDPLaneSet = lane_set;
		if (path->hpd_id <= 5)
			args.v5.asConfig.ucHPDSel = path->hpd_id + 1;
		if (path->encoder_mode == ATOM_ENCODER_MODE_HDMI ||
		    path->encoder_mode == ATOM_ENCODER_MODE_DP)
			args.v5.asConfig.ucCoherentMode = 1;
		break;
	case 6:
	default:
		args.v6.ucAction = action;
		args.v6.ucPhyId = path->phy_id;
		if (action == ATOM_TRANSMITTER_ACTION_SETUP_VSEMPH)
			args.v6.ucDPLaneSet = lane_set;
		else
			args.v6.ucDigMode = path->encoder_mode;
		args.v6.ucLaneNum = transmitter_lane_num;
		args.v6.ulSymClock = sym_clock_10khz;
		args.v6.ucConnObjId = atombios_object_id(path->connector_obj_id);
		args.v6.ucDigEncoderSel = (1 << path->dig_encoder);
		if (path->hpd_id <= 5)
			args.v6.ucHPDSel = path->hpd_id + 1;
		break;
	}

	return atom_execute_table(ctx, cmd_idx.dig1_xmtr_ctrl,
				  (uint32_t *)&args,
				  sizeof(args));
}

/* ---- CRTC Source Selection ---- */

/*
 * Route a CRTC to an encoder via the SelectCRTC_Source ATOM table.
 */
static int atombios_select_crtc_source(struct atom_context *ctx,
				       const struct atombios_display_path *path,
				       int crtc_id)
{
	uint8_t frev, crev;
	union {
		SELECT_CRTC_SOURCE_PARAMETERS    v1;
		SELECT_CRTC_SOURCE_PARAMETERS_V2 v2;
		SELECT_CRTC_SOURCE_PARAMETERS_V3 v3;
	} args;
	uint8_t enc_id = ASIC_INT_DIG1_ENCODER_ID;
	int dev_index;

	if (!atom_parse_cmd_header(ctx, cmd_idx.select_crtc_source,
				   &frev, &crev))
		return -1;

	memset(&args, 0, sizeof(args));

	/* Map encoder to the hardware encoder ID. Legacy TMDS paths use
	 * SelectCRTC_Source v1 on the hardware this driver currently targets. */
	if (path->is_dac) {
		enc_id = (path->dac_type == ATOM_DAC_A)
			 ? ASIC_INT_DAC1_ENCODER_ID
			 : ASIC_INT_DAC2_ENCODER_ID;
	} else if (path->is_legacy_tmds) {
		enc_id = ASIC_INT_DIG1_ENCODER_ID;
	} else {
		switch (path->dig_encoder) {
		case 0: enc_id = ASIC_INT_DIG1_ENCODER_ID; break;
		case 1: enc_id = ASIC_INT_DIG2_ENCODER_ID; break;
		case 2: enc_id = ASIC_INT_DIG3_ENCODER_ID; break;
		case 3: enc_id = ASIC_INT_DIG4_ENCODER_ID; break;
		case 4: enc_id = ASIC_INT_DIG5_ENCODER_ID; break;
		case 5: enc_id = ASIC_INT_DIG6_ENCODER_ID; break;
		default: enc_id = ASIC_INT_DIG7_ENCODER_ID; break;
		}
	}

	dev_index = atombios_device_tag_index(path->device_tag);

	switch (crev) {
	case 1:
		args.v1.ucCRTC = crtc_id;
		if (dev_index >= 0)
			args.v1.ucDevice = dev_index;
		else if (path->is_dac)
			args.v1.ucDevice = (path->dac_type == ATOM_DAC_A)
					   ? ATOM_DEVICE_CRT1_INDEX
					   : ATOM_DEVICE_CRT2_INDEX;
		else
			args.v1.ucDevice = ATOM_DEVICE_DFP1_INDEX;
		break;
	case 2:
		args.v2.ucCRTC = crtc_id;
		args.v2.ucEncoderID = enc_id;
		args.v2.ucEncodeMode = path->encoder_mode;
		break;
	case 3:
	default:
		args.v3.ucCRTC = crtc_id;
		args.v3.ucEncoderID = enc_id;
		args.v3.ucEncodeMode = path->encoder_mode;
		args.v3.ucDstBpc = PANEL_8BIT_PER_COLOR;
		break;
	}

	printk(BIOS_DEBUG,
	       "ATOMBIOS: select CRTC %d -> enc_id=0x%02x mode=%d%s\n",
	       crtc_id, enc_id, path->encoder_mode,
	       path->is_dac ? " (DAC)" : "");

	return atom_execute_table(ctx, cmd_idx.select_crtc_source,
				  (uint32_t *)&args,
				  sizeof(args));
}

/* ---- Basic CRTC operations ---- */

static int atombios_set_crtc_dtd_timing(struct atom_context *ctx,
					 const struct edid_mode *mode,
					 int crtc_id)
{
	SET_CRTC_USING_DTD_TIMING_PARAMETERS args;
	uint16_t misc_flags = 0;

	memset(&args, 0, sizeof(args));

	/* ATOM polarity bits mean active low. */
	if (!mode->phsync)
		misc_flags |= ATOM_HSYNC_POLARITY;
	if (!mode->pvsync)
		misc_flags |= ATOM_VSYNC_POLARITY;

	args.usH_Size = mode->ha;
	args.usH_Blanking_Time = mode->hbl;
	args.usV_Size = mode->va;
	args.usV_Blanking_Time = mode->vbl;
	args.usH_SyncOffset = mode->hso;
	args.usH_SyncWidth = mode->hspw;
	args.usV_SyncOffset = mode->vso;
	args.usV_SyncWidth = mode->vspw;
	args.susModeMiscInfo.usAccess = misc_flags;
	args.ucH_Border = mode->hborder;
	args.ucV_Border = mode->vborder;
	args.ucCRTC = crtc_id;

	printk(BIOS_DEBUG,
	       "ATOMBIOS: DTD h=%u+%u sync=%u/%u v=%u+%u sync=%u/%u misc=0x%04x crtc=%u\n",
	       mode->ha, mode->hbl, mode->hso, mode->hspw, mode->va,
	       mode->vbl, mode->vso, mode->vspw, misc_flags, crtc_id);

	return atombios_exec(ctx, cmd_idx.set_crtc_using_dtd,
			     (uint32_t *)&args, sizeof(args));
}

static int atombios_enable_crtc(struct atom_context *ctx, int crtc_id,
				int enable)
{
	uint32_t params[1];
	params[0] = crtc_id | ((enable ? ATOM_ENABLE : ATOM_DISABLE) << 8);
	return atombios_exec(ctx, cmd_idx.enable_crtc, params, sizeof(params));
}

static int atombios_blank_crtc(struct atom_context *ctx, int crtc_id, int blank)
{
	uint32_t params[1];
	params[0] = crtc_id | ((blank ? ATOM_BLANKING : ATOM_BLANKING_OFF) << 8);
	return atombios_exec(ctx, cmd_idx.blank_crtc, params, sizeof(params));
}

static int atombios_disp_power_gating(struct atom_context *ctx, int crtc_id,
				      int gate)
{
	uint32_t params[1];
	params[0] = crtc_id | ((gate ? ATOM_DISABLE : ATOM_ENABLE) << 8);
	return atombios_exec(ctx, cmd_idx.enable_disp_power_gating, params, sizeof(params));
}

static int atombios_set_crtc_overscan(struct atom_context *ctx, int crtc_id)
{
	SET_CRTC_OVERSCAN_PARAMETERS args;

	if (cmd_idx.set_crtc_overscan < 0)
		return 0;

	memset(&args, 0, sizeof(args));
	args.ucCRTC = crtc_id;
	printk(BIOS_DEBUG, "ATOMBIOS: disabling CRTC overscan\n");
	return atombios_exec(ctx, cmd_idx.set_crtc_overscan,
			     (uint32_t *)&args, sizeof(args));
}

static int atombios_disable_scaler(struct atom_context *ctx, int crtc_id)
{
	ENABLE_SCALER_PARAMETERS args;

	if (cmd_idx.enable_scaler < 0)
		return 0;

	memset(&args, 0, sizeof(args));
	args.ucScaler = crtc_id;
	args.ucEnable = ATOM_SCALER_DISABLE;
	printk(BIOS_DEBUG, "ATOMBIOS: disabling scaler %d\n", crtc_id);
	return atombios_exec(ctx, cmd_idx.enable_scaler,
			     (uint32_t *)&args, sizeof(args));
}

static int atombios_yuv_setup(struct atom_context *ctx, int crtc_id, bool enable)
{
	ENABLE_YUV_PARAMETERS args;
	uint32_t scratch;
	int ret;

	if (cmd_idx.enable_yuv < 0)
		return 0;

	memset(&args, 0, sizeof(args));
	args.ucEnable = enable ? ATOM_ENABLE : ATOM_DISABLE;
	args.ucCRTC = crtc_id;

	/* Linux clears BIOS_3_SCRATCH while disabling YUV for RGB outputs. */
	scratch = atombios_scratch_read(ctx, R600_BIOS_3_SCRATCH);
	atombios_scratch_write(ctx, R600_BIOS_3_SCRATCH, 0);
	printk(BIOS_DEBUG, "ATOMBIOS: %s YUV on CRTC %d\n",
	       enable ? "enabling" : "disabling", crtc_id);
	ret = atombios_exec(ctx, cmd_idx.enable_yuv,
			    (uint32_t *)&args, sizeof(args));
	atombios_scratch_write(ctx, R600_BIOS_3_SCRATCH, scratch);

	return ret;
}

static void atombios_program_crtc_postmode(struct atom_context *ctx, int crtc_id)
{
	atombios_yuv_setup(ctx, crtc_id, false);
	atombios_set_crtc_overscan(ctx, crtc_id);
	atombios_disable_scaler(ctx, crtc_id);
}

static resource_t atombios_get_bar(struct device *dev, unsigned int bar)
{
	struct resource *res = probe_resource(dev, bar);
	uint32_t lo;
	resource_t base;

	if (res && res->base)
		return res->base;

	lo = pci_read_config32(dev, bar);
	base = lo & ~0xfu;
	if ((lo & PCI_BASE_ADDRESS_MEM_LIMIT_MASK) == PCI_BASE_ADDRESS_MEM_LIMIT_64)
		base |= (resource_t)pci_read_config32(dev, bar + 4) << 32;

	return base;
}

/* ---- Main init entry point ---- */

static void atombios_init(struct device *dev)
{
	struct card_info *card;
	struct atom_context *ctx;
	void *vbios;
	resource_t mmio_bar, fb_bar;
	void *mmio_base;
	uint32_t *scratch;
	int crtc_id = ATOM_CRTC1;
	struct edid_mode mode;
	struct atombios_display_path paths[8];
	int num_paths;
	int active_path = -1;
	uint8_t edid_raw[EDID_BLOCK_SIZE];
	struct edid edid;
	uint16_t pixel_clock_10khz;
	bool asic_init_ok = false;
	struct atombios_gpu_profile gpu = atombios_get_gpu_profile(dev->device);
	int i;

	printk(BIOS_INFO, "ATOMBIOS: initializing AMD GPU %04x:%04x\n",
	       dev->vendor, dev->device);

	if (!gpu.supported || !gpu.has_display) {
		printk(BIOS_WARNING,
		       "ATOMBIOS: GPU %04x has no supported display engine\n",
		       dev->device);
		return;
	}

	/* Map MMIO BAR */
	mmio_bar = atombios_get_bar(dev, AMD_GPU_MMIO_BAR);
	if (!mmio_bar)
		mmio_bar = atombios_get_bar(dev, PCI_BASE_ADDRESS_2);
	if (!mmio_bar) {
		printk(BIOS_ERR, "ATOMBIOS: could not find MMIO BAR\n");
		return;
	}
	mmio_base = (void *)(uintptr_t)mmio_bar;
	printk(BIOS_INFO, "ATOMBIOS: MMIO base at %p\n", mmio_base);

	/* Get framebuffer BAR.  This is often a 64-bit BAR above 4 GiB. */
	fb_bar = atombios_get_bar(dev, AMD_GPU_FB_BAR);
	if (!fb_bar) {
		printk(BIOS_ERR, "ATOMBIOS: could not find FB BAR\n");
		return;
	}
	printk(BIOS_INFO, "ATOMBIOS: framebuffer aperture at 0x%llx\n",
	       (unsigned long long)fb_bar);

	/* Load VBIOS */
	vbios = load_vbios(dev);
	if (!vbios)
		return;

	/* Set up card_info callbacks */
	card = calloc(1, sizeof(struct card_info));
	if (!card) {
		printk(BIOS_ERR, "ATOMBIOS: failed to allocate card_info\n");
		return;
	}
	card->mmio_base = mmio_base;
	card->reg_write = cail_reg_write;
	card->reg_read = cail_reg_read;
	card->mc_write = cail_mc_write;
	card->mc_read = cail_mc_read;
	card->pll_write = cail_pll_write;
	card->pll_read = cail_pll_read;

	/* Parse the ATOM BIOS */
	ctx = atom_parse(card, vbios);
	if (!ctx) {
		printk(BIOS_ERR, "ATOMBIOS: failed to parse VBIOS\n");
		return;
	}

	printk(BIOS_INFO, "ATOMBIOS: VBIOS \"%s\" date %s version 0x%08x\n",
	       ctx->name, ctx->date, ctx->version);

	/* Detect v1 (atombios.h) vs v2 (atomfirmware.h) table format.
	 * v2 display object parsing is different, so fail safely until that
	 * path is implemented instead of using v1 object-table casts. */
	if (atombios_detect_v2(ctx)) {
		atombios_init_cmd_indices_v2(&cmd_idx);
		printk(BIOS_WARNING,
		       "ATOMBIOS: atomfirmware.h (v2) display tables are not supported yet\n");
		atom_destroy(ctx);
		return;
	}

	atombios_init_cmd_indices_v1(&cmd_idx);
	printk(BIOS_INFO, "ATOMBIOS: using atombios.h (v1) table format\n");

	/* Allocate scratch memory */
	scratch = calloc(ATOM_SCRATCH_SIZE_DWORDS, sizeof(uint32_t));
	if (!scratch) {
		printk(BIOS_ERR, "ATOMBIOS: failed to allocate scratch memory\n");
		atom_destroy(ctx);
		return;
	}
	ctx->scratch = scratch;
	ctx->scratch_size_bytes = ATOM_SCRATCH_SIZE_DWORDS * sizeof(uint32_t);

	/* Execute AsicInit */
	printk(BIOS_INFO, "ATOMBIOS: executing AsicInit...\n");
	if (atom_asic_init(ctx))
		printk(BIOS_WARNING, "ATOMBIOS: AsicInit failed or not present\n");
	else
		asic_init_ok = true;

	/* Discover display paths from ATOM object tables */
	memset(paths, 0, sizeof(paths));
	num_paths = atombios_discover_display_paths(ctx, paths, 8);
	if (!num_paths) {
		printk(BIOS_WARNING,
		       "ATOMBIOS: no display paths found, using defaults\n");
		goto use_defaults;
	}

	if (!asic_init_ok) {
		printk(BIOS_WARNING,
		       "ATOMBIOS: skipping EDID probe because AsicInit failed\n");
		goto use_defaults;
	}

	/* Try to read EDID on digital paths to find a connected display. */
	for (i = 0; i < num_paths; i++) {
		if (paths[i].connector_type == 0 || paths[i].is_dac ||
		    !paths[i].i2c_valid)
			continue;

		if (atombios_read_edid(ctx, paths[i].i2c_line, edid_raw) == 0) {
			if (decode_edid(edid_raw, EDID_BLOCK_SIZE, &edid) == EDID_CONFORMANT) {
				active_path = i;
				printk(BIOS_INFO,
				       "ATOMBIOS: found display on path %d: %s %dx%d\n",
				       i, edid.ascii_string,
				       edid.mode.ha, edid.mode.va);
				break;
			}
		}
	}

	if (active_path < 0) {
		printk(BIOS_WARNING,
		       "ATOMBIOS: no connected display found via EDID\n");
		goto use_defaults;
	}

	/* Use EDID-discovered mode */
	memcpy(&mode, &edid.mode, sizeof(mode));
	pixel_clock_10khz = mode.pixel_clock / 10;
	goto do_modeset;

use_defaults:
	/* Safe default: 1024x768@60 */
	memset(&mode, 0, sizeof(mode));
	mode.pixel_clock = 65000;
	mode.ha = 1024;
	mode.hbl = 320;
	mode.hso = 24;
	mode.hspw = 136;
	mode.va = 768;
	mode.vbl = 38;
	mode.vso = 3;
	mode.vspw = 6;
	pixel_clock_10khz = 6500;

	/* If EDID failed, prefer a digital path.  This matches Linux DVI
	 * behavior: DDC/EDID decides digital first; DAC load detect is only an
	 * analog fallback for DVI-I/VGA. */
	if (active_path < 0) {
		for (i = 0; i < num_paths; i++) {
			if (paths[i].connector_type != 0 && !paths[i].is_dac) {
				active_path = i;
				break;
			}
		}
	}
	if (active_path < 0) {
		for (i = 0; i < num_paths; i++) {
			if (paths[i].connector_type != 0 && paths[i].is_dac) {
				atombios_dac_load_detect(ctx, &paths[i]);
				printk(BIOS_INFO,
				       "ATOMBIOS: DAC load detection on path %d (VGA/CRT)\n",
				       i);
				active_path = i;
				break;
			}
		}
	}

do_modeset:
	printk(BIOS_INFO, "ATOMBIOS: modesetting %ux%u @ %u kHz\n",
	       mode.ha, mode.va, mode.pixel_clock);

	if (active_path >= 0) {
		atombios_update_scratch_for_path(ctx, &paths[active_path],
						 crtc_id, true, false);
		atombios_output_lock(ctx, true);
	}

	/* Step 1: Power up display pipe */
	atombios_disp_power_gating(ctx, crtc_id, 0);
	atombios_set_disp_eng_clock(ctx, &gpu);

	if (active_path >= 0 && paths[active_path].is_dac) {
		/*
		 * VGA/CRT output path: DAC encoder.
		 * Simpler than digital -- no transmitter PHY or link training.
		 */
		printk(BIOS_INFO, "ATOMBIOS: VGA/CRT output via DAC-%c\n",
		       paths[active_path].dac_type == ATOM_DAC_A ? 'A' : 'B');

		/* Route CRTC to DAC encoder */
		atombios_select_crtc_source(ctx, &paths[active_path], crtc_id);

		/* Program pixel clock */
		atombios_set_pixel_clock(ctx, &gpu, &paths[active_path],
					 pixel_clock_10khz, crtc_id);

		/* Program CRTC timing */
		printk(BIOS_INFO, "ATOMBIOS: programming CRTC timing...\n");
		atombios_set_crtc_dtd_timing(ctx, &mode, crtc_id);

		/* Program CRTC scanout surface */
		atombios_program_framebuffer(ctx, &gpu, fb_bar, &mode);
		atombios_program_crtc_postmode(ctx, crtc_id);

		/* Enable CRTC */
		atombios_enable_crtc(ctx, crtc_id, 1);

		/* Enable DAC encoder */
		atombios_dac_encoder_setup(ctx, &paths[active_path],
					   pixel_clock_10khz, ATOM_ENABLE);

		/* Unblank */
		atombios_blank_crtc(ctx, crtc_id, 0);
		atombios_evergreen_unblank_scanout(ctx, &gpu);
		atombios_update_scratch_for_path(ctx, &paths[active_path],
						 crtc_id, true, true);
		atombios_output_lock(ctx, false);
	} else if (active_path >= 0 && paths[active_path].is_legacy_tmds) {
		/*
		 * Legacy DVI/TMDS output path (pre-UNIPHY).  Linux handles these
		 * with TMDSA/LVTMA encoder and output-control tables, not DIGx
		 * encoder or UNIPHY transmitter tables.
		 */
		printk(BIOS_INFO, "ATOMBIOS: DVI output via legacy TMDS encoder 0x%02x\n",
		       paths[active_path].encoder_obj_id);

		atombios_legacy_tmds_output_control(ctx, &paths[active_path],
						    ATOM_DISABLE);

		/* Route CRTC to TMDS device */
		atombios_select_crtc_source(ctx, &paths[active_path], crtc_id);

		/* Program pixel clock */
		atombios_set_pixel_clock(ctx, &gpu, &paths[active_path],
					 pixel_clock_10khz, crtc_id);

		/* Program CRTC timing */
		printk(BIOS_INFO, "ATOMBIOS: programming CRTC timing...\n");
		atombios_set_crtc_dtd_timing(ctx, &mode, crtc_id);

		/* Program CRTC scanout surface */
		atombios_program_framebuffer(ctx, &gpu, fb_bar, &mode);
		atombios_program_crtc_postmode(ctx, crtc_id);

		/* Enable CRTC */
		atombios_enable_crtc(ctx, crtc_id, 1);

		/* Setup TMDS encoder and enable output */
		atombios_legacy_tmds_encoder_setup(ctx, &paths[active_path],
						   pixel_clock_10khz, ATOM_ENABLE);
		atombios_legacy_tmds_output_control(ctx, &paths[active_path],
						    ATOM_ENABLE);

		/* Unblank */
		atombios_blank_crtc(ctx, crtc_id, 0);
		atombios_evergreen_unblank_scanout(ctx, &gpu);
		atombios_update_scratch_for_path(ctx, &paths[active_path],
						 crtc_id, true, true);
		atombios_output_lock(ctx, false);
	} else {
		/*
		 * Digital output path: DIG encoder + UNIPHY transmitter.
		 */

		/* For eDP, power on the panel first */
		if (active_path >= 0 &&
		    paths[active_path].encoder_mode == ATOM_ENCODER_MODE_DP) {
			uint8_t conn_id = (paths[active_path].connector_obj_id >> 8) & 0xF;
			if (conn_id == CONNECTOR_OBJECT_ID_eDP) {
				atombios_edp_panel_power(ctx, &paths[active_path], 1);
				mdelay(200);
			}
		}

		if (active_path >= 0)
			atombios_dp_prepare_link(ctx, &paths[active_path], mode.pixel_clock);

		/* Initialize transmitter PHY */
		if (active_path >= 0) {
			printk(BIOS_INFO, "ATOMBIOS: initializing transmitter PHY %d...\n",
			       paths[active_path].phy_id);
			atombios_transmitter_control(ctx, &paths[active_path],
						     pixel_clock_10khz,
						     ATOM_TRANSMITTER_ACTION_INIT,
						     0, 0);
		}

		/* Configure DIG encoder */
		if (active_path >= 0) {
			printk(BIOS_INFO, "ATOMBIOS: configuring DIG encoder %d...\n",
			       paths[active_path].dig_encoder);
			atombios_dig_encoder_setup(ctx, &paths[active_path],
						   pixel_clock_10khz,
						   ATOM_ENCODER_CMD_SETUP,
						   PANEL_8BIT_PER_COLOR);
		}

		/* Route CRTC to encoder */
		if (active_path >= 0)
			atombios_select_crtc_source(ctx, &paths[active_path], crtc_id);

		/* Program pixel clock PLL */
		if (active_path >= 0)
			atombios_set_pixel_clock(ctx, &gpu, &paths[active_path],
						 pixel_clock_10khz, crtc_id);

		/* Program CRTC timing */
		printk(BIOS_INFO, "ATOMBIOS: programming CRTC timing...\n");
		atombios_set_crtc_dtd_timing(ctx, &mode, crtc_id);

		/* Program CRTC scanout surface */
		atombios_program_framebuffer(ctx, &gpu, fb_bar, &mode);
		atombios_program_crtc_postmode(ctx, crtc_id);

		/* Enable CRTC */
		atombios_enable_crtc(ctx, crtc_id, 1);

		/* Enable transmitter */
		if (active_path >= 0) {
			printk(BIOS_INFO, "ATOMBIOS: enabling transmitter...\n");
			atombios_transmitter_control(ctx, &paths[active_path],
						     pixel_clock_10khz,
						     ATOM_TRANSMITTER_ACTION_ENABLE,
						     0, 0);
		}

		/* DP link training */
		if (active_path >= 0 &&
		    paths[active_path].encoder_mode == ATOM_ENCODER_MODE_DP) {
			printk(BIOS_INFO, "ATOMBIOS: starting DP link training...\n");
			if (atombios_dp_link_train(ctx, &paths[active_path],
						   pixel_clock_10khz,
						   paths[active_path].i2c_line,
						   paths[active_path].hpd_id))
				printk(BIOS_WARNING,
				       "ATOMBIOS: DP link training failed\n");
		}

		/* Unblank */
		atombios_blank_crtc(ctx, crtc_id, 0);
		atombios_evergreen_unblank_scanout(ctx, &gpu);
		if (active_path >= 0) {
			atombios_update_scratch_for_path(ctx, &paths[active_path],
						     crtc_id, true, true);
			atombios_output_lock(ctx, false);
		}
	}

	/* Register framebuffer with coreboot */
	fb_add_framebuffer_info(fb_bar,
				mode.ha, mode.va,
				mode.ha * BYTES_PER_PIXEL,
				DEFAULT_BPP);

	printk(BIOS_INFO, "ATOMBIOS: framebuffer %ux%u @ 0x%llx registered\n",
	       mode.ha, mode.va, (unsigned long long)fb_bar);
}

static struct device_operations atombios_gfx_ops = {
	.read_resources   = pci_dev_read_resources,
	.set_resources    = pci_dev_set_resources,
	.enable_resources = pci_dev_enable_resources,
	.init             = atombios_init,
};

/*
 * AMD GPU PCI device ID table.
 *
 * Complete list of AMD GPUs extracted from the Linux kernel amdgpu and
 * radeon drivers. The driver auto-detects whether the VBIOS uses v1
 * (atombios.h) or v2 (atomfirmware.h) table format at runtime.
 *
 * To add a new GPU, find its PCI device ID with "lspci -nn" and add
 * it to the appropriate section below.
 */
static const unsigned short atombios_device_ids[] = {
	/* --- R600/R700 (HD 2000-4000, radeon, atombios.h v1) --- */
	/* R600 (HD 2900) */
	0x9400, 0x9401, 0x9402, 0x9403, 0x9405, 0x940A, 0x940B, 0x940F,
	/* RV610 (HD 2400) */
	0x94C0, 0x94C1, 0x94C3, 0x94C4, 0x94C5, 0x94C6, 0x94C7, 0x94C8,
	0x94C9, 0x94CB, 0x94CC, 0x94CD,
	/* RV630 (HD 2600) */
	0x9580, 0x9581, 0x9583, 0x9586, 0x9587, 0x9588, 0x9589, 0x958A,
	0x958B, 0x958C, 0x958D, 0x958E, 0x958F,
	/* RV620 (HD 3450) */
	0x95C0, 0x95C2, 0x95C4, 0x95C5, 0x95C6, 0x95C7, 0x95C9, 0x95CC,
	0x95CD, 0x95CE, 0x95CF,
	/* RV635 (HD 3650) */
	0x9590, 0x9591, 0x9593, 0x9595, 0x9596, 0x9597, 0x9598, 0x9599, 0x959B,
	/* RV670 (HD 3870) */
	0x9500, 0x9501, 0x9504, 0x9505, 0x9506, 0x9507, 0x9508, 0x9509,
	0x950F, 0x9511, 0x9515, 0x9517, 0x9519,
	/* RS780/RS880 (HD 3200/4200 IGP) */
	0x9610, 0x9611, 0x9612, 0x9613, 0x9614, 0x9615, 0x9616,
	0x9710, 0x9711, 0x9712, 0x9713, 0x9714, 0x9715,
	/* RV770 (HD 4870) */
	0x9440, 0x9441, 0x9442, 0x9443, 0x9444, 0x9446, 0x944A, 0x944B,
	0x944C, 0x944E, 0x9450, 0x9452, 0x9456, 0x945A, 0x945B, 0x945E,
	0x9460, 0x9462, 0x946A, 0x946B, 0x947A, 0x947B,
	/* RV730 (HD 4670) */
	0x9480, 0x9487, 0x9488, 0x9489, 0x948A, 0x948F, 0x9490, 0x9491,
	0x9495, 0x9498, 0x949C, 0x949E, 0x949F,
	/* RV710 (HD 4350) */
	0x9540, 0x9541, 0x9542, 0x954E, 0x954F, 0x9552, 0x9553, 0x9555,
	0x9557, 0x955F,
	/* RV740 (HD 4770) */
	0x94A0, 0x94A1, 0x94A3, 0x94B1, 0x94B3, 0x94B4, 0x94B5, 0x94B9,

	/* --- Evergreen (HD 5000, radeon, atombios.h v1) --- */
	/* Cypress (HD 5870) */
	0x6880, 0x6888, 0x6889, 0x688A, 0x688C, 0x688D, 0x6898, 0x6899,
	0x689B, 0x689C, 0x689D, 0x689E,
	/* Juniper (HD 5770) */
	0x68A0, 0x68A1, 0x68A8, 0x68A9, 0x68B0, 0x68B8, 0x68B9, 0x68BA,
	0x68BE, 0x68BF,
	/* Redwood (HD 5670) */
	0x68C0, 0x68C1, 0x68C7, 0x68C8, 0x68C9, 0x68D8, 0x68D9, 0x68DA, 0x68DE,
	/* Cedar (HD 5450) */
	0x68E0, 0x68E1, 0x68E4, 0x68E5, 0x68E8, 0x68E9, 0x68F1, 0x68F2,
	0x68F8, 0x68F9, 0x68FA, 0x68FE,
	/* Palm/Sumo IGP (HD 6310/6xxx) */
	0x9802, 0x9803, 0x9804, 0x9805, 0x9806, 0x9807, 0x9808, 0x9809, 0x980A,
	0x9640, 0x9641, 0x9642, 0x9643, 0x9644, 0x9645, 0x9647, 0x9648,
	0x9649, 0x964A, 0x964B, 0x964C, 0x964E, 0x964F,

	/* --- Northern Islands (HD 6000, radeon, atombios.h v1) --- */
	/* Cayman (HD 6970) */
	0x6700, 0x6701, 0x6702, 0x6703, 0x6704, 0x6705, 0x6706, 0x6707,
	0x6708, 0x6709, 0x6718, 0x6719, 0x671C, 0x671D, 0x671F,
	/* Barts (HD 6870) */
	0x6720, 0x6721, 0x6722, 0x6723, 0x6724, 0x6725, 0x6726, 0x6727,
	0x6728, 0x6729, 0x6738, 0x6739, 0x673E,
	/* Turks (HD 6670) */
	0x6740, 0x6741, 0x6742, 0x6743, 0x6744, 0x6745, 0x6746, 0x6747,
	0x6748, 0x6749, 0x674A, 0x6750, 0x6751, 0x6758, 0x6759, 0x675B,
	0x675D, 0x675F, 0x6840, 0x6841, 0x6842, 0x6843, 0x6849, 0x6850,
	0x6858, 0x6859,
	/* Caicos (HD 6450) */
	0x6760, 0x6761, 0x6762, 0x6763, 0x6764, 0x6765, 0x6766, 0x6767,
	0x6768, 0x6770, 0x6771, 0x6772, 0x6778, 0x6779, 0x677B,
	/* Aruba (Trinity/Richland APU) */
	0x9900, 0x9901, 0x9903, 0x9904, 0x9905, 0x9906, 0x9907, 0x9908,
	0x9909, 0x990A, 0x990B, 0x990C, 0x990D, 0x990E, 0x990F, 0x9910,
	0x9913, 0x9917, 0x9918, 0x9919, 0x9990, 0x9991, 0x9992, 0x9993,
	0x9994, 0x9995, 0x9996, 0x9997, 0x9998, 0x9999, 0x999A, 0x999B,
	0x999C, 0x999D, 0x99A0, 0x99A2, 0x99A4,

	/* --- GCN 1.0 / Southern Islands (SI, amdgpu, atombios.h v1) --- */
	/* Tahiti (HD 7970) */
	0x6780, 0x6784, 0x6788, 0x678A, 0x6790, 0x6791, 0x6792,
	0x6798, 0x6799, 0x679A, 0x679B, 0x679E, 0x679F,
	/* Pitcairn (HD 7870) */
	0x6800, 0x6801, 0x6802, 0x6806, 0x6808, 0x6809,
	0x6810, 0x6811, 0x6816, 0x6817, 0x6818, 0x6819,
	/* Verde (HD 7750) */
	0x6820, 0x6821, 0x6822, 0x6823, 0x6824, 0x6825, 0x6826, 0x6827,
	0x6828, 0x6829, 0x682A, 0x682B, 0x682C, 0x682D, 0x682F,
	0x6830, 0x6831, 0x6835, 0x6837, 0x6838, 0x6839, 0x683B, 0x683D, 0x683F,
	/* Oland (HD 8570) */
	0x6600, 0x6601, 0x6602, 0x6603, 0x6604, 0x6605, 0x6606, 0x6607,
	0x6608, 0x6610, 0x6611, 0x6613, 0x6617, 0x6620, 0x6621, 0x6623, 0x6631,
	/* Hainan (HD 8470) */
	0x6660, 0x6663, 0x6664, 0x6665, 0x6667, 0x666F,

	/* --- GCN 2.0 / Sea Islands (CIK, amdgpu, atombios.h v1) --- */
	/* Bonaire (R7 260X) */
	0x6640, 0x6641, 0x6646, 0x6647, 0x6649,
	0x6650, 0x6651, 0x6658, 0x665C, 0x665D, 0x665F,
	/* Hawaii (R9 290X) */
	0x67A0, 0x67A1, 0x67A2, 0x67A8, 0x67A9, 0x67AA,
	0x67B0, 0x67B1, 0x67B8, 0x67B9, 0x67BA, 0x67BE,
	/* Kaveri/Kabini/Mullins APU */
	0x1304, 0x1305, 0x1306, 0x1307, 0x1309, 0x130A, 0x130B,
	0x130C, 0x130D, 0x130E, 0x130F, 0x1310, 0x1311, 0x1312,
	0x1313, 0x1315, 0x1316, 0x1317, 0x1318, 0x131B, 0x131C, 0x131D,
	0x9830, 0x9831, 0x9832, 0x9833, 0x9834, 0x9835, 0x9836, 0x9837,
	0x9838, 0x9839, 0x983A, 0x983B, 0x983C, 0x983D, 0x983E, 0x983F,
	0x9850, 0x9851, 0x9852, 0x9853, 0x9854, 0x9855, 0x9856, 0x9857,
	0x9858, 0x9859, 0x985A, 0x985B, 0x985C, 0x985D, 0x985E, 0x985F,

	/* --- GCN 3.0 / Volcanic Islands (VI, amdgpu, atombios.h v1) --- */
	/* Topaz/Tonga/Fiji */
	0x6900, 0x6901, 0x6902, 0x6903, 0x6907,
	0x6920, 0x6921, 0x6928, 0x6929, 0x692B, 0x692F,
	0x6930, 0x6938, 0x6939,
	0x7300, 0x730F,
	/* Carrizo/Stoney APU */
	0x9870, 0x9874, 0x9875, 0x9876, 0x9877, 0x98E4,

	/* --- GCN 4.0 / Polaris (amdgpu, atombios.h v1) --- */
	/* Polaris10 (RX 470/480/570/580) */
	0x67C0, 0x67C1, 0x67C2, 0x67C4, 0x67C7, 0x67C8, 0x67C9,
	0x67CA, 0x67CC, 0x67CF, 0x67D0, 0x67DF, 0x6FDF,
	/* Polaris11 (RX 460/560) */
	0x67E0, 0x67E1, 0x67E3, 0x67E7, 0x67E8, 0x67E9,
	0x67EB, 0x67EF, 0x67FF,
	/* Polaris12 (RX 540/550) */
	0x6980, 0x6981, 0x6985, 0x6986, 0x6987, 0x6995, 0x6997, 0x699F,
	/* VegaM (Kaby Lake-G) */
	0x694C, 0x694E, 0x694F,

	/* --- GCN 5.0 / Vega (amdgpu, atomfirmware.h v2) --- */
	/* Vega 10 (RX Vega 56/64) */
	0x6860, 0x6861, 0x6862, 0x6863, 0x6864, 0x6867, 0x6868, 0x6869,
	0x686A, 0x686B, 0x686C, 0x686D, 0x686E, 0x686F, 0x687F,
	/* Vega 12 */
	0x69A0, 0x69A1, 0x69A2, 0x69A3, 0x69AF,
	/* Vega 20 (Radeon VII) */
	0x66A0, 0x66A1, 0x66A2, 0x66A3, 0x66A4, 0x66A7, 0x66AF,
	/* Raven (Ryzen APU) */
	0x15DD, 0x15D8,

	/* --- RDNA 1.0 / Navi (amdgpu, atomfirmware.h v2) --- */
	/* Navi10 (RX 5700) */
	0x7310, 0x7312, 0x7318, 0x7319, 0x731A, 0x731B, 0x731E, 0x731F,
	/* Navi14 (RX 5500) */
	0x7340, 0x7341, 0x7347, 0x734F,
	/* Navi12 (Pro 5600M) */
	0x7360, 0x7362,
	/* Renoir APU */
	0x15E7, 0x1636, 0x1638, 0x164C,

	/* --- RDNA 2.0 / Navi 2x (amdgpu, atomfirmware.h v2) --- */
	/* Sienna Cichlid (RX 6800/6900) */
	0x73A0, 0x73A1, 0x73A2, 0x73A3, 0x73A5, 0x73A8, 0x73A9,
	0x73AB, 0x73AC, 0x73AD, 0x73AE, 0x73AF, 0x73BF,
	/* Navy Flounder (RX 6700) */
	0x73C0, 0x73C1, 0x73C3, 0x73DA, 0x73DB, 0x73DC, 0x73DD, 0x73DE, 0x73DF,
	/* Dimgrey Cavefish (RX 6600) */
	0x73E0, 0x73E1, 0x73E2, 0x73E3, 0x73E8, 0x73E9,
	0x73EA, 0x73EB, 0x73EC, 0x73ED, 0x73EF, 0x73FF,
	/* Beige Goby (RX 6400) */
	0x7420, 0x7421, 0x7422, 0x7423, 0x7424, 0x743F,
	/* Yellow Carp (Rembrandt APU) */
	0x164D, 0x1681,

	/* --- RDNA 3.0 / Navi 3x (amdgpu, atomfirmware.h v2) --- */
	/* Navi 31 (RX 7900 XTX/XT) */
	0x744C, 0x7448,
	/* Navi 32 (RX 7800/7700 XT) */
	0x747E,
	/* Navi 33 (RX 7600) */
	0x7480,

	0,  /* terminator */
};

static const struct pci_driver atombios_driver __pci_driver = {
	.ops     = &atombios_gfx_ops,
	.vendor  = PCI_VID_ATI,
	.devices = atombios_device_ids,
};
