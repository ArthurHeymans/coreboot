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
#include <edid.h>
#include <framebuffer_info.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "atom.h"
#include "atom-bits.h"

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

/* ATOM command table indices */
#define CMD_IDX(field) \
	(offsetof(ATOM_MASTER_LIST_OF_COMMAND_TABLES, field) / sizeof(USHORT))

/* ATOM data table indices */
#define DATA_IDX(field) \
	(offsetof(ATOM_MASTER_LIST_OF_DATA_TABLES, field) / sizeof(USHORT))

#define ATOM_CMD_ASIC_INIT		CMD_IDX(ASIC_Init)
#define ATOM_CMD_SET_CRTC_USING_DTD	CMD_IDX(SetCRTC_UsingDTDTiming)
#define ATOM_CMD_ENABLE_CRTC		CMD_IDX(EnableCRTC)
#define ATOM_CMD_BLANK_CRTC		CMD_IDX(BlankCRTC)
#define ATOM_CMD_SELECT_CRTC_SOURCE	CMD_IDX(SelectCRTC_Source)
#define ATOM_CMD_ENABLE_DISP_POWER_GATING CMD_IDX(EnableDispPowerGating)
#define ATOM_CMD_DIG_ENCODER_CTRL	CMD_IDX(DIGxEncoderControl)
#define ATOM_CMD_DIG1_XMTR_CTRL	CMD_IDX(DIG1TransmitterControl)
#define ATOM_CMD_PROCESS_I2C		CMD_IDX(ProcessI2cChannelTransaction)

/* Maximum EDID block size */
#define EDID_BLOCK_SIZE		128

/* I2C limits for ProcessI2cChannelTransaction */
#define ATOM_MAX_HW_I2C_WRITE	3
#define ATOM_MAX_HW_I2C_READ	255

/* DDC slave addresses */
#define DDC_EDID_ADDRESS	0x50

/* ---- Display path info discovered from ATOM tables ---- */

struct atombios_display_path {
	uint8_t  connector_type;	/* DRM_MODE_CONNECTOR_* style */
	uint8_t  encoder_obj_id;	/* ENCODER_OBJECT_ID_* */
	uint8_t  encoder_type;		/* GRAPH_OBJECT_ENUM_ID */
	uint8_t  i2c_line;		/* I2C line number for DDC */
	uint8_t  hpd_id;		/* HPD pin ID (1-6, 0=none) */
	uint8_t  dig_encoder;		/* DIG index (0=DIG1, ...) */
	uint8_t  phy_id;		/* UNIPHY ID (0=A, 1=B, ...) */
	uint8_t  encoder_mode;		/* ATOM_ENCODER_MODE_* */
	uint16_t connector_obj_id;	/* Full connector object ID */
};

/* ---- card_info MMIO callbacks ---- */

static void cail_reg_write(struct card_info *info, uint32_t reg, uint32_t val)
{
	write32((uint8_t *)info->mmio_base + reg, val);
}

static uint32_t cail_reg_read(struct card_info *info, uint32_t reg)
{
	return read32((uint8_t *)info->mmio_base + reg);
}

/* MC and PLL callbacks are stubs on modern GPUs -- all access goes via MMIO */
static void cail_mc_write(struct card_info *info, uint32_t reg, uint32_t val) { }
static uint32_t cail_mc_read(struct card_info *info, uint32_t reg) { return 0; }
static void cail_pll_write(struct card_info *info, uint32_t reg, uint32_t val) { }
static uint32_t cail_pll_read(struct card_info *info, uint32_t reg) { return 0; }

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

/* ---- EDID reading via ATOM I2C ---- */

/*
 * Read a single I2C transaction via the ProcessI2cChannelTransaction
 * ATOM command table. This is used for DDC/EDID reading.
 *
 * Returns 0 on success, -1 on failure.
 */
static int atombios_i2c_read(struct atom_context *ctx, uint8_t i2c_line,
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

	ret = atom_execute_table(ctx, ATOM_CMD_PROCESS_I2C, (uint32_t *)&args,
				 sizeof(args) / sizeof(uint32_t));
	if (ret) {
		printk(BIOS_DEBUG, "ATOMBIOS: I2C read failed (line=%d addr=0x%02x)\n",
		       i2c_line, slave_addr);
		return -1;
	}

	/* On success, the ATOM table writes data to the scratch area.
	 * The first bytes of the parameter space contain the result. */
	memcpy(buf, &args, len < sizeof(args) ? len : sizeof(args));
	return 0;
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

	printk(BIOS_DEBUG, "ATOMBIOS: reading EDID on I2C line %d\n", i2c_line);

	/*
	 * Read EDID in chunks. The ProcessI2cChannelTransaction table
	 * can read up to 255 bytes, but some implementations are limited.
	 * Use 16-byte chunks for maximum compatibility.
	 */
	for (offset = 0; offset < EDID_BLOCK_SIZE; offset += chunk) {
		chunk = EDID_BLOCK_SIZE - offset;
		if (chunk > 16)
			chunk = 16;

		ret = atombios_i2c_read(ctx, i2c_line, DDC_EDID_ADDRESS,
					(uint8_t)offset,
					edid_buf + offset, (uint8_t)chunk);
		if (ret)
			return -1;
	}

	/* Validate EDID header: 00 FF FF FF FF FF FF 00 */
	if (edid_buf[0] != 0x00 || edid_buf[1] != 0xFF ||
	    edid_buf[6] != 0xFF || edid_buf[7] != 0x00) {
		printk(BIOS_DEBUG, "ATOMBIOS: EDID header invalid on I2C line %d\n",
		       i2c_line);
		return -1;
	}

	printk(BIOS_INFO, "ATOMBIOS: EDID read successfully on I2C line %d\n",
	       i2c_line);
	return 0;
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

	if (!atom_parse_data_header(ctx, DATA_IDX(Object_Header),
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
		obj_type = (conn_obj_id >> 12) & 0xF;

		if (obj_type != GRAPH_OBJECT_TYPE_CONNECTOR) {
			/* Skip to next path using usSize */
			path_offset += disp_path->usSize;
			continue;
		}

		obj_id = (conn_obj_id >> 8) & 0xF;

		paths[count].connector_obj_id = conn_obj_id;

		/* Map ATOM connector object ID to a simple type */
		switch (obj_id) {
		case CONNECTOR_OBJECT_ID_HDMI_TYPE_A:
			paths[count].connector_type = 1; /* HDMI */
			paths[count].encoder_mode = ATOM_ENCODER_MODE_HDMI;
			break;
		case CONNECTOR_OBJECT_ID_DISPLAYPORT:
		case CONNECTOR_OBJECT_ID_eDP:
			paths[count].connector_type = 2; /* DP/eDP */
			paths[count].encoder_mode = ATOM_ENCODER_MODE_DP;
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
		for (j = 0; j < (disp_path->usSize - 4) / 2; j++) {
			enc_obj_id = disp_path->usGraphicObjIds[j];
			obj_type = (enc_obj_id >> 12) & 0xF;

			if (obj_type != GRAPH_OBJECT_TYPE_ENCODER)
				continue;

			enc_id = (enc_obj_id >> 8) & 0xF;
			paths[count].encoder_obj_id = enc_id;

			/* Determine DIG encoder and PHY from encoder object */
			switch (enc_id) {
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
			default:
				paths[count].dig_encoder = 0;
				paths[count].phy_id = 0;
				break;
			}

			/* Check link B (ENUM_ID_2) */
			if (((enc_obj_id >> 4) & 0xF) == 2) {
				paths[count].dig_encoder += 1;
				paths[count].phy_id += 1;
			}
			break; /* use first encoder found */
		}

		printk(BIOS_INFO,
		       "ATOMBIOS: path %d: conn=0x%04x type=%d enc_id=%d dig=%d phy=%d mode=%d\n",
		       count, conn_obj_id, paths[count].connector_type,
		       paths[count].encoder_obj_id,
		       paths[count].dig_encoder, paths[count].phy_id,
		       paths[count].encoder_mode);

		count++;
		path_offset += disp_path->usSize;
	}

	/* Look up I2C lines from GPIO_I2C_Info table */
	if (atom_parse_data_header(ctx, DATA_IDX(GPIO_I2C_Info),
				   NULL, &frev, &crev, &data_offset)) {
		ATOM_GPIO_I2C_INFO *i2c_info;
		i2c_info = (ATOM_GPIO_I2C_INFO *)(base + data_offset);

		/*
		 * Assign I2C line IDs to paths. The typical mapping is
		 * path index -> I2C line index, but this is a heuristic.
		 * The proper approach would be to parse connector records
		 * from the object table, but for initial bring-up we use
		 * the I2C entries in order.
		 */
		for (i = 0; i < count; i++) {
			if (i < ATOM_MAX_SUPPORTED_DEVICE)
				paths[i].i2c_line =
					i2c_info->asGPIO_Info[i].sucI2cId.ucAccess & 0xF;
		}
	}

	return count;
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
	union {
		DIG_ENCODER_CONTROL_PARAMETERS      v1;
		DIG_ENCODER_CONTROL_PARAMETERS_V2   v2;
		DIG_ENCODER_CONTROL_PARAMETERS_V3   v3;
		DIG_ENCODER_CONTROL_PARAMETERS_V4   v4;
	} args;

	if (!atom_parse_cmd_header(ctx, ATOM_CMD_DIG_ENCODER_CTRL, &frev, &crev))
		return -1;

	memset(&args, 0, sizeof(args));

	printk(BIOS_DEBUG,
	       "ATOMBIOS: DIG encoder setup frev=%d crev=%d action=0x%02x mode=%d\n",
	       frev, crev, action, path->encoder_mode);

	switch (crev) {
	case 1:
		args.v1.ucAction = action;
		args.v1.usPixelClock = pixel_clock_10khz;
		args.v1.ucEncoderMode = path->encoder_mode;
		args.v1.ucLaneNum = 4;
		break;
	case 2:
		args.v2.ucAction = action;
		args.v2.usPixelClock = pixel_clock_10khz;
		args.v2.ucEncoderMode = path->encoder_mode;
		args.v2.ucLaneNum = 4;
		args.v2.acConfig.ucLinkSel = path->dig_encoder & 1;
		args.v2.acConfig.ucTransmitterSel = path->phy_id / 2;
		break;
	case 3:
		args.v3.ucAction = action;
		args.v3.usPixelClock = pixel_clock_10khz;
		args.v3.ucEncoderMode = path->encoder_mode;
		args.v3.ucLaneNum = 4;
		args.v3.ucBitPerColor = bpc;
		args.v3.acConfig.ucDigSel = path->dig_encoder;
		break;
	case 4:
	default:
		args.v4.ucAction = action;
		args.v4.usPixelClock = pixel_clock_10khz;
		args.v4.ucEncoderMode = path->encoder_mode;
		args.v4.ucLaneNum = 4;
		args.v4.ucBitPerColor = bpc;
		args.v4.ucHPD_ID = path->hpd_id;
		args.v4.acConfig.ucDigSel = path->dig_encoder;
		break;
	}

	return atom_execute_table(ctx, ATOM_CMD_DIG_ENCODER_CTRL,
				  (uint32_t *)&args,
				  sizeof(args) / sizeof(uint32_t));
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
					uint8_t action)
{
	uint8_t frev, crev;
	union {
		DIG_TRANSMITTER_CONTROL_PARAMETERS       v1;
		DIG_TRANSMITTER_CONTROL_PARAMETERS_V2    v2;
		DIG_TRANSMITTER_CONTROL_PARAMETERS_V3    v3;
		DIG_TRANSMITTER_CONTROL_PARAMETERS_V4    v4;
		DIG_TRANSMITTER_CONTROL_PARAMETERS_V1_5  v5;
		DIG_TRANSMITTER_CONTROL_PARAMETERS_V1_6  v6;
	} args;

	if (!atom_parse_cmd_header(ctx, ATOM_CMD_DIG1_XMTR_CTRL, &frev, &crev))
		return -1;

	memset(&args, 0, sizeof(args));

	printk(BIOS_DEBUG,
	       "ATOMBIOS: transmitter ctrl frev=%d crev=%d action=%d phy=%d\n",
	       frev, crev, action, path->phy_id);

	switch (crev) {
	case 1:
		args.v1.ucAction = action;
		if (action == ATOM_TRANSMITTER_ACTION_INIT)
			args.v1.usInitInfo = path->connector_obj_id;
		else
			args.v1.usPixelClock = pixel_clock_10khz;
		/* Set coherent mode for HDMI/DP */
		if (path->encoder_mode == ATOM_ENCODER_MODE_HDMI ||
		    path->encoder_mode == ATOM_ENCODER_MODE_DP)
			args.v1.ucConfig |= (1 << 1); /* COHERENT */
		break;
	case 2:
		args.v2.ucAction = action;
		if (action == ATOM_TRANSMITTER_ACTION_INIT)
			args.v2.usInitInfo = path->connector_obj_id;
		else
			args.v2.usPixelClock = pixel_clock_10khz;
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
		if (action == ATOM_TRANSMITTER_ACTION_INIT)
			args.v3.usInitInfo = path->connector_obj_id;
		else
			args.v3.usPixelClock = pixel_clock_10khz;
		args.v3.ucLaneNum = 4;
		args.v3.acConfig.ucEncoderSel = (path->dig_encoder & 1);
		args.v3.acConfig.ucTransmitterSel = path->phy_id / 2;
		if (path->encoder_mode == ATOM_ENCODER_MODE_HDMI ||
		    path->encoder_mode == ATOM_ENCODER_MODE_DP)
			args.v3.acConfig.fCoherentMode = 1;
		break;
	case 4:
		args.v4.ucAction = action;
		if (action == ATOM_TRANSMITTER_ACTION_INIT)
			args.v4.usInitInfo = path->connector_obj_id;
		else
			args.v4.usPixelClock = pixel_clock_10khz;
		args.v4.ucLaneNum = 4;
		args.v4.acConfig.ucEncoderSel = (path->dig_encoder & 1);
		args.v4.acConfig.ucTransmitterSel = path->phy_id / 2;
		if (path->encoder_mode == ATOM_ENCODER_MODE_HDMI ||
		    path->encoder_mode == ATOM_ENCODER_MODE_DP)
			args.v4.acConfig.fCoherentMode = 1;
		break;
	case 5:
		args.v5.ucAction = action;
		if (action == ATOM_TRANSMITTER_ACTION_INIT)
			args.v5.usSymClock = path->connector_obj_id;
		else
			args.v5.usSymClock = pixel_clock_10khz;
		args.v5.ucPhyId = path->phy_id;
		args.v5.ucLaneNum = 4;
		args.v5.ucConnObjId = (path->connector_obj_id >> 8) & 0xFF;
		args.v5.ucDigMode = path->encoder_mode;
		args.v5.ucDigEncoderSel = (1 << path->dig_encoder);
		if (path->encoder_mode == ATOM_ENCODER_MODE_HDMI ||
		    path->encoder_mode == ATOM_ENCODER_MODE_DP)
			args.v5.asConfig.ucCoherentMode = 1;
		break;
	case 6:
	default:
		args.v6.ucAction = action;
		args.v6.ucPhyId = path->phy_id;
		args.v6.ucDigMode = path->encoder_mode;
		args.v6.ucLaneNum = 4;
		args.v6.ucConnObjId = (path->connector_obj_id >> 8) & 0xFF;
		args.v6.ucDigEncoderSel = (1 << path->dig_encoder);
		if (action == ATOM_TRANSMITTER_ACTION_INIT)
			args.v6.ulSymClock = path->connector_obj_id;
		else
			args.v6.ulSymClock = pixel_clock_10khz;
		break;
	}

	return atom_execute_table(ctx, ATOM_CMD_DIG1_XMTR_CTRL,
				  (uint32_t *)&args,
				  sizeof(args) / sizeof(uint32_t));
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
	} args;
	uint8_t enc_id;

	if (!atom_parse_cmd_header(ctx, ATOM_CMD_SELECT_CRTC_SOURCE,
				   &frev, &crev))
		return -1;

	memset(&args, 0, sizeof(args));

	/* Map dig_encoder index to ASIC_INT_DIG*_ENCODER_ID */
	switch (path->dig_encoder) {
	case 0: enc_id = ASIC_INT_DIG1_ENCODER_ID; break;
	case 1: enc_id = ASIC_INT_DIG2_ENCODER_ID; break;
	case 2: enc_id = ASIC_INT_DIG3_ENCODER_ID; break;
	case 3: enc_id = ASIC_INT_DIG4_ENCODER_ID; break;
	case 4: enc_id = ASIC_INT_DIG5_ENCODER_ID; break;
	case 5: enc_id = ASIC_INT_DIG6_ENCODER_ID; break;
	default: enc_id = ASIC_INT_DIG7_ENCODER_ID; break;
	}

	switch (crev) {
	case 1:
		args.v1.ucCRTC = crtc_id;
		args.v1.ucDevice = ATOM_DEVICE_DFP1_INDEX;
		break;
	case 2:
	default:
		args.v2.ucCRTC = crtc_id;
		args.v2.ucEncoderID = enc_id;
		args.v2.ucEncodeMode = path->encoder_mode;
		break;
	}

	printk(BIOS_DEBUG,
	       "ATOMBIOS: select CRTC %d -> encoder %d (enc_id=0x%02x mode=%d)\n",
	       crtc_id, path->dig_encoder, enc_id, path->encoder_mode);

	return atom_execute_table(ctx, ATOM_CMD_SELECT_CRTC_SOURCE,
				  (uint32_t *)&args,
				  sizeof(args) / sizeof(uint32_t));
}

/* ---- Basic CRTC operations ---- */

static int atombios_set_crtc_dtd_timing(struct atom_context *ctx,
					 const struct edid_mode *mode,
					 int crtc_id)
{
	uint32_t params[8];
	uint8_t misc_flags = 0;

	memset(params, 0, sizeof(params));

	if (mode->phsync)
		misc_flags |= 0x02;
	if (mode->pvsync)
		misc_flags |= 0x04;

	params[0] = mode->ha | ((uint32_t)mode->hbl << 16);
	params[1] = mode->va | ((uint32_t)mode->vbl << 16);
	params[2] = mode->hso | ((uint32_t)mode->hspw << 16);
	params[3] = mode->vso | ((uint32_t)mode->vspw << 16);
	params[4] = misc_flags | ((uint32_t)crtc_id << 24);

	return atombios_exec(ctx, ATOM_CMD_SET_CRTC_USING_DTD, params,
			     sizeof(params) / sizeof(uint32_t));
}

static int atombios_enable_crtc(struct atom_context *ctx, int crtc_id,
				int enable)
{
	uint32_t params[1];
	params[0] = crtc_id | ((enable ? ATOM_ENABLE : ATOM_DISABLE) << 8);
	return atombios_exec(ctx, ATOM_CMD_ENABLE_CRTC, params, 1);
}

static int atombios_blank_crtc(struct atom_context *ctx, int crtc_id, int blank)
{
	uint32_t params[1];
	params[0] = crtc_id | ((blank ? ATOM_BLANKING : ATOM_BLANKING_OFF) << 8);
	return atombios_exec(ctx, ATOM_CMD_BLANK_CRTC, params, 1);
}

static int atombios_disp_power_gating(struct atom_context *ctx, int crtc_id,
				      int gate)
{
	uint32_t params[1];
	params[0] = crtc_id | ((gate ? ATOM_DISABLE : ATOM_ENABLE) << 8);
	return atombios_exec(ctx, ATOM_CMD_ENABLE_DISP_POWER_GATING, params, 1);
}

/* ---- Main init entry point ---- */

static void atombios_init(struct device *dev)
{
	struct card_info *card;
	struct atom_context *ctx;
	void *vbios;
	uint32_t mmio_bar, fb_bar;
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
	int i;

	printk(BIOS_INFO, "ATOMBIOS: initializing AMD GPU %04x:%04x\n",
	       dev->vendor, dev->device);

	/* Map MMIO BAR */
	mmio_bar = pci_read_config32(dev, AMD_GPU_MMIO_BAR);
	mmio_bar &= ~0xFu;
	if (!mmio_bar) {
		mmio_bar = pci_read_config32(dev, PCI_BASE_ADDRESS_2);
		mmio_bar &= ~0xFu;
	}
	if (!mmio_bar) {
		printk(BIOS_ERR, "ATOMBIOS: could not find MMIO BAR\n");
		return;
	}
	mmio_base = (void *)(uintptr_t)mmio_bar;
	printk(BIOS_INFO, "ATOMBIOS: MMIO base at %p\n", mmio_base);

	/* Get framebuffer BAR */
	fb_bar = pci_read_config32(dev, AMD_GPU_FB_BAR);
	fb_bar &= ~0xFu;
	if (!fb_bar) {
		printk(BIOS_ERR, "ATOMBIOS: could not find FB BAR\n");
		return;
	}
	printk(BIOS_INFO, "ATOMBIOS: framebuffer aperture at 0x%08x\n", fb_bar);

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

	/* Discover display paths from ATOM object tables */
	memset(paths, 0, sizeof(paths));
	num_paths = atombios_discover_display_paths(ctx, paths, 8);
	if (!num_paths) {
		printk(BIOS_WARNING,
		       "ATOMBIOS: no display paths found, using defaults\n");
		goto use_defaults;
	}

	/*
	 * Try to read EDID on each path to find a connected display.
	 * Prefer HDMI/DVI over DP (DP needs link training which is more
	 * complex). Fall back to defaults if no EDID is found.
	 */
	for (i = 0; i < num_paths; i++) {
		if (paths[i].connector_type == 0)
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

	/* If we have at least one path, use it even without EDID */
	if (num_paths > 0 && active_path < 0)
		active_path = 0;

do_modeset:
	printk(BIOS_INFO, "ATOMBIOS: modesetting %ux%u @ %u kHz\n",
	       mode.ha, mode.va, mode.pixel_clock);

	/* Step 1: Power up display pipe */
	atombios_disp_power_gating(ctx, crtc_id, 0);

	/*
	 * Step 2: Initialize transmitter PHY.
	 * The INIT action tells the PHY which connector it serves.
	 */
	if (active_path >= 0) {
		printk(BIOS_INFO, "ATOMBIOS: initializing transmitter PHY %d...\n",
		       paths[active_path].phy_id);
		atombios_transmitter_control(ctx, &paths[active_path],
					     pixel_clock_10khz,
					     ATOM_TRANSMITTER_ACTION_INIT);
	}

	/*
	 * Step 3: Configure DIG encoder for the target mode.
	 */
	if (active_path >= 0) {
		printk(BIOS_INFO, "ATOMBIOS: configuring DIG encoder %d...\n",
		       paths[active_path].dig_encoder);
		atombios_dig_encoder_setup(ctx, &paths[active_path],
					   pixel_clock_10khz,
					   ATOM_ENCODER_CMD_SETUP,
					   PANEL_8BIT_PER_COLOR);
	}

	/* Step 4: Route CRTC to encoder */
	if (active_path >= 0) {
		printk(BIOS_INFO, "ATOMBIOS: routing CRTC %d -> DIG %d...\n",
		       crtc_id, paths[active_path].dig_encoder);
		atombios_select_crtc_source(ctx, &paths[active_path], crtc_id);
	}

	/* Step 5: Program CRTC timing */
	printk(BIOS_INFO, "ATOMBIOS: programming CRTC timing...\n");
	atombios_set_crtc_dtd_timing(ctx, &mode, crtc_id);

	/* Step 6: Enable CRTC */
	atombios_enable_crtc(ctx, crtc_id, 1);

	/* Step 7: Enable transmitter (drive the physical link) */
	if (active_path >= 0) {
		printk(BIOS_INFO, "ATOMBIOS: enabling transmitter...\n");
		atombios_transmitter_control(ctx, &paths[active_path],
					     pixel_clock_10khz,
					     ATOM_TRANSMITTER_ACTION_ENABLE);
	}

	/* Step 8: Unblank */
	atombios_blank_crtc(ctx, crtc_id, 0);

	/* Step 9: For DP, turn on video after link training would go here.
	 * DP link training requires AUX channel communication (DPCD reads/
	 * writes) and is significantly more complex. For now we support
	 * HDMI and DVI which need no link training. */
	if (active_path >= 0 &&
	    paths[active_path].encoder_mode == ATOM_ENCODER_MODE_DP) {
		printk(BIOS_INFO,
		       "ATOMBIOS: DP link training not yet implemented, "
		       "display may not be active\n");
	}

	/* Register framebuffer with coreboot */
	fb_add_framebuffer_info(fb_bar,
				mode.ha, mode.va,
				mode.ha * BYTES_PER_PIXEL,
				DEFAULT_BPP);

	printk(BIOS_INFO, "ATOMBIOS: framebuffer %ux%u @ 0x%08x registered\n",
	       mode.ha, mode.va, fb_bar);
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
 * The ATOM interpreter handles hardware differences between GPU
 * generations -- the VBIOS bytecode carries GPU-specific knowledge.
 * To add a new GPU, add its PCI device ID (from "lspci -nn").
 */
static const unsigned short atombios_device_ids[] = {
	/* Polaris 10/20 (RX 470/480/570/580) */
	0x67df,
	/* Polaris 11 (RX 460) */
	0x67ef,
	/* Polaris 12 (RX 540/550) */
	0x699f,
	/* Vega 10 (Vega 56/64) */
	0x687f,
	/* Vega 20 (Radeon VII) */
	0x66af,
	/* Navi 10 (RX 5600/5700 XT) */
	0x731f, 0x7340,
	/* Navi 14 (RX 5500 XT) */
	0x7341, 0x7347,
	/* Navi 21 (RX 6800/6900 XT) */
	0x73bf, 0x73a5,
	/* Navi 22 (RX 6700 XT) */
	0x73df,
	/* Navi 23 (RX 6600 XT) */
	0x73ff,
	/* Navi 31 (RX 7900 XTX/XT) */
	0x744c, 0x7448,
	/* Navi 32 (RX 7800/7700 XT) */
	0x747e,
	/* Navi 33 (RX 7600) */
	0x7480,
	0,  /* terminator */
};

static const struct pci_driver atombios_driver __pci_driver = {
	.ops     = &atombios_gfx_ops,
	.vendor  = PCI_VID_ATI,
	.devices = atombios_device_ids,
};
