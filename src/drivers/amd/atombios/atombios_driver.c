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

/* ATOM command table indices (via offsetof into ATOM_MASTER_LIST_OF_COMMAND_TABLES) */
#define CMD_IDX(field) \
	(offsetof(ATOM_MASTER_LIST_OF_COMMAND_TABLES, field) / sizeof(USHORT))

#define ATOM_CMD_ASIC_INIT		CMD_IDX(ASIC_Init)
#define ATOM_CMD_SET_PIXEL_CLOCK	CMD_IDX(SetPixelClock)
#define ATOM_CMD_ENABLE_DISP_POWER_GATING	CMD_IDX(EnableDispPowerGating)
#define ATOM_CMD_SET_CRTC_TIMING	CMD_IDX(SetCRTC_Timing)
#define ATOM_CMD_SET_CRTC_USING_DTD	CMD_IDX(SetCRTC_UsingDTDTiming)
#define ATOM_CMD_ENABLE_CRTC		CMD_IDX(EnableCRTC)
#define ATOM_CMD_BLANK_CRTC		CMD_IDX(BlankCRTC)
#define ATOM_CMD_SELECT_CRTC_SOURCE	CMD_IDX(SelectCRTC_Source)
#define ATOM_CMD_DIG1_XMTR_CTRL	CMD_IDX(DIG1TransmitterControl)
#define ATOM_CMD_DIG_ENCODER_CTRL	CMD_IDX(DIGxEncoderControl)
#define ATOM_CMD_ENABLE_SCALER		CMD_IDX(EnableScaler)
#define ATOM_CMD_SET_CRTC_OVERSCAN	CMD_IDX(SetCRTC_OverScan)
#define ATOM_CMD_ENABLE_SS		CMD_IDX(EnableSpreadSpectrumOnPPLL)
#define ATOM_CMD_ADJUST_DISPLAY_PLL	CMD_IDX(AdjustDisplayPll)
#define ATOM_CMD_SET_DCE_CLOCK		CMD_IDX(SetDCEClock)

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
static void cail_mc_write(struct card_info *info, uint32_t reg, uint32_t val)
{
}

static uint32_t cail_mc_read(struct card_info *info, uint32_t reg)
{
	return 0;
}

static void cail_pll_write(struct card_info *info, uint32_t reg, uint32_t val)
{
}

static uint32_t cail_pll_read(struct card_info *info, uint32_t reg)
{
	return 0;
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
		/* Enable ROM */
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
		printk(BIOS_ERR, "ATOMBIOS: ROM BAR does not contain valid BIOS (no AA55)\n");
		pci_write_config32(dev, PCI_ROM_ADDRESS, rom_bar & ~1);
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
	pci_write_config32(dev, PCI_ROM_ADDRESS, rom_bar & ~1);

	printk(BIOS_INFO, "ATOMBIOS: loaded VBIOS from ROM BAR (%zu bytes)\n", rom_size);
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

/* ---- Modesetting operations ---- */

/*
 * Set up a basic CRTC timing using DTD parameters from EDID.
 * This calls SetCRTC_UsingDTDTiming.
 */
static int atombios_set_crtc_dtd_timing(struct atom_context *ctx,
					 const struct edid_mode *mode,
					 int crtc_id)
{
	/* SET_CRTC_USING_DTD_TIMING parameter space (v1) */
	uint32_t params[8];
	uint16_t h_size, h_blanking;
	uint16_t v_size, v_blanking;
	uint16_t h_sync_offset, h_sync_width;
	uint16_t v_sync_offset, v_sync_width;
	uint8_t misc_flags = 0;

	memset(params, 0, sizeof(params));

	h_size = mode->ha;
	h_blanking = mode->hbl;
	v_size = mode->va;
	v_blanking = mode->vbl;
	h_sync_offset = mode->hso;
	h_sync_width = mode->hspw;
	v_sync_offset = mode->vso;
	v_sync_width = mode->vspw;

	if (mode->phsync)
		misc_flags |= 0x02; /* H_SYNC_POLARITY */
	if (mode->pvsync)
		misc_flags |= 0x04; /* V_SYNC_POLARITY */

	/* Pack into parameter space matching
	 * SET_CRTC_USING_DTD_TIMING_PARAMETERS layout */
	params[0] = h_size | ((uint32_t)h_blanking << 16);
	params[1] = v_size | ((uint32_t)v_blanking << 16);
	params[2] = h_sync_offset | ((uint32_t)h_sync_width << 16);
	params[3] = v_sync_offset | ((uint32_t)v_sync_width << 16);
	/* param[4]: flags | interlace | crtc_id */
	params[4] = misc_flags | ((uint32_t)crtc_id << 24);

	return atombios_exec(ctx, ATOM_CMD_SET_CRTC_USING_DTD, params,
			     sizeof(params) / sizeof(uint32_t));
}

/*
 * Enable or disable a CRTC.
 */
static int atombios_enable_crtc(struct atom_context *ctx, int crtc_id,
				int enable)
{
	uint32_t params[1];
	/* ENABLE_CRTC_PARAMETERS: ucCRTC (byte 0), ucEnable (byte 1) */
	params[0] = crtc_id | ((enable ? ATOM_ENABLE : ATOM_DISABLE) << 8);
	return atombios_exec(ctx, ATOM_CMD_ENABLE_CRTC, params, 1);
}

/*
 * Blank or unblank a CRTC.
 */
static int atombios_blank_crtc(struct atom_context *ctx, int crtc_id, int blank)
{
	uint32_t params[1];
	/* BLANK_CRTC_PARAMETERS: ucCRTC (byte 0), ucBlanking (byte 1) */
	params[0] = crtc_id | ((blank ? ATOM_BLANKING : ATOM_BLANKING_OFF) << 8);
	return atombios_exec(ctx, ATOM_CMD_BLANK_CRTC, params, 1);
}

/*
 * Display power gating: enable or disable power to a display pipe.
 */
static int atombios_disp_power_gating(struct atom_context *ctx, int crtc_id,
				       int gate)
{
	uint32_t params[1];
	/* ENABLE_DISP_POWER_GATING_PARAMETERS_V2_1:
	 * ucDispPipeId (byte 0), ucEnable (byte 1) */
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
	struct edid_mode default_mode;

	printk(BIOS_INFO, "ATOMBIOS: initializing AMD GPU %04x:%04x\n",
	       dev->vendor, dev->device);

	/* Map MMIO BAR */
	mmio_bar = pci_read_config32(dev, AMD_GPU_MMIO_BAR);
	mmio_bar &= ~0xF;
	if (!mmio_bar) {
		/* Try BAR2 as fallback */
		mmio_bar = pci_read_config32(dev, PCI_BASE_ADDRESS_2);
		mmio_bar &= ~0xF;
	}
	if (!mmio_bar) {
		printk(BIOS_ERR, "ATOMBIOS: could not find MMIO BAR\n");
		return;
	}
	mmio_base = (void *)(uintptr_t)mmio_bar;
	printk(BIOS_INFO, "ATOMBIOS: MMIO base at %p\n", mmio_base);

	/* Get framebuffer BAR */
	fb_bar = pci_read_config32(dev, AMD_GPU_FB_BAR);
	fb_bar &= ~0xF;
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

	/* Allocate scratch memory (replaces FB scratch window) */
	scratch = calloc(ATOM_SCRATCH_SIZE_DWORDS, sizeof(uint32_t));
	if (!scratch) {
		printk(BIOS_ERR, "ATOMBIOS: failed to allocate scratch memory\n");
		atom_destroy(ctx);
		return;
	}
	ctx->scratch = scratch;
	ctx->scratch_size_bytes = ATOM_SCRATCH_SIZE_DWORDS * sizeof(uint32_t);

	/* Execute AsicInit to bring up the GPU */
	printk(BIOS_INFO, "ATOMBIOS: executing AsicInit...\n");
	if (atom_asic_init(ctx)) {
		printk(BIOS_WARNING, "ATOMBIOS: AsicInit failed or not present "
		       "(may be OK if GPU is already initialized)\n");
		/* Continue anyway - APUs/pre-init GPUs may not need this */
	}

	/*
	 * For a discrete GPU with a connected display, the typical modesetting
	 * sequence is:
	 *
	 * 1. AsicInit (done above)
	 * 2. EnableDispPowerGating(DISABLE) - ungate the display pipe
	 * 3. SetCRTC_UsingDTDTiming - program CRTC timings from EDID
	 * 4. EnableCRTC(ENABLE) - start the CRTC scan engine
	 * 5. BlankCRTC(DISABLE) - unblank the output
	 *
	 * DIG encoder and PHY transmitter configuration depends on the
	 * connected display type (DP, HDMI, DVI, etc.) and would need
	 * display-specific parameters. For initial testing, we program
	 * the CRTC and leave encoder setup to the payload/OS.
	 *
	 * TODO: Add encoder/transmitter setup for specific display types.
	 * TODO: Add EDID reading via ATOM I2C tables or GMBUS.
	 */

	/* Use a safe default mode: 1024x768@60 */
	memset(&default_mode, 0, sizeof(default_mode));
	default_mode.name = "1024x768@60";
	default_mode.pixel_clock = 65000;	/* kHz */
	default_mode.ha = 1024;			/* horizontal active */
	default_mode.hbl = 320;			/* horizontal blanking */
	default_mode.hso = 24;			/* h sync offset */
	default_mode.hspw = 136;		/* h sync pulse width */
	default_mode.va = 768;			/* vertical active */
	default_mode.vbl = 38;			/* vertical blanking */
	default_mode.vso = 3;			/* v sync offset */
	default_mode.vspw = 6;			/* v sync pulse width */
	default_mode.phsync = 0;		/* negative h sync */
	default_mode.pvsync = 0;		/* negative v sync */

	/* Step 2: Power up display pipe */
	printk(BIOS_INFO, "ATOMBIOS: enabling display pipe %d...\n", crtc_id);
	atombios_disp_power_gating(ctx, crtc_id, 0);

	/* Step 3: Program CRTC timing */
	printk(BIOS_INFO, "ATOMBIOS: setting CRTC timing for %s...\n",
	       default_mode.name);
	if (atombios_set_crtc_dtd_timing(ctx, &default_mode, crtc_id))
		printk(BIOS_WARNING, "ATOMBIOS: SetCRTC_UsingDTDTiming failed\n");

	/* Step 4: Enable CRTC */
	printk(BIOS_INFO, "ATOMBIOS: enabling CRTC %d...\n", crtc_id);
	atombios_enable_crtc(ctx, crtc_id, 1);

	/* Step 5: Unblank */
	printk(BIOS_INFO, "ATOMBIOS: unblanking CRTC %d...\n", crtc_id);
	atombios_blank_crtc(ctx, crtc_id, 0);

	/* Register framebuffer with coreboot.
	 * The FB is at the start of the VRAM aperture (BAR0). */
	fb_add_framebuffer_info(fb_bar,
				default_mode.ha,
				default_mode.va,
				default_mode.ha * BYTES_PER_PIXEL,
				DEFAULT_BPP);

	printk(BIOS_INFO, "ATOMBIOS: framebuffer %ux%u @ 0x%08x registered\n",
	       default_mode.ha, default_mode.va, fb_bar);

	/* Leave ctx/scratch alive -- payload may want to use ATOM tables */
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
 * The ATOM interpreter handles the hardware differences between GPU
 * generations -- the VBIOS bytecode carries the GPU-specific knowledge.
 * To add support for a new GPU, simply add its PCI device ID to this
 * list (find it with "lspci -nn").
 *
 * The table includes representative IDs from several generations:
 *   - Polaris (RX 460/470/480/560/570/580)
 *   - Vega (Vega 56/64, VII)
 *   - Navi 10/14 (RX 5600/5700)
 *   - Navi 21/22/23 (RX 6800/6700/6600)
 *   - Navi 31/32/33 (RX 7900/7800/7600)
 */
static const unsigned short atombios_device_ids[] = {
	/* Polaris 10 (RX 470/480) */
	0x67df,
	/* Polaris 11 (RX 460) */
	0x67ef,
	/* Polaris 20 (RX 570/580) */
	0x67df,
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
