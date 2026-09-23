/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/apob_cache.h>
#include <amdblocks/smn.h>
#include <amdblocks/smu.h>
#include <console/console.h>
#include <cpu/amd/mtrr.h>
#include <cpu/x86/cache.h>
#include <cpu/x86/msr.h>
#include <device/pci.h>
#include <device/pci_ops.h>
#include <smbios.h>
#include <soc/platform_descriptors.h>
#include <soc/smu.h>
#include <static.h>
#include <string.h>
#include <types.h>

#include "../chip.h"
#include "apob.h"
#include "nbio.h"
#include "smu.h"

/* The number of DDI descriptors the mainboards can provide */
#define DDI_DESCRIPTOR_COUNT	4

/* GPU registers accessed through SMN */
#define DC_AUDIO_PORT_CONNECTIVITY	0x0000e370
#define  PORT_CONNECTIVITY_MASK		0x7
#define  PORT_CONNECTIVITY_OVERRIDE	BIT(4)
#define LVTMA_PWRSEQ_CNTL		0x0001750c
#define ATHUB_RPB_SDPPORT_CNTL		0x00003440
#define SDMA0_CLK_CTRL			0x000049ec
#define MC_VM_NB_MMIOBASE		0x0006a040
#define MC_VM_NB_MMIOLIMIT		0x0006a044
#define MC_VM_NB_PCI_CTRL		0x0006a048
#define MC_VM_NB_PCI_ARB		0x0006a04c
#define MC_VM_NB_TOP_OF_DRAM_SLOT1	0x0006a050
#define MC_VM_NB_LOWER_TOP_OF_DRAM2	0x0006a054
#define MC_VM_NB_UPPER_TOP_OF_DRAM2	0x0006a058
#define MC_VM_FB_OFFSET			0x0006a05c
#define MC_VM_APT_CNTL			0x0006a07c
#define MC_VM_FB_LOCATION_TOP		0x0006a0b4
#define NBIF_GFX_UMA_SIZE_MB		0x1012378c

/* UMC channel registers */
#define UMC_BASE(ch)			(0x00050000 + ((ch) << 20))
#define UMC_ADDR_MASK			0x20
#define UMC_ADDR_MASK_1			0x24
#define UMC_CONFIG			0x100

/* Fuse registers that determine whether display path 2 needs to be skipped */
#define FUSE_SIGNATURE_0		0x0005d59c
#define FUSE_SIGNATURE_1		0x0005d5a0
#define FUSE_SIGNATURE_2		0x0005d5a4
#define FUSE_SIGNATURE_3		0x0005d5a8

/*
 * The VBIOS reads the integrated system info table from the end of the UMA region. The layout
 * is atom_fusion_system_info_v4 from the AMD atomfirmware.h.
 */
struct atom_common_table_header {
	uint16_t structuresize;
	uint8_t format_revision;
	uint8_t content_revision;
} __packed;

struct atom_ext_display_path {
	uint16_t device_tag;
	uint16_t device_acpi_enum;
	uint16_t connectorobjid;
	uint8_t auxddclut_index;
	uint8_t hpdlut_index;
	uint16_t ext_encoder_objid;
	uint8_t channelmapping;
	uint8_t chpninvert;
	uint16_t caps;
	uint16_t reserved;
} __packed;

struct atom_external_display_connection_info {
	struct atom_common_table_header table_header;
	uint8_t guid[16];
	struct atom_ext_display_path path[7];
	uint8_t checksum;
	uint8_t stereopinid;
	uint8_t remotedisplayconfig;
	uint8_t edptolvdsrxid;
	uint8_t fixdpvoltageswing;
	uint8_t reserved[3];
} __packed;

struct atom_14nm_dpphy_dvihdmi_tuningset {
	uint32_t max_symclk_in10khz;
	uint8_t encoder_mode;
	uint8_t phy_sel;
	uint16_t margindeemph;
	uint8_t deemph_6db_4;
	uint8_t boostadj;
	uint8_t tx_driver_fifty_ohms;
	uint8_t deemph_sel;
} __packed;

struct atom_14nm_dpphy_dp_setting {
	uint8_t dp_vs_pemph_level;
	uint16_t margindeemph;
	uint8_t deemph_6db_4;
	uint8_t boostadj;
} __packed;

struct atom_14nm_dpphy_dp_tuningset {
	uint8_t phy_sel;
	uint8_t version;
	uint16_t table_size;
	uint16_t reserved;
	struct atom_14nm_dpphy_dp_setting dptuning[10];
} __packed;

struct atom_integrated_system_info_v1_11 {
	struct atom_common_table_header table_header;
	uint32_t vbios_misc;
	uint32_t gpucapinfo;
	uint32_t system_config;
	uint32_t cpucapinfo;
	uint16_t gpuclk_ss_percentage;
	uint16_t gpuclk_ss_type;
	uint16_t lvds_ss_percentage;
	uint16_t lvds_ss_rate_10hz;
	uint16_t hdmi_ss_percentage;
	uint16_t hdmi_ss_rate_10hz;
	uint16_t dvi_ss_percentage;
	uint16_t dvi_ss_rate_10hz;
	uint16_t dpphy_override;
	uint16_t lvds_misc;
	uint16_t backlight_pwm_hz;
	uint8_t memorytype;
	uint8_t umachannelnumber;
	uint8_t pwr_on_digon_to_de;
	uint8_t pwr_on_de_to_vary_bl;
	uint8_t pwr_down_vary_bloff_to_de;
	uint8_t pwr_down_de_to_digoff;
	uint8_t pwr_off_delay;
	uint8_t pwr_on_vary_bl_to_blon;
	uint8_t pwr_down_bloff_to_vary_bloff;
	uint8_t min_allowed_bl_level;
	uint8_t htc_hyst_limit;
	uint8_t htc_tmp_limit;
	uint8_t reserved1;
	uint8_t reserved2;
	struct atom_external_display_connection_info extdispconninfo;
	struct atom_14nm_dpphy_dvihdmi_tuningset dvi_tuningset;
	struct atom_14nm_dpphy_dvihdmi_tuningset hdmi_tuningset;
	struct atom_14nm_dpphy_dvihdmi_tuningset hdmi6g_tuningset;
	struct atom_14nm_dpphy_dp_tuningset dp_tuningset;
	struct atom_14nm_dpphy_dp_tuningset dp_hbr3_tuningset;
	uint8_t camera_info_and_retimer_sets[0xf8];
	struct atom_14nm_dpphy_dp_tuningset dp_hbr_tuningset;
	struct atom_14nm_dpphy_dp_tuningset dp_hbr2_tuningset;
	struct atom_14nm_dpphy_dp_tuningset edp_tuningset;
	uint32_t reserved[66];
} __packed;

struct atom_fusion_system_info_v4 {
	struct atom_integrated_system_info_v1_11 sysinfo;
	uint32_t powerplayinfo[256];
} __packed;

check_member(atom_integrated_system_info_v1_11, extdispconninfo, 0x038);
check_member(atom_integrated_system_info_v1_11, dp_tuningset, 0x0e8);
check_member(atom_integrated_system_info_v1_11, dp_hbr_tuningset, 0x250);
check_member(atom_integrated_system_info_v1_11, edp_tuningset, 0x2c0);
_Static_assert(sizeof(struct atom_integrated_system_info_v1_11) == 0x400,
	       "Unexpected integrated system info size");
_Static_assert(sizeof(struct atom_fusion_system_info_v4) == 0x800,
	       "Unexpected fusion system info size");

/* ATOM display device classes */
enum atom_device_class {
	ATOM_DEVICE_NONE,
	ATOM_DEVICE_DFP,
	ATOM_DEVICE_CRT,
	ATOM_DEVICE_LCD,
};

#define CONNECTOR_OBJECT(id)			(0x3000 | (id))
#define CONNECTOR_OBJECT_ID_SINGLE_LINK_DVI_I	0x01
#define CONNECTOR_OBJECT_ID_SINGLE_LINK_DVI_D	0x03
#define CONNECTOR_OBJECT_ID_DUAL_LINK_DVI_D	0x04
#define CONNECTOR_OBJECT_ID_VGA			0x05
#define CONNECTOR_OBJECT_ID_HDMI_TYPE_A		0x0c
#define CONNECTOR_OBJECT_ID_LVDS		0x0e
#define CONNECTOR_OBJECT_ID_DISPLAYPORT		0x13
#define CONNECTOR_OBJECT_ID_EDP			0x14
#define CONNECTOR_OBJECT_ID_LVDS_EDP		0x16
#define ENUM_ID_SHIFT				8

struct ddi_connector {
	enum atom_device_class device_class;
	uint16_t object_id;
};

static const struct ddi_connector ddi_connectors[MAX_CONNECTOR_TYPE] = {
	[DP] = { ATOM_DEVICE_DFP,
		CONNECTOR_OBJECT(CONNECTOR_OBJECT_ID_DISPLAYPORT) },
	[EDP] = { ATOM_DEVICE_DFP, CONNECTOR_OBJECT(CONNECTOR_OBJECT_ID_EDP) },
	[SINGLE_LINK_DVI] = { ATOM_DEVICE_DFP,
		CONNECTOR_OBJECT(CONNECTOR_OBJECT_ID_SINGLE_LINK_DVI_D) },
	[DUAL_LINK_DVI] = { ATOM_DEVICE_DFP,
		CONNECTOR_OBJECT(CONNECTOR_OBJECT_ID_DUAL_LINK_DVI_D) },
	[HDMI] = { ATOM_DEVICE_DFP,
		CONNECTOR_OBJECT(CONNECTOR_OBJECT_ID_HDMI_TYPE_A) },
	[DP_TO_VGA] = { ATOM_DEVICE_CRT, CONNECTOR_OBJECT(CONNECTOR_OBJECT_ID_VGA) },
	[DP_TO_LVDS] = { ATOM_DEVICE_LCD, CONNECTOR_OBJECT(CONNECTOR_OBJECT_ID_LVDS) },
	[NUTMEG_DP_TO_VGA] = { ATOM_DEVICE_CRT,
		CONNECTOR_OBJECT(CONNECTOR_OBJECT_ID_VGA) },
	[SINGLE_LINK_DVI_I] = { ATOM_DEVICE_DFP,
		CONNECTOR_OBJECT(CONNECTOR_OBJECT_ID_SINGLE_LINK_DVI_I) },
	[CRT] = { ATOM_DEVICE_CRT, CONNECTOR_OBJECT(CONNECTOR_OBJECT_ID_VGA) },
	[LVDS] = { ATOM_DEVICE_LCD, CONNECTOR_OBJECT(CONNECTOR_OBJECT_ID_LVDS) },
	[EDP_TO_LVDS] = { ATOM_DEVICE_LCD, CONNECTOR_OBJECT(CONNECTOR_OBJECT_ID_EDP) },
	[EDP_TO_LVDS_SW] = { ATOM_DEVICE_LCD,
		CONNECTOR_OBJECT(CONNECTOR_OBJECT_ID_EDP) },
	[AUTO_DETECT] = { ATOM_DEVICE_LCD,
		CONNECTOR_OBJECT(CONNECTOR_OBJECT_ID_LVDS_EDP) },
	[UNUSED_TYPE] = { ATOM_DEVICE_NONE, 0 },
};

struct atom_device {
	enum atom_device_class device_class;
	uint8_t ordinal;
	uint16_t tag;
	uint16_t acpi_enum;
};

static const struct atom_device atom_devices[] = {
	{ ATOM_DEVICE_CRT, 1, 0x0001, 0x0100 },	/* CRT1 */
	{ ATOM_DEVICE_LCD, 1, 0x0002, 0x0110 },	/* LCD1 */
	{ ATOM_DEVICE_DFP, 1, 0x0008, 0x0210 },	/* DFP1 */
	{ ATOM_DEVICE_DFP, 2, 0x0080, 0x0220 },	/* DFP2 */
	{ ATOM_DEVICE_DFP, 3, 0x0200, 0x0230 },	/* DFP3 */
	{ ATOM_DEVICE_DFP, 4, 0x0400, 0x0240 },	/* DFP4 */
	{ ATOM_DEVICE_DFP, 5, 0x0800, 0x0250 },	/* DFP5 */
	{ ATOM_DEVICE_DFP, 6, 0x0040, 0x0260 },	/* DFP6 */
};

#define DISPLAY_PATH_CAPS_DDI_2_3	0x0100
#define DISPLAY_PATH_CAPS_HDMI20_DIS	0x0200
/* The lanes of the display paths aren't swapped */
#define DISPLAY_PATH_CHANNEL_MAPPING	0xe4

static bool gfx_uma_region(uint32_t *base, uint32_t *size)
{
	struct picasso_apob_hole holes[APOB_MAX_HOLES];
	uint64_t top;
	int count;

	count = picasso_apob_memory_map(soc_fill_apob_cache(), &top, holes);
	for (int i = 0; i < count; i++) {
		const uint64_t end = holes[i].base + holes[i].size;

		if (holes[i].type != APOB_HOLE_UMA ||
		    holes[i].size < sizeof(struct atom_fusion_system_info_v4) || end > 4ULL * GiB)
			continue;
		*base = holes[i].base;
		*size = holes[i].size;
		return true;
	}
	return false;
}

static uint8_t gfx_memory_type(uint32_t umc_config)
{
	switch (umc_config & 7) {
	case 0:
		return MEMORY_TYPE_DDR4;
	case 1:
		return MEMORY_TYPE_DDR3;
	case 4:
		return MEMORY_TYPE_LPDDR3;
	case 5:
		return MEMORY_TYPE_LPDDR4;
	default:
		return MEMORY_TYPE_UNKNOWN;
	}
}

/*
 * The eDP connector is always the first display device. The DP, HDMI and single link DVI
 * connectors get numbered in this order. Other connectors don't get a number.
 */
static unsigned int ddi_ordinal(const ddi_descriptor *ddi, size_t count,
				size_t index)
{
	static const ddi_connector_type order[] = {
		DP,
		HDMI,
		SINGLE_LINK_DVI,
	};
	unsigned int ordinal = 1;

	if (ddi[index].connector_type == EDP)
		return 1;

	for (size_t o = 0; o < ARRAY_SIZE(order); o++) {
		for (size_t i = 0; i < count; i++) {
			if (ddi[i].connector_type != order[o])
				continue;
			if (i == index)
				return ordinal;
			ordinal++;
		}
	}

	return 0;
}

static void fill_display_path(struct atom_ext_display_path *path,
			      const ddi_descriptor *ddi, size_t count,
			      size_t index, uint8_t hdmi2_disable)
{
	const ddi_connector_type type = ddi[index].connector_type;
	const struct ddi_connector *connector = &ddi_connectors[type];
	const unsigned int ordinal = ddi_ordinal(ddi, count, index);
	const struct atom_device *device = NULL;

	if (type == UNUSED_TYPE)
		return;

	for (size_t i = 0; i < ARRAY_SIZE(atom_devices); i++) {
		if (atom_devices[i].device_class != connector->device_class)
			continue;
		/* There is only one LCD device. The DFP and CRT devices need an ordinal. */
		if (connector->device_class == ATOM_DEVICE_LCD ||
		    atom_devices[i].ordinal == ordinal) {
			device = &atom_devices[i];
			break;
		}
	}

	if (type == EDP) {
		/* eDP is the LCD1 device */
		path->device_tag = 0x0002;
		path->device_acpi_enum = 0x0110;
		path->connectorobjid = CONNECTOR_OBJECT(CONNECTOR_OBJECT_ID_EDP) |
				       1 << ENUM_ID_SHIFT;
	} else {
		if (device) {
			path->device_tag = device->tag;
			path->device_acpi_enum = device->acpi_enum;
		}
		path->connectorobjid = connector->object_id | ordinal << ENUM_ID_SHIFT;
	}
	/* The ATOM LUT indices use the same zero-based numbering as the board descriptors. */
	path->auxddclut_index = ddi[index].aux_index;
	path->hpdlut_index = ddi[index].hdp_index;
	path->channelmapping = DISPLAY_PATH_CHANNEL_MAPPING;
	if ((type == DP || type == HDMI) &&
	    (hdmi2_disable & BIT(index)))
		path->caps = DISPLAY_PATH_CAPS_HDMI20_DIS;
	if (index >= 2)
		path->caps = DISPLAY_PATH_CAPS_DDI_2_3;
}

/* Some parts need display path 2 to stay unused */
static bool skip_display_path_2(void)
{
	if ((smn_read32(SMU_FUSE_OPN) >> 30) == 2)
		return true;
	return (smn_read32(FUSE_SIGNATURE_0) >> 16) == 0xa6da &&
		smn_read32(FUSE_SIGNATURE_1) == 0x0d83066c &&
		smn_read32(FUSE_SIGNATURE_2) == 0xd3d95469 &&
		(smn_read32(FUSE_SIGNATURE_3) & 0x7ff) == 0x428;
}

static void fill_dp_tuning_defaults(struct atom_14nm_dpphy_dp_tuningset *set)
{
	memset(set->dptuning, 0xff, sizeof(set->dptuning));
}

static void write_system_info(uint32_t uma_base, uint32_t uma_size,
			      const ddi_descriptor *ddi, size_t ddi_count)
{
	struct atom_fusion_system_info_v4 *const table =
		(void *)(uintptr_t)(uma_base + uma_size - sizeof(*table));
	struct atom_integrated_system_info_v1_11 *const info = &table->sysinfo;
	const struct soc_amd_picasso_config *config = config_of_soc();
	unsigned int channels = 0;
	uint32_t umc_config = 0;
	bool skip_path_2;

	memset(table, 0, sizeof(*table));
	info->table_header = (struct atom_common_table_header) {
		.structuresize = sizeof(*info),
		.format_revision = 1,
		.content_revision = 11,
	};
	info->gpuclk_ss_percentage = 375;	/* 0.375% */
	info->backlight_pwm_hz = 512;
	info->pwr_on_vary_bl_to_blon = 5;
	info->pwr_down_bloff_to_vary_bloff = 5;
	info->extdispconninfo.table_header = (struct atom_common_table_header) {
		.structuresize = sizeof(info->extdispconninfo),
		.format_revision = 1,
		.content_revision = 1,
	};

	fill_dp_tuning_defaults(&info->dp_tuningset);
	fill_dp_tuning_defaults(&info->dp_hbr3_tuningset);
	fill_dp_tuning_defaults(&info->dp_hbr_tuningset);
	fill_dp_tuning_defaults(&info->dp_hbr2_tuningset);
	fill_dp_tuning_defaults(&info->edp_tuningset);

	if (config->edp_pwr_adjust_enable) {
		info->pwr_on_digon_to_de = config->pwron_digon_to_de;
		info->pwr_on_de_to_vary_bl = config->pwron_de_to_varybl;
		info->pwr_down_vary_bloff_to_de = config->pwrdown_varybloff_to_de;
		info->pwr_down_de_to_digoff = config->pwrdown_de_to_digoff;
		info->pwr_off_delay = config->pwroff_delay;
		info->pwr_on_vary_bl_to_blon = config->pwron_varybl_to_blon;
		info->pwr_down_bloff_to_vary_bloff = config->pwrdown_bloff_to_varybloff;
		info->min_allowed_bl_level = config->min_allowed_bl_level;
	}

	if (config->edp_phy_override & ENABLE_EDP_TUNINGSET) {
		struct atom_14nm_dpphy_dp_tuningset *edp = &info->edp_tuningset;

		memset(edp, 0, sizeof(*edp));
		info->dpphy_override = config->edp_phy_override;
		edp->phy_sel = config->edp_physel;
		edp->table_size = sizeof(edp->dptuning[0]);
		edp->dptuning[0].dp_vs_pemph_level = config->edp_tuningset.dp_vs_pemph_level;
		edp->dptuning[0].margindeemph = config->edp_tuningset.margin_deemph;
		edp->dptuning[0].deemph_6db_4 = config->edp_tuningset.deemph_6db4;
		edp->dptuning[0].boostadj = config->edp_tuningset.boostadj;
		for (size_t i = 1; i < ARRAY_SIZE(edp->dptuning); i++)
			edp->dptuning[i].dp_vs_pemph_level = 0xff;
	}

	for (unsigned int ch = 0; ch < 2; ch++) {
		if (smn_read32(UMC_BASE(ch) + UMC_ADDR_MASK) ||
		    smn_read32(UMC_BASE(ch) + UMC_ADDR_MASK_1)) {
			channels++;
			umc_config = smn_read32(UMC_BASE(ch) + UMC_CONFIG);
		}
	}
	info->memorytype = gfx_memory_type(umc_config);
	info->umachannelnumber = channels;

	skip_path_2 = skip_display_path_2();
	for (size_t i = 0; i < ddi_count; i++)
		fill_display_path(&info->extdispconninfo.path[i + (skip_path_2 && i >= 2)],
				  ddi, ddi_count, i, config->hdmi2_disable);

	if (clflush_supported())
		clflush_region((uintptr_t)table, sizeof(*table));
	else
		wbinvd();

	printk(BIOS_DEBUG, "GFX system info at %p: memory type %02x, %u channels\n",
	       table, info->memorytype, channels);
}

static bool ddi_descriptors_valid(const ddi_descriptor *ddi, size_t count)
{
	if ((!ddi && count) || count > DDI_DESCRIPTOR_COUNT)
		return false;

	for (size_t i = 0; i < count; i++) {
		if (ddi[i].connector_type >= MAX_CONNECTOR_TYPE || ddi[i].aux_index >= AUX_MAX ||
		    ddi[i].hdp_index >= HDP_MAX)
			return false;
	}

	return true;
}

/* Set up the GPU's view of the memory map and the display configuration for the VBIOS */
enum cb_err picasso_gfx_init(void)
{
	const dxio_descriptor *dxio = NULL;
	const ddi_descriptor *ddi = NULL;
	struct smu_payload payload = { 0 };
	size_t dxio_count = 0;
	size_t ddi_count = 0;
	uint32_t uma_base;
	uint32_t uma_size;
	uint64_t tom2;

	if (!picasso_gfx_available())
		return CB_SUCCESS;

	mainboard_get_dxio_ddi_descriptors(&dxio, &dxio_count, &ddi, &ddi_count);
	if (!ddi_descriptors_valid(ddi, ddi_count))
		return CB_ERR;

	pci_or_config16(DEV_PTR(gfx), PCI_COMMAND, PCI_COMMAND_MASTER);

	for (size_t i = 0; i < ddi_count; i++) {
		if (ddi[i].connector_type == EDP) {
			/* Enable the panel power sequencer */
			smn_clrsetbits32(LVTMA_PWRSEQ_CNTL, 0, 0x02000011);
			break;
		}
	}

	if (!gfx_uma_region(&uma_base, &uma_size)) {
		printk(BIOS_ERR, "No UMA region in the APOB\n");
		return CB_ERR;
	}
	write_system_info(uma_base, uma_size, ddi, ddi_count);

	smn_write32(NBIF_GFX_UMA_SIZE_MB, uma_size >> 20);
	smn_write32(MC_VM_FB_OFFSET, uma_base >> 24);
	smn_write32(MC_VM_FB_LOCATION_TOP, (uma_base + uma_size - 1) >> 24);
	smn_clrsetbits32(MC_VM_APT_CNTL, BIT(1), BIT(1));

	/* The GPU has its own copy of the DRAM and MMIO map of the IOHC */
	smn_clrsetbits32(MC_VM_NB_TOP_OF_DRAM_SLOT1, 0xff800000, (uint32_t)msr_read(TOP_MEM_MSR));
	tom2 = msr_read(TOP_MEM2_MSR);
	smn_clrsetbits32(MC_VM_NB_LOWER_TOP_OF_DRAM2, 0xff800001, (uint32_t)tom2 | 1);
	smn_clrsetbits32(MC_VM_NB_UPPER_TOP_OF_DRAM2, 0x00000fff, (uint32_t)(tom2 >> 32));
	smn_write32(MC_VM_NB_MMIOBASE, smn_read32(NB_MMIOBASE));
	smn_write32(MC_VM_NB_MMIOLIMIT, smn_read32(NB_MMIOLIMIT));
	smn_write32(MC_VM_NB_PCI_ARB, smn_read32(NB_PCI_ARB));
	smn_clrsetbits32(MC_VM_NB_PCI_CTRL, NB_PCI_CTRL_MMIO_ENABLE, smn_read32(NB_PCI_CTRL));
	smn_clrsetbits32(NB_PCI_CTRL, 0, NB_PCI_CTRL_PMEDIS);

	smn_clrsetbits32(ATHUB_RPB_SDPPORT_CNTL, BIT(1) | BIT(2), 0);

	/* Expose 4 display audio endpoints */
	smn_clrsetbits32(DC_AUDIO_PORT_CONNECTIVITY, PORT_CONNECTIVITY_MASK,
			 (~4 & PORT_CONNECTIVITY_MASK) | PORT_CONNECTIVITY_OVERRIDE);

	smn_clrsetbits32(NBIF_FUNC_STRAP(8), 0, NBIF_FUNC_STRAP_FUNC_EN);
	smn_clrsetbits32(SDMA0_CLK_CTRL, 0xff000000, 0);

	return send_smu_message(SMC_MSG_0E, &payload);
}
