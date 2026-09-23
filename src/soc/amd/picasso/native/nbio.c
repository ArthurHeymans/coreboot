/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/acp.h>
#include <amdblocks/smn.h>
#include <amdblocks/smu.h>
#include <cbmem.h>
#include <console/console.h>
#include <cpu/amd/mtrr.h>
#include <cpu/cpu.h>
#include <cpu/x86/msr.h>
#include <soc/cpu.h>
#include <soc/iomap.h>
#include <soc/smu.h>
#include <soc/soc_util.h>
#include <types.h>

#include "fch_regs.h"
#include "native.h"
#include "nbio.h"
#include "smu.h"

/* NBIO state that romstage determines and ramstage uses. It's kept in CBMEM. */
struct nbio_capabilities {
	uint32_t silicon_id;
	bool gfx_available;
};

/*
 * Register settings: mask has the bits that get changed. A mask of UINT32_MAX writes the
 * whole register.
 */
struct nbio_reg {
	uint32_t reg;
	uint32_t mask;
	uint32_t value;
};

static const struct nbio_reg nbio_base_table[] = {
	/* IOHC configuration */
	{ 0x13b10004, 0x0000ffff, 0x00000010 },
	{ 0x13b10118, 0x00000006, 0x00000002 },
	{ 0x13b10230, 0x00000003, 0x00000001 },

	/* IOHC SION arbitration */
	{ 0x13b14400, UINT32_MAX, 0x08080808 },
	{ 0x13b14404, UINT32_MAX, 0x08080808 },
	{ 0x13b14804, UINT32_MAX, 0x08080808 },
	{ 0x13b14808, UINT32_MAX, 0x08080808 },
	{ 0x13b14c08, UINT32_MAX, 0x08080808 },
	{ 0x13b14c0c, UINT32_MAX, 0x08080808 },
	{ 0x13b1500c, UINT32_MAX, 0x08080808 },
	{ 0x13b15010, UINT32_MAX, 0x08080808 },
	{ 0x13b15410, UINT32_MAX, 0x08080808 },
	{ 0x13b15414, UINT32_MAX, 0x08080808 },
	{ 0x13b14408, UINT32_MAX, 0x21212121 },
	{ 0x13b1440c, UINT32_MAX, 0x21212121 },
	{ 0x13b1480c, UINT32_MAX, 0x21212121 },
	{ 0x13b14810, UINT32_MAX, 0x21212121 },
	{ 0x13b14c10, UINT32_MAX, 0x84218421 },
	{ 0x13b14c14, UINT32_MAX, 0x84218421 },
	{ 0x13b15014, UINT32_MAX, 0x85218521 },
	{ 0x13b15018, UINT32_MAX, 0x85218521 },
	{ 0x13b15418, UINT32_MAX, 0x00000000 },
	{ 0x13b1541c, UINT32_MAX, 0x00000000 },
	{ 0x13b14430, UINT32_MAX, 0x08080808 },
	{ 0x13b14434, UINT32_MAX, 0x08080808 },
	{ 0x13b14834, UINT32_MAX, 0x08080808 },
	{ 0x13b14838, UINT32_MAX, 0x08080808 },
	{ 0x13b14c38, UINT32_MAX, 0x08080808 },
	{ 0x13b14c3c, UINT32_MAX, 0x08080808 },
	{ 0x13b1503c, UINT32_MAX, 0x08080808 },
	{ 0x13b15040, UINT32_MAX, 0x08080808 },
	{ 0x13b15440, UINT32_MAX, 0x08080808 },
	{ 0x13b15444, UINT32_MAX, 0x08080808 },
	{ 0x13b14408, UINT32_MAX, 0x33333030 },
	{ 0x13b1440c, UINT32_MAX, 0x33333030 },
	{ 0x13b1483c, UINT32_MAX, 0x00000000 },
	{ 0x13b14840, UINT32_MAX, 0x00000000 },
	{ 0x13b14c40, UINT32_MAX, 0x00000000 },
	{ 0x13b14c44, UINT32_MAX, 0x00000000 },
	{ 0x13b15044, UINT32_MAX, 0x00000000 },
	{ 0x13b15048, UINT32_MAX, 0x00000000 },
	{ 0x13b15448, UINT32_MAX, 0x00000000 },
	{ 0x13b1544c, UINT32_MAX, 0x00000000 },
	{ 0x13b14410, UINT32_MAX, 0x02020202 },
	{ 0x13b14814, UINT32_MAX, 0x02020202 },
	{ 0x13b14c18, UINT32_MAX, 0x02020202 },
	{ 0x13b1501c, UINT32_MAX, 0x02020202 },
	{ 0x13b15420, UINT32_MAX, 0x02020202 },
	{ 0x13b14414, UINT32_MAX, 0x02020202 },
	{ 0x13b14818, UINT32_MAX, 0x02020202 },
	{ 0x13b14c1c, UINT32_MAX, 0x02020202 },
	{ 0x13b15020, UINT32_MAX, 0x02020202 },
	{ 0x13b15424, UINT32_MAX, 0x02020202 },
	{ 0x13b14440, UINT32_MAX, 0x02020202 },
	{ 0x13b14844, UINT32_MAX, 0x02020202 },
	{ 0x13b14c48, UINT32_MAX, 0x02020202 },
	{ 0x13b1504c, UINT32_MAX, 0x02020202 },
	{ 0x13b15450, UINT32_MAX, 0x02020202 },
	{ 0x13b14444, UINT32_MAX, 0x02020202 },
	{ 0x13b14848, UINT32_MAX, 0x02020202 },
	{ 0x13b14c4c, UINT32_MAX, 0x02020202 },
	{ 0x13b15050, UINT32_MAX, 0x02020202 },
	{ 0x13b15454, UINT32_MAX, 0x02020202 },

	/* IOAGR SION arbitration */
	{ 0x15b00400, UINT32_MAX, 0x08080808 },
	{ 0x15b00800, UINT32_MAX, 0x08080808 },
	{ 0x15b00c00, UINT32_MAX, 0x08080808 },
	{ 0x15b01000, UINT32_MAX, 0x08080808 },
	{ 0x15b00404, UINT32_MAX, 0x08080808 },
	{ 0x15b00804, UINT32_MAX, 0x08080808 },
	{ 0x15b00c04, UINT32_MAX, 0x08080808 },
	{ 0x15b01004, UINT32_MAX, 0x08080808 },
	{ 0x15b00408, UINT32_MAX, 0x21212121 },
	{ 0x15b00808, UINT32_MAX, 0x21212121 },
	{ 0x15b00c08, UINT32_MAX, 0x21212121 },
	{ 0x15b0040c, UINT32_MAX, 0x21212121 },
	{ 0x15b0080c, UINT32_MAX, 0x21212121 },
	{ 0x15b00c0c, UINT32_MAX, 0x21212121 },
	{ 0x15b01008, UINT32_MAX, 0x84218421 },
	{ 0x15b0100c, UINT32_MAX, 0x84218421 },
	{ 0x15b00430, UINT32_MAX, 0x08080808 },
	{ 0x15b00830, UINT32_MAX, 0x08080808 },
	{ 0x15b00c30, UINT32_MAX, 0x08080808 },
	{ 0x15b01030, UINT32_MAX, 0x08080808 },
	{ 0x15b00434, UINT32_MAX, 0x08080808 },
	{ 0x15b00834, UINT32_MAX, 0x08080808 },
	{ 0x15b00c34, UINT32_MAX, 0x08080808 },
	{ 0x15b01034, UINT32_MAX, 0x08080808 },
	{ 0x15b00438, UINT32_MAX, 0x00000000 },
	{ 0x15b00838, UINT32_MAX, 0x00000000 },
	{ 0x15b00c38, UINT32_MAX, 0x00000000 },
	{ 0x15b01038, UINT32_MAX, 0x00000000 },
	{ 0x15b0043c, UINT32_MAX, 0x00000000 },
	{ 0x15b0083c, UINT32_MAX, 0x00000000 },
	{ 0x15b00c3c, UINT32_MAX, 0x00000000 },
	{ 0x15b0103c, UINT32_MAX, 0x00000000 },
	{ 0x15b00410, UINT32_MAX, 0x02020202 },
	{ 0x15b00810, UINT32_MAX, 0x02020202 },
	{ 0x15b00c10, UINT32_MAX, 0x02020202 },
	{ 0x15b01010, UINT32_MAX, 0x02020202 },
	{ 0x15b00414, UINT32_MAX, 0x02020202 },
	{ 0x15b00814, UINT32_MAX, 0x02020202 },
	{ 0x15b00c14, UINT32_MAX, 0x02020202 },
	{ 0x15b01014, UINT32_MAX, 0x02020202 },
	{ 0x15b00440, UINT32_MAX, 0x02020202 },
	{ 0x15b00840, UINT32_MAX, 0x02020202 },
	{ 0x15b00c40, UINT32_MAX, 0x02020202 },
	{ 0x15b01040, UINT32_MAX, 0x02020202 },
	{ 0x15b00444, UINT32_MAX, 0x02020202 },
	{ 0x15b00844, UINT32_MAX, 0x02020202 },
	{ 0x15b00c44, UINT32_MAX, 0x02020202 },
	{ 0x15b01044, UINT32_MAX, 0x02020202 },

	/* IOHC */
	{ 0x13b14040, UINT32_MAX, 0x00000000 },
	{ 0x13b14044, 0x1f7f1f7f, 0x1030102f },

	/* IOAPIC bridge interrupt routing */
	{ 0x14300040, 0x001f0037, 0x00100000 },
	{ 0x14300044, 0x001f0037, 0x00110001 },
	{ 0x14300048, 0x001f0037, 0x00120002 },
	{ 0x1430004c, 0x001f0037, 0x00130003 },
	{ 0x14300050, 0x001f0037, 0x00100004 },
	{ 0x14300054, 0x001f0037, 0x00110005 },
	{ 0x14300058, 0x001f0037, 0x00120006 },
	{ 0x1430005c, 0x001f0037, 0x00130007 },
	{ 0x14300060, 0x001f0037, 0x000c0027 },

	/* NBIF interrupts */
	{ 0x1013a004, UINT32_MAX, 0x00000000 },
	{ 0x1013a008, UINT32_MAX, 0x000001fb },
	{ 0x1013a108, 0x00ffffff, 0x00080808 },

	/* Clock gating and power management of IOHC, IOAGR, IOAPIC, NBIF, SYSHUB, PCIe and IOMMU */
	{ 0x13b10088, 0xffc00000, 0x00000000 },
	{ 0x13b1008c, 0xffc00000, 0x02000000 },
	{ 0x15b00000, 0xffc00000, 0x00000000 },
	{ 0x15b00004, 0xe0000000, 0x00000000 },
	{ 0x14300100, 0xc0000000, 0x00000000 },
	{ 0x13b2009c, 0x00000018, 0x00000018 },
	{ 0x10123484, 0x00000010, 0x00000010 },
	{ 0x1013400c, 0x02000000, 0x00000000 },
	{ 0x1013420c, 0x02000000, 0x00000000 },
	{ 0x10134a34, 0x0000ff00, 0x00008000 },
	{ 0x101235cc, 0x00000400, 0x00000400 },
	{ 0x101313e4, 0x00000400, 0x00000400 },
	{ 0x1013100c, 0x20000000, 0x20000000 },
	{ 0x10131014, 0x00020000, 0x00020000 },
	{ 0x101234c8, 0xb8043e00, 0xb8042000 },
	{ 0x101234a4, 0x00000084, 0x00000084 },
	{ 0x101234c8, 0x00000100, 0x00000100 },
	{ 0x101234d8, 0x00000e00, 0x00000000 },
	{ 0x101234a8, 0x00000006, 0x00000006 },
	{ 0x1013a010, 0x10000000, 0x10000000 },
	{ 0x1012358c, 0x00000001, 0x00000001 },
	{ 0x101313ac, 0x00000001, 0x00000001 },
	{ 0x101234d0, 0x00400000, 0x00400000 },
	{ 0x101234fc, 0x00400000, 0x00400000 },
	{ 0x1013a05c, 0x000003fd, 0x00000101 },
	{ 0x01410310, 0x000003fd, 0x00000101 },
	{ 0x01411040, 0x000003fd, 0x00000101 },
	{ 0x17400404, 0x00000010, 0x00000010 },
	{ 0x17500404, 0x00000010, 0x00000010 },
	{ 0x11180070, 0xe0000000, 0xe0000000 },
	{ 0x11180074, 0x00004000, 0x00004000 },
	{ 0x11180070, 0x00d00000, 0x00d00000 },
	{ 0x11180074, 0x00002000, 0x00002000 },
	{ 0x147000d4, 0x00000003, 0x00000003 },
	{ 0x148000d4, 0x00000003, 0x00000003 },
	{ 0x15b00040, 0xffff8000, 0x00018000 },
	{ 0x1013a0e8, 0x00000800, 0x00000800 },
	{ 0x17400004, 0x0000ff00, 0x0000f000 },
	{ 0x17500004, 0x0000ff00, 0x0000f000 },
	{ 0x01410000, 0x00040010, 0x00040010 },
	{ 0x13f00084, 0x00080000, 0x00080000 },
	{ 0x1013101c, 0xffff0000, 0x28000000 },
	{ 0x1013121c, 0xffff0000, 0x48000000 },
	{ 0x1013a0e0, 0x000001ff, 0x00000150 },

	/* IOHC PCIe CRS count */
	{ 0x13b10028, 0x0fffffff, 0x00060006 },

	/* FCH */
	{ FCH_MISC_SMN(0xb0), UINT32_MAX, 0x00000000 },
	{ FCH_MISC_SMN(0xb4), UINT32_MAX, 0x00000000 },

	/* Poison handling */
	{ IOHC_POISON_ACTION_CONTROL, 0x00000019, 0x00000019 },
};

/* Hide the GFX and display audio functions and expose the dummy function instead */
static const struct nbio_reg nbio_gfx_disabled_table[] = {
	{ 0x101234bc, 0x10000000, 0x00000000 },
	{ 0x1012349c, 0x80000000, 0x80000000 },
	{ 0x101234e0, 0x10000000, 0x00000000 },
	{ 0x101234bc, 0x0000ffff, 0x0000145a },
	{ 0x101234c4, 0x00ffffff, 0x00130000 },
	{ 0x101234d0, 0x70000000, 0x00000000 },
	{ 0x1013400c, 0x00140000, 0x00000000 },
	{ 0x101234d0, 0x0f800000, 0x00000000 },
	{ 0x1013400c, 0x02000000, 0x02000000 },
	{ 0x101234d0, 0x00400000, 0x00000000 },
	{ 0x101234a8, 0x00000006, 0x00000000 },
	{ 0x101234c8, 0x00040000, 0x00000000 },
	{ 0x101234c8, 0x10000000, 0x00000000 },
	{ 0x101234c8, 0x08000000, 0x00000000 },
	{ 0x101234c8, 0x00000100, 0x00000000 },
	{ 0x101234a8, 0x20000000, 0x00000000 },
	{ 0x10131014, 0x00020000, 0x00000000 },
	{ 0x101234c8, 0x00030000, 0x00000000 },
};

/* IOMMU configuration and clock gating */
static const struct nbio_reg nbio_iommu_table[] = {
	{ 0x13f00050, 0x00400000, 0x00000000 },
	{ 0x147000c8, 0x00000005, 0x00000005 },
	{ 0x1470001c, 0x00020040, 0x00020040 },
	{ 0x1470009c, 0x00000006, 0x00000006 },
	{ 0x148000c8, 0x00000005, 0x00000005 },
	{ 0x1480001c, 0x00020040, 0x00020040 },
	{ 0x1480009c, 0x00000006, 0x00000006 },
	{ 0x13f01200, 0x00000001, 0x00000010 },
	{ 0x157000c0, 0x00000001, 0x00000001 },
	{ 0x157000d0, 0x00007f7f, 0x00004949 },
	{ 0x13f01250, 0x00007f7f, 0x00004949 },
	{ 0x13f01130, 0x0007f000, 0x00049000 },
	{ 0x13f0111c, 0x00000007, 0x00000007 },
	{ 0x13f01158, 0x00000006, 0x00000002 },
	{ 0x13f0114c, 0x00000001, 0x00000000 },
	{ 0x15700088, 0x00000001, 0x00000000 },
	{ 0x13f01150, 0x00007100, 0x00007100 },
	{ 0x157000cc, 0x00000007, 0x00000000 },
	{ 0x13f01240, 0x0000000f, 0x00000000 },
	{ 0x147000cc, 0x803fff70, 0x80083f30 },
	{ 0x148000cc, 0x803fff70, 0x80083f70 },
	{ 0x13f01150, 0x00007100, 0x00007100 },
	{ 0x1470001c, 0x00000400, 0x00002000 },
	{ 0x1480001c, 0x00000400, 0x00002000 },
	{ 0x14700034, 0x00000400, 0x00002000 },
	{ 0x13f01200, 0x000000ff, 0x00000001 },
};

/* Only for Raven2 silicon */
static const struct nbio_reg nbio_rv2_table[] = {
	{ 0x01403c54, 0x000f8000, 0x00040000 },
	{ 0x01403c50, 0x000f8000, 0x00040000 },
	{ 0x01403c4c, 0x0003ffff, 0x00004042 },
};

/* IOMMU settings applied after the SB location got set up */
static const struct nbio_reg nbio_iommu_late_table[] = {
	{ 0x157000d4, 0x00000007, 0x00000007 },
	{ 0x13f01254, 0x00000007, 0x00000007 },
	{ 0x15700104, 0x00000001, 0x00000001 },
	{ 0x157000d4, 0x00000010, 0x00000010 },
	{ 0x13f01254, 0x00000010, 0x00000010 },
	{ 0x147000cc, 0x80000000, 0x80000000 },
	{ 0x148000cc, 0x80000000, 0x80000000 },
};

static const struct nbio_reg nbio_iohc_late_table[] = {
	{ 0x13b10340, 0x00000100, 0x00000100 },
	{ 0x15b00080, 0x00000100, 0x00000100 },
	{ 0x13b10344, 0x00008000, 0x00008000 },
	{ 0x15b00040, 0x00008000, 0x00008000 },
};

static void apply_reg_table(const struct nbio_reg *table, size_t count)
{
	for (size_t i = 0; i < count; i++) {
		if (table[i].mask == UINT32_MAX)
			smn_write32(table[i].reg, table[i].value);
		else
			smn_clrsetbits32(table[i].reg, table[i].mask, table[i].value);
	}
}

static void program_smu_bar(void)
{
	const uint64_t bar = (uint64_t)smn_read32(SMU_BASE_ADDR_HI) << 32 |
			     smn_read32(SMU_BASE_ADDR_LO);

	if (!(bar & (IOHC_MMIO_EN | BIT(1)))) {
		smn_write32(SMU_BASE_ADDR_HI, 0);
		smn_write32(SMU_BASE_ADDR_LO, SMU_MMIO_BASE_ADDRESS | IOHC_MMIO_EN | BIT(1));
	} else if ((bar & 0xfffffff00000ULL) != SMU_MMIO_BASE_ADDRESS) {
		printk(BIOS_ERR, "Unexpected SMU MMIO BAR: %016llx\n", bar);
	}
}

/* SYSHUB deep sleep control */
#define SYSHUB_DS_CTRL_SOCCLK	0x01410000
#define SYSHUB_DS_CTRL_SHUBCLK	0x01411000

static void nbio_base_init(bool gfx_available)
{
	uint32_t sb_location;

	apply_reg_table(nbio_base_table, ARRAY_SIZE(nbio_base_table));
	if (!gfx_available)
		apply_reg_table(nbio_gfx_disabled_table, ARRAY_SIZE(nbio_gfx_disabled_table));

	/* The IOMMU needs to know where the FCH is */
	sb_location = smn_read32(SB_LOCATION);
	smn_write32(0x14800024, sb_location);	/* IOMMUL1 PCIE0 L1_SB_LOCATION */
	smn_write32(0x13f0112c, sb_location);	/* IOMMUL2 L2_SB_LOCATION */

	apply_reg_table(nbio_iommu_table, ARRAY_SIZE(nbio_iommu_table));
	if (soc_cpuid_is_raven2())
		apply_reg_table(nbio_rv2_table, ARRAY_SIZE(nbio_rv2_table));

	/* The ACP driver configures the audio pins later */
	smn_clrsetbits32(ACP_SMN_I2S_PIN_CONFIG, ACP_GEN1_I2S_PIN_CONFIG_MASK,
			I2S_PINS_UNCONF);

	smn_write32(0x15704330, smn_read32(0x13f00080));
	smn_write32(0x1570401c, smn_read32(0x0240001c));

	smn_clrsetbits32(IOHC_POISON_ACTION_CONTROL, 0x00171700, 0x00111100);

	apply_reg_table(nbio_iommu_late_table, ARRAY_SIZE(nbio_iommu_late_table));
	smn_write32(0x157000f8, 0x800);
	apply_reg_table(nbio_iohc_late_table, ARRAY_SIZE(nbio_iohc_late_table));

	program_smu_bar();

	/* SYSHUB deep sleep */
	smn_clrsetbits32(SYSHUB_DS_CTRL_SHUBCLK, 0, BIT(19));
	if ((smn_read32(SMU_FUSE_OPN) >> 30) == 3 || soc_cpuid_is_raven2())
		smn_clrsetbits32(SYSHUB_DS_CTRL_SHUBCLK, 0, BIT(18));
	if (get_socket_type() == SOCKET_AM4)
		smn_clrsetbits32(SYSHUB_DS_CTRL_SOCCLK, 0, BIT(20));
}

enum cb_err picasso_nbio_early_init(void)
{
	struct smu_payload payload = { 0 };
	struct nbio_capabilities *caps;

	if (send_smu_message(SMC_MSG_GET_SILICON_ID, &payload))
		return CB_ERR;

	caps = cbmem_add(CBMEM_ID_AMD_NBIO_CAPABILITIES, sizeof(*caps));
	if (!caps)
		return CB_ERR;

	caps->silicon_id = payload.msg[0];
	caps->gfx_available = !(caps->silicon_id & SILICON_ID_NO_GFX) &&
			      smn_read32(NBIF_GFX_STATUS) != 0;
	printk(BIOS_DEBUG, "Silicon ID 0x%08x, GFX %savailable\n", caps->silicon_id,
	       caps->gfx_available ? "" : "not ");

	nbio_base_init(caps->gfx_available);

	return CB_SUCCESS;
}

uint32_t picasso_native_silicon_id(void)
{
	const struct nbio_capabilities *caps = cbmem_find(CBMEM_ID_AMD_NBIO_CAPABILITIES);

	return caps ? caps->silicon_id : 0;
}

bool picasso_gfx_available(void)
{
	const struct nbio_capabilities *caps = cbmem_find(CBMEM_ID_AMD_NBIO_CAPABILITIES);

	if (!caps)
		die("NBIO capabilities are missing\n");

	return caps->gfx_available;
}

/* Tell the IOHC about the DRAM map and the MMIO above 4 GiB */
void picasso_nbio_tom_init(void)
{
	const uint64_t tom = msr_read(TOP_MEM_MSR);
	uint64_t tom2 = 0;
	uint32_t tom1_reg = smn_read32(NB_TOP_OF_DRAM_SLOT1);
	uint32_t tom2_low = smn_read32(NB_LOWER_TOP_OF_DRAM2);
	uint32_t tom2_high = smn_read32(NB_UPPER_TOP_OF_DRAM2);

	tom1_reg = (tom1_reg & 0x007fffff) | ((uint32_t)tom & 0xff800000);
	if (msr_read(SYSCFG_MSR) & SYSCFG_MSR_TOM2En)
		tom2 = msr_read(TOP_MEM2_MSR) & 0x000000ffffc00000ULL;
	if (tom2) {
		/* Bit 0 enables TOM2 */
		tom2_low = (tom2_low & 0x007ffffe) | ((uint32_t)tom2 & 0xff800000) | 1;
		tom2_high = (tom2_high & ~0xffu) | ((uint32_t)(tom2 >> 32) & 0xff);
	}

	smn_write32(NB_TOP_OF_DRAM_SLOT1, tom1_reg);
	smn_write32(NB_LOWER_TOP_OF_DRAM2, tom2_low);
	smn_write32(NB_UPPER_TOP_OF_DRAM2, tom2_high);
	smn_write32(NB_TOP_OF_DRAM3, 0x83ffffff);
	smn_write32(NB_MMIOBASE, 0x01000000);
	smn_write32(NB_MMIOLIMIT, 0x7fffffff);
	smn_clrsetbits32(NB_PCI_CTRL, 0, NB_PCI_CTRL_MMIO_ENABLE);

	printk(BIOS_DEBUG, "NBIO TOM: %08x TOM2: %08x:%08x\n", tom1_reg, tom2_high, tom2_low);
}

void picasso_nbio_ioapic_init(void)
{
	smn_write32(IOAPIC_BASE_ADDR_HI, 0);
	smn_write32(IOAPIC_BASE_ADDR_LO, GNB_IO_APIC_ADDR & 0xffffff00);
	smn_write32(IOAPIC_BASE_ADDR_LO, (GNB_IO_APIC_ADDR & 0xffffff00) | IOHC_MMIO_EN);
	smn_clrsetbits32(IOAPIC_FEATURES_ENABLE, 0, 0x14);
	smn_write32(NB_BUS_NUM_CNTL, 0x100);
}

static struct picasso_pcie_topology pcie_topology;

struct picasso_pcie_topology *picasso_pcie_topology(void)
{
	return &pcie_topology;
}

enum cb_err picasso_pcie_topology_save(void)
{
	struct picasso_pcie_topology *saved;

	saved = cbmem_add(CBMEM_ID_AMD_DXIO_TOPOLOGY, sizeof(*saved));
	if (!saved)
		return CB_ERR;

	pcie_topology.version = PICASSO_PCIE_TOPOLOGY_VERSION;
	pcie_topology.valid = true;
	*saved = pcie_topology;
	return CB_SUCCESS;
}

enum cb_err picasso_pcie_topology_restore(void)
{
	const struct picasso_pcie_topology *saved = cbmem_find(CBMEM_ID_AMD_DXIO_TOPOLOGY);

	if (!saved || saved->version != PICASSO_PCIE_TOPOLOGY_VERSION || !saved->valid)
		return CB_ERR;

	pcie_topology = *saved;
	return CB_SUCCESS;
}
