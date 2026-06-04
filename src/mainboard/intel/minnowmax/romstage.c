/* SPDX-License-Identifier: GPL-2.0-only */

#include <stdint.h>
#include <cbfs.h>
#include <console/console.h>
#include <device/dram/ddr3.h>
#include <device/mmio.h>
#include <soc/gpio.h>
#include <soc/mrc_wrapper.h>
#include <soc/romstage.h>

#define MINNOWMAX_SPD_COUNT 2
#define MINNOWMAX_SPD_1GB 0
#define MINNOWMAX_SPD_2GB 1

/* Bay Trail GPSSUS GPIO-to-pad mapping, from src/soc/intel/baytrail/gpio.c. */
#define GPIO_S5_4_PAD 32
#define GPIO_S5_5_PAD 34
#define GPIO_S5_6_PAD 36
#define GPIO_S5_7_PAD 35

static void ssus_gpio_input(int pad)
{
	ssus_select_func(pad, PAD_FUNC0);
	ssus_disable_internal_pull(pad);
	write32(ssus_pconf0(pad) + (PAD_VAL_REG / sizeof(uint32_t)), PAD_VAL_INPUT);
}

static int read_bom_id(void)
{
	int id = 0;

	ssus_gpio_input(GPIO_S5_4_PAD);
	ssus_gpio_input(GPIO_S5_5_PAD);
	ssus_gpio_input(GPIO_S5_6_PAD);
	ssus_gpio_input(GPIO_S5_7_PAD);

	id |= ssus_get_gpio(GPIO_S5_5_PAD) ? 1 << 0 : 0;
	id |= ssus_get_gpio(GPIO_S5_6_PAD) ? 1 << 1 : 0;
	id |= ssus_get_gpio(GPIO_S5_7_PAD) ? 1 << 2 : 0;
	id |= ssus_get_gpio(GPIO_S5_4_PAD) ? 1 << 3 : 0;

	return id;
}

void mainboard_fill_mrc_params(struct mrc_params *mp)
{
	char *spd_file;
	size_t spd_fsize;
	int bom_id = read_bom_id();
	int spd_index = (bom_id & 1) ? MINNOWMAX_SPD_2GB : MINNOWMAX_SPD_1GB;

	spd_file = cbfs_map("spd.bin", &spd_fsize);
	if (!spd_file)
		die("SPD data not found.");

	if (spd_fsize < MINNOWMAX_SPD_COUNT * SPD_SIZE_MAX_DDR3)
		die("SPD data too small.");

	printk(BIOS_NOTICE, "MinnowBoard Max BOM ID %#x: using fake %dGB SPD.\n",
	       bom_id, spd_index ? 2 : 1);

	mp->mainboard.dram_type = DRAM_DDR3L;
	mp->mainboard.dram_info_location = DRAM_INFO_SPD_MEM;
	mp->mainboard.weaker_odt_settings = 1;

	mp->mainboard.dram_data[0] = spd_file + SPD_SIZE_MAX_DDR3 * spd_index;
}
