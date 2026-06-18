/* SPDX-License-Identifier: GPL-2.0-or-later */

#include <types.h>
#include <console/console.h>
#include <device/smbus_host.h>
#include <northbridge/intel/pineview/pineview.h>

#define CLOCKGEN_ADDR	0x69
#define CLOCKGEN_NREGS	5

void mainboard_pre_raminit(void)
{
	u8 block[CLOCKGEN_NREGS];
	int ret;

	/* Match the vendor pre-ram CK505/clockgen setup. */
	ret = smbus_block_read(CLOCKGEN_ADDR, 0, sizeof(block), block);
	if (ret < 0) {
		printk(BIOS_ERR, "Failed reading clockgen configuration\n");
		return;
	}

	block[1] |= 0x80;
	block[2] = 0xfe;
	block[3] = 0xff;
	block[4] = 0xfc;

	ret = smbus_block_write(CLOCKGEN_ADDR, 0, sizeof(block), block);
	if (ret < 0)
		printk(BIOS_ERR, "Failed writing clockgen configuration\n");
}
