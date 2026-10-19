/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/smn.h>
#include <arch/io.h>
#include <cpu/x86/smm.h>
#include <soc/smi.h>
#include <types.h>

/*
 * GbtNmiBtn, FFS 204435f0-9025-472b-9b4c-4b232f549209, SMM.
 * F09 PE SHA256 (also byte-identical to F02):
 * 4a8331e8cbecda63fcb2218428f6ff0505164d724ed0695e61f37b784b4b4efe
 * Callback 0x510: POST E7, SEL, then the SMN sequence at 0x5c9..0x603.
 * Implement the CPU diagnostic interrupt, without the vendor SEL logging.
 */
void mainboard_smi_gpi(u32 gpi_sts)
{
	if (!(gpi_sts & BIT(SMITYPE_G_AGPIO7)))
		return;

	/* SMM does not link the x86 arch_post_code implementation. */
	if (CONFIG(POST_IO))
		outb(0xe7, CONFIG_POST_IO_PORT);
	smn_write32(0x13b2015c, smn_read32(0x13b2015c) | BIT(17));
	(void)smn_read32(0x13b10108);
	smn_write32(0x13b10108, 1);
}
