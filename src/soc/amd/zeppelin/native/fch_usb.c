/* SPDX-License-Identifier: GPL-2.0-only */

/*
 * The xHCI controller of die 0 (bus behind 0:7.1, function 3). The vendor
 * FchTaishanPei reset task af48 programs it through SMN; Zeppelin has no SMU
 * USB messages (sp4r2 report FCH_TAISHAN, cold path, vendor defaults).
 */

#include <amdblocks/smn.h>
#include <console/console.h>
#include <delay.h>
#include <soc/soc_util.h>
#include <timer.h>
#include <types.h>

#include "fch.h"

#define USB_CTRL_BASE		0x16d80000
#define USB_014			(USB_CTRL_BASE + 0x014)
#define USB_RESET_CTRL		(USB_CTRL_BASE + 0x100)
#define USB_STATUS_108		(USB_CTRL_BASE + 0x108)
#define USB_118			(USB_CTRL_BASE + 0x118)
#define USB_PORT_DISABLE	(USB_CTRL_BASE + 0x128)
#define USB_PORT_130		(USB_CTRL_BASE + 0x130)
#define USB_14C			(USB_CTRL_BASE + 0x14c)

#define USB_PHY_COUNT		4
#define USB3_PHY(n, reg)	(0x16d04000 + (n) * 0x400 + (reg))
#define USB_PHY(n, reg)		(0x16d08000 + (n) * 0x400 + (reg))
#define USB2_PHY(n, reg)	(0x16d0c000 + (n) * 0x400 + (reg))
#define USB_PHY_PARAM		0x01b3c953	/* vendor template, die 0 */

/* USB3 PHY tuning: vendor CBS defaults, the same for every PHY */
#define USB3_VBOOST		2
#define USB3_LOS_BIAS		5
#define USB3_DEEMPH_3P5		0x1c
#define USB3_DEEMPH_6		0x1c
#define USB3_SWING		0x7f

static void usb_wait_108(void)
{
	struct stopwatch sw;

	stopwatch_init_usecs_expire(&sw, 1000);
	while ((smn_read32(USB_STATUS_108) & 0x300) && !stopwatch_expired(&sw))
		udelay(1);
}

static void usb_phy_init(void)
{
	for (unsigned int n = 0; n < USB_PHY_COUNT; n++) {
		smn_write32(USB_PHY(n, 0x200), 0x39);
		smn_write32(USB2_PHY(n, 0x0), USB_PHY_PARAM);
		smn_clrsetbits32(USB_PHY(n, 0x8), 0x380, 0x100);
	}
	smn_clrsetbits32(USB_118, BIT(12), 0);	/* vendor PCD 0x97 */
}

static void usb_post_reset(void)
{
	smn_write32(0x16c0c11c, 0x2004010a);
	smn_write32(0x16c0c12c, 0x0a43487d);
	for (uint32_t reg = 0x16c0c2c0; reg <= 0x16c0c2cc; reg += 4)
		smn_write32(reg, 0x010a0002);

	for (unsigned int n = 0; n < USB_PHY_COUNT; n++) {
		smn_clrsetbits32(USB_PHY(n, 0x4), 0x3ffc00e0,
				 USB3_LOS_BIAS << 5 | USB3_DEEMPH_3P5 << 18 | USB3_DEEMPH_6 << 24);
		smn_clrsetbits32(USB_PHY(n, 0x8), 0x3ff, USB3_SWING | USB3_VBOOST << 7);
		/*
		 * The vendor code first writes the rx_eq tuning value here and then
		 * overwrites the field with 0xb80, which leaves rx_eq at 3.
		 */
		smn_clrsetbits32(USB3_PHY(n, 0x18), 0xfc0, 0xb80);
		smn_write32(USB3_PHY(n, 0x98), 1);
	}
}

void zeppelin_fch_usb_init(void)
{
	smn_clrsetbits32(USB_14C, 0, BIT(31));

	/* Cold path */
	smn_write32(USB_014, 0);
	usb_wait_108();
	smn_clrsetbits32(USB_RESET_CTRL, 0, BIT(0));
	smn_clrsetbits32(USB_RESET_CTRL, 0, BIT(1) | BIT(2));
	udelay(100);

	/* SP4 has only two of the four ports; SP4r2 keeps the reset values */
	if (zeppelin_package() == ZEPPELIN_PKG_SP4) {
		smn_write32(USB_PORT_130, 0x22);
		smn_write32(USB_PORT_DISABLE, 0x000c000c);
	}

	usb_phy_init();

	smn_clrsetbits32(USB_RESET_CTRL, BIT(0), 0);
	smn_clrsetbits32(USB_RESET_CTRL, BIT(2), 0);
	udelay(200);
	smn_clrsetbits32(USB_RESET_CTRL, BIT(1), 0);

	usb_post_reset();
}
