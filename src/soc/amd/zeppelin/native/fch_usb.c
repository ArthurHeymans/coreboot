/* SPDX-License-Identifier: GPL-2.0-only */

/*
 * The xHCI controller of a die (behind 7.1 of the die's root bus, function 3).
 * The vendor FchTaishanPei reset task af48 programs it through SMN; Zeppelin
 * has no SMU USB messages (sp4r2 report FCH_TAISHAN, cold path, vendor defaults).
 */

#include <console/console.h>
#include <delay.h>
#include <soc/soc_util.h>
#include <timer.h>
#include <types.h>

#include "fch.h"
#include "topology.h"

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
/* USB2 PHY parameter of the vendor template, per die */
static const uint32_t usb_phy_param[ZEPPELIN_MAX_DIES] = { 0x01b3c953, 0x01b3cd56 };

/* USB3 PHY tuning: vendor CBS defaults, the same for every PHY */
#define USB3_VBOOST		2
#define USB3_LOS_BIAS		5
#define USB3_DEEMPH_3P5		0x1c
#define USB3_DEEMPH_6		0x1c
#define USB3_SWING		0x7f

static void usb_wait_108(unsigned int die)
{
	struct stopwatch sw;

	stopwatch_init_usecs_expire(&sw, 1000);
	while ((zeppelin_smn_read32(die, USB_STATUS_108) & 0x300) && !stopwatch_expired(&sw))
		udelay(1);
}

static void usb_phy_init(unsigned int die)
{
	for (unsigned int n = 0; n < USB_PHY_COUNT; n++) {
		zeppelin_smn_write32(die, USB_PHY(n, 0x200), 0x39);
		zeppelin_smn_write32(die, USB2_PHY(n, 0x0), usb_phy_param[die]);
		zeppelin_smn_clrsetbits32(die, USB_PHY(n, 0x8), 0x380, 0x100);
	}
	zeppelin_smn_clrsetbits32(die, USB_118, BIT(12), 0);	/* vendor PCD 0x97 */
}

static void usb_post_reset(unsigned int die)
{
	zeppelin_smn_write32(die, 0x16c0c11c, 0x2004010a);
	zeppelin_smn_write32(die, 0x16c0c12c, 0x0a43487d);
	for (uint32_t reg = 0x16c0c2c0; reg <= 0x16c0c2cc; reg += 4)
		zeppelin_smn_write32(die, reg, 0x010a0002);

	for (unsigned int n = 0; n < USB_PHY_COUNT; n++) {
		zeppelin_smn_clrsetbits32(die, USB_PHY(n, 0x4), 0x3ffc00e0,
					  USB3_LOS_BIAS << 5 | USB3_DEEMPH_3P5 << 18 |
					  USB3_DEEMPH_6 << 24);
		zeppelin_smn_clrsetbits32(die, USB_PHY(n, 0x8), 0x3ff,
					  USB3_SWING | USB3_VBOOST << 7);
		/*
		 * The vendor code first writes the rx_eq tuning value here and then
		 * overwrites the field with 0xb80, which leaves rx_eq at 3.
		 */
		zeppelin_smn_clrsetbits32(die, USB3_PHY(n, 0x18), 0xfc0, 0xb80);
		zeppelin_smn_write32(die, USB3_PHY(n, 0x98), 1);
	}
}

void zeppelin_fch_usb_init(unsigned int die)
{
	zeppelin_smn_clrsetbits32(die, USB_14C, 0, BIT(31));

	/* Cold path */
	zeppelin_smn_write32(die, USB_014, 0);
	usb_wait_108(die);
	zeppelin_smn_clrsetbits32(die, USB_RESET_CTRL, 0, BIT(0));
	zeppelin_smn_clrsetbits32(die, USB_RESET_CTRL, 0, BIT(1) | BIT(2));
	udelay(100);

	/* SP4 has only two of the four ports; SP4r2 keeps the reset values */
	if (zeppelin_package() == ZEPPELIN_PKG_SP4) {
		zeppelin_smn_write32(die, USB_PORT_130, 0x22);
		zeppelin_smn_write32(die, USB_PORT_DISABLE, 0x000c000c);
	}

	usb_phy_init(die);

	zeppelin_smn_clrsetbits32(die, USB_RESET_CTRL, BIT(0), 0);
	zeppelin_smn_clrsetbits32(die, USB_RESET_CTRL, BIT(2), 0);
	udelay(200);
	zeppelin_smn_clrsetbits32(die, USB_RESET_CTRL, BIT(1), 0);

	usb_post_reset(die);
}
