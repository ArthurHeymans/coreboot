/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/lpc.h>
#include <bootblock_common.h>
#include <device/pnp_ops.h>
#include <superio/aspeed/ast2400/ast2400.h>
#include <superio/aspeed/common/aspeed.h>
#include <types.h>

#define AST_SIO_PORT		0x2e
#define AST_ILPC2AHB_LDN	0x0d
#define AST_SCU_BASE		0x1e6e2000
#define  SCU_PROTECTION_KEY	0x1688a8a8
#define AST_SCU84		(AST_SCU_BASE + 0x84)	/* multi-function pin control 1 */
#define AST_SCU90		(AST_SCU_BASE + 0x90)	/* multi-function pin control 5 */

#define SERIAL_DEV PNP_DEV(AST_SIO_PORT, AST2400_SUART1)

static void sio_write(uint8_t reg, uint8_t value)
{
	outb(reg, AST_SIO_PORT);
	outb(value, AST_SIO_PORT + 1);
}

static uint8_t sio_read(uint8_t reg)
{
	outb(reg, AST_SIO_PORT);
	return inb(AST_SIO_PORT + 1);
}

/*
 * Read-modify-write a BMC register through the iLPC2AHB bridge (SIO LDN 0xd),
 * the same way the vendor AST2500 PEIM does it.
 */
static void ilpc2ahb_rmw32(uint32_t addr, uint32_t and, uint32_t or)
{
	uint32_t value = 0;
	int i;

	for (i = 0; i < 4; i++)
		sio_write(0xf0 + i, addr >> (24 - 8 * i));
	sio_write(0xf8, (sio_read(0xf8) & ~0x3) | 0x2);	/* 32-bit access */
	sio_read(0xfe);					/* read trigger */
	for (i = 0; i < 4; i++)
		value = value << 8 | sio_read(0xf4 + i);

	value = (value & and) | or;

	for (i = 0; i < 4; i++)
		sio_write(0xf4 + i, value >> (24 - 8 * i));
	sio_write(0xfe, 0xcf);				/* write trigger */
}

static void ast2500_route_uart1(void)
{
	outb(0xa5, AST_SIO_PORT);
	outb(0xa5, AST_SIO_PORT);
	sio_write(0x07, AST_ILPC2AHB_LDN);
	sio_write(0x30, sio_read(0x30) | 1);

	ilpc2ahb_rmw32(AST_SCU_BASE, 0, SCU_PROTECTION_KEY);
	/* Bits 23:16: UART1 pins (TXD1, RXD1 and the modem lines) */
	ilpc2ahb_rmw32(AST_SCU84, 0xff00ffff, 0x00ff0000);
	/* The vendor firmware clears bits 5:4 as well (meaning not known) */
	ilpc2ahb_rmw32(AST_SCU90, 0xffffffcf, 0);

	outb(0xaa, AST_SIO_PORT);
}

void bootblock_mainboard_early_init(void)
{
	lpc_enable_sio_decode(LPC_SELECT_SIO_2E2F);
	lpc_enable_decode(DECODE_ENABLE_SERIAL_PORT0);
	/* AST2500 SIO runtime registers (0xa00-0xbff) and the IPMI KCS interface */
	lpc_set_wideio_range(0xa00, 512);
	lpc_set_wideio_range(0xca0, 16);

	ast2500_route_uart1();
	aspeed_enable_serial(SERIAL_DEV, CONFIG_TTYS0_BASE);
}
