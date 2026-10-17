/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_ZEPPELIN_NATIVE_NBIO_H
#define AMD_ZEPPELIN_NATIVE_NBIO_H

#include <amdblocks/root_complex.h>
#include <amdblocks/smn.h>
#include <soc/platform_descriptors.h>
#include <types.h>

/* NB configuration space (SMN), also D0F0 of the die */
#define NBCFG_BASE			0x13b00000
#define NB_PCI_CTRL			(NBCFG_BASE + 0x4c)
#define  NB_PCI_CTRL_MMIO_ENABLE	BIT(23)
#define NB_TOP_OF_DRAM_SLOT1		0x90	/* D0F0 */

/* IOHC (SMN) */
#define IOHC_BASE			SMN_IOHC_MISC_BASE_13B1
#define NB_BUS_NUM_CNTL			(IOHC_BASE + 0x044)
#define  NB_BUS_NUM_CNTL_LAT_MODE	BIT(8)
#define NB_MMIOBASE			(IOHC_BASE + 0x05c)
#define NB_MMIOLIMIT			(IOHC_BASE + 0x060)
#define NB_LOWER_TOP_OF_DRAM2		(IOHC_BASE + 0x064)
#define NB_UPPER_TOP_OF_DRAM2		(IOHC_BASE + 0x068)
#define SB_LOCATION			(IOHC_BASE + 0x07c)
#define NB_PROG_DEVICE_REMAP(port)	(IOHC_BASE + 0x0c0 + (port) * 4)
#define IOHC_FEATURE_CNTL		(IOHC_BASE + 0x118)
#define  IOHC_FEATURE_CNTL_BIT22	BIT(22)
#define NB_TOP_OF_DRAM3			(IOHC_BASE + 0x138)
/* Bus routing between the root bridges of the dies */
#define IOHC_RB_BUS_ROUTE(rb)		(IOHC_BASE + 0x190 + (rb) * 0x14)
#define PSP_BASE_ADDR_LO		(IOHC_BASE + IOHC_MISC_PSP_MMIO_REG)
#define PSP_BASE_ADDR_HI		(PSP_BASE_ADDR_LO + 4)
#define IOAPIC_BASE_ADDR_LO		(IOHC_BASE + IOHC_MISC_IOAPIC_MMIO_REG)
#define IOAPIC_BASE_ADDR_HI		(IOAPIC_BASE_ADDR_LO + 4)
/* The 16 PCIe bridges of a die each have a bridge control register */
#define IOHC_BRIDGE_CNTL(bridge)	(0x13b31004 + (bridge) * 0x400)
#define  IOHC_BRIDGE_CNTL_HIDE		0x00040005
#define  IOHC_BRIDGE_CNTL_MASK		0x00040005
#define  IOHC_BRIDGE_CNTL_ENABLE	0x00040000

#define IOAPIC_FEATURES_ENABLE		0x14300000

/*
 * PCIe core c of a die (c = 0: lanes 0-15, devices 1.x and 2.1; c = 1: lanes 16-31,
 * devices 3.x and 4.1). Port p (0-15) belongs to core p / 8. The register offsets are
 * the index register numbers * 4.
 */
#define PCIE_CORE_COUNT			2
#define PCIE_PORTS_PER_CORE		8
#define PCIE_PORT_COUNT			(PCIE_CORE_COUNT * PCIE_PORTS_PER_CORE)
#define PCIE_CORE_STRIDE		0x100000

#define PCIE_CORE_BASE			0x11180000
#define PCIE_CNTL			0x040
#define PCIE_CONFIG_CNTL		0x044
#define PCIE_P_CNTL			0x100
#define PCIE_CNTL_18C			0x18c
#define PCIE_STRAP_F0			0x2c0
#define PCIE_LINK_STATUS		0x428	/* bit per port 1-7 of the core */
#define CPM_CONTROL			0x460

#define PCIE_PORT_BASE			0x11140000
#define PCIE_PORT_STRIDE		0x1000
#define PCIE_PORT_008			0x008
#define PCIE_TX_CNTL			0x080
#define PCIE_PORT_284			0x284
#define PCIE_LC_CNTL			0x280
#define PCIE_LC_LINK_WIDTH_CNTL		0x288
#define PCIE_LC_STATE0			0x294
#define PCIE_LC_PORT_2DC		0x2dc	/* bit 28: port 0 of a core not trained */
#define PCIE_LC_CNTL2			0x2c4
#define PCIE_LC_CNTL4			0x2d8
#define PCIE_LC_CNTL6			0x2ec
#define PCIE_LC_CNTL7			0x2f0
#define PCIE_PORT_304			0x304

/* The config space of the PCIe root ports is also accessible through SMN */
#define PCIE_PORT_CFG_BASE		0x11100000

/* Indirect PCIe strap access (SMN): core 0 and core 1 */
#define PCIE_STRAP_INDEX(core)		(0x0004a348 + (core) * 0x80)
#define PCIE_STRAP_DATA(core)		(0x0004a34c + (core) * 0x80)

/* NBIF (SMN): there are three NBIF instances 0x101xxxxx, 0x102xxxxx, 0x103xxxxx */
#define NBIF_COUNT			3
#define NBIF_REG(n, offset)		(0x10100000 + (n) * 0x100000 + (offset))

static inline unsigned int pcie_port_core(unsigned int port)
{
	return port / PCIE_PORTS_PER_CORE;
}

static inline uint32_t pcie_core_reg(unsigned int core, uint32_t offset)
{
	return PCIE_CORE_BASE + core * PCIE_CORE_STRIDE + offset;
}

static inline uint32_t pcie_port_reg(unsigned int port, uint32_t offset)
{
	return PCIE_PORT_BASE + pcie_port_core(port) * PCIE_CORE_STRIDE +
		(port % PCIE_PORTS_PER_CORE) * PCIE_PORT_STRIDE + offset;
}

static inline uint32_t pcie_port_cfg(unsigned int port, uint32_t offset)
{
	return PCIE_PORT_CFG_BASE + pcie_port_core(port) * PCIE_CORE_STRIDE +
		(port % PCIE_PORTS_PER_CORE) * PCIE_PORT_STRIDE + offset;
}

/* A PCIe root port of a die and the state that the DXIO training reported for it */
struct zeppelin_pcie_port {
	uint8_t start_lane;	/* die lanes */
	uint8_t end_lane;
	uint8_t device;
	uint8_t function;
	uint8_t link_speed;
	uint8_t hotplug;
	bool allocated;
	bool trained;
};

struct zeppelin_pcie_topology {
	struct zeppelin_pcie_port ports[PCIE_PORT_COUNT];
};

/* nbio.c */
void zeppelin_nbio_early_init(void);
void zeppelin_nbio_tom_init(void);
void zeppelin_nbio_ioapic_init(void);
struct zeppelin_pcie_topology *zeppelin_pcie_topology(unsigned int die);

/* dxio.c */
enum cb_err zeppelin_dxio_init(void);

/* nbio_late.c */
void zeppelin_nbio_pre_enumeration_init(void);

#endif /* AMD_ZEPPELIN_NATIVE_NBIO_H */
