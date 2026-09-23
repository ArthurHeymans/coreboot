/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_PICASSO_NATIVE_NBIO_H
#define AMD_PICASSO_NATIVE_NBIO_H

#include <amdblocks/acp.h>
#include <amdblocks/root_complex.h>
#include <amdblocks/smn.h>
#include <soc/platform_descriptors.h>
#include <types.h>

#include "pcie.h"

/* NB configuration space (SMN) */
#define NBCFG_BASE			0x13b00000
#define NB_PCI_CTRL			(NBCFG_BASE + 0x4c)
#define  NB_PCI_CTRL_PMEDIS		BIT(4)
#define  NB_PCI_CTRL_MMIO_ENABLE	BIT(23)
#define NB_PCI_ARB			(NBCFG_BASE + 0x84)
#define NB_TOP_OF_DRAM_SLOT1		(NBCFG_BASE + 0x90)

/* IOHC (SMN) */
#define IOHC_BASE			SMN_IOHC_MISC_BASE_13B1
#define NB_BUS_NUM_CNTL			(IOHC_BASE + 0x044)
#define NB_MMIOBASE			(IOHC_BASE + 0x05c)
#define NB_MMIOLIMIT			(IOHC_BASE + 0x060)
#define NB_LOWER_TOP_OF_DRAM2		(IOHC_BASE + 0x064)
#define NB_UPPER_TOP_OF_DRAM2		(IOHC_BASE + 0x068)
#define SB_LOCATION			(IOHC_BASE + 0x07c)
#define NB_PROG_DEVICE_REMAP(port)	(IOHC_BASE + 0x0c0 + (port) * 4)
#define NB_TOP_OF_DRAM3			(IOHC_BASE + 0x138)
#define PSP_BASE_ADDR_LO		(IOHC_BASE + IOHC_MISC_PSP_MMIO_REG)
#define PSP_BASE_ADDR_HI		(PSP_BASE_ADDR_LO + 4)
#define SMU_BASE_ADDR_LO		(IOHC_BASE + IOHC_MISC_SMU_MMIO_REG)
#define SMU_BASE_ADDR_HI		(SMU_BASE_ADDR_LO + 4)
#define IOAPIC_BASE_ADDR_LO		(IOHC_BASE + IOHC_MISC_IOAPIC_MMIO_REG)
#define IOAPIC_BASE_ADDR_HI		(IOAPIC_BASE_ADDR_LO + 4)
#define IOHC_POISON_ACTION_CONTROL	0x13b20400
/* The GPP bridges 0..6 and the internal bridges each have a bridge control register */
#define IOHC_BRIDGE_CNTL(bridge)	(0x13b31004 + (bridge) * 0x400)
#define  IOHC_BRIDGE_CNTL_ENABLE_MASK	0x00040005
#define IOHC_BRIDGE_CNTL_NBIF1		IOHC_BRIDGE_CNTL(29)

#define IOAPIC_FEATURES_ENABLE		0x14300000

/* NBIF (SMN) */
#define NBIF_GFX_COMMAND		0x10100004	/* alias of the GFX PCI command register */
#define NBIF_GFX_AUDIO_COMMAND		0x10101004
/* Straps of the functions of the internal bus */
#define NBIF_FUNC_STRAP(n)		(0x10134000 + (n) * 0x200)
#define  NBIF_FUNC_STRAP_FUNC_EN	BIT(28)
#define NBIF_FUNC_ACP			5
#define NBIF_FUNC_HDA			6
#define NBIF_INTR_LINE_ENABLE		0x1013a008	/* one bit per function */
#define NBIF_GFX_STATUS			0x0004b600

/* ACP (SMN) */
#define ACP_SMN_BASE			0x01240000
#define ACP_SOFT_RESET			(ACP_SMN_BASE + 0x1000)
#define  ACP_SOFT_RESET_AUDIO		BIT(0)
#define  ACP_SOFT_RESET_AUDIO_DONE	BIT(16)
#define ACP_CONTROL			(ACP_SMN_BASE + 0x1004)
#define  ACP_CLOCK_ENABLE		BIT(0)
#define ACP_DYNAMIC_CG_MASTER_CONTROL	(ACP_SMN_BASE + 0x1010)
#define ACP_SMN_I2S_PIN_CONFIG		(ACP_SMN_BASE + ACP_GEN1_I2S_PIN_CONFIG)
#define ACP_PGFSM_CONTROL		(ACP_SMN_BASE + 0x141c)
#define  ACP_PGFSM_POWER_ON		BIT(0)
#define ACP_PGFSM_STATUS		(ACP_SMN_BASE + 0x1420)
#define  ACP_PGFSM_STATUS_MASK		0x3
#define ACP_PGMEM_CTRL			(ACP_SMN_BASE + 0x18c0)
#define  ACP_PGMEM_SHUTDOWN_MASK	0x0fff0000
#define ACP_MEM_PG_STS			(ACP_SMN_BASE + 0x18cc)
#define  ACP_MEM_PG_STS_MASK		0x00000fff

/* PCIe core 0 (SMN). The register offsets are the PCIe core index register numbers * 4. */
#define PCIE_CORE_BASE			0x11180000
#define PCIE_CNTL			0x040
#define PCIE_CONFIG_CNTL		0x044
#define PCIE_CLKREQ_MAP			0x0f4
#define PCIE_P_CNTL			0x100
#define PCIE_STRAP_F0			0x2c0
#define PCIE_TRAINING_INHIBIT		0x428
#define CPM_CONTROL			0x460

/*
 * PCIe root port registers (SMN). The register offsets are the PCIe port index register
 * numbers * 4.
 */
#define PCIE_PORT_BASE			0x11140000
#define PCIE_PORT_STRIDE		0x1000
#define PCIE_TX_CNTL			0x080
#define PCIE_TX_REQUESTER_ID		0x084
#define PCIE_ERR_CNTL			0x1a8
#define  ERR_REPORTING_DIS		BIT(0)
#define PCIE_LC_CNTL			0x280
#define PCIE_LC_LINK_WIDTH_CNTL		0x288
#define PCIE_LC_STATE0			0x294
#define PCIE_LC_CNTL2			0x2c4
#define PCIE_LC_CNTL4			0x2d8
#define PCIE_LC_FORCE_COEFF		0x2e0
#define PCIE_LC_CNTL6			0x2ec
#define  PCIE_LC_CNTL6_L1_ALLOW		BIT(21)

/* The config space of the PCIe root ports is also accessible through SMN */
#define PCIE_PORT_CFG_BASE		0x11100000
#define  PCIE_PORT_L1SS_CTL1		0x378

/* Indirect PCIe PHY/strap access (SMN) */
#define PCIE_PHY_INDEX			0x0004a34c
#define PCIE_PHY_DATA			0x0004a350

/* A PCIe root port of the NBIO and the state that the DXIO training reported for it */
struct picasso_pcie_port {
	uint8_t port;		/* hardware port number */
	uint8_t default_devfn;
	uint8_t start_lane;
	uint8_t end_lane;
	uint8_t device;
	uint8_t function;
	uint8_t link_speed;
	uint8_t aspm;
	uint8_t hotplug;
	uint8_t clk_req;
	uint8_t devfn_index;	/* index of the BDF remap register */
	uint16_t lane_preset;
	bool allocated;
	bool trained;
	bool l1_1;
	bool l1_2;
};

/* PCIe port state that is kept in CBMEM for S3 resume */
#define PICASSO_PCIE_TOPOLOGY_VERSION	2
struct picasso_pcie_topology {
	uint32_t version;
	struct picasso_pcie_port ports[PCIE_PORT_COUNT];
	bool valid;
};

static inline uint32_t pcie_port_reg(unsigned int port, uint32_t offset)
{
	return PCIE_PORT_BASE + port * PCIE_PORT_STRIDE + offset;
}

static inline uint32_t pcie_port_cfg(unsigned int port, uint32_t offset)
{
	return PCIE_PORT_CFG_BASE + port * PCIE_PORT_STRIDE + offset;
}

static inline uint32_t pcie_core_reg(uint32_t offset)
{
	return PCIE_CORE_BASE + offset;
}

/* nbio.c */
enum cb_err picasso_nbio_early_init(void);
void picasso_nbio_tom_init(void);
bool picasso_gfx_available(void);
void picasso_nbio_ioapic_init(void);
struct picasso_pcie_topology *picasso_pcie_topology(void);
enum cb_err picasso_pcie_topology_save(void);
enum cb_err picasso_pcie_topology_restore(void);

/* dxio.c */
enum cb_err picasso_dxio_init(void);

/* nbio_late.c */
enum cb_err picasso_nbio_pre_enumeration_init(void);
enum cb_err picasso_nbio_resume_init(void);
enum cb_err picasso_nbio_ccx_done(void);
enum cb_err picasso_nbio_post_silicon_init(void);

/* acp.c */
enum cb_err picasso_acp_power_off(void);

/* gfx.c */
enum cb_err picasso_gfx_init(void);

#endif /* AMD_PICASSO_NATIVE_NBIO_H */
