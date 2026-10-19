/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/acpi.h>
#include <amdblocks/acpimmio.h>
#include <amdblocks/acpimmio_map.h>
#include <amdblocks/amd_pci_util.h>
#include <amdblocks/smi.h>
#include <amdblocks/smn.h>
#include <bootstate.h>
#include <console/console.h>
#include <delay.h>
#include <device/device.h>
#include <device/mmio.h>
#include <device/pci_def.h>
#include <device/pci_ops.h>
#include <drivers/ipmi/ipmi_if.h>
#include <drivers/ipmi/ipmi_ops.h>
#include <soc/amd_pci_int_defs.h>
#include <soc/native.h>
#include <soc/smi.h>
#include <soc/southbridge.h>
#include <types.h>

#include "board_tables.h"
#include "mainboard.h"

/*
 * The FCH PCI interrupt routing at 0xc00/0xc01. The IOAPIC side matches the
 * vendor firmware; the vendor PIC values aren't known, so INTA-D are left for
 * the OS to route through the ACPI link devices.
 */
static struct fch_irq_routing fch_irq_map[] = {
	{ PIRQ_A,	PIRQ_NC,	16 },
	{ PIRQ_B,	PIRQ_NC,	17 },
	{ PIRQ_C,	PIRQ_NC,	18 },
	{ PIRQ_D,	PIRQ_NC,	19 },
	{ PIRQ_SCI,	ACPI_SCI_IRQ,	ACPI_SCI_IRQ },
	{ PIRQ_GPIO,	7,		7 },
	{ PIRQ_MISC,	0,		0 },
	{ PIRQ_MISC0,	0,		0 },
	{ PIRQ_MISC1,	0,		0 },
	{ PIRQ_MISC2,	0,		0 },
};

const struct fch_irq_routing *mb_get_fch_irq_mapping(size_t *length)
{
	/*
	 * These are control bitfields, not IRQ numbers. Preserve the native FCH
	 * initialization instead of letting the routing code fill them with
	 * PIRQ_NC, which disconnects the legacy timer interrupt.
	 */
	for (size_t i = 0; i < ARRAY_SIZE(fch_irq_map); i++) {
		const uint8_t index = fch_irq_map[i].intr_index;

		if (index < PIRQ_MISC || index > PIRQ_MISC2)
			continue;
		fch_irq_map[i].pic_irq_num = read_pci_int_idx(index, 0);
		fch_irq_map[i].apic_irq_num = read_pci_int_idx(index, 1);
	}

	*length = ARRAY_SIZE(fch_irq_map);
	return fch_irq_map;
}

#define GPIO_BMC_READY		149	/* low when the BMC is up */
#define BMC_READY_POLLS		240	/* 500 ms each, the vendor "2 minutes" default */

/*
 * GbtVgaInitial waits for the BMC before it uses the AST2500 VGA: GPIO149
 * debounce byte 5, IOMUX function 1, then poll the pin state.
 */
static void wait_for_bmc(void)
{
	write8p(MJ11_GPIO_BANK(GPIO_BMC_READY), 5);
	mj11_iomux_write(GPIO_BMC_READY, 1);

	for (int i = 0; i < BMC_READY_POLLS; i++) {
		if (!(read32p(MJ11_GPIO_BANK(GPIO_BMC_READY)) & MJ11_GPIO_PIN_STS))
			return;
		if (!i)
			printk(BIOS_INFO, "Waiting for the BMC\n");
		post_code(0x12);
		mdelay(500);
	}
	printk(BIOS_WARNING, "BMC not ready after %d s\n", BMC_READY_POLLS / 2);
}

/*
 * GbtReDriverPei (after training): board TX tuning of 20 lanes of PHY block
 * 0x1240b5xx and the USB PHY tuning of ports 0, 2 and 3. Then GbtVgaInitial.
 */
void mainboard_dxio_training_done(void)
{
	for (size_t i = 0; i < mj11_post_training_smn_count; i++)
		smn_write32(mj11_post_training_smn[i].reg, mj11_post_training_smn[i].value);

	wait_for_bmc();
}

static void mainboard_init(void *chip_info)
{
	mainboard_program_gpios();
}

/* NetFn 0x2e is Gigabyte's OEM IANA 0x003c0a. ErP off (Setup default): 0x1c. */
static void ipmi_set_erp(void)
{
	static const uint8_t erp_off[] = { 0x0a, 0x3c, 0x00, 0x0d, 0x1c };
	struct ipmi_rsp rsp;

	if (ipmi_message(MJ11_BMC_KCS_PORT, 0x2e, 0, 0x20, erp_off, sizeof(erp_off),
			 (uint8_t *)&rsp, sizeof(rsp)) < (int)sizeof(rsp) || rsp.completion_code)
		printk(BIOS_WARNING, "BMC ErP command failed\n");
}

/* GbtSystemBootEvent: SEL "System Boot Initiated" */
static void ipmi_log_boot(void)
{
	struct sel_event_record sel = {
		.record_type = 2,
		.sel_type.standard_type = {
			.gen_id = 1,
			.evm_rev = 4,
			.sensor_type = 0x12,	/* system event */
			.sensor_num = 0,
			.event_dir_type = 0x6f,
			.event_data = { 0x01, 0xff, 0xff },
		},
	};

	if (ipmi_add_sel(MJ11_BMC_KCS_PORT, &sel) != CB_SUCCESS)
		printk(BIOS_WARNING, "Adding the boot SEL entry failed\n");
}

#define PM_RTC_SHADOW		0x5b

/*
 * GbtACLossIPMI: the BMC's power restore policy (Get Chassis Status, current
 * power state bits 6:5) replaces the AC-loss setting in PMx5B.
 */
static void ipmi_apply_power_policy(void)
{
	static const uint8_t pmx5b[] = { 4, 7, 5 };	/* off, previous, on */
	struct {
		struct ipmi_rsp resp;
		uint8_t power_state;
		uint8_t last_event;
		uint8_t misc_state;
	} __packed rsp;
	uint8_t policy;

	if (ipmi_message(MJ11_BMC_KCS_PORT, IPMI_NETFN_CHASSIS, 0, 0x01, NULL, 0,
			 (uint8_t *)&rsp, sizeof(rsp)) < (int)sizeof(rsp) ||
	    rsp.resp.completion_code) {
		printk(BIOS_WARNING, "Get Chassis Status failed\n");
		return;
	}

	policy = rsp.power_state >> 5;
	if (policy >= ARRAY_SIZE(pmx5b))
		return;
	pm_write8(PM_RTC_SHADOW, (pm_read8(PM_RTC_SHADOW) & 0xf0) | pmx5b[policy]);
	printk(BIOS_INFO, "BMC power restore policy %u\n", policy);
}

/*
 * GbtPreBootSvid: the AST2500 VGA on the last bus behind 00:01.1 gets the
 * Gigabyte subsystem ID; config register 0x80 unlocks the write.
 */
static void ast_vga_set_ssid(void)
{
	const uint8_t bus = pci_s_read_config8(PCI_DEV(0, 1, 1), PCI_SUBORDINATE_BUS);
	const pci_devfn_t vga = PCI_DEV(bus, 0, 0);

	if (!bus || pci_s_read_config32(vga, PCI_VENDOR_ID) != 0x20001a03)
		return;
	pci_s_write_config32(vga, 0x80, 0xaa1a03aa);
	pci_s_write_config32(vga, PCI_SUBSYSTEM_VENDOR_ID, 0x10001458);
	pci_s_write_config32(vga, 0x80, 0);
}

static void mainboard_final(void *chip_info)
{
	ast_vga_set_ssid();
	ipmi_log_boot();
	ipmi_apply_power_policy();
	ipmi_set_erp();

	/* GbtNmiBtn: the BMC diagnostic interrupt arrives on GEVENT11. */
	if (CONFIG(HAVE_SMI_HANDLER)) {
		smi_write32(SMI_REG_SMISTS0, BIT(SMITYPE_G_AGPIO7));
		configure_smi(SMITYPE_G_AGPIO7, SMI_MODE_SMI);
	}
}

#define SMN_IOMUX_136_139	0x02d01d88	/* FCH ACPI MMIO 0xfed80d88 */
#define GPIO_SMBUS_MUX		137

/*
 * SMBusSwitch at ReadyToBoot: bit 1 of the GPIO137 IOMUX byte set and bits 7:4
 * of pin 139 cleared (one SMN dword write), GPIO137 driven low, the PM1
 * write, and pins 113-117 back to function 0 (SMBus0). Inferred purpose: hand
 * the SMBus to the OS.
 */
static void mainboard_ready_to_boot(void *unused)
{
	uint32_t gpio;

	smn_write32(SMN_IOMUX_136_139, (smn_read32(SMN_IOMUX_136_139) & 0x0ffffdff) | BIT(9));
	gpio = read32p(MJ11_GPIO_BANK(GPIO_SMBUS_MUX));
	write32p(MJ11_GPIO_BANK(GPIO_SMBUS_MUX), (gpio & 0x00060800) | GPIO_OUTPUT_ENABLE);

	mj11_clear_pm1();

	/* Two unaligned dword writes, as in the vendor code */
	write32p(ACPIMMIO_BASE(IOMUX) + 113, 0);
	write32p(ACPIMMIO_BASE(IOMUX) + 114, 0);
}

BOOT_STATE_INIT_ENTRY(BS_PAYLOAD_LOAD, BS_ON_EXIT, mainboard_ready_to_boot, NULL);

struct chip_operations mainboard_ops = {
	.init = mainboard_init,
	.final = mainboard_final,
};
