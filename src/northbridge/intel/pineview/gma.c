/* SPDX-License-Identifier: GPL-2.0-only */

#include <bootmode.h>
#include <console/console.h>
#include <delay.h>
#include <device/device.h>
#include <device/mmio.h>
#include <device/pci.h>
#include <device/pci_ids.h>
#include <device/pci_ops.h>
#include <drivers/intel/gma/gma.h>
#include <drivers/intel/gma/i915.h>
#include <drivers/intel/gma/libgfxinit.h>
#include <drivers/intel/gma/opregion.h>
#include <types.h>

#include "chip.h"
#include "pineview.h"

#define PGETBL_CTL	0x2020
#define PGETBL_512KB	0

static int gtt_setup(u8 *mmiobase)
{
	u32 gttbase;
	struct device *dev = pcidev_on_root(0, 0);

	gttbase = pci_read_config32(dev, BGSM);
	printk(BIOS_DEBUG, "gttbase = %08x\n", gttbase);

	write32(mmiobase + PGETBL_CTL, gttbase | PGETBL_512KB);
	udelay(50);
	write32(mmiobase + PGETBL_CTL, gttbase | PGETBL_512KB);

	write32(mmiobase + GFX_FLSH_CNTL, 0);

	return 0;
}

static void generate_vbt(struct device *dev)
{
	const struct northbridge_intel_pineview_config *conf = dev->chip_info;

	generate_fake_intel_oprom(&conf->gfx, dev, "$VBT PINEVIEW");
}

static void gma_func0_init(struct device *dev)
{
	int lightup_ok = 0;

	intel_gma_init_igd_opregion();

	if (!CONFIG(NO_GFX_INIT))
		pci_or_config16(dev, PCI_COMMAND, PCI_COMMAND_MASTER);

	if (CONFIG(MAINBOARD_USE_LIBGFXINIT)) {
		const int vga_disable = (pci_read_config16(dev, GGC) & 2) >> 1;
		struct resource *mmio_res;

		if (acpi_is_wakeup_s3()) {
			printk(BIOS_INFO,
			       "Skipping libgfxinit graphics initialization when resuming from ACPI S3.\n");
			return;
		}

		if (vga_disable) {
			printk(BIOS_INFO,
			       "IGD is not decoding legacy VGA MEM and IO: skipping NATIVE graphic init\n");
			return;
		}

		mmio_res = find_resource(dev, PCI_BASE_ADDRESS_0);
		if (!mmio_res || !mmio_res->base) {
			printk(BIOS_ERR, "Unable to find GMA MMIO resource.\n");
			return;
		}

		printk(BIOS_SPEW, "Initializing graphics with libgfxinit.\n");
		if (gtt_setup(res2mmio(mmio_res, 0, 0))) {
			printk(BIOS_ERR, "GTT setup failed; skipping libgfxinit.\n");
			return;
		}

		gma_gfxinit(&lightup_ok);
		if (lightup_ok) {
			gfx_set_init_done(1);
			generate_vbt(dev);
		}
	} else {
		/* PCI init, will run VBIOS */
		pci_dev_init(dev);
	}
}

static void gma_generate_ssdt(const struct device *device)
{
	const struct northbridge_intel_pineview_config *chip = device->chip_info;

	drivers_intel_gma_displays_ssdt_generate(&chip->gfx);
}

static const char *gma_acpi_name(const struct device *dev)
{
	return "GFX0";
}

static struct device_operations gma_func0_ops = {
	.read_resources         = pci_dev_read_resources,
	.set_resources          = pci_dev_set_resources,
	.enable_resources       = pci_dev_enable_resources,
	.init                   = gma_func0_init,
	.acpi_fill_ssdt         = gma_generate_ssdt,
	.ops_pci                = &pci_dev_ops_pci,
	.acpi_name              = gma_acpi_name,
};

static const unsigned short pci_device_ids[] = {
	0xa001,
	0xa011,
	0,
};

static const struct pci_driver gma __pci_driver = {
	.ops     = &gma_func0_ops,
	.vendor  = PCI_VID_INTEL,
	.devices = pci_device_ids,
};
