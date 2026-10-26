/* SPDX-License-Identifier: GPL-2.0-only */

#include <device/mmio.h>
#include <soc/platform_descriptors.h>
#include <console/console.h>
#include <string.h>

/* S8030 $A26 SATA PHY parameters, Rome ffe46fcc / ffe47b2c. */
static const struct sp3_phy_parameter sata0[] = {
	{ 0xc1, 0 }, { 0xc2, 0xa }, { 0xc0, 0 }, { 0xd2, 1 }, { 0xd3, 1 },
	{ 0x81, 1 }, { 0x82, 0 }, { 0x80, 0 }, { 0x92, 1 }, { 0x93, 1 },
	{ 0x41, 5 }, { 0x42, 0 }, { 0x40, 0 }, { 0x52, 1 }, { 0x53, 1 },
};
static const struct sp3_phy_parameter sata1[] = {
	{ 0xc1, 0 }, { 0xc2, 0xe }, { 0xc0, 0 }, { 0xd2, 1 }, { 0xd3, 1 },
	{ 0x81, 1 }, { 0x82, 0 }, { 0x80, 0 }, { 0x92, 1 }, { 0x93, 1 },
	{ 0x41, 5 }, { 0x42, 0 }, { 0x40, 0 }, { 0x52, 1 }, { 0x53, 1 },
};
static const struct sp3_phy_parameter sata_m2[] = {
	{ 0xc1, 0 }, { 0xc2, 0xa }, { 0xc0, 0 }, { 0xd2, 0 }, { 0xd3, 1 },
	{ 0x81, 4 }, { 0x82, 0 }, { 0x80, 0 }, { 0x92, 1 }, { 0x93, 0 },
	{ 0x41, 0xa }, { 0x42, 0 }, { 0x40, 0 }, { 0x52, 0 }, { 0x53, 1 },
};
static dxio_descriptor engines[40];
static size_t used;

static void add(uint8_t kind, uint8_t start, uint8_t end, uint8_t dev, uint8_t fn,
		uint8_t speed, const struct sp3_phy_parameter *tuning, size_t tuning_count)
{
	if (used == ARRAY_SIZE(engines))
		die("S8030 lane table overflow\n");
	engines[used++] = (dxio_descriptor) {
		.engine_type = kind, .start_logical_lane = start, .end_logical_lane = end,
		.device_number = dev, .function_number = fn, .link_speed_capability = speed,
		.port_present = true, .tuning = tuning, .tuning_count = tuning_count,
	};
}

static void slot(unsigned int number, uint8_t start, uint8_t end)
{
	const char *bifurcation = CONFIG_S8030_SLOT_BIFURCATION;

	if (strlen(bifurcation) != 5 ||
	    !(bifurcation[number - 1] == '1' || bifurcation[number - 1] == '2' ||
	      bifurcation[number - 1] == '4'))
		die("Invalid S8030 slot bifurcation setting\n");
	const unsigned int n = bifurcation[number - 1] - '0';
	const unsigned int width = 16 / n;
	for (unsigned int i = 0; i < n; i++) {
		const uint8_t s = start < end ? start + width * i : start - width * i;
		const uint8_t e = start < end ? s + width - 1 : s - width + 1;

		add(PCIE_ENGINE, s, e, 0, 0, GEN_MAX, NULL, 0);
	}
}

void mainboard_get_dxio_descriptors(unsigned int socket, const dxio_descriptor **dxio,
				    size_t *count)
{
	if (socket)
		die("S8030 has one socket\n");
	used = 0;
	add(SATA_ENGINE, 0, 5, 0, 0, 0, sata0, ARRAY_SIZE(sata0));
	add(PCIE_ENGINE, 8, 11, 3, 1, GEN_MAX, NULL, 0);
	add(PCIE_ENGINE, 12, 15, 3, 2, GEN_MAX, NULL, 0);
	slot(3, 31, 16);
	add(SATA_ENGINE, 32, 39, 0, 0, 0, sata1, ARRAY_SIZE(sata1));
	add(PCIE_ENGINE, 40, 43, 1, 1, GEN_MAX, NULL, 0);
	add(PCIE_ENGINE, 44, 47, 1, 2, GEN_MAX, NULL, 0);
	slot(5, 48, 63);
	slot(2, 95, 80);
	if (read32p(0xfed81500 + 4 * 144) & BIT(16))
		add(PCIE_ENGINE, 64, 67, 3, 2, GEN_MAX, NULL, 0);
	else
		add(SATA_ENGINE, 64, 64, 0, 0, 0, sata0, ARRAY_SIZE(sata0));
	if (read32p(0xfed81500 + 4 * 140) & BIT(16))
		add(PCIE_ENGINE, 68, 71, 3, 3, GEN_MAX, NULL, 0);
	else
		add(SATA_ENGINE, 68, 68, 0, 0, 0, sata_m2, ARRAY_SIZE(sata_m2));
	add(PCIE_ENGINE, 72, 72, 3, 1, GEN2, NULL, 0);
	add(PCIE_ENGINE, 74, 74, 3, 5, GEN1, NULL, 0);
	add(PCIE_ENGINE, 75, 75, 3, 6, GEN1, NULL, 0);
	add(PCIE_ENGINE, 76, 79, 3, 4, GEN3, NULL, 0);
	slot(1, 127, 112);
	slot(4, 111, 96);
	*dxio = engines;
	*count = used;
}
