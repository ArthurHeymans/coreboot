/* SPDX-License-Identifier: GPL-2.0-only */

#include <boot_device.h>
#include <endian.h>
#include <fmap_config.h>
#include <spi_flash.h>
#include <amdblocks/psp.h>
#include <amdblocks/spi.h>

#if CONFIG_ROM_SIZE >= (16 * MiB)
#define ROM_SIZE (16 * MiB)
#else
#define ROM_SIZE CONFIG_ROM_SIZE
#endif

/* The ROM is memory mapped just below 4GiB. Form a pointer for the base. */
#define rom_base ((void *)(uintptr_t)(0x100000000ULL-ROM_SIZE))

static const struct mem_region_device spi_boot_dev =
	MEM_REGION_DEV_RO_INIT(rom_base, ROM_SIZE);

#if CONFIG(PSP_BIOSBIN_INCLUDES_CBFS) && !ENV_SMM

#define psp_cbfs_base ((void *)(CONFIG_ROMSTAGE_ADDR - CONFIG_C_ENV_BOOTBLOCK_SIZE))

static const struct mem_region_device psp_cbfs_dev =
	MEM_REGION_DEV_RO_INIT(psp_cbfs_base, CONFIG_C_ENV_BOOTBLOCK_SIZE);

static const struct xlate_window boot_dev_windows[] = {
	{
		.access_dev = &psp_cbfs_dev.rdev,
		.sub_region = {
			.offset = FMAP_SECTION_COREBOOT_START,
			.size = CONFIG_C_ENV_BOOTBLOCK_SIZE,
		},
	},
	{
		.access_dev = &spi_boot_dev.rdev,
		.sub_region = {
			.offset = 0,
			.size = ROM_SIZE,
		},
	},
};

static const struct xlate_region_device boot_dev =
	XLATE_REGION_DEV_RO_INIT(boot_dev_windows, ROM_SIZE);

#endif

const struct region_device *boot_device_ro(void)
{
	/*
	 * The following code assumes that ROM2 is mapped at flash offset 0. This is the default
	 * configuration currently enforced by soft-straps.
	 * When ROM Armor is enabled, don't call fch_spi_rom_remapping()
	 * because the SPIBAR is no longer accessible.
	 */
	const bool rom_armor = psp_get_hsti_state_rom_armor_enforced();
	if (!rom_armor && fch_spi_rom_remapping() != 0)
		die("Non default SPI ROM remapping is not supported!");

	/* FSP might reconfigure it, so initialize again in every stage */
	fch_spi_configure_4dw_burst();

#if CONFIG(PSP_BIOSBIN_INCLUDES_CBFS) && !ENV_SMM
	return &boot_dev.rdev;
#else
	return &spi_boot_dev.rdev;
#endif
}

uint32_t spi_flash_get_mmap_windows(struct flash_mmap_window *table)
{
	table->flash_base = 0;
	table->host_base = (uint32_t)(uintptr_t)rom_base;
	table->size = ROM_SIZE;

	return 1;
}
