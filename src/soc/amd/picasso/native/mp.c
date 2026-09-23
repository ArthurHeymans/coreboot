/* SPDX-License-Identifier: GPL-2.0-only */

#include <acpi/acpi.h>
#include <amdblocks/cpu.h>
#include <amdblocks/psp.h>
#include <console/console.h>
#include <cpu/x86/cache.h>
#include <cpu/x86/smm.h>
#include <string.h>
#include <types.h>

#include "native.h"
#include "nbio.h"

/* ljmp $(SMM_DEFAULT_BASE >> 4), $0: enter the regular mp_init SIPI vector. */
static const uint8_t ap_far_jump[] = {
	0xea, 0x00, 0x00, (SMM_DEFAULT_BASE >> 4) & 0xff, SMM_DEFAULT_BASE >> 12,
};
static uint8_t saved_reset_vector[sizeof(ap_far_jump)];
static bool reset_vector_patched;

static uint8_t *ap_reset_vector(void)
{
	/* Match BOOTBLOCK_END in the non-CAR memory layout. */
	const uintptr_t bootblock_end = CONFIG_ROMSTAGE_ADDR +
		(CONFIG(SEPARATE_ROMSTAGE) ? 0 : CONFIG_ROMSTAGE_SIZE);

	return (void *)(bootblock_end - 0x10);
}

static void flush_reset_vector(void)
{
	if (clflush_supported())
		clflush_region((uintptr_t)ap_reset_vector(), sizeof(ap_far_jump));
	else
		wbinvd();
}

/*
 * Without FSP the APs can't be started with INIT/SIPI. The SMU releases them from reset.
 * Temporarily replace the BSP reset vector with a jump to the SIPI vector and restore
 * the original bytes after MP init.
 */
enum cb_err soc_amd_mp_start_aps(int ap_count)
{
	uint8_t *const reset_vector = ap_reset_vector();

	if (ap_count <= 0)
		return CB_SUCCESS;

	/* APs use this in parallel, so cache the PSP MMIO base now */
	if (!get_psp_mmio_base()) {
		printk(BIOS_ERR, "PSP MMIO base is not set up\n");
		return CB_ERR;
	}

	memcpy(saved_reset_vector, reset_vector, sizeof(saved_reset_vector));
	reset_vector_patched = true;
	memcpy(reset_vector, ap_far_jump, sizeof(ap_far_jump));
	flush_reset_vector();

	return picasso_smu_launch_aps(ap_count);
}

/* Finish the silicon initialization after all CPUs got initialized */
void soc_amd_post_mp_init(void)
{
	/* All APs have left the reset vector. Restore the BIOS image before S3 can occur. */
	if (reset_vector_patched) {
		memcpy(ap_reset_vector(), saved_reset_vector, sizeof(saved_reset_vector));
		flush_reset_vector();
		reset_vector_patched = false;
	}

	if (acpi_is_wakeup_s3())
		return;

	if (picasso_nbio_ccx_done() || picasso_gfx_init() || picasso_nbio_post_silicon_init())
		die("Native silicon initialization failed\n");
}
