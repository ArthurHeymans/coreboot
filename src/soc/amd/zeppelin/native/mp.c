/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/cpu.h>
#include <amdblocks/psp.h>
#include <console/console.h>
#include <cpu/x86/cache.h>
#include <cpu/x86/smm.h>
#include <string.h>
#include <types.h>

#include "native.h"
#include "smu.h"

/* ljmp $(SMM_DEFAULT_BASE >> 4), $0: enter the regular mp_init SIPI vector. */
static const uint8_t ap_far_jump[] = {
	0xea, 0x00, 0x00, (SMM_DEFAULT_BASE >> 4) & 0xff, SMM_DEFAULT_BASE >> 12,
};
static uint8_t saved_reset_vector[sizeof(ap_far_jump)];
static bool reset_vector_patched;

/*
 * The APs start at the reset vector of the BIOS image copy in DRAM, which is
 * BOOTBLOCK_END - 0x10 (0x76fffff0 on MJ11, the same as the vendor firmware).
 */
static uint8_t *ap_reset_vector(void)
{
	return (void *)(CONFIG_ROMSTAGE_ADDR - 0x10);
}

static void flush_reset_vector(void)
{
	if (clflush_supported())
		clflush_region((uintptr_t)ap_reset_vector(), sizeof(ap_far_jump));
	else
		wbinvd();
}

/*
 * The SMU releases the APs from reset; INIT/SIPI only works after that.
 * Temporarily replace the reset vector with a jump to the SIPI vector, as the
 * vendor firmware does, and restore the original bytes after MP init.
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

	return zeppelin_smu_launch_aps(ap_count);
}

void soc_amd_post_mp_init(void)
{
	if (reset_vector_patched) {
		memcpy(ap_reset_vector(), saved_reset_vector, sizeof(saved_reset_vector));
		flush_reset_vector();
		reset_vector_patched = false;
	}

	if (zeppelin_smu_aps_ready())
		printk(BIOS_ERR, "SMU messages after AP launch failed\n");
	if (zeppelin_smu_ccx_done())
		printk(BIOS_ERR, "SMU CCX-done messages failed\n");
}
