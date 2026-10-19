/* SPDX-License-Identifier: GPL-2.0-only */

#include <bootstate.h>
#include <program_loading.h>

/*
 * Read the payload from flash in the background while ramstage waits on
 * hardware, mostly in libgfxinit's delays during device init.
 */
static void preload_payload(void *unused)
{
	payload_preload();
}

BOOT_STATE_INIT_ENTRY(BS_PRE_DEVICE, BS_ON_ENTRY, preload_payload, NULL);
