/* SPDX-License-Identifier: GPL-2.0-only */

#include <bootstate.h>
#include "nbio.h"
#include "topology.h"

void sp3_nbio_pre_enumeration_init(void)
{
}

static void lock_nbif(void *unused)
{
	for (unsigned int rb = 0; rb < sp3_die_count(); rb++) {
		const uint32_t q = sp3_nbio_index(rb) << 20;

		for (unsigned int k = 0; k < 3; k++)
			sp3_smn_clrsetbits32(rb, 0x1012358c + q + (k << 22), 0, 1);
		sp3_smn_clrsetbits32(rb, 0x13f00044 + q, 0, 1);
	}
}

BOOT_STATE_INIT_ENTRY(BS_DEV_ENABLE, BS_ON_EXIT, lock_nbif, NULL);
