/* SPDX-License-Identifier: GPL-2.0-only */

#include <console/console.h>
#include <fsp/util.h>
#include <FspGuids.h>
#include <misc_data.h>
#include <soc/soc_util.h>
#include <types.h>

uint32_t picasso_fsp_silicon_id(void)
{
	size_t hob_size = 0;
	const struct picasso_misc_data *hob;

	hob = fsp_find_extension_hob_by_guid(PICASSO_MISC_DATA_HOB_GUID.b, &hob_size);

	if (hob == NULL || hob_size == 0) {
		printk(BIOS_ERR, "Couldn't find Picasso misc data HOB.\n");
		return 0;
	}

	if (hob->version != PICASSO_MISC_DATA_VERSION) {
		printk(BIOS_ERR, "Unexpected Picasso misc data HOB version.\n");
		return 0;
	}

	return hob->silicon_id;
}
