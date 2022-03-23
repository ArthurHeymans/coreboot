/* SPDX-License-Identifier: GPL-2.0-only */

#include <console/console.h>
#include <amdblocks/smm.h>
#include <FspGuids.h>
#include <fsp/util.h>
#include <memrange.h>

void fsp_get_tseg_region(uintptr_t *start, size_t *size)
{
       int status;
       struct range_entry tseg;

       *start = 0;
       *size = 0;

       status = fsp_find_range_hob(&tseg, AMD_FSP_TSEG_HOB_GUID.b);

       if (status < 0) {
               printk(BIOS_ERR, "unable to find TSEG HOB\n");
               return;
       }

       *start = (uintptr_t)range_entry_base(&tseg);
       *size = range_entry_size(&tseg);
}
