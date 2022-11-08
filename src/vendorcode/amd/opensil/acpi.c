/* SPDX-License-Identifier: GPL-2.0-only */

#include <acpi/acpi.h>
#include <Sil-api.h>
#include <SilCommon.h>
#include <xSIM-api.h>
#include <FCH/Common/FchCommonCfg.h>
#include "console.h"
#include "opensil.h"

void opensil_fill_fadt(acpi_fadt_t *fadt)
{
	FCHHWACPI_INPUT_BLK *blk = xSimFindStructure(SilId_FchHwAcpiP,  0);

	fadt->pm1a_evt_blk = blk->AcpiPm1EvtBlkAddr;
	fadt->x_pm1a_evt_blk.addrl = blk->AcpiPm1EvtBlkAddr;
	fadt->pm1a_cnt_blk = blk->AcpiPm1CntBlkAddr;
	fadt->x_pm1a_cnt_blk.addrl = blk->AcpiPm1CntBlkAddr;
	fadt->pm_tmr_blk = blk->AcpiPmTmrBlkAddr;
	fadt->x_pm_tmr_blk.addrl = blk->AcpiPmTmrBlkAddr;
	fadt->gpe0_blk = blk->AcpiGpe0BlkAddr;
	fadt->x_gpe0_blk.addrl = blk->AcpiGpe0BlkAddr;
}
