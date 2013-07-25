/* $NoKeywords:$ */
/**
 * @file
 *
 * mns3kb.c
 *
 * KB memory specific function to support S3 resume
 *
 * @xrefitem bom "File Content Label" "Release Content"
 * @e project: AGESA
 * @e sub-project: (Mem/NB/KB)
 * @e \$Revision: 87696 $ @e \$Date: 2013-02-07 12:35:03 -0600 (Thu, 07 Feb 2013) $
 *
 **/
/*****************************************************************************
*
 * Copyright (c) 2008 - 2013, Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Advanced Micro Devices, Inc. nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ADVANCED MICRO DEVICES, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* ***************************************************************************
*
*/

/*
 *----------------------------------------------------------------------------
 *                                MODULES USED
 *
 *----------------------------------------------------------------------------
 */



#include "AGESA.h"
#include "AdvancedApi.h"
#include "amdlib.h"
#include "Ids.h"
#include "OptionMemory.h"
#include "mm.h"
#include "mn.h"
#include "S3.h"
#include "mfs3.h"
#include "mnkb.h"
#include "cpuRegisters.h"
#include "cpuFamRegisters.h"
#include "cpuFamilyTranslation.h"
#include "mnS3kb.h"
#include "heapManager.h"
#include "Filecode.h"
CODE_GROUP (G3_DXE)
RDATA_GROUP (G3_DXE)

#define FILECODE PROC_MEM_NB_KB_MNS3KB_FILECODE
#define DCT0_MEMPSTATE_MASK 0x10
#define DCT1_MEMPSTATE_MASK 0x20
#define DO_NOT_CARE 0
/*----------------------------------------------------------------------------
 *                           TYPEDEFS AND STRUCTURES
 *
 *----------------------------------------------------------------------------
 */

/*----------------------------------------------------------------------------
 *                        PROTOTYPES OF LOCAL FUNCTIONS
 *
 *----------------------------------------------------------------------------
 */
UINT16
STATIC
MemNS3GetRegLstPtrKB (
  IN OUT   MEM_NB_BLOCK *NBPtr,
  IN OUT   DESCRIPTOR_GROUP *DescriptPtr
  );

AGESA_STATUS
STATIC
MemNS3GetDeviceRegLstKB (
  IN       UINT32 RegisterLstID,
     OUT   VOID **RegisterHeader
  );

VOID
STATIC
MemNS3SetDfltPhyRegKB (
  IN       ACCESS_WIDTH AccessWidth,
  IN       PCI_ADDR Address,
  IN       VOID *Value,
  IN OUT   VOID *ConfigPtr
  );

VOID
STATIC
MemNS3SetDynModeChangeKB (
  IN       ACCESS_WIDTH AccessWidth,
  IN       PCI_ADDR Address,
  IN OUT   VOID *Value,
  IN OUT   VOID *ConfigPtr
  );

VOID
STATIC
MemNS3SetPhyStatusRegKB (
  IN       ACCESS_WIDTH AccessWidth,
  IN       PCI_ADDR Address,
  IN OUT   VOID *Value,
  IN OUT   VOID *ConfigPtr
  );

VOID
STATIC
MemNS3DisableChannelKB (
  IN       ACCESS_WIDTH AccessWidth,
  IN       PCI_ADDR Address,
  IN OUT   VOID *Value,
  IN OUT   VOID *ConfigPtr
  );

VOID
STATIC
MemNS3GetConPCIMaskKB (
  IN OUT   MEM_NB_BLOCK *NBPtr,
  IN OUT   DESCRIPTOR_GROUP *DescriptPtr
  );

VOID
STATIC
MemNS3ChangeMemPStateContextAndFlowNb (
  IN       ACCESS_WIDTH AccessWidth,
  IN       PCI_ADDR Address,
  IN OUT   VOID *Value,
  IN OUT   VOID *ConfigPtr
  );

VOID
STATIC
MemNS3GetCSRKB (
  IN       ACCESS_WIDTH AccessWidth,
  IN       PCI_ADDR Address,
  IN       VOID *Value,
  IN OUT   VOID *ConfigPtr
  );

VOID
STATIC
MemNS3SetCSRKB (
  IN       ACCESS_WIDTH AccessWidth,
  IN       PCI_ADDR Address,
  IN OUT   VOID *Value,
  IN OUT   VOID *ConfigPtr
  );

VOID
STATIC
MemNS3SetPhyFenceKB (
  IN       ACCESS_WIDTH AccessWidth,
  IN       PCI_ADDR Address,
  IN       VOID *Value,
  IN OUT   VOID *ConfigPtr
  );

BOOLEAN
MemS3ResumeConstructNBBlockKB (
  IN OUT   VOID *S3NBPtr,
  IN OUT   MEM_DATA_STRUCT *MemPtr,
  IN       UINT8 NodeID
  );

/*----------------------------------------------------------------------------
 *                          DEFINITIONS AND MACROS
 *
 *----------------------------------------------------------------------------
 */
PCI_SPECIAL_CASE PciSpecialCaseFuncKB[] = {
  {MemNS3GetCSRKB, MemNS3SetCSRKB},
  {MemNS3GetBitFieldNb, MemNS3SetBitFieldNb},
  {MemNS3GetNBPStateDepRegUnb, MemNS3SetNBPStateDepRegUnb},
  { (VOID (*) (ACCESS_WIDTH, PCI_ADDR, VOID *, VOID *)) memDefRet, MemNS3SetDfltPhyRegKB},
  {MemNS3ChangeMemPStateContextAndFlowNb, MemNS3ChangeMemPStateContextAndFlowNb},
  { (VOID (*) (ACCESS_WIDTH, PCI_ADDR, VOID *, VOID *)) memDefRet, MemNS3SetDynModeChangeKB},
  { (VOID (*) (ACCESS_WIDTH, PCI_ADDR, VOID *, VOID *)) memDefRet, MemNS3DisableChannelKB},
  {MemNS3SaveNBRegisterUnb, MemNS3RestoreNBRegisterUnb},
  {MemNS3GetBitFieldNb, MemNS3SetPreDriverCalUnb},
  { (VOID (*) (ACCESS_WIDTH, PCI_ADDR, VOID *, VOID *)) memDefRet, MemNS3SetPhyStatusRegKB},
  { (VOID (*) (ACCESS_WIDTH, PCI_ADDR, VOID *, VOID *)) memDefRet, MemNS3SetMemClkFreqValUnb},
  {MemNS3ChangeMemPStateContextNb, MemNS3ChangeMemPStateContextNb},
  {MemNS3GetBitFieldNb, MemNS3SetPhyFenceKB},
  { (VOID (*) (ACCESS_WIDTH, PCI_ADDR, VOID *, VOID *)) memDefRet, MemNS3ReleaseNBPSUnb},
  { (VOID (*) (ACCESS_WIDTH, PCI_ADDR, VOID *, VOID *)) memDefRet, MemNS3ForceNBP0Unb},
  {MemNSaveHobDataUnb, MemNRestoreHobDataUnb}
};

MSR_SPECIAL_CASE MsrSpecialCaseFuncKB[] = {
  { MemNModdifyMtrrFixDramModEn, MemNModdifyMtrrFixDramModEn}
};
PCI_REG_DESCRIPTOR ROMDATA S3PciPreSelfRefDescriptorKB[] = {
  {{14,3, 1}, DO_NOT_CARE, 0, 0},
  {{0, 0, 0}, FUNC_2, 0x110, 0x00000020},
  {{0, 0, 0}, FUNC_1, 0x40,  0xFFFF0703},
  {{0, 0, 0}, FUNC_1, 0x44,  0xFFFF0707},
  {{0, 0, 0}, FUNC_1, 0xF0,  0xFF00FF83},
  {{0, 0, 0}, FUNC_1, 0x120, 0x00001FFF},
  {{0, 0, 0}, FUNC_1, 0x124, 0x00001FFF},
  {{0, 0, 0}, FUNC_1, 0x200, 0x00FFF87B},
  {{0, 0, 0}, FUNC_1, 0x204, 0x00FFF800},
  {{0, 0, 0}, FUNC_2, 0x114, 0x00000200},
  {{0, 0, 0}, FUNC_2, 0x118, 0xF773FFFF},
  {{0, 0, 0}, FUNC_2, 0x11C, 0xAFFFFFFF},
  {{0, 0, 0}, FUNC_2, 0x120, 0x00FFFFFF},
  {{0, 0, 0}, FUNC_2, 0x1B0, 0xFFD3FF3F},
  {{0, 0, 0}, FUNC_2, 0x1B4, 0xFC7FFFFF},
  {{0, 0, 0}, FUNC_2, 0x1BC, 0xFFFFFFFF},
  {{0, 0, 0}, FUNC_2, 0xA4,  0x00F07900},
  {{0, 0, 0}, FUNC_5, 0x240, 0xFFFFFFC1},
  {{0, 0, 0}, FUNC_5, 0x244, 0x0000FFFF},
  {{0, 0, 0}, FUNC_5, 0x248, 0xFFFFFFC0},
  {{0, 0, 0}, FUNC_5, 0x24C, 0x0000FFFF}
};

CONST PCI_REGISTER_BLOCK_HEADER ROMDATA S3PciPreSelfRefKB = {
  0,
  (sizeof (S3PciPreSelfRefDescriptorKB) / sizeof (PCI_REG_DESCRIPTOR)),
  S3PciPreSelfRefDescriptorKB,
  PciSpecialCaseFuncKB
};

CONDITIONAL_PCI_REG_DESCRIPTOR ROMDATA S3CPciPreSelfDescriptorKB[] = {
   // DCT 0
  {{7, 0, 1}, DCT0,   0x40,  0x7FF8FFED, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x44,  0x7FF8FFED, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x48,  0x7FF8FFED, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x4C,  0x7FF8FFED, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x60,  0x7FF8FFE0, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x64,  0x7FF8FFE0, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x78,  0x00020000, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 2, 1}, DCT0,   0x80,  0x0000FFFF, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x84,  0x00800003, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x88,  0x3F000000, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x8C,  0x00070000, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x90,  0x0BFF0000, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0xA8,  0x9C730000, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x1BC, 0xFFFFFFFF, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x200, 0x3F1F1F1F, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x204, 0x0F3F0F3F, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x208, 0x07070707, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x20C, 0x00030F1F, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{2, 0, 1}, DCT0,   SET_S3_NB_PSTATE_OFFSET (0x210, 0), 0xFFC7000F, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{2, 0, 1}, DCT0,   SET_S3_NB_PSTATE_OFFSET (0x210, 1), 0xFFC7000F, DCT0_NBPSTATE_SUPPORT_MASK, DCT0_ANY_DIMM_MASK},
  {{2, 0, 1}, DCT0,   SET_S3_NB_PSTATE_OFFSET (0x210, 2), 0xFFC7000F, DCT0_NBPSTATE_SUPPORT_MASK, DCT0_ANY_DIMM_MASK},
  {{2, 0, 1}, DCT0,   SET_S3_NB_PSTATE_OFFSET (0x210, 3), 0xFFC7000F, DCT0_NBPSTATE_SUPPORT_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x214, 0x000F0F0F, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x218, 0x0F0F0F0F, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x21C, 0x001F1F00, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 2, 1}, DCT0,   0x220, 0x00001F0F, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 2, 1}, DCT0,   0x224, 0x0000070F, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x228, 0xFFFFFFFF, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 1, 1}, DCT0,   0x22C, 0x0000001F, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x230, 0x0F0F0F0F, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x234, 0x0F0F0F0F, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x238, 0x0F0F0F0F, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x23C, 0x0F0F0F0F, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 2, 1}, DCT0,   0x240, 0x000077FF, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 1, 1}, DCT0,   0x244, 0x0000000F, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x248, 0xBF3F1F0F, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x24C, 0x3F3F3F0F, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x2E0, 0x5F700000, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x2E8, 0xFFFFFFFF, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 2, 1}, DCT0,   0x2EC, 0x0000FFFF, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 1, 1}, DCT0,   0x2F0, 0x00000001, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 2, 1}, DCT0,   0x400, 0x00000F0F, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x404, 0x8F1F0F1F, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x408, 0x1F000007, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 2, 1}, DCT0,   0x420, 0x00000F0F, DCT0_MASK, DCT0_ANY_DIMM_MASK},

  // Phy Initialization
  // 1. Program D18F2x9C_x0D0F_E013_dct[1:0] = 0118h.
  {{1, 2, 1}, DCT0,   BFPllRegWaitTime, 0, DCT0_MASK + DCT1_MASK, ANY_DIMM_MASK},
  // 2. Force the phy to M0 with the following sequence:
  // A. Program D18F2x9C_x0D0F_E006_dct[1:0][PllLockTime] = 190h.
  {{3, 3, 1}, DCT0,   BFPllLockTime, 0, DCT0_MASK + DCT1_MASK, ANY_DIMM_MASK},
  // B. For each DCT: Program D18F2x9C_x0000_000B_dct[1:0] = 80800000h.
  {{9, 3, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x0B),  0, DCT0_MASK + DCT1_MASK, ANY_DIMM_MASK},
  // C. Program D18F2x9C_x0000_000B_dct[0] = 40000000h.
  // D. For each DCT: Program D18F2x9C_x0000_000B_dct[1:0] = 80000000h
  {{5, 3, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x0B),  0, DCT0_MASK + DCT1_MASK, ANY_DIMM_MASK},

  // 3. Phy voltage related
  {{1, 1, 1}, DCT0,   BFDataRxVioLvl, 0x00000018, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 1, 1}, DCT0,   BFClkRxVioLvl, 0x00000018, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFCmpVioLvl, 0x0000C000, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 1, 1}, DCT0,   BFCmdRxVioLvl, 0x00000018, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 1, 1}, DCT0,   BFCsrComparator, 0x0000000C, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 1, 1}, DCT0,   BFAddrRxVioLvl, 0x00000018, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 1, 1}, DCT0,   BFObsrvVrefSel, 0x00000007, DCT0_MASK, ANY_DIMM_MASK},

  // 4. Frequency Change
  // Check if a channel needs to be disabled
  {{1, 1, 1}, DCT0,   BFCKETri, 0, DCT0_MASK + DCT1_MASK, ANY_DIMM_MASK},
  {{6, 3, 1}, DCT0,   0, 0, DCT0_MASK + DCT1_MASK, ANY_DIMM_MASK},
  {{1, 1, 1}, DCT0,   BFPhyClkConfig0, 0, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 1, 1}, DCT0,   BFPhyClkConfig1, 0, DCT0_MASK, ANY_DIMM_MASK},

  {{7, 0, 1}, DCT0,   0x94,  0x9FF1CC1F, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFPOdtOff, 0, DCT0_MASK, ANY_DIMM_MASK},

  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x04), 0x003F3F3F, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFProcOdtAdv, 0, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFReducedLoop, 0, DCT0_MASK, ANY_DIMM_MASK},

  // Enable MemClk
  {{10, 0, 1}, DCT0,   0x94, 0, DCT0_MASK, DCT0_ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFPllLockTime, 0, DCT0_MASK, ANY_DIMM_MASK},
  // Hardware programs POdtOff for the current M-state (D18F2x9C_x0D0F_E006_dct[0][PhyPS]) with the value in D18F2x94_dct[0]
  // [ProcOdtDis] (via D18F2x9C_x0000_000B_dct[0][ProcOdtDis]) when the memory frequency is updated. BIOS must reprogram this
  // BIOS must reprogram this field after each frequency change if the target value differs from D18F2x94_dct[0][ProcOdtDis].
  {{1, 2, 1}, DCT0,   BFPOdtOff, 0, DCT0_MASK, ANY_DIMM_MASK},

  // DCT 0
  // 5. Phy Fence
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x0C), 0x7FFF3FFF, DCT0_MASK + DCT1_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFDataFence2, 0, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFFence2, 0x00007C1F, DCT0_MASK, ANY_DIMM_MASK},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x04), 0x003F3F3F, DCT0_MASK, ANY_DIMM_MASK},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x00),  0x70777777, DCT0_MASK, ANY_DIMM_MASK},
  // 6. Phy Compensation Init
  {{3, 3, 1}, DCT0,   BFDisablePredriverCal, 0, DCT0_MASK + DCT1_MASK, ANY_DIMM_MASK},
  {{1, 0, 1}, DCT0,   BFPhy0x0D0F0F0B, 0, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 0, 1}, DCT0,   BFPhy0x0D0F0F07, 0, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 0, 1}, DCT0,   BFPhy0x0D0F0F03, 0, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFDataByteTxPreDriverCal2Pad1, 0, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFDataByteTxPreDriverCal2Pad2, 0, DCT0_MASK, ANY_DIMM_MASK},
  {{8, 2, 1}, DCT0,   BFDataByteTxPreDriverCal, 0, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFCmdAddr0TxPreDriverCal2Pad1, 0, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFCmdAddr0TxPreDriverCal2Pad2, 0, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFCmdAddr1TxPreDriverCal2Pad1, 0, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFCmdAddr1TxPreDriverCal2Pad2, 0, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFAddrTxPreDriverCal2Pad1, 0, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFAddrTxPreDriverCal2Pad2, 0, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFAddrTxPreDriverCal2Pad3, 0, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFAddrTxPreDriverCal2Pad4, 0, DCT0_MASK, ANY_DIMM_MASK},
  {{8, 2, 1}, DCT0,   BFCmdAddr0TxPreDriverCalPad0, 0, DCT0_MASK, ANY_DIMM_MASK},
  {{8, 2, 1}, DCT0,   BFCmdAddr1TxPreDriverCalPad0, 0, DCT0_MASK, ANY_DIMM_MASK},
  {{8, 2, 1}, DCT0,   BFAddrTxPreDriverCalPad0, 0, DCT0_MASK, ANY_DIMM_MASK},
  {{8, 2, 1}, DCT0,   BFClock0TxPreDriverCalPad0, 0, DCT0_MASK, ANY_DIMM_MASK},
  {{8, 2, 1}, DCT0,   BFClock1TxPreDriverCalPad0, 0, DCT0_MASK, ANY_DIMM_MASK},

  {{1, 1, 1}, DCT0,   BFDisablePredriverCal, 0, DCT0_MASK + DCT1_MASK, ANY_DIMM_MASK},
  // Program MemPstate 1 registers
  // Switch to MemPstate context 1
  {{11, 3, 1}, DO_NOT_CARE, 1, DO_NOT_CARE, DCT0_MEMPSTATE_MASK + DCT1_MEMPSTATE_MASK, ANY_DIMM_MASK},
  {{1, 1, 1}, DCT0,   BFRate, 0, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFProcOdtAdv, 0, DCT0_MEMPSTATE_MASK, ANY_DIMM_MASK},
  {{1, 1, 1}, DCT0,   BFDataRxVioLvl, 0x00000018, DCT0_MEMPSTATE_MASK, ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x200, 0x3F1F1F1F, DCT0_MEMPSTATE_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x204, 0x0F3F0F3F, DCT0_MEMPSTATE_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x208, 0x07070707, DCT0_MEMPSTATE_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x20C, 0x00030F1F, DCT0_MEMPSTATE_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x214, 0x000F0F0F, DCT0_MEMPSTATE_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x218, 0x0F0F0F0F, DCT0_MEMPSTATE_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x21C, 0x001F1F00, DCT0_MEMPSTATE_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 1, 1}, DCT0,   0x22C, 0x0000001F, DCT0_MEMPSTATE_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 2, 1}, DCT0,   0x240, 0x000077FF, DCT0_MEMPSTATE_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x248, 0xBF3F1F0F, DCT0_MEMPSTATE_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 0, 1}, DCT0,   0x2E8, 0xFFFFFFFF, DCT0_MEMPSTATE_MASK, DCT0_ANY_DIMM_MASK},
  {{7, 2, 1}, DCT0,   0x2EC, 0x0000FFFF, DCT0_MEMPSTATE_MASK, DCT0_ANY_DIMM_MASK},
  // Compute Phy fence for MemPstate 1
  // DCT 0
  {{12, 2, 1}, DCT0,  BFChAM1FenceSave, 0, DCT0_MEMPSTATE_MASK, ANY_DIMM_MASK},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x04), 0x003F3F3F, DCT0_MEMPSTATE_MASK, ANY_DIMM_MASK},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x00), 0x70777777, DCT0_MEMPSTATE_MASK, ANY_DIMM_MASK},

  {{1, 1, 1}, DCT0,   BFDisablePredriverCal, 0, DCT0_MEMPSTATE_MASK + DCT1_MEMPSTATE_MASK, ANY_DIMM_MASK},
  // Switch back to MemPstate context 0
  {{11, 3, 1}, DO_NOT_CARE, 0, DO_NOT_CARE, DCT0_MEMPSTATE_MASK + DCT1_MEMPSTATE_MASK, ANY_DIMM_MASK},

  // Set Fence back to Fence of M0 to prepare for fine delay restore for M0
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x0C), 0x7FFF3FFF, DCT0_MEMPSTATE_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFDataFence2, 0, DCT0_MEMPSTATE_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFFence2, 0x00007C1F, DCT0_MEMPSTATE_MASK, ANY_DIMM_MASK},

  {{15, 0, 1}, S3_UMA_SIZE, 0, 0, ANY_DIMM_MASK, ANY_DIMM_MASK, RESTORE_TRAINING_MODE | CAPSULE_REBOOT_MODE},
  {{15, 0, 1}, S3_UMA_BASE, 0, 0, ANY_DIMM_MASK, ANY_DIMM_MASK, RESTORE_TRAINING_MODE | CAPSULE_REBOOT_MODE},
  {{15, 1, 1}, S3_UMA_MODE, 0, 0, ANY_DIMM_MASK, ANY_DIMM_MASK, RESTORE_TRAINING_MODE | CAPSULE_REBOOT_MODE},
  {{15, 0, 1}, S3_SUB_4G_CACHE_TOP, 0, 0, ANY_DIMM_MASK, ANY_DIMM_MASK, RESTORE_TRAINING_MODE | CAPSULE_REBOOT_MODE},
  {{15, 0, 1}, S3_SYSLIMIT, 0, 0, ANY_DIMM_MASK, ANY_DIMM_MASK, RESTORE_TRAINING_MODE | CAPSULE_REBOOT_MODE},
  // Always put UMA_ATTRIBUTE after other types for hob data save and restore
  {{15, 0, 1}, S3_UMA_ATTRIBUTE, 0, 0, ANY_DIMM_MASK, ANY_DIMM_MASK, RESTORE_TRAINING_MODE | CAPSULE_REBOOT_MODE},

  {{15, 1, 1}, S3_VDDIO, 0, 0, ANY_DIMM_MASK, ANY_DIMM_MASK, RESTORE_TRAINING_MODE | CAPSULE_REBOOT_MODE}
};

CONST CPCI_REGISTER_BLOCK_HEADER ROMDATA S3CPciPreSelfRefKB = {
  0,
  (sizeof (S3CPciPreSelfDescriptorKB) / sizeof (CONDITIONAL_PCI_REG_DESCRIPTOR)),
  S3CPciPreSelfDescriptorKB,
  PciSpecialCaseFuncKB
};

CONDITIONAL_PCI_REG_DESCRIPTOR ROMDATA S3CPciPostSelfDescriptorKB[] = {
  // DCT0
  {{12, 2, 1}, DCT0,  BFChAM1FenceSave, 0, DCT0_MEMPSTATE_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFRxDqInsDly, 0, DCT0_MASK, ANY_DIMM_MASK},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x10),  0x03FF03FF, DCT0_MASK, 0x01},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x11),  0x03FF03FF, DCT0_MASK, 0x01},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x13),  0x03FF03FF, DCT0_MASK, 0x04},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x14),  0x03FF03FF, DCT0_MASK, 0x04},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x16),  0x03FF03FF, DCT0_MASK, 0x10},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x17),  0x03FF03FF, DCT0_MASK, 0x10},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x19),  0x03FF03FF, DCT0_MASK, 0x40},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x1A),  0x03FF03FF, DCT0_MASK, 0x40},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x20),  0x03FF03FF, DCT0_MASK, 0x01},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x21),  0x03FF03FF, DCT0_MASK, 0x01},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x23),  0x03FF03FF, DCT0_MASK, 0x04},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x24),  0x03FF03FF, DCT0_MASK, 0x04},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x26),  0x03FF03FF, DCT0_MASK, 0x10},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x27),  0x03FF03FF, DCT0_MASK, 0x10},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x29),  0x03FF03FF, DCT0_MASK, 0x40},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x2A),  0x03FF03FF, DCT0_MASK, 0x40},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x01),  0xFFFFFFFF, DCT0_MASK, 0x01},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x02),  0xFFFFFFFF, DCT0_MASK, 0x01},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x101), 0xFFFFFFFF, DCT0_MASK, 0x04},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x102), 0xFFFFFFFF, DCT0_MASK, 0x04},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x201), 0xFFFFFFFF, DCT0_MASK, 0x10},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x202), 0xFFFFFFFF, DCT0_MASK, 0x10},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x301), 0xFFFFFFFF, DCT0_MASK, 0x40},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x302), 0xFFFFFFFF, DCT0_MASK, 0x40},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x05),  0x3E3E3E3E, DCT0_MASK, 0x01},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x06),  0x3E3E3E3E, DCT0_MASK, 0x01},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x105), 0x3E3E3E3E, DCT0_MASK, 0x04},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x106), 0x3E3E3E3E, DCT0_MASK, 0x04},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x205), 0x3E3E3E3E, DCT0_MASK, 0x10},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x206), 0x3E3E3E3E, DCT0_MASK, 0x10},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x305), 0x3E3E3E3E, DCT0_MASK, 0x40},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x306), 0x3E3E3E3E, DCT0_MASK, 0x40},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x30),  0x00FF00FF, DCT0_MASK, 0x01},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x31),  0x00FF00FF, DCT0_MASK, 0x01},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x33),  0x00FF00FF, DCT0_MASK, 0x04},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x34),  0x00FF00FF, DCT0_MASK, 0x04},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x36),  0x00FF00FF, DCT0_MASK, 0x10},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x37),  0x00FF00FF, DCT0_MASK, 0x10},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x39),  0x00FF00FF, DCT0_MASK, 0x40},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x3A),  0x00FF00FF, DCT0_MASK, 0x40},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x40),  0x00FF00FF, DCT0_MASK, 0x01},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x41),  0x00FF00FF, DCT0_MASK, 0x01},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x43),  0x00FF00FF, DCT0_MASK, 0x04},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x44),  0x00FF00FF, DCT0_MASK, 0x04},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x46),  0x00FF00FF, DCT0_MASK, 0x10},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x47),  0x00FF00FF, DCT0_MASK, 0x10},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x49),  0x00FF00FF, DCT0_MASK, 0x40},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x4A),  0x00FF00FF, DCT0_MASK, 0x40},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x0D),  0x037F037F, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 1, 1}, DCT0,   BFPhy0x0D0F0F13, 0x00000083, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFDllCSRBiasTrim, 0x00007000, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFRxSsbMntClkEn, 0, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFEnRxPadStandby, 0, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 1, 1}, DCT0,   BFEccDLLPwrDnConf, 0, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFTxPclkGateEn, 0, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 1, 1}, DCT0,   BFPchgPdPclkGateEn, 0, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 1, 1}, DCT0,   BFDataCtlPipePclkGateEn, 0, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFDataByteDllPowerMgnByte0, 0xBFBF, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFDataByteDllPowerMgnByte1, 0xBFBF, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFDataByteDllPowerMgnByte2, 0xBFBF, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFDataByteDllPowerMgnByte3, 0xBFBF, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFDataByteDllPowerMgnByte4, 0xBFBF, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFDataByteDllPowerMgnByte5, 0xBFBF, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFDataByteDllPowerMgnByte6, 0xBFBF, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFDataByteDllPowerMgnByte7, 0xBFBF, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFDataByteDllPowerMgnByte8, 0xBFBF, DCT0_MASK, ANY_DIMM_MASK},

  {{11, 3, 1}, DO_NOT_CARE, 1, DO_NOT_CARE, DCT0_MEMPSTATE_MASK + DCT1_MEMPSTATE_MASK, ANY_DIMM_MASK},

  // DCT0
  {{12, 2, 1}, DCT0,  BFChAM1FenceSave, 0, DCT0_MEMPSTATE_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFRxDqInsDly, 0, DCT0_MEMPSTATE_MASK, ANY_DIMM_MASK},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x10),  0x03FF03FF, DCT0_MEMPSTATE_MASK, 0x01},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x11),  0x03FF03FF, DCT0_MEMPSTATE_MASK, 0x01},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x13),  0x03FF03FF, DCT0_MEMPSTATE_MASK, 0x04},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x14),  0x03FF03FF, DCT0_MEMPSTATE_MASK, 0x04},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x16),  0x03FF03FF, DCT0_MEMPSTATE_MASK, 0x10},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x17),  0x03FF03FF, DCT0_MEMPSTATE_MASK, 0x10},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x19),  0x03FF03FF, DCT0_MEMPSTATE_MASK, 0x40},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x1A),  0x03FF03FF, DCT0_MEMPSTATE_MASK, 0x40},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x20),  0x03FF03FF, DCT0_MEMPSTATE_MASK, 0x01},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x21),  0x03FF03FF, DCT0_MEMPSTATE_MASK, 0x01},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x23),  0x03FF03FF, DCT0_MEMPSTATE_MASK, 0x04},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x24),  0x03FF03FF, DCT0_MEMPSTATE_MASK, 0x04},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x26),  0x03FF03FF, DCT0_MEMPSTATE_MASK, 0x10},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x27),  0x03FF03FF, DCT0_MEMPSTATE_MASK, 0x10},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x29),  0x03FF03FF, DCT0_MEMPSTATE_MASK, 0x40},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x2A),  0x03FF03FF, DCT0_MEMPSTATE_MASK, 0x40},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x01),  0xFFFFFFFF, DCT0_MEMPSTATE_MASK, 0x01},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x02),  0xFFFFFFFF, DCT0_MEMPSTATE_MASK, 0x01},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x101), 0xFFFFFFFF, DCT0_MEMPSTATE_MASK, 0x04},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x102), 0xFFFFFFFF, DCT0_MEMPSTATE_MASK, 0x04},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x201), 0xFFFFFFFF, DCT0_MEMPSTATE_MASK, 0x10},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x202), 0xFFFFFFFF, DCT0_MEMPSTATE_MASK, 0x10},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x301), 0xFFFFFFFF, DCT0_MEMPSTATE_MASK, 0x40},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x302), 0xFFFFFFFF, DCT0_MEMPSTATE_MASK, 0x40},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x05),  0x3E3E3E3E, DCT0_MEMPSTATE_MASK, 0x01},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x06),  0x3E3E3E3E, DCT0_MEMPSTATE_MASK, 0x01},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x105), 0x3E3E3E3E, DCT0_MEMPSTATE_MASK, 0x04},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x106), 0x3E3E3E3E, DCT0_MEMPSTATE_MASK, 0x04},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x205), 0x3E3E3E3E, DCT0_MEMPSTATE_MASK, 0x10},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x206), 0x3E3E3E3E, DCT0_MEMPSTATE_MASK, 0x10},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x305), 0x3E3E3E3E, DCT0_MEMPSTATE_MASK, 0x40},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x306), 0x3E3E3E3E, DCT0_MEMPSTATE_MASK, 0x40},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x30),  0x00FF00FF, DCT0_MEMPSTATE_MASK, 0x01},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x31),  0x00FF00FF, DCT0_MEMPSTATE_MASK, 0x01},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x33),  0x00FF00FF, DCT0_MEMPSTATE_MASK, 0x04},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x34),  0x00FF00FF, DCT0_MEMPSTATE_MASK, 0x04},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x36),  0x00FF00FF, DCT0_MEMPSTATE_MASK, 0x10},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x37),  0x00FF00FF, DCT0_MEMPSTATE_MASK, 0x10},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x39),  0x00FF00FF, DCT0_MEMPSTATE_MASK, 0x40},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x3A),  0x00FF00FF, DCT0_MEMPSTATE_MASK, 0x40},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x40),  0x00FF00FF, DCT0_MEMPSTATE_MASK, 0x01},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x41),  0x00FF00FF, DCT0_MEMPSTATE_MASK, 0x01},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x43),  0x00FF00FF, DCT0_MEMPSTATE_MASK, 0x04},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x44),  0x00FF00FF, DCT0_MEMPSTATE_MASK, 0x04},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x46),  0x00FF00FF, DCT0_MEMPSTATE_MASK, 0x10},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x47),  0x00FF00FF, DCT0_MEMPSTATE_MASK, 0x10},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x49),  0x00FF00FF, DCT0_MEMPSTATE_MASK, 0x40},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x4A),  0x00FF00FF, DCT0_MEMPSTATE_MASK, 0x40},
  {{0, 0, 1}, FUNC_2, SET_S3_SPECIAL_OFFSET (DCT_PHY_FLAG, 0, 0x0D),  0x037F037F, DCT0_MEMPSTATE_MASK, ANY_DIMM_MASK},
  {{1, 1, 1}, DCT0,   BFPhy0x0D0F0F13, 0x00000083, DCT0_MEMPSTATE_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFDllCSRBiasTrim, 0x00007000, DCT0_MEMPSTATE_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFRxSsbMntClkEn, 0, DCT0_MEMPSTATE_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFEnRxPadStandby, 0, DCT0_MEMPSTATE_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFDataByteDllPowerMgnByte0, 0xBFBF, DCT0_MEMPSTATE_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFDataByteDllPowerMgnByte1, 0xBFBF, DCT0_MEMPSTATE_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFDataByteDllPowerMgnByte2, 0xBFBF, DCT0_MEMPSTATE_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFDataByteDllPowerMgnByte3, 0xBFBF, DCT0_MEMPSTATE_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFDataByteDllPowerMgnByte4, 0xBFBF, DCT0_MEMPSTATE_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFDataByteDllPowerMgnByte5, 0xBFBF, DCT0_MEMPSTATE_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFDataByteDllPowerMgnByte6, 0xBFBF, DCT0_MEMPSTATE_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFDataByteDllPowerMgnByte7, 0xBFBF, DCT0_MEMPSTATE_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFDataByteDllPowerMgnByte8, 0xBFBF, DCT0_MEMPSTATE_MASK, ANY_DIMM_MASK},

  {{11, 3, 1}, DO_NOT_CARE, 0, DO_NOT_CARE, DCT0_MEMPSTATE_MASK + DCT1_MEMPSTATE_MASK, ANY_DIMM_MASK},

  {{1, 1, 1}, DCT0,   BFAddrCmdTri, 0x0000000A1, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 1, 1}, DCT0,   BFDisDllShutdownSR, 0, DCT0_MASK, ANY_DIMM_MASK},

  {{1, 1, 1}, DCT0,   BFVrefSel, 0x00000007, DCT0_MASK, ANY_DIMM_MASK},
  {{1, 2, 1}, DCT0,   BFVrefDAC, 0x000001F8, DCT0_MASK, ANY_DIMM_MASK},

  {{0, 0, 0}, FUNC_2, 0x1B4, 0x08000000, ANY_DIMM_MASK, ANY_DIMM_MASK},
  {{0, 0, 0}, FUNC_3, 0x180, 0x02000000, ANY_DIMM_MASK, ANY_DIMM_MASK},
  {{0, 0, 0}, FUNC_3, 0x58,  0x0000001F, ANY_DIMM_MASK, ANY_DIMM_MASK},
  {{0, 0, 0}, FUNC_3, 0x5C,  0x00000001, ANY_DIMM_MASK, ANY_DIMM_MASK},
  {{0, 0, 0}, FUNC_3, 0x44,  0x00400004, ANY_DIMM_MASK, ANY_DIMM_MASK},

  {{0, 0, 0}, FUNC_2, 0x118, 0x00040000, ANY_DIMM_MASK, ANY_DIMM_MASK},
  {{0, 0, 0}, FUNC_2, 0x118, 0x00080000, ANY_DIMM_MASK, ANY_DIMM_MASK},

  {{13, 3, 1}, DO_NOT_CARE, 0, DO_NOT_CARE, ANY_DIMM_MASK, ANY_DIMM_MASK}
};

CONST CPCI_REGISTER_BLOCK_HEADER ROMDATA S3CPciPostSelfRefKB = {
  0,
  (sizeof (S3CPciPostSelfDescriptorKB) / sizeof (CONDITIONAL_PCI_REG_DESCRIPTOR)),
  S3CPciPostSelfDescriptorKB,
  PciSpecialCaseFuncKB
};

MSR_REG_DESCRIPTOR ROMDATA S3MSRPreSelfRefDescriptorKB[] = {
  {{0, 0, 0}, 0xC0010010, 0x00000000007F0000},
  {{0, 0, 0}, 0xC001001A, 0x0000FFFFFF800000},
  {{0, 0, 0}, 0xC001001D, 0x0000FFFFFF800000},
  {{0, 0, 0}, 0xC001001F, 0x0044601080000600},
  {{0, 0, 1}, 0, 0, RESTORE_TRAINING_MODE | CAPSULE_REBOOT_MODE},
  {{0, 0, 0}, 0x250, 0x1F1F1F1F1F1F1F1F, RESTORE_TRAINING_MODE | CAPSULE_REBOOT_MODE},
  {{0, 0, 0}, 0x258, 0x1F1F1F1F1F1F1F1F, RESTORE_TRAINING_MODE | CAPSULE_REBOOT_MODE},
  {{0, 0, 1}, 1, 0, RESTORE_TRAINING_MODE | CAPSULE_REBOOT_MODE},
  {{0, 0, 0}, 0x00000200, 0x0000FFFFFFFFF007, RESTORE_TRAINING_MODE | CAPSULE_REBOOT_MODE},
  {{0, 0, 0}, 0x00000202, 0x0000FFFFFFFFF007, RESTORE_TRAINING_MODE | CAPSULE_REBOOT_MODE},
  {{0, 0, 0}, 0x00000204, 0x0000FFFFFFFFF007, RESTORE_TRAINING_MODE | CAPSULE_REBOOT_MODE},
  {{0, 0, 0}, 0x00000206, 0x0000FFFFFFFFF007, RESTORE_TRAINING_MODE | CAPSULE_REBOOT_MODE},
  {{0, 0, 0}, 0x00000208, 0x0000FFFFFFFFF007, RESTORE_TRAINING_MODE | CAPSULE_REBOOT_MODE},
  {{0, 0, 0}, 0x0000020A, 0x0000FFFFFFFFF007, RESTORE_TRAINING_MODE | CAPSULE_REBOOT_MODE},
  {{0, 0, 0}, 0x00000201, 0x0000FFFFFFFFF800, RESTORE_TRAINING_MODE | CAPSULE_REBOOT_MODE},
  {{0, 0, 0}, 0x00000203, 0x0000FFFFFFFFF800, RESTORE_TRAINING_MODE | CAPSULE_REBOOT_MODE},
  {{0, 0, 0}, 0x00000205, 0x0000FFFFFFFFF800, RESTORE_TRAINING_MODE | CAPSULE_REBOOT_MODE},
  {{0, 0, 0}, 0x00000207, 0x0000FFFFFFFFF800, RESTORE_TRAINING_MODE | CAPSULE_REBOOT_MODE},
  {{0, 0, 0}, 0x00000209, 0x0000FFFFFFFFF800, RESTORE_TRAINING_MODE | CAPSULE_REBOOT_MODE},
  {{0, 0, 0}, 0x0000020B, 0x0000FFFFFFFFF800, RESTORE_TRAINING_MODE | CAPSULE_REBOOT_MODE}
};

CONST MSR_REGISTER_BLOCK_HEADER ROMDATA S3MSRPreSelfRefKB = {
  0,
  (sizeof (S3MSRPreSelfRefDescriptorKB) / sizeof (MSR_REG_DESCRIPTOR)),
  S3MSRPreSelfRefDescriptorKB,
  MsrSpecialCaseFuncKB
};

VOID *MemS3RegListKB[] = {
  (VOID *)&S3PciPreSelfRefKB,
  NULL,
  (VOID *)&S3CPciPreSelfRefKB,
  (VOID *)&S3CPciPostSelfRefKB,
  (VOID *)&S3MSRPreSelfRefKB,
  NULL,
  NULL,
  NULL
};

/*----------------------------------------------------------------------------
 *                            EXPORTED FUNCTIONS
 *
 *----------------------------------------------------------------------------
 */
/* -----------------------------------------------------------------------------*/
/**
 *
 *
 *   This function initializes the northbridge block for S3 resume
 *
 *     @param[in,out]   *S3NBPtr   - Pointer to MEM_NB_BLOCK.
 *     @param[in,out]   *MemPtr  - Pointer to MEM_DATA_STRUCT.
 *     @param[in]       NodeID   - Node ID of the target node.
 *
 *      @return         BOOLEAN
 *                         TRUE - This is the correct constructor for the targeted node.
 *                         FALSE - This isn't the correct constructor for the targeted node.
 */
BOOLEAN
MemS3ResumeConstructNBBlockKB (
  IN OUT   VOID *S3NBPtr,
  IN OUT   MEM_DATA_STRUCT *MemPtr,
  IN       UINT8 NodeID
  )
{
  INT32 i;
  MEM_NB_BLOCK *NBPtr;

  NBPtr = ((S3_MEM_NB_BLOCK *)S3NBPtr)->NBPtr;

  //
  // Determine if this is the expected NB Type
  //
  GetLogicalIdOfSocket (MemPtr->DiesPerSystem[NodeID].SocketId, &(MemPtr->DiesPerSystem[NodeID].LogicalCpuid), &(MemPtr->StdHeader));
  if (!MemNIsIdSupportedKB (&(MemPtr->DiesPerSystem[NodeID].LogicalCpuid))) {
    return FALSE;
  }

  NBPtr->RefPtr = NULL;
  NBPtr->MemPtr = MemPtr;
  NBPtr->MCTPtr = &(MemPtr->DiesPerSystem[NodeID]);
  NBPtr->PciAddr.AddressValue = MemPtr->DiesPerSystem[NodeID].PciAddr.AddressValue;
  MemNInitNBRegTableKB (NBPtr, NBPtr->NBRegTable);
  NBPtr->Node = ((UINT8) NBPtr->PciAddr.Address.Device) - 24;
  NBPtr->Dct = 0;
  NBPtr->Channel = 0;
  NBPtr->Ganged = FALSE;
  NBPtr->NodeCount = MAX_NODES_SUPPORTED_KB;
  NBPtr->DctCount = MAX_DCTS_PER_NODE_KB;
  NBPtr->MemPstate = MEMORY_PSTATE0;
  NBPtr->MemPstateStage = 0;
  NBPtr->NbPsCtlReg = 0;

  NBPtr->IsSupported[SetDllShutDown] = TRUE;

  for (i = 0; i < EnumSize; i++) {
    NBPtr->IsSupported[i] = FALSE;
  }

  for (i = 0; i < NumberOfHooks; i++) {
    NBPtr->FamilySpecificHook[i] = (BOOLEAN (*) (MEM_NB_BLOCK *, VOID *)) memDefTrue;
  }

  LibAmdMemFill (NBPtr->DctCache, 0, sizeof (NBPtr->DctCache), &MemPtr->StdHeader);

  NBPtr->SwitchDCT = MemNSwitchDCTNb;
  NBPtr->SwitchChannel = MemNSwitchChannelNb;
  NBPtr->MemNCmnGetSetFieldNb = MemNCmnGetSetFieldKB;
  NBPtr->GetBitField = MemNGetBitFieldNb;
  NBPtr->SetBitField = MemNSetBitFieldNb;
  NBPtr->MemNIsIdSupportedNb = MemNIsIdSupportedKB;
  ((S3_MEM_NB_BLOCK *)S3NBPtr)->MemS3ExitSelfRefReg = (VOID (*) (MEM_NB_BLOCK *, AMD_CONFIG_PARAMS *)) memDefRet;
  ((S3_MEM_NB_BLOCK *)S3NBPtr)->MemS3GetConPCIMask = MemNS3GetConPCIMaskKB;
  ((S3_MEM_NB_BLOCK *)S3NBPtr)->MemS3GetConMSRMask = (VOID (*) (MEM_NB_BLOCK *, DESCRIPTOR_GROUP *)) memDefRet;
  ((S3_MEM_NB_BLOCK *)S3NBPtr)->MemS3Resume = MemNS3ResumeUNb;
  ((S3_MEM_NB_BLOCK *)S3NBPtr)->MemS3RestoreScrub = MemNS3RestoreScrubNb;
  ((S3_MEM_NB_BLOCK *)S3NBPtr)->MemS3GetRegLstPtr = MemNS3GetRegLstPtrKB;
  ((S3_MEM_NB_BLOCK *)S3NBPtr)->MemS3GetDeviceRegLst = MemNS3GetDeviceRegLstKB;
  ((S3_MEM_NB_BLOCK *)S3NBPtr)->MemS3SpecialCaseHeapSize = 0;
  NBPtr->FamilySpecificHook[DCTSelectSwitch] = MemNS3DctCfgSelectUnb;

  MemNSetBitFieldNb (NBPtr, BFDctCfgSel, 0);
  MemNSwitchDCTNb (NBPtr, 0);
  MemNSetBitFieldNb (NBPtr, BFMemPsSel, 0);

  if (MemNGetBitFieldNb (NBPtr, BFMemPstateDis) != 1) {
    NBPtr->MemPstateStage = MEMORY_PSTATE_S3_STAGE;
    MemNBrdcstSetUnConditionalNb (NBPtr, BFPStateToAccess, 0);
  }

  return TRUE;
}

/*----------------------------------------------------------------------------
 *                              LOCAL FUNCTIONS
 *
 *----------------------------------------------------------------------------*/

/* -----------------------------------------------------------------------------*/
/**
 *
 *
 *   This function returns the register list for each device for KB
 *
 *     @param[in,out]   *NBPtr   - Pointer to the MEM_NB_BLOCK
 *     @param[in, out]  *DescriptPtr - Pointer to DESCRIPTOR_GROUP
 *     @return          UINT16 - size of the device descriptor on the target node.
 */
UINT16
STATIC
MemNS3GetRegLstPtrKB (
  IN OUT   MEM_NB_BLOCK *NBPtr,
  IN OUT   DESCRIPTOR_GROUP *DescriptPtr
  )
{
  UINT8 i;
  UINT16 Size;
  Size = 0;
  for (i = PRESELFREF; i <= POSTSELFREF; i ++) {
    DescriptPtr->PCIDevice[i].Type = (UINT8) (DEV_TYPE_PCI_PRE_ESR + i);
    DescriptPtr->PCIDevice[i].Node = NBPtr->Node;
    DescriptPtr->PCIDevice[i].RegisterListID = 0xFFFFFFFF;
    if ((PCI_REGISTER_BLOCK_HEADER *) MemS3RegListKB[PCI_LST_ESR_KB - PCI_LST_ESR_KB + i] != NULL) {
      DescriptPtr->PCIDevice[i].RegisterListID = PCI_LST_ESR_KB + i;
      Size += sizeof (PCI_DEVICE_DESCRIPTOR);
    }
    DescriptPtr->CPCIDevice[i].Type = (UINT8) (DEV_TYPE_CPCI_PRE_ESR + i);
    DescriptPtr->CPCIDevice[i].Node = NBPtr->Node;
    DescriptPtr->CPCIDevice[i].RegisterListID = 0xFFFFFFFF;
    if ((CPCI_REGISTER_BLOCK_HEADER *) MemS3RegListKB[CPCI_LST_ESR_KB - PCI_LST_ESR_KB + i] != NULL) {
      DescriptPtr->CPCIDevice[i].RegisterListID = CPCI_LST_ESR_KB + i;
      Size += sizeof (CONDITIONAL_PCI_DEVICE_DESCRIPTOR);
    }
    DescriptPtr->MSRDevice[i].Type = (UINT8) (DEV_TYPE_MSR_PRE_ESR + i);
    DescriptPtr->MSRDevice[i].RegisterListID = 0xFFFFFFFF;
    if ((MSR_REGISTER_BLOCK_HEADER *) MemS3RegListKB[MSR_LST_ESR_KB - PCI_LST_ESR_KB + i] != NULL) {
      DescriptPtr->MSRDevice[i].RegisterListID = MSR_LST_ESR_KB + i;
      Size += sizeof (MSR_DEVICE_DESCRIPTOR);
    }
    DescriptPtr->CMSRDevice[i].Type = (UINT8) (DEV_TYPE_CMSR_PRE_ESR + i);
    DescriptPtr->CMSRDevice[i].RegisterListID = 0xFFFFFFFF;
    if ((CMSR_REGISTER_BLOCK_HEADER *) MemS3RegListKB[CMSR_LST_ESR_KB - PCI_LST_ESR_KB + i] != NULL) {
      DescriptPtr->CMSRDevice[i].RegisterListID = CMSR_LST_ESR_KB + i;
      Size += sizeof (CONDITIONAL_MSR_DEVICE_DESCRIPTOR);
    }
  }
  return Size;
}

/* -----------------------------------------------------------------------------*/
/**
 *
 *
 *   This function return the register list according to the register ID.
 *
 *     @param[in]   RegisterLstID - value of the Register list ID.
 *     @param[out]  **RegisterHeader - pointer to the address of the register list.
 *     @return      AGESA_STATUS
 *                          - AGESA_FATAL
 *                          - AGESA_SUCCESS
 */
AGESA_STATUS
STATIC
MemNS3GetDeviceRegLstKB (
  IN       UINT32 RegisterLstID,
     OUT   VOID **RegisterHeader
  )
{
  if (RegisterLstID >= (sizeof (MemS3RegListKB) / sizeof (VOID *))) {
    ASSERT(FALSE); // RegisterListID exceeded size of Register list
    return AGESA_FATAL;
  }
  if (MemS3RegListKB[RegisterLstID] != NULL) {
    *RegisterHeader = MemS3RegListKB[RegisterLstID];
    return AGESA_SUCCESS;
  }
  ASSERT(FALSE); // Device register list error
  return AGESA_FATAL;
}

/* -----------------------------------------------------------------------------*/
/**
 *
 *
 *   This function that set PllLockTime or disable auto compensation.
 *
 *     @param[in]   AccessWidth - Access width of the register.
 *     @param[in]   Address - address in PCI_ADDR format.
 *     @param[in, out]  *Value - Pointer to the value to be written.
 *     @param[in, out]  *ConfigPtr - Pointer to Config handle.
 *     @return         none
 */
VOID
STATIC
MemNS3SetDfltPhyRegKB (
  IN       ACCESS_WIDTH AccessWidth,
  IN       PCI_ADDR Address,
  IN       VOID *Value,
  IN OUT   VOID *ConfigPtr
  )
{
  UINT16 RegValue;
  BIT_FIELD_NAME BitField;

  IDS_SKIP_HOOK (IDS_BEFORE_S3_SPECIAL, &Address, ConfigPtr) {
    BitField = (BIT_FIELD_NAME) Address.Address.Register;
    RegValue = 0;

    if (BitField == BFPllLockTime) {
      RegValue = 0x190;
    } else if (BitField == BFDisablePredriverCal) {
      RegValue = 3;
    } else {
      ASSERT (FALSE);
    }
    MemNS3SetBitFieldNb (AccessS3SaveWidth16, Address, &RegValue, ConfigPtr);
  }
}

/* -----------------------------------------------------------------------------*/
/**
 *
 *
 *   This function sets bit 31 [DynModeChange] of F2x9C_xB
 *
 *     @param[in]   AccessWidth - Access width of the register.
 *     @param[in]   Address - address in PCI_ADDR format.
 *     @param[in, out]  *Value - Pointer to the value to be written.
 *     @param[in, out]  *ConfigPtr - Pointer to Config handle.
 *     @return         none
 */
VOID
STATIC
MemNS3SetDynModeChangeKB (
  IN       ACCESS_WIDTH AccessWidth,
  IN       PCI_ADDR Address,
  IN OUT   VOID *Value,
  IN OUT   VOID *ConfigPtr
  )
{
  UINT32 RegValue;

  IDS_SKIP_HOOK (IDS_BEFORE_S3_SPECIAL, &Address, ConfigPtr) {
    if ((Address.Address.Register & 0x400) == 0) {
      RegValue = 0x40000000;
      MemNS3SetCSRKB (AccessS3SaveWidth32, Address, &RegValue, ConfigPtr);
    }
    RegValue = 0x80000000;
    MemNS3SetCSRKB (AccessS3SaveWidth32, Address, &RegValue, ConfigPtr);
  }
}

/* -----------------------------------------------------------------------------*/
/**
 *
 *
 *   This function sets F2x9C_xB to 0x80800000
 *
 *     @param[in]   AccessWidth - Access width of the register.
 *     @param[in]   Address - address in PCI_ADDR format.
 *     @param[in, out]  *Value - Pointer to the value to be written.
 *     @param[in, out]  *ConfigPtr - Pointer to Config handle.
 *     @return         none
 */
VOID
STATIC
MemNS3SetPhyStatusRegKB (
  IN       ACCESS_WIDTH AccessWidth,
  IN       PCI_ADDR Address,
  IN OUT   VOID *Value,
  IN OUT   VOID *ConfigPtr
  )
{
  UINT32 RegValue;

  IDS_SKIP_HOOK (IDS_BEFORE_S3_SPECIAL, &Address, ConfigPtr) {
    RegValue = 0x80800000;
    MemNS3SetCSRKB (AccessS3SaveWidth32, Address, &RegValue, ConfigPtr);
  }
}

/* -----------------------------------------------------------------------------*/
/**
 *
 *
 *   This function does the channel disable sequence
 *
 *     @param[in]   AccessWidth - Access width of the register.
 *     @param[in]   Address - address in PCI_ADDR format.
 *     @param[in, out]  *Value - Pointer to the value to be written.
 *     @param[in, out]  *ConfigPtr - Pointer to Config handle.
 *     @return         none
 */
VOID
STATIC
MemNS3DisableChannelKB (
  IN       ACCESS_WIDTH AccessWidth,
  IN       PCI_ADDR Address,
  IN OUT   VOID *Value,
  IN OUT   VOID *ConfigPtr
  )
{
  MEM_NB_BLOCK *NBPtr;
  LOCATE_HEAP_PTR LocateBufferPtr;
  S3_MEM_NB_BLOCK *S3NBPtr;
  UINT32 RegValue;
  UINT8 Die;

  // See which Node should be accessed
  Die = (UINT8) (Address.Address.Device - 24);

  LocateBufferPtr.BufferHandle = AMD_MEM_S3_NB_HANDLE;
  if (HeapLocateBuffer (&LocateBufferPtr, ConfigPtr) == AGESA_SUCCESS) {
    S3NBPtr = (S3_MEM_NB_BLOCK *) LocateBufferPtr.BufferPtr;
    NBPtr = S3NBPtr[Die].NBPtr;

    // Function field contains the DCT number
    NBPtr->SwitchDCT (NBPtr, (UINT8) Address.Address.Function);
    RegValue = MemNGetBitFieldNb (NBPtr, BFCKETri);
    // if CKETri is 0b1111, this channel is disabled
    if (RegValue == 0xF) {
      //Wait for 24 MEMCLKs, which is 60ns under 400MHz
      MemFS3Wait10ns (6, NBPtr->MemPtr);
      MemNSetBitFieldNb (NBPtr, BFMemClkDis, 0xFF);
      MemNSetBitFieldNb (NBPtr, BFDisDramInterface, 1);
    }
  }
}

/* -----------------------------------------------------------------------------*/
/**
 *
 *
 *     This function changes memory Pstate context
 *
 *     @param[in]   AccessWidth - Access width of the register.
 *     @param[in]   Address - address in PCI_ADDR format.
 *     @param[in, out]  *Value - Pointer to the value to be written.
 *     @param[in, out]  *ConfigPtr - Pointer to Config handle.
 *
 *     @return    TRUE
 * ----------------------------------------------------------------------------
 */
VOID
STATIC
MemNS3ChangeMemPStateContextAndFlowNb (
  IN       ACCESS_WIDTH AccessWidth,
  IN       PCI_ADDR Address,
  IN OUT   VOID *Value,
  IN OUT   VOID *ConfigPtr
  )
{
  MEM_NB_BLOCK *NBPtr;
  LOCATE_HEAP_PTR LocateBufferPtr;
  S3_MEM_NB_BLOCK *S3NBPtr;
  UINT8 Die;

  // See which Node should be accessed
  Die = (UINT8) (Address.Address.Device - 24);

  LocateBufferPtr.BufferHandle = AMD_MEM_S3_NB_HANDLE;

  if (HeapLocateBuffer (&LocateBufferPtr, ConfigPtr) == AGESA_SUCCESS) {
    S3NBPtr = (S3_MEM_NB_BLOCK *) LocateBufferPtr.BufferPtr;
    NBPtr = S3NBPtr[Die].NBPtr;
    if (NBPtr->MemPstate == MEMORY_PSTATE0) {
      // If MemoryPstate is not disabled, switch to MemPState 1 context, and reprocess the register list
      MemNChangeMemPStateContextNb (NBPtr, 1);
      *(UINT32 *) Value = RESTART_FROM_BEGINNING_LIST;
    } else {
      // Switch back to MemPstate0 Context
      MemNChangeMemPStateContextNb (NBPtr, 0);
    }
  }
}

/* -----------------------------------------------------------------------------*/
/**
 *
 *
 *   This function returns the conditional PCI device mask
 *
 *     @param[in,out]   *NBPtr   - Pointer to the MEM_NB_BLOCK
 *     @param[in, out]  *DescriptPtr - Pointer to DESCRIPTOR_GROUP
 *      @return         none
 */
VOID
STATIC
MemNS3GetConPCIMaskKB (
  IN OUT   MEM_NB_BLOCK *NBPtr,
  IN OUT   DESCRIPTOR_GROUP *DescriptPtr
  )
{
  BIT_FIELD_NAME bitfield;
  UINT32 RegVal;
  UINT8 DCT;
  UINT8 DimmMask;
  UINT8 BadDimmMask;
  UINT8 NbPsCapMsk;
  UINT8 MemPstateMsk;
  UINT8 CsPerDelay;

  NbPsCapMsk = 0;
  MemPstateMsk = 0;
  DimmMask = 0;
  BadDimmMask = 0;
  CsPerDelay = 1;

  for (DCT = 0; DCT < NBPtr->DctCount; DCT ++) {
    MemNSwitchDCTNb (NBPtr, DCT);
    if (MemNGetBitFieldNb (NBPtr, BFMemClkFreqVal)) {
      if (MemNGetBitFieldNb (NBPtr, BFPerRankTimingEn) == 0) {
        CsPerDelay = 2;
      }
      for (bitfield = BFCSBaseAddr0Reg; bitfield <= BFCSBaseAddr7Reg; bitfield ++) {
        RegVal = MemNGetBitFieldNb (NBPtr, bitfield);
        if (RegVal & 0x1) {
          DimmMask |= (UINT8) (1 << ((((bitfield - BFCSBaseAddr0Reg) / CsPerDelay) << 1) + DCT));
        } else if (RegVal & 0x4) {
          BadDimmMask |= (UINT8) (1 << ((((bitfield - BFCSBaseAddr0Reg) / CsPerDelay) << 1) + DCT));
        }
      }
    }
  }
  // Check if the system is capable of doing NB Pstate change
  if (MemNGetBitFieldNb (NBPtr, BFNbPstateDis) == 0) {
    NbPsCapMsk = DCT0_NBPSTATE_SUPPORT_MASK;
  }
  if (MemNGetBitFieldNb (NBPtr, BFMemPstateDis) == 0) {
    MemPstateMsk = DCT0_MEMPSTATE_MASK;
  }

  MemNSwitchDCTNb (NBPtr, 0);
  // Set channel mask
  DescriptPtr->CPCIDevice[PRESELFREF].Mask1 = 0;
  DescriptPtr->CPCIDevice[POSTSELFREF].Mask1 = 0;
  for (DCT = 0; DCT < NBPtr->DctCount; DCT ++) {
    if (DimmMask & (0x55 << DCT)) {
      // Set mask before exit self refresh
      DescriptPtr->CPCIDevice[PRESELFREF].Mask1 |= (NbPsCapMsk | MemPstateMsk | 1) << DCT;
      // Set mask after exit self refresh
      DescriptPtr->CPCIDevice[POSTSELFREF].Mask1 |= (NbPsCapMsk | MemPstateMsk | 1) << DCT;
    } else if (BadDimmMask & (0x55 << DCT)) {
      // Need to save function 2 registers for bad dimm
      DescriptPtr->CPCIDevice[PRESELFREF].Mask1 |= 1 << DCT;
    }
  }

  // Set dimm mask
  DescriptPtr->CPCIDevice[PRESELFREF].Mask2 = DimmMask;
  DescriptPtr->CPCIDevice[POSTSELFREF].Mask2 = DimmMask;
}

/* -----------------------------------------------------------------------------*/
/**
 *
 *
 *   This function read the value of CSR register.
 *
 *     @param[in]   AccessWidth - Access width of the register
 *     @param[in]   Address - address of the CSR register in PCI_ADDR format.
 *     @param[in]  *Value - Pointer to the value be read.
 *     @param[in, out]  *ConfigPtr - Pointer to Config handle.
 *     @return         none
 */
VOID
STATIC
MemNS3GetCSRKB (
  IN       ACCESS_WIDTH AccessWidth,
  IN       PCI_ADDR Address,
  IN       VOID *Value,
  IN OUT   VOID *ConfigPtr
  )
{
  UINT8 TempValue;
  UINT8 Dct;
  UINT32 ExtendOffset;
  UINT32 TempFunc;

  ExtendOffset = Address.Address.Register;
  TempFunc = Address.Address.Function;

  // Switch Dct
  Address.Address.Function = FUNC_1;
  Address.Address.Register = 0x10C;
  Dct = 0;
  if (ExtendOffset & 0x400) {
    Dct = 1;
  }
  LibAmdPciRead (AccessS3SaveWidth8, Address, &TempValue, ConfigPtr);
  TempValue = (TempValue & 0xF8) | Dct;
  LibAmdPciWrite (AccessS3SaveWidth8, Address, &TempValue, ConfigPtr);
  Address.Address.Function = TempFunc;

  Address.Address.Register = 0x98;
  ExtendOffset &= 0x3FF;
  LibAmdPciWrite (AccessS3SaveWidth32, Address, &ExtendOffset, ConfigPtr);
  IDS_OPTION_HOOK (IDS_AFTER_DCT_PHY_ACCESS, NULL, ConfigPtr);
  Address.Address.Register = 0x9C;
  LibAmdPciRead (AccessWidth, Address, Value, ConfigPtr);
}

/* -----------------------------------------------------------------------------*/
/**
 *
 *
 *   This function write to a CSR register
 *
 *     @param[in]   AccessWidth - Access width of the register
 *     @param[in]   Address - address of the CSR register in PCI_ADDR format.
 *     @param[in, out]  *Value - Pointer to the value be read.
 *     @param[in, out]  *ConfigPtr - Pointer to Config handle.
 *     @return         none
 */
VOID
STATIC
MemNS3SetCSRKB (
  IN       ACCESS_WIDTH AccessWidth,
  IN       PCI_ADDR Address,
  IN OUT   VOID *Value,
  IN OUT   VOID *ConfigPtr
  )
{
  UINT8 TempValue;
  UINT8 Dct;
  UINT32 ExtendOffset;
  UINT32 ValueWrite;
  UINT32 TempFunc;

  ExtendOffset = Address.Address.Register;

  TempFunc = Address.Address.Function;
  // Switch Dct
  Address.Address.Function = FUNC_1;
  Address.Address.Register = 0x10C;
  Dct = 0;
  if (ExtendOffset & 0x400) {
    Dct = 1;
  }
  LibAmdPciRead (AccessS3SaveWidth8, Address, &TempValue, ConfigPtr);
  TempValue = (TempValue & 0xFE) | Dct;
  LibAmdPciWrite (AccessS3SaveWidth8, Address, &TempValue, ConfigPtr);

  Address.Address.Function = TempFunc;
  Address.Address.Register = 0x9C;

  ExtendOffset &= 0x3FF;
  ExtendOffset |= 0x40000000;
  switch (AccessWidth) {
  case AccessS3SaveWidth8:
    ValueWrite = *(UINT8 *) Value;
    break;
  case AccessS3SaveWidth16:
    ValueWrite = *(UINT16 *) Value;
    break;
  case AccessS3SaveWidth32:
    ValueWrite = *(UINT32 *) Value;
    break;
  default:
    ASSERT (FALSE);
  }
  LibAmdPciWrite (AccessS3SaveWidth32, Address, &ValueWrite, ConfigPtr);
  Address.Address.Register = 0x98;
  LibAmdPciWrite (AccessS3SaveWidth32, Address, &ExtendOffset, ConfigPtr);
  IDS_OPTION_HOOK (IDS_AFTER_DCT_PHY_ACCESS, NULL, ConfigPtr);
}

/* -----------------------------------------------------------------------------*/
/**
 *
 *
 *   This function that set Phy Fence
 *
 *     @param[in]   AccessWidth - Access width of the register.
 *     @param[in]   Address - address in PCI_ADDR format.
 *     @param[in, out]  *Value - Pointer to the value to be written.
 *     @param[in, out]  *ConfigPtr - Pointer to Config handle.
 *     @return         none
 */
VOID
STATIC
MemNS3SetPhyFenceKB (
  IN       ACCESS_WIDTH AccessWidth,
  IN       PCI_ADDR Address,
  IN       VOID *Value,
  IN OUT   VOID *ConfigPtr
  )
{
  UINT16 FenceValue;
  UINT16 Fence2Data;
  UINT16 Fence2Reg;
  BIT_FIELD_NAME BitField;
  MEM_NB_BLOCK *NBPtr;
  LOCATE_HEAP_PTR LocateBufferPtr;
  S3_MEM_NB_BLOCK *S3NBPtr;

  BitField = (BIT_FIELD_NAME) Address.Address.Register;
  FenceValue = *(UINT16 *) Value;
  // See which Node should be accessed

  LocateBufferPtr.BufferHandle = AMD_MEM_S3_NB_HANDLE;

  if (HeapLocateBuffer (&LocateBufferPtr, ConfigPtr) == AGESA_SUCCESS) {
    S3NBPtr = (S3_MEM_NB_BLOCK *) LocateBufferPtr.BufferPtr;
    NBPtr = S3NBPtr[0].NBPtr;

    // Do nothing if currently in memory pstate 0 context
    if (NBPtr->MemPstate == MEMORY_PSTATE0) {
      return;
    }

    MAKE_TSEFO (NBPtr->NBRegTable, DCT_PHY_ACCESS, 0x0C, 30, 16, BFPhyFence);
    MemNSetBitFieldNb (NBPtr, BFPhyFence, FenceValue);

    // Program Fence 2 for MState 1
    Fence2Data = 0;
    if ((FenceValue & 0x1F) < 16) {
      Fence2Data |= (FenceValue & 0x1F) | 0x10;
    }
    if (((FenceValue >> 5) & 0x1F) < 16) {
      Fence2Data |= (((FenceValue >> 5) & 0x1F) | 0x10) << 10;
    }
    if (((FenceValue >> 10) & 0x1F) < 16) {
      Fence2Data |= (((FenceValue >> 10) & 0x1F) | 0x10) << 5;
    }
    MemNSetBitFieldNb (NBPtr, BFDataFence2, Fence2Data);

    // Program another Fence 2 register for Mstate 1
    Fence2Reg = (UINT16) MemNGetBitFieldNb (NBPtr, BFFence2);
    Fence2Reg = (Fence2Reg &~(UINT16) ((0x1F << 10) | 0x1F)) | (Fence2Data & 0x1F) | (((Fence2Data >> 5) & 0x1F) << 10);
    MemNSetBitFieldNb (NBPtr, BFFence2, Fence2Reg);
  }
}
