/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/alib.h>
#include <amdblocks/smn.h>
#include <amdblocks/smu.h>

/*
 * ALIB function 1 (AC/DC state) and function 0xC (DPTC parameters) for the native path.
 *
 * With the FSP the ALIB method is provided by the SSDT that the FSP generates. Without it
 * coreboot has to provide the methods that the OS and the DPTC tables call at runtime. They
 * are implemented by sending the corresponding requests (AC/DC state, thermal limit, sustained,
 * fast and slow PPT limits) to the SMU BIOS mailbox from AML, as there is no firmware call
 * path to the SMU after boot. The mailbox registers are the same ones that
 * amdblocks/smu.h uses in coreboot, and are accessed with the SMN index/data pair in the host
 * bridge config space. The fields use the Lock rule, so the accesses are serialized through
 * the ACPI global lock. Waits are bounded so that a stuck SMU can't hang the AML interpreter.
 */
Scope (\_SB)
{
	OperationRegion (SMRG, SystemMemory, CONFIG_ECAM_MMCONF_BASE_ADDRESS, 0x1000)
	Field (SMRG, DWordAcc, Lock, Preserve)
	{
		Offset (SMN_INDEX_ADDR),
		SMIX, 32
	}

	BankField (SMRG, SMIX, SMN_SMU_MESG_ID, DWordAcc, Lock, Preserve)
	{
		Offset (SMN_DATA_ADDR),
		SMCM, 32
	}
	BankField (SMRG, SMIX, SMN_SMU_MESG_RESP, DWordAcc, Lock, Preserve)
	{
		Offset (SMN_DATA_ADDR),
		SMRS, 32
	}
	BankField (SMRG, SMIX, SMN_SMU_MESG_ARG(0), DWordAcc, Lock, Preserve)
	{
		Offset (SMN_DATA_ADDR),
		SMA0, 32
	}
	BankField (SMRG, SMIX, SMN_SMU_MESG_ARG(1), DWordAcc, Lock, Preserve)
	{
		Offset (SMN_DATA_ADDR),
		SMA1, 32
	}
	BankField (SMRG, SMIX, SMN_SMU_MESG_ARG(2), DWordAcc, Lock, Preserve)
	{
		Offset (SMN_DATA_ADDR),
		SMA2, 32
	}
	BankField (SMRG, SMIX, SMN_SMU_MESG_ARG(3), DWordAcc, Lock, Preserve)
	{
		Offset (SMN_DATA_ADDR),
		SMA3, 32
	}
	BankField (SMRG, SMIX, SMN_SMU_MESG_ARG(4), DWordAcc, Lock, Preserve)
	{
		Offset (SMN_DATA_ADDR),
		SMA4, 32
	}
	BankField (SMRG, SMIX, SMN_SMU_MESG_ARG(5), DWordAcc, Lock, Preserve)
	{
		Offset (SMN_DATA_ADDR),
		SMA5, 32
	}

	Method (SMSD, 2, Serialized)
	{
		/* Bound the waits so that a stuck SMU can't hang the AML interpreter. */
		SMRS = SMU_MESG_RESP_TIMEOUT
		Local0 = 10000
		While ((SMRS != SMU_MESG_RESP_TIMEOUT) && Local0)
		{
			Stall (1)
			Local0--
		}
		If (Local0 == Zero)
		{
			Return (Ones)
		}

		SMA0 = Arg1
		SMA1 = Zero
		SMA2 = Zero
		SMA3 = Zero
		SMA4 = Zero
		SMA5 = Zero
		SMCM = Arg0

		Local0 = 10000
		While ((SMRS == SMU_MESG_RESP_TIMEOUT) && Local0)
		{
			Stall (1)
			Local0--
		}
		If (Local0 == Zero)
		{
			Return (Ones)
		}
		If (SMRS == SMU_MESG_RESP_OK)
		{
			Return (Zero)
		}
		Return (SMRS)
	}

	Method (ALIB, 2, Serialized)
	{
		If (Arg0 == ALIB_FUNCTION_REPORT_AC_DC_STATE)
		{
			If (SizeOf (Arg1) < 3)
			{
				Return (Ones)
			}
			/* ALIB uses AC=0 and DC=1. */
			If (DerefOf (Arg1 [2]) == Zero)
			{
				Return (SMSD (SMC_MSG_SET_AC_MODE, Zero))
			}
			Return (SMSD (SMC_MSG_SET_DC_MODE, Zero))
		}

		If (Arg0 == ALIB_FUNCTION_DYNAMIC_POWER_THERMAL_CONFIG)
		{
			If (SizeOf (Arg1) < 2)
			{
				Return (Ones)
			}
			CreateWordField (Arg1, Zero, DSIZ)
			If ((DSIZ < 2) || (DSIZ > SizeOf (Arg1)) || ((DSIZ - 2) % 5))
			{
				Return (Ones)
			}

			Local0 = 2
			Local4 = Zero
			While (Local0 < DSIZ)
			{
				Local1 = DerefOf (Arg1 [Local0])
				Local0++
				Local2 = DerefOf (Arg1 [Local0])
				Local0++
				Local2 |= DerefOf (Arg1 [Local0]) << 8
				Local0++
				Local2 |= DerefOf (Arg1 [Local0]) << 16
				Local0++
				Local2 |= DerefOf (Arg1 [Local0]) << 24
				Local0++

				/* Only the thermal limit and the power limits are supported. */
				Local3 = Zero
				If (Local1 == ALIB_DPTC_THERMAL_CONTROL_LIMIT_ID)
				{
					Local3 = SMSD (SMC_MSG_SET_THERMAL_CONTROL_LIMIT, Local2)
				}
				If (Local1 == ALIB_DPTC_SUSTAINED_POWER_LIMIT_ID)
				{
					Local3 = SMSD (SMC_MSG_SET_SUSTAINED_POWER_LIMIT, Local2)
				}
				If (Local1 == ALIB_DPTC_FAST_PPT_LIMIT_ID)
				{
					Local3 = SMSD (SMC_MSG_SET_FAST_PPT_LIMIT, Local2)
				}
				If (Local1 == ALIB_DPTC_SLOW_PPT_LIMIT_ID)
				{
					Local3 = SMSD (SMC_MSG_SET_SLOW_PPT_LIMIT, Local2)
				}
				/* A timeout means the command may still own the mailbox. Do not
				 * overwrite it with the next request. */
				If (Local3 == Ones)
				{
					Return (Local3)
				}
				If ((Local3 != Zero) && (Local4 == Zero))
				{
					Local4 = Local3
				}
			}
			Return (Local4)
		}
		Return (Zero)
	}
}
