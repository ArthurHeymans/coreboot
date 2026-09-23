/* SPDX-License-Identifier: GPL-2.0-only */

#include <soc/amd/common/acpi/alib.asl>

#if CONFIG(SOC_AMD_PICASSO_NATIVE) && CONFIG(SOC_AMD_COMMON_BLOCK_ACPI_DPTC)
External (\_SB.DPTC, MethodObj)
#endif

Method (PNOT)
{
	/* Report AC/DC state to ALIB using WAL1() */
	\WAL1 ()

#if CONFIG(SOC_AMD_PICASSO_NATIVE) && CONFIG(SOC_AMD_COMMON_BLOCK_ACPI_DPTC)
	/* The native ALIB does not keep the DPTC parameters across S3, so
	   apply them again. */
	If (CondRefOf (\_SB.DPTC))
	{
		\_SB.DPTC ()
	}
#endif
}
