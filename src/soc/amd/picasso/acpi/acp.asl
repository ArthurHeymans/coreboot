/* SPDX-License-Identifier: GPL-2.0-only */
/* ACP Audio Configuration */
Scope (\_SB.PCI0.GP41) {
	Device (ACPD) {
		/* Device addressing for ACP (Audio Coprocessor) */
		Name (_ADR, 0x05)  /* Device 0, Function 5 */

#if CONFIG(SOC_AMD_PICASSO_NATIVE)
		/* Native init powers off a disabled ACP. The ACP driver sets STAT when enabled. */
		Name (STAT, Zero)
#else
		Name (STAT, 0x3) /* Decoding Resources, Hide from UI */
#endif
		Method (_STA, 0x0, NotSerialized)
		{
			Return (STAT)
		}
	}
}
