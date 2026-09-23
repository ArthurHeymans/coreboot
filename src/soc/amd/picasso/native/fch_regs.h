/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_PICASSO_NATIVE_FCH_REGS_H
#define AMD_PICASSO_NATIVE_FCH_REGS_H

/* FCH MISC registers accessed through SMN, not ACPIMMIO. */
#define FCH_MISC_SMN(reg)	(0x02d01e00 + (reg))

/* SATA registers in the SMN aperture. */
#define SATA_SMN_BASE		0x03100000
#define SATA_SMN(reg)		(SATA_SMN_BASE + (reg))
#define SATA_SMN_RESET_STATUS	SATA_SMN(0x4)
#define SATA_SMN_PORT_STATUS(port)	SATA_SMN(0x1128 + (port) * 0x80)

#endif /* AMD_PICASSO_NATIVE_FCH_REGS_H */
