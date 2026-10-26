/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_SP3_NATIVE_FCH_H
#define AMD_SP3_NATIVE_FCH_H

#include <device/pci_type.h>
#include <types.h>

/* fch.c */
void sp3_fch_reset_init(void);
void sp3_fch_env_init(void);
void sp3_fch_late_init(void);

/* fch_sata.c */
void sp3_fch_sata_reset(unsigned int die);
void sp3_fch_sata_init(unsigned int die);
void sp3_fch_sata_late(unsigned int die, pci_devfn_t sata);

/* fch_usb.c */
void sp3_fch_usb_init(unsigned int die);

/* fch_multi.c: die 0's FCH ACPI MMIO is at 0xfed80000, the others are moved */
uintptr_t sp3_fch_acpimmio(unsigned int die);
void sp3_fch_xgbe_init(unsigned int die);
void sp3_fch_multi_reset(void);
void sp3_fch_multi_usb(void);
void sp3_fch_multi_env(void);
void sp3_fch_multi_late(void);

#endif /* AMD_SP3_NATIVE_FCH_H */
