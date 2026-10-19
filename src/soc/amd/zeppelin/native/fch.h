/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_ZEPPELIN_NATIVE_FCH_H
#define AMD_ZEPPELIN_NATIVE_FCH_H

#include <device/pci_type.h>
#include <types.h>

/* fch.c */
void zeppelin_fch_reset_init(void);
void zeppelin_fch_env_init(void);
void zeppelin_fch_late_init(void);

/* fch_sata.c */
void zeppelin_fch_sata_reset(unsigned int die);
void zeppelin_fch_sata_init(unsigned int die);
void zeppelin_fch_sata_late(unsigned int die, pci_devfn_t sata);

/* fch_usb.c */
void zeppelin_fch_usb_init(unsigned int die);

/* fch_multi.c: die 0's FCH ACPI MMIO is at 0xfed80000, the others are moved */
uintptr_t zeppelin_fch_acpimmio(unsigned int die);
void zeppelin_fch_xgbe_init(unsigned int die);
void zeppelin_fch_multi_reset(void);
void zeppelin_fch_multi_usb(void);
void zeppelin_fch_multi_env(void);
void zeppelin_fch_multi_late(void);

#endif /* AMD_ZEPPELIN_NATIVE_FCH_H */
