/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_ZEPPELIN_NATIVE_FCH_H
#define AMD_ZEPPELIN_NATIVE_FCH_H

#include <device/pci_type.h>

/* fch.c */
void zeppelin_fch_reset_init(void);
void zeppelin_fch_env_init(void);
void zeppelin_fch_late_init(void);

/* fch_sata.c */
void zeppelin_fch_sata_reset(void);
void zeppelin_fch_sata_init(void);
void zeppelin_fch_sata_late(pci_devfn_t sata);

/* fch_usb.c */
void zeppelin_fch_usb_init(void);

#endif /* AMD_ZEPPELIN_NATIVE_FCH_H */
