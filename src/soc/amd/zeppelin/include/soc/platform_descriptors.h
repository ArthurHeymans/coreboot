/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_ZEPPELIN_PLATFORM_DESCRIPTORS_H
#define AMD_ZEPPELIN_PLATFORM_DESCRIPTORS_H

#include <types.h>
/* Zeppelin uses the Picasso DXIO descriptor format; there is no ZP FSP */
#include <vendorcode/amd/fsp/picasso/platform_descriptors.h>

/*
 * DXIO engines of die 0. Lanes 0-15 belong to PCIe core 0 (devices 1.x and
 * 2.1), lanes 16-31 to PCIe core 1 (devices 3.x and 4.1).
 */
void mainboard_get_dxio_descriptors(const dxio_descriptor **dxio, size_t *count);

#endif /* AMD_ZEPPELIN_PLATFORM_DESCRIPTORS_H */
