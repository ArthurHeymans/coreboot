/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_GENOA_SOUTHBRIDGE_H
#define AMD_GENOA_SOUTHBRIDGE_H

#include <soc/iomap.h>

#define FCH_LEGACY_UART_DECODE		(ALINK_AHB_ADDRESS + 0x20) /* 0xfedc0020 */

void fch_pre_init(void);

void enable_aoac_devices(void);
void wait_for_aoac_enabled(unsigned int dev);

#endif /* AMD_GENOA_SOUTHBRIDGE_H */
