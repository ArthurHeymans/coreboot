/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_ZEPPELIN_I2C_H
#define AMD_ZEPPELIN_I2C_H

#include <types.h>

/* FCH I2C0..I2C4 and I2C5 */
#define ZEPPELIN_I2C_COUNT	6

/* Set up an FCH I2C controller like the vendor firmware (100 kHz) */
enum cb_err zeppelin_i2c_init(unsigned int bus);

#endif /* AMD_ZEPPELIN_I2C_H */
