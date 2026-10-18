/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef MJ11_BOARD_TABLES_H
#define MJ11_BOARD_TABLES_H

#include <types.h>

/* Written as value & mask when (read & mask) differs */
struct mj11_vr_reg {
	uint8_t reg;
	uint8_t value;
	uint8_t mask;
};

struct mj11_vr_image {
	uint8_t addr;		/* 7-bit, behind the PCA954x at 0x71 */
	uint8_t mux_channel;
	uint8_t lock_reg;	/* 0 to unlock, 7 to lock */
	const struct mj11_vr_reg *regs;
	size_t count;
};

struct mj11_smn_write {
	uint32_t reg;
	uint32_t value;
};

extern const struct mj11_vr_image mj11_vr_images[];
extern const size_t mj11_vr_image_count;

extern const struct mj11_smn_write mj11_post_training_smn[];
extern const size_t mj11_post_training_smn_count;

#endif /* MJ11_BOARD_TABLES_H */
