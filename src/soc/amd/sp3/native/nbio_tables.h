/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_SP3_NATIVE_NBIO_TABLES_H
#define AMD_SP3_NATIVE_NBIO_TABLES_H

#include <types.h>

enum sp3_nbio_op {
	NBIO_OP_RMW,	/* reg = (reg & ~clear) | set */
	NBIO_OP_COPY,	/* reg = SMN[src] */
};

struct sp3_nbio_reg {
	uint8_t op;
	uint32_t reg;
	uint32_t clear;	/* NBIO_OP_COPY: source register */
	uint32_t set;
};

#define NBIO_RMW(_reg, _clear, _set) \
	{ .op = NBIO_OP_RMW, .reg = _reg, .clear = _clear, .set = _set }
#define NBIO_WRITE(_reg, _value)	NBIO_RMW(_reg, UINT32_MAX, _value)
#define NBIO_COPY(_reg, _src) \
	{ .op = NBIO_OP_COPY, .reg = _reg, .clear = _src }

extern const struct sp3_nbio_reg sp3_nbio_socket_table[];
extern const size_t sp3_nbio_socket_table_count;
extern const struct sp3_nbio_reg sp3_nbio_base_table[];
extern const size_t sp3_nbio_base_table_count;
extern const struct sp3_nbio_reg sp3_nbio_iommu_table[];
extern const size_t sp3_nbio_iommu_table_count;

#endif /* AMD_SP3_NATIVE_NBIO_TABLES_H */
