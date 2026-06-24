
/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef SOUTHBRIDGE_INTEL_SPI_H
#define SOUTHBRIDGE_INTEL_SPI_H

#include <stdbool.h>
#include <stdint.h>

enum optype {
	READ_NO_ADDR = 0,
	WRITE_NO_ADDR = 1,
	READ_WITH_ADDR = 2,
	WRITE_WITH_ADDR = 3
};

struct intel_spi_op {
	u8 op;
	enum optype type;
};

struct intel_swseq_spi_config {
	u8 opprefixes[2];
	struct intel_spi_op ops[8];
};

struct intel_spi_vscc_config {
	uint32_t uvscc;
	uint32_t lvscc;
	bool lock;
};

void spi_finalize_ops(void);
void intel_southbridge_override_spi(struct intel_swseq_spi_config *spi_config);
void intel_southbridge_override_spi_vscc(struct intel_spi_vscc_config *vscc_config);

#endif
