
/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef SOUTHBRIDGE_INTEL_SPI_H
#define SOUTHBRIDGE_INTEL_SPI_H

#include <stdbool.h>
#include <stdint.h>

#define SPI_VSCC_ERASE_256B		(0 << 0)
#define SPI_VSCC_ERASE_4K		(1 << 0)
#define SPI_VSCC_ERASE_8K		(2 << 0)
#define SPI_VSCC_ERASE_64K		(3 << 0)
#define SPI_VSCC_WRITE_GRANULARITY_64B	(1 << 2)
#define SPI_VSCC_ERASE_OPCODE(opcode)	((opcode) << 8)
#define SPI_VSCC_ERASE_OPCODE_MASK	(0xff << 8)
#define SPI_VSCC_VCL			(1 << 23)

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
