/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_BLOCK_SMN_H
#define AMD_BLOCK_SMN_H

/* SMN index/data pair in the host bridge's PCI configuration space. */
#define SMN_INDEX_ADDR		0xb8 /* 32 bit */
#define SMN_DATA_ADDR		0xbc /* 32 bit */

#ifndef __ACPI__
#include <types.h>

uint32_t smn_read32(uint32_t reg);
uint64_t smn_read64(uint32_t reg);
void smn_write32(uint32_t reg, uint32_t val);

static inline void smn_clrsetbits32(uint32_t reg, uint32_t clear, uint32_t set)
{
	smn_write32(reg, (smn_read32(reg) & ~clear) | set);
}

#endif /* !__ACPI__ */

#endif /* AMD_BLOCK_SMN_H */
