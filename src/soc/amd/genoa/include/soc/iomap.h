/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_GENOA_IOMAP_H
#define AMD_GENOA_IOMAP_H


#define FLASH_BASE_ADDR			((0xffffffff - CONFIG_ROM_SIZE) + 1)

/* @Todo : Check these values for Genoa */

/* I/O Ranges */
#define ACPI_IO_BASE			0x0400
#define  ACPI_CPU_CONTROL		(ACPI_IO_BASE + 0x10)

#endif /* AMD_GENOA_IOMAP_H */
