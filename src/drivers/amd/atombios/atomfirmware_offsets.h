/* SPDX-License-Identifier: MIT */
/*
 * Subset of atomfirmware.h VBIOS offset constants needed by atom.c.
 * Extracted from Linux kernel drivers/gpu/drm/amd/include/atomfirmware.h.
 *
 * Copyright 2006-2007 Advanced Micro Devices, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef ATOMFIRMWARE_OFFSETS_H
#define ATOMFIRMWARE_OFFSETS_H

#define BIOS_ATOM_PREFIX   "ATOMBIOS"
#define BIOS_VERSION_PREFIX  "ATOMBIOSBK-AMD"
#define BIOS_STRING_LENGTH 43

enum atombios_image_offset {
	OFFSET_TO_ATOM_ROM_HEADER_POINTER          = 0x00000048,
	OFFSET_TO_ATOM_ROM_IMAGE_SIZE              = 0x00000002,
	OFFSET_TO_ATOMBIOS_ASIC_BUS_MEM_TYPE       = 0x94,
	MAXSIZE_OF_ATOMBIOS_ASIC_BUS_MEM_TYPE      = 20,
	OFFSET_TO_GET_ATOMBIOS_NUMBER_OF_STRINGS   = 0x2f,
	OFFSET_TO_GET_ATOMBIOS_STRING_START        = 0x6e,
	OFFSET_TO_VBIOS_PART_NUMBER                = 0x80,
	OFFSET_TO_VBIOS_DATE                       = 0x50,
};

#endif /* ATOMFIRMWARE_OFFSETS_H */
