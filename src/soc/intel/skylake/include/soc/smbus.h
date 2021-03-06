/*
 * This file is part of the coreboot project.
 *
 * Copyright (C) 2005 Yinghai Lu <yinghailu@gmail.com>
 * Copyright (C) 2009 coresystems GmbH
 * Copyright (C) 2014 Google Inc.
 * Copyright (C) 2015 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _SOC_SMBUS_H_
#define _SOC_SMBUS_H_

/* PCI Configuration Space (D31:F3): SMBus */
#define SMB_BASE		0x20
#define HOSTC			0x40
#define HST_EN			(1 << 0)
#define SMB_RCV_SLVA		0x09
/* SMBUS TCO base address. */
#define TCOBASE		0x50
#define TCOCTL		0x54
#define SMBUS_TCO_EN		(1 << 8)

/* TCO registers and fields live behind TCOBASE I/O bar in SMBus device. */
#define TCO1_STS			0x04
#define TCO2_STS			0x06
#define TCO2_STS_SECOND_TO		0x02
#define TCO2_STS_BOOT			0x04
#define TCO1_CNT			0x08
#define TCO_LOCK			(1 << 12)
#define TCO_TMR_HLT			(1 << 11)

/* SMBus I/O bits. */
#define SMBHSTSTAT		0x0
#define SMBHSTCTL		0x2
#define SMBHSTCMD		0x3
#define SMBXMITADD		0x4
#define SMBHSTDAT0		0x5
#define SMBHSTDAT1		0x6
#define SMBBLKDAT		0x7
#define SMBTRNSADD		0x9
#define SMBSLVDATA		0xa
#define SMLINK_PIN_CTL		0xe
#define SMBUS_PIN_CTL		0xf

#define SMBUS_TIMEOUT		(10 * 1000 * 100)
#define SMBUS_SLAVE_ADDR	0x24

int do_smbus_read_byte(unsigned int smbus_base, unsigned int device,
		       unsigned int address);
int do_smbus_write_byte(unsigned int smbus_base, unsigned int device,
			unsigned int address, unsigned int data);

#endif
