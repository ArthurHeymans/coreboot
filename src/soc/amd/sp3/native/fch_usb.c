/* SPDX-License-Identifier: GPL-2.0-only */

#include <console/console.h>
#include "fch.h"
#include "topology.h"

void sp3_fch_usb_init(unsigned int socket)
{
	static const uint32_t requests[][4] = {
		{ 0x16d80128, 0x30003, 0, 0x1000001 },
		{ 0x16f80128, 0x30003, 0, 0x1000001 },
		{ 0x16d80120, 0xffff, 0xffff, 0x1000001 },
		{ 0x16f80120, 0xffff, 0x3232, 0x1000001 },
		{ 0x16d80114, 0x30003, 0, 0x1000001 },
		{ 0x16f80114, 0x30003, 0, 0x1000001 },
		{ 0x16d80118, 0x1000, 0, 0x1000001 },
		{ 0x16f80118, 0x1000, 0, 0x1000001 },
		{ 0x16d80130, 0x100, 0x100, 0x1000001 },
		{ 0x16f80130, 0x100, 0x100, 0x1000001 },
	};
	struct smu_payload p = { 0 };

	sp3_smn_clrsetbits32(0, 0x10534600, BIT(28), 0);
	sp3_smn_clrsetbits32(0, 0x10634600, BIT(28), 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(requests); i++) {
		p = (struct smu_payload) { .msg = {
			requests[i][0], requests[i][1], requests[i][2], requests[i][3],
		} };
		if (sp3_smu_send(0, 0x13, &p))
			die("SMU USB configuration failed\n");
	}
	p = (struct smu_payload) { 0 };
	if (sp3_smu_send(0, 0xa, &p))
		die("SMU USB initialization failed\n");
}
