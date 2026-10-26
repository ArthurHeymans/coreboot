/* SPDX-License-Identifier: GPL-2.0-only */

#include <soc/identity.h>

/* SP3 SoC PEIM gates and logical-ID tables; Documentation/soc/amd/sp3.md. */
bool sp3_identify(uint32_t eax, uint32_t ebx, struct sp3_identity *identity)
{
	struct sp3_identity found;
	const uint32_t family_model = eax & 0x0fff0f00;
	const uint8_t model_step = eax & 0xff;

	if (!identity || (ebx >> 28) != 4)
		return false;

	switch (family_model) {
	case 0x00800f00:
		found.generation = SP3_NAPLES;
		break;
	case 0x00830f00:
		found.generation = SP3_ROME;
		break;
	case 0x00a00f00:
		found.generation = SP3_MILAN;
		break;
	default:
		return false;
	}

	switch (model_step) {
	case 0x00:
		found.revision = SP3_A0;
		break;
	case 0x10:
		found.revision = SP3_B0;
		break;
	case 0x11:
		if (found.generation == SP3_ROME)
			return false;
		found.revision = SP3_B1;
		break;
	case 0x12:
		if (found.generation == SP3_ROME)
			return false;
		found.revision = SP3_B2;
		break;
	default:
		return false;
	}

	*identity = found;
	return true;
}
