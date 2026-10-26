/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef SOC_AMD_SP3_IDENTITY_H
#define SOC_AMD_SP3_IDENTITY_H

#include <stdbool.h>
#include <stdint.h>

enum sp3_generation {
	SP3_NAPLES,
	SP3_ROME,
	SP3_MILAN,
};

enum sp3_revision {
	SP3_A0,
	SP3_B0,
	SP3_B1,
	SP3_B2,
};

struct sp3_identity {
	enum sp3_generation generation;
	enum sp3_revision revision;
};

/* Fn8000_0001 EAX/EBX. Reject unknown revisions and non-SP3 packages. */
bool sp3_identify(uint32_t eax, uint32_t ebx, struct sp3_identity *identity);

#endif /* SOC_AMD_SP3_IDENTITY_H */
