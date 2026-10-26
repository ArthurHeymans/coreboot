/* SPDX-License-Identifier: GPL-2.0-only */

#include <amdblocks/mca.h>
#include <cpu/x86/msr.h>
#include <types.h>

#define SMCA_IPID(bank) (0xc0002005 + 16 * (bank))

/* HWID/McaType from SMCA IPID, not Zeppelin's fixed bank numbering. */
static const struct {
	uint16_t hwid;
	uint16_t type;
	const char *name;
} units[] = {
	{ 0xb0, 0, "Load-store unit" },
	{ 0xb0, 1, "Instruction fetch unit" },
	{ 0xb0, 2, "L2 cache unit" },
	{ 0xb0, 3, "Decode unit" },
	{ 0xb0, 5, "Execution unit" },
	{ 0xb0, 6, "Floating point unit" },
	{ 0xb0, 7, "L3 cache unit" },
	{ 0x96, 0, "UMC" },
	{ 0x96, 1, "UMC" },
	{ 0x01, 0, "SMU" },
	{ 0x01, 1, "SMU" },
	{ 0xff, 0, "PSP" },
	{ 0xff, 1, "PSP" },
	{ 0x05, 0, "Parameter block" },
	{ 0x2e, 0, "Coherent slave" },
	{ 0x2e, 1, "PIE" },
	{ 0x2e, 2, "Coherent slave" },
	{ 0x18, 0, "NBIO" },
	{ 0x46, 0, "PCIe" },
	{ 0xa8, 0, "SATA" },
};

bool mca_has_expected_bank_count(void)
{
	return mca_get_bank_count() == 28;
}

bool mca_is_valid_bank(unsigned int bank)
{
	return bank < mca_get_bank_count();
}

const char *mca_get_bank_name(unsigned int bank)
{
	if (!mca_is_valid_bank(bank))
		return "";
	const msr_t ipid = rdmsr(SMCA_IPID(bank));

	for (size_t i = 0; i < ARRAY_SIZE(units); i++)
		if (units[i].hwid == (ipid.hi & 0xfff) && units[i].type == ipid.hi >> 16)
			return units[i].name;
	return "Scalable MCA bank";
}
