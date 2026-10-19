/* SPDX-License-Identifier: GPL-2.0-only */

#include <commonlib/bsd/helpers.h>
#include <variant/dxio.h>

/* F02 AmdCpmOemInitPeim $A26: the x16 slot as one port */
static const dxio_descriptor mj11_ec0_default[] = {
	MJ11_FIXED_ENGINES,
	MJ11_PCIE(31, 16, 3, 1),
};

void variant_default_dxio(const dxio_descriptor **dxio, size_t *count)
{
	*dxio = mj11_ec0_default;
	*count = ARRAY_SIZE(mj11_ec0_default);
}
