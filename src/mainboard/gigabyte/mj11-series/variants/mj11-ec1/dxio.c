/* SPDX-License-Identifier: GPL-2.0-only */

#include <commonlib/bsd/helpers.h>
#include <variant/dxio.h>

/*
 * F09 AmdCpmOemInitPeim (7b8f8199-42c7-db87-307e-2892f3849538),
 * PEI $A26 at 0xffe09da0: engines 6 and 8 form the default x8/x8 pair.
 * This differs from explicit bifurcation mode 3, which uses engines 6/7.
 */
static const dxio_descriptor mj11_ec1_default[] = {
	MJ11_FIXED_ENGINES,
	MJ11_PCIE(31, 24, 3, 1),
	MJ11_PCIE(23, 16, 3, 3),
};

void variant_default_dxio(const dxio_descriptor **dxio, size_t *count)
{
	*dxio = mj11_ec1_default;
	*count = ARRAY_SIZE(mj11_ec1_default);
}
