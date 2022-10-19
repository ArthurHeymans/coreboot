/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef _OPENSIL_H_
#define _OPENSIL_H_

void SIL_STATUS_report(const char *function, const int status);
// Add the memory map to dev, starting at index idx
void add_opensil_memmap(struct device *dev, int idx);

#endif
