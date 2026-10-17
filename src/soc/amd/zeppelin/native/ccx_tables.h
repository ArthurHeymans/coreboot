/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_ZEPPELIN_NATIVE_CCX_TABLES_H
#define AMD_ZEPPELIN_NATIVE_CCX_TABLES_H

#include <types.h>

/* Which threads apply a record */
enum zeppelin_ccx_scope {
	CCX_SCOPE_THREAD = 0,		/* every thread */
	CCX_SCOPE_CORE = 1,		/* the primary thread of each core */
	CCX_SCOPE_COMPLEX = 2,		/* the first thread of each core complex */
};

struct zeppelin_ccx_msr {
	uint8_t scope;
	uint32_t msr;
	uint64_t clear;
	uint64_t set;
	uint8_t rev;	/* logical revisions it applies to, 0: all */
};

extern const struct zeppelin_ccx_msr zeppelin_ccx_msr_table[];
extern const size_t zeppelin_ccx_msr_table_count;

#endif /* AMD_ZEPPELIN_NATIVE_CCX_TABLES_H */
