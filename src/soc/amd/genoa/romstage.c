/* SPDX-License-Identifier: GPL-2.0-only */

#include <cbmem.h>
#include <program_loading.h>
#include <romstage_common.h>

void __noreturn romstage_main(void)
{
	post_code(0x40);

	cbmem_initialize_empty();
	run_ramstage();
}
