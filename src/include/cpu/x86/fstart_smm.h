/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef CPU_X86_FSTART_SMM_H
#define CPU_X86_FSTART_SMM_H

#include <stdint.h>

#define FSTART_SMM_IMAGE_MAGIC                       0x314d5346 /* "FSM1" little endian */
#define FSTART_SMM_IMAGE_VERSION                     1
#define FSTART_SMM_FLAG_COREBOOT_MODULE_ARGS         (1u << 0)
#define FSTART_SMM_MAX_CPUS                          64
#define FSTART_SMM_PLATFORM_NONE                     0u
#define FSTART_SMM_PLATFORM_INTEL_ICH                1u
#define FSTART_SMM_PLATFORM_DATA_ICH_PM_BASE         0u
#define FSTART_SMM_PLATFORM_DATA_ICH_GPE0_STS_OFFSET 1u

struct fstart_smm_image_header {
	uint32_t magic;
	uint16_t version;
	uint16_t header_size;
	uint32_t flags;
	uint32_t image_size;
	uint16_t entry_count;
	uint16_t entry_desc_size;
	uint32_t entries_offset;
	uint32_t common_offset;
	uint32_t common_size;
	uint32_t common_entry_offset;
	uint32_t runtime_offset;
	uint32_t module_args_offset;
	uint32_t module_args_size;
	uint32_t stack_size;
} __attribute__((packed));

struct fstart_smm_entry_descriptor {
	uint32_t stub_offset;
	uint32_t stub_size;
	uint32_t entry_offset;
	uint32_t params_offset;
} __attribute__((packed));

struct fstart_smm_entry_params {
	uint32_t cpu;
	uint32_t stack_size;
	uint64_t stack_top;
	uint64_t common_entry;
	uint64_t runtime;
	uint64_t coreboot_module_args;
	uint64_t cr3;
	uint64_t entry_base;
	uint32_t platform_kind;
	uint32_t platform_flags;
	uint64_t platform_data[4];
} __attribute__((packed));

struct fstart_smm_runtime {
	uint64_t smram_base;
	uint64_t smram_size;
	uint16_t num_cpus;
	uint16_t save_state_size;
	uint32_t stack_size;
	uint32_t common_offset;
	uint32_t entries_offset;
	uint64_t save_state_top[FSTART_SMM_MAX_CPUS];
	uint32_t flags;
	uint32_t handler_lock;
	uint32_t last_apm_command;
	uint32_t apm_command_counts[256];
	uint32_t cpu_entry_counts[FSTART_SMM_MAX_CPUS];
} __attribute__((packed));

struct fstart_smm_coreboot_module_args {
	uint64_t cpu;
	uint64_t canary;
} __attribute__((packed));

#endif /* CPU_X86_FSTART_SMM_H */
