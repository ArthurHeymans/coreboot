/* SPDX-License-Identifier: MIT */
/*
 * Minimal atomfirmware.h for coreboot AtomBIOS driver.
 *
 * Contains only the v2 master command and data table structures needed
 * for command table index computation on Vega+ (SoC15) GPUs. Extracted
 * from Linux kernel drivers/gpu/drm/amd/include/atomfirmware.h.
 *
 * Copyright 2014 Advanced Micro Devices, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef ATOMFIRMWARE_H
#define ATOMFIRMWARE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * v2 master command function table (atomfirmware.h format).
 * Field order differs from v1 (atombios.h). Used only for offsetof()
 * index computation -- never instantiated as a C struct.
 */
struct atom_master_list_of_command_functions_v2_1 {
	uint16_t asic_init;                    /* 0 */
	uint16_t cmd_function1;                /* 1 */
	uint16_t cmd_function2;                /* 2 */
	uint16_t cmd_function3;                /* 3 */
	uint16_t digxencodercontrol;           /* 4 */
	uint16_t cmd_function5;                /* 5 */
	uint16_t cmd_function6;                /* 6 */
	uint16_t cmd_function7;                /* 7 */
	uint16_t cmd_function8;                /* 8 */
	uint16_t cmd_function9;                /* 9 */
	uint16_t setengineclock;               /* 10 */
	uint16_t setmemoryclock;               /* 11 */
	uint16_t setpixelclock;                /* 12 */
	uint16_t enabledisppowergating;        /* 13 */
	uint16_t cmd_function14;               /* 14 */
	uint16_t cmd_function15;               /* 15 */
	uint16_t cmd_function16;               /* 16 */
	uint16_t cmd_function17;               /* 17 */
	uint16_t cmd_function18;               /* 18 */
	uint16_t cmd_function19;               /* 19 */
	uint16_t cmd_function20;               /* 20 */
	uint16_t cmd_function21;               /* 21 */
	uint16_t cmd_function22;               /* 22 */
	uint16_t cmd_function23;               /* 23 */
	uint16_t cmd_function24;               /* 24 */
	uint16_t cmd_function25;               /* 25 */
	uint16_t cmd_function26;               /* 26 */
	uint16_t cmd_function27;               /* 27 */
	uint16_t cmd_function28;               /* 28 */
	uint16_t cmd_function29;               /* 29 */
	uint16_t cmd_function30;               /* 30 */
	uint16_t cmd_function31;               /* 31 */
	uint16_t cmd_function32;               /* 32 */
	uint16_t cmd_function33;               /* 33 */
	uint16_t blankcrtc;                    /* 34 */
	uint16_t enablecrtc;                   /* 35 */
	uint16_t cmd_function36;               /* 36 */
	uint16_t cmd_function37;               /* 37 */
	uint16_t cmd_function38;               /* 38 */
	uint16_t cmd_function39;               /* 39 */
	uint16_t cmd_function40;               /* 40 */
	uint16_t getsmuclockinfo;              /* 41 */
	uint16_t selectcrtc_source;            /* 42 */
	uint16_t cmd_function43;               /* 43 */
	uint16_t cmd_function44;               /* 44 */
	uint16_t cmd_function45;               /* 45 */
	uint16_t setdceclock;                  /* 46 */
	uint16_t getmemoryclock;               /* 47 */
	uint16_t getengineclock;               /* 48 */
	uint16_t setcrtc_usingdtdtiming;       /* 49 */
	uint16_t externalencodercontrol;       /* 50 */
	uint16_t cmd_function51;               /* 51 */
	uint16_t cmd_function52;               /* 52 */
	uint16_t cmd_function53;               /* 53 */
	uint16_t processi2cchanneltransaction;  /* 54 */
	uint16_t cmd_function55;               /* 55 */
	uint16_t cmd_function56;               /* 56 */
	uint16_t cmd_function57;               /* 57 */
	uint16_t cmd_function58;               /* 58 */
	uint16_t cmd_function59;               /* 59 */
	uint16_t computegpuclockparam;         /* 60 */
	uint16_t cmd_function61;               /* 61 */
	uint16_t cmd_function62;               /* 62 */
	uint16_t dynamicmemorysettings;        /* 63 */
	uint16_t memorytraining;               /* 64 */
	uint16_t cmd_function65;               /* 65 */
	uint16_t cmd_function66;               /* 66 */
	uint16_t setvoltage;                   /* 67 */
	uint16_t cmd_function68;               /* 68 */
	uint16_t readefusevalue;               /* 69 */
	uint16_t cmd_function70;               /* 70 */
	uint16_t cmd_function71;               /* 71 */
	uint16_t cmd_function72;               /* 72 */
	uint16_t cmd_function73;               /* 73 */
	uint16_t cmd_function74;               /* 74 */
	uint16_t cmd_function75;               /* 75 */
	uint16_t dig1transmittercontrol;       /* 76 */
	uint16_t cmd_function77;               /* 77 */
	uint16_t processauxchanneltransaction; /* 78 */
	uint16_t cmd_function79;               /* 79 */
	uint16_t getvoltageinfo;               /* 80 */
};

/*
 * v2 master data table (atomfirmware.h format).
 */
struct atom_master_list_of_data_tables_v2_1 {
	uint16_t utilitypipeline;          /* 0 */
	uint16_t multimedia_info;          /* 1 */
	uint16_t smc_dpm_info;             /* 2 */
	uint16_t sw_datatable3;            /* 3 */
	uint16_t firmwareinfo;             /* 4 */
	uint16_t sw_datatable5;            /* 5 */
	uint16_t lcd_info;                 /* 6 */
	uint16_t sw_datatable7;            /* 7 */
	uint16_t smu_info;                 /* 8 */
	uint16_t sw_datatable9;            /* 9 */
	uint16_t sw_datatable10;           /* 10 */
	uint16_t vram_usagebyfirmware;     /* 11 */
	uint16_t gpio_pin_lut;             /* 12 */
	uint16_t sw_datatable13;           /* 13 */
	uint16_t gfx_info;                 /* 14 */
	uint16_t powerplayinfo;            /* 15 */
	uint16_t sw_datatable16;           /* 16 */
	uint16_t sw_datatable17;           /* 17 */
	uint16_t sw_datatable18;           /* 18 */
	uint16_t sw_datatable19;           /* 19 */
	uint16_t sw_datatable20;           /* 20 */
	uint16_t sw_datatable21;           /* 21 */
	uint16_t displayobjectinfo;        /* 22 */
	uint16_t indirectioaccess;         /* 23 */
	uint16_t umc_info;                 /* 24 */
	uint16_t sw_datatable25;           /* 25 */
	uint16_t sw_datatable26;           /* 26 */
	uint16_t dce_info;                 /* 27 */
	uint16_t vram_info;                /* 28 */
	uint16_t sw_datatable29;           /* 29 */
	uint16_t integratedsysteminfo;     /* 30 */
	uint16_t asic_profiling_info;      /* 31 */
	uint16_t voltageobject_info;       /* 32 */
	uint16_t sw_datatable33;           /* 33 */
	uint16_t sw_datatable34;           /* 34 */
};

enum atom_object_record_type_id {
	ATOM_V2_I2C_RECORD_TYPE = 1,
	ATOM_V2_HPD_INT_RECORD_TYPE = 2,
	ATOM_V2_ENCODER_CAP_RECORD_TYPE = 20,
	ATOM_V2_RECORD_END_TYPE = 0xff,
};

#pragma pack(push, 1)

struct atom_v2_common_table_header {
	uint16_t structure_size;
	uint8_t format_revision;
	uint8_t content_revision;
};

struct atom_v2_common_record_header {
	uint8_t record_type;
	uint8_t record_size;
};

struct atom_v2_i2c_record {
	struct atom_v2_common_record_header header;
	uint8_t i2c_id;
	uint8_t i2c_slave_addr;
};

struct atom_v2_hpd_int_record {
	struct atom_v2_common_record_header header;
	uint8_t pin_id;
	uint8_t plugin_pin_state;
};

struct atom_v2_encoder_caps_record {
	struct atom_v2_common_record_header header;
	uint32_t encoder_caps;
};

struct atom_display_object_path_v2 {
	uint16_t display_objid;
	uint16_t display_record_offset;
	uint16_t encoder_objid;
	uint16_t ext_encoder_objid;
	uint16_t encoder_record_offset;
	uint16_t ext_encoder_record_offset;
	uint16_t device_tag;
	uint8_t priority_id;
	uint8_t reserved;
};

struct atom_display_object_path_v3 {
	uint16_t display_objid;
	uint16_t display_record_offset;
	uint16_t encoder_objid;
	uint16_t reserved1;
	uint16_t reserved2;
	uint16_t reserved3;
	uint16_t device_tag;
	uint16_t reserved4;
};

#pragma pack(pop)

struct atom_display_object_info_v1_4 {
	struct atom_v2_common_table_header header;
	uint16_t supported_devices;
	uint8_t path_count;
	uint8_t reserved;
	struct atom_display_object_path_v2 paths[];
};

struct atom_display_object_info_v1_5 {
	struct atom_v2_common_table_header header;
	uint16_t supported_devices;
	uint8_t path_count;
	uint8_t reserved;
	struct atom_display_object_path_v3 paths[];
};

#define ATOM_V2_MAX_DISPLAY_PATHS 8
#define ATOM_V2_ID_NONE 0xff

struct atom_v2_display_path {
	uint16_t display_objid;
	uint16_t encoder_objid;
	uint16_t ext_encoder_objid;
	uint16_t device_tag;
	uint32_t encoder_caps;
	uint8_t priority_id;
	uint8_t i2c_id;
	uint8_t i2c_slave_addr;
	uint8_t hpd_pin;
	uint8_t hpd_active;
	uint8_t dig_encoder;
	uint8_t phy_id;
	bool i2c_valid;
	bool hpd_valid;
	bool encoder_caps_valid;
};

struct atom_v2_display_info {
	uint16_t supported_devices;
	uint8_t format_revision;
	uint8_t content_revision;
	uint8_t path_count;
	struct atom_v2_display_path paths[ATOM_V2_MAX_DISPLAY_PATHS];
};

struct atom_v2_firmware_info {
	uint8_t format_revision;
	uint8_t content_revision;
	uint32_t firmware_revision;
	uint32_t boot_sclk_10khz;
	uint32_t boot_mclk_10khz;
	uint32_t firmware_capability;
	uint32_t scratch_reg_start;
	uint64_t mc_base;
};

struct atom_v2_dce_info {
	uint8_t format_revision;
	uint8_t content_revision;
	uint32_t display_caps;
	uint32_t boot_dispclk_10khz;
	uint16_t dce_refclk_10khz;
	uint16_t i2c_refclk_10khz;
	uint16_t dpphy_refclk_10khz;
	uint8_t min_version;
	uint8_t max_version;
	uint8_t max_pipes;
	uint8_t max_vbios_pipes;
	uint8_t max_pplls;
	uint8_t max_phys;
	uint8_t max_aux_pairs;
};

struct atom_v2_rom_info {
	uint16_t command_table_offset;
	uint16_t data_table_offset;
};

int atom_v2_parse_rom_header(const void *bios, size_t bios_size,
			     struct atom_v2_rom_info *info);
int atom_v2_get_table(const void *bios, size_t bios_size,
		      size_t master_table_offset, size_t index,
		      const void **table, size_t *table_size);
int atom_v2_parse_display_object_info(const void *table, size_t table_size,
				      struct atom_v2_display_info *info);
int atom_v2_parse_firmware_info(const void *table, size_t table_size,
				struct atom_v2_firmware_info *info);
int atom_v2_parse_dce_info(const void *table, size_t table_size,
			   struct atom_v2_dce_info *info);

#endif /* ATOMFIRMWARE_H */
