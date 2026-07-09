/* SPDX-License-Identifier: MIT */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "atomfirmware.h"
#include "ObjectID.h"

#define ATOM_V2_DISPLAY_INFO_FIXED_SIZE 8
#define ATOM_V2_MAX_RECORDS 256
#define ATOM_ROM_HEADER_POINTER 0x48
#define ATOM_ROM_HEADER_MIN_SIZE 0x22
#define ATOM_ROM_COMMAND_TABLE_OFFSET 0x1e
#define ATOM_ROM_DATA_TABLE_OFFSET 0x20

_Static_assert(sizeof(struct atom_display_object_path_v2) == 16, "bad v2 path ABI");
_Static_assert(sizeof(struct atom_display_object_path_v3) == 16, "bad v3 path ABI");
_Static_assert(sizeof(struct atom_v2_encoder_caps_record) == 6, "bad record ABI");

static uint16_t get_le16(const uint8_t *p)
{
	return p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t get_le32(const uint8_t *p)
{
	return get_le16(p) | ((uint32_t)get_le16(p + 2) << 16);
}

static bool range_valid(size_t offset, size_t length, size_t size)
{
	return offset <= size && length <= size - offset;
}

int atom_v2_parse_rom_header(const void *bios, size_t bios_size,
			     struct atom_v2_rom_info *info)
{
	const uint8_t *image = bios;
	uint16_t header_offset;
	uint16_t header_size;
	uint16_t command_offset;
	uint16_t data_offset;

	if (!image || !info || !range_valid(0, ATOM_ROM_HEADER_POINTER + 2, bios_size) ||
	    get_le16(image) != 0xaa55)
		return -1;

	header_offset = get_le16(image + ATOM_ROM_HEADER_POINTER);
	if (!range_valid(header_offset, ATOM_ROM_HEADER_MIN_SIZE, bios_size) ||
	    memcmp(image + header_offset + 4, "ATOM", 4))
		return -1;

	header_size = get_le16(image + header_offset);
	if (header_size < ATOM_ROM_HEADER_MIN_SIZE ||
	    !range_valid(header_offset, header_size, bios_size) ||
	    image[header_offset + 2] < 2)
		return -1;

	command_offset = get_le16(image + header_offset + ATOM_ROM_COMMAND_TABLE_OFFSET);
	data_offset = get_le16(image + header_offset + ATOM_ROM_DATA_TABLE_OFFSET);
	if (!command_offset || !data_offset || !range_valid(command_offset, 4, bios_size) ||
	    !range_valid(data_offset, 4, bios_size) || image[command_offset + 2] < 2 ||
	    image[data_offset + 2] < 2)
		return -1;

	info->command_table_offset = command_offset;
	info->data_table_offset = data_offset;
	return 0;
}

int atom_v2_get_table(const void *bios, size_t bios_size,
		      size_t master_table_offset, size_t index,
		      const void **table, size_t *table_size)
{
	const uint8_t *image = bios;
	size_t entry_offset;
	uint16_t master_size;
	uint16_t target_offset;
	uint16_t target_size;

	if (!bios || !table || !table_size ||
	    !range_valid(master_table_offset, 4, bios_size))
		return -1;

	master_size = get_le16(image + master_table_offset);
	if (index > (SIZE_MAX - 4) / sizeof(uint16_t))
		return -1;
	entry_offset = 4 + index * sizeof(uint16_t);
	if (master_size < 4 || !range_valid(master_table_offset, master_size, bios_size) ||
	    !range_valid(entry_offset, sizeof(uint16_t), master_size))
		return -1;

	target_offset = get_le16(image + master_table_offset + entry_offset);
	if (!target_offset || !range_valid(target_offset, 4, bios_size))
		return -1;

	target_size = get_le16(image + target_offset);
	if (target_size < 4 || !range_valid(target_offset, target_size, bios_size))
		return -1;

	*table = image + target_offset;
	*table_size = target_size;
	return 0;
}

static int parse_records(const uint8_t *table, size_t table_size, size_t records_start,
			 uint16_t offset, struct atom_v2_display_path *path)
{
	unsigned int i;

	if (!offset)
		return 0;
	if (offset < records_start ||
	    !range_valid(offset, 1, table_size))
		return -1;

	for (i = 0; i < ATOM_V2_MAX_RECORDS; i++) {
		uint8_t type = table[offset];
		uint8_t size;

		if (type == ATOM_V2_RECORD_END_TYPE)
			return 0;
		if (!range_valid(offset, sizeof(struct atom_v2_common_record_header),
				 table_size))
			return -1;
		size = table[offset + 1];
		if (size < sizeof(struct atom_v2_common_record_header) ||
		    !range_valid(offset, size, table_size))
			return -1;

		switch (type) {
		case ATOM_V2_I2C_RECORD_TYPE:
			if (size < sizeof(struct atom_v2_i2c_record))
				return -1;
			if (!path->i2c_valid) {
				path->i2c_id = table[offset + 2];
				path->i2c_slave_addr = table[offset + 3];
				path->i2c_valid = true;
			}
			break;
		case ATOM_V2_HPD_INT_RECORD_TYPE:
			if (size < sizeof(struct atom_v2_hpd_int_record))
				return -1;
			if (!path->hpd_valid) {
				path->hpd_pin = table[offset + 2];
				path->hpd_active = table[offset + 3];
				path->hpd_valid = true;
			}
			break;
		case ATOM_V2_ENCODER_CAP_RECORD_TYPE:
			if (size < sizeof(struct atom_v2_encoder_caps_record))
				return -1;
			if (!path->encoder_caps_valid) {
				path->encoder_caps = get_le32(table + offset + 2);
				path->encoder_caps_valid = true;
			}
			break;
		}

		offset += size;
		if (!range_valid(offset, 1, table_size))
			return -1;
	}

	return -1;
}

static void decode_encoder(uint16_t encoder_objid, struct atom_v2_display_path *path)
{
	uint8_t id = (encoder_objid & OBJECT_ID_MASK) >> OBJECT_ID_SHIFT;
	uint8_t enum_id = (encoder_objid & ENUM_ID_MASK) >> ENUM_ID_SHIFT;
	uint8_t base;

	switch (id) {
	case ENCODER_OBJECT_ID_INTERNAL_UNIPHY:
		base = 0;
		break;
	case ENCODER_OBJECT_ID_INTERNAL_UNIPHY1:
		base = 2;
		break;
	case ENCODER_OBJECT_ID_INTERNAL_UNIPHY2:
		base = 4;
		break;
	case ENCODER_OBJECT_ID_INTERNAL_UNIPHY3:
		base = 6;
		break;
	default:
		return;
	}

	if (enum_id == GRAPH_OBJECT_ENUM_ID2)
		base++;
	path->dig_encoder = base;
	path->phy_id = base;
}

int atom_v2_parse_display_object_info(const void *table_data, size_t table_size,
				      struct atom_v2_display_info *info)
{
	const uint8_t *table = table_data;
	uint16_t structure_size;
	size_t path_size;
	size_t paths_size;
	unsigned int i;

	if (!table || !info || table_size < ATOM_V2_DISPLAY_INFO_FIXED_SIZE)
		return -1;

	structure_size = get_le16(table);
	if (structure_size < ATOM_V2_DISPLAY_INFO_FIXED_SIZE ||
	    structure_size > table_size || table[2] != 1 ||
	    (table[3] != 4 && table[3] != 5))
		return -1;

	path_size = table[3] == 4 ? sizeof(struct atom_display_object_path_v2) :
				      sizeof(struct atom_display_object_path_v3);
	paths_size = (size_t)table[6] * path_size;
	if (table[6] > ATOM_V2_MAX_DISPLAY_PATHS ||
	    !range_valid(ATOM_V2_DISPLAY_INFO_FIXED_SIZE, paths_size, structure_size))
		return -1;

	memset(info, 0, sizeof(*info));
	info->supported_devices = get_le16(table + 4);
	info->format_revision = table[2];
	info->content_revision = table[3];
	info->path_count = table[6];

	for (i = 0; i < info->path_count; i++) {
		const uint8_t *raw = table + ATOM_V2_DISPLAY_INFO_FIXED_SIZE + i * path_size;
		struct atom_v2_display_path *path = &info->paths[i];
		uint16_t display_record_offset = get_le16(raw + 2);
		uint16_t encoder_record_offset = 0;
		uint16_t ext_encoder_record_offset = 0;

		path->hpd_pin = ATOM_V2_ID_NONE;
		path->dig_encoder = ATOM_V2_ID_NONE;
		path->phy_id = ATOM_V2_ID_NONE;
		path->display_objid = get_le16(raw);
		path->encoder_objid = get_le16(raw + 4);
		if (table[3] == 4) {
			path->ext_encoder_objid = get_le16(raw + 6);
			encoder_record_offset = get_le16(raw + 8);
			ext_encoder_record_offset = get_le16(raw + 10);
			path->device_tag = get_le16(raw + 12);
			path->priority_id = raw[14];
		} else {
			path->device_tag = get_le16(raw + 12);
		}

		decode_encoder(path->encoder_objid, path);
		if (parse_records(table, structure_size,
				  ATOM_V2_DISPLAY_INFO_FIXED_SIZE + paths_size,
				  display_record_offset, path) ||
		    parse_records(table, structure_size,
				  ATOM_V2_DISPLAY_INFO_FIXED_SIZE + paths_size,
				  encoder_record_offset, path) ||
		    parse_records(table, structure_size,
				  ATOM_V2_DISPLAY_INFO_FIXED_SIZE + paths_size,
				  ext_encoder_record_offset, path))
			return -1;
	}

	return 0;
}

int atom_v2_parse_firmware_info(const void *table_data, size_t table_size,
				struct atom_v2_firmware_info *info)
{
	const uint8_t *table = table_data;
	uint16_t structure_size;

	if (!table || !info || table_size < 48)
		return -1;
	structure_size = get_le16(table);
	if (structure_size < 48 || structure_size > table_size || table[2] != 3 ||
	    table[3] < 1 || table[3] > 4)
		return -1;

	memset(info, 0, sizeof(*info));
	info->format_revision = table[2];
	info->content_revision = table[3];
	info->firmware_revision = get_le32(table + 4);
	info->boot_sclk_10khz = get_le32(table + 8);
	info->boot_mclk_10khz = get_le32(table + 12);
	info->firmware_capability = get_le32(table + 16);
	info->scratch_reg_start = get_le32(table + 24);
	info->mc_base = ((uint64_t)get_le32(table + 40) << 32) | get_le32(table + 44);
	return 0;
}

int atom_v2_parse_dce_info(const void *table_data, size_t table_size,
			   struct atom_v2_dce_info *info)
{
	const uint8_t *table = table_data;
	uint16_t structure_size;

	if (!table || !info || table_size < 56)
		return -1;
	structure_size = get_le16(table);
	if (structure_size < 56 || structure_size > table_size || table[2] != 4 ||
	    (table[3] != 1 && table[3] != 2))
		return -1;

	memset(info, 0, sizeof(*info));
	info->format_revision = table[2];
	info->content_revision = table[3];
	info->display_caps = get_le32(table + 4);
	info->boot_dispclk_10khz = get_le32(table + 8);
	info->dce_refclk_10khz = get_le16(table + 12);
	info->i2c_refclk_10khz = get_le16(table + 14);
	info->dpphy_refclk_10khz = get_le16(table + 36);
	info->min_version = table[40];
	info->max_version = table[41];
	info->max_pipes = table[42];
	info->max_vbios_pipes = table[43];
	info->max_pplls = table[44];
	info->max_phys = table[45];
	info->max_aux_pairs = table[46];
	return 0;
}
