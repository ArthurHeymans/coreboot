/* SPDX-License-Identifier: GPL-2.0-only */

#include <stdint.h>
#include <string.h>
#include "../include/tests/test.h"

#include "../../src/drivers/amd/atombios/atomfirmware.h"

#ifndef __TEST_NAME__
#define __TEST_NAME__ "atombios-v2-test"
#endif

static void put_le16(uint8_t *p, uint16_t value)
{
	p[0] = value;
	p[1] = value >> 8;
}

static void put_le32(uint8_t *p, uint32_t value)
{
	put_le16(p, value);
	put_le16(p + 2, value >> 16);
}

static void make_display_info_v1_4(uint8_t *table, size_t size)
{
	assert_true(size >= 42);
	memset(table, 0, size);
	put_le16(table, 42);
	table[2] = 1;
	table[3] = 4;
	put_le16(table + 4, 0x0008);
	table[6] = 1;

	put_le16(table + 8, 0x310c);  /* HDMI-A connector, enum 1 */
	put_le16(table + 10, 24);
	put_le16(table + 12, 0x211e); /* UNIPHY, enum 1 */
	put_le16(table + 16, 34);
	put_le16(table + 20, 0x0008);
	table[22] = 1;

	table[24] = ATOM_V2_I2C_RECORD_TYPE;
	table[25] = sizeof(struct atom_v2_i2c_record);
	table[26] = 0x91;
	table[27] = 0;
	table[28] = ATOM_V2_HPD_INT_RECORD_TYPE;
	table[29] = sizeof(struct atom_v2_hpd_int_record);
	table[30] = 3;
	table[31] = 1;
	table[32] = ATOM_V2_RECORD_END_TYPE;

	table[34] = ATOM_V2_ENCODER_CAP_RECORD_TYPE;
	table[35] = sizeof(struct atom_v2_encoder_caps_record);
	put_le32(table + 36, 0x11223344);
	table[40] = ATOM_V2_RECORD_END_TYPE;
}

static void make_display_info_v1_5(uint8_t *table, size_t size)
{
	assert_true(size >= 30);
	memset(table, 0, size);
	put_le16(table, 30);
	table[2] = 1;
	table[3] = 5;
	put_le16(table + 4, 0x0080);
	table[6] = 1;

	put_le16(table + 8, 0x3113);  /* DisplayPort connector, enum 1 */
	put_le16(table + 10, 24);
	put_le16(table + 12, 0x2220); /* UNIPHY1, enum 2 */
	put_le16(table + 20, 0x0080);

	table[24] = ATOM_V2_I2C_RECORD_TYPE;
	table[25] = sizeof(struct atom_v2_i2c_record);
	table[26] = 0x82;
	table[27] = 0;
	table[28] = ATOM_V2_RECORD_END_TYPE;
}

static void test_display_info_v1_4(void **state)
{
	uint8_t table[42];
	struct atom_v2_display_info info;

	make_display_info_v1_4(table, sizeof(table));
	assert_int_equal(atom_v2_parse_display_object_info(table, sizeof(table), &info), 0);
	assert_int_equal(info.content_revision, 4);
	assert_int_equal(info.path_count, 1);
	assert_int_equal(info.paths[0].display_objid, 0x310c);
	assert_int_equal(info.paths[0].encoder_objid, 0x211e);
	assert_int_equal(info.paths[0].i2c_id, 0x91);
	assert_int_equal(info.paths[0].hpd_pin, 3);
	assert_int_equal(info.paths[0].encoder_caps, 0x11223344);
	assert_int_equal(info.paths[0].dig_encoder, 0);
	assert_int_equal(info.paths[0].phy_id, 0);
}

static void test_display_info_v1_5(void **state)
{
	uint8_t table[30];
	struct atom_v2_display_info info;

	make_display_info_v1_5(table, sizeof(table));
	assert_int_equal(atom_v2_parse_display_object_info(table, sizeof(table), &info), 0);
	assert_int_equal(info.content_revision, 5);
	assert_int_equal(info.paths[0].display_objid, 0x3113);
	assert_int_equal(info.paths[0].device_tag, 0x0080);
	assert_int_equal(info.paths[0].i2c_id, 0x82);
	assert_int_equal(info.paths[0].dig_encoder, 3);
	assert_int_equal(info.paths[0].phy_id, 3);
}

static void test_display_info_rejects_malformed_tables(void **state)
{
	uint8_t table[42];
	struct atom_v2_display_info info;

	make_display_info_v1_4(table, sizeof(table));
	table[3] = 6;
	assert_int_not_equal(atom_v2_parse_display_object_info(table, sizeof(table), &info), 0);

	make_display_info_v1_4(table, sizeof(table));
	table[6] = ATOM_V2_MAX_DISPLAY_PATHS + 1;
	assert_int_not_equal(atom_v2_parse_display_object_info(table, sizeof(table), &info), 0);

	make_display_info_v1_4(table, sizeof(table));
	put_le16(table + 10, sizeof(table));
	assert_int_not_equal(atom_v2_parse_display_object_info(table, sizeof(table), &info), 0);

	make_display_info_v1_4(table, sizeof(table));
	put_le16(table + 10, 8);
	assert_int_not_equal(atom_v2_parse_display_object_info(table, sizeof(table), &info), 0);

	make_display_info_v1_4(table, sizeof(table));
	table[25] = 1;
	assert_int_not_equal(atom_v2_parse_display_object_info(table, sizeof(table), &info), 0);

	make_display_info_v1_4(table, sizeof(table));
	put_le16(table, sizeof(table) + 1);
	assert_int_not_equal(atom_v2_parse_display_object_info(table, sizeof(table), &info), 0);
}

static void test_bounded_master_table_lookup(void **state)
{
	uint8_t bios[128] = {0};
	const void *table;
	size_t table_size;

	put_le16(bios + 16, 10);
	put_le16(bios + 16 + 8, 64); /* index 2 */
	put_le16(bios + 64, 8);
	bios[66] = 1;
	bios[67] = 4;
	assert_int_equal(atom_v2_get_table(bios, sizeof(bios), 16, 2,
					   &table, &table_size), 0);
	assert_ptr_equal(table, bios + 64);
	assert_int_equal(table_size, 8);

	put_le16(bios + 16 + 8, 124);
	assert_int_not_equal(atom_v2_get_table(bios, sizeof(bios), 16, 2,
					       &table, &table_size), 0);
}

static void test_bounded_rom_header(void **state)
{
	uint8_t bios[128] = {0};
	struct atom_v2_rom_info info;

	put_le16(bios, 0xaa55);
	put_le16(bios + 0x48, 0x50);
	put_le16(bios + 0x50, 0x26);
	bios[0x52] = 2;
	bios[0x53] = 2;
	memcpy(bios + 0x54, "ATOM", 4);
	put_le16(bios + 0x50 + 0x1e, 0x20);
	put_le16(bios + 0x50 + 0x20, 0x30);
	put_le16(bios + 0x20, 4);
	bios[0x22] = 2;
	put_le16(bios + 0x30, 4);
	bios[0x32] = 2;

	assert_int_equal(atom_v2_parse_rom_header(bios, sizeof(bios), &info), 0);
	assert_int_equal(info.command_table_offset, 0x20);
	assert_int_equal(info.data_table_offset, 0x30);

	put_le16(bios + 0x48, sizeof(bios));
	assert_int_not_equal(atom_v2_parse_rom_header(bios, sizeof(bios), &info), 0);
}

static void test_firmware_and_dce_info(void **state)
{
	uint8_t firmware[48] = {0};
	uint8_t dce[56] = {0};
	struct atom_v2_firmware_info fw_info;
	struct atom_v2_dce_info dce_info;

	put_le16(firmware, sizeof(firmware));
	firmware[2] = 3;
	firmware[3] = 1;
	put_le32(firmware + 4, 0x12345678);
	put_le32(firmware + 8, 10000);
	put_le32(firmware + 12, 20000);
	put_le32(firmware + 24, 0x1724);
	put_le32(firmware + 40, 1);
	put_le32(firmware + 44, 2);
	assert_int_equal(atom_v2_parse_firmware_info(firmware, sizeof(firmware),
						     &fw_info), 0);
	assert_int_equal(fw_info.firmware_revision, 0x12345678);
	assert_int_equal(fw_info.mc_base, 0x100000002ULL);

	put_le16(dce, sizeof(dce));
	dce[2] = 4;
	dce[3] = 1;
	put_le32(dce + 8, 60000);
	put_le16(dce + 12, 2700);
	put_le16(dce + 14, 10000);
	put_le16(dce + 36, 2700);
	dce[40] = 12;
	dce[41] = 12;
	dce[42] = 6;
	dce[45] = 6;
	dce[46] = 6;
	assert_int_equal(atom_v2_parse_dce_info(dce, sizeof(dce), &dce_info), 0);
	assert_int_equal(dce_info.boot_dispclk_10khz, 60000);
	assert_int_equal(dce_info.max_pipes, 6);
}

int main(void)
{
	const struct CMUnitTest tests[] = {
		cmocka_unit_test(test_display_info_v1_4),
		cmocka_unit_test(test_display_info_v1_5),
		cmocka_unit_test(test_display_info_rejects_malformed_tables),
		cmocka_unit_test(test_bounded_master_table_lookup),
		cmocka_unit_test(test_bounded_rom_header),
		cmocka_unit_test(test_firmware_and_dce_info),
	};

	return cb_run_group_tests(tests, NULL, NULL);
}
