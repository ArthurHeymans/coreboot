/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_SP3_SMU_H
#define AMD_SP3_SMU_H

/* Rome SMU V11, shared by the socket's NBIO quadrants. */
#define SMN_SMU_MESG_ID          0x03b10530
#define SMN_SMU_MESG_RESP        0x03b1057c
#define SMN_SMU_MESG_ARGS_BASE   0x03b109c4
#define SMU_NUM_ARGS            6

#ifndef __ACPI__
enum smu_message_id {
	SMC_MSG_TEST = 0x01,
	SMC_MSG_ENABLE_FEATURES = 0x03,
	SMC_MSG_DXIO_SERVICE = 0x08,
	SMC_MSG_GET_CAC_WEIGHT = 0x09,
	SMC_MSG_S3ENTRY = 0x0b,
	SMC_MSG_DC_BTC = 0x0c,
	SMC_MSG_GET_NAME_STRING = 0x0d,
};

void smu_sx_entry(void);
#endif

#endif /* AMD_SP3_SMU_H */
