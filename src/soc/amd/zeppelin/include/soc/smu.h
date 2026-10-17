/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_ZEPPELIN_SMU_H
#define AMD_ZEPPELIN_SMU_H

/* SMU (MP1) V9 mailbox in SMN; the same address on every die */
#define SMN_SMU_MESG_ID		0x3b10528
#define SMN_SMU_MESG_RESP	0x3b10564
#define SMN_SMU_MESG_ARGS_BASE	0x3b10598

#define SMU_NUM_ARGS		6

#ifndef __ACPI__
/* Messages of the SMU V9 firmware that the native initialization uses */
enum smu_message_id {
	SMC_MSG_TEST = 0x01,			/* argument 0xaa55aa55, also applies ThreadEn */
	SMC_MSG_GET_VERSION = 0x03,
	SMC_MSG_FINAL = 0x08,			/* after the APs are up */
	SMC_MSG_ENABLE_FEATURES = 0x09,		/* argument: feature mask */
	SMC_MSG_DXIO_SERVICE = 0x0f,
	SMC_MSG_GET_CAC_WEIGHT = 0x10,		/* argument: index 0..20 */
	SMC_MSG_S3ENTRY = 0x13,			/* sleep entry */
	SMC_MSG_SET_POLICY_ADDRESS_HI = 0x1d,
	SMC_MSG_SET_POLICY_ADDRESS_LO = 0x1e,
	SMC_MSG_TRANSFER_POLICY = 0x22,
	SMC_MSG_AP_READY = 0x23,
	SMC_MSG_GET_NAME_STRING = 0x28,		/* argument: dword index */
	SMC_MSG_38 = 0x38,
	SMC_MSG_39 = 0x39,
	SMC_MSG_3B = 0x3b,			/* argument: root bus of the die */
	SMC_MSG_3D = 0x3d,			/* argument: root bus of the die */
	SMC_MSG_CCX_DONE = 0x41,
};

/*
 * Request the SMU put system into S3, S4, or S5. On entry, SlpTyp determines S-State and
 * SlpTypeEn gets set by the SMU. Function does not return if successful.
 */
void smu_sx_entry(void);

#endif /* !__ACPI__ */

#endif /* AMD_ZEPPELIN_SMU_H */
