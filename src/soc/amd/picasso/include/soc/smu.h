/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef AMD_PICASSO_SMU_H
#define AMD_PICASSO_SMU_H

/* SMU mailbox register offsets in SMN */
#define SMN_SMU_MESG_ID		0x3b10528
#define SMN_SMU_MESG_RESP	0x3b10564
#define SMN_SMU_MESG_ARGS_BASE	0x3b10998

#define SMU_NUM_ARGS		6

/* Runtime power-policy commands used by native ALIB. */
#define SMC_MSG_SET_AC_MODE			0x18
#define SMC_MSG_SET_DC_MODE			0x19
#define SMC_MSG_SET_SUSTAINED_POWER_LIMIT	0x1a
#define SMC_MSG_SET_FAST_PPT_LIMIT		0x1b
#define SMC_MSG_SET_SLOW_PPT_LIMIT		0x1c
#define SMC_MSG_SET_THERMAL_CONTROL_LIMIT	0x1f

#ifndef __ACPI__
enum smu_message_id {
	SMC_MSG_TEST = 0x01,			/* argument 0xaa55aa55 */
	SMC_MSG_SET_POLICY_VERSION = 0x03,
	SMC_MSG_GET_NAME_STRING = 0x04,		/* argument: dword index */
	SMC_MSG_ENABLE_FEATURES = 0x05,		/* arguments: feature mask low/high */
	SMC_MSG_DXIO_SERVICE = 0x0a,
	SMC_MSG_S3ENTRY = 0x0c,
	SMC_MSG_0E = 0x0e,
	SMC_MSG_11 = 0x11,
	SMC_MSG_TRANSFER_POLICY = 0x12,
	SMC_MSG_SET_POLICY_ADDRESS = 0x13,
	SMC_MSG_TRANSFER_SCRATCH = 0x14,
	SMC_MSG_SET_SCRATCH_ADDRESS = 0x15,
	SMC_MSG_FINALIZE_POLICY = 0x17,
	SMC_MSG_USB_PHY_COMPLETE_RESUME = 0x2d,
	SMC_MSG_USB_PHY_COMPLETE = 0x2e,
	SMC_MSG_32 = 0x32,
	SMC_MSG_35 = 0x35,
	SMC_MSG_USB_CONFIG_UPDATE = 0x38,
	SMC_MSG_3A = 0x3a,
	SMC_MSG_42 = 0x42,
	SMC_MSG_USB_DISABLE = 0x44,
	SMC_MSG_GET_SILICON_ID = 0x51,
	SMC_MSG_53 = 0x53,
};

/*
 * Request the SMU put system into S3, S4, or S5. On entry, SlpTyp determines S-State and
 * SlpTypeEn gets set by the SMU. Function does not return if successful.
 */
void smu_sx_entry(void);

#endif /* !__ACPI__ */

#endif /* AMD_PICASSO_SMU_H */
