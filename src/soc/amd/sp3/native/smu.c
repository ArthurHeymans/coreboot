/* SPDX-License-Identifier: GPL-2.0-only */

#include <arch/cpu.h>
#include <console/console.h>
#include <cpu/x86/cache.h>
#include <delay.h>
#include <soc/smu.h>
#include <string.h>
#include "apob.h"
#include "smu.h"
#include "topology.h"

struct thread {
	uint32_t enable;
	uint8_t bit;
	uint16_t apic;
};
static struct thread threads[128];
static unsigned int thread_count;

static uint8_t core_byte(size_t offset)
{
	uint8_t value;

	if (!sp3_apob_read8(sp3_apob(), 3, 3, 0, offset, 0, &value))
		die("Missing/truncated APOB core map\n");
	return value;
}

unsigned int sp3_thread_count(void)
{
	unsigned int logical_ccd = 0;
	const unsigned int smt = ((cpuid_ebx(0x8000001e) >> 8) & 0xff) != 0;

	if (thread_count)
		return thread_count;
	for (unsigned int d = 0; d < 8; d++) {
		const size_t ccd = 0x30 + 0x1b * d;
		const uint8_t physical = core_byte(ccd);
		unsigned int logical_ccx = 0;

		if (physical == 0xff)
			continue;
		if (physical >= 8)
			die("Invalid APOB CCD ID\n");
		for (unsigned int x = 0; x < 2; x++) {
			const size_t ccx = ccd + 1 + 0xd * x;
			unsigned int logical_core = 0;

			if (core_byte(ccx) == 0xff)
				continue;
			for (unsigned int c = 0; c < 4; c++) {
				const size_t core = ccx + 1 + 3 * c;

				if (core_byte(core) == 0xff)
					continue;
				for (unsigned int th = 0; th <= smt; th++) {
					if (!core_byte(core + 1 + th))
						continue;
					if (thread_count == ARRAY_SIZE(threads))
						die("Too many SP3 threads\n");
					threads[thread_count++] = (struct thread) {
						.enable = 0x30081018 + (physical << 25),
						.bit = logical_ccx * 8 + logical_core * (1 + smt) + th,
						.apic = logical_ccd << (3 + smt) |
							logical_ccx << (2 + smt) | logical_core << smt | th,
					};
				}
				logical_core++;
			}
			logical_ccx++;
		}
		logical_ccd++;
	}
	if (!thread_count || threads[0].apic != 0)
		die("Invalid APOB BSP topology\n");
	printk(BIOS_INFO, "SP3 APOB: %u threads on %u CCDs\n", thread_count, logical_ccd);
	return thread_count;
}

static enum cb_err send(unsigned int message, uint32_t argument)
{
	struct smu_payload p = { .msg = { argument } };

	return sp3_smu_send(0, message, &p);
}

enum cb_err sp3_smu_test_message(unsigned int rb)
{
	struct smu_payload p = { .msg = { 0xaa55aa55 } };

	return sp3_smu_send(rb, 1, &p);
}

enum cb_err sp3_smu_launch_aps(unsigned int ap_count)
{
	if (ap_count + 1 != sp3_thread_count())
		return CB_ERR;
	for (unsigned int i = 1; i < thread_count; i++) {
		const struct thread *t = &threads[i];
		const uint32_t value = sp3_smn_read32(0, t->enable);

		if (value & BIT(t->bit)) {
			printk(BIOS_ERR, "SP3 AP %u already released\n", t->apic);
			return CB_ERR;
		}
		sp3_smn_write32(0, t->enable, value | BIT(t->bit));
		if (sp3_smu_test_message(0))
			return CB_ERR;
	}
	return CB_SUCCESS;
}

void sp3_smu_print_name(void)
{
	char name[48];

	for (unsigned int i = 0; i < 12; i++) {
		struct smu_payload p = { .msg = { i } };

		if (sp3_smu_send(0, 0xd, &p))
			return;
		memcpy(name + i * 4, p.msg, 4);
	}
	name[47] = 0;
	printk(BIOS_INFO, "CPU: %s\n", name);
}

enum cb_err sp3_smu_init(void)
{
	static uint8_t pptable[0x27c] __aligned(4096);
	static uint8_t tools[4096] __aligned(4096);
	const unsigned int count = sp3_thread_count();
	struct smu_payload p;

	memset(pptable, 0, sizeof(pptable));
	*(uint32_t *)(pptable + 0x18) = 280; /* Platform PPT; other limits use fused values. */
	pptable[0x3e] = 100;
	*(uint16_t *)(pptable + 0x74) = count;
	memset(pptable + 0x78, 0xff, 512);
	for (unsigned int i = 0; i < count; i++)
		((uint16_t *)(pptable + 0x78))[i] = threads[i].apic;
	/* Publish the SMU's DRAM input before transferring the table. */
	wbinvd();
	p = (struct smu_payload) { .msg = { (uintptr_t)pptable, 0 } };
	if (sp3_smu_send(0, 5, &p) || send(0x10, sizeof(pptable)))
		return CB_ERR;
	p = (struct smu_payload) { .msg = { (uintptr_t)tools, 0 } };
	if (sp3_smu_send(0, 6, &p))
		return CB_ERR;
	for (unsigned int rb = 0; rb < sp3_die_count(); rb++)
		sp3_smn_clrsetbits32(rb, sp3_nbio_address(rb, 0x01811000), 0, BIT(11));
	return CB_SUCCESS;
}

enum cb_err sp3_smu_aps_ready(void)
{
	return CB_SUCCESS;
}

enum cb_err sp3_smu_ccx_done(void)
{
	/* Legacy ACPI P-states, no PCC _CPC yet: leave SMU CPPC disabled. */
	const uint32_t features = 0x9a1fff;

	if (send(3, features & ~BIT(17)))
		return CB_ERR;
	mdelay(10);
	if (send(0xc, 0x86) || send(3, features & BIT(17)))
		return CB_ERR;
	return CB_SUCCESS;
}

enum cb_err sp3_smu_cac_weights(uint64_t weights[SMU_CAC_WEIGHT_COUNT])
{
	for (unsigned int i = 0; i < SMU_CAC_WEIGHT_COUNT; i++) {
		struct smu_payload p = { .msg = { i } };

		if (sp3_smu_send(0, 9, &p))
			return CB_ERR;
		weights[i] = (uint64_t)p.msg[1] << 32 | p.msg[0];
	}
	return CB_SUCCESS;
}
