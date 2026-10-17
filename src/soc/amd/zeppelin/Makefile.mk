# SPDX-License-Identifier: GPL-2.0-only

ifeq ($(CONFIG_SOC_AMD_ZEPPELIN),y)

all_x86-y += gpio.c

bootblock-y += early_fch.c

romstage-y += soc_util.c

ramstage-$(CONFIG_HAVE_ACPI_TABLES) += acpi.c
ramstage-$(CONFIG_HAVE_ACPI_TABLES) += acpi_crat.c
ramstage-y += chip.c
ramstage-y += cpu.c
ramstage-y += fch.c
ramstage-y += mca.c
ramstage-y += pcie_gpp.c
ramstage-y += memmap.c
ramstage-y += root_complex.c
ramstage-y += sata.c
ramstage-y += soc_util.c
ramstage-y += xhci.c

subdirs-y += native

smm-y += gpio.c
smm-y += root_complex.c

CPPFLAGS_common += -I$(src)/soc/amd/zeppelin/include
CPPFLAGS_common += -I$(src)/soc/amd/zeppelin/acpi

# First image: the vendor EFS, PSP/BIOS directories and signed blobs are kept
# verbatim (flash 0 to ZEPPELIN_VENDOR_AMDFW_SIZE). Only the BIOS reset image
# (directory entry 0x62: flash 0xdb0000, 0x250000 bytes, copied to 0x76db0000)
# is replaced with the coreboot bootblock.

ZEPPELIN_VENDOR_ROM := $(call strip_quotes,$(CONFIG_ZEPPELIN_VENDOR_ROM))

$(obj)/amdfw.rom: $(ZEPPELIN_VENDOR_ROM)
	@printf "    AMDFW      $(subst $(obj)/,,$(@)) (vendor directories)\n"
	echo "$(call strip_quotes,$(CONFIG_ZEPPELIN_VENDOR_ROM_SHA256))  $<" | sha256sum -c --quiet
	dd if=$< of=$@ bs=4096 count=$(call int-divide,$(CONFIG_ZEPPELIN_VENDOR_AMDFW_SIZE) 4096) \
		status=none

# Microcode: the BIOS directory type 0x66 entries of the vendor image
# (flash offset:equivalence ID), 0xc80 bytes each. coreboot loads them on every
# core in ramstage; the vendor firmware does it in DXE.
ZEPPELIN_UCODE := 0x25f800:8012 0x260500:8011 0x261200:8010 0x261f00:8000

define zeppelin-add-ucode
$(obj)/cpu_microcode_$(2).bin: $(ZEPPELIN_VENDOR_ROM)
	dd if=$$< of=$$@ bs=64 skip=$$$$(( $(1) / 64 )) count=50 status=none

cbfs-files-y += cpu_microcode_$(2).bin
cpu_microcode_$(2).bin-file := $(obj)/cpu_microcode_$(2).bin
cpu_microcode_$(2).bin-type := microcode
cpu_microcode_$(2).bin-align := 16
endef

$(foreach u,$(ZEPPELIN_UCODE),$(eval $(call zeppelin-add-ucode,$(word 1,$(subst :, ,$(u))),$(word 2,$(subst :, ,$(u))))))

# The PSP copies the whole reset image region to the end of the bootblock
# region, so the loadable part of the bootblock goes at the end and the
# region is padded in front of it. The flash copy is identical to the DRAM
# copy, so x86 may fetch its reset vector from either.
ZEPPELIN_RESET_IMAGE = $(obj)/zeppelin_reset_image.bin

$(ZEPPELIN_RESET_IMAGE): $(objcbfs)/bootblock_fixed_data.elf $(AMDCOMPRESS)
	@printf "    RESETIMG   $(subst $(obj)/,,$(@))\n"
	$(AMDCOMPRESS) --infile $< --outfile $@.load --elfcopy \
		--maxsize $(CONFIG_C_ENV_BOOTBLOCK_SIZE)
	size=$$(stat -c %s $@.load); \
	pad=$$(( $(CONFIG_C_ENV_BOOTBLOCK_SIZE) - size )); \
	test $$pad -ge 0 && \
	{ head -c $$pad /dev/zero; cat $@.load; } > $@

$(call add_intermediate, add_zeppelin_amdfw, $(obj)/amdfw.rom)
	$(CBFSTOOL) $< write -r AMDFW -f $(obj)/amdfw.rom

$(call add_intermediate, add_zeppelin_reset_image, $(ZEPPELIN_RESET_IMAGE))
	$(CBFSTOOL) $< write -r BOOTBLOCK -f $(ZEPPELIN_RESET_IMAGE)

endif # ($(CONFIG_SOC_AMD_ZEPPELIN),y)
