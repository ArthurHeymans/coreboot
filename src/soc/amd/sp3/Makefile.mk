# SPDX-License-Identifier: GPL-2.0-only

ifeq ($(CONFIG_SOC_AMD_SP3),y)

all_x86-y += gpio.c

bootblock-y += early_fch.c

romstage-y += i2c.c
romstage-y += soc_util.c
romstage-y += identity.c
romstage-y += topology.c

ramstage-$(CONFIG_HAVE_ACPI_TABLES) += acpi.c
ramstage-y += chip.c
ramstage-y += cpu.c
ramstage-y += domain.c
ramstage-y += fch.c
ramstage-y += i2c.c
ramstage-y += mca.c
ramstage-y += pcie_gpp.c
ramstage-y += memmap.c
ramstage-y += root_complex.c
ramstage-y += sata.c
ramstage-y += soc_util.c
ramstage-y += identity.c
ramstage-y += topology.c
ramstage-y += xhci.c

subdirs-y += native

smm-y += gpio.c
smm-y += root_complex.c
smm-y += smihandler.c

CPPFLAGS_common += -I$(src)/soc/amd/sp3/include
CPPFLAGS_common += -I$(src)/soc/amd/sp3/acpi

# First image: the vendor EFS, PSP/BIOS directories and signed blobs are kept
# verbatim (flash 0 to SP3_VENDOR_AMDFW_SIZE). Only the BIOS reset image
# (directory entry 0x62: flash 0xdb0000, 0x250000 bytes, copied to 0x75db0000)
# is replaced with the coreboot bootblock.

SP3_VENDOR_ROM := $(call strip_quotes,$(CONFIG_SP3_VENDOR_ROM))
SP3_UCODE_8310_FILE := $(call strip_quotes,$(CONFIG_SP3_UCODE_8310_FILE))

$(obj)/amdfw.rom: $(SP3_VENDOR_ROM)
	@printf "    AMDFW      $(subst $(obj)/,,$(@)) (vendor directories)\n"
	echo "$(call strip_quotes,$(CONFIG_SP3_VENDOR_ROM_SHA256))  $<" | sha256sum -c --quiet
	dd if=$< of=$@ bs=4096 count=$(call int-divide,$(CONFIG_SP3_VENDOR_AMDFW_SIZE) 4096) \
		status=none

# Microcode: the BIOS directory type 0x66 entries of the vendor image
# (flash offset:equivalence ID), 0xc80 bytes each. coreboot loads them on every
# core in ramstage; the vendor firmware does it in DXE.
SP3_UCODE := 0x4f1b00:8310 0x4f2800:8300

define sp3-add-ucode
$(obj)/cpu_microcode_$(2).bin: $(SP3_VENDOR_ROM)
	dd if=$$< of=$$@ bs=64 skip=$$$$(( $(1) / 64 )) count=50 status=none

cbfs-files-y += cpu_microcode_$(2).bin
cpu_microcode_$(2).bin-file := $(if $(and $(filter 8310,$(2)),$(SP3_UCODE_8310_FILE)),$(SP3_UCODE_8310_FILE),$(obj)/cpu_microcode_$(2).bin)
cpu_microcode_$(2).bin-type := microcode
cpu_microcode_$(2).bin-align := 16
endef

$(foreach u,$(SP3_UCODE),$(eval $(call sp3-add-ucode,$(word 1,$(subst :, ,$(u))),$(word 2,$(subst :, ,$(u))))))

# The PSP copies the whole reset image region to the end of the bootblock
# region, so the loadable part of the bootblock goes at the end and the
# region is padded in front of it. The flash copy is identical to the DRAM
# copy, so x86 may fetch its reset vector from either.
SP3_RESET_IMAGE = $(obj)/sp3_reset_image.bin

$(SP3_RESET_IMAGE): $(objcbfs)/bootblock_fixed_data.elf $(AMDCOMPRESS)
	@printf "    RESETIMG   $(subst $(obj)/,,$(@))\n"
	$(AMDCOMPRESS) --infile $< --outfile $@.load --elfcopy \
		--maxsize $(CONFIG_C_ENV_BOOTBLOCK_SIZE)
	size=$$(stat -c %s $@.load); \
	pad=$$(( $(CONFIG_C_ENV_BOOTBLOCK_SIZE) - size )); \
	test $$pad -ge 0 && \
	{ head -c $$pad /dev/zero; cat $@.load; } > $@

$(call add_intermediate, add_sp3_amdfw, $(obj)/amdfw.rom)
	$(CBFSTOOL) $< write -r AMDFW -f $(obj)/amdfw.rom

$(call add_intermediate, add_sp3_reset_image, $(SP3_RESET_IMAGE))
	$(CBFSTOOL) $< write -r BOOTBLOCK -f $(SP3_RESET_IMAGE)

$(obj)/sp3_vendor_upper.bin: $(SP3_VENDOR_ROM)
	dd if=$< of=$@ bs=1M skip=16 count=16 status=none

$(call add_intermediate, add_sp3_vendor_upper, $(obj)/sp3_vendor_upper.bin)
	$(CBFSTOOL) $< write -r VENDOR_UPPER -f $(obj)/sp3_vendor_upper.bin

endif # ($(CONFIG_SOC_AMD_SP3),y)
