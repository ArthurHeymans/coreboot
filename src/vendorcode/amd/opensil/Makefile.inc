ifeq ($(CONFIG_SOC_AMD_OPENSIL),y)

ifeq ($(CONFIG_AMD_OPENSIL_USE_EXTERNAL_ARCHIVE),y)

$(obj)/opensil.a: $(call strip_quotes,$(CONFIG_AMD_OPENSIL_BIN_PATH))
	cp $< $@

else

ifeq ($(CONFIG_ARCH_RAMSTAGE_X86_32),y)
cpu_family_string="x86"
cpu_string="i686"
else
cpu_family_string="x86_64"
cpu_string="x86_64"
endif


$(obj)/meson_crosscompile: $(dir)/meson_cross.template
	sed -e "s,##COMPILER##,$(CC_ramstage)," \
	    -e "s,##AR##,$(AR_ramstage)," \
	    -e "s,##NASM##,$(NASM)," \
	    -e "s,##COREBOOT_DIR##,$(top)," \
	    -e "s,##CPU_FAMILY##,$(cpu_family_string)," \
	    -e "s,##CPU##,$(cpu_string)," \
		$< > $@

# make sure to always rerun meson
PHONY += compile_opensil
compile_opensil: $(obj)/meson_crosscompile $(obj)/config.h
	-rm -rf $(obj)/opensil &> /dev/null
	cd $(opensil_dir); meson --cross-file $(top)/$< $(top)/$(obj)/opensil
	meson compile -C $(obj)/opensil opensil_zen4

$(obj)/opensil.a: compile_opensil
	cp $(obj)/opensil/libopensil_zen4.a $@

endif

ramstage-libs += $(obj)/opensil.a

opensil_dir := $(call strip_quotes,$(CONFIG_AMD_OPENSIL_HEADER_PATH))

CPPFLAGS_ramstage += -I$(opensil_dir)/xUSL -I$(opensil_dir)/xUSL/Include -I$(opensil_dir)/xSIM -I$(opensil_dir)/xPRF

ramstage-y += ramstage.c
$(obj)/ramstage/vendorcode/amd/opensil/ramstage.o: CFLAGS_ramstage += -D_MSC_EXTENSIONS=0 -Wno-unknown-pragmas

endif
