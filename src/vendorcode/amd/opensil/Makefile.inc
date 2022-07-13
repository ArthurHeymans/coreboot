ifeq ($(CONFIG_SOC_AMD_OPENSIL),y)

ifeq ($(CONFIG_AMD_OPENSIL_USE_EXTERNAL_ARCHIVE),y)

$(obj)/opensil.a: $(call strip_quotes,$(CONFIG_AMD_OPENSIL_BIN_PATH))
	cp $< $@

ramstage-libs += $(obj)/opensil.a

opensil_dir := $(call strip_quotes,$(CONFIG_AMD_OPENSIL_HEADER_PATH))

CPPFLAGS_ramstage += -I$(opensil_dir)/xUSL -I$(opensil_dir)/xUSL/Include -I$(opensil_dir)/xSIM -I$(opensil_dir)/xPRF

ramstage-y += ramstage.c
$(obj)/ramstage/vendorcode/amd/opensil/ramstage.o: CFLAGS_ramstage += -D_MSC_EXTENSIONS=0 -Wno-unknown-pragmas

else

classes-y += opensil
opensil-y =
opensil-generic-ccopts += -D__OPENSIL__
opensil-generic-ccopts += -fno-zero-initialized-in-bss
opensil-generic-ccopts += -Wno-unknown-pragmas
opensil-generic-ccopts += -Wno-undef -Wno-strict-prototypes
opensil-generic-ccopts += -Wno-redundant-decls
# remove this once exposed prototypes are there
opensil-generic-ccopts += -Wno-missing-prototypes
opensil-generic-ccopts += -Wno-shadow
opensil-generic-ccopts += -Wno-incompatible-pointer-types
opensil-generic-ccopts += -Wno-pointer-to-int-cast
opensil-generic-ccopts += -Wno-unused-function
opensil-generic-ccopts += -Wno-implicit-function-declaration
opensil-generic-ccopts += -Wno-type-limits
opensil-generic-ccopts += -Wno-discarded-qualifiers
opensil-generic-ccopts += -Wno-unused-variable
opensil-generic-ccopts += -Wno-overflow
opensil-generic-ccopts += -Wno-implicit-fallthrough
opensil-generic-ccopts += -Wno-sequence-point
opensil-generic-ccopts += -Wno-maybe-uninitialized
opensil-generic-ccopts += -Wno-missing-braces
opensil-generic-ccopts += -Wno-error

ifeq ($(CONFIG_ARCH_ROMSTAGE_X86_32),y)
$(eval $(call create_class_compiler,opensil,x86_32))
else
$(eval $(call create_class_compiler,opensil,x86_64))
endif

CPPFLAGS_opensil += $(CPPFLAGS_xsim)

$(call add-special-class,xsim)
xsim-handler = $(eval opensil-srcs += $(1)$(2))
$(call add-special-class,xusl)
xusl-handler = $(eval opensil-srcs += $(1)$(2))
$(call add-special-class,xprf)
xprf-handler = $(eval opensil-srcs += $(1)$(2))

$(obj)/opensil.a: $$(opensil-objs)
	@printf "    OPENSIL      $(subst $(obj)/,,$(@))\n"
	$(AR_opensil) rcsDT $@ $+

subdirs-y += ../../../../3rdparty/opensil/xSIM
subdirs-y += ../../../../3rdparty/opensil/xPRF
subdirs-y += ../../../../3rdparty/opensil/xUSL

romstage-libs += $(obj)/opensil.a
ramstage-libs += $(obj)/opensil.a

romstage-libs += /home/arthur/src/OpenSIL/build/libopensil_zen4.a
ramstage-libs += /home/arthur/src/OpenSIL/build/libopensil_zen4.a

ramstage-y += ramstage.c
endif

CPPFLAGS_ramstage += $(CPPFLAGS_xsim) $(CPPFLAGS_xusl) $(CPPFLAGS_xprf)

# Add handler to copy linker scripts
define generic-objs_nasm_template_gen
de$(EMPTY)fine $(1)-objs_nasm_template
$$(call src-to-obj,$1,$$(1).nasm): $$(1).nasm $(obj)/config.h $(obj)/fmap_config.h
	@printf "    NASM       $$$$(subst $$$$(obj)/,,$$$$(@))\n"
	$$(CC_$(1)) -MMD $$(CPPFLAGS_$(1)) $$($(1)-ld-ccopts) $(PREPROCESS_ONLY) -include $(obj)/config.h -MT $$$$@.cpp -o $$$$@.cpp $$$$<
	$(NASM) -felf -o $$$$@ $$$$@.cpp
en$(EMPTY)def
endef
endif
