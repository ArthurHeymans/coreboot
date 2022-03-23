ramstage-y += chip.c

$(obj)/ramstage/vendorcode/amd/opensil/mpio/chip.o: CFLAGS_ramstage += -D_MSC_EXTENSIONS=0 -DHAS_STRING_H=1 -Wno-unknown-pragmas
