## SPDX-License-Identifier: GPL-2.0-only

subdirs-y += gnat

subdirs-y += mipi
subdirs-y += storage
subdirs-y += bsd/zstd

bootblock-y += mem_pool.c
verstage-y += mem_pool.c
romstage-y += mem_pool.c
ramstage-y += mem_pool.c
postcar-y += mem_pool.c
smm-y += mem_pool.c

bootblock-y += iobuf.c
verstage-y += iobuf.c
romstage-y += iobuf.c
ramstage-y += iobuf.c
smm-y += iobuf.c
postcar-y += iobuf.c

bootblock-y += region.c
verstage-y += region.c
romstage-y += region.c
ramstage-y += region.c
smm-y += region.c
postcar-y += region.c

romstage-y += rational.c
ramstage-y += rational.c

ramstage-$(CONFIG_PLATFORM_USES_FSP1_1) += fsp_relocate.c
ifeq ($(CONFIG_FSP_M_XIP),)
romstage-$(CONFIG_PLATFORM_USES_FSP2_0) += fsp_relocate.c
endif
ramstage-$(CONFIG_PLATFORM_USES_FSP2_0) += fsp_relocate.c

bootblock-$(CONFIG_FLATTENED_DEVICE_TREE) += device_tree.c
romstage-$(CONFIG_FLATTENED_DEVICE_TREE) += device_tree.c
ramstage-$(CONFIG_FLATTENED_DEVICE_TREE) += device_tree.c

bootblock-y += bsd/cbfs_private.c
verstage-y += bsd/cbfs_private.c
romstage-y += bsd/cbfs_private.c
postcar-y += bsd/cbfs_private.c
ramstage-y += bsd/cbfs_private.c
smm-y += bsd/cbfs_private.c

bootblock-y += bsd/cbfs_mcache.c
verstage-y += bsd/cbfs_mcache.c
romstage-y += bsd/cbfs_mcache.c
postcar-y += bsd/cbfs_mcache.c
ramstage-y += bsd/cbfs_mcache.c
smm-y += bsd/cbfs_mcache.c

decompressor-y += bsd/lz4_wrapper.c
bootblock-y += bsd/lz4_wrapper.c
verstage-y += bsd/lz4_wrapper.c
romstage-y += bsd/lz4_wrapper.c
ramstage-y += bsd/lz4_wrapper.c
postcar-y += bsd/lz4_wrapper.c

bootblock-y += bsd/lzsa2.c
verstage-y += bsd/lzsa2.c
romstage-y += bsd/lzsa2.c
ramstage-y += bsd/lzsa2.c
postcar-y += bsd/lzsa2.c

all-y += list.c
all-y += bsd/zstd_wrapper.c

ifneq ($(CONFIG_COMPRESS_RAMSTAGE_FHL)$(CONFIG_COMPRESSED_PAYLOAD_FHL),)
postcar-y += bsd/fhl_wrapper.c
romstage-y += bsd/fhl_wrapper.c
ramstage-y += bsd/fhl_wrapper.c
endif
ifeq ($(CONFIG_FHL_ANZ_O2),y)
# Both codecs are tuned at -O2; -Os leaves their hot helpers out of line.
# ANZ also relies on the compiler expanding its fixed-size memcpy() loads.
$(foreach class,postcar romstage ramstage, \
	$(eval $(obj)/$(class)/commonlib/bsd/fhl_wrapper.o: CFLAGS_$(class) += -O2) \
	$(eval $(obj)/$(class)/commonlib/bsd/anz_wrapper.o: CFLAGS_$(class) += -O2 -fhosted -fbuiltin))
endif
ifneq ($(CONFIG_COMPRESS_RAMSTAGE_ANZ)$(CONFIG_COMPRESSED_PAYLOAD_ANZ),)
postcar-y += bsd/anz_wrapper.c
romstage-y += bsd/anz_wrapper.c
ramstage-y += bsd/anz_wrapper.c
ifeq ($(CONFIG_ANZ_SSE2),y)
# The SSE2 decoder needs the compiler's own <emmintrin.h>.
$(foreach class,postcar romstage ramstage, \
	$(eval $(obj)/$(class)/commonlib/bsd/anz_wrapper.o: \
		CFLAGS_$(class) += -isystem $(shell $(CC_x86_32) -print-file-name=include)))
endif
endif

ramstage-y += sort.c

romstage-y += bsd/elog.c
ramstage-y += bsd/elog.c
smm-y += bsd/elog.c

decompressor-y += bsd/gcd.c
all-y += bsd/gcd.c

all-y += bsd/ipchksum.c

decompressor-y += bsd/string.c
smm-y += bsd/string.c
all-y += bsd/string.c

all-y += bsd/xxhash.c
