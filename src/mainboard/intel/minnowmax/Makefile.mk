## SPDX-License-Identifier: GPL-2.0-only

romstage-y += romstage.c

ramstage-y += gpio.c
ramstage-y += irqroute.c

SPD_BIN = $(obj)/spd.bin

SPD_SOURCES = generic_1GiB_ddr3l_2Gb_x16
SPD_SOURCES += kingston_2GiB_D2516EC4BXGGB-U

SPD_DEPS := $(foreach f, $(SPD_SOURCES), src/mainboard/$(MAINBOARDDIR)/spd/$(f).spd.hex)

# Include spd ROM data
$(SPD_BIN): $(SPD_DEPS)
	for f in $+; \
	  do for c in $$(cat $$f | grep -v ^#); \
	    do printf $$(printf '\%o' 0x$$c); \
	  done; \
	done > $@

cbfs-files-y += spd.bin
spd.bin-file := $(SPD_BIN)
spd.bin-type := spd
