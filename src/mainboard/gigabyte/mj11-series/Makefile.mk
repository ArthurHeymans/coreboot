## SPDX-License-Identifier: GPL-2.0-only

CPPFLAGS_common += -I$(src)/mainboard/$(MAINBOARDDIR)/include

subdirs-y += variants/$(VARIANT_DIR)

bootblock-y += bootblock.c

romstage-y += early.c
ramstage-y += gpio.c
ramstage-y += mainboard.c
ramstage-y += port_descriptors.c
smm-y += smihandler.c
