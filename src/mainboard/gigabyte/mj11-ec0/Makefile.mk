## SPDX-License-Identifier: GPL-2.0-only

bootblock-y += bootblock.c

ifeq ($(CONFIG_BOARD_GIGABYTE_MJ11_EC1),y)
romstage-y += board_ec1_tables.c
ramstage-y += board_ec1_tables.c
else
romstage-y += board_tables.c
ramstage-y += board_tables.c
endif

romstage-y += early.c
ramstage-y += gpio.c
ramstage-y += mainboard.c
ramstage-y += port_descriptors.c
smm-y += smihandler.c
