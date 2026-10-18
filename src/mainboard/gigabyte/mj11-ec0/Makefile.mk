## SPDX-License-Identifier: GPL-2.0-only

bootblock-y += bootblock.c

romstage-y += board_tables.c
romstage-y += early.c

ramstage-y += board_tables.c
ramstage-y += gpio.c
ramstage-y += mainboard.c
ramstage-y += port_descriptors.c
