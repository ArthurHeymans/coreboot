# SPDX-License-Identifier: BSD-3-Clause

ifeq ($(CONFIG_SOC_AMD_GENOA),y)

bootblock-y	+= bootblock.c
bootblock-y	+= gpio.c
bootblock-y	+= early_fch.c
bootblock-y	+= aoac.c
bootblock-y	+= uart.c

romstage-y	+= romstage.c
romstage-y	+= gpio.c
romstage-y	+= uart.c

ramstage-y	+= chip.c
ramstage-y	+= gpio.c
ramstage-y	+= uart.c

CPPFLAGS_common += -I$(src)/soc/amd/genoa/include

endif
