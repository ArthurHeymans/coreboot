# SPDX-License-Identifier: BSD-3-Clause

ifeq ($(CONFIG_SOC_AMD_GENOA),y)

bootblock-y	+= bootblock.c
bootblock-y	+= gpio.c

romstage-y	+= romstage.c
romstage-y	+= gpio.c

ramstage-y	+= chip.c
ramstage-y	+= timer.c
ramstage-y	+= gpio.c

CPPFLAGS_common += -I$(src)/soc/amd/genoa/include

endif
