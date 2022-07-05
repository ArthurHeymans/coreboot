bootblock-y += bootblock.c
bootblock-y += memmap.c

romstage-y += ../qemu-i440fx/memmap.c
romstage-y += memmap.c

postcar-y += ../qemu-i440fx/memmap.c
postcar-y += ../qemu-i440fx/exit_car.S
postcar-y += memmap.c

ramstage-y += ../qemu-i440fx/memmap.c
ramstage-y += ../qemu-i440fx/northbridge.c
ramstage-y += memmap.c
ramstage-y += cpu.c

all-y += ../qemu-i440fx/fw_cfg.c
all-y += ../qemu-i440fx/bootmode.c

ramstage-$(CONFIG_CHROMEOS) += chromeos.c

smm-y += smi.c
smm-y += memmap.c

$(obj)/test.dtb: $(dir)/test.dts
	dtc -I dts -O dtb -o $@ $^

cbfs-files-y += devicetree_handoff
devicetree_handoff-file := $(obj)/test.dtb
devicetree_handoff-type := raw
