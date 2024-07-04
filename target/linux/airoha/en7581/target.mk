ARCH:=aarch64
SUBTARGET:=en7581
BOARDNAME:=en7581 based boards
CPU_TYPE:=cortex-a53
FEATURES:=dt squashfs nand ramdisk gpio source-only pwm

define Target/Description
	Build firmware images for Airoha mt7581 ARM based boards.
endef

