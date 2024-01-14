#
# Copyright (C) 2009 OpenWrt.org
#

ARCH:=aarch64
SUBTARGET:=en7581
BOARDNAME:=en7581 based boards
CPU_TYPE:=cortex-a53
FEATURES:=squashfs nand ramdisk

KERNEL_PATCHVER:=6.1

define Target/Description
	Build firmware images for Airoha mt7581 ARM based boards.
endef

