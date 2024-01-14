include $(TOPDIR)/rules.mk

ARCH:=aarch64
BOARD:=airoha
BOARDNAME:=Airoha AN75xx
SUBTARGETS:=en7523 en7581
CPU_TYPE:=cortex-a53
FEATURES:=dt squashfs nand ramdisk gpio source-only

KERNEL_PATCHVER:=6.6

include $(INCLUDE_DIR)/target.mk

KERNELNAME:=Image dtbs

$(eval $(call BuildTarget))
