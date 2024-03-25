include $(TOPDIR)/rules.mk

ARCH:=aarch64
BOARD:=airoha
BOARDNAME:=Airoha AN75xx
SUBTARGETS:=en7581 en7523
CPU_TYPE:=cortex-a53
FEATURES:=dt squashfs nand ramdisk gpio source-only

KERNEL_PATCHVER:=6.6

include $(INCLUDE_DIR)/target.mk

KERNELNAME:=Image dtbs
DEFAULT_PACKAGES += \
	kmod-leds-gpio kmod-gpio-button-hotplug

$(eval $(call BuildTarget))
