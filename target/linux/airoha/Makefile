include $(TOPDIR)/rules.mk

BOARD:=airoha
BOARDNAME:=Airoha EN75xx
SUBTARGETS:=en7581 en7523
CPU_TYPE:=cortex-a53
FEATURES:=dt squashfs nand ramdisk gpio source-only pwm

KERNEL_PATCHVER:=6.6

include $(INCLUDE_DIR)/target.mk

KERNELNAME:=Image dtbs
DEFAULT_PACKAGES += \
	kmod-leds-gpio kmod-gpio-button-hotplug

$(eval $(call BuildTarget))
