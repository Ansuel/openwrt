include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/image.mk

KERNEL_LOADADDR := 0x80208000

define Target/Description
	Build firmware images for Airoha EN7581 ARM based boards.
endef

# default all platform image(fit) build
define Device/Default
  PROFILES = Default $$(DEVICE_NAME)
  KERNEL_NAME := Image
  KERNEL = kernel-bin | lzma | \
	fit lzma $$(KDIR)/image-$$(firstword $$(DEVICE_DTS)).dtb
  KERNEL_INITRAMFS = kernel-bin | lzma | \
	fit lzma $$(KDIR)/image-$$(firstword $$(DEVICE_DTS)).dtb with-initrd
  FILESYSTEMS := squashfs
  DEVICE_DTS_DIR := $(DTS_DIR)
  IMAGES := sysupgrade.bin
  IMAGE/sysupgrade.bin := append-kernel | pad-to 128k | append-rootfs | \
	pad-rootfs | append-metadata
endef

define Image/Build
	$(call Image/Build/$(1),$(1))
endef

define Device/airoha_en7581-evb
  DEVICE_VENDOR := Airoha
  DEVICE_MODEL := EN7581 Evaluation Board (SNAND)
  DEVICE_DTS := en7581-evb
  DEVICE_DTS_DIR := ../dts
  DEVICE_PACKAGES := kmod-i2c-en7581
endef
TARGET_DEVICES += airoha_en7581-evb

define Device/airoha_en7581-evb-emmc
  DEVICE_VENDOR := Airoha
  DEVICE_MODEL := EN7581 Evaluation Board (EMMC)
  DEVICE_DTS := en7581-evb-emmc
  DEVICE_DTS_DIR := ../dts
  DEVICE_PACKAGES := kmod-i2c-en7581
endef
TARGET_DEVICES += airoha_en7581-evb-emmc

$(eval $(call BuildImage))
