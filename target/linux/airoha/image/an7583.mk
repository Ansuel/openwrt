define Build/an7583-bl2-bl31-uboot
  head -c $$((0x800)) /dev/zero > $@
  cat $(STAGING_DIR_IMAGE)/an7583_$1-bl2.fip >> $@
  dd if=$(STAGING_DIR_IMAGE)/an7583_$1-bl31-uboot.img of=$@ bs=1 seek=$$((0x20000)) conv=notrunc
endef

define Build/an7583-emmc-bl2-bl31-uboot
  head -c $$((0x800)) /dev/zero > $@
  cat $(STAGING_DIR_IMAGE)/an7583_$1-bl2.fip >> $@
  dd if=$(STAGING_DIR_IMAGE)/an7583_$1-bl31-u-boot.fip of=$@ bs=1 seek=$$((0x20000)) conv=notrunc
endef

define Build/an7583-preloader
  cat $(STAGING_DIR_IMAGE)/an7583_$1-bl2.fip >> $@
endef

define Build/an7583-bl31-uboot
  [ ! -f $(STAGING_DIR_ROOT)/lib/firmware/as21x1x_fw.bin ] || \
    $(STAGING_DIR_HOST)/bin/fiptool update  \
      --blob uuid=58704aef-389f-3e52-b475-e0bf2234a6a2,file=$(STAGING_DIR_ROOT)/lib/firmware/as21x1x_fw.bin \
      $(STAGING_DIR_IMAGE)/an7583_$1-bl31-u-boot.fip
  [ ! -f $(STAGING_DIR_ROOT)/lib/firmware/airoha/EthMD32.dm.bin ] || ( \
    cat $(STAGING_DIR_ROOT)/lib/firmware/airoha/EthMD32.dm.bin > $(STAGING_DIR_IMAGE)/en8811h.bin && \
    cat $(STAGING_DIR_ROOT)/lib/firmware/airoha/EthMD32.DSP.bin >> $(STAGING_DIR_IMAGE)/en8811h.bin && \
    $(STAGING_DIR_HOST)/bin/fiptool update  \
      --blob uuid=d39d2cf2-9bd0-3ca7-93e9-e71b4f9250b2,file=$(STAGING_DIR_IMAGE)/en8811h.bin \
      $(STAGING_DIR_IMAGE)/an7583_$1-bl31-u-boot.fip )
  cat $(STAGING_DIR_IMAGE)/an7583_$1-bl31-u-boot.fip >> $@
endef

define Device/FitImageLzma
	KERNEL_SUFFIX := -uImage.itb
	KERNEL = kernel-bin | lzma | fit lzma $$(KDIR)/image-$$(DEVICE_DTS).dtb
	KERNEL_NAME := Image
endef

define Device/airoha_an7583-evb
  $(call Device/FitImageLzma)
  DEVICE_VENDOR := Airoha
  DEVICE_MODEL := AN7583 Evaluation Board (SNAND)
  DEVICE_PACKAGES := kmod-leds-pwm kmod-input-gpio-keys-polled kmod-phy-aeonsemi-as21xxx
  DEVICE_DTS := an7583-evb
  DEVICE_DTS_DIR := ../dts
  DEVICE_DTS_CONFIG := config@1
  KERNEL_LOADADDR := 0x80088000
  IMAGE/sysupgrade.bin := append-kernel | pad-to 128k | append-rootfs | pad-rootfs | append-metadata
  ARTIFACT/bl2-bl31-uboot.bin := an7583-bl2-bl31-uboot rfb
  ARTIFACT/preloader.bin := an7583-preloader rfb
  ARTIFACT/bl31-uboot.fip := an7583-bl31-uboot rfb
  ARTIFACTS := bl2-bl31-uboot.bin preloader.bin bl31-uboot.fip
endef
TARGET_DEVICES += airoha_an7583-evb

define Device/airoha_an7583-evb-emmc
  DEVICE_VENDOR := Airoha
  DEVICE_MODEL := AN7583 Evaluation Board (EMMC)
  DEVICE_DTS := an7583-evb-emmc
  DEVICE_DTS_DIR := ../dts
  DEVICE_PACKAGES := kmod-i2c-an7581
  ARTIFACT/preloader.bin := an7583-preloader rfb
  ARTIFACT/bl31-uboot.fip := an7583-bl31-uboot rfb
  ARTIFACT/bl2-bl31-uboot.bin := an7583-emmc-bl2-bl31-uboot rfb
  ARTIFACTS := bl2-bl31-uboot.bin preloader.bin bl31-uboot.fip
endef
TARGET_DEVICES += airoha_an7583-evb-emmc
