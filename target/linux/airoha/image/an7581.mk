define Build/an7581-bl2-bl31-uboot
  head -c $$((0x800)) /dev/zero > $@
  cat $(STAGING_DIR_IMAGE)/an7581_$1-bl2.fip >> $@
  dd if=$(STAGING_DIR_IMAGE)/an7581_$1-bl31-uboot.img of=$@ bs=1 seek=$$((0x20000)) conv=notrunc
endef

define Build/an7581-emmc-bl2-bl31-uboot
  head -c $$((0x800)) /dev/zero > $@
  cat $(STAGING_DIR_IMAGE)/an7581_$1-bl2.fip >> $@
  dd if=$(STAGING_DIR_IMAGE)/an7581_$1-bl31-u-boot.fip of=$@ bs=1 seek=$$((0x20000)) conv=notrunc
endef

define Build/an7581-preloader
  cat $(STAGING_DIR_IMAGE)/an7581_$1-bl2.fip >> $@
endef

define Build/an7581-bl31-uboot
  [ -f $(STAGING_DIR_ROOT)/lib/firmware/as21x1x_fw.bin ] && \
    $(STAGING_DIR_HOST)/bin/fiptool update \
      --blob uuid=58704aef-389f-3e52-b475-e0bf2234a6a2,file=$(STAGING_DIR_ROOT)/lib/firmware/as21x1x_fw.bin \
      $(STAGING_DIR_IMAGE)/an7581_$1-bl31-u-boot.fip
  [ -f $(STAGING_DIR_ROOT)/lib/firmware/airoha/EthMD32.dm.bin ] && \
    cat $(STAGING_DIR_ROOT)/lib/firmware/airoha/EthMD32.dm.bin > $(STAGING_DIR_IMAGE)/en8811h.bin && \
    cat $(STAGING_DIR_ROOT)/lib/firmware/airoha/EthMD32.DSP.bin >> $(STAGING_DIR_IMAGE)/en8811h.bin
  [ -f $(STAGING_DIR_IMAGE)/en8811h.bin ] && \
    $(STAGING_DIR_HOST)/bin/fiptool update  \
      --blob uuid=d39d2cf2-9bd0-3ca7-93e9-e71b4f9250b2,file=$(STAGING_DIR_IMAGE)/en8811h.bin \
      $(STAGING_DIR_IMAGE)/an7581_$1-bl31-u-boot.fip
  cat $(STAGING_DIR_IMAGE)/an7581_$1-bl31-u-boot.fip >> $@
endef

define Device/FitImageLzma
	KERNEL_SUFFIX := -uImage.itb
	KERNEL = kernel-bin | lzma | fit lzma $$(KDIR)/image-$$(DEVICE_DTS).dtb
	KERNEL_NAME := Image
endef

define Device/airoha_an7581-evb
  $(call Device/FitImageLzma)
  DEVICE_VENDOR := Airoha
  DEVICE_MODEL := AN7581 Evaluation Board (SNAND)
  DEVICE_PACKAGES := kmod-leds-pwm kmod-i2c-an7581 kmod-pwm-airoha kmod-input-gpio-keys-polled \
    kmod-usb-ledtrig-usbport
  DEVICE_DTS := an7581-evb
  DEVICE_DTS_DIR := ../dts
  DEVICE_DTS_CONFIG := config@1
  KERNEL_LOADADDR := 0x80088000
  IMAGE/sysupgrade.bin := append-kernel | pad-to 128k | append-rootfs | pad-rootfs | append-metadata
  ARTIFACT/bl2-bl31-uboot.bin := an7581-bl2-bl31-uboot rfb
  ARTIFACT/preloader.bin := an7581-preloader rfb
  ARTIFACT/bl31-uboot.fip := an7581-bl31-uboot rfb
  ARTIFACTS := bl2-bl31-uboot.bin preloader.bin bl31-uboot.fip
endef
TARGET_DEVICES += airoha_an7581-evb

define Device/airoha_an7581-evb-an8811
  $(call Device/airoha_an7581-evb)
  DEVICE_MODEL := AN7581 Evaluation Board (SNAND + AN8811)
  DEVICE_DTS := an7581-evb-an8811
endef
TARGET_DEVICES += airoha_an7581-evb-an8811

define Device/airoha_an7581-evb-pon
  $(call Device/airoha_an7581-evb)
  DEVICE_MODEL := AN7581 Evaluation Board (SNAND + PON)
  DEVICE_DTS := an7581-evb-pon
endef
TARGET_DEVICES += airoha_an7581-evb-pon

define Device/airoha_an7581-evb-emmc
  DEVICE_VENDOR := Airoha
  DEVICE_MODEL := AN7581 Evaluation Board (EMMC)
  DEVICE_DTS := an7581-evb-emmc
  DEVICE_DTS_DIR := ../dts
  DEVICE_PACKAGES := kmod-i2c-an7581
  ARTIFACT/preloader.bin := an7581-preloader rfb
  ARTIFACT/bl31-uboot.fip := an7581-bl31-uboot rfb
  ARTIFACT/bl2-bl31-uboot.bin := an7581-emmc-bl2-bl31-uboot rfb
  ARTIFACTS := bl2-bl31-uboot.bin preloader.bin bl31-uboot.fip
endef
TARGET_DEVICES += airoha_an7581-evb-emmc

define Device/airoha_an7581-evb-emmc-an8831
  DEVICE_VENDOR := Airoha
  DEVICE_MODEL := AN7581 Evaluation Board (EMMC + AN8831)
  DEVICE_DTS := an7581-evb-emmc-an8831
  DEVICE_DTS_DIR := ../dts
  DEVICE_PACKAGES := kmod-i2c-an7581 kmod-phy-aeonsemi-as21xxx
  ARTIFACT/preloader.bin := an7581-preloader rfb
  ARTIFACT/bl31-uboot.fip := an7581-bl31-uboot rfb
  ARTIFACT/bl2-bl31-uboot.bin := an7581-emmc-bl2-bl31-uboot rfb
  ARTIFACTS := bl2-bl31-uboot.bin preloader.bin bl31-uboot.fip
endef
TARGET_DEVICES += airoha_an7581-evb-emmc-an8831

define Device/airoha_an7581-evb-10g-lan
  $(call Device/airoha_an7581-evb)
  DEVICE_MODEL := AN7581 Evaluation Board (SNAND + ETH-SERDES-LAN + PON)
  DEVICE_DTS := an7581-evb-10g-lan
endef
TARGET_DEVICES += airoha_an7581-evb-10g-lan
