include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_VERSION:=2025.01
PKG_HASH:=cdef7d507c93f1bbd9f015ea9bc21fa074268481405501945abc6f854d5b686f
PKG_BUILD_DEPENDS:=arm-trusted-firmware-tools/host

UBOOT_USE_INTREE_DTC:=1

include $(INCLUDE_DIR)/u-boot.mk
include $(INCLUDE_DIR)/package.mk
include $(INCLUDE_DIR)/host-build.mk

define U-Boot/an7581_rfb
  NAME:=AN7581 Reference Board
  UBOOT_CONFIG:=an7581_evb
  BUILD_DEVICES:=an7581-evb
  BUILD_TARGET:=airoha
  BUILD_SUBTARGET:=an7581
  UBOOT_IMAGE:=u-boot.fip
  FIP_COMPRESS:=1
  BL2_IMAGE:=an7581-bl2.bin
  BL31_IMAGE:=an7581-bl31.lzma
endef

define U-Boot/an7583_rfb
  NAME:=AN7583 Reference Board
  UBOOT_CONFIG:=an7583_evb
  BUILD_DEVICES:=an7583-evb
  BUILD_TARGET:=airoha
  BUILD_SUBTARGET:=an7583
  UBOOT_IMAGE:=u-boot.fip
  FIP_COMPRESS:=1
  BL2_IMAGE:=an7583-bl2.bin
  BL31_IMAGE:=an7583-bl31.lzma
endef

UBOOT_TARGETS := \
	an7581_rfb \
	an7583_rfb

UBOOT_CUSTOMIZE_CONFIG := \
	--disable TOOLS_KWBIMAGE \
	--disable TOOLS_LIBCRYPTO \
	--disable TOOLS_MKEFICAPSULE \
	--enable SERIAL_RX_BUFFER \
	--set-val SERIAL_RX_BUFFER_SIZE 256

define Build/fip-image
	$(if $(FIP_COMPRESS), $(STAGING_DIR_HOST)/bin/lzma e \
		$(PKG_BUILD_DIR)/u-boot.bin \
		$(PKG_BUILD_DIR)/u-boot.bin.lzma)
	$(STAGING_DIR_HOST)/bin/fiptool create \
		--tb-fw files/$(BL2_IMAGE) \
		--soc-fw files/$(BL31_IMAGE) \
		--nt-fw $(PKG_BUILD_DIR)/u-boot.bin$(if $(FIP_COMPRESS),.lzma) \
		$(PKG_BUILD_DIR)/u-boot.fip
endef

define Build/Configure
	$(call Build/Configure/U-Boot)
	sed -i 's/CONFIG_TOOLS_LIBCRYPTO=y/# CONFIG_TOOLS_LIBCRYPTO is not set/' $(PKG_BUILD_DIR)/.config
endef

define Build/Compile
	$(call Build/Compile/U-Boot)
ifeq ($(UBOOT_IMAGE),u-boot.fip)
	$(call Build/fip-image)
endif
endef

# don't stage files to bindir, let target/linux/airoha/image/*.mk do that
define Package/u-boot/install
endef

define Build/InstallDev
	$(INSTALL_DIR) $(STAGING_DIR_IMAGE)
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/$(UBOOT_IMAGE) $(STAGING_DIR_IMAGE)/$(BUILD_VARIANT)-$(UBOOT_IMAGE)
endef

$(eval $(call BuildPackage/U-Boot))
