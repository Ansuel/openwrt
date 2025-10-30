# SPDX-License-Identifier: GPL-2.0-or-later

include $(TOPDIR)/rules.mk

PKG_NAME:=aeonsemi-as21xxx-firmware
PKG_VERSION:=1.9.0
PKG_RELEASE:=1

PKG_LICENSE_FILES:=LICENSE.aeonsemi

PKG_MAINTAINER:=Christian Marangi <ansuelsmth@gmail.com>

include $(INCLUDE_DIR)/package.mk

define Package/aeonsemi-as21xxx-firmware
  TITLE:=AS21xxx Ethrnet PHY firmware
  SECTION:=firmware
  CATEGORY:=Firmware
endef

define Build/Compile

endef

define Package/aeonsemi-as21xxx-firmware/install
	$(INSTALL_DIR) $(1)/lib/firmware
	$(INSTALL_DATA) \
		./files/as21x1x_fw.bin \
		$(1)/lib/firmware/
endef

$(eval $(call BuildPackage,aeonsemi-as21xxx-firmware))
