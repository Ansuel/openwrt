include $(TOPDIR)/rules.mk

PKG_NAME:=airoha-npu-firmware
PKG_SOURCE_DATE:=2024-12-04
PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

RSTRIP:=:
STRIP:=:

define Package/airoha-npu-firmware-default
  SECTION:=firmware
  CATEGORY:=Firmware
endef

define Package/airoha-npu-firmware-an7583
  $(Package/airoha-npu-firmware-default)
  TITLE:=AN7583 airoha firmware
endef

define Build/Compile

endef

define Package/airoha-npu-firmware-an7583/install
	$(INSTALL_DIR) $(1)/lib/firmware/airoha
	$(INSTALL_DATA) \
		./files/an7583_npu_data.bin ./files/an7583_npu_rv32.bin \
		$(1)/lib/firmware/airoha/
endef

$(eval $(call BuildPackage,airoha-npu-firmware-an7583))
