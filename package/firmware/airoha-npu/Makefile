include $(TOPDIR)/rules.mk

PKG_NAME:=airoha-npu-firmware
PKG_SOURCE_DATE:=2024-12-04
PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

RSTRIP:=:
STRIP:=:

define Package/airoha-npu-firmware-en7581
$(Package/ath11k-firmware-default)
  SECTION:=firmware
  CATEGORY:=Firmware
  TITLE:=EN7581 airoha firmware
endef

define Build/Compile

endef

define Package/airoha-npu-firmware-en7581/install
	$(INSTALL_DIR) $(1)/lib/firmware/airoha
	$(INSTALL_DATA) \
		./files/en7581_npu_data.bin ./files/en7581_npu_rv32.bin \
		$(1)/lib/firmware/airoha/
endef

$(eval $(call BuildPackage,airoha-npu-firmware-en7581))
