# SPDX-License-Identifier: GPL-2.0-only
#
# Copyright (C) 2006-2016 OpenWrt.org

OTHER_MENU:=Other modules

I2C_MT7621_MODULES:= \
  CONFIG_I2C_MT7621:drivers/i2c/busses/i2c-mt7621

define KernelPackage/i2c-en7581
  SUBMENU:=$(OTHER_MENU)
  $(call i2c_defaults,$(I2C_MT7621_MODULES),79)
  TITLE:=Airoha I2C Controller
  DEPENDS:=+kmod-i2c-core \
	@(TARGET_airoha_en7581)
endef

define KernelPackage/i2c-en7581/description
 Kernel modules for enable mt7621 i2c controller.
endef

$(eval $(call KernelPackage,i2c-en7581))
