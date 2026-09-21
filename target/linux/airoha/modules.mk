# SPDX-License-Identifier: GPL-2.0-only

SOUND_MENU:=Sound Support
OTHER_MENU:=Other modules

I2C_MT7621_MODULES:= \
  CONFIG_I2C_MT7621:drivers/i2c/busses/i2c-mt7621

define KernelPackage/i2c-an7581
  SUBMENU:=$(OTHER_MENU)
  $(call i2c_defaults,$(I2C_MT7621_MODULES),79)
  TITLE:=Airoha I2C Controller
  DEPENDS:=+kmod-i2c-core \
	  @(TARGET_airoha_an7581)
endef

define KernelPackage/i2c-an7581/description
 Kernel modules for enable mt7621 i2c controller.
endef

$(eval $(call KernelPackage,i2c-an7581))


define KernelPackage/pwm-an7581
  SUBMENU:=$(OTHER_MENU)
  TITLE:=Airoha EN7581 PWM
  DEPENDS:=@(TARGET_airoha_an7581)
  KCONFIG:= \
        CONFIG_PWM=y \
        CONFIG_PWM_AIROHA=y \
        CONFIG_PWM_SYSFS=y
  FILES:= \
        $(LINUX_DIR)/drivers/pwm/pwm-airoha.ko
  AUTOLOAD:=$(call AutoProbe,pwm-airoha)
endef

define KernelPackage/pwm-an7581/description
 Kernel module to use the PWM channel on Airoha SoC
endef

$(eval $(call KernelPackage,pwm-an7581))


define KernelPackage/sound-soc-an7581
  TITLE:=Airoha AN7581 Audio support
  KCONFIG:=CONFIG_SND_SOC_AN7581 CONFIG_SND_SOC_AN7581_WM8960
  FILES:= \
	$(LINUX_DIR)/sound/soc/mediatek/common/snd-soc-mtk-common.ko \
	$(LINUX_DIR)/sound/soc/mediatek/an7581/snd-soc-an7581-afe.ko
  AUTOLOAD:=$(call AutoLoad,56,snd-soc-mtk-common snd-soc-an7581-afe)
  DEPENDS:=@TARGET_airoha +kmod-sound-soc-core
  $(call AddDepends/sound)
endef

define KernelPackage/sound-soc-an7581/description
 Support for audio on systems using the Airoha AN7581 SoC.
endef

$(eval $(call KernelPackage,sound-soc-an7581))


define KernelPackage/sound-soc-an7581-wm8960
  TITLE:=Airoha AN7581 Audio support
  KCONFIG:=CONFIG_SND_SOC_AN7581_WM8960
  FILES:=$(LINUX_DIR)/sound/soc/mediatek/an7581/an7581-wm8960.ko
  AUTOLOAD:=$(call AutoLoad,57,an7581-wm8960)
  DEPENDS:=@TARGET_airoha +kmod-sound-soc-wm8960 +kmod-sound-soc-an7581
  $(call AddDepends/sound)
endef

define KernelPackage/sound-soc-an7581-wm8960/description
 Support for use of the Wolfson Audio WM8960 codec with the Airoha AN7581 SoC.
endef

$(eval $(call KernelPackage,sound-soc-an7581-wm8960))


define KernelPackage/sound-an7581-pcm
  TITLE:=Airoha AN7581 PCM Audio support
  KCONFIG:=CONFIG_SND_SOC_AN7581_PCM
  FILES:=$(LINUX_DIR)/sound/soc/airoha/an7581-pcm.ko
  AUTOLOAD:=$(call AutoLoad,56,an7581-pcm)
  DEPENDS:=@TARGET_airoha +kmod-sound-soc-core
  $(call AddDepends/sound)
endef

define KernelPackage/sound-soc-an7581-wm8960/description
 Support for PCM audio on systems using the Airoha AN7581 SoC.
endef

$(eval $(call KernelPackage,sound-an7581-pcm))

