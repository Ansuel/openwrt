#
# Copyright (C) 2015 CZ.NIC, z. s. p. o. <https://www.nic.cz/>
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

DVB_MENU:=DVB Support

# ------------------------------ core drivers ---------------------------------

define KernelPackage/rc-core
  SUBMENU:=$(DVB_MENU)
  TITLE:=Remote Controller support
  KCONFIG:= \
	CONFIG_MEDIA_SUPPORT=m \
	CONFIG_MEDIA_RC_SUPPORT=y \
	CONFIG_RC_CORE
  FILES:=$(LINUX_DIR)/drivers/media/rc/rc-core.ko
  AUTOLOAD:=$(call AutoLoad,50,rc-core)
  DEPENDS:=+kmod-input-core
endef

define KernelPackage/rc-core/description
 Enable support for Remote Controllers on Linux. This is
 needed in order to support several video capture adapters,
 standalone IR receivers/transmitters, and RF receivers.

 Enable this option if you have a video capture board even
 if you don't need IR, as otherwise, you may not be able to
 compile the driver for your adapter.
endef

$(eval $(call KernelPackage,rc-core))

define KernelPackage/dvb-core
  SUBMENU:=$(DVB_MENU)
  TITLE:=DVB core support
  KCONFIG:= \
	CONFIG_MEDIA_SUPPORT=m \
	CONFIG_MEDIA_DIGITAL_TV_SUPPORT=y \
	CONFIG_DVB_NET=y \
	CONFIG_DVB_MAX_ADAPTERS=8 \
	CONFIG_DVB_CORE
  FILES:=$(LINUX_DIR)/drivers/media/dvb-core/dvb-core.ko
  AUTOLOAD:=$(call AutoLoad,50,dvb-core)
endef

define KernelPackage/dvb-core/description
 Kernel modules for DVB support.
endef

$(eval $(call KernelPackage,dvb-core))

define AddDepends/dvb-core
  SUBMENU:=$(DVB_MENU)
  DEPENDS+=+kmod-dvb-core $1
endef

# ----------------------------- DVB USB drivers -------------------------------

define KernelPackage/dvb-usb
  SUBMENU:=$(DVB_MENU)
  TITLE:=Support for various USB DVB devices
  KCONFIG:= \
	CONFIG_MEDIA_SUPPORT=m \
	CONFIG_MEDIA_DIGITAL_TV_SUPPORT=y \
	CONFIG_MEDIA_USB_SUPPORT=y \
	CONFIG_DVB_USB
  FILES:=$(LINUX_DIR)/drivers/media/usb/dvb-usb/dvb-usb.ko
  DEPENDS:=+kmod-usb-core +kmod-i2c-core +kmod-rc-core +kmod-dvb-core
  AUTOLOAD:=$(call AutoLoad,50,dvb-usb)
endef

define KernelPackage/dvb-usb/description
 By enabling this you will be able to choose the various supported
 USB1.1 and USB2.0 DVB devices.

 Almost every USB device needs a firmware.

 For a complete list of supported USB devices see the LinuxTV DVB Wiki:
 <http://www.linuxtv.org/wiki/index.php/DVB_USB>
endef

$(eval $(call KernelPackage,dvb-usb))

define AddDepends/dvb-usb
  SUBMENU:=$(DVB_MENU)
  DEPENDS+=+kmod-dvb-usb $1
endef


define KernelPackage/dvb-usb-dib0700
  TITLE:=DiBcom DiB0700 USB DVB devices
  KCONFIG:=CONFIG_DVB_USB_DIB0700
  FILES:=$(LINUX_DIR)/drivers/media/usb/dvb-usb/dvb-usb-dib0700.ko
  AUTOLOAD:=$(call AutoLoad,61,dvb-usb-dib0700)
  DEPENDS:= \
	+PACKAGE_kmod-dvb-tuner-dib0070:kmod-dvb-tuner-dib0070 \
	+PACKAGE_kmod-dvb-tuner-dib0090:kmod-dvb-tuner-dib0090 \
	@(PACKAGE_kmod-dvb-dib3000mc||PACKAGE_kmod-dvb-dib7000m||PACKAGE_kmod-dvb-dib7000p||PACKAGE_kmod-dvb-dib8000||PACKAGE_kmod-dvb-dib9000) \
	+PACKAGE_kmod-dvb-dib3000mc:kmod-dvb-dib3000mc \
	+PACKAGE_kmod-dvb-dib7000m:kmod-dvb-dib7000m \
	+PACKAGE_kmod-dvb-dib7000p:kmod-dvb-dib7000p \
	+PACKAGE_kmod-dvb-dib8000:kmod-dvb-dib8000 \
	+PACKAGE_kmod-dvb-dib9000:kmod-dvb-dib9000
  $(call AddDepends/dvb-usb)
endef

define KernelPackage/dvb-usb-dib0700/description
 Support for USB2.0/1.1 DVB receivers based on the DiB0700 USB bridge. The
 USB bridge is also present in devices having the DiB7700 DVB-T-USB
 silicon. This chip can be found in devices offered by Hauppauge,
 Avermedia and other big and small companies.

 For an up-to-date list of devices supported by this driver, have a look
 on the LinuxTV Wiki at www.linuxtv.org.

 Say Y if you own such a device and want to use it. You should build it as
 a module.
endef

$(eval $(call KernelPackage,dvb-usb-dib0700))


# --------------------------- DVB USB v2 drivers ------------------------------

define KernelPackage/dvb-usb-v2
  TITLE:=Support for various USB DVB devices v2
  KCONFIG:= \
	CONFIG_MEDIA_USB_SUPPORT=y \
	CONFIG_DVB_USB_V2
  FILES:=$(LINUX_DIR)/drivers/media/usb/dvb-usb-v2/dvb_usb_v2.ko
  AUTOLOAD:=$(call AutoLoad,61,dvb_usb_v2)
  DEPENDS:=+PACKAGE_kmod-rc-core:kmod-rc-core
  $(call AddDepends/dvb-core,+kmod-usb-core)
endef

define KernelPackage/dvb-usb-v2/description
 By enabling this you will be able to choose the various supported
 USB1.1 and USB2.0 DVB devices.

 Almost every USB device needs a firmware.

 For a complete list of supported USB devices see the LinuxTV DVB Wiki:
 <http://www.linuxtv.org/wiki/index.php/DVB_USB>
endef

$(eval $(call KernelPackage,dvb-usb-v2))

define AddDepends/dvb-usb-v2
  SUBMENU:=$(DVB_MENU)
  DEPENDS+=+kmod-dvb-usb-v2 $1
endef

define KernelPackage/dvb-usb-it913x
  TITLE:=ITE IT913X DVB-T USB2.0 support
  KCONFIG:=CONFIG_DVB_USB_IT913X
  FILES:=$(LINUX_DIR)/drivers/media/usb/dvb-usb-v2/dvb-usb-it913x.ko
  AUTOLOAD:=$(call AutoLoad,62,dvb-usb-it913x)
  $(call AddDepends/dvb-usb-v2,+kmod-dvb-it913x-fe)
endef

define KernelPackage/dvb-usb-it913x/description
 Support for the ITE IT913X DVB-T USB2.0.
endef

$(eval $(call KernelPackage,dvb-usb-it913x))


define KernelPackage/dvb-usb-rtl28xxu
  TITLE:=Realtek RTL28xxU DVB USB support
  KCONFIG:=CONFIG_DVB_USB_RTL28XXU
  FILES:=$(LINUX_DIR)/drivers/media/usb/dvb-usb-v2/dvb-usb-rtl28xxu.ko
  AUTOLOAD:=$(call AutoLoad,62,dvb-usb-rtl28xxu)
  $(call AddDepends/dvb-usb-v2,+kmod-dvb-rtl2830 +kmod-dvb-rtl2832)
endef

define KernelPackage/dvb-usb-rtl28xxu/description
 Realtek RTL28xxU DVB USB support
endef

$(eval $(call KernelPackage,dvb-usb-rtl28xxu))

define KernelPackage/dvb-usb-anysee
  TITLE:=Anysee DVB-T/C USB2.0 support
  KCONFIG:=CONFIG_DVB_USB_ANYSEE
  FILES:=$(LINUX_DIR)/drivers/media/usb/dvb-usb-v2/dvb-usb-anysee.ko
  AUTOLOAD:=$(call AutoLoad,60,dvb-usb-anysee)
  $(call AddDepends/dvb-usb-v2)
endef

define KernelPackage/dvb-usb-anysee/description
 Anysee DVB-T/C USB2.0 support
endef

$(eval $(call KernelPackage,dvb-usb-anysee))

define KernelPackage/dvb-usb-af9015
  TITLE:=Afatech AF9015 DVB-T USB2.0 support
  KCONFIG:=CONFIG_DVB_USB_AF9015
  FILES:=$(LINUX_DIR)/drivers/media/usb/dvb-usb-v2/dvb-usb-af9015.ko
  AUTOLOAD:=$(call AutoLoad,60,dvb-usb-af9015)
  $(call AddDepends/dvb-usb-v2,+kmod-dvb-af9013)
endef

define KernelPackage/dvb-usb-af9015/description
 Support for the Afatech AF9015 based DVB-T USB2.0 receiver.
endef

$(eval $(call KernelPackage,dvb-usb-af9015))

define KernelPackage/dvb-usb-af9035
  TITLE:=Afatech AF9035 DVB-T USB2.0 support
  KCONFIG:=CONFIG_DVB_USB_AF9035
  FILES:=$(LINUX_DIR)/drivers/media/usb/dvb-usb-v2/dvb-usb-af9035.ko
  AUTOLOAD:=$(call AutoLoad,60,dvb-usb-af9035)
  $(call AddDepends/dvb-usb-v2,+kmod-dvb-af9033)
endef

define KernelPackage/dvb-usb-af9035/description
 Say Y here to support the Afatech AF9035 based DVB USB receiver.
endef

$(eval $(call KernelPackage,dvb-usb-af9035))

# ------------------------------ DVB frontends --------------------------------

define DvbFrontend
  SUBMENU:=$(DVB_MENU)
  KCONFIG:= \
	$2
  DEPENDS:=+kmod-i2c-core +kmod-dvb-core
  FILES:=$(LINUX_DIR)/drivers/media/dvb-frontends/$1.ko
  AUTOLOAD:=$(call AutoLoad,61,$1)
endef

define KernelPackage/dvb-pll
  TITLE:=Generic I2C PLL based tuners
  $(call DvbFrontend,dvb-pll,CONFIG_DVB_PLL)
endef

define KernelPackage/dvb-pll/description
 This module drives a number of tuners based on PLL chips with a
 common I2C interface.
endef

$(eval $(call KernelPackage,dvb-pll))

define KernelPackage/dvb-it913x-fe
  TITLE:=it913x frontend and it9137 tuner
  $(call DvbFrontend,it913x-fe,CONFIG_DVB_IT913X_FE)
endef

define KernelPackage/dvb-it913x-fe/description
 A DVB-T tuner module.
endef

$(eval $(call KernelPackage,dvb-it913x-fe))

define KernelPackage/dvb-rtl2830
  TITLE:=Realtek RTL2830 DVB-T
  $(call DvbFrontend,rtl2830,CONFIG_DVB_RTL2830)
endef

define KernelPackage/dvb-rtl2830/description
 Realtek RTL2830 DVB-T
endef

$(eval $(call KernelPackage,dvb-rtl2830))

define KernelPackage/dvb-rtl2832
  TITLE:=Realtek RTL2832 DVB-T
  $(call DvbFrontend,rtl2832,CONFIG_DVB_RTL2832)
endef

define KernelPackage/dvb-rtl2832/description
 Realtek RTL2832 DVB-T
endef

$(eval $(call KernelPackage,dvb-rtl2832))

define KernelPackage/dvb-zl10353
  TITLE:=Zarlink ZL10353 based tuner
  $(call DvbFrontend,zl10353,CONFIG_DVB_ZL10353)
endef

define KernelPackage/dvb-zl10353/description
 A DVB-T tuner module.
endef

$(eval $(call KernelPackage,dvb-zl10353))

define KernelPackage/dvb-tda10023
  TITLE:=Philips TDA10023 based tuner
  $(call DvbFrontend,tda10023,CONFIG_DVB_TDA10023)
endef

define KernelPackage/dvb-tda10023/description
 A DVB-C tuner module.
endef

$(eval $(call KernelPackage,dvb-tda10023))

define KernelPackage/dvb-af9013
  TITLE:=Afatech AF9013 demodulator
  $(call DvbFrontend,af9013,CONFIG_DVB_AF9013)
endef

define KernelPackage/dvb-af9013/description
 Support for AF9013 DVB frontend.
endef

$(eval $(call KernelPackage,dvb-af9013))

define KernelPackage/dvb-af9033
  TITLE:=Afatech AF9033 DVB-T demodulator
  $(call DvbFrontend,af9033,CONFIG_DVB_AF9033)
endef

define KernelPackage/dvb-af9033/description
 Support for AF9033 DVB frontend.
endef

$(eval $(call KernelPackage,dvb-af9033))

define KernelPackage/dvb-tuner-dib0070
  TITLE:=DiBcom DiB0070 silicon base-band tuner
  $(call DvbFrontend,dib0070,CONFIG_DVB_TUNER_DIB0070)
endef

define KernelPackage/dvb-tuner-dib0070/description
 A driver for the silicon baseband tuner DiB0070 from DiBcom.
 This device is only used inside a SiP called together with a
 demodulator for now.
endef

$(eval $(call KernelPackage,dvb-tuner-dib0070))

define KernelPackage/dvb-tuner-dib0090
  TITLE:=DiBcom DiB0090 silicon base-band tuner
  $(call DvbFrontend,dib0090,CONFIG_DVB_TUNER_DIB0090)
endef

define KernelPackage/dvb-tuner-dib0090/description
 A driver for the silicon baseband tuner DiB0090 from DiBcom.
 This device is only used inside a SiP called together with a
 demodulator for now.
endef

$(eval $(call KernelPackage,dvb-tuner-dib0090))

define KernelPackage/dvb-dib3000mb
  TITLE:=DiBcom 3000M-B
  $(call DvbFrontend,dib3000mb,CONFIG_DVB_DIB3000MB)
endef

define KernelPackage/dvb-dib3000mb/description
 A DVB-T tuner module. Designed for mobile usage. Say Y when you want
 to support this frontend.
endef

$(eval $(call KernelPackage,dvb-dib3000mb))

define KernelPackage/dvb-dibx000-common
  SUBMENU:=$(DVB_MENU)
  TITLE:=Common library for DiBX000 drivers
  FILES:=$(LINUX_DIR)/drivers/media/dvb-frontends/dibx000_common.ko
  AUTOLOAD:=$(call AutoLoad,61,dibx000_common)
endef

$(eval $(call KernelPackage,dvb-dibx000-common))

define KernelPackage/dvb-dib3000mc
  TITLE:=DiBcom 3000P/M-C
  $(call DvbFrontend,dib3000mc,CONFIG_DVB_DIB3000MC)
  DEPENDS+=+kmod-dvb-dibx000-common
endef

define KernelPackage/dvb-dib3000mc/description
 A DVB-T tuner module. Designed for mobile usage. Say Y when you want
 to support this frontend.
endef

$(eval $(call KernelPackage,dvb-dib3000mc))

define KernelPackage/dvb-dib7000m
  TITLE:=DiBcom 7000MA/MB/PA/PB/MC
  $(call DvbFrontend,dib7000m,CONFIG_DVB_DIB7000M)
  DEPENDS+=+kmod-dvb-dibx000-common
endef

define KernelPackage/dvb-dib7000m/description
 A DVB-T tuner module. Designed for mobile usage. Say Y when you want
 to support this frontend
endef

$(eval $(call KernelPackage,dvb-dib7000m))

define KernelPackage/dvb-dib7000p
  TITLE:=DiBcom 7000PC
  $(call DvbFrontend,dib7000p,CONFIG_DVB_DIB7000P)
  DEPENDS+=+kmod-dvb-dibx000-common
endef

define KernelPackage/dvb-dib7000p/description
 A DVB-T tuner module. Designed for mobile usage. Say Y when you want
 to support this frontend.
endef

$(eval $(call KernelPackage,dvb-dib7000p))

define KernelPackage/dvb-dib8000
  TITLE:=DiBcom 8000MB/MC
  $(call DvbFrontend,dib8000,CONFIG_DVB_DIB8000)
  DEPENDS+=+kmod-dvb-dibx000-common
endef

define KernelPackage/dvb-dib8000/description
 A driver for DiBcom's DiB8000 ISDB-T/ISDB-Tsb demodulator.
 Say Y when you want to support this frontend.
endef

$(eval $(call KernelPackage,dvb-dib8000))

define KernelPackage/dvb-dib9000
  TITLE:=DiBcom 9000
  $(call DvbFrontend,dib9000,CONFIG_DVB_DIB9000)
  DEPENDS+=+kmod-dvb-dibx000-common
endef

define KernelPackage/dvb-dib9000/description
 A DVB-T tuner module. Designed for mobile usage. Say Y when you want
 to support this frontend
endef

$(eval $(call KernelPackage,dvb-dib9000))

# -----------------------------------------------------------------------------
# ------------------------------- Media tuners --------------------------------
# -----------------------------------------------------------------------------

TUNER_MENU:=Media tuners

define MediaTuner
  SUBMENU:=$(TUNER_MENU)
  KCONFIG:= \
	CONFIG_MEDIA_SUPPORT=m \
	CONFIG_MEDIA_DIGITAL_TV_SUPPORT=m \
	$2
  DEPENDS:=+kmod-i2c-core
  FILES:=$(LINUX_DIR)/drivers/media/tuners/$1.ko
  AUTOLOAD:=$(call AutoLoad,60,$1)
endef

define KernelPackage/media-tuner-e4000
  TITLE:=Elonics E4000 silicon tuner
  $(call MediaTuner,e4000,CONFIG_MEDIA_TUNER_E4000)
endef

define KernelPackage/media-tuner-e4000/description
 Elonics E4000 silicon tuner driver.
endef

$(eval $(call KernelPackage,media-tuner-e4000))

define KernelPackage/media-tuner-fc0011
  TITLE:=Fitipower FC0011 silicon tuner
  $(call MediaTuner,fc0011,CONFIG_MEDIA_TUNER_FC0011)
endef

define KernelPackage/media-tuner-fc0011/description
 Fitipower FC0011 silicon tuner driver.
endef

$(eval $(call KernelPackage,media-tuner-fc0011))

define KernelPackage/media-tuner-fc0012
  TITLE:=Fitipower FC0012 silicon tuner
  $(call MediaTuner,fc0012,CONFIG_MEDIA_TUNER_FC0012)
endef

define KernelPackage/media-tuner-fc0012/description
 Fitipower FC0012 silicon tuner driver.
endef

$(eval $(call KernelPackage,media-tuner-fc0012))

define KernelPackage/media-tuner-fc0013
  TITLE:=Fitipower FC0013 silicon tuner
  $(call MediaTuner,fc0013,CONFIG_MEDIA_TUNER_FC0013)
endef

define KernelPackage/media-tuner-fc0013/description
 Fitipower FC0013 silicon tuner driver.
endef

$(eval $(call KernelPackage,media-tuner-fc0013))

define KernelPackage/media-tuner-fc2580
  TITLE:=FCI FC2580 silicon tuner
  $(call MediaTuner,fc2580,CONFIG_MEDIA_TUNER_FC2580)
endef

define KernelPackage/media-tuner-fc2580/description
 FCI FC2580 silicon tuner driver.
endef

$(eval $(call KernelPackage,media-tuner-fc2580))

define KernelPackage/media-tuner-it913x
  TITLE:=ITE Tech IT913x silicon tuner
  $(call MediaTuner,tuner_it913x,CONFIG_MEDIA_TUNER_IT913X)
endef

define KernelPackage/media-tuner-it913x/description
 ITE Tech IT913x silicon tuner driver.
endef

$(eval $(call KernelPackage,media-tuner-it913x))

define KernelPackage/media-tuner-max2165
  TITLE:=Maxim MAX2165 silicon tuner
  $(call MediaTuner,max2165,CONFIG_MEDIA_TUNER_MAX2165)
endef

define KernelPackage/media-tuner-max2165/description
 A driver for the silicon tuner MAX2165 from Maxim.
endef

$(eval $(call KernelPackage,media-tuner-max2165))

define KernelPackage/media-tuner-mc44s803
  TITLE:=Freescale MC44S803 Broadband tuners
  $(call MediaTuner,mc44s803,CONFIG_MEDIA_TUNER_MC44S803)
endef

define KernelPackage/media-tuner-mc44s803/description
 Say Y here to support the Freescale MC44S803 based tuners
endef

$(eval $(call KernelPackage,media-tuner-mc44s803))

define KernelPackage/media-tuner-mt2060
  TITLE:=Microtune MT2060 silicon IF tuner
  $(call MediaTuner,mt2060,CONFIG_MEDIA_TUNER_MT2060)
endef

define KernelPackage/media-tuner-mt2060/description
 A driver for the silicon IF tuner MT2060 from Microtune.
endef

$(eval $(call KernelPackage,media-tuner-mt2060))

define KernelPackage/media-tuner-mt2063
  TITLE:=Microtune MT2063 silicon IF tuner
  $(call MediaTuner,mt2063,CONFIG_MEDIA_TUNER_MT2063)
endef

define KernelPackage/media-tuner-mt2063/description
 A driver for the silicon IF tuner MT2063 from Microtune.
endef

$(eval $(call KernelPackage,media-tuner-mt2063))

define KernelPackage/media-tuner-mt20xx
  TITLE:=Microtune 2032 / 2050 tuners
  $(call MediaTuner,mt20xx,CONFIG_MEDIA_TUNER_MT20XX)
endef

define KernelPackage/media-tuner-mt20xx/description
 Say Y here to include support for the MT2032 / MT2050 tuner.
endef

$(eval $(call KernelPackage,media-tuner-mt20xx))

define KernelPackage/media-tuner-mt2131
  TITLE:=Microtune MT2131 silicon tuner
  $(call MediaTuner,mt2131,CONFIG_MEDIA_TUNER_MT2131)
endef

define KernelPackage/media-tuner-mt2131/description
 A driver for the silicon baseband tuner MT2131 from Microtune.
endef

$(eval $(call KernelPackage,media-tuner-mt2131))

define KernelPackage/media-tuner-mt2266
  TITLE:=Microtune MT2266 silicon tuner
  $(call MediaTuner,mt2266,CONFIG_MEDIA_TUNER_MT2266)
endef

define KernelPackage/media-tuner-mt2266/description
 A driver for the silicon baseband tuner MT2266 from Microtune.
endef

$(eval $(call KernelPackage,media-tuner-mt2266))

define KernelPackage/media-tuner-mxl5005s
  TITLE:=MaxLinear MSL5005S silicon tuner
  $(call MediaTuner,mxl5005s,CONFIG_MEDIA_TUNER_MXL5005S)
endef

define KernelPackage/media-tuner-mxl5005s/description
 A driver for the silicon tuner MXL5005S from MaxLinear.
endef

$(eval $(call KernelPackage,media-tuner-mxl5005s))

define KernelPackage/media-tuner-mxl5007t
  TITLE:=MaxLinear MxL5007T silicon tuner
  $(call MediaTuner,mxl5007t,CONFIG_MEDIA_TUNER_MXL5007T)
endef

define KernelPackage/media-tuner-mxl5007t/description
 A driver for the silicon tuner MxL5007T from MaxLinear.
endef

$(eval $(call KernelPackage,media-tuner-mxl5007t))

define KernelPackage/media-tuner-qt1010
  TITLE:=Quantek QT1010 silicon tuner
  $(call MediaTuner,qt1010,CONFIG_MEDIA_TUNER_QT1010)
endef

define KernelPackage/media-tuner-qt1010/description
 A driver for the silicon tuner QT1010 from Quantek.
endef

$(eval $(call KernelPackage,media-tuner-qt1010))

define KernelPackage/media-tuner-r820t
  TITLE:=Rafael Micro R820T silicon tuner
  $(call MediaTuner,r820t,CONFIG_MEDIA_TUNER_R820T)
endef

define KernelPackage/media-tuner-r820t/description
 Rafael Micro R820T silicon tuner driver.
endef

$(eval $(call KernelPackage,media-tuner-r820t))

define KernelPackage/media-tuner-simple
  SUBMENU:=$(TUNER_MENU)
  TITLE:=Simple tuner support
  KCONFIG:= \
	CONFIG_MEDIA_SUPPORT=m \
	CONFIG_MEDIA_TUNER_SIMPLE
  DEPENDS:=+kmod-i2c-core +kmod-media-tuner-tda9887
  FILES:= \
	$(LINUX_DIR)/drivers/media/tuners/tuner-simple.ko \
	$(LINUX_DIR)/drivers/media/tuners/tuner-types.ko
  AUTOLOAD:=$(call AutoLoad,60,tuner-simple tuner-types)
endef

define KernelPackage/media-tuner-simple/description
 Say Y here to include support for various simple tuners.
endef

$(eval $(call KernelPackage,media-tuner-simple))

define KernelPackage/media-tuner-tda18212
  TITLE:=NXP TDA18212 silicon tuner
  $(call MediaTuner,tda18212,CONFIG_MEDIA_TUNER_TDA18212)
endef

define KernelPackage/media-tuner-tda18212/description
 NXP TDA18212 silicon tuner driver.
endef

$(eval $(call KernelPackage,media-tuner-tda18212))

define KernelPackage/media-tuner-tda18218
  TITLE:=NXP TDA18218 silicon tuner
  $(call MediaTuner,tda18218,CONFIG_MEDIA_TUNER_TDA18218)
endef

define KernelPackage/media-tuner-tda18218/description
 NXP TDA18218 silicon tuner driver.
endef

$(eval $(call KernelPackage,media-tuner-tda18218))

define KernelPackage/media-tuner-tda18271
  TITLE:=NXP TDA18271 silicon tuner
  $(call MediaTuner,tda18271,CONFIG_MEDIA_TUNER_TDA18271)
endef

define KernelPackage/media-tuner-tda18271/description
 A silicon tuner module. Say Y when you want to support this tuner.
endef

$(eval $(call KernelPackage,media-tuner-tda18271))

define KernelPackage/media-tuner-tda827x
  TITLE:=Philips TDA827X silicon tuner
  $(call MediaTuner,tda827x,CONFIG_MEDIA_TUNER_TDA827X)
endef

define KernelPackage/media-tuner-tda827x/description
 A DVB-T silicon tuner module. Say Y when you want to support this tuner.
endef

$(eval $(call KernelPackage,media-tuner-tda827x))

define KernelPackage/media-tuner-tda8290
  TITLE:=TDA 8290/8295 + 8275(a)/18271 tuner combo
  DEPENDS:=+kmod-media-tuner-tda827x +kmod-media-tuner-tda18271
  $(call MediaTuner,tda8290,CONFIG_MEDIA_TUNER_TDA8290)
endef

define KernelPackage/media-tuner-tda8290/description
 Say Y here to include support for Philips TDA8290+8275(a) tuner.
endef

$(eval $(call KernelPackage,media-tuner-tda8290))

define KernelPackage/media-tuner-tda9887
  TITLE:=TDA 9885/6/7 analog IF demodulator
  $(call MediaTuner,tda9887,CONFIG_MEDIA_TUNER_TDA9887)
endef

define KernelPackage/media-tuner-tda9887/description
 Say Y here to include support for Philips TDA9885/6/7
 analog IF demodulator.
endef

$(eval $(call KernelPackage,media-tuner-tda9887))

define KernelPackage/media-tuner-tea5761
  TITLE:=TEA 5761 radio tuner
  $(call MediaTuner,tea5761,CONFIG_MEDIA_TUNER_TEA5761)
endef

define KernelPackage/media-tuner-tea5761/description
 Say Y here to include support for the Philips TEA5761 radio tuner.
endef

$(eval $(call KernelPackage,media-tuner-tea5761))

define KernelPackage/media-tuner-tea5767
  TITLE:=TEA 5767 radio tuner
  $(call MediaTuner,tea5767,CONFIG_MEDIA_TUNER_TEA5767)
endef

define KernelPackage/media-tuner-tea5767/description
 Say Y here to include support for the Philips TEA5767 radio tuner.
endef

$(eval $(call KernelPackage,media-tuner-tea5767))

define KernelPackage/media-tuner-tua9001
  TITLE:=Infineon TUA 9001 silicon tuner
  $(call MediaTuner,tua9001,CONFIG_MEDIA_TUNER_TUA9001)
endef

define KernelPackage/media-tuner-tua9001/description
 Infineon TUA 9001 silicon tuner driver.
endef

$(eval $(call KernelPackage,media-tuner-tua9001))

define KernelPackage/media-tuner-xc2028
  TITLE:=XCeive xc2028/xc3028 tuners
  $(call MediaTuner,tuner-xc2028,CONFIG_MEDIA_TUNER_XC2028)
endef

define KernelPackage/media-tuner-xc2028/description
 Support for the xc2028/xc3028 tuners.
endef

$(eval $(call KernelPackage,media-tuner-xc2028))

define KernelPackage/media-tuner-xc4000
  TITLE:=Xceive XC4000 silicon tuner
  $(call MediaTuner,xc4000,CONFIG_MEDIA_TUNER_XC4000)
endef

define KernelPackage/media-tuner-xc4000/description
 A driver for the silicon tuner XC4000 from Xceive.
endef

$(eval $(call KernelPackage,media-tuner-xc4000))

define KernelPackage/media-tuner-xc5000
  TITLE:=Xceive XC5000 silicon tuner
  $(call MediaTuner,xc5000,CONFIG_MEDIA_TUNER_XC5000)
endef

define KernelPackage/media-tuner-xc5000/description
 A driver for the silicon tuner XC5000 from Xceive.
 This device is only used inside a SiP called together with a
 demodulator for now.
endef

$(eval $(call KernelPackage,media-tuner-xc5000))
