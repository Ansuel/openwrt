// SPDX-License-Identifier: GPL-2.0-only
/*
 * Author: Lorenzo Bianconi <lorenzo@kernel.org>
 * Author: Benjamin Larsson <benjamin.larsson@genexis.eu>
 */

#include <dt-bindings/pinctrl/mt65xx.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>

#include "core.h"
#include "pinconf.h"
#include "pinmux.h"

#define PINCTRL_PIN_GROUP(id)						\
	PINCTRL_PINGROUP(#id, id##_pins, ARRAY_SIZE(id##_pins))

#define PINCTRL_FUNC_DESC(id)						\
	{								\
		.desc = { #id, id##_groups, ARRAY_SIZE(id##_groups) },	\
		.groups = id##_func_group,				\
		.group_size = ARRAY_SIZE(id##_func_group),		\
	}

#define PINCTRL_CONF_DESC(p, offset, mask)				\
	{								\
		.pin = p,						\
		.reg = { offset, mask },				\
	}

/* MUX */
#define REG_GPIO_2ND_I2C_MODE			0x00
#define GPIO_MDC_IO_MASTER_MODE_MODE		BIT(14)
#define GPIO_I2C_MASTER_MODE_MODE		BIT(13)
#define GPIO_I2S_MODE_MASK			BIT(12)
#define GPIO_I2C_SLAVE_MODE_MODE		BIT(11)
#define GPIO_LAN3_LED1_MODE_MASK		BIT(10)
#define GPIO_LAN3_LED0_MODE_MASK		BIT(9)
#define GPIO_LAN2_LED1_MODE_MASK		BIT(8)
#define GPIO_LAN2_LED0_MODE_MASK		BIT(7)
#define GPIO_LAN1_LED1_MODE_MASK		BIT(6)
#define GPIO_LAN1_LED0_MODE_MASK		BIT(5)
#define GPIO_LAN0_LED1_MODE_MASK		BIT(4)
#define GPIO_LAN0_LED0_MODE_MASK		BIT(3)
#define PON_TOD_1PPS_MODE_MASK			BIT(2)
#define GSW_TOD_1PPS_MODE_MASK			BIT(1)
#define GPIO_2ND_I2C_MODE_MASK			BIT(0)

#define REG_GPIO_SPI_CS1_MODE			0x04
#define GPIO_PCM_SPI_CS4_MODE_MASK		BIT(21)
#define GPIO_PCM_SPI_CS3_MODE_MASK		BIT(20)
#define GPIO_PCM_SPI_CS2_MODE_P156_MASK		BIT(19)
#define GPIO_PCM_SPI_CS2_MODE_P128_MASK		BIT(18)
#define GPIO_PCM_SPI_CS1_MODE_MASK		BIT(17)
#define GPIO_PCM_SPI_MODE_MASK			BIT(16)
#define GPIO_PCM2_MODE_MASK			BIT(13)
#define GPIO_PCM1_MODE_MASK			BIT(12)
#define GPIO_PCM_INT_MODE_MASK			BIT(9)
#define GPIO_PCM_RESET_MODE_MASK		BIT(8)
#define GPIO_SPI_QUAD_MODE_MASK			BIT(4)
#define GPIO_SPI_CS4_MODE_MASK			BIT(3)
#define GPIO_SPI_CS3_MODE_MASK			BIT(2)
#define GPIO_SPI_CS2_MODE_MASK			BIT(1)
#define GPIO_SPI_CS1_MODE_MASK			BIT(0)

#define REG_GPIO_PON_MODE			0x08
#define GPIO_PARALLEL_NAND_MODE_MASK		BIT(14)
#define GPIO_SGMII_MDIO_MODE_MASK		BIT(13)
#define GPIO_PCIE_RESET2_MASK			BIT(12)
#define SIPO_RCLK_MODE_MASK			BIT(11)
#define GPIO_PCIE_RESET1_MASK			BIT(10)
#define GPIO_PCIE_RESET0_MASK			BIT(9)
#define GPIO_UART5_MODE_MASK			BIT(8)
#define GPIO_UART4_MODE_MASK			BIT(7)
#define GPIO_HSUART_CTS_RTS_MODE_MASK		BIT(6)
#define GPIO_HSUART_MODE_MASK			BIT(5)
#define GPIO_UART2_CTS_RTS_MODE_MASK		BIT(4)
#define GPIO_UART2_MODE_MASK			BIT(3)
#define GPIO_SIPO_MODE_MASK			BIT(2)
#define GPIO_EMMC_MODE_MASK			BIT(1)
#define GPIO_PON_MODE_MASK			BIT(0)

#define REG_NPU_UART_EN				0x10
#define JTAG_UDI_EN_MASK			BIT(4)
#define JTAG_DFD_EN_MASK			BIT(3)

#define REG_FORCE_GPIO0_EN			0x14
#define REG_FORCE_GPIO32_EN			0x18
#define REG_INIC_MDIO_SLV_MOD			0x1c

/* CONF */
#define REG_I2C_SDA_E2				0x00
#define SPI_MISO_E2_MASK			BIT(14)
#define SPI_MOSI_E2_MASK			BIT(13)
#define SPI_CLK_E2_MASK				BIT(12)
#define SPI_CS0_E2_MASK				BIT(11)
#define PCIE2_RESET_E2_MASK			BIT(10)
#define PCIE1_RESET_E2_MASK			BIT(9)
#define PCIE0_RESET_E2_MASK			BIT(8)
#define UART1_RXD_E2_MASK			BIT(3)
#define UART1_TXD_E2_MASK			BIT(2)
#define I2C_SCL_E2_MASK				BIT(1)
#define I2C_SDA_E2_MASK				BIT(0)

#define REG_I2C_SDA_E4				0x04
#define SPI_MISO_E4_MASK			BIT(14)
#define SPI_MOSI_E4_MASK			BIT(13)
#define SPI_CLK_E4_MASK				BIT(12)
#define SPI_CS0_E4_MASK				BIT(11)
#define PCIE2_RESET_E4_MASK			BIT(10)
#define PCIE1_RESET_E4_MASK			BIT(9)
#define PCIE0_RESET_E4_MASK			BIT(8)
#define UART1_RXD_E4_MASK			BIT(3)
#define UART1_TXD_E4_MASK			BIT(2)
#define I2C_SCL_E4_MASK				BIT(1)
#define I2C_SDA_E4_MASK				BIT(0)

#define REG_GPIO_L_E2				0x08
#define REG_GPIO_L_E4				0x0c
#define REG_GPIO_H_E2				0x10
#define REG_GPIO_H_E4				0x14

#define REG_I2C_SDA_PU				0x28
#define SPI_MISO_PU_MASK			BIT(14)
#define SPI_MOSI_PU_MASK			BIT(13)
#define SPI_CLK_PU_MASK				BIT(12)
#define SPI_CS0_PU_MASK				BIT(11)
#define PCIE2_RESET_PU_MASK			BIT(10)
#define PCIE1_RESET_PU_MASK			BIT(9)
#define PCIE0_RESET_PU_MASK			BIT(8)
#define UART1_RXD_PU_MASK			BIT(3)
#define UART1_TXD_PU_MASK			BIT(2)
#define I2C_SCL_PU_MASK				BIT(1)
#define I2C_SDA_PU_MASK				BIT(0)

#define REG_I2C_SDA_PD				0x2c
#define SPI_MISO_PD_MASK			BIT(14)
#define SPI_MOSI_PD_MASK			BIT(13)
#define SPI_CLK_PD_MASK				BIT(12)
#define SPI_CS0_PD_MASK				BIT(11)
#define PCIE2_RESET_PD_MASK			BIT(10)
#define PCIE1_RESET_PD_MASK			BIT(9)
#define PCIE0_RESET_PD_MASK			BIT(8)
#define UART1_RXD_PD_MASK			BIT(3)
#define UART1_TXD_PD_MASK			BIT(2)
#define I2C_SCL_PD_MASK				BIT(1)
#define I2C_SDA_PD_MASK				BIT(0)

#define REG_GPIO_L_PU				0x30
#define REG_GPIO_L_PD				0x34
#define REG_GPIO_H_PU				0x38
#define REG_GPIO_H_PD				0x3c

#define REG_PCIE_RESET_OD			0x00
#define PCIE2_RESET_OD_MASK			BIT(2)
#define PCIE1_RESET_OD_MASK			BIT(1)
#define PCIE0_RESET_OD_MASK			BIT(0)

/* PWM MODE CONF */
#define REG_GPIO_FLASH_MODE_CFG			0x00
#define GPIO15_FLASH_MODE_CFG			BIT(15)
#define GPIO14_FLASH_MODE_CFG			BIT(14)
#define GPIO13_FLASH_MODE_CFG			BIT(13)
#define GPIO12_FLASH_MODE_CFG			BIT(12)
#define GPIO11_FLASH_MODE_CFG			BIT(11)
#define GPIO10_FLASH_MODE_CFG			BIT(10)
#define GPIO9_FLASH_MODE_CFG			BIT(9)
#define GPIO8_FLASH_MODE_CFG			BIT(8)
#define GPIO7_FLASH_MODE_CFG			BIT(7)
#define GPIO6_FLASH_MODE_CFG			BIT(6)
#define GPIO5_FLASH_MODE_CFG			BIT(5)
#define GPIO4_FLASH_MODE_CFG			BIT(4)
#define GPIO3_FLASH_MODE_CFG			BIT(3)
#define GPIO2_FLASH_MODE_CFG			BIT(2)
#define GPIO1_FLASH_MODE_CFG			BIT(1)
#define GPIO0_FLASH_MODE_CFG			BIT(0)

/* PWM MODE CONF EXT */
#define REG_GPIO_FLASH_MODE_CFG_EXT		0x00
#define GPIO51_FLASH_MODE_CFG			BIT(31)
#define GPIO50_FLASH_MODE_CFG			BIT(30)
#define GPIO49_FLASH_MODE_CFG			BIT(29)
#define GPIO48_FLASH_MODE_CFG			BIT(28)
#define GPIO47_FLASH_MODE_CFG			BIT(27)
#define GPIO46_FLASH_MODE_CFG			BIT(26)
#define GPIO45_FLASH_MODE_CFG			BIT(25)
#define GPIO44_FLASH_MODE_CFG			BIT(24)
#define GPIO43_FLASH_MODE_CFG			BIT(23)
#define GPIO42_FLASH_MODE_CFG			BIT(22)
#define GPIO41_FLASH_MODE_CFG			BIT(21)
#define GPIO40_FLASH_MODE_CFG			BIT(20)
#define GPIO39_FLASH_MODE_CFG			BIT(19)
#define GPIO38_FLASH_MODE_CFG			BIT(18)
#define GPIO37_FLASH_MODE_CFG			BIT(17)
#define GPIO36_FLASH_MODE_CFG			BIT(16)
#define GPIO31_FLASH_MODE_CFG			BIT(15)
#define GPIO30_FLASH_MODE_CFG			BIT(14)
#define GPIO29_FLASH_MODE_CFG			BIT(13)
#define GPIO28_FLASH_MODE_CFG			BIT(12)
#define GPIO27_FLASH_MODE_CFG			BIT(11)
#define GPIO26_FLASH_MODE_CFG			BIT(10)
#define GPIO25_FLASH_MODE_CFG			BIT(9)
#define GPIO24_FLASH_MODE_CFG			BIT(8)
#define GPIO23_FLASH_MODE_CFG			BIT(7)
#define GPIO22_FLASH_MODE_CFG			BIT(6)
#define GPIO21_FLASH_MODE_CFG			BIT(5)
#define GPIO20_FLASH_MODE_CFG			BIT(4)
#define GPIO19_FLASH_MODE_CFG			BIT(3)
#define GPIO18_FLASH_MODE_CFG			BIT(2)
#define GPIO17_FLASH_MODE_CFG			BIT(1)
#define GPIO16_FLASH_MODE_CFG			BIT(0)

#define AIROHA_NUM_GPIOS			64
#define AIROHA_GPIO_BANK_SIZE			(AIROHA_NUM_GPIOS / 2)
#define AIROHA_REG_GPIOCTRL_NUM_GPIO		(AIROHA_NUM_GPIOS / 4)

struct airoha_pinctrl_reg {
	u32 offset;
	u32 mask;
};

enum airoha_pinctrl_mux_func {
	AIROHA_FUNC_MUX,
	AIROHA_FUNC_PWM_MUX,
	AIROHA_FUNC_PWM_EXT_MUX,
};

struct airoha_pinctrl_func_group {
	const char *name;
	enum airoha_pinctrl_mux_func mux_func;
	struct airoha_pinctrl_reg reg;
};

struct airoha_pinctrl_func {
	const struct function_desc desc;
	const struct airoha_pinctrl_func_group *groups;
	u8 group_size;
};

struct airoha_pinctrl_conf {
	u32 pin;
	struct airoha_pinctrl_reg reg;
};

struct airoha_pinctrl {
	struct pinctrl_dev *ctrl;

	struct mutex mutex;
	struct {
		void __iomem *mux[3];
		void __iomem *conf;
		void __iomem *pcie_rst;
	} regs;

	struct {
		struct gpio_chip chip;

		void __iomem *data[2];
		void __iomem *dir[4];
		void __iomem *out[2];
	} gpiochip;
};

static struct pinctrl_pin_desc airoha_pinctrl_pins[] = {
	PINCTRL_PIN(0, "uart1_txd"),
	PINCTRL_PIN(1, "uart1_rxd"),
	PINCTRL_PIN(2, "i2c_scl"),
	PINCTRL_PIN(3, "i2c_sda"),
	PINCTRL_PIN(4, "spi_cs0"),
	PINCTRL_PIN(5, "spi_clk"),
	PINCTRL_PIN(6, "spi_mosi"),
	PINCTRL_PIN(7, "spi_miso"),
	PINCTRL_PIN(13, "gpio0"),
	PINCTRL_PIN(14, "gpio1"),
	PINCTRL_PIN(15, "gpio2"),
	PINCTRL_PIN(16, "gpio3"),
	PINCTRL_PIN(17, "gpio4"),
	PINCTRL_PIN(18, "gpio5"),
	PINCTRL_PIN(19, "gpio6"),
	PINCTRL_PIN(20, "gpio7"),
	PINCTRL_PIN(21, "gpio8"),
	PINCTRL_PIN(22, "gpio9"),
	PINCTRL_PIN(23, "gpio10"),
	PINCTRL_PIN(24, "gpio11"),
	PINCTRL_PIN(25, "gpio12"),
	PINCTRL_PIN(26, "gpio13"),
	PINCTRL_PIN(27, "gpio14"),
	PINCTRL_PIN(28, "gpio15"),
	PINCTRL_PIN(29, "gpio16"),
	PINCTRL_PIN(30, "gpio17"),
	PINCTRL_PIN(31, "gpio18"),
	PINCTRL_PIN(32, "gpio19"),
	PINCTRL_PIN(33, "gpio20"),
	PINCTRL_PIN(34, "gpio21"),
	PINCTRL_PIN(35, "gpio22"),
	PINCTRL_PIN(36, "gpio23"),
	PINCTRL_PIN(37, "gpio24"),
	PINCTRL_PIN(38, "gpio25"),
	PINCTRL_PIN(39, "gpio26"),
	PINCTRL_PIN(40, "gpio27"),
	PINCTRL_PIN(41, "gpio28"),
	PINCTRL_PIN(42, "gpio29"),
	PINCTRL_PIN(43, "gpio30"),
	PINCTRL_PIN(44, "gpio31"),
	PINCTRL_PIN(45, "gpio32"),
	PINCTRL_PIN(46, "gpio33"),
	PINCTRL_PIN(47, "gpio34"),
	PINCTRL_PIN(48, "gpio35"),
	PINCTRL_PIN(49, "gpio36"),
	PINCTRL_PIN(50, "gpio37"),
	PINCTRL_PIN(51, "gpio38"),
	PINCTRL_PIN(52, "gpio39"),
	PINCTRL_PIN(53, "gpio40"),
	PINCTRL_PIN(54, "gpio41"),
	PINCTRL_PIN(55, "gpio42"),
	PINCTRL_PIN(56, "gpio43"),
	PINCTRL_PIN(57, "gpio44"),
	PINCTRL_PIN(58, "gpio45"),
	PINCTRL_PIN(59, "gpio46"),
	PINCTRL_PIN(61, "pcie_reset0"),
	PINCTRL_PIN(62, "pcie_reset1"),
	PINCTRL_PIN(63, "pcie_reset2"),
};

static const int pon_pins[] = { 49, 50, 51, 52, 53, 54 };
static const int pon_tod_1pps_pins[] = { 46 };
static const int gsw_tod_1pps_pins[] = { 46 };
static const int sipo_pins[] = { 16, 17 };
static const int sipo_rclk_pins[] = { 16, 17, 43 };
static const int mdio1_pins[] = { 14, 15 };
static const int uart2_pins[] = { 48, 55 };
static const int uart2_cts_rts_pins[] = { 46, 47 };
static const int hsuart_pins[] = { 28, 29 };
static const int hsuart_cts_rts_pins[] = { 26, 27 };
static const int uart4_pins[] = { 38, 39 };
static const int uart5_pins[] = { 18, 19 };
static const int i2c0_pins[] = { 2, 3 };
static const int i2c1_pins[] = { 14, 15 };
static const int jtag_udi_pins[] = { 16, 17, 18, 19, 20 };
static const int jtag_dfd_pins[] = { 16, 17, 18, 19, 20 };
static const int i2s_pins[] = { 26, 27, 28, 29 };
static const int pcm1_pins[] = { 22, 23, 24, 25 };
static const int pcm2_pins[] = { 18, 19, 20, 21 };
static const int spi_quad_pins[] = { 32, 33 };
static const int spi_pins[] = { 4, 5, 6, 7 };
static const int spi_cs1_pins[] = { 34 };
static const int pcm_spi_pins[] = { 18, 19, 20, 21, 22, 23, 24, 25 };
static const int pcm_spi_int_pins[] = { 14 };
static const int pcm_spi_rst_pins[] = { 15 };
static const int pcm_spi_cs1_pins[] = { 43 };
static const int pcm_spi_cs2_pins[] = { 40 };
static const int pcm_spi_cs2_p128_pins[] = { 40 };
static const int pcm_spi_cs2_p156_pins[] = { 40 };
static const int pcm_spi_cs3_pins[] = { 41 };
static const int pcm_spi_cs4_pins[] = { 42 };
static const int emmc_pins[] = { 4, 5, 6, 30, 31, 32, 33, 34, 35, 36, 37 };
static const int pnand_pins[] = { 4, 5, 6, 7, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42 };
static const int gpio0_pins[] = { 13 };
static const int gpio1_pins[] = { 14 };
static const int gpio2_pins[] = { 15 };
static const int gpio3_pins[] = { 16 };
static const int gpio4_pins[] = { 17 };
static const int gpio5_pins[] = { 18 };
static const int gpio6_pins[] = { 19 };
static const int gpio7_pins[] = { 20 };
static const int gpio8_pins[] = { 21 };
static const int gpio9_pins[] = { 22 };
static const int gpio10_pins[] = { 23 };
static const int gpio11_pins[] = { 24 };
static const int gpio12_pins[] = { 25 };
static const int gpio13_pins[] = { 26 };
static const int gpio14_pins[] = { 27 };
static const int gpio15_pins[] = { 28 };
static const int gpio16_pins[] = { 29 };
static const int gpio17_pins[] = { 30 };
static const int gpio18_pins[] = { 31 };
static const int gpio19_pins[] = { 32 };
static const int gpio20_pins[] = { 33 };
static const int gpio21_pins[] = { 34 };
static const int gpio22_pins[] = { 35 };
static const int gpio23_pins[] = { 36 };
static const int gpio24_pins[] = { 37 };
static const int gpio25_pins[] = { 38 };
static const int gpio26_pins[] = { 39 };
static const int gpio27_pins[] = { 40 };
static const int gpio28_pins[] = { 41 };
static const int gpio29_pins[] = { 42 };
static const int gpio30_pins[] = { 43 };
static const int gpio31_pins[] = { 44 };
static const int gpio33_pins[] = { 46 };
static const int gpio34_pins[] = { 47 };
static const int gpio35_pins[] = { 48 };
static const int gpio36_pins[] = { 49 };
static const int gpio37_pins[] = { 50 };
static const int gpio38_pins[] = { 51 };
static const int gpio39_pins[] = { 52 };
static const int gpio40_pins[] = { 53 };
static const int gpio41_pins[] = { 54 };
static const int gpio42_pins[] = { 55 };
static const int gpio43_pins[] = { 56 };
static const int gpio44_pins[] = { 57 };
static const int gpio45_pins[] = { 58 };
static const int gpio46_pins[] = { 59 };
static const int pcie_reset0_pins[] = { 61 };
static const int pcie_reset1_pins[] = { 62 };
static const int pcie_reset2_pins[] = { 63 };

static const struct pingroup airoha_pinctrl_groups[] = {
	PINCTRL_PIN_GROUP(pon),
	PINCTRL_PIN_GROUP(pon_tod_1pps),
	PINCTRL_PIN_GROUP(gsw_tod_1pps),
	PINCTRL_PIN_GROUP(sipo),
	PINCTRL_PIN_GROUP(sipo_rclk),
	PINCTRL_PIN_GROUP(mdio1),
	PINCTRL_PIN_GROUP(uart2),
	PINCTRL_PIN_GROUP(uart2_cts_rts),
	PINCTRL_PIN_GROUP(hsuart),
	PINCTRL_PIN_GROUP(hsuart_cts_rts),
	PINCTRL_PIN_GROUP(uart4),
	PINCTRL_PIN_GROUP(uart5),
	PINCTRL_PIN_GROUP(i2c0),
	PINCTRL_PIN_GROUP(i2c1),
	PINCTRL_PIN_GROUP(jtag_udi),
	PINCTRL_PIN_GROUP(jtag_dfd),
	PINCTRL_PIN_GROUP(i2s),
	PINCTRL_PIN_GROUP(pcm1),
	PINCTRL_PIN_GROUP(pcm2),
	PINCTRL_PIN_GROUP(spi),
	PINCTRL_PIN_GROUP(spi_quad),
	PINCTRL_PIN_GROUP(spi_cs1),
	PINCTRL_PIN_GROUP(pcm_spi),
	PINCTRL_PIN_GROUP(pcm_spi_int),
	PINCTRL_PIN_GROUP(pcm_spi_rst),
	PINCTRL_PIN_GROUP(pcm_spi_cs1),
	PINCTRL_PIN_GROUP(pcm_spi_cs2_p128),
	PINCTRL_PIN_GROUP(pcm_spi_cs2_p156),
	PINCTRL_PIN_GROUP(pcm_spi_cs2),
	PINCTRL_PIN_GROUP(pcm_spi_cs3),
	PINCTRL_PIN_GROUP(pcm_spi_cs4),
	PINCTRL_PIN_GROUP(emmc),
	PINCTRL_PIN_GROUP(pnand),
	PINCTRL_PIN_GROUP(gpio0),
	PINCTRL_PIN_GROUP(gpio1),
	PINCTRL_PIN_GROUP(gpio2),
	PINCTRL_PIN_GROUP(gpio3),
	PINCTRL_PIN_GROUP(gpio4),
	PINCTRL_PIN_GROUP(gpio5),
	PINCTRL_PIN_GROUP(gpio6),
	PINCTRL_PIN_GROUP(gpio7),
	PINCTRL_PIN_GROUP(gpio8),
	PINCTRL_PIN_GROUP(gpio9),
	PINCTRL_PIN_GROUP(gpio10),
	PINCTRL_PIN_GROUP(gpio11),
	PINCTRL_PIN_GROUP(gpio12),
	PINCTRL_PIN_GROUP(gpio13),
	PINCTRL_PIN_GROUP(gpio14),
	PINCTRL_PIN_GROUP(gpio15),
	PINCTRL_PIN_GROUP(gpio16),
	PINCTRL_PIN_GROUP(gpio17),
	PINCTRL_PIN_GROUP(gpio18),
	PINCTRL_PIN_GROUP(gpio19),
	PINCTRL_PIN_GROUP(gpio20),
	PINCTRL_PIN_GROUP(gpio21),
	PINCTRL_PIN_GROUP(gpio22),
	PINCTRL_PIN_GROUP(gpio23),
	PINCTRL_PIN_GROUP(gpio24),
	PINCTRL_PIN_GROUP(gpio25),
	PINCTRL_PIN_GROUP(gpio26),
	PINCTRL_PIN_GROUP(gpio27),
	PINCTRL_PIN_GROUP(gpio28),
	PINCTRL_PIN_GROUP(gpio29),
	PINCTRL_PIN_GROUP(gpio30),
	PINCTRL_PIN_GROUP(gpio31),
	PINCTRL_PIN_GROUP(gpio33),
	PINCTRL_PIN_GROUP(gpio34),
	PINCTRL_PIN_GROUP(gpio35),
	PINCTRL_PIN_GROUP(gpio36),
	PINCTRL_PIN_GROUP(gpio37),
	PINCTRL_PIN_GROUP(gpio38),
	PINCTRL_PIN_GROUP(gpio39),
	PINCTRL_PIN_GROUP(gpio40),
	PINCTRL_PIN_GROUP(gpio41),
	PINCTRL_PIN_GROUP(gpio42),
	PINCTRL_PIN_GROUP(gpio43),
	PINCTRL_PIN_GROUP(gpio44),
	PINCTRL_PIN_GROUP(gpio45),
	PINCTRL_PIN_GROUP(gpio46),
	PINCTRL_PIN_GROUP(pcie_reset0),
	PINCTRL_PIN_GROUP(pcie_reset1),
	PINCTRL_PIN_GROUP(pcie_reset2),
};

static const char *const pon_groups[] = { "pon" };
static const char *const tod_1pps_groups[] = { "pon_tod_1pps", "gsw_tod_1pps" };
static const char *const sipo_groups[] = { "sipo", "sipo_rclk" };
static const char *const mdio_groups[] = { "mdio1" };
static const char *const uart_groups[] = { "uart2", "uart2_cts_rts", "hsuart",
					   "hsuart_cts_rts", "uart4",
					   "uart5" };
static const char *const i2c_groups[] = { "i2c1" };
static const char *const jtag_groups[] = { "jtag_udi", "jtag_dfd" };
static const char *const pcm_groups[] = { "pcm1", "pcm2" };
static const char *const spi_groups[] = { "spi_quad", "spi_cs1", "spi_cs2",
					  "spi_cs3", "spi_cs4" };
static const char *const pcm_spi_groups[] = { "pcm_spi", "pcm_spi_int",
					      "pcm_spi_rst", "pcm_spi_cs1",
					      "pcm_spi_cs2_p156",
					      "pcm_spi_cs2_p128" ,
					      "pcm_spi_cs3", "pcm_spi_cs4" };
static const char *const i2s_groups[] = { "i2s" };
static const char *const emmc_groups[] = { "emmc" };
static const char *const pnand_groups[] = { "pnand" };
static const char *const pcie_reset_groups[] = { "pcie_reset0", "pcie_reset1",
						 "pcie_reset2" };
static const char *const led_groups[] = { "gpio33", "gpio43",
					  "gpio34", "gpio44",
					  "gpio35", "gpio45",
					  "gpio42", "gpio46" };

static const char *const pwm_groups[] = { "gpio0", "gpio1",
					  "gpio2", "gpio3",
					  "gpio4", "gpio5",
					  "gpio6", "gpio7",
					  "gpio8", "gpio9",
					  "gpio10", "gpio11",
					  "gpio12", "gpio13",
					  "gpio14", "gpio15",
					  "gpio16", "gpio17",
					  "gpio18", "gpio19",
					  "gpio20", "gpio21",
					  "gpio22", "gpio23",
					  "gpio24", "gpio25",
					  "gpio26", "gpio27",
					  "gpio28", "gpio29",
					  "gpio30", "gpio31",
					  "gpio36", "gpio37",
					  "gpio38", "gpio39",
					  "gpio40", "gpio41",
					  "gpio42", "gpio43",
					  "gpio44", "gpio45",
					  "gpio46", "gpio47" };

static const struct airoha_pinctrl_func_group pon_func_group[] = {
	{ "pon", AIROHA_FUNC_MUX, { REG_GPIO_PON_MODE, GPIO_PON_MODE_MASK }},
};

static const struct airoha_pinctrl_func_group tod_1pps_func_group[] = {
	{ "pon_tod_1pps", AIROHA_FUNC_MUX, { REG_GPIO_2ND_I2C_MODE,
					     PON_TOD_1PPS_MODE_MASK }},
	{ "gsw_tod_1pps", AIROHA_FUNC_MUX, { REG_GPIO_2ND_I2C_MODE,
					     GSW_TOD_1PPS_MODE_MASK }},
};

static const struct airoha_pinctrl_func_group sipo_func_group[] = {
	{ "sipo", AIROHA_FUNC_MUX, { REG_GPIO_PON_MODE, GPIO_SIPO_MODE_MASK }},
	{ "sipo_rclk", AIROHA_FUNC_MUX, { REG_GPIO_PON_MODE,
					  GPIO_SIPO_MODE_MASK |
					  SIPO_RCLK_MODE_MASK }},
};

static const struct airoha_pinctrl_func_group mdio_func_group[] = {
	{ "mdio1", AIROHA_FUNC_MUX, { REG_GPIO_PON_MODE,
				      GPIO_SGMII_MDIO_MODE_MASK }},
};

static const struct airoha_pinctrl_func_group uart_func_group[] = {
	{ "uart2", AIROHA_FUNC_MUX, { REG_GPIO_PON_MODE,
				      GPIO_UART2_MODE_MASK }},
	{ "uart2_cts_rts", AIROHA_FUNC_MUX, { REG_GPIO_PON_MODE,
					      GPIO_UART2_MODE_MASK |
					      GPIO_UART2_CTS_RTS_MODE_MASK }},
	{ "hsuart", AIROHA_FUNC_MUX, { REG_GPIO_PON_MODE,
				       GPIO_HSUART_MODE_MASK }},
	{ "hsuart_cts_rts", AIROHA_FUNC_MUX, { REG_GPIO_PON_MODE,
					       GPIO_HSUART_MODE_MASK |
					       GPIO_HSUART_CTS_RTS_MODE_MASK }},
	{ "uart4", AIROHA_FUNC_MUX, { REG_GPIO_PON_MODE,
				      GPIO_UART4_MODE_MASK }},
	{ "uart5", AIROHA_FUNC_MUX, { REG_GPIO_PON_MODE,
				      GPIO_UART5_MODE_MASK }},
};

static const struct airoha_pinctrl_func_group i2c_func_group[] = {
	{ "i2c1", AIROHA_FUNC_MUX, { REG_GPIO_2ND_I2C_MODE,
				     GPIO_2ND_I2C_MODE_MASK }},
};

static const struct airoha_pinctrl_func_group jtag_func_group[] = {
	{ "jtag_udi", AIROHA_FUNC_MUX, { REG_NPU_UART_EN, JTAG_UDI_EN_MASK }},
	{ "jtag_dfd", AIROHA_FUNC_MUX, { REG_NPU_UART_EN, JTAG_DFD_EN_MASK }},
};

static const struct airoha_pinctrl_func_group pcm_func_group[] = {
	{ "pcm1", AIROHA_FUNC_MUX, { REG_GPIO_SPI_CS1_MODE,
				     GPIO_PCM1_MODE_MASK }},
	{ "pcm2", AIROHA_FUNC_MUX, { REG_GPIO_SPI_CS1_MODE,
				     GPIO_PCM2_MODE_MASK }},
};

static const struct airoha_pinctrl_func_group spi_func_group[] = {
	{ "spi_quad", AIROHA_FUNC_MUX, { REG_GPIO_SPI_CS1_MODE,
					 GPIO_SPI_QUAD_MODE_MASK }},
	{ "spi_cs1", AIROHA_FUNC_MUX, { REG_GPIO_SPI_CS1_MODE,
					GPIO_SPI_CS1_MODE_MASK }},
	{ "spi_cs2", AIROHA_FUNC_MUX, { REG_GPIO_SPI_CS1_MODE,
					GPIO_SPI_CS2_MODE_MASK }},
	{ "spi_cs3", AIROHA_FUNC_MUX, { REG_GPIO_SPI_CS1_MODE,
					GPIO_SPI_CS3_MODE_MASK }},
	{ "spi_cs4", AIROHA_FUNC_MUX, { REG_GPIO_SPI_CS1_MODE,
					GPIO_SPI_CS4_MODE_MASK }},
};

static const struct airoha_pinctrl_func_group pcm_spi_func_group[] = {
	{ "pcm_spi", AIROHA_FUNC_MUX, { REG_GPIO_SPI_CS1_MODE,
					GPIO_PCM_SPI_MODE_MASK }},
	{ "pcm_spi_int", AIROHA_FUNC_MUX, { REG_GPIO_SPI_CS1_MODE,
					    GPIO_PCM_INT_MODE_MASK }},
	{ "pcm_spi_rst", AIROHA_FUNC_MUX, { REG_GPIO_SPI_CS1_MODE,
					    GPIO_PCM_RESET_MODE_MASK }},
	{ "pcm_spi_cs1", AIROHA_FUNC_MUX, { REG_GPIO_SPI_CS1_MODE,
					    GPIO_PCM_SPI_CS1_MODE_MASK }},
	{ "pcm_spi_cs2_p128", AIROHA_FUNC_MUX, { REG_GPIO_SPI_CS1_MODE,
						 GPIO_PCM_SPI_CS2_MODE_P128_MASK }},
	{ "pcm_spi_cs2_p156", AIROHA_FUNC_MUX, { REG_GPIO_SPI_CS1_MODE,
						 GPIO_PCM_SPI_CS2_MODE_P156_MASK }},
	{ "pcm_spi_cs3", AIROHA_FUNC_MUX, { REG_GPIO_SPI_CS1_MODE,
					    GPIO_PCM_SPI_CS3_MODE_MASK }},
	{ "pcm_spi_cs4", AIROHA_FUNC_MUX, { REG_GPIO_SPI_CS1_MODE,
					    GPIO_PCM_SPI_CS4_MODE_MASK }},
};

static const struct airoha_pinctrl_func_group i2s_func_group[] = {
	{ "i2s", AIROHA_FUNC_MUX, { REG_GPIO_2ND_I2C_MODE,
				    GPIO_I2S_MODE_MASK }},
};

static const struct airoha_pinctrl_func_group emmc_func_group[] = {
	{ "emmc", AIROHA_FUNC_MUX, { REG_GPIO_PON_MODE, GPIO_EMMC_MODE_MASK }},
};

static const struct airoha_pinctrl_func_group pnand_func_group[] = {
	{ "pnand", AIROHA_FUNC_MUX, { REG_GPIO_PON_MODE,
				      GPIO_PARALLEL_NAND_MODE_MASK }},
};

static const struct airoha_pinctrl_func_group pcie_reset_func_group[] = {
	{ "pcie_reset0", AIROHA_FUNC_MUX, { REG_GPIO_PON_MODE,
					    GPIO_PCIE_RESET0_MASK }},
	{ "pcie_reset1", AIROHA_FUNC_MUX, { REG_GPIO_PON_MODE,
					    GPIO_PCIE_RESET1_MASK }},
	{ "pcie_reset2", AIROHA_FUNC_MUX, { REG_GPIO_PON_MODE,
					    GPIO_PCIE_RESET2_MASK }},
};

/* LED */
static const struct airoha_pinctrl_func_group led_func_group[] = {
	{ "gpio33", AIROHA_FUNC_MUX, { REG_GPIO_2ND_I2C_MODE,
				       GPIO_LAN0_LED0_MODE_MASK }},
	{ "gpio43", AIROHA_FUNC_MUX, { REG_GPIO_2ND_I2C_MODE,
				       GPIO_LAN0_LED1_MODE_MASK }},
	{ "gpio34", AIROHA_FUNC_MUX, { REG_GPIO_2ND_I2C_MODE,
				       GPIO_LAN1_LED0_MODE_MASK }},
	{ "gpio44", AIROHA_FUNC_MUX, { REG_GPIO_2ND_I2C_MODE,
				       GPIO_LAN1_LED1_MODE_MASK }},
	{ "gpio35", AIROHA_FUNC_MUX, { REG_GPIO_2ND_I2C_MODE,
				       GPIO_LAN2_LED0_MODE_MASK }},
	{ "gpio45", AIROHA_FUNC_MUX, { REG_GPIO_2ND_I2C_MODE,
				       GPIO_LAN2_LED1_MODE_MASK }},
	{ "gpio42", AIROHA_FUNC_MUX, { REG_GPIO_2ND_I2C_MODE,
				       GPIO_LAN3_LED0_MODE_MASK }},
	{ "gpio46", AIROHA_FUNC_MUX, { REG_GPIO_2ND_I2C_MODE,
				       GPIO_LAN3_LED1_MODE_MASK }},
};

/* PWM */
static const struct airoha_pinctrl_func_group pwm_func_group[] = {
	{ "gpio0", AIROHA_FUNC_PWM_MUX, { REG_GPIO_FLASH_MODE_CFG,
					  GPIO0_FLASH_MODE_CFG }},
	{ "gpio1", AIROHA_FUNC_PWM_MUX, { REG_GPIO_FLASH_MODE_CFG,
					  GPIO1_FLASH_MODE_CFG }},
	{ "gpio2", AIROHA_FUNC_PWM_MUX, { REG_GPIO_FLASH_MODE_CFG,
					  GPIO2_FLASH_MODE_CFG }},
	{ "gpio3", AIROHA_FUNC_PWM_MUX, { REG_GPIO_FLASH_MODE_CFG,
					  GPIO3_FLASH_MODE_CFG }},
	{ "gpio4", AIROHA_FUNC_PWM_MUX, { REG_GPIO_FLASH_MODE_CFG,
					  GPIO4_FLASH_MODE_CFG }},
	{ "gpio5", AIROHA_FUNC_PWM_MUX, { REG_GPIO_FLASH_MODE_CFG,
					  GPIO5_FLASH_MODE_CFG }},
	{ "gpio6", AIROHA_FUNC_PWM_MUX, { REG_GPIO_FLASH_MODE_CFG,
					  GPIO6_FLASH_MODE_CFG }},
	{ "gpio7", AIROHA_FUNC_PWM_MUX, { REG_GPIO_FLASH_MODE_CFG,
					  GPIO7_FLASH_MODE_CFG }},
	{ "gpio8", AIROHA_FUNC_PWM_MUX, { REG_GPIO_FLASH_MODE_CFG,
					  GPIO8_FLASH_MODE_CFG }},
	{ "gpio9", AIROHA_FUNC_PWM_MUX, { REG_GPIO_FLASH_MODE_CFG,
					  GPIO9_FLASH_MODE_CFG }},
	{ "gpio10", AIROHA_FUNC_PWM_MUX, { REG_GPIO_FLASH_MODE_CFG,
					   GPIO10_FLASH_MODE_CFG }},
	{ "gpio11", AIROHA_FUNC_PWM_MUX, { REG_GPIO_FLASH_MODE_CFG,
					   GPIO11_FLASH_MODE_CFG }},
	{ "gpio12", AIROHA_FUNC_PWM_MUX, { REG_GPIO_FLASH_MODE_CFG,
					   GPIO12_FLASH_MODE_CFG }},
	{ "gpio13", AIROHA_FUNC_PWM_MUX, { REG_GPIO_FLASH_MODE_CFG,
					   GPIO13_FLASH_MODE_CFG }},
	{ "gpio14", AIROHA_FUNC_PWM_MUX, { REG_GPIO_FLASH_MODE_CFG,
					   GPIO14_FLASH_MODE_CFG }},
	{ "gpio15", AIROHA_FUNC_PWM_MUX, { REG_GPIO_FLASH_MODE_CFG,
					   GPIO15_FLASH_MODE_CFG }},
	{ "gpio16", AIROHA_FUNC_PWM_EXT_MUX, { REG_GPIO_FLASH_MODE_CFG_EXT,
					       GPIO16_FLASH_MODE_CFG }},
	{ "gpio17", AIROHA_FUNC_PWM_EXT_MUX, { REG_GPIO_FLASH_MODE_CFG_EXT,
					       GPIO17_FLASH_MODE_CFG }},
	{ "gpio18", AIROHA_FUNC_PWM_EXT_MUX, { REG_GPIO_FLASH_MODE_CFG_EXT,
					       GPIO18_FLASH_MODE_CFG }},
	{ "gpio19", AIROHA_FUNC_PWM_EXT_MUX, { REG_GPIO_FLASH_MODE_CFG_EXT,
					       GPIO19_FLASH_MODE_CFG }},
	{ "gpio20", AIROHA_FUNC_PWM_EXT_MUX, { REG_GPIO_FLASH_MODE_CFG_EXT,
					       GPIO20_FLASH_MODE_CFG }},
	{ "gpio21", AIROHA_FUNC_PWM_EXT_MUX, { REG_GPIO_FLASH_MODE_CFG_EXT,
					       GPIO21_FLASH_MODE_CFG }},
	{ "gpio22", AIROHA_FUNC_PWM_EXT_MUX, { REG_GPIO_FLASH_MODE_CFG_EXT,
					       GPIO22_FLASH_MODE_CFG }},
	{ "gpio23", AIROHA_FUNC_PWM_EXT_MUX, { REG_GPIO_FLASH_MODE_CFG_EXT,
					       GPIO23_FLASH_MODE_CFG }},
	{ "gpio24", AIROHA_FUNC_PWM_EXT_MUX, { REG_GPIO_FLASH_MODE_CFG_EXT,
					       GPIO24_FLASH_MODE_CFG }},
	{ "gpio25", AIROHA_FUNC_PWM_EXT_MUX, { REG_GPIO_FLASH_MODE_CFG_EXT,
					       GPIO25_FLASH_MODE_CFG }},
	{ "gpio26", AIROHA_FUNC_PWM_EXT_MUX, { REG_GPIO_FLASH_MODE_CFG_EXT,
					       GPIO26_FLASH_MODE_CFG }},
	{ "gpio27", AIROHA_FUNC_PWM_EXT_MUX, { REG_GPIO_FLASH_MODE_CFG_EXT,
					       GPIO27_FLASH_MODE_CFG }},
	{ "gpio28", AIROHA_FUNC_PWM_EXT_MUX, { REG_GPIO_FLASH_MODE_CFG_EXT,
					       GPIO28_FLASH_MODE_CFG }},
	{ "gpio29", AIROHA_FUNC_PWM_EXT_MUX, { REG_GPIO_FLASH_MODE_CFG_EXT,
					       GPIO29_FLASH_MODE_CFG }},
	{ "gpio30", AIROHA_FUNC_PWM_EXT_MUX, { REG_GPIO_FLASH_MODE_CFG_EXT,
					       GPIO30_FLASH_MODE_CFG }},
	{ "gpio31", AIROHA_FUNC_PWM_EXT_MUX, { REG_GPIO_FLASH_MODE_CFG_EXT,
					       GPIO31_FLASH_MODE_CFG }},
	{ "gpio36", AIROHA_FUNC_PWM_EXT_MUX, { REG_GPIO_FLASH_MODE_CFG_EXT,
					       GPIO36_FLASH_MODE_CFG }},
	{ "gpio37", AIROHA_FUNC_PWM_EXT_MUX, { REG_GPIO_FLASH_MODE_CFG_EXT,
					       GPIO37_FLASH_MODE_CFG }},
	{ "gpio38", AIROHA_FUNC_PWM_EXT_MUX, { REG_GPIO_FLASH_MODE_CFG_EXT,
					       GPIO38_FLASH_MODE_CFG }},
	{ "gpio39", AIROHA_FUNC_PWM_EXT_MUX, { REG_GPIO_FLASH_MODE_CFG_EXT,
					       GPIO39_FLASH_MODE_CFG }},
	{ "gpio40", AIROHA_FUNC_PWM_EXT_MUX, { REG_GPIO_FLASH_MODE_CFG_EXT,
					       GPIO40_FLASH_MODE_CFG }},
	{ "gpio41", AIROHA_FUNC_PWM_EXT_MUX, { REG_GPIO_FLASH_MODE_CFG_EXT,
					       GPIO41_FLASH_MODE_CFG }},
	{ "gpio42", AIROHA_FUNC_PWM_EXT_MUX, { REG_GPIO_FLASH_MODE_CFG_EXT,
					       GPIO42_FLASH_MODE_CFG }},
	{ "gpio43", AIROHA_FUNC_PWM_EXT_MUX, { REG_GPIO_FLASH_MODE_CFG_EXT,
					       GPIO43_FLASH_MODE_CFG }},
	{ "gpio44", AIROHA_FUNC_PWM_EXT_MUX, { REG_GPIO_FLASH_MODE_CFG_EXT,
					       GPIO44_FLASH_MODE_CFG }},
	{ "gpio45", AIROHA_FUNC_PWM_EXT_MUX, { REG_GPIO_FLASH_MODE_CFG_EXT,
					       GPIO45_FLASH_MODE_CFG }},
	{ "gpio46", AIROHA_FUNC_PWM_EXT_MUX, { REG_GPIO_FLASH_MODE_CFG_EXT,
					       GPIO46_FLASH_MODE_CFG }},
	{ "gpio47", AIROHA_FUNC_PWM_EXT_MUX, { REG_GPIO_FLASH_MODE_CFG_EXT,
					       GPIO47_FLASH_MODE_CFG }},
};

static const struct airoha_pinctrl_func airoha_pinctrl_funcs[] = {
	PINCTRL_FUNC_DESC(pon),
	PINCTRL_FUNC_DESC(tod_1pps),
	PINCTRL_FUNC_DESC(sipo),
	PINCTRL_FUNC_DESC(mdio),
	PINCTRL_FUNC_DESC(uart),
	PINCTRL_FUNC_DESC(i2c),
	PINCTRL_FUNC_DESC(jtag),
	PINCTRL_FUNC_DESC(pcm),
	PINCTRL_FUNC_DESC(spi),
	PINCTRL_FUNC_DESC(pcm_spi),
	PINCTRL_FUNC_DESC(i2s),
	PINCTRL_FUNC_DESC(emmc),
	PINCTRL_FUNC_DESC(pnand),
	PINCTRL_FUNC_DESC(led),
	PINCTRL_FUNC_DESC(pcie_reset),
	PINCTRL_FUNC_DESC(pwm),
};

static const struct airoha_pinctrl_conf airoha_pinctrl_pullup_conf[] = {
	PINCTRL_CONF_DESC(0, REG_I2C_SDA_PU, UART1_TXD_PU_MASK),
	PINCTRL_CONF_DESC(1, REG_I2C_SDA_PU, UART1_RXD_PU_MASK),
	PINCTRL_CONF_DESC(2, REG_I2C_SDA_PU, I2C_SDA_PU_MASK),
	PINCTRL_CONF_DESC(3, REG_I2C_SDA_PU, I2C_SCL_PU_MASK),
	PINCTRL_CONF_DESC(4, REG_I2C_SDA_PU, SPI_CS0_PU_MASK),
	PINCTRL_CONF_DESC(5, REG_I2C_SDA_PU, SPI_CLK_PU_MASK),
	PINCTRL_CONF_DESC(6, REG_I2C_SDA_PU, SPI_MOSI_PU_MASK),
	PINCTRL_CONF_DESC(7, REG_I2C_SDA_PU, SPI_MISO_PU_MASK),
	PINCTRL_CONF_DESC(13, REG_GPIO_L_PU, BIT(0)),
	PINCTRL_CONF_DESC(14, REG_GPIO_L_PU, BIT(1)),
	PINCTRL_CONF_DESC(15, REG_GPIO_L_PU, BIT(2)),
	PINCTRL_CONF_DESC(16, REG_GPIO_L_PU, BIT(3)),
	PINCTRL_CONF_DESC(17, REG_GPIO_L_PU, BIT(4)),
	PINCTRL_CONF_DESC(18, REG_GPIO_L_PU, BIT(5)),
	PINCTRL_CONF_DESC(19, REG_GPIO_L_PU, BIT(6)),
	PINCTRL_CONF_DESC(20, REG_GPIO_L_PU, BIT(7)),
	PINCTRL_CONF_DESC(21, REG_GPIO_L_PU, BIT(8)),
	PINCTRL_CONF_DESC(22, REG_GPIO_L_PU, BIT(9)),
	PINCTRL_CONF_DESC(23, REG_GPIO_L_PU, BIT(10)),
	PINCTRL_CONF_DESC(24, REG_GPIO_L_PU, BIT(11)),
	PINCTRL_CONF_DESC(25, REG_GPIO_L_PU, BIT(12)),
	PINCTRL_CONF_DESC(26, REG_GPIO_L_PU, BIT(13)),
	PINCTRL_CONF_DESC(27, REG_GPIO_L_PU, BIT(14)),
	PINCTRL_CONF_DESC(28, REG_GPIO_L_PU, BIT(15)),
	PINCTRL_CONF_DESC(29, REG_GPIO_L_PU, BIT(16)),
	PINCTRL_CONF_DESC(30, REG_GPIO_L_PU, BIT(17)),
	PINCTRL_CONF_DESC(31, REG_GPIO_L_PU, BIT(18)),
	PINCTRL_CONF_DESC(32, REG_GPIO_L_PU, BIT(18)),
	PINCTRL_CONF_DESC(33, REG_GPIO_L_PU, BIT(20)),
	PINCTRL_CONF_DESC(34, REG_GPIO_L_PU, BIT(21)),
	PINCTRL_CONF_DESC(35, REG_GPIO_L_PU, BIT(22)),
	PINCTRL_CONF_DESC(36, REG_GPIO_L_PU, BIT(23)),
	PINCTRL_CONF_DESC(37, REG_GPIO_L_PU, BIT(24)),
	PINCTRL_CONF_DESC(38, REG_GPIO_L_PU, BIT(25)),
	PINCTRL_CONF_DESC(39, REG_GPIO_L_PU, BIT(26)),
	PINCTRL_CONF_DESC(40, REG_GPIO_L_PU, BIT(27)),
	PINCTRL_CONF_DESC(41, REG_GPIO_L_PU, BIT(28)),
	PINCTRL_CONF_DESC(42, REG_GPIO_L_PU, BIT(29)),
	PINCTRL_CONF_DESC(43, REG_GPIO_L_PU, BIT(30)),
	PINCTRL_CONF_DESC(44, REG_GPIO_L_PU, BIT(31)),
	PINCTRL_CONF_DESC(45, REG_GPIO_H_PU, BIT(0)),
	PINCTRL_CONF_DESC(46, REG_GPIO_H_PU, BIT(1)),
	PINCTRL_CONF_DESC(47, REG_GPIO_H_PU, BIT(2)),
	PINCTRL_CONF_DESC(48, REG_GPIO_H_PU, BIT(3)),
	PINCTRL_CONF_DESC(49, REG_GPIO_H_PU, BIT(4)),
	PINCTRL_CONF_DESC(50, REG_GPIO_H_PU, BIT(5)),
	PINCTRL_CONF_DESC(51, REG_GPIO_H_PU, BIT(6)),
	PINCTRL_CONF_DESC(52, REG_GPIO_H_PU, BIT(7)),
	PINCTRL_CONF_DESC(53, REG_GPIO_H_PU, BIT(8)),
	PINCTRL_CONF_DESC(54, REG_GPIO_H_PU, BIT(9)),
	PINCTRL_CONF_DESC(55, REG_GPIO_H_PU, BIT(10)),
	PINCTRL_CONF_DESC(56, REG_GPIO_H_PU, BIT(11)),
	PINCTRL_CONF_DESC(57, REG_GPIO_H_PU, BIT(12)),
	PINCTRL_CONF_DESC(58, REG_GPIO_H_PU, BIT(13)),
	PINCTRL_CONF_DESC(59, REG_GPIO_H_PU, BIT(14)),
	PINCTRL_CONF_DESC(61, REG_I2C_SDA_PU, PCIE0_RESET_PU_MASK),
	PINCTRL_CONF_DESC(62, REG_I2C_SDA_PU, PCIE1_RESET_PU_MASK),
	PINCTRL_CONF_DESC(63, REG_I2C_SDA_PU, PCIE2_RESET_PU_MASK),
};

static const struct airoha_pinctrl_conf airoha_pinctrl_pulldown_conf[] = {
	PINCTRL_CONF_DESC(0, REG_I2C_SDA_PD, UART1_TXD_PD_MASK),
	PINCTRL_CONF_DESC(1, REG_I2C_SDA_PD, UART1_RXD_PD_MASK),
	PINCTRL_CONF_DESC(2, REG_I2C_SDA_PD, I2C_SDA_PD_MASK),
	PINCTRL_CONF_DESC(3, REG_I2C_SDA_PD, I2C_SCL_PD_MASK),
	PINCTRL_CONF_DESC(4, REG_I2C_SDA_PD, SPI_CS0_PD_MASK),
	PINCTRL_CONF_DESC(5, REG_I2C_SDA_PD, SPI_CLK_PD_MASK),
	PINCTRL_CONF_DESC(6, REG_I2C_SDA_PD, SPI_MOSI_PD_MASK),
	PINCTRL_CONF_DESC(7, REG_I2C_SDA_PD, SPI_MISO_PD_MASK),
	PINCTRL_CONF_DESC(13, REG_GPIO_L_PD, BIT(0)),
	PINCTRL_CONF_DESC(14, REG_GPIO_L_PD, BIT(1)),
	PINCTRL_CONF_DESC(15, REG_GPIO_L_PD, BIT(2)),
	PINCTRL_CONF_DESC(16, REG_GPIO_L_PD, BIT(3)),
	PINCTRL_CONF_DESC(17, REG_GPIO_L_PD, BIT(4)),
	PINCTRL_CONF_DESC(18, REG_GPIO_L_PD, BIT(5)),
	PINCTRL_CONF_DESC(19, REG_GPIO_L_PD, BIT(6)),
	PINCTRL_CONF_DESC(20, REG_GPIO_L_PD, BIT(7)),
	PINCTRL_CONF_DESC(21, REG_GPIO_L_PD, BIT(8)),
	PINCTRL_CONF_DESC(22, REG_GPIO_L_PD, BIT(9)),
	PINCTRL_CONF_DESC(23, REG_GPIO_L_PD, BIT(10)),
	PINCTRL_CONF_DESC(24, REG_GPIO_L_PD, BIT(11)),
	PINCTRL_CONF_DESC(25, REG_GPIO_L_PD, BIT(12)),
	PINCTRL_CONF_DESC(26, REG_GPIO_L_PD, BIT(13)),
	PINCTRL_CONF_DESC(27, REG_GPIO_L_PD, BIT(14)),
	PINCTRL_CONF_DESC(28, REG_GPIO_L_PD, BIT(15)),
	PINCTRL_CONF_DESC(29, REG_GPIO_L_PD, BIT(16)),
	PINCTRL_CONF_DESC(30, REG_GPIO_L_PD, BIT(17)),
	PINCTRL_CONF_DESC(31, REG_GPIO_L_PD, BIT(18)),
	PINCTRL_CONF_DESC(32, REG_GPIO_L_PD, BIT(18)),
	PINCTRL_CONF_DESC(33, REG_GPIO_L_PD, BIT(20)),
	PINCTRL_CONF_DESC(34, REG_GPIO_L_PD, BIT(21)),
	PINCTRL_CONF_DESC(35, REG_GPIO_L_PD, BIT(22)),
	PINCTRL_CONF_DESC(36, REG_GPIO_L_PD, BIT(23)),
	PINCTRL_CONF_DESC(37, REG_GPIO_L_PD, BIT(24)),
	PINCTRL_CONF_DESC(38, REG_GPIO_L_PD, BIT(25)),
	PINCTRL_CONF_DESC(39, REG_GPIO_L_PD, BIT(26)),
	PINCTRL_CONF_DESC(40, REG_GPIO_L_PD, BIT(27)),
	PINCTRL_CONF_DESC(41, REG_GPIO_L_PD, BIT(28)),
	PINCTRL_CONF_DESC(42, REG_GPIO_L_PD, BIT(29)),
	PINCTRL_CONF_DESC(43, REG_GPIO_L_PD, BIT(30)),
	PINCTRL_CONF_DESC(44, REG_GPIO_L_PD, BIT(31)),
	PINCTRL_CONF_DESC(45, REG_GPIO_H_PD, BIT(0)),
	PINCTRL_CONF_DESC(46, REG_GPIO_H_PD, BIT(1)),
	PINCTRL_CONF_DESC(47, REG_GPIO_H_PD, BIT(2)),
	PINCTRL_CONF_DESC(48, REG_GPIO_H_PD, BIT(3)),
	PINCTRL_CONF_DESC(49, REG_GPIO_H_PD, BIT(4)),
	PINCTRL_CONF_DESC(50, REG_GPIO_H_PD, BIT(5)),
	PINCTRL_CONF_DESC(51, REG_GPIO_H_PD, BIT(6)),
	PINCTRL_CONF_DESC(52, REG_GPIO_H_PD, BIT(7)),
	PINCTRL_CONF_DESC(53, REG_GPIO_H_PD, BIT(8)),
	PINCTRL_CONF_DESC(54, REG_GPIO_H_PD, BIT(9)),
	PINCTRL_CONF_DESC(55, REG_GPIO_H_PD, BIT(10)),
	PINCTRL_CONF_DESC(56, REG_GPIO_H_PD, BIT(11)),
	PINCTRL_CONF_DESC(57, REG_GPIO_H_PD, BIT(12)),
	PINCTRL_CONF_DESC(58, REG_GPIO_H_PD, BIT(13)),
	PINCTRL_CONF_DESC(59, REG_GPIO_H_PD, BIT(14)),
	PINCTRL_CONF_DESC(61, REG_I2C_SDA_PD, PCIE0_RESET_PD_MASK),
	PINCTRL_CONF_DESC(62, REG_I2C_SDA_PD, PCIE1_RESET_PD_MASK),
	PINCTRL_CONF_DESC(63, REG_I2C_SDA_PD, PCIE2_RESET_PD_MASK),
};

static const struct airoha_pinctrl_conf airoha_pinctrl_drive_e2_conf[] = {
	PINCTRL_CONF_DESC(0, REG_I2C_SDA_E2, UART1_TXD_E2_MASK),
	PINCTRL_CONF_DESC(1, REG_I2C_SDA_E2, UART1_RXD_E2_MASK),
	PINCTRL_CONF_DESC(2, REG_I2C_SDA_E2, I2C_SDA_E2_MASK),
	PINCTRL_CONF_DESC(3, REG_I2C_SDA_E2, I2C_SCL_E2_MASK),
	PINCTRL_CONF_DESC(4, REG_I2C_SDA_E2, SPI_CS0_E2_MASK),
	PINCTRL_CONF_DESC(5, REG_I2C_SDA_E2, SPI_CLK_E2_MASK),
	PINCTRL_CONF_DESC(6, REG_I2C_SDA_E2, SPI_MOSI_E2_MASK),
	PINCTRL_CONF_DESC(7, REG_I2C_SDA_E2, SPI_MISO_E2_MASK),
	PINCTRL_CONF_DESC(13, REG_GPIO_L_E2, BIT(0)),
	PINCTRL_CONF_DESC(14, REG_GPIO_L_E2, BIT(1)),
	PINCTRL_CONF_DESC(15, REG_GPIO_L_E2, BIT(2)),
	PINCTRL_CONF_DESC(16, REG_GPIO_L_E2, BIT(3)),
	PINCTRL_CONF_DESC(17, REG_GPIO_L_E2, BIT(4)),
	PINCTRL_CONF_DESC(18, REG_GPIO_L_E2, BIT(5)),
	PINCTRL_CONF_DESC(19, REG_GPIO_L_E2, BIT(6)),
	PINCTRL_CONF_DESC(20, REG_GPIO_L_E2, BIT(7)),
	PINCTRL_CONF_DESC(21, REG_GPIO_L_E2, BIT(8)),
	PINCTRL_CONF_DESC(22, REG_GPIO_L_E2, BIT(9)),
	PINCTRL_CONF_DESC(23, REG_GPIO_L_E2, BIT(10)),
	PINCTRL_CONF_DESC(24, REG_GPIO_L_E2, BIT(11)),
	PINCTRL_CONF_DESC(25, REG_GPIO_L_E2, BIT(12)),
	PINCTRL_CONF_DESC(26, REG_GPIO_L_E2, BIT(13)),
	PINCTRL_CONF_DESC(27, REG_GPIO_L_E2, BIT(14)),
	PINCTRL_CONF_DESC(28, REG_GPIO_L_E2, BIT(15)),
	PINCTRL_CONF_DESC(29, REG_GPIO_L_E2, BIT(16)),
	PINCTRL_CONF_DESC(30, REG_GPIO_L_E2, BIT(17)),
	PINCTRL_CONF_DESC(31, REG_GPIO_L_E2, BIT(18)),
	PINCTRL_CONF_DESC(32, REG_GPIO_L_E2, BIT(18)),
	PINCTRL_CONF_DESC(33, REG_GPIO_L_E2, BIT(20)),
	PINCTRL_CONF_DESC(34, REG_GPIO_L_E2, BIT(21)),
	PINCTRL_CONF_DESC(35, REG_GPIO_L_E2, BIT(22)),
	PINCTRL_CONF_DESC(36, REG_GPIO_L_E2, BIT(23)),
	PINCTRL_CONF_DESC(37, REG_GPIO_L_E2, BIT(24)),
	PINCTRL_CONF_DESC(38, REG_GPIO_L_E2, BIT(25)),
	PINCTRL_CONF_DESC(39, REG_GPIO_L_E2, BIT(26)),
	PINCTRL_CONF_DESC(40, REG_GPIO_L_E2, BIT(27)),
	PINCTRL_CONF_DESC(41, REG_GPIO_L_E2, BIT(28)),
	PINCTRL_CONF_DESC(42, REG_GPIO_L_E2, BIT(29)),
	PINCTRL_CONF_DESC(43, REG_GPIO_L_E2, BIT(30)),
	PINCTRL_CONF_DESC(44, REG_GPIO_L_E2, BIT(31)),
	PINCTRL_CONF_DESC(45, REG_GPIO_H_E2, BIT(0)),
	PINCTRL_CONF_DESC(46, REG_GPIO_H_E2, BIT(1)),
	PINCTRL_CONF_DESC(47, REG_GPIO_H_E2, BIT(2)),
	PINCTRL_CONF_DESC(48, REG_GPIO_H_E2, BIT(3)),
	PINCTRL_CONF_DESC(49, REG_GPIO_H_E2, BIT(4)),
	PINCTRL_CONF_DESC(50, REG_GPIO_H_E2, BIT(5)),
	PINCTRL_CONF_DESC(51, REG_GPIO_H_E2, BIT(6)),
	PINCTRL_CONF_DESC(52, REG_GPIO_H_E2, BIT(7)),
	PINCTRL_CONF_DESC(53, REG_GPIO_H_E2, BIT(8)),
	PINCTRL_CONF_DESC(54, REG_GPIO_H_E2, BIT(9)),
	PINCTRL_CONF_DESC(55, REG_GPIO_H_E2, BIT(10)),
	PINCTRL_CONF_DESC(56, REG_GPIO_H_E2, BIT(11)),
	PINCTRL_CONF_DESC(57, REG_GPIO_H_E2, BIT(12)),
	PINCTRL_CONF_DESC(58, REG_GPIO_H_E2, BIT(13)),
	PINCTRL_CONF_DESC(59, REG_GPIO_H_E2, BIT(14)),
	PINCTRL_CONF_DESC(61, REG_I2C_SDA_E2, PCIE0_RESET_E2_MASK),
	PINCTRL_CONF_DESC(62, REG_I2C_SDA_E2, PCIE1_RESET_E2_MASK),
	PINCTRL_CONF_DESC(63, REG_I2C_SDA_E2, PCIE2_RESET_E2_MASK),
};

static const struct airoha_pinctrl_conf airoha_pinctrl_drive_e4_conf[] = {
	PINCTRL_CONF_DESC(0, REG_I2C_SDA_E4, UART1_TXD_E4_MASK),
	PINCTRL_CONF_DESC(1, REG_I2C_SDA_E4, UART1_RXD_E4_MASK),
	PINCTRL_CONF_DESC(2, REG_I2C_SDA_E4, I2C_SDA_E4_MASK),
	PINCTRL_CONF_DESC(3, REG_I2C_SDA_E4, I2C_SCL_E4_MASK),
	PINCTRL_CONF_DESC(4, REG_I2C_SDA_E4, SPI_CS0_E4_MASK),
	PINCTRL_CONF_DESC(5, REG_I2C_SDA_E4, SPI_CLK_E4_MASK),
	PINCTRL_CONF_DESC(6, REG_I2C_SDA_E4, SPI_MOSI_E4_MASK),
	PINCTRL_CONF_DESC(7, REG_I2C_SDA_E4, SPI_MISO_E4_MASK),
	PINCTRL_CONF_DESC(13, REG_GPIO_L_E4, BIT(0)),
	PINCTRL_CONF_DESC(14, REG_GPIO_L_E4, BIT(1)),
	PINCTRL_CONF_DESC(15, REG_GPIO_L_E4, BIT(2)),
	PINCTRL_CONF_DESC(16, REG_GPIO_L_E4, BIT(3)),
	PINCTRL_CONF_DESC(17, REG_GPIO_L_E4, BIT(4)),
	PINCTRL_CONF_DESC(18, REG_GPIO_L_E4, BIT(5)),
	PINCTRL_CONF_DESC(19, REG_GPIO_L_E4, BIT(6)),
	PINCTRL_CONF_DESC(20, REG_GPIO_L_E4, BIT(7)),
	PINCTRL_CONF_DESC(21, REG_GPIO_L_E4, BIT(8)),
	PINCTRL_CONF_DESC(22, REG_GPIO_L_E4, BIT(9)),
	PINCTRL_CONF_DESC(23, REG_GPIO_L_E4, BIT(10)),
	PINCTRL_CONF_DESC(24, REG_GPIO_L_E4, BIT(11)),
	PINCTRL_CONF_DESC(25, REG_GPIO_L_E4, BIT(12)),
	PINCTRL_CONF_DESC(26, REG_GPIO_L_E4, BIT(13)),
	PINCTRL_CONF_DESC(27, REG_GPIO_L_E4, BIT(14)),
	PINCTRL_CONF_DESC(28, REG_GPIO_L_E4, BIT(15)),
	PINCTRL_CONF_DESC(29, REG_GPIO_L_E4, BIT(16)),
	PINCTRL_CONF_DESC(30, REG_GPIO_L_E4, BIT(17)),
	PINCTRL_CONF_DESC(31, REG_GPIO_L_E4, BIT(18)),
	PINCTRL_CONF_DESC(32, REG_GPIO_L_E4, BIT(18)),
	PINCTRL_CONF_DESC(33, REG_GPIO_L_E4, BIT(20)),
	PINCTRL_CONF_DESC(34, REG_GPIO_L_E4, BIT(21)),
	PINCTRL_CONF_DESC(35, REG_GPIO_L_E4, BIT(22)),
	PINCTRL_CONF_DESC(36, REG_GPIO_L_E4, BIT(23)),
	PINCTRL_CONF_DESC(37, REG_GPIO_L_E4, BIT(24)),
	PINCTRL_CONF_DESC(38, REG_GPIO_L_E4, BIT(25)),
	PINCTRL_CONF_DESC(39, REG_GPIO_L_E4, BIT(26)),
	PINCTRL_CONF_DESC(40, REG_GPIO_L_E4, BIT(27)),
	PINCTRL_CONF_DESC(41, REG_GPIO_L_E4, BIT(28)),
	PINCTRL_CONF_DESC(42, REG_GPIO_L_E4, BIT(29)),
	PINCTRL_CONF_DESC(43, REG_GPIO_L_E4, BIT(30)),
	PINCTRL_CONF_DESC(44, REG_GPIO_L_E4, BIT(31)),
	PINCTRL_CONF_DESC(45, REG_GPIO_H_E4, BIT(0)),
	PINCTRL_CONF_DESC(46, REG_GPIO_H_E4, BIT(1)),
	PINCTRL_CONF_DESC(47, REG_GPIO_H_E4, BIT(2)),
	PINCTRL_CONF_DESC(48, REG_GPIO_H_E4, BIT(3)),
	PINCTRL_CONF_DESC(49, REG_GPIO_H_E4, BIT(4)),
	PINCTRL_CONF_DESC(50, REG_GPIO_H_E4, BIT(5)),
	PINCTRL_CONF_DESC(51, REG_GPIO_H_E4, BIT(6)),
	PINCTRL_CONF_DESC(52, REG_GPIO_H_E4, BIT(7)),
	PINCTRL_CONF_DESC(53, REG_GPIO_H_E4, BIT(8)),
	PINCTRL_CONF_DESC(54, REG_GPIO_H_E4, BIT(9)),
	PINCTRL_CONF_DESC(55, REG_GPIO_H_E4, BIT(10)),
	PINCTRL_CONF_DESC(56, REG_GPIO_H_E4, BIT(11)),
	PINCTRL_CONF_DESC(57, REG_GPIO_H_E4, BIT(12)),
	PINCTRL_CONF_DESC(58, REG_GPIO_H_E4, BIT(13)),
	PINCTRL_CONF_DESC(59, REG_GPIO_H_E4, BIT(14)),
	PINCTRL_CONF_DESC(61, REG_I2C_SDA_E4, PCIE0_RESET_E4_MASK),
	PINCTRL_CONF_DESC(62, REG_I2C_SDA_E4, PCIE1_RESET_E4_MASK),
	PINCTRL_CONF_DESC(63, REG_I2C_SDA_E4, PCIE2_RESET_E4_MASK),
};

static const struct airoha_pinctrl_conf airoha_pinctrl_pcie_rst_od_conf[] = {
	PINCTRL_CONF_DESC(61, REG_PCIE_RESET_OD, PCIE0_RESET_OD_MASK),
	PINCTRL_CONF_DESC(62, REG_PCIE_RESET_OD, PCIE1_RESET_OD_MASK),
	PINCTRL_CONF_DESC(63, REG_PCIE_RESET_OD, PCIE2_RESET_OD_MASK),
};

static u32 airoha_pinctrl_rmw(struct airoha_pinctrl *pinctrl,
			      void __iomem *addr, u32 mask, u32 val)
{
	mutex_lock(&pinctrl->mutex);
	val |= (readl(addr) & ~mask);
	writel(val, addr);
	mutex_unlock(&pinctrl->mutex);

	return val;
}

#define airoha_pinctrl_mux_set(pinctrl, base, offset, val)			\
	airoha_pinctrl_rmw((pinctrl), (base) + (offset), 0, (val));

static void airoha_pinctrl_gpio_set_direction(struct airoha_pinctrl *pinctrl,
					      unsigned int gpio, bool input)
{
	u32 mask, index;

	/* set output enable */
	mask = BIT(gpio % AIROHA_GPIO_BANK_SIZE);
	index = gpio / AIROHA_GPIO_BANK_SIZE;
	airoha_pinctrl_rmw(pinctrl, pinctrl->gpiochip.out[index],
			   mask, !input ? mask : 0);

	/* set gpio direction */
	mask = BIT(2 * (gpio % AIROHA_REG_GPIOCTRL_NUM_GPIO));
	index = gpio / AIROHA_REG_GPIOCTRL_NUM_GPIO;
	airoha_pinctrl_rmw(pinctrl, pinctrl->gpiochip.dir[index],
			   mask, !input ? mask : 0);
}

static int airoha_pinctrl_gpio_get_direction(struct airoha_pinctrl *pinctrl,
					     unsigned int gpio)
{
	u32 val, mask = BIT(2 * (gpio % AIROHA_REG_GPIOCTRL_NUM_GPIO));
	u8 index = gpio / AIROHA_REG_GPIOCTRL_NUM_GPIO;

	val = (readl(pinctrl->gpiochip.dir[index]) & mask);

	return val ? PIN_CONFIG_OUTPUT_ENABLE : PIN_CONFIG_INPUT_ENABLE;
}

static int airoha_pinmux_set_mux(struct pinctrl_dev *pctrl_dev,
				 unsigned int selector,
				 unsigned int group)
{
	struct airoha_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrl_dev);
	const struct airoha_pinctrl_func *func;
	struct function_desc *desc;
	struct group_desc *grp;
	int i;

	desc = pinmux_generic_get_function(pctrl_dev, selector);
	if (!desc)
		return -EINVAL;

	grp = pinctrl_generic_get_group(pctrl_dev, group);
	if (!grp)
		return -EINVAL;

	dev_err(pctrl_dev->dev, "enable function %s group %s\n",
		desc->name, grp->name);

	func = desc->data;
	for (i = 0; i < func->group_size; i++) {
		if (!strcmp(func->groups[i].name, grp->name)) {
			void __iomem *base;

			base = pinctrl->regs.mux[func->groups[i].mux_func];
			airoha_pinctrl_mux_set(pinctrl, base,
					       func->groups[i].reg.offset,
					       func->groups[i].reg.mask);
			return 0;
		}
	}

	return -EINVAL;
}

static int airoha_pinmux_gpio_set_direction(struct pinctrl_dev *pctrl_dev,
					    struct pinctrl_gpio_range *range,
					    unsigned int pin, bool input)
{
	struct airoha_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrl_dev);
	int gpio = pin - range->pin_base;

	airoha_pinctrl_gpio_set_direction(pinctrl, gpio, input);

	return 0;
}

static int airoha_pinctrl_get_gpio_from_pin(struct pinctrl_dev *pctrl_dev,
					    int pin)
{
	struct pinctrl_gpio_range *range;
	int gpio;

	range = pinctrl_find_gpio_range_from_pin(pctrl_dev, pin);
	if (!range)
		return -EINVAL;

	gpio = pin - range->pin_base;
	if (gpio < 0)
		return -EINVAL;

	return gpio;
}

static const struct airoha_pinctrl_reg *
airoha_pinctrl_get_conf_reg(const struct airoha_pinctrl_conf *conf,
			    int conf_size, int pin)
{
	int i;

	for (i = 0; i < conf_size; i++) {
		if (conf[i].pin == pin)
			return &conf[i].reg;
	}

	return NULL;
}

static int airoha_pinctrl_get_conf(void __iomem *base,
				   const struct airoha_pinctrl_conf *conf,
				   int conf_size, int pin, u32 *val)
{
	const struct airoha_pinctrl_reg *reg;

	reg = airoha_pinctrl_get_conf_reg(conf, conf_size, pin);
	if (!reg)
		return -EINVAL;

	*val = readl(base + reg->offset);
	*val = (*val & reg->mask) >> __bf_shf(reg->mask);

	return 0;
}

static int airoha_pinctrl_set_conf(struct airoha_pinctrl *pinctrl,
				   void __iomem *base,
				   const struct airoha_pinctrl_conf *conf,
				   int conf_size, int pin, u32 val)
{
	const struct airoha_pinctrl_reg *reg = NULL;

	reg = airoha_pinctrl_get_conf_reg(conf, conf_size, pin);
	if (!reg)
		return -EINVAL;

	airoha_pinctrl_rmw(pinctrl, base + reg->offset, reg->mask,
			   val << __bf_shf(reg->mask));

	return 0;
}

#define airoha_pinctrl_get_pullup_conf(pinctrl, pin, val)			\
	airoha_pinctrl_get_conf(((pinctrl)->regs.conf),				\
				airoha_pinctrl_pullup_conf,			\
				ARRAY_SIZE(airoha_pinctrl_pullup_conf),		\
				(pin), (val))
#define airoha_pinctrl_get_pulldown_conf(pinctrl, pin, val)			\
	airoha_pinctrl_get_conf(((pinctrl)->regs.conf),				\
				airoha_pinctrl_pulldown_conf,			\
				ARRAY_SIZE(airoha_pinctrl_pulldown_conf),	\
				(pin), (val))
#define airoha_pinctrl_get_drive_e2_conf(pinctrl, pin, val)			\
	airoha_pinctrl_get_conf(((pinctrl)->regs.conf),				\
				airoha_pinctrl_drive_e2_conf,			\
				ARRAY_SIZE(airoha_pinctrl_drive_e2_conf),	\
				(pin), (val))
#define airoha_pinctrl_get_drive_e4_conf(pinctrl, pin, val)			\
	airoha_pinctrl_get_conf(((pinctrl)->regs.conf),				\
				airoha_pinctrl_drive_e4_conf,			\
				ARRAY_SIZE(airoha_pinctrl_drive_e4_conf),	\
				(pin), (val))
#define airoha_pinctrl_get_pcie_rst_od_conf(pinctrl, pin, val)			\
	airoha_pinctrl_get_conf(((pinctrl)->regs.pcie_rst),			\
				airoha_pinctrl_pcie_rst_od_conf,		\
				ARRAY_SIZE(airoha_pinctrl_pcie_rst_od_conf),	\
				(pin), (val))
#define airoha_pinctrl_set_pullup_conf(pinctrl, pin, val)			\
	airoha_pinctrl_set_conf((pinctrl), ((pinctrl)->regs.conf),		\
				airoha_pinctrl_pullup_conf,			\
				ARRAY_SIZE(airoha_pinctrl_pullup_conf),		\
				(pin), (val))
#define airoha_pinctrl_set_pulldown_conf(pinctrl, pin, val)			\
	airoha_pinctrl_set_conf((pinctrl), ((pinctrl)->regs.conf),		\
				airoha_pinctrl_pulldown_conf,			\
				ARRAY_SIZE(airoha_pinctrl_pulldown_conf),	\
				(pin), (val))
#define airoha_pinctrl_set_drive_e2_conf(pinctrl, pin, val)			\
	airoha_pinctrl_set_conf((pinctrl), ((pinctrl)->regs.conf),		\
				airoha_pinctrl_drive_e2_conf,			\
				ARRAY_SIZE(airoha_pinctrl_drive_e2_conf),	\
				(pin), (val))
#define airoha_pinctrl_set_drive_e4_conf(pinctrl, pin, val)			\
	airoha_pinctrl_set_conf((pinctrl), ((pinctrl)->regs.conf),		\
				airoha_pinctrl_drive_e4_conf,			\
				ARRAY_SIZE(airoha_pinctrl_drive_e4_conf),	\
				(pin), (val))
#define airoha_pinctrl_set_pcie_rst_od_conf(pinctrl, pin, val)			\
	airoha_pinctrl_set_conf((pinctrl), ((pinctrl)->regs.pcie_rst),	\
				airoha_pinctrl_pcie_rst_od_conf,		\
				ARRAY_SIZE(airoha_pinctrl_pcie_rst_od_conf),	\
				(pin), (val))

static int airoha_pinconf_get(struct pinctrl_dev *pctrl_dev,
			      unsigned int pin, unsigned long *config)
{
	struct airoha_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrl_dev);
	enum pin_config_param param = pinconf_to_config_param(*config);
	u32 arg;

	dev_err(pctrl_dev->dev, "get conf for pin %d\n", pin);

	switch (param) {
	case PIN_CONFIG_BIAS_PULL_DOWN:
	case PIN_CONFIG_BIAS_DISABLE:
	case PIN_CONFIG_BIAS_PULL_UP: {
		u32 pull_up, pull_down;

		if (airoha_pinctrl_get_pullup_conf(pinctrl, pin, &pull_up) ||
		    airoha_pinctrl_get_pulldown_conf(pinctrl, pin, &pull_down))
			return -EINVAL;

		if (param == PIN_CONFIG_BIAS_PULL_UP &&
		    !(pull_up && !pull_down))
			return -EINVAL;
		else if (param == PIN_CONFIG_BIAS_PULL_DOWN &&
			 !(pull_down && !pull_up))
			return -EINVAL;
		else if (pull_up || pull_down)
			return -EINVAL;

		arg = 1;
		break;
	}
	case PIN_CONFIG_DRIVE_STRENGTH: {
		u32 e2, e4;

		if (airoha_pinctrl_get_drive_e2_conf(pinctrl, pin, &e2) ||
		    airoha_pinctrl_get_drive_e4_conf(pinctrl, pin, &e4))
			return -EINVAL;

		arg = e4 << 1 | e2;
		break;
	}
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		if (airoha_pinctrl_get_pcie_rst_od_conf(pinctrl, pin, &arg))
			return -EINVAL;
		break;
	case PIN_CONFIG_OUTPUT_ENABLE:
	case PIN_CONFIG_INPUT_ENABLE: {
		int gpio = airoha_pinctrl_get_gpio_from_pin(pctrl_dev, pin);

		if (gpio < 0)
			return gpio;

		arg = airoha_pinctrl_gpio_get_direction(pinctrl, gpio);
		if (arg != param)
			return -EINVAL;

		arg = 1;
		break;
	}
	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);

	return 0;
}

static int airoha_pinconf_set(struct pinctrl_dev *pctrl_dev,
			      unsigned int pin, unsigned long *configs,
			      unsigned int num_configs)
{
	struct airoha_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrl_dev);
	int i;

	for (i = 0; i < num_configs; i++) {
		u32 param = pinconf_to_config_param(configs[i]);
		u32 arg = pinconf_to_config_argument(configs[i]);

		switch (param) {
		case PIN_CONFIG_BIAS_DISABLE:
			airoha_pinctrl_set_pulldown_conf(pinctrl, pin, 0);
			airoha_pinctrl_set_pullup_conf(pinctrl, pin, 0);
			break;
		case PIN_CONFIG_BIAS_PULL_UP:
			airoha_pinctrl_set_pulldown_conf(pinctrl, pin, 0);
			airoha_pinctrl_set_pullup_conf(pinctrl, pin, 1);
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
			airoha_pinctrl_set_pulldown_conf(pinctrl, pin, 1);
			airoha_pinctrl_set_pullup_conf(pinctrl, pin, 0);
			break;
		case PIN_CONFIG_DRIVE_STRENGTH: {
			u32 e2 = 0, e4 = 0;

			switch (arg) {
			case MTK_DRIVE_2mA:
				break;
			case MTK_DRIVE_4mA:
				e2 = 1;
				break;
			case MTK_DRIVE_6mA:
				e4 = 1;
				break;
			case MTK_DRIVE_8mA:
				e2 = 1;
				e4 = 1;
				break;
			default:
				return -EINVAL;
			}

			airoha_pinctrl_set_drive_e2_conf(pinctrl, pin, e2);
			airoha_pinctrl_set_drive_e4_conf(pinctrl, pin, e4);
			break;
		}
		case PIN_CONFIG_DRIVE_OPEN_DRAIN:
			airoha_pinctrl_set_pcie_rst_od_conf(pinctrl, pin, !!arg);
			break;
		case PIN_CONFIG_OUTPUT_ENABLE:
		case PIN_CONFIG_INPUT_ENABLE: {
			int gpio = airoha_pinctrl_get_gpio_from_pin(pctrl_dev, pin);

			if (gpio < 0)
				return gpio;

			arg = param == PIN_CONFIG_INPUT_ENABLE;
			airoha_pinctrl_gpio_set_direction(pinctrl, gpio, arg);
			break;
		}
		default:
			return -ENOTSUPP;
		}
	}

	return 0;
}

static int airoha_pinconf_group_get(struct pinctrl_dev *pctrl_dev,
				   unsigned int group, unsigned long *config)
{
	u32 cur_config = 0;
	int i;

	for (i = 0; i < airoha_pinctrl_groups[group].npins; i++) {
		if (airoha_pinconf_get(pctrl_dev,
				       airoha_pinctrl_groups[group].pins[i],
				       config))
			return -ENOTSUPP;

		if (i && cur_config != *config)
			return -ENOTSUPP;

		cur_config= *config;
	}

	return 0;
}


static int airoha_pinconf_group_set(struct pinctrl_dev *pctrl_dev,
				    unsigned int group, unsigned long *configs,
				    unsigned int num_configs)
{
	int i;

	for (i = 0; i < airoha_pinctrl_groups[group].npins; i++) {
		int err;

		err = airoha_pinconf_set(pctrl_dev,
					 airoha_pinctrl_groups[group].pins[i],
					 configs, num_configs);
		if (err)
			return err;
	}

	return 0;
}

static const struct pinconf_ops airoha_confops = {
	.is_generic = true,
	.pin_config_get = airoha_pinconf_get,
	.pin_config_set = airoha_pinconf_set,
	.pin_config_group_get = airoha_pinconf_group_get,
	.pin_config_group_set = airoha_pinconf_group_set,
	.pin_config_config_dbg_show = pinconf_generic_dump_config,
};

static const struct pinctrl_ops airoha_pctlops = {
	.get_groups_count = pinctrl_generic_get_group_count,
	.get_group_name = pinctrl_generic_get_group_name,
	.get_group_pins = pinctrl_generic_get_group_pins,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_all,
	.dt_free_map = pinconf_generic_dt_free_map,
};

static const struct pinmux_ops airoha_pmxops = {
	.get_functions_count = pinmux_generic_get_function_count,
	.get_function_name = pinmux_generic_get_function_name,
	.get_function_groups = pinmux_generic_get_function_groups,
	.gpio_set_direction = airoha_pinmux_gpio_set_direction,
	.set_mux = airoha_pinmux_set_mux,
	.strict = true,
};

static struct pinctrl_desc airoha_pinctrl_desc = {
	.name = KBUILD_MODNAME,
	.owner = THIS_MODULE,
	.pctlops = &airoha_pctlops,
	.pmxops = &airoha_pmxops,
	.confops = &airoha_confops,
	.pins = airoha_pinctrl_pins,
	.npins = ARRAY_SIZE(airoha_pinctrl_pins),
};

static void airoha_pinctrl_gpio_set(struct gpio_chip *chip, unsigned int gpio,
				    int value)
{
	struct airoha_pinctrl *pinctrl = gpiochip_get_data(chip);
	u8 index = gpio / AIROHA_GPIO_BANK_SIZE;
	u32 pin = gpio % AIROHA_GPIO_BANK_SIZE;

	airoha_pinctrl_rmw(pinctrl, pinctrl->gpiochip.data[index],
			   BIT(pin), value ? BIT(pin) : 0);
}

static int airoha_pinctrl_gpio_get(struct gpio_chip *chip, unsigned int gpio)
{
	struct airoha_pinctrl *pinctrl = gpiochip_get_data(chip);
	u8 index = gpio / AIROHA_GPIO_BANK_SIZE;
	u32 pin = gpio % AIROHA_GPIO_BANK_SIZE;

	return !!(readl(pinctrl->gpiochip.data[index]) & BIT(pin));
}

static int airoha_pinctrl_gpio_direction_input(struct gpio_chip *chip,
					       unsigned int gpio)
{
	return pinctrl_gpio_direction_input(chip->base + gpio);
}

static int airoha_pinctrl_gpio_direction_output(struct gpio_chip *chip,
						unsigned int gpio, int value)
{
	int err;

	err = pinctrl_gpio_direction_output(chip->base + gpio);
	if (err)
		return err;

	airoha_pinctrl_gpio_set(chip, gpio, value);

	return 0;
}

static int airoha_pinctrl_add_gpiochip(struct airoha_pinctrl *pinctrl,
				       struct platform_device *pdev,
				       int index)
{
	struct gpio_chip *chip = &pinctrl->gpiochip.chip;
	struct device *dev = &pdev->dev;
	void __iomem *ptr;
	int i;

	for (i = 0; i < ARRAY_SIZE(pinctrl->gpiochip.data); i++) {
		ptr = devm_platform_ioremap_resource(pdev, index++);
		if (IS_ERR(ptr))
			return dev_err_probe(dev, PTR_ERR(ptr),
					     "failed to map gpio data regs\n");

		pinctrl->gpiochip.data[i] = ptr;
	}

	for (i = 0; i < ARRAY_SIZE(pinctrl->gpiochip.dir); i++) {
		ptr = devm_platform_ioremap_resource(pdev, index++);
		if (IS_ERR(ptr))
			return dev_err_probe(dev, PTR_ERR(ptr),
					     "failed to map gpio dir regs\n");

		pinctrl->gpiochip.dir[i] = ptr;
	}

	for (i = 0; i < ARRAY_SIZE(pinctrl->gpiochip.out); i++) {
		ptr = devm_platform_ioremap_resource(pdev, index++);
		if (IS_ERR(ptr))
			return dev_err_probe(dev, PTR_ERR(ptr),
					     "failed to map gpio out regs\n");

		pinctrl->gpiochip.out[i] = ptr;
	}

	chip->parent = dev;
	chip->label = dev_name(dev);
	chip->request = gpiochip_generic_request;
	chip->free = gpiochip_generic_free;
	chip->direction_input = airoha_pinctrl_gpio_direction_input;
	chip->direction_output = airoha_pinctrl_gpio_direction_output;
	chip->set = airoha_pinctrl_gpio_set;
	chip->get = airoha_pinctrl_gpio_get;
	chip->base = -1;
	chip->ngpio = AIROHA_NUM_GPIOS;

	return devm_gpiochip_add_data(dev, chip, pinctrl);
}

static int airoha_pinctrl_probe(struct platform_device *pdev)
{
	struct airoha_pinctrl *pinctrl;
	int err, i, index = 0;

	pinctrl = devm_kzalloc(&pdev->dev, sizeof(*pinctrl), GFP_KERNEL);
	if (!pinctrl)
		return -ENOMEM;

	mutex_init(&pinctrl->mutex);

	for (i = 0; i < ARRAY_SIZE(pinctrl->regs.mux); i++) {
		pinctrl->regs.mux[i] = devm_platform_ioremap_resource(pdev,
								      index++);
		if (IS_ERR(pinctrl->regs.mux[i]))
			return dev_err_probe(&pdev->dev,
					     PTR_ERR(pinctrl->regs.mux[i]),
					     "failed to iomap mux regs\n");
	}

	pinctrl->regs.conf = devm_platform_ioremap_resource(pdev, index++);
	if (IS_ERR(pinctrl->regs.conf))
		return dev_err_probe(&pdev->dev, PTR_ERR(pinctrl->regs.conf),
				     "failed to iomap conf regs\n");

	pinctrl->regs.pcie_rst = devm_platform_ioremap_resource(pdev, index++);
	if (IS_ERR(pinctrl->regs.pcie_rst))
		return dev_err_probe(&pdev->dev,
				     PTR_ERR(pinctrl->regs.pcie_rst),
				     "failed to iomap pcie rst od regs\n");

	err = devm_pinctrl_register_and_init(&pdev->dev, &airoha_pinctrl_desc,
					     pinctrl, &pinctrl->ctrl);
	if (err)
		return err;

	/* build pin groups */
	for (i = 0; i < ARRAY_SIZE(airoha_pinctrl_groups); i++) {
		const struct pingroup *grp = &airoha_pinctrl_groups[i];

		err = pinctrl_generic_add_group(pinctrl->ctrl, grp->name,
						(int *)grp->pins, grp->npins,
						(void *)grp);
		if (err < 0) {
			dev_err(&pdev->dev, "Failed to register group %s\n",
				grp->name);
			return err;
		}
	}

	/* build functions */
	for (i = 0; i < ARRAY_SIZE(airoha_pinctrl_funcs); i++) {
		const struct airoha_pinctrl_func *func;

		func = &airoha_pinctrl_funcs[i];
		err = pinmux_generic_add_function(pinctrl->ctrl,
						  func->desc.name,
						  func->desc.group_names,
						  func->desc.num_group_names,
						  (void *)func);
		if (err < 0) {
			dev_err(&pdev->dev, "Failed to register function %s\n",
				func->desc.name);
			return err;
		}
	}

	err = pinctrl_enable(pinctrl->ctrl);
	if (err)
		return err;

	/* build gpio-chip */
	return airoha_pinctrl_add_gpiochip(pinctrl, pdev, index);
}

const struct of_device_id of_airoha_pinctrl_match[] = {
	{ .compatible = "airoha,en7581-pinctrl" },
	{ /* sentinel */ }
};

static struct platform_driver airoha_pinctrl_driver = {
	.probe = airoha_pinctrl_probe,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = of_airoha_pinctrl_match,
	},
};
module_platform_driver(airoha_pinctrl_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lorenzo Bianconi <lorenzo@kernel.org>");
MODULE_AUTHOR("Benjamin Larsson <benjamin.larsson@genexis.eu>");
MODULE_DESCRIPTION("Pinctrl driver for Airoha SoC");
