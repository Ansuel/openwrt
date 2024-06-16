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

#define PINCTRL_PIN_GROUP(name, id)						\
	PINCTRL_PINGROUP(name, id##_pins, ARRAY_SIZE(id##_pins))

#define PINCTRL_FUNC_DESC(name, id)						\
	{									\
		.desc = { name, id##_groups, ARRAY_SIZE(id##_groups) },		\
		.groups = id##_func_group,					\
		.group_size = ARRAY_SIZE(id##_func_group),			\
	}

#define PINCTRL_CONF_DESC(p, offset, mask)					\
	{									\
		.pin = p,							\
		.reg = { offset, mask },					\
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

#define AIROHA_GPIO_BANK_SIZE			32
#define AIROHA_REG_GPIOCTRL_NUM_GPIO		(AIROHA_GPIO_BANK_SIZE / 2)

struct airoha_pinctrl_reg {
	u32 offset;
	u32 mask;
};

struct airoha_pinctrl_func_group {
	const char *name;
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
		void __iomem *mux;
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
	PINCTRL_PIN(0, "UART1_TXD"),
	PINCTRL_PIN(1, "UART1_RXD"),
	PINCTRL_PIN(2, "I2C_SCL"),
	PINCTRL_PIN(3, "I2C_SDA"),
	PINCTRL_PIN(4, "SPI_CS0"),
	PINCTRL_PIN(5, "SPI_CLK"),
	PINCTRL_PIN(6, "SPI_MOSI"),
	PINCTRL_PIN(7, "SPI_MISO"),
	PINCTRL_PIN(8, "HW_RSTN"),
	PINCTRL_PIN(9, "PKG_SEL0"),
	PINCTRL_PIN(10, "PKG_SEL1"),
	PINCTRL_PIN(11, "PKG_SEL2"),
	PINCTRL_PIN(12, "PKG_SEL3"),
	PINCTRL_PIN(13, "GPIO0"),
	PINCTRL_PIN(14, "GPIO1"),
	PINCTRL_PIN(15, "GPIO2"),
	PINCTRL_PIN(16, "GPIO3"),
	PINCTRL_PIN(17, "GPIO4"),
	PINCTRL_PIN(18, "GPIO5"),
	PINCTRL_PIN(19, "GPIO6"),
	PINCTRL_PIN(20, "GPIO7"),
	PINCTRL_PIN(21, "GPIO8"),
	PINCTRL_PIN(22, "GPIO9"),
	PINCTRL_PIN(23, "GPIO10"),
	PINCTRL_PIN(24, "GPIO11"),
	PINCTRL_PIN(25, "GPIO12"),
	PINCTRL_PIN(26, "GPIO13"),
	PINCTRL_PIN(27, "GPIO14"),
	PINCTRL_PIN(28, "GPIO15"),
	PINCTRL_PIN(29, "GPIO16"),
	PINCTRL_PIN(30, "GPIO17"),
	PINCTRL_PIN(31, "GPIO18"),
	PINCTRL_PIN(32, "GPIO19"),
	PINCTRL_PIN(33, "GPIO20"),
	PINCTRL_PIN(34, "GPIO21"),
	PINCTRL_PIN(35, "GPIO22"),
	PINCTRL_PIN(36, "GPIO23"),
	PINCTRL_PIN(37, "GPIO24"),
	PINCTRL_PIN(38, "GPIO25"),
	PINCTRL_PIN(39, "GPIO26"),
	PINCTRL_PIN(40, "GPIO27"),
	PINCTRL_PIN(41, "GPIO28"),
	PINCTRL_PIN(42, "GPIO29"),
	PINCTRL_PIN(43, "GPIO30"),
	PINCTRL_PIN(44, "GPIO31"),
	PINCTRL_PIN(45, "GPIO32"),
	PINCTRL_PIN(46, "GPIO33"),
	PINCTRL_PIN(47, "GPIO34"),
	PINCTRL_PIN(48, "GPIO35"),
	PINCTRL_PIN(49, "GPIO36"),
	PINCTRL_PIN(50, "GPIO37"),
	PINCTRL_PIN(51, "GPIO38"),
	PINCTRL_PIN(52, "GPIO39"),
	PINCTRL_PIN(53, "GPIO40"),
	PINCTRL_PIN(54, "GPIO41"),
	PINCTRL_PIN(55, "GPIO42"),
	PINCTRL_PIN(56, "GPIO43"),
	PINCTRL_PIN(57, "GPIO44"),
	PINCTRL_PIN(58, "GPIO45"),
	PINCTRL_PIN(59, "GPIO46"),
	PINCTRL_PIN(61, "PCIE_RESET0"),
	PINCTRL_PIN(62, "PCIE_RESET1"),
	PINCTRL_PIN(63, "PCIE_RESET2"),
	PINCTRL_PIN(64, "MDC0"),
	PINCTRL_PIN(65, "MDIO0"),
};

static const int pon_pins[] = { 49, 50, 51, 52, 53, 54 };
static const int tod_1pps_pins[] = { 46 };
static const int sipo_pins[] = { 16, 17 };
static const int sipo_rclk_pins[] = { 16, 17, 43 };
static const int mdio0_pins[] = { 64, 65 };
static const int mdio1_pins[] = { 14, 15 };
static const int uart1_pins[] = { 0, 1 };
static const int uart2_pins[] = { 48, 55 };
static const int uart2_cts_rts_pins[] = { 46, 47 };
static const int hsuart_pins[] = { 28, 29 };
static const int hsuart_cts_rts_pins[] = { 26, 27 };
static const int uart4_pins[] = { 38, 39 };
static const int uart5_pins[] = { 18, 19 };
static const int i2c0_pins[] = { 2, 3 };
static const int i2c1_pins[] = { 14, 15 };
static const int jtag_pins[] = { 16, 17, 18, 19, 20 };
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
static const int pcm_spi_cs3_pins[] = { 41 };
static const int pcm_spi_cs4_pins[] = { 42 };
static const int emmc_pins[] = { 4, 5, 6, 30, 31, 32, 33, 34, 35, 36, 37 };
static const int pnand_pins[] = { 4, 5, 6, 7, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42 };
static const int gpio47_pins[] = { 61 };
static const int gpio48_pins[] = { 62 };
static const int gpio49_pins[] = { 63 };
static const int lan0_led0_pins[] = { 46 };
static const int lan0_led1_pins[] = { 56 };
static const int lan1_led0_pins[] = { 47 };
static const int lan1_led1_pins[] = { 57 };
static const int lan2_led0_pins[] = { 48 };
static const int lan2_led1_pins[] = { 58 };
static const int lan3_led0_pins[] = { 55 };
static const int lan3_led1_pins[] = { 59 };

static const struct pingroup airoha_pinctrl_groups[] = {
	PINCTRL_PIN_GROUP("pon", pon),
	PINCTRL_PIN_GROUP("pon_tod_1pps", tod_1pps),
	PINCTRL_PIN_GROUP("gsw_tod_1pps", tod_1pps),
	PINCTRL_PIN_GROUP("sipo", sipo),
	PINCTRL_PIN_GROUP("sipo_rclk", sipo_rclk),
	PINCTRL_PIN_GROUP("mdio0", mdio0),
	PINCTRL_PIN_GROUP("mdio1", mdio1),
	PINCTRL_PIN_GROUP("uart1", uart1),
	PINCTRL_PIN_GROUP("uart2", uart2),
	PINCTRL_PIN_GROUP("uart2_cts_rts", uart2_cts_rts),
	PINCTRL_PIN_GROUP("hsuart", hsuart),
	PINCTRL_PIN_GROUP("hsuart_cts_rts", hsuart_cts_rts),
	PINCTRL_PIN_GROUP("uart4", uart4),
	PINCTRL_PIN_GROUP("uart5", uart5),
	PINCTRL_PIN_GROUP("i2c0", i2c0),
	PINCTRL_PIN_GROUP("i2c1", i2c1),
	PINCTRL_PIN_GROUP("jtag_udi", jtag),
	PINCTRL_PIN_GROUP("jtag_dfd", jtag),
	PINCTRL_PIN_GROUP("i2s", i2s),
	PINCTRL_PIN_GROUP("pcm1", pcm1),
	PINCTRL_PIN_GROUP("pcm2", pcm2),
	PINCTRL_PIN_GROUP("spi", spi),
	PINCTRL_PIN_GROUP("spi_quad", spi_quad),
	PINCTRL_PIN_GROUP("spi_cs1", spi_cs1),
	PINCTRL_PIN_GROUP("pcm_spi", pcm_spi),
	PINCTRL_PIN_GROUP("pcm_spi_int", pcm_spi_int),
	PINCTRL_PIN_GROUP("pcm_spi_rst", pcm_spi_rst),
	PINCTRL_PIN_GROUP("pcm_spi_cs1", pcm_spi_cs1),
	PINCTRL_PIN_GROUP("pcm_spi_cs2_p128", pcm_spi_cs2),
	PINCTRL_PIN_GROUP("pcm_spi_cs2_p156", pcm_spi_cs2),
	PINCTRL_PIN_GROUP("pcm_spi_cs2", pcm_spi_cs1),
	PINCTRL_PIN_GROUP("pcm_spi_cs3", pcm_spi_cs3),
	PINCTRL_PIN_GROUP("pcm_spi_cs4", pcm_spi_cs4),
	PINCTRL_PIN_GROUP("emmc", emmc),
	PINCTRL_PIN_GROUP("pnand", pnand),
	PINCTRL_PIN_GROUP("gpio47", gpio47),
	PINCTRL_PIN_GROUP("gpio48", gpio48),
	PINCTRL_PIN_GROUP("gpio49", gpio49),
	PINCTRL_PIN_GROUP("lan0_led0", lan0_led0),
	PINCTRL_PIN_GROUP("lan0_led1", lan0_led1),
	PINCTRL_PIN_GROUP("lan1_led0", lan1_led0),
	PINCTRL_PIN_GROUP("lan1_led1", lan1_led1),
	PINCTRL_PIN_GROUP("lan2_led0", lan2_led0),
	PINCTRL_PIN_GROUP("lan2_led1", lan2_led1),
	PINCTRL_PIN_GROUP("lan3_led0", lan3_led0),
	PINCTRL_PIN_GROUP("lan3_led1", lan3_led1),
};

static const char *const pon_groups[] = { "pon" };
static const char *const tod_1pps_groups[] = { "pon_tod_1pps", "gsw_tod_1pps" };
static const char *const sipo_groups[] = { "sipo", "sipo_rclk" };
static const char *const mdio_groups[] = { "mdio0", "mdio1" };
static const char *const uart_groups[] = { "uart1", "uart2", "uart2_cts_rts",
					   "hsuart", "hsuart_cts_rts",
					   "uart4", "uart5" };
static const char *const i2c_groups[] = { "i2c0", "i2c1" };
static const char *const jtag_groups[] = { "jtag_udi", "jtag_dfd" };
static const char *const pcm_groups[] = { "pcm1", "pcm2" };
static const char *const spi_groups[] = { "spi", "spi_quad", "spi_cs1",
					  "spi_cs2", "spi_cs3", "spi_cs4" };
static const char *const pcm_spi_groups[] = { "pcm_spi", "pcm_spi_int",
					      "pcm_spi_rst", "pcm_spi_cs1",
					      "pcm_spi_cs2_p156",
					      "pcm_spi_cs2_p128" ,
					      "pcm_spi_cs3", "pcm_spi_cs4" };
static const char *const i2s_groups[] = { "i2s" };
static const char *const emmc_groups[] = { "emmc" };
static const char *const pnand_groups[] = { "pnand" };
static const char *const gpio_groups[] = { "gpio47", "gpio48", "gpio49" };
static const char *const led_groups[] = { "lan0_led0", "lan0_led1",
					  "lan1_led0", "lan1_led1",
					  "lan2_led0", "lan2_led1",
					  "lan3_led0", "lan3_led1" };

static const struct airoha_pinctrl_func_group pon_func_group[] = {
	{ "pon", { REG_GPIO_PON_MODE, GPIO_PON_MODE_MASK }},
};

static const struct airoha_pinctrl_func_group tod_1pps_func_group[] = {
	{ "pon_tod_1pps", { REG_GPIO_2ND_I2C_MODE, PON_TOD_1PPS_MODE_MASK }},
	{ "gsw_tod_1pps", { REG_GPIO_2ND_I2C_MODE, GSW_TOD_1PPS_MODE_MASK }},
};

static const struct airoha_pinctrl_func_group sipo_func_group[] = {
	{ "sipo", { REG_GPIO_PON_MODE, GPIO_SIPO_MODE_MASK }},
	{ "sipo_rclk", { REG_GPIO_PON_MODE, GPIO_SIPO_MODE_MASK | SIPO_RCLK_MODE_MASK }},
};

static const struct airoha_pinctrl_func_group mdio_func_group[] = {
	{ "mdio1", { REG_GPIO_PON_MODE, GPIO_SGMII_MDIO_MODE_MASK }},
};

static const struct airoha_pinctrl_func_group uart_func_group[] = {
	{ "uart2", { REG_GPIO_PON_MODE, GPIO_UART2_MODE_MASK }},
	{ "uart2_cts_rts", { REG_GPIO_PON_MODE, GPIO_UART2_MODE_MASK | GPIO_UART2_CTS_RTS_MODE_MASK }},
	{ "hsuart", { REG_GPIO_PON_MODE, GPIO_HSUART_MODE_MASK }},
	{ "hsuart_cts_rts", { REG_GPIO_PON_MODE, GPIO_HSUART_MODE_MASK | GPIO_HSUART_CTS_RTS_MODE_MASK }},
	{ "uart4", { REG_GPIO_PON_MODE, GPIO_UART4_MODE_MASK }},
	{ "uart5", { REG_GPIO_PON_MODE, GPIO_UART5_MODE_MASK }},
};

static const struct airoha_pinctrl_func_group i2c_func_group[] = {
	{ "i2c1", { REG_GPIO_2ND_I2C_MODE, GPIO_2ND_I2C_MODE_MASK }},
};

static const struct airoha_pinctrl_func_group jtag_func_group[] = {
	{ "jtag_udi", { REG_NPU_UART_EN, JTAG_UDI_EN_MASK }},
	{ "jtag_dfd", { REG_NPU_UART_EN, JTAG_DFD_EN_MASK }},
};

static const struct airoha_pinctrl_func_group pcm_func_group[] = {
	{ "pcm1", { REG_GPIO_SPI_CS1_MODE, GPIO_PCM1_MODE_MASK }},
	{ "pcm2", { REG_GPIO_SPI_CS1_MODE, GPIO_PCM2_MODE_MASK }},
};

static const struct airoha_pinctrl_func_group spi_func_group[] = {
	{ "spi_quad", { REG_GPIO_SPI_CS1_MODE, GPIO_SPI_QUAD_MODE_MASK }},
	{ "spi_cs1", { REG_GPIO_SPI_CS1_MODE, GPIO_SPI_CS1_MODE_MASK }},
	{ "spi_cs2", { REG_GPIO_SPI_CS1_MODE, GPIO_SPI_CS2_MODE_MASK }},
	{ "spi_cs3", { REG_GPIO_SPI_CS1_MODE, GPIO_SPI_CS3_MODE_MASK }},
	{ "spi_cs4", { REG_GPIO_SPI_CS1_MODE, GPIO_SPI_CS4_MODE_MASK }},
};

static const struct airoha_pinctrl_func_group pcm_spi_func_group[] = {
	{ "pcm_spi", { REG_GPIO_SPI_CS1_MODE, GPIO_PCM_SPI_MODE_MASK }},
	{ "pcm_spi_int", { REG_GPIO_SPI_CS1_MODE, GPIO_PCM_INT_MODE_MASK }},
	{ "pcm_spi_rst", { REG_GPIO_SPI_CS1_MODE, GPIO_PCM_RESET_MODE_MASK }},
	{ "pcm_spi_cs1", { REG_GPIO_SPI_CS1_MODE, GPIO_PCM_SPI_CS1_MODE_MASK }},
	{ "pcm_spi_cs2_p128", { REG_GPIO_SPI_CS1_MODE, GPIO_PCM_SPI_CS2_MODE_P128_MASK }},
	{ "pcm_spi_cs2_p156", { REG_GPIO_SPI_CS1_MODE, GPIO_PCM_SPI_CS2_MODE_P156_MASK }},
	{ "pcm_spi_cs3", { REG_GPIO_SPI_CS1_MODE, GPIO_PCM_SPI_CS3_MODE_MASK }},
	{ "pcm_spi_cs4", { REG_GPIO_SPI_CS1_MODE, GPIO_PCM_SPI_CS4_MODE_MASK }},
};

static const struct airoha_pinctrl_func_group i2s_func_group[] = {
	{ "i2s", { REG_GPIO_2ND_I2C_MODE, GPIO_I2S_MODE_MASK }},
};

static const struct airoha_pinctrl_func_group emmc_func_group[] = {
	{ "emmc", { REG_GPIO_PON_MODE, GPIO_EMMC_MODE_MASK }},
};

static const struct airoha_pinctrl_func_group pnand_func_group[] = {
	{ "pnand", { REG_GPIO_PON_MODE, GPIO_PARALLEL_NAND_MODE_MASK }},
};

static const struct airoha_pinctrl_func_group gpio_func_group[] = {
	{ "gpio47", { REG_GPIO_PON_MODE, GPIO_PCIE_RESET0_MASK }},
	{ "gpio48", { REG_GPIO_PON_MODE, GPIO_PCIE_RESET1_MASK }},
	{ "gpio49", { REG_GPIO_PON_MODE, GPIO_PCIE_RESET2_MASK }},
};

static const struct airoha_pinctrl_func_group led_func_group[] = {
	{ "lan0_led0", { REG_GPIO_2ND_I2C_MODE, GPIO_LAN0_LED0_MODE_MASK }},
	{ "lan0_led1", { REG_GPIO_2ND_I2C_MODE, GPIO_LAN0_LED1_MODE_MASK }},
	{ "lan1_led0", { REG_GPIO_2ND_I2C_MODE, GPIO_LAN1_LED0_MODE_MASK }},
	{ "lan1_led1", { REG_GPIO_2ND_I2C_MODE, GPIO_LAN1_LED1_MODE_MASK }},
	{ "lan2_led0", { REG_GPIO_2ND_I2C_MODE, GPIO_LAN2_LED0_MODE_MASK }},
	{ "lan2_led1", { REG_GPIO_2ND_I2C_MODE, GPIO_LAN2_LED1_MODE_MASK }},
	{ "lan3_led0", { REG_GPIO_2ND_I2C_MODE, GPIO_LAN3_LED0_MODE_MASK }},
	{ "lan3_led1", { REG_GPIO_2ND_I2C_MODE, GPIO_LAN3_LED1_MODE_MASK }},
};

static const struct airoha_pinctrl_func airoha_pinctrl_funcs[] = {
	PINCTRL_FUNC_DESC("pon", pon),
	PINCTRL_FUNC_DESC("tod_1pps", tod_1pps),
	PINCTRL_FUNC_DESC("sipo", sipo),
	PINCTRL_FUNC_DESC("mdio", mdio),
	PINCTRL_FUNC_DESC("uart", uart),
	PINCTRL_FUNC_DESC("i2c", i2c),
	PINCTRL_FUNC_DESC("jtag", jtag),
	PINCTRL_FUNC_DESC("pcm", pcm),
	PINCTRL_FUNC_DESC("spi", spi),
	PINCTRL_FUNC_DESC("pcm_spi", pcm_spi),
	PINCTRL_FUNC_DESC("i2s", i2s),
	PINCTRL_FUNC_DESC("emmc", emmc),
	PINCTRL_FUNC_DESC("pnand", pnand),
	PINCTRL_FUNC_DESC("gpio", gpio),
	PINCTRL_FUNC_DESC("led", led),
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

#define airoha_pinctrl_mux_set(pinctrl, offset, val)			\
	airoha_pinctrl_rmw((pinctrl), ((pinctrl)->regs.mux) + (offset),	\
			   0, (val));

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

	dev_dbg(pctrl_dev->dev, "enable function %s group %s\n",
		desc->name, grp->name);

	func = desc->data;
	for (i = 0; i < func->group_size; i++) {
		if (!strcmp(func->groups[i].name, grp->name)) {
			airoha_pinctrl_mux_set(pinctrl,
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
	u32 dir_mask = BIT(2 * (pin % AIROHA_REG_GPIOCTRL_NUM_GPIO));
	u8 dir_index = pin / AIROHA_REG_GPIOCTRL_NUM_GPIO;
	u32 out_mask = BIT(pin % AIROHA_GPIO_BANK_SIZE);
	u8 out_index = pin / AIROHA_GPIO_BANK_SIZE;

	airoha_pinctrl_rmw(pinctrl, pinctrl->gpiochip.dir[dir_index],
			   dir_mask, !input ? dir_mask : 0);
	airoha_pinctrl_rmw(pinctrl, pinctrl->gpiochip.out[out_index],
			   out_mask, !input ? out_mask : 0);

	return 0;
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

	dev_dbg(pctrl_dev->dev, "get conf for pin %d\n", pin);

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

static void airoha_pinctrl_gpio_set(struct gpio_chip *chip, unsigned int gpio,
				    int value)
{
	struct airoha_pinctrl *pinctrl = gpiochip_get_data(chip);
	u8 index = gpio / AIROHA_GPIO_BANK_SIZE;

	airoha_pinctrl_rmw(pinctrl, pinctrl->gpiochip.data[index],
			   BIT(gpio), !!value ? BIT(gpio) : 0);
}

static int airoha_pinctrl_gpio_get(struct gpio_chip *chip, unsigned int gpio)
{
	struct airoha_pinctrl *pinctrl = gpiochip_get_data(chip);
	u8 index = gpio / AIROHA_GPIO_BANK_SIZE;

	return readl(pinctrl->gpiochip.data[index]) & BIT(gpio);
}

static int airoha_pinctrl_gpio_direction_input(struct gpio_chip *chip,
					       unsigned int gpio)
{
	return pinctrl_gpio_direction_input(chip->base + gpio);
}

static int airoha_pinctrl_gpio_direction_output(struct gpio_chip *chip,
						unsigned int gpio, int value)
{
	airoha_pinctrl_gpio_set(chip, gpio, value);

	return pinctrl_gpio_direction_output(chip->base + gpio);
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
	chip->ngpio = ARRAY_SIZE(airoha_pinctrl_pins);

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

	pinctrl->regs.mux = devm_platform_ioremap_resource(pdev, index++);
	if (IS_ERR(pinctrl->regs.mux))
		return dev_err_probe(&pdev->dev, PTR_ERR(pinctrl->regs.mux),
				     "failed to iomap mux regs\n");

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

	/* build gpio-chips */
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
