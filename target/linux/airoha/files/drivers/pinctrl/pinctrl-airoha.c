// SPDX-License-Identifier: GPL-2.0-only
/*
 * Author: Lorenzo Bianconi <lorenzo@kernel.org>
 * Author: Benjamin Larsson <benjamin.larsson@genexis.eu>
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>

#include "core.h"
#include "pinmux.h"

#define PINCTRL_PIN_GROUP(name, id)						\
	PINCTRL_PINGROUP(name, id##_pins, ARRAY_SIZE(id##_pins))

#define PINCTRL_FUNC_DESC(name, id)						\
	{									\
		.desc = { name, id##_groups, ARRAY_SIZE(id##_groups) },		\
		.regmap = id##_func_regmap,					\
		.regmap_size = ARRAY_SIZE(id##_func_regmap),			\
	}

/* MUX */
#define REG_GPIO_2ND_I2C_MODE			0x00
#define GPIO_MDC_IO_MASTER_MODE_MODE	BIT(14)
#define GPIO_I2C_MASTER_MODE_MODE	BIT(13)
#define GPIO_I2S_MODE_MASK			BIT(12)
#define GPIO_I2C_SLAVE_MODE_MODE		BIT(11)
#define GPIO_PORT3_LED1_MODE_MASK		BIT(10)
#define GPIO_PORT3_LED0_MODE_MASK		BIT(9)
#define GPIO_PORT2_LED1_MODE_MASK		BIT(8)
#define GPIO_PORT2_LED0_MODE_MASK		BIT(7)
#define GPIO_PORT1_LED1_MODE_MASK		BIT(6)
#define GPIO_PORT1_LED0_MODE_MASK		BIT(5)
#define GPIO_PORT0_LED1_MODE_MASK		BIT(4)
#define GPIO_PORT0_LED0_MODE_MASK		BIT(3)
#define PON_TOD_1PPS_MODE_MASK			BIT(2)
#define PON_SW_TOD_1PPS_MODE_MASK		BIT(1)
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
#define GPIO_HSUART3_CTS_RTS_MODE_MASK		BIT(6)
#define GPIO_HSUART3_MODE_MASK			BIT(5)
#define GPIO_UART2_CTS_RTS_MODE_MASK		BIT(4)
#define GPIO_UART2_MODE_MASK			BIT(3)
#define GPIO_SIPO_MODE_MASK			BIT(2)
#define GPIO_EMMC_MODE_MASK			BIT(1)
#define GPIO_PON_MODE_MASK			BIT(0)

#define REG_NPU_UART_EN				0x10
#define JTAG_UDI_EN_MASK			BIT(4)
#define JTAG_DFD_EN_MASK			BIT(3)

struct airoha_pinctrl_func_reg {
	u32 offset;
	u32 mask;
};

struct airoha_pinctrl_func {
	const struct function_desc desc;
	const struct airoha_pinctrl_func_reg *regmap;
	u8 regmap_size;
};

struct airoha_pinctrl {
	struct pinctrl_dev *ctrl;

	struct mutex mutex;
	void __iomem *mux_regs;
	void __iomem *conf_regs;
};

static struct pinctrl_pin_desc airoha_pinctrl_pins[] = {
	PINCTRL_PIN(1, "UART1_TXD"),
	PINCTRL_PIN(2, "UART1_RXD"),
	PINCTRL_PIN(3, "I2C_SCL"),
	PINCTRL_PIN(4, "I2C_SDA"),
	PINCTRL_PIN(5, "SPI_CS0"),
	PINCTRL_PIN(6, "SPI_CLK"),
	PINCTRL_PIN(7, "SPI_MOSI"),
	PINCTRL_PIN(8, "SPI_MISO"),
	PINCTRL_PIN(9, "HW_RSTN"),
	PINCTRL_PIN(10, "PKG_SEL0"),
	PINCTRL_PIN(11, "PKG_SEL1"),
	PINCTRL_PIN(12, "PKG_SEL2"),
	PINCTRL_PIN(13, "PKG_SEL3"),
	PINCTRL_PIN(14, "GPIO0"),
	PINCTRL_PIN(15, "GPIO1"),
	PINCTRL_PIN(16, "GPIO2"),
	PINCTRL_PIN(17, "GPIO3"),
	PINCTRL_PIN(18, "GPIO4"),
	PINCTRL_PIN(19, "GPIO5"),
	PINCTRL_PIN(20, "GPIO6"),
	PINCTRL_PIN(21, "GPIO7"),
	PINCTRL_PIN(22, "GPIO8"),
	PINCTRL_PIN(23, "GPIO9"),
	PINCTRL_PIN(24, "GPIO10"),
	PINCTRL_PIN(25, "GPIO11"),
	PINCTRL_PIN(26, "GPIO12"),
	PINCTRL_PIN(27, "GPIO13"),
	PINCTRL_PIN(28, "GPIO14"),
	PINCTRL_PIN(29, "GPIO15"),
	PINCTRL_PIN(30, "GPIO16"),
	PINCTRL_PIN(31, "GPIO17"),
	PINCTRL_PIN(32, "GPIO18"),
	PINCTRL_PIN(33, "GPIO19"),
	PINCTRL_PIN(34, "GPIO20"),
	PINCTRL_PIN(35, "GPIO21"),
	PINCTRL_PIN(36, "GPIO22"),
	PINCTRL_PIN(37, "GPIO23"),
	PINCTRL_PIN(38, "GPIO24"),
	PINCTRL_PIN(39, "GPIO25"),
	PINCTRL_PIN(40, "GPIO26"),
	PINCTRL_PIN(41, "GPIO27"),
	PINCTRL_PIN(42, "GPIO28"),
	PINCTRL_PIN(43, "GPIO29"),
	PINCTRL_PIN(44, "GPIO30"),
	PINCTRL_PIN(45, "GPIO31"),
	PINCTRL_PIN(46, "GPIO32"),
	PINCTRL_PIN(47, "GPIO33"),
	PINCTRL_PIN(48, "GPIO34"),
	PINCTRL_PIN(49, "GPIO35"),
	PINCTRL_PIN(50, "GPIO36"),
	PINCTRL_PIN(51, "GPIO37"),
	PINCTRL_PIN(52, "GPIO38"),
	PINCTRL_PIN(53, "GPIO39"),
	PINCTRL_PIN(54, "GPIO40"),
	PINCTRL_PIN(55, "GPIO41"),
	PINCTRL_PIN(56, "GPIO42"),
	PINCTRL_PIN(57, "GPIO43"),
	PINCTRL_PIN(58, "GPIO44"),
	PINCTRL_PIN(59, "GPIO45"),
	PINCTRL_PIN(60, "GPIO46"),
	PINCTRL_PIN(62, "PCIE_RESET0"),
	PINCTRL_PIN(63, "PCIE_RESET1"),
	PINCTRL_PIN(64, "PCIE_RESET2"),
	PINCTRL_PIN(65, "MDC0"),
	PINCTRL_PIN(66, "MDIO0"),
};

static const int pon0_pins[] = { 50, 51, 52, 53, 54, 55 };
static const int tod_pins[] = { 47 };
static const int sipo_pins[] = { 17, 18 };
static const int sipo_rclk_pins[] = { 17, 18, 44 };
static const int sgmii_mdio_pins[] = { 15, 16 };
static const int uart2_pins[] = { 49, 56 };
static const int uart2_cts_rts_pins[] = { 47, 48 };
static const int hsuart3_pins[] = { 29, 30 };
static const int hsuart3_cts_rts_pins[] = { 27, 28 };
static const int uart4_pins[] = { 39, 40 };
static const int uart5_pins[] = { 19, 20 };
static const int i2c1_pins[] = { 15, 16 };
static const int jtag_pins[] = { 17, 18, 19, 20, 21 };
static const int i2s0_pins[] = { 27, 28, 29, 30 };
static const int pcm1_pins[] = { 23, 24, 25, 26 };
static const int pcm2_pins[] = { 19, 20, 21, 22 };
static const int spi_quad_pins[] = { 33, 34 };
static const int spi_cs1_pins[] = { 35 };
static const int pcm_spi_pins[] = { 19, 20, 21, 22, 23, 24, 25, 26 };
static const int pcm_spi_int_pins[] = { 15 };
static const int pcm_spi_reset_pins[] = { 16 };
static const int pcm_spi_cs1_pins[] = { 44 };
static const int pcm_spi_cs2_pins[] = { 41 };
static const int pcm_spi_cs3_pins[] = { 42 };
static const int pcm_spi_cs4_pins[] = { 43 };
static const int emmc_pins[] = { 5, 6, 7, 31, 32, 33, 34, 35, 36, 37, 38 };
static const int pnand_pins[] = { 5, 6, 7, 8, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43 };
static const int gpio47_pins[] = { 62 };
static const int gpio48_pins[] = { 63 };
static const int gpio49_pins[] = { 64 };
static const int port0_led0_pins[] = { 47 };
static const int port0_led1_pins[] = { 57 };
static const int port1_led0_pins[] = { 48 };
static const int port1_led1_pins[] = { 58 };
static const int port2_led0_pins[] = { 49 };
static const int port2_led1_pins[] = { 59 };
static const int port3_led0_pins[] = { 56 };
static const int port3_led1_pins[] = { 60 };

static const struct pingroup airoha_pinctrl_groups[] = {
	PINCTRL_PIN_GROUP("pon0", pon0),
	PINCTRL_PIN_GROUP("tod_pon", tod),
	PINCTRL_PIN_GROUP("tod_sw_pon", tod),
	PINCTRL_PIN_GROUP("sipo", sipo),
	PINCTRL_PIN_GROUP("sipo_rclk", sipo_rclk),
	PINCTRL_PIN_GROUP("mdio", sgmii_mdio),
	PINCTRL_PIN_GROUP("uart2", uart2),
	PINCTRL_PIN_GROUP("uart2_cts_rts", uart2_cts_rts),
	PINCTRL_PIN_GROUP("hsuart3", hsuart3),
	PINCTRL_PIN_GROUP("hsuart3_cts_rts", hsuart3_cts_rts),
	PINCTRL_PIN_GROUP("uart4", uart4),
	PINCTRL_PIN_GROUP("uart5", uart5),
	PINCTRL_PIN_GROUP("i2c1", i2c1),
	PINCTRL_PIN_GROUP("jtag_udi", jtag),
	PINCTRL_PIN_GROUP("jtag_dfd", jtag),
	PINCTRL_PIN_GROUP("i2s0", i2s0),
	PINCTRL_PIN_GROUP("pcm1", pcm1),
	PINCTRL_PIN_GROUP("pcm2", pcm2),
	PINCTRL_PIN_GROUP("spi_quad", spi_quad),
	PINCTRL_PIN_GROUP("spi_cs1", spi_cs1),
	PINCTRL_PIN_GROUP("pcm_spi", pcm_spi),
	PINCTRL_PIN_GROUP("pcm_spi_int", pcm_spi_int),
	PINCTRL_PIN_GROUP("pcm_spi_reset", pcm_spi_reset),
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
	PINCTRL_PIN_GROUP("port0_led0", port0_led0),
	PINCTRL_PIN_GROUP("port0_led1", port0_led1),
	PINCTRL_PIN_GROUP("port1_led0", port1_led0),
	PINCTRL_PIN_GROUP("port1_led1", port1_led1),
	PINCTRL_PIN_GROUP("port2_led0", port2_led0),
	PINCTRL_PIN_GROUP("port2_led1", port2_led1),
	PINCTRL_PIN_GROUP("port3_led0", port3_led0),
	PINCTRL_PIN_GROUP("port3_led1", port3_led1),
};

static const char *const pon0_groups[] = { "pon0" };
static const char *const tod_pon_groups[] = { "tod_pon" };
static const char *const tod_sw_pon_groups[] = { "tod_sw_pon" };
static const char *const sipo_groups[] = { "sipo" };
static const char *const sipo_rclk_groups[] = { "sipo_rclk" };
static const char *const mdio_groups[] = { "mdio" };
static const char *const uart2_groups[] = { "uart2" };
static const char *const uart2_cts_rts_groups[] = { "uart2_cts_rts" };
static const char *const hsuart3_groups[] = { "hsuart3" };
static const char *const hsuart3_cts_rts_groups[] = { "hsuart3_cts_rts" };
static const char *const uart4_groups[] = { "uart4" };
static const char *const uart5_groups[] = { "uart5" };
static const char *const i2c1_groups[] = { "i2c1" };
static const char *const jtag_udi_groups[] = { "jtag_udi" };
static const char *const jtag_dfd_groups[] = { "jtag_dfd" };
static const char *const pcm1_groups[] = { "pcm1" };
static const char *const pcm2_groups[] = { "pcm2" };
static const char *const spi_quad_groups[] = { "spi_quad" };
static const char *const spi_cs1_groups[] = { "spi_cs1" };
static const char *const pcm_spi_groups[] = { "pcm_spi" };
static const char *const pcm_spi_int_groups[] = { "pcm_spi_int" };
static const char *const pcm_spi_reset_groups[] = { "pcm_spi_reset" };
static const char *const pcm_spi_cs1_groups[] = { "pcm_spi_cs1" };
static const char *const pcm_spi_cs2_p156_groups[] = { "pcm_spi_cs2_p156" };
static const char *const pcm_spi_cs2_p128_groups[] = { "pcm_spi_cs2_p128" };
static const char *const pcm_spi_cs3_groups[] = { "pcm_spi_cs3" };
static const char *const pcm_spi_cs4_groups[] = { "pcm_spi_cs4" };
static const char *const i2s0_groups[] = { "i2s0" };
static const char *const emmc_groups[] = { "emmc" };
static const char *const pnand_groups[] = { "pnand" };
static const char *const gpio47_groups[] = { "gpio47" };
static const char *const gpio48_groups[] = { "gpio48" };
static const char *const gpio49_groups[] = { "gpio49" };
static const char *const port0_led0_groups[] = { "port0_led0" };
static const char *const port0_led1_groups[] = { "port0_led1" };
static const char *const port1_led0_groups[] = { "port1_led0" };
static const char *const port1_led1_groups[] = { "port1_led1" };
static const char *const port2_led0_groups[] = { "port2_led0" };
static const char *const port2_led1_groups[] = { "port2_led1" };
static const char *const port3_led0_groups[] = { "port3_led0" };
static const char *const port3_led1_groups[] = { "port3_led1" };

static const struct airoha_pinctrl_func_reg pon0_func_regmap[] = {
	{ REG_GPIO_PON_MODE, GPIO_PON_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg tod_pon_func_regmap[] = {
	{ REG_GPIO_2ND_I2C_MODE, PON_TOD_1PPS_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg tod_sw_pon_func_regmap[] = {
	{ REG_GPIO_2ND_I2C_MODE, PON_SW_TOD_1PPS_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg sipo_func_regmap[] = {
	{ REG_GPIO_PON_MODE, GPIO_SIPO_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg sipo_rclk_func_regmap[] = {
	{ REG_GPIO_PON_MODE, GPIO_SIPO_MODE_MASK },
	{ REG_GPIO_PON_MODE, SIPO_RCLK_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg mdio_func_regmap[] = {
	{ REG_GPIO_PON_MODE, GPIO_SGMII_MDIO_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg uart2_func_regmap[] = {
	{ REG_GPIO_PON_MODE, GPIO_UART2_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg uart2_cts_rts_func_regmap[] = {
	{ REG_GPIO_PON_MODE, GPIO_UART2_MODE_MASK },
	{ REG_GPIO_PON_MODE, GPIO_UART2_CTS_RTS_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg hsuart3_func_regmap[] = {
	{ REG_GPIO_PON_MODE, GPIO_HSUART3_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg hsuart3_cts_rts_func_regmap[] = {
	{ REG_GPIO_PON_MODE, GPIO_HSUART3_MODE_MASK },
	{ REG_GPIO_PON_MODE, GPIO_HSUART3_CTS_RTS_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg uart4_func_regmap[] = {
	{ REG_GPIO_PON_MODE, GPIO_UART4_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg uart5_func_regmap[] = {
	{ REG_GPIO_PON_MODE, GPIO_UART5_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg i2c1_func_regmap[] = {
	{ REG_GPIO_2ND_I2C_MODE, GPIO_2ND_I2C_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg jtag_udi_func_regmap[] = {
	{ REG_NPU_UART_EN, JTAG_UDI_EN_MASK },
};
static const struct airoha_pinctrl_func_reg jtag_dfd_func_regmap[] = {
	{ REG_NPU_UART_EN, JTAG_DFD_EN_MASK },
};
static const struct airoha_pinctrl_func_reg pcm1_func_regmap[] = {
	{ REG_GPIO_SPI_CS1_MODE, GPIO_PCM1_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg pcm2_func_regmap[] = {
	{ REG_GPIO_SPI_CS1_MODE, GPIO_PCM2_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg spi_quad_func_regmap[] = {
	{ REG_GPIO_SPI_CS1_MODE, GPIO_SPI_QUAD_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg spi_cs4_func_regmap[] = {
	{ REG_GPIO_SPI_CS1_MODE, GPIO_SPI_CS4_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg spi_cs3_func_regmap[] = {
	{ REG_GPIO_SPI_CS1_MODE, GPIO_SPI_CS3_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg spi_cs2_func_regmap[] = {
	{ REG_GPIO_SPI_CS1_MODE, GPIO_SPI_CS2_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg spi_cs1_func_regmap[] = {
	{ REG_GPIO_SPI_CS1_MODE, GPIO_SPI_CS1_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg pcm_spi_func_regmap[] = {
	{ REG_GPIO_SPI_CS1_MODE, GPIO_PCM_SPI_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg pcm_spi_int_func_regmap[] = {
	{ REG_GPIO_SPI_CS1_MODE, GPIO_PCM_INT_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg pcm_spi_reset_func_regmap[] = {
	{ REG_GPIO_SPI_CS1_MODE, GPIO_PCM_RESET_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg pcm_spi_cs1_func_regmap[] = {
	{ REG_GPIO_SPI_CS1_MODE, GPIO_PCM_SPI_CS1_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg pcm_spi_cs2_p128_func_regmap[] = {
	{ REG_GPIO_SPI_CS1_MODE, GPIO_PCM_SPI_CS2_MODE_P128_MASK },
};
static const struct airoha_pinctrl_func_reg pcm_spi_cs2_p156_func_regmap[] = {
	{ REG_GPIO_SPI_CS1_MODE, GPIO_PCM_SPI_CS2_MODE_P156_MASK },
};
static const struct airoha_pinctrl_func_reg pcm_spi_cs3_func_regmap[] = {
	{ REG_GPIO_SPI_CS1_MODE, GPIO_PCM_SPI_CS3_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg pcm_spi_cs4_func_regmap[] = {
	{ REG_GPIO_SPI_CS1_MODE, GPIO_PCM_SPI_CS4_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg i2s0_func_regmap[] = {
	{ REG_GPIO_2ND_I2C_MODE, GPIO_I2S_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg emmc_func_regmap[] = {
	{ REG_GPIO_PON_MODE, GPIO_EMMC_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg pnand_func_regmap[] = {
	{ REG_GPIO_PON_MODE, GPIO_PARALLEL_NAND_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg gpio47_func_regmap[] = {
	{ REG_GPIO_PON_MODE, GPIO_PCIE_RESET0_MASK },
};
static const struct airoha_pinctrl_func_reg gpio48_func_regmap[] = {
	{ REG_GPIO_PON_MODE, GPIO_PCIE_RESET1_MASK },
};
static const struct airoha_pinctrl_func_reg gpio49_func_regmap[] = {
	{ REG_GPIO_PON_MODE, GPIO_PCIE_RESET2_MASK },
};
static const struct airoha_pinctrl_func_reg port0_led0_func_regmap[] = {
	{ REG_GPIO_2ND_I2C_MODE, GPIO_PORT0_LED0_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg port0_led1_func_regmap[] = {
	{ REG_GPIO_2ND_I2C_MODE, GPIO_PORT0_LED1_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg port1_led0_func_regmap[] = {
	{ REG_GPIO_2ND_I2C_MODE, GPIO_PORT1_LED0_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg port1_led1_func_regmap[] = {
	{ REG_GPIO_2ND_I2C_MODE, GPIO_PORT1_LED1_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg port2_led0_func_regmap[] = {
	{ REG_GPIO_2ND_I2C_MODE, GPIO_PORT2_LED0_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg port2_led1_func_regmap[] = {
	{ REG_GPIO_2ND_I2C_MODE, GPIO_PORT2_LED1_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg port3_led0_func_regmap[] = {
	{ REG_GPIO_2ND_I2C_MODE, GPIO_PORT3_LED0_MODE_MASK },
};
static const struct airoha_pinctrl_func_reg port3_led1_func_regmap[] = {
	{ REG_GPIO_2ND_I2C_MODE, GPIO_PORT3_LED1_MODE_MASK },
};

static const struct airoha_pinctrl_func airoha_pinctrl_funcs[] = {
	PINCTRL_FUNC_DESC("pon0", pon0),
	PINCTRL_FUNC_DESC("tod_pon", tod_pon),
	PINCTRL_FUNC_DESC("tod_sw_pon", tod_sw_pon),
	PINCTRL_FUNC_DESC("sipo", sipo),
	PINCTRL_FUNC_DESC("sipo_rclk", sipo_rclk),
	PINCTRL_FUNC_DESC("mdio", mdio),
	PINCTRL_FUNC_DESC("uart2", uart2),
	PINCTRL_FUNC_DESC("uart2_cts_rts", uart2_cts_rts),
	PINCTRL_FUNC_DESC("hsuart3", hsuart3),
	PINCTRL_FUNC_DESC("hsuart3_cts_rts", hsuart3_cts_rts),
	PINCTRL_FUNC_DESC("uart4", uart4),
	PINCTRL_FUNC_DESC("uart5", uart5),
	PINCTRL_FUNC_DESC("i2c1", i2c1),
	PINCTRL_FUNC_DESC("jtag_udi", jtag_udi),
	PINCTRL_FUNC_DESC("jtag_dfd", jtag_dfd),
	PINCTRL_FUNC_DESC("pcm1", pcm1),
	PINCTRL_FUNC_DESC("pcm2", pcm2),
	PINCTRL_FUNC_DESC("spi_quad", spi_quad),
	PINCTRL_FUNC_DESC("spi_cs1", spi_cs1),
	PINCTRL_FUNC_DESC("pcm_spi", pcm_spi),
	PINCTRL_FUNC_DESC("pcm_spi_int", pcm_spi_int),
	PINCTRL_FUNC_DESC("pcm_spi_reset", pcm_spi_reset),
	PINCTRL_FUNC_DESC("pcm_spi_cs1", pcm_spi_cs1),
	PINCTRL_FUNC_DESC("pcm_spi_cs2_p128", pcm_spi_cs2_p128),
	PINCTRL_FUNC_DESC("pcm_spi_cs2_p156", pcm_spi_cs2_p156),
	PINCTRL_FUNC_DESC("pcm_spi_cs3", pcm_spi_cs3),
	PINCTRL_FUNC_DESC("pcm_spi_cs4", pcm_spi_cs4),
	PINCTRL_FUNC_DESC("i2s0", i2s0),
	PINCTRL_FUNC_DESC("emmc", emmc),
	PINCTRL_FUNC_DESC("pnand", pnand),
	PINCTRL_FUNC_DESC("gpio47", gpio47),
	PINCTRL_FUNC_DESC("gpio48", gpio48),
	PINCTRL_FUNC_DESC("gpio49", gpio49),
	PINCTRL_FUNC_DESC("port0_led0", port0_led0),
	PINCTRL_FUNC_DESC("port0_led1", port0_led1),
	PINCTRL_FUNC_DESC("port1_led0", port1_led0),
	PINCTRL_FUNC_DESC("port1_led1", port1_led1),
	PINCTRL_FUNC_DESC("port2_led0", port2_led0),
	PINCTRL_FUNC_DESC("port2_led1", port2_led1),
	PINCTRL_FUNC_DESC("port3_led0", port3_led0),
	PINCTRL_FUNC_DESC("port3_led1", port3_led1),
};

static u32 airoha_pinctrl_rmw(struct airoha_pinctrl *pinctrl,
			      void __iomem *base, u32 offset,
			      u32 mask, u32 val)
{
	mutex_lock(&pinctrl->mutex);

	val |= (readl(base + offset) & ~mask);
	writel(val, base + offset);

	mutex_unlock(&pinctrl->mutex);

	return val;
}

#define airoha_pinctrl_mux_set(pinctrl, offset, val)			\
	airoha_pinctrl_rmw((pinctrl), ((pinctrl)->mux_regs), (offset),	\
			   0, (val));

static int airoha_pinmux_set_mux(struct pinctrl_dev *pctrl_dev,
				 unsigned int selector,
				 unsigned int group)
{
	struct airoha_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrl_dev);
	const struct airoha_pinctrl_func *func;
	struct function_desc *desc;
	int i;

	desc = pinmux_generic_get_function(pctrl_dev, selector);
	if (!desc)
		return -EINVAL;

	dev_dbg(pctrl_dev->dev, "enable function %s\n", desc->name);

	func = desc->data;
	for (i = 0; i < func->regmap_size; i++) {
		const struct airoha_pinctrl_func_reg *reg = &func->regmap[i];

		airoha_pinctrl_mux_set(pinctrl, reg->offset, reg->mask);
	}

	return 0;
}

static int airoha_pinconf_get(struct pinctrl_dev *pctrl_dev,
			      unsigned int pin, unsigned long *config)
{
	return 0;
}

static int airoha_pinconf_set(struct pinctrl_dev *pctrl_dev,
			      unsigned int pin, unsigned long *configs,
			      unsigned int num_configs)
{
	return 0;
}

static int airoha_pinconf_group_set(struct pinctrl_dev *pctrl_dev,
				    unsigned int group, unsigned long *configs,
				    unsigned int num_configs)
{
	return 0;
}

static const struct pinconf_ops airoha_confops = {
	.is_generic = true,
	.pin_config_get = airoha_pinconf_get,
	.pin_config_set = airoha_pinconf_set,
	.pin_config_group_set = airoha_pinconf_group_set,
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
	.set_mux = airoha_pinmux_set_mux,
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

static int airoha_pinctrl_probe(struct platform_device *pdev)
{
	struct airoha_pinctrl *pinctrl;
	int err, i;

	pinctrl = devm_kzalloc(&pdev->dev, sizeof(*pinctrl), GFP_KERNEL);
	if (!pinctrl)
		return -ENOMEM;

	mutex_init(&pinctrl->mutex);

	pinctrl->mux_regs = devm_platform_ioremap_resource_byname(pdev, "mux");
	if (IS_ERR(pinctrl->mux_regs))
		return dev_err_probe(&pdev->dev, PTR_ERR(pinctrl->mux_regs),
				     "failed to iomap mux regs\n");

	pinctrl->conf_regs = devm_platform_ioremap_resource_byname(pdev,
								   "conf");
	if (IS_ERR(pinctrl->conf_regs))
		return dev_err_probe(&pdev->dev, PTR_ERR(pinctrl->conf_regs),
				     "failed to iomap conf regs\n");

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

	return pinctrl_enable(pinctrl->ctrl);
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
