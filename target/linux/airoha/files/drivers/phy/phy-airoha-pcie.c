// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 Lorenzo Bianconi <lorenzo@kernel.org>
 */

#include <linux/delay.h>
#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

/* ANA2L */
#define REG_PCIE_CSR_ANA2L_PXP_CMN			0x0000
#define CSR_ANA2L_PXP_CMN_LANE_EN			BIT(0)
#define CSR_ANA2L_PXP_CMN_TRIM_MASK			GENMASK(28, 24)

#define REG_PCIE_CSR_ANA2L_PXP_JCPLL_RST_DLY		0x1c
#define CSR_ANA2L_PXP_JCPLL_RST				BIT(8)

#define REG_PCIE_CSR_ANA2L_JCPLL_SSC_DELTA1		0x3c

#define REG_PCIE_CSR_ANA2L_JCPLL_SSC_PERIOD		0x40
#define CSR_ANA2L_PXP_JCPLL_SSC_PERIOD			GENMASK(15, 0)

#define REG_PCIE_CSR_ANA2L_PXP_JCPLL_TCL_VTP_EN		0x004c
#define CSR_ANA2L_PXP_JCPLL_SPARE_LOW			GENMASK(31, 24)

#define REG_PCIE_CSR_ANA2L_PXP_TXPLL_POSTDIV_D256	0x00a0
#define CSR_ANA2L_PXP_CLKTX0_AMP			GENMASK(10, 8)
#define CSR_ANA2L_PXP_CLKTX0_OFFSET			GENMASK(17, 16)
#define CSR_ANA2L_PXP_CLKTX0_SR				GENMASK(25, 24)

#define REG_PCIE_CSR_ANA2L_PXP_CLKTX0_FORCE_OUT1	0x00a4
#define CSR_ANA2L_PXP_CLKTX0_HZ				BIT(8)
#define CSR_ANA2L_PXP_CLKTX0_IMP_SEL			GENMASK(20, 16)
#define CSR_ANA2L_PXP_CLKTX1_AMP			GENMASK(26, 24)

#define REG_PCIE_CSR_ANA2L_PXP_CLKTX1_OFFSET		0x00a8
#define CSR_ANA2L_PXP_CLKTX1_OFFSET			GENMASK(1, 0)
#define CSR_ANA2L_PXP_CLKTX1_SR				GENMASK(9, 8)
#define CSR_ANA2L_PXP_CLKTX1_HZ				BIT(24)

#define REG_PCIE_CSR_ANA2L_PXP_CLKTX1_IMP_SEL		0x00ac
#define CSR_ANA2L_PXP_CLKTX1_IMP_SEL			GENMASK(4, 0)

#define REG_PCIE_CSR_ANA2L_PXP_PLL_CMN_RESERVE0		0x00b0
#define CSR_ANA2L_PXP_PLL_RESERVE_MASK			GENMASK(15, 0)

#define REG_PCIE_CSR_ANA2L_PXP_RX0_PHYCK_DIV		0x100
#define CSR_ANA2L_PXP_RX0_PHYCK_SEL			GENMASK(9, 8)

#define REG_PCIE_CSR_ANA2L_PXP_CDR0_PD_PICAL_CKD8_INV	0x104
#define CSR_ANA2L_PXP_CDR0_PD_EDGE_DISABLE		BIT(8)

#define REG_PCIE_CSR_ANA2L_PXP_CDR0_PR_MONPI		0x134
#define CSR_ANA2L_PXP_CDR0_PR_XFICK_EN			BIT(8)

#define REG_PCIE_CSR_ANA2L_PXP_RX1_PHYCK_DIV		0x1b8
#define CSR_ANA2L_PXP_RX1_PHYCK_SEL			GENMASK(9, 8)

#define REG_PCIE_CSR_ANA2L_PXP_CDR1_PD_PICAL_CKD8_INV	0x1bc
#define CSR_ANA2L_PXP_CDR1_PD_EDGE_DISABLE		BIT(8)

#define REG_PCIE_CSR_ANA2L_PXP_CDR1_PR_MONPI		0x1ec
#define CSR_ANA2L_PXP_CDR1_PR_XFICK_EN			BIT(8)

/* PMA */
#define REG_PCIE_PMA_TX_RESET				0x0260
#define PCIE_TX_TOP_RST					BIT(0)
#define PCIE_TX_CAL_RST					BIT(8)

#define REG_PCIE_PMA_SW_RESET				0x0460
#define PCIE_SW_RX_RST					BIT(1)
#define PCIE_SW_REF_RST					BIT(5)
#define PCIE_SW_XFI_RXPCS_RST				BIT(8)

#define REG_PCIE_PMA_FORCE_PXP_JCPLL_CKOUT		0x0828
#define PCIE_FORCE_DA_PXP_JCPLL_EN			BIT(16)
#define PCIE_FORCE_SEL_DA_PXP_JCPLL_EN			BIT(24)

#define REG_PCIE_PMA_DIG_RESERVE_14			0x08c0
#define PCIE_FLL_LOAD_EN				BIT(16)

#define REG_PCIE_PMA_DIG_RESERVE_17			0x08e0
#define REG_PCIE_PMA_DIG_RESERVE_21			0x08f0
#define REG_PCIE_PMA_DIG_RESERVE_22			0x08f4

/**
 * struct airoha_pcie_phy - PCIe phy driver main structure
 * @dev: pointer to device
 * @phy: pointer to generic phy
 * @csr_ana2l: Analogic lane IO mapped register base address
 * @pma0: IO mapped register base address of PMA0-PCIe
 * @pma1: IO mapped register base address of PMA1-PCIe
 * @data: pointer to SoC dependent data
 */
struct airoha_pcie_phy {
	struct device *dev;
	struct phy *phy;
	void __iomem *csr_ana2l;
	void __iomem *pma0;
	void __iomem *pma1;
};

static void airoha_phy_clear_bits(void __iomem *reg, u32 mask)
{
	u32 val = readl(reg);

	val &= ~mask;
	writel(val, reg);
}

static void airoha_phy_set_bits(void __iomem *reg, u32 mask)
{
	u32 val = readl(reg);

	val |= mask;
	writel(val, reg);
}

static void airoha_phy_update_bits(void __iomem *reg, u32 mask, u32 val)
{
	u32 tmp = readl(reg);

	tmp &= ~mask;
	tmp |= val & mask;
	writel(tmp, reg);
}

#define airoha_phy_update_field(reg, mask, val)					\
({										\
	BUILD_BUG_ON_MSG(!__builtin_constant_p(mask), "mask is not constant");	\
	airoha_phy_update_bits(reg, mask, FIELD_PREP(mask, val)); 		\
})

static void airoha_pcie_init_set_default(struct airoha_pcie_phy *pcie_phy)
{
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_CMN,
				CSR_ANA2L_PXP_CMN_TRIM_MASK, 0x10);
	writel(0xcccbcccb, pcie_phy->pma0 + REG_PCIE_PMA_DIG_RESERVE_21);
	writel(0xcccb, pcie_phy->pma0 + REG_PCIE_PMA_DIG_RESERVE_22);
	writel(0xcccbcccb, pcie_phy->pma1 + REG_PCIE_PMA_DIG_RESERVE_21);
	writel(0xcccb, pcie_phy->pma1 + REG_PCIE_PMA_DIG_RESERVE_22);
	airoha_phy_set_bits(pcie_phy->csr_ana2l + REG_PCIE_CSR_ANA2L_PXP_CMN,
			    CSR_ANA2L_PXP_CMN_LANE_EN);
}

static void airoha_pcie_init_clk_out(struct airoha_pcie_phy *pcie_phy)
{
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_TXPLL_POSTDIV_D256,
				CSR_ANA2L_PXP_CLKTX0_AMP, 0x5);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_CLKTX0_FORCE_OUT1,
				CSR_ANA2L_PXP_CLKTX1_AMP, 0x5);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_TXPLL_POSTDIV_D256,
				CSR_ANA2L_PXP_CLKTX0_OFFSET, 0x2);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_CLKTX1_OFFSET,
				CSR_ANA2L_PXP_CLKTX1_OFFSET, 0x2);
	airoha_phy_clear_bits(pcie_phy->csr_ana2l +
			      REG_PCIE_CSR_ANA2L_PXP_CLKTX0_FORCE_OUT1,
			      CSR_ANA2L_PXP_CLKTX0_HZ);
	airoha_phy_clear_bits(pcie_phy->csr_ana2l +
			      REG_PCIE_CSR_ANA2L_PXP_CLKTX1_OFFSET,
			      CSR_ANA2L_PXP_CLKTX1_HZ);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_CLKTX0_FORCE_OUT1,
				CSR_ANA2L_PXP_CLKTX0_IMP_SEL, 0x12);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_CLKTX1_IMP_SEL,
				CSR_ANA2L_PXP_CLKTX1_IMP_SEL, 0x12);
	airoha_phy_clear_bits(pcie_phy->csr_ana2l +
			      REG_PCIE_CSR_ANA2L_PXP_TXPLL_POSTDIV_D256,
			      CSR_ANA2L_PXP_CLKTX0_SR);
	airoha_phy_clear_bits(pcie_phy->csr_ana2l +
			      REG_PCIE_CSR_ANA2L_PXP_CLKTX1_OFFSET,
			      CSR_ANA2L_PXP_CLKTX1_SR);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_PLL_CMN_RESERVE0,
				CSR_ANA2L_PXP_PLL_RESERVE_MASK, 0xdd);
}

static void airoha_pcie_init_csr_ana2l(struct airoha_pcie_phy *pcie_phy)
{
	airoha_phy_set_bits(pcie_phy->pma0 + REG_PCIE_PMA_SW_RESET,
			    PCIE_SW_XFI_RXPCS_RST | PCIE_SW_REF_RST |
			    PCIE_SW_RX_RST);
	airoha_phy_set_bits(pcie_phy->pma1 + REG_PCIE_PMA_SW_RESET,
			    PCIE_SW_XFI_RXPCS_RST | PCIE_SW_REF_RST |
			    PCIE_SW_RX_RST);
	airoha_phy_set_bits(pcie_phy->pma0 + REG_PCIE_PMA_TX_RESET,
			    PCIE_TX_TOP_RST | REG_PCIE_PMA_TX_RESET);
	airoha_phy_set_bits(pcie_phy->pma1 + REG_PCIE_PMA_TX_RESET,
			    PCIE_TX_TOP_RST | REG_PCIE_PMA_TX_RESET);
}

static void airoha_pcie_phy_init_rx(struct airoha_pcie_phy *pcie_phy)
{
	writel(0x2a00090b, pcie_phy->pma0 + REG_PCIE_PMA_DIG_RESERVE_17);
	writel(0x2a00090b, pcie_phy->pma1 + REG_PCIE_PMA_DIG_RESERVE_17);
	airoha_phy_set_bits(pcie_phy->csr_ana2l +
			    REG_PCIE_CSR_ANA2L_PXP_CDR0_PR_MONPI,
			    CSR_ANA2L_PXP_CDR0_PR_XFICK_EN);
	airoha_phy_set_bits(pcie_phy->csr_ana2l +
			    REG_PCIE_CSR_ANA2L_PXP_CDR1_PR_MONPI,
			    CSR_ANA2L_PXP_CDR1_PR_XFICK_EN);
	airoha_phy_clear_bits(pcie_phy->csr_ana2l +
			      REG_PCIE_CSR_ANA2L_PXP_CDR0_PD_PICAL_CKD8_INV,
			      CSR_ANA2L_PXP_CDR0_PD_EDGE_DISABLE);
	airoha_phy_clear_bits(pcie_phy->csr_ana2l +
			      REG_PCIE_CSR_ANA2L_PXP_CDR1_PD_PICAL_CKD8_INV,
			      CSR_ANA2L_PXP_CDR1_PD_EDGE_DISABLE);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_RX0_PHYCK_DIV,
				CSR_ANA2L_PXP_RX0_PHYCK_SEL, 0x1);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_RX1_PHYCK_DIV,
				CSR_ANA2L_PXP_RX1_PHYCK_SEL, 0x1);
}

static void airoha_pcie_phy_init_jcpll(struct airoha_pcie_phy *pcie_phy)
{
	airoha_phy_set_bits(pcie_phy->pma0 + REG_PCIE_PMA_FORCE_PXP_JCPLL_CKOUT,
			    PCIE_FORCE_SEL_DA_PXP_JCPLL_EN);
	airoha_phy_clear_bits(pcie_phy->pma0 + REG_PCIE_PMA_FORCE_PXP_JCPLL_CKOUT,
			      PCIE_FORCE_DA_PXP_JCPLL_EN);
	airoha_phy_set_bits(pcie_phy->pma1 + REG_PCIE_PMA_FORCE_PXP_JCPLL_CKOUT,
			    PCIE_FORCE_SEL_DA_PXP_JCPLL_EN);
	airoha_phy_clear_bits(pcie_phy->pma1 + REG_PCIE_PMA_FORCE_PXP_JCPLL_CKOUT,
			      PCIE_FORCE_DA_PXP_JCPLL_EN);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_JCPLL_TCL_VTP_EN,
				CSR_ANA2L_PXP_JCPLL_SPARE_LOW, 0x20);
	airoha_phy_set_bits(pcie_phy->csr_ana2l +
			    REG_PCIE_CSR_ANA2L_PXP_JCPLL_RST_DLY,
			    CSR_ANA2L_PXP_JCPLL_RST);
	writel(0x0, pcie_phy->csr_ana2l + REG_PCIE_CSR_ANA2L_JCPLL_SSC_DELTA1);
	airoha_phy_clear_bits(pcie_phy->csr_ana2l +
			      REG_PCIE_CSR_ANA2L_JCPLL_SSC_PERIOD,
			      CSR_ANA2L_PXP_JCPLL_SSC_PERIOD);
}

static void airoha_pcie_phy_txpll(struct airoha_pcie_phy *pcie_phy)
{
}

/**
 * airoha_pcie_phy_init() - Initialize the phy
 * @phy: the phy to be initialized
 *
 * Initialize the phy registers.
 * The hardware settings will be reset during suspend, it should be
 * reinitialized when the consumer calls phy_init() again on resume.
 */
static int airoha_pcie_phy_init(struct phy *phy)
{
	struct airoha_pcie_phy *pcie_phy = phy_get_drvdata(phy);

	printk("[%s-%d]\n", __func__, __LINE__);
	/* enable load FLL-K flow */
	airoha_phy_set_bits(pcie_phy->pma0 + REG_PCIE_PMA_DIG_RESERVE_14,
			    PCIE_FLL_LOAD_EN);
	airoha_phy_set_bits(pcie_phy->pma1 + REG_PCIE_PMA_DIG_RESERVE_14,
			    PCIE_FLL_LOAD_EN);

	airoha_pcie_init_set_default(pcie_phy);
	airoha_pcie_init_clk_out(pcie_phy);
	airoha_pcie_init_csr_ana2l(pcie_phy);

	usleep_range(100, 200);

	airoha_pcie_phy_init_rx(pcie_phy);
	/* phase 1, no ssc for K TXPLL */
	airoha_pcie_phy_init_jcpll(pcie_phy);

	usleep_range(500, 600);

	airoha_pcie_phy_txpll(pcie_phy);
	printk("[%s-%d]\n", __func__, __LINE__);

	return 0;
}

static const struct phy_ops airoha_pcie_phy_ops = {
	.init = airoha_pcie_phy_init,
	.owner = THIS_MODULE,
};

static int airoha_pcie_phy_probe(struct platform_device *pdev)
{
	struct airoha_pcie_phy *pcie_phy;
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;

	pcie_phy = devm_kzalloc(dev, sizeof(*pcie_phy), GFP_KERNEL);
	if (!pcie_phy)
		return -ENOMEM;

	pcie_phy->csr_ana2l = devm_platform_ioremap_resource_byname(pdev, "ana2l");
	if (IS_ERR(pcie_phy->csr_ana2l))
		return dev_err_probe(dev, PTR_ERR(pcie_phy->csr_ana2l),
				     "Failed to map phy-ana2l base\n");

	pcie_phy->pma0 = devm_platform_ioremap_resource_byname(pdev, "pma0");
	if (IS_ERR(pcie_phy->pma0))
		return dev_err_probe(dev, PTR_ERR(pcie_phy->pma0),
				     "Failed to map phy-pma0 base\n");

	pcie_phy->pma1 = devm_platform_ioremap_resource_byname(pdev, "pma1");
	if (IS_ERR(pcie_phy->pma1))
		return dev_err_probe(dev, PTR_ERR(pcie_phy->pma1),
				     "Failed to map phy-pma1 base\n");

	pcie_phy->phy = devm_phy_create(dev, dev->of_node, &airoha_pcie_phy_ops);
	if (IS_ERR(pcie_phy->phy))
		return dev_err_probe(dev, PTR_ERR(pcie_phy->phy),
				     "Failed to create PCIe phy\n");

	pcie_phy->dev = dev;
	phy_set_drvdata(pcie_phy->phy, pcie_phy);

	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(provider))
		return dev_err_probe(dev, PTR_ERR(provider),
				     "PCIe phy probe failed\n");

	return 0;
}

static const struct of_device_id airoha_pcie_phy_of_match[] = {
	{ .compatible = "airoha,en7581-pcie-phy" },
	{ },
};
MODULE_DEVICE_TABLE(of, airoha_pcie_phy_of_match);

static struct platform_driver airoha_pcie_phy_driver = {
	.probe	= airoha_pcie_phy_probe,
	.driver	= {
		.name = "airoha-pcie-phy",
		.of_match_table = airoha_pcie_phy_of_match,
	},
};
module_platform_driver(airoha_pcie_phy_driver);

MODULE_DESCRIPTION("Airoha PCIe PHY driver");
MODULE_AUTHOR("Lorenzo Bianconi <lorenzo@kernel.org>");
MODULE_LICENSE("GPL");
