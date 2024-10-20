// SPDX-License-Identifier: GPL-2.0+

#include <linux/phy.h>
#include <linux/module.h>
#include <linux/bitfield.h>

#define EN8855_PHY_PAGE_CTRL			0x1f
#define   EN8855_PHY_NORMAL_PAGE		0x0
#define   EN8855_PHY_EXT_PAGE			0x1

#define EN8855_PHY_EXT_REG_14			0x14
#define   EN8855_PHY_EN_DOWN_SHFIT		BIT(4)

/* R50 Calibration regs in MDIO_MMD_VEND1 */
#define EN8855_PHY_R500HM_RSEL_TX_AB		0x174
#define EN8855_PHY_R50OHM_RSEL_TX_A_EN		BIT(15)
#define EN8855_PHY_R50OHM_RSEL_TX_A		GENMASK(14, 8)
#define EN8855_PHY_R50OHM_RSEL_TX_B_EN		BIT(7)
#define EN8855_PHY_R50OHM_RSEL_TX_B		GENMASK(6, 0)
#define EN8855_PHY_R500HM_RSEL_TX_CD		0x175
#define EN8855_PHY_R50OHM_RSEL_TX_C_EN		BIT(15)
#define EN8855_PHY_R50OHM_RSEL_TX_C		GENMASK(14, 8)
#define EN8855_PHY_R50OHM_RSEL_TX_D_EN		BIT(7)
#define EN8855_PHY_R50OHM_RSEL_TX_D		GENMASK(6, 0)

/* PHY TX PAIR DELAY SELECT Register */
#define PHY_TX_PAIR_DLY_SEL_GBE			0x013
/* PHY ADC Register */
#define PHY_RXADC_CTRL				0x0d8
#define PHY_RXADC_REV_0				0x0d9
#define PHY_RXADC_REV_1				0x0da

#define EN8855_PHY_ID			0xc0ff0410

static int en8855_config_init(struct phy_device *phydev)
{
	u8 calibration_data[4];

	memcpy(calibration_data, &phydev->dev_flags, sizeof(u32));

	/* Enable HW auto downshift */
	phy_write(phydev, EN8855_PHY_PAGE_CTRL, EN8855_PHY_EXT_PAGE);
	phy_set_bits(phydev, EN8855_PHY_EXT_REG_14,
			     EN8855_PHY_EN_DOWN_SHFIT);
	phy_write(phydev, EN8855_PHY_PAGE_CTRL, EN8855_PHY_NORMAL_PAGE);

	/* Enable Asymmetric Pause Capability */
	phy_set_bits(phydev, MII_ADVERTISE, ADVERTISE_PAUSE_ASYM);

	/* Disable EEE */
	phy_write_mmd(phydev, MDIO_MMD_AN, MDIO_AN_EEE_ADV, 0);

	/* Apply calibration values */
	phy_modify_mmd(phydev, MDIO_MMD_VEND1, EN8855_PHY_R500HM_RSEL_TX_AB,
		       EN8855_PHY_R50OHM_RSEL_TX_A | EN8855_PHY_R50OHM_RSEL_TX_B,
		       FIELD_PREP(EN8855_PHY_R50OHM_RSEL_TX_A, calibration_data[0]) |
		       FIELD_PREP(EN8855_PHY_R50OHM_RSEL_TX_B, calibration_data[1]));
	phy_modify_mmd(phydev, MDIO_MMD_VEND1, EN8855_PHY_R500HM_RSEL_TX_CD,
		       EN8855_PHY_R50OHM_RSEL_TX_C | EN8855_PHY_R50OHM_RSEL_TX_D,
		       FIELD_PREP(EN8855_PHY_R50OHM_RSEL_TX_C, calibration_data[2]) |
		       FIELD_PREP(EN8855_PHY_R50OHM_RSEL_TX_D, calibration_data[3]));

	/* Apply values to decude signal noise */
	phy_write_mmd(phydev, MDIO_MMD_VEND1, PHY_TX_PAIR_DLY_SEL_GBE, 0x4040);
	phy_write_mmd(phydev, MDIO_MMD_VEND1, PHY_RXADC_CTRL, 0x1010);
	phy_write_mmd(phydev, MDIO_MMD_VEND1, PHY_RXADC_REV_0, 0x100);
	phy_write_mmd(phydev, MDIO_MMD_VEND1, PHY_RXADC_REV_1, 0x100);

	return 0;
}

static int en8855_get_downshift(struct phy_device *phydev, u8 *data)
{
	int val;

	phy_write(phydev, EN8855_PHY_PAGE_CTRL, EN8855_PHY_EXT_PAGE);

	val = phy_read(phydev, EN8855_PHY_EXT_REG_14);
	*data = val & EN8855_PHY_EXT_REG_14 ? DOWNSHIFT_DEV_DEFAULT_COUNT :
					      DOWNSHIFT_DEV_DISABLE;

	phy_write(phydev, EN8855_PHY_PAGE_CTRL, EN8855_PHY_NORMAL_PAGE);

	return 0;
}

static int en8855_set_downshift(struct phy_device *phydev, u8 cnt)
{
	phy_write(phydev, EN8855_PHY_PAGE_CTRL, EN8855_PHY_EXT_PAGE);

	if (cnt != DOWNSHIFT_DEV_DISABLE)
		phy_set_bits(phydev, EN8855_PHY_EXT_REG_14,
			     EN8855_PHY_EN_DOWN_SHFIT);
	else
		phy_clear_bits(phydev, EN8855_PHY_EXT_REG_14,
			       EN8855_PHY_EN_DOWN_SHFIT);

	phy_write(phydev, EN8855_PHY_PAGE_CTRL, EN8855_PHY_NORMAL_PAGE);

	return 0;
}

static int en8855_get_tunable(struct phy_device *phydev,
			      struct ethtool_tunable *tuna, void *data)
{
	switch (tuna->id) {
	case ETHTOOL_PHY_DOWNSHIFT:
		return en8855_get_downshift(phydev, data);
	default:
		return -EOPNOTSUPP;
	}
}

static int en8855_set_tunable(struct phy_device *phydev,
			      struct ethtool_tunable *tuna, const void *data)
{
	switch (tuna->id) {
	case ETHTOOL_PHY_DOWNSHIFT:
		return en8855_set_downshift(phydev, *(const u8 *)data);
	default:
		return -EOPNOTSUPP;
	}
}

static struct phy_driver en8855_driver[] = {
{
	PHY_ID_MATCH_EXACT(EN8855_PHY_ID),
	.name			= "Airoha EN8855 internal PHY",
	/* PHY_GBIT_FEATURES */
	.flags			= PHY_IS_INTERNAL,
	.config_init		= en8855_config_init,
	.soft_reset		= genphy_soft_reset,
	.get_tunable		= en8855_get_tunable,
	.set_tunable		= en8855_set_tunable,
	.suspend		= genphy_suspend,
	.resume			= genphy_resume,
}, };

module_phy_driver(en8855_driver);

static struct mdio_device_id __maybe_unused en8855_tbl[] = {
	{ PHY_ID_MATCH_EXACT(EN8855_PHY_ID) },
	{ }
};

MODULE_DEVICE_TABLE(mdio, en8855_tbl);

MODULE_DESCRIPTION("Airoha EN8855 PHY driver");
MODULE_AUTHOR("Christian Marangi <ansuelsmth@gmail.com>");
MODULE_LICENSE("GPL");