// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 AIROHA Inc
 * Author: Lorenzo Bianconi <lorenzo@kernel.org>
 */
#include <linux/etherdevice.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/phylink.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define AIROHA_RX_ETH_HLEN	(ETH_HLEN + ETH_FCS_LEN)
#define AIROHA_MAX_MTU		(2000 - AIROHA_RX_ETH_HLEN)
#define AIROHA_MAX_DEVS		2
#define AIROHA_HW_FEATURES	(NETIF_F_IP_CSUM | NETIF_F_RXCSUM |	\
				 NETIF_F_HW_VLAN_CTAG_TX | 		\
				 NETIF_F_SG | NETIF_F_TSO |		\
				 NETIF_F_TSO6 | NETIF_F_IPV6_CSUM)

/* FE */
#define CDMA1_BASE			0x0400
#define GDMA1_BASE			0x0500
#define GDMA3_BASE			0x1100
#define GDMA4_BASE			0x2400

#define REG_FE_LAN_MAC_H		0x0040
#define REG_FE_LAN_MAC_LMIN		0x0044
#define REG_FE_LAN_MAC_LMAX		0x0048
#define REG_FE_VIP_PORT_EN		0x01f0
#define REG_FE_IFC_PORT_EN		0x01f4

#define REG_CDMA1_VLAN_CTRL		CDMA1_BASE
#define CDMA1_VLAN_MASK			GENMASK(31, 16)

#define REG_GDMA1_FWD_CFG		GDMA1_BASE
#define GDMA1_OCFQ_MASK			GENMASK(3, 0)
#define GDMA1_MCFQ_MASK			GENMASK(7, 4)
#define GDMA1_BCFQ_MASK			GENMASK(11, 8)
#define GDMA1_UCFQ_MASK			GENMASK(15, 12)
#define GDMA1_TCP_CKSUM			BIT(21)

#define REG_GDMA1_LEN_CFG		(GDMA1_BASE + 0x14)
#define GDMA1_SHORT_LEN_MASK		GENMASK(13, 0)
#define GDMA1_LONG_LEN_MASK		GENMASK(29, 16)

#define REG_FE_CPORT_CFG		(GDMA1_BASE + 0x40)
#define FE_CPORT_PAD			BIT(26)

#define REG_GDMA3_FWD_CFG		GDMA3_BASE
#define REG_GDMA4_FWD_CFG		(GDMA4_BASE + 0x100)

#define REG_CDM5_RX_OQ1_DROP_CNT	0x29d4

/* QDMA */
#define REG_DGB_CHNLQVLD_CHNL28_31	0x129c

struct airoha_eth {
	struct device *dev;
	struct regmap *regmap_fe;
	struct regmap *regmap_qdma;


	struct phylink *phylink;
	struct phylink_config phylink_config;
	phy_interface_t interface;
	int speed;
};

static int airoha_eth_init(struct net_device *dev)
{
	struct airoha_eth *eth = netdev_priv(dev);
	const u8 *addr = dev->dev_addr;
	u32 mac_h, mac_lmin, mac_lmax;
	int err;

	mac_h = (addr[0] << 16) | (addr[1] << 8) | addr[2];
	mac_lmin = (addr[3] << 16) | (addr[4] << 8) | addr[5];
	mac_lmax = mac_lmin;

	err = regmap_write(eth->regmap_fe, REG_FE_LAN_MAC_H, mac_h);
	if (err)
		return err;

	err = regmap_write(eth->regmap_fe, REG_FE_LAN_MAC_LMIN, mac_lmin);
	if (err)
		return err;

	return regmap_write(eth->regmap_fe, REG_FE_LAN_MAC_LMAX, mac_lmax);
}

static int airoha_eth_set_port_fwd_cfg(struct airoha_eth *eth, u32 addr,
				       u32 val)
{
	int err;

	err = regmap_update_bits(eth->regmap_fe, addr, GDMA1_OCFQ_MASK, val);
	if (err)
		return err;

	err = regmap_update_bits(eth->regmap_fe, addr, GDMA1_MCFQ_MASK, val);
	if (err)
		return err;

	err = regmap_update_bits(eth->regmap_fe, addr, GDMA1_BCFQ_MASK, val);
	if (err)
		return err;

	return regmap_update_bits(eth->regmap_fe, addr, GDMA1_UCFQ_MASK, val);
}

static int airoha_eth_set_gdma_port(struct airoha_eth *eth, int port,
				    bool enable)
{
	u32 vip_port, cfg_addr, val = enable ? 4 : 0xf;
	int err;

	switch (port) {
	case 0:
		vip_port = BIT(22);
		cfg_addr = REG_GDMA3_FWD_CFG;
		break;
	case 1:
		vip_port = BIT(23);
		cfg_addr = REG_GDMA3_FWD_CFG;
		break;
	case 2:
		vip_port = BIT(25);
		cfg_addr = REG_GDMA4_FWD_CFG;
		break;
	case 4:
		vip_port = BIT(24);
		cfg_addr = REG_GDMA4_FWD_CFG;
		break;
	default:
		return -EINVAL;
	}

	if (enable) {
		err = regmap_set_bits(eth->regmap_fe, REG_FE_VIP_PORT_EN,
				      vip_port);
		if (err)
			return err;

		err = regmap_set_bits(eth->regmap_fe, REG_FE_IFC_PORT_EN,
				      vip_port);
		if (err)
			return err;
	} else {
		err = regmap_clear_bits(eth->regmap_fe, REG_FE_VIP_PORT_EN,
					vip_port);
		if (err)
			return err;

		err = regmap_clear_bits(eth->regmap_fe, REG_FE_IFC_PORT_EN,
					vip_port);
		if (err)
			return err;
	}

	return airoha_eth_set_port_fwd_cfg(eth, cfg_addr, val);
}

static int airoha_eth_set_gdma_ports(struct airoha_eth *eth, bool enable)
{
	const int port_list[] = { 0, 1, 2, 4 };
	int i, err;

	for (i = 0; i < ARRAY_SIZE(port_list); i++) {
		err = airoha_eth_set_gdma_port(eth, port_list[i], enable);
		if (err)
			return err;
	}

	return 0;
}

static int airoha_eth_open(struct net_device *dev)
{
	struct airoha_eth *eth = netdev_priv(dev);

	netif_start_queue(dev);

	return airoha_eth_set_gdma_ports(eth, true);
}

static int airoha_eth_stop(struct net_device *dev)
{
	struct airoha_eth *eth = netdev_priv(dev);

	netif_stop_queue(dev);

	return airoha_eth_set_gdma_ports(eth, false);
}

static netdev_tx_t airoha_eth_start_xmit(struct sk_buff *skb,
					 struct net_device *dev)
{
	return NETDEV_TX_OK;
}

static int airoha_eth_change_mtu(struct net_device *dev, int new_mtu)
{
	dev->mtu = new_mtu;
	return 0;
}

static const struct net_device_ops airoha_eth_netdev_ops = {
	.ndo_init		= airoha_eth_init,
	.ndo_open		= airoha_eth_open,
	.ndo_stop		= airoha_eth_stop,
	.ndo_start_xmit		= airoha_eth_start_xmit,
	.ndo_change_mtu		= airoha_eth_change_mtu,
};

static const struct phylink_mac_ops airoha_eth_phylink_ops = {
	.validate = phylink_generic_validate,
};

static int airoha_eth_init_maccr(struct airoha_eth *eth)
{
	int err;

	err = regmap_set_bits(eth->regmap_fe, REG_GDMA1_FWD_CFG,
			      GDMA1_TCP_CKSUM);
	if (err)
		return err;

	err = airoha_eth_set_port_fwd_cfg(eth, REG_GDMA1_FWD_CFG, 0);
	if (err)
		return err;

	err = regmap_set_bits(eth->regmap_fe, REG_FE_CPORT_CFG, FE_CPORT_PAD);
	if (err)
		return err;

	err = regmap_update_bits(eth->regmap_fe, REG_CDMA1_VLAN_CTRL,
				 CDMA1_VLAN_MASK, 0x8100);
	if (err)
		return err;

	err = regmap_update_bits(eth->regmap_fe, REG_GDMA1_LEN_CFG,
				 GDMA1_SHORT_LEN_MASK, 60);
	if (err)
		return err;

	return regmap_update_bits(eth->regmap_fe, REG_GDMA1_LEN_CFG,
				  GDMA1_LONG_LEN_MASK, 4004);
}

static int airoha_eth_hw_init(struct airoha_eth *eth)
{
	int err;

	err = airoha_eth_init_maccr(eth);
	if (err)
		return err;

	return 0;
}

static const struct regmap_config fe_regmap_config = {
	.name		= "fe",
	.reg_bits	= 32,
	.val_bits	= 32,
	.reg_stride	= 4,
	.max_register	= REG_CDM5_RX_OQ1_DROP_CNT,
};

static const struct regmap_config qdma_regmap_config = {
	.name		= "qdma",
	.reg_bits	= 32,
	.val_bits	= 32,
	.reg_stride	= 4,
	.max_register	= REG_DGB_CHNLQVLD_CHNL28_31,
};

static int airoha_eth_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct net_device *net_dev;
	phy_interface_t phy_mode;
	struct phylink *phylink;
	struct airoha_eth *eth;
	void __iomem *base;
	int err;

	net_dev = devm_alloc_etherdev(dev, sizeof(*eth));
	if (!net_dev) {
		dev_err(dev, "alloc_etherdev failed\n");
		return -ENOMEM;
	}

	eth = netdev_priv(net_dev);
	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	eth->regmap_fe = devm_regmap_init_mmio(dev, base, &fe_regmap_config);
	if (IS_ERR(eth->regmap_fe))
		return dev_err_probe(dev, PTR_ERR(eth->regmap_fe),
				     "failed to init fe regmap\n");

	base = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(base))
		return PTR_ERR(base);

	eth->regmap_qdma = devm_regmap_init_mmio(dev, base,
						 &qdma_regmap_config);
	if (IS_ERR(eth->regmap_qdma))
		return dev_err_probe(dev, PTR_ERR(eth->regmap_qdma),
				     "failed to init qdma regmap\n");

	net_dev->netdev_ops = &airoha_eth_netdev_ops;
	net_dev->max_mtu = AIROHA_MAX_MTU;
	net_dev->watchdog_timeo = 5 * HZ;
	/* XXX check HW features */
	net_dev->hw_features = AIROHA_HW_FEATURES;
	net_dev->features |= net_dev->hw_features;
	net_dev->dev.of_node = np;
	/* XXX missing IRQ line here */
	SET_NETDEV_DEV(net_dev, dev);

	err = of_get_ethdev_address(np, net_dev);
	if (err) {
		if (err == -EPROBE_DEFER)
			return err;

		eth_hw_addr_random(net_dev);
		dev_err(dev, "generated random MAC address %pM\n",
			net_dev->dev_addr);
	}

	eth->dev = dev;
	err = airoha_eth_hw_init(eth);
	if (err)
		return err;

	/* phylink create */
	err = of_get_phy_mode(np, &phy_mode);
	if (err) {
		dev_err(dev, "incorrect phy-mode\n");
		return err;
	}

	/* mac config is not set */
	eth->interface = PHY_INTERFACE_MODE_NA;
	eth->speed = SPEED_UNKNOWN;

	eth->phylink_config.dev = &net_dev->dev;
	eth->phylink_config.type = PHYLINK_NETDEV;
	eth->phylink_config.mac_capabilities = MAC_ASYM_PAUSE | MAC_SYM_PAUSE |
					       MAC_10 | MAC_100 | MAC_1000 |
					       MAC_2500FD;

	/* XXX: missing phylink_ops callbacks */
	phylink = phylink_create(&eth->phylink_config,
				 of_fwnode_handle(np), phy_mode,
				 &airoha_eth_phylink_ops);
	if (IS_ERR(phylink))
		return PTR_ERR(phylink);

	eth->phylink = phylink;

	return register_netdev(net_dev);
}

static void airoha_eth_remove(struct platform_device *pdev)
{
}

const struct of_device_id of_airoha_eth_match[] = {
	{ .compatible = "airoha,en7581-eth" },
	{ /* sentinel */ }
};

static struct platform_driver airoha_eth_driver = {
	.probe = airoha_eth_probe,
	.remove_new = airoha_eth_remove,
	.driver = {
		.name = "airoha_eth",
		.of_match_table = of_airoha_eth_match,
	},
};
module_platform_driver(airoha_eth_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lorenzo Bianconi <lorenzo@kernel.org>");
MODULE_DESCRIPTION("Ethernet driver for Airoha SoC");
