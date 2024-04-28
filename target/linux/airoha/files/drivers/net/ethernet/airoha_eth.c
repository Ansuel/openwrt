#include <linux/etherdevice.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define AIROHA_RX_ETH_HLEN	(ETH_HLEN + ETH_FCS_LEN)
#define AIROHA_MAX_MTU		(2000 - AIROHA_RX_ETH_HLEN)

struct airoha_eth {
	struct device *dev;
	void __iomem *base;
	void __iomem *qdma_base;
};

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
	.ndo_start_xmit		= airoha_eth_start_xmit,
	.ndo_change_mtu		= airoha_eth_change_mtu,
};

static int airoha_eth_probe(struct platform_device *pdev)
{
	struct airoha_eth *eth;
	struct net_device *dev;

	eth = devm_kzalloc(&pdev->dev, sizeof(*eth), GFP_KERNEL);
	if (!eth)
		return -ENOMEM;

	eth->dev = &pdev->dev;
	eth->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(eth->base))
		return PTR_ERR(eth->base);

	eth->qdma_base = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(eth->qdma_base))
		return PTR_ERR(eth->qdma_base);

	dev = devm_alloc_etherdev(&pdev->dev, sizeof(*eth));
	if (!dev)
		return -ENOMEM;

	dev->netdev_ops = &airoha_eth_netdev_ops;
	dev->max_mtu = AIROHA_MAX_MTU;
	SET_NETDEV_DEV(dev, &pdev->dev);

	return register_netdev(dev);
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
