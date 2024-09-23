// SPDX-License-Identifier: GPL-2.0-only
/*
 * Airoha AN8855 DSA Switch driver
 * Copyright (C) 2023 Min Yao <min.yao@airoha.com>
 * Copyright (C) 2024 Christian Marangi <ansuelsmth@gmail.com>
 */
#include <linux/etherdevice.h>
#include <linux/if_bridge.h>
#include <linux/iopoll.h>
#include <linux/mdio.h>
#include <linux/netdevice.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/phylink.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/gpio/consumer.h>
#include <net/dsa.h>
#include <linux/of_address.h>

#include "an8855.h"

static int an8855_mii_read32(struct mii_bus *bus, u32 reg, u32 *val)
{
	u16 lo, hi;
	int ret;

	ret = bus->write(bus, /* TODO */ 1, 0x1f, 0x4);
	ret = bus->write(bus, /* TODO */ 1, 0x10, 0);

	ret = bus->write(bus, /* TODO */ 1, 0x15, ((reg >> 16) & 0xFFFF));
	ret = bus->write(bus, /* TODO */ 1, 0x16, (reg & 0xFFFF));
	if (ret < 0) {
		dev_err_ratelimited(&bus->dev,
				    "failed to read an8855 register\n");
		return ret;
	}

	lo = bus->read(bus, /* TODO */ 1, 0x18);
	hi = bus->read(bus, /* TODO */ 1, 0x17);

	ret = bus->write(bus, /* TODO */ 1, 0x1f, 0);
	if (ret < 0) {
		dev_err_ratelimited(&bus->dev,
				    "failed to read an8855 register\n");
		return ret;
	}

	*val = (hi << 16) | (lo & 0xffff);

	return 0;
}

static int an8855_regmap_read(void *ctx, uint32_t reg, uint32_t *val)
{
	struct an8855_priv *priv = ctx;
	struct mii_bus *bus = priv->bus;
	int ret;

	mutex_lock_nested(&bus->mdio_lock, MDIO_MUTEX_NESTED);

	ret = an8855_mii_read32(bus, reg, val);
	
	mutex_unlock(&bus->mdio_lock);
	if (ret < 0)
		return ret;

	return 0;
}

static int an8855_mii_write32(struct mii_bus *bus, u32 reg, u32 val)
{
	int ret;

	ret = bus->write(bus, /* TODO */ 1, 0x1f, 0x4);
	ret = bus->write(bus, /* TODO */ 1, 0x10, 0);

	ret = bus->write(bus, /* TODO */ 1, 0x11, ((reg >> 16) & 0xFFFF));
	ret = bus->write(bus, /* TODO */ 1, 0x12, (reg & 0xFFFF));

	ret = bus->write(bus, /* TODO */ 1, 0x13, ((val >> 16) & 0xFFFF));
	ret = bus->write(bus, /* TODO */ 1, 0x14, (val & 0xFFFF));

	ret = bus->write(bus, /* TODO */ 1, 0x1f, 0);
	if (ret < 0)
		dev_err_ratelimited(&bus->dev,
				    "failed to write an8855 register\n");

	return 0;
}

static int
an8855_regmap_write(void *ctx, uint32_t reg, uint32_t val)
{
	struct an8855_priv *priv = ctx;
	struct mii_bus *bus = priv->bus;
	int ret;

	mutex_lock_nested(&bus->mdio_lock, MDIO_MUTEX_NESTED);
	ret = an8855_mii_write32(priv->bus, reg, val);
	mutex_unlock(&bus->mdio_lock);

	return ret;
}

static int
an8855_regmap_update_bits(void *ctx, uint32_t reg, uint32_t mask, uint32_t write_val)
{
	struct an8855_priv *priv = ctx;
	struct mii_bus *bus = priv->bus;
	u32 val;
	int ret;

	mutex_lock_nested(&bus->mdio_lock, MDIO_MUTEX_NESTED);
	
	ret = an8855_mii_read32(bus, reg, &val);
	val &= ~mask;
	val |= write_val;
	ret = an8855_mii_write32(bus, reg, val);

	mutex_unlock(&bus->mdio_lock);

	return 0;
}

static struct regmap_config an8855_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x1028c900, /* TODO */
	.reg_read = an8855_regmap_read,
	.reg_write = an8855_regmap_write,
	.reg_update_bits = an8855_regmap_update_bits,
	.disable_locking = true,
};

static int an8855_setup(struct dsa_switch *ds)
{
	// struct an8855_priv *priv = ds->priv;

	return 0;
}

static const struct dsa_switch_ops an8855_switch_ops = {
	// .get_tag_protocol = air_get_tag_protocol,
	.setup = an8855_setup,
	// .get_strings = an8855_get_strings,
	// .phy_read = an8855_sw_phy_read,
	// .phy_write = an8855_sw_phy_write,
	// .get_ethtool_stats = an8855_get_ethtool_stats,
	// .get_sset_count = an8855_get_sset_count,
	// .port_enable = an8855_port_enable,
	// .port_disable = an8855_port_disable,
	// .port_stp_state_set = an8855_stp_state_set,
	// .port_bridge_join = an8855_port_bridge_join,
	// .port_bridge_leave = an8855_port_bridge_leave,
	// .port_fdb_add = an8855_port_fdb_add,
	// .port_fdb_del = an8855_port_fdb_del,
	// .port_fdb_dump = an8855_port_fdb_dump,
	// .port_vlan_filtering = an8855_port_vlan_filtering,
	// .port_vlan_prepare = an8855_port_vlan_prepare,
	// .port_vlan_add = an8855_port_vlan_add,
	// .port_vlan_del = an8855_port_vlan_del,
	// .port_mirror_add = an8855_port_mirror_add,
	// .port_mirror_del = an8855_port_mirror_del,
	// .phylink_validate = an8855_phylink_validate,
	// .get_mac_eee = an8855_get_mac_eee,
	// .set_mac_eee = an8855_set_mac_eee,
};

static int an8855_read_switch_id(struct an8855_priv *priv)
{
	u32 id;
	int ret;

	ret = regmap_read(priv->regmap, AN8855_CREV, &id);
	if (ret)
		return ret;

	if (id != AN8855_ID) {
		dev_err(priv->dev,
			"Switch id detected %x but expected %x",
			id, AN8855_ID);
		return -ENODEV;
	}

	return 0;
}

static int
an8855_sw_probe(struct mdio_device *mdiodev)
{
	struct an8855_priv *priv;
	int ret;

	priv = devm_kzalloc(&mdiodev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	// /* TODO */ 1 = AN8855_GPHY_SMI_ADDR_DEFAULT;
	// priv->id = priv->info->id;

	// priv->reset = devm_gpiod_get_optional(&mdiodev->dev, "reset",
	// 				      GPIOD_OUT_LOW);
	// if (IS_ERR(priv->reset)) {
	// 	dev_err(&mdiodev->dev, "Couldn't get our reset line\n");
	// 	return PTR_ERR(priv->reset);
	// }

	// ret = of_property_read_u32(dn, "changesmiaddr", &/* TODO */ 1_new);
	// if ((ret < 0) || (/* TODO */ 1_new > 0x1f))
	// 	/* TODO */ 1_new = -1;

	// /* Assign AN8855 interrupt pin */
	// if (of_property_read_u32(dn, "airoha,intr", &priv->intr_pin))
	// 	priv->intr_pin = AN8855_DFL_INTR_ID;

	// if (of_property_read_u32(dn, "airoha,extSurge", &priv->extSurge))
	// 	priv->extSurge = AN8855_DFL_EXT_SURGE;

	priv->bus = mdiodev->bus;
	priv->dev = &mdiodev->dev;

	priv->regmap = devm_regmap_init(&mdiodev->dev, NULL, priv,
					&an8855_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(priv->dev, "regmap initialization failed");
		return PTR_ERR(priv->regmap);
	}

	ret = an8855_read_switch_id(priv);
	if (ret)
		return ret;

	priv->ds = devm_kzalloc(&mdiodev->dev, sizeof(*priv->ds), GFP_KERNEL);
	if (!priv->ds)
		return -ENOMEM;

	priv->ds->priv = priv;
	priv->ds->num_ports = AN8855_NUM_PORTS;
	priv->ds->ops = &an8855_switch_ops;
	// mutex_init(&priv->reg_mutex);

	dev_set_drvdata(&mdiodev->dev, priv);

	return -EINVAL;

	// return dsa_register_switch(priv->ds);
}

static void
an8855_sw_remove(struct mdio_device *mdiodev)
{
	struct an8855_priv *priv = dev_get_drvdata(&mdiodev->dev);

	dsa_unregister_switch(priv->ds);
	// mutex_destroy(&priv->reg_mutex);

	// if (priv->base)
	// 	iounmap(priv->base);

	dev_set_drvdata(&mdiodev->dev, NULL);
}

static const struct of_device_id an8855_of_match[] = {
	{.compatible = "airoha,an8855" },
	{ /* sentinel */ },
};

static struct mdio_driver an8855_mdio_driver = {
	.probe = an8855_sw_probe,
	.remove = an8855_sw_remove,
	.mdiodrv.driver = {
		.name = "an8855",
		.of_match_table = an8855_of_match,
	},
};

mdio_module_driver(an8855_mdio_driver);

MODULE_AUTHOR("Min Yao <min.yao@airoha.com>");
MODULE_AUTHOR("Christian Marangi <ansuelsmth@gmail.com>");
MODULE_DESCRIPTION("Driver for Airoha AN8855 Switch");
MODULE_LICENSE("GPL v2");
