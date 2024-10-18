// SPDX-License-Identifier: GPL-2.0-only
/*
 * Airoha AN8855 DSA Switch driver
 * Copyright (C) 2023 Min Yao <min.yao@airoha.com>
 * Copyright (C) 2024 Christian Marangi <ansuelsmth@gmail.com>
 */
#include <linux/bitfield.h>
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

static const struct an8855_mib_desc an8855_mib[] = {
	MIB_DESC(1, 0x00, "TxDrop"),
	MIB_DESC(1, 0x04, "TxCrcErr"),
	MIB_DESC(1, 0x08, "TxUnicast"),
	MIB_DESC(1, 0x0c, "TxMulticast"),
	MIB_DESC(1, 0x10, "TxBroadcast"),
	MIB_DESC(1, 0x14, "TxCollision"),
	MIB_DESC(1, 0x18, "TxSingleCollision"),
	MIB_DESC(1, 0x1c, "TxMultipleCollision"),
	MIB_DESC(1, 0x20, "TxDeferred"),
	MIB_DESC(1, 0x24, "TxLateCollision"),
	MIB_DESC(1, 0x28, "TxExcessiveCollistion"),
	MIB_DESC(1, 0x2c, "TxPause"),
	MIB_DESC(1, 0x30, "TxPktSz64"),
	MIB_DESC(1, 0x34, "TxPktSz65To127"),
	MIB_DESC(1, 0x38, "TxPktSz128To255"),
	MIB_DESC(1, 0x3c, "TxPktSz256To511"),
	MIB_DESC(1, 0x40, "TxPktSz512To1023"),
	MIB_DESC(1, 0x44, "TxPktSz1024To1518"),
	MIB_DESC(1, 0x48, "TxPktSz1519ToMax"),
	MIB_DESC(2, 0x4c, "TxBytes"),
	MIB_DESC(1, 0x54, "TxOversizeDrop"),
	MIB_DESC(2, 0x58, "TxBadPktBytes"),
	MIB_DESC(1, 0x80, "RxDrop"),
	MIB_DESC(1, 0x84, "RxFiltering"),
	MIB_DESC(1, 0x88, "RxUnicast"),
	MIB_DESC(1, 0x8c, "RxMulticast"),
	MIB_DESC(1, 0x90, "RxBroadcast"),
	MIB_DESC(1, 0x94, "RxAlignErr"),
	MIB_DESC(1, 0x98, "RxCrcErr"),
	MIB_DESC(1, 0x9c, "RxUnderSizeErr"),
	MIB_DESC(1, 0xa0, "RxFragErr"),
	MIB_DESC(1, 0xa4, "RxOverSzErr"),
	MIB_DESC(1, 0xa8, "RxJabberErr"),
	MIB_DESC(1, 0xac, "RxPause"),
	MIB_DESC(1, 0xb0, "RxPktSz64"),
	MIB_DESC(1, 0xb4, "RxPktSz65To127"),
	MIB_DESC(1, 0xb8, "RxPktSz128To255"),
	MIB_DESC(1, 0xbc, "RxPktSz256To511"),
	MIB_DESC(1, 0xc0, "RxPktSz512To1023"),
	MIB_DESC(1, 0xc4, "RxPktSz1024To1518"),
	MIB_DESC(1, 0xc8, "RxPktSz1519ToMax"),
	MIB_DESC(2, 0xcc, "RxBytes"),
	MIB_DESC(1, 0xd4, "RxCtrlDrop"),
	MIB_DESC(1, 0xd8, "RxIngressDrop"),
	MIB_DESC(1, 0xdc, "RxArlDrop"),
	MIB_DESC(1, 0xe0, "FlowControlDrop"),
	MIB_DESC(1, 0xe4, "WredDrop"),
	MIB_DESC(1, 0xe8, "MirrorDrop"),
	MIB_DESC(2, 0xec, "RxBadPktBytes"),
	MIB_DESC(1, 0xf4, "RxsFlowSamplingPktDrop"),
	MIB_DESC(1, 0xf8, "RxsFlowTotalPktDrop"),
	MIB_DESC(1, 0xfc, "PortControlDrop"),
};

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

static int
an8855_mib_init(struct an8855_priv *priv)
{
	struct an8855_priv *priv = ds->priv;
	struct dsa_port *dp;

	ret = regmap_write(priv->regmap, AN8855_MIB_CCR, CCR_MIB_FLUSH);
	if (ret)
		return ret;
	
	return regmap_write(priv->regmap, AN8855_MIB_CCR, CCR_MIB_ACTIVATE);
}

static void an8855_fdb_write(struct an8855_priv *priv, u16 vid,
			     u8 port_mask, const u8 *mac, bool add)
{
	u32 mac_reg[2] = { };
	u32 reg;

	reg[0] |= FIELD_PREP(AN8855_ATA1_MAC0, mac[0]);
	reg[0] |= FIELD_PREP(AN8855_ATA1_MAC1, mac[1]);
	reg[0] |= FIELD_PREP(AN8855_ATA1_MAC2, mac[2]);
	reg[0] |= FIELD_PREP(AN8855_ATA1_MAC3, mac[3]);
	reg[1] |= FIELD_PREP(AN8855_ATA1_MAC4, mac[4]);
	reg[1] |= FIELD_PREP(AN8855_ATA1_MAC5, mac[5]);

	regmap_bulk_write(priv->regmap, AN8855_ATA1, mac_reg,
			  ARRAY_SIZE(mac_reg));

	reg = AN8855_ATWD_IVL;
	if (add)
		reg |= AN8855_ATWD_VLD;
	reg |= FIELD_PREP(AN8855_ATWD_VID, vid);
	regmap_write(priv->regmap, AN8855_ATWD, reg);
	regmap_write(priv->regmap, AN8855_ATWD2,
		     FIELD_PREP(AN8855_ATWD2_PORT, port_mask));
}

static void an8855_fdb_read(struct an8855_priv *priv, struct an8855_fdb *fdb)
{
	u32 reg[4];

	regmap_bulk_read(priv->regmap, AN8855_ATRD0, reg,
			 ARRAY_SIZE(reg));

	fdb->live = FIELD_GET(AN8855_ATRD0_LIVE, reg[0]);
	fdb->type = FIELD_GET(AN8855_ATRD0_TYPE, reg[0]);
	fdb->ivl = FIELD_GET(AN8855_ATRD0_IVL, reg[0]);
	fdb->vid = FIELD_GET(AN8855_ATRD0_VID, reg[0]);
	fdb->fid = FIELD_GET(AN8855_ATRD0_FID, reg[0]);
	fdb->aging = FIELD_GET(AN8855_ATRD1_AGING, reg[1]);
	fdb->port_mask = FIELD_GET(AN8855_ATRD1_PORTMASK, reg[3]);
	fdb->mac[0] = FIELD_GET(AN8855_ATRD1_MAC0, reg[2]);
	fdb->mac[1] = FIELD_GET(AN8855_ATRD1_MAC1, reg[2]);
	fdb->mac[2] = FIELD_GET(AN8855_ATRD1_MAC2, reg[2]);
	fdb->mac[3] = FIELD_GET(AN8855_ATRD1_MAC3, reg[2]);
	fdb->mac[4] = FIELD_GET(AN8855_ATRD1_MAC4, reg[1]);
	fdb->mac[5] = FIELD_GET(AN8855_ATRD1_MAC5, reg[1]);
	fdb->noarp = !!FIELD_GET(AN8855_ATRD0_ARP, reg[0]);
}

static int an8855_fdb_cmd(struct an8855_priv *priv, u32 cmd, u32 *rsp)
{
	u32 val;
	int ret;

	/* Set the command operating upon the MAC address entries */
	val = ATC_BUSY | cmd;
	regmap_write(priv->regmap, AN8855_ATC, val);

	ret = regmap_read_poll_timeout(priv->pregmap, AN8855_ATC, val,
				       !(val & ATC_BUSY), 20, 200000);
	if (ret)
		return ret;

	if (rsp)
		*rsp = val;

	return 0;
}

static void
an8855_stp_state_set(struct dsa_switch *ds, int port, u8 state)
{
	struct an8855_priv *priv = ds->priv;

	switch (state) {
	case BR_STATE_DISABLED:
		stp_state = AN8855_STP_DISABLED;
		break;
	case BR_STATE_BLOCKING:
		stp_state = AN8855_STP_BLOCKING;
		break;
	case BR_STATE_LISTENING:
		stp_state = AN8855_STP_LISTENING;
		break;
	case BR_STATE_LEARNING:
		stp_state = AN8855_STP_LEARNING;
		break;
	case BR_STATE_FORWARDING:
	default:
		stp_state = AN8855_STP_FORWARDING;
		break;
	}

	regmap_update_bits(priv->regmap, AN8855_SSP_P(port), FID_PST_MASK,
			   stp_state);
}

static int an8855_port_bridge_join(struct dsa_switch *ds, int port,
				   struct dsa_bridge bridge,
				   bool *tx_fwd_offload,
				   struct netlink_ext_ack *extack)
{
	struct dsa_port *dp = dsa_to_port(ds, port), *other_dp;
	u32 port_mask = BIT(dp->cpu_dp->index);
	struct an8855_priv *priv = ds->priv;
	int i;

	for (i = 0; i < AN8855_NUM_PORTS; i++) {
		if (i == port)
			continue;

		if (dsa_is_cpu_port(ds, i))
			continue;

		other_dp = dsa_to_port(priv->ds, i);
		if (!dsa_port_offloads_bridge_dev(other_dp, bridge.dev))
			continue;

		/* Add this port to the portvlan mask of the other
		 * ports in the bridge
		 */
		port_mask |= BIT(i);
		regmap_set_bits(priv->regmap, AN8855_PORTMATRIX_P(i),
				BIT(i));
	}

	/* Add/remove all other ports to/from this port's portvlan mask */
	regmap_update_bits(priv->regmap, AN8855_PORTMATRIX_P(i),
			   PORTMATRIX_MASK, port_mask);
}

static void an8855_port_bridge_leave(struct dsa_switch *ds, int port,
				     struct dsa_bridge bridge)
{
	struct dsa_port *dp = dsa_to_port(ds, port), *other_dp;
	u32 port_mask = BIT(dp->cpu_dp->index);
	struct an8855_priv *priv = ds->priv;
	int i;

	for (i = 0; i < AN8855_NUM_PORTS; i++) {
		if (i == port)
			continue;

		if (dsa_is_cpu_port(ds, i))
			continue;

		other_dp = dsa_to_port(priv->ds, i);
		if (!dsa_port_offloads_bridge_dev(other_dp, bridge.dev))
			continue;

		/* Remove this port from the portvlan mask of the other
		 * ports in the bridge
		 */
		regmap_clear_bits(priv->regmap, AN8855_PORTMATRIX_P(i),
				  BIT(i));
	}

	/* Add/remove all other ports to/from this port's portvlan mask */
	regmap_update_bits(priv->regmap, AN8855_PORTMATRIX_P(i),
			   PORTMATRIX_MASK, port_mask);
}

static int an8855_port_fdb_add(struct dsa_switch *ds, int port,
			       const unsigned char *addr, u16 vid,
			       struct dsa_db db)
{
	struct an8855_priv *priv = ds->priv;
	u8 port_mask = BIT(port);

	mutex_lock(&priv->reg_mutex);
	an8855_fdb_write(priv, vid, port_mask, addr, 1);
	ret = an8855_fdb_cmd(priv, AN8855_FDB_WRITE, NULL);
	mutex_unlock(&priv->reg_mutex);

	return ret;
}

static int an8855_port_fdb_del(struct qca8k_priv *priv, const u8 *mac,
			       u16 port_mask, u16 vid)
{
	struct an8855_priv *priv = ds->priv;
	u8 port_mask = BIT(port);

	mutex_lock(&priv->reg_mutex);
	an8855_fdb_write(priv, vid, port_mask, addr, 0);
	ret = an8855_fdb_cmd(priv, AN8855_FDB_WRITE, NULL);
	mutex_unlock(&priv->reg_mutex);

	return ret;
}

static int an8855_port_fdb_dump(struct dsa_switch *ds, int port,
				dsa_fdb_dump_cb_t *cb, void *data)
{
	struct an8855_priv *priv = ds->priv;
	struct an8855_fdb _fdb = {  };
	int banks, count = 0;
	u32 rsp;
	int ret;
	int i;

	mutex_lock(&priv->reg_mutex);

	/* Load search port */
	regmap_write(priv->regmap, AN8855_ATWD2,
		     FIELD_PREP(AN8855_ATWD2_PORT, port));
	ret = an8855_fdb_cmd(priv,  ATC_MAT(AND8855_FDB_MAT_MAC_PORT) |
			     AN8855_FDB_START, &rsp);
	if (ret < 0)
		goto err;

	do {
		/* From response get the number of banks to read, exit if 0 */
		banks = FIELD_GET(ATC_HIT, rsp);
		if (!banks)
			break;

		/* Each banks have 4 entry */
		for (i = 0; i < 4; i++) {
			count++;

			/* Check if bank is present */
			if (!banks & BIT(i))
				continue;

			/* Select bank entry index */
			regmap_write(priv->regmap, AN8855_ATRDS,
				FIELD_PREP(AN8855_ATRD_SEL, i));
			/* wait 1ms for the bank entry to be filled */
			usleep_range(1000, 1500);
			an8855_fdb_read(priv, &_fdb);

			if (!_fdb.live)
				continue;
			ret = cb(_fdb.mac, _fdb.vid, _fdb.noarp, data);
			if (ret < 0)
				break;
		}

		/* Stop if reached max FDB number */
		if (count >= AN8855_NUM_FDB_RECORDS)
			break;

		/* Read next bank */
		ret = an8855_fdb_cmd(priv,  ATC_MAT(AND8855_FDB_MAT_MAC_PORT) |
				     AN8855_FDB_NEXT, &rsp);
		if (ret < 0)
			break;
	} while (true);
	
exit:
	mutex_unlock(&priv->reg_mutex);

	return ret;
}

static int an8855_vlan_cmd(struct an8855_priv *priv, enum an8855_vlan_cmd cmd,
			   u16 vid)
{
	u32 val;

	val = VTCR_BUSY | FIELD_PREP(VTCR_FUNC, cmd) |
	      FIELD_PREP(VTCR_VID, vid);
	regmap_write(priv->regmap, AN8855_VTCR, val);

	return regmap_read_poll_timeout(priv->pregmap, AN8855_VTCR, val,
				        !(val & VTCR_BUSY), 20, 200000);
}

static int an8855_vlan_add(struct an8855_priv *priv, u8 port, u16 vid,
			   bool untagged)
{
	u32 port_mask;
	u32 val;
	int ret;

	/* Fetch entry */
	ret = an8855_vlan_cmd(priv, AN8855_VTCR_RD_VID, vid);
	if (ret)
		return ret;

	regmap_read(priv->regmap, AN8855_VARD0, &val);
	port_mask = FIELD_GET(AN8855_VARD0_PORT, val) | BIT(port) |
		    BIT(AN8855_CPU_PORT);

	/* Validate the entry with independent learning, create egress tag per
	 * VLAN and joining the port as one of the port members.
	 */
	val = (val & AN8855_VARD0_ETAG) | IVL_MAC | VTAG_EN | VLAN_VALID |
	      FIELD_PREP(AN8855_VARD0_PORT, port_mask);
	regmap_write(priv->regmap, AN8855_VAWD0, val);
	regmap_write(priv->regmap, AN8855_VAWD1, 0);

	/* Decide whether adding tag or not for those outgoing packets from the
	 * port inside the VLAN.
	 */
	val = FIELD_PREP(AN8855_VARD0_ETAG_PORT_MASK(port),
			 untagged ? AN8855_VLAN_EGRESS_UNTAG :
				    AN8855_VLAN_EGRESS_TAG);
	regmap_update_bits(priv->regmap, AN8855_VAWD0,
			   AN8855_VARD0_ETAG_PORT_MASK(port),
			   val);

	/* CPU port is always taken as a tagged port for serving more than one
	 * VLANs across and also being applied with egress type stack mode for
	 * that VLAN tags would be appended after hardware special tag used as
	 * DSA tag.
	 */
	regmap_update_bits(priv->regmap, AN8855_VAWD0,
			   AN8855_VARD0_ETAG_PORT_MASK(AN8855_CPU_PORT),
			   FIELD_PREP(AN8855_VARD0_ETAG_PORT_MASK(port),
			   	      AN8855_VLAN_EGRESS_STACK));

	/* Flush result to hardware */
	an8855_vlan_cmd(priv, AN8855_VTCR_WR_VID, vid);

	return 0;
}

static int an8855_vlan_del(struct an8855_priv *priv, u8 port, u16 vid)
{
	u32 port_mask;
	u32 val;
	int ret;

	/* Fetch entry */
	ret = an8855_vlan_cmd(priv, AN8855_VTCR_RD_VID, vid);
	if (ret)
		return ret;

	regmap_read(priv->regmap, AN8855_VARD0, &val);
	port_mask = FIELD_GET(AN8855_VARD0_PORT, val) & ~BIT(port);

	if (!(val & VLAN_VALID)) {
		dev_err(priv->dev, "Cannot be deleted due to invalid entry\n");
		return -EINVAL;
	}

	if (port_mask && port_mask != BIT(AN8855_CPU_PORT)) {
		val = (val & AN8855_VARD0_ETAG) | IVL_MAC | VTAG_EN | VLAN_VALID |
	      	       FIELD_PREP(AN8855_VARD0_PORT, port_mask);
		regmap_write(priv->regmap, AN8855_VAWD0, val);
	} else {
		regmap_write(priv->regmap, AN8855_VAWD0, 0);
	}
	regmap_write(priv->regmap, AN8855_VAWD1, 0);

	/* Flush result to hardware */
	an8855_vlan_cmd(priv, AN8855_VTCR_WR_VID, vid);

	return 0;
}

static int an8855_port_set_vlan_mode(struct an8855_priv *priv, int port,
				     enum an8855_port_mode port_mode,
				     enum an8855_vlan_port_eg_tag eg_tag,
				     enum an8855_vlan_port_attr vlan_attr)
{
	int ret;

	ret = regmap_update_bits(priv->regmap, AN8855_PCR_P(port),
				 PORT_VLAN, FIELD_PREP(PORT_VLAN, port_mode));
	if (ret)
		return ret;

	return regmap_update_bits(priv->regmap, AN8855_PVC_P(port),
				  PVC_EG_TAG_MASK | VLAN_ATTR_MASK,
				  FIELD_PREP(PVC_EG_TAG_MASK, eg_tag) |
				  FIELD_PREP(VLAN_ATTR_MASK, vlan_attr));
}

static int an8855_port_vlan_filtering(struct dsa_switch *ds, int port,
				      bool vlan_filtering,
				      struct netlink_ext_ack *extack)
{
	struct an8855_priv *priv = ds->priv;

	/* The port is being kept as VLAN-unaware port when bridge is
	 * set up with vlan_filtering not being set, Otherwise, the
	 * port and the corresponding CPU port is required the setup
	 * for becoming a VLAN-aware port.
	 */
	if (vlan_filtering) {
		/* CPU port is set to fallback mode to let untagged 
		 * frames pass through.
		 */
		an8855_port_set_vlan_mode(priv, AN8855_CPU_PORT,
					  AN8855_PORT_FALLBACK_MODE,
					  AN8855_VLAN_EG_DISABLED,
					  AN8855_VLAN_USER);

		/* Trapped into security mode allows packet forwarding through VLAN
	 	 * table lookup.
		 * Set the port as a user port which is to be able to recognize VID
		 * from incoming packets before fetching entry within the VLAN table.
		 */
		an8855_port_set_vlan_mode(priv, port,
					  AN8855_PORT_SECURITY_MODE,
					  AN8855_VLAN_EG_DISABLED,
					  AN8855_VLAN_USER);
	} else {
		bool disable_cpu_vlan = true;
		struct dsa_port *dp;

		/* When a port is removed from the bridge, the port would be set up
		 * back to the default as is at initial boot which is a VLAN-unaware
		 * port.
		 */
		an8855_port_set_vlan_mode(priv, port, AN8855_PORT_MATRIX_MODE,
					  AN8855_VLAN_EG_CONSISTENT,
					  AN8855_VLAN_TRANSPARENT);

		dsa_switch_for_each_user_port(dp, ds) {
			if (dsa_port_is_vlan_filtering(dp)) {
				disable_cpu_vlan = false;
				break;
			}
		}

		if (disable_cpu_vlan) {
			an8855_port_set_vlan_mode(priv, AN8855_CPU_PORT,
						  AN8855_PORT_MATRIX_MODE,
						  AN8855_VLAN_EG_CONSISTENT,
						  AN8855_VLAN_TRANSPARENT);
		}
	}

	return 0;
}

static int an8855_port_vlan_add(struct dsa_switch *ds, int port,
				const struct switchdev_obj_port_vlan *vlan,
				struct netlink_ext_ack *extack)
{
	bool untagged = vlan->flags & BRIDGE_VLAN_INFO_UNTAGGED;
	bool pvid = vlan->flags & BRIDGE_VLAN_INFO_PVID;
	struct an8855_priv *priv = ds->priv;
	int ret;

	mutex_lock(&priv->reg_mutex);

	ret = an8855_vlan_add(priv, port, vlan->vid, untagged);
	if (ret)
		return ret;

	if (pvid) {
		regmap_update_bits(priv->regmap, AN8855_PVID_P(port),
				   G0_PORT_VID_MASK,
				   FIELD_PREP(G0_PORT_VID_MASK, vlan->vid));
	}

	mutex_unlock(&priv->reg_mutex);

	return 0;
}

static int an8855_port_vlan_del(struct dsa_switch *ds, int port,
				const struct switchdev_obj_port_vlan *vlan)
{
	struct an8855_priv *priv = ds->priv;
	u32 val;
	int ret;

	mutex_lock(&priv->reg_mutex);

	ret = an8855_vlan_del(priv, port, vlan->vid);
	if (ret)
		return ret;

	regmap_read(priv->regmap, AN8855_PVID_P(port), &val);
	if (FIELD_GET(G0_PORT_VID_MASK, val) == vlan->vid)
		regmap_update_bits(priv->regmap, AN8855_PVID_P(port),
				   G0_PORT_VID_MASK, G0_PORT_VID_DEF);

	mutex_unlock(&priv->reg_mutex);

	return 0;
}

static void
an8855_get_strings(struct dsa_switch *ds, int port, u32 stringset,
		   uint8_t *data)
{
	int i;

	if (stringset != ETH_SS_STATS)
		return;

	for (i = 0; i < ARRAY_SIZE(an8855_mib); i++)
		strncpy(data + i * ETH_GSTRING_LEN, an8855_mib[i].name,
			ETH_GSTRING_LEN);
}

static void
an8855_get_ethtool_stats(struct dsa_switch *ds, int port, uint64_t *data)
{
	struct an8855_priv *priv = ds->priv;
	const struct an8855_mib_desc *mib;
	u32 reg, i;
	u64 hi;

	for (i = 0; i < ARRAY_SIZE(an8855_mib); i++) {
		mib = &an8855_mib[i];
		reg = AN8855_PORT_MIB_COUNTER(port) + mib->offset;

		data[i] = regmap_read(priv->regmap, reg);
		if (mib->size == 2) {
			hi = regmap_read(priv->regmap, reg + 4);
			data[i] |= hi << 32;
		}
	}
}

static int
an8855_get_sset_count(struct dsa_switch *ds, int port, int sset)
{
	if (sset != ETH_SS_STATS)
		return 0;

	return ARRAY_SIZE(an8855_mib);
}

static int an8855_port_mirror_add(struct dsa_switch *ds, int port,
				  struct dsa_mall_mirror_tc_entry *mirror,
				  bool ingress)
{
	struct an8855_priv *priv = ds->priv;
	int monitor_port;
	u32 val;

	/* Check for existent entry */
	if ((ingress ? priv->mirror_rx : priv->mirror_tx) & BIT(port))
		return -EEXIST;

	val = regmap_read(priv->regmap, AN8855_MIR);

	/* AN8855 supports 4 monitor port, but only use first group */
	monitor_port = AN8855_MIRROR_PORT_GET(val);
	if (val & AN8855_MIRROR_EN && monitor_port != mirror->to_local_port)
		return -EEXIST;

	val |= AN8855_MIRROR_EN;
	val &= ~AN8855_MIRROR_MASK;
	val |= AN8855_MIRROR_PORT_SET(mirror->to_local_port);
	regmap_write(priv->regmap, AN8855_MIR, val);

	val = regmap_read(priv->regmap, AN8855_PCR_P(port));
	if (ingress) {
		val |= PORT_RX_MIR;
		priv->mirror_rx |= BIT(port);
	} else {
		val |= PORT_TX_MIR;
		priv->mirror_tx |= BIT(port);
	}
	regmap_write(priv->regmap, AN8855_PCR_P(port), val);

	return 0;
}

static void an8855_port_mirror_del(struct dsa_switch *ds, int port,
				   struct dsa_mall_mirror_tc_entry *mirror)
{
	struct an8855_priv *priv = ds->priv;
	u32 val;

	val = regmap_read(priv->regmap, AN8855_PCR_P(port));
	if (mirror->ingress) {
		val &= ~PORT_RX_MIR;
		priv->mirror_rx &= ~BIT(port);
	} else {
		val &= ~PORT_TX_MIR;
		priv->mirror_tx &= ~BIT(port);
	}
	regmap_write(priv->regmap, AN8855_PCR_P(port), val);

	if (!priv->mirror_rx && !priv->mirror_tx) {
		val = regmap_read(priv->regmap, AN8855_MIR);
		val &= ~AN8855_MIRROR_EN;
		regmap_write(priv->regmap, AN8855_MIR, val);
	}
}

static int an8855_port_set_status(struct an8855_priv *priv, int port,
				  bool enable)
{
	if (enable)
		return regmap_set_bits(priv->regmap, AN8855_PMCR_P(port),
				       PMCR_TX_EN | PMCR_RX_EN);
	else
		return regmap_clear_bits(priv->regmap, AN8855_PMCR_P(port),
					 PMCR_TX_EN | PMCR_RX_EN);
}

static int an8855_port_enable(struct dsa_switch *ds, int port,
			      struct phy_device *phy)
{
	struct an8855_priv *priv = ds->priv;
	int ret;

	ret = an8855_port_set_status(ds->priv, port, true);
	if (ret)
		return ret;

	if (dsa_is_user_port(ds, port))
		phy_support_asym_pause(phy);

	return 0;
}

static int an8855_port_disable(struct dsa_switch *ds, int port,
			       struct phy_device *phy)
{
	return an8855_port_set_status(ds->priv, port, false);
}

static int an8855_set_mac_eee(struct dsa_switch *ds, int port,
			      struct ethtool_keee *eee)
{
	struct an8855_priv *priv = ds->priv;
	u32 reg;

	if (eee->eee_enabled) {
		regmap_read(priv->regmap, AN8855_PMCR_P(port), &reg);
		if (reg & AN8855_FORCE_MODE) {
			switch(reg & PMSR_SPEED_MASK) {
			case PMSR_SPEED_1000:
				reg |= PMCR_FORCE_EEE1G;
				break;
			case PMSR_SPEED_100:
				reg |= PMCR_FORCE_EEE100;
				break;
			default:
				break;
			}
			regmap_write(priv->regmap, AN8855_PMCR_P(port),
				     &reg);
		}
		if (eee->tx_lpi_enabled)
			regmap_set_bits(priv->regmap, AN8855_PMEEECR_P(port),
					LPI_MODE_EN);
		else
			regmap_clear_bits(priv->regmap, AN8855_PMEEECR_P(port),
					  LPI_MODE_EN);
	} else {
		regmap_clear_bits(priv->regmap, AN8855_PMCR_P(port),
				  PMCR_FORCE_EEE1G | PMCR_FORCE_EEE100);
		regmap_clear_bits(priv->regmap, AN8855_PMEEECR_P(port),
				  LPI_MODE_EN);
	}

	return 0;
}

static int an8855_get_mac_eee(struct dsa_switch *ds, int port,
			      struct ethtool_keee *eee)
{
	struct an8855_priv *priv = ds->priv;
	u32 reg;

	regmap_read(priv->regmap, AN8855_PMEEECR_P(port), &reg);
	eee->tx_lpi_enabled = reg & LPI_MODE_EN;

	regmap_read(priv->regmap, AN8855_CKGCR, &reg);
	/* Global LPI TXIDLE Threshold, defualt 60ms (unit 2us) */
	e->tx_lpi_time = FIELD_GET(LPI_TXIDLE_THD_MASK, reg) / 500;

	regmap_read(priv->regmap, AN8855_PMSR_P(port), &reg);
	eee->eee_active = reg & (PMCR_FORCE_EEE1G | PMCR_FORCE_EEE100);

	return 0;
}

static enum dsa_tag_protocol
an8855_get_tag_protocol(struct dsa_switch *ds, int port,
		       enum dsa_tag_protocol mp)
{
	/* TODO CHECK DIFFERENCES */
	return DSA_TAG_PROTO_MTK;
}

static int an8855_setup(struct dsa_switch *ds)
{
	struct an8855_priv *priv = ds->priv;
	struct dsa_port *dp;
	int ret;

	/* Enable and reset MIB counters */
	ret = an8855_mib_init(priv);
	if (ret)
		return ret;

	/* TODO */
	dsa_switch_for_each_port(dp, ds) {
		/* Disable forwarding by default on all ports */
		ret = regmap_clear_bits(priv->regmap, AN8855_PORTMATRIX_P(dp->index),
					PORTMATRIX_MASK);
		if (ret)
			return ret;

		/* Enable consistent egress tag */
		regmap_update_bits(priv->regmap, AN8855_PVC_P(dp->index),
				   PVC_EG_TAG_MASK,
				   FIELD_PREP(PVC_EG_TAG_MASK, AN8855_VLAN_EG_CONSISTENT));
	}

	/* Disable MAC by default on all user ports */
	dsa_switch_for_each_user_port(dp, ds)
		an8855_port_set_status(priv, dp->index, false);

	/* Enable AIROHA header mode on all cpu ports */
	dsa_switch_for_each_cpu_port(dp, ds) {
		/* TODO check ref for phy capabilities */

		/* Enable Airoha header mode on the cpu port */
		ret = regmap_write(priv->regmap, AN8855_PVC_P(dp->index),
				   PORT_SPEC_REPLACE_MODE | PORT_SPEC_TAG);
		if (ret)
			return ret;

		/* Unknown multicast frame forwarding to the cpu port */
		ret = regmap_write(priv->regmap, AN8855_UNMF, BIT(dp->index));
		if (ret)
			return ret;

		/* Set CPU port number */
		ret = regmap_update_bits(priv->regmap, AN8855_MFC, CPU_MASK,
					 CPU_EN | CPU_PORT(dp->index));
		if (ret)
			return ret;

		/* CPU port gets connected to all user ports of
		 * the switch.
		 */
		ret = regmap_write(priv->regmap, AN8855_PORTMATRIX_P(dp->index),
				   PORTMATRIX_MATRIX(dsa_user_ports(ds)));
		if (ret)
			return ret;
	}

	/* TODO an8855_phy_setup */

	ds->configure_vlan_while_not_filtering = true;

	/* Flush the FDB table */
	ret = an8855_fdb_cmd(priv, AN8855_FDB_FLUSH, NULL);
	if (ret < 0)
		return ret;

	return 0;
}

static struct phylink_pcs *
an8855_phylink_mac_select_pcs(struct phylink_config *config,
			      phy_interface_t interface)
{
	struct dsa_port *dp = dsa_phylink_to_port(config);
	struct an8855_priv *priv = dp->ds->priv;

	switch (interface) {
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_2500BASEX:
		return &priv->pcs;
	default:
		return NULL;
	}
}

static void
an8855_phylink_mac_config(struct phylink_config *config, unsigned int mode,
			  const struct phylink_link_state *state)
{
	struct dsa_port *dp = dsa_phylink_to_port(config);
	struct dsa_switch *ds = dp->ds;
	struct an8855_priv *priv;
	int port = dp->index;

	priv = ds->priv;

	switch(port) {
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
		return;
	case 5:
		break;
	default:
		dev_err(ds->dev, "unsupported port: %d", port);
		return;
	}

	if (state->interface == PHY_INTERFACE_MODE_2500BASEX &&
	    phylink_autoneg_inband(mode))
	    	dev_err(ds->dev, "in-band negotiation unsupported");
}

static void an8855_phylink_get_caps(struct dsa_switch *ds, int port,
				    struct phylink_config *config)
{
	switch(port) {
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
		__set_bit(PHY_INTERFACE_MODE_INTERNAL,
			  config->supported_interfaces);
		break;
	case 5:
		phy_interface_set_rgmii(config->supported_interfaces);
		__set_bit(PHY_INTERFACE_MODE_SGMII,
			  config->supported_interfaces);
		__set_bit(PHY_INTERFACE_MODE_2500BASEX,
			  config->supported_interfaces);
		break;
	}

	config->mac_capabilities = MAC_ASYM_PAUSE | MAC_SYM_PAUSE |
				   MAC_10 | MAC_100 | MAC_1000FD;
}

static void
an8855_phylink_mac_link_down(struct phylink_config *config, unsigned int mode,
			     phy_interface_t interface)
{
	struct dsa_port *dp = dsa_phylink_to_port(config);
	struct an8855_priv *priv = dp->ds->priv;

	/* Disable TX/RX, force link down */
	regmap_update_bits(priv->regmap, AN8855_PMCR_P(dp->index),
			   PMCR_TX_EN | PMCR_RX_EN | AN8855_FORCE_MODE | PMCR_FORCE_LNK,
			   AN8855_FORCE_MODE);
}

static void
an8855_phylink_mac_link_up(struct phylink_config *config,
			   struct phy_device *phydev, unsigned int mode,
			   phy_interface_t interface, int speed, int duplex,
			   bool tx_pause, bool rx_pause)
{
	struct dsa_port *dp = dsa_phylink_to_port(config);
	struct an8855_priv *priv = dp->ds->priv;
	int port = dp->index;
	u32 reg;

	reg = regmap_read(priv->regmap, AN8855_PMCR_P(port), &reg);
	if (phylink_autoneg_inband(mode)) {
		reg &= ~AN8855_FORCE_MODE;
	} else {
		reg |= AN8855_FORCE_MODE | PMCR_FORCE_LNK;

		reg &= ~PMCR_FORCE_SPEED_MASK;
		switch(speed) {
		case SPEED_10:
			reg |= PMCR_FORCE_SPEED_10;
		case SPEED_100:
			reg |= PMCR_FORCE_SPEED_100;
		case SPEED_1000:
			reg |= PMCR_FORCE_SPEED_1000;
		case SPEED_2500:
			reg |= PMCR_FORCE_SPEED_2500;
		case SPEED_5000:
			reg |= PMCR_FORCE_SPEED_5000;
		}

		reg &= PMCR_FORCE_FDX;
		if (duplex == DUPLEX_FULL)
			reg |= PMCR_FORCE_FDX;

		reg &= PMCR_RX_FC_EN;
		if (rx_pause || dsa_port_is_cpu(dp))
			reg |= PMCR_RX_FC_EN;

		reg &= PMCR_TX_FC_EN;
		if (rx_pause || dsa_port_is_cpu(dp))
			reg |= PMCR_TX_FC_EN;

		/* Disable any EEE options */
		reg &= ~(PMCR_FORCE_EEE1G | PMCR_FORCE_EEE100 |
		         PMCR_FORCE_EEE2P5G | PMCR_FORCE_EEE5G);
	}

	reg |= PMCR_TX_EN | PMCR_RX_EN;

	regmap_write(priv->regmap, AN8855_PMCR_P(port), reg);
}

static void an8855_pcs_get_state(struct phylink_pcs *pcs,
				 struct phylink_link_state *state)
{
	struct an8855_priv *priv = container_of(pcs, struct an8855_priv, pcs);
	u32 reg;
	int ret;

	ret = regmap_read(priv->regmap, AN8855_PMSR_P(AN8855_CPU_PORT), &reg);
	if (ret < 0) {
		state->link = false;
		return;
	}

	state->link = !!(reg & PMSR_LINK);
	state->an_complete = state->link;
	state->duplex = (pmsr & PMSR_DPX) ? DUPLEX_FULL :
					    DUPLEX_HALF;

	switch (pmsr & PMSR_SPEED_MASK) {
	case PMSR_SPEED_10:
		state->speed = SPEED_10;
		break;
	case PMSR_SPEED_100:
		state->speed = SPEED_100;
		break;
	case PMSR_SPEED_1000:
		state->speed = SPEED_1000;
		break;
	case PMSR_SPEED_2500:
		state->speed = SPEED_2500;
		break;
	case PMSR_SPEED_5000:
		state->speed = SPEED_5000;
		break;
	default:
		state->speed = SPEED_UNKNOWN;
		break;
	}

	if (pmsr & PMSR_RX_FC)
		state->pause |= MLO_PAUSE_RX;
	if (pmsr & PMSR_TX_FC)
		state->pause |= MLO_PAUSE_TX;
}

static int an8855_pcs_config(struct phylink_pcs *pcs, unsigned int neg_mode,
			     phy_interface_t interface,
			     const unsigned long *advertising,
			     bool permit_pause_to_mac)
{
	struct an8855_priv *priv = container_of(pcs, struct an8855_priv, pcs);
	u32 val;
	int ret;

	/* Check port 4 */

	switch(interface) {
	case PHY_INTERFACE_MODE_RGMII:
		break;
	case PHY_INTERFACE_MODE_SGMII:
		break;
	case PHY_INTERFACE_MODE_2500BASEX:
		if (neg_mode == PHYLINK_PCS_NEG_INBAND_ENABLED)
			return -EINVAL;

		break;
	default:
		return -EINVAL;
	}

	/* TX FIR - improve TX EYE */
	regmap_update_bits(priv->regmap, INTF_CTRL_10, GENMASK(21, 16),
			   FIELD_PREP(GENMASK(21, 16), 0x20));
	regmap_update_bits(priv->regmap, INTF_CTRL_10, GENMASK(28, 24),
			   FIELD_PREP(GENMASK(28, 24), 0x4));
	regmap_set_bits(priv->regmap, INTF_CTRL_10, BIT(29));

	if (interface == PHY_INTERFACE_MODE_2500BASEX)
		val = 0x0;
	else
		val = 0xd;
	regmap_update_bits(priv->regmap, INTF_CTRL_11, GENMASK(5, 0),
			   FIELD_PREP(GENMASK(5, 0), val));
	regmap_set_bits(priv->regmap, INTF_CTRL_11, BIT(6));

	/* RX CDR - improve RX Jitter Tolerance */
	if (interface == PHY_INTERFACE_MODE_2500BASEX)
		val = 0x5;
	else
		val = 0x6;
	regmap_update_bits(priv->regmap, RG_QP_CDR_LPF_BOT_LIM, GENMASK(26, 24),
			   FIELD_PREP(GENMASK(26, 24), val));
	regmap_update_bits(priv->regmap, RG_QP_CDR_LPF_BOT_LIM, GENMASK(22, 20),
			   FIELD_PREP(GENMASK(22, 20), val));

	/* PLL */
	if (interface == PHY_INTERFACE_MODE_2500BASEX)
		val = 0x1;
	else
		val = 0x0;
	regmap_update_bits(priv->regmap, QP_DIG_MODE_CTRL_1, GENMASK(3, 2),
			   FIELD_PREP(GENMASK(3, 2), val));

	/* PLL - LPF */
	regmap_update_bits(priv->regmap, PLL_CTRL_2, GENMASK(1, 0),
			   FIELD_PREP(GENMASK(1, 0), 0x1));
	regmap_update_bits(priv->regmap, PLL_CTRL_2, GENMASK(4, 2),
			   FIELD_PREP(GENMASK(4, 2), 0x5));
	regmap_clear_bits(priv->regmap, PLL_CTRL_2, BIT(6) | BIT(7));
	regmap_update_bits(priv->regmap, PLL_CTRL_2, GENMASK(10, 8),
			   FIELD_PREP(GENMASK(10, 8), 0x3));
	regmap_set_bits(priv->regmap, PLL_CTRL_2, BIT(29));
	regmap_clear_bits(priv->regmap, PLL_CTRL_2, BIT(12) | BIT(13));

	/* PLL - ICO */
	regmap_set_bits(priv->regmap, PLL_CTRL_4, BIT(2));
	regmap_clear_bits(priv->regmap, PLL_CTRL_2, BIT(14));

	/* PLL - CHP */
	if (interface == PHY_INTERFACE_MODE_2500BASEX)
		val = 0x6;
	else
		val = 0x4;
	regmap_update_bits(priv->regmap, PLL_CTRL_2, GENMASK(19, 16),
			   FIELD_PREP(GENMASK(19, 16), val));

	/* PLL - PFD */
	regmap_update_bits(priv->regmap, PLL_CTRL_2, GENMASK(21, 20),
			   FIELD_PREP(GENMASK(21, 20), 0x1));
	regmap_update_bits(priv->regmap, PLL_CTRL_2, GENMASK(25, 24),
			   FIELD_PREP(GENMASK(25, 24), 0x1));
	regmap_clear_bits(priv->regmap, PLL_CTRL_2, BIT(26));
	
	/* PLL - POSTDIV */
	regmap_set_bits(priv->regmap, PLL_CTRL_2, BIT(22));
	regmap_clear_bits(priv->regmap, PLL_CTRL_2, BIT(27));
	regmap_clear_bits(priv->regmap, PLL_CTRL_2, BIT(28));

	/* PLL - SDM */
	regmap_clear_bits(priv->regmap, PLL_CTRL_4, BIT(3) | BIT(4));
	regmap_clear_bits(priv->regmap, PLL_CTRL_2, BIT(30));

	regmap_update_bits(priv->regmap, SS_LCPLL_PWCTL_SETTING_2,
			   GENMASK(17, 16),
			   FIELD_PREP(GENMASK(17, 16), 0x1));

	if (interface == PHY_INTERFACE_MODE_2500BASEX)
		val = 0x7a000000;
	else
		val = 0x48000000;
	regmap_write(priv->regmap, SS_LCPLL_TDC_FLT_2, val);
	regmap_write(priv->regmap, SS_LCPLL_TDC_PCW_1, val);

	regmap_clear_bits(priv->regmap, SS_LCPLL_TDC_FLT_5, BIT(24));
	regmap_clear_bits(priv->regmap, PLL_CK_CTRL_0, BIT(8));

	/* PLL - SS */
	regmap_clear_bits(priv->regmap, PLL_CTRL_3, GENMASK(15, 0));
	regmap_clear_bits(priv->regmap, PLL_CTRL_4, GENMASK(1, 0));
	regmap_clear_bits(priv->regmap, PLL_CTRL_3, GENMASK(31, 16)); /* ??? */

	/* PLL - TDC */
	regmap_clear_bits(priv->regmap, PLL_CK_CTRL_0, BIT(9));

	regmap_set_bits(priv->regmap, RG_QP_PLL_SDM_ORD, BIT(3));
	regmap_set_bits(priv->regmap, RG_QP_PLL_SDM_ORD, BIT(4));

	regmap_update_bits(priv->regmap, RG_QP_RX_DAC_EN, GENMASK(17, 16),
			   FIELD_PREP(GENMASK(17, 16), 0x2));

	/* TCL Disable (only for Co-SIM) */
	regmap_clear_bits(priv->regmap, PON_RXFEDIG_CTRL_0, BIT(12));

	/* TX Init */
	if (interface == PHY_INTERFACE_MODE_2500BASEX)
		val = 0x4;
	else
		val = 0x0;
	regmap_clear_bits(priv->regmap, RG_QP_TX_MODE_16B_EN, BIT(0));
	regmap_update_bits(priv->regmap, RG_QP_TX_MODE_16B_EN, GENMASK(31, 16),
			   FIELD_PREP(GENMASK(31, 16), val));

	/* RX Control/Init */
	regmap_set_bits(priv->regmap, RG_QP_RXAFE_RESERVE, BIT(11));

	if (interface == PHY_INTERFACE_MODE_2500BASEX)
		val = 0x1;
	else
		val = 0x2;
	regmap_update_bits(priv->regmap, RG_QP_CDR_LPF_MJV_LIM, GENMASK(5, 4),
			   FIELD_PREP(GENMASK(5, 4), val));

	regmap_update_bits(priv->regmap, RG_QP_CDR_LPF_SETVALUE, GENMASK(28, 25),
			   FIELD_PREP(GENMASK(28, 25), 0x1));
	regmap_update_bits(priv->regmap, RG_QP_CDR_LPF_SETVALUE, GENMASK(31, 29),
			   FIELD_PREP(GENMASK(31, 29), 0x6));

	if (interface == PHY_INTERFACE_MODE_2500BASEX)
		val = 0xf;
	else
		val = 0xc;
	regmap_update_bits(priv->regmap, RG_QP_CDR_PR_CKREF_DIV1, GENMASK(12, 8),
			   FIELD_PREP(GENMASK(12, 8), val));

	regmap_update_bits(priv->regmap, RG_QP_CDR_PR_KBAND_DIV_PCIE, GENMASK(12, 8),
			   FIELD_PREP(GENMASK(12, 8), 0x19));
	regmap_clear_bits(priv->regmap, RG_QP_CDR_PR_KBAND_DIV_PCIE, BIT(6));

	regmap_update_bits(priv->regmap, RG_QP_CDR_FORCE_IBANDLPF_R_OFF, GENMASK(12, 6),
			   FIELD_PREP(GENMASK(12, 6), 0x21));
	regmap_update_bits(priv->regmap, RG_QP_CDR_FORCE_IBANDLPF_R_OFF, GENMASK(17, 16),
			   FIELD_PREP(GENMASK(17, 16), 0x2));
	regmap_clear_bits(priv->regmap, RG_QP_CDR_FORCE_IBANDLPF_R_OFF, BIT(13));

	regmap_clear_bits(priv->regmap, RG_QP_CDR_PR_KBAND_DIV_PCIE, BIT(30));

	regmap_update_bits(priv->regmap, RG_QP_CDR_PR_CKREF_DIV1, GENMASK(26, 24),
			   FIELD_PREP(GENMASK(26, 24), 0x4));

	regmap_set_bits(priv->regmap, RX_CTRL_26, BIT(23));
	regmap_clear_bits(priv->regmap, RX_CTRL_26, BIT(24));
	regmap_set_bits(priv->regmap, RX_CTRL_26, BIT(26));

	regmap_update_bits(priv->regmap, RX_DLY_0, GENMASK(7, 0),
			   FIELD_PREP(GENMASK(7, 0), 0x6f));
	regmap_set_bits(priv->regmap, RX_DLY_0, GENMASK(13, 8));

	regmap_update_bits(priv->regmap, RX_CTRL_42, GENMASK(12, 0),
			   FIELD_PREP(GENMASK(12, 0), 0x150));

	regmap_update_bits(priv->regmap, RX_CTRL_2, GENMASK(28, 16),
			   FIELD_PREP(GENMASK(28, 16), 0x150));

	regmap_update_bits(priv->regmap, PON_RXFEDIG_CTRL_9, GENMASK(2, 0),
			   FIELD_PREP(GENMASK(2, 0), 0x1));

	regmap_update_bits(priv->regmap, RX_CTRL_8, GENMASK(27, 16),
			   FIELD_PREP(GENMASK(27, 16), 0x200));
	regmap_update_bits(priv->regmap, RX_CTRL_8, GENMASK(14, 0),
			   FIELD_PREP(GENMASK(14, 0), 0xfff));

	/* Frequency meter */
	if (interface == PHY_INTERFACE_MODE_2500BASEX)
		val = 0x10;
	else
		val = 0x28;
	regmap_update_bits(priv->regmap, RX_CTRL_5, GENMASK(29, 10),
			   FIELD_PREP(GENMASK(29, 10), val));

	regmap_update_bits(priv->regmap, RX_CTRL_6, GENMASK(19, 0),
			   FIELD_PREP(GENMASK(19, 0), 0x64));

	regmap_update_bits(priv->regmap, RX_CTRL_7, GENMASK(19, 0),
			   FIELD_PREP(GENMASK(19, 0), 0x2710));

	regmap_set_bits(priv->regmap, PLL_CTRL_0, BIT(0));

	/* PCS Init */
	if (neg == PHYLINK_PCS_NEG_INBAND_DISABLED) {
		regmap_clear_bits(priv->regmap, QP_DIG_MODE_CTRL_0, BIT(0));
		regmap_clear_bits(priv->regmap, QP_DIG_MODE_CTRL_0, GENMASK(5, 4));
	}

	regmap_clear_bits(priv->regmap, RG_HSGMII_PCS_CTROL_1, BIT(30));

	if (neg == PHYLINK_PCS_NEG_INBAND_ENABLED) {
		/* Set AN Ability - Interrupt */
		regmap_set_bits(priv->regmap, SGMII_REG_AN_FORCE_CL37, BIT(0));

		regmap_update_bits(priv->regmap, SGMII_REG_AN_13, GENMASK(5, 0),
				   FIELD_PREP(GENMASK(5, 4), 0xb));
		regmap_set_bits(priv->regmap, SGMII_REG_AN_13, BIT(8));
	}

	/* Rate Adaption - GMII path config. */
	if (interface == PHY_INTERFACE_MODE_2500BASEX) {
		regmap_clear_bits(priv->regmap, RATE_ADP_P0_CTRL_0, BIT(31));
	} else {
		if (neg == PHYLINK_PCS_NEG_INBAND_ENABLED) {
			regmap_set_bits(priv->regmap, MII_RA_AN_ENABLE, BIT(0));
		} else {
			regmap_set_bits(priv->regmap, RG_AN_SGMII_MODE_FORCE, BIT(0));
			regmap_clear_bits(priv->regmap, RG_AN_SGMII_MODE_FORCE, GENMASK(5, 4));

			regmap_clear_bits(priv->regmap, RATE_ADP_P0_CTRL_0, GENMASK(3, 0));
		}

		regmap_set_bits(priv->regmap, RATE_ADP_P0_CTRL_0, BIT(28));
	}

	regmap_set_bits(priv->regmap, RG_RATE_ADAPT_CTRL_0, BIT(0));
	regmap_set_bits(priv->regmap, RG_RATE_ADAPT_CTRL_0, BIT(4));
	regmap_set_bits(priv->regmap, RG_RATE_ADAPT_CTRL_0, GENMASK(27, 26));

	/* Disable AN */
	if (neg == PHYLINK_PCS_NEG_INBAND_ENABLED)
		regmap_set_bits(priv->regmap, SGMII_REG_AN0, SGMII_AN_ENABLE);
	else
		regmap_clear_bits(priv->regmap, SGMII_REG_AN0, SGMII_AN_ENABLE);

	if (interface == PHY_INTERFACE_MODE_SGMII &&
	    neg == PHYLINK_PCS_NEG_INBAND_DISABLED)
	    	regmap_set_bits(priv->regmap, PHY_RX_FORCE_CTRL_0, BIT(4));

	/* Force Speed */
	if (interface == PHY_INTERFACE_MODE_2500BASEX ||
	    neg == PHYLINK_PCS_NEG_INBAND_ENABLED) {
		if (interface == PHY_INTERFACE_MODE_2500BASEX)
			val = 0x0;
		else
			val = 0x2;
		regmap_set_bits(priv->regmap, SGMII_STS_CTRL_0, BIT(2));
		regmap_update_bits(priv->regmap, SGMII_STS_CTRL_0, GENMASK(5, 4),
				   FIELD_PREP(GENMASK(5, 4), val));
	}

	/* bypass flow control to MAC */
	regmap_write(priv->regmap, MSG_RX_LIK_STS_0, 0x01010107);
	regmap_write(priv->regmap, MSG_RX_LIK_STS_2, 0x00000EEF);

	return 0;
}

static void an8855_pcs_an_restart(struct phylink_pcs *pcs)
{
	struct an8855_priv *priv = container_of(pcs, struct an8855_priv, pcs);

	regmap_set_bits(priv->regmap, SGMII_REG_AN0, SGMII_AN_RESTART);
}

static const struct phylink_pcs_ops an8855_pcs_ops = {
	.pcs_get_state = an8855_pcs_get_state,
	.pcs_config = an8855_pcs_config,
	.pcs_an_restart = an8855_pcs_an_restart,
};

static const struct phylink_mac_ops an8855_phylink_mac_ops = {
	.mac_select_pcs	= an8855_phylink_mac_select_pcs,
	.mac_config	= an8855_phylink_mac_config,
	.mac_link_down	= an8855_phylink_mac_link_down,
	.mac_link_up	= an8855_phylink_mac_link_up,
};

static const struct dsa_switch_ops an8855_switch_ops = {
	.get_tag_protocol = an8855_get_tag_protocol,
	.setup = an8855_setup,
	.get_strings = an8855_get_strings,
	// .phy_read = an8855_sw_phy_read,
	// .phy_write = an8855_sw_phy_write,
	.get_ethtool_stats = an8855_get_ethtool_stats,
	.get_sset_count = an8855_get_sset_count,
	.port_enable = an8855_port_enable,
	.port_disable = an8855_port_disable,
	.port_stp_state_set = an8855_stp_state_set,
	.port_bridge_join = an8855_port_bridge_join,
	.port_bridge_leave = an8855_port_bridge_leave,
	.port_fdb_add = an8855_port_fdb_add,
	.port_fdb_del = an8855_port_fdb_del,
	.port_fdb_dump = an8855_port_fdb_dump,
	.port_vlan_filtering = an8855_port_vlan_filtering,
	.port_vlan_add = an8855_port_vlan_add,
	.port_vlan_del = an8855_port_vlan_del,
	.port_mirror_add = an8855_port_mirror_add,
	.port_mirror_del = an8855_port_mirror_del,
	.phylink_get_caps = an8855_phylink_get_caps,
	.get_mac_eee = an8855_get_mac_eee,
	.set_mac_eee = an8855_set_mac_eee,
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
	priv->ds->phylink_mac_ops = &an8855_phylink_mac_ops;

	priv->pcs.ops = &an8855_pcs_ops;
	priv->pcs.neg_mode = true;
	priv->pcs.poll = true;

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
