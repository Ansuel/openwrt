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

// static void an8855_port_disable(struct dsa_switch *ds, int port)

static int an8855_fdb_cmd(struct an8855_priv *priv, u32 cmd, u32 *rsp)
{
	u32 val;
	int ret;

	/* Set the command operating upon the MAC address entries */
	val = ATC_BUSY | cmd;
	regmap_write(priv->regmap, AN8855_ATC, val);

	/* TODO */
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

static int an8855_setup(struct dsa_switch *ds)
{
	struct an8855_priv *priv = ds->priv;
	int ret;

	/* Enable and reset MIB counters */
	ret = an8855_mib_init(priv);
	if (ret)
		return ret;

	/* TODO */
	/* Disable forwarding by default on all ports */
	dsa_switch_for_each_port(dp, ds) {
		ret = regmap_update_bits(priv->regmap, AN8855_PORTMATRIX_P(dp->index),
					 PORTMATRIX_MASK, PORTMATRIX_CLR);
		if (ret)
			return ret;
	}

	// /* Disable MAC by default on all user ports */
	// dsa_switch_for_each_user_port(dp, ds)
	// 	qca8k_port_set_status(priv, dp->index, 0);

	/* Enable QCA header mode on all cpu ports */
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

static void an8855_pcs_get_state(struct phylink_pcs *pcs,
				 struct phylink_link_state *state)
{
	struct an8855_priv *priv = container_of(pcs, struct an8855_priv, pcs);
	u32 mcr;
	int ret;

	/* TODO FIND port */
	ret = regmap_read(priv->regmap, AN8855_PMSR_P(port), &mcr);
	if (ret < 0) {
		state->link = false;
		return;
	}

	state->link = !!(pmsr & PMSR_LINK);
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
	default:
		state->speed = SPEED_UNKNOWN;
		break;
	}

	if (pmsr & PMSR_RX_FC)
		state->pause |= MLO_PAUSE_RX;
	if (pmsr & PMSR_TX_FC)
		state->pause |= MLO_PAUSE_TX;
}

static int an8855_hsgmii_setup(struct an8855_priv *priv)
{


	return 0;
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
		else {
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
	    neg == PHYLINK_PCS_NEG_INBAND_ENABLED)) {
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
	// .get_tag_protocol = air_get_tag_protocol,
	.setup = an8855_setup,
	.get_strings = an8855_get_strings,
	// .phy_read = an8855_sw_phy_read,
	// .phy_write = an8855_sw_phy_write,
	.get_ethtool_stats = an8855_get_ethtool_stats,
	.get_sset_count = an8855_get_sset_count,
	// .port_enable = an8855_port_enable,
	// .port_disable = an8855_port_disable,
	.port_stp_state_set = an8855_stp_state_set,
	// .port_bridge_join = an8855_port_bridge_join,
	// .port_bridge_leave = an8855_port_bridge_leave,
	// .port_fdb_add = an8855_port_fdb_add,
	// .port_fdb_del = an8855_port_fdb_del,
	// .port_fdb_dump = an8855_port_fdb_dump,
	// .port_vlan_filtering = an8855_port_vlan_filtering,
	// .port_vlan_prepare = an8855_port_vlan_prepare,
	// .port_vlan_add = an8855_port_vlan_add,
	// .port_vlan_del = an8855_port_vlan_del,
	.port_mirror_add = an8855_port_mirror_add,
	.port_mirror_del = an8855_port_mirror_del,
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
