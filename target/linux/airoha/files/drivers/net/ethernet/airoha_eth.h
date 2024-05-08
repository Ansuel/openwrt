// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 Lorenzo Bianconi <lorenzo@kernel.org>
 */

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
#define REG_TX_RING_BASE(idx)		(0x0100 + (idx << 5))

#define REG_RX_RING_BASE(idx)		(0x0200 + (idx << 5))

#define REG_RX_RING_SIZE(idx)		(0x0204 + (idx << 5))
#define RX_RING_THR_MASK		GENMASK(31, 16)
#define RX_RING_SIZE_MASK		GENMASK(15, 0)

#define REG_RX_CPU_IDX(idx)		(0x0208 + (idx << 5))
#define RX_RING_CPU_IDX_MASK		GENMASK(15, 0)

#define REG_RX_DMA_IDX(idx)		(0x020c + (idx << 5))
#define RX_RING_DMA_IDX_MASK		GENMASK(15, 0)

#define REG_LMGR_INIT_CFG		0x1000
#define HWFWD_PKTSIZE_OVERHEAD_MASK	GENMASK(27, 20)

#define TX0_DSCP_NUM			1536
#define TX1_DSCP_NUM			128
#define RX0_DSCP_NUM			1024
#define RX1_DSCP_NUM			1024

struct airoha_qdma_desc {
	u32 len		: 16;
	u32 rsv0	: 8;
	u32 no_drop	: 1;
	u32 dei		: 1;
	u32 rsv1	: 3;
	u32 more	: 1; /* more SG elements */
	u32 drop	: 1; /* tx: drop pkt
			      * rx: overflow
			      */
	u32 done	: 1;
	u32 addr;
	u32 next_idx	: 16;
	u32 rsv2	: 16;
	u32 data[4];
};

struct airoha_queue_entry {
	void *buf;
};

struct airoha_queue {
	spinlock_t lock;
	struct airoha_queue_entry *entry;
	struct airoha_qdma_desc *desc;
	u16 head;
	u16 tail;
	int ndesc;
};

struct airoha_eth {
	struct device *dev;

	void __iomem *qdma_regs;
	void __iomem *fe_regs;

	struct phylink *phylink;
	struct phylink_config phylink_config;
	phy_interface_t interface;
	int speed;

	struct airoha_queue q_xmit[2];
	struct airoha_queue q_rx[2];
};
