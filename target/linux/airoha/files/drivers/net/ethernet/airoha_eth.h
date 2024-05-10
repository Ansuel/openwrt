// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 Lorenzo Bianconi <lorenzo@kernel.org>
 */

#define AIROHA_MAX_NUM_RSTS	3
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
#define REG_FWD_DSCP_BASE		0x0010
#define REG_FWD_BUF_BASE		0x0014

#define REG_HW_FWD_DSCP_CFG		0x0018
#define HW_FWD_DSCP_PAYLOAD_SIZE_MASK	GENMASK(29, 28)

#define REG_TX_IRQ_BASE			0x0050

#define REG_TX_IRQ_CFG			0x0054
#define TX_IRQ_DEPTH_MASK		GENMASK(11, 0)
#define TX_IRQ_THR_MASK			GENMASK(27, 16)

#define REG_TX_RING_BASE(idx)		(0x0100 + (idx << 5))

#define REG_TX_CPU_IDX(idx)		(0x0108 + (idx << 5))
#define TX_RING_CPU_IDX_MASK		GENMASK(15, 0)

#define REG_TX_DMA_IDX(idx)		(0x010c + (idx << 5))
#define TX_RING_DMA_IDX_MASK		GENMASK(15, 0)

#define REG_RX_RING_BASE(idx)		(0x0200 + (idx << 5))

#define REG_RX_RING_SIZE(idx)		(0x0204 + (idx << 5))
#define RX_RING_THR_MASK		GENMASK(31, 16)
#define RX_RING_SIZE_MASK		GENMASK(15, 0)

#define REG_RX_CPU_IDX(idx)		(0x0208 + (idx << 5))
#define RX_RING_CPU_IDX_MASK		GENMASK(15, 0)

#define REG_RX_DMA_IDX(idx)		(0x020c + (idx << 5))
#define RX_RING_DMA_IDX_MASK		GENMASK(15, 0)

#define REG_LMGR_INIT_CFG		0x1000
#define HW_FWD_DESC_NUM_MASK		GENMASK(16, 0)
#define HW_FWD_PKTSIZE_OVERHEAD_MASK	GENMASK(27, 20)
#define LGMR_INIT_START			BIT(31)

#define REG_FWD_DSCP_LOW_THR		0x1004
#define FWD_DSCP_LOW_THR_MASK		GENMASK(17, 0)

#define TX0_DSCP_NUM			1536
#define TX1_DSCP_NUM			128
#define RX0_DSCP_NUM			1024
#define RX1_DSCP_NUM			1024
#define HW_DSCP_NUM			256
#define MAX_PKT_LEN			2048

/* DW0 */
#define QDMA_DESC_LEN_MASK		GENMASK(15, 0)
#define QDMA_DESC_NO_DROP_MASK		BIT(24)
#define QDMA_DESC_DEI_MASK		BIT(25)
#define QDMA_DESC_MORE_MASK		BIT(29) /* more SG elements */
#define QDMA_DESC_DROP_MASK		BIT(30) /* tx: drop pkt - rx: overflow */
#define QDMA_DESC_DONE_MASK		BIT(31)
/* DW2 */
#define QDMA_DESC_NEXT_ID_MASK		GENMASK(15, 0)

struct airoha_qdma_desc {
	__le32 ctrl;
	__le32 addr;
	__le32 data;
	__le32 msg0;
	__le32 msg1;
	__le32 msg2;
	__le32 msg3;
};

/* DW1 */
#define QDMA_FWD_DESC_LEN_MASK		GENMASK(15, 0)
#define QDMA_FWD_DESC_IDX_MASK		GENMASK(27, 16)
#define QDMA_FWD_DESC_RING_MASK		GENMASK(30, 28)
#define QDMA_FWD_DESC_CTX_MASK		BIT(31)
/* DW2 */
#define QDMA_FWD_DESC_FIRST_IDX_MASK	GENMASK(15, 0)
/* DW3 */
#define QDMA_FWD_DESC_MORE_PKT_NUM_MASK	GENMASK(2, 0)

struct airoha_qdma_fwd_desc {
	__le32 addr;
	__le32 ctrl0;
	__le32 ctrl1;
	__le32 ctrl2;
	__le32 msg0;
	__le32 msg1;
	__le32 rsv2;
	__le32 rsv3;
};

struct airoha_queue_entry {
	union {
		void *buf;
		struct sk_buff *skb;
	};
	dma_addr_t dma_addr;
	u16 dma_len;
};

struct airoha_queue {
	spinlock_t lock;
	struct airoha_queue_entry *entry;
	struct airoha_qdma_desc *desc;
	u16 head;
	u16 tail;

	int queued;
	int ndesc;
	int buf_size;

	struct page_pool *page_pool;
};

struct airoha_eth {
	struct device *dev;

	void __iomem *qdma_regs;
	void __iomem *fe_regs;

	struct reset_control_bulk_data resets[AIROHA_MAX_NUM_RSTS];

	struct phylink *phylink;
	struct phylink_config phylink_config;
	phy_interface_t interface;
	int speed;

	struct airoha_queue q_xmit[2];
	struct airoha_queue q_rx[2];

	void *irq_q;
	void *hfwd_desc;
	void *hfwd_q;
};
