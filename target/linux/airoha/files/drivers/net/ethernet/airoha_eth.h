// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 Lorenzo Bianconi <lorenzo@kernel.org>
 */

#define AIROHA_MAX_NUM_RSTS		3
#define AIROHA_MAX_NUM_XSI_RSTS		4
#define AIROHA_MAX_MTU			2000
#define AIROHA_MAX_PACKET_SIZE		2048
#define AIROHA_NUM_TX_RING		32
#define AIROHA_NUM_RX_RING		32
#define AIROHA_FE_MC_MAX_VLAN_TABLE	64
#define AIROHA_FE_MC_MAX_VLAN_PORT	16
#define AIROHA_NUM_TX_IRQ		2
#define HW_DSCP_NUM			2048
#define IRQ_QUEUE_LEN(_n)		((_n) ? 1024 : 2048)
#define TX_DSCP_NUM			1024
#define RX_DSCP_NUM(_n)			\
	((_n) ==  2 ? 128 :		\
	 (_n) == 11 ? 128 :		\
	 (_n) == 15 ? 128 :		\
	 (_n) ==  0 ? 1024 : 16)

/* FE */
#define PSE_BASE			0x0100
#define CSR_IFC_BASE			0x0200
#define CDM1_BASE			0x0400
#define GDM1_BASE			0x0500
#define PPE1_BASE			0x0c00

#define CDM2_BASE			0x1400
#define GDM2_BASE			0x1500

#define GDM3_BASE			0x1100
#define GDM4_BASE			0x2400

#define REG_FE_DMA_GLO_CFG		0x0000
#define FE_DMA_GLO_L2_SPACE_MASK	GENMASK(7, 4)
#define FE_DMA_GLO_PG_SZ_MASK		BIT(3)

#define REG_FE_RST_GLO_CFG		0x0004
#define FE_RST_GDM4_MBI_ARB_MASK	BIT(3)
#define FE_RST_GDM3_MBI_ARB_MASK	BIT(2)
#define FE_RST_CORE_MASK		BIT(0)

#define REG_FE_LAN_MAC_H		0x0040
#define REG_FE_LAN_MAC_LMIN		0x0044
#define REG_FE_LAN_MAC_LMAX		0x0048

#define REG_FE_CDM1_OQ_MAP0		0x0050
#define REG_FE_CDM1_OQ_MAP1		0x0054
#define REG_FE_CDM1_OQ_MAP2		0x0058
#define REG_FE_CDM1_OQ_MAP3		0x005c

#define REG_FE_PCE_CFG			0x0070
#define PCE_DPI_EN			BIT(2)
#define PCE_KA_EN			BIT(1)
#define PCE_MC_EN			BIT(0)

#define PSE_PORT0_QUEUE			6
#define PSE_PORT1_QUEUE			6
#define PSE_PORT2_QUEUE			32
#define PSE_PORT3_QUEUE			6
#define PSE_PORT4_QUEUE			4
#define PSE_PORT5_QUEUE			6
#define PSE_PORT6_QUEUE			8
#define PSE_PORT7_QUEUE			10
#define PSE_PORT8_QUEUE			4
#define PSE_PORT9_QUEUE			2
#define PSE_PORT10_QUEUE		2
#define PSE_PORT11_QUEUE		0
#define PSE_PORT12_QUEUE		0
#define PSE_PORT13_QUEUE		0
#define PSE_PORT14_QUEUE		0
#define PSE_PORT15_QUEUE		0

#define REG_FE_PSE_QUEUE_CFG_WR		0x0080
#define PSE_CFG_PORT_ID_MASK		GENMASK(27, 24)
#define PSE_CFG_QUEUE_ID_MASK		GENMASK(20, 16)
#define PSE_CFG_WR_EN_MASK		BIT(8)
#define PSE_CFG_OQRSV_SEL_MASK		BIT(0)

#define REG_FE_PSE_QUEUE_CFG_VAL	0x0084
#define PSE_CFG_OQ_RSV_MASK		GENMASK(13, 0)

#define PSE_FQ_CFG			0x008c
#define PSE_FQ_LIMIT_MASK		GENMASK(14, 0)

#define REG_FE_PSE_BUF_SET		0x0090
#define PSE_SHARE_USED_LTHD_MASK	GENMASK(31, 16)
#define PSE_ALLRSV_MASK			GENMASK(14, 0)

#define REG_PSE_SHARE_USED_THD		0x0094
#define PSE_SHARE_USED_MTHD_MASK	GENMASK(31, 16)
#define PSE_SHARE_USED_HTHD_MASK	GENMASK(15, 0)

#define REG_GDM_MISC_CFG		0x0148
#define GDM2_RDM_ACK_WAIT_PREF_MASK	BIT(9)
#define GDM2_CHN_VLD_MODE_MASK		BIT(5)

#define REG_FE_CSR_IFC_CFG		CSR_IFC_BASE
#define FE_IFC_EN_MASK			BIT(0)

#define REG_FE_VIP_PORT_EN		0x01f0
#define REG_FE_IFC_PORT_EN		0x01f4

#define REG_PSE_IQ_REV1			(PSE_BASE + 0x08)
#define PSE_IQ_RES1_P2_MASK		GENMASK(23, 16)

#define REG_PSE_IQ_REV2			(PSE_BASE + 0x0c)
#define PSE_IQ_RES2_P5_MASK		GENMASK(15, 8)
#define PSE_IQ_RES2_P4_MASK		GENMASK(7, 0)

#define REG_FE_VIP_EN(_n)		(0x0300 + ((_n) << 3))
#define PATN_FCPU_EN_MASK		BIT(7)
#define PATN_SWP_EN_MASK		BIT(6)
#define PATN_DP_EN_MASK			BIT(5)
#define PATN_SP_EN_MASK			BIT(4)
#define PATN_TYPE_MASK			GENMASK(3, 1)
#define PATN_EN_MASK			BIT(0)

#define REG_FE_VIP_PATN(_n)		(0x0304 + ((_n) << 3))
#define PATN_DP_MASK			GENMASK(31, 16)
#define PATN_SP_MASK			GENMASK(15, 0)

#define REG_CDM1_VLAN_CTRL		CDM1_BASE
#define CDM1_VLAN_MASK			GENMASK(31, 16)

#define REG_CDM1_FWD_CFG		(CDM1_BASE + 0x08)
#define CDM1_VIP_QSEL_MASK		GENMASK(24, 20)

#define REG_CDM1_CRSN_QSEL(_n)		(CDM1_BASE + 0x10 + ((_n) << 2))
#define CDM1_CRSN_QSEL_REASON_MASK(_n)	\
	GENMASK(4 + (((_n) % 4) << 3), (((_n) % 4 ) << 3))

#define REG_CDM2_FWD_CFG		(CDM2_BASE + 0x08)
#define CDM2_OAM_QSEL_MASK		GENMASK(31, 27)
#define CDM2_VIP_QSEL_MASK		GENMASK(24, 20)

#define REG_CDM2_CRSN_QSEL(_n)		(CDM2_BASE + 0x10 + ((_n) << 2))
#define CDM2_CRSN_QSEL_REASON_MASK(_n)	\
	GENMASK(4 + (((_n) % 4) << 3), (((_n) % 4 ) << 3))

#define REG_GDM1_FWD_CFG		GDM1_BASE
#define GDM1_DROP_CRC_ERR		BIT(23)
#define GDM1_IP4_CKSUM			BIT(22)
#define GDM1_TCP_CKSUM			BIT(21)
#define GDM1_UDP_CKSUM			BIT(20)
#define GDM1_UCFQ_MASK			GENMASK(15, 12)
#define GDM1_BCFQ_MASK			GENMASK(11, 8)
#define GDM1_MCFQ_MASK			GENMASK(7, 4)
#define GDM1_OCFQ_MASK			GENMASK(3, 0)

#define REG_GDM1_INGRESS_CFG		(GDM1_BASE + 0x10)
#define GDM1_INGRESS_FC_EN_MASK		BIT(1)
#define GDM1_STAG_EN_MASK		BIT(0)

#define REG_GDM1_LEN_CFG		(GDM1_BASE + 0x14)
#define GDM1_SHORT_LEN_MASK		GENMASK(13, 0)
#define GDM1_LONG_LEN_MASK		GENMASK(29, 16)

#define REG_FE_CPORT_CFG		(GDM1_BASE + 0x40)
#define FE_CPORT_PAD			BIT(26)
#define FE_CPORT_PORT_XFC_MASK		BIT(25)
#define FE_CPORT_QUEUE_XFC_MASK		BIT(24)

#define REG_PPE1_TB_HASH_CFG		(PPE1_BASE + 0x250)
#define PPE1_SRAM_TABLE_EN_MASK		BIT(0)
#define PPE1_SRAM_HASH1_EN_MASK		BIT(8)
#define PPE1_DRAM_TABLE_EN_MASK		BIT(16)
#define PPE1_DRAM_HASH1_EN_MASK		BIT(24)

#define REG_GDM2_CHN_RLS		(GDM2_BASE + 0x20)
#define MBI_RX_AGE_SEL_MASK		GENMASK(18, 17)
#define MBI_TX_AGE_SEL_MASK		GENMASK(18, 17)

#define REG_GDM3_FWD_CFG		GDM3_BASE
#define GDM3_PAD_EN_MASK		BIT(28)

#define REG_GDM4_FWD_CFG		(GDM4_BASE + 0x100)
#define GDM4_PAD_EN_MASK		BIT(28)
#define GDM4_SPORT_OFFSET0_MASK		GENMASK(11, 8)

#define REG_GDM4_SRC_PORT_SET		(GDM4_BASE + 0x33c)
#define GDM4_SPORT_OFF2_MASK		GENMASK(19, 16)
#define GDM4_SPORT_OFF1_MASK		GENMASK(15, 12)
#define GDM4_SPORT_OFF0_MASK		GENMASK(11, 8)

#define REG_IP_FRAG_FP			0x2010
#define IP_ASSEMBLE_PORT_MASK		GENMASK(24, 21)
#define IP_ASSEMBLE_NBQ_MASK		GENMASK(20, 16)
#define IP_FRAGMENT_PORT_MASK		GENMASK(8, 5)
#define IP_FRAGMENT_NBQ_MASK		GENMASK(4, 0)

#define REG_MC_VLAN_EN			0x2100
#define MC_VLAN_EN_MASK			BIT(0)

#define REG_MC_VLAN_CFG			0x2104
#define MC_VLAN_CFG_CMD_DONE_MASK	BIT(31)
#define MC_VLAN_CFG_TABLE_ID_MASK	GENMASK(21, 16)
#define MC_VLAN_CFG_PORT_ID_MASK	GENMASK(11, 8)
#define MC_VLAN_CFG_TABLE_SEL_MASK	BIT(4)
#define MC_VLAN_CFG_RW_MASK		BIT(0)

#define REG_MC_VLAN_DATA		0x2108

#define REG_CDM5_RX_OQ1_DROP_CNT	0x29d4

/* QDMA */
#define REG_QDMA_GLOBAL_CFG		0x0004
#define GLOBAL_CFG_RX_2B_OFFSET		BIT(31)
#define GLOBAL_CFG_DMA_PREFERENCE_MASK	GENMASK(30, 29)
#define GLOBAL_CFG_CPU_TXR_ROUND_ROBIN	BIT(28)
#define GLOBAL_CFG_DSCP_BYTE_SWAP	BIT(27)
#define GLOBAL_CFG_PAYLOAD_BYTE_SWAP	BIT(26)
#define GLOBAL_CFG_MULTICAST_MODIFY_FP	BIT(25)
#define GLOBAL_CFG_OAM_MODIFY_MASK	BIT(24)
#define GLOBAL_CFG_RESET_MASK		BIT(23)
#define GLOBAL_CFG_RESET_DONE_MASK	BIT(22)
#define GLOBAL_CFG_MULTICAST_EN_MASK	BIT(21)
#define GLOBAL_CFG_IRQ1_EN		BIT(20)
#define GLOBAL_CFG_IRQ0_EN		BIT(19)
#define GLOBAL_CFG_LOOPCNT_EN		BIT(18)
#define GLOBAL_CFG_RD_BYPASS_WR		BIT(17)
#define GLOBAL_CFG_QDMA_LOOPBACK	BIT(16)
#define GLOBAL_CFG_LPBK_RXQ_SEL_MASK	GENMASK(13, 8)
#define GLOBAL_CFG_CHECK_DONE		BIT(7)
#define GLOBAL_CFG_TX_WB_DONE		BIT(6)
#define GLOBAL_CFG_MAX_ISSUE_NUM_MASK	GENMASK(5, 4)
#define GLOBAL_CFG_RX_DMA_BUSY		BIT(3)
#define GLOBAL_CFG_RX_DMA_EN		BIT(2)
#define GLOBAL_CFG_TX_DMA_BUSY		BIT(1)
#define GLOBAL_CFG_TX_DMA_EN		BIT(0)

#define REG_FWD_DSCP_BASE		0x0010
#define REG_FWD_BUF_BASE		0x0014

#define REG_HW_FWD_DSCP_CFG			0x0018
#define HW_FWD_DSCP_PAYLOAD_SIZE_MASK		GENMASK(29, 28)
#define HW_FWD_DSCP_SCATTER_LEN_MASK		GENMASK(17, 16)
#define HW_FWD_DSCP_MIN_SCATTER_LEN_MASK	GENMASK(15, 0)

#define REG_INT_STATUS(_n)		\
	(((_n) == 4) ? 0x0730 :		\
	 ((_n) == 3) ? 0x0724 :		\
	 ((_n) == 2) ? 0x0720 :		\
	 ((_n) == 1) ? 0x0024 : 0x0020)

#define REG_INT_ENABLE(_n)		\
	(((_n) == 4) ? 0x0750 :		\
	 ((_n) == 3) ? 0x0744 :		\
	 ((_n) == 2) ? 0x0740 :		\
	 ((_n) == 1) ? 0x002c : 0x0028)

/* QDMA_CSR_INT_ENABLE1 */
#define RX15_COHERENT_INT_MASK		BIT(31)
#define RX14_COHERENT_INT_MASK		BIT(30)
#define RX13_COHERENT_INT_MASK		BIT(29)
#define RX12_COHERENT_INT_MASK		BIT(28)
#define RX11_COHERENT_INT_MASK		BIT(27)
#define RX10_COHERENT_INT_MASK		BIT(26)
#define RX9_COHERENT_INT_MASK		BIT(25)
#define RX8_COHERENT_INT_MASK		BIT(24)
#define RX7_COHERENT_INT_MASK		BIT(23)
#define RX6_COHERENT_INT_MASK		BIT(22)
#define RX5_COHERENT_INT_MASK		BIT(21)
#define RX4_COHERENT_INT_MASK		BIT(20)
#define RX3_COHERENT_INT_MASK		BIT(19)
#define RX2_COHERENT_INT_MASK		BIT(18)
#define RX1_COHERENT_INT_MASK		BIT(17)
#define RX0_COHERENT_INT_MASK		BIT(16)
#define TX7_COHERENT_INT_MASK		BIT(15)
#define TX6_COHERENT_INT_MASK		BIT(14)
#define TX5_COHERENT_INT_MASK		BIT(13)
#define TX4_COHERENT_INT_MASK		BIT(12)
#define TX3_COHERENT_INT_MASK		BIT(11)
#define TX2_COHERENT_INT_MASK		BIT(10)
#define TX1_COHERENT_INT_MASK		BIT(9)
#define TX0_COHERENT_INT_MASK		BIT(8)
#define CNT_OVER_FLOW_INT_MASK		BIT(7)
#define IRQ1_FULL_INT_MASK		BIT(5)
#define IRQ1_INT_MASK			BIT(4)
#define HWFWD_DSCP_LOW_INT_MASK		BIT(3)
#define HWFWD_DSCP_EMPTY_INT_MASK	BIT(2)
#define IRQ0_FULL_INT_MASK		BIT(1)
#define IRQ0_INT_MASK			BIT(0)

#define TX_DONE_INT_MASK(_n)					\
	((_n) ? IRQ1_INT_MASK | IRQ1_FULL_INT_MASK		\
	      : IRQ0_INT_MASK | IRQ0_FULL_INT_MASK)

#define INT_TX_MASK						\
	(IRQ1_INT_MASK | IRQ1_FULL_INT_MASK |			\
	 IRQ0_INT_MASK | IRQ0_FULL_INT_MASK)

#define INT_IDX0_MASK						\
	(TX0_COHERENT_INT_MASK | TX1_COHERENT_INT_MASK |	\
	 TX2_COHERENT_INT_MASK | TX3_COHERENT_INT_MASK |	\
	 TX4_COHERENT_INT_MASK | TX5_COHERENT_INT_MASK |	\
	 TX6_COHERENT_INT_MASK | TX7_COHERENT_INT_MASK |	\
	 RX0_COHERENT_INT_MASK | RX1_COHERENT_INT_MASK |	\
	 RX2_COHERENT_INT_MASK | RX3_COHERENT_INT_MASK |	\
	 RX4_COHERENT_INT_MASK | RX7_COHERENT_INT_MASK |	\
	 RX8_COHERENT_INT_MASK | RX9_COHERENT_INT_MASK |	\
	 RX15_COHERENT_INT_MASK | INT_TX_MASK)

/* QDMA_CSR_INT_ENABLE2 */
#define RX15_NO_CPU_DSCP_INT_MASK	BIT(31)
#define RX14_NO_CPU_DSCP_INT_MASK	BIT(30)
#define RX13_NO_CPU_DSCP_INT_MASK	BIT(29)
#define RX12_NO_CPU_DSCP_INT_MASK	BIT(28)
#define RX11_NO_CPU_DSCP_INT_MASK	BIT(27)
#define RX10_NO_CPU_DSCP_INT_MASK	BIT(26)
#define RX9_NO_CPU_DSCP_INT_MASK	BIT(25)
#define RX8_NO_CPU_DSCP_INT_MASK	BIT(24)
#define RX7_NO_CPU_DSCP_INT_MASK	BIT(23)
#define RX6_NO_CPU_DSCP_INT_MASK	BIT(22)
#define RX5_NO_CPU_DSCP_INT_MASK	BIT(21)
#define RX4_NO_CPU_DSCP_INT_MASK	BIT(20)
#define RX3_NO_CPU_DSCP_INT_MASK	BIT(19)
#define RX2_NO_CPU_DSCP_INT_MASK	BIT(18)
#define RX1_NO_CPU_DSCP_INT_MASK	BIT(17)
#define RX0_NO_CPU_DSCP_INT_MASK	BIT(16)
#define RX15_DONE_INT_MASK		BIT(15)
#define RX14_DONE_INT_MASK		BIT(14)
#define RX13_DONE_INT_MASK		BIT(13)
#define RX12_DONE_INT_MASK		BIT(12)
#define RX11_DONE_INT_MASK		BIT(11)
#define RX10_DONE_INT_MASK		BIT(10)
#define RX9_DONE_INT_MASK		BIT(9)
#define RX8_DONE_INT_MASK		BIT(8)
#define RX7_DONE_INT_MASK		BIT(7)
#define RX6_DONE_INT_MASK		BIT(6)
#define RX5_DONE_INT_MASK		BIT(5)
#define RX4_DONE_INT_MASK		BIT(4)
#define RX3_DONE_INT_MASK		BIT(3)
#define RX2_DONE_INT_MASK		BIT(2)
#define RX1_DONE_INT_MASK		BIT(1)
#define RX0_DONE_INT_MASK		BIT(0)

#define RX_DONE_INT_MASK					\
	(RX0_DONE_INT_MASK | RX1_DONE_INT_MASK |		\
	 RX2_DONE_INT_MASK | RX3_DONE_INT_MASK |		\
	 RX4_DONE_INT_MASK | RX7_DONE_INT_MASK |		\
	 RX8_DONE_INT_MASK | RX9_DONE_INT_MASK |		\
	 RX15_DONE_INT_MASK)
#define INT_IDX1_MASK						\
	(RX_DONE_INT_MASK |					\
	 RX0_NO_CPU_DSCP_INT_MASK | RX1_NO_CPU_DSCP_INT_MASK |	\
	 RX2_NO_CPU_DSCP_INT_MASK | RX3_NO_CPU_DSCP_INT_MASK |	\
	 RX4_NO_CPU_DSCP_INT_MASK | RX7_NO_CPU_DSCP_INT_MASK |	\
	 RX8_NO_CPU_DSCP_INT_MASK | RX9_NO_CPU_DSCP_INT_MASK |	\
	 RX15_NO_CPU_DSCP_INT_MASK)

/* QDMA_CSR_INT_ENABLE5 */
#define TX31_COHERENT_INT_MASK		BIT(31)
#define TX30_COHERENT_INT_MASK		BIT(30)
#define TX29_COHERENT_INT_MASK		BIT(29)
#define TX28_COHERENT_INT_MASK		BIT(28)
#define TX27_COHERENT_INT_MASK		BIT(27)
#define TX26_COHERENT_INT_MASK		BIT(26)
#define TX25_COHERENT_INT_MASK		BIT(25)
#define TX24_COHERENT_INT_MASK		BIT(24)
#define TX23_COHERENT_INT_MASK		BIT(23)
#define TX22_COHERENT_INT_MASK		BIT(22)
#define TX21_COHERENT_INT_MASK		BIT(21)
#define TX20_COHERENT_INT_MASK		BIT(20)
#define TX19_COHERENT_INT_MASK		BIT(19)
#define TX18_COHERENT_INT_MASK		BIT(18)
#define TX17_COHERENT_INT_MASK		BIT(17)
#define TX16_COHERENT_INT_MASK		BIT(16)
#define TX15_COHERENT_INT_MASK		BIT(15)
#define TX14_COHERENT_INT_MASK		BIT(14)
#define TX13_COHERENT_INT_MASK		BIT(13)
#define TX12_COHERENT_INT_MASK		BIT(12)
#define TX11_COHERENT_INT_MASK		BIT(11)
#define TX10_COHERENT_INT_MASK		BIT(10)
#define TX9_COHERENT_INT_MASK		BIT(9)
#define TX8_COHERENT_INT_MASK		BIT(8)

#define INT_IDX4_MASK						\
	(TX8_COHERENT_INT_MASK | TX9_COHERENT_INT_MASK |	\
	 TX10_COHERENT_INT_MASK | TX11_COHERENT_INT_MASK |	\
	 TX12_COHERENT_INT_MASK | TX13_COHERENT_INT_MASK |	\
	 TX14_COHERENT_INT_MASK | TX15_COHERENT_INT_MASK |	\
	 TX16_COHERENT_INT_MASK | TX17_COHERENT_INT_MASK |	\
	 TX18_COHERENT_INT_MASK | TX19_COHERENT_INT_MASK |	\
	 TX20_COHERENT_INT_MASK | TX21_COHERENT_INT_MASK |	\
	 TX20_COHERENT_INT_MASK | TX21_COHERENT_INT_MASK |	\
	 TX22_COHERENT_INT_MASK | TX23_COHERENT_INT_MASK |	\
	 TX24_COHERENT_INT_MASK | TX25_COHERENT_INT_MASK |	\
	 TX26_COHERENT_INT_MASK | TX27_COHERENT_INT_MASK |	\
	 TX28_COHERENT_INT_MASK | TX29_COHERENT_INT_MASK |	\
	 TX30_COHERENT_INT_MASK | TX31_COHERENT_INT_MASK)

#define REG_TX_IRQ_BASE(_n)		((_n) ? 0x0048 : 0x0050)

#define REG_TX_IRQ_CFG(_n)		((_n) ? 0x004c : 0x0054)
#define TX_IRQ_THR_MASK			GENMASK(27, 16)
#define TX_IRQ_DEPTH_MASK		GENMASK(11, 0)

#define REG_IRQ_CLEAR_LEN(_n)		((_n) ? 0x0064 : 0x0058)
#define IRQ_CLEAR_LEN_MASK		GENMASK(7, 0)

#define REG_IRQ_STATUS(_n)		((_n) ? 0x0068 : 0x005c)
#define IRQ_ENTRY_LEN_MASK		GENMASK(27, 16)
#define IRQ_HEAD_IDX_MASK		GENMASK(11, 0)

#define REG_TX_RING_BASE(_n)	\
	(((_n) < 8) ? 0x0100 + ((_n) << 5) : 0x0b00 + (((_n) - 8) << 5))

#define REG_TX_RING_BLOCKING(_n)	\
	(((_n) < 8) ? 0x0104 + ((_n) << 5) : 0x0b04 + (((_n) - 8) << 5))

#define TX_RING_IRQ_BLOCKING_MAP_MASK			BIT(6)
#define TX_RING_IRQ_BLOCKING_CFG_MASK			BIT(4)
#define TX_RING_IRQ_BLOCKING_TX_DROP_EN_MASK		BIT(2)
#define TX_RING_IRQ_BLOCKING_MAX_TH_TXRING_EN_MASK	BIT(1)
#define TX_RING_IRQ_BLOCKING_MIN_TH_TXRING_EN_MASK	BIT(0)

#define REG_TX_CPU_IDX(_n)	\
	(((_n) < 8) ? 0x0108 + ((_n) << 5) : 0x0b08 + (((_n) - 8) << 5))

#define TX_RING_CPU_IDX_MASK		GENMASK(15, 0)

#define REG_TX_DMA_IDX(_n)	\
	(((_n) < 8) ? 0x010c + ((_n) << 5) : 0x0b0c + (((_n) - 8) << 5))

#define TX_RING_DMA_IDX_MASK		GENMASK(15, 0)

#define IRQ_RING_IDX_MASK		GENMASK(20, 16)
#define IRQ_DESC_IDX_MASK		GENMASK(15, 0)

#define REG_RX_RING_BASE(_n)	\
	(((_n) < 16) ? 0x0200 + ((_n) << 5) : 0x0e00 + (((_n) - 16) << 5))

#define REG_RX_RING_SIZE(_n)	\
	(((_n) < 16) ? 0x0204 + ((_n) << 5) : 0x0e04 + (((_n) - 16) << 5))

#define RX_RING_THR_MASK		GENMASK(31, 16)
#define RX_RING_SIZE_MASK		GENMASK(15, 0)

#define REG_RX_CPU_IDX(_n)	\
	(((_n) < 16) ? 0x0208 + ((_n) << 5) : 0x0e08 + (((_n) - 16) << 5))

#define RX_RING_CPU_IDX_MASK		GENMASK(15, 0)

#define REG_RX_DMA_IDX(_n)	\
	(((_n) < 16) ? 0x020c + ((_n) << 5) : 0x0e0c + (((_n) - 16) << 5))

#define REG_RX_DELAY_INT_IDX(_n)	\
	(((_n) < 16) ? 0x0210 + ((_n) << 5) : 0x0e10 + (((_n) - 16) << 5))

#define RX_DELAY_INT_MASK		GENMASK(15, 0)

#define RX_RING_DMA_IDX_MASK		GENMASK(15, 0)

#define REG_INGRESS_TRTCM_CFG		0x0070
#define INGRESS_TRTCM_EN_MASK		BIT(31)
#define INGRESS_TRTCM_MODE_MASK		BIT(30)
#define INGRESS_SLOW_TICK_RATIO_MASK	GENMASK(29, 16)
#define INGRESS_FAST_TICK_MASK		GENMASK(15, 0)

#define REG_TXQ_DIS_CFG_BASE(_n)	((_n) ? 0x20a0 : 0x00a0)
#define REG_TXQ_DIS_CFG(_n, _m)		(REG_TXQ_DIS_CFG_BASE((_n)) + (_m) << 2)

#define REG_LMGR_INIT_CFG		0x1000
#define LMGR_INIT_START			BIT(31)
#define LMGR_SRAM_MODE_MASK		BIT(30)
#define HW_FWD_PKTSIZE_OVERHEAD_MASK	GENMASK(27, 20)
#define HW_FWD_DESC_NUM_MASK		GENMASK(16, 0)

#define REG_FWD_DSCP_LOW_THR		0x1004
#define FWD_DSCP_LOW_THR_MASK		GENMASK(17, 0)

#define REG_EGRESS_RATE_METER_CFG		0x100c
#define EGRESS_RATE_METER_EN_MASK		BIT(29)
#define EGRESS_RATE_METER_EQ_RATE_EN_MASK	BIT(17)
#define EGRESS_RATE_METER_WINDOW_SZ_MASK	GENMASK(16, 12)
#define EGRESS_RATE_METER_TIMESLICE_MASK	GENMASK(10, 0)

#define REG_EGRESS_TRTCM_CFG		0x1010
#define EGRESS_TRTCM_EN_MASK		BIT(31)
#define EGRESS_TRTCM_MODE_MASK		BIT(30)
#define EGRESS_SLOW_TICK_RATIO_MASK	GENMASK(29, 16)
#define EGRESS_FAST_TICK_MASK		GENMASK(15, 0)

#define REG_TXWRR_MODE_CFG		0x1020
#define TWRR_WEIGHT_SCALE_MASK		BIT(31)
#define TWRR_WEIGHT_BASE_MASK		BIT(3)

#define REG_PSE_BUF_USAGE_CFG		0x1028
#define PSE_BUF_ESTIMATE_EN_MASK	BIT(29)

#define REG_GLB_TRTCM_CFG		0x1080
#define GLB_TRTCM_EN_MASK		BIT(31)
#define GLB_TRTCM_MODE_MASK		BIT(30)
#define GLB_SLOW_TICK_RATIO_MASK	GENMASK(29, 16)
#define GLB_FAST_TICK_MASK		GENMASK(15, 0)

#define REG_TXQ_CNGST_CFG		0x10a0
#define TXQ_CNGST_DROP_EN		BIT(31)
#define TXQ_CNGST_DEI_DROP_EN		BIT(30)

#define REG_SLA_TRTCM_CFG		0x1150
#define SLA_TRTCM_EN_MASK		BIT(31)
#define SLA_TRTCM_MODE_MASK		BIT(30)
#define SLA_SLOW_TICK_RATIO_MASK	GENMASK(29, 16)
#define SLA_FAST_TICK_MASK		GENMASK(15, 0)

/* CTRL */
#define QDMA_DESC_DONE_MASK		BIT(31)
#define QDMA_DESC_DROP_MASK		BIT(30) /* tx: drop pkt - rx: overflow */
#define QDMA_DESC_MORE_MASK		BIT(29) /* more SG elements */
#define QDMA_DESC_DEI_MASK		BIT(25)
#define QDMA_DESC_NO_DROP_MASK		BIT(24)
#define QDMA_DESC_LEN_MASK		GENMASK(15, 0)
/* DATA */
#define QDMA_DESC_NEXT_ID_MASK		GENMASK(15, 0)
/* MSG0 */
#define QDMA_ETH_TXMSG_MIC_IDX_MASK	BIT(30)
#define QDMA_ETH_TXMSG_SP_TAG_MASK	GENMASK(29, 14)
#define QDMA_ETH_TXMSG_ICO_MASK		BIT(13)
#define QDMA_ETH_TXMSG_UCO_MASK		BIT(12)
#define QDMA_ETH_TXMSG_TCO_MASK		BIT(11)
#define QDMA_ETH_TXMSG_TSO_MASK		BIT(10)
#define QDMA_ETH_TXMSG_FAST_MASK	BIT(9)
#define QDMA_ETH_TXMSG_OAM_MASK		BIT(8)
#define QDMA_ETH_TXMSG_CHAN_MASK	GENMASK(7, 3)
#define QDMA_ETH_TXMSG_QUEUE_MASK	GENMASK(2, 0)
/* MSG1 */
#define QDMA_ETH_TXMSG_NO_DROP		BIT(31)
#define QDMA_ETH_TXMSG_METER_MASK	GENMASK(30, 24)	/* 0x7f means do not apply meters */
#define QDMA_ETH_TXMSG_FPORT_MASK	GENMASK(23, 20)
#define QDMA_ETH_TXMSG_NBOQ_MASK	GENMASK(19, 15)
#define QDMA_ETH_TXMSG_HWF_MASK		BIT(14)
#define QDMA_ETH_TXMSG_HOP_MASK		BIT(13)
#define QDMA_ETH_TXMSG_PTP_MASK		BIT(12)
#define QDMA_ETH_TXMSG_ACNT_G1_MASK	GENMASK(10, 6)	/* 0x1f means do not count */
#define QDMA_ETH_TXMSG_ACNT_G0_MASK	GENMASK(5, 0)	/* 0x3f means do not count */

struct airoha_qdma_desc {
	__le32 rsv;
	__le32 ctrl;
	__le32 addr;
	__le32 data;
	__le32 msg0;
	__le32 msg1;
	__le32 msg2;
	__le32 msg3;
};

/* CTRL0 */
#define QDMA_FWD_DESC_CTX_MASK		BIT(31)
#define QDMA_FWD_DESC_RING_MASK		GENMASK(30, 28)
#define QDMA_FWD_DESC_IDX_MASK		GENMASK(27, 16)
#define QDMA_FWD_DESC_LEN_MASK		GENMASK(15, 0)
/* CTRL1 */
#define QDMA_FWD_DESC_FIRST_IDX_MASK	GENMASK(15, 0)
/* CTRL2 */
#define QDMA_FWD_DESC_MORE_PKT_NUM_MASK	GENMASK(2, 0)

struct airoha_qdma_fwd_desc {
	__le32 addr;
	__le32 ctrl0;
	__le32 ctrl1;
	__le32 ctrl2;
	__le32 msg0;
	__le32 msg1;
	__le32 rsv0;
	__le32 rsv1;
};

enum {
	QDMA_INT_REG_IDX0,
	QDMA_INT_REG_IDX1,
	QDMA_INT_REG_IDX2,
	QDMA_INT_REG_IDX3,
	QDMA_INT_REG_IDX4,
	QDMA_INT_REG_MAX
};

enum airoha_dport {
	DPORT_PDMA,
	DPORT_GDM1,
	DPORT_GDM2,
	DPORT_GDM3,
	DPORT_PPE,
	DPORT_QDMA,
	DPORT_QDMA_HW,
	DPORT_DISCARD,
	DPORT_GDM4 = 9,
};

enum {
	FE_DP_CPU,
	FE_DP_GDM1,
	FE_DP_GDM2,
	FE_DP_QDMA1_HWF,
	FE_DP_GDMA3_HWF = 3,
	FE_DP_PPE,
	FE_DP_QDMA2_CPU,
	FE_DP_QDMA2_HWF,
	FE_DP_DISCARD,
	FE_DP_PPE2 = 8,
	FE_DP_DROP = 15,
};

enum {
	CDM_CRSN_QSEL_Q1 = 1,
	CDM_CRSN_QSEL_Q5 = 5,
	CDM_CRSN_QSEL_Q6 = 6,
	CDM_CRSN_QSEL_Q15 = 15,
};

enum {
	CRSN_08 = 0x8,
	CRSN_21 = 0x15, /* KA */
	CRSN_22 = 0x16, /* hit bind and force route to CPU */
	CRSN_24 = 0x18,
	CRSN_25 = 0x19,
};

enum {
	DEV_STATE_INITIALIZED,
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
	struct airoha_eth *eth;

	spinlock_t lock;
	struct airoha_queue_entry *entry;
	struct airoha_qdma_desc *desc;
	u16 head;
	u16 tail;

	int queued;
	int ndesc;
	int free_thr;
	int buf_size;

	struct napi_struct napi;
	struct page_pool *page_pool;
};

struct airoha_tx_irq_queue {
	struct airoha_eth *eth;

	struct napi_struct napi;
	u32 *q;

	int size;
	int queued;
	u16 head;
};

struct airoha_eth {
	struct net_device *net_dev;

	unsigned long state;

	void __iomem *qdma_regs;
	void __iomem *fe_regs;

	spinlock_t irq_lock;
	u32 irqmask[QDMA_INT_REG_MAX];
	int irq;

	struct reset_control_bulk_data rsts[AIROHA_MAX_NUM_RSTS];
	struct reset_control_bulk_data xsi_rsts[AIROHA_MAX_NUM_XSI_RSTS];

	struct airoha_queue q_tx[AIROHA_NUM_TX_RING];
	struct airoha_queue q_rx[AIROHA_NUM_RX_RING];

	struct airoha_tx_irq_queue q_tx_irq[AIROHA_NUM_TX_IRQ];

	/* descriptor and packet buffers for qdma hw forward */
	struct {
		void *desc;
		void *q;
	} hfwd;

	struct dentry *debugfs_dir;
	u32 debugfs_reg;
};

#define airoha_qdma_for_each_q_rx(eth, i)		\
	for (i = 0; i < ARRAY_SIZE((eth)->q_rx); i++)	\
		if ((eth)->q_rx[i].ndesc)

static inline void airoha_qdma_start_napi(struct airoha_eth *eth)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(eth->q_tx_irq); i++)
		napi_enable(&eth->q_tx_irq[i].napi);

	airoha_qdma_for_each_q_rx(eth, i)
		napi_enable(&eth->q_rx[i].napi);
}

static inline void airoha_qdma_stop_napi(struct airoha_eth *eth)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(eth->q_tx_irq); i++)
		napi_disable(&eth->q_tx_irq[i].napi);

	airoha_qdma_for_each_q_rx(eth, i)
		napi_disable(&eth->q_rx[i].napi);
}
