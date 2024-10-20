/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2023 Min Yao <min.yao@airoha.com>
 * Copyright (C) 2024 Christian Marangi <ansuelsmth@gmail.com>
 */

#ifndef __AN8855_H
#define __AN8855_H

#include <linux/bitfield.h>

#define AN8855_NUM_PORTS		6
#define AN8855_CPU_PORT			5
#define AN8855_NUM_FDB_RECORDS		2048
#define AN8855_GPHY_SMI_ADDR_DEFAULT	1
#define AN8855_PORT_VID_DEFAULT		1

// #define AN8855_RESERVED_VLAN			2
// #define AN8855_DFL_INTR_ID				0xd

/*	AN8855_SCU			0x10000000 */
#define AN8855_RG_GPIO_LED_MODE		0x10000054
#define AN8855_RG_GPIO_LED_SEL(i)	(0x10000000 + (0x0058 + ((i) * 4)))
#define AN8855_RG_INTB_MODE		0x10000080
#define AN8855_RG_RGMII_TXCK_C		0x100001d0

#define AN8855_PKG_SEL			0x10000094
#define   AN8855_PAG_SEL_AN8855H	0x2

/* Register for hw trap status */
#define AN8855_HWTRAP			0x1000009c

#define AN8855_RG_GPIO_L_INV		0x10000010
#define AN8855_RG_GPIO_CTRL		0x1000a300
#define AN8855_RG_GPIO_DATA		0x1000a304
#define AN8855_RG_GPIO_OE		0x1000a314

#define AN8855_EFUSE_DATA0		0x1000a500
#define   AN8855_EFUSE_R50O		GENMASK(30, 24)

#define AN8855_CREV			0x10005000
#define	  AN8855_ID			0x8855

/* Register for system reset */
#define AN8855_RST_CTRL			0x100050c0
#define	  AN8855_SYS_CTRL_SYS_RST	BIT(31)

#define AN8855_INT_MASK			0x100050f0
#define   AN8855_INT_SYS		BIT(15)

#define AN8855_RG_CLK_CPU_ICG		0x10005034
#define   AN8855_MCU_ENABLE		BIT(3)

#define AN8855_RG_TIMER_CTL		0x1000a100
#define   AN8855_WDOG_ENABLE		BIT(25)

#define AN8855_RG_GDMP_RAM		0x10010000

/* Registers to mac forward control for unknown frames */
#define AN8855_MFC			0x10200010
#define	  AN8855_CPU_EN			BIT(15)
#define	  AN8855_CPU_PORT_IDX		GENMASK(12, 8)

#define AN8855_UNUF			0x102000b4
#define AN8855_UNMF			0x102000b8
#define AN8855_BCF			0x102000bc

/* Registers for mirror port control */
#define	AN8855_MIR			0x102000cc
#define	  AN8855_MIRROR_EN		BIT(7)
#define	  AN8855_MIRROR_PORT		GENMASK(4, 0)

/* Registers for BPDU and PAE frame control*/
#define AN8855_BPC			0x102000D0
#define	  AN8855_BPDU_PORT_FW		GENMASK(2, 0)

enum an8855_bpdu_port_fw {
	AN8855_BPDU_FOLLOW_MFC = 0,
	AN8855_BPDU_CPU_EXCLUDE = 4,
	AN8855_BPDU_CPU_INCLUDE = 5,
	AN8855_BPDU_CPU_ONLY = 6,
	AN8855_BPDU_DROP = 7,
};

/* Register for address table control */
#define AN8855_ATC			0x10200300
#define	  AN8855_ATC_BUSY		BIT(31)
#define	  AN8855_ATC_HASH		GENMASK(24, 16)
#define	  AN8855_ATC_HIT		GENMASK(15, 12)
#define	  AN8855_ATC_MAT_MASK		GENMASK(11, 7)
#define	  AN8855_ATC_MAT(x)		FIELD_PREP(AN8855_ATC_MAT_MASK, x)
#define	  AN8855_ATC_SAT		GENMASK(5, 4)
#define	  AN8855_ATC_CMD		GENMASK(2, 0)

enum an8855_fdb_mat_cmds {
	AND8855_FDB_MAT_ALL = 0,
	AND8855_FDB_MAT_MAC, /* All MAC address */
	AND8855_FDB_MAT_DYNAMIC_MAC, /* All Dynamic MAC address */
	AND8855_FDB_MAT_STATIC_MAC, /* All Static Mac Address */
	AND8855_FDB_MAT_DIP, /* All DIP/GA address */
	AND8855_FDB_MAT_DIP_IPV4, /* All DIP/GA IPv4 address */
	AND8855_FDB_MAT_DIP_IPV6, /* All DIP/GA IPv6 address */
	AND8855_FDB_MAT_DIP_SIP, /* All DIP_SIP address */
	AND8855_FDB_MAT_DIP_SIP_IPV4, /* All DIP_SIP IPv4 address */
	AND8855_FDB_MAT_DIP_SIP_IPV6, /* All DIP_SIP IPv6 address */
	AND8855_FDB_MAT_MAC_CVID, /* All MAC address with CVID */
	AND8855_FDB_MAT_MAC_FID, /* All MAC address with Filter ID */
	AND8855_FDB_MAT_MAC_PORT, /* All MAC address with port */
	AND8855_FDB_MAT_DIP_SIP_DIP_IPV4, /* All DIP_SIP address with DIP_IPV4 */
	AND8855_FDB_MAT_DIP_SIP_SIP_IPV4, /* All DIP_SIP address with SIP_IPV4 */
	AND8855_FDB_MAT_DIP_SIP_DIP_IPV6, /* All DIP_SIP address with DIP_IPV6 */
	AND8855_FDB_MAT_DIP_SIP_SIP_IPV6, /* All DIP_SIP address with SIP_IPV6 */
	/* All MAC address with MAC type (dynamic or static) with CVID */
	AND8855_FDB_MAT_MAC_TYPE_CVID,
	/* All MAC address with MAC type (dynamic or static) with Filter ID */
	AND8855_FDB_MAT_MAC_TYPE_FID,
	/* All MAC address with MAC type (dynamic or static) with port */
	AND8855_FDB_MAT_MAC_TYPE_PORT,
};

enum an8855_fdb_cmds {
	AN8855_FDB_READ = 0,
	AN8855_FDB_WRITE = 1,
	AN8855_FDB_FLUSH = 2,
	AN8855_FDB_START = 4,
	AN8855_FDB_NEXT = 5,
};

/* Registers for address table access */
#define AN8855_ATA1			0x10200304
#define   AN8855_ATA1_MAC0		GENMASK(31, 24)
#define   AN8855_ATA1_MAC1		GENMASK(23, 16)
#define   AN8855_ATA1_MAC2		GENMASK(15, 8)
#define   AN8855_ATA1_MAC3		GENMASK(7, 0)
#define AN8855_ATA2			0x10200308
#define   AN8855_ATA2_MAC4		GENMASK(31, 24)
#define   AN8855_ATA2_MAC5		GENMASK(23, 16)

/* Register for address table write data */
#define AN8855_ATWD			0x10200324
#define   AN8855_ATWD_VLD		BIT(0) /* vid LOAD */
#define   AN8855_ATWD_LEAKY		BIT(1)
#define   AN8855_ATWD_UPRI		GENMASK(4, 2)
#define   AN8855_ATWD_SA_FWD		GENMASK(7, 5)
#define   AN8855_ATWD_SA_MIR		GENMASK(9, 8)
#define   AN8855_ATWD_EG_TAG		GENMASK(14, 12)
#define   AN8855_ATWD_IVL		BIT(15)
#define   AN8855_ATWD_VID		GENMASK(27, 16)
#define   AN8855_ATWD_FID		GENMASK(31, 28)
#define AN8855_ATWD2			0x10200328
#define   AN8855_ATWD2_PORT		GENMASK(7, 0)

/* Registers for table search read address */
#define AN8855_ATRDS			0x10200330
#define   AN8855_ATRD_SEL		GENMASK(1, 0)
#define AN8855_ATRD0			0x10200334
#define   AN8855_ATRD0_LIVE		BIT(0)
#define   AN8855_ATRD0_ARP		GENMASK(2, 1)
#define   AN8855_ATRD0_TYPE		GENMASK(4, 3)
#define   AN8855_ATRD0_IVL		BIT(9)
#define   AN8855_ATRD0_VID		GENMASK(21, 16)
#define   AN8855_ATRD0_FID		GENMASK(28, 25)
#define AN8855_ATRD1			0x10200338
#define   AN8855_ATRD1_MAC4		GENMASK(31, 24)
#define   AN8855_ATRD1_MAC5		GENMASK(23, 16)
#define   AN8855_ATRD1_AGING		GENMASK(11, 3)
#define AN8855_ATRD2			0x1020033c
#define   AN8855_ATRD2_MAC0		GENMASK(31, 24)
#define   AN8855_ATRD2_MAC1		GENMASK(23, 16)
#define   AN8855_ATRD2_MAC2		GENMASK(15, 8)
#define   AN8855_ATRD2_MAC3		GENMASK(7, 0)
#define AN8855_ATRD3			0x10200340
#define   AN8855_ATRD3_PORTMASK		GENMASK(7, 0)

enum an8855_fdb_type {
	AN8855_MAC_TB_TY_MAC = 0,
	AN8855_MAC_TB_TY_DIP = 1,
	AN8855_MAC_TB_TY_DIP_SIP = 2,
};

/* Register for vlan table control */
#define AN8855_VTCR			0x10200600
#define	  AN8855_VTCR_BUSY		BIT(31)
#define	  AN8855_VTCR_FUNC		GENMASK(15, 12)
#define	  AN8855_VTCR_VID		GENMASK(11, 0)

enum an8855_vlan_cmd {
	/* Read/Write the specified VID entry from VAWD register based
	 * on VID.
	 */
	AN8855_VTCR_RD_VID = 0,
	AN8855_VTCR_WR_VID = 1,
};

/* Register for setup vlan write data */
#define AN8855_VAWD0			0x10200604
/* VLAN Member Control */
#define   AN8855_VA0_PORT		GENMASK(31, 26)
/* Egress Tag Control */
#define   AN8855_VA0_ETAG		GENMASK(23, 12)
#define   AN8855_VA0_ETAG_PORT_MASK(port) (GENMASK(13, 12) << ((port) * 2))
#define   AN8855_VA0_ETAG_PORT_VAL(port, val) (val << __bf_shf(AN8855_VA0_ETAG_PORT_MASK(port)))
#define	  AN8855_VA0_VTAG_EN		BIT(10) /* Per VLAN Egress Tag Control */
#define	  AN8855_VA0_IVL_MAC		BIT(5) /* Independent VLAN Learning */
#define	  AN8855_VA0_VLAN_VALID		BIT(0) /* VLAN Entry Valid */
#define AN8855_VAWD1			0x10200608
#define	  AN8855_VA1_PORT_STAG		BIT(1)

/* Same register map of VAWD0 */
#define AN8855_VARD0			0x10200618

enum an8855_vlan_egress_attr {
	AN8855_VLAN_EGRESS_UNTAG = 0,
	AN8855_VLAN_EGRESS_TAG = 2,
	AN8855_VLAN_EGRESS_STACK = 3,
};

/* Register for port STP state control */
#define AN8855_SSP_P(x)			(0x10208000 + ((x) * 0x200))
#define	 AN8855_FID_PST			GENMASK(1, 0)

enum an8855_stp_state {
	AN8855_STP_DISABLED = 0,
	AN8855_STP_BLOCKING = 1,
	AN8855_STP_LISTENING = 1,
	AN8855_STP_LEARNING = 2,
	AN8855_STP_FORWARDING = 3
};

/* Register for port control */
#define AN8855_PCR_P(x)			(0x10208004 + ((x) * 0x200))
#define	  AN8855_EG_TAG			GENMASK(29, 28)
#define	  AN8855_PORT_PRI		GENMASK(26, 24)
#define	  AN8855_PORT_TX_MIR		BIT(20)
#define	  AN8855_PORT_RX_MIR		BIT(16)
#define	  AN8855_PORT_VLAN		GENMASK(1, 0)

enum an8855_port_mode {
	/* Port Matrix Mode: Frames are forwarded by the PCR_MATRIX members. */
	AN8855_PORT_MATRIX_MODE = 0,

	/* Fallback Mode: Forward received frames with ingress ports that do
	 * not belong to the VLAN member. Frames whose VID is not listed on
	 * the VLAN table are forwarded by the PCR_MATRIX members.
	 */
	AN8855_PORT_FALLBACK_MODE = 1,

	/* Check Mode: Forward received frames whose ingress do not
	 * belong to the VLAN member. Discard frames if VID ismiddes on the
	 * VLAN table.
	 */
	AN8855_PORT_CHECK_MODE = 1,

	/* Security Mode: Discard any frame due to ingress membership
	 * violation or VID missed on the VLAN table.
	 */
	AN8855_PORT_SECURITY_MODE = 3,
};

/* Register for port security control */
#define AN8855_PSC_P(x)			(0x1020800c + ((x) * 0x200))
#define	  AN8855_SA_DIS			BIT(4)

/* Register for port vlan control */
#define AN8855_PVC_P(x)			(0x10208010 + ((x) * 0x200))
#define	  AN8855_PVC_EG_TAG		GENMASK(10, 8)
#define	  AN8855_PORT_SPEC_REPLACE_MODE	BIT(11)
#define	  AN8855_VLAN_ATTR		GENMASK(7, 6)
#define	  AN8855_PORT_SPEC_TAG		BIT(5)

enum an8855_vlan_port_eg_tag {
	AN8855_VLAN_EG_DISABLED = 0,
	AN8855_VLAN_EG_CONSISTENT = 1,
	AN8855_VLAN_EG_UNTAGGED = 4,
	AN8855_VLAN_EG_SWAP = 5,
	AN8855_VLAN_EG_TAGGED = 6,
	AN8855_VLAN_EG_STACK = 7,
};

enum an8855_vlan_port_attr {
	AN8855_VLAN_USER = 0,
	AN8855_VLAN_STACK = 1,
	AN8855_VLAN_TRANSPARENT = 3,
};

#define AN8855_PORTMATRIX_P(x)		(0x10208044 + ((x) * 0x200))
#define   AN8855_PORTMATRIX		GENMASK(6, 0)

/* Register for port PVID */
#define AN8855_PVID_P(x)		(0x10208048 + ((x) * 0x200))
#define	  AN8855_G0_PORT_VID		GENMASK(11, 0)

/* Register for port MAC control register */
#define AN8855_PMCR_P(x)		(0x10210000 + ((x) * 0x200))
#define	  AN8855_PMCR_FORCE_MODE	BIT(31)
#define   AN8855_PMCR_FORCE_SPEED	GENMASK(30, 28)
#define	  AN8855_PMCR_FORCE_SPEED_5000	FIELD_PREP_CONST(AN8855_PMCR_FORCE_SPEED, 0x4)
#define	  AN8855_PMCR_FORCE_SPEED_2500	FIELD_PREP_CONST(AN8855_PMCR_FORCE_SPEED, 0x3)
#define	  AN8855_PMCR_FORCE_SPEED_1000	FIELD_PREP_CONST(AN8855_PMCR_FORCE_SPEED, 0x2)
#define	  AN8855_PMCR_FORCE_SPEED_100	FIELD_PREP_CONST(AN8855_PMCR_FORCE_SPEED, 0x1)
#define	  AN8855_PMCR_FORCE_SPEED_10	FIELD_PREP_CONST(AN8855_PMCR_FORCE_SPEED, 0x1)
#define	  AN8855_PMCR_FORCE_FDX		BIT(25)
#define	  AN8855_PMCR_FORCE_LNK		BIT(24)
#define	  AN8855_PMCR_IFG_XMIT		GENMASK(21, 20)
#define	  AN8855_PMCR_EXT_PHY		BIT(19)
#define	  AN8855_PMCR_MAC_MODE		BIT(18)
#define	  AN8855_PMCR_TX_EN		BIT(16)
#define	  AN8855_PMCR_RX_EN		BIT(15)
#define	  AN8855_PMCR_BACKOFF_EN	BIT(12)
#define	  AN8855_PMCR_BACKPR_EN		BIT(11)
#define	  AN8855_PMCR_FORCE_EEE5G	BIT(9)
#define	  AN8855_PMCR_FORCE_EEE2P5G	BIT(8)
#define	  AN8855_PMCR_FORCE_EEE1G	BIT(7)
#define	  AN8855_PMCR_FORCE_EEE100	BIT(6)
#define	  AN8855_PMCR_TX_FC_EN		BIT(5)
#define	  AN8855_PMCR_RX_FC_EN		BIT(4)

#define AN8855_PMSR_P(x)		(0x10210010 + (x) * 0x200)
#define	  AN8855_PMSR_SPEED		GENMASK(30, 28)
#define	  AN8855_PMSR_SPEED_5000	FIELD_PREP_CONST(AN8855_PMSR_SPEED, 0x4)
#define	  AN8855_PMSR_SPEED_2500	FIELD_PREP_CONST(AN8855_PMSR_SPEED, 0x3)
#define	  AN8855_PMSR_SPEED_1000	FIELD_PREP_CONST(AN8855_PMSR_SPEED, 0x2)
#define	  AN8855_PMSR_SPEED_100		FIELD_PREP_CONST(AN8855_PMSR_SPEED, 0x1)
#define	  AN8855_PMSR_SPEED_10		FIELD_PREP_CONST(AN8855_PMSR_SPEED, 0x0)
#define	  AN8855_PMSR_DPX		BIT(25)
#define	  AN8855_PMSR_LNK		BIT(24)
#define	  AN8855_PMSR_EEE1G		BIT(7)
#define	  AN8855_PMSR_EEE100M		BIT(6)
#define	  AN8855_PMSR_RX_FC		BIT(5)
#define	  AN8855_PMSR_TX_FC		BIT(4)

#define AN8855_PMEEECR_P(x)		(0x10210004 + (x) * 0x200)
#define	  AN8855_LPI_MODE_EN		BIT(31)
#define	  AN8855_WAKEUP_TIME_2500(x)	GENMASK(23, 16)
#define	  AN8855_WAKEUP_TIME_1000(x)	GENMASK(15, 8)
#define	  AN8855_WAKEUP_TIME_100(x)	GENMASK(7, 0)
#define AN8855_PMEEECR2_P(x)		(0x10210008 + (x) * 0x200)
#define	  AN8855_WAKEUP_TIME_5000(x)	GENMASK(7, 0)

#define AN8855_CKGCR			(0x10213e1c)
#define   AN8855_LPI_TXIDLE_THD_MASK	GENMASK(31, 14)
#define   AN8855_CKG_LNKDN_PORT_STOP	BIT(1)
#define   AN8855_CKG_LNKDN_GLB_STOP	BIT(0)

/* Register for MIB */
#define AN8855_PORT_MIB_COUNTER(x)	(0x10214000 + (x) * 0x200)
#define AN8855_MIB_CCR			0x10213e30
#define	 AN8855_CCR_MIB_ENABLE		BIT(31)
#define	 AN8855_CCR_RX_OCT_CNT_GOOD	BIT(7)
#define	 AN8855_CCR_RX_OCT_CNT_BAD	BIT(6)
#define	 AN8855_CCR_TX_OCT_CNT_GOOD	BIT(5)
#define	 AN8855_CCR_TX_OCT_CNT_BAD	BIT(4)
#define	 AN8855_CCR_RX_OCT_CNT_GOOD_2	BIT(3)
#define	 AN8855_CCR_RX_OCT_CNT_BAD_2	BIT(2)
#define	 AN8855_CCR_TX_OCT_CNT_GOOD_2	BIT(1)
#define	 AN8855_CCR_TX_OCT_CNT_BAD_2	BIT(0)
#define	 AN8855_CCR_MIB_ACTIVATE	(AN8855_CCR_MIB_ENABLE | \
					 AN8855_CCR_RX_OCT_CNT_GOOD | \
					 AN8855_CCR_RX_OCT_CNT_BAD | \
					 AN8855_CCR_TX_OCT_CNT_GOOD | \
					 AN8855_CCR_TX_OCT_CNT_BAD | \
					 AN8855_CCR_RX_OCT_CNT_BAD_2 | \
					 AN8855_CCR_TX_OCT_CNT_BAD_2)
#define AN8855_MIB_CLR			0x10213e34
#define   AN8855_MIB_PORT6_CLR		BIT(6)
#define   AN8855_MIB_PORT5_CLR		BIT(5)
#define   AN8855_MIB_PORT4_CLR		BIT(4)
#define   AN8855_MIB_PORT3_CLR		BIT(3)
#define   AN8855_MIB_PORT2_CLR		BIT(2)
#define   AN8855_MIB_PORT1_CLR		BIT(1)
#define   AN8855_MIB_PORT0_CLR		BIT(0)

/* HSGMII/SGMII Configuration register */
/*	AN8855_HSGMII_AN_CSR_BASE	0x10220000 */
#define AN8855_SGMII_REG_AN0		0x10220000
#define   AN8855_SGMII_AN_ENABLE	BIT(12)
#define   AN8855_SGMII_AN_RESTART	BIT(9)
#define AN8855_SGMII_REG_AN_13		0x10220034
#define AN8855_SGMII_REG_AN_FORCE_CL37	0x10220060

/*	AN8855_HSGMII_CSR_PCS_BASE	0x10220000 */
#define AN8855_RG_HSGMII_PCS_CTROL_1	0x10220a00
#define AN8855_RG_AN_SGMII_MODE_FORCE	0x10220a24

/*	AN8855_MULTI_SGMII_CSR_BASE	0x10224000 */
#define AN8855_SGMII_STS_CTRL_0		0x10224018
#define AN8855_MSG_RX_CTRL_0		0x10224100
#define AN8855_MSG_RX_LIK_STS_0		0x10224514
#define AN8855_MSG_RX_LIK_STS_2		0x1022451c
#define AN8855_PHY_RX_FORCE_CTRL_0	0x10224520

/*	AN8855_XFI_CSR_PCS_BASE		0x10225000 */
#define AN8855_RG_USXGMII_AN_CONTROL_0	0x10225bf8

/*	AN8855_MULTI_PHY_RA_CSR_BASE	0x10226000 */
#define AN8855_RG_RATE_ADAPT_CTRL_0	0x10226000
#define AN8855_RATE_ADP_P0_CTRL_0	0x10226100
#define AN8855_MII_RA_AN_ENABLE		0x10226300

/*	AN8855_QP_DIG_CSR_BASE		0x1022a000 */
#define AN8855_QP_CK_RST_CTRL_4		0x1022a310
#define AN8855_QP_DIG_MODE_CTRL_0	0x1022a324
#define AN8855_QP_DIG_MODE_CTRL_1	0x1022a330

/*	AN8855_SERDES_WRAPPER_BASE	0x1022c000 */
#define AN8855_USGMII_CTRL_0		0x1022c000

/*	AN8855_QP_PMA_TOP_BASE		0x1022e000 */
#define AN8855_PON_RXFEDIG_CTRL_0	0x1022e100
#define AN8855_PON_RXFEDIG_CTRL_9	0x1022e124

#define AN8855_SS_LCPLL_PWCTL_SETTING_2	0x1022e208
#define AN8855_SS_LCPLL_TDC_FLT_2	0x1022e230
#define AN8855_SS_LCPLL_TDC_FLT_5	0x1022e23c
#define AN8855_SS_LCPLL_TDC_PCW_1	0x1022e248
#define AN8855_INTF_CTRL_8		0x1022e320
#define AN8855_INTF_CTRL_9		0x1022e324
#define AN8855_INTF_CTRL_10		0x1022e328
#define AN8855_INTF_CTRL_11		0x1022e32c
#define AN8855_PLL_CTRL_0		0x1022e400
#define AN8855_PLL_CTRL_2		0x1022e408
#define AN8855_PLL_CTRL_3		0x1022e40c
#define AN8855_PLL_CTRL_4		0x1022e410
#define AN8855_PLL_CK_CTRL_0		0x1022e414
#define AN8855_RX_DLY_0			0x1022e614
#define AN8855_RX_CTRL_2		0x1022e630
#define AN8855_RX_CTRL_5		0x1022e63c
#define AN8855_RX_CTRL_6		0x1022e640
#define AN8855_RX_CTRL_7		0x1022e644
#define AN8855_RX_CTRL_8		0x1022e648
#define AN8855_RX_CTRL_26		0x1022e690
#define AN8855_RX_CTRL_42		0x1022e6d0

/*	AN8855_QP_ANA_CSR_BASE		0x1022f000 */
#define AN8855_RG_QP_RX_DAC_EN		0x1022f000
#define AN8855_RG_QP_RXAFE_RESERVE	0x1022f004
#define AN8855_RG_QP_CDR_LPF_BOT_LIM	0x1022f008
#define AN8855_RG_QP_CDR_LPF_MJV_LIM	0x1022f00c
#define AN8855_RG_QP_CDR_LPF_SETVALUE	0x1022f014
#define AN8855_RG_QP_CDR_PR_CKREF_DIV1	0x1022f018
#define AN8855_RG_QP_CDR_PR_KBAND_DIV_PCIE 0x1022f01c
#define AN8855_RG_QP_CDR_FORCE_IBANDLPF_R_OFF 0x1022f020
#define AN8855_RG_QP_TX_MODE_16B_EN	0x1022f028
#define AN8855_RG_QP_PLL_IPLL_DIG_PWR_SEL 0x1022f03c
#define AN8855_RG_QP_PLL_SDM_ORD	0x1022f040

/*	AN8855_ETHER_SYS_BASE		0x1028c800 */
#define AN8855_RG_GPHY_AFE_PWD		0x1028c840
#define AN8855_RG_GPHY_SMI_ADDR		0x1028c848

#define MIB_DESC(_s, _o, _n)	\
	{			\
		.size = (_s),	\
		.offset = (_o),	\
		.name = (_n),	\
	}

struct an8855_mib_desc {
	unsigned int size;
	unsigned int offset;
	const char *name;
};

struct an8855_fdb {
	u16 vid;
	u8 port_mask;
	u8 aging;
	u8 mac[6];
	bool noarp;
	u8 live;
	u8 type;
	u8 fid;
	u8 ivl;
};

/* Definition of LED */
#define LED_ON_EVENT	(LED_ON_EVT_LINK_1000M | \
			LED_ON_EVT_LINK_100M | LED_ON_EVT_LINK_10M |\
			LED_ON_EVT_LINK_HD | LED_ON_EVT_LINK_FD)

#define LED_BLK_EVENT	(LED_BLK_EVT_1000M_TX_ACT | \
			LED_BLK_EVT_1000M_RX_ACT | \
			LED_BLK_EVT_100M_TX_ACT | \
			LED_BLK_EVT_100M_RX_ACT | \
			LED_BLK_EVT_10M_TX_ACT | \
			LED_BLK_EVT_10M_RX_ACT)

#define LED_FREQ	AIR_LED_BLK_DUR_64M

enum phy_led_idx {
	P0_LED0,
	P0_LED1,
	P0_LED2,
	P0_LED3,
	P1_LED0,
	P1_LED1,
	P1_LED2,
	P1_LED3,
	P2_LED0,
	P2_LED1,
	P2_LED2,
	P2_LED3,
	P3_LED0,
	P3_LED1,
	P3_LED2,
	P3_LED3,
	P4_LED0,
	P4_LED1,
	P4_LED2,
	P4_LED3,
	PHY_LED_MAX
};

/* TBD */
enum an8855_led_blk_dur {
	AIR_LED_BLK_DUR_32M,
	AIR_LED_BLK_DUR_64M,
	AIR_LED_BLK_DUR_128M,
	AIR_LED_BLK_DUR_256M,
	AIR_LED_BLK_DUR_512M,
	AIR_LED_BLK_DUR_1024M,
	AIR_LED_BLK_DUR_LAST
};

enum an8855_led_polarity {
	LED_LOW,
	LED_HIGH,
};
enum an8855_led_mode {
	AN8855_LED_MODE_DISABLE,
	AN8855_LED_MODE_USER_DEFINE,
	AN8855_LED_MODE_LAST
};

struct an8855_led_cfg {
	u16 en;
	u8  phy_led_idx;
	u16 pol;
	u16 on_cfg;
	u16 blk_cfg;
	u8 led_freq;
};

/* struct an8855_priv -	This is the main data structure for holding the state
 *			of the driver
 * @dev:		The device pointer
 * @ds:			The pointer to the dsa core structure
 * @bus:		The bus used for the device and built-in PHY
 * @rstc:		The pointer to reset control used by MCM
 * @core_pwr:		The power supplied into the core
 * @io_pwr:		The power supplied into the I/O
 * @reset:		The descriptor for GPIO line tied to its reset pin
 * @mcm:		Flag for distinguishing if standalone IC or module
 *			coupling
 * @ports:		Holding the state among ports
 * @reg_mutex:		The lock for protecting among process accessing
 *			registers
 * @p6_interface	Holding the current port 6 interface
 * @p5_intf_sel:	Holding the current port 5 interface select
 */
struct an8855_priv {
	struct device *dev;
	struct dsa_switch *ds;
	struct mii_bus *bus;
	struct reset_control *rstc;
	struct regulator *core_pwr;
	struct regulator *io_pwr;
	struct gpio_desc *reset;
	// void __iomem *base;


	const struct an8855_dev_info *info;
	unsigned int phy_base;
	int phy_base_new;
	unsigned int id;
	u32 intr_pin;
	phy_interface_t p5_interface;
	unsigned int p5_intf_sel;
	u8 mirror_rx;
	u8 mirror_tx;
	u8 eee_enable;
	u32 extSurge;

	/* protect among processes for registers access */
	struct mutex reg_mutex;

	struct regmap *regmap;
	struct phylink_pcs pcs;
	struct gpio_desc *reset_gpio;
};

#endif /* __AN8855_H */
