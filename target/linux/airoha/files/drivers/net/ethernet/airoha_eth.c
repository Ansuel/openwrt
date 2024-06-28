// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 AIROHA Inc
 * Author: Lorenzo Bianconi <lorenzo@kernel.org>
 */
#include <linux/debugfs.h>
#include <linux/etherdevice.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/tcp.h>
#include <net/dsa.h>
#include <net/page_pool.h>
#include <uapi/linux/ppp_defs.h>
#include "airoha_eth.h"

static u32 airoha_rr(void __iomem *base, u32 offset)
{
	return readl(base + offset);
}

static void airoha_wr(void __iomem *base, u32 offset, u32 val)
{
	writel(val, base + offset);
}

static u32 airoha_rmw(void __iomem *base, u32 offset, u32 mask, u32 val)
{
	val |= (airoha_rr(base, offset) & ~mask);
	airoha_wr(base, offset, val);

	return val;
}

#define airoha_fe_rr(eth, offset)				\
	airoha_rr((eth)->fe_regs, (offset))
#define airoha_fe_wr(eth, offset, val)				\
	airoha_wr((eth)->fe_regs, (offset), (val))
#define airoha_fe_rmw(eth, offset, mask, val)			\
	airoha_rmw((eth)->fe_regs, (offset), (mask), (val))
#define airoha_fe_set(eth, offset, val)				\
	airoha_rmw((eth)->fe_regs, (offset), 0, (val))
#define airoha_fe_clear(eth, offset, val)			\
	airoha_rmw((eth)->fe_regs, (offset), (val), 0)

#define airoha_qdma_rr(eth, offset)				\
	airoha_rr((eth)->qdma_regs, (offset))
#define airoha_qdma_wr(eth, offset, val)			\
	airoha_wr((eth)->qdma_regs, (offset), (val))
#define airoha_qdma_rmw(eth, offset, mask, val)			\
	airoha_rmw((eth)->qdma_regs, (offset), (mask), (val))
#define airoha_qdma_set(eth, offset, val)			\
	airoha_rmw((eth)->qdma_regs, (offset), 0, (val))
#define airoha_qdma_clear(eth, offset, val)			\
	airoha_rmw((eth)->qdma_regs, (offset), (val), 0)

static void airoha_qdma_set_irqmask(struct airoha_eth *eth, int index,
				    u32 clear, u32 set)
{
	unsigned long flags;

	if (WARN_ON_ONCE(index >= ARRAY_SIZE(eth->irqmask)))
		return;

	spin_lock_irqsave(&eth->irq_lock, flags);

	eth->irqmask[index] &= ~clear;
	eth->irqmask[index] |= set;
	airoha_qdma_wr(eth, REG_INT_ENABLE(index), eth->irqmask[index]);

	spin_unlock_irqrestore(&eth->irq_lock, flags);
}

static void airoha_qdma_irq_enable(struct airoha_eth *eth, int index,
				   u32 mask)
{
	airoha_qdma_set_irqmask(eth, index, 0, mask);
}

static void airoha_qdma_irq_disable(struct airoha_eth *eth, int index,
				    u32 mask)
{
	airoha_qdma_set_irqmask(eth, index, mask, 0);
}

static void airoha_set_macaddr(struct airoha_eth *eth, const u8 *addr)
{
	u32 val;

	val = (addr[0] << 16) | (addr[1] << 8) | addr[2];
	airoha_fe_wr(eth, REG_FE_LAN_MAC_H, val);

	val = (addr[3] << 16) | (addr[4] << 8) | addr[5];
	airoha_fe_wr(eth, REG_FE_LAN_MAC_LMIN, val);
	airoha_fe_wr(eth, REG_FE_LAN_MAC_LMAX, val);
}

static void airoha_set_gdm_port_fwd_cfg(struct airoha_eth *eth, u32 addr,
					u32 val)
{
	airoha_fe_rmw(eth, addr, GDM_OCFQ_MASK,
		      FIELD_PREP(GDM_OCFQ_MASK, val));
	airoha_fe_rmw(eth, addr, GDM_MCFQ_MASK,
		      FIELD_PREP(GDM_MCFQ_MASK, val));
	airoha_fe_rmw(eth, addr, GDM_BCFQ_MASK,
		      FIELD_PREP(GDM_BCFQ_MASK, val));
	airoha_fe_rmw(eth, addr, GDM_UCFQ_MASK,
		      FIELD_PREP(GDM_UCFQ_MASK, val));
}

static int airoha_set_gdm_port(struct airoha_eth *eth, int port, bool enable)
{
	u32 vip_port, cfg_addr, val = enable ? FE_DP_PPE : FE_DP_DROP;

	switch (port) {
	case XSI_PCIE0_PORT:
		vip_port = XSI_PCIE0_VIP_PORT_MASK;
		cfg_addr = REG_GDM_FWD_CFG(2);
		break;
	case XSI_PCIE1_PORT:
		vip_port = XSI_PCIE1_VIP_PORT_MASK;
		cfg_addr = REG_GDM_FWD_CFG(2);
		break;
	case XSI_USB_PORT:
		vip_port = XSI_USB_VIP_PORT_MASK;
		cfg_addr = REG_GDM_FWD_CFG(3);
		break;
	case XSI_ETH_PORT:
		vip_port = XSI_ETH_VIP_PORT_MASK;
		cfg_addr = REG_GDM_FWD_CFG(3);
		break;
	default:
		return -EINVAL;
	}

	if (enable) {
		airoha_fe_set(eth, REG_FE_VIP_PORT_EN, vip_port);
		airoha_fe_set(eth, REG_FE_IFC_PORT_EN, vip_port);
	} else {
		airoha_fe_clear(eth, REG_FE_VIP_PORT_EN, vip_port);
		airoha_fe_clear(eth, REG_FE_IFC_PORT_EN, vip_port);
	}

	airoha_set_gdm_port_fwd_cfg(eth, cfg_addr, val);

	return 0;
}

static int airoha_set_gdm_ports(struct airoha_eth *eth, bool enable)
{
	const int port_list[] = {
		XSI_PCIE0_PORT,
		XSI_PCIE1_PORT,
		XSI_USB_PORT,
		XSI_ETH_PORT
	};
	int i, err;

	for (i = 0; i < ARRAY_SIZE(port_list); i++) {
		err = airoha_set_gdm_port(eth, port_list[i], enable);
		if (err)
			goto error;
	}

	return 0;

error:
	for (i--; i >= 0; i++)
		airoha_set_gdm_port(eth, port_list[i], false);

	return err;
}

static void airoha_fe_maccr_init(struct airoha_eth *eth)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(eth->ports); i++) {
		airoha_fe_set(eth, REG_GDM_FWD_CFG(i),
			      GDM_TCP_CKSUM | GDM_UDP_CKSUM | GDM_IP4_CKSUM |
			      GDM_DROP_CRC_ERR);
		airoha_set_gdm_port_fwd_cfg(eth, REG_GDM_FWD_CFG(i), FE_DP_CPU);
		airoha_fe_rmw(eth, REG_GDM_LEN_CFG(i),
			      GDM_SHORT_LEN_MASK | GDM_LONG_LEN_MASK,
			      FIELD_PREP(GDM_SHORT_LEN_MASK, 60) |
			      FIELD_PREP(GDM_LONG_LEN_MASK, 4004));
	}

	airoha_fe_rmw(eth, REG_CDM1_VLAN_CTRL, CDM1_VLAN_MASK,
		      FIELD_PREP(CDM1_VLAN_MASK, 0x8100));

	airoha_fe_set(eth, REG_FE_CPORT_CFG, FE_CPORT_PAD);
}

static void airoha_fe_vip_setup(struct airoha_eth *eth)
{
	airoha_fe_wr(eth, REG_FE_VIP_PATN(3), ETH_P_PPP_DISC);
	airoha_fe_wr(eth, REG_FE_VIP_EN(3), PATN_FCPU_EN_MASK | PATN_EN_MASK);

	airoha_fe_wr(eth, REG_FE_VIP_PATN(4), PPP_LCP);
	airoha_fe_wr(eth, REG_FE_VIP_EN(4),
		     PATN_FCPU_EN_MASK | FIELD_PREP(PATN_TYPE_MASK, 1) |
		     PATN_EN_MASK);

	airoha_fe_wr(eth, REG_FE_VIP_PATN(6), PPP_IPCP);
	airoha_fe_wr(eth, REG_FE_VIP_EN(6),
		     PATN_FCPU_EN_MASK | FIELD_PREP(PATN_TYPE_MASK, 1) |
		     PATN_EN_MASK);

	airoha_fe_wr(eth, REG_FE_VIP_PATN(7), PPP_CHAP);
	airoha_fe_wr(eth, REG_FE_VIP_EN(7),
		     PATN_FCPU_EN_MASK | FIELD_PREP(PATN_TYPE_MASK, 1) |
		     PATN_EN_MASK);

	airoha_fe_wr(eth, REG_FE_VIP_PATN(8), 0x43); /* BOOTP (0x43) */
	airoha_fe_wr(eth, REG_FE_VIP_EN(8),
		     PATN_FCPU_EN_MASK | PATN_SP_EN_MASK |
		     FIELD_PREP(PATN_TYPE_MASK, 4) | PATN_EN_MASK);

	airoha_fe_wr(eth, REG_FE_VIP_PATN(9), 0x44); /* BOOTP (0x44) */
	airoha_fe_wr(eth, REG_FE_VIP_EN(9),
		     PATN_FCPU_EN_MASK | PATN_SP_EN_MASK |
		     FIELD_PREP(PATN_TYPE_MASK, 4) | PATN_EN_MASK);

	airoha_fe_wr(eth, REG_FE_VIP_PATN(10), 0x1f401f4); /* ISAKMP */
	airoha_fe_wr(eth, REG_FE_VIP_EN(10),
		     PATN_FCPU_EN_MASK | PATN_DP_EN_MASK | PATN_SP_EN_MASK |
		     FIELD_PREP(PATN_TYPE_MASK, 4) | PATN_EN_MASK);

	airoha_fe_wr(eth, REG_FE_VIP_PATN(11), PPP_IPV6CP);
	airoha_fe_wr(eth, REG_FE_VIP_EN(11),
		     PATN_FCPU_EN_MASK | FIELD_PREP(PATN_TYPE_MASK, 1) |
		     PATN_EN_MASK);

	airoha_fe_wr(eth, REG_FE_VIP_PATN(12), 0x2220223); /* DHCPv6 */
	airoha_fe_wr(eth, REG_FE_VIP_EN(12),
		     PATN_FCPU_EN_MASK | PATN_DP_EN_MASK | PATN_SP_EN_MASK |
		     FIELD_PREP(PATN_TYPE_MASK, 4) | PATN_EN_MASK);

	airoha_fe_wr(eth, REG_FE_VIP_PATN(19), PPP_PAP);
	airoha_fe_wr(eth, REG_FE_VIP_EN(19),
		     PATN_FCPU_EN_MASK | FIELD_PREP(PATN_TYPE_MASK, 1) |
		     PATN_EN_MASK);

	airoha_fe_wr(eth, REG_FE_VIP_PATN(20), 0x893a); /* ETH->ETH_P_1905 (0x893a) */
	airoha_fe_wr(eth, REG_FE_VIP_EN(20),
		     PATN_FCPU_EN_MASK | PATN_EN_MASK);

	airoha_fe_wr(eth, REG_FE_VIP_PATN(21), ETH_P_LLDP);
	airoha_fe_wr(eth, REG_FE_VIP_EN(21),
		     PATN_FCPU_EN_MASK | PATN_EN_MASK);
}

static u32 airoha_fe_get_pse_queue_rsv_pages(struct airoha_eth *eth,
					     u32 port, u32 queue)
{
	u32 val;

	airoha_fe_rmw(eth, REG_FE_PSE_QUEUE_CFG_WR,
		      PSE_CFG_PORT_ID_MASK | PSE_CFG_QUEUE_ID_MASK,
		      FIELD_PREP(PSE_CFG_PORT_ID_MASK, port) |
		      FIELD_PREP(PSE_CFG_QUEUE_ID_MASK, queue));
	val = airoha_fe_rr(eth, REG_FE_PSE_QUEUE_CFG_VAL);

	return FIELD_GET(PSE_CFG_OQ_RSV_MASK, val);
}

static void airoha_fe_set_pse_queue_rsv_pages(struct airoha_eth *eth,
					      u32 port, u32 queue, u32 val)
{
	airoha_fe_rmw(eth, REG_FE_PSE_QUEUE_CFG_VAL, PSE_CFG_OQ_RSV_MASK,
		      FIELD_PREP(PSE_CFG_OQ_RSV_MASK, val));
	airoha_fe_rmw(eth, REG_FE_PSE_QUEUE_CFG_WR,
		      PSE_CFG_PORT_ID_MASK | PSE_CFG_QUEUE_ID_MASK |
		      PSE_CFG_WR_EN_MASK | PSE_CFG_OQRSV_SEL_MASK,
		      FIELD_PREP(PSE_CFG_PORT_ID_MASK, port) |
		      FIELD_PREP(PSE_CFG_QUEUE_ID_MASK, queue) |
		      PSE_CFG_WR_EN_MASK | PSE_CFG_OQRSV_SEL_MASK);
}

static int airoha_fe_set_pse_oq_rsv(struct airoha_eth *eth,
				    u32 port, u32 queue, u32 val)
{
	u32 orig_val, tmp, all_rsv, fq_limit;

	airoha_fe_set_pse_queue_rsv_pages(eth, port, queue, val);

	/* modify all rsv */
	orig_val = airoha_fe_get_pse_queue_rsv_pages(eth, port, queue);
	tmp = airoha_fe_rr(eth, REG_FE_PSE_BUF_SET);
	all_rsv = FIELD_GET(PSE_ALLRSV_MASK, tmp);
	all_rsv += (val - orig_val);
	airoha_fe_rmw(eth, REG_FE_PSE_BUF_SET, PSE_ALLRSV_MASK,
		      FIELD_PREP(PSE_ALLRSV_MASK, all_rsv));

	/* modify hthd */
	tmp = airoha_fe_rr(eth, PSE_FQ_CFG);
	fq_limit = FIELD_GET(PSE_FQ_LIMIT_MASK, tmp);
	tmp = fq_limit - all_rsv - 0x20;
	airoha_fe_rmw(eth, REG_PSE_SHARE_USED_THD,
		      PSE_SHARE_USED_HTHD_MASK,
		      FIELD_PREP(PSE_SHARE_USED_HTHD_MASK, tmp));

	tmp = fq_limit - all_rsv - 0x100;
	airoha_fe_rmw(eth, REG_PSE_SHARE_USED_THD,
		      PSE_SHARE_USED_MTHD_MASK,
		      FIELD_PREP(PSE_SHARE_USED_MTHD_MASK, tmp));
	tmp = (3 * tmp) >> 2;
	airoha_fe_rmw(eth, REG_FE_PSE_BUF_SET,
		      PSE_SHARE_USED_LTHD_MASK,
		      FIELD_PREP(PSE_SHARE_USED_LTHD_MASK, tmp));

	return 0;
}

static void airoha_fe_pse_ports_init(struct airoha_eth *eth)
{
	const u32 pse_port_num_queues[] = {
		[FE_PSE_PORT_CDM1] = 6,
		[FE_PSE_PORT_GDM1] = 6,
		[FE_PSE_PORT_GDM2] = 32,
		[FE_PSE_PORT_GDM3] = 6,
		[FE_PSE_PORT_PPE1] = 4,
		[FE_PSE_PORT_CDM2] = 6,
		[FE_PSE_PORT_CDM3] = 8,
		[FE_PSE_PORT_CDM4] = 10,
		[FE_PSE_PORT_PPE2] = 4,
		[FE_PSE_PORT_GDM4] = 2,
		[FE_PSE_PORT_CDM5] = 2,
	};
	int q;

	/* hw misses PPE2 oq rsv */
	airoha_fe_set(eth, REG_FE_PSE_BUF_SET,
		      PSE_RSV_PAGES * pse_port_num_queues[FE_PSE_PORT_PPE2]);

	/* CMD1 */
	for (q = 0; q < pse_port_num_queues[FE_PSE_PORT_CDM1]; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_CDM1, q,
					 PSE_QUEUE_RSV_PAGES);
	/* GMD1 */
	for (q = 0; q < pse_port_num_queues[FE_PSE_PORT_GDM1]; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_GDM1, q,
					 PSE_QUEUE_RSV_PAGES);
	/* GMD2 */
	for (q = 6; q < pse_port_num_queues[FE_PSE_PORT_GDM2]; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_GDM2, q, 0);
	/* GMD3 */
	for (q = 0; q < pse_port_num_queues[FE_PSE_PORT_GDM3]; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_GDM3, q,
					 PSE_QUEUE_RSV_PAGES);
	/* PPE1 */
	for (q = 0; q < pse_port_num_queues[FE_PSE_PORT_PPE1]; q++) {
		if (q < pse_port_num_queues[FE_PSE_PORT_PPE1])
			airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_PPE1, q,
						 PSE_QUEUE_RSV_PAGES);
		else
			airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_PPE1, q, 0);
	}
	/* CDM2 */
	for (q = 0; q < pse_port_num_queues[FE_PSE_PORT_CDM2]; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_CDM2, q,
					 PSE_QUEUE_RSV_PAGES);
	/* CDM3 */
	for (q = 0; q < pse_port_num_queues[FE_PSE_PORT_CDM3] - 1; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_CDM3, q, 0);
	/* CDM4 */
	for (q = 4; q < pse_port_num_queues[FE_PSE_PORT_CDM4]; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_CDM4, q,
					 PSE_QUEUE_RSV_PAGES);
	/* PPE2 */
	for (q = 0; q < pse_port_num_queues[FE_PSE_PORT_PPE2]; q++) {
		if (q < pse_port_num_queues[FE_PSE_PORT_PPE2] / 2)
			airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_PPE2, q,
						 PSE_QUEUE_RSV_PAGES);
		else
			airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_PPE2, q, 0);
	}
	/* GMD4 */
	for (q = 0; q < pse_port_num_queues[FE_PSE_PORT_GDM4]; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_GDM4, q,
					 PSE_QUEUE_RSV_PAGES);
	/* CDM5 */
	for (q = 0; q < pse_port_num_queues[FE_PSE_PORT_CDM5]; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_CDM5, q,
					 PSE_QUEUE_RSV_PAGES);
}

static int airoha_fe_mc_vlan_clear(struct airoha_eth *eth)
{
	int i;

	for (i = 0; i < AIROHA_FE_MC_MAX_VLAN_TABLE; i++) {
		int err, j;
		u32 val;

		airoha_fe_wr(eth, REG_MC_VLAN_DATA, 0x0);

		val = FIELD_PREP(MC_VLAN_CFG_TABLE_ID_MASK, i) |
		      MC_VLAN_CFG_TABLE_SEL_MASK | MC_VLAN_CFG_RW_MASK;
		airoha_fe_wr(eth, REG_MC_VLAN_CFG, val);
		err = read_poll_timeout(airoha_fe_rr, val,
					val & MC_VLAN_CFG_CMD_DONE_MASK,
					USEC_PER_MSEC, 5 * USEC_PER_MSEC,
					false, eth, REG_MC_VLAN_CFG);
		if (err)
			return err;

		for (j = 0; j < AIROHA_FE_MC_MAX_VLAN_PORT; j++) {
			airoha_fe_wr(eth, REG_MC_VLAN_DATA, 0x0);

			val = FIELD_PREP(MC_VLAN_CFG_TABLE_ID_MASK, i) |
			      FIELD_PREP(MC_VLAN_CFG_PORT_ID_MASK, j) |
			      MC_VLAN_CFG_RW_MASK;
			airoha_fe_wr(eth, REG_MC_VLAN_CFG, val);
			err = read_poll_timeout(airoha_fe_rr, val,
						val & MC_VLAN_CFG_CMD_DONE_MASK,
						USEC_PER_MSEC, 5 * USEC_PER_MSEC,
						false, eth, REG_MC_VLAN_CFG);
			if (err)
				return err;
		}
	}

	return 0;
}

static void airoha_fe_crsn_qsel_init(struct airoha_eth *eth)
{
	/* CDM1_CRSN_QSEL */
	airoha_fe_rmw(eth, REG_CDM1_CRSN_QSEL(CRSN_22 >> 2),
		      CDM1_CRSN_QSEL_REASON_MASK(CRSN_22),
		      FIELD_PREP(CDM1_CRSN_QSEL_REASON_MASK(CRSN_22),
				 CDM_CRSN_QSEL_Q1));
	airoha_fe_rmw(eth, REG_CDM1_CRSN_QSEL(CRSN_08 >> 2),
		      CDM1_CRSN_QSEL_REASON_MASK(CRSN_08),
		      FIELD_PREP(CDM1_CRSN_QSEL_REASON_MASK(CRSN_08),
				 CDM_CRSN_QSEL_Q1));
	airoha_fe_rmw(eth, REG_CDM1_CRSN_QSEL(CRSN_21 >> 2),
		      CDM1_CRSN_QSEL_REASON_MASK(CRSN_21),
		      FIELD_PREP(CDM1_CRSN_QSEL_REASON_MASK(CRSN_21),
				 CDM_CRSN_QSEL_Q1));
	airoha_fe_rmw(eth, REG_CDM1_CRSN_QSEL(CRSN_24 >> 2),
		      CDM1_CRSN_QSEL_REASON_MASK(CRSN_24),
		      FIELD_PREP(CDM1_CRSN_QSEL_REASON_MASK(CRSN_24),
				 CDM_CRSN_QSEL_Q6));
	airoha_fe_rmw(eth, REG_CDM1_CRSN_QSEL(CRSN_25 >> 2),
		      CDM1_CRSN_QSEL_REASON_MASK(CRSN_25),
		      FIELD_PREP(CDM1_CRSN_QSEL_REASON_MASK(CRSN_25),
				 CDM_CRSN_QSEL_Q1));
	/* CDM2_CRSN_QSEL */
	airoha_fe_rmw(eth, REG_CDM2_CRSN_QSEL(CRSN_08 >> 2),
		      CDM2_CRSN_QSEL_REASON_MASK(CRSN_08),
		      FIELD_PREP(CDM2_CRSN_QSEL_REASON_MASK(CRSN_08),
				 CDM_CRSN_QSEL_Q1));
	airoha_fe_rmw(eth, REG_CDM2_CRSN_QSEL(CRSN_21 >> 2),
		      CDM2_CRSN_QSEL_REASON_MASK(CRSN_21),
		      FIELD_PREP(CDM2_CRSN_QSEL_REASON_MASK(CRSN_21),
				 CDM_CRSN_QSEL_Q1));
	airoha_fe_rmw(eth, REG_CDM2_CRSN_QSEL(CRSN_22 >> 2),
		      CDM2_CRSN_QSEL_REASON_MASK(CRSN_22),
		      FIELD_PREP(CDM2_CRSN_QSEL_REASON_MASK(CRSN_22),
				 CDM_CRSN_QSEL_Q1));
	airoha_fe_rmw(eth, REG_CDM2_CRSN_QSEL(CRSN_24 >> 2),
		      CDM2_CRSN_QSEL_REASON_MASK(CRSN_24),
		      FIELD_PREP(CDM2_CRSN_QSEL_REASON_MASK(CRSN_24),
				 CDM_CRSN_QSEL_Q6));
	airoha_fe_rmw(eth, REG_CDM2_CRSN_QSEL(CRSN_25 >> 2),
		      CDM2_CRSN_QSEL_REASON_MASK(CRSN_25),
		      FIELD_PREP(CDM2_CRSN_QSEL_REASON_MASK(CRSN_25),
				 CDM_CRSN_QSEL_Q1));
}

static int airoha_fe_init(struct airoha_eth *eth)
{
	airoha_fe_maccr_init(eth);

	/* PSE IQ reserve */
	airoha_fe_rmw(eth, REG_PSE_IQ_REV1, PSE_IQ_RES1_P2_MASK,
		      FIELD_PREP(PSE_IQ_RES1_P2_MASK, 0x10));
	airoha_fe_rmw(eth, REG_PSE_IQ_REV2,
		      PSE_IQ_RES2_P5_MASK | PSE_IQ_RES2_P4_MASK,
		      FIELD_PREP(PSE_IQ_RES2_P5_MASK, 0x40) |
		      FIELD_PREP(PSE_IQ_RES2_P4_MASK, 0x34));

	/* enable FE copy engine for MC/KA/DPI */
	airoha_fe_wr(eth, REG_FE_PCE_CFG,
		     PCE_DPI_EN_MASK | PCE_KA_EN_MASK | PCE_MC_EN_MASK);
	/* set vip queue selection to ring 1 */
	airoha_fe_rmw(eth, REG_CDM1_FWD_CFG, CDM1_VIP_QSEL_MASK,
		      FIELD_PREP(CDM1_VIP_QSEL_MASK, 0x4));
	airoha_fe_rmw(eth, REG_CDM2_FWD_CFG, CDM2_VIP_QSEL_MASK,
		      FIELD_PREP(CDM2_VIP_QSEL_MASK, 0x4));
	/* set GDM4 source interface offset to 8 */
	airoha_fe_rmw(eth, REG_GDM4_SRC_PORT_SET,
		      GDM4_SPORT_OFF2_MASK |
		      GDM4_SPORT_OFF1_MASK |
		      GDM4_SPORT_OFF0_MASK,
		      FIELD_PREP(GDM4_SPORT_OFF2_MASK, 8) |
		      FIELD_PREP(GDM4_SPORT_OFF1_MASK, 8) |
		      FIELD_PREP(GDM4_SPORT_OFF0_MASK, 8));

	/* set PSE Page as 128B */
	airoha_fe_rmw(eth, REG_FE_DMA_GLO_CFG,
		      FE_DMA_GLO_L2_SPACE_MASK | FE_DMA_GLO_PG_SZ_MASK,
		      FIELD_PREP(FE_DMA_GLO_L2_SPACE_MASK, 2) |
		      FE_DMA_GLO_PG_SZ_MASK);
	airoha_fe_wr(eth, REG_FE_RST_GLO_CFG,
		     FE_RST_CORE_MASK | FE_RST_GDM3_MBI_ARB_MASK |
		     FE_RST_GDM4_MBI_ARB_MASK);
	usleep_range(1000, 2000);

	/* connect RxRing1 and RxRing15 to PSE Port0 OQ-1
	 * connect other rings to PSE Port0 OQ-0
	 */
	airoha_fe_wr(eth, REG_FE_CDM1_OQ_MAP0, BIT(4));
	airoha_fe_wr(eth, REG_FE_CDM1_OQ_MAP1, BIT(28));
	airoha_fe_wr(eth, REG_FE_CDM1_OQ_MAP2, BIT(4));
	airoha_fe_wr(eth, REG_FE_CDM1_OQ_MAP3, BIT(28));

	airoha_fe_vip_setup(eth);
	airoha_fe_pse_ports_init(eth);

	airoha_fe_set(eth, REG_GDM_MISC_CFG,
		      GDM2_RDM_ACK_WAIT_PREF_MASK |
		      GDM2_CHN_VLD_MODE_MASK);
	airoha_fe_rmw(eth, REG_CDM2_FWD_CFG, CDM2_OAM_QSEL_MASK, 15);

	/* init fragment and assemble Force Port */
	/* NPU Core-3, NPU Bridge Channel-3 */
	airoha_fe_rmw(eth, REG_IP_FRAG_FP,
		      IP_FRAGMENT_PORT_MASK | IP_FRAGMENT_NBQ_MASK,
		      FIELD_PREP(IP_FRAGMENT_PORT_MASK, 6) |
		      FIELD_PREP(IP_FRAGMENT_NBQ_MASK, 3));
	/* QDMA LAN, RX Ring-22 */
	airoha_fe_rmw(eth, REG_IP_FRAG_FP,
		      IP_ASSEMBLE_PORT_MASK | IP_ASSEMBLE_NBQ_MASK,
		      FIELD_PREP(IP_ASSEMBLE_PORT_MASK, 0) |
		      FIELD_PREP(IP_ASSEMBLE_NBQ_MASK, 22));

	airoha_fe_set(eth, REG_GDM3_FWD_CFG, GDM3_PAD_EN_MASK);
	airoha_fe_set(eth, REG_GDM4_FWD_CFG, GDM4_PAD_EN_MASK);

	airoha_fe_crsn_qsel_init(eth);

	airoha_fe_clear(eth, REG_FE_CPORT_CFG, FE_CPORT_QUEUE_XFC_MASK);
	airoha_fe_set(eth, REG_FE_CPORT_CFG, FE_CPORT_PORT_XFC_MASK);

	/* default aging mode for mbi unlock issue */
	airoha_fe_rmw(eth, REG_GDM2_CHN_RLS,
		      MBI_RX_AGE_SEL_MASK | MBI_TX_AGE_SEL_MASK,
		      FIELD_PREP(MBI_RX_AGE_SEL_MASK, 3) |
		      FIELD_PREP(MBI_TX_AGE_SEL_MASK, 3));

	/* disable IFC by default */
	airoha_fe_clear(eth, REG_FE_CSR_IFC_CFG, FE_IFC_EN_MASK);

	/* enable 1:N vlan action, init vlan table */
	airoha_fe_set(eth, REG_MC_VLAN_EN, MC_VLAN_EN_MASK);

	return airoha_fe_mc_vlan_clear(eth);
}

static int airoha_qdma_fill_rx_queue(struct airoha_queue *q)
{
	enum dma_data_direction dir = page_pool_get_dma_dir(q->page_pool);
	struct airoha_eth *eth = q->eth;
	int qid = q - &eth->q_rx[0];
	int nframes = 0;

	while (q->queued < q->ndesc - 1) {
		struct airoha_queue_entry *e = &q->entry[q->head];
		struct airoha_qdma_desc *desc = &q->desc[q->head];
		struct page *page;
		int offset;
		u32 val;

		page = page_pool_dev_alloc_frag(q->page_pool, &offset,
						q->buf_size);
		if (!page)
			break;

		q->head = (q->head + 1) % q->ndesc;
		q->queued++;
		nframes++;

		e->buf = page_address(page) + offset;
		e->dma_addr = page_pool_get_dma_addr(page) + offset;
		e->dma_len = SKB_WITH_OVERHEAD(q->buf_size);

		dma_sync_single_for_device(eth->dev, e->dma_addr, e->dma_len,
					   dir);

		val = FIELD_PREP(QDMA_DESC_LEN_MASK, e->dma_len);
		WRITE_ONCE(desc->ctrl, cpu_to_le32(val));
		WRITE_ONCE(desc->addr, cpu_to_le32(e->dma_addr));
		val = FIELD_PREP(QDMA_DESC_NEXT_ID_MASK, q->head);
		WRITE_ONCE(desc->data, cpu_to_le32(val));
		WRITE_ONCE(desc->msg0, 0);
		WRITE_ONCE(desc->msg1, 0);
		WRITE_ONCE(desc->msg2, 0);
		WRITE_ONCE(desc->msg3, 0);

		airoha_qdma_rmw(eth, REG_RX_CPU_IDX(qid), RX_RING_CPU_IDX_MASK,
				FIELD_PREP(RX_RING_CPU_IDX_MASK, q->head));
	}

	return nframes;
}

static int airoha_qdma_get_gdm_port_id(struct airoha_eth *eth,
				       struct airoha_qdma_desc *desc)
{
	u32 port, sport, msg1 = le32_to_cpu(desc->msg1);

	sport = FIELD_GET(QDMA_ETH_RXMSG_SPORT_MASK, msg1);
	switch (sport) {
	case 0x10 ... 0x13:
		port = 0;
		break;
	case 0x2 ... 0x4:
		port = sport - 1;
		break;
	default:
		return -EINVAL;
	}

	return port >= ARRAY_SIZE(eth->ports) ? -EINVAL : port;
}

static int airoha_qdma_rx_process(struct airoha_queue *q, int budget)
{
	enum dma_data_direction dir = page_pool_get_dma_dir(q->page_pool);
	struct airoha_eth *eth = q->eth;
	int qid = q - &eth->q_rx[0];
	int done = 0;

	while (done < budget) {
		struct airoha_queue_entry *e = &q->entry[q->tail];
		struct airoha_qdma_desc *desc = &q->desc[q->tail];
		dma_addr_t dma_addr = le32_to_cpu(desc->addr);
		u32 desc_ctrl = le32_to_cpu(desc->ctrl);
		struct airoha_gdm_port *port;
		struct sk_buff *skb;
		int len, p;

		if (!(desc_ctrl & QDMA_DESC_DONE_MASK))
			break;

		if (!dma_addr)
			break;

		len = FIELD_GET(QDMA_DESC_LEN_MASK, desc_ctrl);
		if (!len)
			break;

		q->tail = (q->tail + 1) % q->ndesc;
		q->queued--;

		dma_sync_single_for_cpu(eth->dev, dma_addr,
					SKB_WITH_OVERHEAD(q->buf_size), dir);

		p = airoha_qdma_get_gdm_port_id(eth, desc);
		if (p < 0) {
			page_pool_put_full_page(q->page_pool,
						virt_to_head_page(e->buf),
						true);
			continue;
		}

		skb = napi_build_skb(e->buf, q->buf_size);
		if (!skb) {
			page_pool_put_full_page(q->page_pool,
						virt_to_head_page(e->buf),
						true);
			break;
		}

		port = eth->ports[p];

		skb_reserve(skb, 2);
		__skb_put(skb, len);
		skb_mark_for_recycle(skb);
		skb->dev = port->dev;
		skb->protocol = eth_type_trans(skb, port->dev);
		skb->ip_summed = CHECKSUM_UNNECESSARY;
		skb_record_rx_queue(skb, qid);
		napi_gro_receive(&q->napi, skb);

		done++;
	}
	airoha_qdma_fill_rx_queue(q);

	return done;
}

static int airoha_qdma_rx_napi_poll(struct napi_struct *napi, int budget)
{
	struct airoha_queue *q = container_of(napi, struct airoha_queue, napi);
	struct airoha_eth *eth = q->eth;
	int cur, done = 0;

	do {
		cur = airoha_qdma_rx_process(q, budget - done);
		done += cur;
	} while (cur && done < budget);

	if (done < budget && napi_complete(napi))
		airoha_qdma_irq_enable(eth, QDMA_INT_REG_IDX1,
				       RX_DONE_INT_MASK);

	return done;
}

static int airoha_qdma_init_rx_queue(struct airoha_eth *eth,
				     struct airoha_queue *q, int ndesc)
{
	struct page_pool_params pp_params = {
		.order = 0,
		.pool_size = 256,
		.flags = PP_FLAG_DMA_MAP | PP_FLAG_DMA_SYNC_DEV |
			 PP_FLAG_PAGE_FRAG,
		.dma_dir = DMA_FROM_DEVICE,
		.max_len = PAGE_SIZE,
		.nid = NUMA_NO_NODE,
		.dev = eth->dev,
	};
	int qid = q - &eth->q_rx[0], thr;
	dma_addr_t dma_addr;

	q->buf_size = PAGE_SIZE / 2;
	q->ndesc = ndesc;
	q->eth = eth;

	q->entry = devm_kzalloc(eth->dev, q->ndesc * sizeof(*q->entry),
				GFP_KERNEL);
	if (!q->entry)
		return -ENOMEM;

	q->page_pool = page_pool_create(&pp_params);
	if (IS_ERR(q->page_pool)) {
		int err = PTR_ERR(q->page_pool);

		q->page_pool = NULL;
		return err;
	}

	q->desc = dmam_alloc_coherent(eth->dev, q->ndesc * sizeof(*q->desc),
				      &dma_addr, GFP_KERNEL);
	if (!q->desc)
		return -ENOMEM;

	netif_napi_add(&eth->napi_dev, &q->napi, airoha_qdma_rx_napi_poll);

	airoha_qdma_wr(eth, REG_RX_RING_BASE(qid), dma_addr);
	airoha_qdma_rmw(eth, REG_RX_RING_SIZE(qid), RX_RING_SIZE_MASK,
			FIELD_PREP(RX_RING_SIZE_MASK, ndesc));

	thr = clamp(ndesc >> 3, 1, 32);
	airoha_qdma_rmw(eth, REG_RX_RING_SIZE(qid), RX_RING_THR_MASK,
			FIELD_PREP(RX_RING_THR_MASK, thr));
	airoha_qdma_rmw(eth, REG_RX_DMA_IDX(qid), RX_RING_DMA_IDX_MASK,
			FIELD_PREP(RX_RING_DMA_IDX_MASK, q->head));

	airoha_qdma_fill_rx_queue(q);

	return 0;
}

static void airoha_qdma_cleanup_rx_queue(struct airoha_queue *q)
{
	enum dma_data_direction dir = page_pool_get_dma_dir(q->page_pool);
	struct airoha_eth *eth = q->eth;

	while (q->queued) {
		struct airoha_queue_entry *e = &q->entry[q->tail];
		struct page *page = virt_to_head_page(e->buf);

		dma_sync_single_for_cpu(eth->dev, e->dma_addr, e->dma_len,
					dir);
		page_pool_put_full_page(q->page_pool, page, false);
		q->tail = (q->tail + 1) % q->ndesc;
		q->queued--;
	}
}

static int airoha_qdma_init_rx(struct airoha_eth *eth)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(eth->q_rx); i++) {
		int err;

		if (!(RX_DONE_INT_MASK & BIT(i))) {
			/* rx-queue not binded to irq */
			continue;
		}

		err = airoha_qdma_init_rx_queue(eth, &eth->q_rx[i],
						RX_DSCP_NUM(i));
		if (err)
			return err;
	}

	return 0;
}

static int airoha_qdma_tx_napi_poll(struct napi_struct *napi, int budget)
{
	struct airoha_tx_irq_queue *irq_q;
	struct airoha_eth *eth;
	int id, done = 0;

	irq_q = container_of(napi, struct airoha_tx_irq_queue, napi);
	eth = irq_q->eth;
	id = irq_q - &eth->q_tx_irq[0];

	while (irq_q->queued > 0 && done < budget) {
		u32 qid, last, val = irq_q->q[irq_q->head];
		u32 n_pkts = 0, n_bytes = 0;
		struct airoha_queue *q;

		if (val == 0xff)
			break;

		irq_q->q[irq_q->head] = 0xff; /* mark as done */
		irq_q->head = (irq_q->head + 1) % irq_q->size;
		irq_q->queued--;
		done++;

		last = FIELD_GET(IRQ_DESC_IDX_MASK, val);
		qid = FIELD_GET(IRQ_RING_IDX_MASK, val);

		if (qid >= ARRAY_SIZE(eth->q_tx))
			continue;

		q = &eth->q_tx[qid];
		if (!q->ndesc)
			continue;

		spin_lock_bh(&q->lock);

		while (q->queued > 0) {
			struct airoha_qdma_desc *desc = &q->desc[q->tail];
			struct airoha_queue_entry *e = &q->entry[q->tail];
			u32 desc_ctrl = le32_to_cpu(desc->ctrl);
			struct sk_buff *skb = e->skb;
			u16 index = q->tail;

			if (!(desc_ctrl & QDMA_DESC_DONE_MASK) &&
			    !(desc_ctrl & QDMA_DESC_DROP_MASK))
				break;

			q->tail = (q->tail + 1) % q->ndesc;
			q->queued--;

			dma_unmap_single(eth->dev, e->dma_addr, e->dma_len,
					 DMA_TO_DEVICE);

			WRITE_ONCE(desc->msg0, 0);
			WRITE_ONCE(desc->msg1, 0);

			if (skb) {
				struct netdev_queue *txq;

				n_bytes += skb->len;
				n_pkts++;

				txq = netdev_get_tx_queue(skb->dev, qid);
				netdev_tx_completed_queue(txq, n_pkts,
							  n_bytes);

				if (netif_tx_queue_stopped(txq) &&
				    q->ndesc - q->queued >= q->free_thr)
					netif_tx_wake_queue(txq);

				dev_kfree_skb_any(skb);
				e->skb = NULL;
			}

			if (index == last)
				break;
		}

		spin_unlock_bh(&q->lock);
	}

	if (done) {
		int i, len = done >> 7;

		for (i = 0; i < len; i++)
			airoha_qdma_rmw(eth, REG_IRQ_CLEAR_LEN(id),
					IRQ_CLEAR_LEN_MASK, 0x80);
		airoha_qdma_rmw(eth, REG_IRQ_CLEAR_LEN(id),
				IRQ_CLEAR_LEN_MASK, (done & 0x7f));
	}

	if (done < budget && napi_complete(napi))
		airoha_qdma_irq_enable(eth, QDMA_INT_REG_IDX0,
				       TX_DONE_INT_MASK(id));

	return done;
}

static int airoha_qdma_init_tx_queue(struct airoha_eth *eth,
				     struct airoha_queue *q, int size)
{
	int i, qid = q - &eth->q_tx[0];
	dma_addr_t dma_addr;

	spin_lock_init(&q->lock);
	q->ndesc = size;
	q->eth = eth;
	q->free_thr = MAX_SKB_FRAGS;

	q->entry = devm_kzalloc(eth->dev, q->ndesc * sizeof(*q->entry),
				GFP_KERNEL);
	if (!q->entry)
		return -ENOMEM;

	q->desc = dmam_alloc_coherent(eth->dev, q->ndesc * sizeof(*q->desc),
				      &dma_addr, GFP_KERNEL);
	if (!q->desc)
		return -ENOMEM;

	for (i = 0; i < q->ndesc; i++) {
		u32 val;

		val = FIELD_PREP(QDMA_DESC_DONE_MASK, 1);
		WRITE_ONCE(q->desc[i].ctrl, cpu_to_le32(val));
	}

	airoha_qdma_wr(eth, REG_TX_RING_BASE(qid), dma_addr);
	airoha_qdma_rmw(eth, REG_TX_CPU_IDX(qid), TX_RING_CPU_IDX_MASK,
			FIELD_PREP(TX_RING_CPU_IDX_MASK, q->head));
	airoha_qdma_rmw(eth, REG_TX_DMA_IDX(qid), TX_RING_DMA_IDX_MASK,
			FIELD_PREP(TX_RING_DMA_IDX_MASK, q->head));

	return 0;
}

static int airoha_qdma_tx_irq_init(struct airoha_eth *eth,
				   struct airoha_tx_irq_queue *irq_q,
				   int size)
{
	int id = irq_q - &eth->q_tx_irq[0];
	dma_addr_t dma_addr;

	netif_napi_add_tx(&eth->napi_dev, &irq_q->napi,
			  airoha_qdma_tx_napi_poll);
	irq_q->q = dmam_alloc_coherent(eth->dev, size * sizeof(u32),
				       &dma_addr, GFP_KERNEL);
	if (!irq_q->q)
		return -ENOMEM;

	memset(irq_q->q, 0xff, size * sizeof(u32));
	irq_q->size = size;
	irq_q->eth = eth;

	airoha_qdma_wr(eth, REG_TX_IRQ_BASE(id), dma_addr);
	airoha_qdma_rmw(eth, REG_TX_IRQ_CFG(id), TX_IRQ_DEPTH_MASK,
			FIELD_PREP(TX_IRQ_DEPTH_MASK, size));
	airoha_qdma_rmw(eth, REG_TX_IRQ_CFG(id), TX_IRQ_THR_MASK,
			FIELD_PREP(TX_IRQ_THR_MASK, 1));

	return 0;
}

static int airoha_qdma_init_tx(struct airoha_eth *eth)
{
	int i, err;

	for (i = 0; i < ARRAY_SIZE(eth->q_tx_irq); i++) {
		err = airoha_qdma_tx_irq_init(eth, &eth->q_tx_irq[i],
					      IRQ_QUEUE_LEN(i));
		if (err)
			return err;
	}

	for (i = 0; i < ARRAY_SIZE(eth->q_tx); i++) {
		err = airoha_qdma_init_tx_queue(eth, &eth->q_tx[i],
						TX_DSCP_NUM);
		if (err)
			return err;
	}

	return 0;
}

static void airoha_qdma_cleanup_tx_queue(struct airoha_queue *q)
{
	struct airoha_eth *eth = q->eth;

	spin_lock_bh(&q->lock);
	while (q->queued) {
		struct airoha_queue_entry *e = &q->entry[q->tail];

		dma_unmap_single(eth->dev, e->dma_addr, e->dma_len,
				 DMA_TO_DEVICE);
		dev_kfree_skb_any(e->skb);
		e->skb = NULL;

		q->tail = (q->tail + 1) % q->ndesc;
		q->queued--;
	}
	spin_unlock_bh(&q->lock);
}

static int airoha_qdma_init_hfwd_queues(struct airoha_eth *eth)
{
	dma_addr_t dma_addr;
	u32 status;
	int size;

	size = HW_DSCP_NUM * sizeof(struct airoha_qdma_fwd_desc);
	eth->hfwd.desc = dmam_alloc_coherent(eth->dev, size, &dma_addr,
					     GFP_KERNEL);
	if (!eth->hfwd.desc)
		return -ENOMEM;

	airoha_qdma_wr(eth, REG_FWD_DSCP_BASE, dma_addr);

	size = AIROHA_MAX_PACKET_SIZE * HW_DSCP_NUM;
	eth->hfwd.q = dmam_alloc_coherent(eth->dev, size, &dma_addr,
					  GFP_KERNEL);
	if (!eth->hfwd.q)
		return -ENOMEM;

	airoha_qdma_wr(eth, REG_FWD_BUF_BASE, dma_addr);

	airoha_qdma_rmw(eth, REG_HW_FWD_DSCP_CFG,
			HW_FWD_DSCP_PAYLOAD_SIZE_MASK,
			FIELD_PREP(HW_FWD_DSCP_PAYLOAD_SIZE_MASK, 0));
	airoha_qdma_rmw(eth, REG_FWD_DSCP_LOW_THR, FWD_DSCP_LOW_THR_MASK,
			FIELD_PREP(FWD_DSCP_LOW_THR_MASK, 128));
	airoha_qdma_rmw(eth, REG_LMGR_INIT_CFG,
			LMGR_INIT_START | LMGR_SRAM_MODE_MASK |
			HW_FWD_DESC_NUM_MASK,
			FIELD_PREP(HW_FWD_DESC_NUM_MASK, HW_DSCP_NUM) |
			LMGR_INIT_START);

	return read_poll_timeout(airoha_qdma_rr, status,
				 !(status & LMGR_INIT_START), USEC_PER_MSEC,
				 30 * USEC_PER_MSEC, true, eth,
				 REG_LMGR_INIT_CFG);
}

static void airoha_qdma_init_qos(struct airoha_eth *eth)
{
	airoha_qdma_clear(eth, REG_TXWRR_MODE_CFG, TWRR_WEIGHT_SCALE_MASK);
	airoha_qdma_set(eth, REG_TXWRR_MODE_CFG, TWRR_WEIGHT_BASE_MASK);

	airoha_qdma_clear(eth, REG_PSE_BUF_USAGE_CFG,
			  PSE_BUF_ESTIMATE_EN_MASK);

	airoha_qdma_set(eth, REG_EGRESS_RATE_METER_CFG,
			EGRESS_RATE_METER_EN_MASK |
			EGRESS_RATE_METER_EQ_RATE_EN_MASK);
	/* 2047us x 31 = 63.457ms */
	airoha_qdma_rmw(eth, REG_EGRESS_RATE_METER_CFG,
			EGRESS_RATE_METER_WINDOW_SZ_MASK,
			FIELD_PREP(EGRESS_RATE_METER_WINDOW_SZ_MASK, 0x1f));
	airoha_qdma_rmw(eth, REG_EGRESS_RATE_METER_CFG,
			EGRESS_RATE_METER_TIMESLICE_MASK,
			FIELD_PREP(EGRESS_RATE_METER_TIMESLICE_MASK, 0x7ff));

	/* ratelimit init */
	airoha_qdma_set(eth, REG_GLB_TRTCM_CFG, GLB_TRTCM_EN_MASK);
	airoha_qdma_rmw(eth, REG_GLB_TRTCM_CFG, GLB_FAST_TICK_MASK,
			FIELD_PREP(GLB_FAST_TICK_MASK, 25)); /* fast-tick 25us */
	airoha_qdma_rmw(eth, REG_GLB_TRTCM_CFG, GLB_SLOW_TICK_RATIO_MASK,
			FIELD_PREP(GLB_SLOW_TICK_RATIO_MASK, 40));

	airoha_qdma_set(eth, REG_EGRESS_TRTCM_CFG, EGRESS_TRTCM_EN_MASK);
	airoha_qdma_rmw(eth, REG_EGRESS_TRTCM_CFG, EGRESS_FAST_TICK_MASK,
			FIELD_PREP(EGRESS_FAST_TICK_MASK, 25));
	airoha_qdma_rmw(eth, REG_EGRESS_TRTCM_CFG,
			EGRESS_SLOW_TICK_RATIO_MASK,
			FIELD_PREP(EGRESS_SLOW_TICK_RATIO_MASK, 40));

	airoha_qdma_set(eth, REG_INGRESS_TRTCM_CFG, INGRESS_TRTCM_EN_MASK);
	airoha_qdma_clear(eth, REG_INGRESS_TRTCM_CFG,
			  INGRESS_TRTCM_MODE_MASK);
	airoha_qdma_rmw(eth, REG_INGRESS_TRTCM_CFG, INGRESS_FAST_TICK_MASK,
			FIELD_PREP(INGRESS_FAST_TICK_MASK, 125));
	airoha_qdma_rmw(eth, REG_INGRESS_TRTCM_CFG,
			INGRESS_SLOW_TICK_RATIO_MASK,
			FIELD_PREP(INGRESS_SLOW_TICK_RATIO_MASK, 8));

	airoha_qdma_set(eth, REG_SLA_TRTCM_CFG, SLA_TRTCM_EN_MASK);
	airoha_qdma_rmw(eth, REG_SLA_TRTCM_CFG, SLA_FAST_TICK_MASK,
			FIELD_PREP(SLA_FAST_TICK_MASK, 25));
	airoha_qdma_rmw(eth, REG_SLA_TRTCM_CFG, SLA_SLOW_TICK_RATIO_MASK,
			FIELD_PREP(SLA_SLOW_TICK_RATIO_MASK, 40));
}

static int airoha_qdma_hw_init(struct airoha_eth *eth)
{
	int i;

	/* clear pending irqs */
	for (i = 0; i < ARRAY_SIZE(eth->irqmask); i++)
		airoha_qdma_wr(eth, REG_INT_STATUS(i), 0xffffffff);

	/* setup irqs */
	airoha_qdma_irq_enable(eth, QDMA_INT_REG_IDX0, INT_IDX0_MASK);
	airoha_qdma_irq_enable(eth, QDMA_INT_REG_IDX1, INT_IDX1_MASK);
	airoha_qdma_irq_enable(eth, QDMA_INT_REG_IDX4, INT_IDX4_MASK);

	/* setup irq binding */
	airoha_qdma_for_each_q_tx(eth, i) {
		if (TX_RING_IRQ_BLOCKING_MAP_MASK & BIT(i))
			airoha_qdma_set(eth, REG_TX_RING_BLOCKING(i),
					TX_RING_IRQ_BLOCKING_CFG_MASK);
		else
			airoha_qdma_clear(eth, REG_TX_RING_BLOCKING(i),
					  TX_RING_IRQ_BLOCKING_CFG_MASK);
	}

	airoha_qdma_wr(eth, REG_QDMA_GLOBAL_CFG,
		       GLOBAL_CFG_RX_2B_OFFSET_MASK |
		       FIELD_PREP(GLOBAL_CFG_DMA_PREFERENCE_MASK, 3) |
		       GLOBAL_CFG_CPU_TXR_RR_MASK |
		       GLOBAL_CFG_PAYLOAD_BYTE_SWAP_MASK |
		       GLOBAL_CFG_MULTICAST_MODIFY_FP_MASK |
		       GLOBAL_CFG_MULTICAST_EN_MASK |
		       GLOBAL_CFG_IRQ0_EN_MASK | GLOBAL_CFG_IRQ1_EN_MASK |
		       GLOBAL_CFG_TX_WB_DONE_MASK |
		       FIELD_PREP(GLOBAL_CFG_MAX_ISSUE_NUM_MASK, 2));

	airoha_qdma_init_qos(eth);

	/* disable qdma rx delay interrupt */
	airoha_qdma_for_each_q_rx(eth, i)
		airoha_qdma_clear(eth, REG_RX_DELAY_INT_IDX(i),
				  RX_DELAY_INT_MASK);

	airoha_qdma_set(eth, REG_TXQ_CNGST_CFG,
			TXQ_CNGST_DROP_EN | TXQ_CNGST_DEI_DROP_EN);

	return 0;
}

static irqreturn_t airoha_irq_handler(int irq, void *dev_instance)
{
	struct airoha_eth *eth = dev_instance;
	u32 intr[ARRAY_SIZE(eth->irqmask)];
	int i;

	for (i = 0; i < ARRAY_SIZE(eth->irqmask); i++) {
		intr[i] = airoha_qdma_rr(eth, REG_INT_STATUS(i));
		intr[i] &= eth->irqmask[i];
		airoha_qdma_wr(eth, REG_INT_STATUS(i), intr[i]);
	}

	if (!test_bit(DEV_STATE_INITIALIZED, &eth->state))
		return IRQ_NONE;

	if (intr[1] & RX_DONE_INT_MASK) {
		airoha_qdma_irq_disable(eth, QDMA_INT_REG_IDX1,
					RX_DONE_INT_MASK);
		airoha_qdma_for_each_q_rx(eth, i) {
			if (intr[1] & BIT(i))
				napi_schedule(&eth->q_rx[i].napi);
		}
	}

	if (intr[0] & INT_TX_MASK) {
		for (i = 0; i < ARRAY_SIZE(eth->q_tx_irq); i++) {
			struct airoha_tx_irq_queue *irq_q = &eth->q_tx_irq[i];
			u32 status, head;

			if (!(intr[0] & TX_DONE_INT_MASK(i)))
				continue;

			airoha_qdma_irq_disable(eth, QDMA_INT_REG_IDX0,
						TX_DONE_INT_MASK(i));

			status = airoha_qdma_rr(eth, REG_IRQ_STATUS(i));
			head = FIELD_GET(IRQ_HEAD_IDX_MASK, status);
			irq_q->head = head % irq_q->size;
			irq_q->queued = FIELD_GET(IRQ_ENTRY_LEN_MASK, status);

			napi_schedule(&eth->q_tx_irq[i].napi);
		}
	}

	return IRQ_HANDLED;
}

static int airoha_qdma_init(struct airoha_eth *eth)
{
	int err;

	err = devm_request_irq(eth->dev, eth->irq, airoha_irq_handler,
			       IRQF_SHARED, KBUILD_MODNAME, eth);
	if (err)
		return err;

	err = airoha_qdma_init_rx(eth);
	if (err)
		return err;

	err = airoha_qdma_init_tx(eth);
	if (err)
		return err;

	err = airoha_qdma_init_hfwd_queues(eth);
	if (err)
		return err;

	err = airoha_qdma_hw_init(eth);
	if (err)
		return err;

	set_bit(DEV_STATE_INITIALIZED, &eth->state);

	return 0;
}

static int airoha_hw_init(struct airoha_eth *eth)
{
	int err;

	/* disable xsi */
	reset_control_bulk_assert(ARRAY_SIZE(eth->xsi_rsts), eth->xsi_rsts);

	reset_control_bulk_assert(ARRAY_SIZE(eth->rsts), eth->rsts);
	msleep(20);
	reset_control_bulk_deassert(ARRAY_SIZE(eth->rsts), eth->rsts);
	msleep(20);

	err = airoha_fe_init(eth);
	if (err)
		return err;

	return airoha_qdma_init(eth);
}

static void airoha_hw_cleanup(struct airoha_eth *eth)
{
	int i;

	airoha_qdma_for_each_q_rx(eth, i) {
		struct airoha_queue *q = &eth->q_rx[i];

		netif_napi_del(&q->napi);
		airoha_qdma_cleanup_rx_queue(q);
		if (q->page_pool)
			page_pool_destroy(q->page_pool);
	}

	for (i = 0; i < ARRAY_SIZE(eth->q_tx_irq); i++)
		netif_napi_del(&eth->q_tx_irq[i].napi);

	airoha_qdma_for_each_q_tx(eth, i)
		airoha_qdma_cleanup_tx_queue(&eth->q_tx[i]);
}

static void airoha_qdma_start_napi(struct airoha_eth *eth)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(eth->q_tx_irq); i++)
		napi_enable(&eth->q_tx_irq[i].napi);

	airoha_qdma_for_each_q_rx(eth, i)
		napi_enable(&eth->q_rx[i].napi);
}

static int airoha_dev_open(struct net_device *dev)
{
	struct airoha_gdm_port *port = netdev_priv(dev);
	struct airoha_eth *eth = port->eth;
	int err;

	netif_tx_start_all_queues(dev);
	err = airoha_set_gdm_ports(eth, true);
	if (err)
		return err;

	if (netdev_uses_dsa(dev))
		airoha_fe_set(eth, REG_GDM_INGRESS_CFG(port->id),
			      GDM_STAG_EN_MASK);
	else
		airoha_fe_clear(eth, REG_GDM_INGRESS_CFG(port->id),
				GDM_STAG_EN_MASK);

	airoha_qdma_set(eth, REG_QDMA_GLOBAL_CFG, GLOBAL_CFG_TX_DMA_EN_MASK);
	airoha_qdma_set(eth, REG_QDMA_GLOBAL_CFG, GLOBAL_CFG_RX_DMA_EN_MASK);

	return 0;
}

static int airoha_dev_stop(struct net_device *dev)
{
	struct airoha_gdm_port *port = netdev_priv(dev);
	struct airoha_eth *eth = port->eth;
	int err;

	netif_tx_disable(dev);
	err = airoha_set_gdm_ports(eth, false);
	if (err)
		return err;

	airoha_qdma_clear(eth, REG_QDMA_GLOBAL_CFG, GLOBAL_CFG_TX_DMA_EN_MASK);
	airoha_qdma_clear(eth, REG_QDMA_GLOBAL_CFG, GLOBAL_CFG_RX_DMA_EN_MASK);

	return 0;
}

static int airoha_dev_set_macaddr(struct net_device *dev, void *p)
{
	struct airoha_gdm_port *port = netdev_priv(dev);
	int err;

	err = eth_mac_addr(dev, p);
	if (err)
		return err;

	airoha_set_macaddr(port->eth, dev->dev_addr);

	return 0;
}

static int airoha_dev_init(struct net_device *dev)
{
	struct airoha_gdm_port *port = netdev_priv(dev);

	airoha_set_macaddr(port->eth, dev->dev_addr);

	return 0;
}

static netdev_tx_t airoha_dev_xmit(struct sk_buff *skb,
				   struct net_device *dev)
{
	struct skb_shared_info *sinfo = skb_shinfo(skb);
	struct airoha_gdm_port *port = netdev_priv(dev);
	int i, qid = skb_get_queue_mapping(skb);
	struct airoha_eth *eth = port->eth;
	u32 nr_frags, msg0 = 0, msg1;
	u32 len = skb_headlen(skb);
	struct netdev_queue *txq;
	struct airoha_queue *q;
	void *data = skb->data;
	u16 index;
	u8 fport;

	if (skb->ip_summed == CHECKSUM_PARTIAL)
		msg0 |= FIELD_PREP(QDMA_ETH_TXMSG_TCO_MASK, 1) |
			FIELD_PREP(QDMA_ETH_TXMSG_UCO_MASK, 1) |
			FIELD_PREP(QDMA_ETH_TXMSG_ICO_MASK, 1);

	/* TSO: fill MSS info in tcp checksum field */
	if (skb_is_gso(skb)) {
		if (skb_cow_head(skb, 0))
			goto error;

		if (sinfo->gso_type & (SKB_GSO_TCPV4 | SKB_GSO_TCPV6)) {
			__be16 csum = cpu_to_be16(sinfo->gso_size);

			tcp_hdr(skb)->check = (__force __sum16)csum;
			msg0 |= FIELD_PREP(QDMA_ETH_TXMSG_TSO_MASK, 1);
		}
	}

	fport = port->id == 3 ? DPORT_GDM4 : port->id + 1;
	msg1 = FIELD_PREP(QDMA_ETH_TXMSG_FPORT_MASK, fport) |
	       FIELD_PREP(QDMA_ETH_TXMSG_METER_MASK, 0x7f);

	if (WARN_ON_ONCE(qid >= ARRAY_SIZE(eth->q_tx)))
		qid = 0;

	q = &eth->q_tx[qid];
	if (WARN_ON_ONCE(!q->ndesc))
		goto error;

	spin_lock_bh(&q->lock);

	nr_frags = 1 + sinfo->nr_frags;
	if (q->queued + nr_frags > q->ndesc) {
		/* not enough space in the queue */
		spin_unlock_bh(&q->lock);
		return NETDEV_TX_BUSY;
	}

	index = q->head;
	for (i = 0; i < nr_frags; i++) {
		struct airoha_qdma_desc *desc = &q->desc[index];
		struct airoha_queue_entry *e = &q->entry[index];
		skb_frag_t *frag = &sinfo->frags[i];
		dma_addr_t addr;
		u32 val;

		addr = dma_map_single(dev->dev.parent, data, len,
				      DMA_TO_DEVICE);
		if (unlikely(dma_mapping_error(dev->dev.parent, addr)))
			goto error_unmap;

		index = (index + 1) % q->ndesc;

		val = FIELD_PREP(QDMA_DESC_LEN_MASK, len);
		if (i < nr_frags - 1)
			val |= FIELD_PREP(QDMA_DESC_MORE_MASK, 1);
		WRITE_ONCE(desc->ctrl, cpu_to_le32(val));
		WRITE_ONCE(desc->addr, cpu_to_le32(addr));
		val = FIELD_PREP(QDMA_DESC_NEXT_ID_MASK, index);
		WRITE_ONCE(desc->data, cpu_to_le32(val));
		WRITE_ONCE(desc->msg0, cpu_to_le32(msg0));
		WRITE_ONCE(desc->msg1, cpu_to_le32(msg1));
		WRITE_ONCE(desc->msg2, cpu_to_le32(0xffff));

		e->skb = i ? NULL : skb;
		e->dma_addr = addr;
		e->dma_len = len;

		airoha_qdma_rmw(eth, REG_TX_CPU_IDX(qid), TX_RING_CPU_IDX_MASK,
				FIELD_PREP(TX_RING_CPU_IDX_MASK, index));

		data = skb_frag_address(frag);
		len = skb_frag_size(frag);
	}

	q->head = index;
	q->queued += i;

	txq = netdev_get_tx_queue(dev, qid);
	netdev_tx_sent_queue(txq, skb->len);
	skb_tx_timestamp(skb);

	if (q->ndesc - q->queued < q->free_thr)
		netif_tx_stop_queue(txq);

	spin_unlock_bh(&q->lock);

	return NETDEV_TX_OK;

error_unmap:
	for (i--; i >= 0; i++)
		dma_unmap_single(dev->dev.parent, q->entry[i].dma_addr,
				 q->entry[i].dma_len, DMA_TO_DEVICE);

	spin_unlock_bh(&q->lock);
error:
	dev_kfree_skb_any(skb);
	dev->stats.tx_dropped++;

	return NETDEV_TX_OK;
}

static const char *airoha_ethtool_stats_name[] = {
	"tx_eth_pkt_cnt",
	"tx_eth_byte_cnt",
	"tx_ok_pkt_cnt",
	"tx_ok_byte_cnt",
	"tx_eth_drop_cnt",
	"tx_eth_bc_cnt",
	"tx_eth_mc_cnt",
	"tx_eth_lt64_cnt",
	"tx_eth_eq64_cnt",
	"tx_eth_65_127_cnt",
	"tx_eth_128_255_cnt",
	"tx_eth_256_511_cnt",
	"tx_eth_512_1023_cnt",
	"tx_eth_1024_1518_cnt",
	"tx_eth_gt1518_cnt",
	"rx_eth_pkt_cnt",
	"rx_eth_byte_cnt",
	"rx_ok_pkt_cnt",
	"rx_ok_byte_cnt",
	"rx_eth_drop_cnt",
	"rx_eth_bc_cnt",
	"rx_eth_mc_cnt",
	"rx_eth_crc_drop_cnt",
	"rx_eth_frag_cnt",
	"rx_eth_jabber_cnt",
	"rx_eth_lt64_cnt",
	"rx_eth_eq64_cnt",
	"rx_eth_65_127_cnt",
	"rx_eth_128_255_cnt",
	"rx_eth_256_511_cnt",
	"rx_eth_512_1023_cnt",
	"rx_eth_1024_1518_cnt",
	"rx_eth_gt1518_cnt",
};

static void airoha_ethtool_get_strings(struct net_device *dev, u32 sset,
				       u8 *data)
{
	int i;

	if (sset != ETH_SS_STATS)
		return;

	for (i = 0; i < ARRAY_SIZE(airoha_ethtool_stats_name); i++) {
		memcpy(data + i * ETH_GSTRING_LEN,
		       airoha_ethtool_stats_name[i], ETH_GSTRING_LEN);
	}

	data += ETH_GSTRING_LEN * ARRAY_SIZE(airoha_ethtool_stats_name);
	page_pool_ethtool_stats_get_strings(data);
}

static int airoha_ethtool_get_sset_count(struct net_device *dev, int sset)
{
	if (sset != ETH_SS_STATS)
		return -EOPNOTSUPP;

	return ARRAY_SIZE(airoha_ethtool_stats_name) +
	       page_pool_ethtool_stats_get_count();
}

static void airoha_update_hw_stats(struct airoha_gdm_port *port)
{
	struct airoha_eth *eth = port->eth;
	u32 val, i = 0;

	/* TX */
	val = airoha_fe_rr(eth, REG_FE_GDM_TX_ETH_PKT_CNT_H(port->id));
	port->hw_stats[i] += ((u64)val << 32);
	val = airoha_fe_rr(eth, REG_FE_GDM_TX_ETH_PKT_CNT_L(port->id));
	port->hw_stats[i++] += val;

	val = airoha_fe_rr(eth, REG_FE_GDM_TX_ETH_BYTE_CNT_H(port->id));
	port->hw_stats[i] += ((u64)val << 32);
	val = airoha_fe_rr(eth, REG_FE_GDM_TX_ETH_BYTE_CNT_L(port->id));
	port->hw_stats[i++] += val;

	val = airoha_fe_rr(eth, REG_FE_GDM_TX_OK_PKT_CNT_H(port->id));
	port->hw_stats[i] += ((u64)val << 32);
	val = airoha_fe_rr(eth, REG_FE_GDM_TX_OK_PKT_CNT_L(port->id));
	port->hw_stats[i++] += val;

	val = airoha_fe_rr(eth, REG_FE_GDM_TX_OK_BYTE_CNT_H(port->id));
	port->hw_stats[i] += ((u64)val << 32);
	val = airoha_fe_rr(eth, REG_FE_GDM_TX_OK_BYTE_CNT_L(port->id));
	port->hw_stats[i++] += val;

	val = airoha_fe_rr(eth, REG_FE_GDM_TX_ETH_DROP_CNT(port->id));
	port->hw_stats[i++] += val;

	val = airoha_fe_rr(eth, REG_FE_GDM_TX_ETH_BC_CNT(port->id));
	port->hw_stats[i++] += val;

	val = airoha_fe_rr(eth, REG_FE_GDM_TX_ETH_MC_CNT(port->id));
	port->hw_stats[i++] += val;

	val = airoha_fe_rr(eth, REG_FE_GDM_TX_ETH_RUNT_CNT(port->id));
	port->hw_stats[i++] += val;

	val = airoha_fe_rr(eth, REG_FE_GDM_TX_ETH_E64_CNT_H(port->id));
	port->hw_stats[i] += ((u64)val << 32);
	val = airoha_fe_rr(eth, REG_FE_GDM_TX_ETH_E64_CNT_L(port->id));
	port->hw_stats[i++] += val;

	val = airoha_fe_rr(eth, REG_FE_GDM_TX_ETH_L127_CNT_H(port->id));
	port->hw_stats[i] += ((u64)val << 32);
	val = airoha_fe_rr(eth, REG_FE_GDM_TX_ETH_L127_CNT_L(port->id));
	port->hw_stats[i++] += val;

	val = airoha_fe_rr(eth, REG_FE_GDM_TX_ETH_L255_CNT_H(port->id));
	port->hw_stats[i] += ((u64)val << 32);
	val = airoha_fe_rr(eth, REG_FE_GDM_TX_ETH_L255_CNT_L(port->id));
	port->hw_stats[i++] += val;

	val = airoha_fe_rr(eth, REG_FE_GDM_TX_ETH_L511_CNT_H(port->id));
	port->hw_stats[i] += ((u64)val << 32);
	val = airoha_fe_rr(eth, REG_FE_GDM_TX_ETH_L511_CNT_L(port->id));
	port->hw_stats[i++] += val;

	val = airoha_fe_rr(eth, REG_FE_GDM_TX_ETH_L1023_CNT_H(port->id));
	port->hw_stats[i] += ((u64)val << 32);
	val = airoha_fe_rr(eth, REG_FE_GDM_TX_ETH_L1023_CNT_L(port->id));
	port->hw_stats[i++] += val;

	val = airoha_fe_rr(eth, REG_FE_GDM_TX_ETH_LONG_CNT(port->id));
	port->hw_stats[i++] += val;

	/* RX */
	val = airoha_fe_rr(eth, REG_FE_GDM_RX_ETH_PKT_CNT_H(port->id));
	port->hw_stats[i] += ((u64)val << 32);
	val = airoha_fe_rr(eth, REG_FE_GDM_RX_ETH_PKT_CNT_L(port->id));
	port->hw_stats[i++] += val;

	val = airoha_fe_rr(eth, REG_FE_GDM_RX_ETH_BYTE_CNT_H(port->id));
	port->hw_stats[i] += ((u64)val << 32);
	val = airoha_fe_rr(eth, REG_FE_GDM_RX_ETH_BYTE_CNT_L(port->id));
	port->hw_stats[i++] += val;

	val = airoha_fe_rr(eth, REG_FE_GDM_RX_OK_PKT_CNT_H(port->id));
	port->hw_stats[i] += ((u64)val << 32);
	val = airoha_fe_rr(eth, REG_FE_GDM_RX_OK_PKT_CNT_L(port->id));
	port->hw_stats[i++] += val;

	val = airoha_fe_rr(eth, REG_FE_GDM_RX_OK_BYTE_CNT_H(port->id));
	port->hw_stats[i] += ((u64)val << 32);
	val = airoha_fe_rr(eth, REG_FE_GDM_RX_OK_BYTE_CNT_L(port->id));
	port->hw_stats[i++] += val;

	val = airoha_fe_rr(eth, REG_FE_GDM_RX_ETH_DROP_CNT(port->id));
	port->hw_stats[i++] += val;

	val = airoha_fe_rr(eth, REG_FE_GDM_RX_ETH_BC_CNT(port->id));
	port->hw_stats[i++] += val;

	val = airoha_fe_rr(eth, REG_FE_GDM_RX_ETH_MC_CNT(port->id));
	port->hw_stats[i++] += val;

	val = airoha_fe_rr(eth, REG_FE_GDM_RX_ETH_CRC_ERR_CNT(port->id));
	port->hw_stats[i++] += val;

	val = airoha_fe_rr(eth, REG_FE_GDM_RX_ETH_FRAG_CNT(port->id));
	port->hw_stats[i++] += val;

	val = airoha_fe_rr(eth, REG_FE_GDM_RX_ETH_JABBER_CNT(port->id));
	port->hw_stats[i++] += val;

	val = airoha_fe_rr(eth, REG_FE_GDM_RX_ETH_RUNT_CNT(port->id));
	port->hw_stats[i++] += val;

	val = airoha_fe_rr(eth, REG_FE_GDM_RX_ETH_E64_CNT_H(port->id));
	port->hw_stats[i] += ((u64)val << 32);
	val = airoha_fe_rr(eth, REG_FE_GDM_RX_ETH_E64_CNT_L(port->id));
	port->hw_stats[i++] += val;

	val = airoha_fe_rr(eth, REG_FE_GDM_RX_ETH_L127_CNT_H(port->id));
	port->hw_stats[i] += ((u64)val << 32);
	val = airoha_fe_rr(eth, REG_FE_GDM_RX_ETH_L127_CNT_L(port->id));
	port->hw_stats[i++] += val;

	val = airoha_fe_rr(eth, REG_FE_GDM_RX_ETH_L255_CNT_H(port->id));
	port->hw_stats[i] += ((u64)val << 32);
	val = airoha_fe_rr(eth, REG_FE_GDM_RX_ETH_L255_CNT_L(port->id));
	port->hw_stats[i++] += val;

	val = airoha_fe_rr(eth, REG_FE_GDM_RX_ETH_L511_CNT_H(port->id));
	port->hw_stats[i] += ((u64)val << 32);
	val = airoha_fe_rr(eth, REG_FE_GDM_RX_ETH_L511_CNT_L(port->id));
	port->hw_stats[i++] += val;

	val = airoha_fe_rr(eth, REG_FE_GDM_RX_ETH_L1023_CNT_H(port->id));
	port->hw_stats[i] += ((u64)val << 32);
	val = airoha_fe_rr(eth, REG_FE_GDM_RX_ETH_L1023_CNT_L(port->id));
	port->hw_stats[i++] += val;

	val = airoha_fe_rr(eth, REG_FE_GDM_RX_ETH_LONG_CNT(port->id));
	port->hw_stats[i++] += val;

	/* reset mib counters */
	airoha_fe_set(eth, REG_FE_GDM_MIB_CLEAR(port->id),
		      FE_GDM_MIB_RX_CLEAR_MASK | FE_GDM_MIB_TX_CLEAR_MASK);
}

static void airoha_ethtool_get_stats(struct net_device *dev,
				     struct ethtool_stats *stats, u64 *data)
{
	struct airoha_gdm_port *port = netdev_priv(dev);
	struct page_pool_stats pp_stats = {};
	struct airoha_eth *eth = port->eth;
	int i;

	airoha_update_hw_stats(port);
	for (i = 0; i < ARRAY_SIZE(airoha_ethtool_stats_name); i++)
		*data++ = port->hw_stats[i];

	airoha_qdma_for_each_q_rx(eth, i)
		page_pool_get_stats(eth->q_rx[i].page_pool, &pp_stats);
	page_pool_ethtool_stats_get(data, &pp_stats);
}

static const struct net_device_ops airoha_netdev_ops = {
	.ndo_init		= airoha_dev_init,
	.ndo_open		= airoha_dev_open,
	.ndo_stop		= airoha_dev_stop,
	.ndo_start_xmit		= airoha_dev_xmit,
	.ndo_set_mac_address	= airoha_dev_set_macaddr,
};

static const struct ethtool_ops airoha_ethtool_ops = {
	.get_strings		= airoha_ethtool_get_strings,
	.get_sset_count		= airoha_ethtool_get_sset_count,
	.get_ethtool_stats	= airoha_ethtool_get_stats,
};

static int airoha_fe_reg_set(void *data, u64 val)
{
	struct airoha_eth *eth = data;

	airoha_fe_wr(eth, eth->debugfs_reg, val);

	return 0;
}

static int airoha_fe_reg_get(void *data, u64 *val)
{
	struct airoha_eth *eth = data;

	*val = airoha_fe_rr(eth, eth->debugfs_reg);

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_fe, airoha_fe_reg_get, airoha_fe_reg_set,
			 "0x%08llx\n");

static int airoha_qdma_reg_set(void *data, u64 val)
{
	struct airoha_eth *eth = data;

	airoha_qdma_wr(eth, eth->debugfs_reg, val);

	return 0;
}

static int airoha_qdma_reg_get(void *data, u64 *val)
{
	struct airoha_eth *eth = data;

	*val = airoha_qdma_rr(eth, eth->debugfs_reg);

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_qdma, airoha_qdma_reg_get, airoha_qdma_reg_set,
			 "0x%08llx\n");

static int airoha_rx_queues_show(struct seq_file *s, void *data)
{
	struct airoha_eth *eth = s->private;
	int i;

	seq_puts(s, "     queue | hw-queued |      head |      tail |\n");
	airoha_qdma_for_each_q_rx(eth, i) {
		struct airoha_queue *q = &eth->q_rx[i];

		seq_printf(s, " %9d | %9d | %9d | %9d |\n",
			   i, q->queued, q->head, q->tail);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(airoha_rx_queues);

static int airoha_xmit_queues_show(struct seq_file *s, void *data)
{
	struct airoha_eth *eth = s->private;
	int i;

	seq_puts(s, "     queue | hw-queued |      head |      tail |\n");
	airoha_qdma_for_each_q_tx(eth, i) {
		struct airoha_queue *q = &eth->q_tx[i];

		seq_printf(s, " %9d | %9d | %9d | %9d |\n",
			   i, q->queued, q->head, q->tail);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(airoha_xmit_queues);

static void airoha_register_debugfs(struct airoha_eth *eth)
{
	eth->debugfs_dir = debugfs_create_dir(KBUILD_MODNAME, NULL);
	debugfs_create_u32("regidx", 0600, eth->debugfs_dir,
			   &eth->debugfs_reg);
	debugfs_create_file_unsafe("fe_regval", 0600, eth->debugfs_dir, eth,
				   &fops_fe);
	debugfs_create_file_unsafe("qdma_regval", 0600, eth->debugfs_dir, eth,
				   &fops_qdma);
	debugfs_create_file("rx-queues", 0400, eth->debugfs_dir, eth,
			    &airoha_rx_queues_fops);
	debugfs_create_file("xmit-queues", 0400, eth->debugfs_dir, eth,
			    &airoha_xmit_queues_fops);
}

static int airoha_alloc_gdm_port(struct airoha_eth *eth, struct device_node *np)
{
	const __be32 *id_ptr = of_get_property(np, "reg", NULL);
	struct net_device *dev;
	struct airoha_gdm_port *port;
	int id, err;

	if (!id_ptr) {
		dev_err(eth->dev, "missing gdm port id\n");
		return -EINVAL;
	}

	id = be32_to_cpup(id_ptr);
	if (id >= ARRAY_SIZE(eth->ports)) {
		dev_err(eth->dev, "invalid gdm port id: %d\n", id);
		return -EINVAL;
	}

	if (eth->ports[id]) {
		dev_err(eth->dev, "duplicate gdm port id: %d\n", id);
		return -EINVAL;
	}

	dev = devm_alloc_etherdev_mqs(eth->dev, sizeof(*port),
				      AIROHA_NUM_TX_RING, AIROHA_NUM_RX_RING);
	if (!dev) {
		dev_err(eth->dev, "alloc_etherdev failed\n");
		return -ENOMEM;
	}

	dev->netdev_ops = &airoha_netdev_ops;
	dev->ethtool_ops = &airoha_ethtool_ops;
	dev->max_mtu = AIROHA_MAX_MTU;
	dev->watchdog_timeo = 5 * HZ;
	dev->hw_features = NETIF_F_IP_CSUM | NETIF_F_RXCSUM |
			   NETIF_F_TSO6 | NETIF_F_IPV6_CSUM |
			   NETIF_F_SG | NETIF_F_TSO;
	dev->features |= dev->hw_features;
	dev->dev.of_node = np;
	SET_NETDEV_DEV(dev, eth->dev);

	err = of_get_ethdev_address(np, dev);
	if (err) {
		if (err == -EPROBE_DEFER)
			return err;

		eth_hw_addr_random(dev);
		dev_info(eth->dev, "generated random MAC address %pM\n",
			 dev->dev_addr);
	}

	port = netdev_priv(dev);
	port->dev = dev;
	port->eth = eth;
	port->id = id;

	port->hw_stats = devm_kcalloc(eth->dev,
				      ARRAY_SIZE(airoha_ethtool_stats_name),
				      sizeof(*port->hw_stats), GFP_KERNEL);
	if (!port->hw_stats)
		return -ENOMEM;

	err = register_netdev(dev);
	if (err)
		return err;

	eth->ports[id] = port;

	return 0;
}

static int airoha_probe(struct platform_device *pdev)
{
	struct device_node *np;
	struct airoha_eth *eth;
	int i, err;

	eth = devm_kzalloc(&pdev->dev, sizeof(*eth), GFP_KERNEL);
	if (!eth)
		return -ENOMEM;

	eth->dev = &pdev->dev;

	err = dma_set_mask_and_coherent(eth->dev, DMA_BIT_MASK(32));
	if (err) {
		dev_err(eth->dev, "failed configuring DMA mask\n");
		return err;
	}

	eth->fe_regs = devm_platform_ioremap_resource_byname(pdev, "fe");
	if (IS_ERR(eth->fe_regs))
		return dev_err_probe(eth->dev, PTR_ERR(eth->fe_regs),
				     "failed to iomap fe regs\n");

	eth->qdma_regs = devm_platform_ioremap_resource_byname(pdev, "qdma0");
	if (IS_ERR(eth->qdma_regs))
		return dev_err_probe(eth->dev, PTR_ERR(eth->qdma_regs),
				     "failed to iomap qdma regs\n");

	eth->rsts[0].id = "fe";
	eth->rsts[1].id = "pdma";
	eth->rsts[2].id = "qdma";
	err = devm_reset_control_bulk_get_exclusive(eth->dev,
						    ARRAY_SIZE(eth->rsts),
						    eth->rsts);
	if (err) {
		dev_err(eth->dev, "failed to get bulk reset lines\n");
		return err;
	}

	eth->xsi_rsts[0].id = "xsi-mac";
	eth->xsi_rsts[1].id = "hsi0-mac";
	eth->xsi_rsts[2].id = "hsi1-mac";
	eth->xsi_rsts[3].id = "hsi-mac";
	eth->xsi_rsts[4].id = "xfp-mac";
	err = devm_reset_control_bulk_get_exclusive(eth->dev,
						    ARRAY_SIZE(eth->xsi_rsts),
						    eth->xsi_rsts);
	if (err) {
		dev_err(eth->dev, "failed to get bulk xsi reset lines\n");
		return err;
	}

	spin_lock_init(&eth->irq_lock);
	eth->irq = platform_get_irq(pdev, 0);
	if (eth->irq < 0)
		return eth->irq;

	init_dummy_netdev(&eth->napi_dev);
	platform_set_drvdata(pdev, eth);
	err = airoha_hw_init(eth);
	if (err)
		goto error;

	for_each_child_of_node(pdev->dev.of_node, np) {
		if (!of_device_is_compatible(np, "airoha,eth-mac"))
			continue;

		if (!of_device_is_available(np))
			continue;

		err = airoha_alloc_gdm_port(eth, np);
		if (err) {
			of_node_put(np);
			goto error;
		}
	}

	airoha_qdma_start_napi(eth);
	airoha_register_debugfs(eth);

	return 0;

error:
	airoha_hw_cleanup(eth);
	for (i = 0; i < ARRAY_SIZE(eth->ports); i++) {
		if (eth->ports[i])
			unregister_netdev(eth->ports[i]->dev);
	}
	platform_set_drvdata(pdev, NULL);

	return err;
}

static void airoha_remove(struct platform_device *pdev)
{
	struct airoha_eth *eth = platform_get_drvdata(pdev);
	int i;

	debugfs_remove(eth->debugfs_dir);

	airoha_hw_cleanup(eth);
	for (i = 0; i < ARRAY_SIZE(eth->ports); i++) {
		struct airoha_gdm_port *port = eth->ports[i];

		if (!port)
			continue;

		airoha_dev_stop(port->dev);
		unregister_netdev(port->dev);
	}

	platform_set_drvdata(pdev, NULL);
}

static const struct of_device_id of_airoha_match[] = {
	{ .compatible = "airoha,en7581-eth" },
	{ /* sentinel */ }
};

static struct platform_driver airoha_driver = {
	.probe = airoha_probe,
	.remove_new = airoha_remove,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = of_airoha_match,
	},
};
module_platform_driver(airoha_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lorenzo Bianconi <lorenzo@kernel.org>");
MODULE_DESCRIPTION("Ethernet driver for Airoha SoC");
