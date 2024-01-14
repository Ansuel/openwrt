// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek PCIe host controller driver.
 *
 * Copyright (c) 2020 MediaTek Inc.
 * Author: Jianjun Wang <jianjun.wang@mediatek.com>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/pci.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/of_pci.h>

#include "../pci.h"

//#include <ecnt_event_global/ecnt_event_system.h>
//#include "asm/tc3162/tc3162.h"

//#include <dt-bindings/clock/en7523-clk.h>

#define PCIE_SETTING_REG		0x80
#define PCIE_PCI_IDS_1			0x9c
#define PCI_CLASS(class)		(class << 8)
#define PCIE_RC_MODE			BIT(0)

#define PCIE_CFGNUM_REG			0x140
#define PCIE_CFG_DEVFN(devfn)		((devfn) & GENMASK(7, 0))
#define PCIE_CFG_BUS(bus)		(((bus) << 8) & GENMASK(15, 8))
#define PCIE_CFG_BYTE_EN(bytes)		(((bytes) << 16) & GENMASK(19, 16))
#define PCIE_CFG_FORCE_BYTE_EN		BIT(20)
#define PCIE_CFG_OFFSET_ADDR		0x1000
#define PCIE_CFG_HEADER(bus, devfn) \
	(PCIE_CFG_BUS(bus) | PCIE_CFG_DEVFN(devfn))

#define PCIE_RST_CTRL_REG		0x148
#define PCIE_MAC_RSTB			BIT(0)
#define PCIE_PHY_RSTB			BIT(1)
#define PCIE_BRG_RSTB			BIT(2)
#define PCIE_PE_RSTB			BIT(3)

#define PCIE_LTSSM_STATUS_REG		0x150
#define PCIE_LTSSM_STATE_MASK		GENMASK(28, 24)
#define PCIE_LTSSM_STATE(val)		((val & PCIE_LTSSM_STATE_MASK) >> 24)
#define PCIE_LTSSM_STATE_L2_IDLE	0x14

#define PCIE_LINK_STATUS_REG		0x154
#define PCIE_PORT_LINKUP		BIT(8)

#define PCIE_MSI_SET_NUM		8
#define PCIE_MSI_IRQS_PER_SET		32
#define PCIE_MSI_IRQS_NUM \
	(PCIE_MSI_IRQS_PER_SET * PCIE_MSI_SET_NUM)

#define PCIE_INT_ENABLE_REG		0x180
#define PCIE_MSI_ENABLE			GENMASK(PCIE_MSI_SET_NUM + 8 - 1, 8)
#define PCIE_MSI_SHIFT			8
#define PCIE_INTX_SHIFT			24
#define PCIE_INTX_ENABLE \
	GENMASK(PCIE_INTX_SHIFT + PCI_NUM_INTX - 1, PCIE_INTX_SHIFT)

#define PCIE_INT_STATUS_REG		0x184
#define PCIE_MSI_SET_ENABLE_REG		0x190
#define PCIE_MSI_SET_ENABLE		GENMASK(PCIE_MSI_SET_NUM - 1, 0)

#define PCIE_MSI_SET_BASE_REG		0xc00
#define PCIE_MSI_SET_OFFSET		0x10
#define PCIE_MSI_SET_STATUS_OFFSET	0x04
#define PCIE_MSI_SET_ENABLE_OFFSET	0x08

#define PCIE_MSI_SET_ADDR_HI_BASE	0xc80
#define PCIE_MSI_SET_ADDR_HI_OFFSET	0x04

#define PCIE_ICMD_PM_REG		0x198
#define PCIE_TURN_OFF_LINK		BIT(4)

#define PCIE_TRANS_TABLE_BASE_REG	0x800
#define PCIE_ATR_SRC_ADDR_MSB_OFFSET	0x4
#define PCIE_ATR_TRSL_ADDR_LSB_OFFSET	0x8
#define PCIE_ATR_TRSL_ADDR_MSB_OFFSET	0xc
#define PCIE_ATR_TRSL_PARAM_OFFSET	0x10
#define PCIE_ATR_TLB_SET_OFFSET		0x20

#define PCIE_MAX_TRANS_TABLES		8
#define PCIE_ATR_EN			BIT(0)
#define PCIE_ATR_SIZE(size) \
	(((((size) - 1) << 1) & GENMASK(6, 1)) | PCIE_ATR_EN)
#define PCIE_ATR_ID(id)			((id) & GENMASK(3, 0))
#define PCIE_ATR_TYPE_MEM		PCIE_ATR_ID(0)
#define PCIE_ATR_TYPE_IO		PCIE_ATR_ID(1)
#define PCIE_ATR_TLP_TYPE(type)		(((type) << 16) & GENMASK(18, 16))
#define PCIE_ATR_TLP_TYPE_MEM		PCIE_ATR_TLP_TYPE(0)
#define PCIE_ATR_TLP_TYPE_IO		PCIE_ATR_TLP_TYPE(2)

//extern int pcie_api_init(void);


//extern int pcie_api_init(void);
extern void set_np_scu_data(u32 reg, u32 val);
extern u32 get_np_scu_data(u32 reg);


/*===========for PCIe pbus setting begin============================================ */
extern u32 GET_PBUS_PCIE0_BASE(void);
extern void SET_PBUS_PCIE0_BASE(u32 val);
extern u32 GET_PBUS_PCIE0_MASK(void);
extern void SET_PBUS_PCIE0_MASK(u32 val);
extern u32 GET_PBUS_PCIE1_BASE(void);
extern void SET_PBUS_PCIE1_BASE(u32 val);
extern u32 GET_PBUS_PCIE1_MASK(void);
extern void SET_PBUS_PCIE1_MASK(u32 val);
extern u32 GET_PBUS_PCIE2_BASE(void);
extern void SET_PBUS_PCIE2_BASE(u32 val);
extern u32 GET_PBUS_PCIE2_MASK(void);
extern void SET_PBUS_PCIE2_MASK(u32 val);
/*===========for PCIe pbus setting end============================================ */

/*===========for PCIe pad open drain setting begin============================================ */
extern u32 GET_SCU_RGS_OPEN_DRAIN(void);
extern void SET_SCU_RGS_OPEN_DRAIN(u32 val);
/*===========for PCIe pad open drain setting end============================================ */



/**********************************************************/
struct ecnt_pcie{
	struct device *dev;
	int irq_pcie0;
	int irq_pcie1;
	int irq_pcie2;

	void __iomem *mac0_base; /* PCIe Mac0 base virtual address */
	void __iomem *mac1_base; /* PCIe Mac1 base virtual address */
	void __iomem *mac2_base; /* PCIe Mac2 base virtual address */
#ifdef TCSUPPORT_PCIE_MSI
	phys_addr_t mac0_addr; /* PCIe Mac0 base physical address */
	phys_addr_t mac1_addr; /* PCIe Mac1 base physical address */
	phys_addr_t mac2_addr; /* PCIe Mac2 base physical address */
#endif

};
static struct ecnt_pcie ECNT_pcie;
/**********************************************************/
int base_num=0;

/*====================regs==================*/
u32 get_pcie_mac0_data(u32 reg)
{
	struct ecnt_pcie *ecnt_pcie=NULL;
	ecnt_pcie=&ECNT_pcie;

	return readl(ecnt_pcie->mac0_base+ reg);
}

void set_pcie_mac0_data(u32 reg, u32 val)
{
	struct ecnt_pcie *ecnt_pcie=NULL;
	ecnt_pcie=&ECNT_pcie;

	writel(val, ecnt_pcie->mac0_base + reg); 
}
u32 get_pcie_mac1_data(u32 reg)
{
	struct ecnt_pcie *ecnt_pcie=NULL;
	ecnt_pcie=&ECNT_pcie;

	return readl(ecnt_pcie->mac1_base+ reg);
}

void set_pcie_mac1_data(u32 reg, u32 val)
{
	struct ecnt_pcie *ecnt_pcie=NULL;
	ecnt_pcie=&ECNT_pcie;

	writel(val, ecnt_pcie->mac1_base + reg); 
}

u32 get_pcie_mac2_data(u32 reg)
{
	struct ecnt_pcie *ecnt_pcie=NULL;
	ecnt_pcie=&ECNT_pcie;

	return readl(ecnt_pcie->mac2_base+ reg);
}

void set_pcie_mac2_data(u32 reg, u32 val)
{
	struct ecnt_pcie *ecnt_pcie=NULL;
	ecnt_pcie=&ECNT_pcie;

	writel(val, ecnt_pcie->mac2_base + reg); 
}

u32 regRead_PCIe(u32 reg)		
{	
	u32 val=0xffffffff;

	switch(reg&0xffff0000)
		{
		case 0x1fb00000:
			val=get_np_scu_data(reg&0xfff);
			break;
		case 0x1fc00000:
			val=get_pcie_mac0_data(reg&0xffff);
			break;

		case 0x1fc10000:
			val=get_pcie_mac0_data(reg&0xfffff);
			break;

		case 0x1fc20000:
			val=get_pcie_mac1_data(reg&0xffff);
			break;
		case 0x1fc40000:
			val=get_pcie_mac2_data(reg&0xffff);
			break;
		}
	return val;		  
}		
void regWrite_PCIe(u32 reg, u32 val)	
{                                                	
    
	switch(reg&0xffff0000)
		{
		case 0x1fb00000:
			set_np_scu_data(reg&0xfff,val);
			break;
		case 0x1fc00000:
			set_pcie_mac0_data(reg&0xffff,val);
			break;

		case 0x1fc10000:
			set_pcie_mac0_data(reg&0xfffff,val);
			break;

		case 0x1fc20000:
			set_pcie_mac1_data(reg&0xffff,val);
			break;
		case 0x1fc40000:
			set_pcie_mac2_data(reg&0xffff,val);
			break;

		}
}

EXPORT_SYMBOL(regRead_PCIe);
EXPORT_SYMBOL(regWrite_PCIe);


/*====================regs==================*/
/*====================conf access ecnt==================*/
#if 1//defined(TCSUPPORT_CPU_EN7581)

int get_rc_port(unsigned char bus,unsigned char dev)
{
	//RC       0   1   2
	//bus/dev 0/0- 0/1 0/2
	//EP       0   1   2
	//bus/dev 1/0 2/0 3/0

	int rc = 4;

	if ((bus == 0) && (dev < 3))
    {
    	rc = dev;
    }
    else if ((bus == 1) && (dev == 0))
    {           
        rc = 0;
    }
    else if ((bus == 2) && (dev == 0) )
    {
        rc = 1;
    }
    else if ((bus == 3) && (dev == 0) )
    {
        rc = 2;
    }
    
	return rc;
}

int pcie_write_config_word_extend(unsigned char bus, unsigned char dev,unsigned char func, unsigned int reg, unsigned long int value)
{
	unsigned int rc;
	void __iomem *offset=NULL;	
	struct ecnt_pcie *ecnt_pcie=NULL;
	ecnt_pcie=&ECNT_pcie;

	rc = get_rc_port(bus,dev);

	if (rc == 0){
		offset = ecnt_pcie->mac0_base;
	}else if(rc == 1){
		offset = ecnt_pcie->mac1_base;
	}else if(rc == 2){
		offset = ecnt_pcie->mac2_base;
	}else{
		return 0xffffffff;
	}


	if(0==bus)  //RC
		writel_relaxed(0x1f0000, offset + PCIE_CFGNUM_REG);
	else        //EP
		writel_relaxed(0x1f0100, offset + PCIE_CFGNUM_REG);


	writel(value, offset + PCIE_CFG_OFFSET_ADDR + reg);


 	return 0;

}
EXPORT_SYMBOL(pcie_write_config_word_extend);

unsigned int pcie_read_config_word_extend(unsigned char bus,unsigned char dev,unsigned char func ,unsigned int reg)
{
	unsigned int val,rc;
	struct ecnt_pcie *ecnt_pcie=NULL;
	void __iomem *offset=NULL; 
	ecnt_pcie=&ECNT_pcie;
	
	rc = get_rc_port(bus,dev);


	if (rc == 0){
		offset = ecnt_pcie->mac0_base;
	}else if(rc == 1){
		offset = ecnt_pcie->mac1_base;
	}else if(rc == 2){
		offset = ecnt_pcie->mac2_base;
	}else{
		return 0xffffffff;
	}


	if(0==bus)  //RC
		writel_relaxed(0x1f0000, offset + PCIE_CFGNUM_REG);
	else        //EP
		writel_relaxed(0x1f0100, offset + PCIE_CFGNUM_REG);


	val = readl(offset + PCIE_CFG_OFFSET_ADDR + reg);
	
	return val;
}
EXPORT_SYMBOL(pcie_read_config_word_extend);
#endif
/*====================conf access ecnt==================*/

/********************PCIe MSI API begin**************************************/
#ifdef TCSUPPORT_PCIE_MSI
/** 
* ecnt_msi_teardown_irq - Destroy the MSI 
* @chip: MSI Chip descriptor 
* @irq: MSI IRQ to destroy 
*/
void ecnt_msi_teardown_irq(struct msi_controller *chip, unsigned int irq)
{	
printk("=========== ecnt_msi_teardown_irq \n");
}
/** 
* ecnt_pcie_msi_setup_irq - Setup MSI request 
* @chip: MSI chip pointer 
* @pdev: PCIe device pointer 
* @desc: MSI descriptor pointer 
* 
* Return: '0' on success and error value on failure 
*/
int ecnt_pcie_msi_setup_irqs(struct msi_controller *chip,struct pci_dev *pdev, int nvec, int type)
{	
	unsigned int irq;// = pdev->irq + 1;	
	struct msi_msg msg;		
	struct msi_desc *desc;	
	phys_addr_t msg_addr;	
	int i, ret;

	printk("=========== ecnt_pcie_msi_setup_irqs enter \n");	

	struct ecnt_pcie *ecnt_pcie=NULL;
	ecnt_pcie=&ECNT_pcie;
	if (!ecnt_pcie)
		return -ENOMEM;

	printk("=========== ecnt_pcie_msi_setup_irqs 1111 \n");	

        //7581MSI all port
	irq = pdev->irq;

	printk("\n===========ecnt_pcie_msi_setup_irqs irq:%d===\n",irq);

	desc = list_entry(pdev->dev.msi_list.next, struct msi_desc, list);	
	for (i = 0; i < nvec; i++) 
		{		
			if (irq_set_msi_desc_off(irq, i, desc)) 
				{			
				/* TODO: clear */			
				return -EINVAL;		
				}	
		}	
	desc->nvec_used = nvec;	
	desc->msi_attrib.multiple = order_base_2(nvec);	


        //7581MSI all port
	if(irq == ecnt_pcie->irq_pcie0)
		msg_addr = ecnt_pcie->mac0_addr + PCIE_MSI_SET_BASE_REG;
	else if(irq == ecnt_pcie->irq_pcie1)
		msg_addr = ecnt_pcie->mac1_addr + PCIE_MSI_SET_BASE_REG;
	else if(irq == ecnt_pcie->irq_pcie2)
		msg_addr = ecnt_pcie->mac2_addr + PCIE_MSI_SET_BASE_REG;

	msg.address_hi = 0;	
	msg.address_lo = msg_addr;	
	msg.data = 0;		
	pci_write_msi_msg(irq, &msg);	
	printk("\n===========RC0 msg_addr:%x===\n",msg_addr);
	printk("=========== ecnt_pcie_msi_setup_irqs end \n");	
	return 0;
}

/* MSI Chip Descriptor */
struct msi_controller ecnt_pcie_msi_controller = {	
		.setup_irqs = ecnt_pcie_msi_setup_irqs,	
		.teardown_irq = ecnt_msi_teardown_irq,
	};
#endif
/********************PCIe MSI API end**************************************/


/**
 * struct mtk_msi_set - MSI information for each set
 * @base: IO mapped register base
 * @msg_addr: MSI message address
 * @saved_irq_state: IRQ enable state saved at suspend time
 */
struct mtk_msi_set {
	void __iomem *base;
	phys_addr_t msg_addr;
	u32 saved_irq_state;
};

/**
 * struct mtk_pcie_port - PCIe port information
 * @dev: pointer to PCIe device
 * @base: IO mapped register base
 * @reg_base: physical register base
 * @irq: PCIe controller interrupt number
 * @saved_irq_state: IRQ enable state saved at suspend time
 * @irq_lock: lock protecting IRQ register access
 * @intx_domain: legacy INTx IRQ domain
 * @msi_domain: MSI IRQ domain
 * @msi_bottom_domain: MSI IRQ bottom domain
 * @msi_sets: MSI sets information
 * @lock: lock protecting IRQ bit map
 * @msi_irq_in_use: bit map for assigned MSI IRQ
 */
struct mtk_pcie_port {
	struct device *dev;
	void __iomem *base;
	phys_addr_t reg_base;
	unsigned int busnr;

	int irq;
	u32 saved_irq_state;
	raw_spinlock_t irq_lock;
	struct irq_domain *intx_domain;
	struct irq_domain *msi_domain;
	struct irq_domain *msi_bottom_domain;
	struct mtk_msi_set msi_sets[PCIE_MSI_SET_NUM];
	struct mutex lock;
	DECLARE_BITMAP(msi_irq_in_use, PCIE_MSI_IRQS_NUM);
};

/**
 * mtk_pcie_config_tlp_header() - Configure a configuration TLP header
 * @bus: PCI bus to query
 * @devfn: device/function number
 * @where: offset in config space
 * @size: data size in TLP header
 *
 * Set byte enable field and device information in configuration TLP header.
 */
static void mtk_pcie_config_tlp_header(struct pci_bus *bus, unsigned int devfn,
					int where, int size)
{
	struct mtk_pcie_port *port = bus->sysdata;
	int bytes;
	u32 val;

	bytes = (GENMASK(size - 1, 0) & 0xf) << (where & 0x3);

	val = PCIE_CFG_FORCE_BYTE_EN | PCIE_CFG_BYTE_EN(bytes) |
	      PCIE_CFG_HEADER(bus->number, devfn);

	writel_relaxed(val, port->base + PCIE_CFGNUM_REG);
}

static void __iomem *mtk_pcie_map_bus(struct pci_bus *bus, unsigned int devfn,
				      int where)
{
	struct mtk_pcie_port *port = bus->sysdata;

	return port->base + PCIE_CFG_OFFSET_ADDR + where;
}

static int mtk_pcie_config_read(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 *val)
{
	mtk_pcie_config_tlp_header(bus, devfn, where, size);

	return pci_generic_config_read32(bus, devfn, where, size, val);
}

static int mtk_pcie_config_write(struct pci_bus *bus, unsigned int devfn,
				 int where, int size, u32 val)
{
	mtk_pcie_config_tlp_header(bus, devfn, where, size);

	if (size <= 2)
		val <<= (where & 0x3) * 8;

	return pci_generic_config_write32(bus, devfn, where, 4, val);
}

static struct pci_ops mtk_pcie_ops = {
	.map_bus = mtk_pcie_map_bus,
	.read  = mtk_pcie_config_read,
	.write = mtk_pcie_config_write,
};

static int mtk_pcie_set_trans_table(struct mtk_pcie_port *port,
				    resource_size_t cpu_addr,
				    resource_size_t pci_addr,
				    resource_size_t size,
				    unsigned long type, int num)
{
	void __iomem *table;
	u32 val;

	if (num >= PCIE_MAX_TRANS_TABLES) {
		dev_err(port->dev, "not enough translate table for addr: %#llx, limited to [%d]\n",
			(unsigned long long)cpu_addr, PCIE_MAX_TRANS_TABLES);
		return -ENODEV;
	}

	table = port->base + PCIE_TRANS_TABLE_BASE_REG +
		num * PCIE_ATR_TLB_SET_OFFSET;

	writel_relaxed(lower_32_bits(cpu_addr) | PCIE_ATR_SIZE(fls(size) - 1),
		       table);
	writel_relaxed(upper_32_bits(cpu_addr),
		       table + PCIE_ATR_SRC_ADDR_MSB_OFFSET);
	writel_relaxed(lower_32_bits(pci_addr),
		       table + PCIE_ATR_TRSL_ADDR_LSB_OFFSET);
	writel_relaxed(upper_32_bits(pci_addr),
		       table + PCIE_ATR_TRSL_ADDR_MSB_OFFSET);

	if (type == IORESOURCE_IO)
		val = PCIE_ATR_TYPE_IO | PCIE_ATR_TLP_TYPE_IO;
	else
		val = PCIE_ATR_TYPE_MEM | PCIE_ATR_TLP_TYPE_MEM;

	writel_relaxed(val, table + PCIE_ATR_TRSL_PARAM_OFFSET);

	return 0;
}

static void mtk_pcie_enable_msi(struct mtk_pcie_port *port)
{
	int i;
	u32 val;

	for (i = 0; i < PCIE_MSI_SET_NUM; i++) {
		struct mtk_msi_set *msi_set = &port->msi_sets[i];

		msi_set->base = port->base + PCIE_MSI_SET_BASE_REG +
				i * PCIE_MSI_SET_OFFSET;
		msi_set->msg_addr = port->reg_base + PCIE_MSI_SET_BASE_REG +
				    i * PCIE_MSI_SET_OFFSET;

		/* Configure the MSI capture address */
		writel_relaxed(lower_32_bits(msi_set->msg_addr), msi_set->base);
		writel_relaxed(upper_32_bits(msi_set->msg_addr),
			       port->base + PCIE_MSI_SET_ADDR_HI_BASE +
			       i * PCIE_MSI_SET_ADDR_HI_OFFSET);
	}

	val = readl_relaxed(port->base + PCIE_MSI_SET_ENABLE_REG);
	val |= PCIE_MSI_SET_ENABLE;
	writel_relaxed(val, port->base + PCIE_MSI_SET_ENABLE_REG);

	val = readl_relaxed(port->base + PCIE_INT_ENABLE_REG);
	val |= PCIE_MSI_ENABLE;
	writel_relaxed(val, port->base + PCIE_INT_ENABLE_REG);
}


#define RETRY_LIMIT 3                //for relink

extern void pcie_phy_init(unsigned int port_num);

void mt7512_pcie_reset(void)
{
	unsigned int tmp;
	void *virtAddr1;
    //for relink
	unsigned int tmp1, tmp2;
	void * virtAddr;
	u32 retry_cnt = 0;
	uint8_t serdes_ret=0;	

    tmp = regRead_PCIe(0x1fb00834);
    regWrite_PCIe(0x1fb00834, (tmp | ( (1<<26) | (1<<27) | (1<<29))));
    regWrite_PCIe(0x1fb00834, (tmp | ( (1<<26) | (1<<27)  )));//0c00_0000///////098add
    tmp = regRead_PCIe(0x1fb00830);
    regWrite_PCIe(0x1fb00830, (tmp | (1<<27)));
    mdelay(100);



	/*before reset host,need to pull device low*/
	//port0 port1 0x1fb00088 bit29 bit 26
	tmp = regRead_PCIe(0x1fb00088);
	regWrite_PCIe(0x1fb00088, (tmp & (~((1<<29) | (1<<26)))));
	//port2 0x1fb00088 bit16
	tmp = regRead_PCIe(0x1fb00088);
	regWrite_PCIe(0x1fb00088, (tmp & (~(1<<16))));
	mdelay(1);
	
	regWrite_PCIe(0x1fc10044, 0x23020133);
	virtAddr = ioremap ((phys_addr_t)0x1fc30044,4);
	writel(0x23020133 ,virtAddr);
	regWrite_PCIe(0x1fc15030, 0x50500032);
	regWrite_PCIe(0x1fc15130, 0x50500032);


	pcie_phy_init(3);
	
	mdelay(30);//fix 7916 pbus timeout
	/*first reset to default*///0xbfb00834-pulse-0000_11111_0000
    
    //hostcontroller port0 port1 0x1fb00834 bit29 bit26 bit27
    tmp = regRead_PCIe(0x1fb00834);
    regWrite_PCIe(0x1fb00834, (tmp & (~( (1<<26) | (1<<27) | (1<<29)))));
	//port2 0x1fb00830 bit27
    tmp = regRead_PCIe(0x1fb00830);
    regWrite_PCIe(0x1fb00830, (tmp & (~(1<<27))));



    /*==========some phy setting after mac reset begin======================*/
	/* mac setting, after mac reset, before release device reset */
	regWrite_PCIe(0x1fc00100, 0x41474147);//Preset 1 (initial), add by Carl 10/11
	virtAddr = ioremap ((phys_addr_t)0x1fc20100,4);
	writel(0x41474147 ,virtAddr);
	
	regWrite_PCIe(0x1fc00338, 0x1018020F);//preset to use (final)
	virtAddr = ioremap ((phys_addr_t)0x1fc20338,4);
	writel(0x1018020F ,virtAddr);
	/*==========some phy setting after mac reset end=========================*/
	
	mdelay(10);

    /*release device*///0xbfb00088-0000_11111
    tmp = regRead_PCIe(0x1fb00088);
    regWrite_PCIe(0x1fb00088, (tmp | ((1<<29) | (1<<26))));
	//port2 0x1fb00088 bit16
	tmp = regRead_PCIe(0x1fb00088);
	regWrite_PCIe(0x1fb00088, (tmp | (1<<16)));



	/*wait link up*/
	mdelay(1000); 

	//port0
	printk("debug check 1fc00154 = 0x%x , 1fc00150 = %x, 1fc00018 = %x \n", regRead_PCIe(0x1fc00154), regRead_PCIe(0x1fc00150), regRead_PCIe(0x1fc00018) );
	        
	//port1
	virtAddr = ioremap ((phys_addr_t)0x1fc20154,4);
	tmp = readl(virtAddr);
	virtAddr = ioremap ((phys_addr_t)0x1fc20150,4);
	tmp1 = readl(virtAddr);
	virtAddr = ioremap ((phys_addr_t)0x1fc20018,4);
	tmp2 = readl(virtAddr);	
	printk("debug check 1fc20154 = 0x%x , 1fc20150 = %x, 1fc20018 = %x \n", tmp , tmp1, tmp2 );


	return ;
}


static int mtk_pcie_startup_port(struct mtk_pcie_port *port)
{
	struct resource_entry *entry;
	struct pci_host_bridge *host = pci_host_bridge_from_priv(port);
	unsigned int table_index = 0;
	int err;
	u32 val;

		/* Set as RC mode */
		val = readl_relaxed(port->base + PCIE_SETTING_REG);
		val |= PCIE_RC_MODE;
		writel_relaxed(val, port->base + PCIE_SETTING_REG);
	
		/* Set class code */
		val = readl_relaxed(port->base + PCIE_PCI_IDS_1);
		val &= ~GENMASK(31, 8);
		val |= PCI_CLASS(PCI_CLASS_BRIDGE_PCI << 8);
		writel_relaxed(val, port->base + PCIE_PCI_IDS_1);
	
		/* enable all INTx interrupts */
#ifdef TCSUPPORT_PCIE_MSI
                //MSI
		mtk_pcie_enable_msi(port);

		val = readl_relaxed(port->base + PCIE_MSI_SET_BASE_REG + PCIE_MSI_SET_ENABLE_OFFSET);
		val |= 1;
		writel_relaxed(val, port->base + PCIE_MSI_SET_BASE_REG + PCIE_MSI_SET_ENABLE_OFFSET);
#else        
                //INTX
		val = readl_relaxed(port->base + PCIE_INT_ENABLE_REG);
		val |= PCIE_INTX_ENABLE;
		writel_relaxed(val, port->base + PCIE_INT_ENABLE_REG);
		//0x1ac bit3=1 INTx deassert TLP enable
		val = readl_relaxed(port->base + 0x1ac);
		val |= (1<<3);
		writel_relaxed(val, port->base + 0x1ac);
#endif	


	/* Check if the link is up or not */
	err = readl_poll_timeout(port->base + PCIE_LINK_STATUS_REG, val,
				 !!(val & PCIE_PORT_LINKUP), 20,
				 PCI_PM_D3COLD_WAIT * USEC_PER_MSEC);
	if (err) {
		val = readl_relaxed(port->base + PCIE_LTSSM_STATUS_REG);
		dev_err(port->dev, "PCIe link down, ltssm reg val: %#x\n", val);
		return err;
	}
	dev_info(port->dev, "pcie rc %d linkup success\n", base_num);

	//mtk_pcie_enable_msi(port);

	/* Set PCIe translation windows */
	resource_list_for_each_entry(entry, &host->windows) {
		struct resource *res = entry->res;
		unsigned long type = resource_type(res);
		resource_size_t cpu_addr;
		resource_size_t pci_addr;
		resource_size_t size;
		const char *range_type;

		if (type == IORESOURCE_IO) {
			cpu_addr = pci_pio_to_address(res->start);
			range_type = "IO";
		} else if (type == IORESOURCE_MEM) {
			cpu_addr = res->start;
			range_type = "MEM";
		} else {
			continue;
		}

		pci_addr = res->start - entry->offset;
		size = resource_size(res);
		err = mtk_pcie_set_trans_table(port, cpu_addr, pci_addr, size,
					       type, table_index);
		if (err)
			return err;

		dev_info(port->dev, "set %s trans window[%d]: cpu_addr = %#llx, pci_addr = %#llx, size = %#llx\n",
			range_type, table_index, (unsigned long long)cpu_addr,
			(unsigned long long)pci_addr, (unsigned long long)size);

		table_index++;
	}

	return 0;
}

static int mtk_pcie_set_affinity(struct irq_data *data,
				 const struct cpumask *mask, bool force)
{
	return -EINVAL;
}

static void mtk_pcie_msi_irq_mask(struct irq_data *data)
{
	pci_msi_mask_irq(data);
	irq_chip_mask_parent(data);
}

static void mtk_pcie_msi_irq_unmask(struct irq_data *data)
{
	pci_msi_unmask_irq(data);
	irq_chip_unmask_parent(data);
}

static struct irq_chip mtk_msi_irq_chip = {
	.irq_ack = irq_chip_ack_parent,
	.irq_mask = mtk_pcie_msi_irq_mask,
	.irq_unmask = mtk_pcie_msi_irq_unmask,
	.name = "MSI",
};

static struct msi_domain_info mtk_msi_domain_info = {
	.flags	= (MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS |
		   MSI_FLAG_PCI_MSIX | MSI_FLAG_MULTI_PCI_MSI),
	.chip	= &mtk_msi_irq_chip,
};

static void mtk_compose_msi_msg(struct irq_data *data, struct msi_msg *msg)
{
	struct mtk_msi_set *msi_set = irq_data_get_irq_chip_data(data);
	struct mtk_pcie_port *port = data->domain->host_data;
	unsigned long hwirq;

	hwirq =	data->hwirq % PCIE_MSI_IRQS_PER_SET;

	msg->address_hi = upper_32_bits(msi_set->msg_addr);
	msg->address_lo = lower_32_bits(msi_set->msg_addr);
	msg->data = hwirq;
	dev_dbg(port->dev, "msi#%#lx address_hi %#x address_lo %#x data %d\n",
		hwirq, msg->address_hi, msg->address_lo, msg->data);
}

static void mtk_msi_bottom_irq_ack(struct irq_data *data)
{
	struct mtk_msi_set *msi_set = irq_data_get_irq_chip_data(data);
	unsigned long hwirq;

	hwirq =	data->hwirq % PCIE_MSI_IRQS_PER_SET;

	writel_relaxed(BIT(hwirq), msi_set->base + PCIE_MSI_SET_STATUS_OFFSET);
}

static void mtk_msi_bottom_irq_mask(struct irq_data *data)
{
	struct mtk_msi_set *msi_set = irq_data_get_irq_chip_data(data);
	struct mtk_pcie_port *port = data->domain->host_data;
	unsigned long hwirq, flags;
	u32 val;

	hwirq =	data->hwirq % PCIE_MSI_IRQS_PER_SET;

	raw_spin_lock_irqsave(&port->irq_lock, flags);
	val = readl_relaxed(msi_set->base + PCIE_MSI_SET_ENABLE_OFFSET);
	val &= ~BIT(hwirq);
	writel_relaxed(val, msi_set->base + PCIE_MSI_SET_ENABLE_OFFSET);
	raw_spin_unlock_irqrestore(&port->irq_lock, flags);
}

static void mtk_msi_bottom_irq_unmask(struct irq_data *data)
{
	struct mtk_msi_set *msi_set = irq_data_get_irq_chip_data(data);
	struct mtk_pcie_port *port = data->domain->host_data;
	unsigned long hwirq, flags;
	u32 val;

	hwirq =	data->hwirq % PCIE_MSI_IRQS_PER_SET;

	raw_spin_lock_irqsave(&port->irq_lock, flags);
	val = readl_relaxed(msi_set->base + PCIE_MSI_SET_ENABLE_OFFSET);
	val |= BIT(hwirq);
	writel_relaxed(val, msi_set->base + PCIE_MSI_SET_ENABLE_OFFSET);
	raw_spin_unlock_irqrestore(&port->irq_lock, flags);
}

static struct irq_chip mtk_msi_bottom_irq_chip = {
	.irq_ack		= mtk_msi_bottom_irq_ack,
	.irq_mask		= mtk_msi_bottom_irq_mask,
	.irq_unmask		= mtk_msi_bottom_irq_unmask,
	.irq_compose_msi_msg	= mtk_compose_msi_msg,
	.irq_set_affinity	= mtk_pcie_set_affinity,
	.name			= "MSI",
};

static int mtk_msi_bottom_domain_alloc(struct irq_domain *domain,
				       unsigned int virq, unsigned int nr_irqs,
				       void *arg)
{
	struct mtk_pcie_port *port = domain->host_data;
	struct mtk_msi_set *msi_set;
	int i, hwirq, set_idx;

	mutex_lock(&port->lock);

	hwirq = bitmap_find_free_region(port->msi_irq_in_use, PCIE_MSI_IRQS_NUM,
					order_base_2(nr_irqs));

	mutex_unlock(&port->lock);

	if (hwirq < 0)
		return -ENOSPC;

	set_idx = hwirq / PCIE_MSI_IRQS_PER_SET;
	msi_set = &port->msi_sets[set_idx];

	for (i = 0; i < nr_irqs; i++)
		irq_domain_set_info(domain, virq + i, hwirq + i,
				    &mtk_msi_bottom_irq_chip, msi_set,
				    handle_edge_irq, NULL, NULL);

	return 0;
}

static void mtk_msi_bottom_domain_free(struct irq_domain *domain,
				       unsigned int virq, unsigned int nr_irqs)
{
	struct mtk_pcie_port *port = domain->host_data;
	struct irq_data *data = irq_domain_get_irq_data(domain, virq);

	mutex_lock(&port->lock);

	bitmap_release_region(port->msi_irq_in_use, data->hwirq,
			      order_base_2(nr_irqs));

	mutex_unlock(&port->lock);

	irq_domain_free_irqs_common(domain, virq, nr_irqs);
}

static const struct irq_domain_ops mtk_msi_bottom_domain_ops = {
	.alloc = mtk_msi_bottom_domain_alloc,
	.free = mtk_msi_bottom_domain_free,
};

static void mtk_intx_mask(struct irq_data *data)
{
	struct mtk_pcie_port *port = irq_data_get_irq_chip_data(data);
	unsigned long flags;
	u32 val;

	raw_spin_lock_irqsave(&port->irq_lock, flags);
	val = readl_relaxed(port->base + PCIE_INT_ENABLE_REG);
	val &= ~BIT(data->hwirq + PCIE_INTX_SHIFT);
	writel_relaxed(val, port->base + PCIE_INT_ENABLE_REG);
	raw_spin_unlock_irqrestore(&port->irq_lock, flags);
}

static void mtk_intx_unmask(struct irq_data *data)
{
	struct mtk_pcie_port *port = irq_data_get_irq_chip_data(data);
	unsigned long flags;
	u32 val;

	raw_spin_lock_irqsave(&port->irq_lock, flags);
	val = readl_relaxed(port->base + PCIE_INT_ENABLE_REG);
	val |= BIT(data->hwirq + PCIE_INTX_SHIFT);
	writel_relaxed(val, port->base + PCIE_INT_ENABLE_REG);
	raw_spin_unlock_irqrestore(&port->irq_lock, flags);
}

/**
 * mtk_intx_eoi() - Clear INTx IRQ status at the end of interrupt
 * @data: pointer to chip specific data
 *
 * As an emulated level IRQ, its interrupt status will remain
 * until the corresponding de-assert message is received; hence that
 * the status can only be cleared when the interrupt has been serviced.
 */
static void mtk_intx_eoi(struct irq_data *data)
{
	struct mtk_pcie_port *port = irq_data_get_irq_chip_data(data);
	unsigned long hwirq;

	hwirq = data->hwirq + PCIE_INTX_SHIFT;
	writel_relaxed(BIT(hwirq), port->base + PCIE_INT_STATUS_REG);
}

static struct irq_chip mtk_intx_irq_chip = {
	.irq_mask		= mtk_intx_mask,
	.irq_unmask		= mtk_intx_unmask,
	.irq_eoi		= mtk_intx_eoi,
	.irq_set_affinity	= mtk_pcie_set_affinity,
	.name			= "INTx",
};

static int mtk_pcie_intx_map(struct irq_domain *domain, unsigned int irq,
			     irq_hw_number_t hwirq)
{
	irq_set_chip_data(irq, domain->host_data);
	irq_set_chip_and_handler_name(irq, &mtk_intx_irq_chip,
				      handle_fasteoi_irq, "INTx");
	return 0;
}

static const struct irq_domain_ops intx_domain_ops = {
	.map = mtk_pcie_intx_map,
};

static int mtk_pcie_init_irq_domains(struct mtk_pcie_port *port)
{
	struct device *dev = port->dev;
	struct device_node *intc_node, *node = dev->of_node;
	int ret;

	raw_spin_lock_init(&port->irq_lock);

	/* Setup INTx */
	intc_node = of_get_child_by_name(node, "interrupt-controller");
	if (!intc_node) {
		dev_err(dev, "missing interrupt-controller node\n");
		return -ENODEV;
	}

	port->intx_domain = irq_domain_add_linear(intc_node, PCI_NUM_INTX,
						  &intx_domain_ops, port);
	if (!port->intx_domain) {
		dev_err(dev, "failed to create INTx IRQ domain\n");
		return -ENODEV;
	}

	/* Setup MSI */
	mutex_init(&port->lock);

	port->msi_bottom_domain = irq_domain_add_linear(node, PCIE_MSI_IRQS_NUM,
				  &mtk_msi_bottom_domain_ops, port);
	if (!port->msi_bottom_domain) {
		dev_err(dev, "failed to create MSI bottom domain\n");
		ret = -ENODEV;
		goto err_msi_bottom_domain;
	}

	port->msi_domain = pci_msi_create_irq_domain(dev->fwnode,
						     &mtk_msi_domain_info,
						     port->msi_bottom_domain);
	if (!port->msi_domain) {
		dev_err(dev, "failed to create MSI domain\n");
		ret = -ENODEV;
		goto err_msi_domain;
	}

	return 0;

err_msi_domain:
	irq_domain_remove(port->msi_bottom_domain);
err_msi_bottom_domain:
	irq_domain_remove(port->intx_domain);

	return ret;
}

static void mtk_pcie_irq_teardown(struct mtk_pcie_port *port)
{
	irq_set_chained_handler_and_data(port->irq, NULL, NULL);

	if (port->intx_domain)
		irq_domain_remove(port->intx_domain);

	if (port->msi_domain)
		irq_domain_remove(port->msi_domain);

	if (port->msi_bottom_domain)
		irq_domain_remove(port->msi_bottom_domain);

	irq_dispose_mapping(port->irq);
}

static void mtk_pcie_msi_handler(struct mtk_pcie_port *port, int set_idx)
{
	struct mtk_msi_set *msi_set = &port->msi_sets[set_idx];
	unsigned long msi_enable, msi_status;
	unsigned int virq;
	irq_hw_number_t bit, hwirq;

	msi_enable = readl_relaxed(msi_set->base + PCIE_MSI_SET_ENABLE_OFFSET);

	do {
		msi_status = readl_relaxed(msi_set->base +
					   PCIE_MSI_SET_STATUS_OFFSET);
		msi_status &= msi_enable;
		if (!msi_status)
			break;

		for_each_set_bit(bit, &msi_status, PCIE_MSI_IRQS_PER_SET) {
			hwirq = bit + set_idx * PCIE_MSI_IRQS_PER_SET;
			virq = irq_find_mapping(port->msi_bottom_domain, hwirq);
			generic_handle_irq(virq);
		}
	} while (true);
}

static void mtk_pcie_irq_handler(struct irq_desc *desc)
{
	struct mtk_pcie_port *port = irq_desc_get_handler_data(desc);
	struct irq_chip *irqchip = irq_desc_get_chip(desc);
	unsigned long status;
	unsigned int virq;
	irq_hw_number_t irq_bit = PCIE_INTX_SHIFT;

	chained_irq_enter(irqchip, desc);

	status = readl_relaxed(port->base + PCIE_INT_STATUS_REG);
	for_each_set_bit_from(irq_bit, &status, PCI_NUM_INTX +
			      PCIE_INTX_SHIFT) {
		virq = irq_find_mapping(port->intx_domain,
					irq_bit - PCIE_INTX_SHIFT);
		generic_handle_irq(virq);
	}

	irq_bit = PCIE_MSI_SHIFT;
	for_each_set_bit_from(irq_bit, &status, PCIE_MSI_SET_NUM +
			      PCIE_MSI_SHIFT) {
		mtk_pcie_msi_handler(port, irq_bit - PCIE_MSI_SHIFT);

		writel_relaxed(BIT(irq_bit), port->base + PCIE_INT_STATUS_REG);
	}

	chained_irq_exit(irqchip, desc);
}



static int mtk_pcie_setup_irq(struct mtk_pcie_port *port)
{
	struct device *dev = port->dev;
	struct platform_device *pdev = to_platform_device(dev);
	int err;
#ifdef TCSUPPORT_PCIE_MSI	
	int i =0, irq_msi;
#endif

	//err = mtk_pcie_init_irq_domains(port);
	//if (err)
	//	return err;

	//port->irq = platform_get_irq(pdev, 0);
	//if (port->irq < 0)
	//	return port->irq;

	//irq_set_chained_handler_and_data(port->irq, mtk_pcie_irq_handler, port);

	struct ecnt_pcie *ecnt_pcie=NULL;
	ecnt_pcie=&ECNT_pcie;

#ifndef TCSUPPORT_PCIE_MSI
	port->irq = platform_get_irq(pdev, 0);
#endif
	printk("=======pcie_setup_irq11:base_num=%d=== port->irq = %d ", base_num,port->irq);				

	switch(base_num)
	{
		case 0:
#ifdef TCSUPPORT_PCIE_MSI	
			port->irq = platform_get_irq(pdev, 0);
#endif
			ecnt_pcie->irq_pcie0 = port->irq;
#ifdef TCSUPPORT_PCIE_MSI	
			for(i = 1; i < 8; i++)				
			{						
				irq_msi = platform_get_irq(pdev, i);		
				printk("=======pcie_setup_irq0: i = %d, irq_msi = %d ", i, irq_msi);				
			}
#endif
			break;
		case 1:
#ifdef TCSUPPORT_PCIE_MSI
			port->irq = platform_get_irq(pdev, 0);
#endif
			ecnt_pcie->irq_pcie1 = port->irq;
#ifdef TCSUPPORT_PCIE_MSI	
			for(i = 1; i < 8; i++)				
			{					
				irq_msi = platform_get_irq(pdev, i);		
				printk("=======pcie_setup_irq1: i = %d, irq_msi = %d ", i, irq_msi);				
			}
#endif
			break;
		case 2:
#ifdef TCSUPPORT_PCIE_MSI
			port->irq = platform_get_irq(pdev, 0);
#endif
			ecnt_pcie->irq_pcie2 = port->irq;
#ifdef TCSUPPORT_PCIE_MSI	
			for(i = 1; i < 8; i++)				
			{					
				irq_msi = platform_get_irq(pdev, i);		
				printk("=======pcie_setup_irq2: i = %d, irq_msi = %d ", i, irq_msi);				
			}
#endif
			break;
	}
	printk("=======pcie_setup_irq22:base_num=%d=== port->irq = %d ", base_num,port->irq);				

	return 0;
}

static int mtk_pcie_parse_port(struct mtk_pcie_port *port)
{
	struct device *dev = port->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct resource *regs;

	struct ecnt_pcie *ecnt_pcie=NULL;
	ecnt_pcie=&ECNT_pcie;

	regs = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pcie-mac");
	port->base = devm_ioremap_resource(dev, regs);
	if (IS_ERR(port->base)) {
		dev_err(dev, "failed to map register base\n");
		return PTR_ERR(port->base);
	}

	port->reg_base = regs->start;
	if((port->reg_base&0xffff0000)==0x1fc00000)
		base_num= 0;
	else if((port->reg_base&0xffff0000)==0x1fc20000)
		base_num= 1;
	else if((port->reg_base&0xffff0000)==0x1fc40000)
		base_num= 2;
	

	switch(base_num)
	{
		case 0:
			ecnt_pcie->mac0_base = port->base;
#ifdef TCSUPPORT_PCIE_MSI
			ecnt_pcie->mac0_addr = port->reg_base;
#endif
			break;
		case 1:
			ecnt_pcie->mac1_base = port->base;
#ifdef TCSUPPORT_PCIE_MSI
			ecnt_pcie->mac1_addr = port->reg_base;
#endif
			break;
		case 2:
			ecnt_pcie->mac2_base = port->base;
#ifdef TCSUPPORT_PCIE_MSI
			ecnt_pcie->mac2_addr = port->reg_base;
#endif
			break;
	}


	return 0;
}

static int mtk_pcie_setup(struct mtk_pcie_port *port)
{
	struct device *dev = port->dev;

	int err;
	u32 tmp_reg;


	err = mtk_pcie_parse_port(port);
	if (err)
		return err;

	/* Try link up */

	if(0==base_num)
	{

		/*===========for PCIe pbus setting begin============================================ */	
		printk("===EN7581 Pbus for PCIe init===");
		tmp_reg = GET_PBUS_PCIE0_BASE();
		printk("\n========pcie_probe====0x1fbe3400=%x===========",tmp_reg);
		SET_PBUS_PCIE0_BASE(0x20000000);
		
		tmp_reg = GET_PBUS_PCIE1_BASE();
		printk("\n========pcie_probe====0x1fbe3408=%x===========",tmp_reg);
		SET_PBUS_PCIE1_BASE(0x24000000);
		
		tmp_reg = GET_PBUS_PCIE2_BASE();
		printk("\n========pcie_probe====0x1fbe3410=%x===========",tmp_reg);
		SET_PBUS_PCIE2_BASE(0x28000000);
		
		tmp_reg = GET_PBUS_PCIE0_MASK();
		printk("\n========pcie_probe====0x1fbe3404=%x===========",tmp_reg);
		SET_PBUS_PCIE0_MASK(0xfc000000);

		tmp_reg = GET_PBUS_PCIE1_MASK();
		printk("\n========pcie_probe====0x1fbe340c=%x===========",tmp_reg);
		SET_PBUS_PCIE1_MASK(0xfc000000);

		tmp_reg = GET_PBUS_PCIE2_MASK();
		printk("\n========pcie_probe====0x1fbe3414=%x===========",tmp_reg);
		SET_PBUS_PCIE2_MASK(0xfc000000);


		printk("\n========pcie_probe====after pbus setting=======");
		tmp_reg = GET_PBUS_PCIE0_BASE();
		printk("\n========pcie_probe====0x1fbe3400=%x===========",tmp_reg);
		tmp_reg = GET_PBUS_PCIE1_BASE();
		printk("\n========pcie_probe====0x1fbe3408=%x===========",tmp_reg);
		tmp_reg = GET_PBUS_PCIE2_BASE();
		printk("\n========pcie_probe====0x1fbe3410=%x===========",tmp_reg);
		tmp_reg = GET_PBUS_PCIE0_MASK();
		printk("\n========pcie_probe====0x1fbe3404=%x===========",tmp_reg);
		tmp_reg = GET_PBUS_PCIE1_MASK();
		printk("\n========pcie_probe====0x1fbe340c=%x===========",tmp_reg);
		tmp_reg = GET_PBUS_PCIE2_MASK();
		printk("\n========pcie_probe====0x1fbe3414=%x===========",tmp_reg);
		/*===========for PCIe pbus setting end============================================ */
			
		/*===========for PCIe pad open drain setting begin============================================ */
		tmp_reg = GET_SCU_RGS_OPEN_DRAIN();
		printk("\n========pcie_probe==before==0x1fa2018c=%x===========",tmp_reg);
		SET_SCU_RGS_OPEN_DRAIN(0x7);//bit[2:0] map port2/1/0,1:open drain
		tmp_reg = GET_SCU_RGS_OPEN_DRAIN();
		printk("\n========pcie_probe==after==0x1fa2018c=%x===========\n",tmp_reg);
		/*===========for PCIe pad open drain setting end============================================ */
			
		mt7512_pcie_reset();
		
	}


	err = mtk_pcie_startup_port(port);
	if (err)
		return err;

	err = mtk_pcie_setup_irq(port);
	if (err)
		return err;

	return 0;
}
/*=================7581 PCIe serdes description==============================*/
/*
	ECNT_EVENT_SERDES_SEL_WIFI1	//7581 serdes IF 2 //PCIe0
	ECNT_EVENT_SERDES_SEL_WIFI2	//7581 serdes IF 3 //PCIe1
	ECNT_EVENT_SERDES_SEL_USB2	//7581 serdes IF 5 //PCIe2
*/
/* SerDes-WiFi1 //ECNT_EVENT_SYSTEM_SERDES_WIFI1_SEL_t
	ECNT_EVENT_SERDES_WIFI1_PCIE0_2LANE = 0,	
	ECNT_EVENT_SERDES_WIFI1_PCIE0_1LANE,
	ECNT_EVENT_SERDES_WIFI1_HSGMII,
	ECNT_EVENT_SERDES_WIFI1_USXGMII,
	ECNT_EVENT_SERDES_WIFI1_XFI,
	ECNT_EVENT_SERDES_WIFI1_NONE,
	ECNT_EVENT_SERDES_WIFI1_MAX,
*/	
/* SerDes-WiFi2 //ECNT_EVENT_SYSTEM_SERDES_WIFI2_SEL_t
	ECNT_EVENT_SERDES_WIFI2_PCIE0_2LANE = 0,	
	ECNT_EVENT_SERDES_WIFI2_PCIE1_1LANE,
	ECNT_EVENT_SERDES_WIFI2_HSGMII,
	ECNT_EVENT_SERDES_WIFI2_USXGMII,
	ECNT_EVENT_SERDES_WIFI2_XFI,
	ECNT_EVENT_SERDES_WIFI2_NONE,
	ECNT_EVENT_SERDES_WIFI2_MAX,;
*/
/* SerDes-USB2 //ECNT_EVENT_SYSTEM_SERDES_USB2_SEL_t
	ECNT_EVENT_SERDES_USB2_USB30 = 0,
	ECNT_EVENT_SERDES_USB2_PCIE2_1LANE,
	ECNT_EVENT_SERDES_USB2_NONE,
	ECNT_EVENT_SERDES_USB2_MAX,
*/
/*=================7581 PCIe serdes description==============================*/
static int PCIe_port_num=0;
//ret==1,not PCIe,don't continue init. 

extern u32 GET_PCIC(void);
extern void SET_PCIC(u32 val);
extern u32 GET_NP_SCU_SSTR(void);
extern void SET_NP_SCU_SSTR(u32 val);

#if 1
static int wifi_serdes_select(void)
{	
	int ret=0;
	uint8_t serdes_ret=0;	
	unsigned int tmp;

	printk("7581 wifi_serdes_select: enter PCIe_port_num=%d !\n",PCIe_port_num);


	switch(PCIe_port_num)
	{
		case 0:
			//serdes_ret = get_serdes_interface_sel(ECNT_EVENT_SERDES_SEL_WIFI1);
			serdes_ret = 1; //port0:1lane
			if(0==serdes_ret)
				{
					tmp = GET_NP_SCU_SSTR();
					SET_NP_SCU_SSTR(tmp & (~(3<<13)));			
					tmp = GET_PCIC();
					SET_PCIC((tmp & 0xfffffffc)| (2));
				}
			else if(1==serdes_ret)
				{
					tmp = GET_NP_SCU_SSTR();
					SET_NP_SCU_SSTR(tmp & (~(3<<13)));
					tmp = GET_PCIC();
					SET_PCIC ((tmp | (3)));
				}
			else
				ret = 1;
			break;
		case 1:
			//serdes_ret = get_serdes_interface_sel(ECNT_EVENT_SERDES_SEL_WIFI2);
			serdes_ret = 1; //port1:1lane
			if(0==serdes_ret)
				{
					tmp = GET_NP_SCU_SSTR();
					SET_NP_SCU_SSTR(tmp & (~(3<<11)));
					tmp = GET_PCIC();				
					if(2!=(tmp & 0x3))
						{
							ret = 1;
							printk("Error: serdes_wifi2 not match serdes_wifi1 !!\n");
						}
				}
			else if(1==serdes_ret)
				{
					tmp = GET_NP_SCU_SSTR();
					SET_NP_SCU_SSTR(tmp & (~(3<<11)));
					tmp = GET_PCIC();				
					if(3!=(tmp & 0x3))
						{
							ret = 1;
							printk("Error: serdes_wifi2 not match serdes_wifi1 !!\n");
						}

				}
			else
				ret = 1;
			break;
		case 2:
			//serdes_ret = get_serdes_interface_sel(ECNT_EVENT_SERDES_SEL_USB2);
			serdes_ret = 0;
			if(1==serdes_ret)
				{
					tmp = GET_NP_SCU_SSTR();
					SET_NP_SCU_SSTR(tmp &(~(1<<3)));
				}
			else
				ret = 1;
			break;
	}

	printk("7581 wifi_serdes_select: exit serdes_ret=%d ret=%d !\n",serdes_ret,ret);

	PCIe_port_num++;

	return ret;
}

#endif

u8 pci_common_swizzle_airoha(struct pci_dev *dev, u8 *pinp)
{
	u8 pin = *pinp;

	while (!pci_is_root_bus(dev->bus)) {
		pin = pci_swizzle_interrupt_pin(dev, pin);
		dev = dev->bus->self;
	}
	*pinp = pin;
	//return PCI_SLOT(dev->devfn);
	return pci_domain_nr(dev->bus);
}

int pcibios_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{

	int irq0=0,irq1=0;
	struct ecnt_pcie *ecnt_pcie=NULL;
	ecnt_pcie=&ECNT_pcie;
	
	printk("\n====pcibios_map_irq===slot=%d====\n",slot);


	irq0=ecnt_pcie->irq_pcie0;
	printk("\n====pcibios_map_irq===irq0=%d===\n",irq0);

	irq1=ecnt_pcie->irq_pcie1;	
	printk("\n====pcibios_map_irq===irq1=%d===\n",irq1);		

	if (slot == 0)
		return irq0;
	else if(slot == 1)
		return irq1;

}

static int mtk_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_pcie_port *port;
	struct pci_host_bridge *host;
	int err;
	int value=0;
	u32 check_reg;
	uint8_t serdes_ret=0;	
	
    //pcie_api_init();


	//if(!isFPGA)
	//	{
			err = wifi_serdes_select();
			if (err)
				return err;
	//	}


	host = devm_pci_alloc_host_bridge(dev, sizeof(*port));
	if (!host)
		return -ENOMEM;

	port = pci_host_bridge_priv(host);

	port->dev = dev;
	platform_set_drvdata(pdev, port);

	err = mtk_pcie_setup(port);

        //work around for accidental calltrace when NPU access port1 mac register but only port0 linkup
	//if (err)
	//	goto release_resource;

#ifdef TCSUPPORT_PCIE_MSI
	host->msi = &ecnt_pcie_msi_controller;
	printk("\n===============pcie_probe=====for msi kernel API===========");
#endif


	host->ops = &mtk_pcie_ops;
	host->map_irq = pcibios_map_irq;
	host->swizzle_irq = pci_common_swizzle_airoha;

	host->sysdata = port;

	err = pci_host_probe(host);

	if(0==base_num)
		{
			value=pcie_read_config_word_extend(0,0,0,0x3c);
			value|= port->irq;
			pcie_write_config_word_extend(0,0,0, 0x3c,value);
			pcie_write_config_word_extend(1,0,0, 0x3c,value);

		}
	else if(1==base_num)
		{
			value=pcie_read_config_word_extend(0,1,0,0x3c);
			value|= port->irq;
			pcie_write_config_word_extend(0,1,0, 0x3c,value);
			pcie_write_config_word_extend(2,0,0, 0x3c,value);

		}
	else if(2==base_num)
		{
			value=pcie_read_config_word_extend(0,2,0,0x3c);
			value|= port->irq;
			pcie_write_config_word_extend(0,2,0, 0x3c,value);
			pcie_write_config_word_extend(3,0,0, 0x3c,value);

		}

	if (err)
		goto power_down;


	return 0;

power_down:
	mtk_pcie_irq_teardown(port);
release_resource:
	pci_free_resource_list(&host->windows);

	return err;
}

static const struct of_device_id mtk_pcie_of_match[] = {
	{ .compatible = "ecnt,pcie-ecnt" },
	{},
};

static struct platform_driver mtk_pcie_driver = {
	.probe = mtk_pcie_probe,
	.driver = {
		.name = "mtk-pcie",
		.of_match_table = mtk_pcie_of_match,
	},
};

builtin_platform_driver(mtk_pcie_driver);

//module_platform_driver(mtk_pcie_driver);
//MODULE_LICENSE("GPL v2");
