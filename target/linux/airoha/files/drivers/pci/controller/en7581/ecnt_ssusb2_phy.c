/***************************************************************
Copyright Statement:

This software/firmware and related documentation (¡°Airoha Software¡±) 
are protected under relevant copyright laws. The information contained herein 
is confidential and proprietary to Airoha Limited (¡°Airoha¡±) and/or 
its licensors. Without the prior written permission of Airoha and/or its licensors, 
any reproduction, modification, use or disclosure of Airoha Software, and 
information contained herein, in whole or in part, shall be strictly prohibited.

Airoha Limited. ALL RIGHTS RESERVED.

BY OPENING OR USING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY 
ACKNOWLEDGES AND AGREES THAT THE SOFTWARE/FIRMWARE AND ITS 
DOCUMENTATIONS (¡°AIROHA SOFTWARE¡±) RECEIVED FROM AIROHA 
AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON AN ¡°AS IS¡± 
BASIS ONLY. AIROHA EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES, 
WHETHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED 
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, 
OR NON-INFRINGEMENT. NOR DOES AIROHA PROVIDE ANY WARRANTY 
WHATSOEVER WITH RESPECT TO THE SOFTWARE OF ANY THIRD PARTIES WHICH 
MAY BE USED BY, INCORPORATED IN, OR SUPPLIED WITH THE AIROHA SOFTWARE. 
RECEIVER AGREES TO LOOK ONLY TO SUCH THIRD PARTIES FOR ANY AND ALL 
WARRANTY CLAIMS RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES 
THAT IT IS RECEIVER¡¯S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD 
PARTY ALL PROPER LICENSES CONTAINED IN AIROHA SOFTWARE.

AIROHA SHALL NOT BE RESPONSIBLE FOR ANY AIROHA SOFTWARE RELEASES 
MADE TO RECEIVER¡¯S SPECIFICATION OR CONFORMING TO A PARTICULAR 
STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND 
AIROHA'S ENTIRE AND CUMULATIVE LIABILITY WITH RESPECT TO THE AIROHA 
SOFTWARE RELEASED HEREUNDER SHALL BE, AT AIROHA'S SOLE OPTION, TO 
REVISE OR REPLACE THE AIROHA SOFTWARE AT ISSUE OR REFUND ANY SOFTWARE 
LICENSE FEES OR SERVICE CHARGES PAID BY RECEIVER TO AIROHA FOR SUCH 
AIROHA SOFTWARE.
***************************************************************/

/************************************************************************
*                  I N C L U D E S
*************************************************************************
*/
#include <linux/module.h>
#include <linux/types.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include <asm/io.h>

/************************************************************************
*                  D E F I N E S   &   C O N S T A N T S
*************************************************************************
*/

/************************************************************************
*                  M A C R O S
*************************************************************************
*/

/************************************************************************
*                  D A T A   T Y P E S
*************************************************************************
*/
struct ecnt_ssusb2_phy {
	struct device *dev;
	void __iomem *ssusb2_phy_rg_base; /* SSUSB2_PHY_ASIC_RG physical address */
};

/************************************************************************
*                  E X T E R N A L   D A T A   D E C L A R A T I O N S
*************************************************************************
*/

/************************************************************************
*                  F U N C T I O N   D E C L A R A T I O N S
*************************************************************************
*/

/************************************************************************
*                  P U B L I C   D A T A
*************************************************************************
*/
struct ecnt_ssusb2_phy *ecnt_ssusb2_phy = NULL;

/************************************************************************
*                  P R I V A T E   D A T A
*************************************************************************
*/
static const struct of_device_id ecnt_ssusb2_phy_of_ids[] = {
	{ .compatible = "econet,ecnt-ssusb2_phy"},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ecnt_ssusb2_phy_of_ids);

/************************************************************************
*                  F U N C T I O N   D E F I N I T I O N S
*************************************************************************
*/
u32 get_ssusb2_phy_data(u32 reg)
{
	return readl(ecnt_ssusb2_phy->ssusb2_phy_rg_base + reg);
}

void set_ssusb2_phy_data(u32 reg, u32 val)
{
	writel(val, ecnt_ssusb2_phy->ssusb2_phy_rg_base + reg); 
}


/********************************************************************/

/* APIs */
/***********************************************/
u32 regRead_ssusb2_phy(u32 reg)		
{	
	u32 val=0xDEADBEEF;

	if(reg >= 0x1fae0700 && reg <= 0x1fae0c64)
	{
		val=get_ssusb2_phy_data(reg - 0x1fae0700);
	}
	
	return val;		  
}
EXPORT_SYMBOL(regRead_ssusb2_phy);

void regWrite_ssusb2_phy(u32 reg, u32 val)		
{	

	if(reg >= 0x1fae0700 && reg <= 0x1fae0c64)
	{
		set_ssusb2_phy_data(reg - 0x1fae0700, val);
	}
		  
}
EXPORT_SYMBOL(regWrite_ssusb2_phy);


static int ecnt_ssusb2_phy_probe(struct platform_device *pdev)
{
    struct resource *res = NULL;
	//int irq = 0;
	//int irq_idx = 0;

	printk("ssusb2 phy driver version: 7581.0.20220519\n");

    if (!pdev->dev.of_node) {
        dev_err(&pdev->dev, "No ssusb2 phy DT node found");
        return -EINVAL;
    }

    ecnt_ssusb2_phy = devm_kzalloc(&pdev->dev, sizeof(struct ecnt_ssusb2_phy), GFP_KERNEL);
    if (!ecnt_ssusb2_phy)
        return -ENOMEM;

    platform_set_drvdata(pdev, ecnt_ssusb2_phy);

    /* PON_PHY_ASIC_RG physical address */
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    ecnt_ssusb2_phy->ssusb2_phy_rg_base= devm_ioremap_resource(&pdev->dev, res);
    if (IS_ERR(ecnt_ssusb2_phy->ssusb2_phy_rg_base))
    {
    	devm_kfree(&pdev->dev, ecnt_ssusb2_phy);
        return PTR_ERR(ecnt_ssusb2_phy->ssusb2_phy_rg_base);
    }

    ecnt_ssusb2_phy->dev = &pdev->dev;
    

    return 0;
}

static int ecnt_ssusb2_phy_remove(struct platform_device *pdev)
{
    return 0;
}


/************************************************************************
*      P L A T F O R M   D R I V E R S   D E C L A R A T I O N S
*************************************************************************
*/


static struct platform_driver ecnt_ssusb2_phy_driver = {
    .probe = ecnt_ssusb2_phy_probe,
    .remove = ecnt_ssusb2_phy_remove,
    .driver = {
        .name = "ecnt-ssusb2_phy",
        .of_match_table = ecnt_ssusb2_phy_of_ids
    },
};
module_platform_driver(ecnt_ssusb2_phy_driver);


MODULE_DESCRIPTION("EcoNet SSUSB2 Phy Driver");
