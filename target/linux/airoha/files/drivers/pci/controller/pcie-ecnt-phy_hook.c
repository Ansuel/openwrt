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
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <asm/io.h>
//#include <assert.h>
#include "./H/pcie_ecnt_phy.h"


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

/************************************************************************
*                  P R I V A T E   D A T A
*************************************************************************
*/
static const struct pcie_phy_ops *ops;

/************************************************************************
*                  F U N C T I O N   D E F I N I T I O N S
*************************************************************************
*/
void pcie_phy_init(unsigned int port_num)
{
	if(ops->init != NULL)
	{
		ops->init(port_num);
	}
	else
	{
		printk("pcie_phy_init ops is NULL!\n");
	}
}
EXPORT_SYMBOL(pcie_phy_init);

void pcie_PowerDown(unsigned int port_num)
{
	if(ops->PowerDown != NULL)
	{
		ops->PowerDown(port_num);
	}
	else
	{
		printk("pcie_PowerDown ops is NULL!\n");
	}
}
EXPORT_SYMBOL(pcie_PowerDown);

void pcie_PowerUp(unsigned int port_num)
{
	if(ops->PowerUp != NULL)
	{
		ops->PowerUp(port_num);
	}
	else
	{
		printk("pcie_PowerUp ops is NULL!\n");
	}
}
EXPORT_SYMBOL(pcie_PowerUp);

void pcie_PhyDebug(unsigned int sel)
{
	if(ops->PhyDebug != NULL)
	{
		ops->PhyDebug(sel);
	}
	else
	{
		printk("pcie_PhyDebug ops is NULL!\n");
	}
}
EXPORT_SYMBOL(pcie_PhyDebug);


int pcie_phy_ops_init(const struct pcie_phy_ops *ops_ptr)
{

	if((ops_ptr != NULL) && (ops_ptr->init != NULL) && 
		(ops_ptr->PowerDown != NULL) && (ops_ptr->PowerUp != NULL) && (ops_ptr->PhyDebug != NULL))
	{
	
		ops = ops_ptr;
	}
	else
	{
		printk("pcie_phy_ops_init fail!\n");
		return -1;
	}

	return 0;
}

