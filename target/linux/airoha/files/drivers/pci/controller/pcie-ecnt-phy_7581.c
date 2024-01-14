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

#if 1//def TCSUPPORT_CPU_EN7581
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
#include "./H/pcie_ecnt_phy.h"
//#include <ecnt_event_global/ecnt_event_system.h>


#include "./H/en7581/en7581_pcie_ana_pxp_hal_reg.h"
#include "./H/en7581/en7581_pcie_ana_pxp_reg.h"
#include "./H/en7581/en7581_pcie0_pma_hal_reg.h"
#include "./H/en7581/en7581_pcie0_pma_reg.h"

/************************************************************************
*                  D E F I N E S   &   C O N S T A N T S
*************************************************************************
*/


/************************************************************************
*                  M A C R O S
*************************************************************************
*/
#define ANA_OFFSET 0
#define PMA_OFFSET 0xb000
#define PCIE_ANA_2L 0x1fa5a000
#define PCIE_PMA0 0x1fa5b000
#define PCIE_PMA1 0x1fa5c000
#define PCIE_G3_DEBUG 0
#define FLL_DEBUG 0
#define PR_K_DEFAULT_FLOW 1 //1: load-K flow;	0: auto-K flow

#define mask(bits, offset) (~((0xFFFFFFFF>>(32-bits))<<offset))
#define CHECK_LEN 1//500
extern u32 regRead_ssusb2_phy(u32 reg);
extern void regWrite_ssusb2_phy(u32 reg, u32 val);

/************************************************************************
*                  D A T A   T Y P E S
*************************************************************************
*/
struct ecnt_pcie_phy {
	struct device *dev;
	void __iomem *pc_phy_base;
};

void __iomem *G3_ana2L_phy_rg_base; /* PCIEG3_PHY_PMA_PHYA physical address */
void __iomem *G3_pma0_phy_rg_base; /* PCIEG3_PHY_PMA_PHYD_0 physical address */
void __iomem *G3_pma1_phy_rg_base; /* PCIEG3_PHY_PMA_PHYD_1 physical address */

/************************************************************************
*                  E X T E R N A L   D A T A   D E C L A R A T I O N S
*************************************************************************
*/
extern void __iomem* Get_Base(u32 base);
/************************************************************************
*                  F U N C T I O N   D E C L A R A T I O N S
*************************************************************************
*/
static void pcie_phy_init(unsigned int port_num);
static void pcie_PowerDown(unsigned int port_num);
static void pcie_PowerUp(unsigned int port_num);
static void pcie_PhyDebug(unsigned int sel);


void Rx_PR_Fw_Pre_Cal(char speed, char lane);
void PCIe_G3_L0L1_init(char load_k_en);
void PCIe_G3_L0_init(void);
void PCIe_G3_L1_init(void);
void PCIe_G3_Eye_Scan(char lane, char quick_flag);
void PCIe_G3_L0_Enable(void);
void PCIe_G3_L1_Enable(void);
void PCIe_G3_L0_Disable(void);
void PCIe_G3_L1_Disable(void);
void PCIe_G3_PWR_Saving_on(void);
void PCIe_G3_Wait_Debug_Condition(void);
void PCIe_G3_Rx_Analog_Status_Monitor(void);
void PCIe_G3_Rx_Analog_Status_Print(void);
void PCIe_G3_L0_Relink_Set(void);
void PCIe_G3_L1_Relink_Set(void);


/************************************************************************
*                  P U B L I C   D A T A
*************************************************************************
*/
struct ecnt_pcie_phy *pcie_phy = NULL;

u32 probe[CHECK_LEN];
unsigned int AEQ_cnt, OSCAL_COMPOS, FE_VOS, AEQ0_D0_OS, AEQ0_D1_OS, AEQ0_E0_OS, AEQ0_E1_OS, AEQ0_ERR0_OS, AEQ0_CTLE, sigdet_os;
unsigned int fll_idac[CHECK_LEN], ro_idacf[CHECK_LEN], da_idac[CHECK_LEN], Kcode_done[CHECK_LEN], cor_gain[CHECK_LEN];

//unsigned int tmp1, OSCAL_COMPOS_1, FE_VOS_1, AEQ1_D0_OS, AEQ1_D1_OS, AEQ1_E0_OS, AEQ1_E1_OS, AEQ1_ERR0_OS, AEQ1_CTLE, sigdet_os_1;
//unsigned int fll_idac_1[CHECK_LEN], ro_idacf_1[CHECK_LEN], da_idac_1[CHECK_LEN], freq_1[CHECK_LEN], cor_gain_1[CHECK_LEN];


/************************************************************************
*                  P R I V A T E   D A T A
*************************************************************************
*/
static const struct of_device_id ecnt_pcie_phy_of_id[] = {
    { .compatible = "econet,ecnt-pcie_phy"},
    { /* sentinel */}
};
MODULE_DEVICE_TABLE(of, ecnt_pcie_phy_of_id);

static const struct pcie_phy_ops AN7581_ops = {
	.init		= pcie_phy_init,
	.PowerDown	= pcie_PowerDown,
	.PowerUp		= pcie_PowerUp,
	.PhyDebug	= pcie_PhyDebug,
};

/************************************************************************
*                  F U N C T I O N   D E F I N I T I O N S
*************************************************************************
*/
/*****************************************************************
 ****  Reg   a c c e s s ********************************
 ******************************************************************/
#if 0
static u32 get_pc_phy_reg(u32 reg)
{
	return readl(pcie_phy->pc_phy_base + (reg));
}

static void set_pc_phy_reg(u32 reg, u32 val)
{
	writel(val, (pcie_phy->pc_phy_base + (reg))); 
}
#endif

void Reg_W(u32 Base, u32 Offset, u32 Addr, u32 Data)
{		

#if PCIE_G3_DEBUG
	printk("Reg_W(%x, %x, %x, %x)\n", Base, Offset, Addr, Data);
#endif
	switch (Base)
	{
		case PCIE_ANA_2L:			
			writel(Data, (G3_ana2L_phy_rg_base + (Addr - Offset))); //.h coda file 0x---, match api "PCIE_ANA_2L" + addr			
			break;
		case PCIE_PMA0:			
			writel(Data, (G3_pma0_phy_rg_base + (Addr - Offset))); //.h coda file 0xb---, must - 0xb000 to match  "PCIE_PMA0" + addr			
			break;
		case PCIE_PMA1:
			writel(Data, (G3_pma1_phy_rg_base + (Addr - Offset))); //.h coda file 0xb---, must  - 0xb000 to match  "PCIE_PMA1" + addr			
			break;
		default :
			printk("Write PCIe G3 PMA base addr error !\n");
			break;
	}
}

u32 Reg_R(u32 Base, u32 Offset, u32 Addr)
{
	u32 tmp;
	switch (Base)
	{
		case PCIE_ANA_2L:
			tmp = readl(G3_ana2L_phy_rg_base + (Addr - Offset));
			break;
		case PCIE_PMA0:			
			tmp = readl(G3_pma0_phy_rg_base + (Addr - Offset));			
			break;
		case PCIE_PMA1:
			tmp = readl(G3_pma1_phy_rg_base + (Addr - Offset)); 			
			break;
		default :
			printk("Read PCIe G3 PMA base addr error !\n");
			tmp = 0xdeadbeef;
			break;
	}

#if PCIE_G3_DEBUG
	printk("Reg_R(%x, %x, %x) = %x\n", Base, Offset, Addr, tmp);
#endif
	return tmp;	
}

void Reg_R_then_W(u32 Addr, u32 Data, char MSB, char LSB)
{
	u32 rdata, wdata, Base, rg_adr, write_mask;
	
	Base = Addr & 0xfffff000;
	rg_adr = Addr& 0xfff;
	rdata = Reg_R(Base, 0, rg_adr);

	if((MSB == 31) &&( LSB == 0))
	{
		wdata = Data;		
	}else
	{
		write_mask = (1 << (MSB-LSB+1)) -1;
		wdata = (rdata & (~(write_mask << LSB))) | (Data << LSB);	
	}

	Reg_W(Base, 0, rg_adr, wdata);
	#if 0//FLL_DEBUG
	printk("Write addr: %08x value: %08x \n", Addr, wdata);
	#endif

}




void Check_Reg(u32 Base, u32 Offset, u32 Addr, u32 expect, char MSB, char LSB, u32 line)
{
	u32 read_back, bit_mask;

	bit_mask = (1 << (MSB -LSB + 1)) -1;
	
	if ((MSB -LSB + 1) == 32)
		read_back = Reg_R( Base,  Offset,  Addr);
	else
		read_back = (Reg_R( Base,  Offset,  Addr) >> LSB) & bit_mask;
	
	if(read_back != expect)
	{
		printk("excel: %3d, 0x%08x [%02d:%02d] = 0x%08x, expect 0x%08x !\n",line ,Base + Addr -Offset ,MSB ,LSB, read_back ,expect  );
	}else
	{
		printk("excel: %3d, 0x%08x [%02d:%02d] = 0x%08x \n",line, Base + Addr -Offset ,MSB, LSB,read_back );
	}	
}


/* APIs */
static void pcie_p2_phy_init(void)
{
	unsigned int val;
	//1fae0b20[7:6]=2'b00
	val = regRead_ssusb2_phy(0x1fae0b20) & mask(2,6);
	regWrite_ssusb2_phy(0x1fae0b20,val);
	//1fae0b18[31:24]=8'b00001110
	val = (regRead_ssusb2_phy(0x1fae0b18) & mask(8,24)) | (0x0e<<24);
	regWrite_ssusb2_phy(0x1fae0b18,val);
	//1fae0b00[29:28]=2'b01
	val = (regRead_ssusb2_phy(0x1fae0b00) & mask(2,28)) | (0x1<<28);
	regWrite_ssusb2_phy(0x1fae0b00,val);
	//1fae0b04[20:19]=2'b11	
	val = (regRead_ssusb2_phy(0x1fae0b04) & mask(2,19)) | (0x3<<19);	
	regWrite_ssusb2_phy(0x1fae0b04,val);
	
}
static void pcie_phy_init(unsigned int port_num)
{
	printk("pcie %d phy init.\n", port_num);
	
	switch(port_num)
	{
		case 0:
			PCIe_G3_L0_init();
			break;
		case 1:
			PCIe_G3_L1_init();
			break;
		case 2:
			pcie_p2_phy_init();
			break;
		case 3:
			PCIe_G3_L0L1_init(1);  //1: load-K flow;	0: auto-K flow
			break;
		case 4:
			PCIe_G3_L0L1_init(0);  //1: load-K flow;	0: auto-K flow
			break;
		default:
			break;
	}
}
static void pcie_PowerDown(unsigned int port_num)
{
	printk("pcie %d power down.\n", port_num);
	switch(port_num)
	{
		case 0:
			PCIe_G3_L0_Disable();
			break;
		case 1:
			PCIe_G3_L1_Disable();
			break;
		case 2:
			regWrite_ssusb2_phy(0x1fae080c,0xC0000000);
			break;
		default:
			break;
	}
}
static void pcie_PowerUp(unsigned int port_num)
{
	printk("pcie %d power up.\n", port_num);
	switch(port_num)
	{
		case 0:
			PCIe_G3_L0_Enable(); //maybe need phy init again?
			break;
		case 1:
			PCIe_G3_L1_Enable(); //maybe need phy init again?
			break;
		case 2:
			regWrite_ssusb2_phy(0x1fae080c,0x40000000);
			break;
		default:
			break;
	}
}

static void pcie_PhyDebug(unsigned int sel)
{
	printk("pcie phy debug function: %d \n", sel);
	switch(sel)
	{		
		case 0:
			PCIe_G3_L0_Relink_Set();
			break;
		case 1:
			PCIe_G3_L1_Relink_Set();
			break;
		case 2:			
			PCIe_G3_Wait_Debug_Condition();
			break;
		case 3:
			PCIe_G3_Rx_Analog_Status_Monitor();
			break;
		case 4:
			PCIe_G3_Rx_Analog_Status_Print();
			break;
		case 5:
			//PCIe_G3_Init_RW_Check();
			break;
		case 6:
			PCIe_G3_PWR_Saving_on();
			break;
		case 7:
			PCIe_G3_Eye_Scan(0, 0);	//lane0, full plot
			break;
		case 8:
			PCIe_G3_Eye_Scan(1, 0);	//lane1, full plt
			break;
		case 9:
			PCIe_G3_Eye_Scan(0, 1);	//lane0, quick plot
			break;
		case 10:
			PCIe_G3_Eye_Scan(1, 1);	//lane1, quick plot
			break;
		case 11:
			
			break;
		case 12:
			
			break;
		case 13:
			
			break;
		case 14:
			
			break;
		case 15:
			
			break;
		case 16:
			
			break;
		case 17:
			
			break;
		case 18:
			
			break;
			
		case 19:
			
			break;
		default:
			break;
	}
}




/***************************** Gen3 PCIe API *****************************/


//--------------------------------


void Rx_PR_Fw_Pre_Cal(char speed, char lane)
{
	int i;
	int Thermometer_Search[7] = { 1, 2, 3, 4, 5, 6, 7 };

	u32 FL_Out_target, rg_lock_cyclecnt, FL_Out_locking_rang;
	u32 rg_lock_target_beg; 
	u32 rg_lock_target_end;
	u32 rg_lock_lockth;
	u32 rg_unlock_target_beg;
	u32 rg_unlock_target_end;
	u32 rg_unlock_cyclecnt;
	u32 rg_unlockth;
	u32 pr_idac;
	u32 RO_FL_Out_diff = 0; //20230301 for coverity , give init value 0
	u32 RO_FL_Out_diff_tmp;
	u32 tune_pr_idac_bit_position;
	u32 RO_FL_Out;
	u32 RO_state_freqdet;
	u32 cdr_pr_idac_tmp;

	u32 lane_offset_dig;

	if(speed == 3)
	{
		FL_Out_target = 41600; //Gen3
		rg_lock_cyclecnt = 26000;
	}else
	{
		FL_Out_target = 41941; //Gen2, Gen1
		rg_lock_cyclecnt = 32767;
	}
	
	FL_Out_locking_rang = 100;
	rg_lock_lockth = 3;
	RO_FL_Out = 0;
	rg_lock_target_beg = FL_Out_target - FL_Out_locking_rang; //speed dependent
	rg_lock_target_end = FL_Out_target + FL_Out_locking_rang;//GPON
	rg_unlock_target_beg = rg_lock_target_beg;
	rg_unlock_target_end = rg_lock_target_end;
	rg_unlock_cyclecnt = rg_lock_cyclecnt;
	rg_unlockth = rg_lock_lockth;

	if (lane == 1)
		lane_offset_dig = 0x1000;
	else
		lane_offset_dig = 0;

	
#if FLL_DEBUG
	printk("FL_Out_target=%d, rg_lock_target_beg=%d, rg_lock_target_end=%d, rg_lock_cyclecnt=%d\n", FL_Out_target, rg_lock_target_beg, rg_lock_target_end, rg_lock_cyclecnt);
#endif



	Reg_R_then_W(lane_offset_dig + 0x1fa5b004, 1, 0, 0 ); //rg_lcpll_man_pwdb = 1

	Reg_R_then_W(lane_offset_dig + 0x1fa5b150, rg_lock_target_beg, 15, 0 ); //rg_lock_target_beg = rg_lock_target_beg

	Reg_R_then_W(lane_offset_dig + 0x1fa5b150, rg_lock_target_end, 31, 16 ); //rg_lock_target_end = rg_lock_target_end

	Reg_R_then_W(lane_offset_dig + 0x1fa5b14C, rg_lock_cyclecnt, 15, 0 ); //rg_lock_cyclecnt = rg_lock_cyclecnt

	Reg_R_then_W(lane_offset_dig + 0x1fa5b158, rg_lock_lockth, 11, 8 ); //rg_lock_lockth = rg_lock_lockth

	Reg_R_then_W(lane_offset_dig + 0x1fa5b154, rg_unlock_target_beg, 15, 0 ); //rg_unlock_target_beg = rg_unlock_target_beg

	Reg_R_then_W(lane_offset_dig + 0x1fa5b154, rg_unlock_target_end, 31, 16 ); //rg_unlock_target_end = rg_unlock_target_end

	Reg_R_then_W(lane_offset_dig + 0x1fa5b14C, rg_unlock_cyclecnt, 31, 16 ); //rg_unlock_cyclecnt = rg_unlock_cyclecnt

	Reg_R_then_W(lane_offset_dig + 0x1fa5b158, rg_unlockth, 15, 12 ); //rg_unlockth = rg_unlockth

	if (lane == 1)	
		Reg_R_then_W(0x1fa5a1d4, 1, 24, 24 ); //RG_PXP_CDR_PR_INJ_FORCE_OFF = 1
	else
		Reg_R_then_W(0x1fa5a11c, 1, 24, 24 ); //RG_PXP_CDR_PR_INJ_FORCE_OFF = 1


	Reg_R_then_W(lane_offset_dig + 0x1fa5b820, 1, 24, 24 ); //rg_force_sel_da_pxp_cdr_pr_lpf_r_en = 1

	Reg_R_then_W(lane_offset_dig + 0x1fa5b820, 1, 16, 16 ); //rg_force_da_pxp_cdr_pr_lpf_r_en = 1

	Reg_R_then_W(lane_offset_dig + 0x1fa5b820, 1, 8, 8 ); //rg_force_sel_da_pxp_cdr_pr_lpf_c_en = 1

	Reg_R_then_W(lane_offset_dig + 0x1fa5b820, 0, 0, 0 ); //rg_force_da_pxp_cdr_pr_lpf_c_en = 0

	Reg_R_then_W(lane_offset_dig + 0x1fa5b794, 1, 16, 16 ); //rg_force_sel_da_pxp_cdr_pr_idac = 1

	Reg_R_then_W(lane_offset_dig + 0x1fa5b824, 1, 24, 24 ); //rg_force_sel_da_pxp_cdr_pr_pwdb = 1

	Reg_R_then_W(lane_offset_dig + 0x1fa5b824, 0, 16, 16 ); //rg_force_da_pxp_cdr_pr_pwdb=1'b0 --> 1'b1 (PR Reset) = 0

	Reg_R_then_W(lane_offset_dig + 0x1fa5b824, 1, 16, 16 ); //rg_force_da_pxp_cdr_pr_pwdb=1'b0 --> 1'b1 (PR Reset) = 1
	
	RO_FL_Out_diff_tmp = 0xffff;
	cdr_pr_idac_tmp = 0;

#if 1
	for (i = 0; i < 7; i++)
	{

	    pr_idac = Thermometer_Search[i];
	    
	    Reg_R_then_W(lane_offset_dig + 0x1fa5b794, Thermometer_Search[i] << 8, 10, 0 ); //rg_force_da_pxp_cdr_pr_idac[10:0]

	    Reg_R_then_W(lane_offset_dig + 0x1fa5b158, 0, 2, 0 ); //rg_freqlock_det_en=3'h3 = 0

	    Reg_R_then_W(lane_offset_dig + 0x1fa5b158, 3, 2, 0 ); //rg_freqlock_det_en=3'h3 = 3

	    mdelay(10);

	    // Read BG, ro_fl_out
	    //Addr = 0x1fa5b530;
	    //RO_FL_Out = DUT_RG_READ(dev_id, ref Addr, 31, 16);
	    RO_FL_Out = (Reg_R(lane_offset_dig + 0x1fa5b000, 0, 0x530) >> 16) & 0xffff;
#if FLL_DEBUG		
	    printk("pr_idac = %x, pr_idac = %x, RO_FL_Out=%d\n", pr_idac, pr_idac, RO_FL_Out);
#endif
	    //RO_FL_Out_diff = Convert.ToInt32(Math.Abs(Convert.ToInt32(RO_FL_Out) - Convert.ToInt32(FL_Out_target)));

		
	    if ((RO_FL_Out > FL_Out_target) /*&& (RO_FL_Out_diff < RO_FL_Out_diff_tmp)*/)
	    {
	        RO_FL_Out_diff_tmp = RO_FL_Out_diff;
	        cdr_pr_idac_tmp = Thermometer_Search[i] << 8;
#if FLL_DEBUG			
	        printk("cdr_pr_idac_tmp= %x\n" , cdr_pr_idac_tmp);
#endif
	    }
	}
#endif



	for (i = 7; i > -1; i--)
	{
	    tune_pr_idac_bit_position = i;
	    pr_idac = cdr_pr_idac_tmp | (0x1 << tune_pr_idac_bit_position);

	    Reg_R_then_W(lane_offset_dig + 0x1fa5b794, pr_idac, 10, 0 ); //rg_force_da_pxp_cdr_pr_idac[10:0] = pr_idac

	    Reg_R_then_W(lane_offset_dig + 0x1fa5b158, 0, 2, 0 ); //rg_freqlock_det_en=3'h3 = 0

	    Reg_R_then_W(lane_offset_dig + 0x1fa5b158, 3, 2, 0 ); //rg_freqlock_det_en=3'h3 = 3

	    mdelay(10);

	    // Read BG, ro_fl_out
	    RO_FL_Out = (Reg_R(lane_offset_dig + 0x1fa5b000, 0, 0x530) >> 16) & 0xffff;
#if FLL_DEBUG		
	    printk("loop2: pr_idac = %x, pr_idac = %x, RO_FL_Out=%d\n", pr_idac, pr_idac, RO_FL_Out);
#endif

	    if (RO_FL_Out < FL_Out_target)
	    {
	        pr_idac &= ~(0x1 << tune_pr_idac_bit_position);
	        cdr_pr_idac_tmp = pr_idac;
#if FLL_DEBUG			
	        printk("cdr_pr_idac_tmp= %x\n" , cdr_pr_idac_tmp);
#endif
	    }
	    else
	    {
	        cdr_pr_idac_tmp = pr_idac;
#if FLL_DEBUG			
	        printk("cdr_pr_idac_tmp= %x\n", cdr_pr_idac_tmp);
#endif
	    }
	}

	Reg_R_then_W(lane_offset_dig + 0x1fa5b794, cdr_pr_idac_tmp, 10, 0 ); //rg_force_da_pxp_cdr_pr_idac[10:0] = cdr_pr_idac_tmp

	for (i = 0; i < 10; i++)
	{
	    Reg_R_then_W(lane_offset_dig + 0x1fa5b158, 0, 2, 0 ); //rg_freqlock_det_en=3'h3 = 0

	    Reg_R_then_W(lane_offset_dig + 0x1fa5b158, 3, 2, 0 ); //rg_freqlock_det_en=3'h3 = 3

	    mdelay(10);

	    RO_FL_Out = (Reg_R(lane_offset_dig + 0x1fa5b000, 0, 0x530) >> 16) & 0xffff;
#if FLL_DEBUG
	    printk("selected cdr_pr_idac= %x\n", cdr_pr_idac_tmp);
#endif

	    // Read BG, RO_state_freqdet
	    RO_state_freqdet = Reg_R(lane_offset_dig + 0x1fa5b000, 0, 0x530) & 0x1;
#if FLL_DEBUG		
	    printk("pr_idac = %x, pr_idac = %x, RO_FL_Out=%d\n", pr_idac, pr_idac, RO_FL_Out);
#endif
	    if(RO_state_freqdet==0)
	    {
	        //KBand_fail_flag = true;
#if 1	        
	        printk("FLL KBand_fail\n");
#endif
	    }

	}
	
	//turn off force mode, and write load band value

	if (lane == 1)	
		Reg_R_then_W(0x1fa5a1d4, 0, 24, 24 ); //RG_PXP_CDR_PR_INJ_FORCE_OFF = 0
	else
		Reg_R_then_W(0x1fa5a11c, 0, 24, 24 ); //RG_PXP_CDR_PR_INJ_FORCE_OFF = 0


	Reg_R_then_W(lane_offset_dig + 0x1fa5b820, 0, 24, 24 ); //rg_force_sel_da_pxp_cdr_pr_lpf_r_en = 0

	Reg_R_then_W(lane_offset_dig + 0x1fa5b820, 0, 8, 8 ); //rg_force_sel_da_pxp_cdr_pr_lpf_c_en = 0

	
	Reg_R_then_W(lane_offset_dig + 0x1fa5b824, 0, 24, 24 ); //rg_force_sel_da_pxp_cdr_pr_pwdb=1'b0

	Reg_R_then_W(lane_offset_dig + 0x1fa5b794, 0, 16, 16 ); //rg_force_sel_da_pxp_cdr_pr_idac = 0

	if(speed == 3)
	{
		Reg_R_then_W(lane_offset_dig + 0x1fa5b8c0, cdr_pr_idac_tmp, 10, 0 ); //rg_fll_idac_pcieg3 = cdr_pr_idac_tmp
	}else
	{
		Reg_R_then_W(lane_offset_dig + 0x1fa5b8bc, cdr_pr_idac_tmp, 10, 0 ); //rg_fll_idac_pcieg1 = cdr_pr_idac_tmp
		Reg_R_then_W(lane_offset_dig + 0x1fa5b8bc, cdr_pr_idac_tmp, 26, 16 ); //rg_fll_idac_pcieg2 = cdr_pr_idac_tmp
	}
		
	 printk("lane%d gen%d cdr_pr_idac_tmp= 0x%x\n", lane, speed, cdr_pr_idac_tmp);

}

void PCIe_G3_L0L1_init(char load_k_en) //1: load-K; 		0: auto-K
{
	u32 tmp, rx_fe_gain_ctrl, multlane_en;
	
#ifdef TCSUPPORT_WLAN_MT7916_V7623
	rx_fe_gain_ctrl = 0x3; //target gen2		
#else		
	rx_fe_gain_ctrl = 0x3; //target gen3, need try 1 or 3 and test with eye scan for customer DUT.
#endif		


 //if((get_serdes_interface_sel(ECNT_EVENT_SERDES_SEL_WIFI1) == 0) && (get_serdes_interface_sel(ECNT_EVENT_SERDES_SEL_WIFI2) == 0)) //port0, port1 2L
 //	multlane_en = 1;
 //else
 	multlane_en = 0;

 rg_type_t(HAL_ADD_DIG_RESERVE_14) ADD_DIG_RESERVE_14;		
 rg_type_t(HAL_RG_PXP_2L_CMN_EN) RG_PXP_2L_CMN_EN;
 rg_type_t(HAL_ADD_DIG_RESERVE_21) ADD_DIG_RESERVE_21;
 rg_type_t(HAL_ADD_DIG_RESERVE_22) ADD_DIG_RESERVE_22;
 rg_type_t(HAL_RG_PXP_2L_TXPLL_POSTDIV_D256_EN) RG_PXP_2L_TXPLL_POSTDIV_D256_EN;
 rg_type_t(HAL_RG_PCIE_CLKTX0_FORCE_OUT1) RG_PCIE_CLKTX0_FORCE_OUT1;
 rg_type_t(HAL_RG_PXP_2L_PCIE_CLKTX1_IMP_SEL) RG_PXP_2L_PCIE_CLKTX1_IMP_SEL;
 rg_type_t(HAL_RG_PCIE_CLKTX1_OFFSET) RG_PCIE_CLKTX1_OFFSET;
 rg_type_t(HAL_RG_PXP_2L_PLL_CMN_RESERVE0) RG_PXP_2L_PLL_CMN_RESERVE0;
 rg_type_t(HAL_SW_RST_SET) SW_RST_SET;
 rg_type_t(HAL_SS_TX_RST_B) SS_TX_RST_B;
 rg_type_t(HAL_ADD_DIG_RESERVE_17) ADD_DIG_RESERVE_17;
 rg_type_t(HAL_RG_PXP_2L_CDR0_PR_MONPI_EN) RG_PXP_2L_CDR0_PR_MONPI_EN;
 rg_type_t(HAL_RG_PXP_2L_CDR1_PR_MONPI_EN) RG_PXP_2L_CDR1_PR_MONPI_EN;
 rg_type_t(HAL_RG_PXP_2L_CDR0_PD_PICAL_CKD8_INV) RG_PXP_2L_CDR0_PD_PICAL_CKD8_INV;
 rg_type_t(HAL_RG_PXP_2L_CDR1_PD_PICAL_CKD8_INV) RG_PXP_2L_CDR1_PD_PICAL_CKD8_INV;
 rg_type_t(HAL_RG_PXP_2L_RX0_PHYCK_DIV) RG_PXP_2L_RX0_PHYCK_DIV;
 rg_type_t(HAL_RG_PXP_2L_RX1_PHYCK_DIV) RG_PXP_2L_RX1_PHYCK_DIV;
 rg_type_t(HAL_rg_force_da_pxp_jcpll_ckout_en) rg_force_da_pxp_jcpll_ckout_en;
 rg_type_t(HAL_RG_PXP_2L_JCPLL_TCL_VTP_EN) RG_PXP_2L_JCPLL_TCL_VTP_EN;
 rg_type_t(HAL_RG_PXP_2L_JCPLL_RST_DLY) RG_PXP_2L_JCPLL_RST_DLY;
 rg_type_t(HAL_RG_PXP_2L_JCPLL_SSC_DELTA1) RG_PXP_2L_JCPLL_SSC_DELTA1;
 rg_type_t(HAL_RG_PXP_2L_JCPLL_SSC_PERIOD) RG_PXP_2L_JCPLL_SSC_PERIOD;
 rg_type_t(HAL_RG_PXP_2L_JCPLL_SSC_EN) RG_PXP_2L_JCPLL_SSC_EN;
 rg_type_t(HAL_RG_PXP_2L_JCPLL_LPF_BR) RG_PXP_2L_JCPLL_LPF_BR;
 rg_type_t(HAL_RG_PXP_2L_JCPLL_LPF_BWC) RG_PXP_2L_JCPLL_LPF_BWC;
 rg_type_t(HAL_RG_PXP_2L_JCPLL_MMD_PREDIV_MODE) RG_PXP_2L_JCPLL_MMD_PREDIV_MODE;
 rg_type_t(HAL_RG_PXP_2L_JCPLL_MONCK_EN) RG_PXP_2L_JCPLL_MONCK_EN;
 rg_type_t(HAL_rg_force_da_pxp_rx_fe_vos) rg_force_da_pxp_rx_fe_vos;
 rg_type_t(HAL_rg_force_da_pxp_jcpll_sdm_pcw) rg_force_da_pxp_jcpll_sdm_pcw;
 rg_type_t(HAL_RG_PXP_2L_JCPLL_TCL_KBAND_VREF) RG_PXP_2L_JCPLL_TCL_KBAND_VREF;
 rg_type_t(HAL_RG_PXP_2L_JCPLL_IB_EXT_EN) RG_PXP_2L_JCPLL_IB_EXT_EN;
 rg_type_t(HAL_RG_PXP_2L_JCPLL_VCODIV) RG_PXP_2L_JCPLL_VCODIV;
 rg_type_t(HAL_RG_PXP_2L_JCPLL_KBAND_KFC) RG_PXP_2L_JCPLL_KBAND_KFC;
 rg_type_t(HAL_scan_mode) scan_mode;
 rg_type_t(HAL_RG_PXP_2L_JCPLL_SDM_HREN) RG_PXP_2L_JCPLL_SDM_HREN;
 rg_type_t(HAL_RG_PXP_2L_JCPLL_TCL_CMP_EN) RG_PXP_2L_JCPLL_TCL_CMP_EN;
 rg_type_t(HAL_RG_PXP_2L_JCPLL_VCO_TCLVAR) RG_PXP_2L_JCPLL_VCO_TCLVAR;
 rg_type_t(HAL_rg_force_da_pxp_txpll_ckout_en) rg_force_da_pxp_txpll_ckout_en;
 rg_type_t(HAL_RG_PXP_2L_TXPLL_REFIN_DIV) RG_PXP_2L_TXPLL_REFIN_DIV;
 rg_type_t(HAL_RG_PXP_2L_TXPLL_SSC_DELTA1) RG_PXP_2L_TXPLL_SSC_DELTA1;
 rg_type_t(HAL_RG_PXP_2L_TXPLL_SSC_PERIOD) RG_PXP_2L_TXPLL_SSC_PERIOD;
 rg_type_t(HAL_RG_PXP_2L_TXPLL_CHP_IOFST) RG_PXP_2L_TXPLL_CHP_IOFST;
 rg_type_t(HAL_RG_PXP_2L_750M_SYS_CK_EN) RG_PXP_2L_750M_SYS_CK_EN;
 rg_type_t(HAL_RG_PXP_2L_TXPLL_TCL_LPF_BW) RG_PXP_2L_TXPLL_TCL_LPF_BW;
 rg_type_t(HAL_rg_force_da_pxp_cdr_pr_idac) rg_force_da_pxp_cdr_pr_idac;
 rg_type_t(HAL_rg_force_da_pxp_txpll_sdm_pcw) rg_force_da_pxp_txpll_sdm_pcw;
 rg_type_t(HAL_RG_PXP_2L_TXPLL_SDM_DI_LS) RG_PXP_2L_TXPLL_SDM_DI_LS;
 rg_type_t(HAL_RG_PXP_2L_TXPLL_SSC_EN) RG_PXP_2L_TXPLL_SSC_EN;
 rg_type_t(HAL_RG_PXP_2L_TXPLL_TCL_KBAND_VREF) RG_PXP_2L_TXPLL_TCL_KBAND_VREF;
 rg_type_t(HAL_RG_PXP_2L_TXPLL_TCL_VTP_EN) RG_PXP_2L_TXPLL_TCL_VTP_EN;
 rg_type_t(HAL_RG_PXP_2L_TXPLL_LPF_BWR) RG_PXP_2L_TXPLL_LPF_BWR;
 rg_type_t(HAL_RG_PXP_2L_TXPLL_POSTDIV_EN) RG_PXP_2L_TXPLL_POSTDIV_EN;
 rg_type_t(HAL_RG_PXP_2L_TXPLL_VCO_SCAPWR) RG_PXP_2L_TXPLL_VCO_SCAPWR;
 rg_type_t(HAL_RG_PXP_2L_TXPLL_PHY_CK2_EN) RG_PXP_2L_TXPLL_PHY_CK2_EN;
 rg_type_t(HAL_RG_PXP_2L_TXPLL_VTP_EN) RG_PXP_2L_TXPLL_VTP_EN;
 rg_type_t(HAL_RG_PXP_2L_TXPLL_KBAND_DIV) RG_PXP_2L_TXPLL_KBAND_DIV;
 rg_type_t(HAL_RG_PXP_2L_TXPLL_SDM_OUT) RG_PXP_2L_TXPLL_SDM_OUT;
 rg_type_t(HAL_RG_PXP_2L_TXPLL_TCL_AMP_VREF) RG_PXP_2L_TXPLL_TCL_AMP_VREF;
 rg_type_t(HAL_RG_PXP_2L_JCPLL_SDM_IFM) RG_PXP_2L_JCPLL_SDM_IFM;
 rg_type_t(HAL_RG_PXP_2L_CDR0_PR_COR_HBW_EN) RG_PXP_2L_CDR0_PR_COR_HBW_EN;
 rg_type_t(HAL_ADD_DIG_RESERVE_19) ADD_DIG_RESERVE_19;
 rg_type_t(HAL_ADD_DIG_RESERVE_20) ADD_DIG_RESERVE_20;
 rg_type_t(HAL_RG_PXP_2L_RX0_SIGDET_DCTEST_EN) RG_PXP_2L_RX0_SIGDET_DCTEST_EN;
 rg_type_t(HAL_RG_PXP_2L_RX0_SIGDET_VTH_SEL) RG_PXP_2L_RX0_SIGDET_VTH_SEL;
 rg_type_t(HAL_RG_PXP_2L_RX0_REV_0) RG_PXP_2L_RX0_REV_0;
 rg_type_t(HAL_SS_RX_CAL_2) SS_RX_CAL_2;
 rg_type_t(HAL_RG_PXP_2L_RX0_FE_VB_EQ2_EN) RG_PXP_2L_RX0_FE_VB_EQ2_EN;
 rg_type_t(HAL_rg_force_da_pxp_rx_fe_gain_ctrl) rg_force_da_pxp_rx_fe_gain_ctrl;
 rg_type_t(HAL_RX_FORCE_MODE_0) RX_FORCE_MODE_0;
 rg_type_t(HAL_SS_RX_SIGDET_0) SS_RX_SIGDET_0;
 rg_type_t(HAL_RX_CTRL_SEQUENCE_DISB_CTRL_1) RX_CTRL_SEQUENCE_DISB_CTRL_1;
 rg_type_t(HAL_RX_CTRL_SEQUENCE_FORCE_CTRL_1) RX_CTRL_SEQUENCE_FORCE_CTRL_1;
 rg_type_t(HAL_RG_PXP_2L_CDR1_PR_COR_HBW_EN) RG_PXP_2L_CDR1_PR_COR_HBW_EN;
 rg_type_t(HAL_RG_PXP_2L_RX1_SIGDET_NOVTH) RG_PXP_2L_RX1_SIGDET_NOVTH;
 rg_type_t(HAL_RG_PXP_2L_RX1_REV_0) RG_PXP_2L_RX1_REV_0;
 rg_type_t(HAL_RG_PXP_2L_RX1_DAC_RANGE_EYE) RG_PXP_2L_RX1_DAC_RANGE_EYE;
 rg_type_t(HAL_RG_PXP_2L_RX1_FE_VB_EQ1_EN) RG_PXP_2L_RX1_FE_VB_EQ1_EN;
 rg_type_t(HAL_rg_force_da_pxp_rx_scan_rst_b) rg_force_da_pxp_rx_scan_rst_b;
 rg_type_t(HAL_rg_force_da_pxp_cdr_pd_pwdb) rg_force_da_pxp_cdr_pd_pwdb;
 rg_type_t(HAL_rg_force_da_pxp_rx_fe_pwdb) rg_force_da_pxp_rx_fe_pwdb;
 rg_type_t(HAL_RG_PXP_2L_CDR0_PR_VREG_IBAND_VAL) RG_PXP_2L_CDR0_PR_VREG_IBAND_VAL;
 rg_type_t(HAL_RG_PXP_2L_CDR0_PR_CKREF_DIV) RG_PXP_2L_CDR0_PR_CKREF_DIV;
 rg_type_t(HAL_RG_PXP_2L_CDR1_PR_VREG_IBAND_VAL) RG_PXP_2L_CDR1_PR_VREG_IBAND_VAL;
 rg_type_t(HAL_RG_PXP_2L_CDR1_PR_CKREF_DIV) RG_PXP_2L_CDR1_PR_CKREF_DIV;
 rg_type_t(HAL_RG_PXP_2L_CDR0_LPF_RATIO) RG_PXP_2L_CDR0_LPF_RATIO;
 rg_type_t(HAL_RG_PXP_2L_CDR1_LPF_RATIO) RG_PXP_2L_CDR1_LPF_RATIO;
 rg_type_t(HAL_RG_PXP_2L_CDR0_PR_BETA_DAC) RG_PXP_2L_CDR0_PR_BETA_DAC;
 rg_type_t(HAL_RG_PXP_2L_CDR1_PR_BETA_DAC) RG_PXP_2L_CDR1_PR_BETA_DAC;
 rg_type_t(HAL_RG_PXP_2L_TX0_CKLDO_EN) RG_PXP_2L_TX0_CKLDO_EN;
 rg_type_t(HAL_RG_PXP_2L_TX1_CKLDO_EN) RG_PXP_2L_TX1_CKLDO_EN;
 rg_type_t(HAL_RG_PXP_2L_TX1_MULTLANE_EN) RG_PXP_2L_TX1_MULTLANE_EN;
 rg_type_t(HAL_ADD_DIG_RESERVE_27) ADD_DIG_RESERVE_27;
 rg_type_t(HAL_ADD_DIG_RESERVE_18) ADD_DIG_RESERVE_18;
 rg_type_t(HAL_ADD_DIG_RESERVE_30) ADD_DIG_RESERVE_30;
 rg_type_t(HAL_RG_PXP_2L_CDR0_PR_MONCK_EN) RG_PXP_2L_CDR0_PR_MONCK_EN;
 rg_type_t(HAL_RG_PXP_2L_RX0_OSCAL_CTLE1IOS) RG_PXP_2L_RX0_OSCAL_CTLE1IOS;
 rg_type_t(HAL_RG_PXP_2L_RX0_OSCAL_VGA1VOS) RG_PXP_2L_RX0_OSCAL_VGA1VOS;
 rg_type_t(HAL_RG_PXP_2L_CDR1_PR_MONCK_EN) RG_PXP_2L_CDR1_PR_MONCK_EN;
 rg_type_t(HAL_RG_PXP_2L_RX1_OSCAL_VGA1IOS) RG_PXP_2L_RX1_OSCAL_VGA1IOS;
 rg_type_t(HAL_ADD_DIG_RESERVE_12) ADD_DIG_RESERVE_12;
 rg_type_t(HAL_SS_DA_XPON_PWDB_0) SS_DA_XPON_PWDB_0;
 
 if(load_k_en == 0)
 {
	//*******  disable Load FLL-K flow
	 ADD_DIG_RESERVE_14.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _ADD_DIG_RESERVE_14);
	 tmp = ADD_DIG_RESERVE_14.hal.rg_dig_reserve_14;
	 tmp = (0x0<<16) | (tmp & (~(0x1<<16)));
	 ADD_DIG_RESERVE_14.hal.rg_dig_reserve_14 = tmp;
	 Reg_W(PCIE_PMA0, PMA_OFFSET, _ADD_DIG_RESERVE_14, ADD_DIG_RESERVE_14.dat.value);

	 ADD_DIG_RESERVE_14.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _ADD_DIG_RESERVE_14);
	 tmp = ADD_DIG_RESERVE_14.hal.rg_dig_reserve_14;
	 tmp = (0x0<<16) | (tmp & (~(0x1<<16)));
	 ADD_DIG_RESERVE_14.hal.rg_dig_reserve_14 = tmp;
	 Reg_W(PCIE_PMA1, PMA_OFFSET, _ADD_DIG_RESERVE_14, ADD_DIG_RESERVE_14.dat.value);
 }else
 {
 	//*******  enable Load FLL-K flow
 	 ADD_DIG_RESERVE_14.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _ADD_DIG_RESERVE_14);
	 tmp = ADD_DIG_RESERVE_14.hal.rg_dig_reserve_14;
	 tmp = (0x1<<16) | (tmp & (~(0x1<<16)));
	 ADD_DIG_RESERVE_14.hal.rg_dig_reserve_14 = tmp;
	 Reg_W(PCIE_PMA0, PMA_OFFSET, _ADD_DIG_RESERVE_14, ADD_DIG_RESERVE_14.dat.value); 

	 ADD_DIG_RESERVE_14.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _ADD_DIG_RESERVE_14);
	 tmp = ADD_DIG_RESERVE_14.hal.rg_dig_reserve_14;
	 tmp = (0x1<<16) | (tmp & (~(0x1<<16)));
	 ADD_DIG_RESERVE_14.hal.rg_dig_reserve_14 = tmp;
	 Reg_W(PCIE_PMA1, PMA_OFFSET, _ADD_DIG_RESERVE_14, ADD_DIG_RESERVE_14.dat.value); 
 }

//*******  modify default value
 RG_PXP_2L_CMN_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CMN_EN);
 RG_PXP_2L_CMN_EN.hal.rg_pxp_2l_cmn_trim = 0x10; //modify default value, Carl 10/21 mail
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CMN_EN, RG_PXP_2L_CMN_EN.dat.value);

 ADD_DIG_RESERVE_21.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _ADD_DIG_RESERVE_21);
 ADD_DIG_RESERVE_21.hal.rg_dig_reserve_21 = 0xCCCBCCCB; //?CODA ??,?? FW ??cccbcccb
 Reg_W(PCIE_PMA0, PMA_OFFSET, _ADD_DIG_RESERVE_21, ADD_DIG_RESERVE_21.dat.value);

 ADD_DIG_RESERVE_22.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _ADD_DIG_RESERVE_22);
 ADD_DIG_RESERVE_22.hal.rg_dig_reserve_22 = 0xCCCB; //?CODA ??,?? FW ?? 0000cccb
 Reg_W(PCIE_PMA0, PMA_OFFSET, _ADD_DIG_RESERVE_22, ADD_DIG_RESERVE_22.dat.value);

 ADD_DIG_RESERVE_21.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _ADD_DIG_RESERVE_21);
 ADD_DIG_RESERVE_21.hal.rg_dig_reserve_21 = 0xCCCBCCCB; //?CODA ??,?? FW ??cccbcccb
 Reg_W(PCIE_PMA1, PMA_OFFSET, _ADD_DIG_RESERVE_21, ADD_DIG_RESERVE_21.dat.value);

 ADD_DIG_RESERVE_22.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _ADD_DIG_RESERVE_22);
 ADD_DIG_RESERVE_22.hal.rg_dig_reserve_22 = 0xCCCB; //?CODA ??,?? FW ?? 0000cccb
 Reg_W(PCIE_PMA1, PMA_OFFSET, _ADD_DIG_RESERVE_22, ADD_DIG_RESERVE_22.dat.value);

 RG_PXP_2L_CMN_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CMN_EN);
 RG_PXP_2L_CMN_EN.hal.rg_pxp_2l_cmn_en = 0x1; //before PLL LDO enable (SPARE_L[5]) first line of all setting 12/25
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CMN_EN, RG_PXP_2L_CMN_EN.dat.value);



//*******  clk out
 RG_PXP_2L_TXPLL_POSTDIV_D256_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_POSTDIV_D256_EN);
 RG_PXP_2L_TXPLL_POSTDIV_D256_EN.hal.rg_pcie_clktx0_amp = 0x5; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_POSTDIV_D256_EN, RG_PXP_2L_TXPLL_POSTDIV_D256_EN.dat.value);

 RG_PCIE_CLKTX0_FORCE_OUT1.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PCIE_CLKTX0_FORCE_OUT1);
 RG_PCIE_CLKTX0_FORCE_OUT1.hal.rg_pcie_clktx1_amp = 0x5; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PCIE_CLKTX0_FORCE_OUT1, RG_PCIE_CLKTX0_FORCE_OUT1.dat.value);

 RG_PXP_2L_TXPLL_POSTDIV_D256_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_POSTDIV_D256_EN);
 RG_PXP_2L_TXPLL_POSTDIV_D256_EN.hal.rg_pcie_clktx0_offset = 0x2; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_POSTDIV_D256_EN, RG_PXP_2L_TXPLL_POSTDIV_D256_EN.dat.value);

 RG_PCIE_CLKTX1_OFFSET.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PCIE_CLKTX1_OFFSET);
 RG_PCIE_CLKTX1_OFFSET.hal.rg_pcie_clktx1_offset = 0x2; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PCIE_CLKTX1_OFFSET, RG_PCIE_CLKTX1_OFFSET.dat.value);

 RG_PCIE_CLKTX0_FORCE_OUT1.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PCIE_CLKTX0_FORCE_OUT1);
 RG_PCIE_CLKTX0_FORCE_OUT1.hal.rg_pxp_2l_pcie_clktx0_hz = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PCIE_CLKTX0_FORCE_OUT1, RG_PCIE_CLKTX0_FORCE_OUT1.dat.value);

 RG_PCIE_CLKTX1_OFFSET.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PCIE_CLKTX1_OFFSET);
 RG_PCIE_CLKTX1_OFFSET.hal.rg_pxp_2l_pcie_clktx1_hz = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PCIE_CLKTX1_OFFSET, RG_PCIE_CLKTX1_OFFSET.dat.value);

 RG_PCIE_CLKTX0_FORCE_OUT1.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PCIE_CLKTX0_FORCE_OUT1);
 RG_PCIE_CLKTX0_FORCE_OUT1.hal.rg_pxp_2l_pcie_clktx0_imp_sel = 0x12; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PCIE_CLKTX0_FORCE_OUT1, RG_PCIE_CLKTX0_FORCE_OUT1.dat.value);

 RG_PXP_2L_PCIE_CLKTX1_IMP_SEL.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_PCIE_CLKTX1_IMP_SEL);
 RG_PXP_2L_PCIE_CLKTX1_IMP_SEL.hal.rg_pxp_2l_pcie_clktx1_imp_sel = 0x12; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_PCIE_CLKTX1_IMP_SEL, RG_PXP_2L_PCIE_CLKTX1_IMP_SEL.dat.value);

 RG_PXP_2L_TXPLL_POSTDIV_D256_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_POSTDIV_D256_EN);
 RG_PXP_2L_TXPLL_POSTDIV_D256_EN.hal.rg_pcie_clktx0_sr = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_POSTDIV_D256_EN, RG_PXP_2L_TXPLL_POSTDIV_D256_EN.dat.value);

 RG_PCIE_CLKTX1_OFFSET.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PCIE_CLKTX1_OFFSET);
 RG_PCIE_CLKTX1_OFFSET.hal.rg_pcie_clktx1_sr = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PCIE_CLKTX1_OFFSET, RG_PCIE_CLKTX1_OFFSET.dat.value);

 RG_PXP_2L_PLL_CMN_RESERVE0.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_PLL_CMN_RESERVE0);
 RG_PXP_2L_PLL_CMN_RESERVE0.hal.rg_pxp_2l_pll_cmn_reserve0 = 0xD; 
 RG_PXP_2L_PLL_CMN_RESERVE0.hal.rg_pxp_2l_pll_cmn_reserve1 = 0xD; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_PLL_CMN_RESERVE0, RG_PXP_2L_PLL_CMN_RESERVE0.dat.value);


//*******  ANA enable
 SW_RST_SET.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _SW_RST_SET);
 SW_RST_SET.hal.rg_sw_xfi_rxpcs_rst_n = 0x1; //Lane0 rst
 SW_RST_SET.hal.rg_sw_ref_rst_n = 0x1; 
 SW_RST_SET.hal.rg_sw_rx_rst_n = 0x1; 
 Reg_W(PCIE_PMA0, PMA_OFFSET, _SW_RST_SET, SW_RST_SET.dat.value);

 SW_RST_SET.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _SW_RST_SET);
 SW_RST_SET.hal.rg_sw_xfi_rxpcs_rst_n = 0x1; //Lane1 rst, ???"T_PCIe_init" sheet???O??0/1????lane 0/1
 SW_RST_SET.hal.rg_sw_ref_rst_n = 0x1; 
 SW_RST_SET.hal.rg_sw_rx_rst_n = 0x1; 
 Reg_W(PCIE_PMA1, PMA_OFFSET, _SW_RST_SET, SW_RST_SET.dat.value);

 SS_TX_RST_B.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _SS_TX_RST_B);
 SS_TX_RST_B.hal.txcalib_rst_b = 0x1; 
 SS_TX_RST_B.hal.tx_top_rst_b = 0x1; 
 Reg_W(PCIE_PMA0, PMA_OFFSET, _SS_TX_RST_B, SS_TX_RST_B.dat.value);

 SS_TX_RST_B.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _SS_TX_RST_B);
 SS_TX_RST_B.hal.txcalib_rst_b = 0x1; 
 SS_TX_RST_B.hal.tx_top_rst_b = 0x1; 
 Reg_W(PCIE_PMA1, PMA_OFFSET, _SS_TX_RST_B, SS_TX_RST_B.dat.value);



//*******  RX?
 udelay(1);
 ADD_DIG_RESERVE_17.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _ADD_DIG_RESERVE_17);
 ADD_DIG_RESERVE_17.hal.rg_dig_reserve_17 = 0x2A00090B; //RG_PXP_2L_RX0_PHY_CK_EN (bit20)
 Reg_W(PCIE_PMA0, PMA_OFFSET, _ADD_DIG_RESERVE_17, ADD_DIG_RESERVE_17.dat.value);

 ADD_DIG_RESERVE_17.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _ADD_DIG_RESERVE_17);
 ADD_DIG_RESERVE_17.hal.rg_dig_reserve_17 = 0x2A00090B; //RG_PXP_2L_RX1_PHY_CK_EN (bit20)
 Reg_W(PCIE_PMA1, PMA_OFFSET, _ADD_DIG_RESERVE_17, ADD_DIG_RESERVE_17.dat.value);

 RG_PXP_2L_CDR0_PR_MONPI_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR0_PR_MONPI_EN);
 RG_PXP_2L_CDR0_PR_MONPI_EN.hal.rg_pxp_2l_cdr0_pr_xfick_en = 0x1; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR0_PR_MONPI_EN, RG_PXP_2L_CDR0_PR_MONPI_EN.dat.value);

 RG_PXP_2L_CDR1_PR_MONPI_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR1_PR_MONPI_EN);
 RG_PXP_2L_CDR1_PR_MONPI_EN.hal.rg_pxp_2l_cdr1_pr_xfick_en = 0x1; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR1_PR_MONPI_EN, RG_PXP_2L_CDR1_PR_MONPI_EN.dat.value);

 RG_PXP_2L_CDR0_PD_PICAL_CKD8_INV.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR0_PD_PICAL_CKD8_INV);
 RG_PXP_2L_CDR0_PD_PICAL_CKD8_INV.hal.rg_pxp_2l_cdr0_pd_edge_dis = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR0_PD_PICAL_CKD8_INV, RG_PXP_2L_CDR0_PD_PICAL_CKD8_INV.dat.value);

 RG_PXP_2L_CDR1_PD_PICAL_CKD8_INV.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR1_PD_PICAL_CKD8_INV);
 RG_PXP_2L_CDR1_PD_PICAL_CKD8_INV.hal.rg_pxp_2l_cdr1_pd_edge_dis = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR1_PD_PICAL_CKD8_INV, RG_PXP_2L_CDR1_PD_PICAL_CKD8_INV.dat.value);

 RG_PXP_2L_RX0_PHYCK_DIV.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX0_PHYCK_DIV);
 RG_PXP_2L_RX0_PHYCK_DIV.hal.rg_pxp_2l_rx0_phyck_sel = 0x1; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX0_PHYCK_DIV, RG_PXP_2L_RX0_PHYCK_DIV.dat.value);

 RG_PXP_2L_RX1_PHYCK_DIV.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX1_PHYCK_DIV);
 RG_PXP_2L_RX1_PHYCK_DIV.hal.rg_pxp_2l_rx1_phyck_sel = 0x1; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX1_PHYCK_DIV, RG_PXP_2L_RX1_PHYCK_DIV.dat.value);



//*******  JCPLL SETTING (phase 1, NO SSC for K TXPLL)
 rg_force_da_pxp_jcpll_ckout_en.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_jcpll_ckout_en);
 rg_force_da_pxp_jcpll_ckout_en.hal.rg_force_sel_da_pxp_jcpll_en = 0x1; 
 rg_force_da_pxp_jcpll_ckout_en.hal.rg_force_da_pxp_jcpll_en = 0x0; 
 Reg_W(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_jcpll_ckout_en, rg_force_da_pxp_jcpll_ckout_en.dat.value);

 rg_force_da_pxp_jcpll_ckout_en.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_jcpll_ckout_en);
 rg_force_da_pxp_jcpll_ckout_en.hal.rg_force_sel_da_pxp_jcpll_en = 0x1; 
 rg_force_da_pxp_jcpll_ckout_en.hal.rg_force_da_pxp_jcpll_en = 0x0; 
 Reg_W(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_jcpll_ckout_en, rg_force_da_pxp_jcpll_ckout_en.dat.value);

 RG_PXP_2L_JCPLL_TCL_VTP_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_TCL_VTP_EN);
 RG_PXP_2L_JCPLL_TCL_VTP_EN.hal.rg_pxp_2l_jcpll_spare_l = 0x20; //JCPLL: Wait BG ready 8�h00=>8�h20 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_TCL_VTP_EN, RG_PXP_2L_JCPLL_TCL_VTP_EN.dat.value);

 RG_PXP_2L_JCPLL_RST_DLY.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_RST_DLY);
 RG_PXP_2L_JCPLL_RST_DLY.hal.rg_pxp_2l_jcpll_pll_rstb = 0x1; //Add for 3/22 mail: PLL FLOW ??
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_RST_DLY, RG_PXP_2L_JCPLL_RST_DLY.dat.value);

 RG_PXP_2L_JCPLL_SSC_DELTA1.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SSC_DELTA1);
 RG_PXP_2L_JCPLL_SSC_DELTA1.hal.rg_pxp_2l_jcpll_ssc_delta = 0x0; 
 RG_PXP_2L_JCPLL_SSC_DELTA1.hal.rg_pxp_2l_jcpll_ssc_delta1 = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SSC_DELTA1, RG_PXP_2L_JCPLL_SSC_DELTA1.dat.value);

 RG_PXP_2L_JCPLL_SSC_PERIOD.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SSC_PERIOD);
 RG_PXP_2L_JCPLL_SSC_PERIOD.hal.rg_pxp_2l_jcpll_ssc_period = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SSC_PERIOD, RG_PXP_2L_JCPLL_SSC_PERIOD.dat.value);

 RG_PXP_2L_JCPLL_SSC_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SSC_EN);
 RG_PXP_2L_JCPLL_SSC_EN.hal.rg_pxp_2l_jcpll_ssc_phase_ini = 0x0; 
 RG_PXP_2L_JCPLL_SSC_EN.hal.rg_pxp_2l_jcpll_ssc_tri_en = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SSC_EN, RG_PXP_2L_JCPLL_SSC_EN.dat.value);

 RG_PXP_2L_JCPLL_LPF_BR.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_LPF_BR);
 RG_PXP_2L_JCPLL_LPF_BR.hal.rg_pxp_2l_jcpll_lpf_br = 0xA; 
 RG_PXP_2L_JCPLL_LPF_BR.hal.rg_pxp_2l_jcpll_lpf_bp = 0xC; 
 RG_PXP_2L_JCPLL_LPF_BR.hal.rg_pxp_2l_jcpll_lpf_bc = 0x1F; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_LPF_BR, RG_PXP_2L_JCPLL_LPF_BR.dat.value);

 RG_PXP_2L_JCPLL_LPF_BWC.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_LPF_BWC);
 RG_PXP_2L_JCPLL_LPF_BWC.hal.rg_pxp_2l_jcpll_lpf_bwc = 0x1E; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_LPF_BWC, RG_PXP_2L_JCPLL_LPF_BWC.dat.value);

 RG_PXP_2L_JCPLL_LPF_BR.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_LPF_BR);
 RG_PXP_2L_JCPLL_LPF_BR.hal.rg_pxp_2l_jcpll_lpf_bwr = 0xA; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_LPF_BR, RG_PXP_2L_JCPLL_LPF_BR.dat.value);

 RG_PXP_2L_JCPLL_MMD_PREDIV_MODE.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_MMD_PREDIV_MODE);
 RG_PXP_2L_JCPLL_MMD_PREDIV_MODE.hal.rg_pxp_2l_jcpll_mmd_prediv_mode = 0x1; //Kaiwen improve jitter 10/9
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_MMD_PREDIV_MODE, RG_PXP_2L_JCPLL_MMD_PREDIV_MODE.dat.value);

 RG_PXP_2L_JCPLL_MONCK_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_MONCK_EN);
 RG_PXP_2L_JCPLL_MONCK_EN.hal.rg_pxp_2l_jcpll_refin_div = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_MONCK_EN, RG_PXP_2L_JCPLL_MONCK_EN.dat.value);

 rg_force_da_pxp_rx_fe_vos.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_rx_fe_vos);
 rg_force_da_pxp_rx_fe_vos.hal.rg_force_sel_da_pxp_jcpll_sdm_pcw = 0x1; 
 Reg_W(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_rx_fe_vos, rg_force_da_pxp_rx_fe_vos.dat.value);

 rg_force_da_pxp_rx_fe_vos.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_rx_fe_vos);
 rg_force_da_pxp_rx_fe_vos.hal.rg_force_sel_da_pxp_jcpll_sdm_pcw = 0x1; 
 Reg_W(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_rx_fe_vos, rg_force_da_pxp_rx_fe_vos.dat.value);

 rg_force_da_pxp_jcpll_sdm_pcw.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_jcpll_sdm_pcw);
 rg_force_da_pxp_jcpll_sdm_pcw.hal.rg_force_da_pxp_jcpll_sdm_pcw = 0x50000000; //Kaiwen improve jitter 10/9
 Reg_W(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_jcpll_sdm_pcw, rg_force_da_pxp_jcpll_sdm_pcw.dat.value);

 rg_force_da_pxp_jcpll_sdm_pcw.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_jcpll_sdm_pcw);
 rg_force_da_pxp_jcpll_sdm_pcw.hal.rg_force_da_pxp_jcpll_sdm_pcw = 0x50000000; //Kaiwen improve jitter 10/9
 Reg_W(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_jcpll_sdm_pcw, rg_force_da_pxp_jcpll_sdm_pcw.dat.value);

 RG_PXP_2L_JCPLL_MMD_PREDIV_MODE.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_MMD_PREDIV_MODE);
 RG_PXP_2L_JCPLL_MMD_PREDIV_MODE.hal.rg_pxp_2l_jcpll_postdiv_d5 = 0x1; 
 RG_PXP_2L_JCPLL_MMD_PREDIV_MODE.hal.rg_pxp_2l_jcpll_postdiv_d2 = 0x1; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_MMD_PREDIV_MODE, RG_PXP_2L_JCPLL_MMD_PREDIV_MODE.dat.value);

 RG_PXP_2L_JCPLL_RST_DLY.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_RST_DLY);
 RG_PXP_2L_JCPLL_RST_DLY.hal.rg_pxp_2l_jcpll_rst_dly = 0x4; 
 RG_PXP_2L_JCPLL_RST_DLY.hal.rg_pxp_2l_jcpll_sdm_di_ls = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_RST_DLY, RG_PXP_2L_JCPLL_RST_DLY.dat.value);

 RG_PXP_2L_JCPLL_TCL_KBAND_VREF.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_TCL_KBAND_VREF);
 RG_PXP_2L_JCPLL_TCL_KBAND_VREF.hal.rg_pxp_2l_jcpll_vco_kband_meas_en = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_TCL_KBAND_VREF, RG_PXP_2L_JCPLL_TCL_KBAND_VREF.dat.value);

 RG_PXP_2L_JCPLL_IB_EXT_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_IB_EXT_EN);
 RG_PXP_2L_JCPLL_IB_EXT_EN.hal.rg_pxp_2l_jcpll_chp_iofst = 0x0; 
 RG_PXP_2L_JCPLL_IB_EXT_EN.hal.rg_pxp_2l_jcpll_chp_ibias = 0xC; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_IB_EXT_EN, RG_PXP_2L_JCPLL_IB_EXT_EN.dat.value);

 RG_PXP_2L_JCPLL_MMD_PREDIV_MODE.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_MMD_PREDIV_MODE);
 RG_PXP_2L_JCPLL_MMD_PREDIV_MODE.hal.rg_pxp_2l_jcpll_mmd_prediv_mode = 0x1; //Kaiwen improve jitter 10/9
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_MMD_PREDIV_MODE, RG_PXP_2L_JCPLL_MMD_PREDIV_MODE.dat.value);

 RG_PXP_2L_JCPLL_VCODIV.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_VCODIV);
 RG_PXP_2L_JCPLL_VCODIV.hal.rg_pxp_2l_jcpll_vco_halflsb_en = 0x1; //7/8 kaiwen add
 RG_PXP_2L_JCPLL_VCODIV.hal.rg_pxp_2l_jcpll_vco_cfix = 0x1; 
 RG_PXP_2L_JCPLL_VCODIV.hal.rg_pxp_2l_jcpll_vco_scapwr = 0x4; //7/8 kaiwen add
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_VCODIV, RG_PXP_2L_JCPLL_VCODIV.dat.value);

 RG_PXP_2L_JCPLL_IB_EXT_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_IB_EXT_EN);
 RG_PXP_2L_JCPLL_IB_EXT_EN.hal.rg_pxp_2l_jcpll_lpf_shck_en = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_IB_EXT_EN, RG_PXP_2L_JCPLL_IB_EXT_EN.dat.value);

 RG_PXP_2L_JCPLL_KBAND_KFC.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_KBAND_KFC);
 RG_PXP_2L_JCPLL_KBAND_KFC.hal.rg_pxp_2l_jcpll_postdiv_en = 0x1; 
 RG_PXP_2L_JCPLL_KBAND_KFC.hal.rg_pxp_2l_jcpll_kband_kfc = 0x0; 
 RG_PXP_2L_JCPLL_KBAND_KFC.hal.rg_pxp_2l_jcpll_kband_kf = 0x3; 
 RG_PXP_2L_JCPLL_KBAND_KFC.hal.rg_pxp_2l_jcpll_kband_ks = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_KBAND_KFC, RG_PXP_2L_JCPLL_KBAND_KFC.dat.value);

 RG_PXP_2L_JCPLL_LPF_BWC.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_LPF_BWC);
 RG_PXP_2L_JCPLL_LPF_BWC.hal.rg_pxp_2l_jcpll_kband_div = 0x1; //Kaiwen improve jitter 10/9
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_LPF_BWC, RG_PXP_2L_JCPLL_LPF_BWC.dat.value);

 scan_mode.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _scan_mode);
 scan_mode.hal.rg_force_sel_da_pxp_jcpll_kband_load_en = 0x1; //Lane0,Lane1 ????JCPLL
 scan_mode.hal.rg_force_da_pxp_jcpll_kband_load_en = 0x0; 
 Reg_W(PCIE_PMA0, PMA_OFFSET, _scan_mode, scan_mode.dat.value);

 RG_PXP_2L_JCPLL_LPF_BWC.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_LPF_BWC);
 RG_PXP_2L_JCPLL_LPF_BWC.hal.rg_pxp_2l_jcpll_kband_code = 0xE4; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_LPF_BWC, RG_PXP_2L_JCPLL_LPF_BWC.dat.value);

 RG_PXP_2L_JCPLL_SDM_HREN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SDM_HREN);
 RG_PXP_2L_JCPLL_SDM_HREN.hal.rg_pxp_2l_jcpll_tcl_amp_en = 0x1; //Add for 7/6 mail: ??ana??, kaiwen pll BW setting
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SDM_HREN, RG_PXP_2L_JCPLL_SDM_HREN.dat.value);

 RG_PXP_2L_JCPLL_TCL_CMP_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_TCL_CMP_EN);
 RG_PXP_2L_JCPLL_TCL_CMP_EN.hal.rg_pxp_2l_jcpll_tcl_lpf_en = 0x1; //TCL????????????, ????????
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_TCL_CMP_EN, RG_PXP_2L_JCPLL_TCL_CMP_EN.dat.value);

 RG_PXP_2L_JCPLL_TCL_KBAND_VREF.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_TCL_KBAND_VREF);
 RG_PXP_2L_JCPLL_TCL_KBAND_VREF.hal.rg_pxp_2l_jcpll_tcl_kband_vref = 0xF; //Add for 7/6 mail: ??ana??, kaiwen pll BW setting
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_TCL_KBAND_VREF, RG_PXP_2L_JCPLL_TCL_KBAND_VREF.dat.value);

 RG_PXP_2L_JCPLL_SDM_HREN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SDM_HREN);
 RG_PXP_2L_JCPLL_SDM_HREN.hal.rg_pxp_2l_jcpll_tcl_amp_gain = 0x1; //Add for 7/6 mail: ??ana??, kaiwen pll BW setting
 RG_PXP_2L_JCPLL_SDM_HREN.hal.rg_pxp_2l_jcpll_tcl_amp_vref = 0x5; //Add for 7/6 mail: ??ana??, kaiwen pll BW setting
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SDM_HREN, RG_PXP_2L_JCPLL_SDM_HREN.dat.value);

 RG_PXP_2L_JCPLL_TCL_CMP_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_TCL_CMP_EN);
 RG_PXP_2L_JCPLL_TCL_CMP_EN.hal.rg_pxp_2l_jcpll_tcl_lpf_bw = 0x1; //update for 1/6 mail
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_TCL_CMP_EN, RG_PXP_2L_JCPLL_TCL_CMP_EN.dat.value);

 RG_PXP_2L_JCPLL_VCO_TCLVAR.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_VCO_TCLVAR);
 RG_PXP_2L_JCPLL_VCO_TCLVAR.hal.rg_pxp_2l_jcpll_vco_tclvar = 0x3; //update for 1/6 mail
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_VCO_TCLVAR, RG_PXP_2L_JCPLL_VCO_TCLVAR.dat.value); 

 rg_force_da_pxp_jcpll_ckout_en.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_jcpll_ckout_en);
 rg_force_da_pxp_jcpll_ckout_en.hal.rg_force_sel_da_pxp_jcpll_ckout_en = 0x1; 
 rg_force_da_pxp_jcpll_ckout_en.hal.rg_force_da_pxp_jcpll_ckout_en = 0x1; 
 Reg_W(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_jcpll_ckout_en, rg_force_da_pxp_jcpll_ckout_en.dat.value);

 rg_force_da_pxp_jcpll_ckout_en.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_jcpll_ckout_en);
 rg_force_da_pxp_jcpll_ckout_en.hal.rg_force_sel_da_pxp_jcpll_ckout_en = 0x1; 
 rg_force_da_pxp_jcpll_ckout_en.hal.rg_force_da_pxp_jcpll_ckout_en = 0x1; 
 Reg_W(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_jcpll_ckout_en, rg_force_da_pxp_jcpll_ckout_en.dat.value);

 rg_force_da_pxp_jcpll_ckout_en.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_jcpll_ckout_en);
 rg_force_da_pxp_jcpll_ckout_en.hal.rg_force_sel_da_pxp_jcpll_en = 0x1; 
 rg_force_da_pxp_jcpll_ckout_en.hal.rg_force_da_pxp_jcpll_en = 0x1; 
 Reg_W(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_jcpll_ckout_en, rg_force_da_pxp_jcpll_ckout_en.dat.value);

 rg_force_da_pxp_jcpll_ckout_en.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_jcpll_ckout_en);
 rg_force_da_pxp_jcpll_ckout_en.hal.rg_force_sel_da_pxp_jcpll_en = 0x1; 
 rg_force_da_pxp_jcpll_ckout_en.hal.rg_force_da_pxp_jcpll_en = 0x1; 
 Reg_W(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_jcpll_ckout_en, rg_force_da_pxp_jcpll_ckout_en.dat.value);

 udelay(200);


//*******  TXPLL SETTING
 rg_force_da_pxp_txpll_ckout_en.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_txpll_ckout_en);
 rg_force_da_pxp_txpll_ckout_en.hal.rg_force_sel_da_pxp_txpll_en = 0x1; 
 rg_force_da_pxp_txpll_ckout_en.hal.rg_force_da_pxp_txpll_en = 0x0; 
 Reg_W(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_txpll_ckout_en, rg_force_da_pxp_txpll_ckout_en.dat.value);

 rg_force_da_pxp_txpll_ckout_en.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_txpll_ckout_en);
 rg_force_da_pxp_txpll_ckout_en.hal.rg_force_sel_da_pxp_txpll_en = 0x1; 
 rg_force_da_pxp_txpll_ckout_en.hal.rg_force_da_pxp_txpll_en = 0x0; 
 Reg_W(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_txpll_ckout_en, rg_force_da_pxp_txpll_ckout_en.dat.value);

 RG_PXP_2L_TXPLL_REFIN_DIV.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_REFIN_DIV);
 RG_PXP_2L_TXPLL_REFIN_DIV.hal.rg_pxp_2l_txpll_pll_rstb = 0x1; //Add for 3/22 mail: PLL FLOW ??
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_REFIN_DIV, RG_PXP_2L_TXPLL_REFIN_DIV.dat.value);

 RG_PXP_2L_TXPLL_SSC_DELTA1.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SSC_DELTA1);
 RG_PXP_2L_TXPLL_SSC_DELTA1.hal.rg_pxp_2l_txpll_ssc_delta = 0x0; 
 RG_PXP_2L_TXPLL_SSC_DELTA1.hal.rg_pxp_2l_txpll_ssc_delta1 = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SSC_DELTA1, RG_PXP_2L_TXPLL_SSC_DELTA1.dat.value);

 RG_PXP_2L_TXPLL_SSC_PERIOD.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SSC_PERIOD);
 RG_PXP_2L_TXPLL_SSC_PERIOD.hal.rg_pxp_2l_txpll_ssc_period = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SSC_PERIOD, RG_PXP_2L_TXPLL_SSC_PERIOD.dat.value);

 RG_PXP_2L_TXPLL_CHP_IOFST.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_CHP_IOFST);
 RG_PXP_2L_TXPLL_CHP_IOFST.hal.rg_pxp_2l_txpll_chp_iofst = 0x1; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_CHP_IOFST, RG_PXP_2L_TXPLL_CHP_IOFST.dat.value);

 RG_PXP_2L_750M_SYS_CK_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_750M_SYS_CK_EN);
 RG_PXP_2L_750M_SYS_CK_EN.hal.rg_pxp_2l_txpll_chp_ibias = 0x2D; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_750M_SYS_CK_EN, RG_PXP_2L_750M_SYS_CK_EN.dat.value);

 RG_PXP_2L_TXPLL_REFIN_DIV.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_REFIN_DIV);
 RG_PXP_2L_TXPLL_REFIN_DIV.hal.rg_pxp_2l_txpll_refin_div = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_REFIN_DIV, RG_PXP_2L_TXPLL_REFIN_DIV.dat.value);

 RG_PXP_2L_TXPLL_TCL_LPF_BW.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_LPF_BW);
 RG_PXP_2L_TXPLL_TCL_LPF_BW.hal.rg_pxp_2l_txpll_vco_cfix = 0x3; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_LPF_BW, RG_PXP_2L_TXPLL_TCL_LPF_BW.dat.value);

 rg_force_da_pxp_cdr_pr_idac.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_idac);
 rg_force_da_pxp_cdr_pr_idac.hal.rg_force_sel_da_pxp_txpll_sdm_pcw = 0x1; 
 Reg_W(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_idac, rg_force_da_pxp_cdr_pr_idac.dat.value);

 rg_force_da_pxp_cdr_pr_idac.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_idac);
 rg_force_da_pxp_cdr_pr_idac.hal.rg_force_sel_da_pxp_txpll_sdm_pcw = 0x1; 
 Reg_W(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_idac, rg_force_da_pxp_cdr_pr_idac.dat.value);

 rg_force_da_pxp_txpll_sdm_pcw.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_txpll_sdm_pcw);
 rg_force_da_pxp_txpll_sdm_pcw.hal.rg_force_da_pxp_txpll_sdm_pcw = 0xC800000; 
 Reg_W(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_txpll_sdm_pcw, rg_force_da_pxp_txpll_sdm_pcw.dat.value);

 rg_force_da_pxp_txpll_sdm_pcw.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_txpll_sdm_pcw);
 rg_force_da_pxp_txpll_sdm_pcw.hal.rg_force_da_pxp_txpll_sdm_pcw = 0xC800000; 
 Reg_W(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_txpll_sdm_pcw, rg_force_da_pxp_txpll_sdm_pcw.dat.value);

 RG_PXP_2L_TXPLL_SDM_DI_LS.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SDM_DI_LS);
 RG_PXP_2L_TXPLL_SDM_DI_LS.hal.rg_pxp_2l_txpll_sdm_ifm = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SDM_DI_LS, RG_PXP_2L_TXPLL_SDM_DI_LS.dat.value);

 RG_PXP_2L_TXPLL_SSC_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SSC_EN);
 RG_PXP_2L_TXPLL_SSC_EN.hal.rg_pxp_2l_txpll_ssc_phase_ini = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SSC_EN, RG_PXP_2L_TXPLL_SSC_EN.dat.value);

 RG_PXP_2L_TXPLL_REFIN_DIV.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_REFIN_DIV);
 RG_PXP_2L_TXPLL_REFIN_DIV.hal.rg_pxp_2l_txpll_rst_dly = 0x4; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_REFIN_DIV, RG_PXP_2L_TXPLL_REFIN_DIV.dat.value);

 RG_PXP_2L_TXPLL_SDM_DI_LS.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SDM_DI_LS);
 RG_PXP_2L_TXPLL_SDM_DI_LS.hal.rg_pxp_2l_txpll_sdm_di_ls = 0x0; 
 RG_PXP_2L_TXPLL_SDM_DI_LS.hal.rg_pxp_2l_txpll_sdm_ord = 0x3; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SDM_DI_LS, RG_PXP_2L_TXPLL_SDM_DI_LS.dat.value);

 RG_PXP_2L_TXPLL_TCL_KBAND_VREF.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_KBAND_VREF);
 RG_PXP_2L_TXPLL_TCL_KBAND_VREF.hal.rg_pxp_2l_txpll_vco_kband_meas_en = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_KBAND_VREF, RG_PXP_2L_TXPLL_TCL_KBAND_VREF.dat.value);

 RG_PXP_2L_TXPLL_SSC_DELTA1.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SSC_DELTA1);
 RG_PXP_2L_TXPLL_SSC_DELTA1.hal.rg_pxp_2l_txpll_ssc_delta = 0x0; 
 RG_PXP_2L_TXPLL_SSC_DELTA1.hal.rg_pxp_2l_txpll_ssc_delta1 = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SSC_DELTA1, RG_PXP_2L_TXPLL_SSC_DELTA1.dat.value);

 RG_PXP_2L_TXPLL_CHP_IOFST.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_CHP_IOFST);
 RG_PXP_2L_TXPLL_CHP_IOFST.hal.rg_pxp_2l_txpll_lpf_bp = 0x1; 
 RG_PXP_2L_TXPLL_CHP_IOFST.hal.rg_pxp_2l_txpll_lpf_bc = 0x18; 
 RG_PXP_2L_TXPLL_CHP_IOFST.hal.rg_pxp_2l_txpll_lpf_br = 0x5; 
 RG_PXP_2L_TXPLL_CHP_IOFST.hal.rg_pxp_2l_txpll_chp_iofst = 0x1; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_CHP_IOFST, RG_PXP_2L_TXPLL_CHP_IOFST.dat.value);

 RG_PXP_2L_750M_SYS_CK_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_750M_SYS_CK_EN);
 RG_PXP_2L_750M_SYS_CK_EN.hal.rg_pxp_2l_txpll_chp_ibias = 0x2D; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_750M_SYS_CK_EN, RG_PXP_2L_750M_SYS_CK_EN.dat.value);

 RG_PXP_2L_TXPLL_TCL_VTP_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_VTP_EN);
 RG_PXP_2L_TXPLL_TCL_VTP_EN.hal.rg_pxp_2l_txpll_spare_l = 0x1; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_VTP_EN, RG_PXP_2L_TXPLL_TCL_VTP_EN.dat.value);

 RG_PXP_2L_TXPLL_LPF_BWR.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_LPF_BWR);
 RG_PXP_2L_TXPLL_LPF_BWR.hal.rg_pxp_2l_txpll_lpf_bwc = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_LPF_BWR, RG_PXP_2L_TXPLL_LPF_BWR.dat.value);

 RG_PXP_2L_TXPLL_POSTDIV_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_POSTDIV_EN);
 RG_PXP_2L_TXPLL_POSTDIV_EN.hal.rg_pxp_2l_txpll_mmd_prediv_mode = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_POSTDIV_EN, RG_PXP_2L_TXPLL_POSTDIV_EN.dat.value);

 RG_PXP_2L_TXPLL_REFIN_DIV.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_REFIN_DIV);
 RG_PXP_2L_TXPLL_REFIN_DIV.hal.rg_pxp_2l_txpll_refin_div = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_REFIN_DIV, RG_PXP_2L_TXPLL_REFIN_DIV.dat.value);

 RG_PXP_2L_TXPLL_TCL_LPF_BW.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_LPF_BW);
 RG_PXP_2L_TXPLL_TCL_LPF_BW.hal.rg_pxp_2l_txpll_vco_halflsb_en = 0x1; //7/8 kaiwen add
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_LPF_BW, RG_PXP_2L_TXPLL_TCL_LPF_BW.dat.value);

 RG_PXP_2L_TXPLL_VCO_SCAPWR.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_VCO_SCAPWR);
 RG_PXP_2L_TXPLL_VCO_SCAPWR.hal.rg_pxp_2l_txpll_vco_scapwr = 0x7; //7/8 kaiwen add
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_VCO_SCAPWR, RG_PXP_2L_TXPLL_VCO_SCAPWR.dat.value);

 RG_PXP_2L_TXPLL_TCL_LPF_BW.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_LPF_BW);
 RG_PXP_2L_TXPLL_TCL_LPF_BW.hal.rg_pxp_2l_txpll_vco_cfix = 0x3; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_LPF_BW, RG_PXP_2L_TXPLL_TCL_LPF_BW.dat.value);

 rg_force_da_pxp_cdr_pr_idac.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_idac);
 rg_force_da_pxp_cdr_pr_idac.hal.rg_force_sel_da_pxp_txpll_sdm_pcw = 0x1; 
 Reg_W(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_idac, rg_force_da_pxp_cdr_pr_idac.dat.value);

 rg_force_da_pxp_cdr_pr_idac.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_idac);
 rg_force_da_pxp_cdr_pr_idac.hal.rg_force_sel_da_pxp_txpll_sdm_pcw = 0x1; 
 Reg_W(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_idac, rg_force_da_pxp_cdr_pr_idac.dat.value);

 RG_PXP_2L_TXPLL_SSC_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SSC_EN);
 RG_PXP_2L_TXPLL_SSC_EN.hal.rg_pxp_2l_txpll_ssc_phase_ini = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SSC_EN, RG_PXP_2L_TXPLL_SSC_EN.dat.value);

 RG_PXP_2L_TXPLL_LPF_BWR.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_LPF_BWR);
 RG_PXP_2L_TXPLL_LPF_BWR.hal.rg_pxp_2l_txpll_lpf_bwr = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_LPF_BWR, RG_PXP_2L_TXPLL_LPF_BWR.dat.value);

 RG_PXP_2L_TXPLL_PHY_CK2_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_PHY_CK2_EN);
 RG_PXP_2L_TXPLL_PHY_CK2_EN.hal.rg_pxp_2l_txpll_refin_internal = 0x1; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_PHY_CK2_EN, RG_PXP_2L_TXPLL_PHY_CK2_EN.dat.value);

 RG_PXP_2L_TXPLL_TCL_KBAND_VREF.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_KBAND_VREF);
 RG_PXP_2L_TXPLL_TCL_KBAND_VREF.hal.rg_pxp_2l_txpll_vco_kband_meas_en = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_KBAND_VREF, RG_PXP_2L_TXPLL_TCL_KBAND_VREF.dat.value);

 RG_PXP_2L_TXPLL_VTP_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_VTP_EN);
 RG_PXP_2L_TXPLL_VTP_EN.hal.rg_pxp_2l_txpll_vtp_en = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_VTP_EN, RG_PXP_2L_TXPLL_VTP_EN.dat.value);

 RG_PXP_2L_TXPLL_POSTDIV_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_POSTDIV_EN);
 RG_PXP_2L_TXPLL_POSTDIV_EN.hal.rg_pxp_2l_txpll_phy_ck1_en = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_POSTDIV_EN, RG_PXP_2L_TXPLL_POSTDIV_EN.dat.value);

 RG_PXP_2L_TXPLL_PHY_CK2_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_PHY_CK2_EN);
 RG_PXP_2L_TXPLL_PHY_CK2_EN.hal.rg_pxp_2l_txpll_refin_internal = 0x1; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_PHY_CK2_EN, RG_PXP_2L_TXPLL_PHY_CK2_EN.dat.value);

 RG_PXP_2L_TXPLL_SSC_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SSC_EN);
 RG_PXP_2L_TXPLL_SSC_EN.hal.rg_pxp_2l_txpll_ssc_en = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SSC_EN, RG_PXP_2L_TXPLL_SSC_EN.dat.value);

 RG_PXP_2L_750M_SYS_CK_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_750M_SYS_CK_EN);
 RG_PXP_2L_750M_SYS_CK_EN.hal.rg_pxp_2l_txpll_lpf_shck_en = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_750M_SYS_CK_EN, RG_PXP_2L_750M_SYS_CK_EN.dat.value);

 RG_PXP_2L_TXPLL_POSTDIV_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_POSTDIV_EN);
 RG_PXP_2L_TXPLL_POSTDIV_EN.hal.rg_pxp_2l_txpll_postdiv_en = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_POSTDIV_EN, RG_PXP_2L_TXPLL_POSTDIV_EN.dat.value);

 RG_PXP_2L_TXPLL_KBAND_DIV.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_KBAND_DIV);
 RG_PXP_2L_TXPLL_KBAND_DIV.hal.rg_pxp_2l_txpll_kband_kfc = 0x0; 
 RG_PXP_2L_TXPLL_KBAND_DIV.hal.rg_pxp_2l_txpll_kband_kf = 0x3; 
 RG_PXP_2L_TXPLL_KBAND_DIV.hal.rg_pxp_2l_txpll_kband_ks = 0x1; 
 RG_PXP_2L_TXPLL_KBAND_DIV.hal.rg_pxp_2l_txpll_kband_div = 0x4; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_KBAND_DIV, RG_PXP_2L_TXPLL_KBAND_DIV.dat.value);

 RG_PXP_2L_TXPLL_LPF_BWR.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_LPF_BWR);
 RG_PXP_2L_TXPLL_LPF_BWR.hal.rg_pxp_2l_txpll_kband_code = 0xE4; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_LPF_BWR, RG_PXP_2L_TXPLL_LPF_BWR.dat.value);

 RG_PXP_2L_TXPLL_SDM_OUT.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SDM_OUT);
 RG_PXP_2L_TXPLL_SDM_OUT.hal.rg_pxp_2l_txpll_tcl_amp_en = 0x1; //Add for 7/6 mail: ??ana??for TCL, kaiwen pll BW setting
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SDM_OUT, RG_PXP_2L_TXPLL_SDM_OUT.dat.value);

 RG_PXP_2L_TXPLL_TCL_AMP_VREF.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_AMP_VREF);
 RG_PXP_2L_TXPLL_TCL_AMP_VREF.hal.rg_pxp_2l_txpll_tcl_lpf_en = 0x1; //TCL????????????, ????????
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_AMP_VREF, RG_PXP_2L_TXPLL_TCL_AMP_VREF.dat.value);

 RG_PXP_2L_TXPLL_TCL_KBAND_VREF.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_KBAND_VREF);
 RG_PXP_2L_TXPLL_TCL_KBAND_VREF.hal.rg_pxp_2l_txpll_tcl_kband_vref = 0xF; //Add for 7/6 mail: ??ana??, kaiwen pll BW setting
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_KBAND_VREF, RG_PXP_2L_TXPLL_TCL_KBAND_VREF.dat.value);

 RG_PXP_2L_TXPLL_SDM_OUT.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SDM_OUT);
 RG_PXP_2L_TXPLL_SDM_OUT.hal.rg_pxp_2l_txpll_tcl_amp_gain = 0x3; //Add for 7/6 mail: ??ana??, kaiwen pll BW setting
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SDM_OUT, RG_PXP_2L_TXPLL_SDM_OUT.dat.value);

 RG_PXP_2L_TXPLL_TCL_AMP_VREF.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_AMP_VREF);
 RG_PXP_2L_TXPLL_TCL_AMP_VREF.hal.rg_pxp_2l_txpll_tcl_amp_vref = 0xB; //Add for 7/6 mail: ??ana??, kaiwen pll BW setting
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_AMP_VREF, RG_PXP_2L_TXPLL_TCL_AMP_VREF.dat.value);

 RG_PXP_2L_TXPLL_TCL_LPF_BW.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_LPF_BW);
 RG_PXP_2L_TXPLL_TCL_LPF_BW.hal.rg_pxp_2l_txpll_tcl_lpf_bw = 0x3; //Add for 7/6 mail: ??ana??, kaiwen pll BW setting
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_LPF_BW, RG_PXP_2L_TXPLL_TCL_LPF_BW.dat.value);

 rg_force_da_pxp_txpll_ckout_en.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_txpll_ckout_en);
 rg_force_da_pxp_txpll_ckout_en.hal.rg_force_sel_da_pxp_txpll_ckout_en = 0x1; 
 rg_force_da_pxp_txpll_ckout_en.hal.rg_force_da_pxp_txpll_ckout_en = 0x1; 
 Reg_W(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_txpll_ckout_en, rg_force_da_pxp_txpll_ckout_en.dat.value);

 rg_force_da_pxp_txpll_ckout_en.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_txpll_ckout_en);
 rg_force_da_pxp_txpll_ckout_en.hal.rg_force_sel_da_pxp_txpll_ckout_en = 0x1; 
 rg_force_da_pxp_txpll_ckout_en.hal.rg_force_da_pxp_txpll_ckout_en = 0x1; 
 Reg_W(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_txpll_ckout_en, rg_force_da_pxp_txpll_ckout_en.dat.value);

 rg_force_da_pxp_txpll_ckout_en.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_txpll_ckout_en);
 rg_force_da_pxp_txpll_ckout_en.hal.rg_force_sel_da_pxp_txpll_en = 0x1; 
 rg_force_da_pxp_txpll_ckout_en.hal.rg_force_da_pxp_txpll_en = 0x1; 
 Reg_W(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_txpll_ckout_en, rg_force_da_pxp_txpll_ckout_en.dat.value);

 rg_force_da_pxp_txpll_ckout_en.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_txpll_ckout_en);
 rg_force_da_pxp_txpll_ckout_en.hal.rg_force_sel_da_pxp_txpll_en = 0x1; 
 rg_force_da_pxp_txpll_ckout_en.hal.rg_force_da_pxp_txpll_en = 0x1; 
 Reg_W(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_txpll_ckout_en, rg_force_da_pxp_txpll_ckout_en.dat.value);

 udelay(200);


//*******  SSC JCPLL SETTING
 RG_PXP_2L_JCPLL_SSC_DELTA1.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SSC_DELTA1);
 RG_PXP_2L_JCPLL_SSC_DELTA1.hal.rg_pxp_2l_jcpll_ssc_delta = 0x106; 
 RG_PXP_2L_JCPLL_SSC_DELTA1.hal.rg_pxp_2l_jcpll_ssc_delta1 = 0x106; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SSC_DELTA1, RG_PXP_2L_JCPLL_SSC_DELTA1.dat.value);

 RG_PXP_2L_JCPLL_SSC_PERIOD.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SSC_PERIOD);
 RG_PXP_2L_JCPLL_SSC_PERIOD.hal.rg_pxp_2l_jcpll_ssc_period = 0x31B; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SSC_PERIOD, RG_PXP_2L_JCPLL_SSC_PERIOD.dat.value);

 RG_PXP_2L_JCPLL_SSC_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SSC_EN);
 RG_PXP_2L_JCPLL_SSC_EN.hal.rg_pxp_2l_jcpll_ssc_phase_ini = 0x1; 
 RG_PXP_2L_JCPLL_SSC_EN.hal.rg_pxp_2l_jcpll_ssc_en = 0x1; //enable SSC after TXPLL complete cal
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SSC_EN, RG_PXP_2L_JCPLL_SSC_EN.dat.value);

 RG_PXP_2L_JCPLL_SDM_IFM.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SDM_IFM);
 RG_PXP_2L_JCPLL_SDM_IFM.hal.rg_pxp_2l_jcpll_sdm_ifm = 0x1; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SDM_IFM, RG_PXP_2L_JCPLL_SDM_IFM.dat.value);

 RG_PXP_2L_JCPLL_SDM_HREN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SDM_HREN);
 RG_PXP_2L_JCPLL_SDM_HREN.hal.rg_pxp_2l_jcpll_sdm_hren = 0x1; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SDM_HREN, RG_PXP_2L_JCPLL_SDM_HREN.dat.value);

 RG_PXP_2L_JCPLL_RST_DLY.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_RST_DLY);
 RG_PXP_2L_JCPLL_RST_DLY.hal.rg_pxp_2l_jcpll_sdm_di_en = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_RST_DLY, RG_PXP_2L_JCPLL_RST_DLY.dat.value);

 RG_PXP_2L_JCPLL_SSC_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SSC_EN);
 RG_PXP_2L_JCPLL_SSC_EN.hal.rg_pxp_2l_jcpll_ssc_tri_en = 0x1; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SSC_EN, RG_PXP_2L_JCPLL_SSC_EN.dat.value);

 udelay(30);


//*******  Rx lan0 signal detect setting , add by Carl 10/4 
 RG_PXP_2L_CDR0_PR_COR_HBW_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR0_PR_COR_HBW_EN);
 RG_PXP_2L_CDR0_PR_COR_HBW_EN.hal.rg_pxp_2l_cdr0_pr_ldo_force_on = 0x1; //10/26 PCIe RX flow rx_sigdet_cal_en 1=>0 power on 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR0_PR_COR_HBW_EN, RG_PXP_2L_CDR0_PR_COR_HBW_EN.dat.value);

 udelay(10);
 ADD_DIG_RESERVE_19.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _ADD_DIG_RESERVE_19);
 tmp = ADD_DIG_RESERVE_19.hal.rg_dig_reserve_19;
 tmp = (0x18B0<<16) | (tmp & (~(0xFFFF<<16)));
 ADD_DIG_RESERVE_19.hal.rg_dig_reserve_19 = tmp;//16'b0001,1000,1011,0000 (for auto change  G1)
 Reg_W(PCIE_PMA0, PMA_OFFSET, _ADD_DIG_RESERVE_19, ADD_DIG_RESERVE_19.dat.value);

 ADD_DIG_RESERVE_20.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _ADD_DIG_RESERVE_20);
 tmp = ADD_DIG_RESERVE_20.hal.rg_dig_reserve_20;
 tmp = (0x18B0<<0) | (tmp & (~(0xFFFF<<0)));
 ADD_DIG_RESERVE_20.hal.rg_dig_reserve_20 = tmp;//16'b0001,1000,1011,0000 (for  auto change G2)
 tmp = ADD_DIG_RESERVE_20.hal.rg_dig_reserve_20;
 tmp = (0x1030<<16) | (tmp & (~(0xFFFF<<16)));
 ADD_DIG_RESERVE_20.hal.rg_dig_reserve_20 = tmp;//16'b0001,0000,0011,0000 (for  auto change G3)
 Reg_W(PCIE_PMA0, PMA_OFFSET, _ADD_DIG_RESERVE_20, ADD_DIG_RESERVE_20.dat.value);

 RG_PXP_2L_RX0_SIGDET_DCTEST_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX0_SIGDET_DCTEST_EN);
 RG_PXP_2L_RX0_SIGDET_DCTEST_EN.hal.rg_pxp_2l_rx0_sigdet_peak = 0x2; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX0_SIGDET_DCTEST_EN, RG_PXP_2L_RX0_SIGDET_DCTEST_EN.dat.value);

 RG_PXP_2L_RX0_SIGDET_VTH_SEL.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX0_SIGDET_VTH_SEL);
 RG_PXP_2L_RX0_SIGDET_VTH_SEL.hal.rg_pxp_2l_rx0_sigdet_vth_sel = 0x5; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX0_SIGDET_VTH_SEL, RG_PXP_2L_RX0_SIGDET_VTH_SEL.dat.value);

 RG_PXP_2L_RX0_REV_0.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX0_REV_0);
 tmp = RG_PXP_2L_RX0_REV_0.hal.rg_pxp_2l_rx0_rev_1;
 tmp = (0x2<<2) | (tmp & (~(0x3<<2)));
 RG_PXP_2L_RX0_REV_0.hal.rg_pxp_2l_rx0_rev_1 = tmp;
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX0_REV_0, RG_PXP_2L_RX0_REV_0.dat.value);

 RG_PXP_2L_RX0_SIGDET_DCTEST_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX0_SIGDET_DCTEST_EN);
 RG_PXP_2L_RX0_SIGDET_DCTEST_EN.hal.rg_pxp_2l_rx0_sigdet_lpf_ctrl = 0x1; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX0_SIGDET_DCTEST_EN, RG_PXP_2L_RX0_SIGDET_DCTEST_EN.dat.value);

 SS_RX_CAL_2.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _SS_RX_CAL_2);
 SS_RX_CAL_2.hal.rg_cal_out_os = 0x0; 
 Reg_W(PCIE_PMA0, PMA_OFFSET, _SS_RX_CAL_2, SS_RX_CAL_2.dat.value);

 RG_PXP_2L_RX0_FE_VB_EQ2_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX0_FE_VB_EQ2_EN);
 RG_PXP_2L_RX0_FE_VB_EQ2_EN.hal.rg_pxp_2l_rx0_fe_vcm_gen_pwdb = 0x1; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX0_FE_VB_EQ2_EN, RG_PXP_2L_RX0_FE_VB_EQ2_EN.dat.value);

 rg_force_da_pxp_rx_fe_gain_ctrl.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_rx_fe_gain_ctrl);
 rg_force_da_pxp_rx_fe_gain_ctrl.hal.rg_force_sel_da_pxp_rx_fe_gain_ctrl = 0x1; 
 rg_force_da_pxp_rx_fe_gain_ctrl.hal.rg_force_da_pxp_rx_fe_gain_ctrl = rx_fe_gain_ctrl; //[1:0]=1 for gen3 GT;  [1:0]=3 for gen2 PT
 Reg_W(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_rx_fe_gain_ctrl, rg_force_da_pxp_rx_fe_gain_ctrl.dat.value);

 RX_FORCE_MODE_0.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _RX_FORCE_MODE_0);
 RX_FORCE_MODE_0.hal.rg_force_da_xpon_rx_fe_gain_ctrl = 0x1; 
 Reg_W(PCIE_PMA0, PMA_OFFSET, _RX_FORCE_MODE_0, RX_FORCE_MODE_0.dat.value);

 SS_RX_SIGDET_0.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _SS_RX_SIGDET_0);
 SS_RX_SIGDET_0.hal.rg_sigdet_win_nonvld_times = 0x3; 
 Reg_W(PCIE_PMA0, PMA_OFFSET, _SS_RX_SIGDET_0, SS_RX_SIGDET_0.dat.value);

 RX_CTRL_SEQUENCE_DISB_CTRL_1.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _RX_CTRL_SEQUENCE_DISB_CTRL_1);
 RX_CTRL_SEQUENCE_DISB_CTRL_1.hal.rg_disb_rx_sdcal_en = 0x0; //force mode on for signal detect
 Reg_W(PCIE_PMA0, PMA_OFFSET, _RX_CTRL_SEQUENCE_DISB_CTRL_1, RX_CTRL_SEQUENCE_DISB_CTRL_1.dat.value);

 RX_CTRL_SEQUENCE_FORCE_CTRL_1.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _RX_CTRL_SEQUENCE_FORCE_CTRL_1);
 RX_CTRL_SEQUENCE_FORCE_CTRL_1.hal.rg_force_rx_sdcal_en = 0x1; //force mode value
 Reg_W(PCIE_PMA0, PMA_OFFSET, _RX_CTRL_SEQUENCE_FORCE_CTRL_1, RX_CTRL_SEQUENCE_FORCE_CTRL_1.dat.value);

 udelay(100);
 RX_CTRL_SEQUENCE_FORCE_CTRL_1.hal.rg_force_rx_sdcal_en = 0x0; //10/26 PCIe RX flow rx_sigdet_cal_en 1=>0 before JCPLL, sgdet cal close
 Reg_W(PCIE_PMA0, PMA_OFFSET, _RX_CTRL_SEQUENCE_FORCE_CTRL_1, RX_CTRL_SEQUENCE_FORCE_CTRL_1.dat.value);



//*******  Rx lan1 signal detect
 RG_PXP_2L_CDR1_PR_COR_HBW_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR1_PR_COR_HBW_EN);
 RG_PXP_2L_CDR1_PR_COR_HBW_EN.hal.rg_pxp_2l_cdr1_pr_ldo_force_on = 0x1; //10/26 PCIe RX flow rx_sigdet_cal_en 1=>0 power on 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR1_PR_COR_HBW_EN, RG_PXP_2L_CDR1_PR_COR_HBW_EN.dat.value);

 udelay(10);
 ADD_DIG_RESERVE_19.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _ADD_DIG_RESERVE_19);
 tmp = ADD_DIG_RESERVE_19.hal.rg_dig_reserve_19;
 tmp = (0x18B0<<16) | (tmp & (~(0xFFFF<<16)));
 ADD_DIG_RESERVE_19.hal.rg_dig_reserve_19 = tmp;//16'b0001,1000,1011,0000 (for auto change  G1)
 Reg_W(PCIE_PMA1, PMA_OFFSET, _ADD_DIG_RESERVE_19, ADD_DIG_RESERVE_19.dat.value);

 ADD_DIG_RESERVE_20.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _ADD_DIG_RESERVE_20);
 tmp = ADD_DIG_RESERVE_20.hal.rg_dig_reserve_20;
 tmp = (0x18B0<<0) | (tmp & (~(0xFFFF<<0)));
 ADD_DIG_RESERVE_20.hal.rg_dig_reserve_20 = tmp;//16'b0001,1000,1011,0000 (for  auto change G2)
 tmp = ADD_DIG_RESERVE_20.hal.rg_dig_reserve_20;
 tmp = (0x1030<<16) | (tmp & (~(0xFFFF<<16)));
 ADD_DIG_RESERVE_20.hal.rg_dig_reserve_20 = tmp;//16'b0001,0000,0011,0000 (for  auto change G3)
 Reg_W(PCIE_PMA1, PMA_OFFSET, _ADD_DIG_RESERVE_20, ADD_DIG_RESERVE_20.dat.value);

 RG_PXP_2L_RX1_SIGDET_NOVTH.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX1_SIGDET_NOVTH);
 RG_PXP_2L_RX1_SIGDET_NOVTH.hal.rg_pxp_2l_rx1_sigdet_peak = 0x2; 
 RG_PXP_2L_RX1_SIGDET_NOVTH.hal.rg_pxp_2l_rx1_sigdet_vth_sel = 0x5; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX1_SIGDET_NOVTH, RG_PXP_2L_RX1_SIGDET_NOVTH.dat.value);

 RG_PXP_2L_RX1_REV_0.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX1_REV_0);
 tmp = RG_PXP_2L_RX1_REV_0.hal.rg_pxp_2l_rx1_rev_1;
 tmp = (0x2<<2) | (tmp & (~(0x3<<2)));
 RG_PXP_2L_RX1_REV_0.hal.rg_pxp_2l_rx1_rev_1 = tmp;
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX1_REV_0, RG_PXP_2L_RX1_REV_0.dat.value);

 RG_PXP_2L_RX1_DAC_RANGE_EYE.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX1_DAC_RANGE_EYE);
 RG_PXP_2L_RX1_DAC_RANGE_EYE.hal.rg_pxp_2l_rx1_sigdet_lpf_ctrl = 0x1; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX1_DAC_RANGE_EYE, RG_PXP_2L_RX1_DAC_RANGE_EYE.dat.value);

 SS_RX_CAL_2.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _SS_RX_CAL_2);
 SS_RX_CAL_2.hal.rg_cal_out_os = 0x0; 
 Reg_W(PCIE_PMA1, PMA_OFFSET, _SS_RX_CAL_2, SS_RX_CAL_2.dat.value);

 RG_PXP_2L_RX1_FE_VB_EQ1_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX1_FE_VB_EQ1_EN);
 RG_PXP_2L_RX1_FE_VB_EQ1_EN.hal.rg_pxp_2l_rx1_fe_vcm_gen_pwdb = 0x1; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX1_FE_VB_EQ1_EN, RG_PXP_2L_RX1_FE_VB_EQ1_EN.dat.value);

 rg_force_da_pxp_rx_fe_gain_ctrl.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_rx_fe_gain_ctrl);
 rg_force_da_pxp_rx_fe_gain_ctrl.hal.rg_force_sel_da_pxp_rx_fe_gain_ctrl = 0x1; 
 rg_force_da_pxp_rx_fe_gain_ctrl.hal.rg_force_da_pxp_rx_fe_gain_ctrl = rx_fe_gain_ctrl; //[1:0]=1 for gen3 GT;  [1:0]=3 for gen2 PT
 Reg_W(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_rx_fe_gain_ctrl, rg_force_da_pxp_rx_fe_gain_ctrl.dat.value);

 RX_FORCE_MODE_0.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _RX_FORCE_MODE_0);
 RX_FORCE_MODE_0.hal.rg_force_da_xpon_rx_fe_gain_ctrl = 0x1; 
 Reg_W(PCIE_PMA1, PMA_OFFSET, _RX_FORCE_MODE_0, RX_FORCE_MODE_0.dat.value);

 SS_RX_SIGDET_0.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _SS_RX_SIGDET_0);
 SS_RX_SIGDET_0.hal.rg_sigdet_win_nonvld_times = 0x3; 
 Reg_W(PCIE_PMA1, PMA_OFFSET, _SS_RX_SIGDET_0, SS_RX_SIGDET_0.dat.value);

 RX_CTRL_SEQUENCE_DISB_CTRL_1.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _RX_CTRL_SEQUENCE_DISB_CTRL_1);
 RX_CTRL_SEQUENCE_DISB_CTRL_1.hal.rg_disb_rx_sdcal_en = 0x0; //force mode on for signal detect
 Reg_W(PCIE_PMA1, PMA_OFFSET, _RX_CTRL_SEQUENCE_DISB_CTRL_1, RX_CTRL_SEQUENCE_DISB_CTRL_1.dat.value);

 RX_CTRL_SEQUENCE_FORCE_CTRL_1.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _RX_CTRL_SEQUENCE_FORCE_CTRL_1);
 RX_CTRL_SEQUENCE_FORCE_CTRL_1.hal.rg_force_rx_sdcal_en = 0x1; //force mode value
 Reg_W(PCIE_PMA1, PMA_OFFSET, _RX_CTRL_SEQUENCE_FORCE_CTRL_1, RX_CTRL_SEQUENCE_FORCE_CTRL_1.dat.value);

 udelay(100);
 RX_CTRL_SEQUENCE_FORCE_CTRL_1.hal.rg_force_rx_sdcal_en = 0x0; //10/26 PCIe RX flow rx_sigdet_cal_en 1=>0 before JCPLL
 Reg_W(PCIE_PMA1, PMA_OFFSET, _RX_CTRL_SEQUENCE_FORCE_CTRL_1, RX_CTRL_SEQUENCE_FORCE_CTRL_1.dat.value);



//*******  RX FLOW
 rg_force_da_pxp_rx_scan_rst_b.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_rx_scan_rst_b);
 rg_force_da_pxp_rx_scan_rst_b.hal.rg_force_sel_da_pxp_rx_sigdet_pwdb = 0x1; //toggle power down, reset PR 1st time
 rg_force_da_pxp_rx_scan_rst_b.hal.rg_force_da_pxp_rx_sigdet_pwdb = 0x1; 
 Reg_W(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_rx_scan_rst_b, rg_force_da_pxp_rx_scan_rst_b.dat.value);

 rg_force_da_pxp_rx_scan_rst_b.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_rx_scan_rst_b);
 rg_force_da_pxp_rx_scan_rst_b.hal.rg_force_sel_da_pxp_rx_sigdet_pwdb = 0x1; 
 rg_force_da_pxp_rx_scan_rst_b.hal.rg_force_da_pxp_rx_sigdet_pwdb = 0x1; 
 Reg_W(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_rx_scan_rst_b, rg_force_da_pxp_rx_scan_rst_b.dat.value);

 rg_force_da_pxp_cdr_pd_pwdb.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_cdr_pd_pwdb);
 rg_force_da_pxp_cdr_pd_pwdb.hal.rg_force_sel_da_pxp_cdr_pd_pwdb = 0x1; 
 rg_force_da_pxp_cdr_pd_pwdb.hal.rg_force_da_pxp_cdr_pd_pwdb = 0x1; //remove force mode if need to power down CDR when L1 state
 Reg_W(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_cdr_pd_pwdb, rg_force_da_pxp_cdr_pd_pwdb.dat.value);

 rg_force_da_pxp_rx_fe_pwdb.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_rx_fe_pwdb);
 rg_force_da_pxp_rx_fe_pwdb.hal.rg_force_sel_da_pxp_rx_fe_pwdb = 0x1; //remove force mode if need to power down CDR when L1 state
 rg_force_da_pxp_rx_fe_pwdb.hal.rg_force_da_pxp_rx_fe_pwdb = 0x1; 
 Reg_W(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_rx_fe_pwdb, rg_force_da_pxp_rx_fe_pwdb.dat.value);

 rg_force_da_pxp_cdr_pd_pwdb.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_cdr_pd_pwdb);
 rg_force_da_pxp_cdr_pd_pwdb.hal.rg_force_sel_da_pxp_cdr_pd_pwdb = 0x1; 
 rg_force_da_pxp_cdr_pd_pwdb.hal.rg_force_da_pxp_cdr_pd_pwdb = 0x1; //remove force mode if need to power down CDR when L1 state
 Reg_W(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_cdr_pd_pwdb, rg_force_da_pxp_cdr_pd_pwdb.dat.value);

 rg_force_da_pxp_rx_fe_pwdb.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_rx_fe_pwdb);
 rg_force_da_pxp_rx_fe_pwdb.hal.rg_force_sel_da_pxp_rx_fe_pwdb = 0x1; //remove force mode if need to power down CDR when L1 state
 rg_force_da_pxp_rx_fe_pwdb.hal.rg_force_da_pxp_rx_fe_pwdb = 0x1; 
 Reg_W(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_rx_fe_pwdb, rg_force_da_pxp_rx_fe_pwdb.dat.value);

 RG_PXP_2L_RX0_PHYCK_DIV.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX0_PHYCK_DIV);
 RG_PXP_2L_RX0_PHYCK_DIV.hal.rg_pxp_2l_rx0_tdc_ck_sel = 0x1; 
 RG_PXP_2L_RX0_PHYCK_DIV.hal.rg_pxp_2l_rx0_phyck_rstb = 0x1; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX0_PHYCK_DIV, RG_PXP_2L_RX0_PHYCK_DIV.dat.value);

 RG_PXP_2L_RX1_PHYCK_DIV.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX1_PHYCK_DIV);
 RG_PXP_2L_RX1_PHYCK_DIV.hal.rg_pxp_2l_rx1_tdc_ck_sel = 0x1; 
 RG_PXP_2L_RX1_PHYCK_DIV.hal.rg_pxp_2l_rx1_phyck_rstb = 0x1; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX1_PHYCK_DIV, RG_PXP_2L_RX1_PHYCK_DIV.dat.value);

 SW_RST_SET.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _SW_RST_SET);
 SW_RST_SET.hal.rg_sw_tx_fifo_rst_n = 0x1; 
 SW_RST_SET.hal.rg_sw_allpcs_rst_n = 0x1; 
 SW_RST_SET.hal.rg_sw_pma_rst_n = 0x1; 
 SW_RST_SET.hal.rg_sw_tx_rst_n = 0x1; 
 SW_RST_SET.hal.rg_sw_rx_fifo_rst_n = 0x1; 
 Reg_W(PCIE_PMA0, PMA_OFFSET, _SW_RST_SET, SW_RST_SET.dat.value);

 SW_RST_SET.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _SW_RST_SET);
 SW_RST_SET.hal.rg_sw_tx_fifo_rst_n = 0x1; 
 SW_RST_SET.hal.rg_sw_allpcs_rst_n = 0x1; 
 SW_RST_SET.hal.rg_sw_pma_rst_n = 0x1; 
 SW_RST_SET.hal.rg_sw_tx_rst_n = 0x1; 
 SW_RST_SET.hal.rg_sw_rx_fifo_rst_n = 0x1; 
 Reg_W(PCIE_PMA1, PMA_OFFSET, _SW_RST_SET, SW_RST_SET.dat.value);

 RG_PXP_2L_RX0_FE_VB_EQ2_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX0_FE_VB_EQ2_EN);
 RG_PXP_2L_RX0_FE_VB_EQ2_EN.hal.rg_pxp_2l_rx0_fe_vb_eq3_en = 0x1; 
 RG_PXP_2L_RX0_FE_VB_EQ2_EN.hal.rg_pxp_2l_rx0_fe_vb_eq2_en = 0x1; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX0_FE_VB_EQ2_EN, RG_PXP_2L_RX0_FE_VB_EQ2_EN.dat.value);

 RG_PXP_2L_RX0_SIGDET_VTH_SEL.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX0_SIGDET_VTH_SEL);
 RG_PXP_2L_RX0_SIGDET_VTH_SEL.hal.rg_pxp_2l_rx0_fe_vb_eq1_en = 0x1; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX0_SIGDET_VTH_SEL, RG_PXP_2L_RX0_SIGDET_VTH_SEL.dat.value);

 RG_PXP_2L_RX1_FE_VB_EQ1_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX1_FE_VB_EQ1_EN);
 RG_PXP_2L_RX1_FE_VB_EQ1_EN.hal.rg_pxp_2l_rx1_fe_vb_eq3_en = 0x1; 
 RG_PXP_2L_RX1_FE_VB_EQ1_EN.hal.rg_pxp_2l_rx1_fe_vb_eq2_en = 0x1; 
 RG_PXP_2L_RX1_FE_VB_EQ1_EN.hal.rg_pxp_2l_rx1_fe_vb_eq1_en = 0x1; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX1_FE_VB_EQ1_EN, RG_PXP_2L_RX1_FE_VB_EQ1_EN.dat.value);

 RG_PXP_2L_RX0_REV_0.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX0_REV_0);
 tmp = RG_PXP_2L_RX0_REV_0.hal.rg_pxp_2l_rx0_rev_1;
 tmp = (0x4<<4) | (tmp & (~(0x7<<4)));
 RG_PXP_2L_RX0_REV_0.hal.rg_pxp_2l_rx0_rev_1 = tmp;//Carl update setting: 7/25 mail
 tmp = RG_PXP_2L_RX0_REV_0.hal.rg_pxp_2l_rx0_rev_1;
 tmp = (0x4<<8) | (tmp & (~(0x7<<8)));
 RG_PXP_2L_RX0_REV_0.hal.rg_pxp_2l_rx0_rev_1 = tmp;//Carl update setting: 7/25 mail
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX0_REV_0, RG_PXP_2L_RX0_REV_0.dat.value);

 RG_PXP_2L_RX1_REV_0.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX1_REV_0);
 tmp = RG_PXP_2L_RX1_REV_0.hal.rg_pxp_2l_rx1_rev_1;
 tmp = (0x4<<4) | (tmp & (~(0x7<<4)));
 RG_PXP_2L_RX1_REV_0.hal.rg_pxp_2l_rx1_rev_1 = tmp;//Carl update setting: 7/25 mail
 tmp = RG_PXP_2L_RX1_REV_0.hal.rg_pxp_2l_rx1_rev_1;
 tmp = (0x4<<8) | (tmp & (~(0x7<<8)));
 RG_PXP_2L_RX1_REV_0.hal.rg_pxp_2l_rx1_rev_1 = tmp;//Carl update setting: 7/25 mail
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX1_REV_0, RG_PXP_2L_RX1_REV_0.dat.value);

 udelay(10);


//*******  PR setting : 8/12 mail add 
 RG_PXP_2L_CDR0_PR_VREG_IBAND_VAL.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR0_PR_VREG_IBAND_VAL);
 RG_PXP_2L_CDR0_PR_VREG_IBAND_VAL.hal.rg_pxp_2l_cdr0_pr_vreg_iband_val = 0x5; 
 RG_PXP_2L_CDR0_PR_VREG_IBAND_VAL.hal.rg_pxp_2l_cdr0_pr_vreg_ckbuf_val = 0x5; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR0_PR_VREG_IBAND_VAL, RG_PXP_2L_CDR0_PR_VREG_IBAND_VAL.dat.value);

 RG_PXP_2L_CDR0_PR_CKREF_DIV.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR0_PR_CKREF_DIV);
 RG_PXP_2L_CDR0_PR_CKREF_DIV.hal.rg_pxp_2l_cdr0_pr_ckref_div = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR0_PR_CKREF_DIV, RG_PXP_2L_CDR0_PR_CKREF_DIV.dat.value);

 RG_PXP_2L_CDR0_PR_COR_HBW_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR0_PR_COR_HBW_EN);
 RG_PXP_2L_CDR0_PR_COR_HBW_EN.hal.rg_pxp_2l_cdr0_pr_ckref_div1 = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR0_PR_COR_HBW_EN, RG_PXP_2L_CDR0_PR_COR_HBW_EN.dat.value);

 RG_PXP_2L_CDR1_PR_VREG_IBAND_VAL.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR1_PR_VREG_IBAND_VAL);
 RG_PXP_2L_CDR1_PR_VREG_IBAND_VAL.hal.rg_pxp_2l_cdr1_pr_vreg_iband_val = 0x5; 
 RG_PXP_2L_CDR1_PR_VREG_IBAND_VAL.hal.rg_pxp_2l_cdr1_pr_vreg_ckbuf_val = 0x5; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR1_PR_VREG_IBAND_VAL, RG_PXP_2L_CDR1_PR_VREG_IBAND_VAL.dat.value);

 RG_PXP_2L_CDR1_PR_CKREF_DIV.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR1_PR_CKREF_DIV);
 RG_PXP_2L_CDR1_PR_CKREF_DIV.hal.rg_pxp_2l_cdr1_pr_ckref_div = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR1_PR_CKREF_DIV, RG_PXP_2L_CDR1_PR_CKREF_DIV.dat.value);

 RG_PXP_2L_CDR1_PR_COR_HBW_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR1_PR_COR_HBW_EN);
 RG_PXP_2L_CDR1_PR_COR_HBW_EN.hal.rg_pxp_2l_cdr1_pr_ckref_div1 = 0x0; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR1_PR_COR_HBW_EN, RG_PXP_2L_CDR1_PR_COR_HBW_EN.dat.value);

 RG_PXP_2L_CDR0_LPF_RATIO.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR0_LPF_RATIO);
 RG_PXP_2L_CDR0_LPF_RATIO.hal.rg_pxp_2l_cdr0_lpf_top_lim = 0x20000; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR0_LPF_RATIO, RG_PXP_2L_CDR0_LPF_RATIO.dat.value);

 RG_PXP_2L_CDR1_LPF_RATIO.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR1_LPF_RATIO);
 RG_PXP_2L_CDR1_LPF_RATIO.hal.rg_pxp_2l_cdr1_lpf_top_lim = 0x20000; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR1_LPF_RATIO, RG_PXP_2L_CDR1_LPF_RATIO.dat.value);

 RG_PXP_2L_CDR0_PR_BETA_DAC.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR0_PR_BETA_DAC);
 RG_PXP_2L_CDR0_PR_BETA_DAC.hal.rg_pxp_2l_cdr0_pr_beta_sel = 0x2; //10/27 Improve locking range (default 1)
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR0_PR_BETA_DAC, RG_PXP_2L_CDR0_PR_BETA_DAC.dat.value);

 RG_PXP_2L_CDR1_PR_BETA_DAC.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR1_PR_BETA_DAC);
 RG_PXP_2L_CDR1_PR_BETA_DAC.hal.rg_pxp_2l_cdr1_pr_beta_sel = 0x2; //10/27 Improve locking range
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR1_PR_BETA_DAC, RG_PXP_2L_CDR1_PR_BETA_DAC.dat.value);

 RG_PXP_2L_CDR0_PR_BETA_DAC.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR0_PR_BETA_DAC);
 RG_PXP_2L_CDR0_PR_BETA_DAC.hal.rg_pxp_2l_cdr0_pr_kband_div = 0x4; //10/27 INJOSC Kband need 50us issue (default 4)
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR0_PR_BETA_DAC, RG_PXP_2L_CDR0_PR_BETA_DAC.dat.value);

 RG_PXP_2L_CDR1_PR_BETA_DAC.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR1_PR_BETA_DAC);
 RG_PXP_2L_CDR1_PR_BETA_DAC.hal.rg_pxp_2l_cdr1_pr_kband_div = 0x4; //10/27 INJOSC Kband need 50us issue
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR1_PR_BETA_DAC, RG_PXP_2L_CDR1_PR_BETA_DAC.dat.value);



//*******  TX FLOW
 RG_PXP_2L_TX0_CKLDO_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TX0_CKLDO_EN);
 RG_PXP_2L_TX0_CKLDO_EN.hal.rg_pxp_2l_tx0_ckldo_en = 0x1; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TX0_CKLDO_EN, RG_PXP_2L_TX0_CKLDO_EN.dat.value);

 RG_PXP_2L_TX1_CKLDO_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TX1_CKLDO_EN);
 RG_PXP_2L_TX1_CKLDO_EN.hal.rg_pxp_2l_tx1_ckldo_en = 0x1; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TX1_CKLDO_EN, RG_PXP_2L_TX1_CKLDO_EN.dat.value);

 RG_PXP_2L_TX0_CKLDO_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TX0_CKLDO_EN);
 RG_PXP_2L_TX0_CKLDO_EN.hal.rg_pxp_2l_tx0_dmedgegen_en = 0x1; //add for 7/13 mail "7581 10.3125G XFI SETTING"
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TX0_CKLDO_EN, RG_PXP_2L_TX0_CKLDO_EN.dat.value);

 RG_PXP_2L_TX1_CKLDO_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TX1_CKLDO_EN);
 RG_PXP_2L_TX1_CKLDO_EN.hal.rg_pxp_2l_tx1_dmedgegen_en = 0x1; //add for 7/13 mail "7581 10.3125G XFI SETTING"
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TX1_CKLDO_EN, RG_PXP_2L_TX1_CKLDO_EN.dat.value);

 RG_PXP_2L_TX1_MULTLANE_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TX1_MULTLANE_EN);
 RG_PXP_2L_TX1_MULTLANE_EN.hal.rg_pxp_2l_tx1_multlane_en = multlane_en; //RG_PXP_MULTLANE_EN = 1 for 2Lane tx no skew
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TX1_MULTLANE_EN, RG_PXP_2L_TX1_MULTLANE_EN.dat.value);

 udelay(10);


//*******  PR / RX mode setting, divider, etc�
 ADD_DIG_RESERVE_27.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _ADD_DIG_RESERVE_27);
 ADD_DIG_RESERVE_27.hal.rg_dig_reserve_27 = 0x804000; //RG_PXP_AEQ_OPTION3 for gen1/2/3
 Reg_W(PCIE_PMA0, PMA_OFFSET, _ADD_DIG_RESERVE_27, ADD_DIG_RESERVE_27.dat.value);

 ADD_DIG_RESERVE_18.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _ADD_DIG_RESERVE_18);
 tmp = ADD_DIG_RESERVE_18.hal.rg_dig_reserve_18;
 tmp = (0x5<<0) | (tmp & (~(0x1F<<0)));
 ADD_DIG_RESERVE_18.hal.rg_dig_reserve_18 = tmp;//RX_SIGDET_VTH_SEL for gen1
 tmp = ADD_DIG_RESERVE_18.hal.rg_dig_reserve_18;
 tmp = (0x5<<8) | (tmp & (~(0x1F<<8)));
 ADD_DIG_RESERVE_18.hal.rg_dig_reserve_18 = tmp;//RX_SIGDET_VTH_SEL for gen2
 tmp = ADD_DIG_RESERVE_18.hal.rg_dig_reserve_18;
 tmp = (0x5<<16) | (tmp & (~(0x1F<<16)));
 ADD_DIG_RESERVE_18.hal.rg_dig_reserve_18 = tmp;//RX_SIGDET_VTH_SEL for gen3
 Reg_W(PCIE_PMA0, PMA_OFFSET, _ADD_DIG_RESERVE_18, ADD_DIG_RESERVE_18.dat.value);

 ADD_DIG_RESERVE_30.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _ADD_DIG_RESERVE_30);
 tmp = ADD_DIG_RESERVE_30.hal.rg_dig_reserve_30;
 tmp = (0x7<<8) | (tmp & (~(0x7<<8)));
 ADD_DIG_RESERVE_30.hal.rg_dig_reserve_30 = tmp;//CDR_PR_BUF_IN_SR for gen1
 tmp = ADD_DIG_RESERVE_30.hal.rg_dig_reserve_30;
 tmp = (0x7<<12) | (tmp & (~(0x7<<12)));
 ADD_DIG_RESERVE_30.hal.rg_dig_reserve_30 = tmp;//CDR_PR_BUF_IN_SR for gen2
 tmp = ADD_DIG_RESERVE_30.hal.rg_dig_reserve_30;
 tmp = (0x7<<16) | (tmp & (~(0x7<<16)));
 ADD_DIG_RESERVE_30.hal.rg_dig_reserve_30 = tmp;//CDR_PR_BUF_IN_SR for gen3
 Reg_W(PCIE_PMA0, PMA_OFFSET, _ADD_DIG_RESERVE_30, ADD_DIG_RESERVE_30.dat.value);

 RG_PXP_2L_CDR0_PR_MONCK_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR0_PR_MONCK_EN);
 RG_PXP_2L_CDR0_PR_MONCK_EN.hal.rg_pxp_2l_cdr0_pr_monck_en = 0x0; 
 RG_PXP_2L_CDR0_PR_MONCK_EN.hal.rg_pxp_2l_cdr0_pr_reserve0 = 0x2; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR0_PR_MONCK_EN, RG_PXP_2L_CDR0_PR_MONCK_EN.dat.value);

 RG_PXP_2L_RX0_OSCAL_CTLE1IOS.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX0_OSCAL_CTLE1IOS);
 RG_PXP_2L_RX0_OSCAL_CTLE1IOS.hal.rg_pxp_2l_rx0_oscal_vga1ios = 0x19; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX0_OSCAL_CTLE1IOS, RG_PXP_2L_RX0_OSCAL_CTLE1IOS.dat.value);

 RG_PXP_2L_RX0_OSCAL_VGA1VOS.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX0_OSCAL_VGA1VOS);
 RG_PXP_2L_RX0_OSCAL_VGA1VOS.hal.rg_pxp_2l_rx0_oscal_vga1vos = 0x19; 
 RG_PXP_2L_RX0_OSCAL_VGA1VOS.hal.rg_pxp_2l_rx0_oscal_vga2ios = 0x14; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX0_OSCAL_VGA1VOS, RG_PXP_2L_RX0_OSCAL_VGA1VOS.dat.value);

 ADD_DIG_RESERVE_27.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _ADD_DIG_RESERVE_27);
 ADD_DIG_RESERVE_27.hal.rg_dig_reserve_27 = 0x804000; //RG_PXP_AEQ_OPTION3 for gen1/2/3
 Reg_W(PCIE_PMA1, PMA_OFFSET, _ADD_DIG_RESERVE_27, ADD_DIG_RESERVE_27.dat.value);

 ADD_DIG_RESERVE_18.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _ADD_DIG_RESERVE_18);
 tmp = ADD_DIG_RESERVE_18.hal.rg_dig_reserve_18;
 tmp = (0x5<<0) | (tmp & (~(0x1F<<0)));
 ADD_DIG_RESERVE_18.hal.rg_dig_reserve_18 = tmp;//RX_SIGDET_VTH_SEL for gen1
 tmp = ADD_DIG_RESERVE_18.hal.rg_dig_reserve_18;
 tmp = (0x5<<8) | (tmp & (~(0x1F<<8)));
 ADD_DIG_RESERVE_18.hal.rg_dig_reserve_18 = tmp;//RX_SIGDET_VTH_SEL for gen2
 tmp = ADD_DIG_RESERVE_18.hal.rg_dig_reserve_18;
 tmp = (0x5<<16) | (tmp & (~(0x1F<<16)));
 ADD_DIG_RESERVE_18.hal.rg_dig_reserve_18 = tmp;//RX_SIGDET_VTH_SEL for gen3
 Reg_W(PCIE_PMA1, PMA_OFFSET, _ADD_DIG_RESERVE_18, ADD_DIG_RESERVE_18.dat.value);

 ADD_DIG_RESERVE_30.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _ADD_DIG_RESERVE_30);
 tmp = ADD_DIG_RESERVE_30.hal.rg_dig_reserve_30;
 tmp = (0x7<<8) | (tmp & (~(0x7<<8)));
 ADD_DIG_RESERVE_30.hal.rg_dig_reserve_30 = tmp;//CDR_PR_BUF_IN_SR for gen1
 tmp = ADD_DIG_RESERVE_30.hal.rg_dig_reserve_30;
 tmp = (0x7<<12) | (tmp & (~(0x7<<12)));
 ADD_DIG_RESERVE_30.hal.rg_dig_reserve_30 = tmp;//CDR_PR_BUF_IN_SR for gen2
 tmp = ADD_DIG_RESERVE_30.hal.rg_dig_reserve_30;
 tmp = (0x7<<16) | (tmp & (~(0x7<<16)));
 ADD_DIG_RESERVE_30.hal.rg_dig_reserve_30 = tmp;//CDR_PR_BUF_IN_SR for gen3
 Reg_W(PCIE_PMA1, PMA_OFFSET, _ADD_DIG_RESERVE_30, ADD_DIG_RESERVE_30.dat.value);

 RG_PXP_2L_CDR1_PR_MONCK_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR1_PR_MONCK_EN);
 RG_PXP_2L_CDR1_PR_MONCK_EN.hal.rg_pxp_2l_cdr1_pr_monck_en = 0x0; 
 RG_PXP_2L_CDR1_PR_MONCK_EN.hal.rg_pxp_2l_cdr1_pr_reserve0 = 0x2; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR1_PR_MONCK_EN, RG_PXP_2L_CDR1_PR_MONCK_EN.dat.value);

 RG_PXP_2L_RX1_OSCAL_VGA1IOS.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX1_OSCAL_VGA1IOS);
 RG_PXP_2L_RX1_OSCAL_VGA1IOS.hal.rg_pxp_2l_rx1_oscal_vga1ios = 0x19; 
 RG_PXP_2L_RX1_OSCAL_VGA1IOS.hal.rg_pxp_2l_rx1_oscal_vga1vos = 0x19; 
 RG_PXP_2L_RX1_OSCAL_VGA1IOS.hal.rg_pxp_2l_rx1_oscal_vga2ios = 0x14; 
 Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_RX1_OSCAL_VGA1IOS, RG_PXP_2L_RX1_OSCAL_VGA1IOS.dat.value);


 if(load_k_en == 1)
 {
	//*******  Load K flow
	 ADD_DIG_RESERVE_12.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _ADD_DIG_RESERVE_12);
	 tmp = ADD_DIG_RESERVE_12.hal.rg_dig_reserve_12;
	 tmp = (0x1<<7) | (tmp & (~(0x1<<7)));
	 ADD_DIG_RESERVE_12.hal.rg_dig_reserve_12 = tmp;//rg_force_sel_pma_pcie_rx_speed
	 tmp = ADD_DIG_RESERVE_12.hal.rg_dig_reserve_12;
	 tmp = (0x2<<4) | (tmp & (~(0x7<<4)));
	 ADD_DIG_RESERVE_12.hal.rg_dig_reserve_12 = tmp;//rg_force_pma_pcie_rx_speed ( gen3)
	 Reg_W(PCIE_PMA0, PMA_OFFSET, _ADD_DIG_RESERVE_12, ADD_DIG_RESERVE_12.dat.value);

	 ADD_DIG_RESERVE_12.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _ADD_DIG_RESERVE_12);
	 tmp = ADD_DIG_RESERVE_12.hal.rg_dig_reserve_12;
	 tmp = (0x1<<7) | (tmp & (~(0x1<<7)));
	 ADD_DIG_RESERVE_12.hal.rg_dig_reserve_12 = tmp;//rg_force_sel_pma_pcie_rx_speed
	 tmp = ADD_DIG_RESERVE_12.hal.rg_dig_reserve_12;
	 tmp = (0x2<<4) | (tmp & (~(0x7<<4)));
	 ADD_DIG_RESERVE_12.hal.rg_dig_reserve_12 = tmp;//rg_force_pma_pcie_rx_speed ( gen3)
	 Reg_W(PCIE_PMA1, PMA_OFFSET, _ADD_DIG_RESERVE_12, ADD_DIG_RESERVE_12.dat.value);


	 Rx_PR_Fw_Pre_Cal(3,0); 
	 Rx_PR_Fw_Pre_Cal(3,1);
	 
	 ADD_DIG_RESERVE_12.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _ADD_DIG_RESERVE_12);
	 tmp = ADD_DIG_RESERVE_12.hal.rg_dig_reserve_12;
	 tmp = (0x0<<7) | (tmp & (~(0x1<<7)));
	 ADD_DIG_RESERVE_12.hal.rg_dig_reserve_12 = tmp;//rg_force_sel_pma_pcie_rx_speed
	 tmp = ADD_DIG_RESERVE_12.hal.rg_dig_reserve_12;
	 tmp = (0x0<<4) | (tmp & (~(0x7<<4)));
	 ADD_DIG_RESERVE_12.hal.rg_dig_reserve_12 = tmp;//rg_force_pma_pcie_rx_speed
	 Reg_W(PCIE_PMA0, PMA_OFFSET, _ADD_DIG_RESERVE_12, ADD_DIG_RESERVE_12.dat.value);

	 ADD_DIG_RESERVE_12.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _ADD_DIG_RESERVE_12);
	 tmp = ADD_DIG_RESERVE_12.hal.rg_dig_reserve_12;
	 tmp = (0x0<<7) | (tmp & (~(0x1<<7)));
	 ADD_DIG_RESERVE_12.hal.rg_dig_reserve_12 = tmp;//rg_force_sel_pma_pcie_rx_speed
	 tmp = ADD_DIG_RESERVE_12.hal.rg_dig_reserve_12;
	 tmp = (0x0<<4) | (tmp & (~(0x7<<4)));
	 ADD_DIG_RESERVE_12.hal.rg_dig_reserve_12 = tmp;//rg_force_pma_pcie_rx_speed
	 Reg_W(PCIE_PMA1, PMA_OFFSET, _ADD_DIG_RESERVE_12, ADD_DIG_RESERVE_12.dat.value);

	 udelay(100);

	 Rx_PR_Fw_Pre_Cal(2,0); 
	 Rx_PR_Fw_Pre_Cal(2,1);
 }




#if 0 //for eye scan, remove these if not used for saving power
 rg_type_t(HAL_rg_force_da_pxp_cdr_pr_pieye_pwdb) rg_force_da_pxp_cdr_pr_pieye_pwdb;
 rg_type_t(HAL_rg_force_da_pxp_cdr_pr_fll_cor) rg_force_da_pxp_cdr_pr_fll_cor;


 rg_force_da_pxp_cdr_pr_fll_cor.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_fll_cor);
 rg_force_da_pxp_cdr_pr_fll_cor.hal.rg_force_da_pxp_rx_dac_eye = 0x0; //for eye scan, remove these if not used for saving power
 Reg_W(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_fll_cor, rg_force_da_pxp_cdr_pr_fll_cor.dat.value);

 rg_force_da_pxp_cdr_pr_fll_cor.hal.rg_force_sel_da_pxp_rx_dac_eye = 0x1; //for eye scan, remove these if not used for saving power
 Reg_W(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_fll_cor, rg_force_da_pxp_cdr_pr_fll_cor.dat.value);

 rg_force_da_pxp_cdr_pr_pieye_pwdb.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_pieye_pwdb);
 rg_force_da_pxp_cdr_pr_pieye_pwdb.hal.rg_force_da_pxp_cdr_pr_pieye_pwdb = 0x1; //for eye scan, remove these if not used for saving power
 Reg_W(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_pieye_pwdb, rg_force_da_pxp_cdr_pr_pieye_pwdb.dat.value);

 rg_force_da_pxp_cdr_pr_pieye_pwdb.hal.rg_force_sel_da_pxp_cdr_pr_pieye_pwdb = 0x1; //for eye scan, remove these if not used for saving power
 Reg_W(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_pieye_pwdb, rg_force_da_pxp_cdr_pr_pieye_pwdb.dat.value);

 rg_force_da_pxp_cdr_pr_fll_cor.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_fll_cor);
 rg_force_da_pxp_cdr_pr_fll_cor.hal.rg_force_da_pxp_rx_dac_eye = 0x0; //for eye scan, remove these if not used for saving power
 Reg_W(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_fll_cor, rg_force_da_pxp_cdr_pr_fll_cor.dat.value);

 rg_force_da_pxp_cdr_pr_fll_cor.hal.rg_force_sel_da_pxp_rx_dac_eye = 0x1; //for eye scan, remove these if not used for saving power
 Reg_W(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_fll_cor, rg_force_da_pxp_cdr_pr_fll_cor.dat.value);

 rg_force_da_pxp_cdr_pr_pieye_pwdb.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_pieye_pwdb);
 rg_force_da_pxp_cdr_pr_pieye_pwdb.hal.rg_force_da_pxp_cdr_pr_pieye_pwdb = 0x1; //for eye scan, remove these if not used for saving power
 Reg_W(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_pieye_pwdb, rg_force_da_pxp_cdr_pr_pieye_pwdb.dat.value);

 rg_force_da_pxp_cdr_pr_pieye_pwdb.hal.rg_force_sel_da_pxp_cdr_pr_pieye_pwdb = 0x1; //for eye scan, remove these if not used for saving power
 Reg_W(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_pieye_pwdb, rg_force_da_pxp_cdr_pr_pieye_pwdb.dat.value);
#endif

 

 SS_DA_XPON_PWDB_0.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _SS_DA_XPON_PWDB_0);
 SS_DA_XPON_PWDB_0.hal.rg_da_xpon_cdr_pr_pwdb = 0x0; 
 Reg_W(PCIE_PMA0, PMA_OFFSET, _SS_DA_XPON_PWDB_0, SS_DA_XPON_PWDB_0.dat.value);

 SS_DA_XPON_PWDB_0.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _SS_DA_XPON_PWDB_0);
 SS_DA_XPON_PWDB_0.hal.rg_da_xpon_cdr_pr_pwdb = 0x0; 
 Reg_W(PCIE_PMA1, PMA_OFFSET, _SS_DA_XPON_PWDB_0, SS_DA_XPON_PWDB_0.dat.value);

 udelay(10);
 SS_DA_XPON_PWDB_0.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _SS_DA_XPON_PWDB_0);
 SS_DA_XPON_PWDB_0.hal.rg_da_xpon_cdr_pr_pwdb = 0x1; 
 Reg_W(PCIE_PMA0, PMA_OFFSET, _SS_DA_XPON_PWDB_0, SS_DA_XPON_PWDB_0.dat.value);

 SS_DA_XPON_PWDB_0.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _SS_DA_XPON_PWDB_0);
 SS_DA_XPON_PWDB_0.hal.rg_da_xpon_cdr_pr_pwdb = 0x1; 
 Reg_W(PCIE_PMA1, PMA_OFFSET, _SS_DA_XPON_PWDB_0, SS_DA_XPON_PWDB_0.dat.value);

 udelay(100);



}


void PCIe_G3_L0_Enable(void)
{
	 rg_type_t(HAL_SW_RST_SET) SW_RST_SET;
	 rg_type_t(HAL_rg_force_da_pxp_tx_ck_en) rg_force_da_pxp_tx_ck_en;
	 rg_type_t(HAL_SS_DA_XPON_PWDB_0) SS_DA_XPON_PWDB_0;


	 SW_RST_SET.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _SW_RST_SET);
	 SW_RST_SET.hal.rg_sw_tx_rst_n = 0x1; 
	 SW_RST_SET.hal.rg_sw_rx_rst_n = 0x1; 
	 SW_RST_SET.hal.rg_sw_rx_fifo_rst_n = 0x1; 
	 SW_RST_SET.hal.rg_sw_rx_rst_n = 0x1; 
	 SW_RST_SET.hal.rg_sw_tx_rst_n = 0x1; 
	 SW_RST_SET.hal.rg_sw_pma_rst_n = 0x1; 
	 SW_RST_SET.hal.rg_sw_allpcs_rst_n = 0x1; 
	 SW_RST_SET.hal.rg_sw_ref_rst_n = 0x1; 
	 SW_RST_SET.hal.rg_sw_tx_fifo_rst_n = 0x1; 
	 SW_RST_SET.hal.rg_sw_xfi_txpcs_rst_n = 0x1; 
	 SW_RST_SET.hal.rg_sw_xfi_rxpcs_rst_n = 0x1; 
	 SW_RST_SET.hal.rg_sw_xfi_rxpcs_bist_rst_n = 0x1; 
	 SW_RST_SET.hal.rg_sw_hsg_txpcs_rst_n = 0x1; 
	 SW_RST_SET.hal.rg_sw_hsg_rxpcs_rst_n = 0x1; 
	 Reg_W(PCIE_PMA0, PMA_OFFSET, _SW_RST_SET, SW_RST_SET.dat.value); //excel 9, write addr: 0x1FA5B460 = 0x179

	 rg_force_da_pxp_tx_ck_en.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_tx_ck_en);
	 rg_force_da_pxp_tx_ck_en.hal.rg_force_sel_da_pxp_tx_ck_en = 0x1; //mux sel
	 rg_force_da_pxp_tx_ck_en.hal.rg_force_da_pxp_tx_ck_en = 0x1; //SER CK off, switch to LSDATA path
	 Reg_W(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_tx_ck_en, rg_force_da_pxp_tx_ck_en.dat.value); //excel 11, write addr: 0x1FA5B878 = 0x100

	 SS_DA_XPON_PWDB_0.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _SS_DA_XPON_PWDB_0);
	 SS_DA_XPON_PWDB_0.hal.rg_da_xpon_rx_fe_pwdb = 0x1; //cdr clk off
	 SS_DA_XPON_PWDB_0.hal.rg_da_xpon_cdr_pr_pwdb = 0x1; //cdr clk off
	 SS_DA_XPON_PWDB_0.hal.rg_da_xpon_cdr_pr_pieye_pwdb = 0x1; //cdr clk off
	 SS_DA_XPON_PWDB_0.hal.rg_da_xpon_cdr_pd_pwdb = 0x1; //cdr clk off
	 Reg_W(PCIE_PMA0, PMA_OFFSET, _SS_DA_XPON_PWDB_0, SS_DA_XPON_PWDB_0.dat.value); //excel 15, write addr: 0x1FA5B34C = 0x0


}

void PCIe_G3_L1_Enable(void)
{
	 rg_type_t(HAL_SW_RST_SET) SW_RST_SET;
	 rg_type_t(HAL_rg_force_da_pxp_tx_ck_en) rg_force_da_pxp_tx_ck_en;
	 rg_type_t(HAL_SS_DA_XPON_PWDB_0) SS_DA_XPON_PWDB_0;


	 SW_RST_SET.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _SW_RST_SET);
	 SW_RST_SET.hal.rg_sw_tx_rst_n = 0x1; 
	 SW_RST_SET.hal.rg_sw_rx_rst_n = 0x1; 
	 SW_RST_SET.hal.rg_sw_rx_fifo_rst_n = 0x1; 
	 SW_RST_SET.hal.rg_sw_rx_rst_n = 0x1; 
	 SW_RST_SET.hal.rg_sw_tx_rst_n = 0x1; 
	 SW_RST_SET.hal.rg_sw_pma_rst_n = 0x1; 
	 SW_RST_SET.hal.rg_sw_allpcs_rst_n = 0x1; 
	 SW_RST_SET.hal.rg_sw_ref_rst_n = 0x1; 
	 SW_RST_SET.hal.rg_sw_tx_fifo_rst_n = 0x1; 
	 SW_RST_SET.hal.rg_sw_xfi_txpcs_rst_n = 0x1; 
	 SW_RST_SET.hal.rg_sw_xfi_rxpcs_rst_n = 0x1; 
	 SW_RST_SET.hal.rg_sw_xfi_rxpcs_bist_rst_n = 0x1; 
	 SW_RST_SET.hal.rg_sw_hsg_txpcs_rst_n = 0x1; 
	 SW_RST_SET.hal.rg_sw_hsg_rxpcs_rst_n = 0x1; 
	 Reg_W(PCIE_PMA1, PMA_OFFSET, _SW_RST_SET, SW_RST_SET.dat.value); //excel 9, write addr: 0x1FA5C460 = 0x179

	 rg_force_da_pxp_tx_ck_en.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_tx_ck_en);
	 rg_force_da_pxp_tx_ck_en.hal.rg_force_sel_da_pxp_tx_ck_en = 0x1; //mux sel
	 rg_force_da_pxp_tx_ck_en.hal.rg_force_da_pxp_tx_ck_en = 0x1; //SER CK off, switch to LSDATA path
	 Reg_W(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_tx_ck_en, rg_force_da_pxp_tx_ck_en.dat.value); //excel 11, write addr: 0x1FA5C878 = 0x100

	 SS_DA_XPON_PWDB_0.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _SS_DA_XPON_PWDB_0);
	 SS_DA_XPON_PWDB_0.hal.rg_da_xpon_rx_fe_pwdb = 0x1; //cdr clk off
	 SS_DA_XPON_PWDB_0.hal.rg_da_xpon_cdr_pr_pwdb = 0x1; //cdr clk off
	 SS_DA_XPON_PWDB_0.hal.rg_da_xpon_cdr_pr_pieye_pwdb = 0x1; //cdr clk off
	 SS_DA_XPON_PWDB_0.hal.rg_da_xpon_cdr_pd_pwdb = 0x1; //cdr clk off
	 Reg_W(PCIE_PMA1, PMA_OFFSET, _SS_DA_XPON_PWDB_0, SS_DA_XPON_PWDB_0.dat.value); //excel 15, write addr: 0x1FA5C34C = 0x0

}


void PCIe_G3_L0_Disable(void)
{
	 rg_type_t(HAL_SW_RST_SET) SW_RST_SET;

	/*
	 rg_type_t(HAL_rg_force_da_pxp_tx_ck_en) rg_force_da_pxp_tx_ck_en;
	 rg_type_t(HAL_SS_DA_XPON_PWDB_0) SS_DA_XPON_PWDB_0;
	 rg_force_da_pxp_tx_ck_en.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_tx_ck_en);
	 rg_force_da_pxp_tx_ck_en.hal.rg_force_sel_da_pxp_tx_ck_en = 0x1; //mux sel
	 rg_force_da_pxp_tx_ck_en.hal.rg_force_da_pxp_tx_ck_en = 0x0; //SER CK off, switch to LSDATA path
	 Reg_W(PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_tx_ck_en, rg_force_da_pxp_tx_ck_en.dat.value); //excel 11, write addr: 0x1FA5B878 = 0x100

	 SS_DA_XPON_PWDB_0.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _SS_DA_XPON_PWDB_0);
	 SS_DA_XPON_PWDB_0.hal.rg_da_xpon_rx_fe_pwdb = 0x0; //cdr clk off
	 SS_DA_XPON_PWDB_0.hal.rg_da_xpon_cdr_pr_pwdb = 0x0; //cdr clk off
	 SS_DA_XPON_PWDB_0.hal.rg_da_xpon_cdr_pr_pieye_pwdb = 0x0; //cdr clk off
	 SS_DA_XPON_PWDB_0.hal.rg_da_xpon_cdr_pd_pwdb = 0x0; //cdr clk off
	 Reg_W(PCIE_PMA0, PMA_OFFSET, _SS_DA_XPON_PWDB_0, SS_DA_XPON_PWDB_0.dat.value); //excel 15, write addr: 0x1FA5B34C = 0x0
	 */

	 SW_RST_SET.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _SW_RST_SET);
	 SW_RST_SET.hal.rg_sw_tx_rst_n = 0x0; 
	 SW_RST_SET.hal.rg_sw_rx_rst_n = 0x0; 
	 SW_RST_SET.hal.rg_sw_rx_fifo_rst_n = 0x0; 
	 SW_RST_SET.hal.rg_sw_rx_rst_n = 0x0; 
	 SW_RST_SET.hal.rg_sw_tx_rst_n = 0x0; 
	 SW_RST_SET.hal.rg_sw_pma_rst_n = 0x0; 
	 SW_RST_SET.hal.rg_sw_allpcs_rst_n = 0x0; 
	 SW_RST_SET.hal.rg_sw_ref_rst_n = 0x0; 
	 SW_RST_SET.hal.rg_sw_tx_fifo_rst_n = 0x0; 
	 SW_RST_SET.hal.rg_sw_xfi_txpcs_rst_n = 0x0; 
	 SW_RST_SET.hal.rg_sw_xfi_rxpcs_rst_n = 0x0; 
	 SW_RST_SET.hal.rg_sw_xfi_rxpcs_bist_rst_n = 0x0; 
	 SW_RST_SET.hal.rg_sw_hsg_txpcs_rst_n = 0x0; 
	 SW_RST_SET.hal.rg_sw_hsg_rxpcs_rst_n = 0x0; 
	 Reg_W(PCIE_PMA0, PMA_OFFSET, _SW_RST_SET, SW_RST_SET.dat.value); //excel 21, write addr: 0x1FA5B460 = 0x0
	 

}

void PCIe_G3_L1_Disable(void)
{
	 rg_type_t(HAL_SW_RST_SET) SW_RST_SET;

	 /*
	 rg_type_t(HAL_rg_force_da_pxp_tx_ck_en) rg_force_da_pxp_tx_ck_en;
	 rg_type_t(HAL_SS_DA_XPON_PWDB_0) SS_DA_XPON_PWDB_0;

	 rg_force_da_pxp_tx_ck_en.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_tx_ck_en);
	 rg_force_da_pxp_tx_ck_en.hal.rg_force_sel_da_pxp_tx_ck_en = 0x1; //mux sel
	 rg_force_da_pxp_tx_ck_en.hal.rg_force_da_pxp_tx_ck_en = 0x0; //SER CK off, switch to LSDATA path
	 Reg_W(PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_tx_ck_en, rg_force_da_pxp_tx_ck_en.dat.value); //excel 11, write addr: 0x1FA5C878 = 0x100

	 SS_DA_XPON_PWDB_0.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _SS_DA_XPON_PWDB_0);
	 SS_DA_XPON_PWDB_0.hal.rg_da_xpon_rx_fe_pwdb = 0x0; //cdr clk off
	 SS_DA_XPON_PWDB_0.hal.rg_da_xpon_cdr_pr_pwdb = 0x0; //cdr clk off
	 SS_DA_XPON_PWDB_0.hal.rg_da_xpon_cdr_pr_pieye_pwdb = 0x0; //cdr clk off
	 SS_DA_XPON_PWDB_0.hal.rg_da_xpon_cdr_pd_pwdb = 0x0; //cdr clk off
	 Reg_W(PCIE_PMA1, PMA_OFFSET, _SS_DA_XPON_PWDB_0, SS_DA_XPON_PWDB_0.dat.value); //excel 15, write addr: 0x1FA5C34C = 0x0
	 */

	 SW_RST_SET.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _SW_RST_SET);
	 SW_RST_SET.hal.rg_sw_tx_rst_n = 0x0; 
	 SW_RST_SET.hal.rg_sw_rx_rst_n = 0x0; 
	 SW_RST_SET.hal.rg_sw_rx_fifo_rst_n = 0x0; 
	 SW_RST_SET.hal.rg_sw_rx_rst_n = 0x0; 
	 SW_RST_SET.hal.rg_sw_tx_rst_n = 0x0; 
	 SW_RST_SET.hal.rg_sw_pma_rst_n = 0x0; 
	 SW_RST_SET.hal.rg_sw_allpcs_rst_n = 0x0; 
	 SW_RST_SET.hal.rg_sw_ref_rst_n = 0x0; 
	 SW_RST_SET.hal.rg_sw_tx_fifo_rst_n = 0x0; 
	 SW_RST_SET.hal.rg_sw_xfi_txpcs_rst_n = 0x0; 
	 SW_RST_SET.hal.rg_sw_xfi_rxpcs_rst_n = 0x0; 
	 SW_RST_SET.hal.rg_sw_xfi_rxpcs_bist_rst_n = 0x0; 
	 SW_RST_SET.hal.rg_sw_hsg_txpcs_rst_n = 0x0; 
	 SW_RST_SET.hal.rg_sw_hsg_rxpcs_rst_n = 0x0; 
	 Reg_W(PCIE_PMA1, PMA_OFFSET, _SW_RST_SET, SW_RST_SET.dat.value); //excel 21, write addr: 0x1FA5C460 = 0x0
	 

}

void PCIe_G3_L0_init(void)
{
	PCIe_G3_L0L1_init(PR_K_DEFAULT_FLOW);
	PCIe_G3_L1_Disable();
}

void PCIe_G3_L1_init(void)
{
	PCIe_G3_L0L1_init(PR_K_DEFAULT_FLOW);
	PCIe_G3_L0_Disable();
}


uint PCIe_eyescan_moveX(uint EYE_X_HW, uint EYE_Y_HW, int Ovr_sel, char lane){

	u32 lane_offset_dig;
	rg_type_t(HAL_rg_force_da_pxp_tx_rate_ctrl) rg_force_da_pxp_tx_rate_ctrl;
	
	int i;

	if (lane == 1)
		lane_offset_dig = 0x1000;
	else
		lane_offset_dig = 0;
	
          
       for (i = 0; i < Ovr_sel; i++)
       {
	// X index
	       
	//IO_SPHYA_REG_BITS(EN7581_XPON_PMA_rg_force_da_pxp_tx_rate_ctrl, 22, 16, EYE_X_HW); //rg_force_da_pxp_cdr_pr_pieye
	//IO_SPHYA_REG_BITS(EN7581_XPON_PMA_rg_force_da_pxp_tx_rate_ctrl, 24, 24, 0x1);      //rg_force_sel_da_pxp_cdr_pr_pieye
	 rg_force_da_pxp_tx_rate_ctrl.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_tx_rate_ctrl);
	 rg_force_da_pxp_tx_rate_ctrl.hal.rg_force_da_pxp_cdr_pr_pieye = EYE_X_HW; //X
	 rg_force_da_pxp_tx_rate_ctrl.hal.rg_force_sel_da_pxp_cdr_pr_pieye = 0x1; 
	 Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_tx_rate_ctrl, rg_force_da_pxp_tx_rate_ctrl.dat.value); //excel 72, write addr: 0x1FA5B784 = 0x1000000


	EYE_X_HW++;
       }
    return EYE_X_HW;

}



int PCIe_eyescan_countPoint(uint EYE_X_FW, uint EYE_Y_FW, char lane){
	uint eyecnt = 0;																			
	int eyecnt_rdy = 0;	
	u32 lane_offset_dig;




	rg_type_t(HAL_rg_force_da_pxp_cdr_pr_fll_cor) rg_force_da_pxp_cdr_pr_fll_cor;
	rg_type_t(HAL_RX_EYE_TOP_EYECNT_CTRL_1) RX_EYE_TOP_EYECNT_CTRL_1;
	rg_type_t(HAL_RX_DISB_MODE_7) RX_DISB_MODE_7;
	rg_type_t(HAL_RX_FORCE_MODE_8) RX_FORCE_MODE_8;
	rg_type_t(HAL_rg_force_da_pxp_tx_rate_ctrl) rg_force_da_pxp_tx_rate_ctrl;
	rg_type_t(HAL_RX_DEBUG_0) RX_DEBUG_0;
	rg_type_t(HAL_RX_TORGS_DEBUG_4) RX_TORGS_DEBUG_4;
	rg_type_t(HAL_RX_TORGS_DEBUG_7) RX_TORGS_DEBUG_7;


	if (lane == 1)
		lane_offset_dig = 0x1000;
	else
		lane_offset_dig = 0;

	 
	 rg_force_da_pxp_tx_rate_ctrl.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_tx_rate_ctrl);
	 rg_force_da_pxp_tx_rate_ctrl.hal.rg_force_da_pxp_cdr_pr_pieye = EYE_X_FW; //X
	 rg_force_da_pxp_tx_rate_ctrl.hal.rg_force_sel_da_pxp_cdr_pr_pieye = 0x1; 
	 Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_tx_rate_ctrl, rg_force_da_pxp_tx_rate_ctrl.dat.value); //excel 72, write addr: 0x1FA5B784 = 0x1000000

	 
	 // Y index
	 rg_force_da_pxp_cdr_pr_fll_cor.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_fll_cor);
	 rg_force_da_pxp_cdr_pr_fll_cor.hal.rg_force_da_pxp_rx_dac_eye = EYE_Y_FW; //Y
	 rg_force_da_pxp_cdr_pr_fll_cor.hal.rg_force_sel_da_pxp_rx_dac_eye = 0x1; 
	 Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_fll_cor, rg_force_da_pxp_cdr_pr_fll_cor.dat.value); //excel 74, write addr: 0x1FA5B790 = 0x1000000



	 // EYE cnt enable 

	 RX_EYE_TOP_EYECNT_CTRL_1.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_EYE_TOP_EYECNT_CTRL_1);
	 RX_EYE_TOP_EYECNT_CTRL_1.hal.rg_disb_eyedur_init_b = 0x0; 
	 RX_EYE_TOP_EYECNT_CTRL_1.hal.rg_force_eyedur_init_b = 0x0; 
	 Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_EYE_TOP_EYECNT_CTRL_1, RX_EYE_TOP_EYECNT_CTRL_1.dat.value); //excel 76, write addr: 0x1FA5B084 = 0x1

			 

	 RX_DISB_MODE_7.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_DISB_MODE_7);
	 RX_DISB_MODE_7.hal.rg_disb_eyecnt_rx_rst_b = 0x0; 
	 Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_DISB_MODE_7, RX_DISB_MODE_7.dat.value); //excel 77, write addr: 0x1FA5B338 = 0x10101

	 RX_FORCE_MODE_8.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_FORCE_MODE_8);
	 RX_FORCE_MODE_8.hal.rg_force_eyecnt_rx_rst_b = 0x0; 
	 Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_FORCE_MODE_8, RX_FORCE_MODE_8.dat.value); //excel 78, write addr: 0x1FA5B32C = 0x0
	 

	 RX_EYE_TOP_EYECNT_CTRL_1.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_EYE_TOP_EYECNT_CTRL_1);
	 RX_EYE_TOP_EYECNT_CTRL_1.hal.rg_disb_eyedur_en = 0x0; 
	 RX_EYE_TOP_EYECNT_CTRL_1.hal.rg_force_eyedur_en = 0x0; 
	 Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_EYE_TOP_EYECNT_CTRL_1, RX_EYE_TOP_EYECNT_CTRL_1.dat.value); //excel 80, write addr: 0x1FA5B084 = 0x0
	 

	 RX_FORCE_MODE_8.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_FORCE_MODE_8);
	 RX_FORCE_MODE_8.hal.rg_force_eyecnt_rx_rst_b = 0x1; 
	 Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_FORCE_MODE_8, RX_FORCE_MODE_8.dat.value); //excel 81, write addr: 0x1FA5B32C = 0x1000000

	 RX_EYE_TOP_EYECNT_CTRL_1.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_EYE_TOP_EYECNT_CTRL_1);
	 RX_EYE_TOP_EYECNT_CTRL_1.hal.rg_force_eyedur_init_b = 0x1; 
	 RX_EYE_TOP_EYECNT_CTRL_1.hal.rg_force_eyedur_en = 0x1; 
	 Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_EYE_TOP_EYECNT_CTRL_1, RX_EYE_TOP_EYECNT_CTRL_1.dat.value); //excel 83, write addr: 0x1FA5B084 = 0x1010000
	 
	
	 mdelay(1);


	 RX_DEBUG_0.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_DEBUG_0);
	 RX_DEBUG_0.hal.rg_ro_toggle = 0x0; 
	 Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_DEBUG_0, RX_DEBUG_0.dat.value); //excel 84, write addr: 0x1FA5B20C = 0xFFFF

	 udelay(100);
	 RX_DEBUG_0.hal.rg_ro_toggle = 0x1; 
	 Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_DEBUG_0, RX_DEBUG_0.dat.value); //excel 86, write addr: 0x1FA5B20C = 0x100FFFF


	 RX_TORGS_DEBUG_4.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_TORGS_DEBUG_4);
	 eyecnt_rdy = RX_TORGS_DEBUG_4.hal.eyecnt_rdy; //excel 87 read addr: 0x1FA5B244 [24:24]

	 
	 if (eyecnt_rdy == 1)   // if eyecnt_rdy
	 {
		 //eyecnt = IO_GPHYA_REG_BITS(EN7581_XPON_PMA_RX_TORGS_DEBUG_7, 19, 0);  //eyecnt
		 RX_TORGS_DEBUG_7.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_TORGS_DEBUG_7);
		 eyecnt= RX_TORGS_DEBUG_7.hal.eyecnt; 
	 }
	 else
	 {
	     printk("eyecnt_rdy = %d \n", eyecnt_rdy);
	 }
																			
	return eyecnt;																	
}


void PCIe_eye_setting(char lane)
{
	u32 lane_offset_dig;
   

	rg_type_t(HAL_RG_PXP_2L_CDR0_LPF_RATIO) RG_PXP_2L_CDR0_LPF_RATIO;
	rg_type_t(HAL_RG_PXP_2L_CDR1_LPF_RATIO) RG_PXP_2L_CDR1_LPF_RATIO;
	rg_type_t(HAL_RX_EYE_TOP_EYECNT_CTRL_0) RX_EYE_TOP_EYECNT_CTRL_0;
	rg_type_t(HAL_RX_EYE_TOP_EYEINDEX_CTRL_0) RX_EYE_TOP_EYEINDEX_CTRL_0;
	rg_type_t(HAL_RX_EYE_TOP_EYECNT_CTRL_2) RX_EYE_TOP_EYECNT_CTRL_2;
	rg_type_t(HAL_RX_EYE_TOP_EYEINDEX_CTRL_1) RX_EYE_TOP_EYEINDEX_CTRL_1;
	rg_type_t(HAL_RX_EYE_TOP_EYEINDEX_CTRL_2) RX_EYE_TOP_EYEINDEX_CTRL_2;
	rg_type_t(HAL_RX_EYE_TOP_EYEINDEX_CTRL_3) RX_EYE_TOP_EYEINDEX_CTRL_3;
	rg_type_t(HAL_RX_EYE_TOP_EYEOPENING_CTRL_0) RX_EYE_TOP_EYEOPENING_CTRL_0;
	rg_type_t(HAL_RX_EYE_TOP_EYEOPENING_CTRL_1) RX_EYE_TOP_EYEOPENING_CTRL_1;
	rg_type_t(HAL_PHY_EQ_CTRL_1) PHY_EQ_CTRL_1;
	rg_type_t(HAL_PHY_EQ_CTRL_2) PHY_EQ_CTRL_2;



	if (lane == 1)
	{
		lane_offset_dig = 0x1000;
		RG_PXP_2L_CDR1_LPF_RATIO.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR1_LPF_RATIO);
		RG_PXP_2L_CDR1_LPF_RATIO.hal.rg_pxp_2l_cdr1_lpf_ratio = 0x0; 
		Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR1_LPF_RATIO, RG_PXP_2L_CDR1_LPF_RATIO.dat.value); //excel 12, write addr: 0x1FA5A110 = 0x2000000
		
	}
	else
	{
		lane_offset_dig = 0;
		RG_PXP_2L_CDR0_LPF_RATIO.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR0_LPF_RATIO);
		RG_PXP_2L_CDR0_LPF_RATIO.hal.rg_pxp_2l_cdr0_lpf_ratio = 0x0; 
		Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR0_LPF_RATIO, RG_PXP_2L_CDR0_LPF_RATIO.dat.value); //excel 12, write addr: 0x1FA5A110 = 0x2000000

	}



	RX_EYE_TOP_EYECNT_CTRL_0.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_EYE_TOP_EYECNT_CTRL_0);
	RX_EYE_TOP_EYECNT_CTRL_0.hal.rg_eye_mask = 0xFF; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_EYE_TOP_EYECNT_CTRL_0, RX_EYE_TOP_EYECNT_CTRL_0.dat.value); //excel 13, write addr: 0x1FA5B080 = 0xFF000000

	RX_EYE_TOP_EYEINDEX_CTRL_0.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_EYE_TOP_EYEINDEX_CTRL_0);
	RX_EYE_TOP_EYEINDEX_CTRL_0.hal.rg_x_min = 0x1C0; 
	RX_EYE_TOP_EYEINDEX_CTRL_0.hal.rg_x_max = 0x234; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_EYE_TOP_EYEINDEX_CTRL_0, RX_EYE_TOP_EYEINDEX_CTRL_0.dat.value); //excel 15, write addr: 0x1FA5B068 = 0x24001C0
	

	RX_EYE_TOP_EYECNT_CTRL_0.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_EYE_TOP_EYECNT_CTRL_0);
	RX_EYE_TOP_EYECNT_CTRL_0.hal.rg_cntlen = 0xF8; 
	RX_EYE_TOP_EYECNT_CTRL_0.hal.rg_cntforever = 0x0; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_EYE_TOP_EYECNT_CTRL_0, RX_EYE_TOP_EYECNT_CTRL_0.dat.value); //excel 17, write addr: 0x1FA5B080 = 0xFF0000F8

	RX_EYE_TOP_EYECNT_CTRL_2.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_EYE_TOP_EYECNT_CTRL_2);
	RX_EYE_TOP_EYECNT_CTRL_2.hal.rg_data_shift = 0x0; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_EYE_TOP_EYECNT_CTRL_2, RX_EYE_TOP_EYECNT_CTRL_2.dat.value); //excel 18, write addr: 0x1FA5B088 = 0x0


	RX_EYE_TOP_EYEINDEX_CTRL_1.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_EYE_TOP_EYEINDEX_CTRL_1);
	RX_EYE_TOP_EYEINDEX_CTRL_1.hal.rg_index_mode = 0x0; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_EYE_TOP_EYEINDEX_CTRL_1, RX_EYE_TOP_EYEINDEX_CTRL_1.dat.value); //excel 19, write addr: 0x1FA5B06C = 0x1967

	RX_EYE_TOP_EYEINDEX_CTRL_2.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_EYE_TOP_EYEINDEX_CTRL_2);
	RX_EYE_TOP_EYEINDEX_CTRL_2.hal.rg_eyedur = 0xFFF8; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_EYE_TOP_EYEINDEX_CTRL_2, RX_EYE_TOP_EYEINDEX_CTRL_2.dat.value); //excel 20, write addr: 0x1FA5B070 = 0xFFF8

		 
	RX_EYE_TOP_EYEINDEX_CTRL_3.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_EYE_TOP_EYEINDEX_CTRL_3);
	RX_EYE_TOP_EYEINDEX_CTRL_3.hal.rg_eye_nextpts_sel = 0x0; 
	RX_EYE_TOP_EYEINDEX_CTRL_3.hal.rg_eye_nextpts_toggle = 0x0; 
	RX_EYE_TOP_EYEINDEX_CTRL_3.hal.rg_eye_nextpts = 0x1; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_EYE_TOP_EYEINDEX_CTRL_3, RX_EYE_TOP_EYEINDEX_CTRL_3.dat.value); //excel 23, write addr: 0x1FA5B074 = 0x10000


	RX_EYE_TOP_EYEOPENING_CTRL_0.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_EYE_TOP_EYEOPENING_CTRL_0);
	RX_EYE_TOP_EYEOPENING_CTRL_0.hal.rg_eyecnt_hth = 0x4; 
	RX_EYE_TOP_EYEOPENING_CTRL_0.hal.rg_eyecnt_vth = 0x4; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_EYE_TOP_EYEOPENING_CTRL_0, RX_EYE_TOP_EYEOPENING_CTRL_0.dat.value); //excel 25, write addr: 0x1FA5B078 = 0x404

	RX_EYE_TOP_EYEOPENING_CTRL_1.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_EYE_TOP_EYEOPENING_CTRL_1);
	RX_EYE_TOP_EYEOPENING_CTRL_1.hal.rg_eo_hth = 0x4; 
	RX_EYE_TOP_EYEOPENING_CTRL_1.hal.rg_eo_vth = 0x4; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_EYE_TOP_EYEOPENING_CTRL_1, RX_EYE_TOP_EYEOPENING_CTRL_1.dat.value); //excel 27, write addr: 0x1FA5B07C = 0x40004


	PHY_EQ_CTRL_1.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _PHY_EQ_CTRL_1);
	PHY_EQ_CTRL_1.hal.rg_heo_emphasis = 0x0; 
	PHY_EQ_CTRL_1.hal.rg_a_lgain = 0x0; 
	PHY_EQ_CTRL_1.hal.rg_a_mgain = 0x0; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _PHY_EQ_CTRL_1, PHY_EQ_CTRL_1.dat.value); //excel 29, write addr: 0x1FA5B11C = 0x0

	PHY_EQ_CTRL_2.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _PHY_EQ_CTRL_2);
	PHY_EQ_CTRL_2.hal.rg_a_sel = 0x0; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _PHY_EQ_CTRL_2, PHY_EQ_CTRL_2.dat.value); //excel 30, write addr: 0x1FA5B120 = 0x500

	PHY_EQ_CTRL_1.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _PHY_EQ_CTRL_1);
	PHY_EQ_CTRL_1.hal.rg_b_zero_sel = 0x0; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _PHY_EQ_CTRL_1, PHY_EQ_CTRL_1.dat.value); //excel 31, write addr: 0x1FA5B11C = 0x0
}


void PCIe_eye_Cal(char lane)
{
	u32 lane_offset_dig;
	

	rg_type_t(HAL_rg_force_da_pxp_cdr_pr_fll_cor) rg_force_da_pxp_cdr_pr_fll_cor;
	rg_type_t(HAL_rg_force_da_pxp_tx_rate_ctrl) rg_force_da_pxp_tx_rate_ctrl;
	rg_type_t(HAL_PHY_EQ_CTRL_0) PHY_EQ_CTRL_0;
	rg_type_t(HAL_SS_RX_PI_CAL) SS_RX_PI_CAL;
	rg_type_t(HAL_RX_RESET_0) RX_RESET_0;
	rg_type_t(HAL_RX_DISB_MODE_6) RX_DISB_MODE_6;
	rg_type_t(HAL_RX_FORCE_MODE_7) RX_FORCE_MODE_7;
	rg_type_t(HAL_RX_DISB_MODE_5) RX_DISB_MODE_5;
	rg_type_t(HAL_RX_FORCE_MODE_6) RX_FORCE_MODE_6;
	rg_type_t(HAL_RX_CTRL_SEQUENCE_DISB_CTRL_0) RX_CTRL_SEQUENCE_DISB_CTRL_0;
	rg_type_t(HAL_RX_CTRL_SEQUENCE_FORCE_CTRL_0) RX_CTRL_SEQUENCE_FORCE_CTRL_0;
	rg_type_t(HAL_RX_DISB_MODE_3) RX_DISB_MODE_3;
	rg_type_t(HAL_RX_FORCE_MODE_3) RX_FORCE_MODE_3;


	if (lane == 1)
		lane_offset_dig = 0x1000;
	else
		lane_offset_dig = 0;


	rg_force_da_pxp_tx_rate_ctrl.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_tx_rate_ctrl);
	rg_force_da_pxp_tx_rate_ctrl.hal.rg_force_da_pxp_cdr_pr_pieye = 0x0; 
	rg_force_da_pxp_tx_rate_ctrl.hal.rg_force_sel_da_pxp_cdr_pr_pieye = 0x0; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_tx_rate_ctrl, rg_force_da_pxp_tx_rate_ctrl.dat.value); //excel 42, write addr: 0x1FA5B784 = 0x0


	rg_force_da_pxp_cdr_pr_fll_cor.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_fll_cor);
	rg_force_da_pxp_cdr_pr_fll_cor.hal.rg_force_da_pxp_rx_dac_eye = 0x0; 
	rg_force_da_pxp_cdr_pr_fll_cor.hal.rg_force_sel_da_pxp_rx_dac_eye = 0x1; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_fll_cor, rg_force_da_pxp_cdr_pr_fll_cor.dat.value); //excel 44, write addr: 0x1FA5B790 = 0x1000000






	//pical redo
	//reset block

	PHY_EQ_CTRL_0.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _PHY_EQ_CTRL_0);
	PHY_EQ_CTRL_0.hal.rg_eq_en_delay = 0x80; //pical redo, reset block
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _PHY_EQ_CTRL_0, PHY_EQ_CTRL_0.dat.value); //excel 45, write addr: 0x1FA5B118 = 0xA000A80

	SS_RX_PI_CAL.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _SS_RX_PI_CAL);
	SS_RX_PI_CAL.hal.rg_kpgain = 0x1; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _SS_RX_PI_CAL, SS_RX_PI_CAL.dat.value); //excel 46, write addr: 0x1FA5B15C = 0x100

	RX_RESET_0.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_RESET_0);
	RX_RESET_0.hal.rg_eq_pi_cal_rst_b = 0x0; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_RESET_0, RX_RESET_0.dat.value); //excel 47, write addr: 0x1FA5B204 = 0x1000101	
	

	RX_DISB_MODE_6.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_DISB_MODE_6);
	RX_DISB_MODE_6.hal.rg_disb_rx_and_pical_rstb = 0x0; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_DISB_MODE_6, RX_DISB_MODE_6.dat.value); //excel 48, write addr: 0x1FA5B334 = 0x1010001

	RX_FORCE_MODE_7.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_FORCE_MODE_7);
	RX_FORCE_MODE_7.hal.rg_force_rx_and_pical_rstb = 0x0; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_FORCE_MODE_7, RX_FORCE_MODE_7.dat.value); //excel 49, write addr: 0x1FA5B328 = 0x0

	RX_DISB_MODE_6.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_DISB_MODE_6);
	RX_DISB_MODE_6.hal.rg_disb_ref_and_pical_rstb = 0x0; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_DISB_MODE_6, RX_DISB_MODE_6.dat.value); //excel 50, write addr: 0x1FA5B334 = 0x1010000

	RX_FORCE_MODE_7.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_FORCE_MODE_7);
	RX_FORCE_MODE_7.hal.rg_force_ref_and_pical_rstb = 0x0; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_FORCE_MODE_7, RX_FORCE_MODE_7.dat.value); //excel 51, write addr: 0x1FA5B328 = 0x0

	//enable	

	RX_DISB_MODE_5.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_DISB_MODE_5);
	RX_DISB_MODE_5.hal.rg_disb_rx_or_pical_en = 0x0; //enable
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_DISB_MODE_5, RX_DISB_MODE_5.dat.value); //excel 52, write addr: 0x1FA5B324 = 0x10101

	RX_FORCE_MODE_6.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_FORCE_MODE_6);
	RX_FORCE_MODE_6.hal.rg_force_rx_or_pical_en = 0x0; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_FORCE_MODE_6, RX_FORCE_MODE_6.dat.value); //excel 53, write addr: 0x1FA5B318 = 0x0

	RX_CTRL_SEQUENCE_DISB_CTRL_0.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_CTRL_SEQUENCE_DISB_CTRL_0);
	RX_CTRL_SEQUENCE_DISB_CTRL_0.hal.rg_disb_rx_pical_en = 0x0; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_CTRL_SEQUENCE_DISB_CTRL_0, RX_CTRL_SEQUENCE_DISB_CTRL_0.dat.value); //excel 54, write addr: 0x1FA5B108 = 0x1010001

	RX_CTRL_SEQUENCE_FORCE_CTRL_0.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_CTRL_SEQUENCE_FORCE_CTRL_0);
	RX_CTRL_SEQUENCE_FORCE_CTRL_0.hal.rg_force_rx_pical_en = 0x0; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_CTRL_SEQUENCE_FORCE_CTRL_0, RX_CTRL_SEQUENCE_FORCE_CTRL_0.dat.value); //excel 55, write addr: 0x1FA5B110 = 0x0


	//release reset
	RX_RESET_0.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_RESET_0);
	RX_RESET_0.hal.rg_eq_pi_cal_rst_b = 0x1; //release reset
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_RESET_0, RX_RESET_0.dat.value); //excel 56, write addr: 0x1FA5B204 = 0x1010101

	RX_FORCE_MODE_7.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_FORCE_MODE_7);
	RX_FORCE_MODE_7.hal.rg_force_rx_and_pical_rstb = 0x1; 
	RX_FORCE_MODE_7.hal.rg_force_ref_and_pical_rstb = 0x1; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_FORCE_MODE_7, RX_FORCE_MODE_7.dat.value); //excel 58, write addr: 0x1FA5B328 = 0x101

	RX_FORCE_MODE_6.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_FORCE_MODE_6);
	RX_FORCE_MODE_6.hal.rg_force_rx_or_pical_en = 0x1; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_FORCE_MODE_6, RX_FORCE_MODE_6.dat.value); //excel 59, write addr: 0x1FA5B318 = 0x100


	udelay(1000); //delay for 1ms 


	RX_FORCE_MODE_6.hal.rg_force_rx_or_pical_en = 0x0; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_FORCE_MODE_6, RX_FORCE_MODE_6.dat.value); //excel 61, write addr: 0x1FA5B318 = 0x0
	RX_DISB_MODE_3.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_DISB_MODE_3);
	RX_DISB_MODE_3.hal.rg_disb_eq_pi_cal_rdy = 0x0; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_DISB_MODE_3, RX_DISB_MODE_3.dat.value); //excel 62, write addr: 0x1FA5B31C = 0x1010100
	RX_FORCE_MODE_3.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_FORCE_MODE_3);
	RX_FORCE_MODE_3.hal.rg_force_eq_pi_cal_rdy = 0x1; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_FORCE_MODE_3, RX_FORCE_MODE_3.dat.value); //excel 63, write addr: 0x1FA5B30C = 0x1


}

void PCIe_G3_Eye_Scan(char lane, char quick_flag)
{
	u32 pical_data_out, tmp, val, zero_start, zero_end;
	u32 ro_dac_eye, eye_offset = 0;
	u32 lane_offset_dig;
	u32 y_cnt[130];
	u32 zero_cnt = 0;

	int EYE_X_FW, EYE_Y_FW=0;
	uint EYE_X_HW, EYE_Y_HW=0;            
	uint eyecnt = 0;
	int i,j,k,m = 0;

	int start_p = 0;//start: left move (40)
	int sweep_r = 0; //then sweep right (80)

	int Start_Point = start_p;
	int Sweep_Range = sweep_r;
	int Ovr = 1; 	//need to modify for different RX Rate, //20230301 for coverity , give init value 1
	
	rg_type_t(HAL_rg_force_da_pxp_cdr_pr_pieye_pwdb) rg_force_da_pxp_cdr_pr_pieye_pwdb;
	rg_type_t(HAL_RX_EYE_TOP_EYECNT_CTRL_1) RX_EYE_TOP_EYECNT_CTRL_1;
	rg_type_t(HAL_RX_DISB_MODE_7) RX_DISB_MODE_7;
	rg_type_t(HAL_RX_FORCE_MODE_8) RX_FORCE_MODE_8;
	rg_type_t(HAL_rg_force_da_pxp_tx_rate_ctrl) rg_force_da_pxp_tx_rate_ctrl;
	rg_type_t(HAL_RX_DEBUG_0) RX_DEBUG_0;
	rg_type_t(HAL_ADD_RO_RX2ANA_1) ADD_RO_RX2ANA_1;
	rg_type_t(HAL_RX_TORGS_DEBUG_2) RX_TORGS_DEBUG_2;

	rg_type_t(HAL_RGS_PXP_RX0_OSCAL_FE_VOS) RGS_PXP_RX0_OSCAL_FE_VOS;
	rg_type_t(HAL_RGS_PXP_AEQ0_CTLE) RGS_PXP_AEQ0_CTLE;
	rg_type_t(HAL_RGS_PXP_AEQ0_D1_OS) RGS_PXP_AEQ0_D1_OS;
	rg_type_t(HAL_RGS_PXP_AEQ0_ERR0_OS) RGS_PXP_AEQ0_ERR0_OS;
	rg_type_t(HAL_RGS_PXP_RX1_OSCAL_FE_VOS) RGS_PXP_RX1_OSCAL_FE_VOS;
	rg_type_t(HAL_RGS_PXP_AEQ1_SAOSC_EN) RGS_PXP_AEQ1_SAOSC_EN;
	rg_type_t(HAL_RGS_PXP_AEQ1_E1_OS) RGS_PXP_AEQ1_E1_OS;
	rg_type_t(HAL_RGS_PXP_AEQ1_CTLE) RGS_PXP_AEQ1_CTLE;

	if (lane == 1)
	{
		RGS_PXP_RX1_OSCAL_FE_VOS.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_RX1_OSCAL_FE_VOS);
		val = RGS_PXP_RX1_OSCAL_FE_VOS.hal.rgs_pxp_rx1_oscal_compos; //excel 643 read addr: 0x1FA5A250 [13:08]
		printk("RGS_PXP_RX1_OSCAL_COMPOS : %x\n", val);

		RGS_PXP_RX1_OSCAL_FE_VOS.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_RX1_OSCAL_FE_VOS);
		val = RGS_PXP_RX1_OSCAL_FE_VOS.hal.rgs_pxp_rx1_oscal_fe_vos; //excel 644 read addr: 0x1FA5A250 [05:00]
		printk("RGS_PXP_RX1_OSCAL_FE_VOS : %x\n", val);

		RGS_PXP_AEQ1_SAOSC_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_AEQ1_SAOSC_EN);
		val = RGS_PXP_AEQ1_SAOSC_EN.hal.rgs_pxp_aeq1_d0_os; //excel 645 read addr: 0x1FA5A260 [14:08]
		printk("RGS_PXP_AEQ1_D0_OS : %x\n", val);

		RGS_PXP_AEQ1_SAOSC_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_AEQ1_SAOSC_EN);
		val = RGS_PXP_AEQ1_SAOSC_EN.hal.rgs_pxp_aeq1_d1_os; //excel 646 read addr: 0x1FA5A260 [22:16]
		printk("RGS_PXP_AEQ1_D1_OS : %x\n", val);

		RGS_PXP_AEQ1_SAOSC_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_AEQ1_SAOSC_EN);
		val = RGS_PXP_AEQ1_SAOSC_EN.hal.rgs_pxp_aeq1_e0_os; //excel 647 read addr: 0x1FA5A260 [30:24]
		printk("RGS_PXP_AEQ1_E0_OS : %x\n", val);

		RGS_PXP_AEQ1_E1_OS.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_AEQ1_E1_OS);
		val = RGS_PXP_AEQ1_E1_OS.hal.rgs_pxp_aeq1_e1_os; //excel 648 read addr: 0x1FA5A264 [06:00]
		printk("RGS_PXP_AEQ1_E1_OS : %x\n", val);

		RGS_PXP_AEQ1_E1_OS.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_AEQ1_E1_OS);
		val = RGS_PXP_AEQ1_E1_OS.hal.rgs_pxp_aeq1_err0_os; //excel 649 read addr: 0x1FA5A264 [22:16]
		printk("RGS_PXP_AEQ1_ERR0_OS : %x\n", val);

		RGS_PXP_AEQ1_CTLE.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_AEQ1_CTLE);
		val = RGS_PXP_AEQ1_CTLE.hal.rgs_pxp_aeq1_ctle; //excel 650 read addr: 0x1FA5A25C [04:00]
		printk("RGS_PXP_AEQ1_CTLE : %x\n", val);
		
		lane_offset_dig = 0x1000;
		Reg_R_then_W(0x1fa5b360, 0x1a, 20, 16);
	}
	else
	{
		RGS_PXP_RX0_OSCAL_FE_VOS.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_RX0_OSCAL_FE_VOS);
		val = RGS_PXP_RX0_OSCAL_FE_VOS.hal.rgs_pxp_rx0_oscal_compos; //excel 622 read addr: 0x1FA5A194 [13:08]
		printk("RGS_PXP_RX0_OSCAL_COMPOS : %x\n", val);

		RGS_PXP_RX0_OSCAL_FE_VOS.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_RX0_OSCAL_FE_VOS);
		val = RGS_PXP_RX0_OSCAL_FE_VOS.hal.rgs_pxp_rx0_oscal_fe_vos; //excel 623 read addr: 0x1FA5A194 [05:00]
		printk("RGS_PXP_RX0_OSCAL_FE_VOS : %x\n", val);

		RGS_PXP_AEQ0_CTLE.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_AEQ0_CTLE);
		val = RGS_PXP_AEQ0_CTLE.hal.rgs_pxp_aeq0_d0_os; //excel 624 read addr: 0x1FA5A1A0 [30:24]
		printk("RGS_PXP_AEQ0_D0_OS : %x\n", val);

		RGS_PXP_AEQ0_D1_OS.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_AEQ0_D1_OS);
		val = RGS_PXP_AEQ0_D1_OS.hal.rgs_pxp_aeq0_d1_os; //excel 625 read addr: 0x1FA5A1A4 [06:00]
		printk("RGS_PXP_AEQ0_D1_OS : %x\n", val);

		RGS_PXP_AEQ0_D1_OS.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_AEQ0_D1_OS);
		val = RGS_PXP_AEQ0_D1_OS.hal.rgs_pxp_aeq0_e0_os; //excel 626 read addr: 0x1FA5A1A4 [14:08]
		printk("RGS_PXP_AEQ0_E0_OS : %x\n", val);

		RGS_PXP_AEQ0_D1_OS.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_AEQ0_D1_OS);
		val = RGS_PXP_AEQ0_D1_OS.hal.rgs_pxp_aeq0_e1_os; //excel 627 read addr: 0x1FA5A1A4 [22:16]
		printk("RGS_PXP_AEQ0_E1_OS : %x\n", val);

		RGS_PXP_AEQ0_ERR0_OS.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_AEQ0_ERR0_OS);
		val = RGS_PXP_AEQ0_ERR0_OS.hal.rgs_pxp_aeq0_err0_os; //excel 628 read addr: 0x1FA5A1A8 [06:00]
		printk("RGS_PXP_AEQ0_ERR0_OS : %x\n", val);

		RGS_PXP_AEQ0_CTLE.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_AEQ0_CTLE);
		val = RGS_PXP_AEQ0_CTLE.hal.rgs_pxp_aeq0_ctle; //excel 629 read addr: 0x1FA5A1A0 [12:08]
		printk("RGS_PXP_AEQ0_CTLE : %x\n", val);		
		
		lane_offset_dig = 0;
		Reg_R_then_W(0x1fa5b360, 0x5, 20, 16);
	}

	//Reg_R_then_W(lane_offset_dig + 0x1fa5b360, 0x5, 20, 16);
	tmp = Reg_R(lane_offset_dig + 0x1fa5b000, 0, 0x380) & 0x3;

	switch (tmp)
	{
		case 0:	//8G
			Ovr = 1;
			Reg_R_then_W(lane_offset_dig + 0x1fa5b360, 0x0, 10, 8);
			break;
		case 1:	//5G
			Ovr = 2;	
			Reg_R_then_W(lane_offset_dig + 0x1fa5b360, 0x3, 10, 8);
			break;
		case 2:	//2.5G
			Ovr = 4;		
			Reg_R_then_W(lane_offset_dig + 0x1fa5b360, 0x3, 10, 8);
			break;
		default :
			
			break;
	}

	rg_force_da_pxp_cdr_pr_pieye_pwdb.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_pieye_pwdb);
	rg_force_da_pxp_cdr_pr_pieye_pwdb.hal.rg_force_da_pxp_cdr_pr_pieye_pwdb = 0x1; 
	rg_force_da_pxp_cdr_pr_pieye_pwdb.hal.rg_force_sel_da_pxp_cdr_pr_pieye_pwdb = 0x1; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_pieye_pwdb, rg_force_da_pxp_cdr_pr_pieye_pwdb.dat.value); //excel 10, write addr: 0x1FA5B824 = 0x101

	PCIe_eye_setting(lane);

	 RX_EYE_TOP_EYECNT_CTRL_1.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_EYE_TOP_EYECNT_CTRL_1);
	 RX_EYE_TOP_EYECNT_CTRL_1.hal.rg_disb_eyedur_init_b = 0x0; 
	 RX_EYE_TOP_EYECNT_CTRL_1.hal.rg_force_eyedur_init_b = 0x0; 
	 Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_EYE_TOP_EYECNT_CTRL_1, RX_EYE_TOP_EYECNT_CTRL_1.dat.value); //excel 76, write addr: 0x1FA5B084 = 0x1
	
	RX_DISB_MODE_7.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_DISB_MODE_7);
	RX_DISB_MODE_7.hal.rg_disb_eyecnt_rx_rst_b = 0x0; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_DISB_MODE_7, RX_DISB_MODE_7.dat.value); //excel 77, write addr: 0x1FA5B338 = 0x10101

	RX_FORCE_MODE_8.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_FORCE_MODE_8);
	RX_FORCE_MODE_8.hal.rg_force_eyecnt_rx_rst_b = 0x0; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_FORCE_MODE_8, RX_FORCE_MODE_8.dat.value); //excel 78, write addr: 0x1FA5B32C = 0x0

	RX_EYE_TOP_EYECNT_CTRL_1.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_EYE_TOP_EYECNT_CTRL_1);
	RX_EYE_TOP_EYECNT_CTRL_1.hal.rg_disb_eyedur_init_b = 0x1; 
	RX_EYE_TOP_EYECNT_CTRL_1.hal.rg_force_eyedur_init_b = 0x1; 
	RX_EYE_TOP_EYECNT_CTRL_1.hal.rg_disb_eyedur_en = 0x1; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_EYE_TOP_EYECNT_CTRL_1, RX_EYE_TOP_EYECNT_CTRL_1.dat.value); //excel 80, write addr: 0x1FA5B084 = 0x0
		
	PCIe_eye_Cal(lane);
					   
	RX_DEBUG_0.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_DEBUG_0);
	RX_DEBUG_0.hal.rg_ro_toggle = 0x0; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_DEBUG_0, RX_DEBUG_0.dat.value); //excel 65, write addr: 0x1FA5B20C = 0xFFFF

	udelay(100);
	RX_DEBUG_0.hal.rg_ro_toggle = 0x1; 
	Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_DEBUG_0, RX_DEBUG_0.dat.value); //excel 67, write addr: 0x1FA5B20C = 0x100FFFF

	ADD_RO_RX2ANA_1.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _ADD_RO_RX2ANA_1);
	ro_dac_eye = ADD_RO_RX2ANA_1.hal.ro_rx_dac_eye; //excel 68 read addr: 0x1FA5B424 [06:00]
	printk("ro_rx_dac_eye : 0x%x\n", ro_dac_eye);

	RX_TORGS_DEBUG_2.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _RX_TORGS_DEBUG_2);
	pical_data_out = RX_TORGS_DEBUG_2.hal.ro_pi_cal_data_out; //excel 69 read addr: 0x1FA5B23C [22:16]
	printk("ro_pi_cal_data_out : 0x%x\n", pical_data_out);

	EYE_X_HW = pical_data_out;
	EYE_X_FW = EYE_X_HW;

	if(quick_flag == 0)
	{

		Start_Point = 50;//start: left move 50
		Sweep_Range = 130; //then sweep right 130

		ro_dac_eye = eye_offset + ro_dac_eye;		
		//EYE_X_HW = 0;
	    	EYE_Y_HW = eye_offset + ro_dac_eye;

		//EYE_Y_HW = 64;	
		EYE_Y_HW = 64 + ro_dac_eye;	
		EYE_Y_FW = -64; 

		//printk("Yoffset = %d \n", ro_dac_eye);
		//printk("pical_data_out = %d \n", pical_data_out);		 
					   	
		for (i = 0; i < Start_Point*Ovr; i++){

			 rg_force_da_pxp_tx_rate_ctrl.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_tx_rate_ctrl);
			 rg_force_da_pxp_tx_rate_ctrl.hal.rg_force_da_pxp_cdr_pr_pieye = EYE_X_HW; //X_start=pical_data_out
			 rg_force_da_pxp_tx_rate_ctrl.hal.rg_force_sel_da_pxp_cdr_pr_pieye = 0x1; 
			 Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_tx_rate_ctrl, rg_force_da_pxp_tx_rate_ctrl.dat.value); //excel 72, write addr: 0x1FA5B784 = 0x1000000

			EYE_X_HW--;
			EYE_X_FW--;				
		}	
		
		//snack sequence sweep full eye scan																							  
		for (k = 0; k < (Sweep_Range/2) ; k++)																							  
		{	
			for (i = 0; i < 65; i++)																									  
			{																															  		
				eyecnt =PCIe_eyescan_countPoint(EYE_X_HW, EYE_Y_HW, lane);
				//printk("%d	%d %d \n", eyecnt, EYE_X_FW, EYE_Y_FW);
				printk("%07d ", eyecnt);
				m++;
				if((m%16)==0)
				 	printk("z\n");
					 	  
				EYE_Y_HW++; 																											  
				EYE_Y_FW++; 																											  
			}																															  
			EYE_Y_HW = 1;																												  
			EYE_Y_FW = 1;

			//ssleep(1);
																																			  
			for (j = 0; j < 63; j++)																									  
			{																															  
				eyecnt =PCIe_eyescan_countPoint(EYE_X_HW, EYE_Y_HW, lane);																				  
				//printk("%d	%d %d \n", eyecnt, EYE_X_FW, EYE_Y_FW);	  
				printk("%07d ", eyecnt);	
				m++;
				if((m%16)==0)
				 	printk("z\n");
				
				EYE_Y_HW++; 																											  
				EYE_Y_FW++; 																											  
			}			

			//ssleep(1); 
			printk("EYE_X_HW=%x\n",EYE_X_HW );	
			msleep(100);
			
			EYE_X_HW = PCIe_eyescan_moveX(EYE_X_FW, EYE_Y_FW, Ovr, lane);																					  
			EYE_X_FW = EYE_X_FW + Ovr;																									  
																																		  
			EYE_Y_HW--; 																												  
			EYE_Y_FW--; 																												  
																																		  
			for (j = 0; j < 63; j++)																									  
			{																															  
				eyecnt =PCIe_eyescan_countPoint(EYE_X_HW, EYE_Y_HW, lane);
				y_cnt[EYE_Y_HW] = eyecnt;
				//printk("%d	%d %d \n", eyecnt, EYE_X_FW, EYE_Y_FW);	 	  
				EYE_Y_HW--; 																											  
				EYE_Y_FW--; 																											  
			}																															  
			EYE_Y_HW = 128; 																											  
			EYE_Y_FW = 0;

			//ssleep(1);
																																		  
			for (i = 0; i < 65; i++)																									  
			{																															  
				eyecnt =PCIe_eyescan_countPoint(EYE_X_HW, EYE_Y_HW, lane);	
				y_cnt[EYE_Y_HW%128] = eyecnt;
				//printk("%d	%d %d \n", eyecnt, EYE_X_FW, EYE_Y_FW);	  	  
				EYE_Y_HW--; 																											  
				EYE_Y_FW--; 																											  
			}	

			for (i = 64; i < (64+128); i++)																									  
			{
				printk("%07d ", y_cnt[i%128] );
				m++;
				if((m%16)==0)
				 	printk("z\n");
			}		
			m = 0;

			printk("EYE_X_HW=%x\n",EYE_X_HW );	
			msleep(100);
			
			EYE_X_HW = PCIe_eyescan_moveX(EYE_X_FW, EYE_Y_FW, Ovr, lane);																					  
			EYE_X_FW = EYE_X_FW + Ovr;																									  
																																		  
			EYE_Y_HW++; 																												  
			EYE_Y_FW++; 		
			
		}																																  
																																		  
		//last time bottom-up sweep Y index 																							  
		for (i = 0; i < 65; i++)																										  
		{																																  
			eyecnt =PCIe_eyescan_countPoint(EYE_X_HW, EYE_Y_HW, lane);																				  
			//printk("%d	%d %d \n", eyecnt, EYE_X_FW, EYE_Y_FW);	 				  
			printk("%07d ", eyecnt);
			m++;
			if((m%16)==0)
			 	printk("z\n");
			
			EYE_Y_HW++; 																												  
			EYE_Y_FW++; 																												  
		}			
		EYE_Y_HW = 1;																													  
		EYE_Y_FW = 1;																													  
		for (j = 0; j < 63; j++)																										  
		{																																  
			eyecnt =PCIe_eyescan_countPoint(EYE_X_HW, EYE_Y_HW, lane);																				  
			//printk("%d	%d %d \n", eyecnt, EYE_X_FW, EYE_Y_FW);	 	
			printk("%07d ", eyecnt);
			m++;
			if((m%16)==0)
			 	printk("z\n");
			
			EYE_Y_HW++; 																												  
			EYE_Y_FW++; 																												  
		}

		printk("EYE_X_HW=%x\n",EYE_X_HW );	
	}else //==========================Quick eye scan
	{
		start_p = 45;//start: left move 40
		sweep_r = 95; //then sweep right 90

		printk("scan Y\n");	
		zero_start = 64; //count eye Y center point
		zero_end = 64;

		for (EYE_Y_HW = 64; EYE_Y_HW< (64+128); EYE_Y_HW++)																									  
		{																															  
			eyecnt =PCIe_eyescan_countPoint(EYE_X_HW, EYE_Y_HW&0x7f,  lane);		
			if(eyecnt == 0)
			{
				if(zero_cnt == 0) //first time cnt=0, record Y 
					zero_start = EYE_Y_HW;

				if(EYE_Y_HW == (63+128)) //when offset error, zero area not apear at center
					zero_end = 63+128;
				
				zero_cnt++;
				
			}else if((zero_start >64)&&(zero_end == 64)) //after cnt=0, first time cnt !=0, record Y 
			{
				zero_end = EYE_Y_HW;
				//printk("zero_end:%d",zero_end);
			}
			
			printk("%x ", (eyecnt!=0));	
			 																														  
		}

		printk("\neye Height = %d \nscan X\n", zero_cnt);
		
		ro_dac_eye = (zero_start + zero_end)>>1 ; //get middle of eye center point of Y

		//printk("zero_start:%d, zero_end: %d, ro_dac_eye : %d\n",zero_start, zero_end, ro_dac_eye);

		zero_cnt = 0;

		//move X 
		for (i = 0; i < start_p*Ovr; i++){
		
			 rg_force_da_pxp_tx_rate_ctrl.dat.value = Reg_R(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_tx_rate_ctrl);
			 rg_force_da_pxp_tx_rate_ctrl.hal.rg_force_da_pxp_cdr_pr_pieye = EYE_X_HW; //X_start=pical_data_out
			 rg_force_da_pxp_tx_rate_ctrl.hal.rg_force_sel_da_pxp_cdr_pr_pieye = 0x1; 
			 Reg_W(lane_offset_dig + PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_tx_rate_ctrl, rg_force_da_pxp_tx_rate_ctrl.dat.value); //excel 72, write addr: 0x1FA5B784 = 0x1000000

			EYE_X_HW--;
			EYE_X_FW--;
				
		}	
		
		for (k = 0; k< sweep_r; k++)																									  
		{																															  
			eyecnt =PCIe_eyescan_countPoint(EYE_X_HW, ro_dac_eye&0x7f,  lane);		
			EYE_X_HW = PCIe_eyescan_moveX(EYE_X_FW, EYE_Y_FW, Ovr, lane);																					  
			EYE_X_FW = EYE_X_FW + Ovr;		
			
			if(eyecnt == 0)
				zero_cnt++;
	
			printk("%x ", (eyecnt!=0));																														  
		}

		printk("\neye Width = %d \n", zero_cnt);
			
	}

}

void PCIe_G3_PWR_Saving_on(void)
{
	//to do after HEC
}



void PCIe_G3_Wait_Debug_Condition(void)
{
	//u32  best_peaking_ctrl, best_heo, x_range,pi_os, aeq_ctle;
	//int x, y,flag;
	
	

#if 0 //AEQ done
	AEQ_cnt = 0;
	Reg_W(0x1fa5b000, 0, 0x360, 0x70000);
	while(((Reg_R(0x1fa5b000, 0, 0x380) >> 8) & 0x1) != 0 )  //wait AEQ reset
	{;}

	while(((Reg_R(0x1fa5b000, 0, 0x380) >> 8) & 0x1) == 0 )   //wait AEQ_done
	{AEQ_cnt++;}

#endif

	while(((Reg_R(0x1fa5b000, 0, 0x380) >> 2) & 0x1) != 0 )  //after enter speed, wait lck2data
	{AEQ_cnt++;}



	 

}

void PCIe_G3_Rx_Analog_Status_Monitor(void)
{

	u32 x;

#if 1	
//lane 0
 rg_type_t(HAL_RGS_PXP_RX0_OSCAL_FE_VOS) RGS_PXP_RX0_OSCAL_FE_VOS;
 rg_type_t(HAL_RGS_PXP_AEQ0_CTLE) RGS_PXP_AEQ0_CTLE;
 rg_type_t(HAL_RGS_PXP_AEQ0_D1_OS) RGS_PXP_AEQ0_D1_OS;
 rg_type_t(HAL_RGS_PXP_AEQ0_ERR0_OS) RGS_PXP_AEQ0_ERR0_OS;

  rg_type_t(HAL_SS_RX_FLL_9) SS_RX_FLL_9;
 rg_type_t(HAL_SS_RX_FLL_a) SS_RX_FLL_a;
 
/*
 rg_type_t(HAL_SS_RX_FLL_6) SS_RX_FLL_6;
 rg_type_t(HAL_SS_RX_FLL_7) SS_RX_FLL_7;
 rg_type_t(HAL_SS_RX_FLL_8) SS_RX_FLL_8;

//lane 1
 rg_type_t(HAL_RGS_PXP_RX1_OSCAL_FE_VOS) RGS_PXP_RX1_OSCAL_FE_VOS;
 rg_type_t(HAL_RGS_PXP_AEQ1_SAOSC_EN) RGS_PXP_AEQ1_SAOSC_EN;
 rg_type_t(HAL_RGS_PXP_AEQ1_E1_OS) RGS_PXP_AEQ1_E1_OS;
 rg_type_t(HAL_RGS_PXP_AEQ1_CTLE) RGS_PXP_AEQ1_CTLE;
*/
 

Reg_W(0x1fa5b000, 0, 0x360, 0x30000);

//*******   RX  check  parameter
 RGS_PXP_RX0_OSCAL_FE_VOS.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_RX0_OSCAL_FE_VOS);
 OSCAL_COMPOS = RGS_PXP_RX0_OSCAL_FE_VOS.hal.rgs_pxp_rx0_oscal_compos; //excel 9 read addr: 0x1FA5A194 [13:08]
 

 //RGS_PXP_RX0_OSCAL_FE_VOS.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_RX0_OSCAL_FE_VOS);
 FE_VOS = RGS_PXP_RX0_OSCAL_FE_VOS.hal.rgs_pxp_rx0_oscal_fe_vos; //excel 10 read addr: 0x1FA5A194 [05:00]
 

 RGS_PXP_AEQ0_CTLE.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_AEQ0_CTLE);
 AEQ0_D0_OS = RGS_PXP_AEQ0_CTLE.hal.rgs_pxp_aeq0_d0_os; //excel 11 read addr: 0x1FA5A1A0 [30:24]
 

 RGS_PXP_AEQ0_D1_OS.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_AEQ0_D1_OS);
 AEQ0_D1_OS = RGS_PXP_AEQ0_D1_OS.hal.rgs_pxp_aeq0_d1_os; //excel 12 read addr: 0x1FA5A1A4 [06:00]
 

 //RGS_PXP_AEQ0_D1_OS.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_AEQ0_D1_OS);
 AEQ0_E0_OS = RGS_PXP_AEQ0_D1_OS.hal.rgs_pxp_aeq0_e0_os; //excel 13 read addr: 0x1FA5A1A4 [14:08]
 

 //RGS_PXP_AEQ0_D1_OS.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_AEQ0_D1_OS);
 AEQ0_E1_OS = RGS_PXP_AEQ0_D1_OS.hal.rgs_pxp_aeq0_e1_os; //excel 14 read addr: 0x1FA5A1A4 [22:16]
 

 RGS_PXP_AEQ0_ERR0_OS.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_AEQ0_ERR0_OS);
 AEQ0_ERR0_OS = RGS_PXP_AEQ0_ERR0_OS.hal.rgs_pxp_aeq0_err0_os; //excel 15 read addr: 0x1FA5A1A8 [06:00]
 

 RGS_PXP_AEQ0_CTLE.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_AEQ0_CTLE);
 AEQ0_CTLE = RGS_PXP_AEQ0_CTLE.hal.rgs_pxp_aeq0_ctle; //excel 16 read addr: 0x1FA5A1A0 [12:08]
 

 sigdet_os = (Reg_R(0x1fa5b000, 0, 0x380) >> 9) & 0x1f; //0x380 [13:9]

/*
//lane1

Reg_W(0x1fa5c000, 0, 0x360, 0x30000);


 RGS_PXP_RX1_OSCAL_FE_VOS.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_RX1_OSCAL_FE_VOS);
 OSCAL_COMPOS_1 = RGS_PXP_RX1_OSCAL_FE_VOS.hal.rgs_pxp_rx1_oscal_compos; //excel 9 read addr: 0x1FA5A250 [13:08]

 RGS_PXP_RX1_OSCAL_FE_VOS.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_RX1_OSCAL_FE_VOS);
 FE_VOS_1 = RGS_PXP_RX1_OSCAL_FE_VOS.hal.rgs_pxp_rx1_oscal_fe_vos; //excel 10 read addr: 0x1FA5A250 [05:00]

 RGS_PXP_AEQ1_SAOSC_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_AEQ1_SAOSC_EN);
 AEQ1_D0_OS = RGS_PXP_AEQ1_SAOSC_EN.hal.rgs_pxp_aeq1_d0_os; //excel 11 read addr: 0x1FA5A260 [14:08]

 RGS_PXP_AEQ1_SAOSC_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_AEQ1_SAOSC_EN);
 AEQ1_D1_OS = RGS_PXP_AEQ1_SAOSC_EN.hal.rgs_pxp_aeq1_d1_os; //excel 12 read addr: 0x1FA5A260 [22:16]

 RGS_PXP_AEQ1_SAOSC_EN.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_AEQ1_SAOSC_EN);
 AEQ1_E0_OS = RGS_PXP_AEQ1_SAOSC_EN.hal.rgs_pxp_aeq1_e0_os; //excel 13 read addr: 0x1FA5A260 [30:24]

 RGS_PXP_AEQ1_E1_OS.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_AEQ1_E1_OS);
 AEQ1_E1_OS = RGS_PXP_AEQ1_E1_OS.hal.rgs_pxp_aeq1_e1_os; //excel 14 read addr: 0x1FA5A264 [06:00]

 RGS_PXP_AEQ1_E1_OS.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_AEQ1_E1_OS);
 AEQ1_ERR0_OS = RGS_PXP_AEQ1_E1_OS.hal.rgs_pxp_aeq1_err0_os; //excel 15 read addr: 0x1FA5A264 [30:24]

 RGS_PXP_AEQ1_CTLE.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RGS_PXP_AEQ1_CTLE);
 AEQ1_CTLE = RGS_PXP_AEQ1_CTLE.hal.rgs_pxp_aeq1_ctle; //excel 16 read addr: 0x1FA5A25C [04:00]

 sigdet_os_1 = (Reg_R(0x1fa5c000, 0, 0x380) >> 9) & 0x1f; //0x380 [13:9]
 */
 	Reg_W(0x1fa5b000, 0, 0x360, 0x0);

	for(x=0; x<CHECK_LEN; x++)
	{
		//if(x==100)
		//	udelay(4000);
		
		probe[x] = Reg_R(0x1fa5b000, 0, 0x380);
		
		Reg_W(0x1fa5b000, 0, 0x20c, 0xffff);
		Reg_W(0x1fa5b000, 0, 0x20c, 0x100ffff);
		Reg_W(0x1fa5b000, 0, 0x188, 0x0);
		Reg_W(0x1fa5b000, 0, 0x188, 0x01010101);
		Reg_W(0x1fa5b000, 0, 0x188, 0x0);
		udelay(500);
		Reg_W(0x1fa5b000, 0, 0x20c, 0xffff);
		Reg_W(0x1fa5b000, 0, 0x20c, 0x100ffff);
		
		SS_RX_FLL_9.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _SS_RX_FLL_9);
 		fll_idac[x]  = SS_RX_FLL_9.hal.ro_fll_idac; //excel 329 read addr: 0x1FA5B194 [10:00]
 		ro_idacf[x] = SS_RX_FLL_9.hal.ro_idacf;

		SS_RX_FLL_a.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _SS_RX_FLL_a);
 		da_idac[x] = SS_RX_FLL_a.hal.ro_da_idac; //excel 331 read addr: 0x1FA5B198 [10:00]

		Kcode_done[x] = Reg_R(0x1fa5b000, 0, 0x54c) ;

		 //SS_RX_FLL_7.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _SS_RX_FLL_7);
		 //freq[x] = SS_RX_FLL_7.hal.ro_adc_freq; //excel 329 read addr: 0x1FA5B18C [19:00]

		 //SS_RX_FLL_8.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _SS_RX_FLL_8);
		 //cor_gain[x] = SS_RX_FLL_8.hal.ro_cor_gain; //excel 330 read addr: 0x1FA5B190 [23:00]
		/*
		Reg_W(0x1fa5c000, 0, 0x20c, 0xffff);
		Reg_W(0x1fa5c000, 0, 0x20c, 0x100ffff);
		Reg_W(0x1fa5c000, 0, 0x188, 0x0);
		Reg_W(0x1fa5c000, 0, 0x188, 0x01010101);
		Reg_W(0x1fa5c000, 0, 0x188, 0x0);
		Reg_W(0x1fa5c000, 0, 0x20c, 0xffff);
		Reg_W(0x1fa5c000, 0, 0x20c, 0x100ffff);
		SS_RX_FLL_9.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _SS_RX_FLL_9);
 		fll_idac_1[x]  = SS_RX_FLL_9.hal.ro_fll_idac; //excel 329 read addr: 0x1FA5B194 [10:00]
 		ro_idacf_1[x] = SS_RX_FLL_9.hal.ro_idacf;

		SS_RX_FLL_a.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _SS_RX_FLL_a);
 		da_idac_1[x] = SS_RX_FLL_a.hal.ro_da_idac; //excel 331 read addr: 0x1FA5B198 [10:00]

		 SS_RX_FLL_7.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _SS_RX_FLL_7);
		 freq_1[x] = SS_RX_FLL_7.hal.ro_adc_freq; //excel 329 read addr: 0x1FA5B18C [19:00]

		 SS_RX_FLL_8.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _SS_RX_FLL_8);
		 cor_gain_1[x] = SS_RX_FLL_8.hal.ro_cor_gain; //excel 330 read addr: 0x1FA5B190 [23:00]
		 */


		
	}


#endif
}

void PCIe_G3_Rx_Analog_Status_Print(void)
{
	u32 tmp,x;
	tmp = 0;

#if 0
	//printk("wait AEQ done cnt : %x\n", AEQ_cnt);
	printk("RGS_PXP_RX0_OSCAL_COMPOS : %x\n", OSCAL_COMPOS);
	printk("RGS_PXP_RX0_OSCAL_FE_VOS : %x\n", FE_VOS);
	printk("RGS_PXP_AEQ0_D0_OS : %x\n", AEQ0_D0_OS);
	printk("RGS_PXP_AEQ0_D1_OS : %x\n", AEQ0_D1_OS);
	printk("RGS_PXP_AEQ0_E0_OS : %x\n", AEQ0_E0_OS);
	printk("RGS_PXP_AEQ0_E1_OS : %x\n", AEQ0_E1_OS);
	printk("RGS_PXP_AEQ0_ERR0_OS : %x\n", AEQ0_ERR0_OS);
	printk("RGS_PXP_AEQ0_CTLE : %x\n", AEQ0_CTLE);
	printk("rg_force_da_pxp_rx_sigdet_os : %x\n", sigdet_os);

	printk("\n\n");
	printk("RGS_PXP_RX1_OSCAL_COMPOS : %x\n", OSCAL_COMPOS_1);
	printk("RGS_PXP_RX1_OSCAL_FE_VOS : %x\n", FE_VOS_1);
	printk("RGS_PXP_AEQ1_D0_OS : %x\n", AEQ1_D0_OS);
	printk("RGS_PXP_AEQ1_D1_OS : %x\n", AEQ1_D1_OS);
	printk("RGS_PXP_AEQ1_E0_OS : %x\n", AEQ1_E0_OS);
	printk("RGS_PXP_AEQ1_E1_OS : %x\n", AEQ1_E1_OS);
	printk("RGS_PXP_AEQ1_ERR0_OS : %x\n", AEQ1_ERR0_OS);
	printk("RGS_PXP_AEQ1_CTLE : %x\n", AEQ1_CTLE);
	printk("rg_force_da_pxp_rx_sigdet_os : %x\n", sigdet_os_1);



	for(x=0; x<CHECK_LEN; x++)
	{
		printk("after %05d us, fll_idac = 0x%02x,  ro_idacf = 0x%03x, da_idac = 0x%03x, freq = 0x%05x, cor_gain = 0x%06x \n", x, fll_idac[x], ro_idacf[x], da_idac[x], freq[x], cor_gain[x]);	
	}

	printk("=====================\n\n");

	for(x=0; x<CHECK_LEN; x++)
	{
		printk("after %05d us, fll_idac = 0x%02x,  ro_idacf = 0x%03x, da_idac = 0x%03x, freq = 0x%05x, cor_gain = 0x%06x \n", x, fll_idac_1[x], ro_idacf_1[x], da_idac_1[x], freq_1[x], cor_gain_1[x]);	
	}

#endif

	for(x=0; x<CHECK_LEN; x++)
	{
		//if(x==100)
		//	tmp = 4000;

		
		printk("after %06d tick, probe0 = 0x%04x, kband_done = %d, fll_idac = 0x%02x,  ro_idacf = 0x%03x, da_idac = 0x%03x, \n", x*500, probe[x], (Kcode_done[x]>>24) & 0x1, fll_idac[x], ro_idacf[x], da_idac[x]);	
		
	}
	

}

void PCIe_G3_L0_Relink_Set(void)
{
	
	 rg_type_t(HAL_SW_RST_SET) SW_RST_SET;
	 rg_type_t(HAL_RG_PXP_2L_CDR0_PR_BETA_DAC) RG_PXP_2L_CDR0_PR_BETA_DAC;


	SW_RST_SET.dat.value = Reg_R(PCIE_PMA0, PMA_OFFSET, _SW_RST_SET);
	SW_RST_SET.hal.rg_sw_rx_fifo_rst_n = 0x0; 
	SW_RST_SET.hal.rg_sw_rx_rst_n = 0x0; 
	Reg_W(PCIE_PMA0, PMA_OFFSET, _SW_RST_SET, SW_RST_SET.dat.value); //excel 259, write addr: 0x1FA5B460 = 0x17F
	SW_RST_SET.hal.rg_sw_rx_fifo_rst_n = 0x1; 
	SW_RST_SET.hal.rg_sw_rx_rst_n = 0x1; 
	Reg_W(PCIE_PMA0, PMA_OFFSET, _SW_RST_SET, SW_RST_SET.dat.value); //excel 259, write addr: 0x1FA5B460 = 0x17F

	RG_PXP_2L_CDR0_PR_BETA_DAC.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR0_PR_BETA_DAC);
	RG_PXP_2L_CDR0_PR_BETA_DAC.hal.rg_pxp_2l_cdr0_pr_beta_sel = 0x2; //10/27 Improve locking range (default 1)
	Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR0_PR_BETA_DAC, RG_PXP_2L_CDR0_PR_BETA_DAC.dat.value); //excel 278, write addr: 0x1FA5A120 = 0x4010808



}

void PCIe_G3_L1_Relink_Set(void)
{
	 rg_type_t(HAL_SW_RST_SET) SW_RST_SET;
	 rg_type_t(HAL_RG_PXP_2L_CDR1_PR_BETA_DAC) RG_PXP_2L_CDR1_PR_BETA_DAC;


	SW_RST_SET.dat.value = Reg_R(PCIE_PMA1, PMA_OFFSET, _SW_RST_SET);
	SW_RST_SET.hal.rg_sw_rx_fifo_rst_n = 0x0; 
	SW_RST_SET.hal.rg_sw_rx_rst_n = 0x0; 
	Reg_W(PCIE_PMA1, PMA_OFFSET, _SW_RST_SET, SW_RST_SET.dat.value); //excel 266, write addr: 0x1FA5C460 = 0x17F
	SW_RST_SET.hal.rg_sw_rx_fifo_rst_n = 0x1; 
	SW_RST_SET.hal.rg_sw_rx_rst_n = 0x1; 
	Reg_W(PCIE_PMA1, PMA_OFFSET, _SW_RST_SET, SW_RST_SET.dat.value); //excel 266, write addr: 0x1FA5C460 = 0x17F

	RG_PXP_2L_CDR1_PR_BETA_DAC.dat.value = Reg_R(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR1_PR_BETA_DAC);
	RG_PXP_2L_CDR1_PR_BETA_DAC.hal.rg_pxp_2l_cdr1_pr_beta_sel = 0x2; //10/27 Improve locking range
	Reg_W(PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_CDR1_PR_BETA_DAC, RG_PXP_2L_CDR1_PR_BETA_DAC.dat.value); //excel 279, write addr: 0x1FA5A1D8 = 0x4010808

}

static int pcie_phy_drv_probe(struct platform_device *pdev)
{
    
	printk("\n PCIe phy driver version: 7581.8.20230301 mm \n");	


	/* PCIEG3_PHY_PMA_PHYA physical address */
	G3_ana2L_phy_rg_base = Get_Base(PCIE_ANA_2L);
	if (IS_ERR(G3_ana2L_phy_rg_base)) {
	    printk("\nERROR(%s) G3_ana2L_phy_rg_base\n", __func__);
		return -1;
	}
    
	/* PCIEG3_PHY_PMA_PHYD_0 physical address  */
	G3_pma0_phy_rg_base = Get_Base(PCIE_PMA0);
	if (IS_ERR(G3_pma0_phy_rg_base)) {
	    printk("\nERROR(%s) G3_pma0_phy_rg_base\n", __func__);
		return -1;
	}

	/* PCIEG3_PHY_PMA_PHYD_1 physical address */
	G3_pma1_phy_rg_base = Get_Base(PCIE_PMA1);
	if (IS_ERR(G3_pma1_phy_rg_base)) {
	    printk("\nERROR(%s) G3_pma1_phy_rg_base\n", __func__);
		return -1;
	}

	
#if 0
	struct resource *res = NULL;
    if (!pdev->dev.of_node) {
        dev_err(&pdev->dev, "No pcie_phy DT node found");
        return -EINVAL;
    }

    pcie_phy = devm_kzalloc(&pdev->dev, sizeof(struct ecnt_pcie_phy), GFP_KERNEL);
    if (!pcie_phy)
        return -ENOMEM;

    platform_set_drvdata(pdev, pcie_phy);

    /* get pcie gen3 base address */
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    pcie_phy->pc_phy_base = devm_ioremap_resource(&pdev->dev, res);
    if (IS_ERR(pcie_phy->pc_phy_base))
        return PTR_ERR(pcie_phy->pc_phy_base);

    pcie_phy->dev = &pdev->dev;	
#endif
	pcie_phy_ops_init(&AN7581_ops);
	
	//YMC pretest
	//pcie_phy_init(1);
	//pcie_phy_init(2);
	
	printk("PCIe_phy_drv_probe finish\n");	
	return 0;
}
static int pcie_phy_drv_remove(struct platform_device *pdev)
{
    return 0;
}

/************************************************************************
*      P L A T F O R M   D R I V E R S   D E C L A R A T I O N S
*************************************************************************
*/
static struct platform_driver pcie_phy_driver = {
    .probe = pcie_phy_drv_probe,
    .remove = pcie_phy_drv_remove,
    .driver = {
	    .name = "ecnt-pcie_phy",
	    .of_match_table = ecnt_pcie_phy_of_id
    },
};
builtin_platform_driver(pcie_phy_driver);


//MODULE_DESCRIPTION("EcoNet pcie phy Driver");
#endif
