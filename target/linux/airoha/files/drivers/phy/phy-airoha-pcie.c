// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 Lorenzo Bianconi <lorenzo@kernel.org>
 */

#include <linux/delay.h>
#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "en7581/pcie_ecnt_phy.h"
#include "en7581/en7581_pcie0_pma_hal_reg.h"
#include "en7581/en7581_pcie0_pma_reg.h"
#include "en7581/en7581_pcie_ana_pxp_hal_reg.h"
#include "en7581/en7581_pcie_ana_pxp_reg.h"

/* ANA2L */
#define REG_PCIE_CSR_ANA2L_PXP_CMN			0x0000
#define CSR_ANA2L_PXP_CMN_LANE_EN			BIT(0)
#define CSR_ANA2L_PXP_CMN_TRIM_MASK			GENMASK(28, 24)

#define REG_PCIE_CSR_ANA2L_PXP_JCPLL_RST_DLY		0x001c
#define CSR_ANA2L_PXP_JCPLL_RST				BIT(8)

#define REG_PCIE_CSR_ANA2L_JCPLL_SSC_DELTA1		0x003c

#define REG_PCIE_CSR_ANA2L_JCPLL_SSC_PERIOD		0x0040
#define CSR_ANA2L_PXP_JCPLL_SSC_PERIOD			GENMASK(15, 0)

#define REG_PCIE_CSR_ANA2L_PXP_JCPLL_TCL_VTP_EN		0x004c
#define CSR_ANA2L_PXP_JCPLL_SPARE_LOW			GENMASK(31, 24)

#define REG_PCIE_CSR_ANA2L_PXP_TXPLL_POSTDIV_D256	0x00a0
#define CSR_ANA2L_PXP_CLKTX0_AMP			GENMASK(10, 8)
#define CSR_ANA2L_PXP_CLKTX0_OFFSET			GENMASK(17, 16)
#define CSR_ANA2L_PXP_CLKTX0_SR				GENMASK(25, 24)

#define REG_PCIE_CSR_ANA2L_PXP_CLKTX0_FORCE_OUT1	0x00a4
#define CSR_ANA2L_PXP_CLKTX0_HZ				BIT(8)
#define CSR_ANA2L_PXP_CLKTX0_IMP_SEL			GENMASK(20, 16)
#define CSR_ANA2L_PXP_CLKTX1_AMP			GENMASK(26, 24)

#define REG_PCIE_CSR_ANA2L_PXP_CLKTX1_OFFSET		0x00a8
#define CSR_ANA2L_PXP_CLKTX1_OFFSET			GENMASK(1, 0)
#define CSR_ANA2L_PXP_CLKTX1_SR				GENMASK(9, 8)
#define CSR_ANA2L_PXP_CLKTX1_HZ				BIT(24)

#define REG_PCIE_CSR_ANA2L_PXP_CLKTX1_IMP_SEL		0x00ac
#define CSR_ANA2L_PXP_CLKTX1_IMP_SEL			GENMASK(4, 0)

#define REG_PCIE_CSR_ANA2L_PXP_PLL_CMN_RESERVE0		0x00b0
#define CSR_ANA2L_PXP_PLL_RESERVE_MASK			GENMASK(15, 0)

#define REG_PCIE_CSR_ANA2L_PXP_TX0_CKLDO		0x00cc
#define CSR_ANA2L_PXP_TX0_CKLDO_EN			BIT(0)
#define CSR_ANA2L_PXP_TX0_DMEDGEGEN_EN			BIT(24)

#define REG_PCIE_CSR_ANA2L_PXP_TX1_CKLDO		0x00e8
#define CSR_ANA2L_PXP_TX1_CKLDO_EN			BIT(0)
#define CSR_ANA2L_PXP_TX1_DMEDGEGEN_EN			BIT(24)

#define REG_PCIE_CSR_ANA2L_PXP_TX1_MULTLANE		0x00ec
#define CSR_ANA2L_PXP_TX1_MULTLANE_EN			BIT(0)

#define REG_PCIE_CSR_ANA2L_PXP_RX0_REV0			0x00fc
#define CSR_ANA2L_PXP_VOS_PNINV				GENMASK(3, 2)
#define CSR_ANA2L_PXP_FE_GAIN_NORMAL_MODE		GENMASK(6, 4)
#define CSR_ANA2L_PXP_FE_GAIN_TRAIN_MODE		GENMASK(10, 8)

#define REG_PCIE_CSR_ANA2L_PXP_RX0_PHYCK_DIV		0x0100
#define CSR_ANA2L_PXP_RX0_PHYCK_SEL			GENMASK(9, 8)
#define CSR_ANA2L_PXP_RX0_PHYCK_RSTB			BIT(16)
#define CSR_ANA2L_PXP_RX0_TDC_CK_SEL			BIT(24)

#define REG_PCIE_CSR_ANA2L_PXP_CDR0_PD_PICAL_CKD8_INV	0x0104
#define CSR_ANA2L_PXP_CDR0_PD_EDGE_DISABLE		BIT(8)

#define REG_PCIE_CSR_ANA2L_PXP_CDR0_LPF_RATIO		0x0110
#define CSR_ANA2L_PXP_CDR0_LPF_TOP_LIM			GENMASK(26, 8)

#define REG_PCIE_CSR_ANA2L_PXP_CDR0_PR_BETA_DAC		0x0120
#define CSR_ANA2L_PXP_CDR0_PR_BETA_SEL			GENMASK(19, 16)
#define CSR_ANA2L_PXP_CDR0_PR_KBAND_DIV			GENMASK(26, 24)

#define REG_PCIE_CSR_ANA2L_PXP_CDR0_PR_VREG_IBAND	0x0124
#define CSR_ANA2L_PXP_CDR0_PR_VREG_IBAND		GENMASK(2, 0)
#define CSR_ANA2L_PXP_CDR0_PR_VREG_CKBUF		GENMASK(10, 8)

#define REG_PCIE_CSR_ANA2L_PXP_CDR0_PR_CKREF_DIV	0x0128
#define CSR_ANA2L_PXP_CDR0_PR_CKREF_DIV 		GENMASK(1, 0)

#define REG_PCIE_CSR_ANA2L_PXP_CDR0_PR_MONCK		0x012c
#define CSR_ANA2L_PXP_CDR0_PR_MONCK_ENABLE		BIT(0)
#define CSR_ANA2L_PXP_CDR0_PR_RESERVE0			GENMASK(19, 16)

#define REG_PCIE_CSR_ANA2L_PXP_CDR0_PR_COR_HBW		0x0130
#define CSR_ANA2L_PXP_CDR0_PR_LDO_FORCE_ON		BIT(8)
#define CSR_ANA2L_PXP_CDR0_PR_CKREF_DIV1		GENMASK(17, 16)

#define REG_PCIE_CSR_ANA2L_PXP_CDR0_PR_MONPI		0x0134
#define CSR_ANA2L_PXP_CDR0_PR_XFICK_EN			BIT(8)

#define REG_PCIE_CSR_ANA2L_PXP_RX0_SIGDET_DCTEST	0x0140
#define CSR_ANA2L_PXP_RX0_SIGDET_LPF_CTRL		GENMASK(9, 8)
#define CSR_ANA2L_PXP_RX0_SIGDET_PEAK			GENMASK(25, 24)

#define REG_PCIE_CSR_ANA2L_PXP_RX0_SIGDET_VTH_SEL	0x0144
#define CSR_ANA2L_PXP_RX0_SIGDET_VTH_SEL		GENMASK(4, 0)
#define CSR_ANA2L_PXP_RX0_FE_VB_EQ1_EN			BIT(24)

#define REG_PCIE_CSR_ANA2_PXP_RX0_FE_VB_EQ2		0x0148
#define CSR_ANA2L_PXP_RX0_FE_VB_EQ2_EN			BIT(0)
#define CSR_ANA2L_PXP_RX0_FE_VB_EQ3_EN			BIT(8)
#define CSR_ANA2L_PXP_RX0_FE_VCM_GEN_PWDB		BIT(16)

#define REG_PCIE_CSR_ANA2_PXP_RX0_OSCAL_CTLE1IOS	0x0158
#define CSR_ANA2L_PXP_RX0_PR_OSCAL_VGA1IOS		GENMASK(29, 24)

#define REG_PCIE_CSR_ANA2_PXP_RX0_OSCA_VGA1VOS		0x015c
#define CSR_ANA2L_PXP_RX0_PR_OSCAL_VGA1VOS		GENMASK(5, 0)
#define CSR_ANA2L_PXP_RX0_PR_OSCAL_VGA2IOS		GENMASK(13, 8)

#define REG_PCIE_CSR_ANA2L_PXP_RX1_REV0 		0x01b4

#define REG_PCIE_CSR_ANA2L_PXP_RX1_PHYCK_DIV		0x01b8
#define CSR_ANA2L_PXP_RX1_PHYCK_SEL			GENMASK(9, 8)
#define CSR_ANA2L_PXP_RX1_PHYCK_RSTB			BIT(16)
#define CSR_ANA2L_PXP_RX1_TDC_CK_SEL			BIT(24)

#define REG_PCIE_CSR_ANA2L_PXP_CDR1_PD_PICAL_CKD8_INV	0x01bc
#define CSR_ANA2L_PXP_CDR1_PD_EDGE_DISABLE		BIT(8)

#define REG_PCIE_CSR_ANA2L_PXP_CDR1_PR_BETA_DAC		0x01d8
#define CSR_ANA2L_PXP_CDR1_PR_BETA_SEL			GENMASK(19, 16)
#define CSR_ANA2L_PXP_CDR1_PR_KBAND_DIV			GENMASK(26, 24)

#define REG_PCIE_CSR_ANA2L_PXP_CDR1_PR_MONCK		0x01e4
#define CSR_ANA2L_PXP_CDR1_PR_MONCK_ENABLE		BIT(0)
#define CSR_ANA2L_PXP_CDR1_PR_RESERVE0			GENMASK(19, 16)

#define REG_PCIE_CSR_ANA2L_PXP_CDR1_LPF_RATIO		0x01c8
#define CSR_ANA2L_PXP_CDR1_LPF_TOP_LIM			GENMASK(26, 8)

#define REG_PCIE_CSR_ANA2L_PXP_CDR1_PR_VREG_IBAND_VAL	0x01dc
#define CSR_ANA2L_PXP_CDR1_PR_VREG_IBAND		GENMASK(2, 0)
#define CSR_ANA2L_PXP_CDR1_PR_VREG_CKBUF		GENMASK(10, 8)

#define REG_PCIE_CSR_ANA2L_PXP_CDR1_PR_CKREF_DIV	0x01e0
#define CSR_ANA2L_PXP_CDR1_PR_CKREF_DIV			GENMASK(1, 0)

#define REG_PCIE_CSR_ANA2L_PXP_CDR1_PR_COR_HBW		0x01e8
#define CSR_ANA2L_PXP_CDR1_PR_LDO_FORCE_ON		BIT(8)
#define CSR_ANA2L_PXP_CDR1_PR_CKREF_DIV1		GENMASK(17, 16)

#define REG_PCIE_CSR_ANA2L_PXP_CDR1_PR_MONPI		0x01ec
#define CSR_ANA2L_PXP_CDR1_PR_XFICK_EN			BIT(8)

#define REG_PCIE_CSR_ANA2L_PXP_RX1_DAC_RANGE_EYE	0x01f4
#define CSR_ANA2L_PXP_RX1_SIGDET_LPF_CTRL		GENMASK(25, 24)

#define REG_PCIE_CSR_ANA2L_PXP_RX1_SIGDET_NOVTH		0x01f8
#define CSR_ANA2L_PXP_RX1_SIGDET_PEAK			GENMASK(9, 8)
#define CSR_ANA2L_PXP_RX1_SIGDET_VTH_SEL		GENMASK(20, 16)

#define REG_PCIE_CSR_ANA2L_PXP_RX1_FE_VB_EQ1		0x0200
#define CSR_ANA2L_PXP_RX1_FE_VB_EQ1_EN			BIT(0)
#define CSR_ANA2L_PXP_RX1_FE_VB_EQ2_EN			BIT(8)
#define CSR_ANA2L_PXP_RX1_FE_VB_EQ3_EN			BIT(16)
#define CSR_ANA2L_PXP_RX1_FE_VCM_GEN_PWDB		BIT(24)

#define REG_PCIE_CSR_ANA2L_PXP_RX1_OSCAL_VGA1IOS	0x0214
#define CSR_ANA2L_PXP_RX1_PR_OSCAL_VGA1IOS		GENMASK(5, 0)
#define CSR_ANA2L_PXP_RX1_PR_OSCAL_VGA1VOS		GENMASK(13, 8)
#define CSR_ANA2L_PXP_RX1_PR_OSCAL_VGA2IOS		GENMASK(21, 16)

/* PMA */
#define REG_PCIE_SEQUENCE_DISB_CTRL1			0x010c
#define PCIE_DISB_RX_SDCAL_EN				BIT(0)

#define REG_PCIE_CTRL_SEQUENCE_FORCE_CTRL1		0x0114
#define PCIE_FORCE_RX_SDCAL_EN				BIT(0)

#define REG_PCIE_SS_RX_CAL1				0x0160
#define REG_PCIE_SS_RX_CAL2				0x0164
#define PCIE_CAL_OUT_OS					GENMASK(11, 8)

#define REG_PCIE_PMA_SS_RX_SIGDET0			0x0168
#define PCIE_SIGDET_WIN_NONVLD_TIMES			GENMASK(28, 24)

#define REG_PCIE_PMA_TX_RESET				0x0260
#define PCIE_TX_TOP_RST					BIT(0)
#define PCIE_TX_CAL_RST					BIT(8)

#define REG_PCIE_PMA_RX_FORCE_MODE0			0x0294
#define PCIE_FORCE_DA_XPON_RX_FE_GAIN_CTRL		GENMASK(1, 0)

#define REG_PCIE_PMA_SS_DA_XPON_PWDB0			0x034c
#define PCIE_DA_XPON_CDR_PR_PWDB			BIT(8)

#define REG_PCIE_PMA_SW_RESET				0x0460
#define PCIE_SW_RX_FIFO_RST				BIT(0)
#define PCIE_SW_RX_RST					BIT(1)
#define PCIE_SW_TX_RST					BIT(2)
#define PCIE_SW_PMA_RST					BIT(3)
#define PCIE_SW_ALLPCS_RST				BIT(4)
#define PCIE_SW_REF_RST					BIT(5)
#define PCIE_SW_TX_FIFO_RST				BIT(6)
#define PCIE_SW_XFI_RXPCS_RST				BIT(8)

#define REG_PCIE_PMA_FORCE_DA_PXP_CDR_PD_PWDB		0x081c
#define PCIE_FORCE_DA_PXP_CDR_PD_PWDB			BIT(0)
#define PCIE_FORCE_SEL_DA_PXP_CDR_PD_PWDB		BIT(8)

#define REG_PCIE_PMA_FORCE_PXP_JCPLL_CKOUT		0x0828
#define PCIE_FORCE_DA_PXP_JCPLL_EN			BIT(16)
#define PCIE_FORCE_SEL_DA_PXP_JCPLL_EN			BIT(24)

#define REG_PCIE_PMA_FORCE_DA_PXP_RX_SCAN_RST		0x0084c
#define PCIE_FORCE_DA_PXP_RX_SIGDET_PWDB		BIT(16)
#define PCIE_FORCE_SEL_DA_PXP_RX_SIGDET_PWDB		BIT(24)

#define REG_PCIE_PMA_DIG_RESERVE_14			0x08c0
#define PCIE_FLL_LOAD_EN				BIT(16)

#define REG_PCIE_PMA_FORCE_DA_PXP_RX_FE_GAIN_CTRL	0x088c
#define PCIE_FORCE_DA_PXP_RX_FE_GAIN_CTRL		GENMASK(1, 0)
#define PCIE_FORCE_SEL_DA_PXP_RX_FE_GAIN_CTRL		BIT(8)

#define REG_PCIE_PMA_FORCE_DA_PXP_RX_FE_PWDB		0x0894
#define PCIE_FORCE_DA_PXP_RX_FE_PWDB			BIT(0)
#define PCIE_FORCE_SEL_DA_PXP_RX_FE_PWDB		BIT(8)

#define REG_PCIE_PMA_DIG_RESERVE_12			0x08b8
#define PCIE_FORCE_PMA_RX_SPEED				GENMASK(7, 4)
#define PCIE_FORCE_SEL_PMA_RX_SPEED			BIT(7)

#define REG_PCIE_PMA_DIG_RESERVE_17			0x08e0

#define REG_PCIE_PMA_DIG_RESERVE_18			0x08e4
#define PCIE_PXP_RX_VTH_SEL_PCIE_G1			GENMASK(4, 0)
#define PCIE_PXP_RX_VTH_SEL_PCIE_G2			GENMASK(12, 8)
#define PCIE_PXP_RX_VTH_SEL_PCIE_G3			GENMASK(20, 16)

#define REG_PCIE_PMA_DIG_RESERVE_19			0x08e8
#define PCIE_PCP_RX_REV0_PCIE_GEN1			GENMASK(31, 16)

#define REG_PCIE_PMA_DIG_RESERVE_20			0x08ec
#define PCIE_PCP_RX_REV0_PCIE_GEN2			GENMASK(15, 0)
#define PCIE_PCP_RX_REV0_PCIE_GEN3			GENMASK(31, 16)

#define REG_PCIE_PMA_DIG_RESERVE_21			0x08f0
#define REG_PCIE_PMA_DIG_RESERVE_22			0x08f4
#define REG_PCIE_PMA_DIG_RESERVE_27			0x0908
#define REG_PCIE_PMA_DIG_RESERVE_30			0x0914

/**
 * struct airoha_pcie_phy - PCIe phy driver main structure
 * @dev: pointer to device
 * @phy: pointer to generic phy
 * @csr_ana2l: Analogic lane IO mapped register base address
 * @pma0: IO mapped register base address of PMA0-PCIe
 * @pma1: IO mapped register base address of PMA1-PCIe
 * @data: pointer to SoC dependent data
 */
struct airoha_pcie_phy {
	struct device *dev;
	struct phy *phy;
	void __iomem *csr_ana2l;
	void __iomem *pma0;
	void __iomem *pma1;
};

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

void Reg_W(struct airoha_pcie_phy *pcie_phy, u32 Base, u32 Offset, u32 Addr, u32 Data)
{		

	switch (Base)
	{
		case PCIE_ANA_2L:			
			writel(Data, (pcie_phy->csr_ana2l + (Addr - Offset))); //.h coda file 0x---, match api "PCIE_ANA_2L" + addr			
			break;
		case PCIE_PMA0:			
			writel(Data, (pcie_phy->pma0 + (Addr - Offset))); //.h coda file 0xb---, must - 0xb000 to match  "PCIE_PMA0" + addr			
			break;
		case PCIE_PMA1:
			writel(Data, (pcie_phy->pma1 + (Addr - Offset))); //.h coda file 0xb---, must  - 0xb000 to match  "PCIE_PMA1" + addr			
			break;
		default :
			printk("Write PCIe G3 PMA base addr error !\n");
			break;
	}
}

u32 Reg_R(struct airoha_pcie_phy *pcie_phy, u32 Base, u32 Offset, u32 Addr)
{
	u32 tmp;
	switch (Base)
	{
		case PCIE_ANA_2L:
			tmp = readl(pcie_phy->csr_ana2l + (Addr - Offset));
			break;
		case PCIE_PMA0:			
			tmp = readl(pcie_phy->pma0 + (Addr - Offset));			
			break;
		case PCIE_PMA1:
			tmp = readl(pcie_phy->pma1 + (Addr - Offset)); 			
			break;
		default :
			printk("Read PCIe G3 PMA base addr error !\n");
			tmp = 0xdeadbeef;
			break;
	}

	return tmp;	
}

void Reg_R_then_W(struct airoha_pcie_phy *pcie_phy, u32 Addr, u32 Data, char MSB, char LSB)
{
	u32 rdata, wdata, Base, rg_adr, write_mask;
	
	Base = Addr & 0xfffff000;
	rg_adr = Addr& 0xfff;
	rdata = Reg_R(pcie_phy, Base, 0, rg_adr);

	if((MSB == 31) &&( LSB == 0))
	{
		wdata = Data;		
	}else
	{
		write_mask = (1 << (MSB-LSB+1)) -1;
		wdata = (rdata & (~(write_mask << LSB))) | (Data << LSB);	
	}

	Reg_W(pcie_phy, Base, 0, rg_adr, wdata);
	#if 0//FLL_DEBUG
	printk("Write addr: %08x value: %08x \n", Addr, wdata);
	#endif

}

static void Rx_PR_Fw_Pre_Cal(struct airoha_pcie_phy *pcie_phy, char speed, char lane)
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

	
	Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b004, 1, 0, 0 ); //rg_lcpll_man_pwdb = 1

	Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b150, rg_lock_target_beg, 15, 0 ); //rg_lock_target_beg = rg_lock_target_beg

	Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b150, rg_lock_target_end, 31, 16 ); //rg_lock_target_end = rg_lock_target_end

	Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b14C, rg_lock_cyclecnt, 15, 0 ); //rg_lock_cyclecnt = rg_lock_cyclecnt

	Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b158, rg_lock_lockth, 11, 8 ); //rg_lock_lockth = rg_lock_lockth

	Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b154, rg_unlock_target_beg, 15, 0 ); //rg_unlock_target_beg = rg_unlock_target_beg

	Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b154, rg_unlock_target_end, 31, 16 ); //rg_unlock_target_end = rg_unlock_target_end

	Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b14C, rg_unlock_cyclecnt, 31, 16 ); //rg_unlock_cyclecnt = rg_unlock_cyclecnt

	Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b158, rg_unlockth, 15, 12 ); //rg_unlockth = rg_unlockth

	if (lane == 1)	
		Reg_R_then_W(pcie_phy, 0x1fa5a1d4, 1, 24, 24 ); //RG_PXP_CDR_PR_INJ_FORCE_OFF = 1
	else
		Reg_R_then_W(pcie_phy, 0x1fa5a11c, 1, 24, 24 ); //RG_PXP_CDR_PR_INJ_FORCE_OFF = 1


	Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b820, 1, 24, 24 ); //rg_force_sel_da_pxp_cdr_pr_lpf_r_en = 1

	Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b820, 1, 16, 16 ); //rg_force_da_pxp_cdr_pr_lpf_r_en = 1

	Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b820, 1, 8, 8 ); //rg_force_sel_da_pxp_cdr_pr_lpf_c_en = 1

	Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b820, 0, 0, 0 ); //rg_force_da_pxp_cdr_pr_lpf_c_en = 0

	Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b794, 1, 16, 16 ); //rg_force_sel_da_pxp_cdr_pr_idac = 1

	Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b824, 1, 24, 24 ); //rg_force_sel_da_pxp_cdr_pr_pwdb = 1

	Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b824, 0, 16, 16 ); //rg_force_da_pxp_cdr_pr_pwdb=1'b0 --> 1'b1 (PR Reset) = 0

	Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b824, 1, 16, 16 ); //rg_force_da_pxp_cdr_pr_pwdb=1'b0 --> 1'b1 (PR Reset) = 1
	
	RO_FL_Out_diff_tmp = 0xffff;
	cdr_pr_idac_tmp = 0;

	for (i = 0; i < 7; i++)
	{

	    pr_idac = Thermometer_Search[i];
	    
	    Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b794, Thermometer_Search[i] << 8, 10, 0 ); //rg_force_da_pxp_cdr_pr_idac[10:0]

	    Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b158, 0, 2, 0 ); //rg_freqlock_det_en=3'h3 = 0

	    Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b158, 3, 2, 0 ); //rg_freqlock_det_en=3'h3 = 3

	    mdelay(10);

	    // Read BG, ro_fl_out
	    //Addr = 0x1fa5b530;
	    //RO_FL_Out = DUT_RG_READ(dev_id, ref Addr, 31, 16);
	    RO_FL_Out = (Reg_R(pcie_phy, lane_offset_dig + 0x1fa5b000, 0, 0x530) >> 16) & 0xffff;

		
	    if ((RO_FL_Out > FL_Out_target) /*&& (RO_FL_Out_diff < RO_FL_Out_diff_tmp)*/)
	    {
	        RO_FL_Out_diff_tmp = RO_FL_Out_diff;
	        cdr_pr_idac_tmp = Thermometer_Search[i] << 8;
	    }
	}



	for (i = 7; i > -1; i--)
	{
	    tune_pr_idac_bit_position = i;
	    pr_idac = cdr_pr_idac_tmp | (0x1 << tune_pr_idac_bit_position);

	    Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b794, pr_idac, 10, 0 ); //rg_force_da_pxp_cdr_pr_idac[10:0] = pr_idac

	    Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b158, 0, 2, 0 ); //rg_freqlock_det_en=3'h3 = 0

	    Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b158, 3, 2, 0 ); //rg_freqlock_det_en=3'h3 = 3

	    mdelay(10);

	    // Read BG, ro_fl_out
	    RO_FL_Out = (Reg_R(pcie_phy, lane_offset_dig + 0x1fa5b000, 0, 0x530) >> 16) & 0xffff;
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

	Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b794, cdr_pr_idac_tmp, 10, 0 ); //rg_force_da_pxp_cdr_pr_idac[10:0] = cdr_pr_idac_tmp

	for (i = 0; i < 10; i++)
	{
	    Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b158, 0, 2, 0 ); //rg_freqlock_det_en=3'h3 = 0

	    Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b158, 3, 2, 0 ); //rg_freqlock_det_en=3'h3 = 3

	    mdelay(10);

	    RO_FL_Out = (Reg_R(pcie_phy, lane_offset_dig + 0x1fa5b000, 0, 0x530) >> 16) & 0xffff;
#if FLL_DEBUG
	    printk("selected cdr_pr_idac= %x\n", cdr_pr_idac_tmp);
#endif

	    // Read BG, RO_state_freqdet
	    RO_state_freqdet = Reg_R(pcie_phy, lane_offset_dig + 0x1fa5b000, 0, 0x530) & 0x1;
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
		Reg_R_then_W(pcie_phy, 0x1fa5a1d4, 0, 24, 24 ); //RG_PXP_CDR_PR_INJ_FORCE_OFF = 0
	else
		Reg_R_then_W(pcie_phy, 0x1fa5a11c, 0, 24, 24 ); //RG_PXP_CDR_PR_INJ_FORCE_OFF = 0


	Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b820, 0, 24, 24 ); //rg_force_sel_da_pxp_cdr_pr_lpf_r_en = 0

	Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b820, 0, 8, 8 ); //rg_force_sel_da_pxp_cdr_pr_lpf_c_en = 0

	
	Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b824, 0, 24, 24 ); //rg_force_sel_da_pxp_cdr_pr_pwdb=1'b0

	Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b794, 0, 16, 16 ); //rg_force_sel_da_pxp_cdr_pr_idac = 0

	if(speed == 3)
	{
		Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b8c0, cdr_pr_idac_tmp, 10, 0 ); //rg_fll_idac_pcieg3 = cdr_pr_idac_tmp
	}else
	{
		Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b8bc, cdr_pr_idac_tmp, 10, 0 ); //rg_fll_idac_pcieg1 = cdr_pr_idac_tmp
		Reg_R_then_W(pcie_phy, lane_offset_dig + 0x1fa5b8bc, cdr_pr_idac_tmp, 26, 16 ); //rg_fll_idac_pcieg2 = cdr_pr_idac_tmp
	}
		
	 printk("lane%d gen%d cdr_pr_idac_tmp= 0x%x\n", lane, speed, cdr_pr_idac_tmp);

}

static void airoha_phy_clear_bits(void __iomem *reg, u32 mask)
{
	u32 val = readl(reg);

	val &= ~mask;
	writel(val, reg);
}

static void airoha_phy_set_bits(void __iomem *reg, u32 mask)
{
	u32 val = readl(reg);

	val |= mask;
	writel(val, reg);
}

static void airoha_phy_update_bits(void __iomem *reg, u32 mask, u32 val)
{
	u32 tmp = readl(reg);

	tmp &= ~mask;
	tmp |= val & mask;
	writel(tmp, reg);
}

#define airoha_phy_update_field(reg, mask, val)					\
({										\
	BUILD_BUG_ON_MSG(!__builtin_constant_p(mask), "mask is not constant");	\
	airoha_phy_update_bits(reg, mask, FIELD_PREP(mask, val)); 		\
})

static void airoha_pcie_phy_init_set_default(struct airoha_pcie_phy *pcie_phy)
{
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_CMN,
				CSR_ANA2L_PXP_CMN_TRIM_MASK, 0x10);
	writel(0xcccbcccb, pcie_phy->pma0 + REG_PCIE_PMA_DIG_RESERVE_21);
	writel(0xcccb, pcie_phy->pma0 + REG_PCIE_PMA_DIG_RESERVE_22);
	writel(0xcccbcccb, pcie_phy->pma1 + REG_PCIE_PMA_DIG_RESERVE_21);
	writel(0xcccb, pcie_phy->pma1 + REG_PCIE_PMA_DIG_RESERVE_22);
	airoha_phy_set_bits(pcie_phy->csr_ana2l + REG_PCIE_CSR_ANA2L_PXP_CMN,
			    CSR_ANA2L_PXP_CMN_LANE_EN);
}

static void airoha_pcie_phy_init_clk_out(struct airoha_pcie_phy *pcie_phy)
{
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_TXPLL_POSTDIV_D256,
				CSR_ANA2L_PXP_CLKTX0_AMP, 0x5);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_CLKTX0_FORCE_OUT1,
				CSR_ANA2L_PXP_CLKTX1_AMP, 0x5);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_TXPLL_POSTDIV_D256,
				CSR_ANA2L_PXP_CLKTX0_OFFSET, 0x2);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_CLKTX1_OFFSET,
				CSR_ANA2L_PXP_CLKTX1_OFFSET, 0x2);
	airoha_phy_clear_bits(pcie_phy->csr_ana2l +
			      REG_PCIE_CSR_ANA2L_PXP_CLKTX0_FORCE_OUT1,
			      CSR_ANA2L_PXP_CLKTX0_HZ);
	airoha_phy_clear_bits(pcie_phy->csr_ana2l +
			      REG_PCIE_CSR_ANA2L_PXP_CLKTX1_OFFSET,
			      CSR_ANA2L_PXP_CLKTX1_HZ);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_CLKTX0_FORCE_OUT1,
				CSR_ANA2L_PXP_CLKTX0_IMP_SEL, 0x12);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_CLKTX1_IMP_SEL,
				CSR_ANA2L_PXP_CLKTX1_IMP_SEL, 0x12);
	airoha_phy_clear_bits(pcie_phy->csr_ana2l +
			      REG_PCIE_CSR_ANA2L_PXP_TXPLL_POSTDIV_D256,
			      CSR_ANA2L_PXP_CLKTX0_SR);
	airoha_phy_clear_bits(pcie_phy->csr_ana2l +
			      REG_PCIE_CSR_ANA2L_PXP_CLKTX1_OFFSET,
			      CSR_ANA2L_PXP_CLKTX1_SR);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_PLL_CMN_RESERVE0,
				CSR_ANA2L_PXP_PLL_RESERVE_MASK, 0xdd);
}

static void airoha_pcie_phy_init_csr_ana2l(struct airoha_pcie_phy *pcie_phy)
{
	airoha_phy_set_bits(pcie_phy->pma0 + REG_PCIE_PMA_SW_RESET,
			    PCIE_SW_XFI_RXPCS_RST | PCIE_SW_REF_RST |
			    PCIE_SW_RX_RST);
	airoha_phy_set_bits(pcie_phy->pma1 + REG_PCIE_PMA_SW_RESET,
			    PCIE_SW_XFI_RXPCS_RST | PCIE_SW_REF_RST |
			    PCIE_SW_RX_RST);
	airoha_phy_set_bits(pcie_phy->pma0 + REG_PCIE_PMA_TX_RESET,
			    PCIE_TX_TOP_RST | REG_PCIE_PMA_TX_RESET);
	airoha_phy_set_bits(pcie_phy->pma1 + REG_PCIE_PMA_TX_RESET,
			    PCIE_TX_TOP_RST | REG_PCIE_PMA_TX_RESET);
}

static void airoha_pcie_phy_init_rx(struct airoha_pcie_phy *pcie_phy)
{
	writel(0x2a00090b, pcie_phy->pma0 + REG_PCIE_PMA_DIG_RESERVE_17);
	writel(0x2a00090b, pcie_phy->pma1 + REG_PCIE_PMA_DIG_RESERVE_17);
	airoha_phy_set_bits(pcie_phy->csr_ana2l +
			    REG_PCIE_CSR_ANA2L_PXP_CDR0_PR_MONPI,
			    CSR_ANA2L_PXP_CDR0_PR_XFICK_EN);
	airoha_phy_set_bits(pcie_phy->csr_ana2l +
			    REG_PCIE_CSR_ANA2L_PXP_CDR1_PR_MONPI,
			    CSR_ANA2L_PXP_CDR1_PR_XFICK_EN);
	airoha_phy_clear_bits(pcie_phy->csr_ana2l +
			      REG_PCIE_CSR_ANA2L_PXP_CDR0_PD_PICAL_CKD8_INV,
			      CSR_ANA2L_PXP_CDR0_PD_EDGE_DISABLE);
	airoha_phy_clear_bits(pcie_phy->csr_ana2l +
			      REG_PCIE_CSR_ANA2L_PXP_CDR1_PD_PICAL_CKD8_INV,
			      CSR_ANA2L_PXP_CDR1_PD_EDGE_DISABLE);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_RX0_PHYCK_DIV,
				CSR_ANA2L_PXP_RX0_PHYCK_SEL, 0x1);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_RX1_PHYCK_DIV,
				CSR_ANA2L_PXP_RX1_PHYCK_SEL, 0x1);
}

static void airoha_pcie_phy_init_jcpll(struct airoha_pcie_phy *pcie_phy)
{
	airoha_phy_set_bits(pcie_phy->pma0 +
			    REG_PCIE_PMA_FORCE_PXP_JCPLL_CKOUT,
			    PCIE_FORCE_SEL_DA_PXP_JCPLL_EN);
	airoha_phy_clear_bits(pcie_phy->pma0 +
			      REG_PCIE_PMA_FORCE_PXP_JCPLL_CKOUT,
			      PCIE_FORCE_DA_PXP_JCPLL_EN);
	airoha_phy_set_bits(pcie_phy->pma1 +
			    REG_PCIE_PMA_FORCE_PXP_JCPLL_CKOUT,
			    PCIE_FORCE_SEL_DA_PXP_JCPLL_EN);
	airoha_phy_clear_bits(pcie_phy->pma1 +
			      REG_PCIE_PMA_FORCE_PXP_JCPLL_CKOUT,
			      PCIE_FORCE_DA_PXP_JCPLL_EN);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_JCPLL_TCL_VTP_EN,
				CSR_ANA2L_PXP_JCPLL_SPARE_LOW, 0x20);
	airoha_phy_set_bits(pcie_phy->csr_ana2l +
			    REG_PCIE_CSR_ANA2L_PXP_JCPLL_RST_DLY,
			    CSR_ANA2L_PXP_JCPLL_RST);
	writel(0x0, pcie_phy->csr_ana2l + REG_PCIE_CSR_ANA2L_JCPLL_SSC_DELTA1);
	airoha_phy_clear_bits(pcie_phy->csr_ana2l +
			      REG_PCIE_CSR_ANA2L_JCPLL_SSC_PERIOD,
			      CSR_ANA2L_PXP_JCPLL_SSC_PERIOD);

 RG_PXP_2L_JCPLL_SSC_EN.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SSC_EN);
 RG_PXP_2L_JCPLL_SSC_EN.hal.rg_pxp_2l_jcpll_ssc_phase_ini = 0x0; 
 RG_PXP_2L_JCPLL_SSC_EN.hal.rg_pxp_2l_jcpll_ssc_tri_en = 0x0; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SSC_EN, RG_PXP_2L_JCPLL_SSC_EN.dat.value);

 RG_PXP_2L_JCPLL_LPF_BR.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_LPF_BR);
 RG_PXP_2L_JCPLL_LPF_BR.hal.rg_pxp_2l_jcpll_lpf_br = 0xA; 
 RG_PXP_2L_JCPLL_LPF_BR.hal.rg_pxp_2l_jcpll_lpf_bp = 0xC; 
 RG_PXP_2L_JCPLL_LPF_BR.hal.rg_pxp_2l_jcpll_lpf_bc = 0x1F; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_LPF_BR, RG_PXP_2L_JCPLL_LPF_BR.dat.value);

 RG_PXP_2L_JCPLL_LPF_BWC.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_LPF_BWC);
 RG_PXP_2L_JCPLL_LPF_BWC.hal.rg_pxp_2l_jcpll_lpf_bwc = 0x1E; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_LPF_BWC, RG_PXP_2L_JCPLL_LPF_BWC.dat.value);

 RG_PXP_2L_JCPLL_LPF_BR.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_LPF_BR);
 RG_PXP_2L_JCPLL_LPF_BR.hal.rg_pxp_2l_jcpll_lpf_bwr = 0xA; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_LPF_BR, RG_PXP_2L_JCPLL_LPF_BR.dat.value);

 RG_PXP_2L_JCPLL_MMD_PREDIV_MODE.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_MMD_PREDIV_MODE);
 RG_PXP_2L_JCPLL_MMD_PREDIV_MODE.hal.rg_pxp_2l_jcpll_mmd_prediv_mode = 0x1; //Kaiwen improve jitter 10/9
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_MMD_PREDIV_MODE, RG_PXP_2L_JCPLL_MMD_PREDIV_MODE.dat.value);

 RG_PXP_2L_JCPLL_MONCK_EN.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_MONCK_EN);
 RG_PXP_2L_JCPLL_MONCK_EN.hal.rg_pxp_2l_jcpll_refin_div = 0x0; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_MONCK_EN, RG_PXP_2L_JCPLL_MONCK_EN.dat.value);

 rg_force_da_pxp_rx_fe_vos.dat.value = Reg_R(pcie_phy, PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_rx_fe_vos);
 rg_force_da_pxp_rx_fe_vos.hal.rg_force_sel_da_pxp_jcpll_sdm_pcw = 0x1; 
 Reg_W(pcie_phy, PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_rx_fe_vos, rg_force_da_pxp_rx_fe_vos.dat.value);

 rg_force_da_pxp_rx_fe_vos.dat.value = Reg_R(pcie_phy, PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_rx_fe_vos);
 rg_force_da_pxp_rx_fe_vos.hal.rg_force_sel_da_pxp_jcpll_sdm_pcw = 0x1; 
 Reg_W(pcie_phy, PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_rx_fe_vos, rg_force_da_pxp_rx_fe_vos.dat.value);

 rg_force_da_pxp_jcpll_sdm_pcw.dat.value = Reg_R(pcie_phy, PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_jcpll_sdm_pcw);
 rg_force_da_pxp_jcpll_sdm_pcw.hal.rg_force_da_pxp_jcpll_sdm_pcw = 0x50000000; //Kaiwen improve jitter 10/9
 Reg_W(pcie_phy, PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_jcpll_sdm_pcw, rg_force_da_pxp_jcpll_sdm_pcw.dat.value);

 rg_force_da_pxp_jcpll_sdm_pcw.dat.value = Reg_R(pcie_phy, PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_jcpll_sdm_pcw);
 rg_force_da_pxp_jcpll_sdm_pcw.hal.rg_force_da_pxp_jcpll_sdm_pcw = 0x50000000; //Kaiwen improve jitter 10/9
 Reg_W(pcie_phy, PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_jcpll_sdm_pcw, rg_force_da_pxp_jcpll_sdm_pcw.dat.value);

 RG_PXP_2L_JCPLL_MMD_PREDIV_MODE.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_MMD_PREDIV_MODE);
 RG_PXP_2L_JCPLL_MMD_PREDIV_MODE.hal.rg_pxp_2l_jcpll_postdiv_d5 = 0x1; 
 RG_PXP_2L_JCPLL_MMD_PREDIV_MODE.hal.rg_pxp_2l_jcpll_postdiv_d2 = 0x1; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_MMD_PREDIV_MODE, RG_PXP_2L_JCPLL_MMD_PREDIV_MODE.dat.value);

 RG_PXP_2L_JCPLL_RST_DLY.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_RST_DLY);
 RG_PXP_2L_JCPLL_RST_DLY.hal.rg_pxp_2l_jcpll_rst_dly = 0x4; 
 RG_PXP_2L_JCPLL_RST_DLY.hal.rg_pxp_2l_jcpll_sdm_di_ls = 0x0; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_RST_DLY, RG_PXP_2L_JCPLL_RST_DLY.dat.value);

 RG_PXP_2L_JCPLL_TCL_KBAND_VREF.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_TCL_KBAND_VREF);
 RG_PXP_2L_JCPLL_TCL_KBAND_VREF.hal.rg_pxp_2l_jcpll_vco_kband_meas_en = 0x0; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_TCL_KBAND_VREF, RG_PXP_2L_JCPLL_TCL_KBAND_VREF.dat.value);

 RG_PXP_2L_JCPLL_IB_EXT_EN.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_IB_EXT_EN);
 RG_PXP_2L_JCPLL_IB_EXT_EN.hal.rg_pxp_2l_jcpll_chp_iofst = 0x0; 
 RG_PXP_2L_JCPLL_IB_EXT_EN.hal.rg_pxp_2l_jcpll_chp_ibias = 0xC; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_IB_EXT_EN, RG_PXP_2L_JCPLL_IB_EXT_EN.dat.value);

 RG_PXP_2L_JCPLL_MMD_PREDIV_MODE.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_MMD_PREDIV_MODE);
 RG_PXP_2L_JCPLL_MMD_PREDIV_MODE.hal.rg_pxp_2l_jcpll_mmd_prediv_mode = 0x1; //Kaiwen improve jitter 10/9
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_MMD_PREDIV_MODE, RG_PXP_2L_JCPLL_MMD_PREDIV_MODE.dat.value);

 RG_PXP_2L_JCPLL_VCODIV.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_VCODIV);
 RG_PXP_2L_JCPLL_VCODIV.hal.rg_pxp_2l_jcpll_vco_halflsb_en = 0x1; //7/8 kaiwen add
 RG_PXP_2L_JCPLL_VCODIV.hal.rg_pxp_2l_jcpll_vco_cfix = 0x1; 
 RG_PXP_2L_JCPLL_VCODIV.hal.rg_pxp_2l_jcpll_vco_scapwr = 0x4; //7/8 kaiwen add
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_VCODIV, RG_PXP_2L_JCPLL_VCODIV.dat.value);

 RG_PXP_2L_JCPLL_IB_EXT_EN.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_IB_EXT_EN);
 RG_PXP_2L_JCPLL_IB_EXT_EN.hal.rg_pxp_2l_jcpll_lpf_shck_en = 0x0; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_IB_EXT_EN, RG_PXP_2L_JCPLL_IB_EXT_EN.dat.value);

 RG_PXP_2L_JCPLL_KBAND_KFC.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_KBAND_KFC);
 RG_PXP_2L_JCPLL_KBAND_KFC.hal.rg_pxp_2l_jcpll_postdiv_en = 0x1; 
 RG_PXP_2L_JCPLL_KBAND_KFC.hal.rg_pxp_2l_jcpll_kband_kfc = 0x0; 
 RG_PXP_2L_JCPLL_KBAND_KFC.hal.rg_pxp_2l_jcpll_kband_kf = 0x3; 
 RG_PXP_2L_JCPLL_KBAND_KFC.hal.rg_pxp_2l_jcpll_kband_ks = 0x0; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_KBAND_KFC, RG_PXP_2L_JCPLL_KBAND_KFC.dat.value);

 RG_PXP_2L_JCPLL_LPF_BWC.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_LPF_BWC);
 RG_PXP_2L_JCPLL_LPF_BWC.hal.rg_pxp_2l_jcpll_kband_div = 0x1; //Kaiwen improve jitter 10/9
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_LPF_BWC, RG_PXP_2L_JCPLL_LPF_BWC.dat.value);

 scan_mode.dat.value = Reg_R(pcie_phy, PCIE_PMA0, PMA_OFFSET, _scan_mode);
 scan_mode.hal.rg_force_sel_da_pxp_jcpll_kband_load_en = 0x1; //Lane0,Lane1 ????JCPLL
 scan_mode.hal.rg_force_da_pxp_jcpll_kband_load_en = 0x0; 
 Reg_W(pcie_phy, PCIE_PMA0, PMA_OFFSET, _scan_mode, scan_mode.dat.value);

 RG_PXP_2L_JCPLL_LPF_BWC.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_LPF_BWC);
 RG_PXP_2L_JCPLL_LPF_BWC.hal.rg_pxp_2l_jcpll_kband_code = 0xE4; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_LPF_BWC, RG_PXP_2L_JCPLL_LPF_BWC.dat.value);

 RG_PXP_2L_JCPLL_SDM_HREN.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SDM_HREN);
 RG_PXP_2L_JCPLL_SDM_HREN.hal.rg_pxp_2l_jcpll_tcl_amp_en = 0x1; //Add for 7/6 mail: ??ana??, kaiwen pll BW setting
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SDM_HREN, RG_PXP_2L_JCPLL_SDM_HREN.dat.value);

 RG_PXP_2L_JCPLL_TCL_CMP_EN.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_TCL_CMP_EN);
 RG_PXP_2L_JCPLL_TCL_CMP_EN.hal.rg_pxp_2l_jcpll_tcl_lpf_en = 0x1; //TCL????????????, ????????
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_TCL_CMP_EN, RG_PXP_2L_JCPLL_TCL_CMP_EN.dat.value);

 RG_PXP_2L_JCPLL_TCL_KBAND_VREF.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_TCL_KBAND_VREF);
 RG_PXP_2L_JCPLL_TCL_KBAND_VREF.hal.rg_pxp_2l_jcpll_tcl_kband_vref = 0xF; //Add for 7/6 mail: ??ana??, kaiwen pll BW setting
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_TCL_KBAND_VREF, RG_PXP_2L_JCPLL_TCL_KBAND_VREF.dat.value);

 RG_PXP_2L_JCPLL_SDM_HREN.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SDM_HREN);
 RG_PXP_2L_JCPLL_SDM_HREN.hal.rg_pxp_2l_jcpll_tcl_amp_gain = 0x1; //Add for 7/6 mail: ??ana??, kaiwen pll BW setting
 RG_PXP_2L_JCPLL_SDM_HREN.hal.rg_pxp_2l_jcpll_tcl_amp_vref = 0x5; //Add for 7/6 mail: ??ana??, kaiwen pll BW setting
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SDM_HREN, RG_PXP_2L_JCPLL_SDM_HREN.dat.value);

 RG_PXP_2L_JCPLL_TCL_CMP_EN.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_TCL_CMP_EN);
 RG_PXP_2L_JCPLL_TCL_CMP_EN.hal.rg_pxp_2l_jcpll_tcl_lpf_bw = 0x1; //update for 1/6 mail
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_TCL_CMP_EN, RG_PXP_2L_JCPLL_TCL_CMP_EN.dat.value);

 RG_PXP_2L_JCPLL_VCO_TCLVAR.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_VCO_TCLVAR);
 RG_PXP_2L_JCPLL_VCO_TCLVAR.hal.rg_pxp_2l_jcpll_vco_tclvar = 0x3; //update for 1/6 mail
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_VCO_TCLVAR, RG_PXP_2L_JCPLL_VCO_TCLVAR.dat.value); 

 rg_force_da_pxp_jcpll_ckout_en.dat.value = Reg_R(pcie_phy, PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_jcpll_ckout_en);
 rg_force_da_pxp_jcpll_ckout_en.hal.rg_force_sel_da_pxp_jcpll_ckout_en = 0x1; 
 rg_force_da_pxp_jcpll_ckout_en.hal.rg_force_da_pxp_jcpll_ckout_en = 0x1; 
 Reg_W(pcie_phy, PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_jcpll_ckout_en, rg_force_da_pxp_jcpll_ckout_en.dat.value);

 rg_force_da_pxp_jcpll_ckout_en.dat.value = Reg_R(pcie_phy, PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_jcpll_ckout_en);
 rg_force_da_pxp_jcpll_ckout_en.hal.rg_force_sel_da_pxp_jcpll_ckout_en = 0x1; 
 rg_force_da_pxp_jcpll_ckout_en.hal.rg_force_da_pxp_jcpll_ckout_en = 0x1; 
 Reg_W(pcie_phy, PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_jcpll_ckout_en, rg_force_da_pxp_jcpll_ckout_en.dat.value);

 rg_force_da_pxp_jcpll_ckout_en.dat.value = Reg_R(pcie_phy, PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_jcpll_ckout_en);
 rg_force_da_pxp_jcpll_ckout_en.hal.rg_force_sel_da_pxp_jcpll_en = 0x1; 
 rg_force_da_pxp_jcpll_ckout_en.hal.rg_force_da_pxp_jcpll_en = 0x1; 
 Reg_W(pcie_phy, PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_jcpll_ckout_en, rg_force_da_pxp_jcpll_ckout_en.dat.value);

 rg_force_da_pxp_jcpll_ckout_en.dat.value = Reg_R(pcie_phy, PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_jcpll_ckout_en);
 rg_force_da_pxp_jcpll_ckout_en.hal.rg_force_sel_da_pxp_jcpll_en = 0x1; 
 rg_force_da_pxp_jcpll_ckout_en.hal.rg_force_da_pxp_jcpll_en = 0x1; 
 Reg_W(pcie_phy, PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_jcpll_ckout_en, rg_force_da_pxp_jcpll_ckout_en.dat.value);
}

static void airoha_pcie_phy_txpll(struct airoha_pcie_phy *pcie_phy)
{
}

static void
airoha_pcie_phy_set_rxlan0_signal_detect(struct airoha_pcie_phy *pcie_phy)
{
	airoha_phy_set_bits(pcie_phy->csr_ana2l +
			    REG_PCIE_CSR_ANA2L_PXP_CDR0_PR_COR_HBW,
			    CSR_ANA2L_PXP_CDR0_PR_LDO_FORCE_ON);

	usleep_range(100, 200);

	airoha_phy_update_field(pcie_phy->pma0 + REG_PCIE_PMA_DIG_RESERVE_19,
				PCIE_PCP_RX_REV0_PCIE_GEN1, 0x18b0);
	airoha_phy_update_field(pcie_phy->pma0 + REG_PCIE_PMA_DIG_RESERVE_20,
				PCIE_PCP_RX_REV0_PCIE_GEN2, 0x18b0);
	airoha_phy_update_field(pcie_phy->pma0 + REG_PCIE_PMA_DIG_RESERVE_20,
				PCIE_PCP_RX_REV0_PCIE_GEN3, 0x1030);

	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_RX0_SIGDET_DCTEST,
				CSR_ANA2L_PXP_RX0_SIGDET_PEAK, 0x2);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_RX0_SIGDET_VTH_SEL,
				CSR_ANA2L_PXP_RX0_SIGDET_VTH_SEL, 0x5);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_RX0_REV0,
				CSR_ANA2L_PXP_VOS_PNINV, 0x2);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_RX0_SIGDET_DCTEST,
				CSR_ANA2L_PXP_RX0_SIGDET_LPF_CTRL, 0x1);

	airoha_phy_update_field(pcie_phy->pma0 + REG_PCIE_SS_RX_CAL2,
				PCIE_CAL_OUT_OS, 0x0);

	airoha_phy_set_bits(pcie_phy->csr_ana2l +
			    REG_PCIE_CSR_ANA2_PXP_RX0_FE_VB_EQ2,
			    CSR_ANA2L_PXP_RX0_FE_VCM_GEN_PWDB);

	airoha_phy_set_bits(pcie_phy->pma0 +
			    REG_PCIE_PMA_FORCE_DA_PXP_RX_FE_GAIN_CTRL,
			    PCIE_FORCE_SEL_DA_PXP_RX_FE_PWDB);
	airoha_phy_update_field(pcie_phy->pma0 +
				REG_PCIE_PMA_FORCE_DA_PXP_RX_FE_GAIN_CTRL,
				PCIE_FORCE_DA_PXP_RX_FE_GAIN_CTRL, 0x3);
	airoha_phy_update_field(pcie_phy->pma0 + REG_PCIE_PMA_RX_FORCE_MODE0,
				PCIE_FORCE_DA_XPON_RX_FE_GAIN_CTRL, 0x1);
	airoha_phy_update_field(pcie_phy->pma0 + REG_PCIE_PMA_SS_RX_SIGDET0,
				PCIE_SIGDET_WIN_NONVLD_TIMES, 0x3);
	airoha_phy_clear_bits(pcie_phy->pma0 + REG_PCIE_SEQUENCE_DISB_CTRL1,
			      PCIE_DISB_RX_SDCAL_EN);

	airoha_phy_set_bits(pcie_phy->pma0 +
			    REG_PCIE_CTRL_SEQUENCE_FORCE_CTRL1,
			    PCIE_FORCE_RX_SDCAL_EN);
	usleep_range(150, 200);
	airoha_phy_clear_bits(pcie_phy->pma0 +
			      REG_PCIE_CTRL_SEQUENCE_FORCE_CTRL1,
			      PCIE_FORCE_RX_SDCAL_EN);
}

static void
airoha_pcie_phy_set_rxlan1_signal_detect(struct airoha_pcie_phy *pcie_phy)
{
	airoha_phy_set_bits(pcie_phy->csr_ana2l +
			    REG_PCIE_CSR_ANA2L_PXP_CDR1_PR_COR_HBW,
			    CSR_ANA2L_PXP_CDR1_PR_LDO_FORCE_ON);

	usleep_range(100, 200);

	airoha_phy_update_field(pcie_phy->pma1 + REG_PCIE_PMA_DIG_RESERVE_19,
				PCIE_PCP_RX_REV0_PCIE_GEN1, 0x18b0);
	airoha_phy_update_field(pcie_phy->pma1 + REG_PCIE_PMA_DIG_RESERVE_20,
				PCIE_PCP_RX_REV0_PCIE_GEN2, 0x18b0);
	airoha_phy_update_field(pcie_phy->pma1 + REG_PCIE_PMA_DIG_RESERVE_20,
				PCIE_PCP_RX_REV0_PCIE_GEN3, 0x1030);

	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_RX1_SIGDET_NOVTH,
				CSR_ANA2L_PXP_RX1_SIGDET_PEAK, 0x2);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_RX1_SIGDET_NOVTH,
				CSR_ANA2L_PXP_RX1_SIGDET_VTH_SEL, 0x5);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_RX1_REV0,
				CSR_ANA2L_PXP_VOS_PNINV, 0x2);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_RX1_DAC_RANGE_EYE,
				CSR_ANA2L_PXP_RX1_SIGDET_LPF_CTRL, 0x1);

	airoha_phy_update_field(pcie_phy->pma1 + REG_PCIE_SS_RX_CAL2,
				PCIE_CAL_OUT_OS, 0x0);

	airoha_phy_set_bits(pcie_phy->csr_ana2l +
			    REG_PCIE_CSR_ANA2L_PXP_RX1_FE_VB_EQ1,
			    CSR_ANA2L_PXP_RX1_FE_VCM_GEN_PWDB);

	airoha_phy_set_bits(pcie_phy->pma1 +
			    REG_PCIE_PMA_FORCE_DA_PXP_RX_FE_GAIN_CTRL,
			    PCIE_FORCE_SEL_DA_PXP_RX_FE_PWDB);
	airoha_phy_update_field(pcie_phy->pma1 +
				REG_PCIE_PMA_FORCE_DA_PXP_RX_FE_GAIN_CTRL,
				PCIE_FORCE_DA_PXP_RX_FE_GAIN_CTRL, 0x3);
	airoha_phy_update_field(pcie_phy->pma1 + REG_PCIE_PMA_RX_FORCE_MODE0,
				PCIE_FORCE_DA_XPON_RX_FE_GAIN_CTRL, 0x1);
	airoha_phy_update_field(pcie_phy->pma1 + REG_PCIE_PMA_SS_RX_SIGDET0,
				PCIE_SIGDET_WIN_NONVLD_TIMES, 0x3);
	airoha_phy_clear_bits(pcie_phy->pma1 + REG_PCIE_SEQUENCE_DISB_CTRL1,
			      PCIE_DISB_RX_SDCAL_EN);

	airoha_phy_set_bits(pcie_phy->pma1 +
			    REG_PCIE_CTRL_SEQUENCE_FORCE_CTRL1,
			    PCIE_FORCE_RX_SDCAL_EN);
	usleep_range(150, 200);
	airoha_phy_clear_bits(pcie_phy->pma1 +
			      REG_PCIE_CTRL_SEQUENCE_FORCE_CTRL1,
			      PCIE_FORCE_RX_SDCAL_EN);
}

static void airoha_pcie_phy_set_rxflow(struct airoha_pcie_phy *pcie_phy)
{
	airoha_phy_set_bits(pcie_phy->pma0 +
			    REG_PCIE_PMA_FORCE_DA_PXP_RX_SCAN_RST,
			    PCIE_FORCE_DA_PXP_RX_SIGDET_PWDB |
			    PCIE_FORCE_SEL_DA_PXP_RX_SIGDET_PWDB);
	airoha_phy_set_bits(pcie_phy->pma1 +
			    REG_PCIE_PMA_FORCE_DA_PXP_RX_SCAN_RST,
			    PCIE_FORCE_DA_PXP_RX_SIGDET_PWDB |
			    PCIE_FORCE_SEL_DA_PXP_RX_SIGDET_PWDB);

	airoha_phy_set_bits(pcie_phy->pma0 +
			    REG_PCIE_PMA_FORCE_DA_PXP_CDR_PD_PWDB,
			    PCIE_FORCE_DA_PXP_CDR_PD_PWDB |
			    PCIE_FORCE_SEL_DA_PXP_CDR_PD_PWDB);
	airoha_phy_set_bits(pcie_phy->pma0 +
			    REG_PCIE_PMA_FORCE_DA_PXP_RX_FE_PWDB,
			    PCIE_FORCE_DA_PXP_RX_FE_PWDB |
			    PCIE_FORCE_SEL_DA_PXP_RX_FE_PWDB);
	airoha_phy_set_bits(pcie_phy->pma1 +
			    REG_PCIE_PMA_FORCE_DA_PXP_CDR_PD_PWDB,
			    PCIE_FORCE_DA_PXP_CDR_PD_PWDB |
			    PCIE_FORCE_SEL_DA_PXP_CDR_PD_PWDB);
	airoha_phy_set_bits(pcie_phy->pma1 +
			    REG_PCIE_PMA_FORCE_DA_PXP_RX_FE_PWDB,
			    PCIE_FORCE_DA_PXP_RX_FE_PWDB |
			    PCIE_FORCE_SEL_DA_PXP_RX_FE_PWDB);

	airoha_phy_set_bits(pcie_phy->csr_ana2l +
			    REG_PCIE_CSR_ANA2L_PXP_RX0_PHYCK_DIV,
			    CSR_ANA2L_PXP_RX0_PHYCK_RSTB |
			    CSR_ANA2L_PXP_RX0_TDC_CK_SEL);
	airoha_phy_set_bits(pcie_phy->csr_ana2l +
			    REG_PCIE_CSR_ANA2L_PXP_RX1_PHYCK_DIV,
			    CSR_ANA2L_PXP_RX1_PHYCK_RSTB |
			    CSR_ANA2L_PXP_RX1_TDC_CK_SEL);

	airoha_phy_set_bits(pcie_phy->pma0 + REG_PCIE_PMA_SW_RESET,
			    PCIE_SW_RX_FIFO_RST | PCIE_SW_TX_RST |
			    PCIE_SW_PMA_RST | PCIE_SW_ALLPCS_RST |
			    PCIE_SW_TX_FIFO_RST);
	airoha_phy_set_bits(pcie_phy->pma1 + REG_PCIE_PMA_SW_RESET,
			    PCIE_SW_RX_FIFO_RST | PCIE_SW_TX_RST |
			    PCIE_SW_PMA_RST | PCIE_SW_ALLPCS_RST |
			    PCIE_SW_TX_FIFO_RST);

	airoha_phy_set_bits(pcie_phy->csr_ana2l +
			    REG_PCIE_CSR_ANA2_PXP_RX0_FE_VB_EQ2,
			    CSR_ANA2L_PXP_RX0_FE_VB_EQ2_EN |
			    CSR_ANA2L_PXP_RX0_FE_VB_EQ3_EN);
	airoha_phy_set_bits(pcie_phy->csr_ana2l +
			    REG_PCIE_CSR_ANA2L_PXP_RX0_SIGDET_VTH_SEL,
			    CSR_ANA2L_PXP_RX0_FE_VB_EQ1_EN);
	airoha_phy_set_bits(pcie_phy->csr_ana2l +
			    REG_PCIE_CSR_ANA2L_PXP_RX1_FE_VB_EQ1,
			    CSR_ANA2L_PXP_RX1_FE_VB_EQ1_EN |
			    CSR_ANA2L_PXP_RX1_FE_VB_EQ2_EN |
			    CSR_ANA2L_PXP_RX1_FE_VB_EQ3_EN);

	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_RX0_REV0,
				CSR_ANA2L_PXP_FE_GAIN_NORMAL_MODE, 0x4);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_RX0_REV0,
				CSR_ANA2L_PXP_FE_GAIN_TRAIN_MODE, 0x4);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_RX1_REV0,
				CSR_ANA2L_PXP_FE_GAIN_NORMAL_MODE, 0x4);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_RX1_REV0,
				CSR_ANA2L_PXP_FE_GAIN_TRAIN_MODE, 0x4);
}

static void airoha_pcie_phy_set_pr(struct airoha_pcie_phy *pcie_phy)
{
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_CDR0_PR_VREG_IBAND,
				CSR_ANA2L_PXP_CDR0_PR_VREG_IBAND, 0x5);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_CDR0_PR_VREG_IBAND,
				CSR_ANA2L_PXP_CDR0_PR_VREG_CKBUF, 0x5);

	airoha_phy_clear_bits(pcie_phy->csr_ana2l +
			      REG_PCIE_CSR_ANA2L_PXP_CDR0_PR_CKREF_DIV,
			      CSR_ANA2L_PXP_CDR0_PR_CKREF_DIV);
	airoha_phy_clear_bits(pcie_phy->csr_ana2l +
			      REG_PCIE_CSR_ANA2L_PXP_CDR0_PR_COR_HBW,
			      CSR_ANA2L_PXP_CDR0_PR_CKREF_DIV1);

	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_CDR1_PR_VREG_IBAND_VAL,
				CSR_ANA2L_PXP_CDR1_PR_VREG_IBAND, 0x5);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_CDR1_PR_VREG_IBAND_VAL,
				CSR_ANA2L_PXP_CDR1_PR_VREG_CKBUF, 0x5);

	airoha_phy_clear_bits(pcie_phy->csr_ana2l +
			      REG_PCIE_CSR_ANA2L_PXP_CDR1_PR_CKREF_DIV,
			      CSR_ANA2L_PXP_CDR1_PR_CKREF_DIV);
	airoha_phy_clear_bits(pcie_phy->csr_ana2l +
			      REG_PCIE_CSR_ANA2L_PXP_CDR1_PR_COR_HBW,
			      CSR_ANA2L_PXP_CDR1_PR_CKREF_DIV1);

	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_CDR0_LPF_RATIO,
				CSR_ANA2L_PXP_CDR0_LPF_TOP_LIM, 0x20000);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_CDR1_LPF_RATIO,
				CSR_ANA2L_PXP_CDR1_LPF_TOP_LIM, 0x20000);

	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_CDR0_PR_BETA_DAC,
				CSR_ANA2L_PXP_CDR0_PR_BETA_SEL, 0x2);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_CDR1_PR_BETA_DAC,
				CSR_ANA2L_PXP_CDR1_PR_BETA_SEL, 0x2);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_CDR0_PR_BETA_DAC,
				CSR_ANA2L_PXP_CDR0_PR_KBAND_DIV, 0x4);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_CDR1_PR_BETA_DAC,
				CSR_ANA2L_PXP_CDR1_PR_KBAND_DIV, 0x4);
}

static void airoha_pcie_phy_set_txflow(struct airoha_pcie_phy *pcie_phy)
{
	airoha_phy_set_bits(pcie_phy->csr_ana2l +
			    REG_PCIE_CSR_ANA2L_PXP_TX0_CKLDO,
			    CSR_ANA2L_PXP_TX0_CKLDO_EN);
	airoha_phy_set_bits(pcie_phy->csr_ana2l +
			    REG_PCIE_CSR_ANA2L_PXP_TX1_CKLDO,
			    CSR_ANA2L_PXP_TX1_CKLDO_EN);

	airoha_phy_set_bits(pcie_phy->csr_ana2l +
			    REG_PCIE_CSR_ANA2L_PXP_TX0_CKLDO,
			    CSR_ANA2L_PXP_TX0_DMEDGEGEN_EN);
	airoha_phy_set_bits(pcie_phy->csr_ana2l +
			    REG_PCIE_CSR_ANA2L_PXP_TX1_CKLDO,
			    CSR_ANA2L_PXP_TX1_DMEDGEGEN_EN);
	airoha_phy_clear_bits(pcie_phy->csr_ana2l +
			      REG_PCIE_CSR_ANA2L_PXP_TX1_MULTLANE,
			      CSR_ANA2L_PXP_TX1_MULTLANE_EN);
}

static void airoha_pcie_phy_set_rx_mode(struct airoha_pcie_phy *pcie_phy)
{
	writel(0x804000, pcie_phy->pma0 + REG_PCIE_PMA_DIG_RESERVE_27);
	airoha_phy_update_field(pcie_phy->pma0 + REG_PCIE_PMA_DIG_RESERVE_18,
				PCIE_PXP_RX_VTH_SEL_PCIE_G1, 0x5);
	airoha_phy_update_field(pcie_phy->pma0 + REG_PCIE_PMA_DIG_RESERVE_18,
				PCIE_PXP_RX_VTH_SEL_PCIE_G2, 0x5);
	airoha_phy_update_field(pcie_phy->pma0 + REG_PCIE_PMA_DIG_RESERVE_18,
				PCIE_PXP_RX_VTH_SEL_PCIE_G3, 0x5);
	airoha_phy_set_bits(pcie_phy->pma0 + REG_PCIE_PMA_DIG_RESERVE_30,
			    0x77700);

	airoha_phy_clear_bits(pcie_phy->csr_ana2l +
			      REG_PCIE_CSR_ANA2L_PXP_CDR0_PR_MONCK,
			      CSR_ANA2L_PXP_CDR0_PR_MONCK_ENABLE);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_CDR0_PR_MONCK,
				CSR_ANA2L_PXP_CDR0_PR_RESERVE0, 0x2);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2_PXP_RX0_OSCAL_CTLE1IOS,
				CSR_ANA2L_PXP_RX0_PR_OSCAL_VGA1IOS, 0x19);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2_PXP_RX0_OSCA_VGA1VOS,
				CSR_ANA2L_PXP_RX0_PR_OSCAL_VGA1VOS, 0x19);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2_PXP_RX0_OSCA_VGA1VOS,
				CSR_ANA2L_PXP_RX0_PR_OSCAL_VGA2IOS, 0x14);

	writel(0x804000, pcie_phy->pma1 + REG_PCIE_PMA_DIG_RESERVE_27);
	airoha_phy_update_field(pcie_phy->pma1 + REG_PCIE_PMA_DIG_RESERVE_18,
				PCIE_PXP_RX_VTH_SEL_PCIE_G1, 0x5);
	airoha_phy_update_field(pcie_phy->pma1 + REG_PCIE_PMA_DIG_RESERVE_18,
				PCIE_PXP_RX_VTH_SEL_PCIE_G2, 0x5);
	airoha_phy_update_field(pcie_phy->pma1 + REG_PCIE_PMA_DIG_RESERVE_18,
				PCIE_PXP_RX_VTH_SEL_PCIE_G3, 0x5);

	airoha_phy_set_bits(pcie_phy->pma1 + REG_PCIE_PMA_DIG_RESERVE_30,
			    0x77700);

	airoha_phy_clear_bits(pcie_phy->csr_ana2l +
			      REG_PCIE_CSR_ANA2L_PXP_CDR1_PR_MONCK,
			      CSR_ANA2L_PXP_CDR1_PR_MONCK_ENABLE);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_CDR1_PR_MONCK,
				CSR_ANA2L_PXP_CDR1_PR_RESERVE0, 0x2);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_RX1_OSCAL_VGA1IOS,
				CSR_ANA2L_PXP_RX1_PR_OSCAL_VGA1IOS, 0x19);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_RX1_OSCAL_VGA1IOS,
				CSR_ANA2L_PXP_RX1_PR_OSCAL_VGA1VOS, 0x19);
	airoha_phy_update_field(pcie_phy->csr_ana2l +
				REG_PCIE_CSR_ANA2L_PXP_RX1_OSCAL_VGA1IOS,
				CSR_ANA2L_PXP_RX1_PR_OSCAL_VGA2IOS, 0x14);
}

static void airoha_pcie_phy_load_kflow(struct airoha_pcie_phy *pcie_phy)
{
	airoha_phy_update_field(pcie_phy->pma0 + REG_PCIE_PMA_DIG_RESERVE_12,
				PCIE_FORCE_PMA_RX_SPEED, 0xa);
	airoha_phy_update_field(pcie_phy->pma1 + REG_PCIE_PMA_DIG_RESERVE_12,
				PCIE_FORCE_PMA_RX_SPEED, 0xa);
	Rx_PR_Fw_Pre_Cal(pcie_phy, 3,0); 
	Rx_PR_Fw_Pre_Cal(pcie_phy, 3,1);

	airoha_phy_clear_bits(pcie_phy->pma0 + REG_PCIE_PMA_DIG_RESERVE_12,
			      PCIE_FORCE_PMA_RX_SPEED);
	airoha_phy_clear_bits(pcie_phy->pma1 + REG_PCIE_PMA_DIG_RESERVE_12,
			      PCIE_FORCE_PMA_RX_SPEED);
	usleep_range(100, 200);

	Rx_PR_Fw_Pre_Cal(pcie_phy, 2,0); 
	Rx_PR_Fw_Pre_Cal(pcie_phy, 2,1);
}

/**
 * airoha_pcie_phy_init() - Initialize the phy
 * @phy: the phy to be initialized
 *
 * Initialize the phy registers.
 * The hardware settings will be reset during suspend, it should be
 * reinitialized when the consumer calls phy_init() again on resume.
 */
static int airoha_pcie_phy_init(struct phy *phy)
{
	struct airoha_pcie_phy *pcie_phy = phy_get_drvdata(phy);

	/* enable load FLL-K flow */
	airoha_phy_set_bits(pcie_phy->pma0 + REG_PCIE_PMA_DIG_RESERVE_14,
				 PCIE_FLL_LOAD_EN);
	airoha_phy_set_bits(pcie_phy->pma1 + REG_PCIE_PMA_DIG_RESERVE_14,
				 PCIE_FLL_LOAD_EN);

	airoha_pcie_phy_init_set_default(pcie_phy);
	airoha_pcie_phy_init_clk_out(pcie_phy);
	airoha_pcie_phy_init_csr_ana2l(pcie_phy);

	usleep_range(100, 200);

	airoha_pcie_phy_init_rx(pcie_phy);
	/* phase 1, no ssc for K TXPLL */
	airoha_pcie_phy_init_jcpll(pcie_phy);

	usleep_range(500, 600);

	airoha_pcie_phy_txpll(pcie_phy);

 udelay(200);


//*******  TXPLL SETTING
 rg_force_da_pxp_txpll_ckout_en.dat.value = Reg_R(pcie_phy, PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_txpll_ckout_en);
 rg_force_da_pxp_txpll_ckout_en.hal.rg_force_sel_da_pxp_txpll_en = 0x1; 
 rg_force_da_pxp_txpll_ckout_en.hal.rg_force_da_pxp_txpll_en = 0x0; 
 Reg_W(pcie_phy, PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_txpll_ckout_en, rg_force_da_pxp_txpll_ckout_en.dat.value);

 rg_force_da_pxp_txpll_ckout_en.dat.value = Reg_R(pcie_phy, PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_txpll_ckout_en);
 rg_force_da_pxp_txpll_ckout_en.hal.rg_force_sel_da_pxp_txpll_en = 0x1; 
 rg_force_da_pxp_txpll_ckout_en.hal.rg_force_da_pxp_txpll_en = 0x0; 
 Reg_W(pcie_phy, PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_txpll_ckout_en, rg_force_da_pxp_txpll_ckout_en.dat.value);

 RG_PXP_2L_TXPLL_REFIN_DIV.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_REFIN_DIV);
 RG_PXP_2L_TXPLL_REFIN_DIV.hal.rg_pxp_2l_txpll_pll_rstb = 0x1; //Add for 3/22 mail: PLL FLOW ??
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_REFIN_DIV, RG_PXP_2L_TXPLL_REFIN_DIV.dat.value);

 RG_PXP_2L_TXPLL_SSC_DELTA1.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SSC_DELTA1);
 RG_PXP_2L_TXPLL_SSC_DELTA1.hal.rg_pxp_2l_txpll_ssc_delta = 0x0; 
 RG_PXP_2L_TXPLL_SSC_DELTA1.hal.rg_pxp_2l_txpll_ssc_delta1 = 0x0; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SSC_DELTA1, RG_PXP_2L_TXPLL_SSC_DELTA1.dat.value);

 RG_PXP_2L_TXPLL_SSC_PERIOD.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SSC_PERIOD);
 RG_PXP_2L_TXPLL_SSC_PERIOD.hal.rg_pxp_2l_txpll_ssc_period = 0x0; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SSC_PERIOD, RG_PXP_2L_TXPLL_SSC_PERIOD.dat.value);

 RG_PXP_2L_TXPLL_CHP_IOFST.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_CHP_IOFST);
 RG_PXP_2L_TXPLL_CHP_IOFST.hal.rg_pxp_2l_txpll_chp_iofst = 0x1; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_CHP_IOFST, RG_PXP_2L_TXPLL_CHP_IOFST.dat.value);

 RG_PXP_2L_750M_SYS_CK_EN.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_750M_SYS_CK_EN);
 RG_PXP_2L_750M_SYS_CK_EN.hal.rg_pxp_2l_txpll_chp_ibias = 0x2D; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_750M_SYS_CK_EN, RG_PXP_2L_750M_SYS_CK_EN.dat.value);

 RG_PXP_2L_TXPLL_REFIN_DIV.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_REFIN_DIV);
 RG_PXP_2L_TXPLL_REFIN_DIV.hal.rg_pxp_2l_txpll_refin_div = 0x0; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_REFIN_DIV, RG_PXP_2L_TXPLL_REFIN_DIV.dat.value);

 RG_PXP_2L_TXPLL_TCL_LPF_BW.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_LPF_BW);
 RG_PXP_2L_TXPLL_TCL_LPF_BW.hal.rg_pxp_2l_txpll_vco_cfix = 0x3; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_LPF_BW, RG_PXP_2L_TXPLL_TCL_LPF_BW.dat.value);

 rg_force_da_pxp_cdr_pr_idac.dat.value = Reg_R(pcie_phy, PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_idac);
 rg_force_da_pxp_cdr_pr_idac.hal.rg_force_sel_da_pxp_txpll_sdm_pcw = 0x1; 
 Reg_W(pcie_phy, PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_idac, rg_force_da_pxp_cdr_pr_idac.dat.value);

 rg_force_da_pxp_cdr_pr_idac.dat.value = Reg_R(pcie_phy, PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_idac);
 rg_force_da_pxp_cdr_pr_idac.hal.rg_force_sel_da_pxp_txpll_sdm_pcw = 0x1; 
 Reg_W(pcie_phy, PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_idac, rg_force_da_pxp_cdr_pr_idac.dat.value);

 rg_force_da_pxp_txpll_sdm_pcw.dat.value = Reg_R(pcie_phy, PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_txpll_sdm_pcw);
 rg_force_da_pxp_txpll_sdm_pcw.hal.rg_force_da_pxp_txpll_sdm_pcw = 0xC800000; 
 Reg_W(pcie_phy, PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_txpll_sdm_pcw, rg_force_da_pxp_txpll_sdm_pcw.dat.value);

 rg_force_da_pxp_txpll_sdm_pcw.dat.value = Reg_R(pcie_phy, PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_txpll_sdm_pcw);
 rg_force_da_pxp_txpll_sdm_pcw.hal.rg_force_da_pxp_txpll_sdm_pcw = 0xC800000; 
 Reg_W(pcie_phy, PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_txpll_sdm_pcw, rg_force_da_pxp_txpll_sdm_pcw.dat.value);

 RG_PXP_2L_TXPLL_SDM_DI_LS.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SDM_DI_LS);
 RG_PXP_2L_TXPLL_SDM_DI_LS.hal.rg_pxp_2l_txpll_sdm_ifm = 0x0; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SDM_DI_LS, RG_PXP_2L_TXPLL_SDM_DI_LS.dat.value);

 RG_PXP_2L_TXPLL_SSC_EN.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SSC_EN);
 RG_PXP_2L_TXPLL_SSC_EN.hal.rg_pxp_2l_txpll_ssc_phase_ini = 0x0; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SSC_EN, RG_PXP_2L_TXPLL_SSC_EN.dat.value);

 RG_PXP_2L_TXPLL_REFIN_DIV.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_REFIN_DIV);
 RG_PXP_2L_TXPLL_REFIN_DIV.hal.rg_pxp_2l_txpll_rst_dly = 0x4; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_REFIN_DIV, RG_PXP_2L_TXPLL_REFIN_DIV.dat.value);

 RG_PXP_2L_TXPLL_SDM_DI_LS.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SDM_DI_LS);
 RG_PXP_2L_TXPLL_SDM_DI_LS.hal.rg_pxp_2l_txpll_sdm_di_ls = 0x0; 
 RG_PXP_2L_TXPLL_SDM_DI_LS.hal.rg_pxp_2l_txpll_sdm_ord = 0x3; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SDM_DI_LS, RG_PXP_2L_TXPLL_SDM_DI_LS.dat.value);

 RG_PXP_2L_TXPLL_TCL_KBAND_VREF.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_KBAND_VREF);
 RG_PXP_2L_TXPLL_TCL_KBAND_VREF.hal.rg_pxp_2l_txpll_vco_kband_meas_en = 0x0; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_KBAND_VREF, RG_PXP_2L_TXPLL_TCL_KBAND_VREF.dat.value);

 RG_PXP_2L_TXPLL_SSC_DELTA1.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SSC_DELTA1);
 RG_PXP_2L_TXPLL_SSC_DELTA1.hal.rg_pxp_2l_txpll_ssc_delta = 0x0; 
 RG_PXP_2L_TXPLL_SSC_DELTA1.hal.rg_pxp_2l_txpll_ssc_delta1 = 0x0; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SSC_DELTA1, RG_PXP_2L_TXPLL_SSC_DELTA1.dat.value);

 RG_PXP_2L_TXPLL_CHP_IOFST.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_CHP_IOFST);
 RG_PXP_2L_TXPLL_CHP_IOFST.hal.rg_pxp_2l_txpll_lpf_bp = 0x1; 
 RG_PXP_2L_TXPLL_CHP_IOFST.hal.rg_pxp_2l_txpll_lpf_bc = 0x18; 
 RG_PXP_2L_TXPLL_CHP_IOFST.hal.rg_pxp_2l_txpll_lpf_br = 0x5; 
 RG_PXP_2L_TXPLL_CHP_IOFST.hal.rg_pxp_2l_txpll_chp_iofst = 0x1; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_CHP_IOFST, RG_PXP_2L_TXPLL_CHP_IOFST.dat.value);

 RG_PXP_2L_750M_SYS_CK_EN.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_750M_SYS_CK_EN);
 RG_PXP_2L_750M_SYS_CK_EN.hal.rg_pxp_2l_txpll_chp_ibias = 0x2D; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_750M_SYS_CK_EN, RG_PXP_2L_750M_SYS_CK_EN.dat.value);

 RG_PXP_2L_TXPLL_TCL_VTP_EN.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_VTP_EN);
 RG_PXP_2L_TXPLL_TCL_VTP_EN.hal.rg_pxp_2l_txpll_spare_l = 0x1; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_VTP_EN, RG_PXP_2L_TXPLL_TCL_VTP_EN.dat.value);

 RG_PXP_2L_TXPLL_LPF_BWR.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_LPF_BWR);
 RG_PXP_2L_TXPLL_LPF_BWR.hal.rg_pxp_2l_txpll_lpf_bwc = 0x0; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_LPF_BWR, RG_PXP_2L_TXPLL_LPF_BWR.dat.value);

 RG_PXP_2L_TXPLL_POSTDIV_EN.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_POSTDIV_EN);
 RG_PXP_2L_TXPLL_POSTDIV_EN.hal.rg_pxp_2l_txpll_mmd_prediv_mode = 0x0; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_POSTDIV_EN, RG_PXP_2L_TXPLL_POSTDIV_EN.dat.value);

 RG_PXP_2L_TXPLL_REFIN_DIV.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_REFIN_DIV);
 RG_PXP_2L_TXPLL_REFIN_DIV.hal.rg_pxp_2l_txpll_refin_div = 0x0; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_REFIN_DIV, RG_PXP_2L_TXPLL_REFIN_DIV.dat.value);

 RG_PXP_2L_TXPLL_TCL_LPF_BW.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_LPF_BW);
 RG_PXP_2L_TXPLL_TCL_LPF_BW.hal.rg_pxp_2l_txpll_vco_halflsb_en = 0x1; //7/8 kaiwen add
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_LPF_BW, RG_PXP_2L_TXPLL_TCL_LPF_BW.dat.value);

 RG_PXP_2L_TXPLL_VCO_SCAPWR.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_VCO_SCAPWR);
 RG_PXP_2L_TXPLL_VCO_SCAPWR.hal.rg_pxp_2l_txpll_vco_scapwr = 0x7; //7/8 kaiwen add
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_VCO_SCAPWR, RG_PXP_2L_TXPLL_VCO_SCAPWR.dat.value);

 RG_PXP_2L_TXPLL_TCL_LPF_BW.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_LPF_BW);
 RG_PXP_2L_TXPLL_TCL_LPF_BW.hal.rg_pxp_2l_txpll_vco_cfix = 0x3; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_LPF_BW, RG_PXP_2L_TXPLL_TCL_LPF_BW.dat.value);

 rg_force_da_pxp_cdr_pr_idac.dat.value = Reg_R(pcie_phy, PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_idac);
 rg_force_da_pxp_cdr_pr_idac.hal.rg_force_sel_da_pxp_txpll_sdm_pcw = 0x1; 
 Reg_W(pcie_phy, PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_idac, rg_force_da_pxp_cdr_pr_idac.dat.value);

 rg_force_da_pxp_cdr_pr_idac.dat.value = Reg_R(pcie_phy, PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_idac);
 rg_force_da_pxp_cdr_pr_idac.hal.rg_force_sel_da_pxp_txpll_sdm_pcw = 0x1; 
 Reg_W(pcie_phy, PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_cdr_pr_idac, rg_force_da_pxp_cdr_pr_idac.dat.value);

 RG_PXP_2L_TXPLL_SSC_EN.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SSC_EN);
 RG_PXP_2L_TXPLL_SSC_EN.hal.rg_pxp_2l_txpll_ssc_phase_ini = 0x0; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SSC_EN, RG_PXP_2L_TXPLL_SSC_EN.dat.value);

 RG_PXP_2L_TXPLL_LPF_BWR.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_LPF_BWR);
 RG_PXP_2L_TXPLL_LPF_BWR.hal.rg_pxp_2l_txpll_lpf_bwr = 0x0; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_LPF_BWR, RG_PXP_2L_TXPLL_LPF_BWR.dat.value);

 RG_PXP_2L_TXPLL_PHY_CK2_EN.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_PHY_CK2_EN);
 RG_PXP_2L_TXPLL_PHY_CK2_EN.hal.rg_pxp_2l_txpll_refin_internal = 0x1; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_PHY_CK2_EN, RG_PXP_2L_TXPLL_PHY_CK2_EN.dat.value);

 RG_PXP_2L_TXPLL_TCL_KBAND_VREF.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_KBAND_VREF);
 RG_PXP_2L_TXPLL_TCL_KBAND_VREF.hal.rg_pxp_2l_txpll_vco_kband_meas_en = 0x0; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_KBAND_VREF, RG_PXP_2L_TXPLL_TCL_KBAND_VREF.dat.value);

 RG_PXP_2L_TXPLL_VTP_EN.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_VTP_EN);
 RG_PXP_2L_TXPLL_VTP_EN.hal.rg_pxp_2l_txpll_vtp_en = 0x0; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_VTP_EN, RG_PXP_2L_TXPLL_VTP_EN.dat.value);

 RG_PXP_2L_TXPLL_POSTDIV_EN.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_POSTDIV_EN);
 RG_PXP_2L_TXPLL_POSTDIV_EN.hal.rg_pxp_2l_txpll_phy_ck1_en = 0x0; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_POSTDIV_EN, RG_PXP_2L_TXPLL_POSTDIV_EN.dat.value);

 RG_PXP_2L_TXPLL_PHY_CK2_EN.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_PHY_CK2_EN);
 RG_PXP_2L_TXPLL_PHY_CK2_EN.hal.rg_pxp_2l_txpll_refin_internal = 0x1; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_PHY_CK2_EN, RG_PXP_2L_TXPLL_PHY_CK2_EN.dat.value);

 RG_PXP_2L_TXPLL_SSC_EN.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SSC_EN);
 RG_PXP_2L_TXPLL_SSC_EN.hal.rg_pxp_2l_txpll_ssc_en = 0x0; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SSC_EN, RG_PXP_2L_TXPLL_SSC_EN.dat.value);

 RG_PXP_2L_750M_SYS_CK_EN.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_750M_SYS_CK_EN);
 RG_PXP_2L_750M_SYS_CK_EN.hal.rg_pxp_2l_txpll_lpf_shck_en = 0x0; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_750M_SYS_CK_EN, RG_PXP_2L_750M_SYS_CK_EN.dat.value);

 RG_PXP_2L_TXPLL_POSTDIV_EN.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_POSTDIV_EN);
 RG_PXP_2L_TXPLL_POSTDIV_EN.hal.rg_pxp_2l_txpll_postdiv_en = 0x0; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_POSTDIV_EN, RG_PXP_2L_TXPLL_POSTDIV_EN.dat.value);

 RG_PXP_2L_TXPLL_KBAND_DIV.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_KBAND_DIV);
 RG_PXP_2L_TXPLL_KBAND_DIV.hal.rg_pxp_2l_txpll_kband_kfc = 0x0; 
 RG_PXP_2L_TXPLL_KBAND_DIV.hal.rg_pxp_2l_txpll_kband_kf = 0x3; 
 RG_PXP_2L_TXPLL_KBAND_DIV.hal.rg_pxp_2l_txpll_kband_ks = 0x1; 
 RG_PXP_2L_TXPLL_KBAND_DIV.hal.rg_pxp_2l_txpll_kband_div = 0x4; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_KBAND_DIV, RG_PXP_2L_TXPLL_KBAND_DIV.dat.value);

 RG_PXP_2L_TXPLL_LPF_BWR.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_LPF_BWR);
 RG_PXP_2L_TXPLL_LPF_BWR.hal.rg_pxp_2l_txpll_kband_code = 0xE4; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_LPF_BWR, RG_PXP_2L_TXPLL_LPF_BWR.dat.value);

 RG_PXP_2L_TXPLL_SDM_OUT.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SDM_OUT);
 RG_PXP_2L_TXPLL_SDM_OUT.hal.rg_pxp_2l_txpll_tcl_amp_en = 0x1; //Add for 7/6 mail: ??ana??for TCL, kaiwen pll BW setting
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SDM_OUT, RG_PXP_2L_TXPLL_SDM_OUT.dat.value);

 RG_PXP_2L_TXPLL_TCL_AMP_VREF.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_AMP_VREF);
 RG_PXP_2L_TXPLL_TCL_AMP_VREF.hal.rg_pxp_2l_txpll_tcl_lpf_en = 0x1; //TCL????????????, ????????
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_AMP_VREF, RG_PXP_2L_TXPLL_TCL_AMP_VREF.dat.value);

 RG_PXP_2L_TXPLL_TCL_KBAND_VREF.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_KBAND_VREF);
 RG_PXP_2L_TXPLL_TCL_KBAND_VREF.hal.rg_pxp_2l_txpll_tcl_kband_vref = 0xF; //Add for 7/6 mail: ??ana??, kaiwen pll BW setting
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_KBAND_VREF, RG_PXP_2L_TXPLL_TCL_KBAND_VREF.dat.value);

 RG_PXP_2L_TXPLL_SDM_OUT.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SDM_OUT);
 RG_PXP_2L_TXPLL_SDM_OUT.hal.rg_pxp_2l_txpll_tcl_amp_gain = 0x3; //Add for 7/6 mail: ??ana??, kaiwen pll BW setting
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_SDM_OUT, RG_PXP_2L_TXPLL_SDM_OUT.dat.value);

 RG_PXP_2L_TXPLL_TCL_AMP_VREF.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_AMP_VREF);
 RG_PXP_2L_TXPLL_TCL_AMP_VREF.hal.rg_pxp_2l_txpll_tcl_amp_vref = 0xB; //Add for 7/6 mail: ??ana??, kaiwen pll BW setting
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_AMP_VREF, RG_PXP_2L_TXPLL_TCL_AMP_VREF.dat.value);

 RG_PXP_2L_TXPLL_TCL_LPF_BW.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_LPF_BW);
 RG_PXP_2L_TXPLL_TCL_LPF_BW.hal.rg_pxp_2l_txpll_tcl_lpf_bw = 0x3; //Add for 7/6 mail: ??ana??, kaiwen pll BW setting
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_TXPLL_TCL_LPF_BW, RG_PXP_2L_TXPLL_TCL_LPF_BW.dat.value);

 rg_force_da_pxp_txpll_ckout_en.dat.value = Reg_R(pcie_phy, PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_txpll_ckout_en);
 rg_force_da_pxp_txpll_ckout_en.hal.rg_force_sel_da_pxp_txpll_ckout_en = 0x1; 
 rg_force_da_pxp_txpll_ckout_en.hal.rg_force_da_pxp_txpll_ckout_en = 0x1; 
 Reg_W(pcie_phy, PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_txpll_ckout_en, rg_force_da_pxp_txpll_ckout_en.dat.value);

 rg_force_da_pxp_txpll_ckout_en.dat.value = Reg_R(pcie_phy, PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_txpll_ckout_en);
 rg_force_da_pxp_txpll_ckout_en.hal.rg_force_sel_da_pxp_txpll_ckout_en = 0x1; 
 rg_force_da_pxp_txpll_ckout_en.hal.rg_force_da_pxp_txpll_ckout_en = 0x1; 
 Reg_W(pcie_phy, PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_txpll_ckout_en, rg_force_da_pxp_txpll_ckout_en.dat.value);

 rg_force_da_pxp_txpll_ckout_en.dat.value = Reg_R(pcie_phy, PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_txpll_ckout_en);
 rg_force_da_pxp_txpll_ckout_en.hal.rg_force_sel_da_pxp_txpll_en = 0x1; 
 rg_force_da_pxp_txpll_ckout_en.hal.rg_force_da_pxp_txpll_en = 0x1; 
 Reg_W(pcie_phy, PCIE_PMA0, PMA_OFFSET, _rg_force_da_pxp_txpll_ckout_en, rg_force_da_pxp_txpll_ckout_en.dat.value);

 rg_force_da_pxp_txpll_ckout_en.dat.value = Reg_R(pcie_phy, PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_txpll_ckout_en);
 rg_force_da_pxp_txpll_ckout_en.hal.rg_force_sel_da_pxp_txpll_en = 0x1; 
 rg_force_da_pxp_txpll_ckout_en.hal.rg_force_da_pxp_txpll_en = 0x1; 
 Reg_W(pcie_phy, PCIE_PMA1, PMA_OFFSET, _rg_force_da_pxp_txpll_ckout_en, rg_force_da_pxp_txpll_ckout_en.dat.value);

 udelay(200);


//*******  SSC JCPLL SETTING
 RG_PXP_2L_JCPLL_SSC_DELTA1.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SSC_DELTA1);
 RG_PXP_2L_JCPLL_SSC_DELTA1.hal.rg_pxp_2l_jcpll_ssc_delta = 0x106; 
 RG_PXP_2L_JCPLL_SSC_DELTA1.hal.rg_pxp_2l_jcpll_ssc_delta1 = 0x106; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SSC_DELTA1, RG_PXP_2L_JCPLL_SSC_DELTA1.dat.value);

 RG_PXP_2L_JCPLL_SSC_PERIOD.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SSC_PERIOD);
 RG_PXP_2L_JCPLL_SSC_PERIOD.hal.rg_pxp_2l_jcpll_ssc_period = 0x31B; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SSC_PERIOD, RG_PXP_2L_JCPLL_SSC_PERIOD.dat.value);

 RG_PXP_2L_JCPLL_SSC_EN.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SSC_EN);
 RG_PXP_2L_JCPLL_SSC_EN.hal.rg_pxp_2l_jcpll_ssc_phase_ini = 0x1; 
 RG_PXP_2L_JCPLL_SSC_EN.hal.rg_pxp_2l_jcpll_ssc_en = 0x1; //enable SSC after TXPLL complete cal
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SSC_EN, RG_PXP_2L_JCPLL_SSC_EN.dat.value);

 RG_PXP_2L_JCPLL_SDM_IFM.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SDM_IFM);
 RG_PXP_2L_JCPLL_SDM_IFM.hal.rg_pxp_2l_jcpll_sdm_ifm = 0x1; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SDM_IFM, RG_PXP_2L_JCPLL_SDM_IFM.dat.value);

 RG_PXP_2L_JCPLL_SDM_HREN.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SDM_HREN);
 RG_PXP_2L_JCPLL_SDM_HREN.hal.rg_pxp_2l_jcpll_sdm_hren = 0x1; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SDM_HREN, RG_PXP_2L_JCPLL_SDM_HREN.dat.value);

 RG_PXP_2L_JCPLL_RST_DLY.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_RST_DLY);
 RG_PXP_2L_JCPLL_RST_DLY.hal.rg_pxp_2l_jcpll_sdm_di_en = 0x0; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_RST_DLY, RG_PXP_2L_JCPLL_RST_DLY.dat.value);

 RG_PXP_2L_JCPLL_SSC_EN.dat.value = Reg_R(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SSC_EN);
 RG_PXP_2L_JCPLL_SSC_EN.hal.rg_pxp_2l_jcpll_ssc_tri_en = 0x1; 
 Reg_W(pcie_phy, PCIE_ANA_2L, ANA_OFFSET, _RG_PXP_2L_JCPLL_SSC_EN, RG_PXP_2L_JCPLL_SSC_EN.dat.value);


	usleep_range(100, 200);

	/* Rx lan0 signal detect */
	airoha_pcie_phy_set_rxlan0_signal_detect(pcie_phy);
	/* Rx lan1 signal detect */
	airoha_pcie_phy_set_rxlan1_signal_detect(pcie_phy);
	/* RX FLOW */
	airoha_pcie_phy_set_rxflow(pcie_phy);

	usleep_range(100, 200);

	airoha_pcie_phy_set_pr(pcie_phy);
	/* TX FLOW */
	airoha_pcie_phy_set_txflow(pcie_phy);

	usleep_range(100, 200);
	/* RX mode setting */
	airoha_pcie_phy_set_rx_mode(pcie_phy);
	/* Load K-Flow */
	airoha_pcie_phy_load_kflow(pcie_phy);
	airoha_phy_clear_bits(pcie_phy->pma0 + REG_PCIE_PMA_SS_DA_XPON_PWDB0,
			      PCIE_DA_XPON_CDR_PR_PWDB);
	airoha_phy_clear_bits(pcie_phy->pma1 + REG_PCIE_PMA_SS_DA_XPON_PWDB0,
			      PCIE_DA_XPON_CDR_PR_PWDB);

	usleep_range(100, 200);

	airoha_phy_set_bits(pcie_phy->pma0 + REG_PCIE_PMA_SS_DA_XPON_PWDB0,
			    PCIE_DA_XPON_CDR_PR_PWDB);
	airoha_phy_set_bits(pcie_phy->pma1 + REG_PCIE_PMA_SS_DA_XPON_PWDB0,
			    PCIE_DA_XPON_CDR_PR_PWDB);

	usleep_range(100, 200);

	return 0;
}

static const struct phy_ops airoha_pcie_phy_ops = {
	.init = airoha_pcie_phy_init,
	.owner = THIS_MODULE,
};

static int airoha_pcie_phy_probe(struct platform_device *pdev)
{
	struct airoha_pcie_phy *pcie_phy;
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;

	pcie_phy = devm_kzalloc(dev, sizeof(*pcie_phy), GFP_KERNEL);
	if (!pcie_phy)
		return -ENOMEM;

	pcie_phy->csr_ana2l = devm_platform_ioremap_resource_byname(pdev, "ana2l");
	if (IS_ERR(pcie_phy->csr_ana2l))
		return dev_err_probe(dev, PTR_ERR(pcie_phy->csr_ana2l),
				     "Failed to map phy-ana2l base\n");

	pcie_phy->pma0 = devm_platform_ioremap_resource_byname(pdev, "pma0");
	if (IS_ERR(pcie_phy->pma0))
		return dev_err_probe(dev, PTR_ERR(pcie_phy->pma0),
				     "Failed to map phy-pma0 base\n");

	pcie_phy->pma1 = devm_platform_ioremap_resource_byname(pdev, "pma1");
	if (IS_ERR(pcie_phy->pma1))
		return dev_err_probe(dev, PTR_ERR(pcie_phy->pma1),
				     "Failed to map phy-pma1 base\n");

	pcie_phy->phy = devm_phy_create(dev, dev->of_node, &airoha_pcie_phy_ops);
	if (IS_ERR(pcie_phy->phy))
		return dev_err_probe(dev, PTR_ERR(pcie_phy->phy),
				     "Failed to create PCIe phy\n");

	pcie_phy->dev = dev;
	phy_set_drvdata(pcie_phy->phy, pcie_phy);

	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(provider))
		return dev_err_probe(dev, PTR_ERR(provider),
				     "PCIe phy probe failed\n");

	return 0;
}

static const struct of_device_id airoha_pcie_phy_of_match[] = {
	{ .compatible = "airoha,en7581-pcie-phy" },
	{ },
};
MODULE_DEVICE_TABLE(of, airoha_pcie_phy_of_match);

static struct platform_driver airoha_pcie_phy_driver = {
	.probe	= airoha_pcie_phy_probe,
	.driver	= {
		.name = "airoha-pcie-phy",
		.of_match_table = airoha_pcie_phy_of_match,
	},
};
module_platform_driver(airoha_pcie_phy_driver);

MODULE_DESCRIPTION("Airoha PCIe PHY driver");
MODULE_AUTHOR("Lorenzo Bianconi <lorenzo@kernel.org>");
MODULE_LICENSE("GPL");
