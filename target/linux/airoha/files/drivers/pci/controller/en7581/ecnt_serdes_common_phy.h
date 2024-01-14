#ifndef _COMMON_PHY
#define _COMMON_PHY
struct en7581_serdes_common_phy {
	struct device *dev;
	void __iomem *G3_ana2L_phy_rg_base; /* PCIEG3_PHY_PMA_PHYA physical address */
	void __iomem *G3_pma0_phy_rg_base; /* PCIEG3_PHY_PMA_PHYD_0 physical address */
	void __iomem *G3_pma1_phy_rg_base; /* PCIEG3_PHY_PMA_PHYD_1 physical address */
	void __iomem *xfi_ana_pxp_phy_rg_base; /* xfi_ana_pxp physical address */
	void __iomem *xfi_pma_phy_rg_base; /* xfi_pma physical address */
	void __iomem *pon_ana_pxp_phy_rg_base; /* pon_ana_pxp physical address */
	void __iomem *pon_pma_phy_rg_base; /* pon_pma physical address */
	void __iomem *multi_sgmii_base; /* multi_sgmii, PON_PHY_ASIC_RG range4,only for xilinx_fpga, PON_PHY_ASIC_RG range5,only for xilinx_fpga */
	
};
#endif
