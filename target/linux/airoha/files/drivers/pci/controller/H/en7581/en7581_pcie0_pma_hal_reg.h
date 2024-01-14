#ifndef EN7581_PCIE0_PMA_HAL_REG
#define EN7581_PCIE0_PMA_HAL_REG

typedef struct {
uint32 rg_lcpll_force_on :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_lcpll_hw_ctrl_mode :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_lcpll_en_inv :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_sw_lcpll_en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_LCPLL_PWCTL_SETTING_0, *pHAL_SS_LCPLL_PWCTL_SETTING_0;

typedef struct {
uint32 rg_lcpll_man_pwdb :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_lcpll_en_timer :8;
uint32 rg_lcpll_pcw_man_load_timer :8;
uint32 rg_lcpll_ck_stb_timer :8;
}HAL_SS_LCPLL_PWCTL_SETTING_1, *pHAL_SS_LCPLL_PWCTL_SETTING_1;

typedef struct {
uint32 rg_tdc_ck_stb_timer :8;
uint32 rg_tdc_mst_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_LCPLL_PWCTL_SETTING_2, *pHAL_SS_LCPLL_PWCTL_SETTING_2;

typedef struct {
uint32 rg_lcpll_ledck_div :4;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rsv_8 :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_LCPLL_PWCTL_DBG_SETTING, *pHAL_SS_LCPLL_PWCTL_DBG_SETTING;

typedef struct {
uint32 rg_lcpll_tdc_dig_pwdb :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_lcpll_dpma_autopw_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_lcpll_dpma_dds1_pwdb :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_lcpll_dpma_dds1_iso_disable :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_LCPLL_TDC_PW_0, *pHAL_SS_LCPLL_TDC_PW_0;

typedef struct {
uint32 rg_lcpll_dpma_dds1_iso_disable_dly :2;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_lcpll_dpma_dds1_rstb_dly :2;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_lcpll_dpma_dds1_rstb :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_lcpll_dpma_dds1_pcw_ncpo_chg :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_LCPLL_TDC_PW_1, *pHAL_SS_LCPLL_TDC_PW_1;

typedef struct {
uint32 rg_lcpll_dpma_dds1_pcw_ncpo_chg_dly :2;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_lcpll_dpma_dds1_ncpo_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_lcpll_dpma_dds1_ncpo_en_dly :2;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_lcpll_dpma_txpll1_ddsfbk_en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_LCPLL_TDC_PW_2, *pHAL_SS_LCPLL_TDC_PW_2;

typedef struct {
uint32 rg_lcpll_dpma_txpll1_ddsfbk_en_dly :2;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_lcpll_tdc_cal_trig :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_lcpll_tdc_cal_trig_dly :3;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_lcpll_tdc_cal :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_LCPLL_TDC_PW_3, *pHAL_SS_LCPLL_TDC_PW_3;

typedef struct {
uint32 rg_lcpll_tdc_cal_bw :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_lcpll_tdc_cal_offset :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_lcpll_tdc_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_lcpll_tdc_sigdet_cg :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_LCPLL_TDC_PW_4, *pHAL_SS_LCPLL_TDC_PW_4;

typedef struct {
uint32 rg_lcpll_tdc_en_mux :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_lcpll_tdc_autoen :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_lcpll_autok_tdc :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_lcpll_tdc_sync_in_mode :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_LCPLL_TDC_PW_5, *pHAL_SS_LCPLL_TDC_PW_5;

typedef struct {
uint32 rg_lcpll_pon_rx_cdr_divtdc :2;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_lcpll_ki :3;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_lcpll_tdc_offset :11;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_LCPLL_TDC_FLT_0, *pHAL_SS_LCPLL_TDC_FLT_0;

typedef struct {
uint32 rg_lcpll_gpon_sel :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_lcpll_a_tdc :4;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_lcpll_tdc_gain :11;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_LCPLL_TDC_FLT_1, *pHAL_SS_LCPLL_TDC_FLT_1;

typedef struct {
uint32 rg_lcpll_ncpo_value :31;
uint32 rsv_31 :1;
}HAL_SS_LCPLL_TDC_FLT_2, *pHAL_SS_LCPLL_TDC_FLT_2;

typedef struct {
uint32 rg_lcpll_ncpo_shift :2;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_lcpll_ncpo_load :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_lcpll_ncpo_hold :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_lcpll_tdc_swrst :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_LCPLL_TDC_FLT_3, *pHAL_SS_LCPLL_TDC_FLT_3;

typedef struct {
uint32 rg_lcpll_tdcen_rst_dlf :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_lcpll_tdc_rst :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_lcpll_tdcen_chk_lck2ref :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_lcpll_tdcen_wait_lck2ref :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_LCPLL_TDC_FLT_4, *pHAL_SS_LCPLL_TDC_FLT_4;

typedef struct {
uint32 rg_lcpll_tdc_waitlck_pre_dly :2;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_lcpll_tdc_waitlck_pos_dly :3;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_lcpll_tdc_autopw_ncpo :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_lcpll_ncpo_chg :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_LCPLL_TDC_FLT_5, *pHAL_SS_LCPLL_TDC_FLT_5;

typedef struct {
uint32 rg_lcpll_ncpo_chg_mask :5;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_lcpll_ncpo_chg_delay :2;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_lcpll_dlf_mult_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_lcpll_tdc_pcw_sub1en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_LCPLL_TDC_FLT_6, *pHAL_SS_LCPLL_TDC_FLT_6;

typedef struct {
uint32 rg_lcpll_tdcck_inv_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rsv_8 :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_LCPLL_TDC_FLT_7, *pHAL_SS_LCPLL_TDC_FLT_7;

typedef struct {
uint32 rg_lcpll_pon_hrdds_pcw_ncpo_gpon :31;
uint32 rsv_31 :1;
}HAL_SS_LCPLL_TDC_PCW_1, *pHAL_SS_LCPLL_TDC_PCW_1;

typedef struct {
uint32 rg_lcpll_pon_hrdds_pcw_ncpo_epon :31;
uint32 rsv_31 :1;
}HAL_SS_LCPLL_TDC_PCW_2, *pHAL_SS_LCPLL_TDC_PCW_2;

typedef struct {
uint32 ro_lcpll_tdc_tdcpw_st :4;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 ro_lcpll_tdc_st :5;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 ro_lcpll_tdc_dig :12;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_LCPLL_TDC_RO_1, *pHAL_SS_LCPLL_TDC_RO_1;

typedef struct {
uint32 ro_lcpll_tdc_dlf_gain :20;
uint32 ro_lcpll_tdc_dpma_dds1_pcw_ncpo_chg :1;
uint32 ro_lcpll_tdc_dpma_dds1_ncpo_en :1;
uint32 ro_lcpll_tdc_dpma_dds1_rstb :1;
uint32 ro_lcpll_tdc_dpma_dds1_iso_disable :1;
uint32 ro_lcpll_tdc_dpma_dds1_pwdb :1;
uint32 ro_lcpll_tdc_en_lck2ref_rst :1;
uint32 ro_lcpll_tdc_dpma_rx_ck_stb :1;
uint32 ro_lcpll_tdc_dpma_tdc_en :1;
uint32 ro_lcpll_tdc_dpma_tdc_cal_bw :1;
uint32 ro_lcpll_tdc_dpma_tdc_cal_offset :1;
uint32 ro_lcpll_tdc_dpma_txpll1_ddsfbk_en :1;
uint32 ro_lcpll_tdc_dpma_tdc_ck_stb :1;
}HAL_SS_LCPLL_TDC_RO_2, *pHAL_SS_LCPLL_TDC_RO_2;

typedef struct {
uint32 ro_lcpll_tdc_offset :11;
uint32 ro_lcpll_tdc_cal_done :1;
uint32 ro_lcpll_tdc_en :1;
uint32 ro_lcpll_tdc_decode :3;
uint32 ro_lcpll_tdc_gain :11;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_LCPLL_TDC_RO_3, *pHAL_SS_LCPLL_TDC_RO_3;

typedef struct {
uint32 ro_lcpll_tdc_dpma_dds1_pcw_ncpo :31;
uint32 rsv_31 :1;
}HAL_SS_LCPLL_TDC_RO_4, *pHAL_SS_LCPLL_TDC_RO_4;

typedef struct {
uint32 ro_lcpll_tdc_dpma_tdc_leadlag :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 ro_lcpll_tdc_dpma_tdc_d :8;
uint32 ro_lcpll_tdc_dpma_tdc_c :8;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_LCPLL_TDC_RO_5, *pHAL_SS_LCPLL_TDC_RO_5;

typedef struct {
uint32 rg_lcpll_ki_hold :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rsv_8 :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_LCPLL_TDC_CTRL_0, *pHAL_SS_LCPLL_TDC_CTRL_0;

typedef struct {
uint32 rg_x_min :11;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_x_max :11;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_EYE_TOP_EYEINDEX_CTRL_0, *pHAL_RX_EYE_TOP_EYEINDEX_CTRL_0;

typedef struct {
uint32 rg_y_min :7;
uint32 rsv_7 :1;
uint32 rg_y_max :7;
uint32 rsv_15 :1;
uint32 rg_index_mode :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_EYE_TOP_EYEINDEX_CTRL_1, *pHAL_RX_EYE_TOP_EYEINDEX_CTRL_1;

typedef struct {
uint32 rg_eyedur :20;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_EYE_TOP_EYEINDEX_CTRL_2, *pHAL_RX_EYE_TOP_EYEINDEX_CTRL_2;

typedef struct {
uint32 rg_eye_nextpts_sel :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_eye_nextpts_toggle :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_eye_nextpts :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_EYE_TOP_EYEINDEX_CTRL_3, *pHAL_RX_EYE_TOP_EYEINDEX_CTRL_3;

typedef struct {
uint32 rg_eyecnt_hth :8;
uint32 rg_eyecnt_vth :8;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_EYE_TOP_EYEOPENING_CTRL_0, *pHAL_RX_EYE_TOP_EYEOPENING_CTRL_0;

typedef struct {
uint32 rg_eo_hth :11;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_eo_vth :8;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_EYE_TOP_EYEOPENING_CTRL_1, *pHAL_RX_EYE_TOP_EYEOPENING_CTRL_1;

typedef struct {
uint32 rg_cntlen :10;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_cntforever :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_eye_mask :8;
}HAL_RX_EYE_TOP_EYECNT_CTRL_0, *pHAL_RX_EYE_TOP_EYECNT_CTRL_0;

typedef struct {
uint32 rg_disb_eyedur_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_disb_eyedur_init_b :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_eyedur_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_eyedur_init_b :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_EYE_TOP_EYECNT_CTRL_1, *pHAL_RX_EYE_TOP_EYECNT_CTRL_1;

typedef struct {
uint32 rg_eyecnt_fast :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_data_shift :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_EYE_TOP_EYECNT_CTRL_2, *pHAL_RX_EYE_TOP_EYECNT_CTRL_2;

typedef struct {
uint32 rg_osc_speed_opt :3;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_rx_os_start :16;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_CTRL_SEQUENCE_CTRL_0, *pHAL_RX_CTRL_SEQUENCE_CTRL_0;

typedef struct {
uint32 rg_rx_pical_start :16;
uint32 rg_rx_pical_end :16;
}HAL_RX_CTRL_SEQUENCE_CTRL_1, *pHAL_RX_CTRL_SEQUENCE_CTRL_1;

typedef struct {
uint32 rg_rx_pdos_start :16;
uint32 rg_rx_pdos_end :16;
}HAL_RX_CTRL_SEQUENCE_CTRL_2, *pHAL_RX_CTRL_SEQUENCE_CTRL_2;

typedef struct {
uint32 rg_rx_feos_start :16;
uint32 rg_rx_feos_end :16;
}HAL_RX_CTRL_SEQUENCE_CTRL_3, *pHAL_RX_CTRL_SEQUENCE_CTRL_3;

typedef struct {
uint32 rg_rx_sdcal_start :16;
uint32 rg_rx_sdcal_end :16;
}HAL_RX_CTRL_SEQUENCE_CTRL_4, *pHAL_RX_CTRL_SEQUENCE_CTRL_4;

typedef struct {
uint32 rg_rx_blwc_rdy_en :16;
uint32 rg_rx_rdy :16;
}HAL_RX_CTRL_SEQUENCE_CTRL_5, *pHAL_RX_CTRL_SEQUENCE_CTRL_5;

typedef struct {
uint32 rg_rx_os_end :16;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_CTRL_SEQUENCE_CTRL_6, *pHAL_RX_CTRL_SEQUENCE_CTRL_6;

typedef struct {
uint32 rg_disb_rx_os_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_disb_rx_pical_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_disb_rx_pdos_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_disb_rx_feos_en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_CTRL_SEQUENCE_DISB_CTRL_0, *pHAL_RX_CTRL_SEQUENCE_DISB_CTRL_0;

typedef struct {
uint32 rg_disb_rx_sdcal_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_disb_rx_os_rdy :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_disb_rx_blwc_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_disb_rx_rdy :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_CTRL_SEQUENCE_DISB_CTRL_1, *pHAL_RX_CTRL_SEQUENCE_DISB_CTRL_1;

typedef struct {
uint32 rg_force_rx_os_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_rx_pical_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_rx_pdos_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_rx_feos_en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_CTRL_SEQUENCE_FORCE_CTRL_0, *pHAL_RX_CTRL_SEQUENCE_FORCE_CTRL_0;

typedef struct {
uint32 rg_force_rx_sdcal_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_rx_os_rdy :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_rx_blwc_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_rx_rdy :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_CTRL_SEQUENCE_FORCE_CTRL_1, *pHAL_RX_CTRL_SEQUENCE_FORCE_CTRL_1;

typedef struct {
uint32 rg_eq_en_delay :8;
uint32 rg_heo_mask :11;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_veo_mask :8;
}HAL_PHY_EQ_CTRL_0, *pHAL_PHY_EQ_CTRL_0;

typedef struct {
uint32 rg_a_lgain :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_a_mgain :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_heo_emphasis :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_b_zero_sel :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_PHY_EQ_CTRL_1, *pHAL_PHY_EQ_CTRL_1;

typedef struct {
uint32 rg_a_sel :2;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_fom_num_order :5;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_eq_debug_sel :2;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_PHY_EQ_CTRL_2, *pHAL_PHY_EQ_CTRL_2;

typedef struct {
uint32 rg_fe_gain_ctrl_0 :2;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_fe_gain_ctrl_1 :2;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_fe_gain_ctrl_2 :2;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_fe_gain_ctrl_3 :2;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_PHY_EQ_CTRL_3, *pHAL_PHY_EQ_CTRL_3;

typedef struct {
uint32 rg_fe_gain_ctrl_4 :2;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_fe_gain_ctrl_5 :2;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_fe_gain_ctrl_6 :2;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_fe_gain_ctrl_7 :2;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_PHY_EQ_CTRL_4, *pHAL_PHY_EQ_CTRL_4;

typedef struct {
uint32 rg_fe_gain_ctrl_8 :2;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_fe_gain_ctrl_9 :2;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_fe_gain_ctrl_10 :2;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_fe_gain_ctrl_11 :2;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_PHY_EQ_CTRL_5, *pHAL_PHY_EQ_CTRL_5;

typedef struct {
uint32 rg_fe_gain_ctrl_12 :2;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_fe_gain_ctrl_13 :2;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_fe_gain_ctrl_14 :2;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_fe_gain_ctrl_15 :2;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_PHY_EQ_CTRL_6, *pHAL_PHY_EQ_CTRL_6;

typedef struct {
uint32 rg_fe_peaking_ctrl_0 :3;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_fe_peaking_ctrl_1 :3;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_fe_peaking_ctrl_2 :3;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_fe_peaking_ctrl_3 :3;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_PHY_EQ_CTRL_7, *pHAL_PHY_EQ_CTRL_7;

typedef struct {
uint32 rg_fe_peaking_ctrl_4 :3;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_fe_peaking_ctrl_5 :3;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_fe_peaking_ctrl_6 :3;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_fe_peaking_ctrl_7 :3;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_PHY_EQ_CTRL_8, *pHAL_PHY_EQ_CTRL_8;

typedef struct {
uint32 rg_fe_peaking_ctrl_8 :3;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_fe_peaking_ctrl_9 :3;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_fe_peaking_ctrl_10 :3;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_fe_peaking_ctrl_11 :3;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_PHY_EQ_CTRL_9, *pHAL_PHY_EQ_CTRL_9;

typedef struct {
uint32 rg_fe_peaking_ctrl_12 :3;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_fe_peaking_ctrl_13 :3;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_fe_peaking_ctrl_14 :3;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_fe_peaking_ctrl_15 :3;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_PHY_EQ_CTRL_10, *pHAL_PHY_EQ_CTRL_10;

typedef struct {
uint32 rg_lfsel :8;
uint32 rg_eq_force_blwc_freeze :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_RX_FEOS, *pHAL_SS_RX_FEOS;

typedef struct {
uint32 rg_eq_blwc_pol :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_eq_blwc_gain :4;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_eq_blwc_cnt_top_lim :7;
uint32 rg_eq_blwc_cnt_bot_lim :7;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_RX_BLWC, *pHAL_SS_RX_BLWC;

typedef struct {
uint32 rg_lock_cyclecnt :16;
uint32 rg_unlock_cyclecnt :16;
}HAL_SS_RX_FREQ_DET_1, *pHAL_SS_RX_FREQ_DET_1;

typedef struct {
uint32 rg_lock_target_beg :16;
uint32 rg_lock_target_end :16;
}HAL_SS_RX_FREQ_DET_2, *pHAL_SS_RX_FREQ_DET_2;

typedef struct {
uint32 rg_unlock_target_beg :16;
uint32 rg_unlock_target_end :16;
}HAL_SS_RX_FREQ_DET_3, *pHAL_SS_RX_FREQ_DET_3;

typedef struct {
uint32 rg_freqlock_det_en :3;
uint32 rsv_3 :1;
uint32 rg_lock_cnt_clear :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_lock_lockth :4;
uint32 rg_unlockth :4;
uint32 rg_wait_100us :8;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_RX_FREQ_DET_4, *pHAL_SS_RX_FREQ_DET_4;

typedef struct {
uint32 rg_cdrlpi :7;
uint32 rsv_7 :1;
uint32 rg_kpgain :3;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_eq_pi_cal_rdy_dly :2;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_RX_PI_CAL, *pHAL_SS_RX_PI_CAL;

typedef struct {
uint32 rg_sim_fast_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_cal_1us_set :8;
uint32 rg_cal_stb :2;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_cal_cyc :2;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_RX_CAL_1, *pHAL_SS_RX_CAL_1;

typedef struct {
uint32 rg_cal_os_pulse :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_cal_out_os :4;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_cal_cyc_time :2;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_RX_CAL_2, *pHAL_SS_RX_CAL_2;

typedef struct {
uint32 rg_sigdet_win_size :8;
uint32 rg_sigdet_vld_times :8;
uint32 rg_sigdet_win_vld_times :5;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_sigdet_win_nonvld_times :5;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_RX_SIGDET_0, *pHAL_SS_RX_SIGDET_0;

typedef struct {
uint32 rg_sigdet_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rsv_8 :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_RX_SIGDET_1, *pHAL_SS_RX_SIGDET_1;

typedef struct {
uint32 rg_kband_prediv :3;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_fpkdiv :11;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_kband_kfc :2;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_RX_FLL_0, *pHAL_SS_RX_FLL_0;

typedef struct {
uint32 rg_ipath_idac :11;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_settle_time_sel :3;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_symbol_wd :3;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_RX_FLL_1, *pHAL_SS_RX_FLL_1;

typedef struct {
uint32 rg_prbs_sel :3;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_amp :3;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_ck_rate :3;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_debug_sel :3;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_RX_FLL_2, *pHAL_SS_RX_FLL_2;

typedef struct {
uint32 rg_fll_dig_rst_force_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_fll_dig_rstb_force :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_icdr_kband_done_force_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_icdr_kband_done_force :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_RX_FLL_3, *pHAL_SS_RX_FLL_3;

typedef struct {
uint32 rg_fll_repli :1;
uint32 rg_fll_cor :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_fll_idac_repli :11;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_RX_FLL_4, *pHAL_SS_RX_FLL_4;

typedef struct {
uint32 rg_fll_idac_max :11;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_fll_idac_min :11;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_RX_FLL_5, *pHAL_SS_RX_FLL_5;

typedef struct {
uint32 ro_lnx_sw_fll_ro_1_latch_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 ro_lnx_sw_fll_ro_2_latch_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 ro_lnx_sw_fll_ro_3_latch_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 ro_lnx_sw_fll_ro_4_latch_en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_RX_FLL_6, *pHAL_SS_RX_FLL_6;

typedef struct {
uint32 ro_adc_freq :20;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_RX_FLL_7, *pHAL_SS_RX_FLL_7;

typedef struct {
uint32 ro_cor_gain :24;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_RX_FLL_8, *pHAL_SS_RX_FLL_8;

typedef struct {
uint32 ro_fll_idac :11;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 ro_idacf :11;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_RX_FLL_9, *pHAL_SS_RX_FLL_9;

typedef struct {
uint32 ro_da_idac :11;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 ro_fll_idac_repli :11;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_RX_FLL_a, *pHAL_SS_RX_FLL_a;

typedef struct {
uint32 rg_load_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_ad_sleep_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_RX_FLL_b, *pHAL_SS_RX_FLL_b;

typedef struct {
uint32 rg_data_blwc_add :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_eye_blwc_add :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_sap_sel :3;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_PDOS_CTRL_0, *pHAL_RX_PDOS_CTRL_0;

typedef struct {
uint32 rg_eq_blwc_rst_b :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_feos_rst_b :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_eq_pi_cal_rst_b :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_cal_rst_b :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_RESET_0, *pHAL_RX_RESET_0;

typedef struct {
uint32 rg_pdos_rst_b :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_sigdet_rst_b :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_unlock_stb :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_RESET_1, *pHAL_RX_RESET_1;

typedef struct {
uint32 rg_probe0_en :8;
uint32 rg_probe1_en :8;
uint32 rg_rx_debug_sel :5;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_ro_toggle :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_DEBUG_0, *pHAL_RX_DEBUG_0;

typedef struct {
uint32 rg_bistctl_pat_sel :5;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_bistctl_pat_tx_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_bistctl_pat_rx_check_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_bistctl_io_data_inv :2;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_BISTCTL_CONTROL, *pHAL_BISTCTL_CONTROL;

typedef struct {
uint32 bistctl_align_pattern :32;
}HAL_BISTCTL_ALIGN_PAT, *pHAL_BISTCTL_ALIGN_PAT;

typedef struct {
uint32 rg_bistctl_program_data_0 :32;
}HAL_BISTCTL_PROGRAM_PAT_0, *pHAL_BISTCTL_PROGRAM_PAT_0;

typedef struct {
uint32 rg_bistctl_program_data_1 :32;
}HAL_BISTCTL_PROGRAM_PAT_1, *pHAL_BISTCTL_PROGRAM_PAT_1;

typedef struct {
uint32 rg_bist_tx_data_pollution :16;
uint32 rg_bist_tx_data_pollution_latch :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_BISTCTL_POLLUTION, *pHAL_BISTCTL_POLLUTION;

typedef struct {
uint32 bistctl_prbs_init_seed :32;
}HAL_BISTCTL_PRBS_INITIAL_SEED, *pHAL_BISTCTL_PRBS_INITIAL_SEED;

typedef struct {
uint32 ro_bistctl_prbs_compare :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 ro_bistctl_prbs_fail :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 ro_bistctl_prbs_done :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_BISTCTL_PRBS_EVENT, *pHAL_BISTCTL_PRBS_EVENT;

typedef struct {
uint32 ro_bistctl_prbs_err_cnt :16;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_BISTCTL_PRBS_ERRCNT, *pHAL_BISTCTL_PRBS_ERRCNT;

typedef struct {
uint32 rg_bistctl_prbs_fail_threshold :16;
uint32 rg_bistctl_sq_wave_reach :4;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_BISTCTL_PRBS_FAIL_THRESHOLD, *pHAL_BISTCTL_PRBS_FAIL_THRESHOLD;

typedef struct {
uint32 ro_dac_do :7;
uint32 rsv_7 :1;
uint32 ro_dac_d1 :7;
uint32 rsv_15 :1;
uint32 ro_dac_eo :7;
uint32 rsv_23 :1;
uint32 ro_dac_e1 :7;
uint32 rsv_31 :1;
}HAL_RX_TORGS_DEBUG_0, *pHAL_RX_TORGS_DEBUG_0;

typedef struct {
uint32 ro_dac_eye :7;
uint32 rsv_7 :1;
uint32 ro_blwc_offset :7;
uint32 rsv_15 :1;
uint32 ro_blwc_sum :7;
uint32 rsv_23 :1;
uint32 ro_blwc_filter_din :7;
uint32 rsv_31 :1;
}HAL_RX_TORGS_DEBUG_1, *pHAL_RX_TORGS_DEBUG_1;

typedef struct {
uint32 ro_fl_out2 :16;
uint32 ro_pi_cal_data_out :7;
uint32 rsv_23 :1;
uint32 ro_cal_dir :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_TORGS_DEBUG_2, *pHAL_RX_TORGS_DEBUG_2;

typedef struct {
uint32 ro_cal_out :5;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 ro_cal_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 ro_lslock_cnt2 :4;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_TORGS_DEBUG_3, *pHAL_RX_TORGS_DEBUG_3;

typedef struct {
uint32 ro_heo :11;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 ro_veo :8;
uint32 eyecnt_rdy :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_TORGS_DEBUG_4, *pHAL_RX_TORGS_DEBUG_4;

typedef struct {
uint32 heo_center :11;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 heo_rdy :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 veo_rdy :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_TORGS_DEBUG_5, *pHAL_RX_TORGS_DEBUG_5;

typedef struct {
uint32 ro_cntlen :20;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_TORGS_DEBUG_6, *pHAL_RX_TORGS_DEBUG_6;

typedef struct {
uint32 eyecnt :20;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_TORGS_DEBUG_7, *pHAL_RX_TORGS_DEBUG_7;

typedef struct {
uint32 fom_num :15;
uint32 rsv_15 :1;
uint32 fom_num_type :5;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 fom_num_type_good :5;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_TORGS_DEBUG_8, *pHAL_RX_TORGS_DEBUG_8;

typedef struct {
uint32 rgs_ipll_dig_mon :6;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 ro_feos_out :6;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 eo_x_done :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 eo_y_done :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_TORGS_DEBUG_9, *pHAL_RX_TORGS_DEBUG_9;

typedef struct {
uint32 eye_er :11;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 eye_el :11;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_TORGS_DEBUG_10, *pHAL_RX_TORGS_DEBUG_10;

typedef struct {
uint32 tx_top_rst_b :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 txcalib_rst_b :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_TX_RST_B, *pHAL_SS_TX_RST_B;

typedef struct {
uint32 rg_txcalib_p_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_txcalib_force_termp_calen :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_txcalib_force_termp_sel_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_txcalib_force_termp_sel :2;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_TX_CALIB_0, *pHAL_SS_TX_CALIB_0;

typedef struct {
uint32 rg_txcalib_n_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_txcalib_force_termn_calen :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_txcalib_force_termn_sel_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_txcalib_force_termn_sel :2;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_TX_CALIB_1, *pHAL_SS_TX_CALIB_1;

typedef struct {
uint32 ro_txcal_out_pflag :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 ro_txcal_out_nflag :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 ro_txcal_fsm :3;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_txcal_clear :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_TX_CALIB_2, *pHAL_SS_TX_CALIB_2;

typedef struct {
uint32 rg_tx_dly_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_trans_burst_inv :1;
uint32 rg_trans_tx_data_inv :1;
uint32 rg_trans_rx_data_inv :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_trans_rx_sd_inv :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_trans_tx_fault_inv :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_XPON_SETTING_0, *pHAL_XPON_SETTING_0;

typedef struct {
uint32 rg_trans_tx_sd_inv :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_trans_tx_sd_pro :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_trans_rx_los_pro :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_trans_tx_fault_pro :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_XPON_SETTING_1, *pHAL_XPON_SETTING_1;

typedef struct {
uint32 rg_trans_rx_laser_on_pro :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_ftcal_low :8;
uint32 rg_ftcal_high :8;
uint32 rg_pma_show_probe_en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_XPON_SETTING_2, *pHAL_XPON_SETTING_2;

typedef struct {
uint32 ro_rx_los :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 ro_tx_sd :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 ro_tx_fault :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_XPON_STA, *pHAL_XPON_STA;

typedef struct {
uint32 rg_trans_rogue_onu_int_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_un_stb_os_rdy_int_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_phyrdy_int_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_trans_sd_fail_int_en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_XPON_INT_EN_0, *pHAL_XPON_INT_EN_0;

typedef struct {
uint32 rg_un_stb_eq_rdy_int_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_trans_fault_int_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_un_stb_rx_rdy_int_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_trans_los_int_en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_XPON_INT_EN_1, *pHAL_XPON_INT_EN_1;

typedef struct {
uint32 trans_rogue_onu_int :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 un_stb_os_rdy_int :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 phyrdy_int :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 trans_sd_fail_int :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_XPON_INT_STA_0, *pHAL_XPON_INT_STA_0;

typedef struct {
uint32 un_stb_eq_rdy_int :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 trans_fault_int :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 un_stb_rx_rdy_int :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 trans_los_int :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_XPON_INT_STA_1, *pHAL_XPON_INT_STA_1;

typedef struct {
uint32 eye_eu :7;
uint32 rsv_7 :1;
uint32 eye_eb :7;
uint32 rsv_15 :1;
uint32 ad_xpon_rx_dbg :8;
uint32 ad_xpon_rx_sigdet_out :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_TORGS_DEBUG_11, *pHAL_RX_TORGS_DEBUG_11;

typedef struct {
uint32 rg_force_da_xpon_rx_fe_gain_ctrl :2;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_da_xpon_rx_fe_peaking_ctrl :3;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_xpon_cdr_lpf_lck2data :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_da_xpon_cdr_lpf_rstb :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_FORCE_MODE_0, *pHAL_RX_FORCE_MODE_0;

typedef struct {
uint32 rg_force_da_xpon_rx_sigdet_os :5;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_da_xpon_rx_dac_d0 :7;
uint32 rsv_15 :1;
uint32 rg_force_da_xpon_rx_dac_d1 :7;
uint32 rsv_23 :1;
uint32 rg_force_da_xpon_rx_dac_e0 :7;
uint32 rsv_31 :1;
}HAL_RX_FORCE_MODE_1, *pHAL_RX_FORCE_MODE_1;

typedef struct {
uint32 rg_force_da_xpon_rx_dac_e1 :7;
uint32 rsv_7 :1;
uint32 rg_force_da_xpon_rx_dac_eye :7;
uint32 rsv_15 :1;
uint32 rg_force_da_xpon_rx_fe_vos :6;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_da_xpon_cdr_pr_pieye :7;
uint32 rsv_31 :1;
}HAL_RX_FORCE_MODE_2, *pHAL_RX_FORCE_MODE_2;

typedef struct {
uint32 rg_disb_da_xpon_rx_fe_gain_ctrl :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_disb_da_xpon_rx_fe_peaking_ctrl :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_disb_da_xpon_cdr_lpf_lck2data :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_disb_da_xpon_cdr_lpf_rstb :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_DISB_MODE_0, *pHAL_RX_DISB_MODE_0;

typedef struct {
uint32 rg_disb_da_xpon_rx_sigdet_os :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_disb_da_xpon_rx_dac_d0 :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_disb_da_xpon_rx_dac_d1 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_disb_da_xpon_rx_dac_e0 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_DISB_MODE_1, *pHAL_RX_DISB_MODE_1;

typedef struct {
uint32 rg_disb_da_xpon_rx_dac_e1 :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_disb_da_xpon_rx_dac_eye :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_disb_da_xpon_rx_fe_vos :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_disb_da_xpon_cdr_pr_pieye :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_DISB_MODE_2, *pHAL_RX_DISB_MODE_2;

typedef struct {
uint32 rg_force_eq_pi_cal_rdy :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_rx_dac_eye :7;
uint32 rsv_15 :1;
uint32 rg_force_rx_dac_d0 :7;
uint32 rsv_23 :1;
uint32 rg_force_rx_dac_d1 :7;
uint32 rsv_31 :1;
}HAL_RX_FORCE_MODE_3, *pHAL_RX_FORCE_MODE_3;

typedef struct {
uint32 rg_force_rx_dac_e0 :7;
uint32 rsv_7 :1;
uint32 rg_force_rx_dac_e1 :7;
uint32 rsv_15 :1;
uint32 rg_force_xpon_rx_sigdet_out :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_blwc_offset :7;
uint32 rsv_31 :1;
}HAL_RX_FORCE_MODE_4, *pHAL_RX_FORCE_MODE_4;

typedef struct {
uint32 rg_force_feos_out :6;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_eyecnt :20;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_FORCE_MODE_5, *pHAL_RX_FORCE_MODE_5;

typedef struct {
uint32 rg_force_eyecnt_rdy :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_rx_or_pical_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_FORCE_MODE_6, *pHAL_RX_FORCE_MODE_6;

typedef struct {
uint32 rg_disb_eq_pi_cal_rdy :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_disb_rx_dac_eye :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_disb_rx_dac_d0 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_disb_rx_dac_d1 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_DISB_MODE_3, *pHAL_RX_DISB_MODE_3;

typedef struct {
uint32 rg_disb_rx_dac_e0 :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_disb_rx_dac_e1 :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_disb_xpon_rx_sigdet_out :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_disb_blwc_offset :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_DISB_MODE_4, *pHAL_RX_DISB_MODE_4;

typedef struct {
uint32 rg_disb_feos_out :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_disb_eyecnt :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_disb_eyecnt_rdy :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_disb_rx_or_pical_en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_DISB_MODE_5, *pHAL_RX_DISB_MODE_5;

typedef struct {
uint32 rg_force_ref_and_pical_rstb :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_rx_and_pical_rstb :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_pdos_rx_rst_b :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_da_lock2ref :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_FORCE_MODE_7, *pHAL_RX_FORCE_MODE_7;

typedef struct {
uint32 rg_force_blwc_rx_rst_b :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sdcal_ref_rst_b :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_feos_rx_rst_b :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_eyecnt_rx_rst_b :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_FORCE_MODE_8, *pHAL_RX_FORCE_MODE_8;

typedef struct {
uint32 rg_force_fbck_lock :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_eye_reset_plu_o :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_eye_top_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_pi_cal_data_out_o :7;
uint32 rsv_31 :1;
}HAL_RX_FORCE_MODE_9, *pHAL_RX_FORCE_MODE_9;

typedef struct {
uint32 rg_disb_ref_and_pical_rstb :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_disb_rx_and_pical_rstb :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_disb_pdos_rx_rst_b :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_disb_da_lock2ref :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_DISB_MODE_6, *pHAL_RX_DISB_MODE_6;

typedef struct {
uint32 rg_disb_blwc_rx_rst_b :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_disb_sdcal_ref_rst_b :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_disb_feos_rx_rst_b :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_disb_eyecnt_rx_rst_b :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_DISB_MODE_7, *pHAL_RX_DISB_MODE_7;

typedef struct {
uint32 rg_disb_fbck_lock :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_disb_eye_reset_plu_o :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_disb_eye_top_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_disb_pi_cal_data_out_o :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_DISB_MODE_8, *pHAL_RX_DISB_MODE_8;

typedef struct {
uint32 dbgctl_bistctl_flagl_sel :8;
uint32 dbgctl_bistctl_flagh_sel :8;
uint32 dbgctl_bistctl_px_lnx_flagl_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 dbgctl_bistctl_px_lnx_flagh_en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_BIST_0, *pHAL_SS_BIST_0;

typedef struct {
uint32 anlt_px_lnx_lt_los :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 all_lane_prbs_tx_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 pllctl_px_lnx_tx_por_rdy :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_lnx_bistctl_bit_error_rst_sel :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_BIST_1, *pHAL_SS_BIST_1;

typedef struct {
uint32 rg_lnx_bistctl_prbs_check_timer :24;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_BIST_2, *pHAL_SS_BIST_2;

typedef struct {
uint32 rg_da_xpon_rx_fe_pwdb :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_da_xpon_cdr_pr_pwdb :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_da_xpon_cdr_pr_pieye_pwdb :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_da_xpon_cdr_pd_pwdb :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_DA_XPON_PWDB_0, *pHAL_SS_DA_XPON_PWDB_0;

typedef struct {
uint32 rg_da_xpon_rx_sigdet_pwdb :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rsv_8 :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_DA_XPON_PWDB_1, *pHAL_SS_DA_XPON_PWDB_1;

typedef struct {
uint32 eee_lcpll_hold :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 tdc_sync_pcw_ncpo_chg :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_LCPLL_0, *pHAL_SS_LCPLL_0;

typedef struct {
uint32 tdc_sync_pcw_ncpo :31;
uint32 rsv_31 :1;
}HAL_SS_LCPLL_1, *pHAL_SS_LCPLL_1;

typedef struct {
uint32 flagl_sel :8;
uint32 flagh_sel :8;
uint32 lcpll_top_flagl_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 lcpll_top_flagh_en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_LCPLL_2, *pHAL_SS_LCPLL_2;

typedef struct {
uint32 rg_dig_reserve_0 :32;
}HAL_ADD_DIG_RESERVE_0, *pHAL_ADD_DIG_RESERVE_0;

typedef struct {
uint32 rg_dig_reserve_1 :32;
}HAL_ADD_DIG_RESERVE_1, *pHAL_ADD_DIG_RESERVE_1;

typedef struct {
uint32 rg_dig_reserve_2 :32;
}HAL_ADD_DIG_RESERVE_2, *pHAL_ADD_DIG_RESERVE_2;

typedef struct {
uint32 rg_dig_reserve_3 :32;
}HAL_ADD_DIG_RESERVE_3, *pHAL_ADD_DIG_RESERVE_3;

typedef struct {
uint32 rg_dig_reserve_4 :32;
}HAL_ADD_DIG_RESERVE_4, *pHAL_ADD_DIG_RESERVE_4;

typedef struct {
uint32 rg_xpon_rx_rate_ctrl :2;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rsv_8 :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RG_XPON_RX_RESERVED_1, *pHAL_RG_XPON_RX_RESERVED_1;

typedef struct {
uint32 ro_dig_reserve_0 :32;
}HAL_ADD_DIG_RO_RESERVE_0, *pHAL_ADD_DIG_RO_RESERVE_0;

typedef struct {
uint32 ro_dig_reserve_1 :32;
}HAL_ADD_DIG_RO_RESERVE_1, *pHAL_ADD_DIG_RO_RESERVE_1;

typedef struct {
uint32 ro_dig_reserve_2 :32;
}HAL_ADD_DIG_RO_RESERVE_2, *pHAL_ADD_DIG_RO_RESERVE_2;

typedef struct {
uint32 ro_dig_reserve_3 :32;
}HAL_ADD_DIG_RO_RESERVE_3, *pHAL_ADD_DIG_RO_RESERVE_3;

typedef struct {
uint32 ro_dig_reserve_4 :32;
}HAL_ADD_DIG_RO_RESERVE_4, *pHAL_ADD_DIG_RO_RESERVE_4;

typedef struct {
uint32 rg_da_rx_sys_en_sel :2;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_rx_sys_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_ADD_RX_SYS_EN_SEL_0, *pHAL_ADD_RX_SYS_EN_SEL_0;

typedef struct {
uint32 rg_pll_lock_cyclecnt :16;
uint32 rg_pll_unlock_cyclecnt :16;
}HAL_PLL_TDC_FREQDET_0, *pHAL_PLL_TDC_FREQDET_0;

typedef struct {
uint32 rg_pll_lock_target_beg :16;
uint32 rg_pll_lock_target_end :16;
}HAL_PLL_TDC_FREQDET_1, *pHAL_PLL_TDC_FREQDET_1;

typedef struct {
uint32 rg_pll_unlock_target_beg :16;
uint32 rg_pll_unlock_target_end :16;
}HAL_PLL_TDC_FREQDET_2, *pHAL_PLL_TDC_FREQDET_2;

typedef struct {
uint32 rg_pll_freqlock_det_en :3;
uint32 rsv_3 :1;
uint32 rg_pll_lock_cnt_clear :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_pll_lock_lockth :4;
uint32 rg_pll_unlockth :4;
uint32 rg_pll_wait_100us :8;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_PLL_TDC_FREQDET_3, *pHAL_PLL_TDC_FREQDET_3;

typedef struct {
uint32 rg_disb_tx_ldo_lpf_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_tx_ldo_lpf_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_disb_tx_ck_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_tx_ck_en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_DA_XPON_TX_FORCE_0, *pHAL_DA_XPON_TX_FORCE_0;

typedef struct {
uint32 rg_disb_tx_hsdata_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_tx_hsdata_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_disb_da_xpon_tx_data :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_disb_da_xpon_tx_data_ben :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_DA_XPON_TX_FORCE_1, *pHAL_DA_XPON_TX_FORCE_1;

typedef struct {
uint32 rg_force_da_xpon_tx_data :16;
uint32 rg_force_da_xpon_tx_data_ben :16;
}HAL_DA_XPON_TX_FORCE_2, *pHAL_DA_XPON_TX_FORCE_2;

typedef struct {
uint32 rg_force_status_rx_rst_b :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_disb_status_rx_rst_b :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_FORCE_MODE_10, *pHAL_RX_FORCE_MODE_10;

typedef struct {
uint32 rg_clkpath_rst_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_clkpath_rstb_ck :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_tdc_lck2ref_sel :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_xpon_pma_probe_sel  :4;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_ADD_CLKPATH_RST_0, *pHAL_ADD_CLKPATH_RST_0;

typedef struct {
uint32 rg_xpon_mode :3;
uint32 rg_xfi_tx_mode :3;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_r2t_mode :1;
uint32 rg_xfi_rx_mode :3;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_tx_bist_gen_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_r2t_fifo_en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_ADD_XPON_MODE_1, *pHAL_ADD_XPON_MODE_1;

typedef struct {
uint32 rg_r2t_afifo_adj :4;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_r2t_asic_uses_ff :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_rx_afifo_adj :6;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_rx_asic_uses_ff :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_ADD_R2T_MODE_1, *pHAL_ADD_R2T_MODE_1;

typedef struct {
uint32 rg_t2r_fifo_en :1;
uint32 rg_xpon_t2r_bist_mode :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_t2r_afifo_adj :4;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_t2r_asic_uses_ff :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rg_tx_afifo_adj :5;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rg_tx_asic_uses_ff :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_ADD_T2R_MODE_1, *pHAL_ADD_T2R_MODE_1;

typedef struct {
uint32 ro_tdc_ck_stb :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 ro_lcpll_ck_stb :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 ro_pll_fbck_lock2 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 ro_rx_sequence :8;
}HAL_ADD_LCPLL_RO_1, *pHAL_ADD_LCPLL_RO_1;

typedef struct {
uint32 ro_rx_dac_eye :7;
uint32 rsv_7 :1;
uint32 ro_rx_dac_d0 :7;
uint32 rsv_15 :1;
uint32 ro_rx_dac_d1 :7;
uint32 rsv_23 :1;
uint32 ro_rx_dac_e0 :7;
uint32 rsv_31 :1;
}HAL_ADD_RO_RX2ANA_1, *pHAL_ADD_RO_RX2ANA_1;

typedef struct {
uint32 ro_rx_dac_e1 :7;
uint32 rsv_7 :1;
uint32 ro_rx_feos_out :6;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 ro_rx_blwc_offset :7;
uint32 rsv_23 :1;
uint32 ro_da_xpon_rx_sigdet_os :5;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_ADD_RO_RX2ANA_2, *pHAL_ADD_RO_RX2ANA_2;

typedef struct {
uint32 ro_rx_pi_cal_data_out :7;
uint32 rsv_7 :1;
uint32 ro_rx_fifo_empty_cnt :4;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 ro_rx_fifo_full_cnt :4;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_ADD_RO_RX2ANA_3, *pHAL_ADD_RO_RX2ANA_3;

typedef struct {
uint32 ro_r2t_data_afifo :16;
uint32 ro_r2t_fifo_full_cnt :4;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 ro_r2t_fifo_empty_cnt :4;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_ADD_RO_R2TMODE_1, *pHAL_ADD_RO_R2TMODE_1;

typedef struct {
uint32 rg_xpon_cmn_en_timer :16;
uint32 rg_xpon_cmn_man_pwdb :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_xpon_cmn_force_on :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RG_LCPLL_XPON_CMN_STB_CTRL_0, *pHAL_RG_LCPLL_XPON_CMN_STB_CTRL_0;

typedef struct {
uint32 rg_xpon_cmn_hw_ctrl_mode :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_xpon_cmn_inv :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_tx_d_toggle_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_tx_hsdata_en_ext_mode :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RG_LCPLL_XPON_CMN_STB_CTRL_1, *pHAL_RG_LCPLL_XPON_CMN_STB_CTRL_1;

typedef struct {
uint32 rg_tx_hsdata_en_cnt :32;
}HAL_RG_TX_HSDATA_EN_EXT_CNT_0, *pHAL_RG_TX_HSDATA_EN_EXT_CNT_0;

typedef struct {
uint32 rg_txcalib_50us :16;
uint32 rg_txcalib_5us :16;
}HAL_PON_TX_COUNTER_0, *pHAL_PON_TX_COUNTER_0;

typedef struct {
uint32 rg_tx_ck_en_wait :16;
uint32 rg_tx_hsdata_en_wait :16;
}HAL_PON_TX_COUNTER_1, *pHAL_PON_TX_COUNTER_1;

typedef struct {
uint32 rg_tx_power_on_wait :16;
uint32 rg_tx_serdes_rdy_wait :16;
}HAL_PON_TX_COUNTER_2, *pHAL_PON_TX_COUNTER_2;

typedef struct {
uint32 ro_tx_serdes_rdy :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rsv_8 :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_PON_TX_COUNTER_3, *pHAL_PON_TX_COUNTER_3;

typedef struct {
uint32 rg_force_bg_tx :3;
uint32 rsv_3 :1;
uint32 rg_force_bg_rx :2;
uint32 rsv_6 :1;
uint32 rg_force_xg_txpcsdiv2modesel :1;
uint32 rg_force_ge_txpcsmodesel :1;
uint32 rg_force_xg_txpcsmodesel :1;
uint32 rg_force_xe_txpcsmodesel :1;
uint32 rg_force_ge_rxpcsmodesel :1;
uint32 rg_force_xg_rxpcsmodesel :1;
uint32 rg_force_ge_txmapmodesel :1;
uint32 rg_force_xg_txmapmodesel :1;
uint32 rg_force_xe_txmapmodesel :1;
uint32 rg_force_ge_rxmapmodesel :1;
uint32 rg_force_xg_rxmapmodesel :1;
uint32 rg_force_g_mode :1;
uint32 rg_force_e_mode :1;
uint32 rg_force_ge_mode :1;
uint32 rg_force_xg_mode :1;
uint32 rg_force_xe_mode :1;
uint32 rg_force_xg_txmode :1;
uint32 rg_force_e_txmode :1;
uint32 rg_force_ck_g :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rg_rxck_dbg_inv :1;
uint32 rg_rxck_sync_inv :1;
uint32 rg_rxck_inv :1;
uint32 rg_txck_inv :1;
}HAL_PON_CK_SET, *pHAL_PON_CK_SET;

typedef struct {
uint32 rg_tx_afifo_en :1;
uint32 rg_tx_afifo_mode :1;
uint32 rg_xfi_phyck_inv :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rsv_8 :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_TX_FIFO_MODE_SEL, *pHAL_TX_FIFO_MODE_SEL;

typedef struct {
uint32 rg_xpon_pll_stb_cnt :32;
}HAL_XPON_PLL_STB_CNT, *pHAL_XPON_PLL_STB_CNT;

typedef struct {
uint32 rg_xpon_pll_stop_cnt :32;
}HAL_XPON_PLL_STOP_CNT, *pHAL_XPON_PLL_STOP_CNT;

typedef struct {
uint32 rg_sw_rx_fifo_rst_n :1;
uint32 rg_sw_rx_rst_n :1;
uint32 rg_sw_tx_rst_n :1;
uint32 rg_sw_pma_rst_n :1;
uint32 rg_sw_allpcs_rst_n :1;
uint32 rg_sw_ref_rst_n :1;
uint32 rg_sw_tx_fifo_rst_n :1;
uint32 rg_sw_xfi_txpcs_rst_n :1;
uint32 rg_sw_xfi_rxpcs_rst_n :1;
uint32 rg_sw_xfi_rxpcs_bist_rst_n :1;
uint32 rg_sw_hsg_txpcs_rst_n :1;
uint32 rg_sw_hsg_rxpcs_rst_n :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SW_RST_SET, *pHAL_SW_RST_SET;

typedef struct {
uint32 ro_tx_fifo_empty_cnt :4;
uint32 ro_tx_fifo_full_cnt :4;
uint32 rsv_8 :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_ADD_RO_TX2ANA_1, *pHAL_ADD_RO_TX2ANA_1;

typedef struct {
uint32 rg_tx_dly_data_ftune :7;
uint32 rsv_7 :1;
uint32 rg_tx_dly_ben_ftune :7;
uint32 rg_txben_pre_post_zero_en :1;
uint32 rg_tx_ben_exten_ftune :8;
uint32 rg_rx_non_reverse_epon :1;
uint32 rg_tx_non_reverse_epon :1;
uint32 rg_rx_non_reverse_gpon :1;
uint32 rg_tx_non_reverse_gpon :1;
uint32 rg_outben_data_mode :3;
uint32 rsv_31 :1;
}HAL_TX_DLY_CTRL, *pHAL_TX_DLY_CTRL;

typedef struct {
uint32 rg_up_md32_wrap_pd :1;
uint32 rg_gfifo_fec_mem_pd :1;
uint32 rg_tx_pma_fifo_pd :1;
uint32 rg_rx_pma_fifo_pd :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rsv_8 :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_MEM_WRAPPER_CTRL, *pHAL_MEM_WRAPPER_CTRL;

typedef struct {
uint32 rg_pma_tx_afifo_empty_int_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_pma_tx_afifo_full_int_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_pma_rx_afifo_empty_int_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_pma_rx_afifo_full_int_en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_XPON_INT_EN_2, *pHAL_XPON_INT_EN_2;

typedef struct {
uint32 rg_tx_freqdet_unlock_int_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_rx_freqdet_unlock_int_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_rx_sigdet_int_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_trans_laser_on_int_en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_XPON_INT_EN_3, *pHAL_XPON_INT_EN_3;

typedef struct {
uint32 pma_tx_afifo_empty_int :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 pma_tx_afifo_full_int :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 pma_rx_afifo_empty_int :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 pma_rx_afifo_full_int :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_XPON_INT_STA_2, *pHAL_XPON_INT_STA_2;

typedef struct {
uint32 tx_freqdet_unlock_int :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rx_freqdet_unlock_int :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rx_sigdet_int :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 trans_laser_on_int :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_XPON_INT_STA_3, *pHAL_XPON_INT_STA_3;

typedef struct {
uint32 rg_trans_sd_fail_cnt :16;
uint32 rg_trans_rogue_onu_cnt :16;
}HAL_XPON_SD_BEHAVIOR_SETTING_0, *pHAL_XPON_SD_BEHAVIOR_SETTING_0;

typedef struct {
uint32 rg_trans_sd_fail_cnt_clr :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_trans_rogue_onu_cnt_clr :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_XPON_SD_BEHAVIOR_SETTING_1, *pHAL_XPON_SD_BEHAVIOR_SETTING_1;

typedef struct {
uint32 ro_trans_sd_fail_cnt :16;
uint32 ro_trans_rogue_onu_cnt :16;
}HAL_XPON_SD_BEHAVIOR_STA, *pHAL_XPON_SD_BEHAVIOR_STA;

typedef struct {
uint32 rg_disb_leq :1;
uint32 rg_os_rdy_latch :1;
uint32 rg_heo_rdy_opt :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_l2d_trig_eq_en_time :8;
uint32 rg_blwc_opt :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_abnormal_cnt :3;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_EXTRAL_CTRL, *pHAL_RX_EXTRAL_CTRL;

typedef struct {
uint32 rg_disb_leq_len_ctrl :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_disb_leq_pf_ctrl :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_disb_leq_leak_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_disb_leq_avg_en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_LEQ_DISB_CTRL0, *pHAL_RX_LEQ_DISB_CTRL0;

typedef struct {
uint32 rg_disb_leq_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_disb_leq_done :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_disb_leq_rst_b :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_LEQ_DISB_CTRL1, *pHAL_RX_LEQ_DISB_CTRL1;

typedef struct {
uint32 rg_leq_len_ctrl :4;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_leq_pf_ctrl :4;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_leq_leak_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_leq_avg_en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_LEQ_FORCE_CTRL0, *pHAL_RX_LEQ_FORCE_CTRL0;

typedef struct {
uint32 rg_leq_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_leq_done :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_leq_rst_b :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_abnormal_cnt_reset_disb :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_LEQ_FORCE_CTRL1, *pHAL_RX_LEQ_FORCE_CTRL1;

typedef struct {
uint32 rg_entime_leq_len_ctrl0 :4;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_entime_leq_len_ctrl1 :4;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_entime_leq_len_ctrl2 :4;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_entime_leq_len_ctrl3 :4;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_LEQ_ENTIME_CTRL0, *pHAL_RX_LEQ_ENTIME_CTRL0;

typedef struct {
uint32 rg_entime_leq_pf_ctrl0 :4;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_entime_leq_pf_ctrl1 :4;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_entime_leq_pf_ctrl2 :4;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_entime_leq_pf_ctrl3 :4;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_LEQ_ENTIME_CTRL1, *pHAL_RX_LEQ_ENTIME_CTRL1;

typedef struct {
uint32 rg_entime_leq_avg :4;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_disentime_leq_leak_en :4;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_timeslot_res :2;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_entime_res :2;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_LEQ_ENTIME_CTRL2, *pHAL_RX_LEQ_ENTIME_CTRL2;

typedef struct {
uint32 rg_leq_update_pause :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_leq_gain :4;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_leq_init_val :4;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_leq_init_set :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_LEQ_CTRL0, *pHAL_RX_LEQ_CTRL0;

typedef struct {
uint32 rg_leq_top_lim :4;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_leq_bot_lim :4;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_leq_leak_gain :4;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_leq_leak_target :4;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_LEQ_CTRL1, *pHAL_RX_LEQ_CTRL1;

typedef struct {
uint32 rg_leq_avg_cycle :4;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_leq_gray_out_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_LEQ_CTRL2, *pHAL_RX_LEQ_CTRL2;

typedef struct {
uint32 ro_leq_rst_b :1;
uint32 ro_leq_en :1;
uint32 ro_eq_done :1;
uint32 ro_eq_en :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 ro_leq_len_ctrl :4;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 ro_leq_pf_ctrl :4;
uint32 ro_leq_leak_en :1;
uint32 ro_leq_avg_en :1;
uint32 ro_leq_updn :2;
uint32 ro_eq_rdy :2;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 ro_abnormal_cnt :3;
uint32 rsv_31 :1;
}HAL_RX_LEQ_RO0, *pHAL_RX_LEQ_RO0;

typedef struct {
uint32 ro_leq_time_slot :4;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 ro_eq_time_slot :4;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_LEQ_RO1, *pHAL_RX_LEQ_RO1;

typedef struct {
uint32 rg_pll_ft_lock_cyclecnt :16;
uint32 rg_pll_ft_unlock_cyclecnt :16;
}HAL_PLL_FT_FREQDET_0, *pHAL_PLL_FT_FREQDET_0;

typedef struct {
uint32 rg_pll_ft_lock_target_beg :16;
uint32 rg_pll_ft_lock_target_end :16;
}HAL_PLL_FT_FREQDET_1, *pHAL_PLL_FT_FREQDET_1;

typedef struct {
uint32 rg_pll_ft_unlock_target_beg :16;
uint32 rg_pll_ft_unlock_target_end :16;
}HAL_PLL_FT_FREQDET_2, *pHAL_PLL_FT_FREQDET_2;

typedef struct {
uint32 rg_pll_ft_freqlock_det_en :3;
uint32 rsv_3 :1;
uint32 rg_pll_ft_lock_cnt_clear :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_pll_ft_lock_lockth :4;
uint32 rg_pll_ft_unlockth :4;
uint32 rg_pll_ft_wait_100us :8;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_PLL_FT_FREQDET_3, *pHAL_PLL_FT_FREQDET_3;

typedef struct {
uint32 ro_fbck_lock :1;
uint32 ro_fbck_lock_inner :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 ro_lslock_cnt :4;
uint32 ro_state_freqdet :2;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 ro_fl_out :16;
}HAL_RO_RX_FREQDET, *pHAL_RO_RX_FREQDET;

typedef struct {
uint32 ro_pll_fbck_lock :1;
uint32 ro_pll_fbck_lock_inner :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 ro_pll_lslock_cnt :4;
uint32 ro_pll_state_freqdet :2;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 ro_pll_fl_out :16;
}HAL_RO_PLL_FREQDET, *pHAL_RO_PLL_FREQDET;

typedef struct {
uint32 ro_pll_ft_fbck_lock :1;
uint32 ro_pll_ft_fbck_lock_inner :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 ro_pll_ft_lslock_cnt :4;
uint32 ro_pll_ft_state_freqdet :2;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 ro_pll_ft_fl_out :16;
}HAL_RO_PLL_FT_FREQDET, *pHAL_RO_PLL_FT_FREQDET;

typedef struct {
uint32 ro_pma_freq_cnt_max :16;
uint32 ro_pma_freq_cnt_lsb :16;
}HAL_RO_PMA_FREQDET, *pHAL_RO_PMA_FREQDET;

typedef struct {
uint32 rg_pma_freq_meter_sel :4;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_pma_freq_check_lsb :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RG_PMA_FREQDET, *pHAL_RG_PMA_FREQDET;

typedef struct {
uint32 rg_tx_loff_manu :32;
}HAL_RG_EXT_BEN_DATA, *pHAL_RG_EXT_BEN_DATA;

typedef struct {
uint32 rg_tx_lon_manu :16;
uint32 rg_tx_bh_dl_dsel :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RG_PRE_BEN_DATA, *pHAL_RG_PRE_BEN_DATA;

typedef struct {
uint32 ro_x_index_o :11;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 ro_y_index_o :7;
uint32 rsv_23 :1;
uint32 ro_kband_done :1;
uint32 ro_injosc_rdy :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RX_TORGS_DEBUG_12, *pHAL_RX_TORGS_DEBUG_12;

typedef struct {
uint32 ro_adc_freq_output :20;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rg_fll_dig_pro :1;
}HAL_RO_FLL_ADC_0, *pHAL_RO_FLL_ADC_0;

typedef struct {
uint32 ro_adc_freq_inv_output :20;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RO_FLL_ADC_1, *pHAL_RO_FLL_ADC_1;

typedef struct {
uint32 ro_cor_gain_output :25;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RO_FLL_ADC_2, *pHAL_RO_FLL_ADC_2;

typedef struct {
uint32 ro_cor_integ_output :24;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RO_FLL_ADC_3, *pHAL_RO_FLL_ADC_3;

typedef struct {
uint32 ro_cor_integ_inv_output :24;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RO_FLL_ADC_4, *pHAL_RO_FLL_ADC_4;

typedef struct {
uint32 rg_ad_xpon_pll_ft_ck_mon_mux_sel :2;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rsv_8 :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RG_AD_XPON_PLL_FT_CK_MON_MUX_SEL, *pHAL_RG_AD_XPON_PLL_FT_CK_MON_MUX_SEL;

typedef struct {
uint32 rg_pma_tx_afifo_reach_int_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_pma_rx_afifo_reach_int_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_XPON_INT_EN_4, *pHAL_XPON_INT_EN_4;

typedef struct {
uint32 pma_tx_afifo_reach_int :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 pma_rx_afifo_reach_int :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_XPON_INT_STA_4, *pHAL_XPON_INT_STA_4;

typedef struct {
uint32 rg_pma_tx_afifo_reach_th :6;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_pma_rx_afifo_reach_th :7;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_XPON_PMA_AFIFO_REACH_TH, *pHAL_XPON_PMA_AFIFO_REACH_TH;

typedef struct {
uint32 rg_md32_pbus_arb_en :32;
}HAL_XPON_MD32_PBUS_CSR_CTRL, *pHAL_XPON_MD32_PBUS_CSR_CTRL;

typedef struct {
uint32 rg_bist_prbs_init_seed_benoff :31;
uint32 rsv_31 :1;
}HAL_BENOFF_BIST_SEED, *pHAL_BENOFF_BIST_SEED;

typedef struct {
uint32 rg_pll_pwr_seq_sel :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rsv_8 :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_PWR_PLL_CTRL, *pHAL_PWR_PLL_CTRL;

typedef struct {
uint32 rg_pon_tx_rate_ctrl :2;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rsv_8 :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_xpon_tx_rate_ctrl, *pHAL_xpon_tx_rate_ctrl;

typedef struct {
uint32 rg_benoff_d0 :32;
}HAL_BENOFF_DATA0, *pHAL_BENOFF_DATA0;

typedef struct {
uint32 rg_benoff_d1 :32;
}HAL_BENOFF_DATA1, *pHAL_BENOFF_DATA1;

typedef struct {
uint32 rg_benoff_d2 :32;
}HAL_BENOFF_DATA2, *pHAL_BENOFF_DATA2;

typedef struct {
uint32 rg_benoff_d3 :32;
}HAL_BENOFF_DATA3, *pHAL_BENOFF_DATA3;

typedef struct {
uint32 rg_benoff_cnt_max :3;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_benoff_catch_mem_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_BENOFF_CTRL, *pHAL_BENOFF_CTRL;

typedef struct {
uint32 rg_hsdata_en_sel :3;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rsv_8 :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_HS_DATA_EN_SEL, *pHAL_HS_DATA_EN_SEL;

typedef struct {
uint32 ro_rx_wrfifo_depth :7;
uint32 rsv_7 :1;
uint32 ro_rx_rdfifo_depth :7;
uint32 ro_tx_wrfifo_depth :6;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 ro_tx_rdfifo_depth :6;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_FIFO_CK_STATUS, *pHAL_FIFO_CK_STATUS;

typedef struct {
uint32 ro_dlf_gain_new :8;
uint32 ro_dlf_gain_rund_new :8;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_TDC_DLF_GAIN_STATUS, *pHAL_TDC_DLF_GAIN_STATUS;

typedef struct {
uint32 ro_dlf_gain_new_org :20;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_dlf_mode_sel :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rg_dlf_gain_latch :1;
}HAL_TDC_DLF_MODE_SETTING, *pHAL_TDC_DLF_MODE_SETTING;

typedef struct {
uint32 rg_md32_sfp_rx_loss_inv :32;
}HAL_MD32_ISR_CTRL, *pHAL_MD32_ISR_CTRL;

typedef struct {
uint32 rg_md32pm_ck_sel :32;
}HAL_MD32_MEM_CLK_CTRL, *pHAL_MD32_MEM_CLK_CTRL;

typedef struct {
uint32 rg_force_md32d_ck_g :32;
}HAL_MD32_MEM_CLK_CG_CTRL, *pHAL_MD32_MEM_CLK_CG_CTRL;

typedef struct {
uint32 ro_xg_psync_to_sof_delay_lch :32;
}HAL_XG_EQD_STA, *pHAL_XG_EQD_STA;

typedef struct {
uint32 rg_xg_eqd_rst_n :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rsv_8 :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_xg_mask_window :16;
}HAL_XG_EQD_CTRL, *pHAL_XG_EQD_CTRL;

typedef struct {
uint32 rg_rawdata_tx_en :32;
}HAL_RAWDATA_DUMP_CTRL_0, *pHAL_RAWDATA_DUMP_CTRL_0;

typedef struct {
uint32 rg_rawdata_tx_wdata_sel :32;
}HAL_RAWDATA_DUMP_CTRL_1, *pHAL_RAWDATA_DUMP_CTRL_1;

typedef struct {
uint32 rg_rawdata_mem_sel :32;
}HAL_RAWDATA_DUMP_CTRL_2, *pHAL_RAWDATA_DUMP_CTRL_2;

typedef struct {
uint32 rg_rawdata_tx_start_sel :32;
}HAL_RAWDATA_DUMP_CTRL_3, *pHAL_RAWDATA_DUMP_CTRL_3;

typedef struct {
uint32 ro_rawdata_tx_waddr :32;
}HAL_RAWDATA_DUMP_STA_0, *pHAL_RAWDATA_DUMP_STA_0;

typedef struct {
uint32 ro_rawdata_rx_waddr :32;
}HAL_RAWDATA_DUMP_STA_1, *pHAL_RAWDATA_DUMP_STA_1;

typedef struct {
uint32 ro_rawdata_tx_full :32;
}HAL_RAWDATA_DUMP_STA_2, *pHAL_RAWDATA_DUMP_STA_2;

typedef struct {
uint32 rg_md32_mem_max :32;
}HAL_MD32MEM_DIV2_CNT, *pHAL_MD32MEM_DIV2_CNT;

typedef struct {
uint32 ad_xpon_cdr_lpf_snapshotvalue :19;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_AD_XPON_CDR_LPF_SV, *pHAL_AD_XPON_CDR_LPF_SV;

typedef struct {
uint32 ro_dta_xpon_tx_termn_sel :2;
uint32 ro_dta_xpon_tx_termp_sel :2;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rsv_8 :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_K_VALUE_RD, *pHAL_K_VALUE_RD;

typedef struct {
uint32 rg_xponpll_auto_en :1;
uint32 rg_jcpll_auto_en :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rsv_8 :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_XPONPLL_CTRL, *pHAL_XPONPLL_CTRL;

typedef struct {
uint32 ro_md32_gated :32;
}HAL_MD32_GATE_HALT, *pHAL_MD32_GATE_HALT;

typedef struct {
uint32 ro_mon_ccnt :32;
}HAL_MD32_MONCCNT, *pHAL_MD32_MONCCNT;

typedef struct {
uint32 ro_mon_pcnt0 :32;
}HAL_MD32_MONPCNT0, *pHAL_MD32_MONPCNT0;

typedef struct {
uint32 ro_mon_pcnt1 :32;
}HAL_MD32_MONPCNT1, *pHAL_MD32_MONPCNT1;

typedef struct {
uint32 ro_mon_pcnt2 :32;
}HAL_MD32_MONPCNT2, *pHAL_MD32_MONPCNT2;

typedef struct {
uint32 ro_mon_contid :32;
}HAL_MD32_MONCONTID, *pHAL_MD32_MONCONTID;

typedef struct {
uint32 ro_mon_pc :32;
}HAL_MD32_MONPC, *pHAL_MD32_MONPC;

typedef struct {
uint32 ro_mon_tbuf_wptr :32;
}HAL_MD32_TBUF, *pHAL_MD32_TBUF;

typedef struct {
uint32 ro_tbuf_wdata_l :32;
}HAL_MD32_TBUFWDATA_L, *pHAL_MD32_TBUFWDATA_L;

typedef struct {
uint32 ro_tbuf_wdata_h :32;
}HAL_MD32_TBUFWDATA_H, *pHAL_MD32_TBUFWDATA_H;

typedef struct {
uint32 ro_tbuf_rdata_l :32;
}HAL_MD32_TBUFRDATA_L, *pHAL_MD32_TBUFRDATA_L;

typedef struct {
uint32 ro_tbuf_rdata_h :32;
}HAL_MD32_TBUFRDATA_H, *pHAL_MD32_TBUFRDATA_H;

typedef struct {
uint32 rg_addr_md32_pmem_addr :32;
}HAL_ADDR_MD32_PMEM_ADDR_REV, *pHAL_ADDR_MD32_PMEM_ADDR_REV;

typedef struct {
uint32 rg_addr_md32_pmem_data :32;
}HAL_ADDR_MD32_PMEM_DATA_REV, *pHAL_ADDR_MD32_PMEM_DATA_REV;

typedef struct {
uint32 rg_addr_md32_dmem_addr :32;
}HAL_ADDR_MD32_DMEM_ADDR_REV, *pHAL_ADDR_MD32_DMEM_ADDR_REV;

typedef struct {
uint32 rg_addr_md32_dmem_data :32;
}HAL_ADDR_MD32_DMEM_DATA_REV, *pHAL_ADDR_MD32_DMEM_DATA_REV;

typedef struct {
uint32 rg_jcpll_force_on :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_jcpll_hw_ctrl_mode :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_jcpll_en_inv :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_sw_jcpll_en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rg_jcpll_man_pwdb :1;
}HAL_SS_JCPLL_PWCTL_SETTING_0, *pHAL_SS_JCPLL_PWCTL_SETTING_0;

typedef struct {
uint32 rg_jcpll_en_timer :16;
uint32 rg_jcpll_pcw_man_load_timer :8;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_JCPLL_PWCTL_SETTING_1, *pHAL_SS_JCPLL_PWCTL_SETTING_1;

typedef struct {
uint32 rg_jcpll_ck_stb_timer :21;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_SS_JCPLL_PWCTL_SETTING_2, *pHAL_SS_JCPLL_PWCTL_SETTING_2;

typedef struct {
uint32 rg_jcpll_pcw_chg :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rsv_8 :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_jcpll_ledck_div :4;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_jcpll_tdc_dig_pwdb :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_JCPLL_CHG, *pHAL_JCPLL_CHG;

typedef struct {
uint32 rg_jcpll_sdm_pcw :31;
uint32 rsv_31 :1;
}HAL_RG_JCPLL_SDM_PCW_CTRL, *pHAL_RG_JCPLL_SDM_PCW_CTRL;

typedef struct {
uint32 rg_jcpll_sdm_pcw_chg :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rsv_8 :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RG_JCPLL_SDM_PCW_CHG_CTRL, *pHAL_RG_JCPLL_SDM_PCW_CHG_CTRL;

typedef struct {
uint32 rg_jcpll_ft_lock_cyclecnt :16;
uint32 rg_jcpll_ft_unlock_cyclecnt :16;
}HAL_PLL_JCPLL_FT_FREQDET_0, *pHAL_PLL_JCPLL_FT_FREQDET_0;

typedef struct {
uint32 rg_jcpll_ft_lock_target_beg :16;
uint32 rg_jcpll_ft_lock_target_end :16;
}HAL_PLL_JCPLL_FT_FREQDET_1, *pHAL_PLL_JCPLL_FT_FREQDET_1;

typedef struct {
uint32 rg_jcpll_ft_unlock_target_beg :16;
uint32 rg_jcpll_ft_unlock_target_end :16;
}HAL_PLL_JCPLL_FT_FREQDET_2, *pHAL_PLL_JCPLL_FT_FREQDET_2;

typedef struct {
uint32 rg_jcpll_ft_freqlock_det_en :3;
uint32 rsv_3 :1;
uint32 rg_jcpll_ft_lock_cnt_clear :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_jcpll_ft_lock_lockth :4;
uint32 rg_jcpll_ft_unlockth :4;
uint32 rg_jcpll_ft_wait_100us :8;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_PLL_JCPLL_FT_FREQDET_3, *pHAL_PLL_JCPLL_FT_FREQDET_3;

typedef struct {
uint32 rg_jcpll_ft_ck_mux :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rsv_8 :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_JCPLL_FT_CK_MUX, *pHAL_JCPLL_FT_CK_MUX;

typedef struct {
uint32 rg_jcpll_da_ctrl_hw_timer :16;
uint32 rg_xfi_xtalck_cgm_en_force_on :1;
uint32 rg_jcpll_autok_en_force_on :1;
uint32 rg_jcpll_autok_load_force_on :1;
uint32 rg_jcpll_ldochp_en_force_on :1;
uint32 rg_jcpll_chpup_en_force_on :1;
uint32 rg_jcpll_digtcl_dac_en_force_on :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rg_jcpll_da_ctrl_man_pwdb :1;
}HAL_JCPLL_DA_RG_CTRL_0, *pHAL_JCPLL_DA_RG_CTRL_0;

typedef struct {
uint32 rg_xfi_xtalck_cgm_en_inv :1;
uint32 rg_jcpll_autok_en_inv :1;
uint32 rg_jcpll_autok_load_inv :1;
uint32 rg_jcpll_ldochp_en_inv :1;
uint32 rg_jcpll_chpup_en_inv :1;
uint32 rg_jcpll_digtcl_dac_en_inv :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rsv_8 :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_JCPLL_DA_RG_CTRL_1, *pHAL_JCPLL_DA_RG_CTRL_1;

typedef struct {
uint32 rg_xfi_xtalck_cgm_en_hw_mode :1;
uint32 rg_jcpll_autok_en_hw_mode :1;
uint32 rg_jcpll_autok_load_hw_mode :1;
uint32 rg_jcpll_ldochp_en_hw_mode :1;
uint32 rg_jcpll_chpup_en_hw_mode :1;
uint32 rg_jcpll_digtcl_dac_en_hw_mode :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rsv_8 :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_JCPLL_DA_RG_CTRL_2, *pHAL_JCPLL_DA_RG_CTRL_2;

typedef struct {
uint32 rg_xfi_clkpath_ldo_en_inv :1;
uint32 rg_xfi_cdr_pd_edge_dis_inv :1;
uint32 rg_xfi_rx_fe_vb_eq1_en_inv :1;
uint32 rg_xfi_rx_fe_vb_eq2_en_inv :1;
uint32 rg_xfi_rx_fe_vb_eq3_en_inv :1;
uint32 rg_xfi_rx_fe_vcm_sel_inv :1;
uint32 rg_xfi_rxpll1_refck_pwdb_inv :1;
uint32 rg_xfi_tx_en_inv :1;
uint32 rg_xfi_rx_phyck_rstb_inv :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_XFI_PLL_DA_RG_CTRL_0, *pHAL_XFI_PLL_DA_RG_CTRL_0;

typedef struct {
uint32 rg_xfi_clkpath_ldo_en_hw_mode :1;
uint32 rg_xfi_cdr_pd_edge_dis_hw_mode :1;
uint32 rg_xfi_rx_fe_vb_eq1_en_hw_mode :1;
uint32 rg_xfi_rx_fe_vb_eq2_en_hw_mode :1;
uint32 rg_xfi_rx_fe_vb_eq3_en_hw_mode :1;
uint32 rg_xfi_rx_fe_vcm_gem_pwdb_hw_mode :1;
uint32 rg_xfi_rx_fe_vcm_sel_hw_mode :1;
uint32 rg_xfi_rxpll1_refck_pwdb_hw_mode :1;
uint32 rg_xfi_tx_en_hw_mode :1;
uint32 rg_xfi_rx_phyck_rstb_hw_mode :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_XFI_PLL_DA_RG_CTRL_1, *pHAL_XFI_PLL_DA_RG_CTRL_1;

typedef struct {
uint32 rg_xfi_clkpath_ldo_en_force_on :1;
uint32 rg_xfi_cdr_pd_edge_dis_force_on :1;
uint32 rg_xfi_rx_fe_vb_eq1_en_force_on :1;
uint32 rg_xfi_rx_fe_vb_eq2_en_force_on :1;
uint32 rg_xfi_rx_fe_vb_eq3_en_force_on :1;
uint32 rg_xfi_rx_fe_vcm_gem_pwdb_force_on :1;
uint32 rg_xfi_rx_fe_vcm_sel_force_on :1;
uint32 rg_xfi_rxpll1_refck_pwdb_force_on :1;
uint32 rg_xfi_tx_en_force_on :1;
uint32 rg_xfi_rx_phyck_rstb_force_on :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_XFI_PLL_DA_RG_CTRL_2, *pHAL_XFI_PLL_DA_RG_CTRL_2;

typedef struct {
uint32 ro_ipath_gaindiv :4;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 ro_txpll_tdc_c :8;
uint32 ro_txpll_tdc_d :8;
uint32 ro_jcpll_sdm_scan_out :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_AD_DA_PROBE_STS_0, *pHAL_AD_DA_PROBE_STS_0;

typedef struct {
uint32 ro_ipath_ctrl :24;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_AD_DA_PROBE_STS_1, *pHAL_AD_DA_PROBE_STS_1;

typedef struct {
uint32 rg_jcpll_sdm_scan_mode :1;
uint32 rg_jcpll_sdm_scan_rstb :1;
uint32 rg_jcpll_sdm_cg_scan_enable :1;
uint32 rg_jcpll_sdm_scan_enable :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rsv_8 :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_JCPLL_SCAN_CTRL, *pHAL_JCPLL_SCAN_CTRL;

typedef struct {
uint32 rg_xfi_gpio_sel0 :8;
uint32 rg_xfi_gpio_sel1 :8;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_XFI_GPIO_SEL_0, *pHAL_XFI_GPIO_SEL_0;

typedef struct {
uint32 rsv_0 :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rsv_8 :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_pmatop_probe_sel :5;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_clkgen_pll_lock_en :1;
uint32 rg_clkgen_pll_lock :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_TX_PAR_PROBE_PLL_CTRL, *pHAL_TX_PAR_PROBE_PLL_CTRL;

typedef struct {
uint32 rsv_0 :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_rx_asic_used_ff :1;
uint32 rg_r2t_asic_used_ff :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rg_xfi_pcs_tx_rst_en :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_DUMP_RX_PCSRST_CTRL, *pHAL_DUMP_RX_PCSRST_CTRL;

typedef struct {
uint32 ro_xfi_tx_fifo_status :32;
}HAL_XFI_TX_FIFO_STS, *pHAL_XFI_TX_FIFO_STS;

typedef struct {
uint32 rg_jcpll_ck_rstb_force_on :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_jcpll_ck_rstb_hw_mode :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_jcpll_ck_rstb_inv :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_JCPLL_CK_RSTB_CTRL, *pHAL_JCPLL_CK_RSTB_CTRL;

typedef struct {
uint32 ro_tdc_tx_fbck_lock :1;
uint32 ro_tdc_tx_fbck_lock_inner :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 ro_tdc_tx_lslock_cnt :4;
uint32 ro_tdc_tx_state_freqdet :2;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 ro_tdc_tx_fl_out :16;
}HAL_RO_TDC_TX_FREQDET, *pHAL_RO_TDC_TX_FREQDET;

typedef struct {
uint32 ro_jcpll_ft_fbck_lock :1;
uint32 ro_jcpll_ft_fbck_lock_inner :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 ro_jcpll_ft_lslock_cnt :4;
uint32 ro_jcpll_ft_state_freqdet :2;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 ro_jcpll_ft_fl_out :16;
}HAL_RO_JCPLL_FT_FREQDET, *pHAL_RO_JCPLL_FT_FREQDET;

typedef struct {
uint32 ro_jcpll_500m_fbck_lock :1;
uint32 ro_jcpll_500m_fbck_lock_inner :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 ro_jcpll_500m_lslock_cnt :4;
uint32 ro_jcpll_500m_state_freqdet :2;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 ro_jcpll_500m_fl_out :16;
}HAL_RO_JCPLL_500M_FREQDET, *pHAL_RO_JCPLL_500M_FREQDET;

typedef struct {
uint32 rg_jcpll_500m_lock_cyclecnt :16;
uint32 rg_jcpll_500m_unlock_cyclecnt :16;
}HAL_RG_JCPLL_500M_FREQ_DET_1, *pHAL_RG_JCPLL_500M_FREQ_DET_1;

typedef struct {
uint32 rg_jcpll_500m_lock_target_beg :16;
uint32 rg_jcpll_500m_lock_target_end :16;
}HAL_RG_JCPLL_500M_FREQ_DET_2, *pHAL_RG_JCPLL_500M_FREQ_DET_2;

typedef struct {
uint32 rg_jcpll_500m_unlock_target_beg :16;
uint32 rg_jcpll_500m_unlock_target_end :16;
}HAL_RG_JCPLL_500M_FREQ_DET_3, *pHAL_RG_JCPLL_500M_FREQ_DET_3;

typedef struct {
uint32 rg_jcpll_500m_freqlock_det_en :3;
uint32 rsv_3 :1;
uint32 rg_jcpll_500m_lock_cnt_clear :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_jcpll_500m_lock_lockth :4;
uint32 rg_jcpll_500m_unlockth :4;
uint32 rg_jcpll_500m_wait_100us :8;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_RG_JCPLL_500M_FREQ_DET_4, *pHAL_RG_JCPLL_500M_FREQ_DET_4;

typedef struct {
uint32 rg_da_pxp_jcpll_sdm_scan :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_sel_da_pxp_jcpll_sdm_scan :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_rx_fe_peaking_ctrl :4;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_rx_fe_peaking_ctrl :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_da_pxp_jcpll_sdm_scan, *pHAL_rg_da_pxp_jcpll_sdm_scan;

typedef struct {
uint32 rg_force_da_pxp_aeq_speed :3;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_aeq_speed :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_rx_osr_sel :2;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_rx_osr_sel :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_aeq_speed, *pHAL_rg_force_da_pxp_aeq_speed;

typedef struct {
uint32 rg_force_da_pxp_aeq_mode :3;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_aeq_mode :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_aeq_mode, *pHAL_rg_force_da_pxp_aeq_mode;

typedef struct {
uint32 rg_force_da_pxp_tx_data_ben :16;
uint32 rg_force_sel_da_pxp_tx_data_ben :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_tx_data_ben, *pHAL_rg_force_da_pxp_tx_data_ben;

typedef struct {
uint32 rg_force_da_pxp_tx_fir_c0b :6;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_tx_fir_c0b :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_tx_fir_cn1 :5;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_tx_fir_cn1 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_tx_fir_c0b, *pHAL_rg_force_da_pxp_tx_fir_c0b;

typedef struct {
uint32 rg_force_da_pxp_tx_term_sel :3;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_tx_term_sel :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_tx_ckin_divisor :4;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_tx_ckin_divisor :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_tx_term_sel, *pHAL_rg_force_da_pxp_tx_term_sel;

typedef struct {
uint32 rg_force_da_pxp_tx_fir_c1 :5;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_tx_fir_c1 :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_tx_fir_c2 :3;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_tx_fir_c2 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_tx_fir_c1, *pHAL_rg_force_da_pxp_tx_fir_c1;

typedef struct {
uint32 rg_force_da_pxp_tx_rate_ctrl :2;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_tx_rate_ctrl :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_cdr_pr_pieye :7;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_cdr_pr_pieye :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_tx_rate_ctrl, *pHAL_rg_force_da_pxp_tx_rate_ctrl;

typedef struct {
uint32 rg_force_da_pxp_rx_dac_d0 :7;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_rx_dac_d0 :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_rx_dac_d1 :7;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_rx_dac_d1 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_rx_dac_d0, *pHAL_rg_force_da_pxp_rx_dac_d0;

typedef struct {
uint32 rg_force_da_pxp_rx_dac_e0 :7;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_rx_dac_e0 :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_rx_dac_e1 :7;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_rx_dac_e1 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_rx_dac_e0, *pHAL_rg_force_da_pxp_rx_dac_e0;

typedef struct {
uint32 rg_force_da_pxp_cdr_pr_fll_cor :5;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_cdr_pr_fll_cor :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_rx_dac_eye :7;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_rx_dac_eye :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_cdr_pr_fll_cor, *pHAL_rg_force_da_pxp_cdr_pr_fll_cor;

typedef struct {
uint32 rg_force_da_pxp_cdr_pr_idac :11;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_sel_da_pxp_cdr_pr_idac :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_txpll_sdm_pcw :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_cdr_pr_idac, *pHAL_rg_force_da_pxp_cdr_pr_idac;

typedef struct {
uint32 rg_force_da_pxp_txpll_sdm_pcw :31;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_txpll_sdm_pcw, *pHAL_rg_force_da_pxp_txpll_sdm_pcw;

typedef struct {
uint32 rg_force_da_pxp_rx_fe_vos :6;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_rx_fe_vos :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_sel_da_pxp_jcpll_sdm_pcw :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_rx_fe_vos, *pHAL_rg_force_da_pxp_rx_fe_vos;

typedef struct {
uint32 rg_force_da_pxp_jcpll_sdm_pcw :31;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_jcpll_sdm_pcw, *pHAL_rg_force_da_pxp_jcpll_sdm_pcw;

typedef struct {
uint32 rg_force_da_pcie_cktx0_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pcie_cktx0_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pcie_cktx1_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pcie_cktx1_en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pcie_cktx0_en, *pHAL_rg_force_da_pcie_cktx0_en;

typedef struct {
uint32 rg_force_da_pxp_aeq_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_aeq_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_aeq_en, *pHAL_rg_force_da_pxp_aeq_en;

typedef struct {
uint32 rg_force_da_pxp_aeq_bypass :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_aeq_bypass :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_aeq_ckon :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_aeq_ckon :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_aeq_bypass, *pHAL_rg_force_da_pxp_aeq_bypass;

typedef struct {
uint32 rg_force_da_pxp_aeq_inprgrss :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_aeq_inprgrss :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_aeq_invldreq :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_aeq_invldreq :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_aeq_inprgrss, *pHAL_rg_force_da_pxp_aeq_inprgrss;

typedef struct {
uint32 rg_force_da_pxp_aeq_rstb :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_aeq_rstb :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_cdr_injck_sel :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_cdr_injck_sel :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_aeq_rstb, *pHAL_rg_force_da_pxp_aeq_rstb;

typedef struct {
uint32 rg_force_da_pxp_cdr_lpf_lck2data :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_cdr_lpf_lck2data :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_cdr_lpf_rstb :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_cdr_lpf_rstb :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_cdr_lpf_lck2data, *pHAL_rg_force_da_pxp_cdr_lpf_lck2data;

typedef struct {
uint32 rg_force_da_pxp_cdr_pd_pwdb :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_cdr_pd_pwdb :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_cdr_pr_kband_rstb :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_cdr_pr_kband_rstb :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_cdr_pd_pwdb, *pHAL_rg_force_da_pxp_cdr_pd_pwdb;

typedef struct {
uint32 rg_force_da_pxp_cdr_pr_lpf_c_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_cdr_pr_lpf_c_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_cdr_pr_lpf_r_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_cdr_pr_lpf_r_en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_cdr_pr_lpf_c_en, *pHAL_rg_force_da_pxp_cdr_pr_lpf_c_en;

typedef struct {
uint32 rg_force_da_pxp_cdr_pr_pieye_pwdb :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_cdr_pr_pieye_pwdb :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_cdr_pr_pwdb :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_cdr_pr_pwdb :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_cdr_pr_pieye_pwdb, *pHAL_rg_force_da_pxp_cdr_pr_pieye_pwdb;

typedef struct {
uint32 rg_force_da_pxp_jcpll_ckout_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_jcpll_ckout_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_jcpll_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_jcpll_en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_jcpll_ckout_en, *pHAL_rg_force_da_pxp_jcpll_ckout_en;

typedef struct {
uint32 rg_force_da_pxp_jcpll_kband_scan_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_jcpll_kband_scan_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_jcpll_kband_scan_en_cg :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_jcpll_kband_scan_en_cg :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_jcpll_kband_scan_en, *pHAL_rg_force_da_pxp_jcpll_kband_scan_en;

typedef struct {
uint32 rg_force_da_pxp_jcpll_kband_scan_in :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_jcpll_kband_scan_in :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_jcpll_kband_scan_rstb :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_jcpll_kband_scan_rstb :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_jcpll_kband_scan_in, *pHAL_rg_force_da_pxp_jcpll_kband_scan_in;

typedef struct {
uint32 rg_force_da_pxp_jcpll_sdm_pcw_chg :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_jcpll_sdm_pcw_chg :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_jcpll_sdm_scan_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_jcpll_sdm_scan_en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_jcpll_sdm_pcw_chg, *pHAL_rg_force_da_pxp_jcpll_sdm_pcw_chg;

typedef struct {
uint32 rg_force_da_pxp_jcpll_sdm_scan_en_cg :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_jcpll_sdm_scan_en_cg :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_jcpll_sdm_scan_in :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_jcpll_sdm_scan_in :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_jcpll_sdm_scan_en_cg, *pHAL_rg_force_da_pxp_jcpll_sdm_scan_en_cg;

typedef struct {
uint32 rg_force_da_pxp_jcpll_sdm_scan_rstb :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_jcpll_sdm_scan_rstb :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_rx_oscal_ckon :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_rx_oscal_ckon :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_jcpll_sdm_scan_rstb, *pHAL_rg_force_da_pxp_jcpll_sdm_scan_rstb;

typedef struct {
uint32 rg_force_da_pxp_rx_oscal_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_rx_oscal_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_rx_oscal_rstb :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_rx_oscal_rstb :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_rx_oscal_en, *pHAL_rg_force_da_pxp_rx_oscal_en;

typedef struct {
uint32 rg_force_da_pxp_rx_scan_enable :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_rx_scan_enable :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_rx_scan_enable_cg :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_rx_scan_enable_cg :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_rx_scan_enable, *pHAL_rg_force_da_pxp_rx_scan_enable;

typedef struct {
uint32 rg_force_da_pxp_rx_scan_in :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_rx_scan_in :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_da_pxp_rx_scan :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_sel_da_pxp_rx_scan :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_rx_scan_in, *pHAL_rg_force_da_pxp_rx_scan_in;

typedef struct {
uint32 rg_force_da_pxp_rx_scan_rst_b :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_rx_scan_rst_b :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_rx_sigdet_pwdb :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_rx_sigdet_pwdb :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_rx_scan_rst_b, *pHAL_rg_force_da_pxp_rx_scan_rst_b;

typedef struct {
uint32 rg_force_da_pxp_tdc_cal_bw :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_tdc_cal_bw :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_tdc_cal_offset :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_tdc_cal_offset :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_tdc_cal_bw, *pHAL_rg_force_da_pxp_tdc_cal_bw;

typedef struct {
uint32 rg_force_da_pxp_txpll_ckout_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_txpll_ckout_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_txpll_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_txpll_en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_txpll_ckout_en, *pHAL_rg_force_da_pxp_txpll_ckout_en;

typedef struct {
uint32 rg_force_da_pxp_txpll_kband_load_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_txpll_kband_load_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_txpll_kband_scan_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_txpll_scan_en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_txpll_kband_load_en, *pHAL_rg_force_da_pxp_txpll_kband_load_en;

typedef struct {
uint32 rg_force_da_pxp_txpll_kband_scan_en_cg :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_txpll_kband_scan_en_cg :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_txpll_kband_scan_in :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_txpll_kband_scan_in :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_txpll_kband_scan_en_cg, *pHAL_rg_force_da_pxp_txpll_kband_scan_en_cg;

typedef struct {
uint32 rg_da_pxp_txpll_kband_scan :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_sel_da_pxp_txpll_kband_scan :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_txpll_kband_scan_rstb :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_txpll_kband_scan_rstb :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_da_pxp_txpll_kband_scan, *pHAL_rg_da_pxp_txpll_kband_scan;

typedef struct {
uint32 rg_force_da_pxp_txpll_sdm_pcw_chg :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_txpll_sdm_pcw_chg :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_txpll_sdm_scan_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_txpll_sdm_scan_en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_txpll_sdm_pcw_chg, *pHAL_rg_force_da_pxp_txpll_sdm_pcw_chg;

typedef struct {
uint32 rg_force_da_pxp_txpll_sdm_scan_en_cg :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_txpll_sdm_scan_en_cg :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_txpll_sdm_scan_in :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_txpll_sdm_scan_in :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_txpll_sdm_scan_en_cg, *pHAL_rg_force_da_pxp_txpll_sdm_scan_en_cg;

typedef struct {
uint32 rg_da_pxp_txpll_sdm_scan :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_sel_da_pxp_txpll_sdm_scan :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_txpll_sdm_scan_rstb :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_txpll_sdm_scan_rstb :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_da_pxp_txpll_sdm_scan, *pHAL_rg_da_pxp_txpll_sdm_scan;

typedef struct {
uint32 rg_force_da_pxp_tx_acjtag_dn :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_tx_acjtag_dn :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_tx_acjtag_dp :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_tx_acjtag_dp :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_tx_acjtag_dn, *pHAL_rg_force_da_pxp_tx_acjtag_dn;

typedef struct {
uint32 rg_force_da_pxp_tx_acjtag_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_tx_acjtag_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_tx_ckin_sel :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_tx_ckin_sel :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_tx_acjtag_en, *pHAL_rg_force_da_pxp_tx_acjtag_en;

typedef struct {
uint32 rg_force_da_pxp_tx_ck_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_tx_ck_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_tx_cm_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_tx_cm_en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_tx_ck_en, *pHAL_rg_force_da_pxp_tx_ck_en;

typedef struct {
uint32 rg_force_da_pxp_tx_hsdata_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_tx_hsdata_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_tx_data_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_tx_data_en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_tx_hsdata_en, *pHAL_rg_force_da_pxp_tx_hsdata_en;

typedef struct {
uint32 rg_force_da_pxp_tx_rxdet_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_tx_rxdet_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_tx_termp_clean :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_tx_termp_clean :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_tx_rxdet_en, *pHAL_rg_force_da_pxp_tx_rxdet_en;

typedef struct {
uint32 rg_force_da_pxp_jcpll_kband_load_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_jcpll_kband_load_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rsv_16 :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rsv_24 :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_scan_mode, *pHAL_scan_mode;

typedef struct {
uint32 rg_da_pxp_jcpll_kband_scan :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_sel_da_pxp_jcpll_kband_scan :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_tx_termcal_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_tx_termcal_en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_da_pxp_jcpll_kband_scan, *pHAL_rg_da_pxp_jcpll_kband_scan;

typedef struct {
uint32 rg_force_da_pxp_rx_fe_gain_ctrl :2;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_rx_fe_gain_ctrl :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_rx_sigdet_os :5;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_rx_sigdet_os :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_rx_fe_gain_ctrl, *pHAL_rg_force_da_pxp_rx_fe_gain_ctrl;

typedef struct {
uint32 rg_force_da_pxp_aeq_rmtxskip :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_aeq_rmtxskip :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_aeq_rxeqeval :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_aeq_rxeqeval :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_aeq_rmtxskip, *pHAL_rg_force_da_pxp_aeq_rmtxskip;

typedef struct {
uint32 rg_force_da_pxp_rx_fe_pwdb :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_rx_fe_pwdb :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_rx_pdoscal_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_rx_pdoscal_en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_rx_fe_pwdb, *pHAL_rg_force_da_pxp_rx_fe_pwdb;

typedef struct {
uint32 rg_force_da_pxp_rx_sigdet_cal_en :1;
uint32 rsv_1 :1;
uint32 rsv_2 :1;
uint32 rsv_3 :1;
uint32 rsv_4 :1;
uint32 rsv_5 :1;
uint32 rsv_6 :1;
uint32 rsv_7 :1;
uint32 rg_force_sel_da_pxp_rx_sigdet_cal_en :1;
uint32 rsv_9 :1;
uint32 rsv_10 :1;
uint32 rsv_11 :1;
uint32 rsv_12 :1;
uint32 rsv_13 :1;
uint32 rsv_14 :1;
uint32 rsv_15 :1;
uint32 rg_force_da_pxp_tdc_en :1;
uint32 rsv_17 :1;
uint32 rsv_18 :1;
uint32 rsv_19 :1;
uint32 rsv_20 :1;
uint32 rsv_21 :1;
uint32 rsv_22 :1;
uint32 rsv_23 :1;
uint32 rg_force_sel_da_pxp_tdc_en :1;
uint32 rsv_25 :1;
uint32 rsv_26 :1;
uint32 rsv_27 :1;
uint32 rsv_28 :1;
uint32 rsv_29 :1;
uint32 rsv_30 :1;
uint32 rsv_31 :1;
}HAL_rg_force_da_pxp_rx_sigdet_cal_en, *pHAL_rg_force_da_pxp_rx_sigdet_cal_en;

typedef struct {
uint32 rg_dig_reserve_5 :32;
}HAL_ADD_DIG_RESERVE_5, *pHAL_ADD_DIG_RESERVE_5;

typedef struct {
uint32 rg_dig_reserve_6 :32;
}HAL_ADD_DIG_RESERVE_6, *pHAL_ADD_DIG_RESERVE_6;

typedef struct {
uint32 rg_dig_reserve_7 :32;
}HAL_ADD_DIG_RESERVE_7, *pHAL_ADD_DIG_RESERVE_7;

typedef struct {
uint32 rg_dig_reserve_8 :32;
}HAL_ADD_DIG_RESERVE_8, *pHAL_ADD_DIG_RESERVE_8;

typedef struct {
uint32 rg_dig_reserve_9 :32;
}HAL_ADD_DIG_RESERVE_9, *pHAL_ADD_DIG_RESERVE_9;

typedef struct {
uint32 rg_dig_reserve_10 :32;
}HAL_ADD_DIG_RESERVE_10, *pHAL_ADD_DIG_RESERVE_10;

typedef struct {
uint32 rg_dig_reserve_11 :32;
}HAL_ADD_DIG_RESERVE_11, *pHAL_ADD_DIG_RESERVE_11;

typedef struct {
uint32 rg_dig_reserve_12 :32;
}HAL_ADD_DIG_RESERVE_12, *pHAL_ADD_DIG_RESERVE_12;

typedef struct {
uint32 rg_dig_reserve_13 :32;
}HAL_ADD_DIG_RESERVE_13, *pHAL_ADD_DIG_RESERVE_13;

typedef struct {
uint32 rg_dig_reserve_14 :32;
}HAL_ADD_DIG_RESERVE_14, *pHAL_ADD_DIG_RESERVE_14;

typedef struct {
uint32 ro_dig_reserve_5 :32;
}HAL_ADD_DIG_RO_RESERVE_5, *pHAL_ADD_DIG_RO_RESERVE_5;

typedef struct {
uint32 ro_dig_reserve_6 :32;
}HAL_ADD_DIG_RO_RESERVE_6, *pHAL_ADD_DIG_RO_RESERVE_6;

typedef struct {
uint32 ro_dig_reserve_7 :32;
}HAL_ADD_DIG_RO_RESERVE_7, *pHAL_ADD_DIG_RO_RESERVE_7;

typedef struct {
uint32 ro_dig_reserve_8 :32;
}HAL_ADD_DIG_RO_RESERVE_8, *pHAL_ADD_DIG_RO_RESERVE_8;

typedef struct {
uint32 ro_dig_reserve_9 :32;
}HAL_ADD_DIG_RO_RESERVE_9, *pHAL_ADD_DIG_RO_RESERVE_9;

typedef struct {
uint32 rg_dig_reserve_15 :32;
}HAL_ADD_DIG_RESERVE_15, *pHAL_ADD_DIG_RESERVE_15;

typedef struct {
uint32 rg_dig_reserve_16 :32;
}HAL_ADD_DIG_RESERVE_16, *pHAL_ADD_DIG_RESERVE_16;

typedef struct {
uint32 rg_dig_reserve_17 :32;
}HAL_ADD_DIG_RESERVE_17, *pHAL_ADD_DIG_RESERVE_17;

typedef struct {
uint32 rg_dig_reserve_18 :32;
}HAL_ADD_DIG_RESERVE_18, *pHAL_ADD_DIG_RESERVE_18;

typedef struct {
uint32 rg_dig_reserve_19 :32;
}HAL_ADD_DIG_RESERVE_19, *pHAL_ADD_DIG_RESERVE_19;

typedef struct {
uint32 rg_dig_reserve_20 :32;
}HAL_ADD_DIG_RESERVE_20, *pHAL_ADD_DIG_RESERVE_20;

typedef struct {
uint32 rg_dig_reserve_21 :32;
}HAL_ADD_DIG_RESERVE_21, *pHAL_ADD_DIG_RESERVE_21;

typedef struct {
uint32 rg_dig_reserve_22 :32;
}HAL_ADD_DIG_RESERVE_22, *pHAL_ADD_DIG_RESERVE_22;

typedef struct {
uint32 rg_dig_reserve_23 :32;
}HAL_ADD_DIG_RESERVE_23, *pHAL_ADD_DIG_RESERVE_23;

typedef struct {
uint32 rg_dig_reserve_24 :32;
}HAL_ADD_DIG_RESERVE_24, *pHAL_ADD_DIG_RESERVE_24;

typedef struct {
uint32 rg_dig_reserve_25 :32;
}HAL_ADD_DIG_RESERVE_25, *pHAL_ADD_DIG_RESERVE_25;

typedef struct {
uint32 rg_dig_reserve_26 :32;
}HAL_ADD_DIG_RESERVE_26, *pHAL_ADD_DIG_RESERVE_26;

typedef struct {
uint32 rg_dig_reserve_27 :32;
}HAL_ADD_DIG_RESERVE_27, *pHAL_ADD_DIG_RESERVE_27;

typedef struct {
uint32 rg_dig_reserve_28 :32;
}HAL_ADD_DIG_RESERVE_28, *pHAL_ADD_DIG_RESERVE_28;

typedef struct {
uint32 rg_dig_reserve_29 :32;
}HAL_ADD_DIG_RESERVE_29, *pHAL_ADD_DIG_RESERVE_29;

typedef struct {
uint32 rg_dig_reserve_30 :32;
}HAL_ADD_DIG_RESERVE_30, *pHAL_ADD_DIG_RESERVE_30;

typedef struct {
uint32 rg_dig_reserve_31 :32;
}HAL_ADD_DIG_RESERVE_31, *pHAL_ADD_DIG_RESERVE_31;

typedef struct {
uint32 rg_dig_reserve_32 :32;
}HAL_ADD_DIG_RESERVE_32, *pHAL_ADD_DIG_RESERVE_32;

typedef struct {
uint32 rg_dig_reserve_33 :32;
}HAL_ADD_DIG_RESERVE_33, *pHAL_ADD_DIG_RESERVE_33;

typedef struct {
uint32 rg_dig_reserve_34 :32;
}HAL_ADD_DIG_RESERVE_34, *pHAL_ADD_DIG_RESERVE_34;

typedef struct {
uint32 rg_dig_reserve_35 :32;
}HAL_ADD_DIG_RESERVE_35, *pHAL_ADD_DIG_RESERVE_35;

typedef struct {
uint32 rg_dig_reserve_36 :32;
}HAL_ADD_DIG_RESERVE_36, *pHAL_ADD_DIG_RESERVE_36;

typedef struct {
uint32 rg_dig_reserve_37 :32;
}HAL_ADD_DIG_RESERVE_37, *pHAL_ADD_DIG_RESERVE_37;

typedef struct {
uint32 rg_dig_reserve_38 :32;
}HAL_ADD_DIG_RESERVE_38, *pHAL_ADD_DIG_RESERVE_38;

typedef struct {
uint32 rg_dig_reserve_39 :32;
}HAL_ADD_DIG_RESERVE_39, *pHAL_ADD_DIG_RESERVE_39;

typedef struct {
uint32 rg_dig_reserve_40 :32;
}HAL_ADD_DIG_RESERVE_40, *pHAL_ADD_DIG_RESERVE_40;

typedef struct {
uint32 rg_dig_reserve_41 :32;
}HAL_ADD_DIG_RESERVE_41, *pHAL_ADD_DIG_RESERVE_41;

typedef struct {
uint32 rg_dig_reserve_42 :32;
}HAL_ADD_DIG_RESERVE_42, *pHAL_ADD_DIG_RESERVE_42;

typedef struct {
uint32 rg_dig_reserve_43 :32;
}HAL_ADD_DIG_RESERVE_43, *pHAL_ADD_DIG_RESERVE_43;

typedef struct {
uint32 rg_dig_reserve_44 :32;
}HAL_ADD_DIG_RESERVE_44, *pHAL_ADD_DIG_RESERVE_44;

typedef struct {
uint32 rg_dig_reserve_45 :32;
}HAL_ADD_DIG_RESERVE_45, *pHAL_ADD_DIG_RESERVE_45;

typedef struct {
uint32 rg_dig_reserve_46 :32;
}HAL_ADD_DIG_RESERVE_46, *pHAL_ADD_DIG_RESERVE_46;

typedef struct {
uint32 rg_dig_reserve_47 :32;
}HAL_ADD_DIG_RESERVE_47, *pHAL_ADD_DIG_RESERVE_47;

typedef struct {
uint32 rg_dig_reserve_48 :32;
}HAL_ADD_DIG_RESERVE_48, *pHAL_ADD_DIG_RESERVE_48;

#endif