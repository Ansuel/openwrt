 /***************************************************************************************
 *      Copyright(c) 2016 ECONET Incorporation All rights reserved.
 *
 *      This is unpublished proprietary source code of ECONET Incorporation
 *
 *      The copyright notice above does not evidence any actual or intended
 *      publication of such source code.
 ***************************************************************************************
 */
 
 /*======================================================================================
 * MODULE NAME: spi
 * FILE NAME: spi_nfi.h
 * DATE: 2016/03/18
 * VERSION: 1.00
 * PURPOSE: To Provide SPI NFI(DMA) Access Internace. 
 * NOTES:
 *
 * AUTHOR : Chuck Kuo         REVIEWED by
 *
 * FUNCTIONS
 *
 * DEPENDENCIES
 *
 * * $History: $
 * MODIFICTION HISTORY:
 * Version 1.00 - Date 2016/03/18 By Chuck Kuo
 * ** This is the first versoin for creating to support the functions of
 *    current module.
 *
 *======================================================================================
 */

#ifndef __SPI_NFI_H__
    #define __SPI_NFI_H__

 
/* INCLUDE FILE DECLARATIONS --------------------------------------------------------- */
#include <asm/types.h>


/* NAMING CONSTANT DECLARATIONS ------------------------------------------------------ */

#if defined(CONFIG_ECNT_UBOOT) /* U-boot likes MIPS bootram, don't follow ARMV8 path */
#undef TCSUPPORT_CPU_ARMV8
#endif


/* TYPE DECLARATIONS ----------------------------------------------------------------- */
typedef enum 
{
   SPI_NFI_CON_AUTO_FDM_Disable=0,
   SPI_NFI_CON_AUTO_FDM_Enable,
} SPI_NFI_CONF_AUTO_FDM_T;

typedef enum 
{
   SPI_NFI_CON_HW_ECC_Disable=0,
   SPI_NFI_CON_HW_ECC_Enable,
} SPI_NFI_CONF_HW_ECC_T;

typedef enum 
{
   SPI_NFI_CON_DMA_TRIGGER_READ=0,
   SPI_NFI_CON_DMA_TRIGGER_WRITE,
} SPI_NFI_CONF_DMA_TRIGGER_T;

typedef enum 
{
   SPI_NFI_CON_DMA_BURST_Disable=0,
   SPI_NFI_CON_DMA_BURST_Enable,
} SPI_NFI_CONF_DMA_BURST_T;

typedef enum
{
	SPI_NFI_CONF_SPARE_SIZE_16BYTE = 16,
	SPI_NFI_CONF_SPARE_SIZE_26BYTE = 26,
	SPI_NFI_CONF_SPARE_SIZE_27BYTE = 27,
	SPI_NFI_CONF_SPARE_SIZE_28BYTE = 28,
} SPI_NFI_CONF_SPARE_SIZE_T;

typedef enum
{
	SPI_NFI_CONF_PAGE_SIZE_512BYTE = 512,
	SPI_NFI_CONF_PAGE_SIZE_2KBYTE  = 2048,
	SPI_NFI_CONF_PAGE_SIZE_4KBYTE  = 4096,
} SPI_NFI_CONF_PAGE_SIZE_T;

typedef enum
{
	SPI_NFI_CONF_CUS_SEC_SIZE_Disable=0,
	SPI_NFI_CONF_CUS_SEC_SIZE_Enable,
} SPI_NFI_CONF_CUS_SEC_SIZE_T;


typedef enum
{
	SPI_NFI_MISC_CONTROL_X1		 = 0x000,
	SPI_NFI_MISC_CONTROL_X2		 = 0x001,
	SPI_NFI_MISC_CONTROL_X4		 = 0x002,
	SPI_NFI_MISC_CONTROL_DUAL_IO = 0x005,
	SPI_NFI_MISC_CONTROL_QUAD_IO = 0x006,
	
} SPI_NFI_MISC_SPEDD_CONTROL_T;

typedef struct SPI_NFI_CONFIGURE
{
	SPI_NFI_CONF_AUTO_FDM_T			auto_fdm_t;			/* auto padding oob behind data, or not */
	SPI_NFI_CONF_HW_ECC_T			hw_ecc_t;			/* enable hw ecc or not */
	SPI_NFI_CONF_DMA_BURST_T		dma_burst_t;		/* dma burst */	
	u8								fdm_num;			/* value range : 0 ~ 8 */
	u8								fdm_ecc_num;		/* value range : 0 ~ 8 */
														/* fdm byte under ecc protection */
	SPI_NFI_CONF_SPARE_SIZE_T   	spare_size_t;		/* spare size of eache sector */
	SPI_NFI_CONF_PAGE_SIZE_T		page_size_t;		/* page size (not incluing oob size) */
	u8								sec_num;	   		/* number of sector */
														/* value range : 1 ~ 8 */
	SPI_NFI_CONF_CUS_SEC_SIZE_T	 	cus_sec_size_en_t;	/* To apply user define sector size or not */
														/* Disable : sector size = 512 bytes, 
															and ECC function will work */
														/* Enable  : user define sector size, 
												       		and ECC function will not work */
	u32								sec_size;			/* Only work if cus_sec_size_en is enable */
														/* value range : 1 ~ 8187 */
	SPI_NFI_MISC_SPEDD_CONTROL_T	speed_t;
	
} SPI_NFI_CONF_T;



typedef enum{
	SPI_NFI_RTN_NO_ERROR =0,
	SPI_NFI_RTN_CHECK_AHB_DONE_TIMEOUT,
	SPI_NFI_RTN_LOAD_TO_CACHE_DONE_TIMEOUT,
	SPI_NFI_RTN_READ_FROM_CACHE_DONE_TIMEOUT,

	SPI_NFI_RTN_DEF_NO
} SPI_NFI_RTN_T;
						
typedef enum{
	SPI_NFI_CONF_DMA_WR_BYTE_SWAP_DISABLE =0,
	SPI_NFI_CONF_DMA_WR_BYTE_SWAP_ENABLE
} SPI_NFI_CONF_DMA_WR_BYTE_SWAP_T;	

typedef enum{
	SPI_NFI_CONF_ECC_DATA_SOURCE_INV_DISABLE =0,
	SPI_NFI_CONF_ECC_DATA_SOURCE_INV_ENABLE
} SPI_NFI_CONF_ECC_DATA_SOURCE_INV_T;

/* EXPORTED SUBPROGRAM SPECIFICATION ------------------------------------------------- */ 										
					
SPI_NFI_RTN_T SPI_NFI_Regs_Dump( void );
SPI_NFI_RTN_T SPI_NFI_Read_SPI_NAND_FDM(u8 *ptr_rtn_oob, u32 oob_len);
SPI_NFI_RTN_T SPI_NFI_Write_SPI_NAND_FDM(u8 *ptr_oob, u32 oob_len);
SPI_NFI_RTN_T SPI_NFI_Read_SPI_NAND_Page(SPI_NFI_MISC_SPEDD_CONTROL_T speed_mode, u32 read_cmd, u16 read_addr, unsigned long *prt_rtn_data);
SPI_NFI_RTN_T SPI_NFI_Write_SPI_NAND_page(SPI_NFI_MISC_SPEDD_CONTROL_T speed_mode, u32 write_cmd, u16 write_addr, unsigned long *prt_data);
SPI_NFI_RTN_T SPI_NFI_Read_SPI_NOR(u8 opcode, u16 read_addr, u32 *prt_rtn_data);
SPI_NFI_RTN_T SPI_NFI_Write_SPI_NOR(u8 opcode, u16 write_addr, u32 *prt_data);
SPI_NFI_RTN_T SPI_NFI_Get_Configure( SPI_NFI_CONF_T *ptr_rtn_nfi_conf_t );
SPI_NFI_RTN_T SPI_NFI_Set_Configure( SPI_NFI_CONF_T *ptr_nfi_conf_t );
void SPI_NFI_Reset( void );
SPI_NFI_RTN_T SPI_NFI_Init( void );
void SPI_NFI_DEBUG_ENABLE( void );
void SPI_NFI_DEBUG_DISABLE( void );
/* Set DMA(flash -> SRAM) byte swap*/
void SPI_NFI_DMA_WR_BYTE_SWAP(SPI_NFI_CONF_DMA_WR_BYTE_SWAP_T enable);
/* Set ECC decode invert*/
void SPI_NFI_ECC_DATA_SOURCE_INV(SPI_NFI_CONF_ECC_DATA_SOURCE_INV_T enable);


#endif /* ifndef __SPI_NFI_H__ */
/* End of [spi_nfi.h] package */						   
