 /***************************************************************************************
 *      Copyright(c) 2014 ECONET Incorporation All rights reserved.
 *
 *      This is unpublished proprietary source code of ECONET Incorporation
 *
 *      The copyright notice above does not evidence any actual or intended
 *      publication of such source code.
 ***************************************************************************************
 */

/*======================================================================================
 * MODULE NAME: spi
 * FILE NAME: spi_controller.h
 * DATE: 2014/12/16
 * VERSION: 1.00
 * PURPOSE: To Provide SPI Controller Access interface.
 * NOTES:
 *
 * AUTHOR : Chuck Kuo         REVIEWED by
 *
 * FUNCTIONS
 *
 *      SPI_CONTROLLER_Enable_Manual_Mode To provide interface for Enable SPI Controller Manual Mode.
 *      SPI_CONTROLLER_Write_One_Byte     To provide interface for write one byte to SPI bus. 
 *      SPI_CONTROLLER_Write_NByte        To provide interface for write N bytes to SPI bus. 
 *      SPI_CONTROLLER_Read_NByte         To provide interface for read N bytes from SPI bus. 
 *      SPI_CONTROLLER_Chip_Select_Low    To provide interface for set chip select low in SPI bus. 
 *      SPI_CONTROLLER_Chip_Select_High   To provide interface for set chip select high in SPI bus. 
 *
 * DEPENDENCIES
 *
 * * $History: $
 * MODIFICTION HISTORY:
 * Version 1.00 - Date 2014/12/16 By Chuck Kuo
 * ** This is the first versoin for creating to support the functions of
 *    current module.
 *
 *======================================================================================
 */

#ifndef __SPI_CONTROLLER_H__
    #define __SPI_CONTROLLER_H__

/* INCLUDE FILE DECLARATIONS --------------------------------------------------------- */
#include <asm/types.h>

#if defined(CONFIG_ECNT_UBOOT) /* U-boot likes MIPS bootram, don't follow ARMV8 path */
#undef TCSUPPORT_CPU_ARMV8
#endif

/* NAMING CONSTANT DECLARATIONS ------------------------------------------------------ */

/* MACRO DECLARATIONS ---------------------------------------------------------------- */
#define	_SPI_CONTROLLER_VAL_OP_CSH					(0x00)
#define	_SPI_CONTROLLER_VAL_OP_CSL					(0x01)
#define	_SPI_CONTROLLER_VAL_OP_CK					(0x02)
#define	_SPI_CONTROLLER_VAL_OP_OUTS					(0x08)
#define	_SPI_CONTROLLER_VAL_OP_OUTD					(0x09)
#define	_SPI_CONTROLLER_VAL_OP_OUTQ					(0x0A)
#define	_SPI_CONTROLLER_VAL_OP_INS					(0x0C)
#define	_SPI_CONTROLLER_VAL_OP_INS0					(0x0D)
#define	_SPI_CONTROLLER_VAL_OP_IND					(0x0E)
#define	_SPI_CONTROLLER_VAL_OP_INQ					(0x0F)
#define	_SPI_CONTROLLER_VAL_OP_OS2IS				(0x10)
#define	_SPI_CONTROLLER_VAL_OP_OS2ID				(0x11)
#define	_SPI_CONTROLLER_VAL_OP_OS2IQ				(0x12)
#define	_SPI_CONTROLLER_VAL_OP_OD2IS				(0x13)
#define	_SPI_CONTROLLER_VAL_OP_OD2ID				(0x14)
#define	_SPI_CONTROLLER_VAL_OP_OD2IQ				(0x15)
#define	_SPI_CONTROLLER_VAL_OP_OQ2IS				(0x16)
#define	_SPI_CONTROLLER_VAL_OP_OQ2ID				(0x17)
#define	_SPI_CONTROLLER_VAL_OP_OQ2IQ				(0x18)
#define	_SPI_CONTROLLER_VAL_OP_OSNIS				(0x19)
#define	_SPI_CONTROLLER_VAL_OP_ODNID				(0x1A)


#define isParallelNAND				0

#define IS_SPIFLASH		((spi_type() == SPI_CONTROLLER_SPI_NOR))
#define IS_NANDFLASH	((spi_type() == SPI_CONTROLLER_SPI_NAND) || isParallelNAND)
//#define isSpiControllerECC	(GET_IS_SPI_CONTROLLER_ECC())

#define isSpiNandAndCtrlECC 	0//((IS_NANDFLASH && isSpiControllerECC) || isParallelNAND)

/* TYPE DECLARATIONS ----------------------------------------------------------------- */
typedef enum{
	SPI_CONTROLLER_SPEED_SINGLE =0,
	SPI_CONTROLLER_SPEED_DUAL,
	SPI_CONTROLLER_SPEED_QUAD

} SPI_CONTROLLER_SPEED_T;

typedef enum{
	SPI_CONTROLLER_RTN_NO_ERROR =0,
	SPI_CONTROLLER_RTN_SET_OPFIFO_ERROR,
	SPI_CONTROLLER_RTN_READ_DATAPFIFO_ERROR,
	SPI_CONTROLLER_RTN_WRITE_DATAPFIFO_ERROR,

	SPI_CONTROLLER_RTN_DEF_NO
} SPI_CONTROLLER_RTN_T;


typedef enum{
	SPI_CONTROLLER_MODE_AUTO=0,
	SPI_CONTROLLER_MODE_MANUAL,
	SPI_CONTROLLER_MODE_DMA,
	SPI_CONTROLLER_MODE_NO
} SPI_CONTROLLER_MODE_T;

typedef enum{
	SPI_CONTROLLER_SPI_NAND=0,
	SPI_CONTROLLER_SPI_NOR,
	SPI_CONTROLLER_PARALLEL_NAND
} SPI_CONTROLLER_SPI_TYPE_T;

typedef struct SPI_CONTROLLER_CONFIGURE
{
	SPI_CONTROLLER_MODE_T	mode;
	u32						dummy_byte_num;
} SPI_CONTROLLER_CONF_T;
/* EXPORTED SUBPROGRAM SPECIFICATION ------------------------------------------------- */
SPI_CONTROLLER_RTN_T SPI_CONTROLLER_Set_Configure( SPI_CONTROLLER_CONF_T *ptr_spi_controller_conf_t );
SPI_CONTROLLER_RTN_T SPI_CONTROLLER_Get_Configure( SPI_CONTROLLER_CONF_T *ptr_rtn_spi_controller_conf_t );

/*------------------------------------------------------------------------------------
 * FUNCTION: SPI_CONTROLLER_RTN_T SPI_CONTROLLER_Enable_Manual_Mode( void )
 * PURPOSE : To provide interface for enable SPI Controller Manual Mode Enable.
 * AUTHOR  : Chuck Kuo
 * CALLED BY
 *   -
 * CALLS
 *   -
 * PARAMs  :
 *   INPUT : None
 *   OUTPUT: None
 * RETURN  : SPI_RTN_NO_ERROR - Successful.   Otherwise - Failed.
 * NOTES   :
 * MODIFICTION HISTORY:
 * Date 2014/12/16 by Chuck Kuo - The first revision for this function.
 *------------------------------------------------------------------------------------
 */
SPI_CONTROLLER_RTN_T SPI_CONTROLLER_Enable_Manual_Mode( void );

/*------------------------------------------------------------------------------------
 * FUNCTION: SPI_CONTROLLER_RTN_T SPI_CONTROLLER_Write_One_Byte( u8  data )
 * PURPOSE : To provide interface for write one byte to SPI bus.
 * AUTHOR  : Chuck Kuo
 * CALLED BY
 *   -
 * CALLS
 *   -
 * PARAMs  :
 *   INPUT : data - The data variable of this function.
 *   OUTPUT: None
 * RETURN  : SPI_RTN_NO_ERROR - Successful.   Otherwise - Failed.
 * NOTES   :
 * MODIFICTION HISTORY:
 * Date 2014/12/16 by The first revision for this function.
 *------------------------------------------------------------------------------------
 */
SPI_CONTROLLER_RTN_T SPI_CONTROLLER_Write_One_Byte( u8  data );
SPI_CONTROLLER_RTN_T SPI_CONTROLLER_Write_One_Byte_With_Cmd(u8 op_cmd, u8 data );

/*------------------------------------------------------------------------------------
 * FUNCTION: SPI_CONTROLLER_RTN_T SPI_CONTROLLER_WRITE_NBYTES( u8                        *ptr_data,
 *                                                             u32                       len,
 *                                                             SPI_CONTROLLER_SPEED_T    speed )
 * PURPOSE : To provide interface for write N bytes to SPI bus.
 * AUTHOR  : Chuck Kuo
 * CALLED BY
 *   -
 * CALLS
 *   -
 * PARAMs  :
 *   INPUT : ptr_data  - The data variable of this function.
 *           len   - The len variable of this function.
 *           speed - The speed variable of this function.
 *   OUTPUT: None
 * RETURN  : SPI_RTN_NO_ERROR - Successful.   Otherwise - Failed.
 * NOTES   :
 * MODIFICTION HISTORY:
 * Date 2014/12/16 by Chuck Kuo - The first revision for this function.
 *------------------------------------------------------------------------------------
 */
SPI_CONTROLLER_RTN_T SPI_CONTROLLER_Write_NByte( u8                         *ptr_data,
                                                 u32                        len,
                                                 SPI_CONTROLLER_SPEED_T     speed     );

/*------------------------------------------------------------------------------------
 * FUNCTION: SPI_CONTROLLER_RTN_T SPI_CONTROLLER_READ_NBYTES( u8                         *ptr_rtn_data,
 *                                                            u8                         len,
 *                                                            SPI_CONTROLLER_SPEED_T     speed     )
 * PURPOSE : To provide interface for read N bytes from SPI bus.
 * AUTHOR  : Chuck Kuo
 * CALLED BY
 *   -
 * CALLS
 *   -
 * PARAMs  :
 *   INPUT : len       - The len variable of this function.
 *           speed     - The speed variable of this function.
 *   OUTPUT: ptr_rtn_data  - The ptr_rtn_data variable of this function.
 * RETURN  : SPI_RTN_NO_ERROR - Successful.   Otherwise - Failed.
 * NOTES   :
 * MODIFICTION HISTORY:
 * Date 2014/12/16 by Chuck Kuo - The first revision for this function.
 *------------------------------------------------------------------------------------
 */
SPI_CONTROLLER_RTN_T SPI_CONTROLLER_Read_NByte( u8                      *ptr_rtn_data,
                                                u32                     len,
                                                SPI_CONTROLLER_SPEED_T  speed         );

/*------------------------------------------------------------------------------------
 * FUNCTION: SPI_CONTROLLER_RTN_T SPI_CONTROLLER_Chip_Select_Low( void )
 * PURPOSE : To provide interface for set chip select low in SPI bus.
 * AUTHOR  : Chuck Kuo
 * CALLED BY
 *   -
 * CALLS
 *   -
 * PARAMs  :
 *   INPUT : None
 *   OUTPUT: None
 * RETURN  : SPI_RTN_NO_ERROR - Successful.   Otherwise - Failed.
 * NOTES   :
 * MODIFICTION HISTORY:
 * Date 2014/12/16 by Chuck Kuo - The first revision for this function.
 *------------------------------------------------------------------------------------
 */
SPI_CONTROLLER_RTN_T SPI_CONTROLLER_Chip_Select_Low( void );

/*------------------------------------------------------------------------------------
 * FUNCTION: SPI_CONTROLLER_RTN_T SPI_CONTROLLER_Chip_Select_High( void )
 * PURPOSE : To provide interface for set chip select high in SPI bus.
 * AUTHOR  : Chuck Kuo
 * CALLED BY
 *   -
 * CALLS
 *   -
 * PARAMs  :
 *   INPUT : None
 *   OUTPUT: None
 * RETURN  : SPI_RTN_NO_ERROR - Successful.   Otherwise - Failed.
 * NOTES   :
 * MODIFICTION HISTORY:
 * Date 2014/12/16 by Chuck Kuo - The first revision for this function.
 *------------------------------------------------------------------------------------
 */
SPI_CONTROLLER_RTN_T SPI_CONTROLLER_Chip_Select_High( void );

void SPI_CONTROLLER_DEBUG_ENABLE( void );

/*------------------------------------------------------------------------------------
 * FUNCTION: void SPI_NAND_DEBUG_DISABLE( void )
 * PURPOSE : To disable to printf debug message of SPI NAND driver.
 * CALLED BY
 *   -
 * CALLS
 *   -
 * PARAMs  :
 *   INPUT : None
 *   OUTPUT: None
 * RETURN  : SPI_RTN_NO_ERROR - Successful.   Otherwise - Failed.
 * NOTES   :
 * MODIFICTION HISTORY:
 * Date 2015/01/20 by Chuck Kuo - The first revision for this function.
 *
 *------------------------------------------------------------------------------------
 */
void SPI_CONTROLLER_DEBUG_DISABLE( void );
u32 get_sfc_strap(void);
/*------------------------------------------------------------------------------------
 * FUNCTION: spi_set_clock_speed(u32 clock_factor)
 * PURPOSE : To set SPI clock. 
 *                 clock_factor = 0
 *                   EN7512: SPI clock = 500MHz / 40 = 12.5MHz
 *                   EN7522: SPI clock = 400MHz / 40 = 10MHz
 *                 clock_factor > 0
 *                   EN7512: SPI clock = 500MHz / (clock_factor * 2)
 *                   EN7522: SPI clock = 400MHz / (clock_factor * 2)
 * CALLED BY
 *   -
 * CALLS
 *   -
 * PARAMs  :
 *   INPUT : clock_factor - The SPI clock divider.
 * RETURN  : NONE.
 * NOTES   :
 * MODIFICTION HISTORY:
 *
 *------------------------------------------------------------------------------------
 */
void spi_set_clock_speed(u32 clk);
void set_spi_quad_mode_shared_pin(void);
SPI_CONTROLLER_SPI_TYPE_T spi_type(void);

#endif /* ifndef __SPI_CONTROLLER_H__ */
/* End of [spi_controller.h] package */

