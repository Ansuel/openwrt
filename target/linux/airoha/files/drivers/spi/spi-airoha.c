// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 AIROHA Inc.
 * Author: Lorenzo Bianconi <lorenzo@kernel.org>
 * Author: Ray Liu <ray.liu@airoha.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/iopoll.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/spinand.h>
#include <asm/cacheflush.h>
#include <linux/spi/spi.h> 
#include <linux/regmap.h>

/* SPI */
#define REG_SPI_CTRL_BASE			0x1FA10000

#define REG_SPI_CTRL_READ_MODE			0x0000
#define REG_SPI_CTRL_READ_IDLE_EN		0x0004
#define REG_SPI_CTRL_SIDLY			0x0008
#define REG_SPI_CTRL_CSHEXT			0x000c
#define REG_SPI_CTRL_CSLEXT			0x0010

#define REG_SPI_CTRL_MTX_MODE_TOG		0x0014
#define SPI_CTRL_MTX_MODE_TOG			GENMASK(3, 0)

#define REG_SPI_CTRL_RDCTL_FSM			0x0018
#define SPI_CTRL_RDCTL_FSM			GENMASK(3, 0)

#define REG_SPI_CTRL_MACMUX_SEL			0x001c

#define REG_SPI_CTRL_MANUAL_EN			0x0020
#define SPI_CTRL_MANUAL_EN			BIT(0)

#define REG_SPI_CTRL_OPFIFO_EMPTY		0x0024
#define SPI_CTRL_OPFIFO_EMPTY			BIT(0)

#define REG_SPI_CTRL_OPFIFO_WDATA		0x0028
#define SPI_CTRL_OPFIFO_LEN			GENMASK(8, 0)
#define SPI_CTRL_OPFIFO_OP			GENMASK(13, 9)

#define REG_SPI_CTRL_OPFIFO_FULL 		0x002c
#define SPI_CTRL_OPFIFO_FULL			BIT(0)

#define REG_SPI_CTRL_OPFIFO_WR			0x0030
#define SPI_CTRL_OPFIFO_WR			BIT(0)

#define REG_SPI_CTRL_DFIFO_FULL			0x0034
#define SPI_CTRL_DFIFO_FULL			BIT(0)

#define REG_SPI_CTRL_DFIFO_WDATA		0x0038
#define SPI_CTRL_DFIFO_WDATA			GENMASK(7, 0)

#define REG_SPI_CTRL_DFIFO_EMPTY		0x003c
#define SPI_CTRL_DFIFO_EMPTY			BIT(0)

#define REG_SPI_CTRL_DFIFO_RD			0x0040
#define SPI_CTRL_DFIFO_RD			BIT(0)

#define REG_SPI_CTRL_DFIFO_RDATA		0x0044
#define SPI_CTRL_DFIFO_RDATA			GENMASK(7, 0)

#define REG_SPI_CTRL_DUMMY			0x0080
#define SPI_CTRL_CTRL_DUMMY			GENMASK(3, 0)

#define REG_SPI_CTRL_PROBE_SEL			0x0088
#define REG_SPI_CTRL_INTERRUPT			0x0090
#define REG_SPI_CTRL_INTERRUPT_EN		0x0094
#define REG_SPI_CTRL_SI_CK_SEL			0x009c
#define REG_SPI_CTRL_SW_CFGNANDADDR_VAL		0x010c
#define REG_SPI_CTRL_SW_CFGNANDADDR_EN		0x0110
#define REG_SPI_CTRL_SFC_STRAP			0x0114

#define REG_SPI_CTRL_NFI2SPI_EN			0x0130
#define SPI_CTRL_NFI2SPI_EN			BIT(0)

/* NFI2SPI */
#define REG_SPI_NFI_CNFG			0x0000
#define SPI_NFI_DMA_BURST_EN			BIT(2)
#define SPI_NFI_HW_ECC_EN			BIT(8)
#define SPI_NFI_AUTO_FDM_EN			BIT(9)

#define REG_SPI_NFI_PAGEFMT			0x0004
#define SPI_NFI_PAGE_SIZE			GENMASK(1, 0)
#define SPI_NFI_SPARE_SIZE			GENMASK(5, 4)

#define REG_SPI_NFI_CON				0x0008
#define SPI_NFI_FIFO_FLUSH			BIT(0)
#define SPI_NFI_RST				BIT(1)
#define SPI_NFI_SEC_NUM				GENMASK(15, 12)

#define REG_SPI_NFI_INTR_EN			0x0010
#define SPI_NFI_RD_DONE_EN			BIT(0)
#define SPI_NFI_WR_DONE_EN			BIT(1)
#define SPI_NFI_RST_DONE_EN			BIT(2)
#define SPI_NFI_ERASE_DONE_EN			BIT(3)
#define SPI_NFI_BUSY_RETURN_EN			BIT(4)
#define SPI_NFI_ACCESS_LOCK_EN			BIT(5)
#define SPI_NFI_AHB_DONE_EN			BIT(6)
#define SPI_NFI_ALL_IRQ_EN			(SPI_NFI_RD_DONE_EN | \
						 SPI_NFI_WR_DONE_EN | \
						 SPI_NFI_RST_DONE_EN | \
						 SPI_NFI_ERASE_DONE_EN | \
						 SPI_NFI_BUSY_RETURN_EN | \
						 SPI_NFI_ACCESS_LOCK_EN | \
						 SPI_NFI_AHB_DONE_EN)

#define REG_SPI_NFI_INTR			0x0014
#define REG_SPI_NFI_CMD				0x0020

#define REG_SPI_NFI_ADDR_NOB			0x0030
#define SPI_NFI_ROW_ADDR_NOB			GENMASK(6, 4)

#define REG_SPI_NFI_STA				0x0060
#define REG_SPI_NFI_FIFOSTA			0x0064
#define REG_SPI_NFI_STRADDR			0x0080
#define REG_SPI_NFI_FDM0L			0x00a0
#define REG_SPI_NFI_FDM0M			0x00a4
#define REG_SPI_NFI_FDM7L			0x00d8
#define REG_SPI_NFI_FDM7M			0x00dc
#define REG_SPI_NFI_FIFODATA0			0x0190
#define REG_SPI_NFI_FIFODATA1			0x0194
#define REG_SPI_NFI_FIFODATA2			0x0198
#define REG_SPI_NFI_FIFODATA3			0x019c
#define REG_SPI_NFI_MASTERSTA			0x0224

#define REG_SPI_NFI_SECCUS_SIZE			0x022c
#define SPI_NFI_CUS_SEC_SIZE			GENMASK(12, 0)
#define SPI_NFI_CUS_SEC_SIZE_EN			BIT(16)

#define REG_SPI_NFI_RD_CTL2			0x0510
#define REG_SPI_NFI_RD_CTL3			0x0514
#define REG_SPI_NFI_PG_CTL1			0x0524
#define REG_SPI_NFI_PG_CTL2			0x0528
#define REG_SPI_NFI_NOR_PROG_ADDR		0x052c
#define REG_SPI_NFI_NOR_RD_ADDR			0x0534
#define REG_SPI_NFI_SNF_MISC_CTL		0x0538
#define REG_SPI_NFI_SNF_MISC_CTL2		0x053c
#define REG_SPI_NFI_SNF_STA_CTL1		0x0550
#define REG_SPI_NFI_SNF_STA_CTL2		0x0554

#define REG_SPI_NFI_SNF_NFI_CNFG		0x055c
#define SPI_NFI_SPI_MODE			BIT(0)

/* SPI NAND Protocol OP */
#define _SPI_NAND_OP_GET_FEATURE			0x0f    /* Get Feature */
#define _SPI_NAND_OP_SET_FEATURE			0x1f    /* Set Feature */
#define _SPI_NAND_OP_PAGE_READ				0x13    /* Load page data into cache of SPI NAND chip */
#define _SPI_NAND_OP_READ_FROM_CACHE_SINGLE		0x03    /* Read data from cache of SPI NAND chip, single speed*/
#define _SPI_NAND_OP_READ_FROM_CACHE_SINGLE_FAST	0x0b    /* Read data from cache of SPI NAND chip, single speed*/
#define _SPI_NAND_OP_READ_FROM_CACHE_DUAL		0x3b    /* Read data from cache of SPI NAND chip, dual speed*/
#define _SPI_NAND_OP_READ_FROM_CACHE_QUAD		0x6b    /* Read data from cache of SPI NAND chip, quad speed*/
#define _SPI_NAND_OP_WRITE_ENABLE			0x06    /* Enable write data to  SPI NAND chip */
#define _SPI_NAND_OP_WRITE_DISABLE			0x04    /* Reseting the Write Enable Latch (WEL) */
#define _SPI_NAND_OP_PROGRAM_LOAD_SINGLE		0x02    /* Write data into cache of SPI NAND chip with cache reset, single speed */
#define _SPI_NAND_OP_PROGRAM_LOAD_QUAD			0x32    /* Write data into cache of SPI NAND chip with cache reset, quad speed */
#define _SPI_NAND_OP_PROGRAM_LOAD_RAMDOM_SINGLE		0x84    /* Write data into cache of SPI NAND chip, single speed */
#define _SPI_NAND_OP_PROGRAM_LOAD_RAMDON_QUAD		0x34    /* Write data into cache of SPI NAND chip, quad speed */
#define _SPI_NAND_OP_PROGRAM_EXECUTE			0x10    /* Write data from cache into SPI NAND chip */
#define _SPI_NAND_OP_READ_ID				0x9f    /* Read Manufacture ID and Device ID */
#define _SPI_NAND_OP_BLOCK_ERASE			0xd8    /* Erase Block */
#define _SPI_NAND_OP_RESET				0xff    /* Reset */
#define _SPI_NAND_OP_DIE_SELECT				0xc2    /* Die Select */

#define WRITE_NFI_REG(a,b) writel(b, as->nfi_base + a)
#define READ_NFI_REG(a) readl(as->nfi_base + a)
#define WRITE_NFI_REG_WITH_MASK(a,b,c) WRITE_NFI_REG(a, (READ_NFI_REG(a) & ~(b)) | (c))

enum airoha_spi_mode {
	SPI_MODE_AUTO,
	SPI_MODE_MANUAL,
	SPI_MODE_DMA,
	SPI_MODE_NO
};

typedef enum{
    SPI_CONTROLLER_CHIP_SELECT_HIGH=0,
    SPI_CONTROLLER_CHIP_SELECT_LOW,
} SPI_CONTROLLER_CHIP_SELECT_T;

struct airoha_spi_dev {
	size_t buf_len;

	u8 *tx_buf;
	dma_addr_t tx_dma_addr;
	u8 *rx_buf;
	dma_addr_t rx_dma_addr;
};

struct airoha_snand {
    struct spi_controller *master;
    struct device *dev;
	struct regmap *regmap_ctrl;
	struct regmap *regmap_nfi;
    void __iomem *spi_base;
    void __iomem *nfi_base;
    int irq;

	struct {
		size_t page_size;
		size_t sec_size;
		unsigned char sec_num;
		unsigned char spare_size;
	} nfi_cfg;

    unsigned long current_page_num;
    bool data_need_update;
    bool autofmt; // <-- soc controller ecc need to implement this feature
};

static int airoha_spi_set_fifo_op(struct airoha_snand *as, u8 op_cmd,
				  int op_len)
{
	int err;
	u32 val;

	err = regmap_write(as->regmap_ctrl, REG_SPI_CTRL_OPFIFO_WDATA,
			   FIELD_PREP(SPI_CTRL_OPFIFO_LEN, op_len) |
			   FIELD_PREP(SPI_CTRL_OPFIFO_OP, op_cmd));
	if (err)
		return err;

	err = regmap_read_poll_timeout(as->regmap_ctrl,
				       REG_SPI_CTRL_OPFIFO_FULL,
				       val, !(val & SPI_CTRL_OPFIFO_FULL),
				       USEC_PER_MSEC, 250 * USEC_PER_MSEC);
	if (err)
		return err;

	err = regmap_write(as->regmap_ctrl, REG_SPI_CTRL_OPFIFO_WR,
			   SPI_CTRL_OPFIFO_WR);
	if (err)
		return err;

	return regmap_read_poll_timeout(as->regmap_ctrl,
					REG_SPI_CTRL_OPFIFO_EMPTY,
					val, (val & SPI_CTRL_OPFIFO_EMPTY),
					USEC_PER_MSEC, 250 * USEC_PER_MSEC);
}

static int airoha_spi_set_cs(struct airoha_snand *as, SPI_CONTROLLER_CHIP_SELECT_T cs)
{
	return airoha_spi_set_fifo_op(as, cs, 1);
}

static int airoha_spi_write_data_to_fifo(struct airoha_snand *as,
					 const u8 *data, int len)
{
	int i;
	
	for (i = 0; i < len; i++) {
		int err;
		u32 val;

		/* 1. Wait until dfifo is not full */
		err = regmap_read_poll_timeout(as->regmap_ctrl,
					       REG_SPI_CTRL_DFIFO_FULL, val,
					       !(val & SPI_CTRL_DFIFO_FULL),
					       USEC_PER_MSEC,
					       250 * USEC_PER_MSEC);
		if (err)
			return err;

		/* 2. Write data to register DFIFO_WDATA */
		err = regmap_write(as->regmap_ctrl, REG_SPI_CTRL_DFIFO_WDATA,
				   FIELD_PREP(SPI_CTRL_DFIFO_WDATA, data[i]));
		if (err)
			return err;

		/* 3. Wait until dfifo is not full */
		err = regmap_read_poll_timeout(as->regmap_ctrl,
					       REG_SPI_CTRL_DFIFO_FULL, val,
					       !(val & SPI_CTRL_DFIFO_FULL),
					       USEC_PER_MSEC,
					       250 * USEC_PER_MSEC);
		if (err)
			return err;
	}

	return 0;
}

static int airoha_spi_read_data_from_fifo(struct airoha_snand *as, u8 *ptr,
					  int len)
{
	int i;

	for (i = 0; i < len; i++) {
		int err;
		u32 val;

		/* 1. wait until dfifo is not empty */
		err = regmap_read_poll_timeout(as->regmap_ctrl,
					       REG_SPI_CTRL_DFIFO_EMPTY, val,
					       !(val & SPI_CTRL_DFIFO_EMPTY),
					       USEC_PER_MSEC,
					       250 * USEC_PER_MSEC);
		if (err)
			return err;

		/* 2. read from dfifo to register DFIFO_RDATA */
		err = regmap_read(as->regmap_ctrl,
				  REG_SPI_CTRL_DFIFO_RDATA, &val);
		if (err)
			return err;

		ptr[i] = FIELD_GET(SPI_CTRL_DFIFO_RDATA, val);
		/* 3. enable register DFIFO_RD to read next byte */
		err = regmap_write(as->regmap_ctrl, REG_SPI_CTRL_DFIFO_RD,
				   SPI_CTRL_DFIFO_RD);
		if (err)
			return err;
	}

	return 0;
}

static int airoha_spi_set_mode(struct airoha_snand *as,
			       enum airoha_spi_mode mode)
{
	int err;

	switch (mode) {
	case SPI_MODE_MANUAL: {
		u32 val;

		err = regmap_write(as->regmap_ctrl,
				   REG_SPI_CTRL_NFI2SPI_EN, 0);
		if (err)
			return err;

		err = regmap_write(as->regmap_ctrl,
				   REG_SPI_CTRL_READ_IDLE_EN, 0);
		if (err)
			return err;

		err = regmap_read_poll_timeout(as->regmap_ctrl,
					       REG_SPI_CTRL_RDCTL_FSM, val,
					       !(val & SPI_CTRL_RDCTL_FSM),
					       USEC_PER_MSEC,
					       250 * USEC_PER_MSEC);
		if (err)
			return err;

		err = regmap_write(as->regmap_ctrl,
				   REG_SPI_CTRL_MTX_MODE_TOG, 9);
		if (err)
			return err;

		err = regmap_write(as->regmap_ctrl, REG_SPI_CTRL_MANUAL_EN,
				   SPI_CTRL_MANUAL_EN);
		if (err)
			return err;
		break;
	}
	case SPI_MODE_DMA:
		err = regmap_write(as->regmap_ctrl, REG_SPI_CTRL_NFI2SPI_EN,
				   SPI_CTRL_MANUAL_EN);
		if (err < 0)
			return err;

		err = regmap_write(as->regmap_ctrl,
				   REG_SPI_CTRL_MTX_MODE_TOG, 0x0);
		if (err < 0)
			return err;

		err = regmap_write(as->regmap_ctrl,
				   REG_SPI_CTRL_MANUAL_EN, 0x0);
		if (err < 0)
			return err;
		break;
	case SPI_MODE_AUTO:
	default:
		break;
	}

	return regmap_write(as->regmap_ctrl, REG_SPI_CTRL_DUMMY, 0);
}

#define MAX_TRANSFER_SIZE	511
static int airoha_spi_write_data(struct airoha_snand *as, u8 cmd,
				 const u8 *data, int len)
{
	int i = 0;

	while (i < len) {
		int err, data_len = min(len, MAX_TRANSFER_SIZE);

		err = airoha_spi_set_fifo_op(as, cmd, data_len);
		if (err)
			return err;

		err = airoha_spi_write_data_to_fifo(as, &data[i], data_len);
		if (err < 0)
			return err;

		i += data_len;
	}

	return 0;
}

static int airoha_spi_read_data(struct airoha_snand *as, u8 *data, int len)
{
	int i = 0;

	while (i < len) {
		int err, data_len = min(len, MAX_TRANSFER_SIZE);

		err = airoha_spi_set_fifo_op(as, 0xc, data_len);
		if (err)
			return err;

		err = airoha_spi_read_data_from_fifo(as, &data[i], data_len);
		if (err < 0)
			return err;

		i += data_len;
	}

	return 0;
}

static int airoha_spi_nfi_init(struct airoha_snand *as)
{
	int err;

	/* switch to SNFI mode */
	err = regmap_write(as->regmap_nfi, REG_SPI_NFI_SNF_NFI_CNFG,
			   SPI_NFI_SPI_MODE);
	if (err)
		return err;

	/* Enable DMA */
	return regmap_update_bits(as->regmap_nfi, REG_SPI_NFI_INTR_EN,
				  SPI_NFI_ALL_IRQ_EN, SPI_NFI_AHB_DONE_EN);
}

static int airoha_spi_nfi_config(struct airoha_snand *as)
{
	int err;
	u32 val;

	err = regmap_write(as->regmap_nfi, REG_SPI_NFI_CON,
			   SPI_NFI_FIFO_FLUSH | SPI_NFI_RST);
	if (err)
		return err;

	/* auto FDM */
	err = regmap_clear_bits(as->regmap_nfi, REG_SPI_NFI_CNFG,
				SPI_NFI_AUTO_FDM_EN);
	if (err)
		return err;

	/* HW ECC */
	err = regmap_clear_bits(as->regmap_nfi, REG_SPI_NFI_CNFG,
				SPI_NFI_HW_ECC_EN);
	if (err)
		return err;

	/* DMA Burst */
	err = regmap_set_bits(as->regmap_nfi, REG_SPI_NFI_CNFG,
			      SPI_NFI_AHB_DONE_EN);
	if (err)
		return err;

	/* page format */
	switch (as->nfi_cfg.spare_size) {
	case 26:
		val = FIELD_PREP(SPI_NFI_SPARE_SIZE, 0x1);
		break;
	case 27:
		val = FIELD_PREP(SPI_NFI_SPARE_SIZE, 0x2);
		break;
	case 28:
		val = FIELD_PREP(SPI_NFI_SPARE_SIZE, 0x3);
		break;
	default:
		val = FIELD_PREP(SPI_NFI_SPARE_SIZE, 0x0);
		break;
	}

	err = regmap_update_bits(as->regmap_nfi, REG_SPI_NFI_PAGEFMT,
				 SPI_NFI_SPARE_SIZE, val);
	if (err)
		return err;

	switch (as->nfi_cfg.page_size) {
	case 2048:
	    val = FIELD_PREP(SPI_NFI_PAGE_SIZE, 0x1);
	    break;
	case 4096:
	    val = FIELD_PREP(SPI_NFI_PAGE_SIZE, 0x2);
	    break;
	default:
	    val = FIELD_PREP(SPI_NFI_PAGE_SIZE, 0x0);
	    break;
	}

	err = regmap_update_bits(as->regmap_nfi, REG_SPI_NFI_PAGEFMT,
				 SPI_NFI_PAGE_SIZE, val);
	if (err)
		return err;

	/* sec num */
	val = FIELD_PREP(SPI_NFI_SEC_NUM, as->nfi_cfg.sec_num);
	err = regmap_update_bits(as->regmap_nfi, REG_SPI_NFI_CON,
				 SPI_NFI_SEC_NUM, val);
	if (err)
		return err;
	/* enable cust sec size */
	err = regmap_set_bits(as->regmap_nfi, REG_SPI_NFI_SECCUS_SIZE,
			      SPI_NFI_CUS_SEC_SIZE_EN);
	if (err)
		return err;

	/* set cust sec size */
	val = FIELD_PREP(SPI_NFI_CUS_SEC_SIZE, as->nfi_cfg.sec_size);
	return regmap_update_bits(as->regmap_nfi, REG_SPI_NFI_SECCUS_SIZE,
				  SPI_NFI_CUS_SEC_SIZE, val);
}

static bool airoha_spi_is_page_ops(const struct spi_mem_op *op)
{
	if (op->addr.nbytes != 2)
		return false;

	if (op->addr.buswidth != 1 && op->addr.buswidth != 2 &&
	    op->addr.buswidth != 4)
		return false;

	switch (op->data.dir) {
	case SPI_MEM_DATA_IN:
		/* check dummy cycle first */
		if (op->dummy.nbytes * BITS_PER_BYTE / op->dummy.buswidth > 0xf)
			return false;

		/* quad io / quad out */
		if ((op->addr.buswidth == 4 || op->addr.buswidth == 1) &&
		    op->data.buswidth == 4)
			return true;
		
		/* dual io / dual out */
		if ((op->addr.buswidth == 2 || op->addr.buswidth == 1) &&
		    op->data.buswidth == 2)
			return true;

		/* standard spi */
		if (op->addr.buswidth == 1 && op->data.buswidth == 1)
			return true;
		break;
	case SPI_MEM_DATA_OUT:
		/* check dummy cycle first */
		if (op->dummy.nbytes)
			return false;

		/* program load quad out */
		if (op->addr.buswidth == 1 && op->data.buswidth == 4)
			return true;

		/* standard spi */
		if (op->addr.buswidth == 1 && op->data.buswidth == 1)
			return true;
	default:
		break;
	}

	return false;
}

static int airoha_spi_adjust_op_size(struct spi_mem *mem,
				     struct spi_mem_op *op)
{
	struct airoha_snand *as = spi_master_get_devdata(mem->spi->master);
	size_t len;

	if (airoha_spi_is_page_ops(op)) {
		if (as->autofmt)
			return 0;

		len = as->nfi_cfg.sec_size + as->nfi_cfg.spare_size;
		len *= as->nfi_cfg.sec_num;
		op->data.nbytes = min(op->data.nbytes, len);
	} else {
		len = 1 + op->addr.nbytes + op->dummy.nbytes;
		if (len >= 160)
			return -EOPNOTSUPP;

		op->data.nbytes = min(op->data.nbytes, 160 - len);
	}

	return 0;
}

static bool airoha_spi_supports_op(struct spi_mem *mem,
				   const struct spi_mem_op *op)
{
	if (!spi_mem_default_supports_op(mem, op))
		return false;

	if (op->cmd.buswidth != 1)
		return false;

	if (airoha_spi_is_page_ops(op))
		return true;

	return (!op->addr.nbytes || op->addr.buswidth == 1) &&
	       (!op->dummy.nbytes || op->dummy.buswidth == 1) &&
	       (!op->data.nbytes || op->data.buswidth == 1);
}

static int airoha_spi_dirmap_create(struct spi_mem_dirmap_desc *desc)
{
	struct airoha_spi_dev *aspi_dev = spi_get_ctldata(desc->mem->spi);

	if (!aspi_dev->rx_buf || !aspi_dev->tx_buf)
		return -EINVAL;

	if (desc->info.offset + desc->info.length > U32_MAX)
		return -EINVAL;

	if (!airoha_spi_supports_op(desc->mem, &desc->info.op_tmpl))
		return -EOPNOTSUPP;

	return 0;
}

static ssize_t airoha_spi_dirmap_read(struct spi_mem_dirmap_desc *desc,
				      u64 offs, size_t len, void *buf)
{
	struct spi_device *spi = desc->mem->spi;
	struct airoha_spi_dev *aspi_dev = spi_get_ctldata(spi);
    struct airoha_snand *as = spi_master_get_devdata(spi->master);
    struct spi_mem_op *op = &desc->info.op_tmpl;
    unsigned int rd_mode;
    int ret;
    unsigned int val;

    if(as->data_need_update)
    {
        as->data_need_update = false;
    }
    else
    {
        // memcpy(buf, as->rx_buf + offs, len);
        return len;
    }

    if(op->cmd.opcode == _SPI_NAND_OP_READ_FROM_CACHE_SINGLE || op->cmd.opcode == _SPI_NAND_OP_READ_FROM_CACHE_SINGLE_FAST)
    {
        rd_mode = 0x0;
    }
    else if(op->cmd.opcode == _SPI_NAND_OP_READ_FROM_CACHE_DUAL)
    {
        rd_mode = 0x1;
    }
    else if(op->cmd.opcode == _SPI_NAND_OP_READ_FROM_CACHE_QUAD)
    {
        rd_mode = 0x2;
    }
    else
    {
        rd_mode = 0x0;
    }

    ret = airoha_spi_set_mode(as, SPI_MODE_DMA);
    if (ret < 0)
	    return ret;
    ret = airoha_spi_nfi_config(as);
    if (ret)
	    return ret;
    dma_sync_single_for_device(as->dev, aspi_dev->rx_dma_addr, aspi_dev->buf_len, DMA_FROM_DEVICE);
    mb();
    // set dma addr
    WRITE_NFI_REG(REG_SPI_NFI_STRADDR, aspi_dev->rx_dma_addr);
    // set cust sec size
    WRITE_NFI_REG_WITH_MASK(REG_SPI_NFI_SNF_MISC_CTL2, 0x1fff, (as->nfi_cfg.sec_size * as->nfi_cfg.sec_num) << 0);
    // set read command
    WRITE_NFI_REG(REG_SPI_NFI_RD_CTL2, op->cmd.opcode);
    // set read mode
    WRITE_NFI_REG(REG_SPI_NFI_SNF_MISC_CTL, rd_mode << 16);
    // set read addr
    WRITE_NFI_REG(REG_SPI_NFI_RD_CTL3, 0x0);
    // set nfi read
    WRITE_NFI_REG_WITH_MASK(REG_SPI_NFI_CNFG, 0x7000, (6 << 12));
    WRITE_NFI_REG(REG_SPI_NFI_CNFG, READ_NFI_REG(REG_SPI_NFI_CNFG) | 0x0002);
    WRITE_NFI_REG(REG_SPI_NFI_CNFG, READ_NFI_REG(REG_SPI_NFI_CNFG) | 0x0001);
    WRITE_NFI_REG(REG_SPI_NFI_CMD, 0x00);
    // trigger dma start read
    WRITE_NFI_REG(REG_SPI_NFI_CON, READ_NFI_REG(REG_SPI_NFI_CON) & ~(0x0100));
    WRITE_NFI_REG(REG_SPI_NFI_CON, READ_NFI_REG(REG_SPI_NFI_CON) | (0x0100));

    ret = readl_poll_timeout(as->nfi_base + REG_SPI_NFI_SNF_STA_CTL1, val, (val & 0x02000000) != 0, 0, 1000000);
    if(ret)
    {
        printk("[Error] Read DMA : Check READ FROM CACHE Done Timeout ! \n");
        return -1;
    }
    WRITE_NFI_REG(REG_SPI_NFI_SNF_STA_CTL1, READ_NFI_REG(REG_SPI_NFI_SNF_STA_CTL1) | 0x02000000);
    ret = readl_poll_timeout(as->nfi_base + REG_SPI_NFI_INTR, val, (val & 0x0040) != 0, 0, 1000000);
    if(ret)
    {
        printk("[Error] Read DMA : Check AHB Done Timeout ! \n");
        return -1;
    }

    /* Does DMA read need delay for data ready from controller to DRAM */
    udelay(1);
    dma_sync_single_for_cpu(as->dev, aspi_dev->rx_dma_addr, aspi_dev->buf_len, DMA_FROM_DEVICE);
    ret = airoha_spi_set_mode(as, SPI_MODE_MANUAL);
    if (ret < 0)
	    return ret;
    memcpy(buf, aspi_dev->rx_buf + offs, len);
    return len;
}

static ssize_t airoha_spi_dirmap_write(struct spi_mem_dirmap_desc *desc, u64 offs, size_t len, const void *buf)
{
	struct spi_device *spi = desc->mem->spi;
	struct airoha_spi_dev *aspi_dev = spi_get_ctldata(spi);
    struct airoha_snand *as = spi_master_get_devdata(spi->master);
    struct spi_mem_op *op = &desc->info.op_tmpl;
    unsigned int wr_mode;
    int ret;
    unsigned int val;

	//printk("trying to write at offset : %X\n", offs);
    if(op->cmd.opcode == _SPI_NAND_OP_PROGRAM_LOAD_QUAD || op->cmd.opcode == _SPI_NAND_OP_PROGRAM_LOAD_RAMDON_QUAD)
    {
        wr_mode = 0x2;
    }
    else
    {
        wr_mode = 0x0;
    }

    ret = airoha_spi_set_mode(as, SPI_MODE_MANUAL);
    if (ret < 0)
	    return ret;
    dma_sync_single_for_cpu(as->dev, aspi_dev->tx_dma_addr, aspi_dev->buf_len, DMA_TO_DEVICE);
    memcpy(aspi_dev->tx_buf, aspi_dev->rx_buf, aspi_dev->buf_len);
    memcpy(aspi_dev->tx_buf + offs, buf, len);
    dma_sync_single_for_device(as->dev, aspi_dev->tx_dma_addr, aspi_dev->buf_len, DMA_TO_DEVICE);
    mb();
    ret = airoha_spi_set_mode(as, SPI_MODE_DMA);
    if (ret < 0)
	    return ret;
    ret = airoha_spi_nfi_config(as);
    if (ret)
	    return ret;

    WRITE_NFI_REG(REG_SPI_NFI_STRADDR, aspi_dev->tx_dma_addr);
    WRITE_NFI_REG_WITH_MASK(REG_SPI_NFI_SNF_MISC_CTL2, 0x1fff0000, ((as->nfi_cfg.sec_size * as->nfi_cfg.sec_num) << 16));
    WRITE_NFI_REG(REG_SPI_NFI_PG_CTL1, (op->cmd.opcode << 8));
    WRITE_NFI_REG(REG_SPI_NFI_SNF_MISC_CTL, (wr_mode << 16));
    WRITE_NFI_REG(REG_SPI_NFI_PG_CTL2, 0x0);
    WRITE_NFI_REG(REG_SPI_NFI_CNFG, (READ_NFI_REG(REG_SPI_NFI_CNFG) & ~(0x0002)));
    WRITE_NFI_REG_WITH_MASK(REG_SPI_NFI_CNFG, 0x7000, (3 << 12));
    WRITE_NFI_REG(REG_SPI_NFI_CNFG, (READ_NFI_REG(REG_SPI_NFI_CNFG) | (0x0001)));
    WRITE_NFI_REG(REG_SPI_NFI_CMD, 0x80);
    WRITE_NFI_REG(REG_SPI_NFI_CON, (READ_NFI_REG(REG_SPI_NFI_CON) & ~(0x0200)));
    WRITE_NFI_REG(REG_SPI_NFI_CON, (READ_NFI_REG(REG_SPI_NFI_CON) | (0x0200)));
    udelay(1);

    ret = readl_poll_timeout(as->nfi_base + REG_SPI_NFI_INTR, val, (val & 0x0040) != 0, 0, 1000000);
    if(ret)
    {
        printk("[Error] Write DMA : Check LOAD TO CACHE Done Timeout ! \n");
        return -1;
    }

    ret = readl_poll_timeout(as->nfi_base + REG_SPI_NFI_SNF_STA_CTL1, val, (val & 0x04000000) != 0, 0, 1000000);
    if(ret)
    {
        printk("[Error] Read DMA : Check AHB Done Timeout ! \n");
        return -1;
    }
    WRITE_NFI_REG(REG_SPI_NFI_SNF_STA_CTL1, READ_NFI_REG(REG_SPI_NFI_SNF_STA_CTL1) | 0x04000000);
    ret = airoha_spi_set_mode(as, SPI_MODE_MANUAL);
    if (ret < 0)
	    return ret;
    return len;
}

static int airoha_spi_op_transfer(struct airoha_snand *as, const struct spi_mem_op *op)
{
    unsigned int idx;
    u8 opcode = op->cmd.opcode;
    int err;

    if(opcode == _SPI_NAND_OP_PROGRAM_EXECUTE)
    {
        if(op->addr.val == as->current_page_num)
            as->data_need_update = true;
    }
    else if(opcode == _SPI_NAND_OP_PAGE_READ)
    {
        if(!as->data_need_update && op->addr.val == as->current_page_num)
            return 0;
        as->data_need_update = true;
        as->current_page_num = op->addr.val;
    }

    /* Switch To Manual Mode */
    err = airoha_spi_set_mode(as, SPI_MODE_MANUAL);
    if (err < 0)
	    return err;

    err = airoha_spi_set_cs(as, SPI_CONTROLLER_CHIP_SELECT_LOW);
    if (err < 0)
	    return err;

    // opcode part
    err = airoha_spi_write_data(as, 0x8, &opcode, sizeof(opcode));
    if (err)
	    return err;

    // addr part
    for(idx = 0; idx < op->addr.nbytes; idx++)
    {
        unsigned char addr_data = (op->addr.val >> ((op->addr.nbytes - idx - 1) * 8)) & 0xff;
        if(opcode == _SPI_NAND_OP_GET_FEATURE)
        {
            err = airoha_spi_write_data(as, 0x11, &addr_data, 1);
	    if (err)
		    return err;
        }
        else
        {
            err = airoha_spi_write_data(as, 0x8, &addr_data, 1);
	    if (err)
		    return err;
        }
    }

    // dummy part
    for(idx = 0; idx < op->dummy.nbytes; idx++)
    {
        unsigned char data = 0xff;
        err = airoha_spi_write_data(as, 0x8, &data, 1);
	if (err)
		return err;
    }

    // data part
    if(op->data.dir == SPI_MEM_DATA_IN)
    {
        err = airoha_spi_read_data(as, op->data.buf.in, op->data.nbytes);
	if (err)
		return err;
    }
    else
    {
        err = airoha_spi_write_data(as, 0x8, op->data.buf.out, op->data.nbytes);
	if (err)
		return err;
    }

    return airoha_spi_set_cs(as, SPI_CONTROLLER_CHIP_SELECT_HIGH);
}

static int airoha_spi_exec_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
    struct airoha_snand *as = spi_master_get_devdata(mem->spi->master);
    int ret;

    ret = airoha_spi_op_transfer(as, op); // op cmd

    return ret;
}

static const struct spi_controller_mem_ops airoha_spi_mem_ops = {
	.adjust_op_size = airoha_spi_adjust_op_size,
	.supports_op = airoha_spi_supports_op,
	.exec_op = airoha_spi_exec_op,
	.dirmap_create = airoha_spi_dirmap_create,
	.dirmap_read = airoha_spi_dirmap_read,
	.dirmap_write = airoha_spi_dirmap_write,
};

#define SPI_NAND_CACHE_SIZE	(4096 + 256)

static int airoha_spi_setup(struct spi_device *spi)
{
	struct airoha_spi_dev *aspi_dev = spi_get_ctldata(spi);
	struct airoha_snand *as = spi_master_get_devdata(spi->master);

	aspi_dev = kzalloc(sizeof(*aspi_dev), GFP_KERNEL);
	if (!aspi_dev)
		return -ENOMEM;

	spi_set_ctldata(spi, aspi_dev);

	/* prepare device buffer */
	aspi_dev->buf_len = SPI_NAND_CACHE_SIZE;
	aspi_dev->rx_buf = kzalloc(aspi_dev->buf_len, GFP_KERNEL);
	if (!aspi_dev->rx_buf)
		goto error_dev_free;

	aspi_dev->rx_dma_addr = dma_map_single(as->dev, aspi_dev->rx_buf,
					       aspi_dev->buf_len,
					       DMA_FROM_DEVICE);
	if (dma_mapping_error(as->dev, aspi_dev->rx_dma_addr))
		goto error_rx_free;

	aspi_dev->tx_buf = kzalloc(aspi_dev->buf_len, GFP_KERNEL);
	if (!aspi_dev->tx_buf)
		goto error_rx_unmap;

	aspi_dev->tx_dma_addr = dma_map_single(as->dev, aspi_dev->tx_buf,
					       aspi_dev->buf_len,
					       DMA_TO_DEVICE);
	if (dma_mapping_error(as->dev, aspi_dev->tx_dma_addr))
		goto error_tx_free;

	return 0;

error_tx_free:
	kfree(aspi_dev->tx_buf);
error_rx_unmap:
	dma_unmap_single(as->dev, aspi_dev->rx_dma_addr,
			 aspi_dev->buf_len, DMA_FROM_DEVICE);
error_rx_free:
	kfree(aspi_dev->rx_buf);
error_dev_free:
	kfree(aspi_dev);

	return -ENOMEM;
}

static void airoha_spi_cleanup(struct spi_device *spi)
{
	struct airoha_spi_dev *aspi_dev = spi_get_ctldata(spi);
	struct airoha_snand *as = spi_master_get_devdata(spi->master);

	dma_unmap_single(as->dev, aspi_dev->rx_dma_addr,
			 aspi_dev->buf_len, DMA_FROM_DEVICE);
	dma_unmap_single(as->dev, aspi_dev->tx_dma_addr,
			 aspi_dev->buf_len, DMA_TO_DEVICE);

	kfree(aspi_dev->rx_buf);
	kfree(aspi_dev->tx_buf);

	spi_set_ctldata(spi, NULL);
}

static int airoha_spi_nfi_setup(struct airoha_snand *as)
{
	u32 val, sec_size, sec_num;
	int err;

	err = regmap_read(as->regmap_nfi, REG_SPI_NFI_CON, &val);
	if (err)
		return err;

	sec_num = FIELD_GET(SPI_NFI_SEC_NUM, val);

	err = regmap_read(as->regmap_nfi, REG_SPI_NFI_SECCUS_SIZE, &val);
	if (err)
		return err;

	sec_size = FIELD_GET(SPI_NFI_CUS_SEC_SIZE, val);

	/* init default value */
	as->autofmt = false;
	as->nfi_cfg.sec_size = sec_size;
	as->nfi_cfg.sec_num = sec_num;
	as->nfi_cfg.page_size = rounddown(sec_size * sec_num, 1024);
	as->nfi_cfg.spare_size = 16;
	as->current_page_num = 0;
	as->data_need_update = true;

	err = airoha_spi_nfi_init(as);
	if (err)
		return err;

	return airoha_spi_nfi_config(as);
}

static const struct of_device_id airoha_spi_ids[] = {
    { .compatible = "airoha,en7581-spi" },
    { }
};
MODULE_DEVICE_TABLE(of, airoha_spi_ids);


static bool is_nor(void) { 	
	void __iomem *reg = ioremap((REG_SPI_CTRL_BASE + REG_SPI_CTRL_SFC_STRAP),4);
	u32 val = readl(reg); 
	iounmap(reg);
	if (val & 0x2) { 
		return false; 
	}
	return true; 
}

static const struct regmap_config spi_ctrl_regmap_config = {
	.name		= "ctrl",
	.reg_bits	= 32,
	.val_bits	= 32,
	.reg_stride	= 4,
	.max_register	= REG_SPI_CTRL_NFI2SPI_EN,
};

static const struct regmap_config spi_nfi_regmap_config = {
	.name		= "nfi",
	.reg_bits	= 32,
	.val_bits	= 32,
	.reg_stride	= 4,
	.max_register	= REG_SPI_NFI_SNF_NFI_CNFG,
};

static int airoha_spi_probe(struct platform_device *pdev)
{
    struct resource *spi_res = NULL;
    struct resource *nfi_res = NULL;
    struct spi_controller *master;
    struct airoha_snand *as;
    int err;
    if (is_nor()){ // TODO: EMMC check 
	    return -1;
    }
    printk("NAND device found\n");

    /* linux kernel platform */
    master = spi_alloc_master(&pdev->dev, sizeof(*as));
    if(!master)
    {
        return -ENOMEM;
    }
    platform_set_drvdata(pdev, master);

    /* spi platform */
    as = spi_master_get_devdata(master);
    as->master = master;
    as->dev = &pdev->dev;
    // ECC part tbd
    spi_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if(!spi_res)
    {
        err = -ENODEV;
        goto err_put_controller;
    }
    as->spi_base = devm_ioremap_resource(&pdev->dev, spi_res);
    if(IS_ERR(as->spi_base))
    {
        err = PTR_ERR(as->spi_base);
        goto err_put_controller;
    }

	as->regmap_ctrl = devm_regmap_init_mmio(&pdev->dev, as->spi_base,
						&spi_ctrl_regmap_config);
	if (IS_ERR(as->regmap_ctrl)) {
		 dev_err(&pdev->dev, "failed to init spi ctrl regmap: %ld\n",
			 PTR_ERR(as->regmap_ctrl));
		 err = PTR_ERR(as->regmap_ctrl);
		 goto err_put_controller;
	}

    nfi_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
    if(!nfi_res)
    {
        err = -ENODEV;
        goto err_put_controller;
    }
    as->nfi_base = devm_ioremap_resource(&pdev->dev, nfi_res);
    if(IS_ERR(as->nfi_base))
    {
        err = PTR_ERR(as->nfi_base);
        goto err_put_controller;
    }

	as->regmap_nfi = devm_regmap_init_mmio(&pdev->dev, as->nfi_base,
						&spi_nfi_regmap_config);
	if (IS_ERR(as->regmap_nfi)) {
		 dev_err(&pdev->dev, "failed to init spi nfi regmap: %ld\n",
			 PTR_ERR(as->regmap_nfi));
		 err = PTR_ERR(as->regmap_nfi);
		 goto err_put_controller;
	}

    as->irq = platform_get_irq(pdev, 0);
    if(as->irq < 0)
    {
        err = as->irq;
        goto err_put_controller;
    }
    err = dma_set_mask(as->dev, DMA_BIT_MASK(32));
    if(err)
    {
        goto err_put_controller;
    }

    /* Hook function pointer and settings to kernel */
    master->num_chipselect = 2;
    master->mem_ops = &airoha_spi_mem_ops;
    master->bits_per_word_mask = SPI_BPW_MASK(8);
    master->mode_bits = SPI_RX_DUAL;
    master->dev.of_node = pdev->dev.of_node;
    master->setup = airoha_spi_setup;
    master->cleanup = airoha_spi_cleanup;

	err = airoha_spi_nfi_setup(as);
	if (err)
		return err;

    err = spi_register_master(master);

    if(err)
    {
        goto err_put_controller;
    }


	printk("trying to init new device\n"); 

	printk("spi_device initailized\n"); 

    return 0;

err_put_controller:
    spi_master_put(master);
    return err;
}

static struct platform_driver airoha_spi_driver = {
	.driver = {
		.name = "airoha-spi",
		.of_match_table = airoha_spi_ids,
	},
	.probe = airoha_spi_probe,
};

module_platform_driver(airoha_spi_driver);

MODULE_DESCRIPTION("Airoha SPI driver");
MODULE_AUTHOR("Lorenzo Bianconi <lorenzo@kernel.org>");
MODULE_AUTHOR("Ray Liu <ray.liu@airoha.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:airoha-snand");
