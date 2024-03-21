// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 AIROHA Inc.
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

#define _SPI_CONTROLLER_REGS_BASE                   0x1FA10000
#define _SPI_CONTROLLER_REGS_READ_MODE              0x0000
#define _SPI_CONTROLLER_REGS_READ_IDLE_EN           0x0004
#define _SPI_CONTROLLER_REGS_SIDLY                  0x0008
#define _SPI_CONTROLLER_REGS_CSHEXT                 0x000C
#define _SPI_CONTROLLER_REGS_CSLEXT                 0x0010
#define _SPI_CONTROLLER_REGS_MTX_MODE_TOG           0x0014
#define _SPI_CONTROLLER_REGS_RDCTL_FSM              0x0018
#define _SPI_CONTROLLER_REGS_MACMUX_SEL             0x001C
#define _SPI_CONTROLLER_REGS_MANUAL_EN              0x0020
#define _SPI_CONTROLLER_REGS_MANUAL_OPFIFO_EMPTY    0x0024
#define _SPI_CONTROLLER_REGS_MANUAL_OPFIFO_WDATA    0x0028
#define _SPI_CONTROLLER_REGS_MANUAL_OPFIFO_FULL     0x002C
#define _SPI_CONTROLLER_REGS_MANUAL_OPFIFO_WR       0x0030
#define _SPI_CONTROLLER_REGS_MANUAL_DFIFO_FULL      0x0034
#define _SPI_CONTROLLER_REGS_MANUAL_DFIFO_WDATA     0x0038
#define _SPI_CONTROLLER_REGS_MANUAL_DFIFO_EMPTY     0x003C
#define _SPI_CONTROLLER_REGS_MANUAL_DFIFO_RD        0x0040
#define _SPI_CONTROLLER_REGS_MANUAL_DFIFO_RDATA     0x0044
#define _SPI_CONTROLLER_REGS_DUMMY                  0x0080
#define _SPI_CONTROLLER_REGS_PROBE_SEL              0x0088
#define _SPI_CONTROLLER_REGS_INTERRUPT              0x0090
#define _SPI_CONTROLLER_REGS_INTERRUPT_EN           0x0094
#define _SPI_CONTROLLER_REGS_SI_CK_SEL              0x009C
#define _SPI_CONTROLLER_REGS_SW_CFGNANDADDR_VAL     0x010C
#define _SPI_CONTROLLER_REGS_SW_CFGNANDADDR_EN      0x0110
#define _SPI_CONTROLLER_REGS_SFC_STRAP              0x0114
#define _SPI_CONTROLLER_REGS_NFI2SPI_EN             0x0130
#define _SPI_NFI_REGS_BASE                          0x1FA11000
#define _SPI_NFI_REGS_CNFG                          0x0000
#define _SPI_NFI_REGS_PAGEFMT                       0x0004
#define _SPI_NFI_REGS_CON                           0x0008
#define _SPI_NFI_REGS_INTR_EN                       0x0010
#define _SPI_NFI_REGS_INTR                          0x0014
#define _SPI_NFI_REGS_CMD                           0x0020
#define _SPI_NFI_REGS_STA                           0x0060
#define _SPI_NFI_REGS_FIFOSTA                       0x0064
#define _SPI_NFI_REGS_STRADDR                       0x0080
#define _SPI_NFI_REGS_FDM0L                         0x00A0
#define _SPI_NFI_REGS_FDM0M                         0x00A4
#define _SPI_NFI_REGS_FDM7L                         0x00D8
#define _SPI_NFI_REGS_FDM7M                         0x00DC
#define _SPI_NFI_REGS_FIFODATA0                     0x0190
#define _SPI_NFI_REGS_FIFODATA1                     0x0194
#define _SPI_NFI_REGS_FIFODATA2                     0x0198
#define _SPI_NFI_REGS_FIFODATA3                     0x019C
#define _SPI_NFI_REGS_MASTERSTA                     0x0224
#define _SPI_NFI_REGS_SECCUS_SIZE                   0x022C
#define _SPI_NFI_REGS_RD_CTL2                       0x0510
#define _SPI_NFI_REGS_RD_CTL3                       0x0514
#define _SPI_NFI_REGS_PG_CTL1                       0x0524
#define _SPI_NFI_REGS_PG_CTL2                       0x0528
#define _SPI_NFI_REGS_NOR_PROG_ADDR                 0x052C
#define _SPI_NFI_REGS_NOR_RD_ADDR                   0x0534
#define _SPI_NFI_REGS_SNF_MISC_CTL                  0x0538
#define _SPI_NFI_REGS_SNF_MISC_CTL2                 0x053C
#define _SPI_NFI_REGS_SNF_STA_CTL1                  0x0550
#define _SPI_NFI_REGS_SNF_STA_CTL2                  0x0554
#define _SPI_NFI_REGS_SNF_NFI_CNFG                  0x055C
/* SPI NAND Protocol OP */
#define _SPI_NAND_OP_GET_FEATURE                    0x0F    /* Get Feature */
#define _SPI_NAND_OP_SET_FEATURE                    0x1F    /* Set Feature */
#define _SPI_NAND_OP_PAGE_READ                      0x13    /* Load page data into cache of SPI NAND chip */
#define _SPI_NAND_OP_READ_FROM_CACHE_SINGLE         0x03    /* Read data from cache of SPI NAND chip, single speed*/
#define _SPI_NAND_OP_READ_FROM_CACHE_SINGLE_FAST    0x0B    /* Read data from cache of SPI NAND chip, single speed*/
#define _SPI_NAND_OP_READ_FROM_CACHE_DUAL           0x3B    /* Read data from cache of SPI NAND chip, dual speed*/
#define _SPI_NAND_OP_READ_FROM_CACHE_QUAD           0x6B    /* Read data from cache of SPI NAND chip, quad speed*/
#define _SPI_NAND_OP_WRITE_ENABLE                   0x06    /* Enable write data to  SPI NAND chip */
#define _SPI_NAND_OP_WRITE_DISABLE                  0x04    /* Reseting the Write Enable Latch (WEL) */
#define _SPI_NAND_OP_PROGRAM_LOAD_SINGLE            0x02    /* Write data into cache of SPI NAND chip with cache reset, single speed */
#define _SPI_NAND_OP_PROGRAM_LOAD_QUAD              0x32    /* Write data into cache of SPI NAND chip with cache reset, quad speed */
#define _SPI_NAND_OP_PROGRAM_LOAD_RAMDOM_SINGLE     0x84    /* Write data into cache of SPI NAND chip, single speed */
#define _SPI_NAND_OP_PROGRAM_LOAD_RAMDON_QUAD       0x34    /* Write data into cache of SPI NAND chip, quad speed */
#define _SPI_NAND_OP_PROGRAM_EXECUTE                0x10    /* Write data from cache into SPI NAND chip */
#define _SPI_NAND_OP_READ_ID                        0x9F    /* Read Manufacture ID and Device ID */
#define _SPI_NAND_OP_BLOCK_ERASE                    0xD8    /* Erase Block */
#define _SPI_NAND_OP_RESET                          0xFF    /* Reset */
#define _SPI_NAND_OP_DIE_SELECT                     0xC2    /* Die Select */

#define WRITE_SPI_REG(a,b) writel(b, as->spi_base + a)
#define READ_SPI_REG(a) readl(as->spi_base + a)
#define WRITE_NFI_REG(a,b) writel(b, as->nfi_base + a)
#define READ_NFI_REG(a) readl(as->nfi_base + a)
#define WRITE_NFI_REG_WITH_MASK(a,b,c) WRITE_NFI_REG(a, (READ_NFI_REG(a) & ~(b)) | (c))

#define _SPI_NAND_PAGE_SIZE                         (4096)
#define _SPI_NAND_OOB_SIZE                          (256)
#define _SPI_NAND_CACHE_SIZE                        (_SPI_NAND_PAGE_SIZE+_SPI_NAND_OOB_SIZE)

typedef enum{
    SPI_NFI_SPI=0,
    SPI_NFI_PARALLEL,
    SPI_NFI_EMMC,
} SPI_NFI_TYPE_T;

typedef enum{
    SPI_CONTROLLER_SPI_NAND=0,
    SPI_CONTROLLER_SPI_NOR,
    SPI_CONTROLLER_PARALLEL_NAND,
    SPI_CONTROLLER_EMMC,
} SPI_CONTROLLER_SPI_TYPE_T;

typedef enum {
    SPI_CONTROLLER_MODE_AUTO=0,
    SPI_CONTROLLER_MODE_MANUAL,
    SPI_CONTROLLER_MODE_DMA,
    SPI_CONTROLLER_MODE_NO
} SPI_CONTROLLER_MODE_T;

typedef enum{
    SPI_CONTROLLER_CHIP_SELECT_HIGH=0,
    SPI_CONTROLLER_CHIP_SELECT_LOW,
} SPI_CONTROLLER_CHIP_SELECT_T;

struct airoha_nfi_conf {
    size_t page_size;
    size_t oob_size;
    size_t sec_size;
    unsigned char sec_num;
    unsigned char spare_size;
    unsigned char dummy_byte_num;
};

struct airoha_snand {
    struct spi_controller *master;
    struct device *dev;
    void __iomem *spi_base;
    void __iomem *nfi_base;
    int irq;
    struct airoha_nfi_conf nfi_cfg;
    unsigned char *rx_buf;
    unsigned char *tx_buf;
    dma_addr_t rx_dma_addr;
    dma_addr_t tx_dma_addr;
    size_t buf_len;
    unsigned long current_page_num;
    bool data_need_update;
    bool autofmt; // <-- soc controller ecc need to implement this feature
};

/* old API support start */
static struct airoha_snand *old_as;
DEFINE_SEMAPHORE(SPI_SEM);//Make sure all related SPI operations are atomic
/* old API support end */

static int airoha_spi_set_opfifo(struct airoha_snand *as, unsigned char op_cmd, unsigned int op_len)
{
    WRITE_SPI_REG(_SPI_CONTROLLER_REGS_MANUAL_OPFIFO_WDATA, ((((op_cmd) & 0x1f) << 0x9) | ((op_len) & 0x1ff)));
    while(READ_SPI_REG(_SPI_CONTROLLER_REGS_MANUAL_OPFIFO_FULL));
    WRITE_SPI_REG(_SPI_CONTROLLER_REGS_MANUAL_OPFIFO_WR, 0x1);
    while(!READ_SPI_REG(_SPI_CONTROLLER_REGS_MANUAL_OPFIFO_EMPTY));
    return 0;
}

static void airoha_spi_set_cs(struct airoha_snand *as, SPI_CONTROLLER_CHIP_SELECT_T cs)
{
    airoha_spi_set_opfifo(as, cs, 1);
}

static int airoha_spi_write_data_fifo(struct airoha_snand *as, void *data, unsigned int data_len)
{
    unsigned char *ptr_data = data;
    unsigned int idx;
    for(idx = 0; idx < data_len; idx++)
    {
        /* 1. Wait until dfifo is not full */
        while(READ_SPI_REG(_SPI_CONTROLLER_REGS_MANUAL_DFIFO_FULL));
        /* 2. Write data to register DFIFO_WDATA */
        WRITE_SPI_REG(_SPI_CONTROLLER_REGS_MANUAL_DFIFO_WDATA, ((*(ptr_data+idx)) & 0xff));
        /* 3. Wait until dfifo is not full */
        while(READ_SPI_REG(_SPI_CONTROLLER_REGS_MANUAL_DFIFO_FULL));
    }
    return 0;
}

static int airoha_spi_read_data_fifo(struct airoha_snand *as, unsigned char *ptr_rtn_data, unsigned int data_len)
{
    unsigned int idx;
    for(idx = 0; idx < data_len; idx++)
    {
        /* 1. wait until dfifo is not empty */
        while(READ_SPI_REG(_SPI_CONTROLLER_REGS_MANUAL_DFIFO_EMPTY));
        /* 2. read from dfifo to register DFIFO_RDATA */
        *(ptr_rtn_data+idx) = (READ_SPI_REG(_SPI_CONTROLLER_REGS_MANUAL_DFIFO_RDATA)) & 0xff;
        /* 3. enable register DFIFO_RD to read next byte */
        WRITE_SPI_REG(_SPI_CONTROLLER_REGS_MANUAL_DFIFO_RD, 0x1);
    }
    return 0;
}

static int airoha_spi_set_mode(struct airoha_snand *as, SPI_CONTROLLER_MODE_T mode)
{
    switch(mode)
    {
        case SPI_CONTROLLER_MODE_AUTO:
        {
            break;
        }
        case SPI_CONTROLLER_MODE_MANUAL:
        {
            WRITE_SPI_REG(_SPI_CONTROLLER_REGS_NFI2SPI_EN, 0x0);
            WRITE_SPI_REG(_SPI_CONTROLLER_REGS_READ_IDLE_EN, 0x0);
            while(READ_SPI_REG(_SPI_CONTROLLER_REGS_RDCTL_FSM));
            WRITE_SPI_REG(_SPI_CONTROLLER_REGS_MTX_MODE_TOG, 0x9);
            WRITE_SPI_REG(_SPI_CONTROLLER_REGS_MANUAL_EN, 0x1);
            break;
        }
        case SPI_CONTROLLER_MODE_DMA:
        {
            WRITE_SPI_REG(_SPI_CONTROLLER_REGS_NFI2SPI_EN, 0x1);
            WRITE_SPI_REG(_SPI_CONTROLLER_REGS_MTX_MODE_TOG, 0x0);
            WRITE_SPI_REG(_SPI_CONTROLLER_REGS_MANUAL_EN, 0x0);
            break;
        }
        default:
        {
            break;
        }
    }
    WRITE_SPI_REG(_SPI_CONTROLLER_REGS_DUMMY, as->nfi_cfg.dummy_byte_num);
    return 0;
}

static int airoha_spi_write_one_byte_with_cmd(struct airoha_snand *as, unsigned char cmd, void *data)
{
    airoha_spi_set_opfifo(as, cmd, 1);
    airoha_spi_write_data_fifo(as, data, 1);
    return 0;
}

static int airoha_spi_write_nbytes_with_cmd(struct airoha_snand *as, unsigned char cmd, unsigned char *ptr_data, unsigned int len)
{
    unsigned int data_len, remain_len;
    remain_len = len;
    while (remain_len > 0)
    {
        data_len = (remain_len > 0x1ff) ? 0x1ff : remain_len;
        airoha_spi_set_opfifo(as, cmd, data_len);
        airoha_spi_write_data_fifo(as, ptr_data, data_len);
        remain_len -= data_len;
    }
    return 0;
}

static int airoha_spi_read_nbytes(struct airoha_snand *as, unsigned char *ptr_rtn_data, unsigned int len)
{
    unsigned int data_len, remain_len;
    remain_len = len;
    while (remain_len > 0)
    {
        data_len = (remain_len > 0x1ff) ? 0x1ff : remain_len;
        airoha_spi_set_opfifo(as, 0xc, data_len);
        airoha_spi_read_data_fifo(as, ptr_rtn_data, data_len);
        remain_len -= data_len;
    }
    return 0;
}

static int airoha_snand_nfi_init(struct airoha_snand *as)
{
    /* switch to SNFI mode */
    WRITE_NFI_REG(_SPI_NFI_REGS_SNF_NFI_CNFG, 0x1);
    /* Enable DMA */
    WRITE_NFI_REG(_SPI_NFI_REGS_INTR_EN, READ_NFI_REG(_SPI_NFI_REGS_INTR_EN) | 0x0040);
    return 0;
}

static int airoha_snand_nfi_reset(struct airoha_snand *as)
{
    // reset nfi
    WRITE_NFI_REG(_SPI_NFI_REGS_CON, 0x3);
    return 0;
}

static int airoha_snand_nfi_config(struct airoha_snand *as)
{
    /* AutoFDM */
    WRITE_NFI_REG(_SPI_NFI_REGS_CNFG, READ_NFI_REG(_SPI_NFI_REGS_CNFG) & ~(0x0200));
    /* HW ECC */
    WRITE_NFI_REG(_SPI_NFI_REGS_CNFG, READ_NFI_REG(_SPI_NFI_REGS_CNFG) & ~(0x0100));
    /* DMA Burst */
    WRITE_NFI_REG(_SPI_NFI_REGS_CNFG, READ_NFI_REG(_SPI_NFI_REGS_CNFG) | (0x0004));
    /* page format */
    switch(as->nfi_cfg.spare_size)
    {
        case 16:
        {
            WRITE_NFI_REG_WITH_MASK(_SPI_NFI_REGS_PAGEFMT, 0x0030, (0x0 << 4));
            break;
        }
        case 26:
        {
            WRITE_NFI_REG_WITH_MASK(_SPI_NFI_REGS_PAGEFMT, 0x0030, (0x1 << 4));
            break;
        }
        case 27:
        {
            WRITE_NFI_REG_WITH_MASK(_SPI_NFI_REGS_PAGEFMT, 0x0030, (0x2 << 4));
            break;
        }
        case 28:
        {
            WRITE_NFI_REG_WITH_MASK(_SPI_NFI_REGS_PAGEFMT, 0x0030, (0x3 << 4));
            break;
        }
        default:
        {
            WRITE_NFI_REG_WITH_MASK(_SPI_NFI_REGS_PAGEFMT, 0x0030, (0x0 << 4));
            break;
        }
    }
    switch(as->nfi_cfg.page_size)
    {
        case 512:
        {
            WRITE_NFI_REG_WITH_MASK(_SPI_NFI_REGS_PAGEFMT, 0x0003, (0x0 << 0));
            break;
        }
        case 2048:
        {
            WRITE_NFI_REG_WITH_MASK(_SPI_NFI_REGS_PAGEFMT, 0x0003, (0x1 << 0));
            break;
        }
        case 4096:
        {
            WRITE_NFI_REG_WITH_MASK(_SPI_NFI_REGS_PAGEFMT, 0x0003, (0x2 << 0));
            break;
        }
        default:
        {
            break;
        }
    }
    /* sec num */
    WRITE_NFI_REG_WITH_MASK(_SPI_NFI_REGS_CON, 0xf000, as->nfi_cfg.sec_num << 12);
    /* enable cust sec size */
    WRITE_NFI_REG(_SPI_NFI_REGS_SECCUS_SIZE, READ_NFI_REG(_SPI_NFI_REGS_SECCUS_SIZE) | (0x00010000));
    /* set cust sec size */
    WRITE_NFI_REG_WITH_MASK(_SPI_NFI_REGS_SECCUS_SIZE, 0x00001fff, (as->nfi_cfg.sec_size << 0));
    return 0;
}

static bool airoha_snand_is_page_ops(const struct spi_mem_op *op)
{
    if (op->addr.nbytes != 2)
        return false;

    if (op->addr.buswidth != 1 && op->addr.buswidth != 2 &&
        op->addr.buswidth != 4)
        return false;

    // match read from page instructions
    if (op->data.dir == SPI_MEM_DATA_IN) {
        // check dummy cycle first
        if (op->dummy.nbytes * BITS_PER_BYTE / op->dummy.buswidth >
            0xf)
            return false;
        // quad io / quad out
        if ((op->addr.buswidth == 4 || op->addr.buswidth == 1) &&
            op->data.buswidth == 4)
            return true;

        // dual io / dual out
        if ((op->addr.buswidth == 2 || op->addr.buswidth == 1) &&
            op->data.buswidth == 2)
            return true;

        // standard spi
        if (op->addr.buswidth == 1 && op->data.buswidth == 1)
            return true;
    } else if (op->data.dir == SPI_MEM_DATA_OUT) {
        // check dummy cycle first
        if (op->dummy.nbytes)
            return false;
        // program load quad out
        if (op->addr.buswidth == 1 && op->data.buswidth == 4)
            return true;
        // standard spi
        if (op->addr.buswidth == 1 && op->data.buswidth == 1)
            return true;
    }
    return false;
}

static int airoha_snand_adjust_op_size(struct spi_mem *mem, struct spi_mem_op *op)
{
    struct airoha_snand *as = spi_master_get_devdata(mem->spi->master);
    if(airoha_snand_is_page_ops(op))
    {
        size_t l;
        if(as->autofmt)
            return 0;
        l = as->nfi_cfg.sec_size + as->nfi_cfg.spare_size;
        l *= as->nfi_cfg.sec_num;
        if(op->data.nbytes > l)
            op->data.nbytes = l;
    }
    else
    {
        size_t hl = 1 + op->addr.nbytes + op->dummy.nbytes;
        if(hl >= 0xa0)
            return -EOPNOTSUPP;
        if(op->data.nbytes > 0xa0 - hl)
            op->data.nbytes = 0xa0 - hl;
    }
    return 0;
}

static bool airoha_snand_supports_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
    if (!spi_mem_default_supports_op(mem, op))
        return false;
    if (op->cmd.buswidth != 1)
        return false;
    if (airoha_snand_is_page_ops(op))
        return true;
    return ((op->addr.nbytes == 0 || op->addr.buswidth == 1) &&
        (op->dummy.nbytes == 0 || op->dummy.buswidth == 1) &&
        (op->data.nbytes == 0 || op->data.buswidth == 1));
}

static int airoha_snand_dirmap_create(struct spi_mem_dirmap_desc *desc)
{
    struct airoha_snand *as = spi_master_get_devdata(desc->mem->spi->master);
    if(!as->rx_buf || !as->tx_buf)
        return -EINVAL;
    if(desc->info.offset + desc->info.length > U32_MAX)
        return -EINVAL;
    if(!airoha_snand_supports_op(desc->mem, &desc->info.op_tmpl))
        return -EOPNOTSUPP;
    return 0;
}

static ssize_t airoha_snand_dirmap_read(struct spi_mem_dirmap_desc *desc, u64 offs, size_t len, void *buf)
{
    struct airoha_snand *as = spi_master_get_devdata(desc->mem->spi->master);
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
    // as->nfi_cfg.dummy_byte_num = op->dummy.nbytes;

    airoha_spi_set_mode(as, SPI_CONTROLLER_MODE_DMA);
    airoha_snand_nfi_reset(as);
    airoha_snand_nfi_config(as);
    dma_sync_single_for_device(as->dev, as->rx_dma_addr, as->buf_len, DMA_FROM_DEVICE);
    mb();
    // set dma addr
    WRITE_NFI_REG(_SPI_NFI_REGS_STRADDR, as->rx_dma_addr);
    // set cust sec size
    WRITE_NFI_REG_WITH_MASK(_SPI_NFI_REGS_SNF_MISC_CTL2, 0x1fff, (as->nfi_cfg.sec_size * as->nfi_cfg.sec_num) << 0);
    // set read command
    WRITE_NFI_REG(_SPI_NFI_REGS_RD_CTL2, op->cmd.opcode);
    // set read mode
    WRITE_NFI_REG(_SPI_NFI_REGS_SNF_MISC_CTL, rd_mode << 16);
    // set read addr
    WRITE_NFI_REG(_SPI_NFI_REGS_RD_CTL3, 0x0);
    // set nfi read
    WRITE_NFI_REG_WITH_MASK(_SPI_NFI_REGS_CNFG, 0x7000, (6 << 12));
    WRITE_NFI_REG(_SPI_NFI_REGS_CNFG, READ_NFI_REG(_SPI_NFI_REGS_CNFG) | 0x0002);
    WRITE_NFI_REG(_SPI_NFI_REGS_CNFG, READ_NFI_REG(_SPI_NFI_REGS_CNFG) | 0x0001);
    WRITE_NFI_REG(_SPI_NFI_REGS_CMD, 0x00);
    // trigger dma start read
    WRITE_NFI_REG(_SPI_NFI_REGS_CON, READ_NFI_REG(_SPI_NFI_REGS_CON) & ~(0x0100));
    WRITE_NFI_REG(_SPI_NFI_REGS_CON, READ_NFI_REG(_SPI_NFI_REGS_CON) | (0x0100));

    ret = readl_poll_timeout(as->nfi_base + _SPI_NFI_REGS_SNF_STA_CTL1, val, (val & 0x02000000) != 0, 0, 1000000);
    if(ret)
    {
        printk("[Error] Read DMA : Check READ FROM CACHE Done Timeout ! \n");
        return -1;
    }
    WRITE_NFI_REG(_SPI_NFI_REGS_SNF_STA_CTL1, READ_NFI_REG(_SPI_NFI_REGS_SNF_STA_CTL1) | 0x02000000);
    ret = readl_poll_timeout(as->nfi_base + _SPI_NFI_REGS_INTR, val, (val & 0x0040) != 0, 0, 1000000);
    if(ret)
    {
        printk("[Error] Read DMA : Check AHB Done Timeout ! \n");
        return -1;
    }

    /* Does DMA read need delay for data ready from controller to DRAM */
    udelay(1);
    dma_sync_single_for_cpu(as->dev, as->rx_dma_addr, as->buf_len, DMA_FROM_DEVICE);
    airoha_spi_set_mode(as, SPI_CONTROLLER_MODE_MANUAL);
    memcpy(buf, as->rx_buf + offs, len);
    return len;
}

static ssize_t airoha_snand_dirmap_write(struct spi_mem_dirmap_desc *desc, u64 offs, size_t len, const void *buf)
{
    struct airoha_snand *as = spi_master_get_devdata(desc->mem->spi->master);
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

    airoha_spi_set_mode(as, SPI_CONTROLLER_MODE_MANUAL);
    dma_sync_single_for_cpu(as->dev, as->tx_dma_addr, as->buf_len, DMA_TO_DEVICE);
    memcpy(as->tx_buf, as->rx_buf, as->buf_len);
    memcpy(as->tx_buf + offs, buf, len);
    dma_sync_single_for_device(as->dev, as->tx_dma_addr, as->buf_len, DMA_TO_DEVICE);
    mb();
    airoha_spi_set_mode(as, SPI_CONTROLLER_MODE_DMA);
    airoha_snand_nfi_reset(as);
    airoha_snand_nfi_config(as);
    WRITE_NFI_REG(_SPI_NFI_REGS_STRADDR, as->tx_dma_addr);
    WRITE_NFI_REG_WITH_MASK(_SPI_NFI_REGS_SNF_MISC_CTL2, 0x1fff0000, ((as->nfi_cfg.sec_size * as->nfi_cfg.sec_num) << 16));
    WRITE_NFI_REG(_SPI_NFI_REGS_PG_CTL1, (op->cmd.opcode << 8));
    WRITE_NFI_REG(_SPI_NFI_REGS_SNF_MISC_CTL, (wr_mode << 16));
    WRITE_NFI_REG(_SPI_NFI_REGS_PG_CTL2, 0x0);
    WRITE_NFI_REG(_SPI_NFI_REGS_CNFG, (READ_NFI_REG(_SPI_NFI_REGS_CNFG) & ~(0x0002)));
    WRITE_NFI_REG_WITH_MASK(_SPI_NFI_REGS_CNFG, 0x7000, (3 << 12));
    WRITE_NFI_REG(_SPI_NFI_REGS_CNFG, (READ_NFI_REG(_SPI_NFI_REGS_CNFG) | (0x0001)));
    WRITE_NFI_REG(_SPI_NFI_REGS_CMD, 0x80);
    WRITE_NFI_REG(_SPI_NFI_REGS_CON, (READ_NFI_REG(_SPI_NFI_REGS_CON) & ~(0x0200)));
    WRITE_NFI_REG(_SPI_NFI_REGS_CON, (READ_NFI_REG(_SPI_NFI_REGS_CON) | (0x0200)));
    udelay(1);

    ret = readl_poll_timeout(as->nfi_base + _SPI_NFI_REGS_INTR, val, (val & 0x0040) != 0, 0, 1000000);
    if(ret)
    {
        printk("[Error] Write DMA : Check LOAD TO CACHE Done Timeout ! \n");
        return -1;
    }

    ret = readl_poll_timeout(as->nfi_base + _SPI_NFI_REGS_SNF_STA_CTL1, val, (val & 0x04000000) != 0, 0, 1000000);
    if(ret)
    {
        printk("[Error] Read DMA : Check AHB Done Timeout ! \n");
        return -1;
    }
    WRITE_NFI_REG(_SPI_NFI_REGS_SNF_STA_CTL1, READ_NFI_REG(_SPI_NFI_REGS_SNF_STA_CTL1) | 0x04000000);
    airoha_spi_set_mode(as, SPI_CONTROLLER_MODE_MANUAL);
    return len;
}

static int airoha_snand_op_transfer(struct airoha_snand *as, const struct spi_mem_op *op)
{
    unsigned int idx;

    if(op->cmd.opcode == _SPI_NAND_OP_PROGRAM_EXECUTE)
    {
        if(op->addr.val == as->current_page_num)
            as->data_need_update = true;
    }
    else if(op->cmd.opcode == _SPI_NAND_OP_PAGE_READ)
    {
        if(!as->data_need_update && op->addr.val == as->current_page_num)
            return 0;
        as->data_need_update = true;
        as->current_page_num = op->addr.val;
    }

    /* Switch To Manual Mode */
    airoha_spi_set_mode(as, SPI_CONTROLLER_MODE_MANUAL);

    airoha_spi_set_cs(as, SPI_CONTROLLER_CHIP_SELECT_LOW);

    // opcode part
    airoha_spi_write_one_byte_with_cmd(as, 0x8, &op->cmd.opcode);

    // addr part
    for(idx = 0; idx < op->addr.nbytes; idx++)
    {
        unsigned char addr_data = (op->addr.val >> ((op->addr.nbytes - idx - 1) * 8)) & 0xff;
        if(op->cmd.opcode == _SPI_NAND_OP_GET_FEATURE)
        {
            airoha_spi_write_one_byte_with_cmd(as, 0x11, &addr_data);
        }
        else
        {
            airoha_spi_write_one_byte_with_cmd(as, 0x8, &addr_data);
        }
    }

    // dummy part
    for(idx = 0; idx < op->dummy.nbytes; idx++)
    {
        unsigned char data = 0xff;
        airoha_spi_write_one_byte_with_cmd(as, 0x8, &data);
    }

    // data part
    if(op->data.dir == SPI_MEM_DATA_IN)
    {
        airoha_spi_read_nbytes(as, op->data.buf.in, op->data.nbytes);
    }
    else
    {
        airoha_spi_write_nbytes_with_cmd(as, 0x8, op->data.buf.out, op->data.nbytes);
    }

    airoha_spi_set_cs(as, SPI_CONTROLLER_CHIP_SELECT_HIGH);
    return 0;
}

static int airoha_snand_exec_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
    struct airoha_snand *as = spi_master_get_devdata(mem->spi->master);
    int ret;

    ret = airoha_snand_op_transfer(as, op); // op cmd

    return ret;
}

static const struct spi_controller_mem_ops airoha_snand_mem_ops = {
    .adjust_op_size = airoha_snand_adjust_op_size,
    .supports_op = airoha_snand_supports_op,
    .exec_op = airoha_snand_exec_op,
    .dirmap_create = airoha_snand_dirmap_create,
    .dirmap_read = airoha_snand_dirmap_read,
    .dirmap_write = airoha_snand_dirmap_write,
};

static struct spi_board_info airoha_board_info =  { 
	.modalias = "spidev", 
	.max_speed_hz = 50000000,
	.bus_num = 0 , 
	.chip_select = 1, 
	.mode = SPI_MODE_0,
};

static struct spi_device *airoha_spi_device; 
static int airoha_snand_setup(struct spi_device *spi)
{
    struct airoha_snand *as = spi_master_get_devdata(spi->master);
    unsigned int sec_num = ((READ_NFI_REG(_SPI_NFI_REGS_CON) & 0xf000) >> 12);
    unsigned int sec_size = (READ_NFI_REG(_SPI_NFI_REGS_SECCUS_SIZE) & 0x1fff);
    int ret = 0;

	
    /* init value */
    as->autofmt = false;
    as->nfi_cfg.sec_num = sec_num;
    as->nfi_cfg.sec_size = sec_size;
    as->nfi_cfg.page_size = (((sec_size * sec_num) / 1024) * 1024);
    as->nfi_cfg.oob_size = ((sec_size * sec_num) % 1024);
    as->nfi_cfg.spare_size = 16;
    as->nfi_cfg.dummy_byte_num = 0;
    as->current_page_num = 0;
    as->data_need_update = true;

    /* prepare buffer */
    as->buf_len = _SPI_NAND_CACHE_SIZE;
    as->rx_buf = (unsigned char *)kmalloc(as->buf_len, GFP_KERNEL);
    if(!as->rx_buf)
    {
        printk("[airoha_snand_setup] rx_buf allocate failed\n");
        return -ENOMEM;
    }
    as->rx_dma_addr = dma_map_single(as->dev, (void *)as->rx_buf, as->buf_len, DMA_FROM_DEVICE);
    ret = dma_mapping_error(as->dev, as->rx_dma_addr);
    if(ret)
    {
        printk("[airoha_snand_setup] rx_dma_addr mapping error\n");
        return -EINVAL;
    }
    as->tx_buf = (unsigned char *)kmalloc(as->buf_len, GFP_KERNEL);
    if(!as->tx_buf)
    {
        printk("[airoha_snand_setup] tx_buf allocate failed\n");
        return -ENOMEM;
    }
    as->tx_dma_addr = dma_map_single(as->dev, (void *)as->tx_buf, as->buf_len, DMA_TO_DEVICE);
    ret = dma_mapping_error(as->dev, as->tx_dma_addr);
    if(ret)
    {
        printk("[airoha_snand_setup] tx_dma_addr mapping error\n");
        return -EINVAL;
    }

    airoha_snand_nfi_init(as);
    airoha_snand_nfi_config(as);

    return 0;
}

static const struct of_device_id airoha_snand_ids[] = {
    { .compatible = "airoha,en7581-spi" },
    { }
};
MODULE_DEVICE_TABLE(of, airoha_snand_ids);


static bool is_nor(void) { 	
	void __iomem *reg = ioremap((_SPI_CONTROLLER_REGS_BASE + _SPI_CONTROLLER_REGS_SFC_STRAP),4);
	u32 val = readl(reg); 
	iounmap(reg);
	if (val & 0x2) { 
		return false; 
	}
	return true; 
}

static int airoha_snand_probe(struct platform_device *pdev)
{
    struct resource *spi_res = NULL;
    struct resource *nfi_res = NULL;
    struct spi_controller *master;
    struct airoha_snand *as;
    int ret;
    if (is_nor()){ // TODO: EMMC check 
	    return -1;
    }
    printk("[%s-%d]: NAND device found\n", __func__, __LINE__);

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
        ret = -ENODEV;
        goto err_put_controller;
    }
    as->spi_base = devm_ioremap_resource(&pdev->dev, spi_res);
    if(IS_ERR(as->spi_base))
    {
        ret = PTR_ERR(as->spi_base);
        goto err_put_controller;
    }
    nfi_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
    if(!nfi_res)
    {
        ret = -ENODEV;
        goto err_put_controller;
    }
    as->nfi_base = devm_ioremap_resource(&pdev->dev, nfi_res);
    if(IS_ERR(as->nfi_base))
    {
        ret = PTR_ERR(as->nfi_base);
        goto err_put_controller;
    }
    as->irq = platform_get_irq(pdev, 0);
    if(as->irq < 0)
    {
        ret = as->irq;
        goto err_put_controller;
    }
    ret = dma_set_mask(as->dev, DMA_BIT_MASK(32));
    if(ret)
    {
        goto err_put_controller;
    }

    /* Hook function pointer and settings to kernel */
    master->num_chipselect = 2;
    master->mem_ops = &airoha_snand_mem_ops;
    master->bits_per_word_mask = SPI_BPW_MASK(8);
    master->mode_bits = SPI_RX_DUAL;
    master->dev.of_node = pdev->dev.of_node;
    master->setup = airoha_snand_setup;



    ret = spi_register_master(master);

    if(ret)
    {
        goto err_put_controller;
    }


	printk("trying to init new device\n"); 

	airoha_spi_device = spi_new_device(master , &airoha_board_info);
	if (!airoha_spi_device) { 
		printk("[airoha_snand_setup] spi new device failed\n");
	}
	printk("spi_device initailized\n"); 
    /* old API support start */
    old_as = as;
    /* old API support end */

    return 0;

err_put_controller:
    spi_master_put(master);
    return ret;
}

static int airoha_snand_remove(struct platform_device *pdev)
{
    return 0;
}
static struct platform_driver airoha_snand_driver = {
    .driver = {
        .name = "airoha-snand",
        .of_match_table = airoha_snand_ids,
    },
    .probe = airoha_snand_probe,
    .remove = airoha_snand_remove,
};

module_platform_driver(airoha_snand_driver);

MODULE_DESCRIPTION("AIROHA SPI NAND driver");
MODULE_AUTHOR("Ray Liu <ray.liu@airoha.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:airoha-snand");
