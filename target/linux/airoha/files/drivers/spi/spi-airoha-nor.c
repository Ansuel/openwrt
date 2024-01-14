// SPDX-License-Identifier: GPL-2.0

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mod_devicetable.h>
#include <linux/spi/spi.h>

#include <linux/kernel.h> 


#define _SPI_CONTROLLER_REGS_BASE                   0x1FA10000
#define _SPI_CONTROLLER_REGS_SFC_STRAP              0x0114

#define ENSPI_READ_IDLE_EN			0x0004
#define ENSPI_MTX_MODE_TOG			0x0014
#define ENSPI_RDCTL_FSM				0x0018
#define ENSPI_MANUAL_EN				0x0020
#define ENSPI_MANUAL_OPFIFO_EMPTY		0x0024
#define ENSPI_MANUAL_OPFIFO_WDATA		0x0028
#define ENSPI_MANUAL_OPFIFO_FULL		0x002C
#define ENSPI_MANUAL_OPFIFO_WR			0x0030
#define ENSPI_MANUAL_DFIFO_FULL			0x0034
#define ENSPI_MANUAL_DFIFO_WDATA		0x0038
#define ENSPI_MANUAL_DFIFO_EMPTY		0x003C
#define ENSPI_MANUAL_DFIFO_RD			0x0040
#define ENSPI_MANUAL_DFIFO_RDATA		0x0044
#define ENSPI_IER				0x0090
#define ENSPI_NFI2SPI_EN			0x0130

// TODO not in spi block
#define ENSPI_CLOCK_DIVIDER			((void __iomem *)0x1fa201c4)

#define	OP_CSH					0x00
#define	OP_CSL					0x01
#define	OP_CK					0x02
#define	OP_OUTS					0x08
#define	OP_OUTD					0x09
#define	OP_OUTQ					0x0A
#define	OP_INS					0x0C
#define	OP_INS0					0x0D
#define	OP_IND					0x0E
#define	OP_INQ					0x0F
#define	OP_OS2IS				0x10
#define	OP_OS2ID				0x11
#define	OP_OS2IQ				0x12
#define	OP_OD2IS				0x13
#define	OP_OD2ID				0x14
#define	OP_OD2IQ				0x15
#define	OP_OQ2IS				0x16
#define	OP_OQ2ID				0x17
#define	OP_OQ2IQ				0x18
#define	OP_OSNIS				0x19
#define	OP_ODNID				0x1A

#define MATRIX_MODE_AUTO		1
#define   CONF_MTX_MODE_AUTO		0
#define   MANUALEN_AUTO			0
#define MATRIX_MODE_MANUAL		0
#define   CONF_MTX_MODE_MANUAL		9
#define   MANUALEN_MANUAL		1

#define _ENSPI_MAX_XFER			0x1ff

#define REG(x)			(iobase + x)


static void __iomem *iobase;


static void opfifo_write(u32 cmd, u32 len)
{
	u32 tmp = ((cmd & 0x1f) << 9) | (len & 0x1ff);

	writel(tmp, REG(ENSPI_MANUAL_OPFIFO_WDATA));

	/* Wait for room in OPFIFO */
	while (readl(REG(ENSPI_MANUAL_OPFIFO_FULL)))
		;

	/* Shift command into OPFIFO */
	writel(1, REG(ENSPI_MANUAL_OPFIFO_WR));

	/* Wait for command to finish */
	while (!readl(REG(ENSPI_MANUAL_OPFIFO_EMPTY)))
		;
}

static void set_cs(int state)
{
	if (state)
		opfifo_write(OP_CSH, 1);
	else
		opfifo_write(OP_CSL, 1);
}

static void manual_begin_cmd(void)
{
	/* Disable read idle state */
	writel(0, REG(ENSPI_READ_IDLE_EN));

	/* Wait for FSM to reach idle state */
	while (readl(REG(ENSPI_RDCTL_FSM)))
		;

	/* Set SPI core to manual mode */
	writel(CONF_MTX_MODE_MANUAL, REG(ENSPI_MTX_MODE_TOG));
	writel(MANUALEN_MANUAL, REG(ENSPI_MANUAL_EN));
}

static void manual_end_cmd(void)
{
	/* Set SPI core to auto mode */
	writel(CONF_MTX_MODE_AUTO, REG(ENSPI_MTX_MODE_TOG));
	writel(MANUALEN_AUTO, REG(ENSPI_MANUAL_EN));

	/* Enable read idle state */
	writel(1, REG(ENSPI_READ_IDLE_EN));
}

static void dfifo_read(u8 *buf, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		/* Wait for requested data to show up in DFIFO */
		while (readl(REG(ENSPI_MANUAL_DFIFO_EMPTY)))
			;
		buf[i] = readl(REG(ENSPI_MANUAL_DFIFO_RDATA));
		/* Queue up next byte */
		writel(1, REG(ENSPI_MANUAL_DFIFO_RD));
		// while (readl(REG(ENSPI_MANUAL_DFIFO_EMPTY)))
	}
}

static void dfifo_write(const u8 *buf, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		/* Wait for room in DFIFO */
		while (readl(REG(ENSPI_MANUAL_DFIFO_FULL)))
			;
		writel(buf[i], REG(ENSPI_MANUAL_DFIFO_WDATA));
		while (readl(REG(ENSPI_MANUAL_DFIFO_FULL)))
			;
	}
}

static void set_spi_clock_speed(int freq_mhz)
{
	u32 tmp, val;

	tmp = readl(ENSPI_CLOCK_DIVIDER);
	tmp &= 0xffff0000;
	writel(tmp, ENSPI_CLOCK_DIVIDER);

	val = (400 / (freq_mhz * 2));
	tmp |= (val << 8) | 1;
	writel(tmp, ENSPI_CLOCK_DIVIDER);
}

static void init_hw(void)
{
	/* Disable manual/auto mode clash interrupt */
	writel(0, REG(ENSPI_IER));

	// TODO via clk framework
	// set_spi_clock_speed(50);

	/* Disable DMA */
	writel(0, REG(ENSPI_NFI2SPI_EN));
}

static int xfer_read(struct spi_transfer *xfer)
{
	int opcode;
	uint8_t *buf = xfer->rx_buf;

	switch (xfer->rx_nbits) {
		case SPI_NBITS_SINGLE:
			opcode = OP_INS;
			break;
		case SPI_NBITS_DUAL:
			opcode = OP_IND;
			break;
		case SPI_NBITS_QUAD:
			opcode = OP_INQ;
			break;
	}

	opfifo_write(opcode, xfer->len);
	dfifo_read(buf, xfer->len);

	return xfer->len;
}

static int xfer_write(struct spi_transfer *xfer, int next_xfer_is_rx)
{
	int opcode;
	const uint8_t *buf = xfer->tx_buf; 
	if (next_xfer_is_rx) { /* need to use Ox2Ix opcode to set the core to input afterwards */ 
		switch (xfer->tx_nbits) { 
			case SPI_NBITS_SINGLE:
				opcode = OP_OS2IS;
				break;
			case SPI_NBITS_DUAL:
				opcode = OP_OS2ID;
				break;
			case SPI_NBITS_QUAD:
				opcode = OP_OS2IQ;
				break;
		}
	} else {
		switch (xfer->tx_nbits) {
			case SPI_NBITS_SINGLE:
				opcode = OP_OUTS;
				break;
			case SPI_NBITS_DUAL:
				opcode = OP_OUTD;
				break;
			case SPI_NBITS_QUAD:
				opcode = OP_OUTQ;
				break;
		}
	}

	opfifo_write(opcode, xfer->len);
	dfifo_write(buf, xfer->len);

	return xfer->len;
}

size_t max_transfer_size(struct spi_device *spi)
{
	return _ENSPI_MAX_XFER;
}


int transfer_one_message(struct spi_controller *ctrl, struct spi_message *msg)
{
	struct spi_transfer *xfer;
	int next_xfer_is_rx = 0;

	manual_begin_cmd();
	set_cs(0);
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		if (xfer->tx_buf) {
			if (!list_is_last(&xfer->transfer_list, &msg->transfers)
					&& list_next_entry(xfer, transfer_list)->rx_buf != NULL)
				next_xfer_is_rx = 1;
			else
				next_xfer_is_rx = 0;
			msg->actual_length += xfer_write(xfer, next_xfer_is_rx);
		} else if (xfer->rx_buf) {
			msg->actual_length += xfer_read(xfer);
		}
	}
	set_cs(1);
	manual_end_cmd();

	msg->status = 0;
	spi_finalize_current_message(ctrl);

	return 0;
}

static bool is_nor(void) { 	
	void __iomem *reg = ioremap((_SPI_CONTROLLER_REGS_BASE + _SPI_CONTROLLER_REGS_SFC_STRAP),4);
	u32 val = readl(reg); 
	iounmap(reg);
	if (val & 0x2) { 
		return false; 
	}
	return true; 
}

static int spi_probe(struct platform_device *pdev)
{
	struct spi_controller *ctrl;
	int err;
	if (!is_nor()) {  // EMMC Check
		return -1;
	}
	printk("NOR device found\n");
	ctrl = spi_alloc_master(&pdev->dev, sizeof(*ctrl));
	if (!ctrl) {
		dev_err(&pdev->dev, "Error allocating SPI controller\n");
		return -ENOMEM;
	}

	struct resource *spi_res = NULL, *nfi_res;

	spi_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!spi_res) {
		printk("failed before\n");
		return -ENOMEM;
	}
	iobase = devm_ioremap_resource(&pdev->dev, spi_res);

	if (IS_ERR(iobase)) {
		dev_err(&pdev->dev, "Could not map SPI register address");
		return -ENOMEM;
	}

	int ret;
	nfi_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if(!nfi_res)
	{
		ret = -ENODEV;
		return ret;
	}
	void __iomem *nfi_base = devm_ioremap_resource(&pdev->dev, nfi_res);
	if(IS_ERR(nfi_base))
	{
		ret = PTR_ERR(nfi_base);
		return ret;
	}



	init_hw();

	ctrl->dev.of_node = pdev->dev.of_node;
	ctrl->flags = SPI_CONTROLLER_HALF_DUPLEX;
	ctrl->mode_bits = SPI_RX_DUAL | SPI_TX_DUAL;
	ctrl->max_transfer_size = max_transfer_size;
	ctrl->transfer_one_message = transfer_one_message;
	err = devm_spi_register_controller(&pdev->dev, ctrl);
	if (err) {
		dev_err(&pdev->dev, "Could not register SPI controller\n");
		return -ENODEV;
	}

	return 0;
}

static int spi_remove(struct platform_device *pdev) {
	printk("remove is called\n");
	iounmap(iobase);
	return 0;
}

static const struct of_device_id spi_of_ids[] = {
	{ .compatible = "airoha,airoha-snor" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, spi_of_ids);

static struct platform_driver spi_driver = {
	.probe = spi_probe,
	.driver = {
		.name = "airoha-spi-nor", 
		.of_match_table = spi_of_ids,
	},
	.remove = spi_remove,
};

module_platform_driver(spi_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Bert Vermeulen <bert@biot.com>");
MODULE_DESCRIPTION("Airoha EN7523 SPI driver");


