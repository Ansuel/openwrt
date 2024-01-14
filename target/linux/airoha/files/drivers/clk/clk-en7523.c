// SPDX-License-Identifier: GPL-2.0-only

#include <linux/delay.h>
#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <dt-bindings/clock/en7523-clk.h>

#define REG_PCI_CONTROL			0x88
#define   REG_PCI_CONTROL_PERSTOUT	BIT(29)
#define   REG_PCI_CONTROL_PERSTOUT1	BIT(26)
#define   REG_PCI_CONTROL_REFCLK_EN1	BIT(22)
#define REG_GSW_CLK_DIV_SEL		0x1b4
#define REG_EMI_CLK_DIV_SEL		0x1b8
#define REG_BUS_CLK_DIV_SEL		0x1bc
#define REG_SPI_CLK_DIV_SEL		0x1c4
#define REG_SPI_CLK_FREQ_SEL		0x1c8
#define REG_NPU_CLK_DIV_SEL		0x1fc
#define REG_CRYPTO_CLKSRC		0x200
#define REG_RESET_CONTROL		0x834
#define   REG_RESET_CONTROL_PCIEHB	BIT(29)
#define   REG_RESET_CONTROL_PCIE1	BIT(27)
#define   REG_RESET_CONTROL_PCIE2	BIT(26)

#define CR_NP_SCU_BASE                  (0x00000000)
#define CR_NP_SCU_PCIC	       	(CR_NP_SCU_BASE + 0x88)
#define CR_NP_SCU_SSTR			(CR_NP_SCU_BASE + 0x9C)

#define PBUS_MONITOR_BASE					(0x000000)  /*phys:0x1fbe3400*/

#define CR_CHIP_SCU_BASE				(0x00000000)
#define CR_CHIP_SCU_RGS_OPEN_DRAIN	(CR_CHIP_SCU_BASE + 0x018C)
#define OPEN_DRAIN_MASK     0x7 //bit[2:0] mapping PCIe port2/1/0

/*===========for PCIe begin============================================ */
#define PBUS_PCIE0_MEM_BASE				(PBUS_MONITOR_BASE+0x00)
#define PBUS_PCIE0_MEM_MASK				(PBUS_MONITOR_BASE+0x04)
#define PBUS_PCIE1_MEM_BASE				(PBUS_MONITOR_BASE+0x08)
#define PBUS_PCIE1_MEM_MASK				(PBUS_MONITOR_BASE+0x0C)
#define PBUS_PCIE2_MEM_BASE				(PBUS_MONITOR_BASE+0x10)
#define PBUS_PCIE2_MEM_MASK				(PBUS_MONITOR_BASE+0x14)
/*===========for PCIe end============================================ */

struct en_clk_desc {
	int id;
	const char *name;
	u32 base_reg;
	u8 base_bits;
	u8 base_shift;
	union {
		const unsigned int *base_values;
		unsigned int base_value;
	};
	size_t n_base_values;

	u16 div_reg;
	u8 div_bits;
	u8 div_shift;
	u16 div_val0;
	u8 div_step;
};

struct en_clk_gate {
	void __iomem *base;
	struct clk_hw hw;
};

static const u32 gsw_base[] = { 400000000, 500000000 };
static const u32 emi_base[] = { 333000000, 400000000 };
static const u32 bus_base[] = { 500000000, 540000000 };
static const u32 slic_base[] = { 100000000, 3125000 };
static const u32 npu_base[] = { 333000000, 400000000, 500000000 };

static const struct en_clk_desc en7523_base_clks[] = {
	{
		.id = EN7523_CLK_GSW,
		.name = "gsw",

		.base_reg = REG_GSW_CLK_DIV_SEL,
		.base_bits = 1,
		.base_shift = 8,
		.base_values = gsw_base,
		.n_base_values = ARRAY_SIZE(gsw_base),

		.div_bits = 3,
		.div_shift = 0,
		.div_step = 1,
	}, {
		.id = EN7523_CLK_EMI,
		.name = "emi",

		.base_reg = REG_EMI_CLK_DIV_SEL,
		.base_bits = 1,
		.base_shift = 8,
		.base_values = emi_base,
		.n_base_values = ARRAY_SIZE(emi_base),

		.div_bits = 3,
		.div_shift = 0,
		.div_step = 1,
	}, {
		.id = EN7523_CLK_BUS,
		.name = "bus",

		.base_reg = REG_BUS_CLK_DIV_SEL,
		.base_bits = 1,
		.base_shift = 8,
		.base_values = bus_base,
		.n_base_values = ARRAY_SIZE(bus_base),

		.div_bits = 3,
		.div_shift = 0,
		.div_step = 1,
	}, {
		.id = EN7523_CLK_SLIC,
		.name = "slic",

		.base_reg = REG_SPI_CLK_FREQ_SEL,
		.base_bits = 1,
		.base_shift = 0,
		.base_values = slic_base,
		.n_base_values = ARRAY_SIZE(slic_base),

		.div_reg = REG_SPI_CLK_DIV_SEL,
		.div_bits = 5,
		.div_shift = 24,
		.div_val0 = 20,
		.div_step = 2,
	}, {
		.id = EN7523_CLK_SPI,
		.name = "spi",

		.base_reg = REG_SPI_CLK_DIV_SEL,

		.base_value = 400000000,

		.div_bits = 5,
		.div_shift = 8,
		.div_val0 = 40,
		.div_step = 2,
	}, {
		.id = EN7523_CLK_NPU,
		.name = "npu",

		.base_reg = REG_NPU_CLK_DIV_SEL,
		.base_bits = 2,
		.base_shift = 8,
		.base_values = npu_base,
		.n_base_values = ARRAY_SIZE(npu_base),

		.div_bits = 3,
		.div_shift = 0,
		.div_step = 1,
	}, {
		.id = EN7523_CLK_CRYPTO,
		.name = "crypto",

		.base_reg = REG_CRYPTO_CLKSRC,
		.base_bits = 1,
		.base_shift = 8,
		.base_values = emi_base,
		.n_base_values = ARRAY_SIZE(emi_base),
	}
};

static const struct of_device_id of_match_clk_en7523[] = {
	{ .compatible = "airoha,en7523-scu", },
	{ /* sentinel */ }
};

static unsigned int en7523_get_base_rate(void __iomem *base, unsigned int i)
{
	const struct en_clk_desc *desc = &en7523_base_clks[i];
	u32 val;

	if (!desc->base_bits)
		return desc->base_value;

	val = readl(base + desc->base_reg);
	val >>= desc->base_shift;
	val &= (1 << desc->base_bits) - 1;

	if (val >= desc->n_base_values)
		return 0;

	return desc->base_values[val];
}

static u32 en7523_get_div(void __iomem *base, int i)
{
	const struct en_clk_desc *desc = &en7523_base_clks[i];
	u32 reg, val;

	if (!desc->div_bits)
		return 1;

	reg = desc->div_reg ? desc->div_reg : desc->base_reg;
	val = readl(base + reg);
	val >>= desc->div_shift;
	val &= (1 << desc->div_bits) - 1;

	if (!val && desc->div_val0)
		return desc->div_val0;

	return (val + 1) * desc->div_step;
}

static int en7523_pci_is_enabled(struct clk_hw *hw)
{
	struct en_clk_gate *cg = container_of(hw, struct en_clk_gate, hw);

	return !!(readl(cg->base + REG_PCI_CONTROL) & REG_PCI_CONTROL_REFCLK_EN1);
}

void __iomem *chipScu_base=NULL;
void __iomem *npScu_base=NULL;
void __iomem *pbScu_base=NULL;


u32 get_np_scu_data(u32 reg)
{
	return readl(npScu_base + reg);
}
EXPORT_SYMBOL(get_np_scu_data);

void set_np_scu_data(u32 reg, u32 val)
{
	writel(val, npScu_base + reg); 
}
EXPORT_SYMBOL(set_np_scu_data);

/*=====================for PCIe reset begin=========================*/
//#define CR_NP_SCU_PCIC	       	(CR_NP_SCU_BASE + 0x88)
u32 GET_PCIC(void)
{    
	return get_np_scu_data(CR_NP_SCU_PCIC);
}
EXPORT_SYMBOL(GET_PCIC);

void SET_PCIC(u32 val)
{    
	set_np_scu_data(CR_NP_SCU_PCIC, val);    
	return;
}
EXPORT_SYMBOL(SET_PCIC);

u32 GET_NP_SCU_SSTR(void)
{	
	return get_np_scu_data(CR_NP_SCU_SSTR);
}
EXPORT_SYMBOL(GET_NP_SCU_SSTR);

void SET_NP_SCU_SSTR(u32 val)
{    
	set_np_scu_data(CR_NP_SCU_SSTR, val);    
	return;
}
EXPORT_SYMBOL(SET_NP_SCU_SSTR);

/*=====================for PCIe reset end============================*/


/*===========for PCIe begin============================================ */
static u32 get_pbus_monitor_data(u32 reg)
{
    return readl(pbScu_base + reg);
}
static void set_pbus_monitor_data(u32 reg, u32 val)
{
    writel(val, pbScu_base + reg); 
}

/*======================PCIe0========================================== */
u32 GET_PBUS_PCIE0_BASE(void)
{
	return get_pbus_monitor_data(PBUS_PCIE0_MEM_BASE);
}

void SET_PBUS_PCIE0_BASE(u32 val)
{
	set_pbus_monitor_data(PBUS_PCIE0_MEM_BASE, val);
}

u32 GET_PBUS_PCIE0_MASK(void)
{
	return get_pbus_monitor_data(PBUS_PCIE0_MEM_MASK);
}

void SET_PBUS_PCIE0_MASK(u32 val)
{
	set_pbus_monitor_data(PBUS_PCIE0_MEM_MASK, val);
}

/*======================PCIe1========================================== */
u32 GET_PBUS_PCIE1_BASE(void)
{
	return get_pbus_monitor_data(PBUS_PCIE1_MEM_BASE);
}

void SET_PBUS_PCIE1_BASE(u32 val)
{
	set_pbus_monitor_data(PBUS_PCIE1_MEM_BASE, val);
}

u32 GET_PBUS_PCIE1_MASK(void)
{
	return get_pbus_monitor_data(PBUS_PCIE1_MEM_MASK);
}

void SET_PBUS_PCIE1_MASK(u32 val)
{
	set_pbus_monitor_data(PBUS_PCIE1_MEM_MASK, val);
}

/*======================PCIe2========================================== */
u32 GET_PBUS_PCIE2_BASE(void)
{
	return get_pbus_monitor_data(PBUS_PCIE2_MEM_BASE);
}

void SET_PBUS_PCIE2_BASE(u32 val)
{
	set_pbus_monitor_data(PBUS_PCIE2_MEM_BASE, val);
}

u32 GET_PBUS_PCIE2_MASK(void)
{
	return get_pbus_monitor_data(PBUS_PCIE2_MEM_MASK);
}

void SET_PBUS_PCIE2_MASK(u32 val)
{
	set_pbus_monitor_data(PBUS_PCIE2_MEM_MASK, val);
}
EXPORT_SYMBOL(GET_PBUS_PCIE0_BASE);
EXPORT_SYMBOL(SET_PBUS_PCIE0_BASE);
EXPORT_SYMBOL(GET_PBUS_PCIE0_MASK);
EXPORT_SYMBOL(SET_PBUS_PCIE0_MASK);
EXPORT_SYMBOL(GET_PBUS_PCIE1_BASE);
EXPORT_SYMBOL(SET_PBUS_PCIE1_BASE);
EXPORT_SYMBOL(GET_PBUS_PCIE1_MASK);
EXPORT_SYMBOL(SET_PBUS_PCIE1_MASK);
EXPORT_SYMBOL(GET_PBUS_PCIE2_BASE);
EXPORT_SYMBOL(SET_PBUS_PCIE2_BASE);
EXPORT_SYMBOL(GET_PBUS_PCIE2_MASK);
EXPORT_SYMBOL(SET_PBUS_PCIE2_MASK);


/* don't EXPORT this function. Create API for your purpose instead. */
u32 get_chip_scu_data(u32 reg)
{	
	return readl(chipScu_base + reg);
}
/* don't EXPORT this function. Create API for your purpose instead. */
void set_chip_scu_data(u32 reg, u32 val)
{	
	writel(val, chipScu_base + reg); 
}

void set_chipScuReg_bits(u32 reg, u32 mask, u32 bits)
{    
	u32 val = get_chip_scu_data(reg);    
	val &= (~mask);    
	val |= bits;    
	set_chip_scu_data(reg, val);
}

u32 GET_SCU_RGS_OPEN_DRAIN(void)
{	
	return ((get_chip_scu_data(CR_CHIP_SCU_RGS_OPEN_DRAIN)) & OPEN_DRAIN_MASK);
}
EXPORT_SYMBOL(GET_SCU_RGS_OPEN_DRAIN);

void SET_SCU_RGS_OPEN_DRAIN(u32 val)
{    
	set_chipScuReg_bits(CR_CHIP_SCU_RGS_OPEN_DRAIN, OPEN_DRAIN_MASK, val & OPEN_DRAIN_MASK);
}
EXPORT_SYMBOL(SET_SCU_RGS_OPEN_DRAIN);

/*===========for PCIe end============================================ */






static int en7523_pci_prepare(struct clk_hw *hw)
{
	struct en_clk_gate *cg = container_of(hw, struct en_clk_gate, hw);
	void __iomem *np_base = cg->base;
	u32 val, mask;

	/* Need to pull device low before reset */
	val = readl(np_base + REG_PCI_CONTROL);
	val &= ~(REG_PCI_CONTROL_PERSTOUT1 | REG_PCI_CONTROL_PERSTOUT);
	writel(val, np_base + REG_PCI_CONTROL);
	usleep_range(1000, 2000);

	/* Enable PCIe port 1 */
	val |= REG_PCI_CONTROL_REFCLK_EN1;
	writel(val, np_base + REG_PCI_CONTROL);
	usleep_range(1000, 2000);

	/* Reset to default */
	val = readl(np_base + REG_RESET_CONTROL);
	mask = REG_RESET_CONTROL_PCIE1 | REG_RESET_CONTROL_PCIE2 |
	       REG_RESET_CONTROL_PCIEHB;
	writel(val & ~mask, np_base + REG_RESET_CONTROL);
	usleep_range(1000, 2000);
	writel(val | mask, np_base + REG_RESET_CONTROL);
	msleep(100);
	writel(val & ~mask, np_base + REG_RESET_CONTROL);
	usleep_range(5000, 10000);

	/* Release device */
	mask = REG_PCI_CONTROL_PERSTOUT1 | REG_PCI_CONTROL_PERSTOUT;
	val = readl(np_base + REG_PCI_CONTROL);
	writel(val & ~mask, np_base + REG_PCI_CONTROL);
	usleep_range(1000, 2000);
	writel(val | mask, np_base + REG_PCI_CONTROL);
	msleep(250);

	return 0;
}

static void en7523_pci_unprepare(struct clk_hw *hw)
{
	struct en_clk_gate *cg = container_of(hw, struct en_clk_gate, hw);
	void __iomem *np_base = cg->base;
	u32 val;

	val = readl(np_base + REG_PCI_CONTROL);
	val &= ~REG_PCI_CONTROL_REFCLK_EN1;
	writel(val, np_base + REG_PCI_CONTROL);
}

static struct clk_hw *en7523_register_pcie_clk(struct device *dev,
					       void __iomem *np_base)
{
	static const struct clk_ops pcie_gate_ops = {
		.is_enabled = en7523_pci_is_enabled,
		.prepare = en7523_pci_prepare,
		.unprepare = en7523_pci_unprepare,
	};
	struct clk_init_data init = {
		.name = "pcie",
		.ops = &pcie_gate_ops,
	};
	struct en_clk_gate *cg;

	cg = devm_kzalloc(dev, sizeof(*cg), GFP_KERNEL);
	if (!cg)
		return NULL;

	cg->base = np_base;
	cg->hw.init = &init;
	en7523_pci_unprepare(&cg->hw);

	if (clk_hw_register(dev, &cg->hw))
		return NULL;

	return &cg->hw;
}

static void en7523_register_clocks(struct device *dev, struct clk_hw_onecell_data *clk_data,
				   void __iomem *base, void __iomem *np_base)
{
	struct clk_hw *hw;
	u32 rate;
	int i;

	for (i = 0; i < ARRAY_SIZE(en7523_base_clks); i++) {
		const struct en_clk_desc *desc = &en7523_base_clks[i];

		rate = en7523_get_base_rate(base, i);
		rate /= en7523_get_div(base, i);

		hw = clk_hw_register_fixed_rate(dev, desc->name, NULL, 0, rate);
		if (IS_ERR(hw)) {
			pr_err("Failed to register clk %s: %ld\n",
			       desc->name, PTR_ERR(hw));
			continue;
		}

		clk_data->hws[desc->id] = hw;
	}

	hw = en7523_register_pcie_clk(dev, np_base);
	clk_data->hws[EN7523_CLK_PCIE] = hw;

	clk_data->num = EN7523_NUM_CLOCKS;
}

static int en7523_clk_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct clk_hw_onecell_data *clk_data;
	void __iomem *base, *np_base, *pb_base;
	int r;

	printk("\n=========en7523_clk_probe====11==========\n");
	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);
	chipScu_base = base;
	printk("\n=========en7523_clk_probe====21=chipScu_base=%x========\n",chipScu_base);

	np_base = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(np_base))
		return PTR_ERR(np_base);
	npScu_base = np_base;
	printk("\n=========en7523_clk_probe====22=npScu_base=%x========\n",npScu_base);

	pb_base = devm_platform_ioremap_resource(pdev, 2);
	if (IS_ERR(pb_base))
		return PTR_ERR(pb_base);
	pbScu_base = pb_base;
	printk("\n=========en7523_clk_probe====23=pbScu_base=%x========\n",pbScu_base);

	clk_data = devm_kzalloc(&pdev->dev,
				struct_size(clk_data, hws, EN7523_NUM_CLOCKS),
				GFP_KERNEL);
	if (!clk_data)
		return -ENOMEM;

	en7523_register_clocks(&pdev->dev, clk_data, base, np_base);
	printk("\n=========en7523_clk_probe====33==========\n");

	r = of_clk_add_hw_provider(node, of_clk_hw_onecell_get, clk_data);
	if (r)
		dev_err(&pdev->dev,
			"could not register clock provider: %s: %d\n",
			pdev->name, r);

	return r;
}

static struct platform_driver clk_en7523_drv = {
	.probe = en7523_clk_probe,
	.driver = {
		.name = "clk-en7523",
		.of_match_table = of_match_clk_en7523,
		.suppress_bind_attrs = true,
	},
};

static int __init clk_en7523_init(void)
{
	return platform_driver_register(&clk_en7523_drv);
}

arch_initcall(clk_en7523_init);
