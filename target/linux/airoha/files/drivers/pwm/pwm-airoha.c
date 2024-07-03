// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2022 Markus Gothe <markus.gothe@genexis.eu>
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/gpio.h>
#include <linux/bitops.h>
#include <asm/div64.h>

#define REG_SGPIO_LED_DATE		0x0000
#define SGPIO_LED_SHIFT_FLAG		BIT(31)
#define SGPIO_LED_DATA			GENMASK(16, 0)

#define REG_SGPIO_CLK_DIVR		0x0004
#define REG_SGPIO_CLK_DLY		0x0008

#define REG_SIPO_FLASH_MODE_CFG		0x000c
#define SERIAL_GPIO_FLASH_MODE		BIT(1)
#define SERIAL_GPIO_MODE		BIT(0)

#define REG_GPIO_FLASH_PRD_SET(_n)	(0x0018 + ((_n) << 2))
#define GPIO_FLASH_PRD_MASK(_n)		GENMASK(15 + ((_n) << 4), ((_n) << 4))

#define REG_GPIO_FLASH_MAP(_n)		(0x0028 + ((_n) << 2))
#define GPIO_FLASH_SETID_MASK(_n)	GENMASK(2 + ((_n) << 2), ((_n) << 2))
#define GPIO_FLASH_EN(_n)		BIT(3 << ((_n) << 2))

#define REG_SIPO_FLASH_MAP(_n)		(0x0030 + ((_n) << 2))

#define REG_CYCLE_CFG_VALUE(_n)		(0x0000 + ((_n) << 2))
#define WAVE_GEN_CYCLE_MASK(_n)		GENMASK(7 + ((_n) << 3), ((_n) << 3))

struct airoha_pwm {
	struct pwm_chip chip;

	void __iomem *base;
	void __iomem *cycle_cfg;

	u64 initialized;

	struct {
		u8 clk_divr_val;
		u8 clk_dly_val;
		bool hc74595;
	} sipo;

	struct {
		/* Bitmask of PWM channels using this bucket */
		u64 used;
		/* Relative duty cycle, 0-255 */
		u32 duty;
		/* In 1/250 s, 1-250 permitted */
		u32 period;
	} bucket[8];
};

/*
 * The first 16 GPIO pins, GPIO0-GPIO15, are mapped into 16 PWM channels, 0-15.
 * The SIPO GPIO pins are 16 pins which are mapped into 17 PWM channels, 16-32.
 * However, we've only got 8 concurrent waveform generators and can therefore
 * only use up to 8 different combinations of duty cycle and period at a time.
 */
#define PWM_NUM_GPIO	16
#define PWM_NUM_SIPO	17

/* The PWM hardware supports periods between 4 ms and 1 s */
#define PERIOD_MIN_NS	4000000
#define PERIOD_MAX_NS	1000000000
/* It is represented internally as 1/250 s between 1 and 250 */
#define PERIOD_MIN	1
#define PERIOD_MAX	250
/* Duty cycle is relative with 255 corresponding to 100% */
#define DUTY_FULL	255

static u32 __airoha_pwm_rmw(struct airoha_pwm *pc, void __iomem *addr,
			    u32 mask, u32 val)
{
	val |= (readl(addr) & ~mask);
	writel(val, addr);

	return val;
}

#define airoha_pwm_rmw(pc, offset, mask, val)					\
	__airoha_pwm_rmw((pc), (pc)->base + (offset), (mask), (val))
#define airoha_pwm_cycle_rmw(pc, offset, mask, val)				\
	__airoha_pwm_rmw((pc), (pc)->cycle_cfg + (offset), (mask), (val))
#define airoha_pwm_set(pc, offset, val)						\
	airoha_pwm_rmw((pc), (offset), 0, (val))
#define airoha_pwm_clear(pc, offset, mask)					\
	airoha_pwm_rmw((pc), (offset), (mask), 0)

static inline int find_waveform_index(struct airoha_pwm *pc, u32 duty, u32 period)
{
	int i = 0;

	for (; i < sizeof(pc->bucket)/sizeof(pc->bucket[0]); i++) {
		if (pc->bucket[i].used == 0)
			continue;

		if (duty == pc->bucket[i].duty && period == pc->bucket[i].period)
			return i;

		/* Unlike duty cycle zero, which can be handled by
		 * disabling PWM, a generator is needed for full duty
		 * cycle but it can be reused regardless of period */
		if (duty == DUTY_FULL && pc->bucket[i].duty == DUTY_FULL)
			return i;
	}

	return -1;
}

static inline void free_waveform(struct airoha_pwm *pc, unsigned int hwpwm)
{
	int i = 0;

	for (; i < sizeof(pc->bucket)/sizeof(pc->bucket[0]); i++)
		pc->bucket[i].used &= ~BIT_ULL(hwpwm);
}

static inline int find_unused_waveform(struct airoha_pwm *pc, unsigned int hwpwm)
{
	int i = 0;

	for (; i < sizeof(pc->bucket)/sizeof(pc->bucket[0]); i++)
		if ((pc->bucket[i].used & ~BIT_ULL(hwpwm)) == 0)
			return i;

	return -1;
}

static inline int use_waveform(struct airoha_pwm *pc, u32 duty, u32 period, unsigned int hwpwm)
{
	int exists = find_waveform_index(pc, duty, period);

	if (exists == -1)
		exists = find_unused_waveform(pc, hwpwm);

	if (exists == -1)
		return -1;

	free_waveform(pc, hwpwm);

	pc->bucket[exists].period = period;
	pc->bucket[exists].duty = duty;
	pc->bucket[exists].used |= BIT_ULL(hwpwm);

	return exists;
}

static inline struct airoha_pwm *to_airoha_pwm(struct pwm_chip *chip)
{
	return container_of(chip, struct airoha_pwm, chip);
}

static int airoha_pwm_sipo_init(struct airoha_pwm *pc)
{
	u32 val;

	if (!(pc->initialized >> PWM_NUM_GPIO))
		return 0;

	/* Select the right shift register chip */
	if (pc->sipo.hc74595)
		airoha_pwm_set(pc, REG_SIPO_FLASH_MODE_CFG, SERIAL_GPIO_MODE);
	else
		airoha_pwm_clear(pc, REG_SIPO_FLASH_MODE_CFG,
				 SERIAL_GPIO_MODE);

	/* Configure shift register timings */
	writel(pc->sipo.clk_divr_val, pc->base + REG_SGPIO_CLK_DIVR);
	writel(pc->sipo.clk_dly_val, pc->base + REG_SGPIO_CLK_DLY);

	/* It it necessary to after muxing explicitly shift out all
	 * zeroes to initialize the shift register before enabling PWM
	 * mode because in PWM mode SIPO will not start shifting until
	 * it needs to output a non-zero value (bit 31 of led_data
	 * indicates shifting in progress and it must return to zero
	 * before led_data can be written or PWM mode can be set)
	 */
	if (readl_poll_timeout(pc->base + REG_SGPIO_LED_DATE, val,
			       !(val & SGPIO_LED_SHIFT_FLAG), 10,
			       200 * USEC_PER_MSEC))
		return -ETIMEDOUT;

	airoha_pwm_clear(pc, REG_SGPIO_LED_DATE, SGPIO_LED_DATA);
	if (readl_poll_timeout(pc->base + REG_SGPIO_LED_DATE, val,
			       !(val & SGPIO_LED_SHIFT_FLAG), 10,
			       200 * USEC_PER_MSEC))
		return -ETIMEDOUT;

	/* Set SIPO in PWM mode */
	airoha_pwm_set(pc, REG_SIPO_FLASH_MODE_CFG, SERIAL_GPIO_FLASH_MODE);

	return 0;
}

static void airoha_pwm_config_waveform(struct airoha_pwm *pc, int index,
				       u32 duty, u32 period)
{
	u32 mask, val;

	/* Configure frequency divisor */
	mask = WAVE_GEN_CYCLE_MASK(index % 4);
	val = (period << __bf_shf(mask)) & mask;
	airoha_pwm_cycle_rmw(pc, REG_CYCLE_CFG_VALUE(index / 4), mask, val);

	/* Configure duty cycle */
	duty = ((DUTY_FULL - duty) << 8) | duty;
	mask = GPIO_FLASH_PRD_MASK(index % 2);
	val = (duty << __bf_shf(mask)) & mask;
	airoha_pwm_rmw(pc, REG_GPIO_FLASH_PRD_SET(index / 2), mask, val);
}

static void airoha_pwm_config_flash_map(struct airoha_pwm *pc,
					unsigned int hwpwm, int index)
{
	u32 addr, mask, val;

	if (hwpwm < PWM_NUM_GPIO) {
		addr = REG_GPIO_FLASH_MAP(hwpwm / 8);
	} else {
		addr = REG_SIPO_FLASH_MAP(hwpwm / 8);
		hwpwm -= PWM_NUM_GPIO;
	}

	if (index < 0) {
		/* Change of waveform takes effect immediately but
		 * disabling has some delay so to prevent glitching
		 * only the enable bit is touched when disabling
		 */
		airoha_pwm_clear(pc, addr, GPIO_FLASH_EN(hwpwm % 8));
		return;
	}

	mask = GPIO_FLASH_SETID_MASK(hwpwm % 8);
	val = ((index & 7) << __bf_shf(mask)) & mask;
	airoha_pwm_rmw(pc, addr, mask, val);
	airoha_pwm_set(pc, addr, GPIO_FLASH_EN(hwpwm % 8));
}

static int airoha_pwm_config(struct airoha_pwm *pc, struct pwm_device *pwm,
			     u64 duty_ns, u64 period_ns,
			     enum pwm_polarity polarity)
{
	u32 duty, period;
	int index = -1;

	duty = clamp_val(div64_u64(DUTY_FULL * duty_ns, period_ns), 0,
			 DUTY_FULL);
	if (polarity == PWM_POLARITY_INVERSED)
		duty = DUTY_FULL - duty;

	period = clamp_val(div64_u64(25 * period_ns, 100000000), PERIOD_MIN,
			   PERIOD_MAX);
	if (duty) {
		index = use_waveform(pc, duty, period, pwm->hwpwm);
		if (index < 0) {
			dev_err(pc->chip.dev,
				"%s: no free waveform available: %u / %u\n",
				pwm->label, duty, period);
			return -EBUSY;
		}
	}

	if (!(pc->initialized & BIT_ULL(pwm->hwpwm)) &&
	    pwm->hwpwm >= PWM_NUM_GPIO)
		airoha_pwm_sipo_init(pc);

	if (index < 0) {
		airoha_pwm_config_flash_map(pc, pwm->hwpwm, -1);
		free_waveform(pc, pwm->hwpwm);
	} else {
		airoha_pwm_config_waveform(pc, index, duty, period);
		airoha_pwm_config_flash_map(pc, pwm->hwpwm, index);
	}

	pc->initialized |= BIT_ULL(pwm->hwpwm);

	return 0;
}

static void airoha_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct airoha_pwm *pc = to_airoha_pwm(chip);

	/* Disable PWM and release the waveform */
	airoha_pwm_config_flash_map(pc, pwm->hwpwm, -1);
	free_waveform(pc, pwm->hwpwm);

	pc->initialized &= ~BIT_ULL(pwm->hwpwm);
	if (!(pc->initialized >> PWM_NUM_GPIO))
		airoha_pwm_clear(pc, REG_SIPO_FLASH_MODE_CFG,
				 SERIAL_GPIO_FLASH_MODE);

	/* Clear the state to force re-initialization the next time
	 * this PWM channel is used since we cannot retain state in
	 * hardware due to limited number of waveform generators
	 */
	memset(&pwm->state, 0, sizeof(pwm->state));
}

static int airoha_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			    const struct pwm_state *state)
{
	u64 duty = state->enabled ? state->duty_cycle : 0;
	struct airoha_pwm *pc = to_airoha_pwm(chip);

	if (state->period < PERIOD_MIN_NS || state->period > PERIOD_MAX_NS)
		return -EINVAL;

	return airoha_pwm_config(pc, pwm, duty, state->period,
				 state->polarity);
}

static const struct pwm_ops airoha_pwm_ops = {
	.apply = airoha_pwm_apply,
	.free = airoha_pwm_free,
	.owner = THIS_MODULE,
};

static int airoha_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct airoha_pwm *pc = NULL;
	u32 sipo_clock_divisor = 32;
	u32 sipo_clock_delay = 1;

	pc = devm_kzalloc(dev, sizeof(*pc), GFP_KERNEL);
	if (!pc)
		return -ENOMEM;

	pc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pc->base))
		return PTR_ERR(pc->base);

	pc->cycle_cfg = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(pc->cycle_cfg))
		return PTR_ERR(pc->cycle_cfg);

	of_property_read_u32(np, "sipo-clock-divisor", &sipo_clock_divisor);
	of_property_read_u32(np, "sipo-clock-delay", &sipo_clock_delay);

	switch (sipo_clock_divisor) {
	case 4:
		pc->sipo.clk_divr_val = 0;
		break;
	case 8:
		pc->sipo.clk_divr_val = 1;
		break;
	case 16:
		pc->sipo.clk_divr_val = 2;
		break;
	case 32:
		pc->sipo.clk_divr_val = 3;
		break;
	default:
		return -EINVAL;
	}

	if (sipo_clock_delay < 1 || sipo_clock_delay > sipo_clock_divisor/2)
		return -EINVAL;

	/* The actual delay is sclkdly + 1 so subtract 1 from
	 * sipo-clock-delay to calculate the register value */
	pc->sipo.clk_dly_val = sipo_clock_delay - 1;
	pc->sipo.hc74595 = of_property_read_bool(np, "hc74595");

	pc->chip.dev = dev;
	pc->chip.ops = &airoha_pwm_ops;
	pc->chip.base = -1;
	pc->chip.npwm = PWM_NUM_GPIO + PWM_NUM_SIPO;
	pc->chip.of_xlate = of_pwm_xlate_with_flags;
	pc->chip.of_pwm_n_cells = 3;

	platform_set_drvdata(pdev, pc);

	return pwmchip_add(&pc->chip);
}

static int airoha_pwm_remove(struct platform_device *pdev)
{
	struct airoha_pwm *pc = platform_get_drvdata(pdev);

	pwmchip_remove(&pc->chip);

	return 0;
}

static const struct of_device_id airoha_pwm_of_match[] = {
	{ .compatible = "airoha,en7581-pwm" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, airoha_pwm_of_match);

static struct platform_driver airoha_pwm_driver = {
	.driver = {
		.name = "airoha-pwm",
		.of_match_table = airoha_pwm_of_match,
	},
	.probe = airoha_pwm_probe,
	.remove = airoha_pwm_remove,
};
module_platform_driver(airoha_pwm_driver);

MODULE_ALIAS("platform:airoha-pwm");
MODULE_AUTHOR("Markus Gothe <markus.gothe@genexis.eu>");
MODULE_DESCRIPTION("Airoha EN7581 PWM driver");
MODULE_LICENSE("GPL v2");
