// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2022 Markus Gothe <markus.gothe@genexis.eu>
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/gpio.h>
#include <linux/bitops.h>
#include <asm/div64.h>

struct duty_period_bucket {
	u64 used;		/* Bitmask of PWM channels using this bucket */
	u32 duty;		/* Relative duty cycle, 0-255 */
	u32 period;		/* In 1/250 s, 1-250 permitted */
};

struct airoha_sipo {
	void __iomem *flash_mode_cfg;
	void __iomem *flash_map_cfg[3];
	void __iomem *led_data;
	void __iomem *clk_divr;
	void __iomem *clk_dly;
	u8 clk_divr_val;
	u8 clk_dly_val;
	bool hc74595;
};

struct airoha_pwm {
	struct pwm_chip chip;
	struct mutex mutex;
	struct duty_period_bucket bucket[8];
	u64 initialized;
	void __iomem *flash_prd_set[4];
	void __iomem *flash_map_cfg[2];
	void __iomem *cycle_cfg_value[2];
	struct airoha_sipo sipo;
};

/*
	The first 16 GPIO pins, GPIO0-GPIO15, are mapped into 16 PWM channels, 0-15.
	The SIPO GPIO pins are 16 pins which are mapped into 17 PWM channels, 16-32.

	However, we've only got 8 concurrent waveform generators and can therefore
	only use up to 8 different combinations of duty cycle and period at a time.
 */

#define PWM_NUM_GPIO 16
#define PWM_NUM_SIPO 17

/* Duty cycle is relative with 255 corresponding to 100% */
#define DUTY_FULL 255

/* The PWM hardware supports periods between 4 ms and 1 s */
#define PERIOD_MIN_NS    4000000
#define PERIOD_MAX_NS 1000000000

/* It is represented internally as 1/250 s between 1 and 250 */
#define PERIOD_MIN   1
#define PERIOD_MAX 250

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

static inline u32 wait_clear_and_read(void __iomem *reg, u32 mask)
{
	u32 value = 0;

	do {
		value = readl(reg);
	} while (value & mask);

	return value;
}

static void airoha_sipo_init(struct airoha_pwm *pc)
{
	u32 value = 0;

	/* Select the right shift register chip */
	value = readl(pc->sipo.flash_mode_cfg);
	if (pc->sipo.hc74595)
		value |= BIT(0);
	else
		value &= ~BIT(0);
	writel(value, pc->sipo.flash_mode_cfg);

	/* Configure shift register timings */
	writel(pc->sipo.clk_divr_val, pc->sipo.clk_divr);
	writel(pc->sipo.clk_dly_val, pc->sipo.clk_dly);

	/* It it necessary to after muxing explicitly shift out all
	 * zeroes to initialize the shift register before enabling PWM
	 * mode because in PWM mode SIPO will not start shifting until
	 * it needs to output a non-zero value (bit 31 of led_data
	 * indicates shifting in progress and it must return to zero
	 * before led_data can be written or PWM mode can be set) */
	value = wait_clear_and_read(pc->sipo.led_data, BIT(31));
	value &= ~0x1ffff;
	writel(value, pc->sipo.led_data);
	wait_clear_and_read(pc->sipo.led_data, BIT(31));

	/* Set SIPO in PWM mode */
	value = readl(pc->sipo.flash_mode_cfg);
	value |= BIT(1);
	writel(value, pc->sipo.flash_mode_cfg);
}

static void airoha_sipo_deinit(struct airoha_pwm *pc)
{
	u32 value = 0;

	/* Set SIPO in LED display mode */
	value = readl(pc->sipo.flash_mode_cfg);
	value &= ~BIT(1);
	writel(value, pc->sipo.flash_mode_cfg);
}

static void airoha_pwm_config_waveform(struct airoha_pwm *pc, int index, u32 duty, u32 period)
{
	u32 value = 0;
	u32 bit_shift = 0;
	void __iomem *reg = NULL;

	/* Configure frequency divisor */
	switch (index) {
	case 0 ... 3:
		reg = pc->cycle_cfg_value[0];
		break;
	case 4 ... 7:
		reg = pc->cycle_cfg_value[1];
		break;
	}
	bit_shift = (8 * (index % 4));
	value = readl(reg);
	value &= (~(0xff << bit_shift));
	value |= ((period & 0xff) << bit_shift);
	writel(value, reg);

	/* Configure duty cycle */
	duty = ((DUTY_FULL - duty) << 8) | duty;
	switch (index) {
	case 0 ... 1:
		reg = pc->flash_prd_set[0];
		break;
	case 2 ... 3:
		reg = pc->flash_prd_set[1];
		break;
	case 4 ... 5:
		reg = pc->flash_prd_set[2];
		break;
	case 6 ... 7:
		reg = pc->flash_prd_set[3];
		break;
	}
	bit_shift = (16 * (index % 2));
	value = readl(reg);
	value &= (~(0xffff << bit_shift));
	value |= ((duty & 0xffff) << bit_shift);
	writel(value, reg);
}

static void airoha_pwm_config_flash_map(struct airoha_pwm *pc, unsigned int hwpwm, int index)
{
	u32 value = 0;
	u32 bit_shift = 0;
	void __iomem *reg = NULL;

	if (hwpwm < PWM_NUM_GPIO) {
		switch (hwpwm) {
		case 0 ... 7:
			reg = pc->flash_map_cfg[0];
			break;
		case 8 ... 15:
			reg = pc->flash_map_cfg[1];
			break;
		}
	} else {
		hwpwm -= PWM_NUM_GPIO;
		switch (hwpwm) {
		case 0 ... 7:
			reg = pc->sipo.flash_map_cfg[0];
			break;
		case 8 ... 15:
			reg = pc->sipo.flash_map_cfg[1];
			break;
		case 16:
			reg = pc->sipo.flash_map_cfg[2];
			break;
		default:
			return;
		}
	}
	bit_shift = (4 * (hwpwm % 8));
	value = readl(reg);
	if (index == -1) {
		/* Change of waveform takes effect immediately but
		 * disabling has some delay so to prevent glitching
		 * only the enable bit is touched when disabling */
		value &= (~(8 << bit_shift));
	} else {
		value &= (~(7 << bit_shift));
		value |= ((index & 7) << bit_shift);
		value |= (8 << bit_shift);
	}
	writel(value, reg);
}

static int airoha_pwm_config(struct airoha_pwm *pc, struct pwm_device *pwm,
			     u64 duty_ns, u64 period_ns, enum pwm_polarity polarity)
{
	u32 duty = clamp_val(div64_u64(DUTY_FULL * duty_ns, period_ns), 0, DUTY_FULL);
	u32 period = clamp_val(div64_u64(25 * period_ns, 100000000), PERIOD_MIN, PERIOD_MAX);
	int index = -1;

	if (polarity == PWM_POLARITY_INVERSED)
		duty = DUTY_FULL - duty;

	if (duty > 0 && (index = use_waveform(pc, duty, period, pwm->hwpwm)) == -1) {
		dev_err(pc->chip.dev,
			"%s: Failed to configure PWM, no free waveform available: %u / %u\n",
			pwm->label, duty, period);
		return -EBUSY;
	}

	if ((pc->initialized & BIT_ULL(pwm->hwpwm)) == 0) {
		if (pwm->hwpwm < PWM_NUM_GPIO) {
		} else if ((pc->initialized >> PWM_NUM_GPIO) == 0)
			airoha_sipo_init(pc);
	}

	if (index == -1) {
		airoha_pwm_config_flash_map(pc, pwm->hwpwm, -1);
		free_waveform(pc, pwm->hwpwm);
	} else {
		airoha_pwm_config_waveform(pc, index, duty, period);
		airoha_pwm_config_flash_map(pc, pwm->hwpwm, index);
	}

	if (pc->initialized & BIT_ULL(pwm->hwpwm))
		return 0;

	pc->initialized |= BIT_ULL(pwm->hwpwm);

	return 0;
}

static void airoha_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct airoha_pwm *pc = to_airoha_pwm(chip);

	mutex_lock(&pc->mutex);

	pc->initialized &= ~BIT_ULL(pwm->hwpwm);

	/* Disable PWM and release the waveform */
	airoha_pwm_config_flash_map(pc, pwm->hwpwm, -1);
	free_waveform(pc, pwm->hwpwm);

	if ((pc->initialized >> PWM_NUM_GPIO) == 0)
		airoha_sipo_deinit(pc);

	mutex_unlock(&pc->mutex);

	/* Clear the state to force re-initialization the next time
	 * this PWM channel is used since we cannot retain state in
	 * hardware due to limited number of waveform generators */
	memset(&pwm->state, 0, sizeof(pwm->state));
}

static int airoha_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			    const struct pwm_state *state)
{
	struct airoha_pwm *pc = to_airoha_pwm(chip);
	u64 duty = state->enabled ? state->duty_cycle : 0;
	int ret = 0;

	if (state->period < PERIOD_MIN_NS || state->period > PERIOD_MAX_NS)
		return -EINVAL;

	mutex_lock(&pc->mutex);
	ret = airoha_pwm_config(pc, pwm, duty, state->period, state->polarity);
	mutex_unlock(&pc->mutex);

	return ret;
}

static const struct pwm_ops airoha_pwm_ops = {
	.free = airoha_pwm_free,
	.apply = airoha_pwm_apply,
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

	/* The PWM controller is part of the GPIO block so we need to
	 * address the registers one-by-one */

	pc->flash_prd_set[0] = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pc->flash_prd_set[0]))
		return PTR_ERR(pc->flash_prd_set[0]);

	pc->flash_prd_set[1] = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(pc->flash_prd_set[1]))
		return PTR_ERR(pc->flash_prd_set[1]);

	pc->flash_prd_set[2] = devm_platform_ioremap_resource(pdev, 2);
	if (IS_ERR(pc->flash_prd_set[2]))
		return PTR_ERR(pc->flash_prd_set[2]);

	pc->flash_prd_set[3] = devm_platform_ioremap_resource(pdev, 3);
	if (IS_ERR(pc->flash_prd_set[3]))
		return PTR_ERR(pc->flash_prd_set[3]);

	pc->flash_map_cfg[0] = devm_platform_ioremap_resource(pdev, 4);
	if (IS_ERR(pc->flash_map_cfg[0]))
		return PTR_ERR(pc->flash_map_cfg[0]);

	pc->flash_map_cfg[1] = devm_platform_ioremap_resource(pdev, 5);
	if (IS_ERR(pc->flash_map_cfg[1]))
		return PTR_ERR(pc->flash_map_cfg[1]);

	pc->cycle_cfg_value[0] = devm_platform_ioremap_resource(pdev, 6);
	if (IS_ERR(pc->cycle_cfg_value[0]))
		return PTR_ERR(pc->cycle_cfg_value[0]);

	pc->cycle_cfg_value[1] = devm_platform_ioremap_resource(pdev, 7);
	if (IS_ERR(pc->cycle_cfg_value[1]))
		return PTR_ERR(pc->cycle_cfg_value[1]);

	pc->sipo.flash_mode_cfg = devm_platform_ioremap_resource(pdev, 8);
	if (IS_ERR(pc->sipo.flash_mode_cfg))
		return PTR_ERR(pc->sipo.flash_mode_cfg);

	pc->sipo.flash_map_cfg[0] = devm_platform_ioremap_resource(pdev, 9);
	if (IS_ERR(pc->sipo.flash_map_cfg[0]))
		return PTR_ERR(pc->sipo.flash_map_cfg[0]);

	pc->sipo.flash_map_cfg[1] = devm_platform_ioremap_resource(pdev, 10);
	if (IS_ERR(pc->sipo.flash_map_cfg[1]))
		return PTR_ERR(pc->sipo.flash_map_cfg[1]);

	pc->sipo.flash_map_cfg[2] = devm_platform_ioremap_resource(pdev, 11);
	if (IS_ERR(pc->sipo.flash_map_cfg[2]))
		return PTR_ERR(pc->sipo.flash_map_cfg[2]);

	pc->sipo.led_data = devm_platform_ioremap_resource(pdev, 12);
	if (IS_ERR(pc->sipo.led_data))
		return PTR_ERR(pc->sipo.led_data);

	pc->sipo.clk_divr = devm_platform_ioremap_resource(pdev, 13);
	if (IS_ERR(pc->sipo.clk_divr))
		return PTR_ERR(pc->sipo.clk_divr);

	pc->sipo.clk_dly = devm_platform_ioremap_resource(pdev, 14);
	if (IS_ERR(pc->sipo.clk_dly))
		return PTR_ERR(pc->sipo.clk_dly);

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

	mutex_init(&pc->mutex);

	pc->chip.dev = dev;
	pc->chip.ops = &airoha_pwm_ops;
	pc->chip.base = -1;
	pc->chip.npwm = PWM_NUM_GPIO + PWM_NUM_SIPO;
	pc->chip.of_xlate = of_pwm_xlate_with_flags;
	pc->chip.of_pwm_n_cells = 3;

	platform_set_drvdata(pdev, pc);

	dev_err(dev, "Airoha PWM driver loaded");

	return pwmchip_add(&pc->chip);
}

static int airoha_pwm_remove(struct platform_device *pdev)
{
	struct airoha_pwm *pc = platform_get_drvdata(pdev);

	pwmchip_remove(&pc->chip);

	return 0;
}

static const struct of_device_id airoha_pwm_of_match[] = {
	{ .compatible = "airoha,airoha-pwm", },
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
