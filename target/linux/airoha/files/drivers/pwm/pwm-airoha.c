// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2022 Markus Gothe <markus.gothe@genexis.eu>
 *
 *  Limitations:
 *  - Only 8 concurrent waveform generators are available for 8 combinations of
 *    duty_cycle and period. Waveform generators are shared between 16 GPIO
 *    pins and 17 SIPO GPIO pins.
 *  - Supports only normal polarity.
 *  - On configuration the currently running period is completed.
 */

#include <linux/bitfield.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/gpio.h>
#include <linux/bitops.h>
#include <linux/regmap.h>
#include <asm/div64.h>

#define REG_SGPIO_LED_DATA		0x0024
#define SGPIO_LED_DATA_SHIFT_FLAG_MASK	BIT(31)
#define SGPIO_LED_DATA_DATA_MASK	GENMASK(16, 0)

#define REG_SGPIO_CLK_DIVR		0x0028
#define SGPIO_CLK_DIVR_MASK		GENMASK(1, 0)

#define REG_SGPIO_CLK_DLY		0x002c

#define REG_SIPO_FLASH_MODE_CFG		0x0030
#define SERIAL_GPIO_FLASH_MODE_MASK	BIT(1)
#define SERIAL_GPIO_MODE_74HC164_MASK	BIT(0)

#define REG_GPIO_FLASH_PRD_SET(_n)	(0x003c + (((_n) >> 1) << 2))
#define GPIO_FLASH_PRD_HIGH_MASK(_n)	\
	GENMASK(7 + (((_n) % 2) << 4),	((_n) % 2) << 4)
#define GPIO_FLASH_PRD_LOW_MASK(_n)	\
	GENMASK(15 + (((_n) % 2) << 4), 8 + (((_n) % 2) << 4))

#define REG_GPIO_FLASH_MAP(_n)		(0x004c + (((_n) >> 3) << 2))
#define GPIO_FLASH_EN_MASK(_n)		BIT(3 + (((_n) % 8) << 2))
#define GPIO_FLASH_SET_ID_MASK(_n)	\
	GENMASK(2 + (((_n) % 8) << 2), ((_n) % 8) << 2)

/* Register map is equal to GPIO flash map */
#define REG_SIPO_FLASH_MAP(_n)		(0x0054 + (((_n) >> 3) << 2))

#define REG_CYCLE_CFG_VALUE(_n)		(0x0098 + (((_n) >> 2) << 2))
#define WAVE_GEN_CYCLE_MASK(_n)		\
	GENMASK(7 + (((_n) % 4) << 3), ((_n) % 4) << 3)

#define AIROHA_PWM_NUM_BUCKETS		8
/*
 * The first 16 GPIO pins, GPIO0-GPIO15, are mapped into 16 PWM channels, 0-15.
 * The SIPO GPIO pins are 17 pins which are mapped into 17 PWM channels, 16-32.
 * However, we've only got 8 concurrent waveform generators and can therefore
 * only use up to 8 different combinations of duty cycle and period at a time.
 */
#define AIROHA_PWM_NUM_GPIO		16
#define AIROHA_PWM_NUM_SIPO		17
#define AIROHA_PWM_MAX_CHANNELS		(AIROHA_PWM_NUM_GPIO + AIROHA_PWM_NUM_SIPO)

#define AIROHA_PWM_FIELD_GET(mask, val)	(((val) & (mask)) >> __ffs(mask))
#define AIROHA_PWM_FIELD_SET(mask, val)	(((val) << __ffs(mask)) & (mask))

struct airoha_pwm_bucket {
	/* Bitmask of PWM channels using this bucket */
	u64 used;
	u64 period_ns;
	u64 duty_ns;
};

struct airoha_pwm {
	struct pwm_chip chip;

	struct regmap *regmap;

	u64 initialized;

	struct airoha_pwm_bucket buckets[AIROHA_PWM_NUM_BUCKETS];

	/* Cache bucket used by each pwm channel */
	u8 channel_bucket[AIROHA_PWM_MAX_CHANNELS];
};

/* The PWM hardware supports periods between 4 ms and 1 s */
#define AIROHA_PWM_PERIOD_MIN_NS	(4 * NSEC_PER_MSEC)
#define AIROHA_PWM_PERIOD_MAX_NS	(1 * NSEC_PER_SEC)
/* It is represented internally as 1/250 s between 1 and 250 */
#define AIROHA_PWM_PERIOD_MIN		1
#define AIROHA_PWM_PERIOD_MAX		250
/* Duty cycle is relative with 255 corresponding to 100% */
#define AIROHA_PWM_DUTY_FULL		255

static u32 airoha_pwm_get_duty_tick_from_ns(u64 duty_ns, u64 period_ns)
{
	u32 duty_tick;

	duty_tick = mul_u64_u64_div_u64(duty_ns, AIROHA_PWM_DUTY_FULL,
					period_ns);
	return min_t(u32, duty_tick, AIROHA_PWM_DUTY_FULL);
}

static int airoha_pwm_get_generator(struct airoha_pwm *pc, u64 duty_ns,
				    u64 period_ns)
{
	int i, unused = -1;

	for (i = 0; i < ARRAY_SIZE(pc->buckets); i++) {
		struct airoha_pwm_bucket *bucket = &pc->buckets[i];
		u32 duty_tick, duty_tick_bucket;

		/* If found, save an unused bucket to return it later */
		if (!bucket->used && unused == -1) {
			unused = i;
			continue;
		}

		if (duty_ns == bucket->duty_ns &&
		    period_ns == bucket->period_ns)
			return i;

		/*
		 * Unlike duty cycle zero, which can be handled by
		 * disabling PWM, a generator is needed for full duty
		 * cycle but it can be reused regardless of period
		 */
		duty_tick = airoha_pwm_get_duty_tick_from_ns(duty_ns,
							     period_ns);
		duty_tick_bucket =
			airoha_pwm_get_duty_tick_from_ns(bucket->duty_ns,
							 bucket->period_ns);
		if (duty_tick == AIROHA_PWM_DUTY_FULL &&
		    duty_tick == duty_tick_bucket)
			return i;
	}

	return unused;
}

static void airoha_pwm_release_bucket_config(struct airoha_pwm *pc,
					     unsigned int hwpwm)
{
	int bucket;

	/* Nothing to clear, PWM channel never used */
	if (!(pc->initialized & BIT_ULL(hwpwm)))
		return;

	bucket = pc->channel_bucket[hwpwm];
	pc->buckets[bucket].used &= ~BIT_ULL(hwpwm);
}

static int airoha_pwm_consume_generator(struct airoha_pwm *pc,
					u64 duty_ns, u64 period_ns,
					unsigned int hwpwm)
{
	int bucket;

	/*
	 * Search for a bucket that already satisfy duty and period
	 * or an unused one.
	 * If not found, -1 is returned.
	 */
	bucket = airoha_pwm_get_generator(pc, duty_ns, period_ns);
	if (bucket < 0)
		return bucket;

	airoha_pwm_release_bucket_config(pc, hwpwm);
	pc->buckets[bucket].used |= BIT_ULL(hwpwm);
	pc->buckets[bucket].period_ns = period_ns;
	pc->buckets[bucket].duty_ns = duty_ns;

	return bucket;
}

static int airoha_pwm_sipo_init(struct airoha_pwm *pc)
{
	u32 val;

	if (!(pc->initialized >> AIROHA_PWM_NUM_GPIO))
		return 0;

	regmap_clear_bits(pc->regmap, REG_SIPO_FLASH_MODE_CFG,
			  SERIAL_GPIO_MODE_74HC164_MASK);

	/* Configure shift register timings, use 32x divisor */
	regmap_write(pc->regmap, REG_SGPIO_CLK_DIVR,
		     FIELD_PREP(SGPIO_CLK_DIVR_MASK, 0x3));

	/*
	 * The actual delay is clock + 1.
	 * Notice that clock delay should not be greater
	 * than (divisor / 2) - 1.
	 * Set to 0 by default. (aka 1)
	 */
	regmap_write(pc->regmap, REG_SGPIO_CLK_DLY, 0x0);

	/*
	 * It it necessary to after muxing explicitly shift out all
	 * zeroes to initialize the shift register before enabling PWM
	 * mode because in PWM mode SIPO will not start shifting until
	 * it needs to output a non-zero value (bit 31 of led_data
	 * indicates shifting in progress and it must return to zero
	 * before led_data can be written or PWM mode can be set)
	 */
	if (regmap_read_poll_timeout(pc->regmap, REG_SGPIO_LED_DATA, val,
				     !(val & SGPIO_LED_DATA_SHIFT_FLAG_MASK),
				     10, 200 * USEC_PER_MSEC))
		return -ETIMEDOUT;

	regmap_clear_bits(pc->regmap, REG_SGPIO_LED_DATA,
			  SGPIO_LED_DATA_DATA_MASK);
	if (regmap_read_poll_timeout(pc->regmap, REG_SGPIO_LED_DATA, val,
				     !(val & SGPIO_LED_DATA_SHIFT_FLAG_MASK),
				     10, 200 * USEC_PER_MSEC))
		return -ETIMEDOUT;

	/* Set SIPO in PWM mode */
	regmap_set_bits(pc->regmap, REG_SIPO_FLASH_MODE_CFG,
			SERIAL_GPIO_FLASH_MODE_MASK);

	return 0;
}

static void airoha_pwm_calc_bucket_config(struct airoha_pwm *pc, int bucket,
					  u64 duty_ns, u64 period_ns)
{
	u32 period_tick, duty_tick, mask, val;

	duty_tick = airoha_pwm_get_duty_tick_from_ns(duty_ns, period_ns);
	period_tick = mul_u64_u64_div_u64(period_ns, AIROHA_PWM_PERIOD_MAX,
					  NSEC_PER_SEC);
	period_tick = min_t(u32, period_tick, AIROHA_PWM_PERIOD_MAX);

	/* Configure frequency divisor */
	mask = WAVE_GEN_CYCLE_MASK(bucket);
	val = AIROHA_PWM_FIELD_SET(mask, period_tick);
	regmap_update_bits(pc->regmap, REG_CYCLE_CFG_VALUE(bucket), mask, val);

	/* Configure duty cycle */
	mask = GPIO_FLASH_PRD_HIGH_MASK(bucket);
	val = AIROHA_PWM_FIELD_SET(mask, duty_tick);
	regmap_update_bits(pc->regmap, REG_GPIO_FLASH_PRD_SET(bucket),
			   mask, val);

	mask = GPIO_FLASH_PRD_LOW_MASK(bucket);
	val = AIROHA_PWM_FIELD_SET(mask, AIROHA_PWM_DUTY_FULL - duty_tick);
	regmap_update_bits(pc->regmap, REG_GPIO_FLASH_PRD_SET(bucket),
			   mask, val);
}

static void airoha_pwm_config_flash_map(struct airoha_pwm *pc,
					unsigned int hwpwm, int index)
{
	u32 addr, mask, val;

	if (hwpwm < AIROHA_PWM_NUM_GPIO) {
		addr = REG_GPIO_FLASH_MAP(hwpwm);
	} else {
		hwpwm -= AIROHA_PWM_NUM_GPIO;
		addr = REG_SIPO_FLASH_MAP(hwpwm);
	}

	/* index -1 means disable PWM channel */
	if (index < 0) {
		/*
		 * Change of waveform takes effect immediately but
		 * disabling has some delay so to prevent glitching
		 * only the enable bit is touched when disabling.
		 * Duty cycle can't be set to 0 as it might be shared with
		 * others channels with same duty cycle.
		 */
		regmap_clear_bits(pc->regmap, addr, GPIO_FLASH_EN_MASK(hwpwm));
		return;
	}

	mask = GPIO_FLASH_SET_ID_MASK(hwpwm);
	val = AIROHA_PWM_FIELD_SET(mask, index);
	regmap_update_bits(pc->regmap, addr, mask, val);
	regmap_set_bits(pc->regmap, addr, GPIO_FLASH_EN_MASK(hwpwm));
}

static int airoha_pwm_config(struct airoha_pwm *pc, struct pwm_device *pwm,
			     u64 duty_ns, u64 period_ns)
{
	int bucket, hwpwm = pwm->hwpwm;

	bucket = airoha_pwm_consume_generator(pc, duty_ns, period_ns,
					      hwpwm);
	if (bucket < 0)
		return -EBUSY;

	if (!(pc->initialized & BIT_ULL(hwpwm)) &&
	    hwpwm >= AIROHA_PWM_NUM_GPIO)
		airoha_pwm_sipo_init(pc);

	airoha_pwm_calc_bucket_config(pc, bucket, duty_ns, period_ns);
	airoha_pwm_config_flash_map(pc, hwpwm, bucket);

	pc->initialized |= BIT_ULL(hwpwm);
	pc->channel_bucket[hwpwm] = bucket;

	return 0;
}

static void airoha_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct airoha_pwm *pc = container_of(chip, struct airoha_pwm, chip);

	/* Disable PWM and release the bucket */
	airoha_pwm_config_flash_map(pc, pwm->hwpwm, -1);
	airoha_pwm_release_bucket_config(pc, pwm->hwpwm);

	pc->initialized &= ~BIT_ULL(pwm->hwpwm);
	if (!(pc->initialized >> AIROHA_PWM_NUM_GPIO))
		regmap_clear_bits(pc->regmap, REG_SIPO_FLASH_MODE_CFG,
				  SERIAL_GPIO_FLASH_MODE_MASK);
}

static int airoha_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			    const struct pwm_state *state)
{
	struct airoha_pwm *pc = container_of(chip, struct airoha_pwm, chip);
	u64 duty_ns = state->enabled ? state->duty_cycle : 0;
	u64 period_ns = state->period;

	/* Only normal polarity is supported */
	if (state->polarity == PWM_POLARITY_INVERSED)
		return -EINVAL;

	if (!state->enabled) {
		airoha_pwm_disable(chip, pwm);
		return 0;
	}

	if (period_ns < AIROHA_PWM_PERIOD_MIN_NS)
		return -EINVAL;

	return airoha_pwm_config(pc, pwm, duty_ns, period_ns);
}

static int airoha_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
				struct pwm_state *state)
{
	struct airoha_pwm *pc = container_of(chip, struct airoha_pwm, chip);
	int ret, hwpwm = pwm->hwpwm;
	u32 addr, val;
	u8 bucket;

	if (hwpwm < AIROHA_PWM_NUM_GPIO) {
		addr = REG_GPIO_FLASH_MAP(hwpwm);
	} else {
		hwpwm -= AIROHA_PWM_NUM_GPIO;
		addr = REG_SIPO_FLASH_MAP(hwpwm);
	}

	ret = regmap_read(pc->regmap, addr, &val);
	if (ret)
		return ret;

	state->enabled = AIROHA_PWM_FIELD_GET(GPIO_FLASH_EN_MASK(hwpwm), val);
	if (!state->enabled)
		return 0;

	state->polarity = PWM_POLARITY_NORMAL;

	bucket = AIROHA_PWM_FIELD_GET(GPIO_FLASH_SET_ID_MASK(hwpwm), val);
	ret = regmap_read(pc->regmap, REG_CYCLE_CFG_VALUE(bucket), &val);
	if (ret)
		return ret;

	state->period = pc->buckets[bucket].period_ns;
	state->duty_cycle = pc->buckets[bucket].duty_ns;

	return 0;
}

static const struct pwm_ops airoha_pwm_ops = {
	.apply = airoha_pwm_apply,
	.get_state = airoha_pwm_get_state,
	.owner = THIS_MODULE,
};

static int airoha_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct airoha_pwm *pc;
	int ret;

	pc = devm_kzalloc(dev, sizeof(*pc), GFP_KERNEL);
	if (!pc)
		return -ENOMEM;

	pc->chip.dev = dev;
	pc->chip.ops = &airoha_pwm_ops;
	pc->chip.npwm = AIROHA_PWM_MAX_CHANNELS;

	pc->regmap = device_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(pc->regmap)) {
		dev_err_probe(dev, PTR_ERR(pc->regmap), "failed to get PWM regmap");
		return PTR_ERR(pc->regmap);
	}

	platform_set_drvdata(pdev, pc);

	ret = pwmchip_add(&pc->chip);
	if (ret)
		dev_err_probe(dev, ret, "failed to add PWM chip");

	return ret;
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
		.name = "pwm-airoha",
		.of_match_table = airoha_pwm_of_match,
	},
	.probe = airoha_pwm_probe,
	.remove = airoha_pwm_remove,
};
module_platform_driver(airoha_pwm_driver);

MODULE_AUTHOR("Lorenzo Bianconi <lorenzo@kernel.org>");
MODULE_AUTHOR("Markus Gothe <markus.gothe@genexis.eu>");
MODULE_AUTHOR("Benjamin Larsson <benjamin.larsson@genexis.eu>");
MODULE_DESCRIPTION("Airoha EN7581 PWM driver");
MODULE_LICENSE("GPL");
