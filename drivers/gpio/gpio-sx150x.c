/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/i2c/sx150x.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>

#define NO_UPDATE_PENDING	-1

/* The chip models of sx150x */
#define SX150X_456 0
#define SX150X_789 1

struct sx150x_456_pri {
	u8 reg_pld_mode;
	u8 reg_pld_table0;
	u8 reg_pld_table1;
	u8 reg_pld_table2;
	u8 reg_pld_table3;
	u8 reg_pld_table4;
	u8 reg_advance;
};

struct sx150x_789_pri {
	u8 reg_drain;
	u8 reg_polarity;
	u8 reg_clock;
	u8 reg_misc;
	u8 reg_reset;
	u8 ngpios;
};

struct sx150x_device_data {
	u8 model;
	u8 reg_pullup;
	u8 reg_pulldn;
	u8 reg_dir;
	u8 reg_data;
	u8 reg_irq_mask;
	u8 reg_irq_src;
	u8 reg_sense;
	u8 ngpios;
	union {
		struct sx150x_456_pri x456;
		struct sx150x_789_pri x789;
	} pri;
};

struct sx150x_chip {
	struct gpio_chip                 gpio_chip;
	struct i2c_client               *client;
	const struct sx150x_device_data *dev_cfg;
	int                              irq_summary;
	int                              irq_base;
	int				 irq_update;
	u32                              irq_sense;
	u32				 irq_masked;
	u32				 dev_sense;
	u32				 dev_masked;
	struct irq_chip                  irq_chip;
	struct mutex                     lock;
};

static const struct sx150x_device_data sx150x_devices[] = {
	[0] = { /* sx1508q */
		.model = SX150X_789,
		.reg_pullup	= 0x03,
		.reg_pulldn	= 0x04,
		.reg_dir	= 0x07,
		.reg_data	= 0x08,
		.reg_irq_mask	= 0x09,
		.reg_irq_src	= 0x0c,
		.reg_sense	= 0x0b,
		.pri.x789 = {
			.reg_drain	= 0x05,
			.reg_polarity	= 0x06,
			.reg_clock	= 0x0f,
			.reg_misc	= 0x10,
			.reg_reset	= 0x7d,
		},
		.ngpios = 8,
	},
	[1] = { /* sx1509q */
		.model = SX150X_789,
		.reg_pullup	= 0x07,
		.reg_pulldn	= 0x09,
		.reg_dir	= 0x0f,
		.reg_data	= 0x11,
		.reg_irq_mask	= 0x13,
		.reg_irq_src	= 0x19,
		.reg_sense	= 0x17,
		.pri.x789 = {
			.reg_drain	= 0x0b,
			.reg_polarity	= 0x0d,
			.reg_clock	= 0x1e,
			.reg_misc	= 0x1f,
			.reg_reset	= 0x7d,
		},
		.ngpios	= 16
	},
	[2] = { /* sx1506q */
		.model = SX150X_456,
		.reg_pullup	= 0x05,
		.reg_pulldn	= 0x07,
		.reg_dir	= 0x03,
		.reg_data	= 0x01,
		.reg_irq_mask	= 0x09,
		.reg_irq_src	= 0x0f,
		.reg_sense	= 0x0d,
		.pri.x456 = {
			.reg_pld_mode	= 0x21,
			.reg_pld_table0	= 0x23,
			.reg_pld_table1	= 0x25,
			.reg_pld_table2	= 0x27,
			.reg_pld_table3	= 0x29,
			.reg_pld_table4	= 0x2b,
			.reg_advance	= 0xad,
		},
		.ngpios	= 16
	},
};

static const struct i2c_device_id sx150x_id[] = {
	{"sx1508q", 0},
	{"sx1509q", 1},
	{"sx1506q", 2},
	{}
};
MODULE_DEVICE_TABLE(i2c, sx150x_id);

static const struct of_device_id sx150x_of_match[] = {
	{ .compatible = "semtech,sx1508q" },
	{ .compatible = "semtech,sx1509q" },
	{ .compatible = "semtech,sx1506q" },
	{},
};
MODULE_DEVICE_TABLE(of, sx150x_of_match);

struct sx150x_chip *to_sx150x(struct gpio_chip *gc)
{
	return container_of(gc, struct sx150x_chip, gpio_chip);
}

static s32 sx150x_i2c_write(struct i2c_client *client, u8 reg, u8 val)
{
	s32 err = i2c_smbus_write_byte_data(client, reg, val);

	if (err < 0)
		dev_warn(&client->dev,
			"i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, err);
	return err;
}

static s32 sx150x_i2c_read(struct i2c_client *client, u8 reg, u8 *val)
{
	s32 err = i2c_smbus_read_byte_data(client, reg);

	if (err >= 0)
		*val = err;
	else
		dev_warn(&client->dev,
			"i2c read fail: can't read from %02x: %d\n",
			reg, err);
	return err;
}

static inline bool offset_is_oscio(struct sx150x_chip *chip, unsigned offset)
{
	return (chip->dev_cfg->ngpios == offset);
}

/*
 * These utility functions solve the common problem of locating and setting
 * configuration bits.  Configuration bits are grouped into registers
 * whose indexes increase downwards.  For example, with eight-bit registers,
 * sixteen gpios would have their config bits grouped in the following order:
 * REGISTER N-1 [ f e d c b a 9 8 ]
 *          N   [ 7 6 5 4 3 2 1 0 ]
 *
 * For multi-bit configurations, the pattern gets wider:
 * REGISTER N-3 [ f f e e d d c c ]
 *          N-2 [ b b a a 9 9 8 8 ]
 *          N-1 [ 7 7 6 6 5 5 4 4 ]
 *          N   [ 3 3 2 2 1 1 0 0 ]
 *
 * Given the address of the starting register 'N', the index of the gpio
 * whose configuration we seek to change, and the width in bits of that
 * configuration, these functions allow us to locate the correct
 * register and mask the correct bits.
 */
static inline void sx150x_find_cfg(u8 offset, u8 width,
				u8 *reg, u8 *mask, u8 *shift)
{
	*reg   -= offset * width / 8;
	*mask   = (1 << width) - 1;
	*shift  = (offset * width) % 8;
	*mask <<= *shift;
}

static s32 sx150x_write_cfg(struct sx150x_chip *chip,
			u8 offset, u8 width, u8 reg, u8 val)
{
	u8  mask;
	u8  data;
	u8  shift;
	s32 err;

	sx150x_find_cfg(offset, width, &reg, &mask, &shift);
	err = sx150x_i2c_read(chip->client, reg, &data);
	if (err < 0)
		return err;

	data &= ~mask;
	data |= (val << shift) & mask;
	return sx150x_i2c_write(chip->client, reg, data);
}

static int sx150x_get_io(struct sx150x_chip *chip, unsigned offset)
{
	u8  reg = chip->dev_cfg->reg_data;
	u8  mask;
	u8  data;
	u8  shift;
	s32 err;

	sx150x_find_cfg(offset, 1, &reg, &mask, &shift);
	err = sx150x_i2c_read(chip->client, reg, &data);
	if (err >= 0)
		err = (data & mask) != 0 ? 1 : 0;

	return err;
}

static void sx150x_set_oscio(struct sx150x_chip *chip, int val)
{
	sx150x_i2c_write(chip->client,
			chip->dev_cfg->pri.x789.reg_clock,
			(val ? 0x1f : 0x10));
}

static void sx150x_set_io(struct sx150x_chip *chip, unsigned offset, int val)
{
	sx150x_write_cfg(chip,
			offset,
			1,
			chip->dev_cfg->reg_data,
			(val ? 1 : 0));
}

static int sx150x_io_input(struct sx150x_chip *chip, unsigned offset)
{
	return sx150x_write_cfg(chip,
				offset,
				1,
				chip->dev_cfg->reg_dir,
				1);
}

static int sx150x_io_output(struct sx150x_chip *chip, unsigned offset, int val)
{
	int err;

	err = sx150x_write_cfg(chip,
			offset,
			1,
			chip->dev_cfg->reg_data,
			(val ? 1 : 0));
	if (err >= 0)
		err = sx150x_write_cfg(chip,
				offset,
				1,
				chip->dev_cfg->reg_dir,
				0);
	return err;
}

static int sx150x_gpio_get(struct gpio_chip *gc, unsigned offset)
{
	struct sx150x_chip *chip = to_sx150x(gc);
	int status = -EINVAL;

	if (!offset_is_oscio(chip, offset)) {
		mutex_lock(&chip->lock);
		status = sx150x_get_io(chip, offset);
		mutex_unlock(&chip->lock);
	}

	return status;
}

static void sx150x_gpio_set(struct gpio_chip *gc, unsigned offset, int val)
{
	struct sx150x_chip *chip = to_sx150x(gc);

	mutex_lock(&chip->lock);
	if (offset_is_oscio(chip, offset))
		sx150x_set_oscio(chip, val);
	else
		sx150x_set_io(chip, offset, val);
	mutex_unlock(&chip->lock);
}

static int sx150x_gpio_direction_input(struct gpio_chip *gc, unsigned offset)
{
	struct sx150x_chip *chip = to_sx150x(gc);
	int status = -EINVAL;

	if (!offset_is_oscio(chip, offset)) {
		mutex_lock(&chip->lock);
		status = sx150x_io_input(chip, offset);
		mutex_unlock(&chip->lock);
	}
	return status;
}

static int sx150x_gpio_direction_output(struct gpio_chip *gc,
					unsigned offset,
					int val)
{
	struct sx150x_chip *chip = to_sx150x(gc);
	int status = 0;

	if (!offset_is_oscio(chip, offset)) {
		mutex_lock(&chip->lock);
		status = sx150x_io_output(chip, offset, val);
		mutex_unlock(&chip->lock);
	}
	return status;
}

static void sx150x_irq_mask(struct irq_data *d)
{
	struct sx150x_chip *chip = to_sx150x(irq_data_get_irq_chip_data(d));
	unsigned n = d->hwirq;

	chip->irq_masked |= (1 << n);
	chip->irq_update = n;
}

static void sx150x_irq_unmask(struct irq_data *d)
{
	struct sx150x_chip *chip = to_sx150x(irq_data_get_irq_chip_data(d));
	unsigned n = d->hwirq;

	chip->irq_masked &= ~(1 << n);
	chip->irq_update = n;
}

static int sx150x_irq_set_type(struct irq_data *d, unsigned int flow_type)
{
	struct sx150x_chip *chip = to_sx150x(irq_data_get_irq_chip_data(d));
	unsigned n, val = 0;

	if (flow_type & (IRQ_TYPE_LEVEL_HIGH | IRQ_TYPE_LEVEL_LOW))
		return -EINVAL;

	n = d->hwirq;

	if (flow_type & IRQ_TYPE_EDGE_RISING)
		val |= 0x1;
	if (flow_type & IRQ_TYPE_EDGE_FALLING)
		val |= 0x2;

	chip->irq_sense &= ~(3UL << (n * 2));
	chip->irq_sense |= val << (n * 2);
	chip->irq_update = n;
	return 0;
}

static irqreturn_t sx150x_irq_thread_fn(int irq, void *dev_id)
{
	struct sx150x_chip *chip = (struct sx150x_chip *)dev_id;
	unsigned nhandled = 0;
	unsigned sub_irq;
	unsigned n;
	s32 err;
	u8 val;
	int i;

	for (i = (chip->dev_cfg->ngpios / 8) - 1; i >= 0; --i) {
		err = sx150x_i2c_read(chip->client,
				      chip->dev_cfg->reg_irq_src - i,
				      &val);
		if (err < 0)
			continue;

		sx150x_i2c_write(chip->client,
				chip->dev_cfg->reg_irq_src - i,
				val);
		for (n = 0; n < 8; ++n) {
			if (val & (1 << n)) {
				sub_irq = irq_find_mapping(
					chip->gpio_chip.irqdomain,
					(i * 8) + n);
				handle_nested_irq(sub_irq);
				++nhandled;
			}
		}
	}

	return (nhandled > 0 ? IRQ_HANDLED : IRQ_NONE);
}

static void sx150x_irq_bus_lock(struct irq_data *d)
{
	struct sx150x_chip *chip = to_sx150x(irq_data_get_irq_chip_data(d));

	mutex_lock(&chip->lock);
}

static void sx150x_irq_bus_sync_unlock(struct irq_data *d)
{
	struct sx150x_chip *chip = to_sx150x(irq_data_get_irq_chip_data(d));
	unsigned n;

	if (chip->irq_update == NO_UPDATE_PENDING)
		goto out;

	n = chip->irq_update;
	chip->irq_update = NO_UPDATE_PENDING;

	/* Avoid updates if nothing changed */
	if (chip->dev_sense == chip->irq_sense &&
	    chip->dev_masked == chip->irq_masked)
		goto out;

	chip->dev_sense = chip->irq_sense;
	chip->dev_masked = chip->irq_masked;

	if (chip->irq_masked & (1 << n)) {
		sx150x_write_cfg(chip, n, 1, chip->dev_cfg->reg_irq_mask, 1);
		sx150x_write_cfg(chip, n, 2, chip->dev_cfg->reg_sense, 0);
	} else {
		sx150x_write_cfg(chip, n, 1, chip->dev_cfg->reg_irq_mask, 0);
		sx150x_write_cfg(chip, n, 2, chip->dev_cfg->reg_sense,
				 chip->irq_sense >> (n * 2));
	}
out:
	mutex_unlock(&chip->lock);
}

static void sx150x_init_chip(struct sx150x_chip *chip,
			struct i2c_client *client,
			kernel_ulong_t driver_data,
			struct sx150x_platform_data *pdata)
{
	mutex_init(&chip->lock);

	chip->client                     = client;
	chip->dev_cfg                    = &sx150x_devices[driver_data];
	chip->gpio_chip.parent              = &client->dev;
	chip->gpio_chip.label            = client->name;
	chip->gpio_chip.direction_input  = sx150x_gpio_direction_input;
	chip->gpio_chip.direction_output = sx150x_gpio_direction_output;
	chip->gpio_chip.get              = sx150x_gpio_get;
	chip->gpio_chip.set              = sx150x_gpio_set;
	chip->gpio_chip.base             = pdata->gpio_base;
	chip->gpio_chip.can_sleep        = true;
	chip->gpio_chip.ngpio            = chip->dev_cfg->ngpios;
#ifdef CONFIG_OF_GPIO
	chip->gpio_chip.of_node          = client->dev.of_node;
	chip->gpio_chip.of_gpio_n_cells  = 2;
#endif
	if (pdata->oscio_is_gpo)
		++chip->gpio_chip.ngpio;

	chip->irq_chip.name                = client->name;
	chip->irq_chip.irq_mask            = sx150x_irq_mask;
	chip->irq_chip.irq_unmask          = sx150x_irq_unmask;
	chip->irq_chip.irq_set_type        = sx150x_irq_set_type;
	chip->irq_chip.irq_bus_lock        = sx150x_irq_bus_lock;
	chip->irq_chip.irq_bus_sync_unlock = sx150x_irq_bus_sync_unlock;
	chip->irq_summary                  = -1;
	chip->irq_base                     = -1;
	chip->irq_masked                   = ~0;
	chip->irq_sense                    = 0;
	chip->dev_masked                   = ~0;
	chip->dev_sense                    = 0;
	chip->irq_update		   = NO_UPDATE_PENDING;
}

static int sx150x_init_io(struct sx150x_chip *chip, u8 base, u16 cfg)
{
	int err = 0;
	unsigned n;

	for (n = 0; err >= 0 && n < (chip->dev_cfg->ngpios / 8); ++n)
		err = sx150x_i2c_write(chip->client, base - n, cfg >> (n * 8));
	return err;
}

static int sx150x_reset(struct sx150x_chip *chip)
{
	int err;

	err = i2c_smbus_write_byte_data(chip->client,
					chip->dev_cfg->pri.x789.reg_reset,
					0x12);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(chip->client,
					chip->dev_cfg->pri.x789.reg_reset,
					0x34);
	return err;
}

static int sx150x_init_hw(struct sx150x_chip *chip,
			struct sx150x_platform_data *pdata)
{
	int err = 0;

	if (pdata->reset_during_probe) {
		err = sx150x_reset(chip);
		if (err < 0)
			return err;
	}

	if (chip->dev_cfg->model == SX150X_789)
		err = sx150x_i2c_write(chip->client,
				chip->dev_cfg->pri.x789.reg_misc,
				0x01);
	else
		err = sx150x_i2c_write(chip->client,
				chip->dev_cfg->pri.x456.reg_advance,
				0x04);
	if (err < 0)
		return err;

	err = sx150x_init_io(chip, chip->dev_cfg->reg_pullup,
			pdata->io_pullup_ena);
	if (err < 0)
		return err;

	err = sx150x_init_io(chip, chip->dev_cfg->reg_pulldn,
			pdata->io_pulldn_ena);
	if (err < 0)
		return err;

	if (chip->dev_cfg->model == SX150X_789) {
		err = sx150x_init_io(chip,
				chip->dev_cfg->pri.x789.reg_drain,
				pdata->io_open_drain_ena);
		if (err < 0)
			return err;

		err = sx150x_init_io(chip,
				chip->dev_cfg->pri.x789.reg_polarity,
				pdata->io_polarity);
		if (err < 0)
			return err;
	} else {
		/* Set all pins to work in normal mode */
		err = sx150x_init_io(chip,
				chip->dev_cfg->pri.x456.reg_pld_mode,
				0);
		if (err < 0)
			return err;
	}


	if (pdata->oscio_is_gpo)
		sx150x_set_oscio(chip, 0);

	return err;
}

static int sx150x_install_irq_chip(struct sx150x_chip *chip,
				int irq_summary,
				int irq_base)
{
	int err;

	chip->irq_summary = irq_summary;
	chip->irq_base    = irq_base;

	/* Add gpio chip to irq subsystem */
	err = gpiochip_irqchip_add(&chip->gpio_chip,
		&chip->irq_chip, chip->irq_base,
		handle_edge_irq, IRQ_TYPE_EDGE_BOTH);
	if (err) {
		dev_err(&chip->client->dev,
			"could not connect irqchip to gpiochip\n");
		return  err;
	}

	err = devm_request_threaded_irq(&chip->client->dev,
			irq_summary, NULL, sx150x_irq_thread_fn,
			IRQF_ONESHOT | IRQF_SHARED | IRQF_TRIGGER_FALLING,
			chip->irq_chip.name, chip);
	if (err < 0) {
		chip->irq_summary = -1;
		chip->irq_base    = -1;
	}

	return err;
}
static struct sx150x_platform_data *of_sx150x_get_platdata(
                                        struct i2c_client *client)
{
        int rc, gpio;
        struct sx150x_platform_data *pdata;
        struct device_node *np;
	u32 value;
        if (!client->dev.of_node)
                return NULL;
        np = client->dev.of_node;
        pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
        if (!pdata)
                return ERR_PTR(-ENOMEM);
        gpio = of_get_named_gpio(np, "int-gpios", 0);
        if (gpio_is_valid(gpio)) {
                rc = devm_gpio_request_one(&client->dev, gpio,
                        GPIOF_DIR_IN, "sx150x_interrupt");
                if (rc)
                        return ERR_PTR(rc);
        }
        pdata->irq_summary = irq_of_parse_and_map(np, 0);
        if (!pdata->irq_summary)
                return ERR_PTR(-EPROBE_DEFER);
        pdata->oscio_is_gpo = of_property_read_bool(np, "oscio_is_gpo");
        pdata->reset_during_probe =of_property_read_bool(np, "reset_during_probe");
        rc = of_property_read_u32(np, "pullup_ena",&value);
        if (rc)
                pdata->io_pullup_ena = 0;
	else
		pdata->io_pullup_ena = value&0xffff;
        rc = of_property_read_u32(np, "pulldn_ena",&value);
        if (rc)
                pdata->io_pulldn_ena = 0;
	else
		pdata->io_pulldn_ena = value&0xffff;
        rc = of_property_read_u32(np, "open_drain_ena",&value);
        if (rc)
                pdata->io_open_drain_ena = 0;
	else
		pdata->io_pulldn_ena = value&0xffff;
        rc = of_property_read_u32(np, "polarity",&value);
        if (rc)
                pdata->io_polarity = 0;
	else
		pdata->io_polarity = value&0xffff;
	printk("io_pullup_ena=0x%x,io_pulldn_ena=0x%x,io_open_drain_ena=0x%x,pdata->io_polarity=0x%x\n",pdata->io_pullup_ena,pdata->io_pulldn_ena,pdata->io_open_drain_ena,pdata->io_polarity);
        pdata->gpio_base = -1;
        pdata->irq_base = 0;
        return pdata;
}
//test pwd
#if 0
void init(struct i2c_client *client)
{
#define REG_MISC 0x1f
#define REG_CLOCK 0x1e
	sx150x_i2c_write(client,REG_MISC,16);
	sx150x_i2c_write(client,REG_CLOCK,64);
}
#define 	REG_INPUT_DISABLE_B		0x00	//	RegInputDisableB Input buffer disable register _ I/O[15_8] (Bank B) 0000 0000
#define 	REG_INPUT_DISABLE_A		0x01	//	RegInputDisableA Input buffer disable register _ I/O[7_0] (Bank A) 0000 0000
#define 	REG_LONG_SLEW_B			0x02	//	RegLongSlewB Output buffer long slew register _ I/O[15_8] (Bank B) 0000 0000
#define 	REG_LONG_SLEW_A			0x03	//	RegLongSlewA Output buffer long slew register _ I/O[7_0] (Bank A) 0000 0000
#define 	REG_LOW_DRIVE_B			0x04	//	RegLowDriveB Output buffer low drive register _ I/O[15_8] (Bank B) 0000 0000
#define 	REG_LOW_DRIVE_A			0x05	//	RegLowDriveA Output buffer low drive register _ I/O[7_0] (Bank A) 0000 0000
#define 	REG_PULL_UP_B			0x06	//	RegPullUpB Pull_up register _ I/O[15_8] (Bank B) 0000 0000
#define 	REG_PULL_UP_A			0x07	//	RegPullUpA Pull_up register _ I/O[7_0] (Bank A) 0000 0000
#define 	REG_PULL_DOWN_B			0x08	//	RegPullDownB Pull_down register _ I/O[15_8] (Bank B) 0000 0000
#define 	REG_PULL_DOWN_A			0x09	//	RegPullDownA Pull_down register _ I/O[7_0] (Bank A) 0000 0000
#define 	REG_OPEN_DRAIN_B		0x0A	//	RegOpenDrainB Open drain register _ I/O[15_8] (Bank B) 0000 0000
#define 	REG_OPEN_DRAIN_A		0x0B	//	RegOpenDrainA Open drain register _ I/O[7_0] (Bank A) 0000 0000
#define 	REG_POLARITY_B			0x0C	//	RegPolarityB Polarity register _ I/O[15_8] (Bank B) 0000 0000
#define 	REG_POLARITY_A			0x0D	//	RegPolarityA Polarity register _ I/O[7_0] (Bank A) 0000 0000
#define 	REG_DIR_B				0x0E	//	RegDirB Direction register _ I/O[15_8] (Bank B) 1111 1111
#define 	REG_DIR_A				0x0F	//	RegDirA Direction register _ I/O[7_0] (Bank A) 1111 1111
#define 	REG_DATA_B				0x10	//	RegDataB Data register _ I/O[15_8] (Bank B) 1111 1111*
#define 	REG_DATA_A				0x11	//	RegDataA Data register _ I/O[7_0] (Bank A) 1111 1111*
#define 	REG_INTERRUPT_MASK_B	0x12	//	RegInterruptMaskB Interrupt mask register _ I/O[15_8] (Bank B) 1111 1111
#define 	REG_INTERRUPT_MASK_A	0x13	//	RegInterruptMaskA Interrupt mask register _ I/O[7_0] (Bank A) 1111 1111
#define 	REG_SENSE_HIGH_B		0x14	//	RegSenseHighB Sense register for I/O[15:12] 0000 0000
#define 	REG_SENSE_LOW_B			0x15	//	RegSenseLowB Sense register for I/O[11:8] 0000 0000
#define 	REG_SENSE_HIGH_A		0x16	//	RegSenseHighA Sense register for I/O[7:4] 0000 0000
#define 	REG_SENSE_LOW_A			0x17	//	RegSenseLowA Sense register for I/O[3:0] 0000 0000
#define 	REG_INTERRUPT_SOURCE_B	0x18	//	RegInterruptSourceB Interrupt source register _ I/O[15_8] (Bank B) 0000 0000
#define 	REG_INTERRUPT_SOURCE_A	0x19	//	RegInterruptSourceA Interrupt source register _ I/O[7_0] (Bank A) 0000 0000
#define 	REG_EVENT_STATUS_B		0x1A	//	RegEventStatusB Event status register _ I/O[15_8] (Bank B) 0000 0000
#define 	REG_EVENT_STATUS_A		0x1B	//	RegEventStatusA Event status register _ I/O[7_0] (Bank A) 0000 0000
#define 	REG_LEVEL_SHIFTER_1		0x1C	//	RegLevelShifter1 Level shifter register 0000 0000
#define 	REG_LEVEL_SHIFTER_2		0x1D	//	RegLevelShifter2 Level shifter register 0000 0000
#define 	REG_LED_DRIVER_ENABLE_B	0x20	//	RegLEDDriverEnableB LED driver enable register _ I/O[15_8] (Bank B) 0000 0000
#define 	REG_LED_DRIVER_ENABLE_A	0x21	//	RegLEDDriverEnableA LED driver enable register _ I/O[7_0] (Bank A) 0000 0000
#define 	REG_DEBOUNCE_CONFIG		0x22	//	RegDebounceConfig Debounce configuration register 0000 0000
#define 	REG_DEBOUNCE_ENABLE_B	0x23	//	RegDebounceEnableB Debounce enable register _ I/O[15_8] (Bank B) 0000 0000
#define 	REG_DEBOUNCE_ENABLE_A	0x24	//	RegDebounceEnableA Debounce enable register _ I/O[7_0] (Bank A) 0000 0000
#define 	REG_KEY_CONFIG_1		0x25	//	RegKeyConfig1 Key scan configuration register 0000 0000
#define 	REG_KEY_CONFIG_2		0x26	//	RegKeyConfig2 Key scan configuration register 0000 0000
#define 	REG_KEY_DATA_1			0x27	//	RegKeyData1 Key value (column) 1111 1111
#define 	REG_KEY_DATA_2			0x28	//	RegKeyData2 Key value (row) 1111 1111
#define 	REG_T_ON_0				0x29	//	RegTOn0 ON time register for I/O[0] 0000 0000
#define 	REG_I_ON_0				0x2A	//	RegIOn0 ON intensity register for I/O[0] 1111 1111
#define 	REG_OFF_0				0x2B	//	RegOff0 OFF time/intensity register for I/O[0] 0000 0000
#define 	REG_T_ON_1				0x2C	//	RegTOn1 ON time register for I/O[1] 0000 0000
#define 	REG_I_ON_1				0x2D	//	RegIOn1 ON intensity register for I/O[1] 1111 1111
#define 	REG_OFF_1				0x2E	//	RegOff1 OFF time/intensity register for I/O[1] 0000 0000
#define 	REG_T_ON_2				0x2F	//	RegTOn2 ON time register for I/O[2] 0000 0000
#define 	REG_I_ON_2				0x30	//	RegIOn2 ON intensity register for I/O[2] 1111 1111
#define 	REG_OFF_2				0x31	//	RegOff2 OFF time/intensity register for I/O[2] 0000 0000
#define 	REG_T_ON_3				0x32	//	RegTOn3 ON time register for I/O[3] 0000 0000
#define 	REG_I_ON_3				0x33	//	RegIOn3 ON intensity register for I/O[3] 1111 1111
#define 	REG_OFF_3				0x34	//	RegOff3 OFF time/intensity register for I/O[3] 0000 0000
#define 	REG_T_ON_4				0x35	//	RegTOn4 ON time register for I/O[4] 0000 0000
#define 	REG_I_ON_4				0x36	//	RegIOn4 ON intensity register for I/O[4] 1111 1111
#define 	REG_OFF_4				0x37	//	RegOff4 OFF time/intensity register for I/O[4] 0000 0000
#define 	REG_T_RISE_4			0x38	//	RegTRise4 Fade in register for I/O[4] 0000 0000
#define 	REG_T_FALL_4			0x39	//	RegTFall4 Fade out register for I/O[4] 0000 0000
#define 	REG_T_ON_5				0x3A	//	RegTOn5 ON time register for I/O[5] 0000 0000
#define 	REG_I_ON_5				0x3B	//	RegIOn5 ON intensity register for I/O[5] 1111 1111
#define 	REG_OFF_5				0x3C	//	RegOff5 OFF time/intensity register for I/O[5] 0000 0000
#define 	REG_T_RISE_5			0x3D	//	RegTRise5 Fade in register for I/O[5] 0000 0000
#define 	REG_T_FALL_5			0x3E	//	RegTFall5 Fade out register for I/O[5] 0000 0000
#define 	REG_T_ON_6				0x3F	//	RegTOn6 ON time register for I/O[6] 0000 0000
#define 	REG_I_ON_6				0x40	//	RegIOn6 ON intensity register for I/O[6] 1111 1111
#define 	REG_OFF_6				0x41	//	RegOff6 OFF time/intensity register for I/O[6] 0000 0000
#define 	REG_T_RISE_6			0x42	//	RegTRise6 Fade in register for I/O[6] 0000 0000
#define 	REG_T_FALL_6			0x43	//	RegTFall6 Fade out register for I/O[6] 0000 0000
#define 	REG_T_ON_7				0x44	//	RegTOn7 ON time register for I/O[7] 0000 0000
#define 	REG_I_ON_7				0x45	//	RegIOn7 ON intensity register for I/O[7] 1111 1111
#define 	REG_OFF_7				0x46	//	RegOff7 OFF time/intensity register for I/O[7] 0000 0000
#define 	REG_T_RISE_7			0x47	//	RegTRise7 Fade in register for I/O[7] 0000 0000
#define 	REG_T_FALL_7			0x48	//	RegTFall7 Fade out register for I/O[7] 0000 0000
#define 	REG_T_ON_8				0x49	//	RegTOn8 ON time register for I/O[8] 0000 0000
#define 	REG_I_ON_8				0x4A	//	RegIOn8 ON intensity register for I/O[8] 1111 1111
#define 	REG_OFF_8				0x4B	//	RegOff8 OFF time/intensity register for I/O[8] 0000 0000
#define 	REG_T_ON_9				0x4C	//	RegTOn9 ON time register for I/O[9] 0000 0000
#define 	REG_I_ON_9				0x4D	//	RegIOn9 ON intensity register for I/O[9] 1111 1111
#define 	REG_OFF_9				0x4E	//	RegOff9 OFF time/intensity register for I/O[9] 0000 0000
#define 	REG_T_ON_10				0x4F	//	RegTOn10 ON time register for I/O[10] 0000 0000
#define 	REG_I_ON_10				0x50	//	RegIOn10 ON intensity register for I/O[10] 1111 1111
#define 	REG_OFF_10				0x51	//	RegOff10 OFF time/intensity register for I/O[10] 0000 0000
#define 	REG_T_ON_11				0x52	//	RegTOn11 ON time register for I/O[11] 0000 0000
#define 	REG_I_ON_11				0x53	//	RegIOn11 ON intensity register for I/O[11] 1111 1111
#define 	REG_OFF_11				0x54	//	RegOff11 OFF time/intensity register for I/O[11] 0000 0000
#define 	REG_T_ON_12				0x55	//	RegTOn12 ON time register for I/O[12] 0000 0000
#define 	REG_I_ON_12				0x56	//	RegIOn12 ON intensity register for I/O[12] 1111 1111
#define 	REG_OFF_12				0x57	//	RegOff12 OFF time/intensity register for I/O[12] 0000 0000
#define 	REG_T_RISE_12			0x58	//	RegTRise12 Fade in register for I/O[12] 0000 0000
#define 	REG_T_FALL_12			0x59	//	RegTFall12 Fade out register for I/O[12] 0000 0000
#define 	REG_T_ON_13				0x5A	//	RegTOn13 ON time register for I/O[13] 0000 0000
#define 	REG_I_ON_13				0x5B	//	RegIOn13 ON intensity register for I/O[13] 1111 1111
#define 	REG_OFF_13				0x5C	//	RegOff13 OFF time/intensity register for I/O[13] 0000 0000
#define 	REG_T_RISE_13			0x5D	//	RegTRise13 Fade in register for I/O[13] 0000 0000
#define 	REG_T_FALL_13			0x5E	//	RegTFall13 Fade out register for I/O[13] 0000 0000
#define 	REG_T_ON_14				0x5F	//	RegTOn14 ON time register for I/O[14] 0000 0000
#define 	REG_I_ON_14				0x60	//	RegIOn14 ON intensity register for I/O[14] 1111 1111
#define 	REG_OFF_14				0x61	//	RegOff14 OFF time/intensity register for I/O[14] 0000 0000
#define 	REG_T_RISE_14			0x62	//	RegTRise14 Fade in register for I/O[14] 0000 0000
#define 	REG_T_FALL_14			0x63	//	RegTFall14 Fade out register for I/O[14] 0000 0000
#define 	REG_T_ON_15				0x64	//	RegTOn15 ON time register for I/O[15] 0000 0000
#define 	REG_I_ON_15				0x65	//	RegIOn15 ON intensity register for I/O[15] 1111 1111
#define 	REG_OFF_15				0x66	//	RegOff15 OFF time/intensity register for I/O[15] 0000 0000
#define 	REG_T_RISE_15			0x67	//	RegTRise15 Fade in register for I/O[15] 0000 0000
#define 	REG_T_FALL_15			0x68	//	RegTFall15 Fade out register for I/O[15] 0000 0000
#define 	REG_HIGH_INPUT_B		0x69	//	RegHighInputB High input enable register _ I/O[15_8] (Bank B) 0000 0000
#define 	REG_HIGH_INPUT_A		0x6A	//	RegHighInputA High input enable register _ I/O[7_0] (Bank A) 0000 0000
#define 	REG_RESET				0x7D	//	RegReset Software reset register 0000 0000
#define 	REG_TEST_1				0x7E	//	RegTest1 Test register 0000 0000
#define 	REG_TEST_2				0x7F	//	RegTest2 Test register 0000 0000
uint8_t REG_I_ON[16] = {REG_I_ON_0, REG_I_ON_1, REG_I_ON_2, REG_I_ON_3,
					REG_I_ON_4, REG_I_ON_5, REG_I_ON_6, REG_I_ON_7,
					REG_I_ON_8, REG_I_ON_9, REG_I_ON_10, REG_I_ON_11,
					REG_I_ON_12, REG_I_ON_13, REG_I_ON_14, REG_I_ON_15};
void  ledDriverInit(struct i2c_client *client, uint8_t pin)
{
	if(pin > 7)
	{
		uint8_t pinb = pin-8;
		uint8_t temp;
		sx150x_i2c_read(client,REG_INPUT_DISABLE_B,&temp);
		temp |= (1<<pinb);
		sx150x_i2c_write(client,REG_INPUT_DISABLE_B,temp);
		sx150x_i2c_read(client,REG_PULL_UP_B,&temp);
		temp &= ~(1<<pinb);
		sx150x_i2c_write(client,REG_PULL_UP_B,temp);
		sx150x_i2c_read(client,REG_DIR_B,&temp);
                temp &= ~(1<<pinb);
                sx150x_i2c_write(client,REG_DIR_B,temp);
		sx150x_i2c_read(client,REG_LED_DRIVER_ENABLE_B,&temp);
                temp |= (1<<pinb);
                sx150x_i2c_write(client,REG_LED_DRIVER_ENABLE_B,temp);
		sx150x_i2c_read(client,REG_DATA_B,&temp);
                temp &= ~(1<<pinb);
                sx150x_i2c_write(client,REG_DATA_B,temp);
	}
	else
	{
		uint8_t temp;
                sx150x_i2c_read(client,REG_INPUT_DISABLE_A,&temp);
                temp |= (1<<pin);
                sx150x_i2c_write(client,REG_INPUT_DISABLE_A,temp);
                sx150x_i2c_read(client,REG_PULL_UP_A,&temp);
                temp &= ~(1<<pin);
                sx150x_i2c_write(client,REG_PULL_UP_A,temp);
                sx150x_i2c_read(client,REG_DIR_A,&temp);
                temp &= ~(1<<pin);
                sx150x_i2c_write(client,REG_DIR_A,temp);
                sx150x_i2c_read(client,REG_LED_DRIVER_ENABLE_A,&temp);
                temp |= (1<<pin);
                sx150x_i2c_write(client,REG_LED_DRIVER_ENABLE_A,temp);
                sx150x_i2c_read(client,REG_DATA_A,&temp);
                temp &= ~(1<<pin);
                sx150x_i2c_write(client,REG_DATA_A,temp);
	}	
}
void  analogWrite(struct i2c_client *client,uint8_t pin,uint8_t cycle)
{
	sx150x_i2c_write(client,REG_I_ON[pin], cycle);
}
#endif

static int sx150x_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	static const u32 i2c_funcs = I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_WRITE_WORD_DATA;
	struct sx150x_platform_data *pdata;
	struct sx150x_chip *chip;
	int rc;
	int irq;
	pdata = dev_get_platdata(&client->dev);
	if (!pdata) {
                pdata = of_sx150x_get_platdata(client);
	if (!pdata)
		return -EINVAL;
                else if (IS_ERR(pdata))
                        return PTR_ERR(pdata);
        }

	if (!i2c_check_functionality(client->adapter, i2c_funcs))
		return -ENOSYS;

	chip = devm_kzalloc(&client->dev,
		sizeof(struct sx150x_chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	sx150x_init_chip(chip, client, id->driver_data, pdata);
	rc = sx150x_init_hw(chip, pdata);
	if (rc < 0)
		return rc;

	rc = gpiochip_add(&chip->gpio_chip);
	if (rc)
		return rc;
	printk("pdata->irq_summary=%d\n",pdata->irq_summary);
	if (pdata->irq_summary >= 0) {
		rc = sx150x_install_irq_chip(chip,
					pdata->irq_summary,
					pdata->irq_base);
		if (rc < 0)
			goto probe_fail_post_gpiochip_add;
	}

	i2c_set_clientdata(client, chip);

	gpio_direction_input(504);//pin 8
	irq=gpio_to_irq(504);
	printk("sx1509 irq=%d\n",irq);
	enable_irq(irq);
	printk("###########probe ok\n");
#if 0
	printk("sx150x test for pwm\n");
	init(client);
	for(i=0; i < 16; i++)
	{
		ledDriverInit(client,i);
		analogWrite(client,i,127);
	}
	 printk("sx150xtest for pwm end\n");
#endif
	return 0;
probe_fail_post_gpiochip_add:
	gpiochip_remove(&chip->gpio_chip);
	return rc;
}

static int sx150x_remove(struct i2c_client *client)
{
	struct sx150x_chip *chip;

	chip = i2c_get_clientdata(client);
	gpiochip_remove(&chip->gpio_chip);

	return 0;
}

static struct i2c_driver sx150x_driver = {
	.driver = {
		.name = "sx150x",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(sx150x_of_match),
	},
	.probe    = sx150x_probe,
	.remove   = sx150x_remove,
	.id_table = sx150x_id,
};

static int __init sx150x_init(void)
{
	return i2c_add_driver(&sx150x_driver);
}
subsys_initcall(sx150x_init);

static void __exit sx150x_exit(void)
{
	return i2c_del_driver(&sx150x_driver);
}
module_exit(sx150x_exit);

MODULE_AUTHOR("Gregory Bean <gbean@codeaurora.org>");
MODULE_DESCRIPTION("Driver for Semtech SX150X I2C GPIO Expanders");
MODULE_LICENSE("GPL v2");
