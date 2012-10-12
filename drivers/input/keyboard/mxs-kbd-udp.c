/*
 * Keypad ladder driver for Freescale MXS boards
 *
 * Author: dmitry pervushin <dimka@embeddedalley.com>
 *
 * Copyright 2008-2010 Freescale Semiconductor, Inc.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/pm.h>

#include <mach/device.h>
#include <mach/hardware.h>
#include <mach/regs-lradc.h>
#include <mach/lradc.h>
#include <mach/regs-power.h>

#define BUTTON_PRESS_THRESHOLD  3300
#define LRADC_NOISE_MARGIN      200

/* this value represents the the lradc value at 3.3V ( 3.3V / 0.000879 V/b ) */
#define TARGET_VDDIO_LRADC_VALUE 3754

struct mxskbd_data {
	struct input_dev *input;

	int btn_irq1;
	int btn_irq2;
	int btn_irq3;

	struct mxskbd_keypair *keycodes;
	int keycodes_offset;
	unsigned int base;
	int chan1;
	int chan1_last_button;
	int chan2;
	int chan2_last_button;
	int chan3;
	int chan3_last_button;

	int end_button;
};

static struct mxskbd_data *_devdata;
static struct timer_list _bl_timer;
static int delay1 = 500;
static int delay2 = 200;

static int mxskbd_open(struct input_dev *dev);
static void mxskbd_close(struct input_dev *dev);

static void
_keypad_set_backlight(int is_on)
{
	mxs_keypad_gpio_init();

	if (is_on) {
		mxs_keypad_gpio_set(1);

		del_timer(&_bl_timer);
		_bl_timer.expires = jiffies + msecs_to_jiffies(5000);
		add_timer(&_bl_timer);
	} else {
		mxs_keypad_gpio_set(0);

		del_timer(&_bl_timer);
	}
}

static void
_keypad_bl_timer_handler(unsigned long data)
{
	_keypad_set_backlight(0);
}

static void
_keypad_set_pm_power_off(void)
{
	__raw_writel((0x3e77 << 16) | 1, REGS_POWER_BASE + HW_POWER_RESET);
}

static struct mxskbd_data *mxskbd_data_alloc(struct platform_device *pdev,
		struct mxskbd_keypair *keys, int keys_offset)
{
	struct mxskbd_data *d = kzalloc(sizeof(*d), GFP_KERNEL);

	if (!d)
		return NULL;

	if (!keys) {
		dev_err(&pdev->dev,
			"No keycodes in platform_data, bailing out.\n");
		kfree(d);
		return NULL;
	}
	d->keycodes = keys;
	d->keycodes_offset = keys_offset;

	d->input = input_allocate_device();
	if (!d->input) {
		kfree(d);
		return NULL;
	}

	d->input->phys = "onboard";
	d->input->uniq = "0000'0000";
	d->input->name = pdev->name;
	d->input->id.bustype = BUS_HOST;
	d->input->open = mxskbd_open;
	d->input->close = mxskbd_close;
	d->input->dev.parent = &pdev->dev;

	set_bit(EV_KEY, d->input->evbit);
	set_bit(EV_REL, d->input->evbit);
	set_bit(EV_REP, d->input->evbit);

	d->chan1_last_button = -1;
	d->chan2_last_button = -1;
	d->chan3_last_button = -1;

	d->end_button = 0;

	while (keys->raw >= 0) {
		set_bit(keys->kcode, d->input->keybit);
		keys++;
	}

	set_bit(KEY_END, d->input->keybit);

	return d;
}

static inline struct input_dev *GET_INPUT_DEV(struct mxskbd_data *d)
{
	BUG_ON(!d);
	return d->input;
}

static void mxskbd_data_free(struct mxskbd_data *d)
{
	if (!d)
		return;
	if (d->input)
		input_free_device(d->input);
	kfree(d);
}

static unsigned mxskbd_decode_button(struct mxskbd_keypair *codes,
			int raw_button)

{
	pr_debug("Decoding %d\n", raw_button);
	while (codes->raw != -1) {
		if ((raw_button >= (codes->raw - LRADC_NOISE_MARGIN)) &&
		    (raw_button < (codes->raw + LRADC_NOISE_MARGIN))) {
			pr_debug("matches code 0x%x = %d\n",
				codes->kcode, codes->kcode);
			return codes->kcode;
		}
		codes++;
	}
	return (unsigned)-1; /* invalid key */
}

static irqreturn_t mxskbd_irq_handler(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct mxskbd_data *devdata = platform_get_drvdata(pdev);
	u16 raw_button, normalized_button, vddio;
	unsigned btn;
	int last_button = -1;
	int keycodes_offset = 0;
	int pin_value = 0;

	if (devdata->btn_irq1 == irq) {
		raw_button = __raw_readl(devdata->base +
				HW_LRADC_CHn(devdata->chan1)) &
			BM_LRADC_CHn_VALUE;
		last_button = devdata->chan1_last_button;
		keycodes_offset = 0;
	} else if (devdata->btn_irq2 == irq) {
		raw_button = __raw_readl(devdata->base +
				HW_LRADC_CHn(devdata->chan2)) &
			BM_LRADC_CHn_VALUE;
		last_button = devdata->chan2_last_button;
		keycodes_offset = devdata->keycodes_offset;
	} else if (devdata->btn_irq3 == irq) {
		raw_button = __raw_readl(devdata->base +
				HW_LRADC_CHn(devdata->chan3)) &
			BM_LRADC_CHn_VALUE;
		last_button = devdata->chan3_last_button;
		keycodes_offset = devdata->keycodes_offset * 2;
	}

	vddio = hw_lradc_vddio();
	BUG_ON(vddio == 0);

	normalized_button = (raw_button * TARGET_VDDIO_LRADC_VALUE) / vddio;

	if (last_button < 0 &&
			raw_button < BUTTON_PRESS_THRESHOLD) {
		btn = mxskbd_decode_button(devdata->keycodes + keycodes_offset,
				raw_button);
		if (btn < KEY_MAX) {
			last_button = btn;
			input_report_key(GET_INPUT_DEV(devdata),
					 last_button, !0);
			_keypad_set_backlight(1);
		} else
			dev_err(&pdev->dev, "Invalid button: raw = %d, "
				"normalized = %d, vddio = %d\n",
				raw_button, normalized_button, vddio);
	} else if (last_button > 0 &&
			raw_button >= BUTTON_PRESS_THRESHOLD) {
		input_report_key(GET_INPUT_DEV(devdata),
				 last_button, 0);
		last_button = -1;
	} else {
		/* END key process */
		if (devdata->btn_irq1 == irq &&
				 devdata->chan2_last_button < 0 &&
				 devdata->chan3_last_button < 0) {
			pin_value = __raw_readl(REGS_POWER_BASE +
					HW_POWER_STS) &
				BF_POWER_STS_PSWITCH(0x1);
			if (pin_value) {
				if (!devdata->end_button) {
					input_report_key(
							GET_INPUT_DEV(_devdata),
							KEY_END, !0);
					_keypad_set_backlight(1);
				}

				if (devdata->end_button++ > 400)
					_keypad_set_pm_power_off();
			} else if (devdata->end_button) {
				input_report_key(
						GET_INPUT_DEV(_devdata),
						KEY_END, 0);
				devdata->end_button = 0;
			}
		}
	}

	if (devdata->btn_irq1 == irq) {
		__raw_writel(BM_LRADC_CTRL1_LRADC0_IRQ << devdata->chan1,
				devdata->base + HW_LRADC_CTRL1_CLR);
		 devdata->chan1_last_button = last_button;
	} else if (devdata->btn_irq2 == irq) {
		__raw_writel(BM_LRADC_CTRL1_LRADC0_IRQ << devdata->chan2,
				devdata->base + HW_LRADC_CTRL1_CLR);
		 devdata->chan2_last_button = last_button;
	} else if (devdata->btn_irq3 == irq) {
		__raw_writel(BM_LRADC_CTRL1_LRADC0_IRQ << devdata->chan3,
				devdata->base + HW_LRADC_CTRL1_CLR);
		 devdata->chan3_last_button = last_button;
	}
	return IRQ_HANDLED;
}

static int mxskbd_open(struct input_dev *dev)
{
	/* enable clock */
	return 0;
}

static void mxskbd_close(struct input_dev *dev)
{
	/* disable clock */
}

static void mxskbd_hwinit(struct platform_device *pdev)
{
	struct mxskbd_data *d = platform_get_drvdata(pdev);

	hw_lradc_init_ladder(d->chan1, LRADC_DELAY_TRIGGER_BUTTON, 50);
	hw_lradc_init_ladder(d->chan2, LRADC_DELAY_TRIGGER_BUTTON, 50);
	hw_lradc_init_ladder(d->chan3, LRADC_DELAY_TRIGGER_BUTTON, 50);
	__raw_writel(BM_LRADC_CTRL1_LRADC0_IRQ << d->chan1,
			d->base + HW_LRADC_CTRL1_CLR);
	__raw_writel(BM_LRADC_CTRL1_LRADC0_IRQ_EN << d->chan1,
			d->base + HW_LRADC_CTRL1_SET);
	__raw_writel(BM_LRADC_CTRL1_LRADC0_IRQ << d->chan2,
			d->base + HW_LRADC_CTRL1_CLR);
	__raw_writel(BM_LRADC_CTRL1_LRADC0_IRQ_EN << d->chan2,
			d->base + HW_LRADC_CTRL1_SET);
	__raw_writel(BM_LRADC_CTRL1_LRADC0_IRQ << d->chan3,
			d->base + HW_LRADC_CTRL1_CLR);
	__raw_writel(BM_LRADC_CTRL1_LRADC0_IRQ_EN << d->chan3,
			d->base + HW_LRADC_CTRL1_SET);
	hw_lradc_set_delay_trigger_kick(LRADC_DELAY_TRIGGER_BUTTON, !0);
}

#ifdef CONFIG_PM
static int mxskbd_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct mxskbd_data *d = platform_get_drvdata(pdev);

	hw_lradc_stop_ladder(d->chan1, LRADC_DELAY_TRIGGER_BUTTON);
	hw_lradc_stop_ladder(d->chan2, LRADC_DELAY_TRIGGER_BUTTON);
	hw_lradc_stop_ladder(d->chan3, LRADC_DELAY_TRIGGER_BUTTON);
	hw_lradc_set_delay_trigger_kick(LRADC_DELAY_TRIGGER_BUTTON, 0);
	hw_lradc_unuse_channel(d->chan1);
	hw_lradc_unuse_channel(d->chan2);
	hw_lradc_unuse_channel(d->chan3);
	__raw_writel(BM_LRADC_CTRL1_LRADC0_IRQ_EN << d->chan1,
		     d->base + HW_LRADC_CTRL1_CLR);
	__raw_writel(BM_LRADC_CTRL1_LRADC0_IRQ_EN << d->chan2,
		     d->base + HW_LRADC_CTRL1_CLR);
	__raw_writel(BM_LRADC_CTRL1_LRADC0_IRQ_EN << d->chan3,
		     d->base + HW_LRADC_CTRL1_CLR);
	mxskbd_close(d->input);

	_keypad_set_backlight(0);

	return 0;
}

static int mxskbd_resume(struct platform_device *pdev)
{
	struct mxskbd_data *d = platform_get_drvdata(pdev);

	__raw_writel(BM_LRADC_CTRL1_LRADC0_IRQ_EN << d->chan1,
		     d->base + HW_LRADC_CTRL1_SET);
	__raw_writel(BM_LRADC_CTRL1_LRADC0_IRQ_EN << d->chan2,
		     d->base + HW_LRADC_CTRL1_SET);
	__raw_writel(BM_LRADC_CTRL1_LRADC0_IRQ_EN << d->chan3,
		     d->base + HW_LRADC_CTRL1_SET);
	mxskbd_open(d->input);
	hw_lradc_use_channel(d->chan1);
	hw_lradc_use_channel(d->chan2);
	hw_lradc_use_channel(d->chan3);
	mxskbd_hwinit(pdev);
	return 0;
}
#endif

static int __devinit mxskbd_probe(struct platform_device *pdev)
{
	int err = 0;
	struct resource *res;
	struct mxskbd_data *d;
	struct mxs_kbd_plat_data *plat_data;

	plat_data = (struct mxs_kbd_plat_data *)pdev->dev.platform_data;
	if (plat_data == NULL)
		return -ENODEV;

	/* Create and register the input driver. */
	d = mxskbd_data_alloc(pdev, plat_data->keypair,
			plat_data->keypair_offset);
	if (!d) {
		dev_err(&pdev->dev, "Cannot allocate driver structures\n");
		err = -ENOMEM;
		goto err_out;
	}

	_devdata = d;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		err = -ENODEV;
		goto err_out;
	}
	d->base = (unsigned int)IO_ADDRESS(res->start);
	d->chan1 = plat_data->channel1;
	d->chan2 = plat_data->channel2;
	d->chan3 = plat_data->channel3;
	d->btn_irq1 = platform_get_irq(pdev, 0);
	d->btn_irq2 = platform_get_irq(pdev, 1);
	d->btn_irq3 = platform_get_irq(pdev, 2);

	platform_set_drvdata(pdev, d);

	if (d->btn_irq1 > 0) {
		err = request_irq(d->btn_irq1, mxskbd_irq_handler,
				IRQF_DISABLED, pdev->name, pdev);
		if (err) {
			dev_err(&pdev->dev, "Cannot request keypad IRQ\n");
			goto err_free_dev;
		}
	}

	if (d->btn_irq2 > 0) {
		err = request_irq(d->btn_irq2, mxskbd_irq_handler,
			  IRQF_DISABLED, pdev->name, pdev);
		if (err) {
			dev_err(&pdev->dev,
				"Cannot request keybad detect IRQ\n");
			goto err_free_irq1;
		}
	}

	if (d->btn_irq3 > 0) {
		err = request_irq(d->btn_irq3, mxskbd_irq_handler,
			  IRQF_DISABLED, pdev->name, pdev);
		if (err) {
			dev_err(&pdev->dev,
				"Cannot request keybad detect IRQ\n");
			goto err_free_irq2;
		}
	}

	/* Register the input device */
	err = input_register_device(GET_INPUT_DEV(d));
	if (err)
		goto err_free_irq3;

	/* these two have to be set after registering the input device */
	d->input->rep[REP_DELAY] = delay1;
	d->input->rep[REP_PERIOD] = delay2;

	hw_lradc_use_channel(d->chan1);
	hw_lradc_use_channel(d->chan2);
	hw_lradc_use_channel(d->chan3);
	mxskbd_hwinit(pdev);

	init_timer(&_bl_timer);
	_bl_timer.function = _keypad_bl_timer_handler;

	return 0;

err_free_irq3:
	platform_set_drvdata(pdev, NULL);
	if (d->btn_irq3 > 0)
		free_irq(d->btn_irq3, pdev);
err_free_irq2:
	if (d->btn_irq2 > 0)
		free_irq(d->btn_irq2, pdev);
err_free_irq1:
	if (d->btn_irq1 > 0)
		free_irq(d->btn_irq1, pdev);
err_free_dev:
	mxskbd_data_free(d);
err_out:
	return err;
}

static int __devexit mxskbd_remove(struct platform_device *pdev)
{
	struct mxskbd_data *d = platform_get_drvdata(pdev);

	hw_lradc_unuse_channel(d->chan1);
	hw_lradc_unuse_channel(d->chan2);
	hw_lradc_unuse_channel(d->chan3);
	input_unregister_device(GET_INPUT_DEV(d));
	if (d->btn_irq1 > 0)
		free_irq(d->btn_irq1, pdev);
	if (d->btn_irq2 > 0)
		free_irq(d->btn_irq2, pdev);
	if (d->btn_irq3 > 0)
		free_irq(d->btn_irq3, pdev);
	mxskbd_data_free(d);

	platform_set_drvdata(pdev, NULL);

	_keypad_set_backlight(0);

	return 0;
}

static struct platform_driver mxskbd_driver = {
	.probe		= mxskbd_probe,
	.remove		= __devexit_p(mxskbd_remove),
#ifdef CONFIG_PM
	.suspend	= mxskbd_suspend,
	.resume		= mxskbd_resume,
#endif
	.driver		= {
		.name	= "mxs-kbd",
	},
};

static int __init mxskbd_init(void)
{
	return platform_driver_register(&mxskbd_driver);
}

static void __exit mxskbd_exit(void)
{
	platform_driver_unregister(&mxskbd_driver);
}

module_init(mxskbd_init);
module_exit(mxskbd_exit);
MODULE_DESCRIPTION("Freescale keyboard driver for mxs family");
MODULE_AUTHOR("dmitry pervushin <dimka@embeddedalley.com>")
MODULE_LICENSE("GPL");
