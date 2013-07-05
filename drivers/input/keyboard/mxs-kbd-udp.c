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
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif

#define BUTTON_PRESS_THRESHOLD  3300
#define LRADC_NOISE_MARGIN      200

#define LRADC_DELAY		50

/* this value represents the the lradc value at 3.3V ( 3.3V / 0.000879 V/b ) */
#define TARGET_VDDIO_LRADC_VALUE 3754

/* 3 keypad, 1 headset mic detect */
#define MAX_CH			4

#define MIC_DET_THRESHOLD	60

struct mxskbd_data {
	struct input_dev *input;

	int btn_irq1;

	struct mxskbd_keypair *keycodes;
	int keycodes_offset;
	unsigned int base;
	int chan[MAX_CH];
	int last_button;

	int end_button;

	int jack_last;
	int jack_cnt;
};

static struct mxskbd_data *_devdata;
static int delay1 = 500;
static int delay2 = 200;

static int mxskbd_open(struct input_dev *dev);
static void mxskbd_close(struct input_dev *dev);

#ifdef CONFIG_HAS_WAKELOCK
static struct wake_lock key_wake_lock;
#endif

#ifdef ENABLE_BACKLIGHT_GPIO_CONTROL
static struct timer_list _bl_timer;

static void
_keypad_set_backlight(int is_on)
{
	if (is_on) {
		mxs_key_backlight_gpio_set(1);

		del_timer(&_bl_timer);
		_bl_timer.expires = jiffies + msecs_to_jiffies(5000);
		add_timer(&_bl_timer);
	} else {
		mxs_key_backlight_gpio_set(0);

		del_timer(&_bl_timer);
	}
}

static void
_keypad_bl_timer_handler(unsigned long data)
{
	_keypad_set_backlight(0);
}
#endif

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

	d->last_button = -1;

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

static int mxskbd_decode_button(struct mxskbd_keypair *codes,
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
	return -1; /* invalid key */
}

static unsigned mxskbd_incode_button(struct mxskbd_keypair *codes,
			int kcode)
{
	pr_debug("Incoding %d\n", kcode);
	while (codes->raw != -1) {
		if (kcode == codes->kcode) {
			pr_debug("matches code 0x%x = %d(%d)\n",
				codes->kcode, codes->kcode, codes->raw);
			return codes->raw;
		}
		codes++;
	}
	return (unsigned)-1; /* invalid key */
}

static void jack_process(struct mxskbd_data *d)
{
	int i;

	i = !!mxs_audio_jack_gpio_get();
	if (!!d->jack_last == i) {
		d->jack_cnt = 0;
		return;
	}

	/* mic detect bias */
	if (d->jack_cnt == 0)
		mxs_audio_headset_mic_detect_amp_gpio_set(1);

	int x = __raw_readl(d->base + HW_LRADC_CHn(d->chan[MAX_CH-1])) &
		BM_LRADC_CHn_VALUE;

	if (++d->jack_cnt < 100)
		return;

	/* changed */
	d->jack_cnt = 0;

	input_report_switch(GET_INPUT_DEV(d),
			    SW_HEADPHONE_INSERT, i);

	if (i == 0) {
		if (d->jack_last == SW_MICROPHONE_INSERT)
			input_report_switch(GET_INPUT_DEV(d),
					    SW_MICROPHONE_INSERT, 0);
		d->jack_last = 0;
	} else {
		i = __raw_readl(d->base + HW_LRADC_CHn(d->chan[MAX_CH-1])) &
			BM_LRADC_CHn_VALUE;

		if (i > MIC_DET_THRESHOLD) {	/* mic */
			d->jack_last = SW_MICROPHONE_INSERT;
			input_report_switch(GET_INPUT_DEV(d),
					    SW_MICROPHONE_INSERT, 1);
		} else {
			d->jack_last = SW_HEADPHONE_INSERT;
		}
	}
}

static irqreturn_t mxskbd_irq_handler(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct mxskbd_data *d = platform_get_drvdata(pdev);
	int i, f_key = -1;
	u32 vddio;

	/* end key process */
	i = (__raw_readl(REGS_POWER_BASE + HW_POWER_STS) &
		   BF_POWER_STS_PSWITCH(0x1)) ? KEY_END : -1;

	if (i == KEY_END) {
		if (i == d->last_button)
			goto _end;

		f_key = i;
	}

	/* jack detect */
	jack_process(d);
	if (f_key < 0 && d->jack_last == SW_MICROPHONE_INSERT) {
		i = __raw_readl(d->base + HW_LRADC_CHn(d->chan[MAX_CH-1])) &
			BM_LRADC_CHn_VALUE;
		if (i <= MIC_DET_THRESHOLD) {
			if (d->last_button == KEY_PHONE)
				goto _end;
			f_key = KEY_PHONE;
		}
	}

	/* adc key process */
	vddio = __raw_readl(d->base + HW_LRADC_CHn(VDDIO_VOLTAGE_CH)) &
		BM_LRADC_CHn_VALUE;
	BUG_ON(vddio == 0);

	for (i = 0; i < MAX_CH-1; i++) {
		int raw, norm, key;
		raw = __raw_readl(d->base + HW_LRADC_CHn(d->chan[i])) &
			BM_LRADC_CHn_VALUE;
		norm = (raw * TARGET_VDDIO_LRADC_VALUE) / vddio;

		key = mxskbd_decode_button(d->keycodes +
					   d->keycodes_offset*i, norm);

		if (key >= 0) {
			if (key == d->last_button)
				goto _end;

			if (f_key < 0)
				f_key = key;
		}
	}

	if (d->last_button >= 0) {
		input_report_key(GET_INPUT_DEV(d), d->last_button, 0);
		d->last_button = -1;
	}

	if (f_key >= 0 && f_key != d->last_button) {
		input_report_key(GET_INPUT_DEV(d), f_key, !0);
#ifdef ENABLE_BACKLIGHT_GPIO_CONTROL
		_keypad_set_backlight(1);
#endif
#ifdef CONFIG_HAS_WAKELOCK
		wake_lock_timeout(&key_wake_lock, 5*HZ);
#endif
		d->last_button = f_key;
	}

_end:
	__raw_writel((BM_LRADC_CTRL1_LRADC0_IRQ << d->chan[0]) +
		     (BM_LRADC_CTRL1_LRADC0_IRQ << d->chan[1]) +
		     (BM_LRADC_CTRL1_LRADC0_IRQ << d->chan[2]) +
		     (BM_LRADC_CTRL1_LRADC0_IRQ << d->chan[3]) +
		     (BM_LRADC_CTRL1_LRADC0_IRQ << VDDIO_VOLTAGE_CH),
		     d->base + HW_LRADC_CTRL1_CLR);
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
	int i;
	int mask = 0;
	struct mxskbd_data *d = platform_get_drvdata(pdev);

	for (i = 0; i < MAX_CH; i++) {
		hw_lradc_init_ladder(d->chan[i],
				     LRADC_DELAY_TRIGGER_BUTTON, LRADC_DELAY);
		mask |= BM_LRADC_CTRL1_LRADC0_IRQ << d->chan[i];
	}
	__raw_writel(mask, d->base + HW_LRADC_CTRL1_CLR);

	/* set vddio lradc manually */
	hw_lradc_configure_channel(VDDIO_VOLTAGE_CH, 0, 0, 0); /* no dev2 */
	hw_lradc_set_delay_trigger(LRADC_DELAY_TRIGGER_BUTTON,
				   1 << VDDIO_VOLTAGE_CH,
				   1 << LRADC_DELAY_TRIGGER_BUTTON,
				   0, LRADC_DELAY);

	/* used only one interrupt */
	__raw_writel(BM_LRADC_CTRL1_LRADC0_IRQ_EN << d->chan[0],
		     d->base + HW_LRADC_CTRL1_SET);


	hw_lradc_set_delay_trigger_kick(LRADC_DELAY_TRIGGER_BUTTON, !0);
}

#ifdef CONFIG_PM
static int mxskbd_suspend(struct platform_device *pdev, pm_message_t state)
{
	int i;
	struct mxskbd_data *d = platform_get_drvdata(pdev);

	for (i = 0; i < MAX_CH; i++) {
		hw_lradc_stop_ladder(d->chan[i], LRADC_DELAY_TRIGGER_BUTTON);
		hw_lradc_unuse_channel(d->chan[i]);
	}
	hw_lradc_set_delay_trigger_kick(LRADC_DELAY_TRIGGER_BUTTON, 0);
	__raw_writel(BM_LRADC_CTRL1_LRADC0_IRQ_EN << d->chan[0],
		     d->base + HW_LRADC_CTRL1_CLR);
	mxskbd_close(d->input);

#ifdef ENABLE_BACKLIGHT_GPIO_CONTROL
	_keypad_set_backlight(0);
#endif

	return 0;
}

static int mxskbd_resume(struct platform_device *pdev)
{
	int i;
	struct mxskbd_data *d = platform_get_drvdata(pdev);

	__raw_writel(BM_LRADC_CTRL1_LRADC0_IRQ_EN << d->chan[i],
		     d->base + HW_LRADC_CTRL1_SET);
	mxskbd_open(d->input);
	for (i = 0; i < MAX_CH; i++)
		hw_lradc_use_channel(d->chan[i]);
	mxskbd_hwinit(pdev);
	return 0;
}
#endif

static int __devinit mxskbd_probe(struct platform_device *pdev)
{
	int i;
	int err = 0;
	struct resource *res;
	struct mxskbd_data *d;
	struct mxs_kbd_plat_data *plat_data;

	plat_data = (struct mxs_kbd_plat_data *)pdev->dev.platform_data;
	if (plat_data == NULL)
		return -ENODEV;

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&key_wake_lock, WAKE_LOCK_SUSPEND, "mxs-keypad");
#endif

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
	for (i = 0; i < MAX_CH; i++)
		d->chan[i] = plat_data->channel[i];
	d->btn_irq1 = platform_get_irq(pdev, 0);

	platform_set_drvdata(pdev, d);

	if (d->btn_irq1 > 0) {
		err = request_irq(d->btn_irq1, mxskbd_irq_handler,
				IRQF_DISABLED, pdev->name, pdev);
		if (err) {
			dev_err(&pdev->dev, "Cannot request keypad IRQ\n");
			goto err_free_dev;
		}
	}

	/* Register the input device */
	err = input_register_device(GET_INPUT_DEV(d));
	if (err)
		goto err_free_dev;

	/* these two have to be set after registering the input device */
	d->input->rep[REP_DELAY] = delay1;
	d->input->rep[REP_PERIOD] = delay2;

	for (i = 0; i < MAX_CH; i++)
		hw_lradc_use_channel(d->chan[i]);
	mxskbd_hwinit(pdev);

#ifdef ENABLE_BACKLIGHT_GPIO_CONTROL
	init_timer(&_bl_timer);
	_bl_timer.function = _keypad_bl_timer_handler;
#endif

	return 0;

err_free_dev:
	if (d->btn_irq1 > 0)
		free_irq(d->btn_irq1, pdev);

	mxskbd_data_free(d);
err_out:
#ifdef CONFIG_HAS_WAKELOCK
		wake_lock_destroy(&key_wake_lock);
#endif
	return err;
}

static int __devexit mxskbd_remove(struct platform_device *pdev)
{
	int i;
	struct mxskbd_data *d = platform_get_drvdata(pdev);

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_destroy(&key_wake_lock);
#endif

	for (i = 0; i < MAX_CH; i++)
		hw_lradc_unuse_channel(d->chan[i]);

	input_unregister_device(GET_INPUT_DEV(d));
	if (d->btn_irq1 > 0)
		free_irq(d->btn_irq1, pdev);
	mxskbd_data_free(d);

	platform_set_drvdata(pdev, NULL);

#ifdef ENABLE_BACKLIGHT_GPIO_CONTROL
	_keypad_set_backlight(0);
#endif

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
