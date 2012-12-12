/*-----------------------------------------------------------------------------
 * FILE NAME : canopus_jack.c
 *
 * PURPOSE :
 *
 * Copyright 2012 WINNERTEC ENG
 * All right reserved.
 *
 * This software is confidential and proprietary to UniData
 * Communication Systems, Inc. No Part of this software may
 * be reproduced, stored, transmitted, disclosed or used in any
 * form or by any means other than as expressly provide by the
 * written license agreement between UniData Communication
 * Systems and its licensee.
 *
 * NOTES:
 *
 * REQUIREMENTS/FUNCTIONAL SPECIFICATIONS REFERENCES:
 *
 * ALGORITHM (PDL):
 *
 *---------------------------------------------------------------------------*/

/*_____________________ Include Header ______________________________________*/

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/poll.h>

#include <mach/device.h>

/*_____________________ Constants Definitions _______________________________*/

/*_____________________ Type definitions ____________________________________*/

/*_____________________ Imported Variables __________________________________*/

/*_____________________ Variables Definitions _______________________________*/

static void jack_irq_work(struct work_struct *work);

static struct platform_device *_pdev;

static const char jack_name[] = "canopus-jack";
static const char jack_mini_name[] = "canopus_jack";

static DECLARE_DELAYED_WORK(work, jack_irq_work);
static DECLARE_WAIT_QUEUE_HEAD(jack_wait);
static DEFINE_SPINLOCK(jack_lock);

static int last_state;
static int update;

/*_____________________ Local Declarations __________________________________*/

/*_____________________ internal functions __________________________________*/

static ssize_t jack_read(struct file *f, char __user *d, size_t s, loff_t *o)
{
	ssize_t ret;
	unsigned int data;

	if (s < sizeof(unsigned int))
		return -EINVAL;

	spin_lock(&jack_lock);
	last_state = data = mxs_audio_jack_gpio_get() ? 1 : 0;
	update = 0;

	spin_unlock(&jack_lock);

	ret = put_user(data, (unsigned int __user *)d);
	if (ret == 0)
		ret = sizeof(unsigned int);
	return ret;
}

static unsigned int jack_poll(struct file *f, struct poll_table_struct *p)
{
	unsigned int data;

	poll_wait(f, &jack_wait, p);

	spin_lock(&jack_lock);
	data = mxs_audio_jack_gpio_get() ? 1 : 0;
	if (last_state != data)
		update = 1;
	spin_unlock(&jack_lock);

	return (update) ? POLLIN | POLLWRNORM : 0;
}

static const struct file_operations fops = {
	.owner		= THIS_MODULE,
	.read		= jack_read,
	.poll		= jack_poll
};

static struct miscdevice miscdev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= jack_mini_name,
	.fops		= &fops
};

static void jack_irq_work(struct work_struct *work)
{
	wake_up_interruptible(&jack_wait);
}

static irqreturn_t jack_irq_handler(int irq, void *data)
{
	cancel_delayed_work(&work);
	schedule_delayed_work(&work, msecs_to_jiffies(50));
	return IRQ_HANDLED;
}

static int __init canopus_jack_probe(struct platform_device *pdev)
{
	int ret;
	int irq = mxs_audio_jack_gpio_irq();

	ret = request_irq(irq, jack_irq_handler,
			IRQF_DISABLED|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,
			"canopus-jack", 0);
	ret = misc_register(&miscdev);
	return ret;
}

static int canopus_jack_remove(struct platform_device *pdev)
{
	int irq = mxs_audio_jack_gpio_irq();

	misc_deregister(&miscdev);
	free_irq(irq, 0);
	return 0;
}

static struct platform_driver canopus_jack_driver = {
       .probe          = canopus_jack_probe,
       .remove         = canopus_jack_remove,
       .driver		= {
		.owner	= THIS_MODULE,
		.name	= jack_name,
	},
};

/*_____________________ Program Body ________________________________________*/

int __init canopus_jack_init(void)
{
	int rc;

	_pdev = platform_device_alloc(jack_name, 0);
	if (!_pdev)
		return -ENOMEM;

	rc = platform_device_add(_pdev);
	if (rc)
		goto undo_malloc;

	return platform_driver_register(&canopus_jack_driver);

undo_malloc:
	platform_device_put(_pdev);
	_pdev = 0;
	return -1;
}

void __exit canopus_jack_exit(void)
{
	platform_driver_unregister(&canopus_jack_driver);
}

module_init(canopus_jack_init);
module_exit(canopus_jack_exit);

MODULE_AUTHOR("Jong-Rak Kim <jrkim@winnertec.co.kr>");
MODULE_DESCRIPTION("canopus jack status driver");
MODULE_LICENSE("GPL");
