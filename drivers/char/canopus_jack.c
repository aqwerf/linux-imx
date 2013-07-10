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
#include <linux/delay.h>

#include <mach/device.h>

/*_____________________ Constants Definitions _______________________________*/

#define MXS_JACK_PATH		_IOW('P', 0, int)

/*_____________________ Type definitions ____________________________________*/

enum {
	_JACK_PATH_FLOAT = -1,
	_JACK_PATH_SPEAKER = 0,
	_JACK_PATH_HEADSET,
	_JACK_PATH_HANDSET,
	_JACK_PATH_BOTH,
};

/*_____________________ Imported Variables __________________________________*/

/*_____________________ Variables Definitions _______________________________*/

static struct platform_device *_pdev;

static const char jack_name[] = "canopus-jack";
static const char jack_mini_name[] = "canopus_jack";

/*_____________________ Local Declarations __________________________________*/

/*_____________________ internal functions __________________________________*/

static int jack_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	int path = 0;
	int val = 0;

	if (_IOC_TYPE(cmd) != 'P')
		return -ENOTTY;

	if (cmd == MXS_JACK_PATH) {
		if (get_user(path, (int __user *)arg))
			return -EFAULT;

		switch (path) {
		case _JACK_PATH_HANDSET:
			mxs_audio_receiver_amp_gpio_set(1);
			break;
		case _JACK_PATH_HEADSET:
			mxs_audio_receiver_amp_gpio_set(0);
			break;
		case _JACK_PATH_BOTH:
		case _JACK_PATH_SPEAKER:
		case _JACK_PATH_FLOAT:
		default:
			mxs_audio_receiver_amp_gpio_set(0);
			break;
		}

		return 0;
	}

	return -ENOIOCTLCMD;
}

static const struct file_operations fops = {
	.owner		= THIS_MODULE,
	.ioctl		= jack_ioctl
};

static struct miscdevice miscdev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= jack_mini_name,
	.fops		= &fops
};

static int __init canopus_jack_probe(struct platform_device *pdev)
{
	int ret;
	ret = misc_register(&miscdev);
	return ret;
}

static int canopus_jack_remove(struct platform_device *pdev)
{
	misc_deregister(&miscdev);
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
