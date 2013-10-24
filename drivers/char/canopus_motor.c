/*-----------------------------------------------------------------------------
 * FILE NAME : canopus_motor.c
 *
 * PURPOSE :
 *
 * Copyright 2013 INCOM
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

/*_____________________ Include Header ______________________________________*/

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <mach/device.h>

/*_____________________ Constants Definitions _______________________________*/

#define _MOTOR_SET	_IOW('M', 0, int)

/*_____________________ Type definitions ____________________________________*/

/*_____________________ Imported Variables __________________________________*/

/*_____________________ Variables Definitions _______________________________*/

static struct platform_device *_pdev;

static const char motor_name[] = "canopus-motor";
static const char motor_mini_name[] = "canopus_motor";

/*_____________________ Local Declarations __________________________________*/

/*_____________________ internal functions __________________________________*/

static int motor_ioctl(struct inode *node, struct file *f, unsigned int cmd,
		unsigned long arg)
{
	int ret = -EINVAL;
	int val = 0;

	if (_IOC_TYPE(cmd) != 'M')
		return -ENOTTY;

	switch (cmd) {
	case _MOTOR_SET:
		ret = get_user(val, (int __user *)arg);
		if (ret)
			return -EFAULT;

		mxs_motor_gpio_set(val);
		break;
	default:
		break;
	}

	return ret;
}

static const struct file_operations fops = {
	.owner		= THIS_MODULE,
	.ioctl		= motor_ioctl,
};

static struct miscdevice miscdev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= motor_mini_name,
	.fops		= &fops
};

static int __init canopus_motor_probe(struct platform_device *pdev)
{
	int ret = misc_register(&miscdev);

	return ret;
}

static int canopus_motor_remove(struct platform_device *pdev)
{
	misc_deregister(&miscdev);

	return 0;
}

static struct platform_driver canopus_motor_driver = {
       .probe          = canopus_motor_probe,
       .remove         = canopus_motor_remove,
       .driver		= {
		.owner	= THIS_MODULE,
		.name	= motor_name,
	},
};

/*_____________________ Program Body ________________________________________*/

int __init canopus_motor_init(void)
{
	int rc;

	_pdev = platform_device_alloc(motor_name, 0);
	if (!_pdev)
		return -ENOMEM;

	rc = platform_device_add(_pdev);
	if (rc)
		goto undo_malloc;

	return platform_driver_register(&canopus_motor_driver);

undo_malloc:
	platform_device_put(_pdev);
	_pdev = 0;
	return -1;
}

void __exit canopus_motor_exit(void)
{
	platform_driver_unregister(&canopus_motor_driver);
}

module_init(canopus_motor_init);
module_exit(canopus_motor_exit);

MODULE_AUTHOR("Jong-Rak Kim <jrkim@incominc.com>");
MODULE_DESCRIPTION("canopus motor driver");
MODULE_LICENSE("GPL");
