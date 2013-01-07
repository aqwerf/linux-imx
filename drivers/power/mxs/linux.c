/*
 * Linux glue to MXS battery state machine.
 *
 * Author: Steve Longerbeam <stevel@embeddedalley.com>
 *
 * Copyright (C) 2008 EmbeddedAlley Solutions Inc.
 * Copyright (C) 2008-2012 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/jiffies.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/clk.h>
#include <mach/ddi_bc.h>
#include "ddi_bc_internal.h"
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <mach/regulator.h>
#include <mach/regs-power.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/clock.h>
#include <mach/regs-rtc.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <asm/fiq.h>
#include <linux/uaccess.h>
#include <linux/circ_buf.h>
#include <mach/device.h>
#include "ddi_bc_internal.h"
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>

static struct wake_lock _charger_wake_lock;
#endif

enum application_5v_status{
	_5v_connected_verified,
	_5v_connected_unverified,
	_5v_disconnected_unverified,
	_5v_disconnected_verified,
};

struct mxs_info {
	struct device *dev;
	struct regulator *regulator;
	struct regulator *onboard_vbus5v;

	struct power_supply bat;
	struct power_supply ac;
	struct power_supply usb;

	ddi_bc_Cfg_t *sm_cfg;
	struct mutex sm_lock;
	struct timer_list sm_timer;
	struct work_struct sm_work;
	struct resource *irq_vdd5v;
	struct resource *irq_dcdc4p2_bo;
	struct resource *irq_batt_brnout;
	struct resource *irq_vddd_brnout;
	struct resource *irq_vdda_brnout;
	struct resource *irq_vddio_brnout;
	struct resource *irq_vdd5v_droop;
	int is_ac_online;
	int source_protection_mode;
	uint32_t sm_new_5v_connection_jiffies;
	uint32_t sm_new_5v_disconnection_jiffies;
	enum application_5v_status sm_5v_connection_status;




#define USB_ONLINE      0x01
#define USB_REG_SET     0x02
#define USB_SM_RESTART  0x04
#define USB_SHUTDOWN    0x08
#define USB_N_SEND      0x10
	int is_usb_online;
	int onboard_vbus5v_online;
};

#define to_mxs_info(x) container_of((x), struct mxs_info, bat)

#ifndef NON_USB_5V_SUPPLY_CURRENT_LIMIT_MA
#define NON_USB_5V_SUPPLY_CURRENT_LIMIT_MA 780
#endif

#ifndef POWERED_USB_5V_CURRENT_LIMIT_MA
#define POWERED_USB_5V_CURRENT_LIMIT_MA 450
#endif

#ifndef UNPOWERED_USB_5V_CURRENT_LIMIT_MA
#define UNPOWERED_USB_5V_CURRENT_LIMIT_MA 80
#endif

#ifndef _5V_DEBOUNCE_TIME_MS
#define _5V_DEBOUNCE_TIME_MS 500
#endif

#ifndef OS_SHUTDOWN_BATTERY_VOLTAGE_THRESHOLD_MV
#ifdef CONFIG_MACH_MX23_CANOPUS
#define OS_SHUTDOWN_BATTERY_VOLTAGE_THRESHOLD_MV 3400
#else
#define OS_SHUTDOWN_BATTERY_VOLTAGE_THRESHOLD_MV 3350
#endif
#endif

#ifdef CONFIG_ARCH_MX23
#define IRQ_DCDC4P2_BRNOUT IRQ_DCDC4P2_BO
#endif

#define  POWER_FIQ

/* #define DEBUG_IRQS */

/* There is no direct way to detect wall power presence, so assume the AC
 * power source is valid if 5V presents and USB device is disconnected.
 * If USB device is connected then assume that AC is offline and USB power
 * is online.
 */


#define is_ac_online()	\
		(ddi_power_Get5vPresentFlag() ? (!fsl_is_usb_plugged()) : 0)
#define is_usb_online()	\
		(ddi_power_Get5vPresentFlag() ? (!!fsl_is_usb_plugged()) : 0)

#ifdef CONFIG_MACH_MX23_CANOPUS
#define MXS_DEV_NAME			"mxs_bat"

#define MXS_CIRC_BUF_MAX		16
#define MXS_CIRC_ADD(elem, cir_buf, size) \
	{ down(&mxs_event_mutex); \
	if (CIRC_SPACE(cir_buf.head, cir_buf.tail, size)) { \
		cir_buf.buf[cir_buf.head] = (char)elem; \
		cir_buf.head = (cir_buf.head + 1) & (size - 1); \
	} else { \
		pr_debug("Failed to notify event to the user\n"); \
	} \
	up(&mxs_event_mutex); }

#define MXS_CIRC_REMOVE(elem, cir_buf, size) \
	{ down(&mxs_event_mutex); \
	if (CIRC_CNT(cir_buf.head, cir_buf.tail, size)) { \
		elem = (int)cir_buf.buf[cir_buf.tail]; \
		cir_buf.tail = (cir_buf.tail + 1) & (size - 1); \
	} else { \
		elem = -1; \
		pr_debug("No valid notified event\n"); \
	} \
	up(&mxs_event_mutex); }

#define MXS_BATTERY_VOLTAGE		_IOR('P', 0xa4, int)
#define MXS_AC_ONLINE			_IOR('P', 0xa5, int)
#define MXS_USB_ONLINE			_IOR('P', 0xa6, int)
#define MXS_BAT_NOTIFY_USER		_IOR('P', 0xa8, int)
#define MXS_BAT_GET_NOTIFY		_IOR('P', 0xa9, int)
#define MXS_BAT_LED_CONTROL		_IOWR('P', 0xab, int)

enum {
	MXS_EVENT_CHARGING = 0,
	MXS_EVENT_BATTERY,
	MXS_EVENT_FULL_CHARGED,
	MXS_EVENT_FAULT,
	MXS_EVENT_NUM,
};

enum {
	MXS_LED_GREEN = 0,
	MXS_LED_RED,
	MXS_LED_ALL_ON,
	MXS_LED_ALL_OFF,
	MXS_LED_NUM,
};

struct mxs_bat_event_cb {
	void (*func) (void *);
	void *param;
};

struct mxs_bat_event_cb_list {
	/* keeps a list of subsecibed clients to an event */
	struct list_head list;
	/* Callback function with parameter, called whend event occurs */
	struct mxs_bat_event_cb callback;
};

struct mxs_led_data {
	unsigned int command;
	unsigned int control;
};

static int mxs_bat_major;
static struct class *mxs_dev_class;
static struct fasync_struct *mxs_bat_queue;
static struct circ_buf mxs_event;
static struct list_head mxs_events[MXS_EVENT_NUM];
static struct mxs_info *mxs_pdev_info;
static int bat_fault_led_status;
static int _main_state = MXS_EVENT_BATTERY;
static int _tmpr_state;
static DECLARE_MUTEX(mxs_event_mutex);

static void _mxs_bat_fault_led_work(struct work_struct *work);
DECLARE_DELAYED_WORK(_bat_fault_led, _mxs_bat_fault_led_work);

int mxs_bat_get_ui_status(void)
{
	return _main_state;
}

static void mxs_bat_led_status(int state)
{
	int red, green;

	switch (state) {
	case MXS_LED_GREEN:
		red = 0; green = 1;
		break;
	case MXS_LED_RED:
		red = 1; green = 0;
		break;
	case MXS_LED_ALL_ON:
		red = 1; green = 1;
		break;
	case MXS_LED_ALL_OFF:
		red = 0; green = 0;
		break;
	default:
		printk(KERN_DEBUG "BATTERRY LED NOT CONTROL..\n");
		return ;
	}

	mxs_charger_led_green_gpio_set(!green);
	mxs_charger_led_red_gpio_set(!red);
}

static void mxs_bat_fault_led_control(int val)
{
	if (val)
		mxs_bat_led_status(MXS_LED_ALL_ON);
	else
		mxs_bat_led_status(MXS_LED_ALL_OFF);
}

static int mxs_bat_average(int reset, int count, int input)
{
	static int _output;
	static int _count;
	int output = 0;

	if (reset) {
		_output = 0;
		_count = 0;
	} else {
		if (_count >= count) {
			output = _output;
			_output = 0;
			_count = 0;
		} else {
			_output  = ((_output * _count) + input) / (_count + 1);
			_count++;
		}
	}

	return output;
}

static void _change_state(int new_state)
{
	const char *mode = "";
	struct list_head *p;
	struct mxs_bat_event_cb_list *temp = NULL;

	/* state control */
	if (new_state == _main_state)
		return;

	_main_state = new_state;

	if (!list_empty(&mxs_events[_main_state])) {
		list_for_each(p, &mxs_events[_main_state]) {
			temp = list_entry(p,
					struct mxs_bat_event_cb_list,
					list);
			temp->callback.func(temp->callback.param);
		}
	}

	cancel_delayed_work(&_bat_fault_led);

	switch (_main_state) {
	case MXS_EVENT_CHARGING:
#ifdef CONFIG_HAS_WAKELOCK
		wake_lock(&_charger_wake_lock);
#endif
		mode = "MXS_EVENT_CHARGING";
		mxs_bat_led_status(MXS_LED_RED);

		mxs_bat_average(1, 0, 0);
		break;
	case MXS_EVENT_BATTERY:
#ifdef CONFIG_HAS_WAKELOCK
		wake_unlock(&_charger_wake_lock);
#endif
		mode = "MXS_EVENT_BATTERY";
		mxs_bat_led_status(MXS_LED_ALL_OFF);
		break;
	case MXS_EVENT_FULL_CHARGED:
		mode = "MXS_EVENT_FULL_CHARGED";
		mxs_bat_led_status(MXS_LED_GREEN);
		break;
	case MXS_EVENT_FAULT:
		mode = "MXS_EVENT_FAULT";
		bat_fault_led_status = 0;
		_mxs_bat_fault_led_work(NULL);
	default:
		break;
	}
	printk(KERN_DEBUG "mxs-power: mode = %s\n", mode);
}

static void _mxs_bat_fault_led_work(struct work_struct *work)
{
	if (bat_fault_led_status) {
		bat_fault_led_status = 0;
		mxs_bat_fault_led_control(1);
	} else {
		bat_fault_led_status = 1;
		mxs_bat_fault_led_control(0);
	}

	schedule_delayed_work(&_bat_fault_led, msecs_to_jiffies(500));
}

static int mxs_bat_led_control(unsigned long arg)
{
	struct mxs_led_data led_data = {0};

	if (copy_from_user((void *)&led_data,
				(const void *)arg,
				sizeof(led_data)))
		return -EFAULT;

	if (led_data.command == MXS_LED_GREEN)
		mxs_charger_led_green_gpio_set(!led_data.control);
	else
		mxs_charger_led_red_gpio_set(!led_data.control);

	return 0;
}

static void mxs_bat_event_list_init(void)
{
	int i;

	for (i = 0; i < MXS_EVENT_NUM; i++)
		INIT_LIST_HEAD(&mxs_events[i]);

	sema_init(&mxs_event_mutex, 1);
}

static void mxs_bat_callbackfn(void *event)
{
	printk(KERN_DEBUG "\n\n DETECT BAT EVENT : %d\n\n",
			(unsigned int)event);
}

static void mxs_bat_user_notify_callback(void *event)
{
	printk(KERN_DEBUG "user_notify_callback ...\n");

	MXS_CIRC_ADD((int)event, mxs_event, MXS_CIRC_BUF_MAX);
	kill_fasync(&mxs_bat_queue, SIGIO, POLL_IN);
}

static int mxs_bat_event_subscribe(int event,
		struct mxs_bat_event_cb callback)
{
	struct mxs_bat_event_cb_list *new = NULL;

	pr_debug("Event: %d Subscribe\n", event);

	/* Check wheter the event & callback are valid? */
	if (event >= MXS_EVENT_NUM) {
		printk(KERN_DEBUG "Invalid Event : %d\n", event);
		return -EINVAL;
	}

	if (NULL == callback.func) {
		pr_debug("Null or Invalid Callback\n");
		return -EINVAL;
	}

	/* Create a new linked list entry */
	new = kmalloc(sizeof(struct mxs_bat_event_cb_list), GFP_KERNEL);
	if (NULL == new)
		return -ENOMEM;

	/* Initialize the list node fields */
	new->callback.func = callback.func;
	new->callback.param = callback.param;
	INIT_LIST_HEAD(&new->list);

	/* Obtain the lock to access the list */
	if (down_interruptible(&mxs_event_mutex)) {
		kfree(new);
		return -EINTR;
	}

	/* Add this entry to the event list */
	list_add_tail(&new->list, &mxs_events[event]);

	/* Release the lock */
	up(&mxs_event_mutex);

	return 0;
}

static int mxs_bat_event_unsubscribe(int event,
		struct mxs_bat_event_cb callback)
{
	struct list_head *p;
	struct list_head *n;
	struct mxs_bat_event_cb_list *temp = NULL;
	int ret = -1;

	pr_debug("Event:%d Unsubscribe\n", event);

	/* Check whether the event & callback are valid? */
	if (event >= MXS_EVENT_NUM) {
		printk(KERN_DEBUG "Invalid Event:%d\n", event);
		return -EINVAL;
	}

	if (NULL == callback.func) {
		pr_debug("Null or Invalid Callback\n");
		return -EINVAL;
	}

	/* Obtain the lock to access the list */
	if (down_interruptible(&mxs_event_mutex))
		return -EINTR;

	/* Find the entry in the list */
	list_for_each_safe(p, n, &mxs_events[event]) {
		temp = list_entry(p, struct mxs_bat_event_cb_list, list);
		if (temp->callback.func == callback.func
		    && temp->callback.param == callback.param) {
			/* Remove the entry from the list */
			list_del(p);
			kfree(temp);
			ret = 0;
			break;
		}
	}

	/* Release the lock */
	up(&mxs_event_mutex);

	return ret;
}

static int mxs_bat_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int mxs_bat_release(struct inode *inode, struct file *file)
{
	int i;
	struct mxs_bat_event_cb event_sub;

	for (i = 0; i < MXS_EVENT_NUM; i++) {
		event_sub.func = mxs_bat_user_notify_callback;
		event_sub.param = (void *)i;
		mxs_bat_event_unsubscribe(i, event_sub);

		event_sub.func = mxs_bat_callbackfn;
		mxs_bat_event_unsubscribe(i, event_sub);
	}

	mxs_event.head = mxs_event.tail = 0;

	return 0;
}

static int mxs_bat_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int val = 0;
	struct mxs_bat_event_cb event_sub;
	int event;

	if (_IOC_TYPE(cmd) != 'P')
		return -ENOTTY;

	switch (cmd) {
	case MXS_BATTERY_VOLTAGE:
		val = ddi_power_GetBattery() * 1000;
		if (copy_to_user((int *) arg, &val, sizeof(int)))
			return -EFAULT;
		break;
	case MXS_AC_ONLINE:
		val = is_ac_online();
		if (copy_to_user((int *) arg, &val, sizeof(int)))
			return -EFAULT;
		break;
	case MXS_USB_ONLINE:
		val = is_usb_online();
		if (copy_to_user((int *) arg, &val, sizeof(int)))
			return -EFAULT;
		break;
	case MXS_BAT_NOTIFY_USER:
		if (get_user(event, (int __user *)arg))
			return -EFAULT;

		event_sub.func = mxs_bat_user_notify_callback;
		event_sub.param = (void *)event;
		ret = mxs_bat_event_subscribe(event, event_sub);
		break;
	case MXS_BAT_GET_NOTIFY:
		MXS_CIRC_REMOVE(event, mxs_event, MXS_CIRC_BUF_MAX);
		if (put_user(event, (int __user *)arg))
			return -EFAULT;
		break;
	case MXS_BAT_LED_CONTROL:
		ret = mxs_bat_led_control(arg);
		break;
	default:
		printk(KERN_DEBUG "mxs bat : unsupported ioctl command 0x%x\n",
				cmd);
		break;
	}

	return ret;
}

static int mxs_bat_fasync(int fd, struct file *filp, int mode)
{
	return fasync_helper(fd, filp, mode, &mxs_bat_queue);
}

static const struct file_operations mxs_bat_fos = {
	.owner		= THIS_MODULE,
	.ioctl		= mxs_bat_ioctl,
	.open		= mxs_bat_open,
	.release	= mxs_bat_release,
	.fasync		= mxs_bat_fasync,
};
#endif

void init_protection(struct mxs_info *info)
{
	enum ddi_power_5v_status pmu_5v_status;
	uint16_t battery_voltage;

	pmu_5v_status = ddi_power_GetPmu5vStatus();
	battery_voltage = ddi_power_GetBattery();

	/* InitializeFiqSystem(); */
	ddi_power_InitOutputBrownouts();


	/* if we start the kernel with 4p2 already started
	 * by the bootlets, we need to hand off from this
	 * state to the kernel 4p2 enabled state.
	 */
	if ((pmu_5v_status == existing_5v_connection) &&
		 ddi_power_check_4p2_bits()) {
		ddi_power_enable_5v_disconnect_detection();

		/* includes VBUS DROOP workaround for errata */
		ddi_power_init_4p2_protection();

		/* if we still have our 5V connection, we can disable
		 * battery brownout interrupt.  This is because the
		 * VDD5V DROOP IRQ handler will also shutdown if battery
		 * is browned out and it will enable the battery brownout
		 * and bring VBUSVALID_TRSH level back to a normal level
		 * which caused the hardware battery brownout shutdown
		 * to be enabled.  The benefit of this is that device
		 * that have detachable batteries (or devices going through
		 * the assembly line and running this firmware to test
		 *  with) can avoid shutting down if 5V is present and
		 *  battery voltage goes away.
		 */
		ddi_power_EnableBatteryBoInterrupt(false);

		info->sm_5v_connection_status = _5v_connected_verified;

#ifdef CONFIG_MACH_MX23_CANOPUS
		_change_state(MXS_EVENT_CHARGING);
#endif
	} else {
#ifdef DEBUG_IRQS
		if (battery_voltage <
			OS_SHUTDOWN_BATTERY_VOLTAGE_THRESHOLD_MV) {
			printk(KERN_CRIT "Polled battery voltage measurement is\
				less than %dmV.  Kernel should be halted/\
				shutdown\n",
				OS_SHUTDOWN_BATTERY_VOLTAGE_THRESHOLD_MV);

			return;
		}
#endif
		info->sm_5v_connection_status = _5v_disconnected_verified;
		ddi_power_EnableBatteryBoInterrupt(true);

	}


	/* all brownouts are now handled software fiqs.  We
	 * can now disable the hardware protection mechanisms
	 *  because leaving them on yields ~2kV ESD level
	 *  versus ~4kV ESD levels when they are off.  This
	 *  difference is suspected to be cause by the fast
	 *  falling edge pswitch functionality being tripped
	 *  by ESD events.  This functionality is disabled
	 *  when PWD_OFF is disabled.
	 */
#ifdef DISABLE_HARDWARE_PROTECTION_MECHANISMS
	__raw_writel(BM_POWER_RESET_PWD_OFF,
		HW_POWER_RESET_SET_ADDR);
#endif




}



static void check_and_handle_5v_connection(struct mxs_info *info)
{

	switch (ddi_power_GetPmu5vStatus()) {

	case new_5v_connection:
		ddi_power_enable_5v_disconnect_detection();
		info->sm_5v_connection_status = _5v_connected_unverified;

	case existing_5v_connection:
		if (info->sm_5v_connection_status != _5v_connected_verified) {
			/* we allow some time to pass before considering
			 * the 5v connection to be ready to use.  This
			 * will give the USB system time to enumerate
			 * (coordination with USB driver to be added
			 * in the future).
			 */

			/* handle jiffies rollover case */
			if ((jiffies - info->sm_new_5v_connection_jiffies)
				< 0) {
				info->sm_new_5v_connection_jiffies = jiffies;
				break;
			}

			if ((jiffies_to_msecs(jiffies -
				info->sm_new_5v_connection_jiffies)) >
				_5V_DEBOUNCE_TIME_MS) {
				info->sm_5v_connection_status =
					_5v_connected_verified;
				dev_dbg(info->dev,
					"5v connection verified\n");
			if (info->onboard_vbus5v) {
				if (regulator_is_enabled(
					info->onboard_vbus5v) > 0) {
					info->onboard_vbus5v_online = 1;
					pr_debug("When supply from \
					onboard vbus 5v ,\
					DO NOT switch to 4p2 \n");
					break;
			}
		}
#ifdef CONFIG_MXS_VBUS_CURRENT_DRAW
	#ifdef CONFIG_USB_GADGET
		/* if there is USB 2.0 current limitation requirement,
		* waiting for USB enum done.
		*/
		if ((__raw_readl(REGS_POWER_BASE + HW_POWER_5VCTRL)
			& BM_POWER_5VCTRL_CHARGE_4P2_ILIMIT) ==
			(0x20 << BP_POWER_5VCTRL_CHARGE_4P2_ILIMIT)) {
			dev_info(info->dev, "waiting USB enum done...\r\n");
		}
		while ((__raw_readl(REGS_POWER_BASE + HW_POWER_5VCTRL)
			& BM_POWER_5VCTRL_CHARGE_4P2_ILIMIT)
			== (0x20 << BP_POWER_5VCTRL_CHARGE_4P2_ILIMIT)) {
			msleep(50);
		}
	#endif
#endif
				ddi_power_Enable4p2(450);

				/* part of handling for errata.  It is
				 *  now "somewhat" safe to
				 * turn on vddio interrupts again
				 */
				ddi_power_enable_vddio_interrupt(true);

#ifdef CONFIG_MACH_MX23_CANOPUS
				_change_state(MXS_EVENT_CHARGING);
#endif
			}
		}

		break;

	case new_5v_disconnection:

		ddi_bc_SetDisable();
		ddi_bc_SetCurrentLimit(0);
		if (info->regulator)
			regulator_set_current_limit(info->regulator, 0, 0);
		info->is_usb_online = 0;
		info->is_ac_online = 0;
		info->onboard_vbus5v_online = 0;

		info->sm_5v_connection_status = _5v_disconnected_unverified;

	case existing_5v_disconnection:

		if (info->sm_5v_connection_status !=
			_5v_disconnected_verified) {
			if ((jiffies - info->sm_new_5v_disconnection_jiffies)
				< 0) {
				info->sm_new_5v_connection_jiffies = jiffies;
				break;
			}

			if ((jiffies_to_msecs(jiffies -
				info->sm_new_5v_disconnection_jiffies)) >
				_5V_DEBOUNCE_TIME_MS) {
				info->sm_5v_connection_status =
					_5v_disconnected_verified;
				ddi_power_execute_5v_to_battery_handoff();
				ddi_power_enable_5v_connect_detection();

				/* part of handling for errata.
				 * It is now safe to
				 * turn on vddio interrupts again
				 */
				ddi_power_enable_vddio_interrupt(true);
				dev_dbg(info->dev,
					"5v disconnection handled\n");

				__raw_writel(__raw_readl(REGS_POWER_BASE +
				HW_POWER_5VCTRL) &
				(~BM_POWER_5VCTRL_CHARGE_4P2_ILIMIT)
				| (0x20 << BP_POWER_5VCTRL_CHARGE_4P2_ILIMIT),
				REGS_POWER_BASE + HW_POWER_5VCTRL);

#ifdef CONFIG_MACH_MX23_CANOPUS
				_change_state(MXS_EVENT_BATTERY);
#endif
			}
		}

		break;
	}
}


static void handle_battery_voltage_changes(struct mxs_info *info)
{
#if 0
	uint16_t battery_voltage;

	battery_voltage = ddi_power_GetBattery();

	if (info->sm_5v_connection_status != _5v_connected_verified) {
		if (battery_voltage <
			OS_SHUTDOWN_BATTERY_VOLTAGE_THRESHOLD_MV) {
			printk(KERN_CRIT "Polled battery voltage measurement is\
				less than %dmV.  Shutting down the \
				system\n",
				OS_SHUTDOWN_BATTERY_VOLTAGE_THRESHOLD_MV);

			shutdown_os();
			return;
		}
	} else
#endif
	{
		ddi_power_handle_cmptrip();

		if (ddi_power_IsBattRdyForXfer())
			ddi_power_enable_5v_to_battery_xfer(true);
		else
			ddi_power_enable_5v_to_battery_xfer(false);

	}
}


/*
 * Power properties
 */
static enum power_supply_property mxs_power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int mxs_power_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	struct mxs_info *info;
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
			/* ac online */
			info = container_of(psy, struct mxs_info, ac);
			val->intval = info->onboard_vbus5v_online ?
				0 : is_ac_online();
		} else {
			/* usb online */
			info = container_of(psy, struct mxs_info, usb);
			val->intval = info->onboard_vbus5v_online ?
				0 : is_usb_online();
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
/*
 * Battery properties
 */
static enum power_supply_property mxs_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
};

static int mxs_bat_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	struct mxs_info *info = to_mxs_info(psy);
	ddi_bc_State_t state;
	ddi_bc_BrokenReason_t reason;
	int temp_alarm;
	int16_t temp_lo, temp_hi;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		state = ddi_bc_GetState();
		switch (state) {
		case DDI_BC_STATE_CONDITIONING:
		case DDI_BC_STATE_CHARGING:
		case DDI_BC_STATE_TOPPING_OFF:
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;
		case DDI_BC_STATE_DISABLED:
			val->intval = (ddi_power_Get5vPresentFlag()
				&& !info->onboard_vbus5v_online) ?
				POWER_SUPPLY_STATUS_NOT_CHARGING :
			POWER_SUPPLY_STATUS_DISCHARGING;
			break;
#ifdef CONFIG_MACH_MX23_CANOPUS
		case DDI_BC_STATE_TOPPING_OFF_COMPLETE:
			val->intval = POWER_SUPPLY_STATUS_FULL;
			break;
#endif
		default:
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		/* is battery present */
		state = ddi_bc_GetState();
		switch (state) {
#ifdef CONFIG_MACH_MX23_CANOPUS
		case DDI_BC_STATE_TOPPING_OFF_COMPLETE:
#endif
		case DDI_BC_STATE_WAITING_TO_CHARGE:
		case DDI_BC_STATE_DCDC_MODE_WAITING_TO_CHARGE:
		case DDI_BC_STATE_CONDITIONING:
		case DDI_BC_STATE_CHARGING:
		case DDI_BC_STATE_TOPPING_OFF:
		case DDI_BC_STATE_DISABLED:
			val->intval = 1;
			break;
		case DDI_BC_STATE_BROKEN:
			val->intval = !(ddi_bc_GetBrokenReason() ==
					DDI_BC_BROKEN_NO_BATTERY_DETECTED);
			break;
		default:
			val->intval = 0;
			break;
		}
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		temp_alarm = ddi_bc_RampGetDieTempAlarm();
		if (temp_alarm) {
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		} else {
			state = ddi_bc_GetState();
			switch (state) {
			case DDI_BC_STATE_BROKEN:
				reason = ddi_bc_GetBrokenReason();
				val->intval =
				   (reason == DDI_BC_BROKEN_CHARGING_TIMEOUT) ?
					POWER_SUPPLY_HEALTH_DEAD :
					POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
				break;
			case DDI_BC_STATE_UNINITIALIZED:
				val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
				break;
			default:
				val->intval = POWER_SUPPLY_HEALTH_GOOD;
				break;
			}
		}
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		/* uV */
		val->intval = ddi_power_GetBattery() * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		/* uA */
		val->intval = ddi_power_GetMaxBatteryChargeCurrent() * 1000;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		mutex_lock(&info->sm_lock);
		ddi_power_GetDieTemp(&temp_lo, &temp_hi);
		mutex_unlock(&info->sm_lock);
		val->intval = temp_lo + (temp_hi - temp_lo) / 2;

		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void state_machine_timer(unsigned long data)
{
	struct mxs_info *info = (struct mxs_info *)data;
	ddi_bc_Cfg_t *cfg = info->sm_cfg;
	int ret;

	/* schedule next call to state machine */
	mod_timer(&info->sm_timer,
		  jiffies + msecs_to_jiffies(cfg->u32StateMachinePeriod));

	ret = schedule_work(&info->sm_work);
	if (!ret)
		dev_dbg(info->dev, "state machine failed to schedule\n");

}
/*
 * Assumption:
 * AC power can't be switched to USB w/o system reboot
 * and vice-versa
 */
static void state_machine_work(struct work_struct *work)
{
	struct mxs_info *info =
		container_of(work, struct mxs_info, sm_work);

	mutex_lock(&info->sm_lock);

	handle_battery_voltage_changes(info);

	check_and_handle_5v_connection(info);

#ifdef MXS_CANOPUS_LOG_ENABLE
	static unsigned long i;
	int timeout = mxs_log_charge_get_timeout();

	if (timeout && !(i++ % (10 * timeout)))
		mxs_log_charge_update(1);
#endif

#ifdef CONFIG_MACH_MX23_CANOPUS
	if (info->sm_5v_connection_status == _5v_connected_verified) {
		int die_tmpr = ddi_bc_RampGetDieTempAlarm();
		int bat_tmpr = ddi_bc_RampGetBatteryTempAlarm();
		int voltage = ddi_power_GetBattery();
		int voltage_avr = mxs_bat_average(0, 3000, voltage);

		ddi_bc_State_t state = ddi_bc_GetState();
		ddi_bc_BrokenReason_t reason = ddi_bc_GetBrokenReason();

		if (!_tmpr_state && (die_tmpr || bat_tmpr)) {
			_tmpr_state = 1;
			_change_state(MXS_EVENT_FAULT);
		} else if (_tmpr_state && !die_tmpr && !bat_tmpr) {
			_tmpr_state = 0;
			_change_state(MXS_EVENT_CHARGING);
		} else if (state == DDI_BC_STATE_BROKEN) {
			if (reason == DDI_BC_BROKEN_CHARGING_TIMEOUT) {
				if (voltage >= DDI_BC_LIION_CHARGING_VOLTAGE) {
					ddi_bc_SetFixed();
					ddi_bc_SetEnable();
					_change_state(MXS_EVENT_FULL_CHARGED);
				}
			} else
				_change_state(MXS_EVENT_FAULT);
		} else if (state == DDI_BC_STATE_TOPPING_OFF_COMPLETE ||
				voltage_avr >= DDI_BC_LIION_CHARGING_VOLTAGE+50)
			_change_state(MXS_EVENT_FULL_CHARGED);
	}
#endif

	if ((info->sm_5v_connection_status != _5v_connected_verified) ||
			!(info->regulator)) {
		mod_timer(&info->sm_timer, jiffies + msecs_to_jiffies(100));
		goto out;
	}

	/* if we made it here, we have a verified 5v connection */
#ifndef CONFIG_MXS_VBUS_CURRENT_DRAW
		if (info->is_ac_online || info->onboard_vbus5v_online)
			goto done;
		/* ac supply connected */
		dev_dbg(info->dev, "changed power connection to ac/5v.\n)");
		dev_dbg(info->dev, "5v current limit set to %u.\n",
			NON_USB_5V_SUPPLY_CURRENT_LIMIT_MA);

		info->is_ac_online = 1;
		info->is_usb_online = 0;

		ddi_power_set_4p2_ilimit(
				NON_USB_5V_SUPPLY_CURRENT_LIMIT_MA);
		ddi_bc_SetCurrentLimit(
				NON_USB_5V_SUPPLY_CURRENT_LIMIT_MA /*mA*/);
		if (regulator_set_current_limit(info->regulator,
				0,
				NON_USB_5V_SUPPLY_CURRENT_LIMIT_MA*1000)) {
			dev_err(info->dev, "reg_set_current(%duA) failed\n",
				NON_USB_5V_SUPPLY_CURRENT_LIMIT_MA*1000);
		}
		ddi_bc_SetEnable();
		goto done;
#else

	if (!is_usb_online())
		goto out;

	if (info->is_usb_online & USB_REG_SET)
		goto done;

	info->is_ac_online = 0;
	info->is_usb_online |= USB_ONLINE;



	if (!(info->is_usb_online & USB_N_SEND)) {
		info->is_usb_online |= USB_N_SEND;
	}


	dev_dbg(info->dev, "%s: charge current set to %dmA\n", __func__,
			POWERED_USB_5V_CURRENT_LIMIT_MA);

	if (regulator_set_current_limit(info->regulator,
		0,
		POWERED_USB_5V_CURRENT_LIMIT_MA*1000)) {
		dev_err(info->dev, "reg_set_current(%duA) failed\n",
				POWERED_USB_5V_CURRENT_LIMIT_MA*1000);
	} else {

		if (info->onboard_vbus5v_online == 0) {
			ddi_bc_SetCurrentLimit(
				POWERED_USB_5V_CURRENT_LIMIT_MA/*mA*/);
			ddi_bc_SetEnable();
		} else
			pr_debug("DO NOT charge from onboard 5v");
	}

	if (info->is_usb_online & USB_SM_RESTART) {
		info->is_usb_online &= ~USB_SM_RESTART;
		ddi_bc_SetEnable();
	}

	info->is_usb_online |= USB_REG_SET;

#endif
	dev_dbg(info->dev, "changed power connection to usb/5v present\n");

done:
	ddi_bc_StateMachine();
out:
	mutex_unlock(&info->sm_lock);
}



static int bc_sm_restart(struct mxs_info *info)
{
	ddi_bc_Status_t bcret;
	int ret = 0;

	mutex_lock(&info->sm_lock);

	/* ungate power clk */
	ddi_power_SetPowerClkGate(0);

	/*
	 * config battery charger state machine and move it to the Disabled
	 * state. This must be done before starting the state machine.
	 */
	bcret = ddi_bc_Init(info->sm_cfg);
	if (bcret != DDI_BC_STATUS_SUCCESS) {
		dev_err(info->dev, "battery charger init failed: %d\n", bcret);
		ret = -EIO;
		goto out;
	} else {

		if (!info->regulator) {
			info->regulator = regulator_get(NULL, "charger-1");
			if (!info->regulator || IS_ERR(info->regulator)) {
				dev_err(info->dev,
				"%s: failed to get regulator\n", __func__);
				info->regulator = NULL;
			} else {
				regulator_set_current_limit(
						info->regulator, 0, 0);
				regulator_set_mode(info->regulator,
						   REGULATOR_MODE_FAST);
			}
		}
	}



	/* schedule first call to state machine */
	mod_timer(&info->sm_timer, jiffies + 1);
out:
	mutex_unlock(&info->sm_lock);
	return ret;
}

#ifndef POWER_FIQ

static irqreturn_t mxs_irq_dcdc4p2_bo(int irq, void *cookie)
{
#ifdef DEBUG_IRQS
	struct mxs_info *info = (struct mxs_info *)cookie;
	dev_info(info->dev, "dcdc4p2 brownout interrupt occurred\n");

#endif
	ddi_power_handle_dcdc4p2_bo();
	return IRQ_HANDLED;
}

static irqreturn_t mxs_irq_batt_brnout(int irq, void *cookie)
{
#ifdef DEBUG_IRQS
	struct mxs_info *info = (struct mxs_info *)cookie;
	dev_info(info->dev, "battery brownout interrupt occurred\n");
	ddi_power_disable_power_interrupts();
#else
	ddi_power_shutdown();
#endif
	return IRQ_HANDLED;
}


static irqreturn_t mxs_irq_vddd_brnout(int irq, void *cookie)
{
#ifdef DEBUG_IRQS
	struct mxs_info *info = (struct mxs_info *)cookie;
	dev_info(info->dev, "vddd brownout interrupt occurred\n");
	ddi_power_disable_power_interrupts();
#else
	ddi_power_shutdown();
#endif
	return IRQ_HANDLED;
}
static irqreturn_t mxs_irq_vdda_brnout(int irq, void *cookie)
{
#ifdef DEBUG_IRQS
	struct mxs_info *info = (struct mxs_info *)cookie;
	dev_info(info->dev, "vdda brownout interrupt occurred\n");
	ddi_power_disable_power_interrupts();
#else
	ddi_power_shutdown();
#endif
	return IRQ_HANDLED;
}

static irqreturn_t mxs_irq_vdd5v_droop(int irq, void *cookie)
{
#ifdef DEBUG_IRQS
	struct mxs_info *info = (struct mxs_info *)cookie;
	dev_info(info->dev, "vdd5v droop interrupt occurred\n");
#endif
	ddi_power_handle_vdd5v_droop();

	return IRQ_HANDLED;
}

#endif /* if POWER_FIQ */

static irqreturn_t mxs_irq_vddio_brnout(int irq, void *cookie)
{
#ifdef DEBUG_IRQS
	struct mxs_info *info = (struct mxs_info *)cookie;
	dev_info(info->dev, "vddio brownout interrupt occurred\n");
	ddi_power_disable_power_interrupts();
#else
	ddi_power_handle_vddio_brnout();
#endif
	return IRQ_HANDLED;
}

static irqreturn_t mxs_irq_vdd5v(int irq, void *cookie)
{
	struct mxs_info *info = (struct mxs_info *)cookie;

	switch (ddi_power_GetPmu5vStatus()) {

	case new_5v_connection:
		ddi_power_disable_5v_connection_irq();
		dev_dbg(info->dev, "new 5v connection detected\n");
		info->sm_new_5v_connection_jiffies = jiffies;
		mod_timer(&info->sm_timer, jiffies + 1);
		break;

	case new_5v_disconnection:
		/* due to 5v connect vddio bo chip bug, we need to
		 * disable vddio interrupts until we reset the 5v
		 * detection for 5v connect detect.  We want to allow
		 * some debounce time before enabling connect detection.
		 * This is handled in the vdd5v_droop interrupt for now.
		 */
		/* ddi_power_enable_vddio_interrupt(false); */

		ddi_power_disable_5v_connection_irq();
		dev_dbg(info->dev, "new 5v disconnection detected\n");
		info->sm_new_5v_disconnection_jiffies = jiffies;
		mod_timer(&info->sm_timer, jiffies + 1);
		break;

	default:

		break;

	}

	return IRQ_HANDLED;
}

#ifdef MXS_CANOPUS_LOG_ENABLE
static int mxs_bat_log_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int timeout = mxs_log_charge_get_timeout();

	return snprintf(buf, PAGE_SIZE, "%d\n", timeout);
}

static int mxs_bat_log_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long long timeout;

	strict_strtoull(buf, 0, &timeout);

	mxs_log_charge_set_timeout(timeout);

	return len;
}

static DEVICE_ATTR(log_timeout, 0644,
		   mxs_bat_log_show,
		   mxs_bat_log_store);
#endif

static int mxs_bat_probe(struct platform_device *pdev)
{
	struct mxs_info *info;
	int ret = 0;
	void *base = IO_ADDRESS(RTC_PHYS_ADDR);


	/* enable usb device presence detection */
	fsl_enable_usb_plugindetect();



    /* check bit 11, bit 11 is set in power_prep.c if it is 5V only build.
       we don't need initialize battery for 5V only build
    */
	if ((__raw_readl(base + HW_RTC_PERSISTENT1) & 0x800)) {

		__raw_writel(0x800, base + HW_RTC_PERSISTENT1_CLR);

		/* InitializeFiqSystem(); */
		ddi_power_InitOutputBrownouts();
		return 0;
    }

	ret = ddi_power_init_battery();
	if (ret) {
		printk(KERN_ERR "Aborting power driver initialization\n");
		return 1;
	}


	if (!pdev->dev.platform_data) {
		printk(KERN_ERR "%s: missing platform data\n", __func__);
		return -ENODEV;
	}

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->irq_vdd5v = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (info->irq_vdd5v == NULL) {
		printk(KERN_ERR "%s: failed to get irq resouce\n", __func__);
		goto free_info;
	}

	info->irq_vddio_brnout = platform_get_resource(
		pdev, IORESOURCE_IRQ, 5);
	if (info->irq_vddio_brnout == NULL) {
		printk(KERN_ERR "%s: failed to get irq resouce\n", __func__);
		goto free_info;
	}

#ifndef POWER_FIQ
	info->irq_dcdc4p2_bo = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
	if (info->irq_dcdc4p2_bo == NULL) {
		printk(KERN_ERR "%s: failed to get irq resouce\n", __func__);
		goto free_info;
	}

	info->irq_batt_brnout = platform_get_resource(pdev, IORESOURCE_IRQ, 2);
	if (info->irq_batt_brnout == NULL) {
		printk(KERN_ERR "%s: failed to get irq resouce\n", __func__);
		goto free_info;
	}

	info->irq_vddd_brnout = platform_get_resource(pdev, IORESOURCE_IRQ, 3);
	if (info->irq_vddd_brnout == NULL) {
		printk(KERN_ERR "%s: failed to get irq resouce\n", __func__);
		goto free_info;
	}

	info->irq_vdda_brnout = platform_get_resource(pdev, IORESOURCE_IRQ, 4);
	if (info->irq_vdda_brnout == NULL) {
		printk(KERN_ERR "%s: failed to get irq resouce\n", __func__);
		goto free_info;
	}


	info->irq_vdd5v_droop = platform_get_resource(pdev, IORESOURCE_IRQ, 6);
	if (info->irq_vdd5v_droop == NULL) {
		printk(KERN_ERR "%s: failed to get irq resouce\n", __func__);
		goto free_info;
	}
#endif


	platform_set_drvdata(pdev, info);

	info->dev    = &pdev->dev;
	info->sm_cfg = pdev->dev.platform_data;

	/* initialize bat power_supply struct */
	info->bat.name           = "battery";
	info->bat.type           = POWER_SUPPLY_TYPE_BATTERY;
	info->bat.properties     = mxs_bat_props;
	info->bat.num_properties = ARRAY_SIZE(mxs_bat_props);
	info->bat.get_property   = mxs_bat_get_property;

	/* initialize ac power_supply struct */
	info->ac.name           = "ac";
	info->ac.type           = POWER_SUPPLY_TYPE_MAINS;
	info->ac.properties     = mxs_power_props;
	info->ac.num_properties = ARRAY_SIZE(mxs_power_props);
	info->ac.get_property   = mxs_power_get_property;

	/* initialize usb power_supply struct */
	info->usb.name           = "usb";
	info->usb.type           = POWER_SUPPLY_TYPE_USB;
	info->usb.properties     = mxs_power_props;
	info->usb.num_properties = ARRAY_SIZE(mxs_power_props);
	info->usb.get_property   = mxs_power_get_property;

#ifdef CONFIG_MACH_MX23_CANOPUS
	struct class_device *mxs_device;

	mxs_pdev_info = platform_get_drvdata(pdev);
	mxs_bat_major = register_chrdev(0, MXS_DEV_NAME, &mxs_bat_fos);
	if (mxs_bat_major < 0) {
		printk(KERN_DEBUG "unable to get a major for mxs_bat\n");
		ret = mxs_bat_major;
		goto free_info;
	}

	mxs_dev_class = class_create(THIS_MODULE, MXS_DEV_NAME);
	if (IS_ERR(mxs_dev_class)) {
		printk(KERN_DEBUG "error creating mxs_bat dev class\n");
		ret = -1;
		goto class_failed;
	}

	mxs_device = device_create(mxs_dev_class, NULL,
			MKDEV(mxs_bat_major, 0), NULL, MXS_DEV_NAME);
	if (IS_ERR(mxs_device)) {
		printk(KERN_DEBUG "error creating mxs_bat class device\n");
		ret = -1;
		goto dev_failed;
	}

	mxs_event.buf = kmalloc(MXS_CIRC_BUF_MAX * sizeof(char), GFP_KERNEL);
	if (NULL == mxs_event.buf) {
		ret = -ENOMEM;
		goto buf_failed;
	}

	mxs_bat_event_list_init();
	mxs_event.head = mxs_event.tail = 0;
#endif

#ifdef MXS_CANOPUS_LOG_ENABLE
	ret = device_create_file(&(pdev->dev), &dev_attr_log_timeout);
	if (ret < 0) {
		printk(KERN_DEBUG "error adding mxs_bat attr\n");
		ret = -1;
		goto buf_failed;
	}
#endif

	init_timer(&info->sm_timer);
	info->sm_timer.data = (unsigned long)info;
	info->sm_timer.function = state_machine_timer;

	mutex_init(&info->sm_lock);
	INIT_WORK(&info->sm_work, state_machine_work);

	/* init LRADC channels to measure battery voltage and die temp */

	__raw_writel(BM_POWER_5VCTRL_ENABLE_LINREG_ILIMIT,
		REGS_POWER_BASE + HW_POWER_5VCTRL_CLR);

	ret = bc_sm_restart(info);
	if (ret)
#ifdef CONFIG_MACH_MX23_CANOPUS
		goto buf_failed;
#else
		goto free_info;
#endif


	ret = request_irq(info->irq_vdd5v->start,
			mxs_irq_vdd5v, IRQF_DISABLED | IRQF_SHARED,
			pdev->name, info);
	if (ret) {
		dev_err(info->dev, "failed to request irq\n");
		goto stop_sm;
	}

	ret = request_irq(info->irq_vddio_brnout->start,
			mxs_irq_vddio_brnout, IRQF_DISABLED,
			pdev->name, info);
	if (ret) {
		dev_err(info->dev, "failed to request irq\n");
		goto stop_sm;
	}

#ifndef POWER_FIQ
	ret = request_irq(info->irq_dcdc4p2_bo->start,
			mxs_irq_dcdc4p2_bo, IRQF_DISABLED,
			pdev->name, info);
	if (ret) {
		dev_err(info->dev, "failed to request irq\n");
		goto stop_sm;
	}

	ret = request_irq(info->irq_batt_brnout->start,
			mxs_irq_batt_brnout, IRQF_DISABLED,
			pdev->name, info);
	if (ret) {
		dev_err(info->dev, "failed to request irq\n");
		goto stop_sm;
	}

	ret = request_irq(info->irq_vddd_brnout->start,
			mxs_irq_vddd_brnout, IRQF_DISABLED,
			pdev->name, info);
	if (ret) {
		dev_err(info->dev, "failed to request irq\n");
		goto stop_sm;
	}

	ret = request_irq(info->irq_vdda_brnout->start,
			mxs_irq_vdda_brnout, IRQF_DISABLED,
			pdev->name, info);
	if (ret) {
		dev_err(info->dev, "failed to request irq\n");
		goto stop_sm;
	}


	ret = request_irq(info->irq_vdd5v_droop->start,
			mxs_irq_vdd5v_droop, IRQF_DISABLED,
			pdev->name, info);
	if (ret) {
		dev_err(info->dev, "failed to request irq\n");
		goto stop_sm;
	}
#endif

	ret = power_supply_register(&pdev->dev, &info->bat);
	if (ret) {
		dev_err(info->dev, "failed to register battery\n");
		goto free_irq;
	}

	ret = power_supply_register(&pdev->dev, &info->ac);
	if (ret) {
		dev_err(info->dev, "failed to register ac power supply\n");
		goto unregister_bat;
	}

	ret = power_supply_register(&pdev->dev, &info->usb);
	if (ret) {
		dev_err(info->dev, "failed to register usb power supply\n");
		goto unregister_ac;
	}

	/* handoff protection handling from bootlets protection method
	 * to kernel protection method
	 */
	init_protection(info);


	info->onboard_vbus5v = regulator_get(NULL, "vbus5v");
	if (IS_ERR(info->onboard_vbus5v)) {
		pr_debug("No onboard vbus 5v reg provided\n");
		info->onboard_vbus5v = NULL;
	}

	return 0;

unregister_ac:
	power_supply_unregister(&info->ac);
unregister_bat:
	power_supply_unregister(&info->bat);
free_irq:
	free_irq(info->irq_vdd5v->start, pdev);
	free_irq(info->irq_vddio_brnout->start, pdev);
#ifndef	POWER_FIQ
	free_irq(info->irq_dcdc4p2_bo->start, pdev);
	free_irq(info->irq_batt_brnout->start, pdev);
	free_irq(info->irq_vddd_brnout->start, pdev);
	free_irq(info->irq_vdda_brnout->start, pdev);
	free_irq(info->irq_vdd5v_droop->start, pdev);
#endif

stop_sm:
	ddi_bc_ShutDown();
#ifdef CONFIG_MACH_MX23_CANOPUS
buf_failed:
#if 0
	class_device_destroy(mxs_dev_class, MKDEV(mxs_bat_major, 0));
#endif
dev_failed:
	class_destroy(mxs_dev_class);
class_failed:
	unregister_chrdev(mxs_bat_major, MXS_DEV_NAME);
#endif
free_info:
	kfree(info);
	return ret;
}

static int mxs_bat_remove(struct platform_device *pdev)
{
	struct mxs_info *info = platform_get_drvdata(pdev);

	if (info->regulator)
		regulator_put(info->regulator);
	free_irq(info->irq_vdd5v->start, pdev);
	free_irq(info->irq_vddio_brnout->start, pdev);
#ifndef	POWER_FIQ
	free_irq(info->irq_dcdc4p2_bo->start, pdev);
	free_irq(info->irq_batt_brnout->start, pdev);
	free_irq(info->irq_vddd_brnout->start, pdev);
	free_irq(info->irq_vdda_brnout->start, pdev);
	free_irq(info->irq_vdd5v_droop->start, pdev);
#endif
	ddi_bc_ShutDown();
	power_supply_unregister(&info->usb);
	power_supply_unregister(&info->ac);
	power_supply_unregister(&info->bat);

#ifdef CONFIG_MACH_MX23_CANOPUS
	mxs_pdev_info = NULL;
#if 0
	class_device_destroy(mxs_dev_class, MKDEV(mxs_bat_major, 0));
#endif
	class_destroy(mxs_dev_class);
	unregister_chrdev(mxs_bat_major, MXS_DEV_NAME);
#endif

	return 0;
}

static void mxs_bat_shutdown(struct platform_device *pdev)
{
	ddi_bc_ShutDown();
}


#ifdef CONFIG_PM

static int mxs_bat_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct mxs_info *info = platform_get_drvdata(pdev);

	mutex_lock(&info->sm_lock);

	/* enable USB 5v wake up so don't disable irq here*/

	ddi_bc_SetDisable();
	/* cancel state machine timer */
	del_timer_sync(&info->sm_timer);

	mutex_unlock(&info->sm_lock);
	return 0;
}

static int mxs_bat_resume(struct platform_device *pdev)
{
	struct mxs_info *info = platform_get_drvdata(pdev);
	ddi_bc_Cfg_t *cfg = info->sm_cfg;

	mutex_lock(&info->sm_lock);

	if (is_ac_online()) {
		/* ac supply connected */
		dev_dbg(info->dev, "ac/5v present, enabling state machine\n");

		info->is_ac_online = 1;
		info->is_usb_online = 0;
		ddi_bc_SetCurrentLimit(
			NON_USB_5V_SUPPLY_CURRENT_LIMIT_MA /*mA*/);
		ddi_bc_SetEnable();
	} else if (is_usb_online()) {
		/* usb supply connected */
		dev_dbg(info->dev, "usb/5v present, enabling state machine\n");

		info->is_ac_online = 0;
		info->is_usb_online = 1;
		ddi_bc_SetCurrentLimit(POWERED_USB_5V_CURRENT_LIMIT_MA /*mA*/);
		ddi_bc_SetEnable();
	} else {
		/* not powered */
		dev_dbg(info->dev, "%s: 5v not present\n", __func__);

		info->is_ac_online = 0;
		info->is_usb_online = 0;
	}

	/* enable 5v irq */
	__raw_writel(BM_POWER_CTRL_ENIRQ_VDD5V_GT_VDDIO,
		REGS_POWER_BASE + HW_POWER_CTRL_SET);

	/* reschedule calls to state machine */
	mod_timer(&info->sm_timer,
		  jiffies + msecs_to_jiffies(cfg->u32StateMachinePeriod));

	mutex_unlock(&info->sm_lock);
	return 0;
}

#else
#define mxs_bat_suspend NULL
#define mxs_bat_resume  NULL
#endif

static struct platform_driver mxs_batdrv = {
	.probe		= mxs_bat_probe,
	.remove		= mxs_bat_remove,
	.shutdown       = mxs_bat_shutdown,
	.suspend	= mxs_bat_suspend,
	.resume		= mxs_bat_resume,
	.driver		= {
		.name	= "mxs-battery",
		.owner	= THIS_MODULE,
	},
};

#ifdef POWER_FIQ
static int power_relinquish(void *data, int relinquish)
{
	return -1;
}

static struct fiq_handler power_fiq = {
	.name = "mxs-battery",
	.fiq_op = power_relinquish
};

static struct pt_regs fiq_regs;
extern char power_fiq_start[], power_fiq_end[];
extern void lock_vector_tlb(void *);
extern long power_fiq_count;
static struct proc_dir_entry *power_fiq_proc;
#endif

static int __init mxs_bat_init(void)
{
	struct clk *cpu, *pll0;
	int ret;
    int no_battery = false;

#ifdef POWER_FIQ

    /* check bit 11, bit 11 is set in power_prep.c if it is 5V only build.
    */

	if ((__raw_readl(IO_ADDRESS(RTC_PHYS_ADDR) + HW_RTC_PERSISTENT1)
		& 0x800)) {
		no_battery = true;
		printk(KERN_NOTICE "This is 5V only build. \r\n");
	}

	ret = claim_fiq(&power_fiq);
	if (ret) {
		pr_err("Can't claim fiq");
	} else {
		get_fiq_regs(&fiq_regs);
		set_fiq_handler(power_fiq_start, power_fiq_end-power_fiq_start);
		lock_vector_tlb((void *)0xffff0000);
		lock_vector_tlb(REGS_POWER_BASE);

		/* disable interrupts to be configured as FIQs */
		disable_irq(IRQ_DCDC4P2_BRNOUT);
		disable_irq(IRQ_BATT_BRNOUT);
		disable_irq(IRQ_VDDD_BRNOUT);
#ifndef CONFIG_ARCH_MX23
		disable_irq(IRQ_VDDA_BRNOUT);
#endif
		if (no_battery)
			disable_irq(IRQ_VDDIO_BRNOUT);
#ifndef CONFIG_ARCH_MX28
		disable_irq(IRQ_VDD18_BRNOUT);
#endif
		disable_irq(IRQ_VDD5V_DROOP);


		/* Enable these interrupts as FIQs */
		mxs_set_irq_fiq(IRQ_DCDC4P2_BRNOUT, 1);
		mxs_set_irq_fiq(IRQ_BATT_BRNOUT, 1);
		mxs_set_irq_fiq(IRQ_VDDD_BRNOUT, 1);
#ifndef CONFIG_ARCH_MX23
		mxs_set_irq_fiq(IRQ_VDDA_BRNOUT, 1);
#endif
		if (no_battery)
			mxs_set_irq_fiq(IRQ_VDDIO_BRNOUT, 1);
#ifndef CONFIG_ARCH_MX28
		mxs_set_irq_fiq(IRQ_VDD18_BRNOUT, 1);
#endif
		mxs_set_irq_fiq(IRQ_VDD5V_DROOP, 1);


		/* enable FIQ functionality */
		mxs_enable_fiq_functionality(1);

		enable_irq(IRQ_DCDC4P2_BRNOUT);
		enable_irq(IRQ_BATT_BRNOUT);
		enable_irq(IRQ_VDDD_BRNOUT);
#ifndef CONFIG_ARCH_MX23
		enable_irq(IRQ_VDDA_BRNOUT);
#endif
		if (no_battery)
			enable_irq(IRQ_VDDIO_BRNOUT);
#ifndef CONFIG_ARCH_MX28
		enable_irq(IRQ_VDD18_BRNOUT);
#endif
		enable_irq(IRQ_VDD5V_DROOP);

	}
#endif

#ifdef CONFIG_MXS_VBUS_CURRENT_DRAW
	if (((__raw_readl(REGS_POWER_BASE + HW_POWER_5VCTRL) &
		BM_POWER_5VCTRL_CHARGE_4P2_ILIMIT) == 0x20000)
		&& ((__raw_readl(REGS_POWER_BASE + HW_POWER_5VCTRL) &
		BM_POWER_5VCTRL_PWD_CHARGE_4P2) == 0)) {
#ifdef CONFIG_USB_GADGET
		printk(KERN_INFO "USB GADGET exist,wait USB enum done...\r\n");
		while (((__raw_readl(REGS_POWER_BASE + HW_POWER_5VCTRL)
			& BM_POWER_5VCTRL_CHARGE_4P2_ILIMIT) == 0x20000) &&
			((__raw_readl(REGS_POWER_BASE + HW_POWER_5VCTRL) &
			BM_POWER_5VCTRL_PWD_CHARGE_4P2) == 0))
			;
#else
		printk(KERN_INFO "USB GADGET not exist,\
		release current limit and let CPU clock up...\r\n");
#endif
	}
	cpu = clk_get(NULL, "cpu");
	pll0 = clk_get(NULL, "ref_cpu");
	clk_set_parent(cpu, pll0);
#endif
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&_charger_wake_lock, WAKE_LOCK_SUSPEND, "mxs-charger");
#endif
	return platform_driver_register(&mxs_batdrv);
}

static void __exit mxs_bat_exit(void)
{
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_destroy(&_charger_wake_lock);
#endif
	platform_driver_unregister(&mxs_batdrv);
}
#ifdef CONFIG_MXS_VBUS_CURRENT_DRAW
	fs_initcall(mxs_bat_init);
#else
	module_init(mxs_bat_init);
#endif
module_exit(mxs_bat_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Steve Longerbeam <stevel@embeddedalley.com>");
MODULE_DESCRIPTION("Linux glue to MXS battery state machine");
