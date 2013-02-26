/*-----------------------------------------------------------------------------
 * FILE NAME : drivers/rtc/rtc-wakers.c
 *
 * PURPOSE : suspend wakers
 *
 * Copyright 1999 - 2010 UniData Communication Systems, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * NOTES: N/A
 *---------------------------------------------------------------------------*/

/*_____________________ Include Header ______________________________________*/
#include <linux/ctype.h>
#include <linux/module.h>
#include <linux/rtc.h>
#include <linux/err.h>
#include <linux/slab.h>
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif

/*_____________________ Constants Definitions _______________________________*/
struct waker {
	struct list_head    link;
	char                name[24];
	unsigned long       second;
	unsigned long       expired;
};

/*_____________________ Type definitions ____________________________________*/

/*_____________________ Imported Variables __________________________________*/

/*_____________________ Variables Definitions _______________________________*/

/*_____________________ Local Declarations __________________________________*/
static DEFINE_SPINLOCK(list_lock);
static LIST_HEAD(list_wakers);
static struct wake_lock wakers_wake_lock;

static ssize_t rtc_sysfs_show_wakers(struct rtc_device *rtc,
		struct device_attribute *attr, char *buf);
static ssize_t rtc_sysfs_store_wakers(struct rtc_device *rtc,
		struct device_attribute *attr, const char *buf, size_t n);

static const DEVICE_ATTR(wakers, S_IRUGO | S_IWUSR,
		rtc_sysfs_show_wakers, rtc_sysfs_store_wakers);

static void _alarm_triggered_func(void *p);
static struct rtc_task alarm_rtc_task = {
	.func = _alarm_triggered_func
};

/*_____________________ internal functions __________________________________*/
static void
_alarm_triggered_func(void *p)
{
	struct rtc_device *rtc = (struct rtc_device *)p;
	if (!(rtc->irq_data & RTC_AF))
		return;
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_timeout(&wakers_wake_lock, 1 * HZ);
#endif
}

static int
_print_waker(char *buf, struct waker *waker, unsigned long now)
{
	struct rtc_time tm;

	rtc_time_to_tm(waker->expired, &tm);

	return sprintf(buf, "%s\t%d\t%d/%d/%d %02d:%02d:%02d\t%d\n",
			waker->name,
			waker->second,
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			tm.tm_hour, tm.tm_min, tm.tm_sec,
			(int)(waker->expired - now));
}

static int
_cmp_rtc_time(struct rtc_time *tm1, struct rtc_time *tm2)
{
	int *t1 = &tm1->tm_sec;
	int *t2 = &tm2->tm_sec;
	int i, t;

	for (i = 5; i >= 0; i--) {
		t = t1[i] - t2[i];
		if (t)
			return t;
	}

	return 0;
}

static int
_cmp_time(unsigned long time1, unsigned long time2)
{
	return (int)(time1 - time2);
}

static unsigned long
_get_now(struct rtc_device *rtc)
{
	struct rtc_time tm;
	unsigned long now;

	rtc_read_time(rtc, &tm);
	rtc_tm_to_time(&tm, &now);

	return now;
}

static struct waker *
_get_waker(struct rtc_device *rtc, const char *name)
{
	struct waker *waker, *nwaker;

	list_for_each_entry_safe(waker, nwaker, &list_wakers, link) {
		if (!strcmp(waker->name, name))
			return waker;
	}

	return NULL;
}

static int
_get_alarm_time(struct rtc_device *rtc, unsigned long now)
{
	struct waker *waker, *nwaker;
	int diff = -10, min = 10000;

	list_for_each_entry_safe(waker, nwaker, &list_wakers, link) {
		diff = _cmp_time(waker->expired, now);
		if (diff <= -5) {
			list_del(&waker->link);
			kfree(waker);
		} else {
			if (diff < min)
				min = diff;
		}
	}

	if (min < 2)
		min = now + 2;
	else
		min = now + min;

	return min;
}

static int
_parse_waker(const char *buf, char **name, long *second)
{
	char *p = buf, *n = NULL;

	int len = 0;
	int ret = -1;

	*name = NULL;
	*second = 0;

	while (*p && !isspace(*p) && (*p != '\n'))
		p++;

	len = p - buf;
	if (!len)
		return -1;

	n = kzalloc(len + 1, GFP_ATOMIC);
	memcpy(n, buf, len);

	while (isspace(*p) || (*p == 'n'))
		p++;

	if (!*p) {
		ret = 0;
		goto _end_parse;
	}

	*second = simple_strtol(p, NULL, 0);
	ret = 0;

_end_parse:
	if (ret)
		kfree(n);
	else
		*name = n;

	return ret;
}

static void
_waker_set(struct rtc_device *rtc, const char *name, long second,
		unsigned long now)
{
	struct waker *waker;

	if (!name)
		return ;

	waker = _get_waker(rtc, name);
	if (second > 0) {
		if (!waker) {
			waker = kzalloc(sizeof(*waker), GFP_ATOMIC);
			strncpy(waker->name, name, sizeof(waker->name) - 1);

			list_add(&waker->link, &list_wakers);
		}

		waker->second = second;
		waker->expired = now + second;
	} else {
		if (waker) {
			list_del(&waker->link);
			kfree(waker);
		}
	}
}

static ssize_t rtc_sysfs_show_wakers(struct rtc_device *rtc,
		struct device_attribute *attr, char *buf)
{
	unsigned long irqflags, now;
	struct rtc_time tm;
	struct waker *waker;
	int len = 0;
	char *p = buf;

	spin_lock_irqsave(&list_lock, irqflags);

	rtc_read_time(rtc, &tm);
	p += sprintf(p, "------------- %d/%d/%d %02d:%02d:%02d -------------\n",
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			tm.tm_hour, tm.tm_min, tm.tm_sec);

	p += sprintf(p, "name\tsecond\talarm_time\t\tremain\n");

	rtc_tm_to_time(&tm, &now);
	list_for_each_entry(waker, &list_wakers, link) {
		p += _print_waker(p, waker, now);
	}

	spin_unlock_irqrestore(&list_lock, irqflags);

	len = p - buf;

	return len;
}

static ssize_t rtc_sysfs_store_wakers(struct rtc_device *rtc,
		struct device_attribute *attr, const char *buf, size_t n)
{
	unsigned long irqflags;
	char *name = NULL;
	long second = 0;
	int ret = -1;
	unsigned long now;

	now = _get_now(rtc);
	spin_lock_irqsave(&list_lock, irqflags);
	ret = _parse_waker(buf, &name, &second);
	if (ret < 0 || second < 0)
		goto bad_name;

	_waker_set(rtc, name, second, now);

	kfree(name);

bad_name:
	spin_unlock_irqrestore(&list_lock, irqflags);
	return n;
}

/*_____________________ Program Body ________________________________________*/
int wakers_set_alarm(struct rtc_device *rtc)
{
	struct rtc_wkalrm alarm;
	unsigned long irqflags = 0;
	unsigned long ret = 0;
	unsigned long now;

	now = _get_now(rtc);
	spin_lock_irqsave(&list_lock, irqflags);
	ret = _get_alarm_time(rtc, now);
	spin_unlock_irqrestore(&list_lock, irqflags);

	if (ret) {
		rtc_time_to_tm(ret, &alarm.time);
		alarm.enabled = 0;
		alarm.pending = 0;
		alarm.time.tm_wday = -1;
		alarm.time.tm_yday = -1;
		alarm.time.tm_isdst = -1;

		rtc_set_alarm(rtc, &alarm);
	}

	return ret;
}
EXPORT_SYMBOL(wakers_set_alarm);

int wakers_register(struct rtc_device *rtc)
{
	int err;

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&wakers_wake_lock, WAKE_LOCK_SUSPEND, "wakers");
#endif

	alarm_rtc_task.private_data = rtc;
	err = rtc_irq_register(rtc, &alarm_rtc_task);
	if (err) {
		dev_err(&rtc->dev, "failed to register rtc irq\n");
#ifdef CONFIG_HAS_WAKELOCK
		wake_lock_destroy(&wakers_wake_lock);
#endif

		return err;
	}

	return device_create_file(&rtc->dev, &dev_attr_wakers);
}
EXPORT_SYMBOL(wakers_register);

void wakers_unregister(struct rtc_device *rtc)
{
	rtc_irq_unregister(rtc, &alarm_rtc_task);
	device_remove_file(&rtc->dev, &dev_attr_wakers);
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_destroy(&wakers_wake_lock);
#endif
}
EXPORT_SYMBOL(wakers_unregister);

