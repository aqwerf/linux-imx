#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <linux/uaccess.h>
#include <linux/rtc.h>

#include "ddi_bc_internal.h"
#include "mxs-log.h"

int mxs_log_open(char *filename)
{
	int fd;
	int ret = -1;

	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);

	fd = sys_open(filename, O_WRONLY|O_CREAT|O_APPEND, 0644);
	if (fd >= 0) {
		sys_close(fd);
		ret = 0;
	}

	set_fs(old_fs);

	return ret;
}

void mxs_log_write(char *filename, char *str)
{
	struct file *file;
	loff_t pos = 0;
	int fd;

	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);

	fd = sys_open(filename, O_WRONLY|O_CREAT|O_APPEND, 0644);
	if (fd >= 0) {
#if 0
		sys_write(fd, str, strlen(str));
#endif
		file = fget(fd);
		if (file) {
			vfs_write(file, str, strlen(str), &pos);
			fput(file);
		}
		sys_close(fd);
	}

	set_fs(old_fs);
}

#define _FILE_NAME "/udp-flash/log_charge.txt"
#define _LOG_TITLE "Time,Mode,Count,Voltage,Status(core),Status(ui),Broken,DieTemp"

/* for state backup */
static ddi_bc_State_t _state = -1;
/* for start time(s) */
static __kernel_time_t _time;
static __kernel_time_t _gi_time;
/* for enable log and auto timeout(s) */
static int _timeout;

void mxs_log_charge_set_timeout(int timeout)
{
	_timeout = timeout;
}

int mxs_log_charge_get_timeout(void)
{
	return _timeout;
}

void mxs_log_charge_update(int mode) /* 0 : event, 1 : auto */
{
	static unsigned long count;
	char log[100];
	struct timespec ts;
	struct rtc_time tm;
	int16_t i16Low;
	int16_t i16High;

	if (!_timeout || mxs_log_open(_FILE_NAME) < 0)
		return;

	getnstimeofday(&ts);

	if (_state == -1) {
		_time = ts.tv_sec;
		_gi_time = ts.tv_sec;
		rtc_time_to_tm(ts.tv_sec, &tm);
		sprintf(log, "%d-%02d-%02d %02d:%02d:%02d,,,,,,\n%s\n",
				tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
				tm.tm_hour, tm.tm_min, tm.tm_sec,
				_LOG_TITLE);
		mxs_log_write(_FILE_NAME, log);
	}

	if (!mode && _state == ddi_bc_GetState())
		return;

	count++;
	_state = ddi_bc_GetState();

	if (ts.tv_sec - _gi_time < _timeout)
		return;

	ddi_bc_hwGetDieTemp(&i16Low, &i16High);

	sprintf(log, "%ld,%d,%d,%d.%03d,%d,%d,%d,%d\n",
			(ts.tv_sec - _time),
			mode,
			count,
			(ddi_power_GetBattery()/1000),
			(ddi_power_GetBattery()%1000),
			_state,
			mxs_bat_get_ui_status(),
			ddi_bc_GetBrokenReason(),
			(i16High - 5)); /* remove margin */

	mxs_log_write(_FILE_NAME, log);
	_gi_time = ts.tv_sec;
	count = 0;
}

