#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <linux/uaccess.h>

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

#define _FILE_NAME	"/udp-flash/log_charge.txt"

/* for state backup */
static ddi_bc_State_t _state = -1;
/* for start time(s) */
static __kernel_time_t _time;
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
	char log[100];
	struct timeval tv;
	int16_t i16Low;
	int16_t i16High;

	if (!_timeout || mxs_log_open(_FILE_NAME) < 0)
		return;

	do_gettimeofday(&tv);

	if (_state == -1) {
		_time = tv.tv_sec;
		mxs_log_write(_FILE_NAME,
				"Time,Mode,Voltage,Status,Broken,DieTemp\n");
	}

	if (!mode && _state == ddi_bc_GetState())
		return;

	_state = ddi_bc_GetState();

	ddi_bc_hwGetDieTemp(&i16Low, &i16High);

	sprintf(log, "%ld,%d,%d.%03d,%d,%d,%d\n",
			(tv.tv_sec - _time),
			mode,
			(ddi_power_GetBattery()/1000),
			(ddi_power_GetBattery()%1000),
			_state,
			ddi_bc_GetBrokenReason(),
			(i16High - 5)); /* remove margin */

	mxs_log_write(_FILE_NAME, log);
}

