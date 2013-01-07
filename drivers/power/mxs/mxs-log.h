#ifndef _MXS_LOG_H
#define _MXS_LOG_H

#define MXS_CANOPUS_LOG_ENABLE

extern void fput(struct file *file);
extern struct file *fget(unsigned int fd);

extern int mxs_log_open(char *filename);
extern void mxs_log_write(char *filename, char *str);

extern void mxs_log_charge_set_timeout(int timeout);
extern int mxs_log_charge_get_timeout(void);
extern void mxs_log_charge_update(int mode);

extern int mxs_bat_get_ui_status(void);

#endif
