/*-----------------------------------------------------------------------------
 * FILE NAME : lcd_st7789s.c
 *
 * PURPOSE : ST7789S 240RGB x 320 dot 262K Color
 *	     with Frame Memory Single-Chip TFT Controller/Driver
 *
 * Copyright (c) 2013 INCOM CO., Ltd.
 * All right reserved.
 *
 * This software is confidential and proprietary to UniData
 * Communication Systems, Inc. No Part of this software may
 * be reproduced, stored, transmitted, disclosed or used in any
 * form or by any means other than as expressly provide by the
 * written license agreement between UniData Communication
 * Systems and its licensee.
 *---------------------------------------------------------------------------*/

/*_____________________ Include Header ______________________________________*/

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/notifier.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>

#include <mach/device.h>
#include <mach/lcdif.h>
#include <mach/regs-pwm.h>
#include <mach/system.h>

#include "loading.h"

/*_____________________ Constants Definitions _______________________________*/

#define REGS_PWM_BASE IO_ADDRESS(PWM_PHYS_ADDR)

#define _H_ACTIVE		240
#define _V_ACTIVE		320

#define _DATA			0
#define _CMD			1

#define _lcdif_write(reg, data)	__raw_writel(data, REGS_LCDIF_BASE + reg)
#define _lcdif_read(reg)	__raw_readl(REGS_LCDIF_BASE + reg)

/*_____________________ Type definitions ____________________________________*/

/*_____________________ Imported Variables __________________________________*/

/*_____________________ Variables Definitions _______________________________*/

atomic_t _init_panel;

static struct mxs_platform_bl_data _bl_data;
static struct clk *_lcd_clk;
static struct clk *_pwm_clk;

/*_____________________ Local Declarations __________________________________*/

/*_____________________ internal functions __________________________________*/

static void _lcd_panel_multi_write(int w, unsigned short *p)
{
	int i;
	_lcdif_write(HW_LCDIF_CTRL1_CLR,
		     BM_LCDIF_CTRL1_BYTE_PACKING_FORMAT);
	_lcdif_write(HW_LCDIF_CTRL1_SET,
		     BF_LCDIF_CTRL1_BYTE_PACKING_FORMAT(0x3));

	_lcdif_write(HW_LCDIF_CTRL_CLR,
		     BM_LCDIF_CTRL_LCDIF_MASTER |
		     BM_LCDIF_CTRL_RUN);

	_lcdif_write(HW_LCDIF_TRANSFER_COUNT,
		     BF_LCDIF_TRANSFER_COUNT_V_COUNT(1) |
		     BF_LCDIF_TRANSFER_COUNT_H_COUNT(w));

	_lcdif_write(HW_LCDIF_CTRL_SET, BM_LCDIF_CTRL_DATA_SELECT);

	_lcdif_write(HW_LCDIF_CTRL_SET, BM_LCDIF_CTRL_RUN);

	for (i = 0; i < w; i++) {
		while (_lcdif_read(HW_LCDIF_STAT) & BM_LCDIF_STAT_LFIFO_FULL)
			;
		_lcdif_write(HW_LCDIF_DATA, p ? *p++ : 0);
	}

	while (_lcdif_read(HW_LCDIF_CTRL) & BM_LCDIF_CTRL_RUN)
		;

	_lcdif_write(HW_LCDIF_CTRL1_CLR, BM_LCDIF_CTRL1_CUR_FRAME_DONE_IRQ);
}

static void _lcd_write(int is_cmd, int data)
{
	_lcdif_write(HW_LCDIF_CTRL_CLR,
			BM_LCDIF_CTRL_LCDIF_MASTER |
			BM_LCDIF_CTRL_RUN);

	_lcdif_write(HW_LCDIF_TRANSFER_COUNT,
			BF_LCDIF_TRANSFER_COUNT_V_COUNT(1) |
			BF_LCDIF_TRANSFER_COUNT_H_COUNT(1));

	_lcdif_write((is_cmd ? HW_LCDIF_CTRL_CLR : HW_LCDIF_CTRL_SET),
			BM_LCDIF_CTRL_DATA_SELECT);

	_lcdif_write(HW_LCDIF_CTRL_SET, BM_LCDIF_CTRL_RUN);

	while (_lcdif_read(HW_LCDIF_STAT) & BM_LCDIF_STAT_LFIFO_FULL)
		;

	_lcdif_write(HW_LCDIF_DATA, data);

	while (_lcdif_read(HW_LCDIF_CTRL) & BM_LCDIF_CTRL_RUN)
		;

	_lcdif_write(HW_LCDIF_CTRL1_CLR, BM_LCDIF_CTRL1_CUR_FRAME_DONE_IRQ);
}

static void _lcd_set_range(int x, int y, int w, int h)
{
	int i;
	_lcd_write(_CMD, 0x2a);	/* CASET */
	_lcd_write(_DATA, (x & 0xff00) >> 8);
	_lcd_write(_DATA, (x & 0x00ff));
	i = x + w - 1;
	_lcd_write(_DATA, (i & 0xff00) >> 8);
	_lcd_write(_DATA, (i & 0x00ff));

	_lcd_write(_CMD, 0x2b);	/* RASET */
	_lcd_write(_DATA, (y & 0xff00) >> 8);
	_lcd_write(_DATA, (y & 0x00ff));
	i = y + h - 1;
	_lcd_write(_DATA, (i & 0xff00) >> 8);
	_lcd_write(_DATA, (i & 0x00ff));

	_lcd_write(_CMD, 0x2C);
}

static struct timer_list _timer;
static int _init_once;
static int _loading;

static void _draw_loading_handler(unsigned long data)
{
	static int pos;
	const unsigned short *p;
	int i;

	int x = 240 / 2 - 32 / 2;
	int y = 320 - 100;

	p = loading[pos];

	_lcd_set_range(x, y, 32, 32);
	for (i = 0; i < 32*32; i++)
		_lcd_write(_DATA, *p++);

	if (++pos >= 8)
		pos = 0;

	del_timer(&_timer);
	if (_loading) {
		_timer.expires = jiffies + msecs_to_jiffies(100);
		add_timer(&_timer);
	}
}

static int splash_show(struct device *dev,
		       struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", _loading);
}

static int splash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	if (len < 1)
		return -EINVAL;

	if (strnicmp(buf, "off", 3) == 0 || strnicmp(buf, "0", 1) == 0) {
		if (_loading) {
			_loading = 0;
			del_timer(&_timer);
			_draw_loading_handler(0x1000);
		}
	} else {
		return -EINVAL;
	}

	return len;
}

static DEVICE_ATTR(splash, 0644,
		   splash_show,
		   splash_store);

static void _loading_icon_start(void)
{
	struct platform_device *pdev;
	_loading = 1;
	pdev = mxs_get_device("mxs-fb", 0);
	device_create_file(&(pdev->dev), &dev_attr_splash);

	init_timer(&_timer);
	_timer.function = _draw_loading_handler;
	_timer.expires = jiffies + msecs_to_jiffies(100);
	add_timer(&_timer);
}

static void force_lcd_update(dma_addr_t phys)
{
	int i;
	short *p = phys_to_virt(phys);
	_lcd_set_range(0, 0, _H_ACTIVE, _V_ACTIVE);
	for (i = 0; i < 320 * 240; i++)
		_lcd_write(_DATA, *p++);
}

static int _lcd_panel_init(void)
{
	_lcd_write(_CMD, 0x11);	/* SLPOUT(11h): Sleep Out */
	mdelay(5);		/* Delay 120ms */

	/* ST7789S Frame rate setting */
	_lcd_write(_CMD, 0xb2);	/* PORCTRL(B2h): Porch Setting */
	_lcd_write(_DATA, 0x0c);
	_lcd_write(_DATA, 0x0c);
	_lcd_write(_DATA, 0x00);
	_lcd_write(_DATA, 0x33);
	_lcd_write(_DATA, 0x33);
	_lcd_write(_CMD, 0xb7);	/* GCTRL(B7h): Gate Control */
	_lcd_write(_DATA, 0x35);
	/* ST7789S Power setting */
	_lcd_write(_CMD, 0xbb);	 /* VCOMS(BBh): VCOM Setting */
	_lcd_write(_DATA, 0x38);
	_lcd_write(_CMD, 0xc0);	 /* LCMCTRL(C0h): LCM Control */
	_lcd_write(_DATA, 0x2c);
	_lcd_write(_CMD, 0xc2);	/* VDVVRHEN(C2h): VDV and VRH Command Enable */
	_lcd_write(_DATA, 0x01);

	_lcd_write(_CMD, 0xc0);	/* LCMCTRL(C0h): LCM Control */
	_lcd_write(_DATA, 0x2c);
	_lcd_write(_CMD, 0xc2);	/* VDVVRHEN(C2h): VDV and VRH Command Enable */
	_lcd_write(_DATA, 0x01);

	_lcd_write(_CMD, 0xc3);	 /* VRHS(C3h): VRH Set */
	_lcd_write(_DATA, 0x18);

	_lcd_write(_CMD, 0xc6);	/* FRCTRL2(C6h): Frame Rate Control */
	_lcd_write(_DATA, 0x0B); /* 69Hz */

	_lcd_write(_CMD, 0xc4);	/* VDVS(C4h): VDV Set */
	_lcd_write(_DATA, 0x20);
	_lcd_write(_CMD, 0xc6);	 /* FRCTRL2(C6h): Frame Rate Control */
	_lcd_write(_DATA, 0xEB); /* 69Hz, column inversion */
	_lcd_write(_CMD, 0xca);	 /* REGSEL2(CAh): Register Value Selection 2 */
	_lcd_write(_DATA, 0x0f);
	_lcd_write(_CMD, 0xc8);	/* REGSEL1(C8h): Register Value Selection 1 */
	_lcd_write(_DATA, 0x08);
	_lcd_write(_CMD, 0x55);	/* WRCACE(55h): Brightness and Color */
	_lcd_write(_DATA, 0x90); /* medium color enhancement */
	_lcd_write(_CMD, 0xd0);	 /* PWCTRL(D0h): Power Control */
	_lcd_write(_DATA, 0xa4);
	_lcd_write(_DATA, 0xa1);
	/* ST7789S gamma setting */
	_lcd_write(_CMD, 0xe0);	 /* PVGAMCTRL(E0h) */
	_lcd_write(_DATA, 0x00);
	_lcd_write(_DATA, 0x00);
	_lcd_write(_DATA, 0x02);
	_lcd_write(_DATA, 0x07);
	_lcd_write(_DATA, 0x12);
	_lcd_write(_DATA, 0x2A);
	_lcd_write(_DATA, 0x37);
	_lcd_write(_DATA, 0x33);
	_lcd_write(_DATA, 0x47);
	_lcd_write(_DATA, 0x1A);
	_lcd_write(_DATA, 0x18);
	_lcd_write(_DATA, 0x15);
	_lcd_write(_DATA, 0x18);
	_lcd_write(_DATA, 0x1A);

	_lcd_write(_CMD, 0xe1);	/* NVGAMCTRL(E1h) */
	_lcd_write(_DATA, 0x00);
	_lcd_write(_DATA, 0x00);
	_lcd_write(_DATA, 0x02);
	_lcd_write(_DATA, 0x07);
	_lcd_write(_DATA, 0x12);
	_lcd_write(_DATA, 0x2A);
	_lcd_write(_DATA, 0x37);
	_lcd_write(_DATA, 0x43);
	_lcd_write(_DATA, 0x47);
	_lcd_write(_DATA, 0x1A);
	_lcd_write(_DATA, 0x18);
	_lcd_write(_DATA, 0x15);
	_lcd_write(_DATA, 0x18);
	_lcd_write(_DATA, 0x1A);

	_lcd_write(_CMD, 0x3A);	/* COLMOD(3Ah): Interface Pixel Format */
	_lcd_write(_DATA, 0x55); /* 16bits RGB */
	_lcd_write(_CMD, 0x29);	 /* DISPON(29h): Display On */
	_lcd_write(_CMD, 0x2C);	/* RAMWR(2Ch): Memory Write */
	return 0;
}

static int _lcd_panel_power(int set, dma_addr_t phys)
{
	if (atomic_read(&_init_panel) == set)
		return 0;

	if (set) {
		_lcd_write(_CMD, 0x11); /* SLPOUT(11h): Sleep Out */
		mdelay(5);
		force_lcd_update(phys);
	} else {
		atomic_set(&_init_panel, set);
		_lcd_write(_CMD, 0x10); /* SLPIN(10h): Sleep in */
	}

	return 0;
}

static void _lcdif_setup_system_panel(void)
{
	_lcdif_write(HW_LCDIF_CTRL_CLR, BM_LCDIF_CTRL_CLKGATE);
	_lcdif_write(HW_LCDIF_CTRL_SET, BM_LCDIF_CTRL_SFTRST);
	udelay(10);
	_lcdif_write(HW_LCDIF_CTRL_CLR, BM_LCDIF_CTRL_CLKGATE |
		     BM_LCDIF_CTRL_SFTRST |
		     BM_LCDIF_CTRL_LCDIF_MASTER |
		     BM_LCDIF_CTRL_BYPASS_COUNT);

	_lcdif_write(HW_LCDIF_CTRL1_CLR, BM_LCDIF_CTRL1_MODE86 |
		     BM_LCDIF_CTRL1_BUSY_ENABLE);

	_lcdif_write(HW_LCDIF_VDCTRL0_CLR,
			BM_LCDIF_VDCTRL0_VSYNC_OEB);

	_lcdif_write(HW_LCDIF_CTRL_CLR,
			BM_LCDIF_CTRL_VSYNC_MODE |
			BM_LCDIF_CTRL_WAIT_FOR_VSYNC_EDGE |
			BM_LCDIF_CTRL_DVI_MODE |
			BM_LCDIF_CTRL_DOTCLK_MODE);

	_lcdif_write(HW_LCDIF_TRANSFER_COUNT,
			BF_LCDIF_TRANSFER_COUNT_V_COUNT(_V_ACTIVE) |
			BF_LCDIF_TRANSFER_COUNT_H_COUNT(_H_ACTIVE));

	_lcdif_write(HW_LCDIF_CTRL,
			BF_LCDIF_CTRL_INPUT_DATA_SWIZZLE(
				BV_LCDIF_CTRL_INPUT_DATA_SWIZZLE__NO_SWAP) |
			BF_LCDIF_CTRL_LCD_DATABUS_WIDTH(
				BV_LCDIF_CTRL_LCD_DATABUS_WIDTH__16_BIT) |
			BF_LCDIF_CTRL_WORD_LENGTH(
				BV_LCDIF_CTRL_WORD_LENGTH__16_BIT));

	_lcdif_write(HW_LCDIF_CTRL1_CLR,
			BM_LCDIF_CTRL1_BYTE_PACKING_FORMAT);
	_lcdif_write(HW_LCDIF_CTRL1_SET,
			BF_LCDIF_CTRL1_BYTE_PACKING_FORMAT(0x3));

	_lcdif_write(HW_LCDIF_TIMING,
			BF_LCDIF_TIMING_CMD_HOLD(1) |
			BF_LCDIF_TIMING_CMD_SETUP(1) |
			BF_LCDIF_TIMING_DATA_HOLD(4) |
			BF_LCDIF_TIMING_DATA_SETUP(4));
}

static int _lcdif_init_panel(struct device *dev, dma_addr_t phys, int memsize,
		struct mxs_platform_fb_entry *pentry)
{
	_lcd_clk = clk_get(NULL, "lcdif");
	if (IS_ERR(_lcd_clk))
		return -1;

	if (clk_enable(_lcd_clk) != 0)
		goto _error;

	if (clk_set_rate(_lcd_clk, 1000000000 / pentry->cycle_time_ns) != 0)
		goto _error;

	_lcdif_setup_system_panel();
	_lcdif_write(HW_LCDIF_CUR_BUF, phys);
	atomic_set(&_init_panel, 1);

	if (!_init_once) {
		/* for booting */
		_init_once = 1;
		_loading_icon_start();
	} else {
#if 1
		/* for external LCD reset */
		_lcdif_write(HW_LCDIF_CTRL1_CLR, BM_LCDIF_CTRL1_RESET);
		udelay(4); /* Shorter than 5us */
		_lcdif_write(HW_LCDIF_CTRL1_SET, BM_LCDIF_CTRL1_RESET);
		mdelay(1);
		_lcdif_write(HW_LCDIF_CTRL1_CLR, BM_LCDIF_CTRL1_RESET);
		mdelay(1);
		_lcdif_write(HW_LCDIF_CTRL1_SET, BM_LCDIF_CTRL1_RESET);
		mdelay(10);

		/* for external LCD */
		_lcd_panel_init();
#else
		_lcd_write(_CMD, 0x11); /* SLPOUT(11h): Sleep Out */
		mdelay(5);
#endif
		force_lcd_update(phys);
	}

	mxs_lcd_set_bl_pdata(pentry->bl_data);
	mxs_lcdif_notify_clients(MXS_LCDIF_PANEL_INIT, pentry);
	return 0;

_error:
	clk_disable(_lcd_clk);
	clk_put(_lcd_clk);
	_lcd_clk = NULL;
	return -1;
}

static void _lcdif_release_panel(struct device *dev,
			  struct mxs_platform_fb_entry *pentry)
{
#if 1
	_lcd_write(_CMD, 0x10); /* SLPIN(10h): Sleep in */
#else
	/* Reset LCD panel signel. */
	_lcdif_write(HW_LCDIF_CTRL1_CLR, BM_LCDIF_CTRL1_RESET);	/* low */
	mdelay(100);
	_lcdif_write(HW_LCDIF_CTRL1_SET, BM_LCDIF_CTRL1_RESET);
#endif

	_lcdif_write(HW_LCDIF_CTRL_CLR, BM_LCDIF_CTRL_LCDIF_MASTER);
	_lcdif_write(HW_LCDIF_CTRL_SET, BM_LCDIF_CTRL_CLKGATE);

	if (_lcd_clk) {
		clk_disable(_lcd_clk);
		clk_put(_lcd_clk);
	}
	mxs_lcdif_notify_clients(MXS_LCDIF_PANEL_RELEASE, pentry);
}

static int _lcdif_blank_panel(int blank)
{
	int ret = 0;

	if (!atomic_read(&_init_panel))
		return ret;

	switch (blank) {
	case FB_BLANK_NORMAL: /* lcd on, backlight on */
	case FB_BLANK_UNBLANK:
		break;
	case FB_BLANK_VSYNC_SUSPEND: /* lcd on, backlight off */
	case FB_BLANK_HSYNC_SUSPEND:
		break;
	case FB_BLANK_POWERDOWN: /* lcd and backlight off */
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static void _lcdif_run_panel(void)
{
	/* do nothing */
}

static void _lcdif_stop_panel(void)
{
	/* do nothing */
}

static int _lcdif_pan_display(dma_addr_t addr)
{
	_lcdif_write(HW_LCDIF_CUR_BUF, addr);

	return 0;
}

static struct mxs_platform_fb_entry fb_entry = {
	.name = "st7789s",
	.x_res = _V_ACTIVE,
	.y_res = _H_ACTIVE,
	.bpp = 16,
	.cycle_time_ns = 10,
	.lcd_type = MXS_LCD_PANEL_SYSTEM,
	.init_panel = _lcdif_init_panel,
	.release_panel = _lcdif_release_panel,
	.blank_panel = _lcdif_blank_panel,
	.run_panel = _lcdif_run_panel,
	.stop_panel = _lcdif_stop_panel,
	.pan_display = _lcdif_pan_display,
	.bl_data = &_bl_data,
};

#define _BL_PWM_PERIODn_SETTINGS			\
		(BF_PWM_PERIODn_CDIV(6) |		\
		BF_PWM_PERIODn_INACTIVE_STATE(2) |	\
		BF_PWM_PERIODn_ACTIVE_STATE(3) |	\
		BF_PWM_PERIODn_PERIOD(100))

static int _bl_init(struct mxs_platform_bl_data *data)
{
	int ret = 0;

	_pwm_clk = clk_get(NULL, "pwm");
	if (IS_ERR(_pwm_clk)) {
		ret = PTR_ERR(_pwm_clk);
		return ret;
	}
	clk_enable(_pwm_clk);
	mxs_reset_block(REGS_PWM_BASE, 1);

	__raw_writel(BF_PWM_ACTIVEn_INACTIVE(0) |
			BF_PWM_ACTIVEn_ACTIVE(0),
			REGS_PWM_BASE + HW_PWM_ACTIVEn(2));
	__raw_writel(_BL_PWM_PERIODn_SETTINGS,
			REGS_PWM_BASE + HW_PWM_PERIODn(2));
	__raw_writel(BM_PWM_CTRL_PWM2_ENABLE,
			REGS_PWM_BASE + HW_PWM_CTRL_SET);

	return 0;
}

static void _bl_free(struct mxs_platform_bl_data *data)
{
	__raw_writel(BF_PWM_ACTIVEn_INACTIVE(0) |
			BF_PWM_ACTIVEn_ACTIVE(0),
			REGS_PWM_BASE + HW_PWM_ACTIVEn(2));
	__raw_writel(_BL_PWM_PERIODn_SETTINGS,
			REGS_PWM_BASE + HW_PWM_PERIODn(2));
	__raw_writel(BM_PWM_CTRL_PWM2_ENABLE, REGS_PWM_BASE + HW_PWM_CTRL_CLR);

	clk_disable(_pwm_clk);
	clk_put(_pwm_clk);
}

static int _bl_set_intensity(struct mxs_platform_bl_data *data,
		struct backlight_device *bd, int suspended)
{
	int intensity = bd->props.brightness;

	if (bd->props.power != FB_BLANK_UNBLANK ||
			bd->props.fb_blank != FB_BLANK_UNBLANK ||
			suspended)
		intensity = 0;

	if (intensity > 100)
		intensity = 100;

	__raw_writel(BF_PWM_ACTIVEn_INACTIVE(intensity) |
			BF_PWM_ACTIVEn_ACTIVE(0),
			REGS_PWM_BASE + HW_PWM_ACTIVEn(2));
	__raw_writel(_BL_PWM_PERIODn_SETTINGS,
			REGS_PWM_BASE + HW_PWM_PERIODn(2));

	return 0;
}

static struct mxs_platform_bl_data _bl_data = {
	.bl_max_intensity = 100,
	.bl_default_intensity = 100,
	.bl_cons_intensity = 100,
	.init_bl = _bl_init,
	.free_bl = _bl_free,
	.set_bl_intensity = _bl_set_intensity,
};

static int __init register_devices(void)
{
	struct platform_device *pdev;
	pdev = mxs_get_device("mxs-fb", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return -ENODEV;

	mxs_lcd_register_entry(&fb_entry, pdev->dev.platform_data);

	return 0;
}

subsys_initcall(register_devices);

/*_____________________ Program Body ________________________________________*/

int canopus_lcdif_dma_send(dma_addr_t addr)
{
	if (!atomic_read(&_init_panel) || addr <= 0)
		return -1;

	while (_lcdif_read(HW_LCDIF_CTRL) & BM_LCDIF_CTRL_RUN)
		;

	_lcdif_write(HW_LCDIF_CUR_BUF, (uint32_t)addr);
	_lcdif_write(HW_LCDIF_CTRL1_CLR,
		     BM_LCDIF_CTRL1_BYTE_PACKING_FORMAT);
	_lcdif_write(HW_LCDIF_CTRL1_SET,
		     BF_LCDIF_CTRL1_BYTE_PACKING_FORMAT(0x3));

	_lcd_set_range(0, 0, _H_ACTIVE, _V_ACTIVE);

	_lcdif_write(HW_LCDIF_CTRL_SET, BM_LCDIF_CTRL_DATA_SELECT);

	_lcdif_write(HW_LCDIF_TRANSFER_COUNT,
		     BF_LCDIF_TRANSFER_COUNT_V_COUNT(_V_ACTIVE) |
		     BF_LCDIF_TRANSFER_COUNT_H_COUNT(_H_ACTIVE));

	_lcdif_write(HW_LCDIF_CTRL_SET, BM_LCDIF_CTRL_LCDIF_MASTER);

	_lcdif_write(HW_LCDIF_CTRL1_SET,
		     BF_LCDIF_CTRL1_BYTE_PACKING_FORMAT(0x0f));

	_lcdif_write(HW_LCDIF_CTRL_SET, BM_LCDIF_CTRL_RUN);

	return 0;
}
EXPORT_SYMBOL(canopus_lcdif_dma_send);

int canopus_lcd_panel_power(int set, dma_addr_t phys)
{
	return _lcd_panel_power(set, phys);
}
EXPORT_SYMBOL(canopus_lcd_panel_power);
