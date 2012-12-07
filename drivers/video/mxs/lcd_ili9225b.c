/*-----------------------------------------------------------------------------
 * FILE NAME : lcd_ili9225b.c
 *
 * PURPOSE : ILI9225B a-Si TFT LCD Driver 176RGBx220 Resolution and 262K color
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

#define _LCD_BYD		0
#define _LCD_TCL		1

#define _H_ACTIVE		176
#define _V_ACTIVE		220

#define _DATA			0
#define _CMD			1

#define _lcdif_write(reg, data)	__raw_writel(data, REGS_LCDIF_BASE + reg)
#define _lcdif_read(reg)	__raw_readl(REGS_LCDIF_BASE + reg)

/*_____________________ Type definitions ____________________________________*/

/*_____________________ Imported Variables __________________________________*/

/*_____________________ Variables Definitions _______________________________*/

static int _init_temp;
atomic_t _init_panel;

static struct mxs_platform_bl_data _bl_data;
static struct clk *_lcd_clk;
static struct clk *_pwm_clk;

static int _bl_values[] = {
	0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100
};

static int _bl_power[] = {
	0, 1500, 3600, 6100, 10300, 15500, 74200, 114200, 155200, 190100, 191000
};

static int _lcd_power = 1;

/*_____________________ Local Declarations __________________________________*/

/*_____________________ internal functions __________________________________*/

static void _lcd_panel_set_prepare(int x, int y);

static void
_lcd_panel_multi_write(int w, unsigned short * p)
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
		while (_lcdif_read(HW_LCDIF_STAT) & BM_LCDIF_STAT_LFIFO_FULL);
		_lcdif_write(HW_LCDIF_DATA, *p++);
	}

	while (_lcdif_read(HW_LCDIF_CTRL) & BM_LCDIF_CTRL_RUN);

	_lcdif_write(HW_LCDIF_CTRL1_CLR, BM_LCDIF_CTRL1_CUR_FRAME_DONE_IRQ);
}

static struct timer_list _timer;
static int _loading;

static void _draw_loading_handler(unsigned long data)
{
	static int pos;
	unsigned short *p;
	
	int x = 176 / 2 - 32 / 2;
	int y = 150;
	int ye = y + 32;

	p = loading[pos];
	for (; y < ye; y++) {
		_lcd_panel_set_prepare(x, y);
		_lcd_panel_multi_write(32, p);
		p += 32;
	}
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
	_loading = 1;
	struct platform_device *pdev;
	pdev = mxs_get_device("mxs-fb", 0);
	device_create_file(&(pdev->dev), &dev_attr_splash);
	
	init_timer(&_timer);
	_timer.function = _draw_loading_handler;
	_timer.expires = jiffies + msecs_to_jiffies(100);
	add_timer(&_timer);
}

static void
_lcd_panel_data_write(int is_cmd, int data)
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

static void
_lcd_panel_pair_write(int reg, int data)
{
	_lcd_panel_data_write(_CMD, reg);
	_lcd_panel_data_write(_DATA, data);
}

static void
_lcd_panel_pixel_write(int color)
{
	_lcd_panel_data_write(_DATA, color);
}

static void
_lcd_panel_set_prepare(int x, int y)
{
	_lcd_panel_pair_write(0x20, x & 0xff);
	_lcd_panel_pair_write(0x21, y & 0xff);

	_lcd_panel_data_write(_CMD, 0x22);
}

static void
_lcd_panel_set_display(int is_on)
{
	if (is_on)
		_lcd_panel_pair_write(0x0007, 0x1017);
	else
		_lcd_panel_pair_write(0x0007, 0x0000);
}

static void
_lcd_panel_init_byd(void)
{
	/* byd lcd init */
	_lcd_panel_pair_write(0x0001, 0x011C); /* set SS and NL bit */
	_lcd_panel_pair_write(0x0002, 0x0100); /* set 1 line inversion */
	_lcd_panel_pair_write(0x0003, 0x1030); /* set GRAM write direction
						  and BGR=1. */
	_lcd_panel_pair_write(0x0008, 0x0808); /* set BP and FP */
	_lcd_panel_pair_write(0x000C, 0x0000); /* RGB interface setting
						  R0Ch=0x0110 for RGB 18Bit and
						  R0Ch=0111for RGB16Bit */
	_lcd_panel_pair_write(0x000F, 0x0801); /* Set frame rate */
	/* Power On sequence */
	mdelay(50);
	_lcd_panel_pair_write(0x0010, 0x0800); /* Set SAP,DSTB,STB */
	_lcd_panel_pair_write(0x0011, 0x1038); /* Set APON,PON,AON,VCI1EN,VC */
	mdelay(50);
	_lcd_panel_pair_write(0x0012, 0x1121); /* Internal reference
						  voltage= Vci; */
	_lcd_panel_pair_write(0x0013, 0x0062); /* Set GVDD */
	_lcd_panel_pair_write(0x0014, 0x5379); /* Set VCOMH/VCOML voltage */
	/* Set GRAM area */
	_lcd_panel_pair_write(0x0030, 0x0000);
	_lcd_panel_pair_write(0x0031, 0x00DB);
	_lcd_panel_pair_write(0x0032, 0x0000);
	_lcd_panel_pair_write(0x0033, 0x0000);
	_lcd_panel_pair_write(0x0034, 0x00DB);
	_lcd_panel_pair_write(0x0035, 0x0000);
	/* Adjust the Gamma Curve */
	_lcd_panel_pair_write(0x0050, 0x0000);
	_lcd_panel_pair_write(0x0051, 0x010B);
	_lcd_panel_pair_write(0x0052, 0x0B04);
	_lcd_panel_pair_write(0x0053, 0x0403);
	_lcd_panel_pair_write(0x0054, 0x040B);
	_lcd_panel_pair_write(0x0055, 0x0B01);
	_lcd_panel_pair_write(0x0056, 0x0000);
	_lcd_panel_pair_write(0x0057, 0x0304);
	_lcd_panel_pair_write(0x0058, 0x0000);
	_lcd_panel_pair_write(0x0059, 0x0000);
	mdelay(50);
	_lcd_panel_pair_write(0x0007, 0x1017);
	_lcd_panel_pair_write(0x0037, 0x0000);
	_lcd_panel_pair_write(0x0036, 0x00b0);
	_lcd_panel_pair_write(0x0039, 0x0000);
	_lcd_panel_pair_write(0x0038, 0x00dc);
	_lcd_panel_pair_write(0x0020, 0x0000); /* Set GRAM Address */
	_lcd_panel_pair_write(0x0021, 0x0000); /* Set GRAM Address */
}

static void
_lcd_panel_init_tcl(void)
{
	_lcd_panel_pair_write(0x0000, 0x0001);
	_lcd_panel_pair_write(0x0001, 0x011C);
	_lcd_panel_pair_write(0x0002, 0x0100);	/* set 1 line inversion */
	_lcd_panel_pair_write(0x0003, 0x1030);
	_lcd_panel_pair_write(0x0007, 0x0000);
	_lcd_panel_pair_write(0x0008, 0x0808);
	_lcd_panel_pair_write(0x000B, 0x0100);
	_lcd_panel_pair_write(0x000C, 0x0000);
	_lcd_panel_pair_write(0x000F, 0x0d01);	/* Oscillator Control */
	/* power On sequence */
	_lcd_panel_pair_write(0x0010, 0x0000);
	mdelay(10);
	_lcd_panel_pair_write(0x0011, 0x0008);
	_lcd_panel_pair_write(0x0012, 0x6332);
	mdelay(40);
	_lcd_panel_pair_write(0x0013, 0x0000);
	mdelay(40);
	_lcd_panel_pair_write(0x0010, 0x0A00);
	mdelay(10);
	_lcd_panel_pair_write(0x0011, 0x1038);
	_lcd_panel_pair_write(0x0012, 0x6332);
	mdelay(40);
	_lcd_panel_pair_write(0x0013, 0x0068);
	mdelay(40);
	_lcd_panel_pair_write(0x0014, 0x4a60);
	/* Gamma Control Setting */
	_lcd_panel_pair_write(0x0050, 0x0400);
	_lcd_panel_pair_write(0x0051, 0x060B);
	_lcd_panel_pair_write(0x0052, 0x0C0A);
	_lcd_panel_pair_write(0x0053, 0x0105);
	_lcd_panel_pair_write(0x0054, 0x0A0C);
	_lcd_panel_pair_write(0x0055, 0x0B06);
	_lcd_panel_pair_write(0x0056, 0x0004);
	_lcd_panel_pair_write(0x0057, 0x0501);
	_lcd_panel_pair_write(0x0058, 0x0E00);
	_lcd_panel_pair_write(0x0059, 0x000E);
	/* set GRAM area */
	_lcd_panel_pair_write(0x0020, 0x0000);
	_lcd_panel_pair_write(0x0021, 0x0000);
	_lcd_panel_pair_write(0x0030, 0x0000);
	_lcd_panel_pair_write(0x0031, 0x00DB);
	_lcd_panel_pair_write(0x0032, 0x0000);
	_lcd_panel_pair_write(0x0033, 0x0000);
	_lcd_panel_pair_write(0x0034, 0x00DB);
	_lcd_panel_pair_write(0x0035, 0x0000);
	_lcd_panel_pair_write(0x0036, 0x00AF);
	_lcd_panel_pair_write(0x0037, 0x0000);
	_lcd_panel_pair_write(0x0038, 0x00DB);
	_lcd_panel_pair_write(0x0039, 0x0000);
	mdelay(50);
	_lcd_panel_pair_write(0x0007, 0x1017);
}

static int
_lcd_panel_init(int type)
{
	int ret = 0;

	switch (type) {
	case _LCD_BYD:
		_lcd_panel_init_byd();
		break;
	case _LCD_TCL:
		_lcd_panel_init_tcl();
		break;
	default:
		ret = -1;
		break;
	}

	return ret;
}

static int
_lcd_panel_power(int set, dma_addr_t phys)
{
	if (!_init_temp)
		return 0;

	_lcd_power = set;

	if (atomic_read(&_init_panel) == set)
		return 0;

	if (set) {
		_lcd_panel_pair_write(0x0010, 0x0A00);

		/* for external LCD reset */
		_lcdif_write(HW_LCDIF_CTRL1_CLR, BM_LCDIF_CTRL1_RESET);
		mdelay(10);
		_lcdif_write(HW_LCDIF_CTRL1_SET, BM_LCDIF_CTRL1_RESET);
		mdelay(50);

		/* for external LCD */
		_lcd_panel_init(_LCD_BYD);
		_lcd_panel_set_prepare(0, 0);
		mdelay(2);

		atomic_set(&_init_panel, set);

		ili9225b_lcdif_dma_send(phys);
	} else {
		atomic_set(&_init_panel, set);

		_lcd_panel_pair_write(0x0007, 0x0000);
		mdelay(50);
		_lcd_panel_pair_write(0x0011, 0x0007);
		mdelay(50);
		_lcd_panel_pair_write(0x0010, 0x0A02);
	}

	return 0;
}

static void
_lcdif_setup_system_panel(void)
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
			BF_LCDIF_TIMING_CMD_HOLD(0x1) |
			BF_LCDIF_TIMING_CMD_SETUP(0x1) |
			BF_LCDIF_TIMING_DATA_HOLD(0x3) |
			BF_LCDIF_TIMING_DATA_SETUP(0x3));
}

static int
_lcdif_dma_init(struct device *dev, dma_addr_t phys, int memsize)
{
	_lcdif_write(HW_LCDIF_CTRL_SET, BM_LCDIF_CTRL_LCDIF_MASTER);
	_lcdif_write(HW_LCDIF_CTRL1_SET,
		     BF_LCDIF_CTRL1_BYTE_PACKING_FORMAT(0x0f));
	_lcdif_write(HW_LCDIF_CUR_BUF, phys);

	return 0;
}

static int
_lcdif_init_panel(struct device *dev, dma_addr_t phys, int memsize,
		struct mxs_platform_fb_entry *pentry)
{
	_lcd_clk = clk_get(NULL, "lcdif");
	if (IS_ERR(_lcd_clk))
		return -1;

	if (clk_enable(_lcd_clk) != 0) {
		clk_put(_lcd_clk);
		return -1;
	}

	if (clk_set_rate(_lcd_clk, 1000000000 / pentry->cycle_time_ns) != 0) {
		clk_disable(_lcd_clk);
		clk_put(_lcd_clk);
		return -1;
	}

	_lcdif_setup_system_panel();
	_lcdif_write(HW_LCDIF_CUR_BUF, phys);
	atomic_set(&_init_panel, 1);

	if (_init_temp) {
		/* for external LCD reset */
		_lcdif_write(HW_LCDIF_CTRL1_CLR, BM_LCDIF_CTRL1_RESET);
		mdelay(10);
		_lcdif_write(HW_LCDIF_CTRL1_SET, BM_LCDIF_CTRL1_RESET);
		mdelay(50);
		
		/* for external LCD */
		_lcd_panel_init(_LCD_BYD);

		ili9225b_lcdif_dma_send(phys);
	} else {
		_loading_icon_start();
	}

	mxs_lcd_set_bl_pdata(pentry->bl_data);
	mxs_lcdif_notify_clients(MXS_LCDIF_PANEL_INIT, pentry);

	_init_temp = 1;
	return 0;
}

static void
_lcdif_release_panel(struct device *dev,
			  struct mxs_platform_fb_entry *pentry)
{
	_lcdif_write(HW_LCDIF_CTRL_CLR, BM_LCDIF_CTRL_LCDIF_MASTER);
	_lcdif_write(HW_LCDIF_CTRL_SET, BM_LCDIF_CTRL_CLKGATE);
	
	clk_disable(_lcd_clk);
	clk_put(_lcd_clk);

	mxs_lcdif_notify_clients(MXS_LCDIF_PANEL_RELEASE, pentry);
}

static int
_lcdif_blank_panel(int blank)
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

static void
_lcdif_run_panel(void)
{
	/* do nothing */
}

static void
_lcdif_stop_panel(void)
{
	/* do nothing */
}

static int
_lcdif_pan_display(dma_addr_t addr)
{
	_lcdif_write(HW_LCDIF_CUR_BUF, addr);

	return 0;
}

static struct mxs_platform_fb_entry fb_entry = {
	.name = "ili9225b",
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

static int
_bl_init(struct mxs_platform_bl_data *data)
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
	__raw_writel(BF_PWM_PERIODn_CDIV(6) | /* divide by 64 */
		     BF_PWM_PERIODn_INACTIVE_STATE(2) |	/* low */
		     BF_PWM_PERIODn_ACTIVE_STATE(3) | /* high */
		     BF_PWM_PERIODn_PERIOD(599),
		     REGS_PWM_BASE + HW_PWM_PERIODn(2));
	__raw_writel(BM_PWM_CTRL_PWM2_ENABLE, REGS_PWM_BASE + HW_PWM_CTRL_SET);

	return 0;
}

static void
_bl_free(struct mxs_platform_bl_data *data)
{
	__raw_writel(BF_PWM_ACTIVEn_INACTIVE(0) |
		     BF_PWM_ACTIVEn_ACTIVE(0),
		     REGS_PWM_BASE + HW_PWM_ACTIVEn(2));
	__raw_writel(BF_PWM_PERIODn_CDIV(6) | /* divide by 64 */
		     BF_PWM_PERIODn_INACTIVE_STATE(2) | /* low */
		     BF_PWM_PERIODn_ACTIVE_STATE(3) | /* high */
		     BF_PWM_PERIODn_PERIOD(599),
		     REGS_PWM_BASE + HW_PWM_PERIODn(2));
	__raw_writel(BM_PWM_CTRL_PWM2_ENABLE, REGS_PWM_BASE + HW_PWM_CTRL_CLR);

	clk_disable(_pwm_clk);
	clk_put(_pwm_clk);
}

static int
_bl_to_power(int br)
{
	int base;
	int rem;

	if (br > 100)
		br = 100;
	base = _bl_power[br / 10];
	rem = br % 10;

	if (!rem)
		return base;
	else
		return base + (rem * (_bl_power[br / 10 + 1]) - base) / 10;
}

static int
_bl_set_intensity(struct mxs_platform_bl_data *data,
		struct backlight_device *bd, int suspended)
{
	int intensity = bd->props.brightness;
	int scaled_int;

	if (bd->props.power != FB_BLANK_UNBLANK)
		intensity = 0;
	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		intensity = 0;
	if (suspended)
		intensity = 0;
	if (!_lcd_power)
		intensity = 0;

	/*
	 * This is not too cool but what can we do?
	 * Luminance changes non-linearly...
	 */
	if (regulator_set_current_limit
			(data->regulator,
			 _bl_to_power(intensity),
			 _bl_to_power(intensity)))
		return -EBUSY;

	scaled_int = _bl_values[intensity / 10];

	if (scaled_int < 100) {
		int rem = intensity - 10 * (intensity / 10); /* r = i % 10; */

		scaled_int += rem * (_bl_values[intensity / 10 + 1] -
				_bl_values[intensity / 10]) / 10;
	}

	__raw_writel(BF_PWM_ACTIVEn_INACTIVE(scaled_int) |
			BF_PWM_ACTIVEn_ACTIVE(0),
			REGS_PWM_BASE + HW_PWM_ACTIVEn(2));
	if (scaled_int >= 99) {
		__raw_writel(BF_PWM_PERIODn_CDIV(6) | /* divide by 64 */
				BF_PWM_PERIODn_INACTIVE_STATE(3) | /* high */
				BF_PWM_PERIODn_ACTIVE_STATE(3) | /* high */
				BF_PWM_PERIODn_PERIOD(399),
				REGS_PWM_BASE + HW_PWM_PERIODn(2));
	} else {
		__raw_writel(BF_PWM_PERIODn_CDIV(6) | /* divide by 64 */
				BF_PWM_PERIODn_INACTIVE_STATE(2) | /* low */
				BF_PWM_PERIODn_ACTIVE_STATE(3) | /* high */
				BF_PWM_PERIODn_PERIOD(399),
				REGS_PWM_BASE + HW_PWM_PERIODn(2));
	}

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

static int
__init register_devices(void)
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

int
ili9225b_lcdif_dma_send(dma_addr_t addr)
{
	if (!atomic_read(&_init_panel) || addr <= 0)
		return -1;

	_lcdif_write(HW_LCDIF_CUR_BUF, (uint32_t)addr);

	_lcdif_write(HW_LCDIF_CTRL_CLR, BM_LCDIF_CTRL_RUN);

	_lcdif_write(HW_LCDIF_CTRL1_CLR,
		     BM_LCDIF_CTRL1_BYTE_PACKING_FORMAT);
	_lcdif_write(HW_LCDIF_CTRL1_SET,
		     BF_LCDIF_CTRL1_BYTE_PACKING_FORMAT(0x3));
	_lcd_panel_set_prepare(0, 0);

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
EXPORT_SYMBOL(ili9225b_lcdif_dma_send);

int
ili9225b_lcd_panel_power(int set, dma_addr_t phys)
{
	return _lcd_panel_power(set, phys);
}
EXPORT_SYMBOL(ili9225b_lcd_panel_power);
