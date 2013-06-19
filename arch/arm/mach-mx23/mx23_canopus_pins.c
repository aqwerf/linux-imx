/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include <mach/pinctrl.h>

#include "mx23_pins.h"

static struct pin_desc canopus_fixed_pins[] = {
	{
		.name		= "DUART.RX",
		.id		= PINID_PWM0,
		.fun		= PIN_FUN3,
	},
	{
		.name		= "DUART.TX",
		.id		= PINID_PWM1,
		.fun		= PIN_FUN3,
	},
#ifdef CONFIG_MXS_AUART1_DEVICE_ENABLE
	{
		.name		= "AUART1.RX",
		.id		= PINID_I2C_SDA,
		.fun		= PIN_FUN3,
	},
	{
		.name		= "AUART1.TX",
		.id		= PINID_I2C_SCL,
		.fun		= PIN_FUN3,
	},
	{
		.name		= "AUART1.CTS",
		.id		= PINID_AUART1_CTS,
		.fun		= PIN_FUN1,
	},
	{
		.name		= "AUART1.RTS",
		.id		= PINID_AUART1_RTS,
		.fun		= PIN_FUN1,
	},
#endif
#ifdef CONFIG_MXS_AUART2_DEVICE_ENABLE
	{
		.name		= "AUART2.RX",
		.id		= PINID_GPMI_D14,
		.fun		= PIN_FUN2,
	},
	{
		.name		= "AUART2.TX",
		.id		= PINID_GPMI_D15,
		.fun		= PIN_FUN2,
	},
	{
		.name		= "AUART2.CTS",
		.id		= PINID_ROTARYB,
		.fun		= PIN_FUN2,
	},
	{
		.name		= "AUART2.RTS",
		.id		= PINID_ROTARYA,
		.fun		= PIN_FUN2,
	},
#endif
#if defined(CONFIG_I2C_MXS) || defined(CONFIG_I2C_MXS_MODULE)
	{
		.name		= "I2C_SCL",
		.id		= PINID_I2C_SCL,
		.fun		= PIN_FUN1,
		.strength	= PAD_4MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	},
	{
		.name		= "I2C_SDA",
		.id		= PINID_I2C_SDA,
		.fun		= PIN_FUN1,
		.strength	= PAD_4MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	},
#endif
#ifndef CONFIG_FB_MXS_LCD_ILI9225B
#error "Canopus board support LCD ILI9225B only."
#endif
#if defined(CONFIG_FB_MXS) || defined(CONFIG_FB_MXS_MODULE)
	{
		.name		= "LCD_D00",
		.id		= PINID_LCD_D00,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	},
	{
		.name		= "LCD_D01",
		.id		= PINID_LCD_D01,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	},
	{
		.name		= "LCD_D02",
		.id		= PINID_LCD_D02,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	},
	{
		.name		= "LCD_D03",
		.id		= PINID_LCD_D03,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	},
	{
		.name		= "LCD_D04",
		.id		= PINID_LCD_D04,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	},
	{
		.name		= "LCD_D05",
		.id		= PINID_LCD_D05,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	},
	{
		.name		= "LCD_D06",
		.id		= PINID_LCD_D06,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	},
	{
		.name		= "LCD_D07",
		.id		= PINID_LCD_D07,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	},
	{
		.name		= "LCD_D08",
		.id		= PINID_LCD_D08,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	},
	{
		.name		= "LCD_D09",
		.id		= PINID_LCD_D09,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	},
	{
		.name		= "LCD_D10",
		.id		= PINID_LCD_D10,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	},
	{
		.name		= "LCD_D11",
		.id		= PINID_LCD_D11,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	},
	{
		.name		= "LCD_D12",
		.id		= PINID_LCD_D12,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	},
	{
		.name		= "LCD_D13",
		.id		= PINID_LCD_D13,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	},
	{
		.name		= "LCD_D14",
		.id		= PINID_LCD_D14,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	},
	{
		.name		= "LCD_D15",
		.id		= PINID_LCD_D15,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	},
	{
		.name		= "LCD_RESET",
		.id		= PINID_LCD_RESET,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	},
	{
		.name		= "LCD_RS",
		.id		= PINID_LCD_RS,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	},
	{
		.name		= "LCD_CS",
		.id		= PINID_LCD_CS,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	},
	{
		.name		= "LCD_WR",
		.id		= PINID_LCD_WR,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	},
	{
		.name		= "LCD_BACKLIGHT",
		.id		= PINID_PWM2,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	},
#endif
#if defined(CONFIG_USB_OTG)
	{
		.name		= "USB_OTG_ID",
		.id		= PINID_ROTARYA,
		.fun		= PIN_GPIO,
		.pull		= 1,
		.pullup		= 1,
		.voltage	= PAD_3_3V,
	},
#endif
#if defined(CONFIG_MTD_NAND_GPMI_NFC) || \
	defined(CONFIG_MTD_NAND_GPMI_NFC_MODULE)
	{
		.name		= "GPMI D0",
		.id		= PINID_GPMI_D00,
		.fun		= PIN_FUN1,
		.strength	= PAD_4MA,
		.voltage	= PAD_3_3V,
		.drive		= 1
	},
	{
		.name		= "GPMI D1",
		.id		= PINID_GPMI_D01,
		.fun		= PIN_FUN1,
		.strength	= PAD_4MA,
		.voltage	= PAD_3_3V,
		.drive		= 1
	},
	{
		.name		= "GPMI D2",
		.id		= PINID_GPMI_D02,
		.fun		= PIN_FUN1,
		.strength	= PAD_4MA,
		.voltage	= PAD_3_3V,
		.drive		= 1
	},
	{
		.name		= "GPMI D3",
		.id		= PINID_GPMI_D03,
		.fun		= PIN_FUN1,
		.strength	= PAD_4MA,
		.voltage	= PAD_3_3V,
		.drive		= 1
	},
	{
		.name		= "GPMI D4",
		.id		= PINID_GPMI_D04,
		.fun		= PIN_FUN1,
		.strength	= PAD_4MA,
		.voltage	= PAD_3_3V,
		.drive		= 1
	},
	{
		.name		= "GPMI D5",
		.id		= PINID_GPMI_D05,
		.fun		= PIN_FUN1,
		.strength	= PAD_4MA,
		.voltage	= PAD_3_3V,
		.drive		= 1
	},
	{
		.name		= "GPMI D6",
		.id		= PINID_GPMI_D06,
		.fun		= PIN_FUN1,
		.strength	= PAD_4MA,
		.voltage	= PAD_3_3V,
		.drive		= 1
	},
	{
		.name		= "GPMI D7",
		.id		= PINID_GPMI_D07,
		.fun		= PIN_FUN1,
		.strength	= PAD_4MA,
		.voltage	= PAD_3_3V,
		.drive		= 1
	},
	{
		.name		= "GPMI CLE",
		.id		= PINID_GPMI_CLE,
		.fun		= PIN_FUN1,
		.strength	= PAD_4MA,
		.voltage	= PAD_3_3V,
		.drive		= 1
	},
	{
		.name		= "GPMI ALE",
		.id		= PINID_GPMI_ALE,
		.fun		= PIN_FUN1,
		.strength	= PAD_4MA,
		.voltage	= PAD_3_3V,
		.drive		= 1
	},
	{
		.name		= "GPMI WPN-",
		.id		= PINID_GPMI_WPN,
		.fun		= PIN_FUN1,
		.strength	= PAD_12MA,
		.voltage	= PAD_3_3V,
		.drive		= 1
	},
	{
		.name		= "GPMI WR-",
		.id		= PINID_GPMI_WRN,
		.fun		= PIN_FUN1,
		.strength	= PAD_12MA,
		.voltage	= PAD_3_3V,
		.drive		= 1
	},
	{
		.name		= "GPMI RD-",
		.id		= PINID_GPMI_RDN,
		.fun		= PIN_FUN1,
		.strength	= PAD_12MA,
		.voltage	= PAD_3_3V,
		.drive		= 1
	},
	{
		.name		= "GPMI RDY0",
		.id		= PINID_GPMI_RDY0,
		.fun		= PIN_FUN1,
		.strength	= PAD_4MA,
		.voltage	= PAD_3_3V,
		.drive		= 1
	},
	{
		.name		= "GPMI CE0-",
		.id		= PINID_GPMI_CE0N,
		.fun		= PIN_FUN1,
		.strength	= PAD_4MA,
		.voltage	= PAD_3_3V,
		.drive		= 1
	},
	{
		.name		= "GPMI CE1-",
		.id		= PINID_GPMI_CE1N,
		.fun		= PIN_FUN1,
		.strength	= PAD_4MA,
		.voltage	= PAD_3_3V,
		.drive		= 1
	},
#endif
#if defined(CONFIG_MMC_MXS) || defined(CONFIG_MMC_MXS_MODULE)
	/* Configurations of SSP0 SD/MMC port pins */
	{
		.name		= "SSP1_DATA0",
		.id		= PINID_SSP1_DATA0,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.pullup		= 1,
		.drive		= 1,
		.pull		= 1,
	},
	{
		.name		= "SSP1_DATA1",
		.id		= PINID_SSP1_DATA1,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.pullup		= 1,
		.drive		= 1,
		.pull		= 1,
	},
	{
		.name		= "SSP1_DATA2",
		.id		= PINID_SSP1_DATA2,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.pullup		= 1,
		.drive		= 1,
		.pull		= 1,
	},
	{
		.name		= "SSP1_DATA3",
		.id		= PINID_SSP1_DATA3,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.pullup		= 1,
		.drive		= 1,
		.pull		= 1,
	},
	{
		.name		= "SSP1_CMD",
		.id		= PINID_SSP1_CMD,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.pullup		= 1,
		.drive		= 1,
		.pull		= 1,
	},
	{
		.name		= "SSP1_DETECT",
		.id		= PINID_SSP1_DETECT,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.pullup		= 0,
		.drive		= 1,
		.pull		= 0,
	},
	{
		.name		= "SSP1_SCK",
		.id		= PINID_SSP1_SCK,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.pullup		= 0,
		.drive		= 1,
		.pull		= 0,
	},
#endif
	/* Active High,  AR6003 Internal PMU Enable */
	{
		.name		= "ATH6KL_PMU",
		.id		= PINID_GPMI_RDY3,
		.fun		= PIN_GPIO,
		.voltage	= PAD_3_3V,
		.drive		= 1,
		.output		= 1,
		.data		= 1,
	},
	/* Acrtive Low, Reset signal to power down the AR6003 */
	{
		.name		= "ATH6KL_PWD_L",
		.id		= PINID_GPMI_RDY2,
		.fun		= PIN_GPIO,
		.voltage	= PAD_3_3V,
		.drive		= 1,
		.output		= 1,
		.data		= 1,
	},
	/* Optional Wake On Wireless output */
	{
		.name		= "ATH6KL_WOW",
		.id		= PINID_ROTARYB,
		.fun		= PIN_GPIO,
		.irq		= 1,
	},
	/* External 24MHz Clock, to use instead of crystal oscillator */
	{
		.name		= "ATH6KL_ECLK",
		.id		= PINID_PWM3,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	},
	/* Key PAD Backlight LED PWM Control signal */
	{
		.name		= "KEY_LED_CTRL",
		.id		= PINID_PWM4,
		.fun		= PIN_FUN1,
		.strength	= PAD_8MA,
		.voltage	= PAD_3_3V,
		.drive		= 1,
	},
	/* Active Low, Charge State_Green LED On/Off Signal */
	{
		.name		= "CHGSTS_GLED",
		.id		= PINID_GPMI_D10,
		.fun		= PIN_GPIO,
		.drive		= 0,
		.output		= 1,
		.data		= 1,
	},
	/* Active Low, Charge State_RED LED On/Off Signal */
	{
		.name		= "CHGSTS_RLED",
		.id		= PINID_GPMI_RDY1,
		.fun		= PIN_GPIO,
		.drive		= 0,
		.output		= 1,
		.data		= 1,
	},
	/* Active High, Vibration Motor-On/Off Driving Signal */
	{
		.name		= "MOTOR_CTRL",
		.id		= PINID_GPMI_D15,
		.fun		= PIN_GPIO,
		.voltage	= PAD_3_3V,
		.drive		= 1,
		.output		= 1,
		.data		= 0,
	},
	/* Earmicrop phone Inject Detection Signal */
	{
		.name		= "HDET",
		.id		= PINID_LCD_DOTCK,
		.fun		= PIN_GPIO,
	},
	/* Receiver Amp Enable Signal in the case of Global Model */
	{
		.name		= "RCV_EN",
		.id		= PINID_GPMI_D09,
		.fun		= PIN_GPIO,
		.voltage	= PAD_3_3V,
		.drive		= 1,
		.output		= 1,
		.data		= 0,
	},
	/* Active High, MIC Detect Amp On Sign */
	{
		.name		= "MDET_EN",
		.id		= PINID_GPMI_D08,
		.fun		= PIN_GPIO,
		.voltage	= PAD_3_3V,
		.drive		= 1,
		.output		= 1,
		.data		= 0,
	},
	/* After headset is injected, The low input means that Hds mic on */
	{
		.name		= "MIC_DET",
		.id		= PINID_GPMI_D13,
		.fun		= PIN_GPIO,
		.voltage	= PAD_3_3V,
		.drive		= 1,
		.output		= 0,
		.data		= 1,
	},
	/* Interrupt Input Siganl for Key Pressing */
	{
		.name		= "KEY_INT",
		.id		= PINID_GPMI_D11,
		.fun		= PIN_GPIO,
		.irq		= 1,
	},
	/* Factory Mode Input, Normal Mode=High, Factory Test Mode=Low */
	{
		.name		= "FAC_MODE",
		.id		= PINID_GPMI_D12,
		.fun		= PIN_GPIO,
	},
	/* Cal Mode Control Input, Normal Mode=High, Cal Mode=Low */
	{
		.name		= "CAL_MODE",
		.id		= PINID_I2C_SDA,
		.fun		= PIN_GPIO,
	},
	/* LCD Vendor Discriminator */
	{
		.name		= "LCD_ID1",
		.id		= PINID_LCD_HSYNC,
		.fun		= PIN_GPIO,
		.voltage	= PAD_3_3V,
		.drive		= 1,
		.output		= 0,
		.data		= 0,
	},
	/* LCD Vendor Discriminator */
	{
		.name		= "LCD_ID2",
		.id		= PINID_LCD_VSYNC,
		.fun		= PIN_GPIO,
		.voltage	= PAD_3_3V,
		.drive		= 1,
		.output		= 0,
		.data		= 0,
	},
};

static void mxs_request_pins(struct pin_desc *pins, int nr)
{
	int i;
	struct pin_desc *pin;
	unsigned gpio;

	/* configure the pins */
	for (i = 0; i < nr; i++) {
		pin = &pins[i];
		if (pin->fun == PIN_GPIO)
			gpio_request(MXS_PIN_TO_GPIO(pin->id), pin->name);
		else
			mxs_request_pin(pin->id, pin->fun, pin->name);
		if (pin->drive) {
			mxs_set_strength(pin->id, pin->strength, pin->name);
			mxs_set_voltage(pin->id, pin->voltage, pin->name);
		}
		if (pin->pull)
			mxs_set_pullup(pin->id, pin->pullup, pin->name);
		if (pin->fun == PIN_GPIO) {
			if (pin->output)
				gpio_direction_output(MXS_PIN_TO_GPIO(pin->id),
						      pin->data);
			else
				gpio_direction_input(MXS_PIN_TO_GPIO(pin->id));

			if (pin->irq) {
				gpio = MXS_PIN_TO_GPIO(pin->id);
				set_irq_type(gpio_to_irq(gpio),
						IRQ_TYPE_LEVEL_HIGH);
			}
		}
	}
}

static void mxs_release_pins(struct pin_desc *pins, int nr)
{
	int i;
	struct pin_desc *pin;
	unsigned gpio;

	/* release the pins */
	for (i = 0; i < nr; i++) {
		pin = &pins[i];
		if (pin->fun == PIN_GPIO) {
			gpio_free(MXS_PIN_TO_GPIO(pin->id));
			if (pin->irq) {
				gpio = MXS_PIN_TO_GPIO(pin->id);
				set_irq_type(gpio_to_irq(gpio),
						IRQ_TYPE_NONE);
			}
		} else
			mxs_release_pin(pin->id, pin->name);
	}
}

int mxs_mmc_get_wp_mmc0(void)
{
	return 0;
}

int mxs_mmc_hw_init_mmc0(void)
{
	return 0;
}

void mxs_mmc_hw_release_mmc0(void)
{
}

void mxs_mmc_cmd_pullup_mmc0(int enable)
{
	mxs_set_pullup(PINID_SSP1_CMD, enable, "mmc0_cmd");
}

void __init mx23_canopus_pins_init(void)
{
	mxs_request_pins(canopus_fixed_pins, ARRAY_SIZE(canopus_fixed_pins));
}

int mxs_key_backlight_gpio_set(int set)
{
	gpio_set_value(MXS_PIN_TO_GPIO(PINID_PWM4), set);

	return 0;
}

int mxs_audio_jack_gpio_irq(void)
{
	return gpio_to_irq(MXS_PIN_TO_GPIO(PINID_LCD_DOTCK));
}

int mxs_audio_jack_gpio_get(void)
{
	return gpio_get_value(MXS_PIN_TO_GPIO(PINID_LCD_DOTCK));
}

int mxs_audio_receiver_amp_gpio_set(int set)
{
	gpio_set_value(MXS_PIN_TO_GPIO(PINID_GPMI_D09), set);

	return 0;
}

int mxs_audio_headset_mic_detect_amp_gpio_set(int set)
{
	gpio_set_value(MXS_PIN_TO_GPIO(PINID_GPMI_D08), set);

	return 0;
}

int mxs_audio_headset_mic_status_gpio_get(void)
{
	return gpio_get_value(MXS_PIN_TO_GPIO(PINID_GPMI_D13));
}

int mxs_charger_led_green_gpio_set(int set)
{
	gpio_set_value(MXS_PIN_TO_GPIO(PINID_GPMI_D10), set);

	return 0;
}

int mxs_charger_led_red_gpio_set(int set)
{
	gpio_set_value(MXS_PIN_TO_GPIO(PINID_GPMI_RDY1), set);

	return 0;
}

int mxs_audio_lcd_id1_gpio_get(void)
{
	return gpio_get_value(MXS_PIN_TO_GPIO(PINID_LCD_HSYNC));
}

int mxs_audio_lcd_id2_gpio_get(void)
{
	return gpio_get_value(MXS_PIN_TO_GPIO(PINID_LCD_VSYNC));
}

void mxs_wow_irq_enable(void)
{
	unsigned int irq = gpio_to_irq(MXS_PIN_TO_GPIO(PINID_ROTARYB));
	struct irq_desc *desc = irq_to_desc(irq);

	desc->chip->unmask(irq);
	desc->status &= ~IRQ_MASKED;
}

void mxs_key_irq_enable(void)
{
	unsigned int irq = gpio_to_irq(MXS_PIN_TO_GPIO(PINID_GPMI_D11));
	struct irq_desc *desc = irq_to_desc(irq);

	desc->chip->unmask(irq);
	desc->status &= ~IRQ_MASKED;
}
