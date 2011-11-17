/*
 * Board support file for OMAP4430 SDP.
 *
 * Copyright (C) 2009 Texas Instruments
 *
 * Author: Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * Based on mach-omap2/board-3430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/usb/otg.h>
#include <linux/spi/spi.h>
#include <linux/i2c/twl.h>
#include <linux/i2c/cma3000.h>
#include <linux/i2c/bq2415x.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/tps6130x.h>
#include <linux/input/sfh7741.h>
#include <linux/leds.h>
#include <linux/leds_pwm.h>
#include <linux/leds-omap4430sdp-display.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/twl6040-vib.h>
#include <linux/wl12xx.h>
#include <linux/cdc_tcxo.h>
#include <linux/mfd/twl6040-codec.h>

#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <mach/emif.h>
#include <mach/lpddr2-elpida.h>
#include <mach/lpddr2-samsung.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/control.h>
#include <plat/timer-gp.h>
#include <plat/display.h>
#include <plat/usb.h>
#include <plat/omap_device.h>
#include <plat/omap_hwmod.h>
#ifdef CONFIG_SERIAL_OMAP
#include <plat/omap-serial.h>
#include <plat/serial.h>
#endif
#include <linux/wakelock.h>
#include <plat/opp_twl_tps.h>
#include <plat/mmc.h>
#include <plat/syntm12xx.h>
#include <plat/omap4-keypad.h>
#include <plat/hwspinlock.h>
#include <plat/nokia-dsi-panel.h>
#include <plat/dmtimer.h>
#include "mux.h"
#include "hsmmc.h"
#include "smartreflex-class3.h"
#include "board-4430sdp-wifi.h"
#include "board-4430-acclaim.h"

#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>

#include <linux/input/cyttsp.h>
#include <linux/input/ft5x06.h>
#include <linux/input/kxtf9.h>
#include <linux/max17042.h>
#include <linux/max8903.h>

#define KXTF9_DEVICE_ID                 "kxtf9"
#define KXTF9_I2C_SLAVE_ADDRESS         0x0F
#define KXTF9_GPIO_FOR_PWR              34

#define CONFIG_SERIAL_OMAP_IDLE_TIMEOUT 5
#define OMAP4_SFH7741_SENSOR_OUTPUT_GPIO	184
#define OMAP4_SFH7741_ENABLE_GPIO		188

#define OMAP4_TOUCH_IRQ_1		35

#define OMAP4_CMA3000ACCL_GPIO		186
#define OMAP4SDP_MDM_PWR_EN_GPIO	157

#define LED_SEC_DISP_GPIO 27
#define DSI2_GPIO_59	59

#define LED_PWM2ON		0x03
#define LED_PWM2OFF		0x04
#define LED_TOGGLE3		0x92

#define GPIO_WIFI_PMENA		54
#define GPIO_WIFI_IRQ		53

#define CYTTSP_I2C_SLAVEADDRESS 	34
#define OMAP_CYTTSP_GPIO        	37 /*99*/
#define OMAP_CYTTSP_RESET_GPIO 		39 /*46*/

#define FT5x06_I2C_SLAVEADDRESS  (0x70 >> 1)
#define OMAP_FT5x06_GPIO         37 /*99*/
#define OMAP_FT5x06_RESET_GPIO   39 /*46*/


#define TWL6030_RTC_GPIO 6
#define BLUETOOTH_UART UART2
#define BLUETOOTH_UART_DEV_NAME "/dev/ttyO1"
#define CONSOLE_UART UART1

#define  SAMSUNG_SDRAM 0x1 
#define  ELPIDA_SDRAM  0x3 
#define  HYNIX_SDRAM   0x6

static int max17042_gpio_for_irq = 0;
static int kxtf9_gpio_for_irq = 0;

#ifdef CONFIG_BATTERY_MAX17042
static void max17042_dev_init(void)
{
	printk("board-4430sdp.c: max17042_dev_init ...\n");

	if (gpio_request(max17042_gpio_for_irq, "max17042_irq") < 0) {
		printk(KERN_ERR "Can't get GPIO for max17042 IRQ\n");
		return;
	}

	printk("board-4430sdp.c: max17042_dev_init > Init max17042 irq pin %d !\n", max17042_gpio_for_irq);
	gpio_direction_input(max17042_gpio_for_irq);
	printk("max17042 GPIO pin read %d\n", gpio_get_value(max17042_gpio_for_irq));
}
#endif

static void kxtf9_dev_init(void)
{
	printk("board-4430sdp.c: kxtf9_dev_init ...\n");

	if (gpio_request(kxtf9_gpio_for_irq, "kxtf9_irq") < 0)
	{
		printk("Can't get GPIO for kxtf9 IRQ\n");
		return;
	}

	printk("board-4430sdp.c: kxtf9_dev_init > Init kxtf9 irq pin %d !\n",
			kxtf9_gpio_for_irq);
	gpio_direction_input(kxtf9_gpio_for_irq);
}


struct kxtf9_platform_data kxtf9_platform_data_here = {
	.min_interval   = 1,
	.poll_interval  = 1000,

	.g_range        = KXTF9_G_8G,
	.shift_adj      = SHIFT_ADJ_2G,

	/* Map the axes from the sensor to the device */
	/* SETTINGS FOR acclaim */
	.axis_map_x = 1,
	.axis_map_y = 0,
	.axis_map_z = 2,
	.axis_map_x     = 1,
	.axis_map_y     = 0,
	.axis_map_z     = 2,
	.negate_x       = 1,
	.negate_y       = 0,
	.negate_z       = 0,
	.data_odr_init          = ODR12_5F,
	.ctrl_reg1_init         = KXTF9_G_8G | RES_12BIT | TDTE | WUFE | TPE,
	.int_ctrl_init          = KXTF9_IEN | KXTF9_IEA | KXTF9_IEL,
	.int_ctrl_init          = KXTF9_IEN,
	.tilt_timer_init        = 0x03,
	.engine_odr_init        = OTP12_5 | OWUF50 | OTDT400,
	.wuf_timer_init         = 0x16,
	.wuf_thresh_init        = 0x28,
	.tdt_timer_init         = 0x78,
	.tdt_h_thresh_init      = 0xFF,
	.tdt_l_thresh_init      = 0x14,
	.tdt_tap_timer_init     = 0x53,
	.tdt_total_timer_init   = 0x24,
	.tdt_latency_timer_init = 0x10,
	.tdt_window_timer_init  = 0xA0,

	.gpio = 0,
};

int  ft5x06_dev_init(int resource)
{
	if (resource){
		omap_mux_init_signal("gpmc_ad13.gpio_37", OMAP_PIN_INPUT | OMAP_PIN_OFF_INPUT_PULLUP |  OMAP_PIN_OFF_WAKEUPENABLE);
		omap_mux_init_signal("gpmc_ad15.gpio_39", OMAP_PIN_INPUT | OMAP_PIN_OFF_INPUT_PULLUP);

		if (gpio_request(OMAP_FT5x06_RESET_GPIO, "ft5x06_reset") < 0){
			printk(KERN_ERR "can't get ft5x06 xreset GPIO\n");
			return -1;
		}

		if (gpio_request(OMAP_FT5x06_GPIO, "ft5x06_touch") < 0) {
			printk(KERN_ERR "can't get ft5x06 interrupt GPIO\n");
			return -1;
		}

		gpio_direction_input(OMAP_FT5x06_GPIO);
	} else {
		gpio_free(OMAP_FT5x06_GPIO);
		gpio_free(OMAP_FT5x06_RESET_GPIO);
	}

	return 0;
}

static struct ft5x06_platform_data ft5x06_platform_data = {
	.maxx = 600,
	.maxy = 1024,
	.flags = 0,
	.reset_gpio = OMAP_FT5x06_RESET_GPIO,
	.use_st = FT_USE_ST,
	.use_mt = FT_USE_MT,
	.use_trk_id = FT_USE_TRACKING_ID,
	.use_sleep = FT_USE_SLEEP,
	.use_gestures = 1,
};

int  cyttsp_dev_init(int resource)
{
	if (resource) {
		omap_mux_init_signal("gpmc_ad13.gpio_37", OMAP_PIN_INPUT | OMAP_PIN_OFF_INPUT_PULLUP |  OMAP_PIN_OFF_WAKEUPENABLE);
		omap_mux_init_signal("gpmc_ad15.gpio_39", OMAP_PIN_INPUT | OMAP_PIN_OFF_INPUT_PULLUP);

		if (gpio_request(OMAP_CYTTSP_RESET_GPIO, "tma340_reset") < 0) {
			printk(KERN_ERR "can't get tma340 xreset GPIO\n");
			return -1;
		}

		if (gpio_request(OMAP_CYTTSP_GPIO, "cyttsp_touch") < 0) {
			printk(KERN_ERR "can't get cyttsp interrupt GPIO\n");
			return -1;
		}

		gpio_direction_input(OMAP_CYTTSP_GPIO);
		/* omap_set_gpio_debounce(OMAP_CYTTSP_GPIO, 0); */
	}
	else {
		printk ("\n%s: Free resources",__FUNCTION__);
		gpio_free(OMAP_CYTTSP_GPIO);
		gpio_free(OMAP_CYTTSP_RESET_GPIO);
	}
	return 0;
}

static struct cyttsp_platform_data cyttsp_platform_data = {
	.maxx = 480,
	.maxy = 800,
	.flags = 0,
	.gen = CY_GEN3,
	.use_st = CY_USE_ST,
	.use_mt = CY_USE_MT,
	.use_hndshk = CY_SEND_HNDSHK,
	.use_trk_id = CY_USE_TRACKING_ID,
	.use_sleep = CY_USE_SLEEP,
	.use_gestures = CY_USE_GESTURES,
	/* activate up to 4 groups
	 * and set active distance
	 */
	.gest_set = CY_GEST_GRP1 | CY_GEST_GRP2 | CY_GEST_GRP3 | CY_GEST_GRP4 | CY_ACT_DIST,
	/* change act_intrvl to customize the Active power state.
	 * scanning/processing refresh interval for Operating mode
	 */
	.act_intrvl = CY_ACT_INTRVL_DFLT,
	/* change tch_tmout to customize the touch timeout for the
	 * Active power state for Operating mode
	 */
	.tch_tmout = CY_TCH_TMOUT_DFLT,
	/* change lp_intrvl to customize the Low Power power state.
	 * scanning/processing refresh interval for Operating mode
	 */
	.lp_intrvl = CY_LP_INTRVL_DFLT,
};

static struct wake_lock uart_lock;
static struct platform_device sdp4430_hdmi_audio_device = {
	.name		= "hdmi-dai",
	.id		= -1,
};

static int sdp4430_keymap[] = {
	KEY(0, 0, KEY_E),
	KEY(0, 1, KEY_R),
	KEY(0, 2, KEY_T),
	KEY(0, 3, KEY_HOME),
	KEY(0, 4, KEY_F5),
	KEY(0, 5, KEY_UNKNOWN),
	KEY(0, 6, KEY_I),
	KEY(0, 7, KEY_LEFTSHIFT),

	KEY(1, 0, KEY_D),
	KEY(1, 1, KEY_F),
	KEY(1, 2, KEY_G),
	KEY(1, 3, KEY_SEND),
	KEY(1, 4, KEY_F6),
	KEY(1, 5, KEY_UNKNOWN),
	KEY(1, 6, KEY_K),
	KEY(1, 7, KEY_ENTER),

	KEY(2, 0, KEY_X),
	KEY(2, 1, KEY_C),
	KEY(2, 2, KEY_V),
	KEY(2, 3, KEY_END),
	KEY(2, 4, KEY_F7),
	KEY(2, 5, KEY_UNKNOWN),
	KEY(2, 6, KEY_DOT),
	KEY(2, 7, KEY_CAPSLOCK),

	KEY(3, 0, KEY_Z),
	KEY(3, 1, KEY_KPPLUS),
	KEY(3, 2, KEY_B),
	KEY(3, 3, KEY_VOLUMEUP),
	KEY(3, 4, KEY_F8),
	KEY(3, 5, KEY_UNKNOWN),
	KEY(3, 6, KEY_O),
	KEY(3, 7, KEY_SPACE),

	KEY(4, 0, KEY_W),
	KEY(4, 1, KEY_Y),
	KEY(4, 2, KEY_U),
	KEY(4, 3, KEY_VOLUMEDOWN),
	KEY(4, 4, KEY_VOLUMEUP),
	KEY(4, 5, KEY_UNKNOWN),
	KEY(4, 6, KEY_L),
	KEY(4, 7, KEY_LEFT),

	KEY(5, 0, KEY_S),
	KEY(5, 1, KEY_H),
	KEY(5, 2, KEY_J),
	KEY(5, 3, KEY_F3),
	KEY(5, 4, KEY_F9),
	KEY(5, 5, KEY_VOLUMEDOWN),
	KEY(5, 6, KEY_M),
	KEY(5, 7, KEY_RIGHT),

	KEY(6, 0, KEY_Q),
	KEY(6, 1, KEY_A),
	KEY(6, 2, KEY_N),
	KEY(6, 3, KEY_BACK),
	KEY(6, 4, KEY_BACKSPACE),
	KEY(6, 5, KEY_UNKNOWN),
	KEY(6, 6, KEY_P),
	KEY(6, 7, KEY_UP),

	KEY(7, 0, KEY_PROG1),
	KEY(7, 1, KEY_PROG2),
	KEY(7, 2, KEY_PROG3),
	KEY(7, 3, KEY_PROG4),
	KEY(7, 4, KEY_F4),
	KEY(7, 5, KEY_UNKNOWN),
	KEY(7, 6, KEY_OK),
	KEY(7, 7, KEY_DOWN),
};

static struct matrix_keymap_data sdp4430_keymap_data = {
	.keymap			= sdp4430_keymap,
	.keymap_size		= ARRAY_SIZE(sdp4430_keymap),
};

static struct omap4_keypad_platform_data sdp4430_keypad_data = {
	.keymap_data		= &sdp4430_keymap_data,
	.rows			= 8,
	.cols			= 8,
	.rep		        = 0,
};

void keyboard_mux_init(void)
{
	omap_mux_init_signal("kpd_col0.kpd_col0",
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1);
	omap_mux_init_signal("kpd_col1.kpd_col1",
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1);
	omap_mux_init_signal("kpd_col2.kpd_col2",
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1);
	omap_mux_init_signal("kpd_col3.kpd_col3",
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1);
	omap_mux_init_signal("kpd_col4.kpd_col4",
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1);
	omap_mux_init_signal("kpd_col5.kpd_col5",
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1);
	omap_mux_init_signal("gpmc_a23.kpd_col7",
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1);
	omap_mux_init_signal("gpmc_a22.kpd_col6",
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1);
	omap_mux_init_signal("kpd_row0.kpd_row0",
				OMAP_PULL_ENA | OMAP_PULL_UP |
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1 |
				OMAP_INPUT_EN);
	omap_mux_init_signal("kpd_row1.kpd_row1",
				OMAP_PULL_ENA | OMAP_PULL_UP |
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1 |
				OMAP_INPUT_EN);
	omap_mux_init_signal("kpd_row2.kpd_row2",
				OMAP_PULL_ENA | OMAP_PULL_UP |
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1 |
				OMAP_INPUT_EN);
	omap_mux_init_signal("kpd_row3.kpd_row3",
				OMAP_PULL_ENA | OMAP_PULL_UP |
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1 |
				OMAP_INPUT_EN);
	omap_mux_init_signal("kpd_row4.kpd_row4",
				OMAP_PULL_ENA | OMAP_PULL_UP |
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1 |
				OMAP_INPUT_EN);
	omap_mux_init_signal("kpd_row5.kpd_row5",
				OMAP_PULL_ENA | OMAP_PULL_UP |
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1 |
				OMAP_INPUT_EN);
	omap_mux_init_signal("gpmc_a18.kpd_row6",
				OMAP_PULL_ENA | OMAP_PULL_UP |
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1 |
				OMAP_INPUT_EN);
	omap_mux_init_signal("gpmc_a19.kpd_row7",
				OMAP_PULL_ENA | OMAP_PULL_UP |
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1 |
				OMAP_INPUT_EN);
}

/* Proximity Sensor */
static void omap_prox_activate(int state)
{
	gpio_set_value(OMAP4_SFH7741_ENABLE_GPIO , state);
}

static int omap_prox_read(void)
{
	int proximity;
	proximity = gpio_get_value(OMAP4_SFH7741_SENSOR_OUTPUT_GPIO);
#ifdef CONFIG_ANDROID
	/* Invert the output from the prox sensor for Android as 0 should
	be near and 1 should be far */
	return !proximity;
#else
	return proximity;
#endif
}

static void omap_sfh7741prox_init(void)
{
	int  error;

	error = gpio_request(OMAP4_SFH7741_SENSOR_OUTPUT_GPIO, "sfh7741");
	if (error < 0) {
		pr_err("%s: GPIO configuration failed: GPIO %d, error %d\n"
			, __func__, OMAP4_SFH7741_SENSOR_OUTPUT_GPIO, error);
		return ;
	}

	error = gpio_direction_input(OMAP4_SFH7741_SENSOR_OUTPUT_GPIO);
	if (error < 0) {
		pr_err("Proximity GPIO input configuration failed\n");
		goto fail1;
	}

	error = gpio_request(OMAP4_SFH7741_ENABLE_GPIO, "sfh7741");
	if (error < 0) {
		pr_err("failed to request GPIO %d, error %d\n",
			OMAP4_SFH7741_ENABLE_GPIO, error);
		goto fail1;
	}

	error = gpio_direction_output(OMAP4_SFH7741_ENABLE_GPIO , 0);
	if (error < 0) {
		pr_err("%s: GPIO configuration failed: GPIO %d,\
			error %d\n",__func__, OMAP4_SFH7741_ENABLE_GPIO, error);
		goto fail3;
	}
	return;

fail3:
	gpio_free(OMAP4_SFH7741_ENABLE_GPIO);
fail1:
	gpio_free(OMAP4_SFH7741_SENSOR_OUTPUT_GPIO);
}

static struct sfh7741_platform_data omap_sfh7741_data = {
	.flags = SFH7741_WAKEABLE_INT,
	.irq_flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
	.prox_enable = 0,
	.activate_func = omap_prox_activate,
	.read_prox = omap_prox_read,
};

static struct platform_device sdp4430_proximity_device = {
	.name		= SFH7741_NAME,
	.id		= 1,
	.dev		= {
		.platform_data = &omap_sfh7741_data,
	},
};

#ifdef CONFIG_CHARGER_MAX8903

static struct resource max8903_gpio_resources_evt1a[] = {
	{	.name	= MAX8903_TOKEN_GPIO_CHG_EN,
		.start	= MAX8903_GPIO_CHG_EN,
		.end	= MAX8903_GPIO_CHG_EN,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_FLT,
		.start	= MAX8903_GPIO_CHG_FLT,
		.end	= MAX8903_GPIO_CHG_FLT,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_IUSB,
		.start	= MAX8903_GPIO_CHG_IUSB,
		.end	= MAX8903_GPIO_CHG_IUSB,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_USUS,
		.start	= MAX8903_GPIO_CHG_USUS_EVT1A,
		.end	= MAX8903_GPIO_CHG_USUS_EVT1A,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_ILM,
		.start	= MAX8903_GPIO_CHG_ILM_EVT1A,
		.end	= MAX8903_GPIO_CHG_ILM_EVT1A,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_UOK,
		.start	= MAX8903_UOK_GPIO_FOR_IRQ,
		.end	= MAX8903_UOK_GPIO_FOR_IRQ,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_DOK,
		.start	= MAX8903_DOK_GPIO_FOR_IRQ,
		.end	= MAX8903_DOK_GPIO_FOR_IRQ,
		.flags	= IORESOURCE_IO,
	}
};

static struct resource max8903_gpio_resources_evt1b[] = {
	{	.name	= MAX8903_TOKEN_GPIO_CHG_EN,
		.start	= MAX8903_GPIO_CHG_EN,
		.end	= MAX8903_GPIO_CHG_EN,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_FLT,
		.start	= MAX8903_GPIO_CHG_FLT,
		.end	= MAX8903_GPIO_CHG_FLT,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_IUSB,
		.start	= MAX8903_GPIO_CHG_IUSB,
		.end	= MAX8903_GPIO_CHG_IUSB,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_USUS,
		.start	= MAX8903_GPIO_CHG_USUS_EVT1B,
		.end	= MAX8903_GPIO_CHG_USUS_EVT1B,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_ILM,
		.start	= MAX8903_GPIO_CHG_ILM_EVT1B,
		.end	= MAX8903_GPIO_CHG_ILM_EVT1B,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_UOK,
		.start	= MAX8903_UOK_GPIO_FOR_IRQ,
		.end	= MAX8903_UOK_GPIO_FOR_IRQ,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_DOK,
		.start	= MAX8903_DOK_GPIO_FOR_IRQ,
		.end	= MAX8903_DOK_GPIO_FOR_IRQ,
		.flags	= IORESOURCE_IO,
	}
};

static struct platform_device max8903_charger_device = {
	.name           = "max8903_charger",
	.id             = -1,
};

static inline void acclaim_init_charger(void)
{
	int board_type = acclaim_board_type();

	if (board_type == EVT1A) {
		max8903_charger_device.resource = max8903_gpio_resources_evt1a;
		max8903_charger_device.num_resources = ARRAY_SIZE(max8903_gpio_resources_evt1a);
	} else if (board_type == EVT1B) {
		max8903_charger_device.resource = max8903_gpio_resources_evt1b;
		max8903_charger_device.num_resources = ARRAY_SIZE(max8903_gpio_resources_evt1b);
	} else {
		pr_err("%s: Acclaim board %d not supported\n", __func__, board_type);
		return;
	}
	platform_device_register(&max8903_charger_device);
}

#endif

#ifdef CONFIG_BATTERY_MAX17042
struct max17042_platform_data max17042_platform_data_here = {

	.gpio = 0,

};
#endif

static struct spi_board_info sdp4430_spi_board_info[] __initdata = {
	{
		.modalias		= "boxer_disp_spi",
		.bus_num		= 4,	/* 4: McSPI4 */
		.chip_select		= 0,
		.max_speed_hz		= 375000,
	},
};


static struct gpio_led sdp4430_gpio_leds[] = {
	{
		.name	= "omap4:green:debug0",
		.gpio	= 61,
	},
	{
		.name	= "omap4:green:debug1",
		.gpio	= 30,
	},
	{
		.name	= "omap4:green:debug2",
		.gpio	= 7,
	},
	{
		.name	= "omap4:green:debug3",
		.gpio	= 8,
	},
	{
		.name	= "omap4:green:debug4",
		.gpio	= 50,
	},
	{
		.name	= "blue",
		.default_trigger = "timer",
		.gpio	= 169,
	},
	{
		.name	= "red",
		.default_trigger = "timer",
		.gpio	= 170,
	},
	{
		.name	= "green",
		.default_trigger = "timer",
		.gpio	= 139,
	},
};

/*--------------------------------------------------------------------------*/

static struct gpio_keys_button acclaim_gpio_buttons[] = {
	
	{
		.code 		= KEY_POWER,
		.gpio 		= 29,
		.desc 		= "POWER",
		.active_low 	= 0,
		.wakeup 	= 1,
	},
	{

		.code 		= KEY_HOME,
		.gpio 		= 32,
		.desc 		= "HOME",
		.active_low 	= 1,
		.wakeup 	= 1,
	}
};

static struct gpio_keys_platform_data acclaim_gpio_key_info ={
	.buttons	= acclaim_gpio_buttons,
	.nbuttons	= ARRAY_SIZE ( acclaim_gpio_buttons ),
};

static struct platform_device acclaim_keys_gpio = {
	.name		= "gpio-keys",
	.id		= -1,
	.dev		= {
				.platform_data = &acclaim_gpio_key_info,
	},
};

static void __init acclaim4430_init_display_led(void)
{
	omap_mux_init_signal("usbb1_ulpitll_dat4.dmtimer8_pwm_evt", OMAP_MUX_MODE1);
}

static void omap_pwm_led_set_pwm_cycle(struct omap_dm_timer *pwm_timer,
		int cycle, int def_on)
{
	int n;
	unsigned int timerval;

	if (cycle == 0)
		n = 0xff;
	else
		n = cycle - 1;

	if (cycle == LED_FULL) {
		omap_dm_timer_set_pwm(pwm_timer,
				def_on ? 1 : 0, 1,
				OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
		omap_dm_timer_stop(pwm_timer);
	} else {
		omap_dm_timer_set_pwm(pwm_timer,
				def_on ? 0 : 1, 1,
				OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
		omap_dm_timer_set_match(pwm_timer, 1,
				(0xffffff00) | cycle);
		/* ensure timer value is in range */
		timerval = omap_dm_timer_read_counter(pwm_timer);
		if (timerval < 0xffffff00)
			omap_dm_timer_write_counter(pwm_timer, -2);

		omap_dm_timer_start(pwm_timer);
	}
}

void acclaim4430_set_primary_brightness(u8 brightness)
{
	static struct omap_dm_timer *pwm_timer;
	static int pwm_state;
	const int def_on = 0;

	if (!pwm_timer) {
		pwm_timer = omap_dm_timer_request_specific(8);

		if (pwm_timer) {
			/* omap_dm_timer_request_specific start pwm */
			omap_dm_timer_disable(pwm_timer);
			pwm_state = 0;
		}
	}

	if (!pwm_timer)
		return;

	if (brightness == pwm_state)
		return;

	if (brightness) {
		omap_dm_timer_enable(pwm_timer);
		omap_dm_timer_set_source(pwm_timer, OMAP_TIMER_SRC_32_KHZ);
		omap_dm_timer_set_load(pwm_timer, 1, 0xffffff00);
		omap_pwm_led_set_pwm_cycle(pwm_timer, brightness, def_on);
	} else {
		/* Everything off */
		omap_dm_timer_set_pwm(pwm_timer,
				def_on ? 0 : 1, 1,
				OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
		omap_dm_timer_stop(pwm_timer);
		omap_dm_timer_disable(pwm_timer);
	}

	pwm_state = brightness;
}

static struct omap4430_sdp_disp_led_platform_data acclaim4430_disp_led_data = {
	.flags = LEDS_CTRL_AS_ONE_DISPLAY,
	.display_led_init = acclaim4430_init_display_led,
	.primary_display_set = acclaim4430_set_primary_brightness,
	.secondary_display_set = NULL,
};

static struct platform_device omap_disp_led = {
	.name	=	"display_led",
	.id	=	-1,
	.dev	= {
		.platform_data = &acclaim4430_disp_led_data,
	},
};

/*--------------------------------------------------------------------------*/

static struct gpio_led_platform_data sdp4430_led_data = {
	.leds	= sdp4430_gpio_leds,
	.num_leds	= ARRAY_SIZE(sdp4430_gpio_leds),
};

static struct platform_device sdp4430_leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data = &sdp4430_led_data,
	},
};

static struct led_pwm sdp4430_pwm_leds[] = {
	{
	.name = "battery-led",
	.pwm_id = 1,
	.max_brightness = 255,
	.pwm_period_ns = 7812500,
	},
};

static struct led_pwm_platform_data sdp4430_pwm_data = {
	.num_leds = 1,
	.leds = sdp4430_pwm_leds,
};

static struct platform_device sdp4430_leds_pwm = {
	.name	= "leds_pwm",
	.id	= -1,
	.dev	= {
		.platform_data = &sdp4430_pwm_data,
	},
};

/* Begin Synaptic Touchscreen TM-01217 */

static char *tm12xx_idev_names[] = {
	"syn_tm12xx_ts_1",
	"syn_tm12xx_ts_2",
	"syn_tm12xx_ts_3",
	"syn_tm12xx_ts_4",
	"syn_tm12xx_ts_5",
	"syn_tm12xx_ts_6",
	NULL,
};

static u8 tm12xx_button_map[] = {
	KEY_F1,
	KEY_F2,
};

static struct tm12xx_ts_platform_data tm12xx_platform_data[] = {
	{ /* Primary Controller */
		.gpio_intr = 35,
		.idev_name = tm12xx_idev_names,
		.button_map = tm12xx_button_map,
		.num_buttons = ARRAY_SIZE(tm12xx_button_map),
		.repeat = 0,
		.swap_xy = 1,
		.controller_num = 0,
	/* Android does not have touchscreen as wakeup source */
#if !defined(CONFIG_ANDROID)
		.suspend_state = SYNTM12XX_ON_ON_SUSPEND,
#else
		.suspend_state = SYNTM12XX_SLEEP_ON_SUSPEND,
#endif
	},
	{ /* Secondary Controller */
		.gpio_intr = 36,
		.idev_name = tm12xx_idev_names,
		.button_map = tm12xx_button_map,
		.num_buttons = ARRAY_SIZE(tm12xx_button_map),
		.repeat = 0,
		.swap_xy = 1,
		.controller_num = 1,
	/* Android does not have touchscreen as wakeup source */
#if !defined(CONFIG_ANDROID)
		.suspend_state = SYNTM12XX_ON_ON_SUSPEND,
#else
		.suspend_state = SYNTM12XX_SLEEP_ON_SUSPEND,
#endif
	},
};

/* End Synaptic Touchscreen TM-01217 */

static void __init sdp4430_init_display_led(void)
{
	twl_i2c_write_u8(TWL_MODULE_PWM, 0xFF, LED_PWM2ON);
	twl_i2c_write_u8(TWL_MODULE_PWM, 0x7F, LED_PWM2OFF);
	twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x30, TWL6030_TOGGLE3);
}

static void sdp4430_set_primary_brightness(u8 brightness)
{
	if (brightness > 1) {
		if (brightness == 255)
			brightness = 0x7f;
		else
			brightness = (~(brightness/2)) & 0x7f;

		twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x30, TWL6030_TOGGLE3);
		twl_i2c_write_u8(TWL_MODULE_PWM, brightness, LED_PWM2ON);
	} else if (brightness <= 1) {
		twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x08, TWL6030_TOGGLE3);
		twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x38, TWL6030_TOGGLE3);
	}
}

static void sdp4430_set_secondary_brightness(u8 brightness)
{
	if (brightness > 0)
		brightness = 1;

	gpio_set_value(LED_SEC_DISP_GPIO, brightness);
}

static struct omap4430_sdp_disp_led_platform_data sdp4430_disp_led_data = {
	.flags = LEDS_CTRL_AS_ONE_DISPLAY,
	.display_led_init = sdp4430_init_display_led,
	.primary_display_set = sdp4430_set_primary_brightness,
	.secondary_display_set = sdp4430_set_secondary_brightness,
};

static void __init omap_disp_led_init(void)
{
	/* Seconday backlight control */
	gpio_request(DSI2_GPIO_59, "dsi2_bl_gpio");
	gpio_direction_output(DSI2_GPIO_59, 0);

	if (sdp4430_disp_led_data.flags & LEDS_CTRL_AS_ONE_DISPLAY) {
		pr_info("%s: Configuring as one display LED\n", __func__);
		gpio_set_value(DSI2_GPIO_59, 1);
	}

	gpio_request(LED_SEC_DISP_GPIO, "dsi1_bl_gpio");
	gpio_direction_output(LED_SEC_DISP_GPIO, 1);
	mdelay(120);
	gpio_set_value(LED_SEC_DISP_GPIO, 0);

}

static struct platform_device sdp4430_disp_led = {
	.name	=	"display_led",
	.id	=	-1,
	.dev	= {
		.platform_data = &acclaim4430_disp_led_data,/*&sdp4430_disp_led_data,*/
	},
};

static struct platform_device sdp4430_keypad_led = {
	.name	=	"keypad_led",
	.id	=	-1,
	.dev	= {
		.platform_data = NULL,
	},
};
/*
static struct nokia_dsi_panel_data dsi_panel = {
		.name	= "taal",
		.reset_gpio	= 102,
		.use_ext_te	= false,
		.ext_te_gpio	= 101,
		.use_esd_check	= false,
		.set_backlight	= NULL,
};
*/
static struct nokia_dsi_panel_data dsi2_panel = {
		.name   = "taal2",
		.reset_gpio     = 104,
		.use_ext_te     = false,
		.ext_te_gpio    = 103,
		.use_esd_check  = false,
		.set_backlight  = NULL,
};
/*
static struct omap_dss_device sdp4430_lcd_device = {
	.name			= "lcd",
	.driver_name		= "taal",
	.type			= OMAP_DISPLAY_TYPE_DSI,
	.data			= &dsi_panel,
	.phy.dsi		= {
		.clk_lane	= 1,
		.clk_pol	= 0,
		.data1_lane	= 2,
		.data1_pol	= 0,
		.data2_lane	= 3,
		.data2_pol	= 0,
		.div		= {
			.lck_div	= 1,
			.pck_div	= 5,
			.regm		= 150,
			.regn		= 17,
			.regm_dispc	= 4,
			.regm_dsi	= 4,
			.lp_clk_div	= 8,
		},
	},
	.channel		= OMAP_DSS_CHANNEL_LCD,
};
*/
static struct omap_dss_device sdp4430_lcd2_device = {
	.name			= "lcd2",
	.driver_name		= "taal2",
	.type			= OMAP_DISPLAY_TYPE_DSI,
	.data			= &dsi2_panel,
	.phy.dsi		= {
		.clk_lane	= 1,
		.clk_pol	= 0,
		.data1_lane	= 2,
		.data1_pol	= 0,
		.data2_lane	= 3,
		.data2_pol	= 0,
		.div		= {
			.lck_div	= 1,
			.pck_div	= 5,
			.regm		= 150,
			.regn		= 17,
			.regm_dispc	= 4,
			.regm_dsi	= 4,
			.lp_clk_div	= 8,
		},
	},
	.channel		= OMAP_DSS_CHANNEL_LCD2,
};

static int sdp4430_panel_enable_hdmi(struct omap_dss_device *dssdev)
{
	gpio_request(HDMI_GPIO_60 , "hdmi_gpio_60");
	gpio_request(HDMI_GPIO_41 , "hdmi_gpio_41");
	gpio_direction_output(HDMI_GPIO_60, 0);
	gpio_direction_output(HDMI_GPIO_41, 0);
	gpio_set_value(HDMI_GPIO_60, 1);
	gpio_set_value(HDMI_GPIO_41, 1);
	gpio_set_value(HDMI_GPIO_60, 0);
	gpio_set_value(HDMI_GPIO_41, 0);
	mdelay(5);
	gpio_set_value(HDMI_GPIO_60, 1);
	gpio_set_value(HDMI_GPIO_41, 1);
	return 0;
}

static void sdp4430_panel_disable_hdmi(struct omap_dss_device *dssdev)
{
	gpio_set_value(HDMI_GPIO_60, 1);
	gpio_set_value(HDMI_GPIO_41, 1);
}

static __attribute__ ((unused)) void __init sdp4430_hdmi_init(void)
{
	return;
}

static struct omap_dss_device sdp4430_hdmi_device = {
	.name = "hdmi",
	.driver_name = "hdmi_panel",
	.type = OMAP_DISPLAY_TYPE_HDMI,
	.phy.dpi.data_lines = 24,
	.platform_enable = sdp4430_panel_enable_hdmi,
	.platform_disable = sdp4430_panel_disable_hdmi,
	.channel = OMAP_DSS_CHANNEL_DIGIT,
};

static int sdp4430_panel_enable_pico_DLP(struct omap_dss_device *dssdev)
{
	int i = 0;
#if 0
	gpio_request(DLP_4430_GPIO_59, "DLP DISPLAY SEL");
	gpio_direction_output(DLP_4430_GPIO_59, 0);
	gpio_request(DLP_4430_GPIO_45, "DLP PARK");
	gpio_direction_output(DLP_4430_GPIO_45, 0);
	gpio_request(DLP_4430_GPIO_40, "DLP PHY RESET");
	gpio_direction_output(DLP_4430_GPIO_40, 0);
	gpio_request(DLP_4430_GPIO_44, "DLP READY RESET");
	gpio_direction_input(DLP_4430_GPIO_44);
	mdelay(500);

	gpio_set_value(DLP_4430_GPIO_59, 1);
	gpio_set_value(DLP_4430_GPIO_45, 1);
	mdelay(1000);

	gpio_set_value(DLP_4430_GPIO_40, 1);
	mdelay(1000);

	/* FIXME with the MLO gpio changes,
		gpio read is not retuning correct value even though
		it is  set in hardware so the check is comment
		till the problem is fixed */
	/* while (i == 0)
		i = gpio_get_value(DLP_4430_GPIO_44); */
	mdelay(2000);
#endif
	return 0;
}

static void sdp4430_panel_disable_pico_DLP(struct omap_dss_device *dssdev)
{
	gpio_set_value(DLP_4430_GPIO_40, 0);
	gpio_set_value(DLP_4430_GPIO_45, 0);
	gpio_set_value(DLP_4430_GPIO_59, 0);
}

static struct omap_dss_device sdp4430_picoDLP_device = {
	.name			= "pico_DLP",
	.driver_name		= "picoDLP_panel",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines	= 24,
	.platform_enable	= sdp4430_panel_enable_pico_DLP,
	.platform_disable	= sdp4430_panel_disable_pico_DLP,
	.channel		= OMAP_DSS_CHANNEL_LCD2,
};

int acclaim_board_type(void)
{
	int ret_val = 0;
	if ( ( gpio_get_value( 49 ) == 0 ) &&
			(gpio_get_value( 50 ) == 0 ) &&
			( gpio_get_value( 51 ) == 1 )){
		ret_val = EVT1A;
		printk("\n EVT1A\n");
	}

	if ( ( gpio_get_value ( 49 ) == 0) &&
			( gpio_get_value( 50 ) == 1 ) &&
			( gpio_get_value( 51 ) == 0)){
		ret_val = EVT1B;
		printk("\nEVT1B\n");

	}

	printk("\n *** HWID IS : %d:%d:%d \n", gpio_get_value ( 49 ),gpio_get_value ( 50 ),gpio_get_value ( 51 ));	
	return ret_val;
}

static void sdp4430_panel_get_resource(void)
{
	int ret_val = 0;

	ret_val = gpio_request(36, "BOXER LCD PWR EN");
	if ( ret_val ){
		printk("%s : Could not request lcd pwr enable\n",__FUNCTION__);
	} 
	ret_val = gpio_request(38, "BOXER BL PWR EN");
	
	if ( ret_val ) {
		printk("%s : Could not request bl pwr en\n",__FUNCTION__);
	}

	ret_val = gpio_request(44, "BOXER CABC0");
	if ( ret_val ){
		printk( "%s : could not request CABC0\n",__FUNCTION__);
	}

	ret_val = gpio_request(45, "BOXER CABC1");
	if ( ret_val ) {
		printk("%s: could not request CABC1\n",__FUNCTION__);
	}
}

static int sdp4430_panel_enable_boxer(struct omap_dss_device *dssdev)
{
	gpio_request(36, "BOXER LCD PWR EN");
	gpio_direction_output(36, 1);
	gpio_direction_output(38, 0);
	gpio_direction_output(44, 0);
	gpio_direction_output(45, 0);

	return 0;
}

static void sdp4430_panel_disable_boxer(struct omap_dss_device *dssdev)
{
	printk( KERN_INFO " boxer : %s : %d\n", __FUNCTION__, __LINE__);
	gpio_direction_output(38, 1);
	gpio_direction_output(36, 0);
}

static struct omap_dss_device sdp4430_boxer_device = {
	.name			= "boxerLCD",
	.driver_name		= "boxer_panel_drv",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines	= 24,
	.platform_enable	= sdp4430_panel_enable_boxer,
	.platform_disable	= sdp4430_panel_disable_boxer,
	.channel		= OMAP_DSS_CHANNEL_LCD2,
};

static struct omap_dss_device *sdp4430_dss_devices[] = {
	&sdp4430_boxer_device,
};

static struct omap_dss_board_info sdp4430_dss_data = {
	.num_devices	=	ARRAY_SIZE(sdp4430_dss_devices),
	.devices	=	sdp4430_dss_devices,
	.default_device	=	&sdp4430_boxer_device,
};


int plat_kim_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* TODO: wait for HCI-LL sleep */
	return 0;
}
int plat_kim_resume(struct platform_device *pdev)
{
	return 0;
}
/* wl128x BT, FM, GPS connectivity chip */
struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = 55,
	.dev_name = BLUETOOTH_UART_DEV_NAME,
	.flow_cntrl = 1,
	.baud_rate = 3000000,
	.suspend = plat_kim_suspend,
	.resume = plat_kim_resume,
};
static struct platform_device wl128x_device = {
	.name		= "kim",
	.id		= -1,
	.dev.platform_data = &wilink_pdata,
};
static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

#ifdef CONFIG_TI_ST
static bool is_bt_active(void)
{
	struct platform_device  *pdev;
	struct kim_data_s       *kim_gdata;

	pdev = &wl128x_device;
	kim_gdata = dev_get_drvdata(&pdev->dev);
	if (st_ll_getstate(kim_gdata->core_data) != ST_LL_ASLEEP &&
			st_ll_getstate(kim_gdata->core_data) != ST_LL_INVALID)
		return true;
	else
		return false;
}
#else
#define is_bt_active NULL
#endif

static struct platform_device sdp4430_aic3100 = {
        .name           = "tlv320aic3100",
        .id             = -1,
};

static struct platform_device *sdp4430_devices[] __initdata = {
	&sdp4430_disp_led,
	//&sdp4430_proximity_device,
	&sdp4430_leds_pwm,
	&sdp4430_leds_gpio,
	//&wl128x_device,
	//&sdp4430_hdmi_audio_device,
	&acclaim_keys_gpio,
};

static void __init omap_4430sdp_init_irq(void)
{
	omap2_init_common_hw(NULL, NULL);
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(1);
#endif
	gic_init_irq();
	sr_class3_init();
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_UTMI,
#ifdef CONFIG_USB_MUSB_OTG
	.mode			= MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode			= MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode			= MUSB_PERIPHERAL,
#endif
	.power			= 100,
};

#ifndef CONFIG_TIWLAN_SDIO
static int wifi_set_power(struct device *dev, int slot, int power_on, int vdd)
{
	static int power_state;

	pr_debug("Powering %s wifi", (power_on ? "on" : "off"));

	if (power_on == power_state)
		return 0;
	power_state = power_on;

	if (power_on) {
		gpio_set_value(GPIO_WIFI_PMENA, 1);
		mdelay(15);
		gpio_set_value(GPIO_WIFI_PMENA, 0);
		mdelay(1);
		gpio_set_value(GPIO_WIFI_PMENA, 1);
		mdelay(70);
	} else {
		gpio_set_value(GPIO_WIFI_PMENA, 0);
	}

	return 0;
}
#endif

static struct twl4030_usb_data omap4_usbphy_data = {
	.phy_init	= omap4430_phy_init,
	.phy_exit	= omap4430_phy_exit,
	.phy_power	= omap4430_phy_power,
	.phy_set_clock	= omap4430_phy_set_clk,
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA |
					MMC_CAP_1_8V_DDR,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195,
		.nonremovable   = true,
#ifdef CONFIG_PM_RUNTIME
		.power_saving	= true,
#endif
	},
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA |
					MMC_CAP_1_8V_DDR,
		.gpio_wp	= -EINVAL,
#ifdef CONFIG_PM_RUNTIME
		.power_saving	= true,
#endif
	},
#ifdef CONFIG_TIWLAN_SDIO
	{
		.mmc		= 3,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp        = 4,
		.ocr_mask	= MMC_VDD_165_195,
	},
#else
	{
		.mmc		= 5,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195,
		.nonremovable	= true,
	},
#endif
	{}	/* Terminator */
};

#ifndef CONFIG_TIWLAN_SDIO
static struct wl12xx_platform_data omap4_panda_wlan_data __initdata = {
	.irq = OMAP_GPIO_IRQ(GPIO_WIFI_IRQ),
	.board_ref_clock = WL12XX_REFCLOCK_26,
	.board_tcxo_clock = 1,
};
#endif

static struct regulator_consumer_supply sdp4430_vmmc_supply[] = {
	{
		.supply = "vmmc",
		.dev_name = "mmci-omap-hs.0",
	},
};

static struct regulator_consumer_supply sdp4430_cam2_supply[] = {
	{
		.supply = "cam2pwr",
	},
};
static int omap4_twl6030_hsmmc_late_init(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = container_of(dev,
				struct platform_device, dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;

	/* Setting MMC1 Card detect Irq */
	if (pdev->id == 0) {
		ret = twl6030_mmc_card_detect_config();
		if (ret)
			pr_err("Failed configuring MMC1 card detect\n");
		pdata->slots[0].card_detect_irq = TWL6030_IRQ_BASE +
						MMCDETECT_INTR_OFFSET;
		pdata->slots[0].card_detect = twl6030_mmc_card_detect;
	}

#ifndef CONFIG_TIWLAN_SDIO
	/* Set the MMC5 (wlan) power function */
	if (pdev->id == 4)
		pdata->slots[0].set_power = wifi_set_power;
#endif

	return ret;
}

static __init void omap4_twl6030_hsmmc_set_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;

	/* dev can be null if CONFIG_MMC_OMAP_HS is not set */
	if (!dev)
		return;

	pdata = dev->platform_data;
	pdata->init = omap4_twl6030_hsmmc_late_init;
}

static int __init omap4_twl6030_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
	struct omap2_hsmmc_info *c;

	omap2_hsmmc_init(controllers);
	for (c = controllers; c->mmc; c++)
		omap4_twl6030_hsmmc_set_late_init(c->dev);

	return 0;
}

static struct regulator_init_data sdp4430_vaux1 = {
	.constraints = {
		.min_uV			= 1000000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
		.always_on	= true,
	},
};

static struct regulator_consumer_supply sdp4430_vaux2_supply[] = {
	REGULATOR_SUPPLY("av-switch", "soc-audio"),
};

static struct regulator_init_data sdp4430_vaux2 = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 2800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= sdp4430_vaux2_supply,
};

static struct regulator_init_data sdp4430_vaux3 = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
		.always_on	= true,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = sdp4430_cam2_supply,
};

/* VMMC1 for MMC1 card */
static struct regulator_init_data sdp4430_vmmc = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = sdp4430_vmmc_supply,
};

static struct regulator_init_data sdp4430_vpp = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 2500000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
	},
};

static struct regulator_init_data sdp4430_vusim = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 2900000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
	},
};

static struct regulator_init_data sdp4430_vana = {
	.constraints = {
		.min_uV			= 2100000,
		.max_uV			= 2100000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
		.always_on	= true,
	},
};

static struct regulator_init_data sdp4430_vcxio = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
		.always_on	= true,
	},
};

static struct regulator_init_data sdp4430_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
		.always_on	= true,
	},
};

static struct regulator_init_data sdp4430_vusb = {
	.constraints = {
		.min_uV			= 3300000,
		.max_uV			= 3300000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 =	REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
	},
};

static struct twl4030_madc_platform_data sdp4430_gpadc_data = {
	.irq_line	= 1,
};

static int sdp4430_batt_table[] = {
	/* adc code for temperature in degree C */
	929, 925, /* -2 ,-1 */
	920, 917, 912, 908, 904, 899, 895, 890, 885, 880, /* 00 - 09 */
	875, 869, 864, 858, 853, 847, 841, 835, 829, 823, /* 10 - 19 */
	816, 810, 804, 797, 790, 783, 776, 769, 762, 755, /* 20 - 29 */
	748, 740, 732, 725, 718, 710, 703, 695, 687, 679, /* 30 - 39 */
	671, 663, 655, 647, 639, 631, 623, 615, 607, 599, /* 40 - 49 */
	591, 583, 575, 567, 559, 551, 543, 535, 527, 519, /* 50 - 59 */
	511, 504, 496 /* 60 - 62 */
};

static struct twl4030_bci_platform_data sdp4430_bci_data = {
	.monitoring_interval		= 10,
	.max_charger_currentmA		= 1500,
	.max_charger_voltagemV		= 4560,
	.max_bat_voltagemV		= 4200,
	.low_bat_voltagemV		= 3300,
	.battery_tmp_tbl		= sdp4430_batt_table,
	.tblsize			= ARRAY_SIZE(sdp4430_batt_table),
};


static int tps6130x_enable(int on)
{
        u8 val = 0;
        int ret;

        ret = twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE, &val, TWL6040_REG_GPOCTL);
        if (ret < 0) {
		pr_err("%s: failed to read GPOCTL %d\n", __func__, ret);
                return ret;
	}

	/* TWL6040 GPO2 connected to TPS6130X NRESET */
	if (on)
		val |= TWL6040_GPO2;
	else
		val &= ~TWL6040_GPO2;

	ret = twl_i2c_write_u8(TWL_MODULE_AUDIO_VOICE, val, TWL6040_REG_GPOCTL);
	if (ret < 0)
		pr_err("%s: failed to write GPOCTL %d\n", __func__, ret);

	return ret;
}

struct tps6130x_platform_data tps6130x_pdata = {
	.chip_enable	= tps6130x_enable,
};

static struct regulator_consumer_supply twl6040_vddhf_supply[] = {
	REGULATOR_SUPPLY("vddhf", "twl6040-codec"),
};

static struct regulator_init_data twl6040_vddhf = {
	.constraints = {
		.min_uV			= 4075000,
		.max_uV			= 4950000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(twl6040_vddhf_supply),
	.consumer_supplies	= twl6040_vddhf_supply,
	.driver_data		= &tps6130x_pdata,
};

#if 0
static void omap4_audio_conf(void)
{
	/* twl6040 naudint */
	omap_mux_init_signal("sys_nirq2.sys_nirq2", \
		OMAP_PIN_INPUT_PULLUP);
}
static struct twl4030_codec_audio_data twl6040_audio = {
	.vddhf_uV	= 4075000,
};

static struct twl4030_codec_vibra_data twl6040_vibra = {
	.max_timeout	= 15000,
	.initial_vibrate = 0,
};

static struct twl4030_codec_data twl6040_codec = {
	.audio		= &twl6040_audio,
	.vibra		= &twl6040_vibra,
	.audpwron_gpio	= 127,
	.naudint_irq	= OMAP44XX_IRQ_SYS_2N,
	.irq_base	= TWL6040_CODEC_IRQ_BASE,
};
#endif
static struct regulator_init_data sdp4430_clk32kg = {
       .constraints = {
		.valid_ops_mask         = REGULATOR_CHANGE_STATUS,
       },
};

static struct twl4030_platform_data sdp4430_twldata = {
	.irq_base	= TWL6030_IRQ_BASE,
	.irq_end	= TWL6030_IRQ_END,

	/* Regulators */
	.vmmc		= &sdp4430_vmmc,
	.vpp		= &sdp4430_vpp,
	.vusim		= &sdp4430_vusim,
	.vana		= &sdp4430_vana,
	.vcxio		= &sdp4430_vcxio,
	.vdac		= &sdp4430_vdac,
	.vusb		= &sdp4430_vusb,
	.vaux1		= &sdp4430_vaux1,
	.vaux2		= &sdp4430_vaux2,
	.vaux3		= &sdp4430_vaux3,
	.usb		= &omap4_usbphy_data,
	.clk32kg        = &sdp4430_clk32kg,
	.madc           = &sdp4430_gpadc_data,
	.bci            = &sdp4430_bci_data,

	/* children */
};

static struct bq2415x_platform_data sdp4430_bqdata = {
	.max_charger_voltagemV = 4200,
	.max_charger_currentmA = 1550,
};

/*
 * The Clock Driver Chip (TCXO) on OMAP4 based SDP needs to
 * be programmed to output CLK1 based on REQ1 from OMAP.
 * By default CLK1 is driven based on an internal REQ1INT signal
 * which is always set to 1.
 * Doing this helps gate sysclk (from CLK1) to OMAP while OMAP
 * is in sleep states.
 */
static struct cdc_tcxo_platform_data sdp4430_cdc_data = {
	.buf = {
		CDC_TCXO_REQ4INT | CDC_TCXO_REQ1INT |
		CDC_TCXO_REQ4POL | CDC_TCXO_REQ3POL |
		CDC_TCXO_REQ2POL | CDC_TCXO_REQ1POL,

		CDC_TCXO_MREQ4 | CDC_TCXO_MREQ3 |
		CDC_TCXO_MREQ2 | CDC_TCXO_MREQ1,

		CDC_TCXO_LDOEN1,

		0 },
};

static struct cma3000_platform_data cma3000_platform_data = {
	.def_poll_rate = 200,
	.fuzz_x = 25,
	.fuzz_y = 25,
	.fuzz_z = 25,
	.g_range = CMARANGE_8G,
	.mode = CMAMODE_MEAS400,
	.mdthr = 0x8,
	.mdfftmr = 0x33,
	.ffthr = 0x8,
	.irqflags = IRQF_TRIGGER_HIGH,
};

static struct pico_platform_data picodlp_platform_data[] = {
	[0] = { /* DLP Controller */
		.gpio_intr = 40,
	},
};

static struct i2c_board_info __initdata sdp4430_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl6030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = OMAP44XX_IRQ_SYS_1N,
		.platform_data = &sdp4430_twldata,
	},
	{
		I2C_BOARD_INFO("bq24156", 0x6a),
		.platform_data = &sdp4430_bqdata,
	},
	{
		I2C_BOARD_INFO("cdc_tcxo_driver", 0x6c),
		.platform_data = &sdp4430_cdc_data,
	},
	{
		I2C_BOARD_INFO(KXTF9_DEVICE_ID, KXTF9_I2C_SLAVE_ADDRESS),
		.platform_data = &kxtf9_platform_data_here,
		.irq = 0,
	},
	{
		I2C_BOARD_INFO(MAX17042_DEVICE_ID, MAX17042_I2C_SLAVE_ADDRESS),
		.platform_data = &max17042_platform_data_here,
		.irq = 0,
	},
};

void __init acclaim_board_init(void)
{
	int board_type = acclaim_board_type();

	if ( board_type == EVT1A ){
		max17042_gpio_for_irq = 98;
		kxtf9_gpio_for_irq = 99;
	} else if ( board_type == EVT1B ) {
		max17042_gpio_for_irq = 65;
		kxtf9_gpio_for_irq = 66;
	}

	max17042_platform_data_here.gpio = max17042_gpio_for_irq;
	sdp4430_i2c_boardinfo[4].irq = OMAP_GPIO_IRQ(max17042_gpio_for_irq);
	kxtf9_platform_data_here.gpio = kxtf9_gpio_for_irq;
	sdp4430_i2c_boardinfo[3].irq = OMAP_GPIO_IRQ(kxtf9_gpio_for_irq);
	omap_mux_init_signal("sys_pwron_reset_out", OMAP_MUX_MODE3);
}

static struct i2c_board_info __initdata sdp4430_i2c_2_boardinfo[] = {
	{
		I2C_BOARD_INFO(CY_I2C_NAME, CYTTSP_I2C_SLAVEADDRESS),
		.platform_data = &cyttsp_platform_data,
		.irq = OMAP_GPIO_IRQ(OMAP_CYTTSP_GPIO),
	},
	{
		I2C_BOARD_INFO(FT_I2C_NAME, FT5x06_I2C_SLAVEADDRESS),
		.platform_data = &ft5x06_platform_data,
		.irq = OMAP_GPIO_IRQ(OMAP_FT5x06_GPIO),
	},

	{
		I2C_BOARD_INFO("tm12xx_ts_primary", 0x4b),
		.platform_data = &tm12xx_platform_data[0],
	},
	{
		I2C_BOARD_INFO("picoDLP_i2c_driver", 0x1b),
		.platform_data = &picodlp_platform_data[0],
	},
	{
		I2C_BOARD_INFO("tlv320aic3100", 0x18),
	},
};

static struct i2c_board_info __initdata sdp4430_i2c_3_boardinfo[] = {
	{
		I2C_BOARD_INFO("tm12xx_ts_secondary", 0x4b),
		.platform_data = &tm12xx_platform_data[1],
	},
	{
		I2C_BOARD_INFO("tmp105", 0x48),
	},
	{
		I2C_BOARD_INFO("bh1780", 0x29),
	},
};

static struct i2c_board_info __initdata sdp4430_i2c_4_boardinfo[] = {
	{
		I2C_BOARD_INFO("bmp085", 0x77),
	},
	{
		I2C_BOARD_INFO("hmc5843", 0x1e),
	},
	{
		I2C_BOARD_INFO("cma3000_accl", 0x1c),
		.platform_data = &cma3000_platform_data,
	},
};

static struct usbhs_omap_platform_data usbhs_pdata __initconst = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_OHCI_PORT_MODE_PHY_6PIN_DATSE0,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
	.phy_reset  = false,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

static struct omap_i2c_bus_board_data __initdata sdp4430_i2c_bus_pdata;
static struct omap_i2c_bus_board_data __initdata sdp4430_i2c_2_bus_pdata;
static struct omap_i2c_bus_board_data __initdata sdp4430_i2c_3_bus_pdata;
static struct omap_i2c_bus_board_data __initdata sdp4430_i2c_4_bus_pdata;

/*
 * LPDDR2 Configeration Data:
 * The memory organisation is as below :
 *	EMIF1 - CS0 -	2 Gb
 *		CS1 -	2 Gb
 *	EMIF2 - CS0 -	2 Gb
 *		CS1 -	2 Gb
 *	--------------------
 *	TOTAL -		8 Gb
 *
 * Same devices installed on EMIF1 and EMIF2
 */
static __initdata struct emif_device_details emif_devices_samsung = {
	.cs0_device = &samsung_4G_S4,
	.cs1_device = 0
};

static __initdata struct emif_device_details emif_devices_elpida = {
	.cs0_device = &elpida_2G_S4,
	.cs1_device = &elpida_2G_S4
};

static void __init omap_i2c_hwspinlock_init(int bus_id, unsigned int
			spinlock_id, struct omap_i2c_bus_board_data *pdata)
{
	pdata->handle = hwspinlock_request_specific(spinlock_id);
	if (pdata->handle != NULL) {
		pdata->hwspinlock_lock = hwspinlock_lock;
		pdata->hwspinlock_unlock = hwspinlock_unlock;
	} else {
		pr_err("I2C hwspinlock request failed for bus %d\n", bus_id);
	}
}

static int __init omap4_i2c_init(void)
{
	omap_i2c_hwspinlock_init(1, 0, &sdp4430_i2c_bus_pdata);
	omap_i2c_hwspinlock_init(2, 1, &sdp4430_i2c_2_bus_pdata);
	omap_i2c_hwspinlock_init(3, 2, &sdp4430_i2c_3_bus_pdata);
	omap_i2c_hwspinlock_init(4, 3, &sdp4430_i2c_4_bus_pdata);

	regulator_has_full_constraints();

	/*
	 * Phoenix Audio IC needs I2C1 to
	 * start with 400 KHz or less
	 */
	omap_register_i2c_bus(1, 400, &sdp4430_i2c_bus_pdata,
		sdp4430_i2c_boardinfo, ARRAY_SIZE(sdp4430_i2c_boardinfo));
	omap_register_i2c_bus(2, 400, &sdp4430_i2c_2_bus_pdata,
		sdp4430_i2c_2_boardinfo, ARRAY_SIZE(sdp4430_i2c_2_boardinfo));
	omap_register_i2c_bus(3, 400, &sdp4430_i2c_3_bus_pdata,
		sdp4430_i2c_3_boardinfo, ARRAY_SIZE(sdp4430_i2c_3_boardinfo));
	omap_register_i2c_bus(4, 400, &sdp4430_i2c_4_bus_pdata,
		sdp4430_i2c_4_boardinfo, ARRAY_SIZE(sdp4430_i2c_4_boardinfo));
	return 0;
}

static void omap_cma3000accl_init(void)
{
	if (gpio_request(OMAP4_CMA3000ACCL_GPIO, "Accelerometer") < 0) {
		pr_err("Accelerometer GPIO request failed\n");
		return;
	}
	gpio_direction_input(OMAP4_CMA3000ACCL_GPIO);
}

/*
 * As OMAP4430 mux HSI and USB signals, when HSI is used (for instance HSI
 * modem is plugged) we should configure HSI pad conf and disable some USB
 * configurations.
 * HSI usage is declared using bootargs variable:
 * board-4430sdp.modem_ipc=hsi
 * Any other or missing value will not setup HSI pad conf, and port_mode[0]
 * will be used by USB.
 * Variable modem_ipc is used to catch bootargs parameter value.
 */

static char *modem_ipc = "n/a";

module_param(modem_ipc, charp, 0);
MODULE_PARM_DESC(modem_ipc, "Modem IPC setting");

static void omap_4430hsi_pad_conf(void)
{
	/*
	 * HSI pad conf: hsi1_ca/ac_wake/flag/data/ready
	 * Also configure gpio_92/95/157/187 used by modem
	 */

	/* hsi1_cawake */
	omap_mux_init_signal("usbb1_ulpitll_clk.hsi1_cawake", \
		OMAP_PIN_INPUT_PULLDOWN | \
		OMAP_PIN_OFF_NONE | \
		OMAP_PIN_OFF_WAKEUPENABLE);
	/* hsi1_caflag */
	omap_mux_init_signal("usbb1_ulpitll_dir.hsi1_caflag", \
		OMAP_PIN_INPUT | \
		OMAP_PIN_OFF_NONE);
	/* hsi1_cadata */
	omap_mux_init_signal("usbb1_ulpitll_stp.hsi1_cadata", \
		OMAP_PIN_INPUT | \
		OMAP_PIN_OFF_NONE);
	/* hsi1_acready */
	omap_mux_init_signal("usbb1_ulpitll_nxt.hsi1_acready", \
		OMAP_PIN_OUTPUT | \
		OMAP_PIN_OFF_OUTPUT_LOW);
	/* hsi1_acwake */
	omap_mux_init_signal("usbb1_ulpitll_dat0.hsi1_acwake", \
		OMAP_PIN_OUTPUT | \
		OMAP_PIN_OFF_NONE);
	/* hsi1_acdata */
	omap_mux_init_signal("usbb1_ulpitll_dat1.hsi1_acdata", \
		OMAP_PIN_OUTPUT | \
		OMAP_PIN_OFF_NONE);
	/* hsi1_acflag */
	omap_mux_init_signal("usbb1_ulpitll_dat2.hsi1_acflag", \
		OMAP_PIN_OUTPUT | \
		OMAP_PIN_OFF_NONE);
	/* hsi1_caready */
	omap_mux_init_signal("usbb1_ulpitll_dat3.hsi1_caready", \
		OMAP_PIN_INPUT | \
		OMAP_PIN_OFF_NONE);
	/* gpio_92 */
	omap_mux_init_signal("usbb1_ulpitll_dat4.gpio_92", \
		OMAP_PULL_ENA);
	/* gpio_95 */
	omap_mux_init_signal("usbb1_ulpitll_dat7.gpio_95", \
		OMAP_PIN_INPUT_PULLDOWN | \
		OMAP_PIN_OFF_NONE);
	/* gpio_157 */
	omap_mux_init_signal("usbb2_ulpitll_clk.gpio_157", \
		OMAP_PIN_OUTPUT | \
		OMAP_PIN_OFF_NONE);
	/* gpio_187 */
	omap_mux_init_signal("sys_boot3.gpio_187", \
		OMAP_PIN_OUTPUT | \
		OMAP_PIN_OFF_NONE);
}

static void enable_board_wakeup_source(void)
{
	int gpio_val;
	/* Android does not have touchscreen as wakeup source */
#if !defined(CONFIG_ANDROID)
	gpio_val = omap_mux_get_gpio(OMAP4_TOUCH_IRQ_1);
	if ((gpio_val & OMAP44XX_PADCONF_WAKEUPENABLE0) == 0) {
		gpio_val |= OMAP44XX_PADCONF_WAKEUPENABLE0;
		omap_mux_set_gpio(gpio_val, OMAP4_TOUCH_IRQ_1);
	}

#endif
	gpio_val = omap_mux_get_gpio(32);

	if ((gpio_val & OMAP44XX_PADCONF_WAKEUPENABLE0) == 0) {
		gpio_val |= OMAP44XX_PADCONF_WAKEUPENABLE0;
		omap_mux_set_gpio(gpio_val, 32);
	}

	gpio_val = omap_mux_get_gpio(29);

	if ((gpio_val & OMAP44XX_PADCONF_WAKEUPENABLE0) == 0) {
		gpio_val |= OMAP44XX_PADCONF_WAKEUPENABLE0;
		omap_mux_set_gpio(gpio_val, 29);
	}


	gpio_val = omap_mux_get_gpio(OMAP4_SFH7741_SENSOR_OUTPUT_GPIO);
	if ((gpio_val & OMAP44XX_PADCONF_WAKEUPENABLE0) == 0) {
		gpio_val |= OMAP44XX_PADCONF_WAKEUPENABLE0;
		omap_mux_set_gpio(gpio_val, OMAP4_SFH7741_SENSOR_OUTPUT_GPIO);
	}
	/*
	 * Enable IO daisy for sys_nirq1/2, to be able to
	 * wakeup from interrupts from PMIC/Audio IC.
	 * Needed only in Device OFF mode.
	 */
	omap_mux_enable_wakeup("sys_nirq1");
	omap_mux_enable_wakeup("sys_nirq2");

	if (!strcmp(modem_ipc, "hsi")) {
		/*
		 * Enable IO daisy for HSI CAWAKE line, to be able to
		 * wakeup from interrupts from Modem.
		 * Needed only in Device OFF mode.
		 */
		omap_mux_enable_wakeup("usbb1_ulpitll_clk.hsi1_cawake");
	}

}

static struct omap_volt_pmic_info omap_pmic_core = {
	.name = "twl",
	.slew_rate = 4000,
	.step_size = 12500,
	.i2c_addr = 0x12,
	.i2c_vreg = 0x61,
	.i2c_cmdreg = 0x62,
	.vsel_to_uv = omap_twl_vsel_to_uv,
	.uv_to_vsel = omap_twl_uv_to_vsel,
	.onforce_cmd = omap_twl_onforce_cmd,
	.on_cmd = omap_twl_on_cmd,
	.sleepforce_cmd = omap_twl_sleepforce_cmd,
	.sleep_cmd = omap_twl_sleep_cmd,
	.vp_config_erroroffset = 0,
	.vp_vstepmin_vstepmin = 0x01,
	.vp_vstepmax_vstepmax = 0x04,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0xA,
	.vp_vlimitto_vddmax = 0x28,
};

static struct omap_volt_pmic_info omap_pmic_mpu = {
	.name = "twl",
	.slew_rate = 4000,
	.step_size = 12500,
	.i2c_addr = 0x12,
	.i2c_vreg = 0x55,
	.i2c_cmdreg = 0x56,
	.vsel_to_uv = omap_twl_vsel_to_uv,
	.uv_to_vsel = omap_twl_uv_to_vsel,
	.onforce_cmd = omap_twl_onforce_cmd,
	.on_cmd = omap_twl_on_cmd,
	.sleepforce_cmd = omap_twl_sleepforce_cmd,
	.sleep_cmd = omap_twl_sleep_cmd,
	.vp_config_erroroffset = 0,
	.vp_vstepmin_vstepmin = 0x01,
	.vp_vstepmax_vstepmax = 0x04,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0xA,
	.vp_vlimitto_vddmax = 0x39,
};

static struct omap_volt_pmic_info omap_pmic_iva = {
	.name = "twl",
	.slew_rate = 4000,
	.step_size = 12500,
	.i2c_addr = 0x12,
	.i2c_vreg = 0x5b,
	.i2c_cmdreg = 0x5c,
	.vsel_to_uv = omap_twl_vsel_to_uv,
	.uv_to_vsel = omap_twl_uv_to_vsel,
	.onforce_cmd = omap_twl_onforce_cmd,
	.on_cmd = omap_twl_on_cmd,
	.sleepforce_cmd = omap_twl_sleepforce_cmd,
	.sleep_cmd = omap_twl_sleep_cmd,
	.vp_config_erroroffset = 0,
	.vp_vstepmin_vstepmin = 0x01,
	.vp_vstepmax_vstepmax = 0x04,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0xA,
	.vp_vlimitto_vddmax = 0x2D,
};

static struct omap_volt_vc_data vc_config = {
	.vdd0_on = 1375000,        /* 1.375v */
	.vdd0_onlp = 1375000,      /* 1.375v */
	.vdd0_ret = 837500,       /* 0.8375v */
	.vdd0_off = 0,		/* 0 v */
	.vdd1_on = 1300000,        /* 1.3v */
	.vdd1_onlp = 1300000,      /* 1.3v */
	.vdd1_ret = 837500,       /* 0.8375v */
	.vdd1_off = 0,		/* 0 v */
	.vdd2_on = 1200000,        /* 1.2v */
	.vdd2_onlp = 1200000,      /* 1.2v */
	.vdd2_ret = 837500,       /* .8375v */
	.vdd2_off = 0,		/* 0 v */
};

void plat_hold_wakelock(void *up, int flag)
{
	struct uart_omap_port *up2 = (struct uart_omap_port *)up;
	/*Specific wakelock for bluetooth usecases*/
	if ((up2->pdev->id == BLUETOOTH_UART)
		&& ((flag == WAKELK_TX) || (flag == WAKELK_RX)))
		wake_lock_timeout(&uart_lock, 2*HZ);

	/*Specific wakelock for console usecases*/
	if ((up2->pdev->id == CONSOLE_UART)
		&& ((flag == WAKELK_IRQ) || (flag == WAKELK_RESUME)))
		wake_lock_timeout(&uart_lock, 5*HZ);
	return;
}

static struct omap_uart_port_info omap_serial_platform_data[] = {
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
		.plat_hold_wakelock = plat_hold_wakelock,
		.plat_omap_bt_active = NULL,
		.rts_padconf	= OMAP4_CTRL_MODULE_PAD_MCSPI1_CS3_OFFSET,
		.rts_override	= 0,
		.cts_padconf	= OMAP4_CTRL_MODULE_PAD_MCSPI1_CS2_OFFSET,
		.padconf	= OMAP4_CTRL_MODULE_PAD_MCSPI1_CS1_OFFSET,
		.padconf_wake_ev =
			OMAP4_CTRL_MODULE_PAD_CORE_PADCONF_WAKEUPEVENT_3,
		.wk_mask	=
			OMAP4_MCSPI1_CS1_DUPLICATEWAKEUPEVENT_MASK |
			OMAP4_MCSPI1_CS2_DUPLICATEWAKEUPEVENT_MASK |
			OMAP4_MCSPI1_CS3_DUPLICATEWAKEUPEVENT_MASK |
			OMAP4_UART3_CTS_RCTX_DUPLICATEWAKEUPEVENT_MASK,
	},
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
		.plat_hold_wakelock = plat_hold_wakelock,
		.plat_omap_bt_active = is_bt_active,
		.rts_padconf	= OMAP4_CTRL_MODULE_PAD_UART2_RTS_OFFSET,
		.rts_override	= 0,
		.cts_padconf	= OMAP4_CTRL_MODULE_PAD_UART2_CTS_OFFSET,
		.padconf	= OMAP4_CTRL_MODULE_PAD_UART2_RX_OFFSET,
		.padconf_wake_ev =
			OMAP4_CTRL_MODULE_PAD_CORE_PADCONF_WAKEUPEVENT_3,
		.wk_mask	=
			OMAP4_UART2_TX_DUPLICATEWAKEUPEVENT_MASK |
			OMAP4_UART2_RX_DUPLICATEWAKEUPEVENT_MASK |
			OMAP4_UART2_RTS_DUPLICATEWAKEUPEVENT_MASK |
			OMAP4_UART2_CTS_DUPLICATEWAKEUPEVENT_MASK,
	},
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
		.plat_hold_wakelock = plat_hold_wakelock,
		.plat_omap_bt_active = NULL,
		.rts_padconf	= 0,
		.rts_override	= 0,
		.cts_padconf	= 0,
		.padconf	= OMAP4_CTRL_MODULE_PAD_UART3_RX_IRRX_OFFSET,
		.padconf_wake_ev =
			OMAP4_CTRL_MODULE_PAD_CORE_PADCONF_WAKEUPEVENT_4,
		.wk_mask	=
			OMAP4_UART3_TX_IRTX_DUPLICATEWAKEUPEVENT_MASK |
			OMAP4_UART3_RX_IRRX_DUPLICATEWAKEUPEVENT_MASK |
			OMAP4_UART3_RTS_SD_DUPLICATEWAKEUPEVENT_MASK |
			OMAP4_UART3_CTS_RCTX_DUPLICATEWAKEUPEVENT_MASK,
	},
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
		.plat_hold_wakelock = NULL,
		.plat_omap_bt_active = NULL,
		.rts_padconf	= 0,
		.rts_override	= 0,
		.cts_padconf	= 0,
		.padconf	= OMAP4_CTRL_MODULE_PAD_UART4_RX_OFFSET,
		.padconf_wake_ev =
			OMAP4_CTRL_MODULE_PAD_CORE_PADCONF_WAKEUPEVENT_4,
		.wk_mask	=
			OMAP4_UART4_TX_DUPLICATEWAKEUPEVENT_MASK |
			OMAP4_UART4_RX_DUPLICATEWAKEUPEVENT_MASK,
	},
	{
		.flags		= 0
	}
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
#ifndef CONFIG_TIWLAN_SDIO
	/* WLAN IRQ - GPIO 53 */
	OMAP4_MUX(GPMC_NCS3, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	/* WLAN_EN - GPIO 54 */
	OMAP4_MUX(GPMC_NWP, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* WLAN SDIO: MMC5 CMD */
	OMAP4_MUX(SDMMC5_CMD, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* WLAN SDIO: MMC5 CLK */
	OMAP4_MUX(SDMMC5_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* WLAN SDIO: MMC5 DAT[0-3] */
	OMAP4_MUX(SDMMC5_DAT0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
#endif
	OMAP4_MUX(USBB1_ULPITLL_CLK, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

static void enable_rtc_gpio(void){
	/* To access twl registers we enable gpio6
	 * we need this so the RTC driver can work.
	 */
	gpio_request(TWL6030_RTC_GPIO, "h_SYS_DRM_MSEC");
	gpio_direction_output(TWL6030_RTC_GPIO, 1);

	omap_mux_init_signal("fref_clk0_out.gpio_wk6", \
		OMAP_PIN_OUTPUT | OMAP_PIN_OFF_NONE);
	return;
}

#ifndef CONFIG_TIWLAN_SDIO
static void omap4_4430sdp_wifi_init(void)
{
	if (gpio_request(GPIO_WIFI_PMENA, "wl12xx") ||
	    gpio_direction_output(GPIO_WIFI_PMENA, 0))
		pr_err("Error initializing up WLAN_EN\n");
	if (wl12xx_set_platform_data(&omap4_panda_wlan_data))
		pr_err("Error setting wl12xx data\n");
}
#endif
static void __init omap_4430sdp_init(void)
{
	int status;
	int package = OMAP_PACKAGE_CBS;

	if (omap_rev() == OMAP4430_REV_ES1_0)
		package = OMAP_PACKAGE_CBL;
	omap4_mux_init(board_mux, package);
	acclaim_board_init();

	if (sdram_vendor() == SAMSUNG_SDRAM)	
	{	
	omap_emif_setup_device_details(&emif_devices_samsung, &emif_devices_samsung);
	printk(KERN_INFO"Samsung DDR Memory \n");
	}
	else if (sdram_vendor() == ELPIDA_SDRAM)
	{
	omap_emif_setup_device_details(&emif_devices_elpida, &emif_devices_elpida);
	printk(KERN_INFO"Elpida DDR Memory \n");
	} else if (sdram_vendor() == HYNIX_SDRAM) {
		/* Re-use ELPIDA timings as they are absolutely the same */
		omap_emif_setup_device_details(&emif_devices_elpida, &emif_devices_elpida);
		printk(KERN_INFO"Hynix DDR Memory \n");
	}
	else pr_err("Memory type does not exist\n");

	omap_init_emif_timings();

	enable_rtc_gpio();
	//omap4_audio_conf();
	omap4_i2c_init();
	//omap4_display_init();
	//omap_disp_led_init();
	platform_add_devices(sdp4430_devices, ARRAY_SIZE(sdp4430_devices));
	acclaim_init_charger();

	wake_lock_init(&uart_lock, WAKE_LOCK_SUSPEND, "uart_wake_lock");
	omap_serial_init(omap_serial_platform_data);
	omap4_twl6030_hsmmc_init(mmc);

#ifdef CONFIG_TIWLAN_SDIO
	//config_wlan_mux();
#else
	//omap4_4430sdp_wifi_init();
#endif

	kxtf9_dev_init();
#ifdef CONFIG_BATTERY_MAX17042
	max17042_dev_init();
#endif

	/* Power on the ULPI PHY */
	if (gpio_is_valid(OMAP4SDP_MDM_PWR_EN_GPIO)) {
		/* FIXME: Assumes pad is muxed for GPIO mode */
		gpio_request(OMAP4SDP_MDM_PWR_EN_GPIO, "USBB1 PHY VMDM_3V3");
		gpio_direction_output(OMAP4SDP_MDM_PWR_EN_GPIO, 1);
	}

	/*
	 * Test board-4430sdp.modem_ipc bootargs value to detect if HSI pad
	 * conf is required
	 */
	pr_info("Configured modem_ipc: %s", modem_ipc);
	if (!strcmp(modem_ipc, "hsi")) {
		pr_info("Modem HSI detected, set USB port_mode[0] as UNUSED");
		/* USBB1 I/O pads conflict with HSI1 port */
		usbhs_pdata.port_mode[0] = OMAP_USBHS_PORT_MODE_UNUSED;
		/* Setup HSI pad conf for OMAP4430 platform */
		omap_4430hsi_pad_conf();
	} else
		pr_info("Modem HSI not detected");

	usb_uhhtll_init(&usbhs_pdata);
	usb_musb_init(&musb_board_data);

	status = omap4_keypad_initialization(&sdp4430_keypad_data);
	if (status)
		pr_err("Keypad initialization failed: %d\n", status);

	spi_register_board_info(sdp4430_spi_board_info,
			ARRAY_SIZE(sdp4430_spi_board_info));

	//omap_sfh7741prox_init();
	//omap_cma3000accl_init();
	sdp4430_panel_get_resource();
	omap_display_init(&sdp4430_dss_data);
	enable_board_wakeup_source();
	omap_voltage_register_pmic(&omap_pmic_core, "core");
	omap_voltage_register_pmic(&omap_pmic_mpu, "mpu");
	omap_voltage_register_pmic(&omap_pmic_iva, "iva");
	omap_voltage_init_vc(&vc_config);
}

static void __init omap_4430sdp_map_io(void)
{
	omap2_set_globals_443x();
	omap44xx_map_common_io();
}

MACHINE_START(OMAP_4430SDP, "OMAP4430")
	/* Maintainer: Santosh Shilimkar - Texas Instruments Inc */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap_4430sdp_map_io,
	.init_irq	= omap_4430sdp_init_irq,
	.init_machine	= omap_4430sdp_init,
	.timer		= &omap_timer,
MACHINE_END
