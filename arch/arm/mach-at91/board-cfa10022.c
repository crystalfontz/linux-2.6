/*
 * Platform configuration for a Crystalfontz CFA-10022
 *
 * Copyright 2009 Crystalfontz America, Inc. <www.crystalfontz.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/gpio_keys.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/leds.h>
#include <linux/atmel-mci.h>
#include <linux/clk.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/irq.h>

#include <mach/hardware.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/at91sam9_smc.h>

#include "generic.h"

#include <video/broadsheetfb.h>
#include <linux/i2c/ltc3576.h>
#include <linux/i2c/tps65180.h>
#include <linux/inverted_gpio.h>

#define INVERTED_GPIO_START 	(PIN_BASE + (MAX_GPIO_BANKS)*32)
#define INVERTED_GPIO_0		(INVERTED_GPIO_START+0)
#define INVERTED_GPIO_1		(INVERTED_GPIO_START+1)

static struct inverted_gpio_data __initdata cfa_10022_inverted_gpios[] = {
	{
		.actual_gpio	= AT91_PIN_PC28,
		.name		= "TPS65180 Wakeup",
		.real_inverter	= 1,
	},
	{
		.actual_gpio	= AT91_PIN_PB25,
		.name		= "OTG Enable",
		.real_inverter	= 0,
	},
};

static struct inverted_gpio_platform_data __initdata cfa_10022_inverted_gpio_platform_data = {
	.gpio_base	= INVERTED_GPIO_START,
	.gpio_count	= ARRAY_SIZE(cfa_10022_inverted_gpios),
	.inverted_gpios = cfa_10022_inverted_gpios,
};

static struct platform_device __initdata cfa_10022_inverted_gpio_device = {
	.name		= "inverted_gpio",
	.id		= -1,
	.dev		= {
				.platform_data		= &cfa_10022_inverted_gpio_platform_data,
	},
	.num_resources	= 0,
};

#define EXP_MASK_SERIAL0	(1<<0)
#define EXP_MASK_SERIAL1	(1<<1)
#define EXP_MASK_SPI		(1<<2)

static unsigned int exp_mask = 0x0;
module_param(exp_mask, int, 0);

static void __init cfa_10022_map_io(void)
{
	/* Initialize processor: 12.000 MHz crystal */
	at91sam9g45_initialize(12000000);

	/*  It is up to the boot loader to have configured these pins,
	    so we know to keep using them. */
	if (at91_is_A_periph(AT91_PIN_PB12) && at91_is_A_periph(AT91_PIN_PB13)) {
		/* DGBU on ttyS0. (Rx & Tx only) */
		at91_register_uart(0, 0, 0);
		/* set early serial console to ttyS0 (ie, DBGU) */
		at91_set_serial_console(0);
	}
}

static void __init cfa_10022_init_irq(void)
{
	at91sam9g45_init_interrupts(NULL);
}


/*
 * Serial Ports
 */
static void __init cfa_10022_serial_init(void)
{
	/* USART1 on ttyS1. (Rx, Tx) */
	if (exp_mask & EXP_MASK_SERIAL0)
		at91_register_uart(AT91SAM9G45_ID_US1, 1, 0);

	/* USART1 on ttyS2. (Rx, Tx) */
	if (exp_mask & EXP_MASK_SERIAL1)
		at91_register_uart(AT91SAM9G45_ID_US2, 2, 0);
}


/*
 * USB HS Host port (common to OHCI & EHCI)
 */
static struct at91_usbh_data __initdata cfa_10022_usbh_hs_data = {
	.ports		= 2,
	.vbus_pin	= {AT91_PIN_PB24, INVERTED_GPIO_1},
};


/*
 * USB HS Device port
 */
static struct usba_platform_data __initdata cfa_10022_usba_udc_data = {
	.vbus_pin	= AT91_PIN_PA6,
};


/*
 * MCI (SD/MMC)
 */
static void mci0_poweroff(void)
{
	/* MCI0 power off */
	at91_set_gpio_output(AT91_PIN_PB21, 1);

	/* CLK */
	at91_set_gpio_input(AT91_PIN_PA0, 0);

	/* CMD */
	at91_set_gpio_input(AT91_PIN_PA1, 0);

	/* DAT0, maybe DAT1..DAT3 and maybe DAT4..DAT7 */
	at91_set_gpio_input(AT91_PIN_PA2, 0);
	at91_set_gpio_input(AT91_PIN_PA3, 0);
	at91_set_gpio_input(AT91_PIN_PA4, 0);
	at91_set_gpio_input(AT91_PIN_PA5, 0);

	printk(KERN_INFO "mci0 powered off\n");
}

static void mci0_poweron(void)
{
	/* MCI1 power on */
	at91_set_gpio_output(AT91_PIN_PB21, 0);

	/* CLK */
	at91_set_A_periph(AT91_PIN_PA0, 0);

	/* CMD */
	at91_set_A_periph(AT91_PIN_PA1, 1);

	/* DAT0, maybe DAT1..DAT3 and maybe DAT4..DAT7 */
	at91_set_A_periph(AT91_PIN_PA2, 1);
	at91_set_A_periph(AT91_PIN_PA3, 1);
	at91_set_A_periph(AT91_PIN_PA4, 1);
	at91_set_A_periph(AT91_PIN_PA5, 1);

	printk(KERN_INFO "mci0 powered on\n");
}

static struct mci_platform_data __initdata mci0_data = {
	/* Micro SD Slot */
	.slot[0] = {
		.bus_width	= 4,
		.detect_pin	= -1,
		.wp_pin		= -1,
		.poweroff	= mci0_poweroff,
		.poweron	= mci0_poweron,
	},
};

static void mci1_poweroff(void)
{
	/* MCI1 power off */
	at91_set_gpio_output(AT91_PIN_PB23, 1);

	/* CLK */
	at91_set_gpio_input(AT91_PIN_PA31, 0);

	/* CMD */
	at91_set_gpio_input(AT91_PIN_PA22, 0);

	/* DAT0..DAT3 */
	at91_set_gpio_input(AT91_PIN_PA23, 0);
	at91_set_gpio_input(AT91_PIN_PA24, 0);
	at91_set_gpio_input(AT91_PIN_PA25, 0);
	at91_set_gpio_input(AT91_PIN_PA26, 0);

	printk(KERN_INFO "mci1 powered off\n");
}

static void mci1_poweron(void)
{
	/* MCI1 power on */
	at91_set_gpio_output(AT91_PIN_PB23, 0);

	/* CLK */
	at91_set_A_periph(AT91_PIN_PA31, 0);

	/* CMD */
	at91_set_A_periph(AT91_PIN_PA22, 1);

	/* DAT0..DAT3 */
	at91_set_A_periph(AT91_PIN_PA23, 1);
	at91_set_A_periph(AT91_PIN_PA24, 1);
	at91_set_A_periph(AT91_PIN_PA25, 1);
	at91_set_A_periph(AT91_PIN_PA26, 1);

	printk(KERN_INFO "mci1 powered on\n");
}

static struct mci_platform_data __initdata mci1_data = {
	/* Full size SD Slot */
	.slot[0] = {
		.bus_width	= 4,
		.detect_pin	= AT91_PIN_PB22,
		.wp_pin		= AT91_PIN_PB9,
		.poweroff	= mci1_poweroff,
		.poweron	= mci1_poweron,
	},
};


/*
 * MACB Ethernet device
 */
static struct at91_eth_data __initdata cfa_10022_macb_data = {
	.phy_irq_pin	= AT91_PIN_PD5,
	.is_rmii	= 1,
};


/*
 * Touchscreen
 */
static struct at91_tsadcc_data cfa_10022_tsadcc_data = {
	.adc_clock		= 250000,
	.pendet_debounce	= 0x0d,
	.ts_sample_hold_time	= 0x0f, // only low 4-bits
};

/*
 * GPIO Buttons
 */
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
static struct gpio_keys_button cfa_10022_buttons[] = {
	{	/* S1, "rightclick" */
		.code		= BTN_RIGHT,
		.gpio		= AT91_PIN_PD30,
		.active_low	= 1,
		.desc		= "right_click",
		.wakeup		= 1,
		.debounce_interval = 70,
	},
	{	/* S2, "leftclick" */
		.code		= BTN_LEFT,
		.gpio		= AT91_PIN_PD29,
		.active_low	= 1,
		.desc		= "left_click",
		.wakeup		= 1,
		.debounce_interval = 70,
	},
	{	/* SW_WAKE, "wakeup/shutdown gpio" */
		.code		= KEY_POWER,
		.gpio		= AT91_PIN_PB30,
		.active_low	= 1,
		.desc		= "wake_shdn",
		.wakeup		= 1,
		.debounce_interval = 70,
	},
};

static struct gpio_keys_platform_data cfa_10022_button_data = {
	.buttons	= cfa_10022_buttons,
	.nbuttons	= ARRAY_SIZE(cfa_10022_buttons),
};

static struct platform_device cfa_10022_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &cfa_10022_button_data,
	}
};

static void __init cfa_10022_add_device_buttons(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cfa_10022_buttons); i++) {
		at91_set_GPIO_periph(cfa_10022_buttons[i].gpio, 1);
		at91_set_deglitch(cfa_10022_buttons[i].gpio, 1);
	}

	platform_device_register(&cfa_10022_button_device);
}
#else
static void __init cfa_10022_add_device_buttons(void) {}
#endif


/*
 * LEDs
 */
static struct gpio_led cfa_10022_leds[] = {
	{	/* "LED1" label, red, schematic LED2 */
		.name			= "LED1",
		.gpio			= AT91_PIN_PD0,
		.active_low		= 1,
		.default_trigger	= "none",
	},
	{	/* "LED2" label, yellow, schematic LED3 */
		.name			= "LED2",
		.gpio			= AT91_PIN_PD1,
		.active_low		= 1,
		.default_trigger	= "none",
	},
	{	/* "LED3" label, green, schematic LED4 */
		.name			= "LED3",
		.gpio			= AT91_PIN_PD2,
		.active_low		= 1,
		.default_trigger	= "mmc0",
	},
	{	/* "PWR" label, amber, schematic LED1 */
		.name			= "PWR",
		.gpio			= AT91_PIN_PD31,
		.default_trigger	= "heartbeat",
	},
};

/*
 * E-Ink MMIO
 */
#define EINK_BASE		AT91_CHIPSELECT_4
#define EINK_RDY_GPIO_PIN	AT91_PIN_PC15

static int eink_mmio_offset = 0;
static struct platform_device *eink_device;
static void* eink_virtual_mmio = 0;

static int eink_wait_event(struct broadsheetfb_par *par)
{
	wait_event(par->waitq, gpio_get_value(EINK_RDY_GPIO_PIN));
	return 0;
}

static int eink_init_board(struct broadsheetfb_par *par)
{
	struct resource	*mem;

	mem = platform_get_resource(eink_device, IORESOURCE_MEM, 0);
	if (!mem)
		return -ENXIO;

	eink_virtual_mmio = ioremap(mem->start, mem->end - mem->start + 1);
	if (!eink_virtual_mmio)
		return -ENXIO;

	/* Take the EINK processor out of Reset */
	at91_set_gpio_output(AT91_PIN_PC31, 0);

	return 0;
}

static int eink_init_board_pre_run(struct broadsheetfb_par *par)
{
	/* Wakeup the TPS65180 out of Sleep */
	return tps65180_wakeup(1);
}

static void eink_cleanup(struct broadsheetfb_par *par)
{
	/* Sleep the TPS65180 */
	tps65180_wakeup(0);

	/* Put the EINK processor in Reset */
	at91_set_gpio_output(AT91_PIN_PC31, 1);

	free_irq(gpio_to_irq(EINK_RDY_GPIO_PIN), par);

	iounmap(eink_virtual_mmio);
}

static u16 eink_get_hdb(struct broadsheetfb_par *par)
{
	return __raw_readw(eink_virtual_mmio+eink_mmio_offset);
}

static void eink_set_hdb(struct broadsheetfb_par *par, u16 data)
{
	__raw_writew(data, eink_virtual_mmio+eink_mmio_offset);
}

static void eink_set_ctl(struct broadsheetfb_par *par, unsigned char bit,
				u8 state)
{
	switch (bit) {
	case BS_WR:
		break;
	case BS_CS:
		if (state)
			enable_irq(gpio_to_irq(EINK_RDY_GPIO_PIN));
		else
			disable_irq_nosync(gpio_to_irq(EINK_RDY_GPIO_PIN));

#if 0
		gpio_set_value(AT91_PIN_PD1,state);
#endif
		break;
	case BS_DC:
#if 0
		gpio_set_value(AT91_PIN_PD0,state);
#endif
		if (!state)
			eink_mmio_offset = 0;
		else
			eink_mmio_offset = 2;
		break;
	}
}

static int eink_get_panel_type(void)
{
	return 6;
}

static irqreturn_t eink_handle_irq(int irq, void *dev_id)
{
	struct broadsheetfb_par *par = dev_id;

	wake_up(&par->waitq);
	return IRQ_HANDLED;
}

static int eink_setup_irq(struct fb_info *info)
{
	int ret;
	struct broadsheetfb_par *par = info->par;

	ret = gpio_request(EINK_RDY_GPIO_PIN, "eink_ready");
	if (ret)
		dev_err(&eink_device->dev, "no HRDY pin available\n");

	if (!ret) {
		ret = request_irq(gpio_to_irq(EINK_RDY_GPIO_PIN), eink_handle_irq,
				  IRQF_DISABLED | IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				  "eink-ready", par);
		if (ret) {
			gpio_free(EINK_RDY_GPIO_PIN);
			dev_err(&eink_device->dev, "request_irq %d failed: %d\n", gpio_to_irq(EINK_RDY_GPIO_PIN), ret);
		}
	}

	return ret;
}

static struct resource eink_resources[] = {
	[0] = {
		.start	= EINK_BASE,
		.end	= EINK_BASE +  + SZ_16K - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static int eink_get_ext_temp(void)
{
	/* Wakeup the TPS65180 out of Sleep */
	tps65180_temp_conv_start();
	return tps65180_temp_read();
}

static struct broadsheet_board eink_board = {
	.owner			= THIS_MODULE,
	.init			= eink_init_board,
	.init_pre_run		= eink_init_board_pre_run,
	.cleanup		= eink_cleanup,
	.set_hdb		= eink_set_hdb,
	.get_hdb		= eink_get_hdb,
	.set_ctl		= eink_set_ctl,
	.wait_for_rdy		= eink_wait_event,
	.get_panel_type		= eink_get_panel_type,
	.setup_irq		= eink_setup_irq,
	.get_ext_temp		= eink_get_ext_temp,
};

static void __init add_device_eink(void)
{
	request_module("broadsheetfb");

	eink_device = platform_device_alloc("broadsheetfb", -1);

	platform_device_add_resources(eink_device, eink_resources, ARRAY_SIZE(eink_resources));

	platform_device_add_data(eink_device, &eink_board, sizeof(eink_board));

	platform_device_add(eink_device);
}



static void __init init_pck_for_ethernet(void)
{
	struct clk *pck0, *plla;
	unsigned long rate = 50000000;
	int ret;

	pck0 = clk_get(NULL,"pck0");
	if (!pck0) {
		printk(KERN_ERR "PCK0 clock not found!\n");
		return;
	}

	plla = clk_get(NULL,"plla");
	if (!plla) {
		printk(KERN_ERR "PCK0 clock setup could not find plla clock!\n");
		return;
	}

	ret = clk_set_parent(pck0,plla);
	if (ret != 0) {
		printk(KERN_ERR "PCK0 clock setup could not set plla as parent clock (ret=%d)!\n",ret);
		return;
	}

	rate = clk_set_rate(pck0,rate);
	clk_enable(pck0);
	printk(KERN_INFO "PCK0 clock set to %lu\n", rate);

	/* PCK0 */
	at91_set_A_periph(AT91_PIN_PD26, 1);
}


static void __init init_pck_for_eink(void)
{
	struct clk *pck1, *plla;
	unsigned long rate = 25000000;
	int ret;

	pck1 = clk_get(NULL,"pck1");
	if (!pck1) {
		printk(KERN_ERR "PCK1 clock not found!\n");
		return;
	}

	plla = clk_get(NULL,"plla");
	if (!plla) {
		printk(KERN_ERR "PCK1 clock setup could not find plla clock!\n");
		return;
	}

	ret = clk_set_parent(pck1,plla);
	if (ret != 0) {
		printk(KERN_ERR "PCK1 clock setup could not set plla as parent clock (ret=%d)!\n",ret);
		return;
	}

	rate = clk_set_rate(pck1,rate);
	clk_enable(pck1);
	printk(KERN_INFO "PCK1 clock set to %lu\n", rate);

	/* PCK1 */
	at91_set_B_periph(AT91_PIN_PB31, 1);

}

/*
 * ltc3576
 */
static struct ltc3576_platform_data __initdata ltc3576_data = {
	.nchrg_pin		= AT91_PIN_PD7,
	.nacpr_pin		= AT91_PIN_PD6,
};

/*
 * tps65180
 */
static struct tps65180_platform_data __initdata tps65180_data = {
	.wakeup_pin		= INVERTED_GPIO_0,
	.pwr_good_pin		= AT91_PIN_PC26,
	.int_pin		= AT91_PIN_PC27,
};

/*
 * I2C
 */
static struct i2c_board_info __initdata cfa10022_i2c_devices[] = {
	{
		I2C_BOARD_INFO("ltc3576", 0x09),
		.platform_data = &ltc3576_data,
	},
	{
		I2C_BOARD_INFO("tps65180", 0x48),
		.platform_data = &tps65180_data,
	},
};

static void __init cfa_10022_board_init(void)
{
	printk(KERN_INFO "CFA-10022 exp_mask set to 0x%08x\n", exp_mask);

        /* Turn off the pull up on nACPR */
	at91_set_gpio_input(AT91_PIN_PD6, 0);

	platform_device_register(&cfa_10022_inverted_gpio_device);

	/* Serial */
	cfa_10022_serial_init();
	at91_add_device_serial();
	/* USB HS Host */
	at91_add_device_usbh_ohci(&cfa_10022_usbh_hs_data);
	at91_add_device_usbh_ehci(&cfa_10022_usbh_hs_data);
	/* USB HS Device */
	at91_add_device_usba(&cfa_10022_usba_udc_data);
	/* SPI */
	if (exp_mask & EXP_MASK_SPI) {
		at91_add_device_spi(NULL, 0);
	}
	/* MMC */
	at91_add_device_mci(0, &mci0_data);
	at91_add_device_mci(1, &mci1_data);
	/* Enable external clk for ethernet */
	init_pck_for_ethernet();
	/* Ethernet */
	at91_add_device_eth(&cfa_10022_macb_data);
	/* I2C */
	at91_add_device_i2c(0, NULL, 0);
	at91_add_device_i2c(1, cfa10022_i2c_devices, ARRAY_SIZE(cfa10022_i2c_devices));
	/* Touch Screen */
	at91_add_device_tsadcc(&cfa_10022_tsadcc_data);
	/* Push Buttons */
	cfa_10022_add_device_buttons();
	/* LEDs */
	at91_gpio_leds(cfa_10022_leds, ARRAY_SIZE(cfa_10022_leds));

	/* E-Ink */
	init_pck_for_eink();
#if 1
	add_device_eink();
#else
        msleep(1000);
	at91_set_gpio_output(AT91_PIN_PC31, 0);
#endif
}

MACHINE_START(CFA_10022, "Crystalfontz CFA-10022")
	/* Maintainer: Crystalfontz America */
	.phys_io	= AT91_BASE_SYS,
	.io_pg_offst	= (AT91_VA_BASE_SYS >> 18) & 0xfffc,
	.boot_params	= AT91_SDRAM_BASE + 0x100,
	.timer		= &at91sam926x_timer,
	.map_io		= cfa_10022_map_io,
	.init_irq	= cfa_10022_init_irq,
	.init_machine	= cfa_10022_board_init,
MACHINE_END
