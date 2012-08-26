/*
 * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2009 IVL Audio, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @defgroup Framebuffer Framebuffer Driver for SDC and ADC.
 */

/*!
 * @file mxcfb_wandisplay_wvga.c
 *
 * @brief MXC Frame buffer driver for SDC
 *
 * @ingroup Framebuffer
 */

/*!
 * Include files
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/regulator/regulator.h>
#include <linux/mxcfb.h>

#define DEBUG
#include <linux/device.h>

static void lcd_poweron(void);
static void lcd_poweroff(void);

static struct platform_device *plcd_dev;
static struct regulator *io_reg;
static struct regulator *core_reg;
static int lcd_on;

static struct fb_videomode video_modes[] = {
	{
	 .name = "KWH035NJ11-F01",
	 .refresh = 60,
	 .xres = 320,
	 .yres = 240,
	 .pixclock = 154000,
	 .left_margin = 38,
	 .right_margin = 50,
	 .upper_margin = 18,
	 .lower_margin = 4,
	 .hsync_len = 1,
	 .vsync_len = 1,
	 .sync = 0,
	 .vmode = FB_VMODE_NONINTERLACED,
	 .flag = 0, },
};

static void lcd_init_fb(struct fb_info *info)
{
	struct fb_var_screeninfo var;

	printk("LCD: >%s\n", __func__);

	memset(&var, 0, sizeof(var));

	fb_videomode_to_var(&var, &video_modes[0]);

	var.activate = FB_ACTIVATE_ALL;
	var.yres_virtual = var.yres * 2;

	acquire_console_sem();
	info->flags |= FBINFO_MISC_USEREVENT;
	fb_set_var(info, &var);
	info->flags &= ~FBINFO_MISC_USEREVENT;
	release_console_sem();
}

static int lcd_fb_event(struct notifier_block *nb, unsigned long val, void *v)
{
	struct fb_event *event = v;

	printk("LCD: >%s\n", __func__);
	if (strcmp(event->info->fix.id, "DISP3 BG")) {
		return 0;
	}

	switch (val) {
	case FB_EVENT_FB_REGISTERED:
		printk("LCD: %s FB_EVENT_FB_REGISTERED\n", __func__);
		lcd_init_fb(event->info);
		lcd_poweron();
		break;
	case FB_EVENT_BLANK:
		printk("LCD: %s FB_EVENT_BLANK\n", __func__);
		if ((event->info->var.xres != 320) ||
		    (event->info->var.yres != 240)) {
			break;
		}
		if (*((int *)event->data) == FB_BLANK_UNBLANK) {
			lcd_poweron();
		} else {
			lcd_poweroff();
		}
		break;
	}
	return 0;
}

static struct notifier_block nb = {
	.notifier_call = lcd_fb_event,
};

/*!
 * This function is called whenever the SPI slave device is detected.
 *
 * @param	spi	the SPI slave device
 *
 * @return 	Returns 0 on SUCCESS and error on FAILURE.
 */
static int __devinit lcd_probe(struct platform_device *pdev)
{
	int i;
	struct mxc_lcd_platform_data *plat = pdev->dev.platform_data;

	printk("LCD: >%s\n", __func__);

	if (plat) {
		if (plat->reset)
			plat->reset();

		io_reg = regulator_get(&pdev->dev, plat->io_reg);
		if (IS_ERR(io_reg))
			io_reg = NULL;
		core_reg = regulator_get(&pdev->dev, plat->core_reg);
		if (!IS_ERR(core_reg)) {
			printk("LCD: %s regulator_set_voltage(io_reg, 1.8V)\n", __func__);
			regulator_set_voltage(io_reg, 1800000);
		} else {
			core_reg = NULL;
		}
	}

	for (i = 0; i < num_registered_fb; i++) {
		if (strcmp(registered_fb[i]->fix.id, "DISP3 BG") == 0) {
			printk("LCD: %s init_fb, show_logo, poweron\n", __func__);
			lcd_init_fb(registered_fb[i]);
			fb_show_logo(registered_fb[i], 0);
			lcd_poweron();
		} else if (strcmp(registered_fb[i]->fix.id, "DISP3 FG") == 0) {
			printk("LCD: %s init_fb\n", __func__);
			lcd_init_fb(registered_fb[i]);
		}
	}

	printk("LCD: %s register_client\n", __func__);
	fb_register_client(&nb);

	plcd_dev = pdev;

	return 0;
}

static int __devexit lcd_remove(struct platform_device *pdev)
{
	fb_unregister_client(&nb);
	lcd_poweroff();
	regulator_put(io_reg, &pdev->dev);
	if (core_reg)
		regulator_put(core_reg, &pdev->dev);

	return 0;
}

#ifdef CONFIG_PM
static int lcd_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int lcd_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define lcd_suspend NULL
#define lcd_resume NULL
#endif

/*!
 * platform driver structure
 */
static struct platform_driver lcd_driver = {
	.driver = {
		   .name = "lcd_wandisplay"},
	.probe = lcd_probe,
	.remove = __devexit_p(lcd_remove),
	.suspend = lcd_suspend,
	.resume = lcd_resume,
};

/*
 * Enable power
 *
 */
static void lcd_poweron(void)
{
	int err;
	if (lcd_on)
		return;
	printk("turning on LCD\n");
	if (core_reg)
		regulator_enable(core_reg);
	if (io_reg)
		regulator_enable(io_reg);
	lcd_on = 1;
}

/*
 * Disable power
 *
 */
static void lcd_poweroff(void)
{
	lcd_on = 0;
	printk("turning off LCD\n");
	regulator_disable(io_reg);
	if (core_reg)
		regulator_disable(core_reg);
}

static int __init wandisplay_lcd_init(void)
{
	return platform_driver_register(&lcd_driver);
}

static void __exit wandisplay_lcd_exit(void)
{
	platform_driver_unregister(&lcd_driver);
}

module_init(wandisplay_lcd_init);
module_exit(wandisplay_lcd_exit);

MODULE_AUTHOR("Jim Barlow");
MODULE_DESCRIPTION("Wandisplay LCD driver");
MODULE_LICENSE("GPL");
