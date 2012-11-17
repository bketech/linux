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
 * @file mxcfb_hx8238a_qvga.c
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
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/mxcfb.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <mach/hardware.h>

#define CSPI_BASE_ADDRESS 0x43FA4000
#define CSPI_RXDATA		0x0000 // Receive data register
#define CSPI_TXDATA		0x0004 // Transmit data register
#define CSPI_CONREG		0x0008 // Control register
#define CSPI_INTREF		0x000C // Interrupt Control Register
#define CSPI_DMAREG		0x0010 // DMA Control Register
#define CSPI_STATREG	0x0014 // Status Register
#define CSPI_PERIODREG	0x0018 // Sample Period Control Register

extern void gpio_lcd_active(void);
extern void gpio_lcd_inactive(void);
extern void gpio_lcd_disable_for_emi(void);

static void lcd_poweron(void);
static void lcd_poweroff(void);

static struct platform_device *plcd_dev;
static int lcd_on;
static struct workqueue_struct *lcd_wq;

struct lcd_rotate_work_struct {
	struct delayed_work dwork;
	struct fb_info *info;
	struct clk *spi_clk;
};

static struct lcd_rotate_work_struct lcd_rotate_screen_work;

static struct fb_videomode video_modes[] = {
	{
	 .name = "HX8238A-1",
	 .refresh = 60,
	 .xres = 320,
	 .yres = 240,
	 .pixclock = 154000,
	 .left_margin = 68,
	 .right_margin = 20,
	 .upper_margin = 18,
	 .lower_margin = 4,
	 .hsync_len = 3,
	 .vsync_len = 3,
	 .sync = FB_SYNC_OE_LOW_ACT,
	 .vmode = FB_VMODE_NONINTERLACED,
	 .flag = 0, },
};

static void lcd_init_fb(struct fb_info *info)
{
	struct fb_var_screeninfo var;

	dev_dbg(info->device, "init\n");

	lcd_rotate_screen_work.info = info;

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

	dev_dbg(event->info->device, "event\n");
	if (strcmp(event->info->fix.id, "DISP3 BG")) {
		return 0;
	}

	switch (val) {
	case FB_EVENT_FB_REGISTERED:
		dev_dbg(event->info->device, "fb registered\n");
		lcd_init_fb(event->info);
		lcd_poweron();
		break;
	case FB_EVENT_BLANK:
		dev_dbg(event->info->device, "fb blank\n");
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

static int lcd_signals_disabled = 0;

static ssize_t lcd_signals_disable_show(struct device_driver *dev, char *buf)
{
	return snprintf(buf, 3, "%i\n", lcd_signals_disabled);
}

static ssize_t lcd_signals_disable_store(struct device_driver *dev, const char *buf, size_t count)
{
	int var;
	sscanf(buf, "%du", &var);
	if (var)
		var = 1;

	/* Take no action if nothing changed */
	if (var == lcd_signals_disabled)
		return count;

	lcd_signals_disabled = var;
	if (lcd_signals_disabled)
		gpio_lcd_disable_for_emi();	/* Don't use gpio_lcd_inactive() here -- see comment on this function */
	else
		gpio_lcd_active();

	return count;
}

static DRIVER_ATTR(lcd_signals_disable, S_IRUGO | S_IWUGO, lcd_signals_disable_show,
		lcd_signals_disable_store);

static void lcd_wait_for_spi(int us)
{
	long i;

	for(i=0;i<5000*us;i++) {
		if((__raw_readl(IO_ADDRESS(CSPI_BASE_ADDRESS + CSPI_CONREG)) & 4) == 0) 
			break;
	}
	/* printk("LCD: Waited for SPI: %ld\n",i); */
}

/*
 * The screen is physically upside down and we must send it a command to flip
 * it the right way.  Because ESD can cause the screen to lose its state,
 * we have an option to repeat the command every second which works by this
 * function rescheduling itself.  It runs in a private workqueue so that
 * delays will not hold up other functions.
 *
 * It is possible to tie this function to VSYNC, but probably not necessary.
 */
static void lcd_rotate_screen(struct work_struct *work)
{
	static int first=1;
#if 0
	/* This code would cause us to wait for a VSYNC, in case there are
	 * noticeable artifacts from setting the screen like this.  Not tested.
	 */
	lcd_rotate_screen_work.info->fbops->fb_ioctl(lcd_rotate_screen_work.info, MXCFB_WAIT_FOR_VSYNC, 0);
#endif

	/* Configure SPI:
	 *  17=24 bits
	 *  4=divide clock by 64
	 *  1=chip select 1
	 *  0=don't use SPI_RDY
	 *  7=active low, negate CS between bursts, phase 1, active low SCLK
	 *  3=start when XCH is set, don't set XCH, master mode, enable module
	 * The "negate CS between bursts" causes CS to be negated after every 24
	 * bit word.  That's contrary to the HX8283A specification, but when this
	 * bit is not set, only the first word is transmitted.  Not sure why.
	 */
	__raw_writel (0x01741073, IO_ADDRESS(CSPI_BASE_ADDRESS + CSPI_CONREG));

#if 0
	status = (u32)(__raw_readl(IO_ADDRESS(CSPI_BASE_ADDRESS + CSPI_STATREG)));
	//printk(" - SPI LCD - STATREG before writes 0x%x\n",rx_data);
#endif
	/* Send a request to set the contents of R01h on the LCD */
	__raw_writel (0x00700001, IO_ADDRESS(CSPI_BASE_ADDRESS + CSPI_TXDATA));

	/* Set the screen upside down.  See HX8238A datasheet.
	 * 7 = 0111 = start pattern
	 * 2 = 0010 = write data
	 * 3 = 0011 = !RL | REVerse white/black | Polarity INVert
	 * 1 = 0001 = !BGR | !TB | Charge Pump Enabled
	 * 00 = ignored
	 * It seems that the default is something like 7A7B.
	 */
#if 1
	__raw_writel (0x00723100, IO_ADDRESS(CSPI_BASE_ADDRESS + CSPI_TXDATA));
#else
	{
		/* For testing -- make the screen rotate 180 degrees each second */
		static int alternate = 0;
		if (alternate) {
			__raw_writel (0x007A7B00, IO_ADDRESS(CSPI_BASE_ADDRESS + CSPI_TXDATA));
			alternate = 0;
		} else {
			__raw_writel (0x00723100, IO_ADDRESS(CSPI_BASE_ADDRESS + CSPI_TXDATA));
			alternate = 1;
		}
	}
#endif

	/* Set XCH bit to start transfer */
	__raw_writel (0x01741077, IO_ADDRESS(CSPI_BASE_ADDRESS + CSPI_CONREG));
	if(first) {
		first = 0;
		lcd_wait_for_spi(20);  // Let command finish
		// Select register 04h, start transfer
		__raw_writel (0x00700004, IO_ADDRESS(CSPI_BASE_ADDRESS + CSPI_TXDATA));
		__raw_writel (0x01741077, IO_ADDRESS(CSPI_BASE_ADDRESS + CSPI_CONREG));
		// Let transfer finish (Again, the crude way - refined is to wait on bit 2)
		// Not sure why this is needed, but if this isn't here, we write to register 1 instead.
		lcd_wait_for_spi(10);
		/* Set the SEL bits to 0, with attendant reset values on OEA (01).  See HX8238A datasheet.
		 * 7 = 0111 = start pattern
		 * 2 = 00 10 = start pattern cont, write data
		 * 0 = Not used
		 * 4 = PALM=1, BLT[1:0] = 0
		 * 4 = OEA[1:0] = 01, SEL[2:1] = 00
		 * 0 = SEL[0] = 0, SWD[2:1] = 000
		 * This should be default if SEL and SWD pins are all 0.
		 */
		__raw_writel (0x00720440, IO_ADDRESS(CSPI_BASE_ADDRESS + CSPI_TXDATA));
		__raw_writel (0x01741077, IO_ADDRESS(CSPI_BASE_ADDRESS + CSPI_CONREG));
	}

	/* Because we are repeating the message, we don't need to wait for it.
	 * If you want to check, the appropriate method is to wait for XCH=0.
	 * IF YOU ARE USING THIS CODE ELSEWHERE, YOU SHOULD PROBABLY FOR THE
	 * TRANSFER TO COMPLETE!
	 */

	/* We could shut off the module here, but that would require waiting
	 * over 50 us for the transfer to complete.  For a wait that small, we
	 * have to spin-wait, if the system is playing a short audio frame, that
	 * could cause an underrun.  The right thing to do is probably use SPI
	 * interrupts, but that's a lot of extra work.
	 */

#ifdef CONFIG_FB_MXC_HX8238A_QVGA_PANEL_FORCE_SCREEN_FLIP
	/* Repeat every 1 second */
	queue_delayed_work(lcd_wq, &lcd_rotate_screen_work.dwork, msecs_to_jiffies(1000));
#endif
}

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
	int err;

	dev_dbg(&pdev->dev, "probe\n");

	err = driver_create_file(pdev->dev.driver, &driver_attr_lcd_signals_disable);
	if (err) {
		dev_err(&pdev->dev, "failed to create attribute %s\n",
				driver_attr_lcd_signals_disable.attr.name);
		return err;
	}

	lcd_wq = create_singlethread_workqueue("lcd_rotate_screen_wq");
	if (!lcd_wq) {
		dev_err(&pdev->dev, "failed to create workqueue\n");
		return err;
	}

	lcd_rotate_screen_work.spi_clk = clk_get(&pdev->dev, "cspi_clk");
	clk_enable(lcd_rotate_screen_work.spi_clk);
	INIT_DELAYED_WORK(&lcd_rotate_screen_work.dwork, lcd_rotate_screen);
	queue_delayed_work(lcd_wq, &lcd_rotate_screen_work.dwork, msecs_to_jiffies(1));

	for (i = 0; i < num_registered_fb; i++) {
		if (strcmp(registered_fb[i]->fix.id, "DISP3 BG") == 0) {
			dev_dbg(&pdev->dev, "LCD: init_fb, show_logo, poweron\n");
			lcd_init_fb(registered_fb[i]);
			fb_show_logo(registered_fb[i], 0);
			lcd_poweron();
		} else if (strcmp(registered_fb[i]->fix.id, "DISP3 FG") == 0) {
			dev_dbg(&pdev->dev, "LCD: init_fb\n");
			lcd_init_fb(registered_fb[i]);
		}
	}
	fb_register_client(&nb);

	plcd_dev = pdev;

	return 0;
}

static int __devexit lcd_remove(struct platform_device *pdev)
{
	fb_unregister_client(&nb);
	lcd_poweroff();

	cancel_delayed_work(&lcd_rotate_screen_work.dwork);
	flush_workqueue(lcd_wq);
	destroy_workqueue(lcd_wq);
	__raw_writel (0, IO_ADDRESS(CSPI_BASE_ADDRESS + CSPI_CONREG)); /* Disable SPI */
	clk_disable(lcd_rotate_screen_work.spi_clk);
	clk_put(lcd_rotate_screen_work.spi_clk);

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
		   .name = "lcd_hx8238a"},
	.probe = lcd_probe,
	.remove = __devexit_p(lcd_remove),
	.suspend = lcd_suspend,
	.resume = lcd_resume,
};

/*
 * Enable LCD regulators
 *
 */
static void lcd_poweron(void)
{
	if (lcd_on)
		return;
	lcd_on = 1;
}

/*
 * Disable LCD regulators
 *
 */
static void lcd_poweroff(void)
{
	lcd_on = 0;
}

static int __init hx8238a_lcd_init(void)
{
	return platform_driver_register(&lcd_driver);
}

static void __exit hx8238a_lcd_exit(void)
{
	platform_driver_unregister(&lcd_driver);
}

module_init(hx8238a_lcd_init);
module_exit(hx8238a_lcd_exit);

MODULE_AUTHOR("Jim Barlow");
MODULE_DESCRIPTION("HX8238A QVGA LCD display driver");
MODULE_LICENSE("GPL");
