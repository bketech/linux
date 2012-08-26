/*
 * Copyright 2005-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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

#define DEBUG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/delay.h>
#include <linux/usb/fsl_xcvr.h>
#include <linux/clk.h>

#include <mach/hardware.h>
#include <mach/arc_otg.h>
#include <asm/mach-types.h>

/* USB3315 register addresses */
#define USB3315_VID_LOW		0x00	/* Vendor ID low */
#define USB3315_VID_HIGH	0x01	/* Vendor ID high */
#define USB3315_PID_LOW		0x02	/* Product ID low */
#define USB3315_PID_HIGH	0x03	/* Product ID high */
#define USB3315_FUNC		0x04	/* Function Control */
#define USB3315_ITFCTL		0x07	/* Interface Control */
#define USB3315_OTGCTL		0x0A	/* OTG Control */

/* add to above register address to access Set/Clear functions */
#define USB3315_REG_SET		0x01
#define USB3315_REG_CLEAR	0x02

/* OTG Control Register bits */
#define USE_EXT_VBUS_IND	(1 << 7)	/* Use ext. Vbus indicator */
#define DRV_VBUS_EXT		(1 << 6)	/* Drive Vbus external */
#define DRV_VBUS		(1 << 5)	/* Drive Vbus */
#define CHRG_VBUS		(1 << 4)	/* Charge Vbus */
#define DISCHRG_VBUS		(1 << 3)	/* Discharge Vbus */
#define DM_PULL_DOWN		(1 << 2)	/* enable DM Pull Down */
#define DP_PULL_DOWN		(1 << 1)	/* enable DP Pull Down */
#define ID_PULL_UP		(1 << 0)	/* enable ID Pull Up */

/* OTG Function Control Register bits */
#define SUSPENDM		(1 << 6)	/* places the PHY into
						   low-power mode      */
#define DRV_RESET		(1 << 5)	/* Active HIGH transceiver
						   reset                  */

/*!
 * read ULPI register 'reg' thru VIEWPORT register 'view'
 *
 * @param       reg   register to read
 * @param       view  the ULPI VIEWPORT register address
 * @return	return usb3315 register value
 */
static u8 usb3315_read(int reg, volatile u32 *view)
{
	u32 data;

	/* make sure interface is running */
	if (!(__raw_readl(view) && ULPIVW_SS)) {
		__raw_writel(ULPIVW_WU, view);
		do {		/* wait for wakeup */
			data = __raw_readl(view);
		} while (data & ULPIVW_WU);
	}

	/* read the register */
	__raw_writel((ULPIVW_RUN | (reg << ULPIVW_ADDR_SHIFT)), view);

	do {			/* wait for completion */
		data = __raw_readl(view);
	} while (data & ULPIVW_RUN);

	return (u8) (data >> ULPIVW_RDATA_SHIFT) & ULPIVW_RDATA_MASK;
}

/*!
 * set bits into OTG USB3315 register 'reg' thru VIEWPORT register 'view'
 *
 * @param       bits  set value
 * @param	reg   which register
 * @param       view  the ULPI VIEWPORT register address
 */
static void usb3315_set(u8 bits, int reg, volatile u32 *view)
{
	u32 data;

	/* make sure interface is running */
	if (!(__raw_readl(view) && ULPIVW_SS)) {
		__raw_writel(ULPIVW_WU, view);
		do {		/* wait for wakeup */
			data = __raw_readl(view);
		} while (data & ULPIVW_WU);
	}

	__raw_writel((ULPIVW_RUN | ULPIVW_WRITE |
		      ((reg + USB3315_REG_SET) << ULPIVW_ADDR_SHIFT) |
		      ((bits & ULPIVW_WDATA_MASK) << ULPIVW_WDATA_SHIFT)),
		     view);

	while (__raw_readl(view) & ULPIVW_RUN)	/* wait for completion */
		continue;
}

/*!
 * clear bits in OTG USB3315 register 'reg' thru VIEWPORT register 'view'
 *
 * @param       bits  bits to clear
 * @param	reg   in this register
 * @param       view  the ULPI VIEWPORT register address
 */
static void usb3315_clear(u8 bits, int reg, volatile u32 *view)
{
	__raw_writel((ULPIVW_RUN | ULPIVW_WRITE |
		      ((reg + USB3315_REG_CLEAR) << ULPIVW_ADDR_SHIFT) |
		      ((bits & ULPIVW_WDATA_MASK) << ULPIVW_WDATA_SHIFT)),
		     view);

	while (__raw_readl(view) & ULPIVW_RUN)	/* wait for completion */
		continue;
}

/*!
 * set vbus power
 *
 * @param       view  viewport register
 * @param       on    power on or off
 */
static void usb3315_set_vbus_power(struct fsl_xcvr_ops *this,
				   struct fsl_usb2_platform_data *pdata, int on)
{
	u32 *view = pdata->regs + ULPIVW_OFF;

	pr_debug("real %s(on=%d) view=0x%p\n", __FUNCTION__, on, view);

	pr_debug("ULPI Vendor ID 0x%x    Product ID 0x%x\n",
		 (usb3315_read(USB3315_VID_HIGH, view) << 8) |
		 usb3315_read(USB3315_VID_LOW, view),
		 (usb3315_read(USB3315_PID_HIGH, view) << 8) |
		 usb3315_read(USB3315_PID_LOW, view));

	pr_debug("OTG Control before=0x%x\n",
		 usb3315_read(USB3315_OTGCTL, view));

	if (on) {
		usb3315_set(DRV_VBUS_EXT |	/* enable external Vbus */
				DRV_VBUS,	/* enable internal Vbus */
				USB3315_OTGCTL, view);
	} else {
		usb3315_clear(DRV_VBUS_EXT |	/* disable external Vbus */
				DRV_VBUS,	/* disable internal Vbus */
				USB3315_OTGCTL, view);
	}

	pr_debug("OTG Control after = 0x%x\n",
		 usb3315_read(USB3315_OTGCTL, view));
	pr_debug("IF control = 0x%x, Func control = 0x%x, Dbg = 0x%x\n",
			usb3315_read(USB3315_ITFCTL, view),
			usb3315_read(USB3315_FUNC, view),
			usb3315_read(0x15, view));
}

/*!
 * set remote wakeup
 *
 * @param       view  viewport register
 */
static void usb3315_set_remote_wakeup(u32 * view)
{
	__raw_writel(~ULPIVW_WRITE & __raw_readl(view), view);
	__raw_writel((1 << ULPIVW_PORT_SHIFT) | __raw_readl(view), view);
	__raw_writel(ULPIVW_RUN | __raw_readl(view), view);

	while (__raw_readl(view) & ULPIVW_RUN)	/* wait for completion */
		continue;
}

static void usb3315_init(struct fsl_xcvr_ops *this)
{
	pr_debug("%s:\n", __FUNCTION__);
}

static void usb3315_uninit(struct fsl_xcvr_ops *this)
{
	pr_debug("%s:\n", __FUNCTION__);
}

static void usb3315_suspend(struct fsl_xcvr_ops *this)
{
	/* If this is the last message that appears during a kernel boot hangs, most likely the
	 * ULPI interface is misconfigured (wrong IOMUX/GPIO settings).
	 */
	pr_debug("%s\n", __func__);

	/* send suspend command */
	usb3315_clear(SUSPENDM, USB3315_FUNC, &UH2_ULPIVIEW);
}

/*!
 * Set the 1504 transceiver to the proper mode for testing purposes.
 *
 * @param       view  the ULPI VIEWPORT register address
 * @param       test_mode Set the 1504 transceiver to disable bit stuffing and NRZI
 */
static void usb3315_set_test_mode(u32 *view, enum usb_test_mode test_mode)
{
	if (test_mode == USB_TEST_J || test_mode == USB_TEST_K) {
		printk(KERN_INFO "udc: disable bit stuffing and NRZI\n");
		/* Disable bit-stuffing and NRZI encoding. */
		usb3315_set(0x10, 0x04, view);
	}
}

static struct fsl_xcvr_ops usb3315_ops = {
	.name = "usb3315",
	.xcvr_type = PORTSC_PTS_ULPI,
	.init = usb3315_init,
	.uninit = usb3315_uninit,
	.suspend = usb3315_suspend,
	.set_vbus_power = usb3315_set_vbus_power,
	.set_remote_wakeup = usb3315_set_remote_wakeup,
	.set_test_mode = usb3315_set_test_mode,
};

extern void fsl_usb_xcvr_register(struct fsl_xcvr_ops *xcvr_ops);
extern int fsl_usb_xcvr_suspend(struct fsl_xcvr_ops *xcvr_ops);
extern bool gpio_fec_is_chip_present(void);

static int __init usb3315xc_init(void)
{
	pr_debug("%s\n", __FUNCTION__);

	if (gpio_fec_is_chip_present()) {
		pr_info("USB transceiver not initialized because FEC is present\n");
		return 0;
	}

	/* Configure CLKO to generate 24 MHz output clock required by transceiver */
	if (is_ivlboard()) {
		struct clk *ckol_clk;
		struct clk *ckih_clk;
		int err;

		ckih_clk = clk_get(NULL, "ckih");
		ckol_clk = clk_get(NULL, "cko1_clk");
		err = clk_set_parent(ckol_clk, ckih_clk);
		if (err) {
			pr_err("Failed to reconfigure CLKO for USB.  Error = %i\n", err);
		}
		err = clk_set_rate(ckol_clk, 24000000); /* 24 MHz needed for USB3315 */
		if (err) {
			pr_err("Failed to set rate CLKO for USB.  Error = %i\n", err);
		}
		clk_enable(ckol_clk);
		clk_put(ckol_clk);
		clk_put(ckih_clk);
	}

	fsl_usb_xcvr_register(&usb3315_ops);

	/* suspend usb3315 */
	if (fsl_usb_xcvr_suspend(&usb3315_ops))
		pr_warning("%s: failed to suspend usb3315\n", __func__);

	return 0;
}

extern void fsl_usb_xcvr_unregister(struct fsl_xcvr_ops *xcvr_ops);

static void __exit usb3315xc_exit(void)
{
	fsl_usb_xcvr_unregister(&usb3315_ops);
}

module_init(usb3315xc_init);
module_exit(usb3315xc_exit);

MODULE_AUTHOR("IVL Audio Inc.");
MODULE_DESCRIPTION("usb3315 xcvr driver");
MODULE_LICENSE("GPL");
