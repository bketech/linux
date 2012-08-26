/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2009 IVL Audio Inc. All Rights Reserved
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/ata.h>
#include <linux/pmic_external.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>
#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/delay.h>
#include <asm/mach/flash.h>
#endif
#include <linux/leds.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <mach/common.h>
#include <mach/memory.h>
#include <mach/gpio.h>
#include <mach/mmc.h>

#include "board-mx35_ivlboard.h"
#include "crm_regs.h"
#include "iomux.h"

/*!
 * @file mach-mx35/mx35_ivlboard.c
 *
 * @brief This file contains the board specific initialization routines.
 *
 * @ingroup MSL_MX35
 */

unsigned int mx35_ivlboard_board_io;

int ivl_board_revision = -1;
int ivl_carrier_revision = -1;
EXPORT_SYMBOL(ivl_board_revision);
EXPORT_SYMBOL(ivl_carrier_revision);

static void mxc_nop_release(struct device *dev)
{
	/* Nothing */
}

/* MTD NAND flash */

#if defined(CONFIG_MTD_NAND_MXC) || defined(CONFIG_MTD_NAND_MXC_MODULE)	\
|| defined(CONFIG_MTD_NAND_MXC_V2) || defined(CONFIG_MTD_NAND_MXC_V2_MODULE)

#ifdef CONFIG_BOARD_BEAT_THANG
static struct mtd_partition mxc_nand_partitions[] = {
	{
	 .name = "nand.bootloader",
	 .offset = 0,
	 .size = 2 * 1024 * 1024},
	{
	 .name = "nand.kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 10 * 1024 * 1024},
	{
	 .name = "nand.rootfs",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 50 * 1024 * 1024},
	{
	 .name = "nand.backup",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 50 * 1024 * 1024},
	{
	 .name = "nand.firmware",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 50 * 1024 * 1024},
	{
	 .name = "nand.dropbox",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 112 * 1024 * 1024},
	{
	 .name = "nand.factorycontent",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 750 * 1024 * 1024},
	{
	 .name = "nand.writable",
	 .offset = MTDPART_OFS_APPEND,
	 .size = (2048 - (1 + 750 + 112 + 50 + 50 + 50 + 10 + 2)) * 1024 * 1024 /*MTDPART_SIZ_FULL*/},
   // We reserve the last 1 MB for BBT, etc.
	{
	 .name = "nand.ubi.ro",
	 .offset = (2 + 10 + 50) * 1024 * 1024,
	 .size = (50 + 750) * 1024 * 1024},
	{
	 .name = "nand.ubi.rw",
	 .offset = (2 + 10 + 50 + 50 + 750) * 1024 * 1024,
	 .size = (50 + 112 + 1023) * 1024 * 1024},
	{  // To avoid shifting the mtd partition numbers yet again
      // (and requiring changes to everything that references them),
      // create a small overlapping partition for just the redboot binary
	 .name = "nand.redboot",
	 .offset = 0,
	 .size = 1 * 512 * 1024},
};
#else
static struct mtd_partition mxc_nand_partitions[] = {
	{
	 .name = "nand.bootloader",
	 .offset = 0,
	 .size = 1024 * 1024},
	{
	 .name = "nand.kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 5 * 1024 * 1024},
	{
	 .name = "nand.rootfs",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 256 * 1024 * 1024},
	{
	 .name = "nand.configure",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 8 * 1024 * 1024},
	{
	 .name = "nand.userfs",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL},
};
#endif

static struct flash_platform_data mxc_nand_data = {
	.parts = mxc_nand_partitions,
	.nr_parts = ARRAY_SIZE(mxc_nand_partitions),
	.width = 1,
};

static struct platform_device mxc_nand_mtd_device = {
	.name = "mxc_nandv2_flash",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_nand_data,
		},
};

static void mxc_init_nand_mtd(void)
{
	if (__raw_readl(MXC_CCM_RCSR) & MXC_CCM_RCSR_NF16B)
		mxc_nand_data.width = 2;

	platform_device_register(&mxc_nand_mtd_device);
}
#else
static inline void mxc_init_nand_mtd(void)
{
}
#endif

static struct mxc_lcd_platform_data lcd_data = {
	.io_reg = "LCD"
};

static struct platform_device lcd_dev = {
	.name = "lcd_hx8238a",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = (void *)&lcd_data,
		},
};

static void mxc_init_lcd(void)
{
	platform_device_register(&lcd_dev);
}

#if defined(CONFIG_FB_MXC_SYNC_PANEL) || defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
/* mxc lcd driver */
static struct platform_device mxc_fb_device = {
	.name = "mxc_sdc_fb",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.coherent_dma_mask = 0xFFFFFFFF,
		},
};

static void mxc_init_fb(void)
{
	(void)platform_device_register(&mxc_fb_device);
}
#else
static inline void mxc_init_fb(void)
{
}
#endif

#if defined(CONFIG_BACKLIGHT_MXC)
static struct platform_device mxcbl_devices[] = {
#if defined(CONFIG_BACKLIGHT_MXC_IPU) || defined(CONFIG_BACKLIGHT_MXC_IPU_MODULE)
	{
	 .name = "mxc_ipu_bl",
	 .id = 0,
	 .dev = {
		 .platform_data = (void *)3,	/* DISP # for this backlight */
		 },
	 }
#endif
};

static inline void mxc_init_bl(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(mxcbl_devices); i++) {
		platform_device_register(&mxcbl_devices[i]);
	}
}
#else
static inline void mxc_init_bl(void)
{
}
#endif

#if defined(CONFIG_SDIO_UNIFI_FS) || defined(CONFIG_SDIO_UNIFI_FS_MODULE)
#error See mx35_3stack.c for an example of how to set up SDIO_UNIFI
#else
struct mxc_unifi_platform_data *get_unifi_plat_data(void)
{
	return NULL;
}
#endif

EXPORT_SYMBOL(get_unifi_plat_data);


#if defined(CONFIG_BOARD_BEAT_THANG)
static struct i2c_board_info mxc_i2c_board_info[] __initdata = {
	{
	 .type = "tpa6130-headphone1",
	 .addr = 0x60,
	 },
	{
	 .type = "tlc59116-led1",
	 .addr = 0x64,
	 },
	{
	 .type = "tlc59116-led2",
	 .addr = 0x65,
	 },
	{
	 .type = "tlc59116-led3",
	 .addr = 0x66,
	 },
	{
	 .type = "tlc59116-led4",
	 .addr = 0x67,
	 },
	{
	 .type = "tlc59116-led5",
	 .addr = 0x6f,
	 },
	{
	 .type = "mcf51qe", /* coldfire */
	 .addr = 0x0c,
	 },
#if defined(CONFIG_I2C_SLAVE_CLIENT)
	{
	 .type = "i2c-slave-client",
	 .addr = 0x55,
	 },
#endif
};

static struct i2c_board_info mxc_i2c_board_info_bus2[] __initdata = {
	{
	 .type = "tpa6130-headphone2",
	 .addr = 0x60,
	 },
#if defined(CONFIG_I2C_SLAVE_CLIENT)
	{
	 .type = "i2c-slave-client",
	 .addr = 0x55,
	 },
#endif
};

#else

static struct i2c_board_info mxc_i2c_board_info[] __initdata = {
	{
	 .type = "mc9sdz60",
	 .addr = 0x69,
	 },
	{
	.type = "sgtl5000-i2c",
	.addr = 0x0a,
	 },
#if defined(CONFIG_I2C_SLAVE_CLIENT)
	{
	 .type = "i2c-slave-client",
	 .addr = 0x55,
	 },
#endif
	{
	 .type = "mc13892",
	 .addr = 0x08,
	 .platform_data = (void *)MX35_PIN_GPIO2_0,
	 },
};

#endif

#if defined(CONFIG_FEC) || defined(CONFIG_FEC_MODULE)
extern void gpio_fec_active(void);
extern void gpio_fec_inactive(void);
static int fec_enable(void);
static int fec_disable(void);
static struct resource mxc_fec_resources[] = {
	{
		.start	= MXC_FEC_BASE_ADDR,
		.end	= MXC_FEC_BASE_ADDR + 0xfff,
		.flags	= IORESOURCE_MEM
	}, {
		.start	= MXC_INT_FEC,
		.end	= MXC_INT_FEC,
		.flags	= IORESOURCE_IRQ
	},
};

static struct fec_platform_data mxc_fec_data = {
	.init = fec_enable,
	.uninit = fec_disable,
};

struct platform_device mxc_fec_device = {
	.name = "fec",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_fec_data,
	},
	.num_resources = ARRAY_SIZE(mxc_fec_resources),
	.resource = mxc_fec_resources,
};

static int fec_enable(void)
{
	gpio_fec_active();
	return 0;
}

static int fec_disable(void)
{
	gpio_fec_inactive();
	return 0;
}

static __init int mxc_init_fec(void)
{
	return platform_device_register(&mxc_fec_device);
}
#else
static inline int mxc_init_fec(void)
{
	return 0;
}
#endif

#if defined(CONFIG_MMC_IMX_ESDHCI) || defined(CONFIG_MMC_IMX_ESDHCI_MODULE)
static struct mxc_mmc_platform_data mmc1_data = {
	.ocr_mask = MMC_VDD_32_33,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 150000,
	.max_clk = 52000000,
	.card_inserted_state = 0,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "sdhc_clk",
};

/*!
 * Resource definition for the SDHC1
 */
static struct resource mxcsdhc1_resources[] = {
	[0] = {
	       .start = MMC_SDHC1_BASE_ADDR,
	       .end = MMC_SDHC1_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MX35_INT_MMC_SDHC1,
	       .end = MX35_INT_MMC_SDHC1,
	       .flags = IORESOURCE_IRQ,
	       },
	[2] = {
	       .flags = IORESOURCE_IRQ,
	       },
};

/*! Device Definition for MXC SDHC1 */
static struct platform_device mxcsdhc1_device = {
	.name = "mxsdhci",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mmc1_data,
		},
	.num_resources = ARRAY_SIZE(mxcsdhc1_resources),
	.resource = mxcsdhc1_resources,
};

static struct mxc_mmc_platform_data mmc2_data = {
	.ocr_mask = MMC_VDD_32_33,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 150000,
	.max_clk = 50000000,
	.card_inserted_state = 0,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "sdhc_clk",
};

static struct resource mxcsdhc2_resources[] = {
	[0] = {
	       .start = MMC_SDHC2_BASE_ADDR,
	       .end = MMC_SDHC2_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_MMC_SDHC2,
	       .end = MXC_INT_MMC_SDHC2,
	       .flags = IORESOURCE_IRQ,
	       },
	[2] = {
	       .flags = IORESOURCE_IRQ,
	       },
};

static struct platform_device mxcsdhc2_device = {
	.name = "mxsdhci",
	.id = 1,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mmc2_data,
		},
	.num_resources = ARRAY_SIZE(mxcsdhc2_resources),
	.resource = mxcsdhc2_resources,
};
#endif

#if defined(CONFIG_MMC_IMX_ESDHCI) || defined(CONFIG_MMC_IMX_ESDHCI_MODULE)
static inline void mxc_init_mmc(void)
{
	mxcsdhc1_resources[2].start = mxcsdhc1_resources[2].end = mx35_ivlboard_get_gpio_irq(IVLBOARD_IRQ_SDCARD1_DET);
	(void)platform_device_register(&mxcsdhc1_device);

	mxcsdhc2_resources[2].start = mxcsdhc2_resources[2].end = mx35_ivlboard_get_gpio_irq(IVLBOARD_IRQ_SDCARD2_DET);
	(void)platform_device_register(&mxcsdhc2_device);
}
#else
static inline void mxc_init_mmc(void)
{
}
#endif

#define IVL_PIN_CPU_HEARTBEAT	MX35_PIN_MLB_CLK

static struct gpio_led led_info[1] = {
		[0] = {
				.name = "cpu-heartbeat",
				.default_trigger = "heartbeat",
				.gpio = IOMUX_TO_GPIO(IVL_PIN_CPU_HEARTBEAT),
				.active_low = 1,
				.default_state = LEDS_GPIO_DEFSTATE_OFF,
		},
};

static struct gpio_led_platform_data led_data = {
		.num_leds = 1,
		.leds = led_info,
};

static struct platform_device cpu_heartbeat_led_device = {
		.name = "leds-gpio",
		.id = 0,
		.dev = {
				.release = mxc_nop_release,
				.platform_data = &led_data,
		},
};

static void mxc_init_cpu_heartbeat(void)
{
	mxc_request_iomux(IVL_PIN_CPU_HEARTBEAT, MUX_CONFIG_GPIO);
	mxc_iomux_set_pad(IVL_PIN_CPU_HEARTBEAT, PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW);

	/* FSL didn't properly integrated their IOMUX block into their GPIO code.
	 * Calling request_iomux() implicitly calls gpio_request() on the pin.  The LED
	 * driver code will call gpio_request again on the pin, find that the pin is
	 * reserved, and fail.  Freeing it here leaves the IOMUX code in the correct
	 * state and reconfigures the gpiolib code, freeing the pin so it can be taken
	 * again (sigh).
	 */
	gpio_free(led_info[0].gpio);

	(void)platform_device_register(&cpu_heartbeat_led_device);
}


#ifdef CONFIG_MXC_PSEUDO_IRQS
/*! Device Definition for MXC SDHC1 */
static struct platform_device mxc_pseudo_irq_device = {
	.name = "mxc_pseudo_irq",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		},
};

static inline int mxc_init_pseudo_irq(void)
{
	return platform_device_register(&mxc_pseudo_irq_device);
}

late_initcall(mxc_init_pseudo_irq);

/*!
 * Power Key interrupt handler.
 */
static irqreturn_t power_key_int(int irq, void *dev_id)
{
	pr_info(KERN_INFO "on-off key pressed\n");
	return 0;
}

/*!
 * Power Key initialization.
 */
static int __init mxc_init_power_key(void)
{
	if (!board_is_mx35(BOARD_REV_2)) {
		/*Set power key as wakeup resource */
		int irq, ret;
		irq = MXC_PSEUDO_IRQ_POWER_KEY;
		set_irq_type(irq, IRQF_TRIGGER_RISING);
		ret = request_irq(irq, power_key_int, 0, "power_key", 0);
		if (ret)
			pr_info("register on-off key interrupt failed\n");
		else
			enable_irq_wake(irq);
		return ret;
	}
	return 0;
}

late_initcall(mxc_init_power_key);
#endif

/*!
 * Board specific fixup function. It is called by \b setup_arch() in
 * setup.c file very early on during kernel starts. It allows the user to
 * statically fill in the proper values for the passed-in parameters. None of
 * the parameters is used currently.
 *
 * @param  desc         pointer to \b struct \b machine_desc
 * @param  tags         pointer to \b struct \b tag
 * @param  cmdline      pointer to the command line
 * @param  mi           pointer to \b struct \b meminfo
 */
static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	mxc_cpu_init();

#ifdef CONFIG_DISCONTIGMEM
	do {
		int nid;
		mi->nr_banks = MXC_NUMNODES;
		for (nid = 0; nid < mi->nr_banks; nid++)
			SET_NODE(mi, nid);
	} while (0);
#endif
}

extern void gpio_shutdown_request_active(void);
extern void gpio_shutdown_request_access(bool enable);

static void ivlboard_power_off(void)
{
	gpio_shutdown_request_access(true);
}

#if defined(CONFIG_SND_SOC_IMX_IVLBOARD_BEAT_THANG) \
    || defined(CONFIG_SND_SOC_IMX_IVLBOARD_BEAT_THANG_MODULE)

#if 0
unsigned int headphone_det_status(void)
{
	int ret = 0;
	if (0 != pmic_gpio_get_designation_bit_val(0, &ret))
		printk(KERN_ERR "Get headphone status error.");
	return ret;
}

static int mxc_btaudio_plat_init(void);
static int mxc_btaudio_plat_finit(void);
static int mxc_btaudio_amp_enable(int enable);
#endif

static struct mxc_btaudio_platform_data btaudio_data;

static struct platform_device btaudio_device = {
	.name = "ivlboard-btaudio",
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &btaudio_data,
		} ,
};

#if 0
static int mxc_btaudio_plat_init(void)
{
	struct regulator *reg;
	reg = regulator_get(&btaudio_device.dev, "SPKR");
	if (IS_ERR(reg))
		return -EINVAL;
	btaudio_data.priv = reg;
	return 0;
}

static int mxc_btaudio_plat_finit(void)
{
	struct regulator *reg;
	reg = btaudio_data.priv;
	if (reg) {
		regulator_put(reg, &btaudio_device.dev);
		btaudio_data.priv = NULL;
	}
	return 0;
}

static int mxc_btaudio_amp_enable(int enable)
{
	struct regulator *reg;
	reg = btaudio_data.priv;

	if (!reg)
		return -EINVAL;
	if (enable)
		regulator_enable(reg);
	else
		regulator_disable(reg);
	return 0;
}
#endif

static void mxc_btaudio_init(void)
{
	struct clk *ckie;

	ckie = clk_get(NULL, "ckie");
	if (IS_ERR(ckie))
		return;
	clk_enable(ckie);
	clk_put(ckie);

	btaudio_data.mic_in_irq = mx35_ivlboard_get_gpio_irq(IVLBOARD_IRQ_MIC_IN_DET);
	btaudio_data.line_in_irq = mx35_ivlboard_get_gpio_irq(IVLBOARD_IRQ_LINE_IN_DET);
	btaudio_data.hp_irq[0] = mx35_ivlboard_get_gpio_irq(IVLBOARD_IRQ_HP1_DET);
	btaudio_data.hp_irq[1] = mx35_ivlboard_get_gpio_irq(IVLBOARD_IRQ_HP2_DET);
	platform_device_register(&btaudio_device);
}
#else
static void mxc_btaudio_init(void)
{
}
#endif


/*!
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
	mxc_cpu_common_init();

	early_console_setup(saved_command_line);
	mxc_register_gpios();
	mxc_init_devices();
	mx35_ivlboard_gpio_init();
	mxc_init_nand_mtd();

	mxc_init_cpu_heartbeat();

	mxc_init_lcd();
	mxc_init_fb();
	mxc_init_bl();

	i2c_register_board_info(0, mxc_i2c_board_info,
				ARRAY_SIZE(mxc_i2c_board_info));
#if defined(CONFIG_BOARD_BEAT_THANG)
	i2c_register_board_info(1, mxc_i2c_board_info_bus2,
				ARRAY_SIZE(mxc_i2c_board_info_bus2));
#endif

	mxc_btaudio_init();
	mxc_init_mmc();
	mxc_init_fec();

	gpio_shutdown_request_active();
	pm_power_off = ivlboard_power_off;
}

/*!
 * Assert a specific GPIO without relying on GPIO code.  Useful for debugging kernel start failures.
 * It's a one bit JTAG in that sense you get one bit of debugging information per attempted boot.  Currently
 * used as a signal to notify whenever "late_initcall" is processed.
 */
static int __init one_bit_jtag(void)
{
	unsigned long reg;
	reg = __raw_readl(IO_ADDRESS(GPIO1_BASE_ADDR + 0x4));
	reg |= 0x2;
	__raw_writel(reg, IO_ADDRESS(GPIO1_BASE_ADDR + 0x4));
	reg = __raw_readl(IO_ADDRESS(GPIO1_BASE_ADDR));
	reg |= 0x2;
	__raw_writel(reg, IO_ADDRESS(GPIO1_BASE_ADDR));
	return 0;
}
late_initcall(one_bit_jtag);

#define PLL_PCTL_REG(brmo, pd, mfd, mfi, mfn)		\
		(((brmo) << 31) + (((pd) - 1) << 26) + (((mfd) - 1) << 16) + \
		((mfi)  << 10) + mfn)

/* For 24MHz input clock */
#define PLL_665MHZ		PLL_PCTL_REG(1, 1, 48, 13, 41)
#define PLL_532MHZ		PLL_PCTL_REG(1, 1, 12, 11, 1)
#define PLL_399MHZ		PLL_PCTL_REG(0, 1, 16, 8, 5)

/* working point(wp): 0,1 - 133MHz; 2,3 - 266MHz; 4,5 - 399MHz;*/
/* auto input clock table */
static struct cpu_wp cpu_wp_auto[] = {
	{
	 .pll_reg = PLL_399MHZ,
	 .pll_rate = 399000000,
	 .cpu_rate = 133000000,
	 .pdr0_reg = (0x2 << MXC_CCM_PDR0_AUTO_MUX_DIV_OFFSET),},
	{
	 .pll_reg = PLL_399MHZ,
	 .pll_rate = 399000000,
	 .cpu_rate = 133000000,
	 .pdr0_reg = (0x6 << MXC_CCM_PDR0_AUTO_MUX_DIV_OFFSET),},
	{
	 .pll_reg = PLL_399MHZ,
	 .pll_rate = 399000000,
	 .cpu_rate = 266000000,
	 .pdr0_reg = (0x1 << MXC_CCM_PDR0_AUTO_MUX_DIV_OFFSET),},
	{
	 .pll_reg = PLL_399MHZ,
	 .pll_rate = 399000000,
	 .cpu_rate = 266000000,
	 .pdr0_reg = (0x5 << MXC_CCM_PDR0_AUTO_MUX_DIV_OFFSET),},
	{
	 .pll_reg = PLL_399MHZ,
	 .pll_rate = 399000000,
	 .cpu_rate = 399000000,
	 .pdr0_reg = (0x0 << MXC_CCM_PDR0_AUTO_MUX_DIV_OFFSET),},
	{
	 .pll_reg = PLL_399MHZ,
	 .pll_rate = 399000000,
	 .cpu_rate = 399000000,
	 .pdr0_reg = (0x6 << MXC_CCM_PDR0_AUTO_MUX_DIV_OFFSET),},
};

/* consumer input clock table */
static struct cpu_wp cpu_wp_con[] = {
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 133000000,
	 .pdr0_reg = (0x6 << MXC_CCM_PDR0_CON_MUX_DIV_OFFSET),},
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 133000000,
	 .pdr0_reg = (0xE << MXC_CCM_PDR0_CON_MUX_DIV_OFFSET),},
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 266000000,
	 .pdr0_reg = (0x2 << MXC_CCM_PDR0_CON_MUX_DIV_OFFSET),},
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 266000000,
	 .pdr0_reg = (0xA << MXC_CCM_PDR0_CON_MUX_DIV_OFFSET),},
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 399000000,
	 .pdr0_reg = (0x1 << MXC_CCM_PDR0_CON_MUX_DIV_OFFSET),},
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 399000000,
	 .pdr0_reg = (0x9 << MXC_CCM_PDR0_CON_MUX_DIV_OFFSET),},
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 532000000,
	 .pdr0_reg = (0x0 << MXC_CCM_PDR0_CON_MUX_DIV_OFFSET),},
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 532000000,
	 .pdr0_reg = (0x8 << MXC_CCM_PDR0_CON_MUX_DIV_OFFSET),},
	{
	 .pll_reg = PLL_665MHZ,
	 .pll_rate = 665000000,
	 .cpu_rate = 665000000,
	 .pdr0_reg = (0x7 << MXC_CCM_PDR0_CON_MUX_DIV_OFFSET),},
};

struct cpu_wp *get_cpu_wp(int *wp)
{
	if (cpu_is_mx35_rev(CHIP_REV_2_0) >= 1) {
		*wp = 9;
		return cpu_wp_con;
	} else {
		if (__raw_readl(MXC_CCM_PDR0) & MXC_CCM_PDR0_AUTO_CON) {
			*wp = 9;
			return cpu_wp_con;
		} else {
			*wp = 6;
			return cpu_wp_auto;
		}
	}
}

static void __init mx35_ivlboard_timer_init(void)
{
	mx35_clocks_init();
}

static struct sys_timer mxc_timer = {
       .init = mx35_ivlboard_timer_init,
};

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX35_3DS data structure.
 */
/* *INDENT-OFF* */
MACHINE_START(MX35_3DS, "IVL Audio Inc MX35 Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.phys_io = AIPS1_BASE_ADDR,
	.io_pg_offst = ((AIPS1_BASE_ADDR_VIRT) >> 18) & 0xfffc,
	.boot_params = PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx35_map_io,
	.init_irq = mxc_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mxc_timer,
MACHINE_END
