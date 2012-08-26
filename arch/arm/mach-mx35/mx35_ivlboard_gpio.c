/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2009 IVL Audio Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/stringify.h>
#include <mach/hardware.h>
#include <mach/gpio.h>

//#include "board-mx35_3stack.h"
#include "board-mx35_ivlboard.h"
#include "iomux.h"

#define IVL_PIN_UART3_RX			MX35_PIN_ATA_DATA10
#define IVL_PIN_UART3_TX			MX35_PIN_ATA_DATA11

#define IVL_PIN_SD1_WP				MX35_PIN_RTS1
#define IVL_PIN_SD1_DETECT			MX35_PIN_CAPTURE

#define IVL_PIN_SD2_CLK				MX35_PIN_SD2_CLK
#define IVL_PIN_SD2_CMD				MX35_PIN_SD2_CMD
#define IVL_PIN_SD2_DATA0			MX35_PIN_SD2_DATA0
#define IVL_PIN_SD2_DATA1			MX35_PIN_SD2_DATA1
#define IVL_PIN_SD2_DATA2			MX35_PIN_SD2_DATA2
#define IVL_PIN_SD2_DATA3			MX35_PIN_SD2_DATA3

#define IVL_PIN_SD2_WP				MX35_PIN_CTS1
#define IVL_PIN_SD2_DETECT			MX35_PIN_COMPARE

#define IVL_PIN_FEC_RESET			(ivl_board_revision >= IVL_PCB0423_A ? MX35_PIN_MLB_DAT : MX35_PIN_ATA_DATA6)
#define IVL_PIN_FEC_USB_MUX			MX35_PIN_CTS2		/* FEC#/USB mux on IVL_PCB0417_A only */

#define IVL_PIN_BOARD_REV0			MX35_PIN_RTS2
#define IVL_PIN_BOARD_REV1			MX35_PIN_SCK4
#define IVL_PIN_BOARD_REV2			MX35_PIN_STXFS4

#define IVL_PIN_BIST_MODE			MX35_PIN_CTS2

#define IVL_PIN_ADC_PDN				MX35_PIN_ATA_DATA4	/* ADC power down. Only used in Rev A; removed in B. */
#define IVL_PIN_MIC_PP_EN			MX35_PIN_ATA_DIOW	/* Phantom power enable */
#define IVL_PIN_HP_EN1				MX35_PIN_ATA_DATA0	/* Headphone amp 1 enable */
#define IVL_PIN_HP_EN2				MX35_PIN_ATA_DATA1	/* Headphone amp 2 enable */
#define IVL_PIN_AUDIO_IN_PWR_EN		MX35_PIN_STXD4		/* Enable all mic/line input power */

#define IVL_PIN_VDD_AN_EN_OLD		MX35_PIN_ATA_CS1
#define IVL_PIN_VDD_AN_EN			MX35_PIN_ATA_DATA3	/* Enable audio power supplies */

#define IVL_PIN_HP_DET1				MX35_PIN_ATA_DATA8	/* Headphone 1 detect */
#define IVL_PIN_HP_DET2				MX35_PIN_ATA_DATA9	/* Headphone 2 detect */

#define IVL_PIN_LINE_IN_DETECT		MX35_PIN_ATA_DATA7	/* Line in detect on carrier rev C; test point on B */
#define IVL_PIN_MIC_IN_DETECT_OLD	MX35_PIN_ATA_BUFF_EN	/* For rev B carrier boards; no longer supported */
#define IVL_PIN_MIC_IN_DETECT		MX35_PIN_ATA_DATA14

#define IVL_PIN_USBH2_OC			MX35_PIN_FEC_RDATA1	/* USB host 2 overcurrent signal */

#define IVL_PIN_AUDIO_ATT			MX35_PIN_SRXD4		/* L: +4 dBu, H: -10 dBV */
#define IVL_PIN_MUTE_LOW			MX35_PIN_ATA_DATA5	/* Active low mute */

#define IVL_PIN_SHDN_RQ				MX35_PIN_VSTBY		/* Shutdown request */

#define IVL_PIN_USB_XCVR_RST		MX35_PIN_MLB_SIG	/* USB transceiver reset, PCB0423 rev A and higher */

/*!
 * @file mach-mx35/mx35_ivlboard_gpio.c
 *
 * @brief This file contains all the GPIO setup functions for the board.
 *
 * @ingroup GPIO_MX35
 */

extern int ivl_board_revision;
extern int ivl_carrier_revision;

static bool fec_is_present;

#include "iomux_input_table.h"

/*!
 * Check CPU board revision, accounting for a Rev A bug that causes the Rev1
 * pin to be read incorrectly.
 */
static void check_cpu_board_revision(void)
{
	int val, revnum;

	/* config CS5 */
	mxc_request_iomux(MX35_PIN_CS5, MUX_CONFIG_FUNC);

	/* PCB0417 has a 10k PU on GPIO1.1; PCB0423 does not.  Use a 100k PD
	 * to test for the PU. */
	mxc_request_iomux(MX35_PIN_GPIO1_1, MUX_CONFIG_FUNC);
	mxc_iomux_set_pad(MX35_PIN_GPIO1_1, 0xC0); /* 100k PD */
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_GPIO1_1), __stringify(MX35_PIN_GPIO1_1));
	udelay(10);
	val = gpio_get_value(IOMUX_TO_GPIO(MX35_PIN_GPIO1_1));
	gpio_free(IOMUX_TO_GPIO(MX35_PIN_GPIO1_1));

	if (val)
		ivl_board_revision = IVL_PCB0417_A;
	else
		ivl_board_revision = IVL_PCB_UNKNOWN;

	if (ivl_board_revision == IVL_PCB0417_A)
		return;

	/* Board revision settings */
	mxc_request_iomux(IVL_PIN_BOARD_REV0, MUX_CONFIG_GPIO);
	mxc_iomux_set_pad(IVL_PIN_BOARD_REV0, PAD_CTL_HYS_SCHMITZ);
	set_iomux_input_gpio(IVL_PIN_BOARD_REV0);
	gpio_request(IOMUX_TO_GPIO(IVL_PIN_BOARD_REV0), __stringify(IVL_PIN_BOARD_REV0));
	gpio_direction_input(IOMUX_TO_GPIO(IVL_PIN_BOARD_REV0));
	udelay(10);
	val = gpio_get_value(IOMUX_TO_GPIO(IVL_PIN_BOARD_REV0));
	revnum = val;

	mxc_request_iomux(IVL_PIN_BOARD_REV1, MUX_CONFIG_GPIO);
	mxc_iomux_set_pad(IVL_PIN_BOARD_REV1, PAD_CTL_HYS_SCHMITZ);
	set_iomux_input_gpio(IVL_PIN_BOARD_REV1);
	gpio_request(IOMUX_TO_GPIO(IVL_PIN_BOARD_REV1), __stringify(IVL_PIN_BOARD_REV1));
	gpio_direction_input(IOMUX_TO_GPIO(IVL_PIN_BOARD_REV1));
	udelay(10);
	val = gpio_get_value(IOMUX_TO_GPIO(IVL_PIN_BOARD_REV1));
	revnum |= val << 1;

	mxc_request_iomux(IVL_PIN_BOARD_REV2, MUX_CONFIG_GPIO);
	mxc_iomux_set_pad(IVL_PIN_BOARD_REV2, PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD
			| PAD_CTL_100K_PD);
	set_iomux_input_gpio(IVL_PIN_BOARD_REV2);
	gpio_request(IOMUX_TO_GPIO(IVL_PIN_BOARD_REV2), __stringify(IVL_PIN_BOARD_REV2));
	gpio_direction_input(IOMUX_TO_GPIO(IVL_PIN_BOARD_REV2));
	udelay(10);
	val = gpio_get_value(IOMUX_TO_GPIO(IVL_PIN_BOARD_REV2));
	revnum |= val << 2;

	switch (revnum) {
	case 0:
		ivl_board_revision = IVL_PCB0423_A;
		break;
	default:
		ivl_board_revision = IVL_PCB_UNKNOWN;
		break;
	}
}

/*!
 * Check carrier board revision based on differences in the LINE_IN_DETECT pin.
 * On tests for B or C.
 */
static void check_carrier_board_revision(void)
{
	int val;

	switch (ivl_board_revision) {
	case IVL_PCB0417_A:
		/* Find out if the carrier board revision is B or C using the LINE_IN_DETECT
		 * pin.  */
		mxc_request_iomux(IVL_PIN_LINE_IN_DETECT, MUX_CONFIG_GPIO);
		mxc_iomux_set_pad(IVL_PIN_LINE_IN_DETECT, PAD_CTL_PKE_ENABLE
				| PAD_CTL_PUE_PUD | PAD_CTL_100K_PU);
		set_iomux_input_gpio(IVL_PIN_LINE_IN_DETECT);
		gpio_request(IOMUX_TO_GPIO(IVL_PIN_LINE_IN_DETECT), __stringify(IVL_PIN_LINE_IN_DETECT));
		gpio_direction_input(IOMUX_TO_GPIO(IVL_PIN_LINE_IN_DETECT));
		udelay(10);
		val = gpio_get_value(IOMUX_TO_GPIO(IVL_PIN_LINE_IN_DETECT));
		if (val == 0) {
			/* Rev C with 10k PU, 100k PU from pad, and a closed switch to GND. */
			ivl_carrier_revision = 'C';
		} else {
			/* Can't tell if it is rev B or C yet. */
			mxc_iomux_set_pad(IVL_PIN_LINE_IN_DETECT, PAD_CTL_PKE_ENABLE
					| PAD_CTL_PUE_PUD | PAD_CTL_100K_PD);
			udelay(10);
			val = gpio_get_value(IOMUX_TO_GPIO(IVL_PIN_LINE_IN_DETECT));

			/* If val = 1, a 100k PD does not change the value, so it has an
			 * external pull-up, i.e. it must be Rev C. */
			if (val == 0)
				ivl_carrier_revision = 'B';
			else
				ivl_carrier_revision = 'C';
		}

		/* Reset pin */
		mxc_free_iomux(IVL_PIN_LINE_IN_DETECT, MUX_CONFIG_GPIO);
		mxc_iomux_set_pad(IVL_PIN_LINE_IN_DETECT, PAD_CTL_HYS_SCHMITZ);
		break;

	case IVL_PCB0423_A:
		/* Only D is available for PCB0423_A */
		ivl_carrier_revision = 'D';
		break;

	default:
		ivl_carrier_revision = '?';
		break;
	}
}

/*!
 * Test to see if FEC (debug Ethernet hardware) is present.
 */
static void check_fec_pin(void)
{
	/* Find out if the FEC hardware is present */
	mxc_request_iomux(IVL_PIN_FEC_RESET, MUX_CONFIG_GPIO);
	mxc_iomux_set_pad(IVL_PIN_FEC_RESET, PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD
			| PAD_CTL_100K_PD); /* 100k pulldown to stabilize */
	set_iomux_input_gpio(IVL_PIN_FEC_RESET);
	gpio_request(IOMUX_TO_GPIO(IVL_PIN_FEC_RESET), __stringify(IVL_PIN_FEC_RESET));
	gpio_direction_input(IOMUX_TO_GPIO(IVL_PIN_FEC_RESET));

	fec_is_present = gpio_get_value(IOMUX_TO_GPIO(IVL_PIN_FEC_RESET));

	/* Configure FEC_RESET as output */
	mxc_iomux_set_pad(IVL_PIN_FEC_RESET, PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW);
	gpio_direction_output(IOMUX_TO_GPIO(IVL_PIN_FEC_RESET), 0);
	gpio_set_value(IOMUX_TO_GPIO(IVL_PIN_FEC_RESET), 0);	/* Hold reset low for 100 us */

	/* Configure FEC/USB as output */
	if (ivl_board_revision == IVL_PCB0417_A) {
		mxc_request_iomux(IVL_PIN_FEC_USB_MUX, MUX_CONFIG_GPIO);
		mxc_iomux_set_pad(IVL_PIN_FEC_USB_MUX, PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW);
		gpio_request(IOMUX_TO_GPIO(IVL_PIN_FEC_USB_MUX), __stringify(IVL_PIN_FEC_USB_MUX));
		gpio_direction_output(IOMUX_TO_GPIO(IVL_PIN_FEC_USB_MUX), 0);
		gpio_set_value(IOMUX_TO_GPIO(IVL_PIN_FEC_USB_MUX), 1);
	}
}

/*!
 * This system-wise GPIO function initializes the pins during system startup.
 * All the statically linked device drivers should put the proper GPIO
 * initialization code inside this function. It is called by \b fixup_mx31ads()
 * during system startup. This function is board specific.
 */
void mx35_ivlboard_gpio_init(void)
{
	const char *str_board_rev;

	/* config CS5 */
	mxc_request_iomux(MX35_PIN_CS5, MUX_CONFIG_FUNC);

	/* limit strength of CLKO pin for EMI control */
	mxc_iomux_set_pad(MX35_PIN_CLKO, PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW);

	check_cpu_board_revision();
	check_carrier_board_revision();

	switch (ivl_board_revision) {
	case IVL_PCB0417_A:
		str_board_rev = "PCB0417 rev A";
		break;
	case IVL_PCB0423_A:
		str_board_rev = "PCB0423 rev A01";
		break;
	default:
		str_board_rev = "UNKNOWN";
		break;
	}

	printk("IVL MX35 CPU board %s, carrier board rev %c\n",
			str_board_rev, (char)ivl_carrier_revision);

	check_fec_pin();

	/* Misc: BIST mode pin */
	mxc_request_iomux(IVL_PIN_BIST_MODE, MUX_CONFIG_GPIO);
	mxc_iomux_set_pad(IVL_PIN_BIST_MODE, PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD | PAD_CTL_100K_PD);
	set_iomux_input_gpio(IVL_PIN_BIST_MODE);
	gpio_request(IOMUX_TO_GPIO(IVL_PIN_BIST_MODE), __stringify(IVL_PIN_BIST_MODE));
	gpio_direction_input(IOMUX_TO_GPIO(IVL_PIN_BIST_MODE));
}

/*!
 * Get IRQ for an MX35 pin used as a GPIO
 */
int mx35_ivlboard_get_gpio_irq(enum ivlboard_irq irqno)
{
	int irq = -1;

	switch (irqno) {
	case IVLBOARD_IRQ_SDCARD1_DET:
		irq = IOMUX_TO_IRQ(IVL_PIN_SD1_DETECT);
		break;
	case IVLBOARD_IRQ_SDCARD2_DET:
		irq = IOMUX_TO_IRQ(IVL_PIN_SD2_DETECT);
		break;
	case IVLBOARD_IRQ_HP1_DET:
		irq = IOMUX_TO_IRQ(IVL_PIN_HP_DET1);
		break;
	case IVLBOARD_IRQ_HP2_DET:
		irq = IOMUX_TO_IRQ(IVL_PIN_HP_DET2);
		break;
	case IVLBOARD_IRQ_LINE_IN_DET:
		irq = IOMUX_TO_IRQ(IVL_PIN_LINE_IN_DETECT);
		break;
	case IVLBOARD_IRQ_MIC_IN_DET:
		irq = IOMUX_TO_IRQ(IVL_PIN_MIC_IN_DETECT);
		break;
	default:
		BUG();
		break;
	}
	return irq;
}

/*!
 * Setup GPIO for a UART port to be active
 *
 * @param  port         a UART port
 * @param  no_irda      indicates if the port is used for SIR
 */
void gpio_uart_active(int port, int no_irda)
{
	/*
	 * Configure the IOMUX control registers for the UART signals.
	 * Hardware flow control pins are not configured because they are used
	 * for other purposes.
	 */
	switch (port) {
		/* UART 1 IOMUX Configs */
	case 0:
		mxc_request_iomux(MX35_PIN_RXD1, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_TXD1, MUX_CONFIG_FUNC);

		mxc_iomux_set_pad(MX35_PIN_RXD1,
				  PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE |
				  PAD_CTL_PUE_PUD | PAD_CTL_100K_PU);
		mxc_iomux_set_pad(MX35_PIN_TXD1,
				  PAD_CTL_PUE_PUD | PAD_CTL_100K_PD);
		break;
		/* UART 2 IOMUX Configs */
	case 1:
		mxc_request_iomux(MX35_PIN_TXD2, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_RXD2, MUX_CONFIG_FUNC);
		mxc_iomux_set_pad(MX35_PIN_RXD2,
				  PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE |
				  PAD_CTL_PUE_PUD | PAD_CTL_100K_PU);
		mxc_iomux_set_pad(MX35_PIN_TXD2,
				  PAD_CTL_PUE_PUD | PAD_CTL_100K_PD);
		break;
		/* UART 3 IOMUX Configs */
	case 2:
		mxc_request_iomux(IVL_PIN_UART3_RX, MUX_CONFIG_ALT1);	//UART3_RX
		mxc_request_iomux(IVL_PIN_UART3_TX, MUX_CONFIG_ALT1);	//UART3_TX
		mxc_iomux_set_pad(IVL_PIN_UART3_RX,
				  PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE |
				  PAD_CTL_PUE_PUD | PAD_CTL_100K_PU);
		mxc_iomux_set_pad(IVL_PIN_UART3_TX,
				  PAD_CTL_PUE_PUD | PAD_CTL_100K_PD);

		mxc_iomux_set_input(MUX_IN_UART3_UART_RXD_MUX, INPUT_CTL_PATH2);
		break;
	default:
		break;
	}

}

EXPORT_SYMBOL(gpio_uart_active);

/*!
 * Setup GPIO for a UART port to be inactive
 *
 * @param  port         a UART port
 * @param  no_irda      indicates if the port is used for SIR
 */
void gpio_uart_inactive(int port, int no_irda)
{
	switch (port) {
	case 0:
		gpio_request(IOMUX_TO_GPIO(MX35_PIN_RXD1), NULL);
		gpio_request(IOMUX_TO_GPIO(MX35_PIN_TXD1), NULL);

		mxc_free_iomux(MX35_PIN_RXD1, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_TXD1, MUX_CONFIG_GPIO);
		break;
	case 1:
		gpio_request(IOMUX_TO_GPIO(MX35_PIN_RXD2), NULL);
		gpio_request(IOMUX_TO_GPIO(MX35_PIN_TXD2), NULL);

		mxc_free_iomux(MX35_PIN_RXD2, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_TXD2, MUX_CONFIG_GPIO);
		break;
	case 2:
		gpio_request(IOMUX_TO_GPIO(IVL_PIN_UART3_RX), NULL);
		gpio_request(IOMUX_TO_GPIO(IVL_PIN_UART3_TX), NULL);

		mxc_free_iomux(IVL_PIN_UART3_RX, MUX_CONFIG_GPIO);
		mxc_free_iomux(IVL_PIN_UART3_TX, MUX_CONFIG_GPIO);
		mxc_iomux_set_input(MUX_IN_UART3_UART_RXD_MUX, INPUT_CTL_PATH0);
		break;
	default:
		break;
	}
}

EXPORT_SYMBOL(gpio_uart_inactive);

/*!
 * Configure the IOMUX GPR register to receive shared SDMA UART events
 *
 * @param  port         a UART port
 */
void config_uartdma_event(int port)
{
}

EXPORT_SYMBOL(config_uartdma_event);

/*!
 * Test if the FEC is present. The FEC is removable and for debug only.
 */
bool gpio_fec_is_chip_present(void)
{
	/* Value is precalculated on GPIO init to avoid fighting over the pin. */
	return fec_is_present;
}

EXPORT_SYMBOL(gpio_fec_is_chip_present);

/*!
 * Activate FEC (mutually exclusive with USB host transceiver)
 */
void gpio_fec_active(void)
{
	mxc_request_iomux(MX35_PIN_FEC_TX_CLK, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_RX_CLK, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_RX_DV, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_COL, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_RDATA0, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_TDATA0, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_TX_EN, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_MDC, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_MDIO, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_TX_ERR, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_RX_ERR, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_CRS, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_RDATA1, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_TDATA1, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_RDATA2, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_TDATA2, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_RDATA3, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_TDATA3, MUX_CONFIG_FUNC);

#define FEC_PAD_CTL_COMMON (PAD_CTL_DRV_3_3V|PAD_CTL_PUE_PUD| \
			PAD_CTL_ODE_CMOS|PAD_CTL_DRV_NORMAL|PAD_CTL_SRE_SLOW)
	mxc_iomux_set_pad(MX35_PIN_FEC_TX_CLK, FEC_PAD_CTL_COMMON |
			  PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE |
			  PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_RX_CLK,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_SCHMITZ |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_RX_DV,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_SCHMITZ |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_COL,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_SCHMITZ |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_RDATA0,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_SCHMITZ |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_TDATA0,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_CMOS |
			  PAD_CTL_PKE_NONE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_TX_EN,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_CMOS |
			  PAD_CTL_PKE_NONE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_MDC,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_CMOS |
			  PAD_CTL_PKE_NONE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_MDIO,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_SCHMITZ |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_22K_PU);
	mxc_iomux_set_pad(MX35_PIN_FEC_TX_ERR,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_CMOS |
			  PAD_CTL_PKE_NONE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_RX_ERR,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_SCHMITZ |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_CRS,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_SCHMITZ |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_RDATA1,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_SCHMITZ |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_TDATA1,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_CMOS |
			  PAD_CTL_PKE_NONE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_RDATA2,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_SCHMITZ |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_TDATA2,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_CMOS |
			  PAD_CTL_PKE_NONE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_RDATA3,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_SCHMITZ |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_TDATA3,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_CMOS |
			  PAD_CTL_PKE_NONE | PAD_CTL_100K_PD);
#undef FEC_PAD_CTL_COMMON

	/* Select FEC in FEC/USB mux */
	if (ivl_board_revision == IVL_PCB0417_A)
		gpio_set_value(IOMUX_TO_GPIO(IVL_PIN_FEC_USB_MUX), 0);

	/* Deassert FEC reset */
	udelay(100);
	gpio_set_value(IOMUX_TO_GPIO(IVL_PIN_FEC_RESET), 1);
}

EXPORT_SYMBOL(gpio_fec_active);

void gpio_fec_inactive(void)
{
	gpio_set_value(IOMUX_TO_GPIO(IVL_PIN_FEC_RESET), 0);		/* Assert RESET# */
	if (ivl_board_revision == IVL_PCB0417_A)
		gpio_set_value(IOMUX_TO_GPIO(IVL_PIN_FEC_USB_MUX), 1);	/* Select USB */

	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_TX_CLK), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_RX_CLK), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_RX_DV), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_COL), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_RDATA0), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_TDATA0), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_MDIO), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_TX_ERR), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_RX_ERR), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_CRS), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_RDATA1), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_TDATA1), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_RDATA2), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_TDATA2), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_RDATA3), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_TDATA3), NULL);
	/* TODO Check: Don't gpio_request(...MX35_PIN_FEC_TX_EN or FEC_MDC) because the
	 * corresponding GPIOs are in use elsewhere.
	 */

	mxc_free_iomux(MX35_PIN_FEC_TX_CLK, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_RX_CLK, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_RX_DV, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_COL, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_RDATA0, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_TDATA0, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_MDIO, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_TX_ERR, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_RX_ERR, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_CRS, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_RDATA1, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_TDATA1, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_RDATA2, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_TDATA2, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_RDATA3, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_TDATA3, MUX_CONFIG_GPIO);

	/* The GPIO that these pins mux to is already in use, so they can't be
	 * switched off in the normal way by assigning them to that GPIO.
	 * Instead we just "disown" the pin without changing the mux state.
	 * We could probably mxc_free_iomux(..., MUX_CONFIG_FUNC) for all of
	 * them safely.
	 */
	mxc_free_iomux(MX35_PIN_FEC_TX_EN, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_FEC_MDC, MUX_CONFIG_FUNC);
}

EXPORT_SYMBOL(gpio_fec_inactive);

/*!
 * Setup GPIO for an I2C device to be active
 *
 * @param  i2c_num         an I2C device
 */
void gpio_i2c_active(int i2c_num)
{

#define PAD_CONFIG (PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD | PAD_CTL_ODE_OpenDrain)

	switch (i2c_num) {
	case 0:
		mxc_request_iomux(MX35_PIN_I2C1_CLK, MUX_CONFIG_SION);
		mxc_request_iomux(MX35_PIN_I2C1_DAT, MUX_CONFIG_SION);

		mxc_iomux_set_pad(MX35_PIN_I2C1_CLK, PAD_CONFIG);
		mxc_iomux_set_pad(MX35_PIN_I2C1_DAT, PAD_CONFIG);
		break;
	case 1:
		mxc_request_iomux(MX35_PIN_I2C2_CLK, MUX_CONFIG_SION);
		mxc_request_iomux(MX35_PIN_I2C2_DAT, MUX_CONFIG_SION);

		mxc_iomux_set_pad(MX35_PIN_I2C2_CLK, PAD_CONFIG);
		mxc_iomux_set_pad(MX35_PIN_I2C2_DAT, PAD_CONFIG);

		break;
	case 2:
		mxc_request_iomux(MX35_PIN_ATA_DATA12, MUX_CONFIG_ALT1 | MUX_CONFIG_SION);	//I2C3_CLK
		mxc_request_iomux(MX35_PIN_ATA_DATA13, MUX_CONFIG_ALT1 | MUX_CONFIG_SION);	//I2C3_DAT

		mxc_iomux_set_pad(MX35_PIN_ATA_DATA12, PAD_CONFIG);
		mxc_iomux_set_pad(MX35_PIN_ATA_DATA13, PAD_CONFIG);
		mxc_iomux_set_input(MUX_IN_I2C3_SCL_IN, INPUT_CTL_PATH3);	//Input from ATA_DATA12
		mxc_iomux_set_input(MUX_IN_I2C3_SDA_IN, INPUT_CTL_PATH3);	//Input from ATA_DATA13
		break;
	default:
		break;
	}

#undef PAD_CONFIG

}

EXPORT_SYMBOL(gpio_i2c_active);

/*!
 * Setup GPIO for an I2C device to be inactive
 *
 * @param  i2c_num         an I2C device
 */
void gpio_i2c_inactive(int i2c_num)
{
	switch (i2c_num) {
	case 0:
		break;
	case 1:
		break;
	case 2:
		mxc_request_iomux(MX35_PIN_ATA_DATA12, MUX_CONFIG_GPIO);
		mxc_request_iomux(MX35_PIN_ATA_DATA13, MUX_CONFIG_GPIO);
		break;
	default:
		break;
	}
}

EXPORT_SYMBOL(gpio_i2c_inactive);

/*!
 * Setup GPIO for a CSPI device to be active
 *
 * @param  cspi_mod         an CSPI device
 */
void gpio_spi_active(int cspi_mod)
{
	switch (cspi_mod) {
	case 0:
		/* SPI1 */
		mxc_request_iomux(MX35_PIN_CSPI1_MOSI, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_CSPI1_MISO, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_CSPI1_SS0, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_CSPI1_SS1, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_CSPI1_SCLK, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_CSPI1_SPI_RDY, MUX_CONFIG_FUNC);

		mxc_iomux_set_pad(MX35_PIN_CSPI1_MOSI,
				  PAD_CTL_DRV_3_3V | PAD_CTL_HYS_SCHMITZ |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD |
				  PAD_CTL_100K_PD | PAD_CTL_DRV_NORMAL);
		mxc_iomux_set_pad(MX35_PIN_CSPI1_MISO,
				  PAD_CTL_DRV_3_3V | PAD_CTL_HYS_SCHMITZ |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD |
				  PAD_CTL_100K_PD | PAD_CTL_DRV_NORMAL);
		mxc_iomux_set_pad(MX35_PIN_CSPI1_SS0,
				  PAD_CTL_DRV_3_3V | PAD_CTL_HYS_SCHMITZ |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD |
				  PAD_CTL_100K_PU | PAD_CTL_ODE_CMOS |
				  PAD_CTL_DRV_NORMAL);
		mxc_iomux_set_pad(MX35_PIN_CSPI1_SS1,
				  PAD_CTL_DRV_3_3V | PAD_CTL_HYS_SCHMITZ |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD |
				  PAD_CTL_100K_PU | PAD_CTL_ODE_CMOS |
				  PAD_CTL_DRV_NORMAL);
		mxc_iomux_set_pad(MX35_PIN_CSPI1_SCLK,
				  PAD_CTL_DRV_3_3V | PAD_CTL_HYS_SCHMITZ |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD |
				  PAD_CTL_100K_PD | PAD_CTL_DRV_NORMAL);
		mxc_iomux_set_pad(MX35_PIN_CSPI1_SPI_RDY,
				  PAD_CTL_DRV_3_3V | PAD_CTL_HYS_SCHMITZ |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD |
				  PAD_CTL_100K_PU | PAD_CTL_DRV_NORMAL);
		break;
	case 1:
		/* SPI2 */
		break;
	default:
		break;
	}
}

EXPORT_SYMBOL(gpio_spi_active);

/*!
 * Setup GPIO for a CSPI device to be inactive
 *
 * @param  cspi_mod         a CSPI device
 */
void gpio_spi_inactive(int cspi_mod)
{
	switch (cspi_mod) {
	case 0:
		/* SPI1 */
		gpio_request(IOMUX_TO_GPIO(MX35_PIN_CSPI1_MOSI), NULL);
		gpio_request(IOMUX_TO_GPIO(MX35_PIN_CSPI1_MISO), NULL);
		gpio_request(IOMUX_TO_GPIO(MX35_PIN_CSPI1_SS0), NULL);
		gpio_request(IOMUX_TO_GPIO(MX35_PIN_CSPI1_SS1), NULL);
		gpio_request(IOMUX_TO_GPIO(MX35_PIN_CSPI1_SCLK), NULL);
		gpio_request(IOMUX_TO_GPIO(MX35_PIN_CSPI1_SPI_RDY), NULL);

		mxc_free_iomux(MX35_PIN_CSPI1_MOSI, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_CSPI1_MISO, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_CSPI1_SS0, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_CSPI1_SS1, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_CSPI1_SCLK, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_CSPI1_SPI_RDY, MUX_CONFIG_GPIO);
		break;
	case 1:
		/* SPI2 */
		break;
	default:
		break;
	}
}

EXPORT_SYMBOL(gpio_spi_inactive);

static iomux_pin_name_t lcd_pins[] = {
		MX35_PIN_LD0,
		MX35_PIN_LD1,
		MX35_PIN_LD2,
		MX35_PIN_LD3,
		MX35_PIN_LD4,
		MX35_PIN_LD5,
		MX35_PIN_LD6,
		MX35_PIN_LD7,
		MX35_PIN_LD8,
		MX35_PIN_LD9,
		MX35_PIN_LD10,
		MX35_PIN_LD11,
		MX35_PIN_LD12,
		MX35_PIN_LD13,
		MX35_PIN_LD14,
		MX35_PIN_LD15,
		MX35_PIN_LD16,
		MX35_PIN_LD17,
		MX35_PIN_D3_VSYNC,
		MX35_PIN_D3_HSYNC,
		MX35_PIN_D3_FPSHIFT,
		MX35_PIN_D3_DRDY,
		MX35_PIN_CONTRAST,
};

/*!
 * Setup GPIO for LCD to be active
 */
void gpio_lcd_active(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(lcd_pins); i++) {
		mxc_request_iomux(lcd_pins[i], MUX_CONFIG_FUNC);
		/* Lower drive strength on LCD to reduce EMI radiation. This
		 * will cause problems for larger LCD strengths with higher clock
		 * rates.
		 */
		mxc_iomux_set_pad(lcd_pins[i], PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW);
	}
}

EXPORT_SYMBOL(gpio_lcd_active);

/*!
 * Setup GPIO for LCD to be inactive
 */
void gpio_lcd_inactive(void)
{
	/* Do nothing.  See gpio_lcd_disable_for_emi() */
}

EXPORT_SYMBOL(gpio_lcd_inactive);

/*!
 * For the EMI scan it is necessary to shut down the LCD system to see if it is a harmful contributor.
 * This function steals the pins and grounds them.  However, some of the GPIOs involved are already assigned
 * to other tasks (since pin count > GPIO count, some pins share a GPIO output driver), so this will complain
 * and generate a pile of stack traces.  So we have a special function for this purpose, which complains but
 * seems harmless.  For normal purposes, we don't care of the LCD interface still sort of runs, so we leave it
 * be.  Turning off the framebuffer directly seemed more difficult than shutting down the pins.
 */
void gpio_lcd_disable_for_emi(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(lcd_pins); i++) {
		mxc_free_iomux(lcd_pins[i], MUX_CONFIG_FUNC);
		mxc_request_iomux(lcd_pins[i], MUX_CONFIG_GPIO);
		gpio_request(IOMUX_TO_GPIO(lcd_pins[i]), NULL);
		gpio_direction_output(IOMUX_TO_GPIO(lcd_pins[i]), 0);
		gpio_set_value(IOMUX_TO_GPIO(lcd_pins[i]), 0);
	}
}

EXPORT_SYMBOL(gpio_lcd_disable_for_emi);

/*!
 * Setup GPIO for SDHC to be active
 *
 * @param module SDHC module number
 */
void gpio_sdhc_active(int module)
{
#define SD_PAD_SIGNAL	(PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD | PAD_CTL_DRV_HIGH | PAD_CTL_SRE_SLOW)
#define SD_PAD_OOB		(PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD | PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW)

	unsigned int pad_val;

	switch (module) {
	case 0:
		mxc_request_iomux(MX35_PIN_SD1_CLK,
				  MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_request_iomux(MX35_PIN_SD1_CMD,
				  MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_request_iomux(MX35_PIN_SD1_DATA0,
				  MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_request_iomux(MX35_PIN_SD1_DATA1,
				  MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_request_iomux(MX35_PIN_SD1_DATA2,
				  MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_request_iomux(MX35_PIN_SD1_DATA3,
				  MUX_CONFIG_FUNC | MUX_CONFIG_SION);

		pad_val = SD_PAD_SIGNAL | PAD_CTL_47K_PU | PAD_CTL_HYS_SCHMITZ;
		mxc_iomux_set_pad(MX35_PIN_SD1_CMD, pad_val);
		mxc_iomux_set_pad(MX35_PIN_SD1_DATA0, pad_val);
		mxc_iomux_set_pad(MX35_PIN_SD1_DATA1, pad_val);
		mxc_iomux_set_pad(MX35_PIN_SD1_DATA2, pad_val);

		/* No Schmitz for CLK */
		pad_val = SD_PAD_SIGNAL | PAD_CTL_47K_PU;
		mxc_iomux_set_pad(MX35_PIN_SD1_CLK, pad_val);

		/* Freescale has DATA3 with 100k PU while others have 47k PU.
		 * The board has 10k PUs for all signals anyway.
		 */
		pad_val = SD_PAD_SIGNAL | PAD_CTL_100K_PU | PAD_CTL_HYS_SCHMITZ;
		mxc_iomux_set_pad(MX35_PIN_SD1_DATA3, pad_val);

		/* Set up card detect GPIOs */
		mxc_request_iomux(IVL_PIN_SD1_WP, MUX_CONFIG_GPIO);
		mxc_request_iomux(IVL_PIN_SD1_DETECT, MUX_CONFIG_GPIO);

		pad_val = SD_PAD_OOB | PAD_CTL_47K_PU | PAD_CTL_HYS_SCHMITZ;
		mxc_iomux_set_pad(IVL_PIN_SD1_WP, pad_val);
		mxc_iomux_set_pad(IVL_PIN_SD1_DETECT, pad_val);

		set_iomux_input_gpio(IVL_PIN_SD1_WP);
		set_iomux_input_gpio(IVL_PIN_SD1_DETECT);

		gpio_request(IOMUX_TO_GPIO(IVL_PIN_SD1_WP), __stringify(IVL_PIN_SD1_WP));
		gpio_request(IOMUX_TO_GPIO(IVL_PIN_SD1_DETECT), __stringify(IVL_PIN_SD1_DETECT));
		gpio_direction_input(IOMUX_TO_GPIO(IVL_PIN_SD1_WP));
		gpio_direction_input(IOMUX_TO_GPIO(IVL_PIN_SD1_DETECT));
		break;
	case 1:
		mxc_request_iomux(IVL_PIN_SD2_CLK,
				  MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_request_iomux(IVL_PIN_SD2_CMD,
				  MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_request_iomux(IVL_PIN_SD2_DATA0,
				  MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_request_iomux(IVL_PIN_SD2_DATA1,
				  MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_request_iomux(IVL_PIN_SD2_DATA2,
				  MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_request_iomux(IVL_PIN_SD2_DATA3,
				  MUX_CONFIG_FUNC | MUX_CONFIG_SION);

		pad_val = SD_PAD_SIGNAL | PAD_CTL_47K_PU | PAD_CTL_HYS_SCHMITZ;
		mxc_iomux_set_pad(IVL_PIN_SD2_CMD, pad_val);
		mxc_iomux_set_pad(IVL_PIN_SD2_DATA0, pad_val);
		mxc_iomux_set_pad(IVL_PIN_SD2_DATA1, pad_val);
		mxc_iomux_set_pad(IVL_PIN_SD2_DATA2, pad_val);

		/* No Schmitz for CLK */
		pad_val = SD_PAD_SIGNAL | PAD_CTL_47K_PU;
		mxc_iomux_set_pad(IVL_PIN_SD2_CLK, pad_val);

		/* Freescale has DATA3 with 100k PU while others have 47k PU.
		 * The board has 10k PUs for all signals anyway.
		 */
		pad_val = SD_PAD_SIGNAL | PAD_CTL_100K_PU | PAD_CTL_HYS_SCHMITZ;
		mxc_iomux_set_pad(IVL_PIN_SD2_DATA3, pad_val);

		/* Set up card detect GPIOs */
		mxc_request_iomux(IVL_PIN_SD2_WP, MUX_CONFIG_GPIO);
		mxc_request_iomux(IVL_PIN_SD2_DETECT, MUX_CONFIG_GPIO);

		pad_val = SD_PAD_OOB | PAD_CTL_47K_PU | PAD_CTL_HYS_SCHMITZ;
		mxc_iomux_set_pad(IVL_PIN_SD2_WP, pad_val);
		mxc_iomux_set_pad(IVL_PIN_SD2_DETECT, pad_val);

		set_iomux_input_gpio(IVL_PIN_SD2_WP);
		set_iomux_input_gpio(IVL_PIN_SD2_DETECT);

		gpio_request(IOMUX_TO_GPIO(IVL_PIN_SD2_WP), __stringify(IVL_PIN_SD2_WP));
		gpio_request(IOMUX_TO_GPIO(IVL_PIN_SD2_DETECT), __stringify(IVL_PIN_SD2_DETECT));
		gpio_direction_input(IOMUX_TO_GPIO(IVL_PIN_SD2_WP));
		gpio_direction_input(IOMUX_TO_GPIO(IVL_PIN_SD2_DETECT));
		break;
	case 2:
		BUG(); /* Not available */
		break;
	default:
		break;
	}

#undef SD_PAD_SIGNAL
#undef SD_PAD_OOB
}

EXPORT_SYMBOL(gpio_sdhc_active);

/*!
 * Setup GPIO for SDHC1 to be inactive
 *
 * @param module SDHC module number
 */
void gpio_sdhc_inactive(int module)
{
	switch (module) {
	case 0:
		mxc_free_iomux(MX35_PIN_SD1_CLK,
			       MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_free_iomux(MX35_PIN_SD1_CMD,
			       MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_free_iomux(MX35_PIN_SD1_DATA0,
			       MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_free_iomux(MX35_PIN_SD1_DATA1,
			       MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_free_iomux(MX35_PIN_SD1_DATA2,
			       MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_free_iomux(MX35_PIN_SD1_DATA3,
			       MUX_CONFIG_FUNC | MUX_CONFIG_SION);

		mxc_iomux_set_pad(MX35_PIN_SD1_CLK,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX35_PIN_SD1_CMD,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX35_PIN_SD1_DATA0,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX35_PIN_SD1_DATA1,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX35_PIN_SD1_DATA2,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX35_PIN_SD1_DATA3,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));

		gpio_request(IOMUX_TO_GPIO(IVL_PIN_SD1_DETECT), NULL);
		mxc_free_iomux(IVL_PIN_SD1_DETECT, MUX_CONFIG_GPIO);
		gpio_request(IOMUX_TO_GPIO(IVL_PIN_SD1_WP), NULL);
		mxc_free_iomux(IVL_PIN_SD1_WP, MUX_CONFIG_GPIO);
		break;
	case 1:
		mxc_free_iomux(IVL_PIN_SD2_CLK,
			       MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_free_iomux(IVL_PIN_SD2_CMD,
			       MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_free_iomux(IVL_PIN_SD2_DATA0,
			       MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_free_iomux(IVL_PIN_SD2_DATA1,
			       MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_free_iomux(IVL_PIN_SD2_DATA2,
			       MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_free_iomux(IVL_PIN_SD2_DATA3,
			       MUX_CONFIG_FUNC | MUX_CONFIG_SION);

		mxc_iomux_set_pad(IVL_PIN_SD2_CLK,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(IVL_PIN_SD2_CMD,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(IVL_PIN_SD2_DATA0,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(IVL_PIN_SD2_DATA1,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(IVL_PIN_SD2_DATA2,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(IVL_PIN_SD2_DATA3,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));

		gpio_request(IOMUX_TO_GPIO(IVL_PIN_SD2_DETECT), NULL);
		mxc_free_iomux(IVL_PIN_SD2_DETECT, MUX_CONFIG_GPIO);
		gpio_request(IOMUX_TO_GPIO(IVL_PIN_SD2_WP), NULL);
		mxc_free_iomux(IVL_PIN_SD2_WP, MUX_CONFIG_GPIO);
		break;
	case 2:
		BUG(); /* Not available */
		break;
	default:
		break;
	}
}

EXPORT_SYMBOL(gpio_sdhc_inactive);

/*
 * Probe for the card. If present the GPIO data would be set.
 */
unsigned int sdhc_get_card_det_status(struct device *dev)
{
	int val;

	if (to_platform_device(dev)->id == 0) {
		val = gpio_get_value(IOMUX_TO_GPIO(IVL_PIN_SD1_DETECT));
		if (val < 0)
			BUG();

		return val;
	} else if (to_platform_device(dev)->id == 1) {
		val = gpio_get_value(IOMUX_TO_GPIO(IVL_PIN_SD2_DETECT));
		if (val < 0)
			BUG();

		return val;
	} else {
		BUG();
		return 0;
	}
}

EXPORT_SYMBOL(sdhc_get_card_det_status);

/*!
 * Get pin value to detect write protection
 */
int sdhc_write_protect(struct device *dev)
{
	unsigned int val;

	if (to_platform_device(dev)->id == 0) {
		val = gpio_get_value(IOMUX_TO_GPIO(IVL_PIN_SD1_WP));
		if (val < 0)
			BUG();

		return (int)val;
	} else if (to_platform_device(dev)->id == 1) {
		val = gpio_get_value(IOMUX_TO_GPIO(IVL_PIN_SD2_WP));
		if (val < 0)
			BUG();

		return (int)val;
	} else {
		BUG();
		return 0;
	}
}

EXPORT_SYMBOL(sdhc_write_protect);

struct gpio_pin_config {
	iomux_pin_name_t pin;
	const char *name;
	iomux_pin_cfg_t cfg;
	iomux_input_select_t input;
	u32 input_cfg;
};

/*
 * While trying to get USB host working, I experimented with copying the init procedure from the MX51.  They
 * hijack the STP pin as a way of letting the ULPI xcvr know that the bus is being taken over.  I don't know
 * if this is necessary, or even desirable.  See gpio_usbh2_setback_stp.  This seems to be removed from the
 * 2.6.28 release for the MX51.  Still, it works, so doesn't seem to be worth fussing over.
 */
#define MX51_STYLE_INIT 0
static struct gpio_pin_config usbh2_pins_pcb0417[] = {
		{ MX35_PIN_SD2_DATA1, "USBH2_DAT0", MUX_CONFIG_ALT4, MUX_IN_USB_UH2_DATA_0, INPUT_CTL_PATH0 },
		{ MX35_PIN_SD2_DATA2, "USBH2_DAT1", MUX_CONFIG_ALT4, MUX_IN_USB_UH2_DATA_1, INPUT_CTL_PATH0 },
		{ MX35_PIN_SD2_DATA3, "USBH2_DAT2", MUX_CONFIG_ALT4, MUX_IN_USB_UH2_DATA_2, INPUT_CTL_PATH0 },
		{ MX35_PIN_FEC_TX_EN, "USBH2_DAT3", MUX_CONFIG_ALT3, MUX_IN_USB_UH2_DATA_3, INPUT_CTL_PATH1 },

		{ MX35_PIN_SD2_CMD, "USBH2_DAT4", MUX_CONFIG_ALT4, MUX_IN_USB_UH2_DATA_4, INPUT_CTL_PATH0 },
		{ MX35_PIN_SD2_CLK, "USBH2_DAT5", MUX_CONFIG_ALT4, MUX_IN_USB_UH2_DATA_5, INPUT_CTL_PATH0 },
		{ MX35_PIN_SD2_DATA0, "USBH2_DAT6", MUX_CONFIG_ALT4, MUX_IN_USB_UH2_DATA_6, INPUT_CTL_PATH0 },
		{ MX35_PIN_FEC_RX_ERR, "USBH2_DAT7", MUX_CONFIG_ALT3, MUX_IN_USB_UH2_DATA_7, INPUT_CTL_PATH1 },
		{ MX35_PIN_FEC_TX_CLK, "USBH2_DIR", MUX_CONFIG_ALT3, MUX_IN_USB_UH2_DIR, INPUT_CTL_PATH1 },
#if !MX51_STYLE_INIT
		{ MX35_PIN_FEC_RX_CLK, "USBH2_STP", MUX_CONFIG_ALT3, UINT_MAX, 0 },
#endif
		{ MX35_PIN_FEC_RX_DV, "USBH2_NXT", MUX_CONFIG_ALT3, MUX_IN_USB_UH2_NXT, INPUT_CTL_PATH1 },
		{ MX35_PIN_GPIO3_0, "USBH2_CLK", MUX_CONFIG_ALT1, UINT_MAX, 0 },
};

static struct gpio_pin_config usbh2_pins_pcb0423[] = {
		{ MX35_PIN_FEC_COL, "USBH2_DAT0", MUX_CONFIG_ALT3, MUX_IN_USB_UH2_DATA_0, INPUT_CTL_PATH1 },
		{ MX35_PIN_FEC_RDATA0, "USBH2_DAT1", MUX_CONFIG_ALT3, MUX_IN_USB_UH2_DATA_1, INPUT_CTL_PATH1 },
		{ MX35_PIN_FEC_TDATA0, "USBH2_DAT2", MUX_CONFIG_ALT3, MUX_IN_USB_UH2_DATA_2, INPUT_CTL_PATH1 },
		{ MX35_PIN_FEC_TX_EN, "USBH2_DAT3", MUX_CONFIG_ALT3, MUX_IN_USB_UH2_DATA_3, INPUT_CTL_PATH1 },

		{ MX35_PIN_FEC_MDC, "USBH2_DAT4", MUX_CONFIG_ALT3, MUX_IN_USB_UH2_DATA_4, INPUT_CTL_PATH1 },
		{ MX35_PIN_FEC_MDIO, "USBH2_DAT5", MUX_CONFIG_ALT3, MUX_IN_USB_UH2_DATA_5, INPUT_CTL_PATH1 },
		{ MX35_PIN_FEC_TX_ERR, "USBH2_DAT6", MUX_CONFIG_ALT3, MUX_IN_USB_UH2_DATA_6, INPUT_CTL_PATH1 },
		{ MX35_PIN_FEC_RX_ERR, "USBH2_DAT7", MUX_CONFIG_ALT3, MUX_IN_USB_UH2_DATA_7, INPUT_CTL_PATH1 },
		{ MX35_PIN_FEC_TX_CLK, "USBH2_DIR", MUX_CONFIG_ALT3, MUX_IN_USB_UH2_DIR, INPUT_CTL_PATH1 },
#if !MX51_STYLE_INIT
		{ MX35_PIN_FEC_RX_CLK, "USBH2_STP", MUX_CONFIG_ALT3, UINT_MAX, 0 },
#endif
		{ MX35_PIN_FEC_RX_DV, "USBH2_NXT", MUX_CONFIG_ALT3, MUX_IN_USB_UH2_NXT, INPUT_CTL_PATH1 },
		{ MX35_PIN_GPIO3_0, "USBH2_CLK", MUX_CONFIG_ALT1, UINT_MAX, 0 },
		{ IVL_PIN_USBH2_OC, "USBH2_OC", MUX_CONFIG_ALT3, MUX_IN_USB_UH2_USB_OC, INPUT_CTL_PATH2 },
};

static struct gpio_pin_config *usbh2_pins;
static int usbh2_pins_len;

/*
 *  USB Host2
 */
int gpio_usbh2_active(void)
{
	int i;

	/* Can't start USB host2 if FEC is inserted */
	if (gpio_fec_is_chip_present())
		return -EBUSY;

	if (ivl_board_revision == IVL_PCB0417_A) {
		usbh2_pins = usbh2_pins_pcb0417;
		usbh2_pins_len = ARRAY_SIZE(usbh2_pins_pcb0417);
	} else {
		if (ivl_board_revision != IVL_PCB0423_A)
			pr_info("%s: no config defined for this PCB, assuming PCB0423A is okay\n", __func__);
		usbh2_pins = usbh2_pins_pcb0423;
		usbh2_pins_len = ARRAY_SIZE(usbh2_pins_pcb0423);
	}

	if (ivl_board_revision != IVL_PCB0417_A) {
		/* Reset the transceiver while we manipulate the pins. Rev A does not have this pin. */
		mxc_request_iomux(IVL_PIN_USB_XCVR_RST, MUX_CONFIG_GPIO);
		mxc_iomux_set_pad(IVL_PIN_USB_XCVR_RST, PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW);
		gpio_request(IOMUX_TO_GPIO(IVL_PIN_USB_XCVR_RST), __stringify(IVL_PIN_USB_XCVR_RST));
		gpio_direction_output(IOMUX_TO_GPIO(IVL_PIN_USB_XCVR_RST), 0);
		gpio_set_value(IOMUX_TO_GPIO(IVL_PIN_USB_XCVR_RST), 0);

		/* Ensure FEC is held in reset */
		gpio_set_value(IOMUX_TO_GPIO(IVL_PIN_FEC_RESET), 0);
	}

#if MX51_STYLE_INIT
	/* Set USBH2_STP to GPIO and toggle it */
	mxc_request_iomux(MX35_PIN_FEC_RX_CLK, MUX_CONFIG_GPIO);
	mxc_iomux_set_pad(MX35_PIN_FEC_RX_CLK, PAD_CTL_DRV_NORMAL |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_RX_CLK), "USBH2_STP_GPIO");
	gpio_direction_output(IOMUX_TO_GPIO(MX35_PIN_FEC_RX_CLK), 0);
	gpio_set_value(IOMUX_TO_GPIO(MX35_PIN_FEC_RX_CLK), 1);

	msleep(3);
#endif

#if MX51_STYLE_INIT
	#define H2_PAD_CFG (PAD_CTL_DRV_3_3V | PAD_CTL_HYS_SCHMITZ | PAD_CTL_PUE_KEEPER | \
			PAD_CTL_PKE_ENABLE | PAD_CTL_ODE_CMOS | PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_FAST)
#else
	#define H2_PAD_CFG (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_FAST)
#endif
	for (i = 0; i < usbh2_pins_len; i++) {
		struct gpio_pin_config *pincfg = &usbh2_pins[i];
		mxc_request_iomux(pincfg->pin, pincfg->cfg);
		mxc_iomux_set_pad(pincfg->pin, H2_PAD_CFG);
		if (pincfg->input < UINT_MAX)
			mxc_iomux_set_input(pincfg->input, pincfg->input_cfg);
	}


	if (ivl_board_revision == IVL_PCB0417_A) {
		/* Set up USB/FEC mux */
		gpio_set_value(IOMUX_TO_GPIO(IVL_PIN_FEC_USB_MUX), 1);
	} else {
		/* If we have a reset line, come out of reset */
		msleep(3);
		gpio_set_value(IOMUX_TO_GPIO(IVL_PIN_USB_XCVR_RST), 1);
		pr_info("usb3315 reset completed\n");
	}

	return 0;
}

EXPORT_SYMBOL(gpio_usbh2_active);

void gpio_usbh2_setback_stp(void)
{
#if MX51_STYLE_INIT
	/* setback USBH2_STP to be function */
	gpio_free(IOMUX_TO_GPIO(MX35_PIN_FEC_RX_CLK));
	mxc_free_iomux(MX35_PIN_FEC_RX_CLK, MUX_CONFIG_GPIO);
	mxc_request_iomux(MX35_PIN_FEC_RX_CLK, MUX_CONFIG_ALT3);
	mxc_iomux_set_pad(MX35_PIN_FEC_RX_CLK, H2_PAD_CFG);
#endif
}

EXPORT_SYMBOL(gpio_usbh2_setback_stp);

void gpio_usbh2_inactive(void)
{
	int i;

	/* If FEC is present, Host2 will not be active, so it should be not inactive either. */
	if (gpio_fec_is_chip_present())
		return;

	for (i = 0; i < usbh2_pins_len; i++) {
		struct gpio_pin_config *pincfg = &usbh2_pins[i];
		mxc_free_iomux(pincfg->pin, MUX_CONFIG_FUNC);
	}
}

EXPORT_SYMBOL(gpio_usbh2_inactive);

#undef H2_PAD_CFG

/*
 *  USB OTG UTMI
 */
int gpio_usbotg_utmi_active(void)
{
	mxc_request_iomux(MX35_PIN_USBOTG_PWR, MUX_CONFIG_FUNC);
	mxc_iomux_set_pad(MX35_PIN_USBOTG_PWR, 0x0040);
	mxc_request_iomux(MX35_PIN_USBOTG_OC, MUX_CONFIG_FUNC);
	mxc_iomux_set_pad(MX35_PIN_USBOTG_OC, 0x01c0);

	return 0;
}

EXPORT_SYMBOL(gpio_usbotg_utmi_active);

void gpio_usbotg_utmi_inactive(void)
{
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_USBOTG_PWR), NULL);
	mxc_free_iomux(MX35_PIN_USBOTG_PWR, MUX_CONFIG_GPIO);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_USBOTG_OC), NULL);
	mxc_free_iomux(MX35_PIN_USBOTG_OC, MUX_CONFIG_GPIO);
}

EXPORT_SYMBOL(gpio_usbotg_utmi_inactive);

/*!
 * This function activates DAM ports 3 to enable
 * audio I/O.
 */
void gpio_activate_audio_ports(void)
{
	unsigned int pad_val;

	mxc_request_iomux(MX35_PIN_STXD4, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_SRXD4, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_SCK4, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_STXFS4, MUX_CONFIG_FUNC);

	pad_val = PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PU |
	    PAD_CTL_PUE_PUD;
	mxc_iomux_set_pad(MX35_PIN_STXD4, pad_val);
	mxc_iomux_set_pad(MX35_PIN_SRXD4, pad_val);
	mxc_iomux_set_pad(MX35_PIN_SCK4, pad_val);
	mxc_iomux_set_pad(MX35_PIN_STXFS4, pad_val);
}

EXPORT_SYMBOL(gpio_activate_audio_ports);

/*!
 * This function activates ESAI ports to enable
 * surround sound I/O
 */
void gpio_activate_esai_ports(void)
{
	unsigned int pad_val;

	mxc_request_iomux(MX35_PIN_HCKT, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_SCKT, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FST, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_TX0, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_TX1, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_TX5_RX0, MUX_CONFIG_FUNC);

	pad_val = PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PU |
	    PAD_CTL_PUE_PUD;
	mxc_iomux_set_pad(MX35_PIN_SCKT, pad_val);
	mxc_iomux_set_pad(MX35_PIN_FST, pad_val);
	mxc_iomux_set_pad(MX35_PIN_TX0, pad_val);
	mxc_iomux_set_pad(MX35_PIN_TX1, pad_val);
	mxc_iomux_set_pad(MX35_PIN_TX5_RX0, pad_val);

	pad_val =
	    PAD_CTL_DRV_HIGH | PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PU |
	    PAD_CTL_PUE_PUD;

	mxc_iomux_set_pad(MX35_PIN_HCKT, pad_val);
}

EXPORT_SYMBOL(gpio_activate_esai_ports);

/*!
 * This function deactivates ESAI ports to disable
 * surround sound I/O
 */
void gpio_deactivate_esai_ports(void)
{
	mxc_free_iomux(MX35_PIN_HCKT, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_SCKT, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_FST, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_TX0, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_TX1, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_TX2_RX3, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_TX5_RX0, MUX_CONFIG_FUNC);
	/*mxc_free_iomux(MX35_PIN_HCKR, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_SCKR, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FSR, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_TX3_RX2, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_TX4_RX1, MUX_CONFIG_GPIO);*/
}

EXPORT_SYMBOL(gpio_deactivate_esai_ports);

static struct gpio_pin_config audiodetect_pins[] = {
		{ IVL_PIN_HP_DET1, "IVL_PIN_HP_DET1", MUX_CONFIG_GPIO, 1 },
		{ IVL_PIN_HP_DET2, "IVL_PIN_HP_DET2", MUX_CONFIG_GPIO, 1 },
		{ IVL_PIN_LINE_IN_DETECT, "IVL_PIN_LINE_IN_DETECT", MUX_CONFIG_GPIO, 1 },
		{ IVL_PIN_MIC_IN_DETECT, "IVL_PIN_MIC_IN_DETECT", MUX_CONFIG_GPIO, 1 },
};

/* Enable pins related to detecting that sensing whether devices are
 * inserted into audio jacks.
 */
void gpio_audiodetect_active(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(audiodetect_pins); i++) {
		struct gpio_pin_config *pincfg = &audiodetect_pins[i];
		mxc_request_iomux(pincfg->pin, pincfg->cfg);
		mxc_iomux_set_pad(pincfg->pin, PAD_CTL_HYS_SCHMITZ |
				PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PD | PAD_CTL_PUE_PUD);
		if (pincfg->input)
			set_iomux_input_gpio(pincfg->pin);

		gpio_request(IOMUX_TO_GPIO(pincfg->pin), pincfg->name);
		gpio_direction_input(IOMUX_TO_GPIO(pincfg->pin));
	}
}
EXPORT_SYMBOL(gpio_audiodetect_active);

unsigned int gpio_hp1_detect(void)
{
	int val;
	val = gpio_get_value(IOMUX_TO_GPIO(IVL_PIN_HP_DET1));
	if (val < 0)
		BUG();
	return val;
}
EXPORT_SYMBOL(gpio_hp1_detect);

unsigned int gpio_hp2_detect(void)
{
	int val;
	val = gpio_get_value(IOMUX_TO_GPIO(IVL_PIN_HP_DET2));
	if (val < 0)
		BUG();
	return val;
}
EXPORT_SYMBOL(gpio_hp2_detect);

unsigned int gpio_line_in_detect(void)
{
	int val;
	val = gpio_get_value(IOMUX_TO_GPIO(IVL_PIN_LINE_IN_DETECT));
	if (val < 0)
		BUG();
	return val;
}
EXPORT_SYMBOL(gpio_line_in_detect);

unsigned int gpio_mic_in_detect(void)
{
	int val;
	if (ivl_carrier_revision == 'B')
		val = 0; /* Mic detection not supported for carrier rev B.  It can be done, but why bother? */
	else
		val = gpio_get_value(IOMUX_TO_GPIO(IVL_PIN_MIC_IN_DETECT));

	if (val < 0)
		BUG();
	return val;
}
EXPORT_SYMBOL(gpio_mic_in_detect);

/* Disable audio detect pins */
void gpio_audiodetect_inactive(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(audiodetect_pins); i++) {
		struct gpio_pin_config *pincfg = &audiodetect_pins[i];
		mxc_free_iomux(pincfg->pin, pincfg->cfg);
	}
}

EXPORT_SYMBOL(gpio_audiodetect_inactive);

int gpio_bist_mode_access(void)
{
	int val;
	val = gpio_get_value(IOMUX_TO_GPIO(IVL_PIN_BIST_MODE));
	if (val < 0)
		BUG();
	return val;
}
EXPORT_SYMBOL(gpio_bist_mode_access);

void gpio_pmic_active(void)
{

}

EXPORT_SYMBOL(gpio_pmic_active);

void gpio_shutdown_request_active(void)
{
	mxc_request_iomux(IVL_PIN_SHDN_RQ, MUX_CONFIG_GPIO);
	mxc_iomux_set_pad(IVL_PIN_SHDN_RQ, PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW);
	gpio_request(IOMUX_TO_GPIO(IVL_PIN_SHDN_RQ), __stringify(IVL_PIN_SHDN_RQ));
	gpio_direction_output(IOMUX_TO_GPIO(IVL_PIN_SHDN_RQ), 0);
	gpio_set_value(IOMUX_TO_GPIO(IVL_PIN_SHDN_RQ), 0);
}

EXPORT_SYMBOL(gpio_shutdown_request_active);

void gpio_shutdown_request_access(bool enable)
{
	gpio_set_value(IOMUX_TO_GPIO(IVL_PIN_SHDN_RQ), enable);
}

EXPORT_SYMBOL(gpio_shutdown_request_access);


struct reg_pin {
	iomux_pin_name_t pin;
	const char *name;
	bool active_high;
};

static struct reg_pin reg_pins[] = {
		{ .pin = IVL_PIN_ADC_PDN, .name = "IVL_PIN_ADC_PDN", .active_high = true }, /* ADC reset = low, so ADC enable = high */
		{ .pin = IVL_PIN_AUDIO_IN_PWR_EN, .name = "IVL_PIN_AUDIO_IN_PWR_EN", .active_high = true },
		{ .pin = IVL_PIN_MIC_PP_EN, .name = "IVL_PIN_MIC_PP_EN", .active_high = false },	/* Active high on rev B, low on others */
		{ .pin = IVL_PIN_HP_EN1, .name = "IVL_PIN_HP_EN1", .active_high = true },
		{ .pin = IVL_PIN_HP_EN2, .name = "IVL_PIN_HP_EN2", .active_high = true },
		{ .pin = IVL_PIN_MUTE_LOW, .name = "IVL_PIN_MUTE_LOW", .active_high = true }, /* Mute = low, line out enable = high */
		{ .pin = IVL_PIN_AUDIO_ATT, .name = "IVL_PIN_AUDIO_ATT", .active_high = true },
		{ .pin = IVL_PIN_VDD_AN_EN, .name = "IVL_PIN_VDD_AN_EN", .active_high = true },
};


static void gpio_reg_enable(int pin, bool enable)
{
	bool state;
	BUG_ON(pin >= ARRAY_SIZE(reg_pins));
	if (reg_pins[pin].active_high)
		state = enable;
	else
		state = !enable;
	gpio_set_value(IOMUX_TO_GPIO(reg_pins[pin].pin), state);
}

/*!
 * This function sets up GPIOs that turn peripherals on and off.
 */
void gpio_regulators_active(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(reg_pins); i++) {
		/* Compatibility hack */
		if (ivl_board_revision == IVL_PCB0417_A && reg_pins[i].pin == IVL_PIN_VDD_AN_EN)
			reg_pins[i].pin = IVL_PIN_VDD_AN_EN_OLD;

		mxc_request_iomux(reg_pins[i].pin, MUX_CONFIG_GPIO);
		mxc_iomux_set_pad(reg_pins[i].pin, PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW);
		gpio_request(IOMUX_TO_GPIO(reg_pins[i].pin), reg_pins[i].name);
		gpio_direction_output(IOMUX_TO_GPIO(reg_pins[i].pin), 0);

		/* Don't disable VDD_AN_EN for now; disable all others */
		if (reg_pins[i].pin == IVL_PIN_VDD_AN_EN_OLD
				|| reg_pins[i].pin == IVL_PIN_VDD_AN_EN)
			gpio_reg_enable(i, true); /* Start enabled */
		else
			gpio_reg_enable(i, false); /* Start disabled */
	}
}

EXPORT_SYMBOL(gpio_regulators_active);

void gpio_regulators_inactive(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(reg_pins); i++) {
		gpio_reg_enable(i, false);
		/* Don't disable GPIOs */
	}
}

EXPORT_SYMBOL(gpio_regulators_inactive);

void gpio_adc_enable(bool enable)
{
	/* Compatibility fix: only manipulate this pin for Rev B boards. */
	if (ivl_carrier_revision == 'B')
		gpio_reg_enable(0, enable);
}

EXPORT_SYMBOL(gpio_adc_enable);

void gpio_audio_in_pwr_enable(bool enable)
{
	gpio_reg_enable(1, enable);
}

EXPORT_SYMBOL(gpio_audio_in_pwr_enable);

void gpio_mic_phantom_enable(bool enable)
{
	/* Compatibility fix: for Carrier rev B, this pin is active high;
	 * for future versions it is active low.  Flip the logic accordingly.
	 */
	if (ivl_carrier_revision == 'B')
		gpio_reg_enable(2, !enable);
	else
		gpio_reg_enable(2, enable);
}

EXPORT_SYMBOL(gpio_mic_phantom_enable);

void gpio_headphone1_amp_enable(bool enable)
{
	gpio_reg_enable(3, enable);
}

EXPORT_SYMBOL(gpio_headphone1_amp_enable);

void gpio_headphone2_amp_enable(bool enable)
{
	gpio_reg_enable(4, enable);
}

EXPORT_SYMBOL(gpio_headphone2_amp_enable);

void gpio_line_out_amp_enable(bool enable)
{
	pr_info("line out amp enable\n");
	gpio_reg_enable(5, enable);
}

EXPORT_SYMBOL(gpio_line_out_amp_enable);

void gpio_audio_attn_enable(bool enable)
{
	gpio_reg_enable(6, enable);
}

EXPORT_SYMBOL(gpio_audio_attn_enable);

void gpio_vdd_an_enable(bool enable)
{
	gpio_reg_enable(7, enable);
}

EXPORT_SYMBOL(gpio_vdd_an_enable);

