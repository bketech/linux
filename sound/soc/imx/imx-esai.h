/*
 * imx-esai.h  --  ESAI driver header file for Freescale IMX
 *
 * Copyright 2008-2009 Freescale  Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef _MXC_ESAI_H
#define _MXC_ESAI_H

#define IMX_DAI_ESAI_TX 0x04
#define IMX_DAI_ESAI_RX 0x08
#define IMX_DAI_ESAI_TXRX (IMX_DAI_ESAI_TX | IMX_DAI_ESAI_RX)

struct imx_esai {
	bool network_mode;
	bool sync_mode;
};

extern struct snd_soc_dai imx_esai_dai[];

/* ESAI clock source */
#define ESAI_CLK_FSYS	0
#define ESAI_CLK_EXTAL 1

/* ESAI clock divider */
#define ESAI_TX_DIV_PSR	0
#define ESAI_TX_DIV_PM 1
#define ESAI_TX_DIV_FP	2
#define ESAI_RX_DIV_PSR	3
#define ESAI_RX_DIV_PM	4
#define ESAI_RX_DIV_FP	5

#define ESAI_TCCR_TPSR_BYPASS (1 << 8)
#define ESAI_TCCR_TPSR_DIV8 (0 << 8)

#define ESAI_RCCR_RPSR_BYPASS (1 << 8)
#define ESAI_RCCR_RPSR_DIV8 (0 << 8)

#endif
