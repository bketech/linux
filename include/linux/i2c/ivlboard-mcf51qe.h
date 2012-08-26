/*
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

/* These register values are poached from <linux/pmic_external.h> */
enum mcf51qe_i2c_registers {
	REG_IDENTIFICATION = 7,
	REG_RTC_TIME = 20,		/*20 */
	REG_RTC_ALARM = 21,
	REG_RTC_DAY = 22,
	REG_RTC_DAY_ALARM = 23,
};
