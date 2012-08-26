
/*!
 * Configures the input selection mux for the specified pin in GPIO mode.
 * I was tired of getting this wrong, so the computer is going to do the work now.
 *
 * This beast of a table was parsed out of the MX35 RM.  It's in a separate file so it
 * doesn't pollute the _gpio.c file with boring data.
 */
static void set_iomux_input_gpio(iomux_pin_name_t pin)
{
	iomux_input_select_t input_select;
	u32 config;

	switch (pin) {
	case MX35_PIN_GPIO1_0: input_select = MUX_IN_GPIO1_IN_0; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_STXD5: input_select = MUX_IN_GPIO1_IN_0; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_D3_DRDY: input_select = MUX_IN_GPIO1_IN_0; config = INPUT_CTL_PATH2; break;
	case MX35_PIN_TX5_RX0: input_select = MUX_IN_GPIO1_IN_10; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_SD1_DATA2: input_select = MUX_IN_GPIO1_IN_10; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_TX4_RX1: input_select = MUX_IN_GPIO1_IN_11; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_SD1_DATA3: input_select = MUX_IN_GPIO1_IN_11; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_GPIO1_1: input_select = MUX_IN_GPIO1_IN_1; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_SRXD5: input_select = MUX_IN_GPIO1_IN_1; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_CONTRAST: input_select = MUX_IN_GPIO1_IN_1; config = INPUT_CTL_PATH2; break;
	case MX35_PIN_CS4: input_select = MUX_IN_GPIO1_IN_20; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_CSI_D8: input_select = MUX_IN_GPIO1_IN_20; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_CS5: input_select = MUX_IN_GPIO1_IN_21; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_CSI_D9: input_select = MUX_IN_GPIO1_IN_21; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_CSI_D10: input_select = MUX_IN_GPIO1_IN_22; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_SCK5: input_select = MUX_IN_GPIO1_IN_2; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_D3_VSYNC: input_select = MUX_IN_GPIO1_IN_2; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_STXFS5: input_select = MUX_IN_GPIO1_IN_3; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_D3_REV: input_select = MUX_IN_GPIO1_IN_3; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_CAPTURE: input_select = MUX_IN_GPIO1_IN_4; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_SCKR: input_select = MUX_IN_GPIO1_IN_4; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_D3_CLS: input_select = MUX_IN_GPIO1_IN_4; config = INPUT_CTL_PATH2; break;
	case MX35_PIN_COMPARE: input_select = MUX_IN_GPIO1_IN_5; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_FSR: input_select = MUX_IN_GPIO1_IN_5; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_D3_SPL: input_select = MUX_IN_GPIO1_IN_5; config = INPUT_CTL_PATH2; break;
	case MX35_PIN_WATCHDOG_RST: input_select = MUX_IN_GPIO1_IN_6; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_HCKR: input_select = MUX_IN_GPIO1_IN_6; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_SD1_CMD: input_select = MUX_IN_GPIO1_IN_6; config = INPUT_CTL_PATH2; break;
	case MX35_PIN_VSTBY: input_select = MUX_IN_GPIO1_IN_7; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_SCKT: input_select = MUX_IN_GPIO1_IN_7; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_SD1_CLK: input_select = MUX_IN_GPIO1_IN_7; config = INPUT_CTL_PATH2; break;
	case MX35_PIN_CLKO: input_select = MUX_IN_GPIO1_IN_8; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_FST: input_select = MUX_IN_GPIO1_IN_8; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_SD1_DATA0: input_select = MUX_IN_GPIO1_IN_8; config = INPUT_CTL_PATH2; break;
	case MX35_PIN_HCKT: input_select = MUX_IN_GPIO1_IN_9; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_SD1_DATA1: input_select = MUX_IN_GPIO1_IN_9; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_GPIO2_0: input_select = MUX_IN_GPIO2_IN_0; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_LD0: input_select = MUX_IN_GPIO2_IN_0; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_SD2_CMD: input_select = MUX_IN_GPIO2_IN_0; config = INPUT_CTL_PATH2; break;
	case MX35_PIN_LD10: input_select = MUX_IN_GPIO2_IN_10; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_ATA_DMACK: input_select = MUX_IN_GPIO2_IN_10; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_LD11: input_select = MUX_IN_GPIO2_IN_11; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_ATA_RESET_B: input_select = MUX_IN_GPIO2_IN_11; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_LD12: input_select = MUX_IN_GPIO2_IN_12; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_ATA_IORDY: input_select = MUX_IN_GPIO2_IN_12; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_LD13: input_select = MUX_IN_GPIO2_IN_13; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_ATA_DATA0: input_select = MUX_IN_GPIO2_IN_13; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_LD14: input_select = MUX_IN_GPIO2_IN_14; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_ATA_DATA1: input_select = MUX_IN_GPIO2_IN_14; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_LD15: input_select = MUX_IN_GPIO2_IN_15; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_ATA_DATA2: input_select = MUX_IN_GPIO2_IN_15; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_LD16: input_select = MUX_IN_GPIO2_IN_16; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_ATA_DATA3: input_select = MUX_IN_GPIO2_IN_16; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_LD17: input_select = MUX_IN_GPIO2_IN_17; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_ATA_DATA4: input_select = MUX_IN_GPIO2_IN_17; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_NFWE_B: input_select = MUX_IN_GPIO2_IN_18; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_ATA_DATA5: input_select = MUX_IN_GPIO2_IN_18; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_NFRE_B: input_select = MUX_IN_GPIO2_IN_19; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_ATA_DATA6: input_select = MUX_IN_GPIO2_IN_19; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_LD1: input_select = MUX_IN_GPIO2_IN_1; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_SD2_CLK: input_select = MUX_IN_GPIO2_IN_1; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_NFALE: input_select = MUX_IN_GPIO2_IN_20; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_ATA_DATA7: input_select = MUX_IN_GPIO2_IN_20; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_NFCLE: input_select = MUX_IN_GPIO2_IN_21; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_ATA_DATA8: input_select = MUX_IN_GPIO2_IN_21; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_NFWP_B: input_select = MUX_IN_GPIO2_IN_22; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_ATA_DATA9: input_select = MUX_IN_GPIO2_IN_22; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_NFRB: input_select = MUX_IN_GPIO2_IN_23; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_ATA_DATA10: input_select = MUX_IN_GPIO2_IN_23; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_I2C1_CLK: input_select = MUX_IN_GPIO2_IN_24; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_ATA_DATA11: input_select = MUX_IN_GPIO2_IN_24; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_I2C1_DAT: input_select = MUX_IN_GPIO2_IN_25; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_ATA_DATA12: input_select = MUX_IN_GPIO2_IN_25; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_I2C2_CLK: input_select = MUX_IN_GPIO2_IN_26; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_ATA_DATA13: input_select = MUX_IN_GPIO2_IN_26; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_I2C2_DAT: input_select = MUX_IN_GPIO2_IN_27; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_ATA_DATA14: input_select = MUX_IN_GPIO2_IN_27; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_STXD4: input_select = MUX_IN_GPIO2_IN_28; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_ATA_DATA15: input_select = MUX_IN_GPIO2_IN_28; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_SRXD4: input_select = MUX_IN_GPIO2_IN_29; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_ATA_INTRQ: input_select = MUX_IN_GPIO2_IN_29; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_LD2: input_select = MUX_IN_GPIO2_IN_2; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_SD2_DATA0: input_select = MUX_IN_GPIO2_IN_2; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_SCK4: input_select = MUX_IN_GPIO2_IN_30; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_ATA_BUFF_EN: input_select = MUX_IN_GPIO2_IN_30; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_STXFS4: input_select = MUX_IN_GPIO2_IN_31; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_ATA_DMARQ: input_select = MUX_IN_GPIO2_IN_31; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_LD3: input_select = MUX_IN_GPIO2_IN_3; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_SD2_DATA1: input_select = MUX_IN_GPIO2_IN_3; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_LD4: input_select = MUX_IN_GPIO2_IN_4; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_SD2_DATA2: input_select = MUX_IN_GPIO2_IN_4; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_LD5: input_select = MUX_IN_GPIO2_IN_5; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_SD2_DATA3: input_select = MUX_IN_GPIO2_IN_5; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_LD6: input_select = MUX_IN_GPIO2_IN_6; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_ATA_CS0: input_select = MUX_IN_GPIO2_IN_6; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_LD7: input_select = MUX_IN_GPIO2_IN_7; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_ATA_CS1: input_select = MUX_IN_GPIO2_IN_7; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_LD8: input_select = MUX_IN_GPIO2_IN_8; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_ATA_DIOR: input_select = MUX_IN_GPIO2_IN_8; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_LD9: input_select = MUX_IN_GPIO2_IN_9; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_ATA_DIOW: input_select = MUX_IN_GPIO2_IN_9; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_GPIO3_0: input_select = MUX_IN_GPIO3_IN_0; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_ATA_DA0: input_select = MUX_IN_GPIO3_IN_0; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_RXD2: input_select = MUX_IN_GPIO3_IN_10; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_FEC_RDATA0: input_select = MUX_IN_GPIO3_IN_10; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_TXD2: input_select = MUX_IN_GPIO3_IN_11; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_FEC_TDATA0: input_select = MUX_IN_GPIO3_IN_11; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_RTS2: input_select = MUX_IN_GPIO3_IN_12; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_FEC_TX_EN: input_select = MUX_IN_GPIO3_IN_12; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_CTS2: input_select = MUX_IN_GPIO3_IN_13; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_FEC_MDC: input_select = MUX_IN_GPIO3_IN_13; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_USBOTG_PWR: input_select = MUX_IN_GPIO3_IN_14; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_FEC_MDIO: input_select = MUX_IN_GPIO3_IN_14; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_USBOTG_OC: input_select = MUX_IN_GPIO3_IN_15; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_FEC_TX_ERR: input_select = MUX_IN_GPIO3_IN_15; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_CSPI1_SCLK: input_select = MUX_IN_GPIO3_IN_4; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_MLB_DAT: input_select = MUX_IN_GPIO3_IN_4; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_CSPI1_SPI_RDY: input_select = MUX_IN_GPIO3_IN_5; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_MLB_SIG: input_select = MUX_IN_GPIO3_IN_5; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_RXD1: input_select = MUX_IN_GPIO3_IN_6; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_FEC_TX_CLK: input_select = MUX_IN_GPIO3_IN_6; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_TXD1: input_select = MUX_IN_GPIO3_IN_7; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_FEC_RX_CLK: input_select = MUX_IN_GPIO3_IN_7; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_RTS1: input_select = MUX_IN_GPIO3_IN_8; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_FEC_RX_DV: input_select = MUX_IN_GPIO3_IN_8; config = INPUT_CTL_PATH1; break;
	case MX35_PIN_CTS1: input_select = MUX_IN_GPIO3_IN_9; config = INPUT_CTL_PATH0; break;
	case MX35_PIN_FEC_COL: input_select = MUX_IN_GPIO3_IN_9; config = INPUT_CTL_PATH1; break;
	default:
		pr_err("Not sure about input_select for pin with iomux offset=0x%X\n", PIN_TO_IOMUX_MUX(pin));
		return;
	}

	mxc_iomux_set_input(input_select, config);
}
