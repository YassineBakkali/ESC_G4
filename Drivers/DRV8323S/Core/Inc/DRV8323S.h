/*
 * DRV8323S.h
 *
 *  Created on: Feb 14, 2023
 *      Author: YassineBakkali
 */

#ifndef DRV8323S_CORE_INC_DRV8323S_H_
#define DRV8323S_CORE_INC_DRV8323S_H_


#ifndef DRV8323_RS_DRV8323_H_
#define DRV8323_RS_DRV8323_H_



/// Registers addresses ///
#define FSR1 			0x00  // Fault status register 1
#define FSR2 			0x01  // Fault status register 1
#define DCR 			0x02  // Driver Control Register
#define GDHS 			0x03  // Gate Drive HS Register
#define GDLS			0x04  // Gate Drive LS Register
#define OCP 			0x05  // OCP Control Register
#define CSA			    0x06  // CSA Control Register

/// FSR1 register fields ///
#define VDS_LC			0x00  // VDS overcurrent on C low-side MOSFET
#define VDS_HC			0x01  // VDS overcurrent on C high-side MOSFET
#define VDS_LB			0x02  // VDS overcurrent on B low-side MOSFET
#define VDS_HB			0x03  // VDS overcurrent on B high-side MOSFET
#define VDS_LA			0x04  // VDS overcurrent on A low-side MOSFET
#define VDS_HA			0x05  // VDS overcurrent on A high-side MOSFET
#define OTSD			0x06  // Overtemperature shutdown
#define UVLO			0x07  // Undervoltage lockout fault condition
#define GDF				0x08  // Gate drive fault condition
#define VDS_OCP			0x09  // VDS monitor overcurrent fault condition
#define FAULT			0x0a  // Logic OR of FAULT status registers. Mirrors nFAULT pin.

/// FSR2 register fields ///
#define VGS_LC			0x00  // Gate drive fault on the C low-side of MOSFET
#define VGS_HC			0x01  // Gate drive fault on the C high-side of MOSFET
#define VGS_LB			0x02  // Gate drive fault on the B low-side of MOSFET
#define VGS_HB			0x03  // Gate drive fault on the B high-side of MOSFET
#define VGS_LA			0x04  // Gate drive fault on the A low-side of MOSFET
#define VGS_HA			0x05  // Gate drive fault on the A high-side of MOSFET
#define CPUV			0x06  // Charge pump undervoltage fault condition
#define OTW 			0x07  // Overtemperature warning
#define SC_OC			0x08  // Overcurrent on Phase C sense amplifier
#define SB_OC			0x09  // Overcurrent on Phase B sense amplifier
#define SA_OC			0x0a  // Overcurrent on Phase A sense amplifier

/// DCR register fields ///
#define CLR_FLT_REG		0x00  // CLR_FLT reg position.
#define CLR_FLT_ON		0x01  // Write 1 to clear latched fault bits. Auto reset to 0 after
#define BRAKE_REG		0x01  // This bit is ORed with INLC input. Controls low-side braking
#define BRAKE_ON		0x01  // Activates low-side braking.
#define BRAKE_OFF		0x00  // Turns off lowswide braking
#define COAST_REG		0x02  // Coasting command register.
#define COAST_ON		0x01  // Turns all MOS to Hi-Z.
#define COAST_OFF		0x00  // Reverts effect of COASTING.
#define PWM1_DIR_REG	0x03  // Controls direction of turning (reverts pwm). Pin is ORed with INHC.
#define PWM1_DIR_OFF	0x00  //
#define PWM1_DIR_ON		0x01  //
#define PWM1_COM_REG	0x04  // Choose 1xPWM type of rectification.
#define PWM1_COM_SYNC	0x00  // 1xPWM mode uses synchronous rectification
#define PWM1_COM_ASYNC	0x01  // 1xPWM mode uses asynchronous rectification (diode freewheeling)
#define PWM_MODE_REG	0x05  // Register position of bits controlling PWM mode.
#define PWM_MODE_6		0x00  // 6x PWM mode
#define PWM_MODE_3		0x01  // 3x PWM mode
#define PWM_MODE_1		0x02  // 1x PWM mode
#define PWM_MODE_IND	0x03  // independant PWM mode
#define OTW_REP_REG		0x06  // This bit decides if OTW is reported on nFAULT or Fault bit or not
#define OTW_REP_OFF		0x00
#define OTW_REP_ON		0x01
#define DIS_GDF_REG		0x07  // This bit decides if Gate drive fault is enabled or not.
#define DIS_GDF_ON		0x00
#define DIS_GDF_OFF		0x01
#define DIS_CPUV_REG	0x08  // This bit enables pump UVLO fault or not.
#define DIS_CPUV_ON 	0x00
#define DIS_CPUV_OFF	0x01

/// GDHS register fields ///
#define IDRIVEN_HS_MSK	0x03  // Shifting mask for peak sink current of high-side MOSFETs from DRV8323
#define IDRIVEN_HS_20	0x00  // Peak sink current In_HS = 20mA
#define IDRIVEN_HS_60	0x01  // Peak sink current In_HS = 60mA
#define IDRIVEN_HS_120	0x02  // Peak sink current In_HS = 120mA
#define IDRIVEN_HS_160	0x03  // Peak sink current In_HS = 160mA
#define IDRIVEN_HS_240	0x04  // Peak sink current In_HS = 240mA
#define IDRIVEN_HS_280	0x05  // Peak sink current In_HS = 280mA
#define IDRIVEN_HS_340	0x06  // Peak sink current In_HS = 340mA
#define IDRIVEN_HS_380	0x07  // Peak sink current In_HS = 380mA
#define IDRIVEN_HS_520	0x08  // Peak sink current In_HS = 520mA
#define IDRIVEN_HS_660	0x09  // Peak sink current In_HS = 660mA
#define IDRIVEN_HS_740	0x0a  // Peak sink current In_HS = 740mA
#define IDRIVEN_HS_880	0x0b  // Peak sink current In_HS = 880mA
#define IDRIVEN_HS_1140	0x0c  // Peak sink current In_HS = 1140mA
#define IDRIVEN_HS_1360	0x0d  // Peak sink current In_HS = 1360mA
#define IDRIVEN_HS_1640	0x0e  // Peak sink current In_HS = 1640mA
#define IDRIVEN_HS_2000	0x0f  // Peak sink current In_HS = 2000mA

// ---------------------------------------------------------------//

#define IDRIVEP_HS_MSK	0x07  // Shifting mask for peak source current of high-side MOSFETs from DRV8323
#define IDRIVEP_HS_10	0x00  // Peak source current Ip_HS = 10mA
#define IDRIVEP_HS_30	0x01  // Peak source current Ip_HS = 30mA
#define IDRIVEP_HS_60	0x02  // Peak source current Ip_HS = 60mA
#define IDRIVEP_HS_80	0x03  // Peak source current Ip_HS = 80mA
#define IDRIVEP_HS_120	0x04  // Peak source current Ip_HS = 120mA
#define IDRIVEP_HS_140	0x05  // Peak source current Ip_HS = 140mA
#define IDRIVEP_HS_170	0x06  // Peak source current Ip_HS = 170mA
#define IDRIVEP_HS_190	0x07  // Peak source current Ip_HS = 190mA
#define IDRIVEP_HS_260	0x08  // Peak source current Ip_HS = 260mA
#define IDRIVEP_HS_330	0x09  // Peak source current Ip_HS = 330mA
#define IDRIVEP_HS_370	0x0a  // Peak source current Ip_HS = 370mA
#define IDRIVEP_HS_440	0x0b  // Peak source current Ip_HS = 440mA
#define IDRIVEP_HS_570	0x0c  // Peak source current Ip_HS = 570mA
#define IDRIVEP_HS_680	0x0d  // Peak source current Ip_HS = 680mA
#define IDRIVEP_HS_820	0x0e  // Peak source current Ip_HS = 820mA
#define IDRIVEP_HS_1000	0x0f  // Peak source current Ip_HS = 1000mA

// ---------------------------------------------------------------//

#define LOCK_MSK		0x0a  // Shifting mask for settings locking options
#define SETTINGS_LOCK	0x06  // Locks all settings except for these bits and address 0x02 bits 0-2 (coast, brake, clr_flt)
#define SETTINGS_UNLOCK	0x03  // Unlocks all settings


/// GDLS register fields ///
#define IDRIVEN_LS_MSK	0x03  // Shifting mask for peak sink current of low-side MOSFETs from DRV8323
#define IDRIVEN_LS_20	0x00  // Peak sink current In_LS = 20mA
#define IDRIVEN_LS_60	0x01  // Peak sink current In_LS = 60mA
#define IDRIVEN_LS_120	0x02  // Peak sink current In_LS = 120mA
#define IDRIVEN_LS_160	0x03  // Peak sink current In_LS = 160mA
#define IDRIVEN_LS_240	0x04  // Peak sink current In_LS = 240mA
#define IDRIVEN_LS_280	0x05  // Peak sink current In_LS = 280mA
#define IDRIVEN_LS_340	0x06  // Peak sink current In_LS = 340mA
#define IDRIVEN_LS_380	0x07  // Peak sink current In_LS = 380mA
#define IDRIVEN_LS_520	0x08  // Peak sink current In_LS = 520mA
#define IDRIVEN_LS_660	0x09  // Peak sink current In_LS = 660mA
#define IDRIVEN_LS_740	0x0a  // Peak sink current In_LS = 740mA
#define IDRIVEN_LS_880	0x0b  // Peak sink current In_LS = 880mA
#define IDRIVEN_LS_1140	0x0c  // Peak sink current In_LS = 1140mA
#define IDRIVEN_LS_1360	0x0d  // Peak sink current In_LS = 1360mA
#define IDRIVEN_LS_1640	0x0e  // Peak sink current In_LS = 1640mA
#define IDRIVEN_LS_2000	0x0f  // Peak sink current In_LS = 2000mA

// ---------------------------------------------------------------//

#define IDRIVEP_LS_MSK	0x07  // Shifting mask for peak source current of low-side MOSFETs from DRV8323
#define IDRIVEP_LS_10	0x00  // Peak source current Ip_LS = 10mA
#define IDRIVEP_LS_30	0x01  // Peak source current Ip_LS = 30mA
#define IDRIVEP_LS_60	0x02  // Peak source current Ip_LS = 60mA
#define IDRIVEP_LS_80	0x03  // Peak source current Ip_LS = 80mA
#define IDRIVEP_LS_120	0x04  // Peak source current Ip_LS = 120mA
#define IDRIVEP_LS_140	0x05  // Peak source current Ip_LS = 140mA
#define IDRIVEP_LS_170	0x06  // Peak source current Ip_LS = 170mA
#define IDRIVEP_LS_190	0x07  // Peak source current Ip_LS = 190mA
#define IDRIVEP_LS_260	0x08  // Peak source current Ip_LS = 260mA
#define IDRIVEP_LS_330	0x09  // Peak source current Ip_LS = 330mA
#define IDRIVEP_LS_370	0x0a  // Peak source current Ip_LS = 370mA
#define IDRIVEP_LS_440	0x0b  // Peak source current Ip_LS = 440mA
#define IDRIVEP_LS_570	0x0c  // Peak source current Ip_LS = 570mA
#define IDRIVEP_LS_680	0x0d  // Peak source current Ip_LS = 680mA
#define IDRIVEP_LS_820	0x0e  // Peak source current Ip_LS = 820mA
#define IDRIVEP_LS_1000	0x0f  // Peak source current Ip_LS = 1000mA

// ---------------------------------------------------------------//

#define TDRIVE_REG		0x09  // This register controls the peak gate-current drive time
#define TDRIVE_500ns	0x00  // 500ns of peak gate-current drive time
#define TDRIVE_1000ns	0x01  // 1000ns of peak gate-current drive time
#define TDRIVE_2000ns	0x02  // 2000ns of peak gate-current drive time
#define TDRIVE_4000ns	0x03  // 4000ns of peak gate-current drive time

// ---------------------------------------------------------------//

#define CBC_REG			0x0a  // Cycle-by cycle operation register.
#define CBC_ON			0x01  // Cycle-by cycle operation register.
#define CBC_OFF			0x00  // Cycle-by cycle operation register.


/// OCP register fields ///
#define VDS_LVL_MSK		0x03  // This controls the max authorized VDS sensed.
#define VDS_LVL_006		0x00  // Vds_max = 0.06V
#define VDS_LVL_013		0x01  // Vds_max = 0.13V
#define VDS_LVL_020		0x02  // Vds_max = 0.20V
#define VDS_LVL_026		0x03  // Vds_max = 0.26V
#define VDS_LVL_031		0x04  // Vds_max = 0.31V
#define VDS_LVL_045		0x05  // Vds_max = 0.45V
#define VDS_LVL_053		0x06  // Vds_max = 0.53V
#define VDS_LVL_060		0x07  // Vds_max = 0.60V
#define VDS_LVL_068		0x08  // Vds_max = 0.68V
#define VDS_LVL_075		0x09  // Vds_max = 0.75V
#define VDS_LVL_094		0x0a  // Vds_max = 0.94V
#define VDS_LVL_113		0x0b  // Vds_max = 1.13V
#define VDS_LVL_130		0x0c  // Vds_max = 1.30V
#define VDS_LVL_150		0x0d  // Vds_max = 1.50V
#define VDS_LVL_170		0x0e  // Vds_max = 1.70V
#define VDS_LVL_188		0x0f  // Vds_max = 1.88V

// ---------------------------------------------------------------//

#define OCP_DEG_MSK		0x05  // This controls the overcurrent deglitch time
#define OCP_DEG_2us		0x00  // Overcurrent deglitch time of 2us
#define OCP_DEG_4us		0x01  // Overcurrent deglitch time of 4us
#define OCP_DEG_6us		0x02  // Overcurrent deglitch time of 6us
#define OCP_DEG_8us		0x03  // Overcurrent deglitch time of 8us

// ---------------------------------------------------------------//

#define OCP_MODE_MSK	0x07  // This controls what action will cause un Overcurrent on the DRV8323.
#define OCP_MODE_1		0x00  // Overcurrent causes a latched fault.
#define OCP_MODE_2		0x01  // Overcurrent causes an automatic retrying fault.
#define OCP_MODE_3		0x02  // Overcurrent is report only but not action is taken.
#define OCP_MODE_4		0x03  // Overcurrent is not reported and no action is taken.

// ---------------------------------------------------------------//

#define DEAD_TIME_MSK	0x09  // This controls the duration of the dead time
#define DEAD_TIME_50ns	0x00  // Dead time = 50ns
#define DEAD_TIME_100ns	0x01  // Dead time = 100ns
#define DEAD_TIME_200ns	0x02  // Dead time = 200ns
#define DEAD_TIME_400ns	0x03  // Dead time = 400ns

// ---------------------------------------------------------------//

#define TRETRY_MSK 		0x0a  // This controls VDS_OCP and SEN_OCP retry time
#define TRETRY_4ms 		0x00  // Retry time = 4 ms
#define TRETRY_50us 	0x01  // Retry time = 50 us


/// CSA register fields ///

#define SEN_LVL_MSK		0x01  // This controls the Sense OCP voltage level.
#define SEN_LVL_025		0x00  // Sense OCP = 0.25V
#define SEN_LVL_050		0x01  // Sense OCP = 0.50V
#define SEN_LVL_075		0x02  // Sense OCP = 0.75V
#define SEN_LVL_100		0x03  // Sense OCP = 1.00V

// ---------------------------------------------------------------//

#define CSA_CAL_C_MSK	0x02  // Controls the current sense amplifier C's operation.
#define CSA_CAL_C_NORM	0x00  // Normal operation
#define CSA_CAL_C_CAL	0x01  // Short inputs for offset calibration
#define CSA_CAL_B_MSK	0x03  // Controls the current sense amplifier B's operation.
#define CSA_CAL_B_NORM	0x00  // Normal operation
#define CSA_CAL_B_CAL	0x01  // Short inputs for offset calibration
#define CSA_CAL_A_MSK	0x04  // Controls the current sense amplifier A's operation.
#define CSA_CAL_A_NORM	0x00  // Normal operation
#define CSA_CAL_A_CAL	0x01  // Short inputs for offset calibration

// ---------------------------------------------------------------//

#define DIS_SEN_MSK		0x05  // Decides if sense overcurrent fault is enabled.
#define DIS_SEN_ON		0x00  //
#define DIS_SEN_OFF		0x01  //

#define CSA_GAIN_MSK	0x07  // Allows to choose between different current sense amplifier gains
#define CSA_GAIN_5		0x00  // Gain = 5-V/V
#define CSA_GAIN_10		0x01  // Gain = 10-V/V
#define CSA_GAIN_20		0x02  // Gain = 20-V/V
#define CSA_GAIN_40		0x03  // Gain = 40-V/V

// ---------------------------------------------------------------//

#define LS_REF_MSK		0x08  // Controls where VDS_OCP for low-side MOSFETS is measured
#define DIS_SEN_1		0x00  // Measured from SHx to SPx
#define DIS_SEN_2		0x01  // Measured from SHx to SNx

#define VREF_DIV_MSK	0x09  // Controls what is the reference voltage VREF
#define VREF_DIV_1		0x00  // Ref is VREF (unidirectional mode)
#define VREF_DIV_2		0x01  // Ref is VREF/2

#define CSA_FET			0x0a  // Controls what is the current sense amplifier's positive input
#define CSA_FET_1		0x00  // Positive input is SPx
#define CSA_FET_2		0x01  // Positive input is SHx (also sets automatically LS_REF bit to 1)

// ---------------------------------------------------------------//


typedef struct {
	GPIO_TypeDef Enable_GPIOx;      /// ENABLE GPIO port
	uint16_t Enable_GPIO_Pin; 		///	ENABLE GPIO pin
	GPIO_TypeDef spiCS_GPIOx;       /// Chip select GPIO port
	uint16_t spiCS_GPIO_Pin; 		/// Chip select GPIO pin
	SPI_HandleTypeDef spi;        	/// SPI instance
} drv8323S;

void DRV8323_enable();
void DRV8323_disable();
void DRV8323_write(uint16_t val);


#endif /* DRV8323S_CORE_INC_DRV8323S_H_ */
