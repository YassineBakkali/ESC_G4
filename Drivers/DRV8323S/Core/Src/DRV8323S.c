/*
 * DRV8323S.c
 *
 *  Created on: Feb 14, 2023
 *      Author: YassineBakkali
 */

#include <DRV8323S.h>

void DRV8323_write(drv8323S* drv, uint16_t val)
{
	uint16_t TX_Data = val;
	HAL_GPIO_WritePin(drv.spiCS_GPIOx, drv.spiCS_GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_IT(drv.spi, (uint8_t*)s_drv832xSPI.SPI_TX_Data, (uint8_t*)s_drv832xSPI.SPI_RX_Data, 1);
}

