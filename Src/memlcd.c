#include "main.h"
#include "stm32l1xx_hal.h"
#include "memlcd.h"

#include <stdint.h>

void MEMLCD_init(MEMLCD_HandleTypeDef *hmemlcd) {
	HAL_GPIO_WritePin(hmemlcd->EXTMODE_Port, hmemlcd->EXTMODE_Pin, 1);
	if (hmemlcd->model == MEMLCD_LS027B7DH01) { /* Larger Sharp LCDs need 5v */
		HAL_GPIO_WritePin(hmemlcd->BOOST_Port, hmemlcd->BOOST_Pin, 1);
	}

	hmemlcd->hspi->Init.Mode = SPI_MODE_MASTER;
	hmemlcd->hspi->Init.Direction = SPI_DIRECTION_2LINES;
	hmemlcd->hspi->Init.DataSize = SPI_DATASIZE_8BIT;
	hmemlcd->hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
	hmemlcd->hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
	hmemlcd->hspi->Init.NSS = SPI_NSS_SOFT;
	hmemlcd->hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	hmemlcd->hspi->Init.TIMode = SPI_TIMODE_DISABLE;
	hmemlcd->hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hmemlcd->hspi->Init.CRCPolynomial = 10;

	switch (hmemlcd->model) {
	case MEMLCD_LS013B7DH05:
	case MEMLCD_LS027B7DH01:
		hmemlcd->updatecmd = 0b001;
		hmemlcd->hspi->Init.FirstBit = SPI_FIRSTBIT_LSB;
		break;
	case MEMLCD_LPM013M126A:
	case MEMLCD_LPM027M128B:
		hmemlcd->updatecmd = 0b10000000;
		hmemlcd->hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
		break;
	}

	HAL_SPI_Init(hmemlcd->hspi);

	MEMLCD_clear_all(hmemlcd);
	HAL_GPIO_WritePin(hmemlcd->DISP_Port, hmemlcd->DISP_Pin, 1);
}

void MEMLCD_power_off(MEMLCD_HandleTypeDef *hmemlcd) {
	HAL_GPIO_WritePin(hmemlcd->DISP_Port, hmemlcd->DISP_Pin, 0);
	HAL_GPIO_WritePin(hmemlcd->EXTMODE_Port, hmemlcd->EXTMODE_Pin, 0);
	HAL_GPIO_WritePin(hmemlcd->BOOST_Port, hmemlcd->BOOST_Pin, 0);
	HAL_GPIO_WritePin(hmemlcd->CS_Port, hmemlcd->CS_Pin, 0);
}

void MEMLCD_clear_all(MEMLCD_HandleTypeDef *hmemlcd){
	uint8_t cmd[2] = {0b100, 0};
	/* For some reason we need to send the command twice for it to work */
	HAL_GPIO_WritePin(hmemlcd->CS_Port, hmemlcd->CS_Pin, 1);
	HAL_SPI_Transmit(hmemlcd->hspi, cmd, 2, 10);
	HAL_GPIO_WritePin(hmemlcd->CS_Port, hmemlcd->CS_Pin, 0);
	HAL_Delay(1);
	HAL_GPIO_WritePin(hmemlcd->CS_Port, hmemlcd->CS_Pin, 1);
	HAL_SPI_Transmit(hmemlcd->hspi, cmd, 2, 10);
	HAL_GPIO_WritePin(hmemlcd->CS_Port, hmemlcd->CS_Pin, 0);
	HAL_Delay(1);
}


void MEMLCD_set_disp(MEMLCD_HandleTypeDef *hmemlcd, uint8_t state) {
	HAL_GPIO_WritePin(hmemlcd->DISP_Port, hmemlcd->DISP_Pin, state);
}

void MEMLCD_update_area(MEMLCD_HandleTypeDef *hmemlcd, uint8_t start, uint8_t end) {
	uint8_t cmd[2] = {hmemlcd->updatecmd, 0};
	uint8_t line_len = MEMLCD_line_length[hmemlcd->model];
	uint8_t line_ct = MEMLCD_line_count[hmemlcd->model];
	end = (end <= line_ct)? end : line_ct;
	start = (start <= end)? start : end;
	uint8_t *buffer = hmemlcd->buffer + (line_len * start);
	cmd[1] = start;
	HAL_GPIO_WritePin(hmemlcd->CS_Port, hmemlcd->CS_Pin, 1);
	while(cmd[1] <= end) {
		HAL_SPI_Transmit(hmemlcd->hspi, cmd, 2, 10);
		HAL_SPI_Transmit(hmemlcd->hspi, buffer, line_len, 10);
		buffer += line_len;
		cmd[1]++;
	}
	cmd[0] = cmd[1] = 0;
	HAL_SPI_Transmit(hmemlcd->hspi, cmd, 2, 10);
	HAL_GPIO_WritePin(hmemlcd->CS_Port, hmemlcd->CS_Pin, 0);
}
