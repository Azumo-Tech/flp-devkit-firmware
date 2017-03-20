#include "main.h"
#include "stm32l1xx_hal.h"
#include "memlcd.h"

#include <stdint.h>


void MEMLCD_init(MEMLCD_HandleTypeDef *hmemlcd) {
	HAL_GPIO_WritePin(hmemlcd->EXTMODE_Port, hmemlcd->EXTMODE_Pin, 1);
	if (hmemlcd->model == MEMLCD_SHARP_270) {
		HAL_GPIO_WritePin(hmemlcd->BOOST_Port, hmemlcd->BOOST_Pin, 1);
	}
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

void MEMLCD_update_area(MEMLCD_HandleTypeDef *hmemlcd,
		uint8_t *buffer, uint8_t start, uint8_t end) {
	uint8_t cmd[2] = {0b001, start}, line_len = 50;
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
