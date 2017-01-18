#include "main.h"
#include "stm32l1xx_hal.h"
#include "memlcd.h"

#include <stdint.h>

extern SPI_HandleTypeDef hspi1;

void MEMLCD_init(MEMLCD_HandleTypeDef *hmemlcd) {
	HAL_GPIO_WritePin(hmemlcd->EXTMODE_Port, hmemlcd->EXTMODE_Pin, 1);
}

void MEMLCD_clear_all(MEMLCD_HandleTypeDef *hmemlcd){
	uint8_t msg[6] = {0b100, 0};
	/* For some reason we need to send the command twice for it to work */
	HAL_GPIO_WritePin(hmemlcd->CS_Port, hmemlcd->CS_Pin, 1);
	HAL_SPI_Transmit(&hspi1, msg, 2, 10);
	HAL_GPIO_WritePin(hmemlcd->CS_Port, hmemlcd->CS_Pin, 0);
	HAL_Delay(1);
	HAL_GPIO_WritePin(hmemlcd->CS_Port, hmemlcd->CS_Pin, 1);
	HAL_SPI_Transmit(&hspi1, msg, 2, 10);
	HAL_GPIO_WritePin(hmemlcd->CS_Port, hmemlcd->CS_Pin, 0);
	HAL_Delay(1);
}


void MEMLCD_set_disp(MEMLCD_HandleTypeDef *hmemlcd, uint8_t state) {
	HAL_GPIO_WritePin(hmemlcd->DISP_Port, hmemlcd->DISP_Pin, state);
}
