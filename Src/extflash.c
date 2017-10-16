/*******************************************************************************
 * Copyright (c) 2017 FLEx Lighting II, LLC "FLEx Lighting". 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted, provided that the following conditions are met:
 * 
 * 1. Redistribution of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of FLEx Lighting nor the names of other 
 *    contributors to this software may be used to endorse or promote products 
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this 
 *    software, must execute solely and exclusively on devices manufactured by
 *    or for FLEx Lighting.
 * 5. Redistribution and use of this software other than as permitted under 
 *    this license is void and will automatically terminate your rights under 
 *    this license. 
 * 
 * THIS SOFTWARE IS PROVIDED BY FLEX LIGHTING AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
 * SHALL FLEX LIGHTING OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
#include "main.h"
#include "stm32l1xx_hal.h"
#include "extflash.h"

#include <stdint.h>

void EXTFLASH_power_down(EXTFLASH_HandleTypeDef *hflash) {
    uint8_t cmd[] =  {0xB9};
    HAL_GPIO_WritePin(hflash->CS_Port, hflash->CS_Pin, 0);
    HAL_SPI_Transmit(hflash->hspi, (void*)cmd, 1, 10);
    HAL_GPIO_WritePin(hflash->CS_Port, hflash->CS_Pin, 1);
    for (volatile int i=0; i<200; i++);
}

void EXTFLASH_power_up(EXTFLASH_HandleTypeDef *hflash) {
    uint8_t cmd[] =  {0xAB};
    HAL_GPIO_WritePin(hflash->CS_Port, hflash->CS_Pin, 0);
    HAL_SPI_Transmit(hflash->hspi, (void*)cmd, 1, 10);
    HAL_GPIO_WritePin(hflash->CS_Port, hflash->CS_Pin, 1);
    for (volatile int i=0; i<200; i++);
}

void EXTFLASH_read_screen(EXTFLASH_HandleTypeDef *hflash, uint8_t index, void *buffer, uint16_t bufsize) {
    uint32_t addr = index*hflash->stride;
    uint8_t cmd[] =  {0x03, addr>>16, (addr>>8)&0xff, (addr)&0xff};
    HAL_GPIO_WritePin(hflash->CS_Port, hflash->CS_Pin, 0);
    HAL_SPI_Transmit(hflash->hspi, (void*)cmd, 4, 10);
    HAL_SPI_Receive(hflash->hspi, buffer, bufsize, 100);
    HAL_GPIO_WritePin(hflash->CS_Port, hflash->CS_Pin, 1);
}


void EXTFLASH_write_enable(EXTFLASH_HandleTypeDef *hflash) {
    uint8_t cmd[] =  {0x06};
    HAL_GPIO_WritePin(hflash->CS_Port, hflash->CS_Pin, 0);
    HAL_SPI_Transmit(hflash->hspi, (void*)cmd, 1, 10);
    HAL_GPIO_WritePin(hflash->CS_Port, hflash->CS_Pin, 1);
}

void EXTFLASH_wait_for_busy(EXTFLASH_HandleTypeDef *hflash) {
    uint8_t cmd = 0x05, status;
    do {
        HAL_GPIO_WritePin(hflash->CS_Port, hflash->CS_Pin, 0);
        HAL_SPI_Transmit(hflash->hspi, (void*)&cmd, 1, 10);
        HAL_SPI_Receive(hflash->hspi, (void*)&status, 1, 10);
        HAL_GPIO_WritePin(hflash->CS_Port, hflash->CS_Pin, 1);
    } while (status & 1);
}

void EXTFLASH_write_aligned_page(EXTFLASH_HandleTypeDef *hflash, uint32_t addr, void *buffer, uint16_t size) {
    EXTFLASH_write_enable(hflash);
    uint8_t cmd[] =  {0x02, addr>>16, (addr>>8)&0xff, 0};
    HAL_GPIO_WritePin(hflash->CS_Port, hflash->CS_Pin, 0);
    HAL_SPI_Transmit(hflash->hspi, (void*)cmd, 4, 10);
    HAL_SPI_Transmit(hflash->hspi, buffer, size, 100);
    HAL_GPIO_WritePin(hflash->CS_Port, hflash->CS_Pin, 1);
    EXTFLASH_wait_for_busy(hflash);
}

void EXTFLASH_sector_erase(EXTFLASH_HandleTypeDef *hflash, uint32_t addr) {
    EXTFLASH_write_enable(hflash);
    uint8_t cmd[] =  {0x20, (addr>>16)&0xff, (addr>>8)&0xff, 0};
    HAL_GPIO_WritePin(hflash->CS_Port, hflash->CS_Pin, 0);
    HAL_SPI_Transmit(hflash->hspi, (void*)cmd, 4, 10);
    HAL_GPIO_WritePin(hflash->CS_Port, hflash->CS_Pin, 1);
    EXTFLASH_wait_for_busy(hflash);
}

void EXTFLASH_write_screen(EXTFLASH_HandleTypeDef *hflash, uint8_t index, void *buffer, uint16_t bufsize) {
    uint32_t addr = index*hflash->stride;
    /* Erase the sectors for the image */
    for (uint32_t sector=addr; sector < addr+hflash->stride; sector += 4096) {
    	EXTFLASH_sector_erase(hflash, sector);
    }
    /* And overwrite page by page */
    while(bufsize > 256) {
        EXTFLASH_write_aligned_page(hflash, addr, buffer, 256);
        buffer += 256;
        addr += 256;
        bufsize -= 256;
    }
    EXTFLASH_write_aligned_page(hflash, addr, buffer, bufsize);
}
