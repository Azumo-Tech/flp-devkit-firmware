/* This file is the part of the FLEx FLP Dev Kit Firmware
 *
 * Copyright ©2017,2018 FLEx Lighting II, LLC.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *   http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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

void EXTFLASH_read_screen_sector(EXTFLASH_HandleTypeDef *hflash, uint8_t index, uint8_t sector, void *buffer) {
    uint32_t addr = index*hflash->stride+4096*sector;
    uint8_t cmd[] =  {0x03, addr>>16, (addr>>8)&0xff, (addr)&0xff};
    HAL_GPIO_WritePin(hflash->CS_Port, hflash->CS_Pin, 0);
    HAL_SPI_Transmit(hflash->hspi, (void*)cmd, 4, 10);
    HAL_SPI_Receive(hflash->hspi, buffer, 4096, 100);
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
    uint8_t cmd[] =  {0x02, (addr>>16)&0xff, (addr>>8)&0xff, 0};
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

void EXTFLASH_write_screen_sector(EXTFLASH_HandleTypeDef *hflash, uint8_t index, uint8_t sector, void *buffer) {
    uint32_t addr = index * hflash->stride;
    addr += 4096*sector;
    /* Erase the sectors for the image */
    EXTFLASH_sector_erase(hflash, addr);
    /* And overwrite page by page */
    uint16_t bufsize = 4096;
    while(bufsize > 256) {
        EXTFLASH_write_aligned_page(hflash, addr, buffer, 256);
        buffer += 256;
        addr += 256;
        bufsize -= 256;
    }
    EXTFLASH_write_aligned_page(hflash, addr, buffer, bufsize);
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
