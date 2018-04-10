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

#include "bsp.h"
#include "main.h"
#include "stm32l1xx_hal.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

#include "i2c.h"
#include "adc.h"

#include "eeprom.h"
#include "flp.h"
#include "memlcd.h"
#include "extflash.h"

extern EXTFLASH_HandleTypeDef hflash;
extern MEMLCD_HandleTypeDef hmemlcd;
void SystemClock_Config(void);

static const uint32_t DFU_BOOTKEY = 0x157F32D4;
static uint32_t *bootkey_adr = (void *)(0x20013ffc); /* This is where the bootloader looks for the magic number */

void BSP_init() {
    if (EEPROM_Settings->version != FIRMWARE_VERSION) {
        HAL_FLASHEx_DATAEEPROM_Unlock();
        HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, (size_t)&EEPROM_Settings->version, FIRMWARE_VERSION);
        HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, (size_t)&EEPROM_Settings->default_delay, 150);
        HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, (size_t)&EEPROM_Settings->slide_count, 10);
        HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_HALFWORD, (size_t)&EEPROM_Settings->default_led_current, 20000);
        if (EEPROM_Settings->lcd_model > MEMLCD_max_model) {
        	HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, (size_t)&EEPROM_Settings->lcd_model, 5);
        }
        HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, (size_t)&EEPROM_Settings->flags, 0);
        for (uint8_t i=0; i<64; i++) {
            HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, (size_t)&EEPROM_Settings->slides[i].delay, 0);
            HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, (size_t)&EEPROM_Settings->slides[i].img, i);
            HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_HALFWORD, (size_t)&EEPROM_Settings->slides[i].led_current, 0);
        }
        HAL_FLASHEx_DATAEEPROM_Lock();
    }
    hmemlcd.model = EEPROM_Settings->lcd_model;
    MEMLCD_init(&hmemlcd);
    // Set the flash image stride to be able to hold the whole screen, rounded up to a whole number of sectors.
    hflash.stride = 4096 * ((MEMLCD_bufsize(&hmemlcd) + 4095 ) / 4096);
}

int BSP_sleep() {
    int ret = 0;
    FLP_off();
    MEMLCD_power_off(&hmemlcd);
    EXTFLASH_power_down(&hflash);
    USBD_Stop(&hUsbDeviceFS);
    USBD_DeInit(&hUsbDeviceFS);
    SysTick->CTRL = 0;
    HAL_I2C_DeInit(&hi2c2);
    HAL_SPI_DeInit(&hspi1);
    HAL_SPI_DeInit(&hspi2);
    HAL_SPI_DeInit(&hspi3);
    HAL_ADC_DeInit(&hadc);
    while (!HAL_GPIO_ReadPin(BT1_GPIO_Port, BT1_Pin));
    while (HAL_GPIO_ReadPin(BT1_GPIO_Port, BT1_Pin) && HAL_GPIO_ReadPin(N_PGOOD_GPIO_Port, N_PGOOD_Pin)) {
        HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    }
    if (!HAL_GPIO_ReadPin(BT1_GPIO_Port, BT1_Pin)) ret = 1;
    SystemClock_Config();
    MX_ADC_Init();
    MX_I2C2_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();
    MX_SPI3_Init();
    MX_USB_DEVICE_Init();
    EXTFLASH_power_up(&hflash);
    MEMLCD_init(&hmemlcd);
    return ret;
}

void BSP_reset_to_bootloader() {
	*bootkey_adr = DFU_BOOTKEY;
	BSP_reset();
}

void BSP_reset() {
	HAL_GPIO_WritePin(USB_DISCONNECT_GPIO_Port, USB_DISCONNECT_Pin, 1);
	USBD_Stop(&hUsbDeviceFS);
	HAL_Delay(1000);
	NVIC_SystemReset();
}
