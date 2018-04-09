
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
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
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l1xx_hal.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "usbd_cdc_if.h"
#include "memlcd.h"
#include "extflash.h"
#include "segfont.h"
#include "font8x16.xbm"
#include "font8x16_transp.xbm"
#include "command.h"
#include "eeprom.h"
#include "bsp.h"
#include "flp.h"
#include "buttons.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

static uint8_t tilemap[1408];

#define printxy(X,Y, ...) sprintf((char *)&tilemap[hmemlcd.tilemaps[0].width*(Y)+(X)], __VA_ARGS__)
#define clrscr() memset(tilemap, 0, sizeof(tilemap))


MEMLCD_HandleTypeDef hmemlcd = {
        .hspi = &hspi3,
        .htim = &htim3,
        .tim_ch = TIM_CHANNEL_2,
        .CS_Port = LCD_CS_GPIO_Port,
        .CS_Pin = LCD_CS_Pin,
        .DISP_Port = LCD_DISP_GPIO_Port,
        .DISP_Pin = LCD_DISP_Pin,
        .EXTMODE_Port = LCD_EXTMODE_GPIO_Port,
        .EXTMODE_Pin = LCD_EXTMODE_Pin,
        .EXTCOM_Port = LCD_EXTCOM_GPIO_Port,
        .EXTCOM_Pin = LCD_EXTCOM_Pin,
        .BOOST_Port = EN_BOOST_GPIO_Port,
        .BOOST_Pin = EN_BOOST_Pin,
};

EXTFLASH_HandleTypeDef hflash = {
        .hspi = &hspi2,
        .CS_Port = MEM_CS_GPIO_Port,
        .CS_Pin = MEM_CS_Pin,
        .size = 2*1024*1024,
};

volatile uint8_t dirty, cur_idx, running;
uint16_t runticks, vbat_avg;
uint8_t b_ticks;
uint8_t led_message[] = {
        201,205,205,205,205,205,205,205,205,205,205,205,205,205,205,187,
        186,' ','1',' ','L','E','D',' ','2','0','.','0','m','A',' ',186,
        200,205,205,205,205,205,205,205,205,205,205,205,205,205,205,188,
};

static enum BoardState {
    STATE_OFF,
    STATE_SPLASH_INIT,
    STATE_SPLASH,
    STATE_LOW_BATT_INIT,
    STATE_LOW_BATT,
    STATE_CHARGING_INIT,
    STATE_CHARGING,
    STATE_SLIDESHOW_INIT,
    STATE_SLIDESHOW_LOAD,
    STATE_SLIDESHOW_WAIT,
} State;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint16_t BATTERY_read_voltage() {
    HAL_GPIO_WritePin(VBAT_MEASURE_GPIO_Port, VBAT_MEASURE_Pin, 1);
    HAL_Delay(5);
    ADC_ChannelConfTypeDef adcChannel;
    adcChannel.Channel = ADC_CHANNEL_VREFINT;
    adcChannel.Rank = 1;
    adcChannel.SamplingTime = ADC_SAMPLETIME_192CYCLES;
    HAL_ADC_ConfigChannel(&hadc, &adcChannel);
    HAL_ADC_Start(&hadc);
    HAL_ADC_PollForConversion(&hadc, 1000);
    uint16_t vdda_raw_adc = HAL_ADC_GetValue(&hadc);
    /* uint16_t vdda = (3000L*(*(uint16_t*)0x1FF800F8)) / vdda_raw_adc; */

    adcChannel.Channel = ADC_CHANNEL_13;
    adcChannel.Rank = 1;
    adcChannel.SamplingTime = ADC_SAMPLETIME_192CYCLES;
    HAL_ADC_ConfigChannel(&hadc, &adcChannel);
    HAL_ADC_Start(&hadc);
    HAL_ADC_PollForConversion(&hadc, 1000);
    uint16_t vbat_raw_adc = HAL_ADC_GetValue(&hadc);
    uint16_t vbat = (3000LL * (*(uint16_t*)0x1FF800F8) * vbat_raw_adc) / (2048L * vdda_raw_adc);
    HAL_GPIO_WritePin(VBAT_MEASURE_GPIO_Port, VBAT_MEASURE_Pin, 0);

    HAL_ADC_Stop(&hadc);

    return vbat;
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
    // Set up the default tile maps, common to both orientations
    // Background text
    hmemlcd.tilemaps[0].scroll_x = 0;
    hmemlcd.tilemaps[0].scroll_y = 0;
    // Overlay (for LED current)
    hmemlcd.tilemaps[1].height = 3;
    hmemlcd.tilemaps[1].width = 16;
    // Set up attributes that depend on orientation
    if (hmemlcd.flags & MEMLCD_ROT270) {
        // Background text
        hmemlcd.tilemaps[0].height = hmemlcd.width/16;
        hmemlcd.tilemaps[0].width = hmemlcd.height/8;
        hmemlcd.tilemaps[0].tile_size = 1 | 0<<2;
        hmemlcd.tilemaps[0].tiles = font8x16_transp_bits;
        hmemlcd.tilemaps[0].flags = TILE_TRANSPOSE;
        // Overlay (for LED current)
        hmemlcd.tilemaps[1].tile_size = 1 | 0<<2;
        hmemlcd.tilemaps[1].scroll_x = (hmemlcd.width-48)/2;
        hmemlcd.tilemaps[1].scroll_y = (hmemlcd.height-128)/2;
        hmemlcd.tilemaps[1].tiles = font8x16_transp_bits;
        hmemlcd.tilemaps[1].flags = TILE_TRANSPOSE;
    } else {
        // Background text
        hmemlcd.tilemaps[0].height = hmemlcd.height/16;
        hmemlcd.tilemaps[0].width = hmemlcd.width/8;
        hmemlcd.tilemaps[0].tile_size = 0 | 1<<2;
        hmemlcd.tilemaps[0].tiles = font8x16_bits;
        hmemlcd.tilemaps[0].flags = 0;
        // Overlay (for LED current)
        hmemlcd.tilemaps[1].tile_size = 0 | 1<<2;
        hmemlcd.tilemaps[1].scroll_x = (hmemlcd.width-128)/2;
        hmemlcd.tilemaps[1].scroll_y = (hmemlcd.height-48)/2;
        hmemlcd.tilemaps[1].tiles = font8x16_bits;
        hmemlcd.tilemaps[1].flags = 0;
    }
    hmemlcd.tilemaps[0].map = tilemap;
    hmemlcd.tilemaps[1].map = NULL;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    uint8_t ledmsg_tim=0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */
  BSP_init();
  State = STATE_OFF;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint32_t looptime = HAL_GetTick();
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
      if (dirty && !MEMLCD_busy()){
          MEMLCD_update_area(&hmemlcd, 0, -1);
          dirty = 0;
      }
      BUTTON_poll();

      if (BUTTON_is_released(2, 40)) {
    	  FLP_toggle();
      } else if(BUTTON_held_time(2) > 40 && FLP_is_on()) {
    	  b_ticks++;
    	  if (b_ticks >= 4) {
    		  FLP_change_brightness(1);
    		  b_ticks = 0;
    	  }
    	  ledmsg_tim = 50;
      }
      if(ledmsg_tim) ledmsg_tim--;

      if (BUTTON_is_released(3, 50)) {
    	  running = 0;
    	  runticks = 0;
      } else if (BUTTON_held_time(3) == 100) {
    	  running = 1;
    	  runticks = 0;
      }

      switch (State) {
      case STATE_OFF:
          while (MEMLCD_busy());
          MEMLCD_clear_all(&hmemlcd);
          if (BSP_sleep() == 1) { // Woke up with button
              State = STATE_SPLASH_INIT;
          } else { // Woke up with USB
              State = STATE_CHARGING_INIT;
          }
          break;

      case STATE_CHARGING_INIT:
          if (!MEMLCD_busy()) {
              clrscr();
              printxy(1, 1, "USB CONNECTED");
              printxy(1, 2, "CHARGING...");
              printxy(1, 3, "PRESS POWER");
              printxy(1, 4, "TO START");
              dirty = 1;
              State = STATE_CHARGING;
              runticks = 0;
          }
          break;
      case STATE_CHARGING:
          if (runticks < 50) {
              runticks++;
              break;
          }
          if (BUTTON_held_time(1)) State = STATE_SPLASH_INIT;
          if(!HAL_GPIO_ReadPin(N_PGOOD_GPIO_Port, N_PGOOD_Pin) && HAL_GPIO_ReadPin(N_CHARGING_GPIO_Port, N_CHARGING_Pin)) {
              State = STATE_SPLASH_INIT; // If charging is done
          }
          if (HAL_GPIO_ReadPin(N_PGOOD_GPIO_Port, N_PGOOD_Pin)) {
              State = STATE_OFF; // If disconnected
          }
          break;
      case STATE_SPLASH_INIT:
          FLP_set_current(20);
          FLP_on();
          clrscr();
          printxy(1,1,"FLEx Lighting");
          printxy(1,2,"FLP Dev Kit");
          printxy(1,3,"FW Ver %i", FIRMWARE_VERSION);
          vbat_avg = BATTERY_read_voltage();
          printxy(1,5,"Vbat = %04i mV", vbat_avg);
          dirty = 1;
          runticks = 0;
          State = STATE_SPLASH;
          break;
      case STATE_SPLASH:
          if (runticks < 16) {
              vbat_avg = (vbat_avg + BATTERY_read_voltage()) / 2;
          }
          printxy(1,5,"Vbat = %04i mV", vbat_avg);
          dirty = 1;
          if (runticks == 20) FLP_off();
          if (runticks >= 100 && BUTTON_held_time(1) == 0) {
              if (vbat_avg > 3300 || vbat_avg < 1500 || !HAL_GPIO_ReadPin(N_PGOOD_GPIO_Port, N_PGOOD_Pin)) {
                  State = STATE_SLIDESHOW_INIT;
              } else {
                  State = STATE_LOW_BATT_INIT;
              }
          }
          if (runticks < 150) {
              runticks++;
          }
          break;

      case STATE_LOW_BATT_INIT:
          clrscr();
          printxy(1, 1, "LOW BATTERY!");
          printxy(1, 2, "PLEASE CONNECT");
          printxy(1, 3, "USB CHARGER");
          dirty = 1;
          runticks = 0;
          State = STATE_LOW_BATT;
          break;
      case STATE_LOW_BATT:
          if (runticks >= 300 || !HAL_GPIO_ReadPin(N_PGOOD_GPIO_Port, N_PGOOD_Pin))
              State = STATE_OFF;
          runticks++;
          break;
      case STATE_SLIDESHOW_INIT:
          clrscr();
          cur_idx = 0;
          running = 1;
          FLP_set_current(EEPROM_Settings->default_led_current);
          State = STATE_SLIDESHOW_LOAD;
          break;
      case STATE_SLIDESHOW_WAIT:
          if (BUTTON_held_time(1) >= 100) {
              State = STATE_OFF;
          }
          if (!runticks) {
              cur_idx = (cur_idx+1) % (EEPROM_Settings->slide_count);
              State = STATE_SLIDESHOW_LOAD;
          }
          if (running) runticks--;
          if (!MEMLCD_busy()) {
              if(ledmsg_tim) {
                  hmemlcd.tilemaps[1].map = led_message;
                  uint8_t current_ma = FLP_get_current() / 100;
                  led_message[16+11] = '0' + current_ma % 10;
                  current_ma /= 10;
                  led_message[16+9] = '0' + current_ma % 10;
                  current_ma /= 10;
                  led_message[16+8] = (current_ma) ? '0' + current_ma % 10 : ' ';
                  dirty = 1;
              } else {
                  if (hmemlcd.tilemaps[1].map) {
                      hmemlcd.tilemaps[1].map = NULL;
                      dirty = 1;
                  }
              }
          }
          break;
      case STATE_SLIDESHOW_LOAD:
          runticks = EEPROM_Settings->slides[cur_idx].delay ? EEPROM_Settings->slides[cur_idx].delay : EEPROM_Settings->default_delay;
          uint16_t led_current = EEPROM_Settings->slides[cur_idx].led_current;
          if (led_current) {
              FLP_set_current(led_current);
              FLP_on();
          }
          EXTFLASH_read_screen(&hflash, EEPROM_Settings->slides[cur_idx].img, (void*)hmemlcd.buffer, MEMLCD_bufsize(&hmemlcd));
          dirty = 1;
          State = STATE_SLIDESHOW_WAIT;
          break;
      default:
          State = STATE_OFF;
          break;
      }

      CMD_tick();
      while (HAL_GetTick() - looptime < 20); // Cycle time = 20ms
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
      HAL_NVIC_SystemReset();
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
