/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "usbd_cdc_if.h"
#include "memlcd.h"
#include "extflash.h"
#include "command.h"
#include "eeprom.h"
#include "bsp.h"
#include "flp.h"
#include "buttons.h"
#include "console.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

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

uint8_t dirty, cur_idx, running;
uint16_t runticks, vbat_avg;
uint8_t b_ticks;

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

uint8_t led_message[] = {
        201,205,205,205,205,205,205,205,205,205,205,205,205,205,205,187,
        186,' ','1',' ','L','E','D',' ','2','0','.','0','m','A',' ',186,
        200,205,205,205,205,205,205,205,205,205,205,205,205,205,205,188,
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    uint8_t ledmsg_tim=0;
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

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
  CON_init();
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
          //EXTFLASH_read_screen(&hflash, EEPROM_Settings->slides[cur_idx].img, (void*)hmemlcd.buffer, MEMLCD_bufsize(&hmemlcd));
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
