/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
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
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "memlcd.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

static const uint16_t brightable[256] = {
		0, 0, 0, 0, 1, 1, 2, 3, 4, 5, 6, 7, 9, 10, 12, 14, 16, 18, 20, 22, 25, 27, 30, 33, 36, 39, 42, 45, 49, 52, 56, 60, 64, 68, 72, 76, 81, 85, 90, 95, 100, 105, 110, 115, 121, 126, 132, 138, 144, 150, 156, 162, 169, 175, 182, 189, 196, 203, 210, 217, 225, 232, 240, 248, 256, 264, 272, 280, 289, 297, 306, 315, 324, 333, 342, 351, 361, 370, 380, 390, 400, 410, 420, 430, 441, 451, 462, 473, 484, 495, 506, 517, 529, 540, 552, 564, 576, 588, 600, 612, 625, 637, 650, 663, 676, 689, 702, 715, 729, 742, 756, 770, 784, 798, 812, 826, 841, 855, 870, 885, 900, 915, 930, 945, 961, 976, 992, 1008, 1024, 1040, 1056, 1072, 1089, 1105, 1122, 1139, 1156, 1173, 1190, 1207, 1225, 1242, 1260, 1278, 1296, 1314, 1332, 1350, 1369, 1387, 1406, 1425, 1444, 1463, 1482, 1501, 1521, 1540, 1560, 1580, 1600, 1620, 1640, 1660, 1681, 1701, 1722, 1743, 1764, 1785, 1806, 1827, 1849, 1870, 1892, 1914, 1936, 1958, 1980, 2002, 2025, 2047, 2070, 2093, 2116, 2139, 2162, 2185, 2209, 2232, 2256, 2280, 2304, 2328, 2352, 2376, 2401, 2425, 2450, 2475, 2500, 2525, 2550, 2575, 2601, 2626, 2652, 2678, 2704, 2730, 2756, 2782, 2809, 2835, 2862, 2889, 2916, 2943, 2970, 2997, 3025, 3052, 3080, 3108, 3136, 3164, 3192, 3220, 3249, 3277, 3306, 3335, 3364, 3393, 3422, 3451, 3481, 3510, 3540, 3570, 3600, 3630, 3660, 3690, 3721, 3751, 3782, 3813, 3844, 3875, 3906, 3937, 3969, 4000, 4032, 4064
};

MEMLCD_HandleTypeDef hmemlcd = {
    .model = MEMLCD_SHARP_270,
	.hspi = &hspi3,
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

volatile uint8_t screenbuf[240][50];
volatile uint8_t dirty, cur_idx;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C2_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM3_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void EXTFLASH_power_down() {
	uint8_t cmd[] =  {0xB9};
	HAL_GPIO_WritePin(MEM_CS_GPIO_Port, MEM_CS_Pin, 0);
	HAL_SPI_Transmit(&hspi2, (void*)cmd, 1, 10);
	HAL_GPIO_WritePin(MEM_CS_GPIO_Port, MEM_CS_Pin, 1);
	for (volatile int i=0; i<200; i++);
}

void EXTFLASH_power_up() {
	uint8_t cmd[] =  {0xAB};
	HAL_GPIO_WritePin(MEM_CS_GPIO_Port, MEM_CS_Pin, 0);
	HAL_SPI_Transmit(&hspi2, (void*)cmd, 1, 10);
	HAL_GPIO_WritePin(MEM_CS_GPIO_Port, MEM_CS_Pin, 1);
	for (volatile int i=0; i<200; i++);
}

void EXTFLASH_read_screen(uint8_t index, uint16_t stride, void *buffer, uint16_t bufsize) {
	uint32_t addr = index*stride;
	uint8_t cmd[] =  {0x03, addr>>16, (addr>>8)&0xff, (addr)&0xff};
	HAL_GPIO_WritePin(MEM_CS_GPIO_Port, MEM_CS_Pin, 0);
	HAL_SPI_Transmit(&hspi2, (void*)cmd, 4, 10);
	HAL_SPI_Receive(&hspi2, buffer, bufsize, 100);
	HAL_GPIO_WritePin(MEM_CS_GPIO_Port, MEM_CS_Pin, 1);
}


void EXTFLASH_write_enable() {
	uint8_t cmd[] =  {0x06};
	HAL_GPIO_WritePin(MEM_CS_GPIO_Port, MEM_CS_Pin, 0);
	HAL_SPI_Transmit(&hspi2, (void*)cmd, 1, 10);
	HAL_GPIO_WritePin(MEM_CS_GPIO_Port, MEM_CS_Pin, 1);
}

void EXTFLASH_wait_for_busy() {
	uint8_t cmd = 0x05, status;
	do {
		HAL_GPIO_WritePin(MEM_CS_GPIO_Port, MEM_CS_Pin, 0);
		HAL_SPI_Transmit(&hspi2, (void*)&cmd, 1, 10);
		HAL_SPI_Receive(&hspi2, (void*)&status, 1, 10);
		HAL_GPIO_WritePin(MEM_CS_GPIO_Port, MEM_CS_Pin, 1);
	} while (status & 1);
}

void EXTFLASH_write_aligned_page(uint32_t addr, void *buffer, uint16_t size) {
	EXTFLASH_write_enable();
	uint8_t cmd[] =  {0x02, addr>>16, (addr>>8)&0xff, 0};
	HAL_GPIO_WritePin(MEM_CS_GPIO_Port, MEM_CS_Pin, 0);
	HAL_SPI_Transmit(&hspi2, (void*)cmd, 4, 10);
	HAL_SPI_Transmit(&hspi2, buffer, size, 100);
	HAL_GPIO_WritePin(MEM_CS_GPIO_Port, MEM_CS_Pin, 1);
	EXTFLASH_wait_for_busy();
}

void EXTFLASH_write_screen(uint8_t index, uint16_t stride, void *buffer, uint16_t bufsize) {
	uint32_t addr = index*stride;
	while(bufsize > 256) {
		EXTFLASH_write_aligned_page(addr, buffer, 256);
		buffer += 256;
		addr += 256;
		bufsize -= 256;
	}
	EXTFLASH_write_aligned_page(addr, buffer, bufsize);
}

void SystemClock_Config_SLOW(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time
    */
  //HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  //HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  //HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  SysTick->CTRL = 0;//SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void SleepyTime() {
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	HAL_GPIO_WritePin(LED_PWR_GPIO_Port, LED_PWR_Pin, 0); // Turn off LED
	HAL_DAC_Stop(&hdac, DAC_CHANNEL_1); // Stop LED DAC
	MEMLCD_power_off(&hmemlcd);
	EXTFLASH_power_down();
	USBD_Stop(&hUsbDeviceFS);
	USBD_DeInit(&hUsbDeviceFS);
	SysTick->CTRL = 0;
	SystemClock_Config_SLOW();
	while (!HAL_GPIO_ReadPin(BT1_GPIO_Port, BT1_Pin));
	while (HAL_GPIO_ReadPin(BT1_GPIO_Port, BT1_Pin) && HAL_GPIO_ReadPin(N_PGOOD_GPIO_Port, N_PGOOD_Pin)) {
		HAL_GPIO_WritePin(DBGPIN0_GPIO_Port, DBGPIN0_Pin, 1);
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		HAL_GPIO_WritePin(DBGPIN0_GPIO_Port, DBGPIN0_Pin, 0);
	}
	SystemClock_Config();
	MX_USB_DEVICE_Init();
	EXTFLASH_power_up();
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	MEMLCD_init(&hmemlcd);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	TIM3->CCR2 = 6000;
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t bt1_tim=0, bt2_tim=0, bt3_tim=0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DAC_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  MEMLCD_init(&hmemlcd);
  SleepyTime();
  for (int y=0; y<240; y++) {
	  for(int x=0; x<50; x++) {
		  //screenbuf[y][x] = ((y&15) >= 2) ? (x&1? 0b00111111 : 0xff): 0b00000000;
	  }
  }
  //EXTFLASH_write_screen(0, 12*1024, (void*)screenbuf, sizeof(screenbuf));
  EXTFLASH_read_screen(0, 12*1024, (void*)screenbuf, sizeof(screenbuf));
  dirty = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint32_t looptime = HAL_GetTick();
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, 2730/4);
	  if (dirty){
		  MEMLCD_update_area(&hmemlcd, &screenbuf[0][0], 0, 240);
		  dirty = 0;
	  }
	  if (!HAL_GPIO_ReadPin(BT1_GPIO_Port, BT1_Pin)) {
		  if (bt1_tim < 250) bt1_tim++;
		  if (bt1_tim == 100) {
			  bt1_tim = 250;
			  SleepyTime();
			  dirty = 1;
		 }
	  } else {
		  bt1_tim = 0;
	  }
	  if (!HAL_GPIO_ReadPin(BT2_GPIO_Port, BT2_Pin)) {
		  if (bt2_tim < 250) bt2_tim++;
	  } else {
		  if (bt2_tim > 5) HAL_GPIO_TogglePin(LED_PWR_GPIO_Port, LED_PWR_Pin);
		  bt2_tim = 0;
	  }

	  while (HAL_GetTick() - looptime < 20); // Cycle time = 20ms
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
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
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
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

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 200;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 12000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
     PB6   ------> USART1_TX
     PB7   ------> USART1_RX
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CHG_LIMIT_GPIO_Port, CHG_LIMIT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MEM_CS_GPIO_Port, MEM_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DBGPIN0_Pin|DBGPIN1_Pin|EN_BOOST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_EXTMODE_Pin|LCD_DISP_Pin|LED_PWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : N_PGOOD_Pin N_CHARGING_Pin */
  GPIO_InitStruct.Pin = N_PGOOD_Pin|N_CHARGING_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 
                           PC6 PC7 PC8 PC9 
                           PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9 
                          |GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BT1_Pin */
  GPIO_InitStruct.Pin = BT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BT2_Pin BT3_Pin */
  GPIO_InitStruct.Pin = BT2_Pin|BT3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CHG_LIMIT_Pin */
  GPIO_InitStruct.Pin = CHG_LIMIT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CHG_LIMIT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO0_Pin GPIO1_Pin PB2 PB9 */
  GPIO_InitStruct.Pin = GPIO0_Pin|GPIO1_Pin|GPIO_PIN_2|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEM_CS_Pin */
  GPIO_InitStruct.Pin = MEM_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MEM_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DBGPIN0_Pin DBGPIN1_Pin EN_BOOST_Pin */
  GPIO_InitStruct.Pin = DBGPIN0_Pin|DBGPIN1_Pin|EN_BOOST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_EXTMODE_Pin LCD_DISP_Pin LED_PWR_Pin */
  GPIO_InitStruct.Pin = LCD_EXTMODE_Pin|LCD_DISP_Pin|LED_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
