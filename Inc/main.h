/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"
#include "stm32l1xx_hal.h"
#include "stm32l1xx_ll_rtc.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_cortex.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_pwr.h"
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_dma.h"

#include "stm32l1xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define N_PGOOD_Pin GPIO_PIN_13
#define N_PGOOD_GPIO_Port GPIOC
#define N_PGOOD_EXTI_IRQn EXTI15_10_IRQn
#define VBAT_IN_Pin GPIO_PIN_3
#define VBAT_IN_GPIO_Port GPIOC
#define BT1_Pin GPIO_PIN_0
#define BT1_GPIO_Port GPIOA
#define BT1_EXTI_IRQn EXTI0_IRQn
#define BT2_Pin GPIO_PIN_1
#define BT2_GPIO_Port GPIOA
#define BT3_Pin GPIO_PIN_2
#define BT3_GPIO_Port GPIOA
#define VBAT_MEASURE_Pin GPIO_PIN_3
#define VBAT_MEASURE_GPIO_Port GPIOA
#define LED_ISET_Pin GPIO_PIN_4
#define LED_ISET_GPIO_Port GPIOA
#define CHG_LIMIT_Pin GPIO_PIN_4
#define CHG_LIMIT_GPIO_Port GPIOC
#define N_CHARGING_Pin GPIO_PIN_5
#define N_CHARGING_GPIO_Port GPIOC
#define GPIO0_Pin GPIO_PIN_0
#define GPIO0_GPIO_Port GPIOB
#define GPIO1_Pin GPIO_PIN_1
#define GPIO1_GPIO_Port GPIOB
#define MEM_CS_Pin GPIO_PIN_12
#define MEM_CS_GPIO_Port GPIOB
#define MEM_SCK_Pin GPIO_PIN_13
#define MEM_SCK_GPIO_Port GPIOB
#define MEM_SO_Pin GPIO_PIN_14
#define MEM_SO_GPIO_Port GPIOB
#define MEM_SI_Pin GPIO_PIN_15
#define MEM_SI_GPIO_Port GPIOB
#define USB_DISCONNECT_Pin GPIO_PIN_8
#define USB_DISCONNECT_GPIO_Port GPIOA
#define DBGPIN0_Pin GPIO_PIN_9
#define DBGPIN0_GPIO_Port GPIOA
#define DBGPIN1_Pin GPIO_PIN_10
#define DBGPIN1_GPIO_Port GPIOA
#define EN_BOOST_Pin GPIO_PIN_15
#define EN_BOOST_GPIO_Port GPIOA
#define LCD_SCK_Pin GPIO_PIN_10
#define LCD_SCK_GPIO_Port GPIOC
#define LCD_SI_Pin GPIO_PIN_12
#define LCD_SI_GPIO_Port GPIOC
#define LCD_CS_Pin GPIO_PIN_2
#define LCD_CS_GPIO_Port GPIOD
#define LCD_EXTMODE_Pin GPIO_PIN_3
#define LCD_EXTMODE_GPIO_Port GPIOB
#define LCD_DISP_Pin GPIO_PIN_4
#define LCD_DISP_GPIO_Port GPIOB
#define LCD_EXTCOM_Pin GPIO_PIN_5
#define LCD_EXTCOM_GPIO_Port GPIOB
#define LED_PWR_Pin GPIO_PIN_8
#define LED_PWR_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
