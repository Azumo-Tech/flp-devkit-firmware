/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define N_PGOOD_Pin GPIO_PIN_13
#define N_PGOOD_GPIO_Port GPIOC
#define VBAT_IN_Pin GPIO_PIN_3
#define VBAT_IN_GPIO_Port GPIOC
#define BT1_Pin GPIO_PIN_0
#define BT1_GPIO_Port GPIOA
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
#define FIRMWARE_VERSION 20170428

void LED_set_current(uint16_t current);
/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
