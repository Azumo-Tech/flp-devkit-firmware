/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define IDD_CNT_EN_Pin GPIO_PIN_13
#define IDD_CNT_EN_GPIO_Port GPIOC
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define IDD_Measurement_Pin GPIO_PIN_4
#define IDD_Measurement_GPIO_Port GPIOA
#define GRP2_Sampling_Pin GPIO_PIN_6
#define GRP2_Sampling_GPIO_Port GPIOA
#define GRP2_Ground_Pin GPIO_PIN_7
#define GRP2_Ground_GPIO_Port GPIOA
#define GRP9_Sampling_Pin GPIO_PIN_4
#define GRP9_Sampling_GPIO_Port GPIOC
#define GRP9_Ground_Pin GPIO_PIN_5
#define GRP9_Ground_GPIO_Port GPIOC
#define GRP3_Sampling_Pin GPIO_PIN_0
#define GRP3_Sampling_GPIO_Port GPIOB
#define GRP3_Ground_Pin GPIO_PIN_1
#define GRP3_Ground_GPIO_Port GPIOB
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define LCD_CS_Pin GPIO_PIN_15
#define LCD_CS_GPIO_Port GPIOA
#define LCD_EXTMODE_Pin GPIO_PIN_12
#define LCD_EXTMODE_GPIO_Port GPIOC
#define LCD_DISP_Pin GPIO_PIN_2
#define LCD_DISP_GPIO_Port GPIOD
#define LCD_CK_Pin GPIO_PIN_3
#define LCD_CK_GPIO_Port GPIOB
#define LCD_EXTCOM_Pin GPIO_PIN_4
#define LCD_EXTCOM_GPIO_Port GPIOB
#define LCD_DI_Pin GPIO_PIN_5
#define LCD_DI_GPIO_Port GPIOB
#define LD4_Pin GPIO_PIN_6
#define LD4_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_7
#define LD3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
