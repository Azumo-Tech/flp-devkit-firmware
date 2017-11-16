/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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
#include <stdio.h>
#include "usbd_cdc_if.h"
#include "memlcd.h"
#include "extflash.h"
#include "segfont.h"
#include "font8x16.xbm"
#include "font8x16_transp.xbm"
#include "command.h"
#include "eeprom.h"
#include "led.h"
#include "battery-icon.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

static uint8_t tilemap[1408];

#define printxy(X,Y, ...) sprintf((char *)&tilemap[hmemlcd.tilemaps[0].width*(Y)+(X)], __VA_ARGS__)

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
uint8_t batticks, batidx, batdirty, b_ticks;
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

#define SYSMEM_RESET_VECTOR        0x1ff00004
#define DFU_RESET_COOKIE           0xDEADBEEF
#define BOOTLOADER_STACK_POINTER   0x20001000
/* This code relies on data after BSS being uninitialized on reset, so we know if we wanted to jump to the bootloader */
extern uint32_t _ebss;
static uint32_t *dfu_reset_flag = &_ebss+1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C2_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

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
    uint16_t vdda = (3000L*(*(uint16_t*)0x1FF800F8)) / vdda_raw_adc;

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

int SleepyTime() {
    int ret = 0;
    HAL_GPIO_WritePin(LED_PWR_GPIO_Port, LED_PWR_Pin, 0); // Turn off LED
    HAL_DAC_Stop(&hdac, DAC_CHANNEL_1); // Stop LED DAC
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
    HAL_GPIO_WritePin(USB_DISCONNECT_GPIO_Port, USB_DISCONNECT_Pin, 1);
    HAL_Delay(10);
    HAL_GPIO_WritePin(USB_DISCONNECT_GPIO_Port, USB_DISCONNECT_Pin, 0);
    MX_USB_DEVICE_Init();
    EXTFLASH_power_up(&hflash);
    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
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

int main(void)
{

  /* USER CODE BEGIN 1 */
    uint8_t bt1_tim=0, bt2_tim=0, bt3_tim=0, ledmsg_tim=0;

    if (*dfu_reset_flag == DFU_RESET_COOKIE) {
            void (*bootloader)(
                    void) = (void (*)(void)) (*((uint32_t *) SYSMEM_RESET_VECTOR));
            *dfu_reset_flag = 0;
            __set_MSP(BOOTLOADER_STACK_POINTER);
            SYSCFG->MEMRMP = 1;
            bootloader();
            while (1);
    }
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
      if (!HAL_GPIO_ReadPin(BT1_GPIO_Port, BT1_Pin)) {
          if (bt1_tim < 250) bt1_tim++;
      } else {
          bt1_tim = 0;
      }
      if (!HAL_GPIO_ReadPin(BT2_GPIO_Port, BT2_Pin)) {
          if (bt2_tim < 250) bt2_tim++;
          if (bt2_tim >= 40 && HAL_GPIO_ReadPin(LED_PWR_GPIO_Port, LED_PWR_Pin)) {
              b_ticks++;
              if (b_ticks >= 4) {
                  LED_change_brightness(1);
                  b_ticks = 0;
              }
              ledmsg_tim = 50;
          }
      } else {
          if(ledmsg_tim) ledmsg_tim--;
          if (bt2_tim > 3 && bt2_tim < 40)
              HAL_GPIO_TogglePin(LED_PWR_GPIO_Port, LED_PWR_Pin);
          bt2_tim = 0;
      }

      if (!HAL_GPIO_ReadPin(BT3_GPIO_Port, BT3_Pin)) {
          if (bt3_tim < 250) bt3_tim++;
          if (bt3_tim == 100) {
              running = 1;
              runticks = 0;
          }
      } else {
          if (bt3_tim > 3 && bt3_tim < 100) {
              running = 0;
              runticks = 0;
          }
          bt3_tim = 0;
      }
      switch (State) {
      case STATE_OFF:
          while (MEMLCD_busy());
          memset(hmemlcd.buffer, 0, MEMLCD_bufsize(&hmemlcd));
          if (SleepyTime() == 1) { // Woke up with button
              State = STATE_SPLASH_INIT;
          } else { // Woke up with USB
              State = STATE_CHARGING_INIT;
          }
          break;

      case STATE_CHARGING_INIT:
          if (!MEMLCD_busy()) {
              memset(tilemap, 0, sizeof(tilemap));
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
          if (bt1_tim) State = STATE_SPLASH_INIT;
          if(!HAL_GPIO_ReadPin(N_PGOOD_GPIO_Port, N_PGOOD_Pin) && HAL_GPIO_ReadPin(N_CHARGING_GPIO_Port, N_CHARGING_Pin)) {
              State = STATE_SPLASH_INIT; // If charging is done
          }
          if (HAL_GPIO_ReadPin(N_PGOOD_GPIO_Port, N_PGOOD_Pin)) {
              State = STATE_OFF; // If disconnected
          }
          break;
      case STATE_SPLASH_INIT:
          LED_set_current(20);
          HAL_GPIO_WritePin(LED_PWR_GPIO_Port, LED_PWR_Pin, 1);
          memset(tilemap, 0, sizeof(tilemap));
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
          if (runticks == 20) HAL_GPIO_WritePin(LED_PWR_GPIO_Port, LED_PWR_Pin, 0);
          if (runticks >= 100 && bt1_tim == 0) {
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
          memset(tilemap, 0, sizeof(tilemap));
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
          memset(tilemap, 0, sizeof(tilemap));
          cur_idx = 0;
          running = 1;
          LED_set_current(EEPROM_Settings->default_led_current);
          State = STATE_SLIDESHOW_LOAD;
          break;
      case STATE_SLIDESHOW_WAIT:
          if (bt1_tim >= 100) {
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
                  uint8_t current_ma = LED_get_current() / 100;
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
              LED_set_current(led_current);
              HAL_GPIO_WritePin(LED_PWR_GPIO_Port, LED_PWR_Pin, 1);
          }
          EXTFLASH_read_screen(&hflash, EEPROM_Settings->slides[cur_idx].img, (void*)hmemlcd.buffer, MEMLCD_bufsize(&hmemlcd));
          dirty = 1;
          State = STATE_SLIDESHOW_WAIT;
          break;
      default:
          State = STATE_OFF;
          break;
      }

#ifdef _USE_BAT_ICON
      if (HAL_GPIO_ReadPin(N_CHARGING_GPIO_Port, N_CHARGING_Pin) == 0) {
          if (batticks++ > 20 || dirty) {
              batticks=0;
              batidx = (((batidx-1) + 1) & 3) + 1;
              batdirty=1;
          }
      } else if (HAL_GPIO_ReadPin(N_PGOOD_GPIO_Port, N_PGOOD_Pin) == 0) {
          if (batidx != 5 || dirty) {
              batidx = 5;
              batdirty=1;
          }
      }
      if (batdirty){
          uint8_t bpp = (hmemlcd.flags & MEMLCD_RGB)? 3 : 1;
          for (int y=0; y<8; y++) {
              uint32_t *dest = &MEMLCD_get_bb_buffer(&hmemlcd)[(y+8)*hmemlcd.line_len*8];
              for (int x=0; x<16; x++) {
                  for (int b=0; b<bpp; b++) {
                      dest[b+(x+hmemlcd.line_len*8-24)*bpp] = (battery_bin[(batidx)*16+2*y+x/8] & 1<<(x&7))? 1 : 0;
                  }
              }
          }
          batdirty = 0;
          dirty = 1;
      }
#endif

      CMD_tick();
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

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  LL_RTC_InitTypeDef RTC_InitStruct;

  /* Peripheral clock enable */
  LL_RCC_EnableRTC();

    /**Initialize RTC and set the Time and Date 
    */
  RTC_InitStruct.HourFormat = LL_RTC_HOURFORMAT_24HOUR;
  RTC_InitStruct.AsynchPrescaler = 127;
  RTC_InitStruct.SynchPrescaler = 255;
  LL_RTC_Init(RTC, &RTC_InitStruct);

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

  /* SPI3 parameter configuration*/
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
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, VBAT_MEASURE_Pin|USB_DISCONNECT_Pin|DBGPIN0_Pin|DBGPIN1_Pin 
                          |EN_BOOST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CHG_LIMIT_GPIO_Port, CHG_LIMIT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MEM_CS_GPIO_Port, MEM_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_EXTMODE_Pin|LCD_DISP_Pin|LED_PWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : N_PGOOD_Pin */
  GPIO_InitStruct.Pin = N_PGOOD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(N_PGOOD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC6 
                           PC7 PC8 PC9 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_11;
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

  /*Configure GPIO pins : VBAT_MEASURE_Pin USB_DISCONNECT_Pin DBGPIN0_Pin DBGPIN1_Pin 
                           EN_BOOST_Pin */
  GPIO_InitStruct.Pin = VBAT_MEASURE_Pin|USB_DISCONNECT_Pin|DBGPIN0_Pin|DBGPIN1_Pin 
                          |EN_BOOST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CHG_LIMIT_Pin */
  GPIO_InitStruct.Pin = CHG_LIMIT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CHG_LIMIT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : N_CHARGING_Pin */
  GPIO_InitStruct.Pin = N_CHARGING_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(N_CHARGING_GPIO_Port, &GPIO_InitStruct);

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

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
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
