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

#include <stdio.h>

#include "main.h"
#include "stm32l1xx_hal.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

#include "command.h"
#include "memlcd.h"
#include "extflash.h"
#include "eeprom.h"
#include "bsp.h"
#include "flp.h"

extern EXTFLASH_HandleTypeDef hflash;
extern MEMLCD_HandleTypeDef hmemlcd;
extern volatile uint8_t running;

static enum CMDState {
    CMD_NORMAL,
    CMD_ESC,
    CMD_READ_INT_ARG,
    CMD_READ_BIN_ARG,
} Mode = CMD_NORMAL;

static enum CMDCommand {
    CMD_LOAD,
    CMD_WRITE,
    CMD_DOWNLOAD,
    CMD_FILL,
    CMD_WRITE_OP,
    CMD_GET_SETTING,
    CMD_SAVE_SETTING,
    CMD_SET_CURRENT,
} Command;

uint8_t CurArg, Argc, ArgHexMode;
int ArgSign = 1;
int IntArgv[8];

static void beginIntArgs(uint8_t count) {
    Argc = count;
    CurArg = 0;
    ArgSign = 1;
    ArgHexMode = 0;
    for (int i=0; i<count; i++) {
        IntArgv[i] = 0;
    }
    Mode = CMD_READ_INT_ARG;
}

uint8_t *BinArgBuf;
size_t BinArgCount;

static void beginBinTransfer(uint8_t *buf, size_t count) {
    BinArgBuf = buf;
    BinArgCount = count;
    Mode = CMD_READ_BIN_ARG;
}

uint16_t BATTERY_read_voltage();

uint8_t EchoOn=0;

void CMD_tick() {
    uint8_t chr;
    while (CDC_read(&chr, 1)) {
        while (EchoOn && CDC_Transmit_FS(&chr, 1) == USBD_BUSY); /* Echo the character back */
        switch (Mode) {
        case CMD_NORMAL:
            switch(chr) {
            case 0x0E: /* Turn LED on */
                FLP_on();
                break;
            case 0x0F: /* Turn LED off */
                FLP_off();
                break;
            case 0x1B: /* Start of escape sequence */
                Mode = CMD_ESC;
                break;
            default:
                break; /* Ignore */
            }
            break;
        case CMD_ESC:
            switch(chr) {
            case 'c': /* Clear screen */
                MEMLCD_clear_all(&hmemlcd);
                memset(hmemlcd.buffer, 0, MEMLCD_bufsize(&hmemlcd));
                running = 0;
                Mode = CMD_NORMAL;
                break;
            case 'B': /* Set led current*/
                Command = CMD_SET_CURRENT;
                beginIntArgs(1);
                break;
            case 'E': /* Toggle Echo */
                EchoOn ^= 0xFF;
                Mode = CMD_NORMAL;
                break;
            case 'D': /* Download bitmap */
                Command = CMD_DOWNLOAD;
                beginBinTransfer(hmemlcd.buffer, MEMLCD_bufsize(&hmemlcd));
                break;
            case 'L': /* Load from flash */
                Command = CMD_LOAD;
                beginIntArgs(1);
                break;
            case 'W': /* Save to flash */
                Command = CMD_WRITE;
                beginIntArgs(1);
                break;
            case 'F': /* Fill with a color */
                Command = CMD_FILL;
                beginIntArgs(1);
                break;
            case '!': /* Save option */
                Command = CMD_SAVE_SETTING;
                beginIntArgs(2);
                break;
            case '?': /* Get option */
                Command = CMD_GET_SETTING;
                beginIntArgs(1);
                break;
            case 'b':
                BSP_reset();
                Mode = CMD_NORMAL;
                break;
            default:
                Mode = CMD_NORMAL;
                break; /* Ignore */
            }
            break;
        case CMD_READ_INT_ARG:
            if (ArgHexMode && chr >= '0' && chr <= '9' ){
                            IntArgv[CurArg] = IntArgv[CurArg]*16 + chr-'0';
            } else if (ArgHexMode && chr >= 'A' && chr <= 'F' ){
                IntArgv[CurArg] = IntArgv[CurArg]*16 + 10+chr-'A';
            } else if (!ArgHexMode && chr >= '0' && chr <= '9') {
                IntArgv[CurArg] = IntArgv[CurArg]*10 + chr-'0';
            } else if (!ArgHexMode && chr == '-') {
                ArgSign *= -1;
            } else if (chr == '$') {
                ArgHexMode = 1;
            } else if (chr == ';' || chr == ',') {
                IntArgv[CurArg] *= ArgSign;
                CurArg++;
                ArgHexMode = 0;
            } else if (!ArgHexMode && chr >= '@') {
                IntArgv[CurArg] = chr - '@';
                CurArg++;
            } else {
                Mode = CMD_NORMAL;
            }
            if (CurArg >= Argc) {
                switch (Command) {
                case CMD_LOAD: {
                    uint8_t idx = (IntArgv[0] > 24)? 24: IntArgv[0];
                    EXTFLASH_read_screen(&hflash, idx, (void*)hmemlcd.buffer, MEMLCD_bufsize(&hmemlcd));
                    MEMLCD_update_area(&hmemlcd, 0, -1);
                    break; }
                case CMD_WRITE: {
                    uint8_t idx = (IntArgv[0] > 24)? 24: IntArgv[0];
                    EXTFLASH_write_screen(&hflash, idx, (void*)hmemlcd.buffer, MEMLCD_bufsize(&hmemlcd));
                    break; }
                case CMD_FILL: {
                    if (hmemlcd.flags & MEMLCD_RGB) {
                        uint32_t colormask = (IntArgv[0] & 7) * 0b001001001001001001001001;
                        for (int y=0; y<hmemlcd.line_ct; y++) {
                            for (int x=0; x<hmemlcd.line_len; x+=3) {
                                hmemlcd.buffer[y*hmemlcd.line_len+x+2] = (colormask >> 16) & 0xff;
                                hmemlcd.buffer[y*hmemlcd.line_len+x+1] = (colormask >> 8) & 0xff;
                                hmemlcd.buffer[y*hmemlcd.line_len+x+0] = (colormask) & 0xff;
                            }
                        }
                    } else {
                        uint8_t color = (IntArgv[0])? 0xff: 0;
                        for (int y=0; y<hmemlcd.line_ct; y++) {
                            for (int x=0; x<hmemlcd.line_len; x++) {
                                hmemlcd.buffer[y*hmemlcd.line_len+x] = color;
                            }
                        }
                    }
                    MEMLCD_update_area(&hmemlcd, 0,-1 );
                    break; }
                case CMD_SET_CURRENT: {
                    FLP_set_current(IntArgv[0]);
                    break; }
                case CMD_GET_SETTING: {
                    char response[32];
                    size_t rlen = 0;
                    switch(IntArgv[0]+'@') {
                    case 'N': /* Number of slides */
                        rlen = snprintf(response, 32, "slides = %i\r\n", EEPROM_Settings->slide_count);
                        break;
                    case 'D': /* Default Delay */
                        rlen = snprintf(response, 32, "delay = %i\r\n", EEPROM_Settings->default_delay);
                        break;
                    case 'C': /* Default LED Current */
                        rlen = snprintf(response, 32, "current = %i\r\n", EEPROM_Settings->default_led_current);
                        break;
                    case 'M': /* Display Model */
                        rlen = snprintf(response, 32, "display_model = %s\r\n", MEMLCD_get_model_name(&hmemlcd));
                        break;
                    case 'B':
                        rlen = snprintf(response, 32, "vbat = %i\r\n", BATTERY_read_voltage());
                        break;
                    case 'V':
                        rlen = snprintf(response, 32, "version = %i\r\n", FIRMWARE_VERSION);
                        break;
                    default:
                        rlen = snprintf(response, 32, "E:Unknown variable %i\r\n", IntArgv[0]);
                        break;
                    }
                    while (CDC_Transmit_FS((uint8_t*)response, rlen) == USBD_BUSY);
                    break; }
                case CMD_SAVE_SETTING: {
                    char response[32];
                    size_t rlen = 0;
                    switch(IntArgv[0]+'@') {
                    case 'N': /* Number of slides */
                        HAL_FLASHEx_DATAEEPROM_Unlock();
                        HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, (size_t)&EEPROM_Settings->slide_count, IntArgv[1]);
                        HAL_FLASHEx_DATAEEPROM_Lock();
                        rlen = snprintf(response, 32, "!slides = %i\r\n", IntArgv[1]);
                        break;
                    case 'D': /* Default Delay */
                        HAL_FLASHEx_DATAEEPROM_Unlock();
                        HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, (size_t)&EEPROM_Settings->default_delay, IntArgv[1]);
                        HAL_FLASHEx_DATAEEPROM_Lock();
                        rlen = snprintf(response, 32, "!delay = %i\r\n", IntArgv[1]);
                        break;
                    case 'C': /* Default led current */
                        HAL_FLASHEx_DATAEEPROM_Unlock();
                        HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_HALFWORD, (size_t)&EEPROM_Settings->default_led_current, IntArgv[1]);
                        HAL_FLASHEx_DATAEEPROM_Lock();
                        rlen = snprintf(response, 32, "!current = %i\r\n", IntArgv[1]);
                        break;
                    case 'M':
                        if (IntArgv[1] >= 0 && IntArgv[1] < MEMLCD_max_model) {
                            HAL_FLASHEx_DATAEEPROM_Unlock();
                            HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, (size_t)&EEPROM_Settings->lcd_model, IntArgv[1]);
                            HAL_FLASHEx_DATAEEPROM_Lock();
                            BSP_init();
                            rlen = snprintf(response, 32, "!display_model = %s\r\n", MEMLCD_get_model_name(&hmemlcd));
                        } else {
                            rlen = snprintf(response, 32, "E:Unknown Model %i\r\n", IntArgv[1]);
                        }
                        break;
                    case 0xCAFEFF2D:
                        rlen = snprintf(response, 32, "!dfu_flag = %X\r\n", IntArgv[1]);
                        if (IntArgv[1] == 0xDEADBEEF) {
                        	BSP_reset_to_bootloader();
                        }
                        break;
                    default:
                        rlen = snprintf(response, 32, "E:Unknown variable %i = %i\r\n", IntArgv[0], IntArgv[1]);
                        break;
                    }
                    while (CDC_Transmit_FS((uint8_t*)response, rlen) == USBD_BUSY);
                    break; }
                default:
                    break;
                }
                Mode = CMD_NORMAL;
            }
            break;
        case CMD_READ_BIN_ARG:
            *BinArgBuf++ = chr;
            BinArgCount--;
            if (BinArgCount <= 0) {
                switch (Command) {
                case CMD_DOWNLOAD:
                    MEMLCD_update_area(&hmemlcd, 0, -1);
                    break;
                default:
                    break;
                }
                Mode = CMD_NORMAL;
            }
            break;
        }

    }
}
