#include <stdio.h>

#include "main.h"
#include "stm32l1xx_hal.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

#include "command.h"
#include "memlcd.h"
#include "extflash.h"
#include "eeprom.h"
#include "led.h"

extern EXTFLASH_HandleTypeDef hflash;
extern MEMLCD_HandleTypeDef hmemlcd;
extern volatile uint8_t running;

/* This code relies on data after BSS being uninitialized on reset, so we know if we wanted to jump to the bootloader */
extern uint32_t _ebss;
static uint32_t *dfu_reset_flag = &_ebss+1;

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


uint8_t EchoOn=0;

void CMD_tick() {
    uint8_t chr;
    while (CDC_read(&chr, 1)) {
        while (EchoOn && CDC_Transmit_FS(&chr, 1) == USBD_BUSY); /* Echo the character back */
        switch (Mode) {
        case CMD_NORMAL:
            switch(chr) {
            case 0x0E: /* Turn LED on */
                HAL_GPIO_WritePin(LED_PWR_GPIO_Port, LED_PWR_Pin, 1);
                break;
            case 0x0F: /* Turn LED off */
                HAL_GPIO_WritePin(LED_PWR_GPIO_Port, LED_PWR_Pin, 0);
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
            case '!': /* Save option */
                Command = CMD_SAVE_SETTING;
                beginIntArgs(2);
                break;
            case '?': /* Get option */
                Command = CMD_GET_SETTING;
                beginIntArgs(1);
                break;
            case 'b':
                HAL_GPIO_WritePin(USB_DISCONNECT_GPIO_Port, USB_DISCONNECT_Pin, 1);
                USBD_Stop(&hUsbDeviceFS);
                HAL_Delay(1000);
                HAL_NVIC_SystemReset();
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
                    MEMLCD_update_area(&hmemlcd, 1, -1);
                    break; }
                case CMD_WRITE: {
                    uint8_t idx = (IntArgv[0] > 24)? 24: IntArgv[0];
                    EXTFLASH_write_screen(&hflash, idx, (void*)hmemlcd.buffer, MEMLCD_bufsize(&hmemlcd));
                    break; }
                case CMD_SET_CURRENT: {
                    LED_set_current(IntArgv[0]);
                    break; }
                case CMD_GET_SETTING: {
                    char response[32];
                    size_t rlen = 0;
                    switch(IntArgv[0]+'@') {
                    case 'N': /* Number of slides */
                        rlen = snprintf(response, 32, "slides = %i\n", EEPROM_Settings->slide_count);
                        break;
                    case 'D': /* Default Delay */
                        rlen = snprintf(response, 32, "delay = %i\n", EEPROM_Settings->default_delay);
                        break;
                    case 'C': /* Default LED Current */
                        rlen = snprintf(response, 32, "current = %i\n", EEPROM_Settings->default_led_current);
                        break;
                    default:
                        rlen = snprintf(response, 32, "UNKNOWN VARIABLE\n");
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
                        rlen = snprintf(response, 32, "!slides = %i\n", IntArgv[1]);
                        break;
                    case 'D': /* Default Delay */
                        HAL_FLASHEx_DATAEEPROM_Unlock();
                        HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, (size_t)&EEPROM_Settings->default_delay, IntArgv[1]);
                        HAL_FLASHEx_DATAEEPROM_Lock();
                        rlen = snprintf(response, 32, "!delay = %i\n", IntArgv[1]);
                        break;
                    case 'C': /* Default led current */
                        HAL_FLASHEx_DATAEEPROM_Unlock();
                        HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_HALFWORD, (size_t)&EEPROM_Settings->default_led_current, IntArgv[1]);
                        HAL_FLASHEx_DATAEEPROM_Lock();
                        rlen = snprintf(response, 32, "!current = %i\n", IntArgv[1]);
                        break;
                    case 0xCAFEFF2D:
                        *dfu_reset_flag = IntArgv[1];
                        rlen = snprintf(response, 32, "!dfu_flag = %X\n", IntArgv[1]);
                        break;
                    default:
                        rlen = snprintf(response, 32, "Unknown variable %i = %i\n", IntArgv[0], IntArgv[1]);
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
                    MEMLCD_update_area(&hmemlcd, 1, -1);
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
