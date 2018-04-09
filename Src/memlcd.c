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

#include "main.h"
#include "stm32l1xx_hal.h"
#include "memlcd.h"

#include <string.h>
#include <stdint.h>

struct MEMLCD_Attributes {
    const char *model_name;
    uint16_t width, height;
    uint16_t flags;
    uint8_t vcom_freq;
};

const struct MEMLCD_Attributes MEMLCD_database[] = {
    {
        /* Name    */ "LS012B7DH02",
        /* W, H    */ 240, 240,
        /* Flags   */ MEMLCD_ADDR_SHARP | MEMLCD_MONO | MEMLCD_PWR_3V,
        /* VCOM Hz */ 60
    },
    {
        /* Name    */ "LS010B7DH04",
        /* W, H    */ 128, 128,
        /* Flags   */ MEMLCD_ADDR_SHARP | MEMLCD_MONO | MEMLCD_PWR_3V,
        /* VCOM Hz */ 60
    },
    {
        /* Name    */ "LS013B7DH03",
        /* W, H    */ 128, 128,
        /* Flags   */ MEMLCD_ADDR_SHARP | MEMLCD_MONO | MEMLCD_PWR_3V,
        /* VCOM Hz */ 60
    },
    {
        /* Name    */ "LS013B7DH05",
        /* W, H    */ 144, 168,
        /* Flags   */ MEMLCD_ADDR_SHARP | MEMLCD_MONO | MEMLCD_PWR_3V | MEMLCD_HFLIP | MEMLCD_VFLIP,
        /* VCOM Hz */ 60
    },
    {
        /* Name    */ "LS013B7DH06",
        /* W, H    */ 128, 128,
        /* Flags   */ MEMLCD_ADDR_SHARP | MEMLCD_RGB | MEMLCD_PWR_5V,
        /* VCOM Hz */ 5
    },
    {
        /* Name    */ "LS027B7DH01",
        /* W, H    */ 400, 240,
        /* Flags   */ MEMLCD_ADDR_SHARP | MEMLCD_MONO | MEMLCD_PWR_5V | MEMLCD_ROT270,
        /* VCOM Hz */ 5
    },
    {
        /* Name    */ "LS032B7DD02",
        /* W, H    */ 336, 536,
        /* Flags   */ MEMLCD_ADDR_SHARP_LONG | MEMLCD_MONO | MEMLCD_PWR_5V,
        /* VCOM Hz */ 5
    },
    {
        /* Name    */ "LS044Q7DH01",
        /* W, H    */ 320, 240,
        /* Flags   */ MEMLCD_ADDR_SHARP | MEMLCD_MONO | MEMLCD_PWR_5V | MEMLCD_ROT270,
        /* VCOM Hz */ 5
    },
    {
        /* Name    */ "LPM013M126A",
        /* W, H    */ 176, 176,
        /* Flags   */ MEMLCD_ADDR_JDI | MEMLCD_RGB | MEMLCD_PWR_3V,
        /* VCOM Hz */ 5
    },
    {
        /* Name    */ "LPM027M128B",
        /* W, H    */ 400, 240,
        /* Flags   */ MEMLCD_ADDR_JDI | MEMLCD_RGB | MEMLCD_PWR_3V | MEMLCD_ROT270,
        /* VCOM Hz */ 5
    },
    {
        /* Name    */ "LS011B7DH03",
        /* W, H    */ 160, 68,
        /* Flags   */ MEMLCD_ADDR_SHARP | MEMLCD_MONO | MEMLCD_PWR_3V,
        /* VCOM Hz */ 60
    },
};

const int MEMLCD_max_model = sizeof(MEMLCD_database)  / sizeof(*MEMLCD_database);

struct MEMLCD_UpdateState {
    MEMLCD_HandleTypeDef *hmemlcd;
    uint16_t start, end, line;
} Upd;


void MEMLCD_init(MEMLCD_HandleTypeDef *hmemlcd) {
    hmemlcd->flags = MEMLCD_database[hmemlcd->model].flags;
    hmemlcd->width = MEMLCD_database[hmemlcd->model].width;
    hmemlcd->height = MEMLCD_database[hmemlcd->model].height;

    hmemlcd->line_ct = hmemlcd->height;
    hmemlcd->line_len = hmemlcd->width/8 * ((hmemlcd->flags & MEMLCD_RGB) ? 3 : 1);

    HAL_GPIO_WritePin(hmemlcd->EXTMODE_Port, hmemlcd->EXTMODE_Pin, 1);

    HAL_GPIO_WritePin(hmemlcd->BOOST_Port, hmemlcd->BOOST_Pin, hmemlcd->flags & MEMLCD_PWR_5V);

    hmemlcd->hspi->Init.Mode = SPI_MODE_MASTER;
    hmemlcd->hspi->Init.Direction = SPI_DIRECTION_2LINES;
    hmemlcd->hspi->Init.DataSize = SPI_DATASIZE_8BIT;
    hmemlcd->hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
    hmemlcd->hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
    hmemlcd->hspi->Init.NSS = SPI_NSS_SOFT;
    hmemlcd->hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hmemlcd->hspi->Init.TIMode = SPI_TIMODE_DISABLE;
    hmemlcd->hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hmemlcd->hspi->Init.CRCPolynomial = 10;
    hmemlcd->hspi->Init.FirstBit = SPI_FIRSTBIT_LSB;

    HAL_SPI_Init(hmemlcd->hspi);

    MEMLCD_clear_all(hmemlcd);
    HAL_GPIO_WritePin(hmemlcd->DISP_Port, hmemlcd->DISP_Pin, 1);

    hmemlcd->htim->Init.Period = 60000/MEMLCD_database[hmemlcd->model].flags;
    HAL_TIM_Base_Init(hmemlcd->htim);
    TIM_OC_InitTypeDef pwm_conf;
    pwm_conf.OCMode = TIM_OCMODE_PWM1;
    pwm_conf.Pulse = 30000/MEMLCD_database[hmemlcd->model].vcom_freq;
    pwm_conf.OCPolarity = TIM_OCPOLARITY_HIGH;
    pwm_conf.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(hmemlcd->htim, &pwm_conf, hmemlcd->tim_ch);
    HAL_TIM_PWM_Start(hmemlcd->htim, hmemlcd->tim_ch);
}

void MEMLCD_power_off(MEMLCD_HandleTypeDef *hmemlcd) {
    HAL_GPIO_WritePin(hmemlcd->DISP_Port, hmemlcd->DISP_Pin, 0);
    HAL_GPIO_WritePin(hmemlcd->EXTMODE_Port, hmemlcd->EXTMODE_Pin, 0);
    HAL_GPIO_WritePin(hmemlcd->BOOST_Port, hmemlcd->BOOST_Pin, 0);
    HAL_GPIO_WritePin(hmemlcd->CS_Port, hmemlcd->CS_Pin, 0);

    HAL_TIM_PWM_Stop(hmemlcd->htim, hmemlcd->tim_ch);
}

void MEMLCD_clear_all(MEMLCD_HandleTypeDef *hmemlcd){
	memset(hmemlcd->buffer, 0, MEMLCD_bufsize(hmemlcd));

    uint8_t cmd[2] = {0b100, 0};
    /* For some reason we need to send the command twice for it to work */
    HAL_GPIO_WritePin(hmemlcd->CS_Port, hmemlcd->CS_Pin, 1);
    HAL_SPI_Transmit(hmemlcd->hspi, cmd, 2, 10);
    HAL_GPIO_WritePin(hmemlcd->CS_Port, hmemlcd->CS_Pin, 0);
    HAL_Delay(1);
    HAL_GPIO_WritePin(hmemlcd->CS_Port, hmemlcd->CS_Pin, 1);
    HAL_SPI_Transmit(hmemlcd->hspi, cmd, 2, 10);
    HAL_GPIO_WritePin(hmemlcd->CS_Port, hmemlcd->CS_Pin, 0);
    HAL_Delay(1);
}


void MEMLCD_set_disp(MEMLCD_HandleTypeDef *hmemlcd, uint8_t state) {
    HAL_GPIO_WritePin(hmemlcd->DISP_Port, hmemlcd->DISP_Pin, state);
}

inline static void MEMLCD_tilelayers_RGB(MEMLCD_HandleTypeDef *hmemlcd, uint16_t line, uint8_t* linebuf) {
    for (int i=0; i<3; i++) {
        struct TileLayer tl = hmemlcd->tilemaps[i];
        if (tl.tiles == NULL || tl.map == NULL) continue;
        uint8_t tilewidth = (tl.tile_size & 0x3) + 1;
        uint8_t tileheight = ((tl.tile_size >> 2) & 0x3) + 1;
        int y = ((hmemlcd->flags & MEMLCD_VFLIP)? (hmemlcd->line_ct - 1 - line - tl.scroll_y) : line - tl.scroll_y);
        int ytile = y/(tileheight*8);
        if (y < 0 || ytile >= ((tl.flags & TILE_TRANSPOSE)? tl.width: tl.height)) continue;
        uint8_t ysubtile = (y/8) % tileheight;
        for (int x=0; x < hmemlcd->line_len; x += 3) {
            int xt = ((hmemlcd->flags & MEMLCD_HFLIP)? ((hmemlcd->line_len-x)/3 - 1 - tl.scroll_x/8) : x/3 - tl.scroll_x/8);
            int xtile = xt/tilewidth;
            if (xt < 0) continue;
            if(xtile >= ((tl.flags & TILE_TRANSPOSE)?  tl.height: tl.width)) continue;
            uint8_t xsubtile = xt % tilewidth;

            uint16_t tilecoord = (tl.flags & TILE_TRANSPOSE)? (ytile + (tl.height-1-xtile)*tl.width) : (ytile*tl.width + xtile);
            if(tl.map[tilecoord]) {
                uint16_t subtile = ((tl.map[tilecoord]*tileheight + ysubtile)*tilewidth + xsubtile)*8 + (y & 7);
                uint8_t tilebits = (hmemlcd->flags & MEMLCD_HFLIP) ? (__RBIT(~tl.tiles[subtile])>>24): ~tl.tiles[subtile];
                uint32_t tilergb = 0, mask = 0x7;
                while (tilebits) {
                    tilergb |= (tilebits&1)*mask;
                    mask <<= 3;
                    tilebits >>= 1;
                }
                linebuf[x+2] = (tilergb>>16)&0xff;
                linebuf[x+1] = (tilergb>>8)&0xff;
                linebuf[x] = (tilergb)&0xff;
            }
        }
    }
}


inline static void MEMLCD_tilelayers_mono(MEMLCD_HandleTypeDef *hmemlcd, uint16_t line, uint8_t* linebuf) {
    for (int i=0; i<3; i++) {
        struct TileLayer tl = hmemlcd->tilemaps[i];
        if (tl.tiles == NULL || tl.map == NULL) continue;
        uint8_t tilewidth = (tl.tile_size & 0x3) + 1;
        uint8_t tileheight = ((tl.tile_size >> 2) & 0x3) + 1;
        int y = ((hmemlcd->flags & MEMLCD_VFLIP)? (hmemlcd->line_ct - 1 - line - tl.scroll_y) : line - tl.scroll_y);
        int ytile = y/(tileheight*8);
        if (y < 0 || ytile >= ((tl.flags & TILE_TRANSPOSE)? tl.width: tl.height)) continue;
        uint8_t ysubtile = (y/8) % tileheight;
        for (int x=0; x < hmemlcd->line_len; x++) {
            int xt = ((hmemlcd->flags & MEMLCD_HFLIP)? ((hmemlcd->line_len-x) - 1 - tl.scroll_x/8) : x - tl.scroll_x/8);
            int xtile = xt/tilewidth;
            if (xt < 0) continue;
            if(xtile >= ((tl.flags & TILE_TRANSPOSE)?  tl.height: tl.width)) continue;
            uint8_t xsubtile = xt % tilewidth;

            uint16_t tilecoord = (tl.flags & TILE_TRANSPOSE)? (ytile + (tl.height-1-xtile)*tl.width) : (ytile*tl.width + xtile);
            if(tl.map[tilecoord]) {
                uint16_t subtile = ((tl.map[tilecoord]*tileheight + ysubtile)*tilewidth + xsubtile)*8 + (y & 7);
                uint8_t tilebits = (hmemlcd->flags & MEMLCD_HFLIP) ? (__RBIT(~tl.tiles[subtile])>>24): ~tl.tiles[subtile];
                linebuf[x] = tilebits;
            }
        }
    }
}


void MEMLCD_send_next_line() {
    if (Upd.hmemlcd == NULL) return;
    MEMLCD_HandleTypeDef *hmemlcd = Upd.hmemlcd;
    if (++Upd.line > Upd.end) {
        uint8_t tail[2];
        HAL_SPI_Transmit(hmemlcd->hspi, tail, 2, 10);
        HAL_GPIO_WritePin(hmemlcd->CS_Port, hmemlcd->CS_Pin, 0);
        Upd.hmemlcd = NULL;
    } else {
        uint8_t *cmd = &hmemlcd->linebuf[Upd.line&1][0];
        HAL_GPIO_TogglePin(DBGPIN0_GPIO_Port, DBGPIN0_Pin);
        switch (hmemlcd->flags & 0xF) {
        case MEMLCD_ADDR_SHARP:
            cmd[0] = 1;
            cmd[1] = Upd.line & 0xff;
            break;
        case MEMLCD_ADDR_SHARP_LONG:
            cmd[0] = 1 | ((Upd.line<<6)&0xff);
            cmd[1] = (Upd.line>>2) & 0xff;
            break;
        case MEMLCD_ADDR_JDI: {
            uint32_t rev_addr = __RBIT(Upd.line) >> 22;
            cmd[0] = 1 | ((rev_addr<<6)&0xff);
            cmd[1] = (rev_addr>>2) & 0xff;
            break; }
        }
        memset(&cmd[2], 0xff, hmemlcd->line_len);
        memcpy(&cmd[2], &hmemlcd->buffer[hmemlcd->line_len * (Upd.line-1)], hmemlcd->line_len);
        if (hmemlcd->flags & MEMLCD_RGB) {
            MEMLCD_tilelayers_RGB(hmemlcd, Upd.line-1, &cmd[2]);
        } else {
            MEMLCD_tilelayers_mono(hmemlcd, Upd.line-1, &cmd[2]);
        }
        while(hmemlcd->hspi->State == HAL_SPI_STATE_BUSY_TX);
        HAL_SPI_Transmit_DMA(hmemlcd->hspi, cmd, hmemlcd->line_len+2);
        HAL_GPIO_TogglePin(DBGPIN0_GPIO_Port, DBGPIN0_Pin);
    }
}

int MEMLCD_busy() {
    return (Upd.hmemlcd != NULL);
}

void MEMLCD_update_area(MEMLCD_HandleTypeDef *hmemlcd, uint16_t start, uint16_t end) {
    if (Upd.hmemlcd != NULL) return;
    end = (end <= hmemlcd->line_ct)? end : hmemlcd->line_ct;
    start = (start <= end)? start : end;
    HAL_GPIO_WritePin(hmemlcd->CS_Port, hmemlcd->CS_Pin, 1);
    Upd.start = start+1;
    Upd.end = end+1;
    Upd.line = start;
    Upd.hmemlcd = hmemlcd;
    MEMLCD_send_next_line();
}

const char* MEMLCD_get_model_name(MEMLCD_HandleTypeDef *hmemlcd) {
    return MEMLCD_database[hmemlcd->model].model_name;
}

void MEMLCD_set_model_by_name(MEMLCD_HandleTypeDef *hmemlcd, char* name) {
    for (uint8_t i = 0; i < MEMLCD_max_model; i++) {
        if (!strncmp(MEMLCD_database[i].model_name, name, 12)) {
            hmemlcd->model = i;
            break;
        }
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
    if (Upd.hmemlcd != NULL || hspi == Upd.hmemlcd->hspi) {
        MEMLCD_send_next_line();
    }
}
