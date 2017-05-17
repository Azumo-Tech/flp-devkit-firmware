#include "main.h"
#include "stm32l1xx_hal.h"
#include "memlcd.h"

#include <string.h>
#include <stdint.h>

static const uint8_t MEMLCD_flags[] = {
        /* Sharp */
        [MEMLCD_LS012B7DH02] = MEMLCD_ADDR_SHARP | MEMLCD_MONO | MEMLCD_PWR_3V,
        [MEMLCD_LS013B7DH05] = MEMLCD_ADDR_SHARP | MEMLCD_MONO | MEMLCD_PWR_3V,
        [MEMLCD_LS027B7DH01] = MEMLCD_ADDR_SHARP | MEMLCD_MONO | MEMLCD_PWR_5V,
        [MEMLCD_LS032B7DD02] = MEMLCD_ADDR_SHARP_LONG | MEMLCD_MONO | MEMLCD_PWR_5V,
        [MEMLCD_LS044Q7DH01] = MEMLCD_ADDR_SHARP | MEMLCD_MONO | MEMLCD_PWR_5V,
        /* JDI */
        [MEMLCD_LPM027M128B] = MEMLCD_ADDR_JDI | MEMLCD_RGB | MEMLCD_PWR_3V,
        [MEMLCD_LPM013M126A] = MEMLCD_ADDR_JDI | MEMLCD_RGB | MEMLCD_PWR_3V,
};

static const uint8_t MEMLCD_vcom_freq[] = {
        /* Sharp */
        [MEMLCD_LS012B7DH02] = 60,
        [MEMLCD_LS013B7DH05] = 60,
        [MEMLCD_LS027B7DH01] = 5,
        [MEMLCD_LS032B7DD02] = 5,
        [MEMLCD_LS044Q7DH01] = 5,
        /* JDI */
        [MEMLCD_LPM027M128B] = 5,
        [MEMLCD_LPM013M126A] = 5,
};

static const uint16_t MEMLCD_line_count[] = {
        /* Sharp */
        [MEMLCD_LS012B7DH02] = 240,
        [MEMLCD_LS013B7DH05] = 168,
        [MEMLCD_LS027B7DH01] = 240,
        [MEMLCD_LS032B7DD02] = 536,
        [MEMLCD_LS044Q7DH01] = 240,
        /* JDI */
        [MEMLCD_LPM027M128B] = 240,
        [MEMLCD_LPM013M126A] = 176,
};

static const uint8_t MEMLCD_line_length[] = {
        /* Sharp */
        [MEMLCD_LS012B7DH02] = 240/8,
        [MEMLCD_LS013B7DH05] = 144/8,
        [MEMLCD_LS027B7DH01] = 400/8,
        [MEMLCD_LS032B7DD02] = 336/8,
        [MEMLCD_LS044Q7DH01] = 320/8,
        /* JDI */
        [MEMLCD_LPM013M126A] = 176/8*3,
        [MEMLCD_LPM027M128B] = 400/8*3,
};


void MEMLCD_BW_writepixel(MEMLCD_HandleTypeDef *hmemlcd, uint16_t x, uint16_t y, uint8_t color) {
    if (color) {
        hmemlcd->buffer[MEMLCD_line_length[hmemlcd->model]*y + x/8] |= 1 << (x&7);
    } else {
        hmemlcd->buffer[MEMLCD_line_length[hmemlcd->model]*y + x/8] &= ~(1 << (x&7));
    }
}

void MEMLCD_BW_blitline(MEMLCD_HandleTypeDef *hmemlcd, uint16_t x, uint16_t y, uint8_t *buff, uint16_t bx, uint16_t width) {
    uint8_t *line = &(hmemlcd->buffer[y*MEMLCD_line_length[hmemlcd->model]]);
    while (width) {
        if (buff[bx/8] & (1 << (bx&7))) {
            line[x/8] |= 1 << (x&7);
        } else {
            line[x/8] &= ~(1 << (x&7));
        }
        width -= 1;
        x += 1;
        bx += 1;
    }
}


void MEMLCD_init(MEMLCD_HandleTypeDef *hmemlcd) {
    hmemlcd->flags = MEMLCD_flags[hmemlcd->model];
    hmemlcd->line_ct = MEMLCD_line_count[hmemlcd->model];
    hmemlcd->line_len = MEMLCD_line_length[hmemlcd->model];

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

    hmemlcd->htim->Init.Period = 60000/MEMLCD_vcom_freq[hmemlcd->model];
    HAL_TIM_Base_Init(hmemlcd->htim);
    TIM_OC_InitTypeDef pwm_conf;
    pwm_conf.OCMode = TIM_OCMODE_PWM1;
    pwm_conf.Pulse = 100;
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

void MEMLCD_update_area(MEMLCD_HandleTypeDef *hmemlcd, uint16_t start, uint16_t end) {
    uint8_t cmd[2] = {1, 0};
    end = (end <= hmemlcd->line_ct)? end : hmemlcd->line_ct;
    start = (start <= end)? start : end;
    uint8_t *buffer = hmemlcd->buffer + (hmemlcd->line_len * start);
    HAL_GPIO_WritePin(hmemlcd->CS_Port, hmemlcd->CS_Pin, 1);
    for(uint16_t line = start+1; line < end+1; line++) {
        switch (hmemlcd->flags & 0xF) {
        case MEMLCD_ADDR_SHARP:
            cmd[0] = 1;
            cmd[1] = line & 0xff;
            break;
        case MEMLCD_ADDR_SHARP_LONG:
            cmd[0] = 1 | ((line<<6)&0xff);
            cmd[1] = (line>>2) & 0xff;
            break;
        case MEMLCD_ADDR_JDI: {
            uint32_t rev_addr = __RBIT(line) >> 22;
            cmd[0] = 1 | ((rev_addr<<6)&0xff);
            cmd[1] = (rev_addr>>2) & 0xff;
            break; }
        }
        HAL_SPI_Transmit(hmemlcd->hspi, cmd, 2, HAL_MAX_DELAY);
        HAL_SPI_Transmit(hmemlcd->hspi, buffer, hmemlcd->line_len, HAL_MAX_DELAY);
        buffer += hmemlcd->line_len;
    }
    cmd[0] = cmd[1] = 0;
    HAL_SPI_Transmit(hmemlcd->hspi, cmd, 2, 10);
    HAL_GPIO_WritePin(hmemlcd->CS_Port, hmemlcd->CS_Pin, 0);
}
