#ifndef __MEMLCD_H
#define __MEMLCD_H

#include "tim.h"
#include "spi.h"
#include "gpio.h"

extern const int MEMLCD_max_model;

enum MEMLCD_Flags {
    MEMLCD_ADDR_SHARP = 0x0,
    MEMLCD_ADDR_SHARP_LONG = 0x1,
    MEMLCD_ADDR_JDI = 0x2,
    MEMLCD_PWR_5V = 1 << 4,
    MEMLCD_PWR_3V = 0 << 4,
    MEMLCD_MONO = 0 << 5,
    MEMLCD_RGB = 1 << 5,
    MEMLCD_VFLIP = 1 << 6,
    MEMLCD_HFLIP = 1 << 7,
    MEMLCD_ROT270 = 1<< 9,
};

enum MEMLCD_tile_Flags {
    TILE_TRANSPOSE = 0x1,
};

struct TileLayer {
    uint8_t tile_size;
    uint8_t width;
    uint8_t height;
    uint8_t flags;
    int16_t scroll_x;
    int16_t scroll_y;
    uint8_t *tiles;
    uint8_t *map;
    uint8_t *attrmap;
};

typedef struct MEMLCD_Handle {
    uint8_t model;

    SPI_HandleTypeDef *hspi;

    TIM_HandleTypeDef *htim;
    uint32_t tim_ch;

    GPIO_TypeDef *CS_Port;
    uint16_t CS_Pin;
    GPIO_TypeDef *DISP_Port;
    uint16_t DISP_Pin;
    GPIO_TypeDef *EXTMODE_Port;
    uint16_t EXTMODE_Pin;
    GPIO_TypeDef *EXTCOM_Port;
    uint16_t EXTCOM_Pin;
    GPIO_TypeDef *BOOST_Port;
    uint16_t BOOST_Pin;

    uint16_t flags;
    uint8_t line_len;
    uint16_t line_ct;
    uint16_t width, height;

    uint8_t linebuf[2][256];
    uint8_t buffer[240*150];
    struct TileLayer tilemaps[3];
} MEMLCD_HandleTypeDef;

void MEMLCD_init(MEMLCD_HandleTypeDef *hmemlcd);

void MEMLCD_power_off(MEMLCD_HandleTypeDef *hmemlcd);

void MEMLCD_clear_all(MEMLCD_HandleTypeDef *hmemlcd);

void MEMLCD_set_disp(MEMLCD_HandleTypeDef *hmemlcd, uint8_t state);

void MEMLCD_update_area(MEMLCD_HandleTypeDef *hmemlcd, uint16_t start, uint16_t end);

int MEMLCD_busy();

const char * MEMLCD_get_model_name(MEMLCD_HandleTypeDef *hmemlcd);
void MEMLCD_set_model_by_name(MEMLCD_HandleTypeDef *hmemlcd, char* name);


static inline int MEMLCD_bufsize(MEMLCD_HandleTypeDef *hmemlcd) {
    return hmemlcd->line_ct * hmemlcd->line_len;
}

#endif
