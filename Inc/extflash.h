#ifndef __EXTFLASH_H
#define __EXTFLASH_H

typedef struct EXTFLASH_Handle {
    SPI_HandleTypeDef *hspi;

    GPIO_TypeDef *CS_Port;
    uint16_t CS_Pin;

    uint32_t size;
    uint32_t stride;
} EXTFLASH_HandleTypeDef;

void EXTFLASH_power_down(EXTFLASH_HandleTypeDef *hflash);

void EXTFLASH_power_up(EXTFLASH_HandleTypeDef *hflash);

void EXTFLASH_write_enable(EXTFLASH_HandleTypeDef *hflash);

void EXTFLASH_wait_for_busy(EXTFLASH_HandleTypeDef *hflash);

void EXTFLASH_write_aligned_page(EXTFLASH_HandleTypeDef *hflash, uint32_t addr, void *buffer, uint16_t size);

void EXTFLASH_read_screen(EXTFLASH_HandleTypeDef *hflash, uint8_t index, void *buffer, uint16_t bufsize);

void EXTFLASH_write_screen(EXTFLASH_HandleTypeDef *hflash, uint8_t index, void *buffer, uint16_t bufsize);

#endif
