#ifndef __MEMLCD_H
#define __MEMLCD_H

enum MEMLCD_Model {
    /* Sharp */
    MEMLCD_LS012B7DH02,
    MEMLCD_LS013B7DH05,
    MEMLCD_LS027B7DH01,
    MEMLCD_LS032B7DD02,
    MEMLCD_LS044Q7DH01,
    /* JDI */
    MEMLCD_LPM013M126A,
    MEMLCD_LPM027M128B,
};

enum MEMLCD_Flags {
    MEMLCD_ADDR_SHARP = 0x0,
    MEMLCD_ADDR_SHARP_LONG = 0x1,
    MEMLCD_ADDR_JDI = 0x2,
    MEMLCD_PWR_5V = 1 << 4,
    MEMLCD_PWR_3V = 0 << 4,
    MEMLCD_MONO = 0 << 5,
    MEMLCD_RGB = 1 << 5,
};

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

typedef struct MEMLCD_Handle {
	enum MEMLCD_Model model;

	SPI_HandleTypeDef *hspi;

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

	 uint8_t flags;
	 uint8_t line_len;
	 uint16_t line_ct;

	 uint8_t buffer[240*150];
} MEMLCD_HandleTypeDef;

void MEMLCD_init(MEMLCD_HandleTypeDef *hmemlcd);

void MEMLCD_power_off(MEMLCD_HandleTypeDef *hmemlcd);

void MEMLCD_clear_all(MEMLCD_HandleTypeDef *hmemlcd);

void MEMLCD_set_disp(MEMLCD_HandleTypeDef *hmemlcd, uint8_t state);

void MEMLCD_update_area(MEMLCD_HandleTypeDef *hmemlcd, uint16_t start, uint16_t end);

void MEMLCD_BW_blitline(MEMLCD_HandleTypeDef *hmemlcd, uint16_t x, uint16_t y, uint8_t *buff, uint16_t bx, uint16_t width);

static inline int MEMLCD_bufsize(MEMLCD_HandleTypeDef *hmemlcd) {
	return MEMLCD_line_count[hmemlcd->model] * MEMLCD_line_length[hmemlcd->model];
}

#endif
