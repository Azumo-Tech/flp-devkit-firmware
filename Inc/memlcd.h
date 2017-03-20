#ifndef __MEMLCD_H
#define __MEMLCD_H

enum MEMLCD_Model {
	/* Sharp */
	MEMLCD_LS027B7DH01,
	MEMLCD_LS013B7DH05,
	/* JDI */
	MEMLCD_LPM027M128B,
	MEMLCD_LPM013M126A
};

static const uint8_t MEMLCD_line_count[] = {
		/* Sharp */
		[MEMLCD_LS027B7DH01] = 240,
		[MEMLCD_LS013B7DH05] = 168,
		/* JDI */
		[MEMLCD_LPM027M128B] = 240,
		[MEMLCD_LPM013M126A] = 176,
};

static const uint8_t MEMLCD_line_length[] = {
		/* Sharp */
		[MEMLCD_LS027B7DH01] = 400/8,
		[MEMLCD_LS013B7DH05] = 144/8,
		/* JDI */
		[MEMLCD_LPM027M128B] = 400/8*3,
		[MEMLCD_LPM013M126A] = 176/8*3,
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
	 volatile uint8_t buffer[240*150];
} MEMLCD_HandleTypeDef;

void MEMLCD_init(MEMLCD_HandleTypeDef *hmemlcd);

void MEMLCD_power_off(MEMLCD_HandleTypeDef *hmemlcd);

void MEMLCD_clear_all(MEMLCD_HandleTypeDef *hmemlcd);

void MEMLCD_set_disp(MEMLCD_HandleTypeDef *hmemlcd, uint8_t state);

void MEMLCD_update_area(MEMLCD_HandleTypeDef *hmemlcd, uint8_t start, uint8_t end);

static inline int MEMLCD_bufsize(MEMLCD_HandleTypeDef *hmemlcd) {
	return MEMLCD_line_count[hmemlcd->model] * MEMLCD_line_length[hmemlcd->model];
}

#endif
