#ifndef __MEMLCD_H
#define __MEMLCD_H

enum MEMLCD_Model {
	MEMLCD_SHARP_270,
	MEMLCD_SHARP_126,
	MEMLCD_JDI_270
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
} MEMLCD_HandleTypeDef;

void MEMLCD_init(MEMLCD_HandleTypeDef *hmemlcd);

void MEMLCD_power_off(MEMLCD_HandleTypeDef *hmemlcd);

void MEMLCD_clear_all(MEMLCD_HandleTypeDef *hmemlcd);

void MEMLCD_set_disp(MEMLCD_HandleTypeDef *hmemlcd, uint8_t state);

void MEMLCD_update_area(MEMLCD_HandleTypeDef *hmemlcd,
		uint8_t *buffer, uint8_t start, uint8_t end);

#endif
