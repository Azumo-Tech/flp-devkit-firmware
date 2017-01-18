#ifndef __MEMLCD_H
#define __MEMLCD_H

enum MEMLCD_Model {
	SHARP_270,
	SHARP_126,
	JDI_270
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
} MEMLCD_HandleTypeDef;

void MEMLCD_init(MEMLCD_HandleTypeDef *hmemlcd);

void MEMLCD_clear_all(MEMLCD_HandleTypeDef *hmemlcd);

#endif
