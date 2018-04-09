#ifndef __FLP_H
#define __FLP_H

#include <stdbool.h>
#include <stdint.h>

uint16_t FLP_get_current(void);
void FLP_set_current(uint16_t current);

void FLP_set_brightness(uint8_t new_brightness);
void FLP_change_brightness(int8_t amount);

bool FLP_is_on(void);
void FLP_on(void);
void FLP_off(void);
bool FLP_toggle(void);

#endif
