#ifndef __LED_H
#define __LED_H

void LED_set_current(uint16_t current);

void LED_change_brightness(int8_t amount);

uint16_t LED_get_current();

#endif
