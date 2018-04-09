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
#include "buttons.h"

static int button_state[BUTTON_COUNT];
static const uint16_t button_pin[BUTTON_COUNT] = {BT1_Pin, BT2_Pin, BT3_Pin};

void BUTTON_poll() {
	for (int i=0; i<BUTTON_COUNT; i++) {
		if (!HAL_GPIO_ReadPin(BT1_GPIO_Port, button_pin[i])) {
			if (button_state[i] < 0) {
				button_state[i] = 0;
			} else if (button_state[i] < BUTTON_MAX_TIME) {
				button_state[i]++;
			}
		} else {
			if (button_state[i] > 0)
				button_state[i] = -button_state[i];
			else {
				button_state[i] = 0;
			}
		}
	}
}

int BUTTON_held_time(uint8_t button) {
	button = button-1;
	if (button < BUTTON_COUNT && button_state[button] > 0) {
		return button_state[button];
	} else {
		return 0;
	}
}

bool BUTTON_is_released(uint8_t button, uint16_t max_time) {
	button = button-1;
	if (button < BUTTON_COUNT) {
		return (button_state[button] < -BUTTON_DEBOUNCE_TIME
				&& button_state[button] >= -max_time);
	} else {
		return false;
	}
}
