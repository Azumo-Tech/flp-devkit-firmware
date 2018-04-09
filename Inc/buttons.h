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

#ifndef BUTTONS_H_
#define BUTTONS_H_

#include <stdint.h>
#include <stdbool.h>

#define BUTTON_COUNT 3
#define BUTTON_MAX_TIME 10000
#define BUTTON_DEBOUNCE_TIME 3

void BUTTON_poll(void);
int BUTTON_held_time(uint8_t button);
bool BUTTON_is_released(uint8_t button, uint16_t max_time);

#endif /* BUTTONS_H_ */
