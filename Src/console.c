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

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "memlcd.h"

#include "font8x16.xbm"
#include "font8x16_transp.xbm"

extern MEMLCD_HandleTypeDef hmemlcd;

static uint8_t tilemap[1408];


void printxy(int x, int y, char* format, ...) {
	va_list varargs;
	va_start(varargs, format);
	vsprintf((char *)&tilemap[hmemlcd.tilemaps[0].width*(y)+(x)], format, varargs);
	va_end(varargs);
}

void clrscr() {
	memset(tilemap, 0, sizeof(tilemap));
}

void CON_init() {
	// Set up the default tile maps, common to both orientations
	// Background text
	hmemlcd.tilemaps[0].scroll_x = 0;
	hmemlcd.tilemaps[0].scroll_y = 0;
	// Overlay (for LED current)
	hmemlcd.tilemaps[1].height = 3;
	hmemlcd.tilemaps[1].width = 16;
	// Set up attributes that depend on orientation
	if (hmemlcd.flags & MEMLCD_ROT270) {
		// Background text
		hmemlcd.tilemaps[0].height = hmemlcd.width/16;
		hmemlcd.tilemaps[0].width = hmemlcd.height/8;
		hmemlcd.tilemaps[0].tile_size = 1 | 0<<2;
		hmemlcd.tilemaps[0].tiles = font8x16_transp_bits;
		hmemlcd.tilemaps[0].flags = TILE_TRANSPOSE;
		// Overlay (for LED current)
		hmemlcd.tilemaps[1].tile_size = 1 | 0<<2;
		hmemlcd.tilemaps[1].scroll_x = (hmemlcd.width-48)/2;
		hmemlcd.tilemaps[1].scroll_y = (hmemlcd.height-128)/2;
		hmemlcd.tilemaps[1].tiles = font8x16_transp_bits;
		hmemlcd.tilemaps[1].flags = TILE_TRANSPOSE;
	} else {
		// Background text
		hmemlcd.tilemaps[0].height = hmemlcd.height/16;
		hmemlcd.tilemaps[0].width = hmemlcd.width/8;
		hmemlcd.tilemaps[0].tile_size = 0 | 1<<2;
		hmemlcd.tilemaps[0].tiles = font8x16_bits;
		hmemlcd.tilemaps[0].flags = 0;
		// Overlay (for LED current)
		hmemlcd.tilemaps[1].tile_size = 0 | 1<<2;
		hmemlcd.tilemaps[1].scroll_x = (hmemlcd.width-128)/2;
		hmemlcd.tilemaps[1].scroll_y = (hmemlcd.height-48)/2;
		hmemlcd.tilemaps[1].tiles = font8x16_bits;
		hmemlcd.tilemaps[1].flags = 0;
	}
	hmemlcd.tilemaps[0].map = tilemap;
	hmemlcd.tilemaps[1].map = NULL;
}

