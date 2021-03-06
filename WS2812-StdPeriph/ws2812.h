/*
 *  Project:	Lightwight WS2812 STM32 Lib
 *  Author:	DraconiX
 *
 *  File:	ws2812.h
 *  Version:	V1.0
 *  Header:	StdPeripherial (STM)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef __WS2812_H__
#define __WS2812_H__

// The Timervalues - on Nucleo 64Mhz 8/2
#define CMPH 8
#define CMPL 2

/* Freerunning Mode
 * normaly, the Timer sends permanently to the WS2812
 * to manualy update the WS2812 strip, uncomment this
 * and use the function ws2812_Refresh() to send the
 * data out.
 */
#define FREERUNNING


/* Number of LEDs at the String
 * Remember that the Number of LED is limited
 * to the maximum amount of RAM. The STM32F103RB
 * mounted at the Nucleo-F103B has 20kb of usable RAM.
 *
 * We use 24 - 8bit values to store the RGB Data in it.
 * Sure, its a waste of RAM :-D But therefore, it is easy
 * to use. 
 *
 * With the STM32F103 it is possible to drive ~800 LEDs.
 * This are 13 Meters of LED-Strip.
 */
#define NUM_LED 20
#define BUFFER_SIZE (((NUM_LED+1)*24) + 200)
uint8_t ws2812_buffer[BUFFER_SIZE];


void ws2812_Refresh	(void);

uint8_t Get_Led_Red	(uint16_t LedPos);
uint8_t Get_Led_Green	(uint16_t LedPos);
uint8_t Get_Led_Blue	(uint16_t LedPos);

void    Set_Led         (uint32_t LedPos,
			 uint16_t SetR,
			 uint16_t SetG,
			 uint16_t SetB);

void 	ws2812_init	(void);
