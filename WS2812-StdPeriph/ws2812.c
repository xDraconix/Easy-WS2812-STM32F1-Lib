/*
 *  Project:	Lightwight WS2812 STM32 Lib
 *  Author:	DraconiX
 *
 *  File:	ws2812.c
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

#include <stm32f10x.h>
#include <stm32f10x_dma.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include "ws2812.h"


/* WS2812 Init function
 *
 * value:	Nothing
 * return:	Nothing
 */
void ws2812_init() {
	//Enabling Clocks
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	//Setting up the Pin (PB6) for PWM use - the Pin for the DIN at the WS2812
	GPIO_InitTypeDef pwm_out;
	pwm_out.GPIO_Pin = GPIO_Pin_6;
	pwm_out.GPIO_Speed = GPIO_Speed_50MHz;
	pwm_out.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &pwm_out);

	//Setting up the Timer, we use Timer4.
	TIM_TimeBaseInitTypeDef tim4;
	tim4.TIM_Prescaler = 8;
	tim4.TIM_Period = 9;
	tim4.TIM_ClockDivision = TIM_CKD_DIV1;
	tim4.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &tim4);

	// Init PWM & Output Compare Mode
	TIM_OCInitTypeDef pwm;
	pwm.TIM_OCMode = TIM_OCMode_PWM1;
	pwm.TIM_OutputState = TIM_OutputState_Enable;
	pwm.TIM_Pulse = 0;
	pwm.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OCStructInit(&pwm);
	TIM_OC1PreloadConfig(TIM4, 8);
	pwm.TIM_OCMode = TIM_OCMode_PWM1;
	pwm.TIM_OutputState = TIM_OutputState_Enable;

	TIM_OC1Init(TIM4, &pwm);

	// Init the DMA Channel Memory -> Timer4.CompareRegister
	DMA_InitTypeDef dma;
	dma.DMA_PeripheralBaseAddr = (uint32_t)&TIM4->CCR1;
	dma.DMA_MemoryBaseAddr = (uint32_t)(&ws2812_buffer[0]);
	dma.DMA_DIR = DMA_DIR_PeripheralDST;
	dma.DMA_BufferSize = BUFFER_SIZE;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize =  DMA_PeripheralDataSize_HalfWord;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_Mode = DMA_Mode_Circular;
	dma.DMA_Priority = DMA_Priority_High;
	dma.DMA_M2M = DMA_M2M_Disable;

	DMA_Init(DMA1_Channel1, &dma);
	TIM_DMACmd(TIM4, TIM_DMA_CC1, ENABLE);
	
	DMA_Cmd(DMA1_Channel1, ENABLE);
	TIM_Cmd(TIM4, ENABLE);


#ifdef FREERUNNING
	// Set to Freeruning Mode (see ws2812.h)
	DMA_Cmd(DMA1_Channel1, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
#endif
}

/* WS2812 Refresh function
 * it you dont use the freeruning Mode,
 * you MUST refresh the WS2812 manualy.
 *
 * value:	Nothing
 * return:	Nothing
 */
void ws2812_Refresh()
{
	   DMA_Cmd(DMA1_Channel1, ENABLE);
	    TIM_Cmd(TIM4, ENABLE);
	     while(!DMA_GetFlagStatus(DMA1_FLAG_TC1));
	    TIM_Cmd(TIM4, DISABLE);
	   DMA_Cmd(DMA1_Channel1, DISABLE);
	   DMA_ClearFlag(DMA1_FLAG_TC1);
}

/* Get LED Red Color function
 *
 * value:	LedPos - the LED Position in the String
 * return:	Red Color Value - Range 0-255
 */
uint8_t Get_Led_Red(uint16_t LedPos)
{
	uint8_t Red = 0;
	uint16_t pos = (LedPos*24)+8;

	if(ws2812_buffer[pos+0] == CMPH) Red |= (1<<7);
	if(ws2812_buffer[pos+1] == CMPH) Red |= (1<<6);
	if(ws2812_buffer[pos+2] == CMPH) Red |= (1<<5);
	if(ws2812_buffer[pos+3] == CMPH) Red |= (1<<4);
	if(ws2812_buffer[pos+4] == CMPH) Red |= (1<<3);
	if(ws2812_buffer[pos+5] == CMPH) Red |= (1<<2);
	if(ws2812_buffer[pos+6] == CMPH) Red |= (1<<1);
	if(ws2812_buffer[pos+7] == CMPH) Red |= (1<<0);

	return Red;
}

/* Get LED Green Color function
 *
 * value:	LedPos - the LED Position in the String
 * return:	Green Color Value - Range 0-255
 */
uint8_t Get_Led_Green(uint16_t LedPos)
{
	uint8_t Color = 0;
	uint16_t pos = LedPos*24;

	if(ws2812_buffer[pos+0] == CMPH) Color |= (1<<7);
	if(ws2812_buffer[pos+1] == CMPH) Color |= (1<<6);
	if(ws2812_buffer[pos+2] == CMPH) Color |= (1<<5);
	if(ws2812_buffer[pos+3] == CMPH) Color |= (1<<4);
	if(ws2812_buffer[pos+4] == CMPH) Color |= (1<<3);
	if(ws2812_buffer[pos+5] == CMPH) Color |= (1<<2);
	if(ws2812_buffer[pos+6] == CMPH) Color |= (1<<1);
	if(ws2812_buffer[pos+7] == CMPH) Color |= (1<<0);

	return Color;
}

/* Get LED Blue Color function
 *
 * value:	LedPos - the LED Position in the String
 * return:	Blue Color Value - Range 0-255
 */
uint8_t Get_Led_Blue(uint16_t LedPos)
{
	uint8_t Color = 0;
	uint16_t pos = (LedPos*24)+16;

	if(ws2812_buffer[pos+0] == CMPH) Color |= (1<<7);
	if(ws2812_buffer[pos+1] == CMPH) Color |= (1<<6);
	if(ws2812_buffer[pos+2] == CMPH) Color |= (1<<5);
	if(ws2812_buffer[pos+3] == CMPH) Color |= (1<<4);
	if(ws2812_buffer[pos+4] == CMPH) Color |= (1<<3);
	if(ws2812_buffer[pos+5] == CMPH) Color |= (1<<2);
	if(ws2812_buffer[pos+6] == CMPH) Color |= (1<<1);
	if(ws2812_buffer[pos+7] == CMPH) Color |= (1<<0);

	return Color;
}

/* Set Led to Color
 *
 * value:	LedPos 	- the LED Position in the String
 * 		SetR	- the Red RGB Value
 * 		SetG	- the Green RGB Value
 * 		SetB	- the Blue RGB Value
 * return:	Nothing
 */
void Set_Led(uint32_t LedPos,uint16_t SetR, uint16_t SetG, uint16_t SetB)
{
	// Calc Array Position
	uint32_t PosG = LedPos*24;
	uint32_t PosR = PosG+8;
	uint32_t PosB = PosG+16;

	// Setting up the Green Color (the WS281 is not RGB, it is GRB)
	ws2812_buffer[PosG+0] = (SetG & 0x80) ? CMPH:CMPL;
	ws2812_buffer[PosG+1] = (SetG & 0x40) ? CMPH:CMPL;
	ws2812_buffer[PosG+2] = (SetG & 0x20) ? CMPH:CMPL;
	ws2812_buffer[PosG+3] = (SetG & 0x10) ? CMPH:CMPL;
	ws2812_buffer[PosG+4] = (SetG & 0x08) ? CMPH:CMPL;
	ws2812_buffer[PosG+5] = (SetG & 0x04) ? CMPH:CMPL;
	ws2812_buffer[PosG+6] = (SetG & 0x02) ? CMPH:CMPL;
	ws2812_buffer[PosG+7] = (SetG & 0x01) ? CMPH:CMPL;

	// Setting up the Red Color (the WS281 is not RGB, it is GRB)
	ws2812_buffer[PosR+0] = (SetR & 0x80) ? CMPH:CMPL;
	ws2812_buffer[PosR+1] = (SetR & 0x40) ? CMPH:CMPL;
	ws2812_buffer[PosR+2] = (SetR & 0x20) ? CMPH:CMPL;
	ws2812_buffer[PosR+3] = (SetR & 0x10) ? CMPH:CMPL;
	ws2812_buffer[PosR+4] = (SetR & 0x08) ? CMPH:CMPL;
	ws2812_buffer[PosR+5] = (SetR & 0x04) ? CMPH:CMPL;
	ws2812_buffer[PosR+6] = (SetR & 0x02) ? CMPH:CMPL;
	ws2812_buffer[PosR+7] = (SetR & 0x01) ? CMPH:CMPL;

	// Setting up the Blue Color (the WS281 is not RGB, it is GRB)
	ws2812_buffer[PosB+0] = (SetB & 0x80) ? CMPH:CMPL;
	ws2812_buffer[PosB+1] = (SetB & 0x40) ? CMPH:CMPL;
	ws2812_buffer[PosB+2] = (SetB & 0x20) ? CMPH:CMPL;
	ws2812_buffer[PosB+3] = (SetB & 0x10) ? CMPH:CMPL;
	ws2812_buffer[PosB+4] = (SetB & 0x08) ? CMPH:CMPL;
	ws2812_buffer[PosB+5] = (SetB & 0x04) ? CMPH:CMPL;
	ws2812_buffer[PosB+6] = (SetB & 0x02) ? CMPH:CMPL;
	ws2812_buffer[PosB+7] = (SetB & 0x01) ? CMPH:CMPL;
}
