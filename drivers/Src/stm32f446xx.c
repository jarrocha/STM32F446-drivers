/*
 * stm32f446xx.c
 *
 *  Created on: Dec 8, 2021
 *      Author: jaime
 */
#include "stm32f446xx.h"

uint32_t TestBit(uint32_t reg, uint32_t pos)
{
	return (reg & (1 << pos));
}

void Delay(uint32_t waitNum)
{
	for (uint32_t i = 0; i < waitNum; i++);
}