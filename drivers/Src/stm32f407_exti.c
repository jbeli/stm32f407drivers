/*
 * stm32f407_exti.c
 *
 *  Created on: Nov 6, 2022
 *      Author: Ahmed
 */

#include "stm32f407_exti.h"

void EXTI_MaskInterrupt(uint8_t line)
{
	EXTI->IMR &=~(Interrupt_Request_NotMasked<<line) ;
	EXTI->IMR |= (Interupt_Request_Masked<< line);
}

void EXTI_UnmaskInterrupt(uint8_t line)
{
	EXTI->IMR |= (Interrupt_Request_NotMasked<<line);
}

void EXTI_MaskEvent(uint8_t line)
{
	EXTI->IMR &=~(Event_Request_NotMasked<<line) ;
	EXTI->IMR |= (Event_Request_Masked<< line);
}

void EXTI_UnmaskEvent(uint8_t line)
{
	EXTI->IMR |= (Event_Request_NotMasked<<line);
}

void EXTI_SetRisingTrigger(uint8_t PinNumber)
{
	EXTI->RTSR |=(1<< PinNumber);
	EXTI->FTSR &= ~(1<< PinNumber);
}

void EXTI_SetFallingTrigger(uint8_t PinNumber)
{
	EXTI->FTSR |=(1<< PinNumber);
	EXTI->RTSR &= ~(1<< PinNumber);
}

void EXTI_SetRisingFallingTrigger(uint8_t PinNumber)
{
	EXTI->FTSR |=(1<< PinNumber);
	EXTI->RTSR |=(1<< PinNumber);
}

void EXTI_PendingReg(uint8_t line)
{
	if(EXTI->PR &(1<<line))
	{
		//static uint8_t flag  = 1 ;
		EXTI->PR |=(1< line);
	}
}

