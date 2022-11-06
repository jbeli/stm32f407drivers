/*
 * stm32f407_exti.h
 *
 *  Created on: Nov 6, 2022
 *      Author: Ahmed
 */

#ifndef INC_STM32F407_EXTI_H_
#define INC_STM32F407_EXTI_H_


#include "stm32f407_gpio.h"
#include "stm32f407.h"

#define EXTI_LINE0                 EXTI_IMR1_IM0
#define EXTI_LINE1                 EXTI_IMR1_IM1
#define EXTI_LINE2                 EXTI_IMR1_IM2
#define EXTI_LINE3                 EXTI_IMR1_IM3
#define EXTI_LINE4                 EXTI_IMR1_IM4
#define EXTI_LINE5                 EXTI_IMR1_IM5
#define EXTI_LINE6                 EXTI_IMR1_IM6
#define EXTI_LINE7                 EXTI_IMR1_IM7
#define EXTI_LINE8                 EXTI_IMR1_IM8
#define EXTI_LINE9                 EXTI_IMR1_IM9
#define EXTI_LINE10                EXTI_IMR1_IM10
#define EXTI_LINE11                EXTI_IMR1_IM11
#define EXTI_LINE12                EXTI_IMR1_IM12
#define EXTI_LINE13                EXTI_IMR1_IM13
#define EXTI_LINE14                EXTI_IMR1_IM14
#define EXTI_LINE15                EXTI_IMR1_IM15
#define EXTI_LINE16                EXTI_IMR1_IM16
#define EXTI_LINE17                EXTI_IMR1_IM17
#define EXTI_LINE18                EXTI_IMR1_IM18
#define EXTI_LINE19                EXTI_IMR1_IM19
#define EXTI_LINE20                EXTI_IMR1_IM20
#define EXTI_LINE21                EXTI_IMR1_IM21
#define EXTI_LINE22                EXTI_IMR1_IM22
#define EXTI_LINE23                EXTI_IMR1_IM23

#define Interupt_Request_Masked		0
#define Interrupt_Request_NotMasked	1

#define Event_Request_Masked		0
#define Event_Request_NotMasked		1


void EXTI_MaskInterrupt(uint8_t line);
void EXTI_UnmaskInterrupt(uint8_t line);
void EXTI_MaskEvent(uint8_t line);
void EXTI_UnmaskEvent(uint8_t line);
void EXTI_SetRisingTrigger(uint8_t PinNumber) ;
void EXTI_SetFallingTrigger(uint8_t PinNumber);
void EXTI_SetRisingFallingTrigger(uint8_t PinNumber);
void EXTI_PendingReg(uint8_t line);

#endif /* INC_STM32F407_EXTI_H_ */
