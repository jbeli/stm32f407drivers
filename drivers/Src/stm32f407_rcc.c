/*
 * stm32f407_rcc.c
 *
 *  Created on: Nov 6, 2022
 *      Author: Ahmed
 */


#include "stm32f407_rcc.h"


void RCC_HSIConfig(HSI_State state)
{
	if (state == RCC_HSI_ON)
	{
		RCC->CR |= RCC_CR_HSION ;
		while(!((RCC-> CR)& RCC_CR_HSIRDY_Pos)>> (RCC_CR_HSIRDY_Pos)) ;
		//!((RCC->CR )>> RCC_CR_HSIRDY_Pos) & RCC_CR_HSIRDY)

	} else {
		RCC->CR &= ~ RCC_CR_HSION ;
		//while((RCC->CR & RCC_CR_HSERDY)) ;

	}

}

void RCC_SetPrescaler(RCC_CLKInitTypeDef RCC_AxB_DIV)
{
	RCC->CFGR |= (RCC_AxB_DIV.AHBdiv << RCC_CFGR_HPRE_Pos);
	RCC->CFGR |= (RCC_AxB_DIV.APB1div <<RCC_CFGR_PPRE1_Pos) ;
	RCC->CFGR |= (RCC_AxB_DIV.APB2div << RCC_CFGR_PPRE2_Pos);
}
void RCC_DeInitPrescaler()
{
	RCC->CFGR &= ~(RCC_CFGR_HPRE_Msk << RCC_CFGR_HPRE_Pos);
	RCC->CFGR &= ~(RCC_CFGR_PPRE1_Msk <<RCC_CFGR_PPRE1_Pos) ;
	RCC->CFGR &= ~(RCC_CFGR_PPRE2_Msk << RCC_CFGR_PPRE2_Pos);
}

void RCC_SetSystemClockSwitch(uint8_t selected_sysclk)
{
	uint8_t mask = (selected_sysclk << RCC_CFGR_SW_Pos );
	RCC->CFGR &=~ (RCC_CFGR_SW_Msk << RCC_CFGR_SW_Pos) ;
	RCC->CFGR |= (selected_sysclk << RCC_CFGR_SW_Pos) ;
	while(((RCC->CFGR) & mask) != mask ) ;

}

RCC_OSC_Source RCC_GetOscillatorSoucre(void)
{
	uint32_t temp ;
	temp = (RCC->CFGR >> RCC_CFGR_SWS0_Pos) & 0x03 ;
	return temp ;
}

void RCC_GetSYSCLK(void)
{
	if (RCC_GetOscillatorSoucre() ==  RCC_OSC_HSE)
	{
		//val =  HSI_VALUE ;
	}
	else if (RCC_GetOscillatorSoucre() ==  RCC_OSC_HSI)
	{
		#define HSI_VALUE             ((uint32_t)16000000U)
		#define SYSCLK			HSI_VALUE
	}

}

int RCC_GetHCLK(RCC_CLKInitTypeDef RCC_AxB_DIV)
{
	return (SYSCLK / RCC_AxB_DIV.AHBdiv) ;
}
float RCC_GetPLCLK1(RCC_CLKInitTypeDef RCC_AxB_DIV)
{
	return (RCC_GetHCLK(RCC_AxB_DIV) / RCC_AxB_DIV.APB1div) ;
}
float RCC_GetPLCLK2(RCC_CLKInitTypeDef RCC_AxB_DIV)
{
	return (RCC_GetHCLK(RCC_AxB_DIV) / RCC_AxB_DIV.APB2div) ;
}


