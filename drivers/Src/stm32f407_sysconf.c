/*
 * stm32f407_sysconf.c
 *
 *  Created on: Nov 6, 2022
 *      Author: Ahmed
 */

#include "stm32f407_sysconf.h"


void SYSCFG_MemoryRemapConfig(uint8_t SYSCFG_MemoryRemap)
{
	SYSCFG->MEMRMP &= ~(0x03) ; // clear bits first
	SYSCFG->MEMRMP |= SYSCFG_MemoryRemap ;
}

void SYSCFG_SetPeriphMode(uint8_t MII_RMII_SEL)
{
	if(MII_RMII_SEL == RMII_Interface)
	{
		SYSCFG->PMC |= (MII_RMII_SEL<<23) ;
	}
	else
	{
		SYSCFG->PMC &= (MII_RMII_SEL<<23) ;
	}

}

void SYSCFG_SetClockPeriph()
{
	RCC->APB2ENR |= (SET<<14) ;
}
void SYSCFG_SetExternalInt(GPIO_Handle_t *pGPIOHandle)
{
	uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4 ;
	uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4 ;
	uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
	SYSCFG->EXTICR[temp1] = portcode <<(temp2*4);
}

void SYSCFG_SetCompensationCellPowerDown(uint8_t CMP_PD)
{
	SYSCFG->CMPCR &=~(0x01 <<0) ;
	SYSCFG->CMPCR |= (CMP_PD<<0);
}
uint8_t SYSCFG_GetCompensationCellReadyFlag()
{
	uint8_t val ;
	val = (SYSCFG->CMPCR)>> 8 ;
	return val ;
}

uint8_t SYSCFG_GetMemoryRemap()
{
	uint8_t val ;
	val = (SYSCFG->MEMRMP)>>0 ;
	return val ;

}

uint8_t SYSCFG_GetPeriphMode()
{
	uint8_t val ;
	val = (SYSCFG->PMC)>>23 ;
	return val ;
}

