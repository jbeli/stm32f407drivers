/*
 * stm32f407_gpio.c
 *
 *  Created on: Nov 6, 2022
 *      Author: Ahmed
 */


#include "stm32f407_gpio.h"
#include "stm32f407_exti.h"
#include "stm32f407_sysconf.h"


/*
 * GPIO_PeriClockControl(&GPIOA, Enable); // GPIO is defined as address
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}

	else if(EnorDi == DISABLE)
	{
			// pass Diasble periph
	}
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{

	uint32_t temp = 0 ;

	// 1. configure the mode of the gpio pin

	// 2. configure the speed

	// 3. configure the pupd settings

	// 4. configure the optype

	// 5. configure the alt functionality

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)) ;
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
		pGPIOHandle->pGPIOx->MODER |= temp ; // setting


	}
	else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. configure FTSR
			//EXTI->FTSR |=(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//EXTI->RTSR &= ~(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // make sure to clear the corresponding RTSR bit
			EXTI_SetRisingTrigger(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) ;


		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. Configure RTSR
			//EXTI->RTSR |=(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//EXTI->FTSR &= ~(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // make sure to clear the corresponding FTSR bit
			EXTI_SetFallingTrigger(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) ;
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1. Configure both FTSR and RTSR
			//EXTI->FTSR |=(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//EXTI->RTSR |=(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI_SetRisingFallingTrigger(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) ;

		}

		//2. Configure the GPIO port selection in SYSGFG_EXTICR
		/*uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode /4 ;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode %4 ;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSGFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode <<(temp2*4);*/
		//SYSCFG_SetExternalInt(&pGPIOHandle);

		//3 enable exti interrupt delivery using IMR
		//EXTI->IMR |= (1 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) ;
	}
	temp = 0 ;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp ;

	temp = 0 ;

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp ;

	temp = 0 ;

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp ;

	temp = 0;

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		// TODO configure alternate function
		uint32_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber /8 ;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber %8 ;
		if(temp1 == 0)
		{
			pGPIOHandle->pGPIOx->AFRL &=~ (0xF<< (4 * temp2)); // clearing
			pGPIOHandle->pGPIOx->AFRL |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode<< (4 * temp2));
		}
		else if (temp1 == 1)
		{
			pGPIOHandle->pGPIOx->AFRH |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode<< (4 * temp2));
		}

	}

}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value ;
	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001) ;
	return value ;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value ;
	value = (uint16_t)pGPIOx->IDR  ;
	return value ;
}

void GPIO_WriteToInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if (value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1<<PinNumber) ;
	}
	else
	{
		pGPIOx->ODR &=~ (1<<PinNumber) ;
	}
}

void GPIO_WriiteToInputPort(GPIO_RegDef_t *pGPIOx, uint8_t value)
{
	pGPIOx->ODR = value ;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^=(1<<PinNumber);
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	// clear the exti register corresponding to the pin number
	if (EXTI->PR &(1 << PinNumber))
	{
		// clear
		EXTI->PR |= (1<<PinNumber);
	}
}
