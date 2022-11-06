/*
 * arm4.c
 *
 *  Created on: Nov 6, 2022
 *      Author: Ahmed
 */

#include "arm4.h"


void NVIC_EnableIRQ(IRQn_Type IRQNumber)
{
	if (IRQNumber <= 31)
	{
		*NVIC_ISER0 |= (1<< IRQNumber);
	}
	else if (IRQNumber >31 && IRQNumber < 64)
	{
		*NVIC_ISER1 |= (1<< (IRQNumber%32));
	}
	else if (IRQNumber >= 64 && IRQNumber < 96)
	{
		*NVIC_ISER2 |= (1<< (IRQNumber%64));
			// program ISER2 register no need to finish the rest we have 81 interrupts
	}
}

void NVIC_DisableIRQ(IRQn_Type IRQNumber)
{
	if (IRQNumber <=31)
	{
		*NVIC_ICER0 |= (1<< IRQNumber);
	}
	else if (IRQNumber >31 && IRQNumber < 64)
	{
		*NVIC_ICER1 |= (1<< (IRQNumber%32));
	}
	else if (IRQNumber >= 64 && IRQNumber < 96)
	{
		*NVIC_ICER2 |= (1<< (IRQNumber%64));
	}

}

void NVIC_SetPending(IRQn_Type IRQNumber)
{
	if (IRQNumber <= 31)
	{
		*NVIC_ISPR0 |= (1<< IRQNumber);
	}
	else if (IRQNumber >31 && IRQNumber < 64)
	{
		*NVIC_ISPR1 |= (1<< (IRQNumber%32));
	}
	else if (IRQNumber >= 64 && IRQNumber < 96)
	{
		*NVIC_ISPR2 |= (1<< (IRQNumber%64));
			// program ISER2 register no need to finish the rest we have 81 interrupts
	}
}

void NVIC_ClearPending(IRQn_Type IRQNumber)
{
	if (IRQNumber <=31)
	{
		*NVIC_ICPR0 |= (1<< IRQNumber);
	}
	else if (IRQNumber >31 && IRQNumber < 64)
	{
		*NVIC_ICPR1 |= (1<< (IRQNumber%32));
	}
	else if (IRQNumber >= 64 && IRQNumber < 96)
	{
		*NVIC_ICPR2 |= (1<< (IRQNumber%64));
	}

}

uint8_t NVIC_GetIRQPdStatus(IRQn_Type IRQNumber)
{
	uint8_t regVal ;
	if (IRQNumber <= 31)
	{
		regVal =  (((uint32_t)(*NVIC_IABR0)>> IRQNumber) & 0x01);
	}
	else if (IRQNumber >31 && IRQNumber < 64)
	{
		regVal =  (((uint32_t)(*NVIC_IABR1)>> IRQNumber) & 0x01);
	}
	else if (IRQNumber >= 64 && IRQNumber < 96)
	{
		regVal =  (((uint32_t)(*NVIC_IABR2)>> IRQNumber) & 0x01);
					// program ISER2 register no need to finish the rest we have 81 interrupts
	}
	return regVal ;
}

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if (IRQNumber <=31)
		{
			// program ISER0 register
			*NVIC_ISER0 |= (1<< IRQNumber);
		}
		else if (IRQNumber >31 && IRQNumber < 64)
		{
			// program ISER1 register
			*NVIC_ISER1 |= (1<< (IRQNumber%32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ISER2 |= (1<< (IRQNumber%64));
			// program ISER2 register
		}

	}else
	{
		if (IRQNumber <=31)
		{
			*NVIC_ICER0 |= (1<< IRQNumber);
		}
		else if (IRQNumber >31 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= (1<< (IRQNumber%32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ICER2 |= (1<< (IRQNumber%64));
		}

	}

}

void NVIC_SetPriority(IRQn_Type IRQNumber, NVIC_IRQ_PRIO IRQPriority)
{
	uint8_t IRP = IRQNumber / 4 ;
	uint8_t byteoffset = IRQNumber % 4 ;
	uint8_t shift_amount = (8 * byteoffset) + (8 - NO_PR_BITS_IMPLEMENTED) ;
	*(NVIC_IPR0 + (IRP*4)) |= (IRQPriority << shift_amount);
}

uint8_t NVIC_GetPriority(IRQn_Type IRQNumber)
{
	uint8_t prio ;
	uint8_t IRP = IRQNumber / 4 ;
	uint8_t byteoffset = IRQNumber % 4 ;
	//uint8_t shift_amount = (8 * byteoffset) + (8 - NO_PR_BITS_IMPLEMENTED) ;
	prio = ((((uint8_t)*(NVIC_IPR0 + (IRP*4))) >> byteoffset) & 0x0001 );
	return prio ;
}

