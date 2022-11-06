/*
 * stm32f407_gpio.h
 *
 *  Created on: Nov 6, 2022
 *      Author: Ahmed
 */

#ifndef STM32F407_GPIO_H_
#define STM32F407_GPIO_H_

#include "stm32f407.h"

#define GPIO_PIN_0                 0
#define GPIO_PIN_1                 1
#define GPIO_PIN_2                 2
#define GPIO_PIN_3                 3
#define GPIO_PIN_4                 4
#define GPIO_PIN_5                 5
#define GPIO_PIN_6                 6
#define GPIO_PIN_7                 7
#define GPIO_PIN_8                 8
#define GPIO_PIN_9                 9
#define GPIO_PIN_10                10
#define GPIO_PIN_11                11
#define GPIO_PIN_12                12
#define GPIO_PIN_13                13
#define GPIO_PIN_14                14
#define GPIO_PIN_15                15


typedef struct
{
	uint8_t GPIO_PinNumber;		/*!< possible values from @GPIO_PIN_NUMBERS>*/
	uint8_t GPIO_PinMode;		/*!< possible values from @GPIO_PIN_MODES>*/
	uint8_t GPIO_PinOPType;		/*!< possible values from @GPIO_PIN_OPTYPES>*/
	uint8_t GPIO_PinSpeed;		/*!< possible values from @GPIO_PIN_SPEED>*/
	uint8_t GPIO_PinPuPdControl;  // Pull up Pull down control
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;



typedef struct
{
	GPIO_RegDef_t *pGPIOx ;			/* this holds the base address of the GPIO port to which the pin belongs */
	GPIO_PinConfig_t GPIO_PinConfig ;
}GPIO_Handle_t;


/*
 * @GPIO_PIN_MODES
 *  GPIO pin Possible modes
 */

#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4 // falling trigger
#define GPIO_MODE_IT_RT		5 // rising trigger
#define GPIO_MODE_IT_RFT	6 // Rising fallin triger

/*
 *  GPIO pin output type
 */

#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/*
 *  GPIO pin output speed
 */

#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_HIGH			2
#define GPIO_SPEED_VERY_HIGH	3

/*
 *  GPIO pin Pull-up /Pull-down settings
 */

#define GPIO_NO_PUPD	0
#define GPIO_PIN_PU		1
#define GPIO_PIN_PD		2


void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 *   Init and De-Init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 *   Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriiteToInputPort(GPIO_RegDef_t *pGPIOx, uint8_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


/*
 *   IRQ configuration and ISR
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void IRQInterruptEnabler(IRQn_Type IRQNumber, uint8_t EnorDi);
void IRQInterruptPending(IRQn_Type IRQNumber, uint8_t PendingSts);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);
uint32_t GET_GPIO_BASEADDR(GPIO_RegDef_t *pGPIOx) ;


#endif /* STM32F407_GPIO_H_ */
