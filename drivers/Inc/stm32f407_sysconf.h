/*
 * stm32f407_sysconf.h
 *
 *  Created on: Nov 6, 2022
 *      Author: Ahmed
 */

#ifndef STM32F407_SYSCONF_H_
#define STM32F407_SYSCONF_H_

#include "stm32f407.h"
#include "stm32f407_gpio.h"

#define __vo 	volatile

#define SYSCFG_MainFlashMemory		0x00
#define SYSCFG_SystemFlashMemory	0x01
#define SYSCFG_FSMCBank1			0x02
#define SYSCFG_EmbeddedSRAM		    0x03

#define MII_Interface		0
#define RMII_Interface		1

#define I_OCompensationCellPowerDown	0
#define I_OCompensationCellEnabled		1

void SYSCFG_SetMemoryRemapConfig(uint8_t SYSCFG_MemoryRemap);
void SYSCFG_SetPeriphMode(uint8_t mode);
void SYSCFG_SetClockPeriph();
void SYSCFG_SetExternalInt(GPIO_Handle_t *pGPIOHandle);
void SYSCFG_SetCompensationCellPowerDown(uint8_t CMP_PD);
uint8_t SYSCFG_GetCompensationCellReadyFlag();
uint8_t SYSCFG_GetMemoryRemap();
uint8_t SYSCFG_GetPeriphMode();


#endif /* STM32F407_SYSCONF_H_ */
