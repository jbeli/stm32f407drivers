/*
 * stm32f407_rcc.h
 *
 *  Created on: Nov 6, 2022
 *      Author: Ahmed
 */

#ifndef INC_STM32F407_RCC_H_
#define INC_STM32F407_RCC_H_

#include "stm32f407.h"

#if !defined(HSI_VALUE)
  #define HSI_VALUE             ((uint32_t)16000000U) // Hz
#endif // HSI_VALUE


/*********** AHB Prescaler **********/
#define AHB_SYSCLK_DIV1		0x0
#define AHB_SYSCLK_DIV2		0x8
#define AHB_SYSCLK_DIV4		0x9
#define AHB_SYSCLK_DIV8		0xA
#define AHB_SYSCLK_DIV16	0xB
#define AHB_SYSCLK_DIV64	0xC
#define AHB_SYSCLK_DIV128	0xD
#define AHB_SYSCLK_DIV256	0xE
#define AHB_SYSCLK_DIV512	0xF

/************APB1 Prescaler***********/
#define APB1_SYSCLK_DIV1	0x0
#define APB1_SYSCLK_DIV2	0x4
#define APB1_SYSCLK_DIV4	0x5
#define APB1_SYSCLK_DIV8	0x6
#define APB1_SYSCLK_DIV16	0x7

/************APB2 Prescaler***********/
#define APB2_SYSCLK_DIV1	0x0
#define APB2_SYSCLK_DIV2	0x4
#define APB2_SYSCLK_DIV4	0x5
#define APB2_SYSCLK_DIV8	0x6
#define APB2_SYSCLK_DIV16	0x7

/************System Clock Switch***********/

#define HSI_OSC_SYSTEM_CLK		0x00
#define HSE_OSC_SYSTEM_CLK		0x01
#define PLL_SYSTEM_CLK			0x02



// Clock dividers
typedef struct {
	uint32_t AHBdiv;  // AHB divider, one of RCC_AHB_DIVx values
	uint32_t APB1div; // APB1 divider, one of RCC_APB1_DIVx values
	uint32_t APB2div; // APB2 divider, one of RCC_APB2_DIVx values
} RCC_CLKInitTypeDef;

// AHB1 Peripherals

typedef struct
{
	uint8_t OTGHSULPIEN ;
	uint8_t OTGHSEN ;
	uint8_t ETHMACPTPEN ;
	uint8_t ETHMACRXEN ;
	uint8_t ETHMACTXEN ;
	uint8_t ETHMACEN ;
	uint8_t DMA2EN ;
	uint8_t DMA1EN ;
	uint8_t CCMDATARAMEN ;
	uint8_t BKPSRAMEN ;
	uint8_t CRCEN ;
	uint8_t GPIOIEN ;
	uint8_t GPIOHEN ;
	uint8_t GPIOGEN ;
	uint8_t GPIOFEN ;
	uint8_t GPIOEEN ;
	uint8_t GPIODEN ;
	uint8_t GPIOCEN ;
	uint8_t GPIOBEN ;
	uint8_t GPIOAEN ;

}RCC_AHB1PeriphEnab;

typedef struct
{
	uint8_t OTGFSEN ;
	uint8_t RNGEN ;
	uint8_t HASHEN ;
	uint8_t CRYPEN ;
	uint8_t DCMIEN ;

}RCC_AHB2PeriphEnab;

typedef struct
{
	uint8_t FSMCEN;

}RCC_AHB23PeriphEnab;

typedef struct
{
	uint8_t DACEN;
	uint8_t PWREN;
	uint8_t CAN2EN;
	uint8_t CAN1EN;
	uint8_t I2C3EN;
	uint8_t I2C2EN;
	uint8_t I2C1EN;
	uint8_t UART5EN;
	uint8_t UART4EN;
	uint8_t USART3EN;
	uint8_t USART2EN;
	uint8_t SPI3EN;
	uint8_t SPI2EN;
	uint8_t WWDGEN;
	uint8_t TIM14EN;
	uint8_t TIM13EN;
	uint8_t TIM12EN;
	uint8_t TIM7EN;
	uint8_t TIM6EN;
	uint8_t TIM5EN;
	uint8_t TIM4EN;
	uint8_t TIM3EN;
	uint8_t TIM2EN;

}RCC_APB1PeriphEnab;

typedef struct
{
	uint8_t TIM11EN;
	uint8_t TIM10EN;
	uint8_t TIM9EN;
	uint8_t SYSCFGEN;
	uint8_t SPI1EN;
	uint8_t SDIOEN;
	uint8_t ADC3EN;
	uint8_t ADC2EN;
	uint8_t ADC1EN;
	uint8_t USART6EN;
	uint8_t USART1EN;
	uint8_t TIM8EN;
	uint8_t TIM1EN;
}RCC_APB2PeriphEnab;

typedef enum
{
	RCC_HSI_OFF = 0,
	RCC_HSI_ON
} HSI_State;

typedef enum
{
	RCC_OSC_HSI = 0 ,
	RCC_OSC_HSE,
	RCC_OSC_PLL
}RCC_OSC_Source;

#define RCC_Periph_Enable		 1
#define RCC_Periph_Disable		 0

void RCC_GetSYSCLK(void);
int RCC_GetHCLK(RCC_CLKInitTypeDef RCC_AxB_DIV);
float RCC_GetPLCLK1(RCC_CLKInitTypeDef RCC_AxB_DIV) ;
float RCC_GetPLCLK2(RCC_CLKInitTypeDef RCC_AxB_DIV);
void RCC_HSIConfig(HSI_State state) ;
void RCC_SetPrescaler(RCC_CLKInitTypeDef RCC_AxB_DIV) ;
void RCC_DeInitPrescaler(void);
void RCC_EnableAHB1PeriphClockOTGHSULPIEN(void);
void RCC_SetSystemClockSwitch(uint8_t selected_sysclk);
RCC_OSC_Source RCC_GetOscillatorSoucre(void) ;

void RCC_DisableAHB1PeriphClockOTGHSULPIEN(void);

void RCC_EnableAHB1PeriphClockOTGHSEN(void);

void RCC_DisableAHB1PeriphClockOTGHSEN(void);

void RCC_EnableAHB1PeriphClockETHMACPTPEN(void);

void RCC_DisableAHB1PeriphClockETHMACPTPEN(void);

void RCC_EnableAHB1PeriphClockETHMACRXEN(void);

void RCC_DisableAHB1PeriphClockETHMACRXEN(void);

void RCC_EnableAHB1PeriphClockETHMACTXEN(void);

void RCC_DisableAHB1PeriphClockETHMACTXEN(void);

void RCC_EnableAHB1PeriphClockETHMACEN(void);

void RCC_DisableAHB1PeriphClockETHMACEN(void);

void RCC_EnableAHB1PeriphClockDMA2EN(void);

void RCC_DisableAHB1PeriphClockDMA2EN(void);

void RCC_EnableAHB1PeriphClockDMA1EN(void);

void RCC_DisableAHB1PeriphClockDMA1EN(void);

void RCC_EnableAHB1PeriphClockCCMDATARAMEN(void);

void DisableAHB1PeriphClockCCMDATARAMEN(void);

void EnableAHB1PeriphClockBKPSRAMEN(void);

void DisableAHB1PeriphClockBKPSRAMEN(void);

void EnableAHB1PeriphClockCRCEN(void);

void DisableAHB1PeriphClockCRCEN(void);

void EnableAHB1PeriphClockGPIOIEN(void);

void DisableAHB1PeriphClockGPIOIEN(void);

void EnableAHB1PeriphClockGPIOHEN(void);

void DisableAHB1PeriphClockGPIOHEN(void);

void EnableAHB1PeriphClockGPIOGEN(void);

void DisableAHB1PeriphClockGPIOGEN(void);

void EnableAHB1PeriphClockGPIOFEN(void);

void DisableAHB1PeriphClockGPIOFEN(void);

void EnableAHB1PeriphClockGPIOEEN(void);

void DisableAHB1PeriphClockGPIOEEN(void);

void EnableAHB1PeriphClockGPIODEN(void);

void DisableAHB1PeriphClockGPIODEN(void);

void EnableAHB1PeriphClockGPIOCEN(void);

void DisableAHB1PeriphClockGPIOCEN(void);

void EnableAHB1PeriphClockGPIOBEN(void);

void DisableAHB1PeriphClockGPIOBEN(void);

void EnableAHB1PeriphClockGPIOAEN(void);

void DisableAHB1PeriphClockGPIOAEN(void);



/* AHB2 */

void EnableAHB2PeriphClockOTGFSEN(void);

void DisableAHB2PeriphClockOTGFSEN(void);

void EnableAHB2PeriphClockRNGEN(void);

void DisableAHB2PeriphClockRNGEN(void);

void EnableAHB2PeriphClockHASHEN(void);

void DisableAHB2PeriphClockHASHEN(void);

void EnableAHB2PeriphClockCRYPEN(void);

void DisableAHB2PeriphClockCRYPEN(void);

void EnableAHB2PeriphClockDCMIEN(void);

void DisableAHB2PeriphClockDCMIEN(void);



/* AHB3 */

void EnableAHB3PeriphClockFSMCEN(void);

void DisableAHB3PeriphClockFSMCEN(void);



/* APB1 */

void EnableAPB1PeriphClockDACEN(void);

void DisableAPB1PeriphClockDACEN(void);

void EnableAPB1PeriphClockPWREN(void);

void DisableAPB1PeriphClockPWREN(void);

void EnableAPB1PeriphClockCAN2EN(void);

void DisableAPB1PeriphClockCAN2EN(void);

void EnableAPB1PeriphClockCAN1EN(void);

void DisableAPB1PeriphClockCAN1EN(void);

void EnableAPB1PeriphClockI2C3EN(void);

void DisableAPB1PeriphClockI2C3EN(void);

void EnableAPB1PeriphClockI2C2EN(void);

void DisableAPB1PeriphClockI2C2EN(void);

void EnableAPB1PeriphClockI2C1EN(void);

void DisableAPB1PeriphClockI2C1EN(void);

void EnableAPB1PeriphClockUART5EN(void);

void DisableAPB1PeriphClockUART5EN(void);

void EnableAPB1PeriphClockUART4EN(void);

void DisableAPB1PeriphClockUART4EN(void);

void EnableAPB1PeriphClockUSART3EN(void);

void DisableAPB1PeriphClockUSART3EN(void);

void EnableAPB1PeriphClockUSART2EN(void);

void DisableAPB1PeriphClockUSART2EN(void);

void EnableAPB1PeriphClockSPI3EN(void);

void DisableAPB1PeriphClockSPI3EN(void);

void EnableAPB1PeriphClockSPI2EN(void);

void DisableAPB1PeriphClockSPI2EN(void);

void EnableAPB1PeriphClockWWDGEN(void);

void DisableAPB1PeriphClockWWDGEN(void);

void EnableAPB1PeriphClockTIM14EN(void);

void DisableAPB1PeriphClockTIM14EN(void);

void EnableAPB1PeriphClockTIM13EN(void);

void DisableAPB1PeriphClockTIM13EN(void);

void EnableAPB1PeriphClockTIM12EN(void);

void DisableAPB1PeriphClockTIM12EN(void);

void EnableAPB1PeriphClockTIM7EN(void);

void DisableAPB1PeriphClockTIM7EN(void);

void EnableAPB1PeriphClockTIM6EN(void);

void DisableAPB1PeriphClockTIM6EN(void);

void EnableAPB1PeriphClockTIM5EN(void);

void DisableAPB1PeriphClockTIM5EN(void);

void EnableAPB1PeriphClockTIM4EN(void);

void DisableAPB1PeriphClockTIM4EN(void);

void EnableAPB1PeriphClockTIM3EN(void);

void DisableAPB1PeriphClockTIM3EN(void);

void EnableAPB1PeriphClockTIM2EN(void);

void DisableAPB1PeriphClockTIM2EN(void);



/* APB2 */

void EnableAPB2PeriphClockTIM11EN(void);

void DisableAPB2PeriphClockTIM11EN(void);

void EnableAPB2PeriphClockTIM10EN(void);

void DisableAPB2PeriphClockTIM10EN(void);

void EnableAPB2PeriphClockTIM9EN(void);

void DisableAPB2PeriphClockTIM9EN(void);

void EnableAPB2PeriphClockSYSCFGEN(void);

void DisableAPB2PeriphClockSYSCFGEN(void);

void EnableAPB2PeriphClockSPI1EN(void);

void DisableAPB2PeriphClockSPI1EN(void);

void EnableAPB2PeriphClockSDIOEN(void);

void DisableAPB2PeriphClockSDIOEN(void);

void EnableAPB2PeriphClockADC3EN(void);

void DisableAPB2PeriphClockADC3EN(void);

void EnableAPB2PeriphClockADC2EN(void);

void DisableAPB2PeriphClockADC2EN(void);

void EnableAPB2PeriphClockADC1EN(void);

void DisableAPB2PeriphClockADC1EN(void);

void EnableAPB2PeriphClockUSART6EN(void);

void DisableAPB2PeriphClockUSART6EN(void);

void EnableAPB2PeriphClockUSART1EN(void);

void DisableAPB2PeriphClockUSART1EN(void);

void EnableAPB2PeriphClockTIM8EN(void);

void DisableAPB2PeriphClockTIM8EN(void);

void EnableAPB2PeriphClockTIM1EN(void);

void DisableAPB2PeriphClockTIM1EN(void);

#endif /* INC_STM32F407_RCC_H_ */
