/*
 * arm4.h
 *
 *  Created on: Nov 6, 2022
 *      Author: Ahmed
 */

#ifndef ARM4_H_
#define ARM4_H_

#include "stm32f407.h"
/*
 *  ARM Cortex m4 NVIC ISRx register Addresses
 */

#define NVIC_ISER0				((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1				((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2				((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3				((__vo uint32_t*)0xE000E10C)
#define NVIC_ISER4				((__vo uint32_t*)0xE000E110)
#define NVIC_ISER5				((__vo uint32_t*)0xE000E114)
#define NVIC_ISER6				((__vo uint32_t*)0xE000E118)
#define NVIC_ISER7				((__vo uint32_t*)0xE000E11C)

#define NVIC_ICER0				((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1				((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2				((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3				((__vo uint32_t*)0xE000E18C)
#define NVIC_ICER4				((__vo uint32_t*)0xE000E190)
#define NVIC_ICER5				((__vo uint32_t*)0xE000E194)
#define NVIC_ICER6				((__vo uint32_t*)0xE000E198)
#define NVIC_ICER7				((__vo uint32_t*)0xE000E19C)

#define NVIC_ISPR0				((__vo uint32_t*)0xE000E200)
#define NVIC_ISPR1				((__vo uint32_t*)0xE000E204)
#define NVIC_ISPR2				((__vo uint32_t*)0xE000E208)
#define NVIC_ISPR3				((__vo uint32_t*)0xE000E20C)
#define NVIC_ISPR4				((__vo uint32_t*)0xE000E210)
#define NVIC_ISPR5				((__vo uint32_t*)0xE000E214)
#define NVIC_ISPR6				((__vo uint32_t*)0xE000E218)
#define NVIC_ISPR7				((__vo uint32_t*)0xE000E21C)

#define NVIC_ICPR0				((__vo uint32_t*)0xE000E280)
#define NVIC_ICPR1				((__vo uint32_t*)0xE000E284)
#define NVIC_ICPR2				((__vo uint32_t*)0xE000E288)
#define NVIC_ICPR3				((__vo uint32_t*)0xE000E18C)
#define NVIC_ICPR4				((__vo uint32_t*)0xE000E290)
#define NVIC_ICPR5				((__vo uint32_t*)0xE000E294)
#define NVIC_ICPR6				((__vo uint32_t*)0xE000E298)
#define NVIC_ICPR7				((__vo uint32_t*)0xE000E29C)

#define NVIC_IABR0				((__vo uint32_t*)0xE000E300)
#define NVIC_IABR1				((__vo uint32_t*)0xE000E304)
#define NVIC_IABR2				((__vo uint32_t*)0xE000E308)
#define NVIC_IABR3				((__vo uint32_t*)0xE000E30C)
#define NVIC_IABR4				((__vo uint32_t*)0xE000E310)
#define NVIC_IABR5				((__vo uint32_t*)0xE000E314)
#define NVIC_IABR6				((__vo uint32_t*)0xE000E318)
#define NVIC_IABR7				((__vo uint32_t*)0xE000E31C)

#define NVIC_IPR0				((__vo uint32_t*)0xE000E400)
#define NVIC_IPR1				((__vo uint32_t*)0xE000E404)
#define NVIC_IPR2				((__vo uint32_t*)0xE000E408)
#define NVIC_IPR3				((__vo uint32_t*)0xE000E40C)
#define NVIC_IPR4				((__vo uint32_t*)0xE000E410)
#define NVIC_IPR5				((__vo uint32_t*)0xE000E414)
#define NVIC_IPR6				((__vo uint32_t*)0xE000E418)
#define NVIC_IPR7				((__vo uint32_t*)0xE000E41C)

#define NVIC_IPR59				((__vo uint32_t*)0xE000E4EF)

#define NVIC_STIR				((__vo uint32_t*)0xE000EF00)

// FPU

#define ARM_CPACR				((__vo uint32_t*)0xE000ED88)
#define ARM_FPCCR				((__vo uint32_t*)0xE000EF34)
#define ARM_FPCAR				((__vo uint32_t*)0xE000EF38)
#define ARM_FPDSCR				((__vo uint32_t*)0xE000EF3C)


typedef enum
{
	NVIC_PRI0		= 0,
	NVIC_PRI1		= 1,
	NVIC_PRI2		= 2,
	NVIC_PRI3		= 3,
	NVIC_PRI4		= 4,
	NVIC_PRI5		= 5,
	NVIC_PRI6		= 6,
	NVIC_PRI7		= 7,
	NVIC_PRI8		= 8,
	NVIC_PRI9		= 9,
	NVIC_PRI10		= 10,
	NVIC_PRI11		= 11,
	NVIC_PRI12		= 12,
	NVIC_PRI13		= 13,
	NVIC_PRI14		= 14,
	NVIC_PRI15		= 15

} NVIC_IRQ_PRIO;

typedef enum
{
/******  STM32 specific Interrupt Numbers **********************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_PVM_IRQn                = 1,      /*!< PVD/PVM1/PVM2/PVM3/PVM4 through EXTI Line detection Interrupts    */
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Channel1_IRQn          = 11,     /*!< DMA1 Channel 1 global Interrupt                                   */
  DMA1_Channel2_IRQn          = 12,     /*!< DMA1 Channel 2 global Interrupt                                   */
  DMA1_Channel3_IRQn          = 13,     /*!< DMA1 Channel 3 global Interrupt                                   */
  DMA1_Channel4_IRQn          = 14,     /*!< DMA1 Channel 4 global Interrupt                                   */
  DMA1_Channel5_IRQn          = 15,     /*!< DMA1 Channel 5 global Interrupt                                   */
  DMA1_Channel6_IRQn          = 16,     /*!< DMA1 Channel 6 global Interrupt                                   */
  DMA1_Channel7_IRQn          = 17,     /*!< DMA1 Channel 7 global Interrupt                                   */
  ADC1_2_IRQn                 = 18,     /*!< ADC1, ADC2 SAR global Interrupts                                  */
  CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
  CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM15_IRQn         = 24,     /*!< TIM1 Break interrupt and TIM15 global interrupt                   */
  TIM1_UP_TIM16_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM16 global interrupt                  */
  TIM1_TRG_COM_TIM17_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM17 global interrupt */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  DFSDM1_FLT3_IRQn            = 42,     /*!< DFSDM1 Filter 3 global Interrupt                                  */
  TIM8_BRK_IRQn               = 43,     /*!< TIM8 Break Interrupt                                              */
  TIM8_UP_IRQn                = 44,     /*!< TIM8 Update Interrupt                                             */
  TIM8_TRG_COM_IRQn           = 45,     /*!< TIM8 Trigger and Commutation Interrupt                            */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare Interrupt                                    */
  ADC3_IRQn                   = 47,     /*!< ADC3 global  Interrupt                                            */
  FMC_IRQn                    = 48,     /*!< FMC global Interrupt                                              */
  SDMMC1_IRQn                 = 49,     /*!< SDMMC1 global Interrupt                                           */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
  TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
  DMA2_Channel1_IRQn          = 56,     /*!< DMA2 Channel 1 global Interrupt                                   */
  DMA2_Channel2_IRQn          = 57,     /*!< DMA2 Channel 2 global Interrupt                                   */
  DMA2_Channel3_IRQn          = 58,     /*!< DMA2 Channel 3 global Interrupt                                   */
  DMA2_Channel4_IRQn          = 59,     /*!< DMA2 Channel 4 global Interrupt                                   */
  DMA2_Channel5_IRQn          = 60,     /*!< DMA2 Channel 5 global Interrupt                                   */
  DFSDM1_FLT0_IRQn            = 61,     /*!< DFSDM1 Filter 0 global Interrupt                                  */
  DFSDM1_FLT1_IRQn            = 62,     /*!< DFSDM1 Filter 1 global Interrupt                                  */
  DFSDM1_FLT2_IRQn            = 63,     /*!< DFSDM1 Filter 2 global Interrupt                                  */
  COMP_IRQn                   = 64,     /*!< COMP1 and COMP2 Interrupts                                        */
  LPTIM1_IRQn                 = 65,     /*!< LP TIM1 interrupt                                                 */
  LPTIM2_IRQn                 = 66,     /*!< LP TIM2 interrupt                                                 */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Channel6_IRQn          = 68,     /*!< DMA2 Channel 6 global interrupt                                   */
  DMA2_Channel7_IRQn          = 69,     /*!< DMA2 Channel 7 global interrupt                                   */
  LPUART1_IRQn                = 70,     /*!< LP UART1 interrupt                                                */
  QUADSPI_IRQn                = 71,     /*!< Quad SPI global interrupt                                         */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  SAI1_IRQn                   = 74,     /*!< Serial Audio Interface 1 global interrupt                         */
  SAI2_IRQn                   = 75,     /*!< Serial Audio Interface 2 global interrupt                         */
  SWPMI1_IRQn                 = 76,     /*!< Serial Wire Interface 1 global interrupt                          */
  TSC_IRQn                    = 77,     /*!< Touch Sense Controller global interrupt                           */
  LCD_IRQn                    = 78,     /*!< LCD global interrupt                                              */
  RNG_IRQn                    = 80,     /*!< RNG global interrupt                                              */
  FPU_IRQn                    = 81      /*!< FPU global interrupt                                              */
} IRQn_Type;


void NVIC_EnableIRQ(IRQn_Type IRQNumber);
void NVIC_DisableIRQ(IRQn_Type IRQNumber);
void NVIC_SetPending(IRQn_Type IRQNumber);
void NVIC_ClearPending(IRQn_Type IRQNumber);
uint8_t NVIC_GetIRQPdStatus(IRQn_Type IRQNumber);
void NVIC_SetPriority(IRQn_Type IRQNumber, NVIC_IRQ_PRIO IRQPriority) ;
uint8_t NVIC_GetPriority(IRQn_Type IRQNumber) ;

#endif /* ARM4_H_ */
