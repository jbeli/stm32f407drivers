/*
 * stm32f407.h
 *
 *  Created on: Nov 6, 2022
 *      Author: Ahmed
 */

#ifndef STM32F407_H_
#define STM32F407_H_

#include <stdint.h>
#include "arm4.h"

// The value of Internal High Speed oscillator (HSI)
#if !defined(HSI_VALUE)
  #define HSI_VALUE             ((uint32_t)16000000U) // Hz
#endif // HSI_VALUE

#define __vo 					volatile







#define FLASH_BASEADDR			0x08000000U 	/* Base address for Flash Memory*/
#define SRAM1_BASEADDR			0x20000000U		/* Base address for SRAM1 Memory*/
#define SRAM2_BASEADDR			0x2001C000U		/* Base address for SRAM2 Memory*/
#define SYS_MEMORY_BASEADDR		0x1FFF0000U		/* Base address for System Memory*/

#define SRAM 					SRAM1_BASEADDR	/* Base address for SRAM Memory*/


/*!< Peripheral memory map */

#define PERIPH_BASEADDR				0x40000000U
#define APB1PERIPH_BASEADDR			PERIPH_BASE
#define APB2PERIPH_BASEADDR			0x40010000U
#define AHB1PERIPH_BASEADDR			0x40020000U
#define AHB2PERIPH_BASEADRR			0x50000000U

/*!< APB1 peripherals */

#define TIM2_BASEADDR				(APB1PERIPH_BASEADDR + 0x0000)
#define TIM3_BASEADDR				(APB1PERIPH_BASEADDR + 0x0400)
#define TIM4_BASEADDR				(APB1PERIPH_BASEADDR + 0x0800)
#define TIM5_BASEADDR				(APB1PERIPH_BASEADDR + 0x0C00)
#define TIM6_BASEADDR				(APB1PERIPH_BASEADDR + 0x1000)
#define TIM7_BASEADDR				(APB1PERIPH_BASEADDR + 0x1400)
#define TIM12_BASEADDR				(APB1PERIPH_BASEADDR + 0x1800)
#define TIM13_BASEADDR				(APB1PERIPH_BASEADDR + 0x1C00)
#define TIM14_BASEADDR				(APB1PERIPH_BASEADDR + 0x2000)
#define APB1_RESERVED0				(APB1PERIPH_BASEADDR + 0x2400)
#define RTC_BKP_REG_BASEADDR		(APB1PERIPH_BASEADDR + 0x2800)
#define WWDG_BASEADDR				(APB1PERIPH_BASEADDR + 0x2C00)
#define IWDG_BASEADDR				(APB1PERIPH_BASEADDR + 0x3000)
#define I2S2ext_BASEADDR			(APB1PERIPH_BASEADDR + 0x3400)
#define SPI2_I2S2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_I2S3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00)
#define I2S3ext_BASEADDR			(APB1PERIPH_BASEADDR + 0x4000)
#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR				(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR				(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR				(APB1PERIPH_BASEADDR + 0x5000)
#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR				(APB1PERIPH_BASEADDR + 0x5C00)
#define APB1_RESERVED1				(APB1PERIPH_BASEADDR + 0x6000)
#define CAN1_BASEADDR				(APB1PERIPH_BASEADDR + 0x6400)
#define CAN2_BASEADDR				(APB1PERIPH_BASEADDR + 0x6800)
#define APB1_RESERVED2				(APB1PERIPH_BASEADDR + 0x6C00)
#define PWR_BASEADDR				(APB1PERIPH_BASEADDR + 0x7000)
#define DAC_BASADDR					(APB1PERIPH_BASEADDR + 0x7400)
#define UART7_BASEADDR				(APB1PERIPH_BASEADDR + 0x7800)
#define UART8_BASEADDR				(APB1PERIPH_BASEADDR + 0x7C00)


/*!< APB2 peripherals */

#define TIM1_BASEADDR				(APB2PERIPH_BASEADDR + 0x0000)
#define TIM8_BASEADDR				(APB2PERIPH_BASEADDR + 0x0400)
#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR				(APB2PERIPH_BASEADDR + 0x1400)
#define ADC1_2_3_BASEADDR			(APB2PERIPH_BASEADDR + 0x2000)
#define SDIO_BASEADDR				(APB2PERIPH_BASEADDR + 0x2C00)
#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR				(APB2PERIPH_BASEADDR + 0x3400)
#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + 0x3800)
#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x3C00)
#define TIM9_BASEADDR				(APB2PERIPH_BASEADDR + 0x4000)
#define TIM10_BASEADDR				(APB2PERIPH_BASEADDR + 0x4400)
#define TIM11_BASEADDR				(APB2PERIPH_BASEADDR + 0x4800)
#define SPI5_BASEADDR				(APB2PERIPH_BASEADDR + 0x5000)
#define SPI6_BASEADDR				(APB2PERIPH_BASEADDR + 0x5400)
#define SAI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x5800)
#define LCD_TFT_BASEADDR			(APB2PERIPH_BASEADDR + 0x6800)

/*!< AHB1 peripherals */

#define GPIOA_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2000)
#define GPIOJ_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2400)
#define GPIOK_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2800)
#define RCC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x3800)
#define FLASH_INTRF_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3C00)
#define BKPSRAM_BASEADDR				(AHB1PERIPH_BASEADDR + 0x4000)
#define DMA1_BASEADDR					(AHB1PERIPH_BASEADDR + 0x6000)
#define DMA2_BASEADDR					(AHB1PERIPH_BASEADDR + 0x6400)
#define ETHERNET_MAC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x8000)
#define DMA2D_BASEADDR					(AHB1PERIPH_BASEADDR + 0xB000)
#define USB_OTG_HS_BASEADDR				(AHB1PERIPH_BASEADDR + 0x00020000)

/*!< AHB2 peripherals */

#define USB_OTG_FS_BASEADDR				(AHB2PERIPH_BASEADRR + 0x0000)
#define DCMI_BASEADDR					(AHB2PERIPH_BASEADRR + 0x00050000)
#define CRYP_BASEADDR					(AHB2PERIPH_BASEADRR + 0x00060000)
#define HASH_BASEADDR					(AHB2PERIPH_BASEADRR + 0x00060400)
#define RNG_BASEADDR					(AHB2PERIPH_BASEADRR + 0x00060800)





/**************************************** Peripheral register definition structures ****************************************/

/**
  * @brief General Purpose I/O
  */

typedef struct
{
	__vo uint32_t	MODER;
	__vo uint32_t	OTYPER;
	__vo uint32_t	OSPEEDR;
	__vo uint32_t	PUPDR;
	__vo uint32_t	IDR;
	__vo uint32_t	ODR;
	__vo uint32_t	BSRR;
	__vo uint32_t	LCKR;
	__vo uint32_t	AFRL; // low register
	__vo uint32_t	AFRH; // high register

}GPIO_RegDef_t;


typedef struct
{
	__vo uint32_t	CR;
	__vo uint32_t	PLLCFGR;
	__vo uint32_t	CFGR;
	__vo uint32_t	CIR;
	__vo uint32_t	AHB1RSTR;
	__vo uint32_t	AHB2RSTR;
	__vo uint32_t	AHB3RSTR;
	uint32_t   RESERVED0;
	__vo uint32_t	APB1RSTR;
	__vo uint32_t	APB2RSTR;
	 uint32_t	RESERVED1[2];
	__vo uint32_t	AHB1ENR;
	__vo uint32_t	AHB2ENR;
	__vo uint32_t	AHB3ENR;
	 uint32_t	RESERVED3;
	__vo uint32_t	APB1ENR;
	__vo uint32_t	APB2ENR;
	uint32_t   RESERVED4[2];
	__vo uint32_t	AHB1LPENR;
	__vo uint32_t	AHB2LPENR;
	__vo uint32_t	AHB3LPENR;
	 uint32_t	RESERVED6;
	__vo uint32_t	APB1LPENR;
	__vo uint32_t	APB2LPENR;
	uint32_t   RESERVED7[2];
	__vo uint32_t	BDCR;
	__vo uint32_t	CSR;
	 uint32_t	RESERVED9[2];
	__vo uint32_t	SSCGR;
	__vo uint32_t   PLLI2SCFGR;

}RCC_RegDef_t;

typedef struct
{
	__vo uint32_t IMR ;
	__vo uint32_t EMR ;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

}EXTI_RegDef_t;

typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t	  RESERVED[2];
	__vo uint32_t CMPCR;
}SYSCFG_RegDef_t;


/** @addtogroup Peripheral_declaration
  * @{
  */

#define GPIOA		((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF		((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG		((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH		((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI		((GPIO_RegDef_t*) GPIOI_BASEADDR)
#define GPIOJ		((GPIO_RegDef_t*) GPIOJ_BASEADDR)
#define GPIOK		((GPIO_RegDef_t*) GPIOK_BASEADDR)

#define RCC			((RCC_RegDef_t*) RCC_BASEADDR)

#define EXTI		((EXTI_RegDef_t*) EXTI_BASEADDR)
#define SYSCFG		((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)



/******************************************************************************/
/*                         Peripheral Registers_Bits_Definition               */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                         Reset & Clock COntrol (RCC)                         */
/*                                                                            */
/******************************************************************************/

/********************** Bit definition for RCC_CR register *******************/


#define RCC_CR_HSION_Pos					(0U)
#define RCC_CR_HSION_Msk					(0x1UL << RCC_CR_HSION_Pos)			/*!<0x00000001>*/
#define RCC_CR_HSION						RCC_CR_HSION_Msk					/*!< Internal High Speed oscillator (HSI16) clock enable */

#define RCC_CR_HSIRDY_Pos					(1U)
#define RCC_CR_HSIRDY_Msk					(0x1UL << RCC_CR_HSIRDY_Pos)
#define RCC_CR_HSIRDY						RCC_CR_HSIRDY_Msk

#define RCC_CR_HSITRIM_Pos					(3U)
#define RCC_CR_HSITRIM_Msk					(0x1FUL << RCC_CR_HSITRIM_Pos)
#define RCC_CR_HSITRIM						RCC_CR_HSITRIM_Msk

#define RCC_CR_HSICAL_Pos					(8U)
#define RCC_CR_HSICAL_Msk					(0x1UL << RCC_CR_HSICAL_Pos)
#define RCC_CR_HSICAL						RCC_CR_HSICAL_Msk

#define RCC_CR_HSEON_Pos					(16U)
#define RCC_CR_HSEON_Msk					(0x1UL << RCC_CR_HSEON_Pos)
#define RCC_CR_HSEON						RCC_CR_HSEON_Msk

#define RCC_CR_HSERDY_Pos					(17U)
#define RCC_CR_HSERDY_Msk					(0x1UL << RCC_CR_HSERDY_Pos)
#define RCC_CR_HSERDY						RCC_CR_HSERDY_Msk

#define RCC_CR_HSEBYP_Pos					(18U)
#define RCC_CR_HSEBYP_Msk					(0x1UL << RCC_CR_HSEBYP_Pos)
#define RCC_CR_HSEBYP						RCC_CR_HSEBYP_Msk

#define RCC_CR_CSSON_Pos					(19U)
#define RCC_CR_CSSON_Msk					(0x1UL << RCC_CR_CSSON_Pos)
#define RCC_CR_CSSON						RCC_CR_CSSON_Msk

#define RCC_CR_PLLON_Pos					(24U)
#define RCC_CR_PLLON_Msk					(0x1UL << RCC_CR_PLLON_Pos)
#define RCC_CR_PLLON						RCC_CR_PLLON_Msk

#define RCC_CR_PLLRDY_Pos					(25U)
#define RCC_CR_PLLRDY_Msk					(0x1UL << RCC_CR_PLLRDY_Pos)
#define RCC_CR_PLLRDY						RCC_CR_PLLRDY_Msk

#define RCC_CR_PLLI2SON_Pos					(26U)
#define RCC_CR_PLLI2SON_Msk					(0x1UL << RCC_CR_PLLI2SON_Pos)
#define RCC_CR_PLLI2SON						RCC_CR_PLLI2SON_Msk

#define RCC_CR_PLLI2SRDY_Pos				(27U)
#define RCC_CR_PLLI2SRDY_Msk				(0x01UL << RCC_CR_PLLI2SRDY_Pos)
#define RCC_CR_PLLI2SRDY					RCC_CR_PLLI2SRDY_Msk


/********************  Bit definition for RCC_PLLCFGR register  ***************/
#define RCC_PLLCFGR_PLLSRC_Pos               (0U)
#define RCC_PLLCFGR_PLLSRC_Msk               (0x3UL << RCC_PLLCFGR_PLLSRC_Pos) /*!< 0x00000003 */
#define RCC_PLLCFGR_PLLSRC                   RCC_PLLCFGR_PLLSRC_Msk

#define RCC_PLLCFGR_PLLSRC_MSI_Pos           (0U)
#define RCC_PLLCFGR_PLLSRC_MSI_Msk           (0x1UL << RCC_PLLCFGR_PLLSRC_MSI_Pos) /*!< 0x00000001 */
#define RCC_PLLCFGR_PLLSRC_MSI               RCC_PLLCFGR_PLLSRC_MSI_Msk        /*!< MSI oscillator source clock selected */
#define RCC_PLLCFGR_PLLSRC_HSI_Pos           (1U)
#define RCC_PLLCFGR_PLLSRC_HSI_Msk           (0x1UL << RCC_PLLCFGR_PLLSRC_HSI_Pos) /*!< 0x00000002 */
#define RCC_PLLCFGR_PLLSRC_HSI               RCC_PLLCFGR_PLLSRC_HSI_Msk        /*!< HSI16 oscillator source clock selected */
#define RCC_PLLCFGR_PLLSRC_HSE_Pos           (0U)
#define RCC_PLLCFGR_PLLSRC_HSE_Msk           (0x3UL << RCC_PLLCFGR_PLLSRC_HSE_Pos) /*!< 0x00000003 */
#define RCC_PLLCFGR_PLLSRC_HSE               RCC_PLLCFGR_PLLSRC_HSE_Msk        /*!< HSE oscillator source clock selected */

#define RCC_PLLCFGR_PLLM_Pos                 (0U)
#define RCC_PLLCFGR_PLLM_Msk                 (0x7UL << RCC_PLLCFGR_PLLM_Pos)   /*!< 0x00000070 */
#define RCC_PLLCFGR_PLLM                     RCC_PLLCFGR_PLLM_Msk
#define RCC_PLLCFGR_PLLM_0                   (0x1UL << RCC_PLLCFGR_PLLM_Pos)   /*!< 0x00000010 */
#define RCC_PLLCFGR_PLLM_1                   (0x2UL << RCC_PLLCFGR_PLLM_Pos)   /*!< 0x00000020 */
#define RCC_PLLCFGR_PLLM_2                   (0x4UL << RCC_PLLCFGR_PLLM_Pos)   /*!< 0x00000040 */

#define RCC_PLLCFGR_PLLN_Pos                 (8U)
#define RCC_PLLCFGR_PLLN_Msk                 (0x7FUL << RCC_PLLCFGR_PLLN_Pos)  /*!< 0x00007F00 */
#define RCC_PLLCFGR_PLLN                     RCC_PLLCFGR_PLLN_Msk
#define RCC_PLLCFGR_PLLN_0                   (0x01UL << RCC_PLLCFGR_PLLN_Pos)  /*!< 0x00000100 */
#define RCC_PLLCFGR_PLLN_1                   (0x02UL << RCC_PLLCFGR_PLLN_Pos)  /*!< 0x00000200 */
#define RCC_PLLCFGR_PLLN_2                   (0x04UL << RCC_PLLCFGR_PLLN_Pos)  /*!< 0x00000400 */
#define RCC_PLLCFGR_PLLN_3                   (0x08UL << RCC_PLLCFGR_PLLN_Pos)  /*!< 0x00000800 */
#define RCC_PLLCFGR_PLLN_4                   (0x10UL << RCC_PLLCFGR_PLLN_Pos)  /*!< 0x00001000 */
#define RCC_PLLCFGR_PLLN_5                   (0x20UL << RCC_PLLCFGR_PLLN_Pos)  /*!< 0x00002000 */
#define RCC_PLLCFGR_PLLN_6                   (0x40UL << RCC_PLLCFGR_PLLN_Pos)  /*!< 0x00004000 */

#define RCC_PLLCFGR_PLLPEN_Pos               (16U)
#define RCC_PLLCFGR_PLLPEN_Msk               (0x1UL << RCC_PLLCFGR_PLLPEN_Pos) /*!< 0x00010000 */
#define RCC_PLLCFGR_PLLPEN                   RCC_PLLCFGR_PLLPEN_Msk
#define RCC_PLLCFGR_PLLP_Pos                 (17U)
#define RCC_PLLCFGR_PLLP_Msk                 (0x1UL << RCC_PLLCFGR_PLLP_Pos)   /*!< 0x00020000 */
#define RCC_PLLCFGR_PLLP                     RCC_PLLCFGR_PLLP_Msk
#define RCC_PLLCFGR_PLLQEN_Pos               (20U)
#define RCC_PLLCFGR_PLLQEN_Msk               (0x1UL << RCC_PLLCFGR_PLLQEN_Pos) /*!< 0x00100000 */
#define RCC_PLLCFGR_PLLQEN                   RCC_PLLCFGR_PLLQEN_Msk

#define RCC_PLLCFGR_PLLQ_Pos                 (21U)
#define RCC_PLLCFGR_PLLQ_Msk                 (0x3UL << RCC_PLLCFGR_PLLQ_Pos)   /*!< 0x00600000 */
#define RCC_PLLCFGR_PLLQ                     RCC_PLLCFGR_PLLQ_Msk
#define RCC_PLLCFGR_PLLQ_0                   (0x1UL << RCC_PLLCFGR_PLLQ_Pos)   /*!< 0x00200000 */
#define RCC_PLLCFGR_PLLQ_1                   (0x2UL << RCC_PLLCFGR_PLLQ_Pos)   /*!< 0x00400000 */

#define RCC_PLLCFGR_PLLREN_Pos               (24U)
#define RCC_PLLCFGR_PLLREN_Msk               (0x1UL << RCC_PLLCFGR_PLLREN_Pos) /*!< 0x01000000 */
#define RCC_PLLCFGR_PLLREN                   RCC_PLLCFGR_PLLREN_Msk
#define RCC_PLLCFGR_PLLR_Pos                 (25U)
#define RCC_PLLCFGR_PLLR_Msk                 (0x3UL << RCC_PLLCFGR_PLLR_Pos)   /*!< 0x06000000 */
#define RCC_PLLCFGR_PLLR                     RCC_PLLCFGR_PLLR_Msk
#define RCC_PLLCFGR_PLLR_0                   (0x1UL << RCC_PLLCFGR_PLLR_Pos)   /*!< 0x02000000 */
#define RCC_PLLCFGR_PLLR_1                   (0x2UL << RCC_PLLCFGR_PLLR_Pos)   /*!< 0x04000000 */

/********************  Bit definition for RCC_CFGR register  ******************/

/* ! <Bits position>*/

#define RCC_CFGR_MCO2_Pos			0x1F
#define RCC_CFGR_MCO2_PRE_Pos		0x1B
#define RCC_CFGR_MCO1_PRE_Pos		0x18
#define RCC_CFGR_I2SSCR_Pos			0x17
#define RCC_CFGR_MCO1_Pos			0x16
#define RCC_CFGR_RTCPRE_Pos			0x10
#define RCC_CFGR_PPRE2_Pos			0x0D
#define RCC_CFGR_PPRE1_Pos			0x0A
#define RCC_CFGR_HPRE_Pos			0x04
#define RCC_CFGR_SWS_Pos			0x02
#define RCC_CFGR_SWS0_Pos			0x02
#define RCC_CFGR_SWS1_Pos			0x03
#define RCC_CFGR_SW_Pos				0x00
#define RCC_CFGR_SW0_Pos			0x00
#define RCC_CFGR_SW1_Pos			0x01

/* ! <Bits masks>*/

#define RCC_CFGR_MCO2_Msk			0x01
#define RCC_CFGR_MCO2_PRE_Msk		0x07
#define RCC_CFGR_MCO1_PRE_Msk		0x07
#define RCC_CFGR_I2SSCR_Msk			0x01
#define RCC_CFGR_MCO1_Msk			0x01
#define RCC_CFGR_RTCPRE_Msk			0x1F
#define RCC_CFGR_PPRE2_Msk			0x07
#define RCC_CFGR_PPRE1_Msk			0x07
#define RCC_CFGR_HPRE_Msk			0x0F
#define RCC_CFGR_SWS_Msk			0x03
#define RCC_CFGR_SW_Msk				0x03

/*!< SW configuration */
#define  RCC_CFGR_SW                         ((uint32_t)0x00000003)        /*!< SW[1:0] bits (System clock Switch) */
#define  RCC_CFGR_SW_0                       ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  RCC_CFGR_SW_1                       ((uint32_t)0x00000002)        /*!< Bit 1 */

#define  RCC_CFGR_SW_HSI                     ((uint32_t)0x00000000)        /*!< HSI selected as system clock */
#define  RCC_CFGR_SW_HSE                     ((uint32_t)0x00000001)        /*!< HSE selected as system clock */
#define  RCC_CFGR_SW_PLL                     ((uint32_t)0x00000002)        /*!< PLL selected as system clock */

/*!< SWS configuration */
#define  RCC_CFGR_SWS                        ((uint32_t)0x0000000C)        /*!< SWS[1:0] bits (System Clock Switch Status) */
#define  RCC_CFGR_SWS_0                      ((uint32_t)0x00000004)        /*!< Bit 0 */
#define  RCC_CFGR_SWS_1                      ((uint32_t)0x00000008)        /*!< Bit 1 */

#define  RCC_CFGR_SWS_HSI                    ((uint32_t)0x00000000)        /*!< HSI oscillator used as system clock */
#define  RCC_CFGR_SWS_HSE                    ((uint32_t)0x00000004)        /*!< HSE oscillator used as system clock */
#define  RCC_CFGR_SWS_PLL                    ((uint32_t)0x00000008)        /*!< PLL used as system clock */

/*!< HPRE configuration */
#define  RCC_CFGR_HPRE                       ((uint32_t)0x000000F0)        /*!< HPRE[3:0] bits (AHB prescaler) */
#define  RCC_CFGR_HPRE_0                     ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  RCC_CFGR_HPRE_1                     ((uint32_t)0x00000020)        /*!< Bit 1 */
#define  RCC_CFGR_HPRE_2                     ((uint32_t)0x00000040)        /*!< Bit 2 */
#define  RCC_CFGR_HPRE_3                     ((uint32_t)0x00000080)        /*!< Bit 3 */

#define  RCC_CFGR_HPRE_DIV1                  ((uint32_t)0x00000000)        /*!< SYSCLK not divided */
#define  RCC_CFGR_HPRE_DIV2                  ((uint32_t)0x00000080)        /*!< SYSCLK divided by 2 */
#define  RCC_CFGR_HPRE_DIV4                  ((uint32_t)0x00000090)        /*!< SYSCLK divided by 4 */
#define  RCC_CFGR_HPRE_DIV8                  ((uint32_t)0x000000A0)        /*!< SYSCLK divided by 8 */
#define  RCC_CFGR_HPRE_DIV16                 ((uint32_t)0x000000B0)        /*!< SYSCLK divided by 16 */
#define  RCC_CFGR_HPRE_DIV64                 ((uint32_t)0x000000C0)        /*!< SYSCLK divided by 64 */
#define  RCC_CFGR_HPRE_DIV128                ((uint32_t)0x000000D0)        /*!< SYSCLK divided by 128 */
#define  RCC_CFGR_HPRE_DIV256                ((uint32_t)0x000000E0)        /*!< SYSCLK divided by 256 */
#define  RCC_CFGR_HPRE_DIV512                ((uint32_t)0x000000F0)        /*!< SYSCLK divided by 512 */

/*!< PPRE1 configuration */
#define  RCC_CFGR_PPRE1                      ((uint32_t)0x00001C00)        /*!< PRE1[2:0] bits (APB1 prescaler) */
#define  RCC_CFGR_PPRE1_0                    ((uint32_t)0x00000400)        /*!< Bit 0 */
#define  RCC_CFGR_PPRE1_1                    ((uint32_t)0x00000800)        /*!< Bit 1 */
#define  RCC_CFGR_PPRE1_2                    ((uint32_t)0x00001000)        /*!< Bit 2 */

#define  RCC_CFGR_PPRE1_DIV1                 ((uint32_t)0x00000000)        /*!< HCLK not divided */
#define  RCC_CFGR_PPRE1_DIV2                 ((uint32_t)0x00001000)        /*!< HCLK divided by 2 */
#define  RCC_CFGR_PPRE1_DIV4                 ((uint32_t)0x00001400)        /*!< HCLK divided by 4 */
#define  RCC_CFGR_PPRE1_DIV8                 ((uint32_t)0x00001800)        /*!< HCLK divided by 8 */
#define  RCC_CFGR_PPRE1_DIV16                ((uint32_t)0x00001C00)        /*!< HCLK divided by 16 */

/*!< PPRE2 configuration */
#define  RCC_CFGR_PPRE2                      ((uint32_t)0x0000E000)        /*!< PRE2[2:0] bits (APB2 prescaler) */
#define  RCC_CFGR_PPRE2_0                    ((uint32_t)0x00002000)        /*!< Bit 0 */
#define  RCC_CFGR_PPRE2_1                    ((uint32_t)0x00004000)        /*!< Bit 1 */
#define  RCC_CFGR_PPRE2_2                    ((uint32_t)0x00008000)        /*!< Bit 2 */

#define  RCC_CFGR_PPRE2_DIV1                 ((uint32_t)0x00000000)        /*!< HCLK not divided */
#define  RCC_CFGR_PPRE2_DIV2                 ((uint32_t)0x00008000)        /*!< HCLK divided by 2 */
#define  RCC_CFGR_PPRE2_DIV4                 ((uint32_t)0x0000A000)        /*!< HCLK divided by 4 */
#define  RCC_CFGR_PPRE2_DIV8                 ((uint32_t)0x0000C000)        /*!< HCLK divided by 8 */
#define  RCC_CFGR_PPRE2_DIV16                ((uint32_t)0x0000E000)        /*!< HCLK divided by 16 */

/*!< RTCPRE configuration */
#define  RCC_CFGR_RTCPRE                     ((uint32_t)0x001F0000)
#define  RCC_CFGR_RTCPRE_0                   ((uint32_t)0x00010000)
#define  RCC_CFGR_RTCPRE_1                   ((uint32_t)0x00020000)
#define  RCC_CFGR_RTCPRE_2                   ((uint32_t)0x00040000)
#define  RCC_CFGR_RTCPRE_3                   ((uint32_t)0x00080000)
#define  RCC_CFGR_RTCPRE_4                   ((uint32_t)0x00100000)

/*!< MCO1 configuration */
#define  RCC_CFGR_MCO1                       ((uint32_t)0x00600000)
#define  RCC_CFGR_MCO1_0                     ((uint32_t)0x00200000)
#define  RCC_CFGR_MCO1_1                     ((uint32_t)0x00400000)

#define  RCC_CFGR_I2SSRC                     ((uint32_t)0x00800000)

#define  RCC_CFGR_MCO1PRE                    ((uint32_t)0x07000000)
#define  RCC_CFGR_MCO1PRE_0                  ((uint32_t)0x01000000)
#define  RCC_CFGR_MCO1PRE_1                  ((uint32_t)0x02000000)
#define  RCC_CFGR_MCO1PRE_2                  ((uint32_t)0x04000000)

#define  RCC_CFGR_MCO2PRE                    ((uint32_t)0x38000000)
#define  RCC_CFGR_MCO2PRE_0                  ((uint32_t)0x08000000)
#define  RCC_CFGR_MCO2PRE_1                  ((uint32_t)0x10000000)
#define  RCC_CFGR_MCO2PRE_2                  ((uint32_t)0x20000000)

#define  RCC_CFGR_MCO2                       ((uint32_t)0xC0000000)
#define  RCC_CFGR_MCO2_0                     ((uint32_t)0x40000000)
#define  RCC_CFGR_MCO2_1                     ((uint32_t)0x80000000)

/********************  Bit definition for RCC_CIR register  *******************/
#define  RCC_CIR_LSIRDYF                     ((uint32_t)0x00000001)
#define  RCC_CIR_LSERDYF                     ((uint32_t)0x00000002)
#define  RCC_CIR_HSIRDYF                     ((uint32_t)0x00000004)
#define  RCC_CIR_HSERDYF                     ((uint32_t)0x00000008)
#define  RCC_CIR_PLLRDYF                     ((uint32_t)0x00000010)
#define  RCC_CIR_PLLI2SRDYF                  ((uint32_t)0x00000020)
#define  RCC_CIR_CSSF                        ((uint32_t)0x00000080)
#define  RCC_CIR_LSIRDYIE                    ((uint32_t)0x00000100)
#define  RCC_CIR_LSERDYIE                    ((uint32_t)0x00000200)
#define  RCC_CIR_HSIRDYIE                    ((uint32_t)0x00000400)
#define  RCC_CIR_HSERDYIE                    ((uint32_t)0x00000800)
#define  RCC_CIR_PLLRDYIE                    ((uint32_t)0x00001000)
#define  RCC_CIR_PLLI2SRDYIE                 ((uint32_t)0x00002000)
#define  RCC_CIR_LSIRDYC                     ((uint32_t)0x00010000)
#define  RCC_CIR_LSERDYC                     ((uint32_t)0x00020000)
#define  RCC_CIR_HSIRDYC                     ((uint32_t)0x00040000)
#define  RCC_CIR_HSERDYC                     ((uint32_t)0x00080000)
#define  RCC_CIR_PLLRDYC                     ((uint32_t)0x00100000)
#define  RCC_CIR_PLLI2SRDYC                  ((uint32_t)0x00200000)
#define  RCC_CIR_CSSC                        ((uint32_t)0x00800000)

/********************  Bit definition for RCC_AHB1RSTR register  **************/
#define  RCC_AHB1RSTR_GPIOARST               ((uint32_t)0x00000001)
#define  RCC_AHB1RSTR_GPIOBRST               ((uint32_t)0x00000002)
#define  RCC_AHB1RSTR_GPIOCRST               ((uint32_t)0x00000004)
#define  RCC_AHB1RSTR_GPIODRST               ((uint32_t)0x00000008)
#define  RCC_AHB1RSTR_GPIOERST               ((uint32_t)0x00000010)
#define  RCC_AHB1RSTR_GPIOFRST               ((uint32_t)0x00000020)
#define  RCC_AHB1RSTR_GPIOGRST               ((uint32_t)0x00000040)
#define  RCC_AHB1RSTR_GPIOHRST               ((uint32_t)0x00000080)
#define  RCC_AHB1RSTR_GPIOIRST               ((uint32_t)0x00000100)
#define  RCC_AHB1RSTR_CRCRST                 ((uint32_t)0x00001000)
#define  RCC_AHB1RSTR_DMA1RST                ((uint32_t)0x00200000)
#define  RCC_AHB1RSTR_DMA2RST                ((uint32_t)0x00400000)
#define  RCC_AHB1RSTR_ETHMACRST              ((uint32_t)0x02000000)
#define  RCC_AHB1RSTR_OTGHRST                ((uint32_t)0x10000000)

/********************  Bit definition for RCC_AHB2RSTR register  **************/
#define  RCC_AHB2RSTR_DCMIRST                ((uint32_t)0x00000001)
#define  RCC_AHB2RSTR_CRYPRST                ((uint32_t)0x00000010)
#define  RCC_AHB2RSTR_HSAHRST                ((uint32_t)0x00000020)
#define  RCC_AHB2RSTR_RNGRST                 ((uint32_t)0x00000040)
#define  RCC_AHB2RSTR_OTGFSRST               ((uint32_t)0x00000080)

/********************  Bit definition for RCC_AHB3RSTR register  **************/
#define  RCC_AHB3RSTR_FSMCRST                ((uint32_t)0x00000001)

/********************  Bit definition for RCC_APB1RSTR register  **************/
#define  RCC_APB1RSTR_TIM2RST                ((uint32_t)0x00000001)
#define  RCC_APB1RSTR_TIM3RST                ((uint32_t)0x00000002)
#define  RCC_APB1RSTR_TIM4RST                ((uint32_t)0x00000004)
#define  RCC_APB1RSTR_TIM5RST                ((uint32_t)0x00000008)
#define  RCC_APB1RSTR_TIM6RST                ((uint32_t)0x00000010)
#define  RCC_APB1RSTR_TIM7RST                ((uint32_t)0x00000020)
#define  RCC_APB1RSTR_TIM12RST               ((uint32_t)0x00000040)
#define  RCC_APB1RSTR_TIM13RST               ((uint32_t)0x00000080)
#define  RCC_APB1RSTR_TIM14RST               ((uint32_t)0x00000100)
#define  RCC_APB1RSTR_WWDGEN                 ((uint32_t)0x00000800)
#define  RCC_APB1RSTR_SPI2RST                ((uint32_t)0x00008000)
#define  RCC_APB1RSTR_SPI3RST                ((uint32_t)0x00010000)
#define  RCC_APB1RSTR_USART2RST              ((uint32_t)0x00020000)
#define  RCC_APB1RSTR_USART3RST              ((uint32_t)0x00040000)
#define  RCC_APB1RSTR_UART4RST               ((uint32_t)0x00080000)
#define  RCC_APB1RSTR_UART5RST               ((uint32_t)0x00100000)
#define  RCC_APB1RSTR_I2C1RST                ((uint32_t)0x00200000)
#define  RCC_APB1RSTR_I2C2RST                ((uint32_t)0x00400000)
#define  RCC_APB1RSTR_I2C3RST                ((uint32_t)0x00800000)
#define  RCC_APB1RSTR_CAN1RST                ((uint32_t)0x02000000)
#define  RCC_APB1RSTR_CAN2RST                ((uint32_t)0x04000000)
#define  RCC_APB1RSTR_PWRRST                 ((uint32_t)0x10000000)
#define  RCC_APB1RSTR_DACRST                 ((uint32_t)0x20000000)

/********************  Bit definition for RCC_APB2RSTR register  **************/
#define  RCC_APB2RSTR_TIM1RST                ((uint32_t)0x00000001)
#define  RCC_APB2RSTR_TIM8RST                ((uint32_t)0x00000002)
#define  RCC_APB2RSTR_USART1RST              ((uint32_t)0x00000010)
#define  RCC_APB2RSTR_USART6RST              ((uint32_t)0x00000020)
#define  RCC_APB2RSTR_ADCRST                 ((uint32_t)0x00000100)
#define  RCC_APB2RSTR_SDIORST                ((uint32_t)0x00000800)
#define  RCC_APB2RSTR_SPI1RST                ((uint32_t)0x00001000)
#define  RCC_APB2RSTR_SYSCFGRST              ((uint32_t)0x00004000)
#define  RCC_APB2RSTR_TIM9RST                ((uint32_t)0x00010000)
#define  RCC_APB2RSTR_TIM10RST               ((uint32_t)0x00020000)
#define  RCC_APB2RSTR_TIM11RST               ((uint32_t)0x00040000)
/* Old SPI1RST bit definition, maintained for legacy purpose */
#define  RCC_APB2RSTR_SPI1                   RCC_APB2RSTR_SPI1RST

/********************  Bit definition for RCC_AHB1ENR register  ***************/
#define  RCC_AHB1ENR_GPIOAEN                 ((uint32_t)0x00000001)
#define  RCC_AHB1ENR_GPIOBEN                 ((uint32_t)0x00000002)
#define  RCC_AHB1ENR_GPIOCEN                 ((uint32_t)0x00000004)
#define  RCC_AHB1ENR_GPIODEN                 ((uint32_t)0x00000008)
#define  RCC_AHB1ENR_GPIOEEN                 ((uint32_t)0x00000010)
#define  RCC_AHB1ENR_GPIOFEN                 ((uint32_t)0x00000020)
#define  RCC_AHB1ENR_GPIOGEN                 ((uint32_t)0x00000040)
#define  RCC_AHB1ENR_GPIOHEN                 ((uint32_t)0x00000080)
#define  RCC_AHB1ENR_GPIOIEN                 ((uint32_t)0x00000100)
#define  RCC_AHB1ENR_CRCEN                   ((uint32_t)0x00001000)
#define  RCC_AHB1ENR_BKPSRAMEN               ((uint32_t)0x00040000)
#define  RCC_AHB1ENR_CCMDATARAMEN            ((uint32_t)0x00100000)
#define  RCC_AHB1ENR_DMA1EN                  ((uint32_t)0x00200000)
#define  RCC_AHB1ENR_DMA2EN                  ((uint32_t)0x00400000)
#define  RCC_AHB1ENR_ETHMACEN                ((uint32_t)0x02000000)
#define  RCC_AHB1ENR_ETHMACTXEN              ((uint32_t)0x04000000)
#define  RCC_AHB1ENR_ETHMACRXEN              ((uint32_t)0x08000000)
#define  RCC_AHB1ENR_ETHMACPTPEN             ((uint32_t)0x10000000)
#define  RCC_AHB1ENR_OTGHSEN                 ((uint32_t)0x20000000)
#define  RCC_AHB1ENR_OTGHSULPIEN             ((uint32_t)0x40000000)

/********************  Bit definition for RCC_AHB2ENR register  ***************/
#define  RCC_AHB2ENR_DCMIEN                  ((uint32_t)0x00000001)
#define  RCC_AHB2ENR_CRYPEN                  ((uint32_t)0x00000010)
#define  RCC_AHB2ENR_HASHEN                  ((uint32_t)0x00000020)
#define  RCC_AHB2ENR_RNGEN                   ((uint32_t)0x00000040)
#define  RCC_AHB2ENR_OTGFSEN                 ((uint32_t)0x00000080)

/********************  Bit definition for RCC_AHB3ENR register  ***************/
#define  RCC_AHB3ENR_FSMCEN                  ((uint32_t)0x00000001)

/********************  Bit definition for RCC_APB1ENR register  ***************/
#define  RCC_APB1ENR_TIM2EN                  ((uint32_t)0x00000001)
#define  RCC_APB1ENR_TIM3EN                  ((uint32_t)0x00000002)
#define  RCC_APB1ENR_TIM4EN                  ((uint32_t)0x00000004)
#define  RCC_APB1ENR_TIM5EN                  ((uint32_t)0x00000008)
#define  RCC_APB1ENR_TIM6EN                  ((uint32_t)0x00000010)
#define  RCC_APB1ENR_TIM7EN                  ((uint32_t)0x00000020)
#define  RCC_APB1ENR_TIM12EN                 ((uint32_t)0x00000040)
#define  RCC_APB1ENR_TIM13EN                 ((uint32_t)0x00000080)
#define  RCC_APB1ENR_TIM14EN                 ((uint32_t)0x00000100)
#define  RCC_APB1ENR_WWDGEN                  ((uint32_t)0x00000800)
#define  RCC_APB1ENR_SPI2EN                  ((uint32_t)0x00004000)
#define  RCC_APB1ENR_SPI3EN                  ((uint32_t)0x00008000)
#define  RCC_APB1ENR_USART2EN                ((uint32_t)0x00020000)
#define  RCC_APB1ENR_USART3EN                ((uint32_t)0x00040000)
#define  RCC_APB1ENR_UART4EN                 ((uint32_t)0x00080000)
#define  RCC_APB1ENR_UART5EN                 ((uint32_t)0x00100000)
#define  RCC_APB1ENR_I2C1EN                  ((uint32_t)0x00200000)
#define  RCC_APB1ENR_I2C2EN                  ((uint32_t)0x00400000)
#define  RCC_APB1ENR_I2C3EN                  ((uint32_t)0x00800000)
#define  RCC_APB1ENR_CAN1EN                  ((uint32_t)0x02000000)
#define  RCC_APB1ENR_CAN2EN                  ((uint32_t)0x04000000)
#define  RCC_APB1ENR_PWREN                   ((uint32_t)0x10000000)
#define  RCC_APB1ENR_DACEN                   ((uint32_t)0x20000000)

/********************  Bit definition for RCC_APB2ENR register  ***************/
#define  RCC_APB2ENR_TIM1EN                  ((uint32_t)0x00000001)
#define  RCC_APB2ENR_TIM8EN                  ((uint32_t)0x00000002)
#define  RCC_APB2ENR_USART1EN                ((uint32_t)0x00000010)
#define  RCC_APB2ENR_USART6EN                ((uint32_t)0x00000020)
#define  RCC_APB2ENR_ADC1EN                  ((uint32_t)0x00000100)
#define  RCC_APB2ENR_ADC2EN                  ((uint32_t)0x00000200)
#define  RCC_APB2ENR_ADC3EN                  ((uint32_t)0x00000400)
#define  RCC_APB2ENR_SDIOEN                  ((uint32_t)0x00000800)
#define  RCC_APB2ENR_SPI1EN                  ((uint32_t)0x00001000)
#define  RCC_APB2ENR_SYSCFGEN                ((uint32_t)0x00004000)
#define  RCC_APB2ENR_TIM11EN                 ((uint32_t)0x00040000)
#define  RCC_APB2ENR_TIM10EN                 ((uint32_t)0x00020000)
#define  RCC_APB2ENR_TIM9EN                  ((uint32_t)0x00010000)

/********************  Bit definition for RCC_AHB1LPENR register  *************/
#define  RCC_AHB1LPENR_GPIOALPEN             ((uint32_t)0x00000001)
#define  RCC_AHB1LPENR_GPIOBLPEN             ((uint32_t)0x00000002)
#define  RCC_AHB1LPENR_GPIOCLPEN             ((uint32_t)0x00000004)
#define  RCC_AHB1LPENR_GPIODLPEN             ((uint32_t)0x00000008)
#define  RCC_AHB1LPENR_GPIOELPEN             ((uint32_t)0x00000010)
#define  RCC_AHB1LPENR_GPIOFLPEN             ((uint32_t)0x00000020)
#define  RCC_AHB1LPENR_GPIOGLPEN             ((uint32_t)0x00000040)
#define  RCC_AHB1LPENR_GPIOHLPEN             ((uint32_t)0x00000080)
#define  RCC_AHB1LPENR_GPIOILPEN             ((uint32_t)0x00000100)
#define  RCC_AHB1LPENR_CRCLPEN               ((uint32_t)0x00001000)
#define  RCC_AHB1LPENR_FLITFLPEN             ((uint32_t)0x00008000)
#define  RCC_AHB1LPENR_SRAM1LPEN             ((uint32_t)0x00010000)
#define  RCC_AHB1LPENR_SRAM2LPEN             ((uint32_t)0x00020000)
#define  RCC_AHB1LPENR_BKPSRAMLPEN           ((uint32_t)0x00040000)
#define  RCC_AHB1LPENR_DMA1LPEN              ((uint32_t)0x00200000)
#define  RCC_AHB1LPENR_DMA2LPEN              ((uint32_t)0x00400000)
#define  RCC_AHB1LPENR_ETHMACLPEN            ((uint32_t)0x02000000)
#define  RCC_AHB1LPENR_ETHMACTXLPEN          ((uint32_t)0x04000000)
#define  RCC_AHB1LPENR_ETHMACRXLPEN          ((uint32_t)0x08000000)
#define  RCC_AHB1LPENR_ETHMACPTPLPEN         ((uint32_t)0x10000000)
#define  RCC_AHB1LPENR_OTGHSLPEN             ((uint32_t)0x20000000)
#define  RCC_AHB1LPENR_OTGHSULPILPEN         ((uint32_t)0x40000000)

/********************  Bit definition for RCC_AHB2LPENR register  *************/
#define  RCC_AHB2LPENR_DCMILPEN              ((uint32_t)0x00000001)
#define  RCC_AHB2LPENR_CRYPLPEN              ((uint32_t)0x00000010)
#define  RCC_AHB2LPENR_HASHLPEN              ((uint32_t)0x00000020)
#define  RCC_AHB2LPENR_RNGLPEN               ((uint32_t)0x00000040)
#define  RCC_AHB2LPENR_OTGFSLPEN             ((uint32_t)0x00000080)

/********************  Bit definition for RCC_AHB3LPENR register  *************/
#define  RCC_AHB3LPENR_FSMCLPEN              ((uint32_t)0x00000001)

/********************  Bit definition for RCC_APB1LPENR register  *************/
#define  RCC_APB1LPENR_TIM2LPEN              ((uint32_t)0x00000001)
#define  RCC_APB1LPENR_TIM3LPEN              ((uint32_t)0x00000002)
#define  RCC_APB1LPENR_TIM4LPEN              ((uint32_t)0x00000004)
#define  RCC_APB1LPENR_TIM5LPEN              ((uint32_t)0x00000008)
#define  RCC_APB1LPENR_TIM6LPEN              ((uint32_t)0x00000010)
#define  RCC_APB1LPENR_TIM7LPEN              ((uint32_t)0x00000020)
#define  RCC_APB1LPENR_TIM12LPEN             ((uint32_t)0x00000040)
#define  RCC_APB1LPENR_TIM13LPEN             ((uint32_t)0x00000080)
#define  RCC_APB1LPENR_TIM14LPEN             ((uint32_t)0x00000100)
#define  RCC_APB1LPENR_WWDGLPEN              ((uint32_t)0x00000800)
#define  RCC_APB1LPENR_SPI2LPEN              ((uint32_t)0x00004000)
#define  RCC_APB1LPENR_SPI3LPEN              ((uint32_t)0x00008000)
#define  RCC_APB1LPENR_USART2LPEN            ((uint32_t)0x00020000)
#define  RCC_APB1LPENR_USART3LPEN            ((uint32_t)0x00040000)
#define  RCC_APB1LPENR_UART4LPEN             ((uint32_t)0x00080000)
#define  RCC_APB1LPENR_UART5LPEN             ((uint32_t)0x00100000)
#define  RCC_APB1LPENR_I2C1LPEN              ((uint32_t)0x00200000)
#define  RCC_APB1LPENR_I2C2LPEN              ((uint32_t)0x00400000)
#define  RCC_APB1LPENR_I2C3LPEN              ((uint32_t)0x00800000)
#define  RCC_APB1LPENR_CAN1LPEN              ((uint32_t)0x02000000)
#define  RCC_APB1LPENR_CAN2LPEN              ((uint32_t)0x04000000)
#define  RCC_APB1LPENR_PWRLPEN               ((uint32_t)0x10000000)
#define  RCC_APB1LPENR_DACLPEN               ((uint32_t)0x20000000)

/********************  Bit definition for RCC_APB2LPENR register  *************/
#define  RCC_APB2LPENR_TIM1LPEN              ((uint32_t)0x00000001)
#define  RCC_APB2LPENR_TIM8LPEN              ((uint32_t)0x00000002)
#define  RCC_APB2LPENR_USART1LPEN            ((uint32_t)0x00000010)
#define  RCC_APB2LPENR_USART6LPEN            ((uint32_t)0x00000020)
#define  RCC_APB2LPENR_ADC1LPEN              ((uint32_t)0x00000100)
#define  RCC_APB2LPENR_ADC2PEN               ((uint32_t)0x00000200)
#define  RCC_APB2LPENR_ADC3LPEN              ((uint32_t)0x00000400)
#define  RCC_APB2LPENR_SDIOLPEN              ((uint32_t)0x00000800)
#define  RCC_APB2LPENR_SPI1LPEN              ((uint32_t)0x00001000)
#define  RCC_APB2LPENR_SYSCFGLPEN            ((uint32_t)0x00004000)
#define  RCC_APB2LPENR_TIM9LPEN              ((uint32_t)0x00010000)
#define  RCC_APB2LPENR_TIM10LPEN             ((uint32_t)0x00020000)
#define  RCC_APB2LPENR_TIM11LPEN             ((uint32_t)0x00040000)

/********************  Bit definition for RCC_BDCR register  ******************/
#define  RCC_BDCR_LSEON                      ((uint32_t)0x00000001)
#define  RCC_BDCR_LSERDY                     ((uint32_t)0x00000002)
#define  RCC_BDCR_LSEBYP                     ((uint32_t)0x00000004)

#define  RCC_BDCR_RTCSEL                    ((uint32_t)0x00000300)
#define  RCC_BDCR_RTCSEL_0                  ((uint32_t)0x00000100)
#define  RCC_BDCR_RTCSEL_1                  ((uint32_t)0x00000200)

#define  RCC_BDCR_RTCEN                      ((uint32_t)0x00008000)
#define  RCC_BDCR_BDRST                      ((uint32_t)0x00010000)

/********************  Bit definition for RCC_CSR register  *******************/
#define  RCC_CSR_LSION                       ((uint32_t)0x00000001)
#define  RCC_CSR_LSIRDY                      ((uint32_t)0x00000002)
#define  RCC_CSR_RMVF                        ((uint32_t)0x01000000)
#define  RCC_CSR_BORRSTF                     ((uint32_t)0x02000000)
#define  RCC_CSR_PADRSTF                     ((uint32_t)0x04000000)
#define  RCC_CSR_PORRSTF                     ((uint32_t)0x08000000)
#define  RCC_CSR_SFTRSTF                     ((uint32_t)0x10000000)
#define  RCC_CSR_WDGRSTF                     ((uint32_t)0x20000000)
#define  RCC_CSR_WWDGRSTF                    ((uint32_t)0x40000000)
#define  RCC_CSR_LPWRRSTF                    ((uint32_t)0x80000000)

/********************  Bit definition for RCC_SSCGR register  *****************/
#define  RCC_SSCGR_MODPER                    ((uint32_t)0x00001FFF)
#define  RCC_SSCGR_INCSTEP                   ((uint32_t)0x0FFFE000)
#define  RCC_SSCGR_SPREADSEL                 ((uint32_t)0x40000000)
#define  RCC_SSCGR_SSCGEN                    ((uint32_t)0x80000000)

/********************  Bit definition for RCC_PLLI2SCFGR register  ************/
#define  RCC_PLLI2SCFGR_PLLI2SN              ((uint32_t)0x00007FC0)
#define  RCC_PLLI2SCFGR_PLLI2SR              ((uint32_t)0x70000000)

#define RCC_PLLI2SRDY_Pos		27
#define RCC_PLLI2SON_Pos		26
#define RCC_PLLRDY_Pos			25
#define RCC_PLLON_Pos			24
#define RCC_CSSON_Pos			19
#define RCC_HSEBYP_Pos			18
#define RCC_HSERDY_Pos			17
#define RCC_HSEON_Pos			16
#define RCC_HSIRDY_Pos			1
#define RCC_HSION_Pos			0
/******************************************************************************/
/*                                                                            */
/*                         General Purpose IOs (GPIO)                         */
/*                                                                            */
/******************************************************************************/

/*
 * Clock Enable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()	RCC->AHB1ENR |=(1 <<0)
#define GPIOB_PCLK_EN()	RCC->AHB1ENR |=(1 <<1)
#define GPIOC_PCLK_EN()	RCC->AHB1ENR |=(1 <<2)
#define GPIOD_PCLK_EN()	RCC->AHB1ENR |=(1 <<3)
#define GPIOE_PCLK_EN()	RCC->AHB1ENR |=(1 <<4)
#define GPIOF_PCLK_EN()	RCC->AHB1ENR |=(1 <<5)
#define GPIOG_PCLK_EN()	RCC->AHB1ENR |=(1 <<6)
#define GPIOH_PCLK_EN()	RCC->AHB1ENR |=(1 <<7)
#define GPIOI_PCLK_EN()	RCC->AHB1ENR |=(1 <<8)





/*
 * Clock Enable macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN() RCC->APB1ENR |=(1 << 21)
#define I2C2_PCLK_EN() RCC->APB1ENR |=(1 << 22)
#define I2C3_PCLK_EN() RCC->APB1ENR |=(1 << 23)

/*
 * Clock Enable macros for SPIx peripherals
 */

#define SPI1_PCLK_EN() RCC->APB2ENR |= (1<<12)
#define SPI2_PCLK_EN() RCC->APB1ENR |= (1<<14)
#define SPI3_PCLK_EN() RCC->APB1ENR |= (1<<15)


/*
 * Clock Enable macros for SYSGFG peripherals
 */

#define SYSGFG_PCLK_EN() RCC->APB2ENR |= (1<<14)

/*
 *  Macros to rest GPIOx Peripherals
 */

#define GPIOA_REG_RESET()		do { (RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &=~(1<<0));}while(0)
#define GPIOB_REG_RESET()		do { (RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &=~(1<<1));}while(0)
#define GPIOC_REG_RESET()		do { (RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &=~(1<<2));}while(0)
/*#define GPIOD_REG_RESET()		do { (RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &=~(1<<0));}while(0)
#define GPIOE_REG_RESET()		do { (RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &=~(1<<0));}while(0)
#define GPIOF_REG_RESET()		do { (RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &=~(1<<0));}while(0)
#define GPIOG_REG_RESET()		do { (RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &=~(1<<0));}while(0)
#define GPIOH_REG_RESET()		do { (RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &=~(1<<0));}while(0)
#define GPIOI_REG_RESET()		do { (RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &=~(1<<0));}while(0) */

#define GPIO_BASEADDR_TO_CODE(x)      (x == GPIOA ? 0:\
									  x == GPIOB ? 1:\
									  x == GPIOC ? 2:\
									  x == GPIOD ? 3:\
									  x == GPIOE ? 4:\
									  x == GPIOF ? 5:\
							          x == GPIOG ? 6:\
							          x == GPIOH ? 7:0)


/*
 * Defines IRQ number for EXTI
 */

#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

/*
 *   NVIC IRQ PRIORITY
 */

#define NVIC_IRQ_PRIO0		0
#define NVIC_IRQ_PRIO1		1
#define NVIC_IRQ_PRIO2		2
#define NVIC_IRQ_PRIO3		3
#define NVIC_IRQ_PRIO4		4
#define NVIC_IRQ_PRIO5		5
#define NVIC_IRQ_PRIO6		6
#define NVIC_IRQ_PRIO7		7
#define NVIC_IRQ_PRIO8		8
#define NVIC_IRQ_PRIO9		9
#define NVIC_IRQ_PRIO10		10
#define NVIC_IRQ_PRIO11		11
#define NVIC_IRQ_PRIO12		12
#define NVIC_IRQ_PRIO13		13
#define NVIC_IRQ_PRIO14		14
#define NVIC_IRQ_PRIO15		15

/*
 *  Some generic macros
 */

#define ENABLE			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define SET_PENDING		SET
#define CLEAR_PENDING	RESET
#define INT_NOT_ACTIVE	DISABLE
#define INT_ACTIVE		ENABLE

#define NO_PR_BITS_IMPLEMENTED 			4

typedef enum
{
  SUCCESS = 0,
  ERROR = !SUCCESS
} ErrorStatus;



#endif /* STM32F407_H_ */
