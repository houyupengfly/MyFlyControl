/*=============================================================================
** 作者：侯玉鹏
** 日期：2017/10/31
** 版本：V1.0
** 头文件
=============================================================================*/

#ifndef __DRV_SPI_H__
#define __DRV_SPI_H__
/*============================ INCLUDES ======================================*/
#include "basetype.h"
#include "stm32f10x.h"
/*============================ MACROS ========================================*/


#define GPIO_SPI		    GPIOA
#define GPIO_CE			    GPIOC
#define RCC_GPIO_SPI		RCC_APB2Periph_GPIOA
#define RCC_GPIO_CE			RCC_APB2Periph_GPIOC

#define SPI_Pin_CE			GPIO_Pin_4
#define SPI_Pin_CSN			GPIO_Pin_4
#define SPI_Pin_SCK			GPIO_Pin_5
#define SPI_Pin_MISO		GPIO_Pin_6
#define SPI_Pin_MOSI		GPIO_Pin_7


#define GPIO_SPI2		    GPIOB
#define RCC_GPIO_SPI2		RCC_APB2Periph_GPIOB

#define SPI2_Pin_CSN		GPIO_Pin_12
#define SPI2_Pin_SCK		GPIO_Pin_13
#define SPI2_Pin_MISO		GPIO_Pin_14
#define SPI2_Pin_MOSI		GPIO_Pin_15

/*============================ TYPES =========================================*/

/*============================ GLOBAL VARIABLES ==============================*/
/*============================ STATIC VARIABLES ==============================*/

/*============================ FUNCTION ======================================*/
extern void Fc_Spi_Init(void);
extern u8   SPI_RW(u8 dat);
extern void SPI_CE_H(void);
extern void SPI_CE_L(void);
extern void SPI_CSN_H(void);
extern void SPI_CSN_L(void);

extern void Fc_spi2_Init(void);
extern u8 SPI2_RW(u8 dat);
extern void SPI2_CSN_H(void);
extern void SPI2_CSN_L(void);
#endif










