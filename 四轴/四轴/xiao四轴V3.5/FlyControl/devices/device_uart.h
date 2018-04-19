
/*=============================================================================
** 作者：侯玉鹏
** 日期：2017/10/31
** 版本：V1.0
** 头文件
=============================================================================*/

#ifndef ___DRV_UART_H__
#define ___DRV_UART_H__
/*============================ INCLUDES ======================================*/

#include "basetype.h"
/*============================ MACROS ========================================*/
#define UART4_CLK          RCC_APB1Periph_UART4
#define UART4_GPIO_CLK     RCC_APB2Periph_GPIOA
#define UART4_GPIO         GPIOC
#define UART4_TxPin        GPIO_Pin_10
#define UART4_RxPin        GPIO_Pin_11

#define U4_SendMaxSize        25
#define U4_RxMaxSize     	  25 //定义最大接收字节数
#define U4_Rx_ing              0
#define U4_Rx_OK               1
#define U4_Tx_ing              0
#define U4_Tx_OK               1
#define U4_BAUD              9600

/*============================ TYPES =========================================*/

/*============================ GLOBAL VARIABLES ==============================*/

/*============================ STATIC VARIABLES ==============================*/

/*============================ FUNCTION ======================================*/


/***************硬件中断优先级******************/


extern u8 RxState1;

extern void Fc_Uart4_Init(u32 br_num);

void Fc_Uart4_Put_Buf(unsigned char *DataToSend , u8 data_num);


#endif
