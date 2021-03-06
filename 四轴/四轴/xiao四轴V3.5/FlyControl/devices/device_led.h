/*=============================================================================
** 作者：侯玉鹏
** 日期：2017/10/31
** 版本：V1.0
** 头文件
=============================================================================*/
#ifndef _DEVICE_LED_H_
#define	_DEVICE_LED_H_

/*============================ INCLUDES ======================================*/
#include "stm32f10x.h"
/*============================ MACROS ========================================*/

/*============================ TYPES =========================================*/

/*============================ GLOBAL VARIABLES ==============================*/
/*============================ STATIC VARIABLES ==============================*/

/*============================ FUNCTION ======================================*/

/*----------------------------------------------------------
 + 实现功能：由任务调度调用周期2ms
 + 调用参数功能：被读取的加速度计陀螺仪数据数组
----------------------------------------------------------*/

/* LED 电平宏定义 */
#define LED1_OFF         GPIOA->BRR   = Pin_LED1
#define LED1_ON          GPIOA->BSRR  = Pin_LED1
#define LED2_OFF         GPIOA->BRR   = Pin_LED2
#define LED2_ON          GPIOA->BSRR  = Pin_LED2
#define LED3_OFF         GPIOA->BRR   = Pin_LED3
#define LED3_ON          GPIOA->BSRR  = Pin_LED3
#define LED4_OFF         GPIOB->BRR   = Pin_LED4
#define LED4_ON          GPIOB->BSRR  = Pin_LED4

/* LED 的GPIO宏定义 */
#define RCC_LED123		RCC_APB2Periph_GPIOA
#define RCC_LED4		RCC_APB2Periph_GPIOB

#define Pin_LED1		GPIO_Pin_9
#define Pin_LED2		GPIO_Pin_1
#define Pin_LED3		GPIO_Pin_0
#define Pin_LED4		GPIO_Pin_0

extern void Fc_led_init(void);
extern void Fc_led_control(u8 lednum,u8 stats,u8 timems);
extern void Fc_led_mpu_err(void);
extern void Fc_led_baro_err(void);



#endif
