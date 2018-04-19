/*============================================================================
** 作者：侯玉鹏
** 日期：2017/10/31
** 版本：V1.0
** 函数名:
** 函数功能：
** 输入参数：
** 输出 ：1  返回值：
          2  改变的全局变量
** 本函数调用的其它函数（版本号）：
** 备注：
** 修改日志： （作者，日期，修改内容及原因）
==============================================================================*/
#ifndef _DRVICE_PWM_OUT_H_
#define _DRVICE_PWM_OUT_H_

/*============================ INCLUDES ======================================*/
#include "stm32f10x.h"

/*============================ MACROS ========================================*/


/*============================ TYPES =========================================*/

/*============================ GLOBAL VARIABLES ==============================*/

/*============================ STATIC VARIABLES ==============================*/

/*============================ FUNCTION ======================================*/


/*----------------------------------------------------------
 + 实现功能：PWM输出初始化
 + 调用参数功能：uint16_t hz：PWM输出的频率
----------------------------------------------------------*/
extern void Fc_Pwm_Out_Config(void);

#endif

