/*=============================================================================
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

/*============================ INCLUDES ======================================*/
#include "device_led.h"
#include "Fc_systream_time.h"
#include "Fc_data_Struct.h"
/*============================ MACROS ========================================*/

/*============================ TYPES =========================================*/

/*============================ GLOBAL VARIABLES ==============================*/
extern FlyControlDataStruct g_FlyControlDataStruct;
/*============================ STATIC VARIABLES ==============================*/
u32 g_led_cnt;
u32 g_led_num;
/*============================ FUNCTION ======================================*/
#define RCC_LED123		RCC_APB2Periph_GPIOA
#define RCC_LED4		RCC_APB2Periph_GPIOB

/*----------------------------------------------------------
 + 实现功能：控制LED初始化
----------------------------------------------------------*/
void Fc_led_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_LED123 | RCC_LED4, ENABLE );

    GPIO_InitStructure.GPIO_Pin =  Pin_LED1| Pin_LED2| Pin_LED3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin =  Pin_LED4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

}

/* 100ms times /100ms*/
void Fc_led_control(u8 lednum,u8 stats,u8 timems)
{
    g_led_cnt++;

    if(g_led_cnt == timems)
    {
        g_led_num++;
        g_led_cnt=0;

    }
    switch(stats)
    {
    case Init_OK:
        LED1_ON;
        LED2_ON;
        LED3_ON;
        LED4_ON;
        break;

    case Bat_low :

        if(g_led_num%4 == 0)
        {
            LED1_ON;
            LED2_ON;
            LED3_ON;
            LED4_ON;
        }
        else
        {
            LED3_OFF;
            LED4_OFF;
            LED1_OFF;
            LED2_OFF;
        }
        break;

    case Rc_lost:

        if(g_led_num%2 == 0)
        {
            LED1_ON;
            LED2_ON;
            LED3_OFF;
            LED4_OFF;
        }
        else
        {
            LED3_ON;
            LED4_ON;
            LED1_OFF;
            LED2_OFF;
        }
        break;
    }

    if(g_led_num>=1000)
    {
        g_led_num=0;
    }
}

void Fc_led_mpu_err(void)
{
    LED1_OFF;
    LED2_OFF;
    LED3_OFF;
    LED4_OFF;

    while(1)
    {
        LED1_ON;
        LED3_OFF;
        Delay_ms(150);
        LED1_OFF;
        LED3_ON;
        Delay_ms(150);
    }
}


void Fc_led_baro_err(void)
{
    LED1_OFF;
    LED2_OFF;
    LED3_OFF;
    LED4_OFF;

    while(1)
    {
        LED2_ON;
        LED4_OFF;
        Delay_ms(150);
        LED2_OFF;
        LED4_ON;
        Delay_ms(150);
    }
}
