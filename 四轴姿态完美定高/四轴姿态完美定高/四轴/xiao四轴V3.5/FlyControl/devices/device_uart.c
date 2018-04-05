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
#include "device_Uart.h"
#include "device_Flow.h"
#include "device_Bluetooth.h"

#include "Fc_data_Transfer.h"
/*============================ INCLUDES ======================================*/


/*============================ MACROS ========================================*/


/*============================ TYPES =========================================*/

/*============================ GLOBAL VARIABLES ==============================*/
/*============================ STATIC VARIABLES ==============================*/

u8 TxBuffer2[256];
u8 TxCounter2=0;
u8 count2=0;


u8 TxBuffer5[256];
u8 TxCounter5=0;
u8 count5=0;

u8 TxBuffer4[256];
u8 TxCounter4=0;
u8 count4=0;

/*============================ FUNCTION ======================================*/

/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
void Fc_Uart4_Init(u32 br_num)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    USART_ClockInitTypeDef USART_ClockInitStruct;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO,ENABLE);
    //Tx
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC , &GPIO_InitStructure);
    //Rx
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC , &GPIO_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //时钟低电平活动
    USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK引脚上时钟输出的极性->低电平
    USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //时钟第二个边沿进行数据捕获
    USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出
    USART_ClockInit(UART4, &USART_ClockInitStruct);
    USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
    USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
    USART_Init(UART4, &USART_InitStructure);
    //使能USART4接收中断
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
    //使能USART4
    USART_Cmd(UART4, ENABLE);
}
/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
void UART4_IRQHandler(void)
{
    u8 com_data;
    if(UART4->SR & USART_SR_ORE)
        com_data = UART4->DR;

    if( USART_GetITStatus(UART4,USART_IT_RXNE) )
    {
        USART_ClearITPendingBit(UART4,USART_IT_RXNE);
        com_data = UART4->DR;
        Bluet_Receive_Prepare(com_data);
    }

    if( USART_GetITStatus(UART4,USART_IT_TXE ) )
        UART4->DR = TxBuffer4[TxCounter4++];
    
        if(TxCounter4 >= count4)
            UART4->CR1 &= ~USART_CR1_TXEIE;
    }
/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
void Fc_Uart4_Put_Buf(unsigned char *DataToSend , u8 data_num)
{
    for(u8 i=0; i<data_num; i++)
        TxBuffer4[count4++] = *(DataToSend+i);

    if(!(UART4->CR1 & USART_CR1_TXEIE))
        USART_ITConfig(UART4, USART_IT_TXE, ENABLE);
}


