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
#include "device_spi.h"
#include "stm32f10x.h"
/*============================ MACROS ========================================*/

/*============================ TYPES =========================================*/

/*============================ GLOBAL VARIABLES ==============================*/
/*============================ STATIC VARIABLES ==============================*/

/*============================ FUNCTION ======================================*/

/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
void Fc_Spi_Init(void)
{
    SPI_InitTypeDef SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_GPIO_SPI, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_GPIO_CE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

    /*配置 SPI_NRF_SPI的 SCK,MISO,MOSI引脚 */
    GPIO_InitStructure.GPIO_Pin = SPI_Pin_SCK| SPI_Pin_MISO| SPI_Pin_MOSI;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用功能
    GPIO_Init(GPIO_SPI, &GPIO_InitStructure);
    /*SPI_NRF_SPI的 CSN 引脚:*/
    GPIO_InitStructure.GPIO_Pin = SPI_Pin_CSN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIO_SPI, &GPIO_InitStructure);

    GPIO_SetBits(GPIO_SPI, SPI_Pin_CSN);

    /*SPI_NRF_SPI的 CE 引脚:*/
    GPIO_InitStructure.GPIO_Pin = SPI_Pin_CE;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIO_CE, &GPIO_InitStructure);

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //双线全双工
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master; //主模式
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; //数据大小8位
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; //时钟极性，空闲时为低
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //第1个边沿有效，上升沿为采样时刻
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; //NSS信号由软件产生
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //4分频，9MHz
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; //高位在前
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &SPI_InitStructure);
    /* Enable SPI1 */
    SPI_Cmd(SPI1, ENABLE);

}
/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
u8 SPI_RW(u8 dat)
{
    /* 当 SPI发送缓冲器非空时等待 */
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    /* 通过 SPI2发送一字节数据 */
    SPI_I2S_SendData(SPI1, dat);
    /* 当SPI接收缓冲器为空时等待 */
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    /* Return the byte read from the SPI bus */
    return SPI_I2S_ReceiveData(SPI1);
}
/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
void SPI_CSN_H(void)
{
    GPIO_SetBits(GPIO_SPI, SPI_Pin_CSN);
}
/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
void SPI_CSN_L(void)
{
    GPIO_ResetBits(GPIO_SPI, SPI_Pin_CSN);
}
/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
void SPI_CE_H(void)
{
    GPIO_SetBits(GPIO_CE, SPI_Pin_CE);
}
/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
void SPI_CE_L(void)
{
    GPIO_ResetBits(GPIO_CE, SPI_Pin_CE);
}

/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
void Fc_spi2_Init(void)
{
    SPI_InitTypeDef SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;


    RCC_APB2PeriphClockCmd(	RCC_GPIO_SPI2|RCC_APB2Periph_AFIO, ENABLE );
    RCC_APB1PeriphClockCmd(	RCC_APB1Periph_SPI2,ENABLE );

    GPIO_InitStructure.GPIO_Pin = SPI2_Pin_SCK|SPI2_Pin_MISO|SPI2_Pin_MOSI ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIO_SPI2, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = SPI2_Pin_CSN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIO_SPI2, &GPIO_InitStructure);
    GPIO_SetBits(GPIO_SPI2,SPI2_Pin_CSN);

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//串行同步时钟的空闲状态为高电平
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;		//定义波特率预分频的值:波特率预分频值为256
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
    SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
    SPI_Init(SPI2, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
    SPI_Cmd(SPI2, ENABLE); //使能SPI外设
}
/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
u8 SPI2_RW(u8 dat)
{
    /* 当 SPI发送缓冲器非空时等待 */
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
    /* 通过 SPI2发送一字节数据 */
    SPI_I2S_SendData(SPI2, dat);
    /* 当SPI接收缓冲器为空时等待 */
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
    /* Return the byte read from the SPI bus */
    return SPI_I2S_ReceiveData(SPI2);
}
/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
void SPI2_CSN_H(void)
{
    GPIO_SetBits(GPIO_SPI2, SPI_Pin_CSN);
}
/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
void SPI2_CSN_L(void)
{
    GPIO_ResetBits(GPIO_SPI2, SPI_Pin_CSN);
}


