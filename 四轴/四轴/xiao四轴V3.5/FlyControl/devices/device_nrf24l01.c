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
#include "device_Nrf24l01.h"
#include "device_spi.h"

#include "Fc_data_transfer.h"
/*============================ MACROS ========================================*/

/*============================ TYPES =========================================*/

/*============================ GLOBAL VARIABLES ==============================*/
/*============================ STATIC VARIABLES ==============================*/
u8	TX_ADDRESS[TX_ADR_WIDTH] = {0xAA,0xBB,0xCC,0x00,0x01};	//本地地址
u8	RX_ADDRESS[RX_ADR_WIDTH] = {0xAA,0xBB,0xCC,0x00,0x01};	//接收地址

u8 NRF24L01_2_RXDATA[RX_PLOAD_WIDTH];//nrf24l01接收到的数据
u8 NRF24L01_2_TXDATA[RX_PLOAD_WIDTH];//nrf24l01需要发送的数据

u8 NRF_SSI,NRF_SSI_CNT;//NRF信号强度
u16 Nrf_Erro;
/*============================ FUNCTION ======================================*/


/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
u8 NRF_Write_Reg(u8 reg, u8 value)
{
    u8 status;
    SPI_CSN_L();					  /* 选通器件 */
    status = SPI_RW(reg);  /* 写寄存器地址 */
    SPI_RW(value);		  /* 写数据 */
    SPI_CSN_H();					  /* 禁止该器件 */
    return 	status;
}
/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
u8 NRF_Read_Reg(u8 reg)
{
    u8 reg_val;
    SPI_CSN_L();					  /* 选通器件 */
    SPI_RW(reg);			  /* 写寄存器地址 */
    reg_val = SPI_RW(0);	  /* 读取该寄存器返回数据 */
    SPI_CSN_H();					  /* 禁止该器件 */
    return 	reg_val;
}

/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
u8 NRF_Write_Buf(u8 reg, u8 *pBuf, u8 uchars)
{
    u8 i;
    u8 status;
    SPI_CSN_L();				        /* 选通器件 */
    status = SPI_RW(reg);	/* 写寄存器地址 */

    for(i=0; i<uchars; i++)
    {
        SPI_RW(pBuf[i]);		/* 写数据 */
    }

    SPI_CSN_H();						/* 禁止该器件 */
    return 	status;
}
/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
u8 NRF_Read_Buf(u8 reg, u8 *pBuf, u8 uchars)
{
    u8 i;
    u8 status;
    SPI_CSN_L();						/* 选通器件 */
    status = SPI_RW(reg);	/* 写寄存器地址 */

    for(i=0; i<uchars; i++)
    {
        pBuf[i] = SPI_RW(0); /* 读取返回数据 */
    }

    SPI_CSN_H();						/* 禁止该器件 */
    return 	status;
}

/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
void NRF_TxPacket(u8 * tx_buf, u8 len)
{
    SPI_CE_L();		 //StandBy I模式
    NRF_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // 装载接收端地址
    NRF_Write_Buf(WR_TX_PLOAD, tx_buf, len); 			 // 装载数据
    SPI_CE_H();		 //置高CE，激发数据发送
}
/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
void Nrf_TxPacket_AP(u8 * tx_buf, u8 len)
{
    SPI_CE_L();		 //StandBy I模式
    NRF_Write_Buf(0xa8, tx_buf, len); 			 // 装载数据
    SPI_CE_H();		 //置高CE
}


/*=============================================================================
+ 实现功能：
+ 调用参数功能：
==============================================================================*/
void Fc_Nrf_Init(u8 model, u8 ch)
{
    SPI_CE_L();
    NRF_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADR_WIDTH);	//写RX节点地址
    NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,TX_ADR_WIDTH); 		//写TX节点地址
    NRF_Write_Reg(NRF_WRITE_REG+EN_AA,0x01); 							//使能通道0的自动应答
    NRF_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);						//使能通道0的接收地址
    NRF_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);						//设置自动重发间隔时间:500us;最大自动重发次数:10次 2M波特率下
    NRF_Write_Reg(NRF_WRITE_REG+RF_CH,ch);								//设置RF通道为CHANAL
    NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f); 					    //设置TX发射参数,0db增益,2Mbps,低噪声增益开启


    if(model==1)				//RX
    {
        NRF_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);								//选择通道0的有效数据宽度
        NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);   		 // IRQ收发完成中断开启,16位CRC,主接收
    }
    else if(model==2)		//TX
    {
        NRF_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);								//选择通道0的有效数据宽度
        NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);   		 // IRQ收发完成中断开启,16位CRC,主发送
    }
    else if(model==3)		//RX2
    {
        NRF_Write_Reg(FLUSH_TX,0xff);
        NRF_Write_Reg(FLUSH_RX,0xff);
        NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);   		 // IRQ收发完成中断开启,16位CRC,主接收
        SPI_RW(0x50);
        SPI_RW(0x73);
        NRF_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
        NRF_Write_Reg(NRF_WRITE_REG+0x1d,0x06);
    }
    else								//TX2
    {
        NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);   		 // IRQ收发完成中断开启,16位CRC,主发送
        NRF_Write_Reg(FLUSH_TX,0xff);
        NRF_Write_Reg(FLUSH_RX,0xff);
        SPI_RW(0x50);
        SPI_RW(0x73);
        NRF_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
        NRF_Write_Reg(NRF_WRITE_REG+0x1d,0x06);
    }

    SPI_CE_H();
}

u8 Nrf_Check(void)
{
    u8 buf[5]= {0XA0,0XA0,0XA0,0XA0,0XA0};
    u8 i;
    u8 buf1[5];
    NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.
    NRF_Read_Buf(TX_ADDR,buf1,5); //读出写入的地址

    for(i=0; i<5; i++)if(buf[i]!=buf1[i])break;

    if(i!=5)return 0;//检测24L01错误

    return 1;		 //检测到24L01
}

u8 Nrf_Check_Event(void)
{
    u8 rx_len;
    u8 sta = NRF_Read_Reg(NRF_READ_REG + NRFRegSTATUS);

    if(sta & (1<<RX_DR))
    {
        rx_len = NRF_Read_Reg(R_RX_PL_WID);

        if(rx_len<33)
        {
            NRF_Read_Buf(RD_RX_PLOAD,NRF24L01_2_RXDATA,rx_len);// read receive payload from RX_FIFO buffer
            Nrf_Erro=0;
        }
        else
        {
            NRF_Write_Reg(FLUSH_RX,0xff);//清空缓冲区
        }
    }

    if(sta & (1<<TX_DS))
    {
    }

    if(sta & (1<<MAX_RT))
    {
        if(sta & 0x01)	//TX FIFO FULL
        {
            NRF_Write_Reg(FLUSH_TX,0xff);
        }
    }

    NRF_Write_Reg(NRF_WRITE_REG + NRFRegSTATUS, sta);
    return rx_len;
}

u8 Nrf_Connect(void)
{
    static u8 Connect_flag;
    Nrf_Erro ++;

    if(Nrf_Erro==1)
    {
        DT_Data_Receive_Anl(NRF24L01_2_RXDATA,NRF24L01_2_RXDATA[3]+5);
        NRF_SSI_CNT++;
        Connect_flag = 1;
    }

    if(Nrf_Erro>=500)
    {
        Nrf_Erro = 1;
        Connect_flag = 0;
    }

    return Connect_flag;
}
