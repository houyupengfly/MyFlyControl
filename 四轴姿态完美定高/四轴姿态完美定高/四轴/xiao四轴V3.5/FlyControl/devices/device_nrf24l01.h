/*=============================================================================
** 作者：侯玉鹏
** 日期：2017/10/31
** 版本：V1.0
** 头文件
=============================================================================*/

#ifndef __NRF24L01_H__
#define __NRF24L01_H__
/*============================ INCLUDES ======================================*/
#include "basetype.h"
/*============================ MACROS ========================================*/
#define MODEL_RX				1			//普通接收
#define MODEL_TX				2			//普通发送
#define MODEL_RX2				3			//接收模式2,用于双向传输
#define MODEL_TX2				4			//发送模式2,用于双向传输

#define RX_PLOAD_WIDTH  255
#define TX_PLOAD_WIDTH  255
#define TX_ADR_WIDTH    5
#define RX_ADR_WIDTH    5

//***************************************NRF24L01寄存器指令*******************************************************
#define NRF_READ_REG    0x00  	// 读寄存器指令
#define NRF_WRITE_REG   0x20 	// 写寄存器指令
#define R_RX_PL_WID   	0x60
#define RD_RX_PLOAD     0x61  	// 读取接收数据指令
#define WR_TX_PLOAD     0xA0  	// 写待发数据指令
#define FLUSH_TX        0xE1 	// 冲洗发送 FIFO指令
#define FLUSH_RX        0xE2  	// 冲洗接收 FIFO指令
#define REUSE_TX_PL     0xE3  	// 定义重复装载数据指令
#define NOP             0xFF  	// 保留
//*************************************SPI(nRF24L01)寄存器地址****************************************************
#define CONFIG          0x00  // 配置收发状态，CRC校验模式以及收发状态响应方式
#define EN_AA           0x01  // 自动应答功能设置
#define EN_RXADDR       0x02  // 可用信道设置
#define SETUP_AW        0x03  // 收发地址宽度设置
#define SETUP_RETR      0x04  // 自动重发功能设置
#define RF_CH           0x05  // 工作频率设置
#define RF_SETUP        0x06  // 发射速率、功耗功能设置
#define NRFRegSTATUS    0x07  // 状态寄存器
#define OBSERVE_TX      0x08  // 发送监测功能
#define CD              0x09  // 地址检测           
#define RX_ADDR_P0      0x0A  // 频道0接收数据地址
#define RX_ADDR_P1      0x0B  // 频道1接收数据地址
#define RX_ADDR_P2      0x0C  // 频道2接收数据地址
#define RX_ADDR_P3      0x0D  // 频道3接收数据地址
#define RX_ADDR_P4      0x0E  // 频道4接收数据地址
#define RX_ADDR_P5      0x0F  // 频道5接收数据地址
#define TX_ADDR         0x10  // 发送地址寄存器
#define RX_PW_P0        0x11  // 接收频道0接收数据长度
#define RX_PW_P1        0x12  // 接收频道1接收数据长度
#define RX_PW_P2        0x13  // 接收频道2接收数据长度
#define RX_PW_P3        0x14  // 接收频道3接收数据长度
#define RX_PW_P4        0x15  // 接收频道4接收数据长度
#define RX_PW_P5        0x16  // 接收频道5接收数据长度
#define FIFO_STATUS     0x17  // FIFO栈入栈出状态寄存器设置
//**************************************************************************************
//*********************************************NRF24L01*************************************
#define RX_DR				6		//中断标志
#define TX_DS				5
#define MAX_RT			    4
/*============================ TYPES =========================================*/

/*============================ GLOBAL VARIABLES ==============================*/
/*============================ STATIC VARIABLES ==============================*/

/*============================ FUNCTION ======================================*/

extern void Fc_Nrf_Init(u8 model, u8 ch);
extern void Nrf_TxPacket(u8 * tx_buf, u8 len);
extern void Nrf_TxPacket_AP(u8 * tx_buf, u8 len);
extern u8 Nrf_Check(void);
extern u8 Nrf_Check_Event(void);
extern u8 NRF24L01_2_RXDATA[RX_PLOAD_WIDTH];
extern u8 NRF24L01_2_TXDATA[RX_PLOAD_WIDTH];
extern u8 NRF_SSI,NRF_SSI_CNT;
extern u8 Nrf_Connect(void);
extern u8 NRF24L01_2_RXDATA[RX_PLOAD_WIDTH];
extern u8 NRF24L01_2_TXDATA[RX_PLOAD_WIDTH];
extern u8 NRF_SSI,NRF_SSI_CNT;

#endif















