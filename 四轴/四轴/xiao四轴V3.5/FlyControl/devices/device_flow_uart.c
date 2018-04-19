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
#include "device_flow.h"
#include "device_Uart.h"

#include "Fc_data_Struct.h"
/*============================ MACROS ========================================*/

/*============================ TYPES =========================================*/

/*============================ GLOBAL VARIABLES ==============================*/
extern FlyControlDataStruct g_FlyControlDataStruct;
u8 Flow_SSI,Flow_SSI_CNT,Flow_Err;
/*============================ STATIC VARIABLES ==============================*/

/*============================ FUNCTION ======================================*/
/*----------------------------------------------------------
 + 实现功能：数据分析
 + 调用参数：接收到的单字节数据
----------------------------------------------------------*/
void Fc_Flow_Init(void)
{
    Fc_Usart2_Init(115200);

}
void Flow_Receive_Anl(u8 *data_buf,u8 num)
{
    u8 sum = 0;
    /* 计算校验累加和 */
    for(u8 i=0; i<(num-1); i++)
        sum += *(data_buf+i);
    /* 判断校验累加和 */
    if(!(sum==*(data_buf+num-1)))		return;
    /* 判断帧头 */
    if(!(*(data_buf)==0xAA && *(data_buf+1)==0XAA))		return;
    /* 判断功能字：主要命令集 */

    /* 回传校验累加和 */
    if(*(data_buf+2)==0Xf1)
    {
        g_FlyControlDataStruct.AppOptflowStruct.x_offest   = ( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        g_FlyControlDataStruct.AppOptflowStruct.y_offest   = ( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        g_FlyControlDataStruct.AppHightCtrl.UltHight       = ( (vs16)(*(data_buf+10)<<8)|*(data_buf+11));
        g_FlyControlDataStruct.AppOptflowStruct.NSS        = ( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        Flow_SSI_CNT++;
    }
}

/*----------------------------------------------------------
 + 实现功能：数据接收并保存
 + 调用参数：接收到的单字节数据
----------------------------------------------------------*/
void Flow_Data_Receive_Prepare(u8 data)
{
    /* 局部静态变量：接收缓存 */
    static u8 RxBuffer[50];
    /* 数据长度 *//* 数据数组下标 */
    static u8 _data_len = 0,_data_cnt = 0;
    /* 接收状态 */
    static u8 state = 0;

    /* 帧头1 */
    if(state==0&&data==0XAA)
    {
        state=1;
        RxBuffer[0]=data;
    }
    /* 帧头2 */
    else if(state==1&&data==0XAA)
    {
        state=2;
        RxBuffer[1]=data;
    }
    /* 功能字 */
    else if(state==2&&data<0XF2)
    {
        state=3;
        RxBuffer[2]=data;
    }
    /* 长度 */
    else if(state==3&&data<50)
    {
        state = 4;
        RxBuffer[3]=data;
        _data_len = data;
        _data_cnt = 0;
    }
    /* 接收数据租 */
    else if(state==4&&_data_len>0)
    {
        _data_len--;
        RxBuffer[4+_data_cnt++]=data;
        if(_data_len==0)
            state = 5;
    }
    /* 校验累加和 */
    else if(state==5)
    {
        state = 0;
        RxBuffer[4+_data_cnt]=data;
        Flow_Receive_Anl(RxBuffer,_data_cnt+5);
    }
    /* 若有错误重新等待接收帧头 */
    else
        state = 0;
}

