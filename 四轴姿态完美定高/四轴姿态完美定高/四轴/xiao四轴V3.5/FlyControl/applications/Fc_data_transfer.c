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
#include "stm32f10x.h"

#include "Fc_data_transfer.h"
#include "Fc_data_Struct.h"
#include "Fc_SeedEstimated.h"
#include "Fc_revice.h"
#include "Fc_parameter.h"
#include "filter.h"
#include "device_Uart.h"
#include "device_Nrf24l01.h"
/*============================ MACROS ========================================*/

/* 数据拆分宏定义，在发送大于8位的数据类型时，比如int16、int32等，需要把数据拆分成8位逐个发送 */
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp) + 0) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
/* 发送帧头 接收帧头 */
#define title1_send 0xAA
#define title2_send 0xAA
#define title1_received 0xAA
#define title2_received 0xAF
/*============================ TYPES =========================================*/

/*============================ GLOBAL VARIABLES ==============================*/
extern FlyControlDataStruct g_FlyControlDataStruct;
extern ParamCtl_Type g_AllParamStruct;
extern SINS NewSinsData;
extern u8 Bluet_SSI,Bluet_SSI_CNT;

extern float exp_height_speed;
extern u8 Bluet_SSI_CNT;
extern u8 NRF_SSI,NRF_SSI_CNT;//NRF信号强度;
extern u16 set_flag;
extern _fix_inte_filter_st wcz_spe_fus,wcz_hei_fus;
extern u16 save_pid_en;
extern float exp_height;
/*============================ STATIC VARIABLES ==============================*/
/* 等待发送数据的标志 */
u8 wait_for_translate;
/* 等待发送数据的标志 */
dt_flag_t f;
/* 发送数据缓存数组 */
u8 data_to_send[50];
/*============================ FUNCTION ======================================*/
/*----------------------------------------------------------
 + 实现功能：数传数据发送
 + 调用参数：要发送的数据组 数据长度
----------------------------------------------------------*/
void DT_Send_Data(u8 *dataToSend , u8 length)
{
    /* 串口1发送 要发送的数据组 数据长度 */
    if(wait_for_translate)
    {   switch(g_FlyControlDataStruct.AppConfigDataStruct.NS)
        {
        case 1://已连接遥控器
            Nrf_TxPacket_AP(dataToSend,length);
            break;

        case 3://已连接蓝牙模块
            Fc_Uart4_Put_Buf(dataToSend,length);
            break;

        default:
            break;
        }
    }
}

/*----------------------------------------------------------
 + 实现功能：校验累加和回传
 + 调用参数：字帧 校验累加和
----------------------------------------------------------*/
static void DT_Send_Check(u8 head, u8 check_sum)
{
    /* 数据内容 */
    data_to_send[0]=title1_send;
    data_to_send[1]=title2_send;
    data_to_send[2]=0xEF;
    data_to_send[3]=2;
    data_to_send[4]=head;
    data_to_send[5]=check_sum;
    /* 校验累加和计算 */
    u8 sum = 0;

    for(u8 i=0; i<6; i++)
        sum += data_to_send[i];

    data_to_send[6]=sum;
    /* 发送 要发送的数据组 数据长度 */
    DT_Send_Data(data_to_send, 7);
}

/*----------------------------------------------------------
 + 实现功能：数据分析
 + 调用参数：接收到的单字节数据
----------------------------------------------------------*/
void DT_Data_Receive_Anl(u8 *data_buf,u8 num)
{
    u8 sum = 0;

    /* 计算校验累加和 */
    for(u8 i=0; i<(num-1); i++)
        sum += *(data_buf+i);

    /* 判断校验累加和 */
    if(!(sum==*(data_buf+num-1)))		return;

    /* 判断帧头 */
    if(!(*(data_buf)==title1_received && *(data_buf+1)==title2_received))		return;

    /* 判断功能字：主要命令集 */
    if(*(data_buf+2)==0X01)
    {
        /* 加速度计校准 */
        if(*(data_buf+4)==0X01|| *(data_buf+4)==0X02)
        {
            if(g_FlyControlDataStruct.AppConfigDataStruct.AirStatus==0)
            {
                g_FlyControlDataStruct.AppConfigDataStruct.calibratingG = 1;
                g_FlyControlDataStruct.AppConfigDataStruct.calibratingA = 1;
                g_FlyControlDataStruct.AppHightCtrl.baroStart=1;
            }
        }
        /* 磁力计校准 */
        else if(*(data_buf+4)==0X04)
            g_FlyControlDataStruct.AppConfigDataStruct.calibratingM = 1;

//        /* 飞控锁定 */
//        else if(*(data_buf+4)==0XA0)
//            unlocked_to_fly=0;
//        /* 飞控解锁 */
//        else if(*(data_buf+4)==0XA1)
//            unlocked_to_fly=1;
//        /* PWM正常输出 */
//        else if(*(data_buf+4)==0XD0)
//            PWM_Mode=0;
//        /* PWM全小输出 */
//        else if(*(data_buf+4)==0XD1)
//            PWM_Mode=1;
//        /* PWM全大输出 */
//        else if(*(data_buf+4)==0XD2)
//            PWM_Mode=2;
    }

    /* 判断功能字：次要命令集 */
    if(*(data_buf+2)==0X02)
    {
        /* 读取全部10组PID参数 */
        if(*(data_buf+4)==0X01)
        {
            f.send_pid1 = 1;
            f.send_pid2 = 1;
            f.send_pid3 = 1;
            f.send_pid4 = 1;
        }

//        /* 恢复默认PID参数 */
        if(*(data_buf+4)==0XA1)
        {
            Fc_Para_Init();
        }

        else if(*(data_buf+4)==0XA0)		//读取版本信息
        {
            f.send_version = 1;
        }
    }

    /* 判断功能字 接收数据 */
    if(*(data_buf+2)==0X03)
    {
        g_FlyControlDataStruct.AppConfigDataStruct.NS =1;//遥控数据来源与遥控器
        /* 数据赋值 */
        g_FlyControlDataStruct.AppRcDataStruct.THR= ((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
        g_FlyControlDataStruct.AppRcDataStruct.Yaw= ((vs16)(*(data_buf+6)<<8)|*(data_buf+7)) ;
        g_FlyControlDataStruct.AppRcDataStruct.Roll= ((vs16)(*(data_buf+8)<<8)|*(data_buf+9)) ;
        g_FlyControlDataStruct.AppRcDataStruct.Pitch= ((vs16)(*(data_buf+10)<<8)|*(data_buf+11)) ;
        g_FlyControlDataStruct.AppRcDataStruct.AUX1= (vs16)(*(data_buf+12)<<8)|*(data_buf+13) ;
        g_FlyControlDataStruct.AppRcDataStruct.AUX2 = (vs16)(*(data_buf+14)<<8)|*(data_buf+15) ;
        g_FlyControlDataStruct.AppRcDataStruct.AUX3 = (vs16)(*(data_buf+16)<<8)|*(data_buf+17) ;
        g_FlyControlDataStruct.AppRcDataStruct.AUX4= (vs16)(*(data_buf+18)<<8)|*(data_buf+19) ;
        Fc_key_function(g_FlyControlDataStruct.AppRcDataStruct.AUX4);
    }

    /* PID数组：1,2,3 */
    if(*(data_buf+2)==0X10)
    {
        /* 数据赋值 */
        g_AllParamStruct.Roll_In_Loop.Kp  = 0.001*((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
        g_AllParamStruct.Roll_In_Loop.Ki = 0.001*((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
        g_AllParamStruct.Roll_In_Loop.Kd  = 0.001*((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
        g_AllParamStruct.Pitch_In_Loop.Kp = 0.001*((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
        g_AllParamStruct.Pitch_In_Loop.Ki = 0.001*((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
        g_AllParamStruct.Pitch_In_Loop.Kd = 0.001*((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
        g_AllParamStruct.Yaw_In_Loop.Kp   = 0.001*((vs16)(*(data_buf+16)<<8)|*(data_buf+17));
        g_AllParamStruct.Yaw_In_Loop.Ki   = 0.001*((vs16)(*(data_buf+18)<<8)|*(data_buf+19));
        g_AllParamStruct.Yaw_In_Loop.Kd	 = 0.001*((vs16)(*(data_buf+20)<<8)|*(data_buf+21));
        DT_Send_Check(*(data_buf+2),sum);
        save_pid_en = 1;
        /* 写入并保存数据 */
    }

    /* PID数组：4,5,6 */
    if(*(data_buf+2)==0X11)
    {
        /* 数据赋值 */
        g_AllParamStruct.Roll_Out_Loop.Kp  = 0.001*((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
        g_AllParamStruct.Roll_Out_Loop.Ki = 0.001*((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
        g_AllParamStruct.Roll_Out_Loop.Kd  = 0.001*((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
        g_AllParamStruct.Pitch_Out_Loop.Kp = 0.001*((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
        g_AllParamStruct.Pitch_Out_Loop.Ki = 0.001*((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
        g_AllParamStruct.Pitch_Out_Loop.Kd = 0.001*((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
        g_AllParamStruct.Yaw_Out_Loop.Kp   = 0.001*((vs16)(*(data_buf+16)<<8)|*(data_buf+17));
        g_AllParamStruct.Yaw_Out_Loop.Ki   = 0.001*((vs16)(*(data_buf+18)<<8)|*(data_buf+19));
        g_AllParamStruct.Yaw_Out_Loop.Kd	 = 0.001*((vs16)(*(data_buf+20)<<8)|*(data_buf+21));
        DT_Send_Check(*(data_buf+2),sum);
        save_pid_en = 1;
        /* 写入并保存数据 */
    }

//    /* PID数组：7,8,9 */
    if(*(data_buf+2)==0X12)
    {
        /* 数据赋值 */
        g_AllParamStruct.Alti_In_Loop.Kp     = 0.001*((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
        g_AllParamStruct.Alti_In_Loop.Ki     = 0.001*((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
        g_AllParamStruct.Alti_In_Loop.Kd     = 0.001*((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
        g_AllParamStruct.Alti_Out_Loop.Kp     = 0.001*((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
        g_AllParamStruct.Alti_Out_Loop.Ki     = 0.001*((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
        g_AllParamStruct.Alti_Out_Loop.Kd     = 0.001*((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
//        g_FlyControlDataStruct.AppPostionCtrl.SpeedCtrlX.p 	= 0.001*((vs16)(*(data_buf+16)<<8)|*(data_buf+17));
//        g_FlyControlDataStruct.AppPostionCtrl.SpeedCtrlX.i	= 0.001*((vs16)(*(data_buf+18)<<8)|*(data_buf+19));
//        g_FlyControlDataStruct.AppPostionCtrl.SpeedCtrlX.d 	= 0.001*((vs16)(*(data_buf+20)<<8)|*(data_buf+21));
        DT_Send_Check(*(data_buf+2),sum);
        /* 立即重重当前接收到数据 */

        /* 写入并保存数据 */
        save_pid_en = 1;
    }

    /* PID数组：10 */
    if(*(data_buf+2)==0X13)
    {
        /* 数据赋值 */
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.p    = 0.001*((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.i    = 0.001*((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.d    = 0.001*((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
////        g_FlyControlDataStruct.AppPostionCtrl.SpeedCtrlY.p      = 0.001*((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
////        g_FlyControlDataStruct.AppPostionCtrl.SpeedCtrlY.i      = 0.001*((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
////        g_FlyControlDataStruct.AppPostionCtrl.SpeedCtrlY.d      = 0.001*((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlY.p 	= 0.001*((vs16)(*(data_buf+16)<<8)|*(data_buf+17));
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlY.i	= 0.001*((vs16)(*(data_buf+18)<<8)|*(data_buf+19));
//        g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlY.d 	= 0.001*((vs16)(*(data_buf+20)<<8)|*(data_buf+21));
        DT_Send_Check(*(data_buf+2),sum);
        save_pid_en = 1;
    }

    /* 回传校验累加和 */
    if(*(data_buf+2)==0X14)
    {
        DT_Send_Check(*(data_buf+2),sum);
    }

    /* 回传校验累加和 */
    if(*(data_buf+2)==0X15)
    {
        DT_Send_Check(*(data_buf+2),sum);
    }

    if(*(data_buf+2)==0xF1) //飞控设置
    {
        set_flag = ((vs16)(*(data_buf+4)<<8)|*(data_buf+5));

        if((set_flag&BIT0))	g_FlyControlDataStruct.AppConfigDataStruct.FlyMode=1;
        else g_FlyControlDataStruct.AppConfigDataStruct.FlyMode=0;

        if(set_flag&BIT1)	g_FlyControlDataStruct.AppConfigDataStruct.Throw =1;
        else g_FlyControlDataStruct.AppConfigDataStruct.Throw =0;

        if(set_flag&BIT2)	g_FlyControlDataStruct.AppConfigDataStruct.No_Head_Mode=1;
        else g_FlyControlDataStruct.AppConfigDataStruct.No_Head_Mode=0;

        if(set_flag&BIT5 )//&& Flow_Err)==0 )
        {
            g_FlyControlDataStruct.AppConfigDataStruct.Fixed_Point_Mode=1;
        }
        else
        {
            g_FlyControlDataStruct.AppConfigDataStruct.Fixed_Point_Mode=0;
        }
    }
}

/*----------------------------------------------------------
 + 实现功能：数据接收并保存
 + 调用参数：接收到的单字节数据
----------------------------------------------------------*/
void DT_Data_Receive_Prepare(u8 data)
{
    /* 局部静态变量：接收缓存 */
    static u8 RxBuffer[50];
    /* 数据长度 *//* 数据数组下标 */
    static u8 _data_len = 0,_data_cnt = 0;
    /* 接收状态 */
    static u8 state = 0;

    /* 帧头1 */
    if(state==0&&data==title1_received)
    {
        state=1;
        RxBuffer[0]=data;
    }
    /* 帧头2 */
    else if(state==1&&data==title2_received)
    {
        state=2;
        RxBuffer[1]=data;
    }
    /* 功能字 */
    else if(state==2&&data<0XF1)
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
        DT_Data_Receive_Anl(RxBuffer,_data_cnt+5);
    }
    /* 若有错误重新等待接收帧头 */
    else
        state = 0;
}

/*----------------------------------------------------------
 + 实现功能：发送速度信息
 + 调用参数：向北速度 向西速度 向上速度
 + 数量级：这里函数调用发送多少上位机就显示多少，保留2位小数
----------------------------------------------------------*/
void DT_Send_Speed(float x_s,float y_s,float z_s)
{
    u8 _cnt=0;
    vs16 _temp;
    data_to_send[_cnt++]=title1_send;
    data_to_send[_cnt++]=title2_send;
    data_to_send[_cnt++]=0x0B;
    data_to_send[_cnt++]=0;
    _temp = (int)(x_s*100.0f);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int)(y_s*100.0f);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int)(z_s*100.0f);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    data_to_send[3] = _cnt-4;
    u8 sum = 0;

    for(u8 i=0; i<_cnt; i++)
        sum += data_to_send[i];

    data_to_send[_cnt++]=sum;
    DT_Send_Data(data_to_send, _cnt);
}

/*----------------------------------------------------------
 + 实现功能：发送位置信息
 + 调用参数：可见卫星数、经度坐标、纬度坐标、悬停飞行模式 其中坐标信息 放大倍率10E5
----------------------------------------------------------*/
void DT_Send_Location(u8 state,u8 sat_num,s32 lon,s32 lat,s16 mode)
{
    u8 _cnt=0;
    vs16 _temp;
    vs32 _temp2;
    data_to_send[_cnt++]=title1_send;
    data_to_send[_cnt++]=title2_send;
    data_to_send[_cnt++]=0x04;
    data_to_send[_cnt++]=0;
    data_to_send[_cnt++]=state;//定位状态
    data_to_send[_cnt++]=sat_num;//可见卫星数
    _temp2 = lon;//经度
    data_to_send[_cnt++]=BYTE3(_temp2);
    data_to_send[_cnt++]=BYTE2(_temp2);
    data_to_send[_cnt++]=BYTE1(_temp2);
    data_to_send[_cnt++]=BYTE0(_temp2);
    _temp2 = lat;//纬度
    data_to_send[_cnt++]=BYTE3(_temp2);
    data_to_send[_cnt++]=BYTE2(_temp2);
    data_to_send[_cnt++]=BYTE1(_temp2);
    data_to_send[_cnt++]=BYTE0(_temp2);
    _temp = mode;//悬停飞行模式
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    data_to_send[3] = _cnt-4;
    u8 sum = 0;

    for(u8 i=0; i<_cnt; i++)
        sum += data_to_send[i];

    data_to_send[_cnt++]=sum;
    DT_Send_Data(data_to_send, _cnt);
}

/*----------------------------------------------------------
 + 实现功能：发送姿态信息
 + 调用参数：横滚、俯仰、航向、气压高度 厘米、控制高度模式、解锁状态
----------------------------------------------------------*/
void DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
{
    u8 _cnt=0;
    vs16 _temp;
    vs32 _temp2 = alt;
    data_to_send[_cnt++]=title1_send;
    data_to_send[_cnt++]=title2_send;
    data_to_send[_cnt++]=0x01;
    data_to_send[_cnt++]=0;
    _temp = (int)(angle_rol*100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int)(angle_pit*100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int)(angle_yaw*100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    data_to_send[_cnt++]=BYTE3(_temp2);
    data_to_send[_cnt++]=BYTE2(_temp2);
    data_to_send[_cnt++]=BYTE1(_temp2);
    data_to_send[_cnt++]=BYTE0(_temp2);
    data_to_send[_cnt++] = fly_model;
    data_to_send[_cnt++] = armed;
    data_to_send[3] = _cnt-4;
    u8 sum = 0;

    for(u8 i=0; i<_cnt; i++)
        sum += data_to_send[i];

    data_to_send[_cnt++]=sum;
    DT_Send_Data(data_to_send, _cnt);
}

/*----------------------------------------------------------
 + 实现功能：发送姿态传感器信息
 + 调用参数：发送加速度计 陀螺仪 磁力计 的原始数据
----------------------------------------------------------*/
void DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
    u8 _cnt=0;
    vs16 _temp;
    data_to_send[_cnt++]=title1_send;
    data_to_send[_cnt++]=title2_send;
    data_to_send[_cnt++]=0x02;
    data_to_send[_cnt++]=0;
    _temp = a_x;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = a_y;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = a_z;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = g_x;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = g_y;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = g_z;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = m_x;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = m_y;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = m_z;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = 0;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    data_to_send[3] = _cnt-4;
    u8 sum = 0;

    for(u8 i=0; i<_cnt; i++)
        sum += data_to_send[i];

    data_to_send[_cnt++] = sum;
    DT_Send_Data(data_to_send, _cnt);
}

/*----------------------------------------------------------
 + 实现功能：发送高度传感器信息
 + 调用参数：发送气压计高度 超声波高度 发送单位厘米
----------------------------------------------------------*/
void DT_Send_Senser2(s32 bar_alt,u16 csb_alt)
{
    u8 _cnt=0;
    data_to_send[_cnt++]=title1_send;
    data_to_send[_cnt++]=title2_send;
    data_to_send[_cnt++]=0x07;
    data_to_send[_cnt++]=0;
    data_to_send[_cnt++]=BYTE3(bar_alt);
    data_to_send[_cnt++]=BYTE2(bar_alt);
    data_to_send[_cnt++]=BYTE1(bar_alt);
    data_to_send[_cnt++]=BYTE0(bar_alt);
    data_to_send[_cnt++]=BYTE1(csb_alt);
    data_to_send[_cnt++]=BYTE0(csb_alt);
    data_to_send[3] = _cnt-4;
    u8 sum = 0;

    for(u8 i=0; i<_cnt; i++)
        sum += data_to_send[i];

    data_to_send[_cnt++] = sum;
    DT_Send_Data(data_to_send, _cnt);
}

/*----------------------------------------------------------
 + 实现功能：发送飞控收到遥控量信息
 + 调用参数：发送飞控收到遥控量数据
----------------------------------------------------------*/
void DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
    u8 _cnt=0;
    data_to_send[_cnt++]=title1_send;
    data_to_send[_cnt++]=title2_send;
    data_to_send[_cnt++]=0x03;
    data_to_send[_cnt++]=0;
    data_to_send[_cnt++]=BYTE1(thr);
    data_to_send[_cnt++]=BYTE0(thr);
    data_to_send[_cnt++]=BYTE1(yaw);
    data_to_send[_cnt++]=BYTE0(yaw);
    data_to_send[_cnt++]=BYTE1(rol);
    data_to_send[_cnt++]=BYTE0(rol);
    data_to_send[_cnt++]=BYTE1(pit);
    data_to_send[_cnt++]=BYTE0(pit);
    data_to_send[_cnt++]=BYTE1(aux1);
    data_to_send[_cnt++]=BYTE0(aux1);
    data_to_send[_cnt++]=BYTE1(aux2);
    data_to_send[_cnt++]=BYTE0(aux2);
    data_to_send[_cnt++]=BYTE1(aux3);
    data_to_send[_cnt++]=BYTE0(aux3);
    data_to_send[_cnt++]=BYTE1(aux4);
    data_to_send[_cnt++]=BYTE0(aux4);
    data_to_send[_cnt++]=BYTE1(aux5);
    data_to_send[_cnt++]=BYTE0(aux5);
    data_to_send[_cnt++]=BYTE1(aux6);
    data_to_send[_cnt++]=BYTE0(aux6);
    
    data_to_send[3] = _cnt-4;
    u8 sum = 0;

    for(u8 i=0; i<_cnt; i++)
        sum += data_to_send[i];

    data_to_send[_cnt++]=sum;
    DT_Send_Data(data_to_send, _cnt);
}

/*----------------------------------------------------------
 + 实现功能：发送电压数据,电流数据信息
 + 调用参数：电压数据,电流数据
----------------------------------------------------------*/
void DT_Send_Power(u16 votage, u16 current, u8 flag0, u8 flag1, u16 flag2, u16 flag3)
{
    u8 _cnt=0;
    u16 temp;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x05;
    data_to_send[_cnt++]=0;
    temp = votage;
    data_to_send[_cnt++]=BYTE1(temp);
    data_to_send[_cnt++]=BYTE0(temp);
    temp = current;
    data_to_send[_cnt++]=BYTE1(temp);
    data_to_send[_cnt++]=BYTE0(temp);
    data_to_send[_cnt++]=flag0;
    data_to_send[_cnt++]=flag1;
    temp = flag2;
    data_to_send[_cnt++]=BYTE1(temp);
    data_to_send[_cnt++]=BYTE0(temp);
    temp = flag3;
    data_to_send[_cnt++]=BYTE1(temp);
    data_to_send[_cnt++]=BYTE0(temp);
    data_to_send[3] = _cnt-4;
    u8 sum = 0;

    for(u8 i=0; i<_cnt; i++)
        sum += data_to_send[i];

    data_to_send[_cnt++]=sum;
    DT_Send_Data(data_to_send, _cnt);
}

/*----------------------------------------------------------
 + 实现功能：发送飞控对电机控制量信息
 + 调用参数：发送飞控对电机控制量数据
----------------------------------------------------------*/
void DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
{
    u8 _cnt=0;
    data_to_send[_cnt++]=title1_send;
    data_to_send[_cnt++]=title2_send;
    data_to_send[_cnt++]=0x06;
    data_to_send[_cnt++]=0;
    data_to_send[_cnt++]=BYTE1(m_1);
    data_to_send[_cnt++]=BYTE0(m_1);
    data_to_send[_cnt++]=BYTE1(m_2);
    data_to_send[_cnt++]=BYTE0(m_2);
    data_to_send[_cnt++]=BYTE1(m_3);
    data_to_send[_cnt++]=BYTE0(m_3);
    data_to_send[_cnt++]=BYTE1(m_4);
    data_to_send[_cnt++]=BYTE0(m_4);
    data_to_send[_cnt++]=BYTE1(m_5);
    data_to_send[_cnt++]=BYTE0(m_5);
    data_to_send[_cnt++]=BYTE1(m_6);
    data_to_send[_cnt++]=BYTE0(m_6);
    data_to_send[_cnt++]=BYTE1(m_7);
    data_to_send[_cnt++]=BYTE0(m_7);
    data_to_send[_cnt++]=BYTE1(m_8);
    data_to_send[_cnt++]=BYTE0(m_8);
    data_to_send[3] = _cnt-4;
    u8 sum = 0;

    for(u8 i=0; i<_cnt; i++)
        sum += data_to_send[i];

    data_to_send[_cnt++]=sum;
    DT_Send_Data(data_to_send, _cnt);
}

/*----------------------------------------------------------
 + 实现功能：发送PID信息
 + 调用参数：发送PID数据 放大倍率10E3
----------------------------------------------------------*/
void DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
    u8 _cnt=0;
    vs16 _temp;
    data_to_send[_cnt++]=title1_send;
    data_to_send[_cnt++]=title2_send;
    data_to_send[_cnt++]=0x10+group-1;
    data_to_send[_cnt++]=0;
    _temp = p1_p  * 1000;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = p1_i  * 1000;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = p1_d  * 1000;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = p2_p  * 1000;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = p2_i  * 1000;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = p2_d  * 1000;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = p3_p  * 1000;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = p3_i  * 1000;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = p3_d  * 1000;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    data_to_send[3] = _cnt-4;
    u8 sum = 0;

    for(u8 i=0; i<_cnt; i++)
        sum += data_to_send[i];

    data_to_send[_cnt++]=sum;
    DT_Send_Data(data_to_send, _cnt);
}

/*----------------------------------------------------------
 + 实现功能：用户自定义发送
----------------------------------------------------------*/
void DT_Send_User()
{
    u8 _cnt=0;
    vs16 _temp;
    data_to_send[_cnt++]=title1_send;
    data_to_send[_cnt++]=title2_send;
    data_to_send[_cnt++]=0xf1; //用户定义功能字
    data_to_send[_cnt++]=0;
    _temp = 0;           //1
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = 0;           //1
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = 0;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = 0;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    data_to_send[3] = _cnt-4;
    u8 sum = 0;

    for(u8 i=0; i<_cnt; i++)
        sum += data_to_send[i];

    data_to_send[_cnt++]=sum;
    DT_Send_Data(data_to_send, _cnt);
}

void DT_Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver)
{
    u8 _cnt=0;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x00;
    data_to_send[_cnt++]=0;

    data_to_send[_cnt++]=hardware_type;
    data_to_send[_cnt++]=BYTE1(hardware_ver);
    data_to_send[_cnt++]=BYTE0(hardware_ver);
    data_to_send[_cnt++]=BYTE1(software_ver);
    data_to_send[_cnt++]=BYTE0(software_ver);
    data_to_send[_cnt++]=BYTE1(protocol_ver);
    data_to_send[_cnt++]=BYTE0(protocol_ver);
    data_to_send[_cnt++]=BYTE1(bootloader_ver);
    data_to_send[_cnt++]=BYTE0(bootloader_ver);

    data_to_send[3] = _cnt-4;

    u8 sum = 0;
    for(u8 i=0; i<_cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++]=sum;

    DT_Send_Data(data_to_send, _cnt);
}

/*----------------------------------------------------------
 + 实现功能：由任务调度调用周期10ms
----------------------------------------------------------*/
void Fc_Data_transfer(void)
{
    /* 定义巨部静态变量控制发送周期 */
    static int cnt = 0;

    ++cnt;

    if(cnt==1)
        f.send_status = 1;

//    if(cnt==2)
//        f.send_speed = 2;

    if(cnt==3)
        f.send_rcdata = 1;

//    if(cnt ==4)
//        f.send_motopwm = 4;

    if(cnt==2)
        f.send_senser = 1;

//    if(cnt==6)
//        f.send_senser2 = 6;

//    if(cnt==7)
//        f.send_location = 7;

    if(cnt==4)
        f.send_power = 1;

    if(cnt==5)
    {
        f.send_user = 1;
        cnt=0;
    }
    if(f.send_status)
    {
        f.send_status = 0;
        /* 横滚、俯仰、航向、气压cm高度、控制高度模式、解锁状态 */
        DT_Send_Status(g_FlyControlDataStruct.App_AngleStruct.Roll,g_FlyControlDataStruct.App_AngleStruct.Pitch,g_FlyControlDataStruct.App_AngleStruct.Yaw,g_FlyControlDataStruct.AppHightCtrl.BaroHight*100,0,g_FlyControlDataStruct.AppConfigDataStruct.AirStatus);
    }
//    /* 2发送速度数据，周期199ms */
//    else if(f.send_speed)
//    {
//        f.send_speed = 0;
//        #if 0
//        /* 调用参数：向北速度 向西速度 向上速度 单位毫米每秒 -> 上位机显示厘米每秒 */
//        DT_Send_Speed(0.1f*north_speed,0.1f*west_speed,0.1f*wz_speed);
//        #else
//        /* X像素速度 Y像素速度 光流像素要结合高度才能算出速度 百分比画面不变量 */
//        DT_Send_Speed(Optical_flow_x, Optical_flow_y,0.1f*wz_speed);
//        #endif
//    }
//    /* 3发送遥控量数据 */
    else if(f.send_rcdata)
    {
        f.send_rcdata = 0;
        /* 发送飞控收到遥控量数据 */
        DT_Send_RCData(g_FlyControlDataStruct.AppRcDataStruct.THR,g_FlyControlDataStruct.AppRcDataStruct.Yaw,g_FlyControlDataStruct.AppRcDataStruct.Roll,g_FlyControlDataStruct.AppRcDataStruct.Pitch, \
                       g_FlyControlDataStruct.AppRcDataStruct.mode,g_FlyControlDataStruct.AppRcDataStruct.AUX1,g_FlyControlDataStruct.AppRcDataStruct.AUX2,0,0,0);
    }
    /* 4发送遥控量数据 */
//    else if(f.send_motopwm)
//    {
//        f.send_motopwm = 0;
//        /* 发送飞控对电机控制量数据 */
//        DT_Send_MotoPWM(g_FlyControlDataStruct.AppCtrlStruct.Moto[0],g_FlyControlDataStruct.AppCtrlStruct.Moto[1],g_FlyControlDataStruct.AppCtrlStruct.Moto[2],g_FlyControlDataStruct.AppCtrlStruct.Moto[3],g_FlyControlDataStruct.AppCtrlStruct.Moto[4],g_FlyControlDataStruct.AppCtrlStruct.Moto[5],g_FlyControlDataStruct.AppCtrlStruct.Moto[6],g_FlyControlDataStruct.AppCtrlStruct.Moto[7]);
//    }
//    /* 5发送传感器数据 */
    else if(f.send_senser)
    {
        f.send_senser = 0;
        /* 发送加速度计 陀螺仪 磁力计 的原始数据 */
        DT_Send_Senser(g_FlyControlDataStruct.AppImuDataStruct.AccXFilterout,g_FlyControlDataStruct.AppImuDataStruct.AccYFilterout,g_FlyControlDataStruct.AppImuDataStruct.AccZFilterout, \
                       g_FlyControlDataStruct.AppImuDataStruct.GyroXFilterout,g_FlyControlDataStruct.AppImuDataStruct.GyroYFilterout,g_FlyControlDataStruct.AppImuDataStruct.GyroZFilterout, \
                       NewSinsData.Speed[0],NewSinsData.Position[0],exp_height);
    }
//    /* 6发送高度数据 */
//    else if(f.send_senser2)
//    {
//        f.send_senser2 = 0;
//        /* 发送气压计高度 超声波高度 发送单位厘米 */
//        DT_Send_Senser2(g_FlyControlDataStruct.AppHightCtrl.BaroHight*100,0);
//    }
//    /* 7发送位置数据，周期499ms */
//    else if(f.send_location)
//    {
//        f.send_location = 0;
//        /* 可见卫星数、经度坐标、纬度坐标、悬停飞行模式 其中坐标信息 放大倍率10E5  */
//        DT_Send_Location(gpsx.fixmode,gpsx.svnum,t_longitude,t_latitude,position_ctrl_mode);
//    }
//    /* 8发送电压数据 */
    else if(f.send_power)
    {
        f.send_power = 0;
        /* 电压数据 电流数据 并没有被使用 */
        DT_Send_Power(g_FlyControlDataStruct.AppVoltage.V/10,1,1,NRF_SSI,0,set_flag);
    }

    else if(f.send_pid1)
    {
        f.send_pid1 = 0;
        /* 3组PID数据 1、2、3 */
        DT_Send_PID(1,g_AllParamStruct.Roll_In_Loop.Kp,g_AllParamStruct.Roll_In_Loop.Ki,g_AllParamStruct.Roll_In_Loop.Kd, \
                    g_AllParamStruct.Pitch_In_Loop.Kp,g_AllParamStruct.Pitch_In_Loop.Ki,g_AllParamStruct.Pitch_In_Loop.Kd, \
                    g_AllParamStruct.Yaw_In_Loop.Kp,g_AllParamStruct.Yaw_In_Loop.Ki,g_AllParamStruct.Yaw_In_Loop.Kd);
    }
    else if(f.send_pid2)
    {
        f.send_pid2 = 0;
        /* 3组PID数据 4、5、6 */
        DT_Send_PID(2,g_AllParamStruct.Roll_Out_Loop.Kp,g_AllParamStruct.Roll_Out_Loop.Ki,g_AllParamStruct.Roll_Out_Loop.Kd, \
                    g_AllParamStruct.Pitch_Out_Loop.Kp,g_AllParamStruct.Pitch_Out_Loop.Ki,g_AllParamStruct.Pitch_Out_Loop.Kd, \
                    g_AllParamStruct.Yaw_Out_Loop.Kp,g_AllParamStruct.Yaw_Out_Loop.Ki,g_AllParamStruct.Yaw_Out_Loop.Kd);
    }
    else if(f.send_pid3)
    {
        f.send_pid3 = 0;
        /* 3组PID数据 7、8、9 */
        DT_Send_PID(3,g_AllParamStruct.Alti_In_Loop.Kp,g_AllParamStruct.Alti_In_Loop.Ki,g_AllParamStruct.Alti_In_Loop.Kd, \
                    g_AllParamStruct.Alti_Out_Loop.Kp, g_AllParamStruct.Alti_Out_Loop.Ki, g_AllParamStruct.Alti_Out_Loop.Kd,
                    //g_FlyControlDataStruct.AppPostionCtrl.SpeedCtrlX.p,g_FlyControlDataStruct.AppPostionCtrl.SpeedCtrlX.i,g_FlyControlDataStruct.AppPostionCtrl.SpeedCtrlX.d
                    0,0,0);
    }
    else if(f.send_pid4)
    {
        f.send_pid4 = 0;
        /* 3组PID数据 10 */
//        DT_Send_PID(4,g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.p,g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.i,g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlX.d,
////                    g_FlyControlDataStruct.AppPostionCtrl.SpeedCtrlY.p,g_FlyControlDataStruct.AppPostionCtrl.SpeedCtrlY.i,g_FlyControlDataStruct.AppPostionCtrl.SpeedCtrlY.d,
//                    0,0,0,
//                    g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlY.p,g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlY.i,g_FlyControlDataStruct.AppPostionCtrl.PostionCtrlY.d);
    }

//    else if(f.send_version==1)
//    {
//        f.send_version = 0;
//        DT_Send_Version(1,510,510,510,0);
//    }
}

/*----------------------------------------------------------
 + 实现功能：数传初始化
----------------------------------------------------------*/
void Fc_Data_transfer_init()
{
//    Fc_Usart5_Init(115200);
}

