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
#include "driver_mpu6050.h"
#include "device_mpu6050.h"
#include "device_iic.h"
#include "Fc_parameter.h"
#include "Fc_systream_time.h"
#include "Fc_data_Struct.h"
#include "filter.h"
/*============================ MACROS ========================================*/
/* 校准时的计算次数 */
#define  OFFSET_AV_NUM 	        50
/* 滑动窗口滤波数值个数 */
#define  FILTER_NUM 			10
/* Gyro角速度的单位转换 */
#define  TO_ANGLE 			    0.06103f
#define  IIR_ORDER              4      //使用IIR滤波器的阶数
#define MPU_WINDOW_NUM 5
#define MPU_STEEPEST_NUM 5

#define MPU_WINDOW_NUM_ACC 20
#define MPU_STEEPEST_NUM_ACC 20
/*============================ TYPES =========================================*/

/*============================ GLOBAL VARIABLES ==============================*/
extern FlyControlDataStruct g_FlyControlDataStruct;
extern ParamCtl_Type g_AllParamStruct;
/*============================ STATIC VARIABLES ==============================*/
/* MPU6050结构体 */
/* 传感器校准数据累加和 */
s32 sum_temp[7]= {0,0,0,0,0,0,0};
/* 传感器校准临时计数 */
u16 acc_sum_cnt = 0,gyro_sum_cnt = 0;
double b_IIR[IIR_ORDER+1] ={0.0008f, 0.0032f, 0.0048f, 0.0032f, 0.0008f};  //系数b
double a_IIR[IIR_ORDER+1] ={1.0000f, -3.0176f, 3.5072f, -1.8476f, 0.3708f};//系数a
double InPut_IIR[3][IIR_ORDER+1] = {0};
double OutPut_IIR[3][IIR_ORDER+1] = {0};
u8 flag_6050;


_steepest_st steepest_ax;
_steepest_st steepest_ay;
_steepest_st steepest_az;
_steepest_st steepest_gx;
_steepest_st steepest_gy;
_steepest_st steepest_gz;

s32 steepest_ax_arr[MPU_WINDOW_NUM_ACC ];
s32 steepest_ay_arr[MPU_WINDOW_NUM_ACC ];
s32 steepest_az_arr[MPU_WINDOW_NUM_ACC ];
s32 steepest_gx_arr[MPU_WINDOW_NUM ];
s32 steepest_gy_arr[MPU_WINDOW_NUM ];
s32 steepest_gz_arr[MPU_WINDOW_NUM ];
void mpu_type(float x,float y,float z)//6050批次问题检测
{
	float length;
	length = my_sqrt(my_pow(x)+my_pow(y)+my_pow(z));
	
	if(length > 1500 && length < 2600)
	{
		flag_6050=1;
	}
	else
	{
		flag_6050=0;
	}

}
/*============================ FUNCTION ======================================*/
/*----------------------------------------------------------
 + 实现功能：MPU6050加速度计、陀螺仪数据校准
----------------------------------------------------------*/
void MPU6050_Data_Offset()
{
     /* 加速度计校准 */ /* 陀螺仪校准 */
    if(g_FlyControlDataStruct.AppConfigDataStruct.calibratingA== 1 || g_FlyControlDataStruct.AppConfigDataStruct.calibratingG == 1)
    {
        /* 计数及累加 */
        acc_sum_cnt++;
        sum_temp[A_X] += g_FlyControlDataStruct.AppImuDataStruct.iAccX;
        sum_temp[A_Y] += g_FlyControlDataStruct.AppImuDataStruct.iAccY;
        sum_temp[A_Z] += g_FlyControlDataStruct.AppImuDataStruct.iAccZ;
        
        sum_temp[G_X] += g_FlyControlDataStruct.AppImuDataStruct.iGyroX;
        sum_temp[G_Y] += g_FlyControlDataStruct.AppImuDataStruct.iGyroY;
        sum_temp[G_Z] += g_FlyControlDataStruct.AppImuDataStruct.iGyroZ;


        /* 判断计数符合条件 */
        if( acc_sum_cnt >= OFFSET_AV_NUM )
        {
            /* 计算校验数据 */
            g_AllParamStruct.IMU1_Up_AccBias.x = sum_temp[A_X]/OFFSET_AV_NUM;
            g_AllParamStruct.IMU1_Up_AccBias.y = sum_temp[A_Y]/OFFSET_AV_NUM;
            g_AllParamStruct.IMU1_Up_AccBias.z = sum_temp[A_Z]/OFFSET_AV_NUM;
            
            g_AllParamStruct.IMU1_GyroBias.x  = (float)sum_temp[G_X]/OFFSET_AV_NUM;
            g_AllParamStruct.IMU1_GyroBias.y = (float)sum_temp[G_Y]/OFFSET_AV_NUM;
            g_AllParamStruct.IMU1_GyroBias.z = (float)sum_temp[G_Z]/OFFSET_AV_NUM;
            /* 清零过程变量 */
            acc_sum_cnt =0;
            sum_temp[A_X] = sum_temp[A_Y] = sum_temp[A_Z] = sum_temp[TEM] = 0;
            sum_temp[G_X] = sum_temp[G_Y] = sum_temp[G_Z] = sum_temp[TEM] = 0;
           
            /* 保存校验数据 */
            Param_Save();
            g_FlyControlDataStruct.AppConfigDataStruct.calibratingG = 0;
		    g_FlyControlDataStruct.AppConfigDataStruct.calibratingA = 0;
        }
    }

   
}

/*
传感器默认方向 西北天坐标系
     +x
     |
 +y--|--
     |

0:默认  
1：传感器顺时针90 度
2：传感器顺时针180度
3：传感器顺时针270度
*/

void sensor_dir(u8 dir,float itx,float ity,float itz,float *it_x,float *it_y,float *it_z)
{
	switch(dir)
	{
		case 1: //传感器顺时针90度
				*it_x = ity;
				*it_y = -itx;
				*it_z = itz;
		break;
		case 2://传感器顺时针180度
				*it_x = -itx;
				*it_y = -ity;
				*it_z = itz;
		break;
		case 3://传感器顺时针270度
				*it_x = -ity;
				*it_y = itx;
				*it_z = itz;
		break;
		default://传感器默认方向
			*it_x = itx;
			*it_y = ity;
			*it_z = itz;			
		break;
	}
	
}


/*----------------------------------------------------------
 + 实现功能：由任务调度调用周期2ms
----------------------------------------------------------*/
void Call_MPU6050_Data_Prepare(void)
{
    /* 原始数据读取临时数组 */
    u8 mpu6050_buffer[14];
    /* 陀螺仪原始数据临时变量 */
    float Gyro_tmp[3];
    float Acc_tmp[3];
	
    /*读取原始数据，用mpu6050_buffer[]解析*/
    Call_MPU6050(mpu6050_buffer);
    MPU6050_Data_Offset();
    g_FlyControlDataStruct.AppImuDataStruct.iAccX= ((((s16)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]);
    g_FlyControlDataStruct.AppImuDataStruct.iAccY= ((((s16)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]) ;
    g_FlyControlDataStruct.AppImuDataStruct.iAccZ= ((((s16)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]) ;
    g_FlyControlDataStruct.AppImuDataStruct.iGyroX= ((((s16)mpu6050_buffer[ 8]) << 8) | mpu6050_buffer[9]) ;
    g_FlyControlDataStruct.AppImuDataStruct.iGyroY= ((((s16)mpu6050_buffer[10]) << 8) | mpu6050_buffer[11]) ;
    g_FlyControlDataStruct.AppImuDataStruct.iGyroZ= ((((s16)mpu6050_buffer[12]) << 8) | mpu6050_buffer[13]) ;               

    /* 得出校准后的临时数据 */
    if(flag_6050)
    {
         Acc_tmp[0] =  2*(g_FlyControlDataStruct.AppImuDataStruct.iAccX - g_AllParamStruct.IMU1_Up_AccBias.x);
         Acc_tmp[1]=   2*(g_FlyControlDataStruct.AppImuDataStruct.iAccY - g_AllParamStruct.IMU1_Up_AccBias.y);
         Acc_tmp[2] =  2*(g_FlyControlDataStruct.AppImuDataStruct.iAccZ - g_AllParamStruct.IMU1_Up_AccBias.z + 2048);
    }
    else
    {
        
        Acc_tmp[0] =  (g_FlyControlDataStruct.AppImuDataStruct.iAccX - g_AllParamStruct.IMU1_Up_AccBias.x);
        Acc_tmp[1] =  (g_FlyControlDataStruct.AppImuDataStruct.iAccY - g_AllParamStruct.IMU1_Up_AccBias.y);
        Acc_tmp[2] =  (g_FlyControlDataStruct.AppImuDataStruct.iAccZ - g_AllParamStruct.IMU1_Up_AccBias.z) +4096;
    }
    
  
    
    Gyro_tmp[0] = g_FlyControlDataStruct.AppImuDataStruct.iGyroX - g_AllParamStruct.IMU1_GyroBias.x;
    Gyro_tmp[1] = g_FlyControlDataStruct.AppImuDataStruct.iGyroY - g_AllParamStruct.IMU1_GyroBias.y ;
    Gyro_tmp[2] = g_FlyControlDataStruct.AppImuDataStruct.iGyroZ - g_AllParamStruct.IMU1_GyroBias.z;
//	
//	// 加速度计IIR滤波 IIR效果很好 有效滤波了跳动的原始值 
//	g_FlyControlDataStruct.AppImuDataStruct.AccXFilterout = IIR_I_Filter(Acc_tmp[0], InPut_IIR[0], OutPut_IIR[0], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
//	g_FlyControlDataStruct.AppImuDataStruct.AccYFilterout = IIR_I_Filter(Acc_tmp[1], InPut_IIR[1], OutPut_IIR[1], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
//	g_FlyControlDataStruct.AppImuDataStruct.AccZFilterout = IIR_I_Filter(Acc_tmp[2] , InPut_IIR[2], OutPut_IIR[2], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
//	

    steepest_descend(steepest_ax_arr ,MPU_WINDOW_NUM_ACC ,&steepest_ax ,MPU_STEEPEST_NUM_ACC,(s32)  Acc_tmp[0]);
    steepest_descend(steepest_ay_arr ,MPU_WINDOW_NUM_ACC ,&steepest_ay ,MPU_STEEPEST_NUM_ACC,(s32)  Acc_tmp[1]);
    steepest_descend(steepest_az_arr ,MPU_WINDOW_NUM_ACC ,&steepest_az ,MPU_STEEPEST_NUM_ACC,(s32)  Acc_tmp[2]);
	steepest_descend(steepest_gx_arr ,MPU_WINDOW_NUM ,&steepest_gx ,MPU_STEEPEST_NUM,(s32) Gyro_tmp[0] );
	steepest_descend(steepest_gy_arr ,MPU_WINDOW_NUM ,&steepest_gy ,MPU_STEEPEST_NUM,(s32) Gyro_tmp[1]);
	steepest_descend(steepest_gz_arr ,MPU_WINDOW_NUM ,&steepest_gz ,MPU_STEEPEST_NUM,(s32) Gyro_tmp[2]);
   
   sensor_dir( 2,													//加速度计方向
				(float)steepest_ax.now_out,
				(float)steepest_ay.now_out,//sensor_val[A_Z],//
				(float)steepest_az.now_out,
				&g_FlyControlDataStruct.AppImuDataStruct.AccXFilterout ,
				&g_FlyControlDataStruct.AppImuDataStruct.AccYFilterout ,
				&g_FlyControlDataStruct.AppImuDataStruct.AccZFilterout);
						
	sensor_dir( 2,													//陀螺仪方向
				(float)steepest_gx.now_out,
				(float)steepest_gy.now_out,
				(float)steepest_gz.now_out,
				&g_FlyControlDataStruct.AppImuDataStruct.GyroXFilterout,
				&g_FlyControlDataStruct.AppImuDataStruct.GyroYFilterout,
				&g_FlyControlDataStruct.AppImuDataStruct.GyroZFilterout);
	
//======================================================================
	/*陀螺仪转换到度每秒*/
	g_FlyControlDataStruct.AppImuDataStruct.GyroXFilteroutdeg = g_FlyControlDataStruct.AppImuDataStruct.GyroXFilterout*0.0610361f ;//  /65535 * 4000; +-2000度
	g_FlyControlDataStruct.AppImuDataStruct.GyroYFilteroutdeg = g_FlyControlDataStruct.AppImuDataStruct.GyroYFilterout*0.0610361f ;
    g_FlyControlDataStruct.AppImuDataStruct.GyroZFilteroutdeg = g_FlyControlDataStruct.AppImuDataStruct.GyroZFilterout *0.0610361f ;
	
	/*加速度计转换到毫米每平方秒*/
	g_FlyControlDataStruct.AppImuDataStruct.AccXFilteroutcm =   g_FlyControlDataStruct.AppImuDataStruct.AccXFilterout *0.23926;
	g_FlyControlDataStruct.AppImuDataStruct.AccYFilteroutcm =   g_FlyControlDataStruct.AppImuDataStruct.AccYFilterout *0.23926;
	g_FlyControlDataStruct.AppImuDataStruct.AccZFilteroutcm =   g_FlyControlDataStruct.AppImuDataStruct.AccZFilterout *0.23926;
                
//  g_FlyControlDataStruct.AppImuDataStruct.GyroXFilterout = Gyro_tmp[0];
//	g_FlyControlDataStruct.AppImuDataStruct.GyroYFilterout=  Gyro_tmp[1];
//	g_FlyControlDataStruct.AppImuDataStruct.GyroZFilterout=  Gyro_tmp[2];
//    
//////	// 陀螺仪一阶低通滤波  给系统带来了严重滞后 降低滤波频率来提高灵敏度 但是现在没有时间来测试了  先记下 有空再看一下 
// 	g_FlyControlDataStruct.AppImuDataStruct.GyroXFilteroutdeg =-LPF_1st(x, Gyro_tmp[0]* TO_ANGLE,0.2f);	 x = g_FlyControlDataStruct.AppImuDataStruct.GyroXFilteroutdeg;
// 	g_FlyControlDataStruct.AppImuDataStruct.GyroYFilteroutdeg =-LPF_1st(y, Gyro_tmp[1]* TO_ANGLE,0.2f);	 y = g_FlyControlDataStruct.AppImuDataStruct.GyroYFilteroutdeg;
// 	g_FlyControlDataStruct.AppImuDataStruct.GyroZFilteroutdeg = LPF_1st(z, Gyro_tmp[2]* TO_ANGLE,0.2f);    z = g_FlyControlDataStruct.AppImuDataStruct.GyroZFilteroutdeg;

// 	g_FlyControlDataStruct.AppImuDataStruct.GyroXFilteroutdeg = -Gyro_tmp[0]*TO_ANGLE;
// 	g_FlyControlDataStruct.AppImuDataStruct.GyroYFilteroutdeg = -Gyro_tmp[1]*TO_ANGLE;
// 	g_FlyControlDataStruct.AppImuDataStruct.GyroZFilteroutdeg =  Gyro_tmp[2]*TO_ANGLE;

}

