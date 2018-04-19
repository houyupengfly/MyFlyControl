
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
#include "Fc_SeedEstimated.h"
#include "device_mpc2520.h"
#include "Algorithm_ahrs.h"
#include "filter.h"
#include "Fc_data_Struct.h"
#include "Fc_systream_time.h"
#include "mymath.h"

/*============================ MACROS ========================================*/
float   TIME_CONTANST_ZER=3.0f;
#define K_ACC_ZER 	        (1.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER * TIME_CONTANST_ZER))
#define K_VEL_ZER	        (3.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER))//20															
#define K_POS_ZER           (3.0f / TIME_CONTANST_ZER)
#define Num  50


#define TIME_CONTANST_XY      2.5f
#define K_ACC_XY	         (1.0f / (TIME_CONTANST_XY * TIME_CONTANST_XY * TIME_CONTANST_XY))
#define K_VEL_XY             (3.0f / (TIME_CONTANST_XY * TIME_CONTANST_XY))															
#define K_POS_XY             (3.0f / TIME_CONTANST_XY)

#define CNTLCYCLE  0.005f


/*============================ TYPES =========================================*/
SINS NewSinsData;
SINS OrigionSinsData;
typedef struct
{
 float Pit;
 float Rol;
}Vector2_Ang;


typedef struct
{
 float Pit;
 float Rol;
}Vector2_Body;

/*============================ GLOBAL VARIABLES ==============================*/
extern FlyControlDataStruct g_FlyControlDataStruct;


float Altitude_Dealt=0;
float Altitude_Estimate=0;

float X_Delta=0,Y_Delta=0;
float k;

Vector2_Body Pos_Err_On_Accel={0};
Vector2_Body  Accel_Correction_BF={0};

/*============================ STATIC VARIABLES ==============================*/

float Acceleration_Length=0;

float pos_correction[3]={0,0,0};
float acc_correction[3]={0,0,0};
float vel_correction[3]={0,0,0};

float SpeedDealt[3]={0}; //速度修正后

float Position_History[2][100]={0};
float Vel_History[2][100]={0};

float z_est[3] = {0};//! z轴，1.估算高度; 2.速度; 3.加速度
_fix_inte_filter_st wcz_spe_fus,wcz_hei_fus;

s32 ref_height_old,ref_speed_old;

s32 wcz_ref_height,wcz_ref_speed;

static s32 wcz_acc;

s32 baro_height,baro_height_old;
s32 baro_speed_o,baro_speed;

#define MONUM 10
float speed_av_arr[MONUM];
float speed_av;
u16 speed_av_cnt;

float speed_delta;
float speed_delta_lpf;
/*============================ FUNCTION ======================================*/

void Fc_Hight_Get(float dT)
{
    baro_height_old = baro_height;
	baro_height = (user_spl0601_get());
	baro_speed_o = safe_div(baro_height - baro_height_old,dT,0);

	//计算速度
	Moving_Average(speed_av_arr,MONUM ,&speed_av_cnt,baro_speed_o,&speed_av);
	speed_delta = LIMIT(speed_av - baro_speed,-2000*dT,2000*dT);
	LPF_1_(0.5f,dT,speed_delta,speed_delta_lpf);
	baro_speed += speed_delta *LIMIT((ABS(speed_delta_lpf)/(2000*dT)),0,1);
}


void Fc_Alti_Estimate(u8 dT_ms)
{
    wcz_ref_height = baro_height;
	wcz_ref_speed  = baro_speed;
	wcz_acc        = (s32)g_FlyControlDataStruct.AppImuDataStruct.Nz;
	
	//陀螺仪加速度积分速度和气压计速度融合
	wcz_spe_fus.fix_kp = 0.8f;
	wcz_spe_fus.in_est_d = wcz_acc;
	wcz_spe_fus.in_obs = wcz_ref_speed;
	wcz_spe_fus.e_limit = 200;
	fix_inte_filter(dT_ms*1e-3f,&wcz_spe_fus);
	
	//高度
	wcz_hei_fus.fix_kp = 0.8f;
	wcz_hei_fus.in_est_d = wcz_spe_fus.out;
	wcz_hei_fus.in_obs = baro_height;
	wcz_hei_fus.e_limit = 100;
	fix_inte_filter(dT_ms*1e-3f,&wcz_hei_fus);
   
    NewSinsData.Position[0]=wcz_hei_fus.out;
    NewSinsData.Speed[0]=wcz_spe_fus.out;
}
void Fc_Alti_Estimate_Reset()
{
	wcz_spe_fus.out = 0;
	wcz_spe_fus.e = 0;

	wcz_hei_fus.out = 0;
	wcz_hei_fus.e = 0;	
}
//光流数据与高度简单融合系数
void Flow_High_Cal(void)
{
	if( wcz_hei_fus.out<=0 ) k = 0.0f;
	else
	{
		k = wcz_hei_fus.out * 0.02f;
		k = LIMIT(k,0.0f,1.0f);
	}
}
/*=============================================================================
 + 实现功能：
 + 调用参数功能：
//==============================================================================*/
void Fc_Position_Estimate(u8 dT_ms)
{    
    float angle;   
    angle = my_sqrt(g_FlyControlDataStruct.App_AngleStruct.Roll*g_FlyControlDataStruct.App_AngleStruct.Roll + g_FlyControlDataStruct.App_AngleStruct.Pitch*g_FlyControlDataStruct.App_AngleStruct.Pitch);
    if(angle>30)	angle=30;	//限幅
    angle /= 57.29f;	 	//角度转弧度
   
    //对光流数据进行倾角补偿
    g_FlyControlDataStruct.AppPostionCtrl.Postionx/= my_cos(angle);
    g_FlyControlDataStruct.AppPostionCtrl.Postiony/= my_cos(angle);
    g_FlyControlDataStruct.AppPostionCtrl.Postionx*=my_abs(wcz_hei_fus.out*0.2);
    g_FlyControlDataStruct.AppPostionCtrl.Postiony*=my_abs(wcz_hei_fus.out*0.2);
    //积分用来求位移 
    g_FlyControlDataStruct.AppPostionCtrl.Postionx_i+= g_FlyControlDataStruct.AppPostionCtrl.Postionx;
    g_FlyControlDataStruct.AppPostionCtrl.Postiony_i+= g_FlyControlDataStruct.AppPostionCtrl.Postiony;
//    g_FlyControlDataStruct.AppPostionCtrl.Postionx_i*=my_abs(wcz_hei_fus.out*0.02);
//    g_FlyControlDataStruct.AppPostionCtrl.Postiony_i*=my_abs(wcz_hei_fus.out*0.02);
    //进行低通滤波
    g_FlyControlDataStruct.AppPostionCtrl.Postionx_i_filter += (g_FlyControlDataStruct.AppPostionCtrl.Postionx_i - g_FlyControlDataStruct.AppPostionCtrl.Postionx_i_filter) *0.2;  
    g_FlyControlDataStruct.AppPostionCtrl.Postiony_i_filter += (g_FlyControlDataStruct.AppPostionCtrl.Postiony_i - g_FlyControlDataStruct.AppPostionCtrl.Postiony_i_filter) *0.2;  
//    //姿态纠正融合补偿
//    g_FlyControlDataStruct.AppPostionCtrl.Postionx_i_angle += (600*tan(g_FlyControlDataStruct.App_AngleStruct.Pitch *0.017453) - g_FlyControlDataStruct.AppPostionCtrl.Postionx_i_angle) *0.2;  
//    g_FlyControlDataStruct.AppPostionCtrl.Postiony_i_angle += (600*tan(g_FlyControlDataStruct.App_AngleStruct.Roll  *0.017453) - g_FlyControlDataStruct.AppPostionCtrl.Postiony_i_angle) *0.2;  
//    //位置最终输出 
//    g_FlyControlDataStruct.AppPostionCtrl.Postionx_out =  g_FlyControlDataStruct.AppPostionCtrl.Postionx_i_filter - g_FlyControlDataStruct.AppPostionCtrl.Postionx_i_angle;  
//    g_FlyControlDataStruct.AppPostionCtrl.Postiony_out =  g_FlyControlDataStruct.AppPostionCtrl.Postiony_i_filter - g_FlyControlDataStruct.AppPostionCtrl.Postiony_i_angle; 
    //微分求位移速度
    g_FlyControlDataStruct.AppPostionCtrl.Postionx_speed = ( g_FlyControlDataStruct.AppPostionCtrl.Postionx_i_filter - g_FlyControlDataStruct.AppPostionCtrl.Postionx_out_old)/100;
    g_FlyControlDataStruct.AppPostionCtrl.Postionx_out_old = g_FlyControlDataStruct.AppPostionCtrl.Postionx_i_filter;  
    
    g_FlyControlDataStruct.AppPostionCtrl.Postiony_speed = ( g_FlyControlDataStruct.AppPostionCtrl.Postiony_i_filter - g_FlyControlDataStruct.AppPostionCtrl.Postiony_out_old)/100;  
    g_FlyControlDataStruct.AppPostionCtrl.Postiony_out_old = g_FlyControlDataStruct.AppPostionCtrl.Postiony_i_filter;  
  
    //速度低通滤波  
    g_FlyControlDataStruct.AppPostionCtrl.Postionx_speed_out += ( g_FlyControlDataStruct.AppPostionCtrl.Postionx_speed  - g_FlyControlDataStruct.AppPostionCtrl.Postionx_speed_out) * 0.1f;  
    g_FlyControlDataStruct.AppPostionCtrl.Postiony_speed_out += ( g_FlyControlDataStruct.AppPostionCtrl.Postiony_speed  - g_FlyControlDataStruct.AppPostionCtrl.Postiony_speed_out) * 0.1f;
    
}
/*=============================================================================
 + 实现功能：
 + 调用参数功能：
//==============================================================================*/
//void Fc_Alti_Estimatepx4(void)
//{ 
//  	float alt_estimate_loop_time = Fc_Call_timer_cycle(1);
//    
//	float err_alti = 0.0f;	//! 估计与气压计误差
//  	
//  	static float w_z_baro	= 0.8;		//0.05f; ! Z轴气压计权重
// 
//  	float e = 0;
//  	float w = 0;
//  	
//  	float corr_pos = 0.0f;	//! 高度校正系数
//  	float corr_vel = 0.0f;	//! 速度校正系数
//  	float corr_acc = 0.0f;	//! acc校正系数
//  	
//	  //高度采集结束
//  	if(g_FlyControlDataStruct.AppConfigDataStruct.BaroFlag == Baro_State_Ready ) 
//	    {		   	
//  		z_est[2] = g_FlyControlDataStruct.AppImuDataStruct.Nz;
//  		
//  		//! 当前高度误差
//  		err_alti = g_FlyControlDataStruct.AppHightCtrl.BaroHight - z_est[0];
//  		
//  		
//  		//! 修正加速度计
//  		e = err_alti;
//  		w = w_z_baro;
//  		corr_acc = e * w * w * w * alt_estimate_loop_time;
//  		z_est[2] += corr_acc;
//  		
//  		
//  		//! 速度变化量
//  		float deltVel = z_est[2] * alt_estimate_loop_time;//AT
//  		
//  		
//  		//! 估计高度
//  		z_est[0] +=  z_est[1] * alt_estimate_loop_time + 0.5 * deltVel * alt_estimate_loop_time; //V0T+1/2AT2
//  		
//		
//  		//! 修正位置
//  		e = err_alti;
//  		w = w_z_baro;
//  		corr_pos = e * w * alt_estimate_loop_time;
//  		z_est[0] += corr_pos;
//  		
//  		
//  		//! 修正速度
//  		z_est[1] += deltVel;	//! 得到速度
//  		e = err_alti;
//  		w = w_z_baro;
//  		corr_vel = e * w * w * alt_estimate_loop_time;
//  		z_est[1] += corr_vel;
//  		

//  		//! \note: 更新数据
//  		
//  	    NewSinsData.Position[YAW]  = z_est[0];
//  		NewSinsData.Speed[YAW] = z_est[1];
//  		NewSinsData.Acceleration[YAW] = z_est[2];
//		
//   	}
//        
//}
/*=============================================================================
 + 实现功能：
 + 调用参数功能：
==============================================================================*/
///*确定以地理坐标系作为导航坐标系*/
//void SpeedEstimated(void)
//{	
//	 
//	 OrigionSinsData.acc_b[0] = g_FlyControlDataStruct.AppImuDataStruct.AccXFilterout;
//	 OrigionSinsData.acc_b[1] = g_FlyControlDataStruct.AppImuDataStruct.AccYFilterout;
//	 OrigionSinsData.acc_b[2] = g_FlyControlDataStruct.AppImuDataStruct.AccZFilterout;
//	
//	//! 转换到地理坐标系
//	 for(u8 i=0; i<3; i++)
//	 {
//			 for(u8 j=0; j<3; j++)
//		  {
//			  OrigionSinsData.acc_n[i] += DCMgb[j][i]* OrigionSinsData.acc_b[j];
//		  }
//	 }

//	 OrigionSinsData.accNED[0] = OrigionSinsData.acc_n[0]; //地理坐标系下X轴速度
//	 OrigionSinsData.accNED[1] = OrigionSinsData.acc_n[1]; //地理坐标系下Y轴速度
//	 OrigionSinsData.accNED[2] = OrigionSinsData.acc_n[2]; //地理坐标系下Z轴速度
//	
//	 OrigionSinsData.acc_n[0] =0;
//	 OrigionSinsData.acc_n[1] =0;
//	 OrigionSinsData.acc_n[2] =0;
//	 
//	 /*X轴  机头*/
//     OrigionSinsData.accNED[0]*=100;//加速度cm/s^2
//     /*Y轴  右侧*/  
//     OrigionSinsData.accNED[1]*=100;//加速度cm/s^2

//     /*Z轴  向下*/
//   
//     OrigionSinsData.accNED[2] =OrigionSinsData.accNED[2]-9.80;//减去重力加速度
//     OrigionSinsData.accNED[2]*=100;//加速度cm/s^2

////     /*导航坐标系下加速度的模*/
////     Acceleration_Length=my_sqrt(g_FlyControlDataStruct.AppSINS.accNED[2]*g_FlyControlDataStruct.AppSINS.accNED[2] + g_FlyControlDataStruct.AppSINS.accNED[1]*g_FlyControlDataStruct.AppSINS.accNED[1] + g_FlyControlDataStruct.AppSINS.accNED[0]*g_FlyControlDataStruct.AppSINS.accNED[0]);
//                                 
// }
/*=============================================================================
 + 实现功能：
 + 调用参数功能：
==============================================================================*/
//void Strapdown_INS_Reset(SINS *Ins,u8 Axis,float Pos_Target,float Vel_Target)
//{
//      u16 Cnt=0;
//      Ins->Position[Axis]=Pos_Target;//位置重置
//      Ins->Speed[Axis]=Vel_Target;//速度重置
//      Ins->Acceleration[Axis]=0.0f;//加速度清零
//      Ins->Acce_Bias[Axis]=0.0f;
//      for(Cnt=Num-1;Cnt>0;Cnt--)//历史位置值，全部装载为当前观测值
//      {
//         Ins->Pos_History[Axis][Cnt]=Pos_Target;
//      }
//         Ins->Pos_History[Axis][0]=Pos_Target;
//      for(Cnt=Num-1;Cnt>0;Cnt--)//历史速度值，全部装载为当前观测值
//      {
//         Ins->Vel_History[Axis][Cnt]=Vel_Target;
//      }
//         Ins->Vel_History[Axis][0]=Vel_Target;
//      pos_correction[Axis]=0;//清空惯导融合量
//      acc_correction[Axis]=0;
//      vel_correction[Axis]=0;
//}
 /*=============================================================================
 + 实现功能：
 + 调用参数功能：
==============================================================================*/
/*SINS Z轴加速度 位置 速度 修正*/
//void SINS_Z(float dt)
//{     
//	  static u16 Save_Cnt=0;
//      Save_Cnt++;//数据存储周期
//      //由观测量（气压计）得到状态误差
//      Altitude_Dealt=g_FlyControlDataStruct.AppHightCtrl.BaroHight -NewSinsData.Position[YAW];//气压计(超声波)与SINS估计量的差，单位cm
//      //三路积分反馈量修正惯导
//      acc_correction[YAW] +=Altitude_Dealt* K_ACC_ZER*dt ;//加速度矫正量(s)
//      vel_correction[YAW] +=Altitude_Dealt* K_VEL_ZER*dt ;//速度矫正量
//      pos_correction[YAW] +=Altitude_Dealt* K_POS_ZER*dt ;//位置矫正量
//	
//      //加速度计矫正后更新  矫正量就是估计量加上根据误差而来的修正量
//      NewSinsData.Last_Acceleration[YAW]= NewSinsData.Acceleration[YAW];//上一次加速度量
//      NewSinsData.Acceleration[YAW]     = g_FlyControlDataStruct.AppImuDataStruct.Nz + acc_correction[YAW];
//	
//      //速度增量矫正后更新，用于更新位置,由于步长h=0.005,相对较长，
//      //这里采用二阶龙格库塔法更新微分方程，不建议用更高阶段，因为加速度信号非平滑
//	  //两次加速度平均值乘以时间求速度
//      SpeedDealt[YAW]=+(NewSinsData.Last_Acceleration[YAW] + NewSinsData.Acceleration[YAW])*dt/2.0;
//	
//      //原始位置更新 新速度加速度修正
//      OrigionSinsData.Position[YAW]+=(NewSinsData.Speed[YAW]+0.5* SpeedDealt[YAW])*dt;//vot +1/2att
//      //位置矫正后更新  原始位置加修正
//      NewSinsData.Position[YAW] = OrigionSinsData.Position[YAW] + pos_correction[YAW];
//      //原始速度更新
//      OrigionSinsData.Speed[YAW] += SpeedDealt[YAW];
//      //速度矫正后更新
//      NewSinsData.Speed[YAW] = OrigionSinsData.Speed[YAW] + vel_correction[YAW];
//     
//}
//void SINS_XY(float dt)
//{     
//      vel_correction[YAW] +=g_FlyControlDataStruct.AppImuDataStruct.Nx*dt ;//速度矫正量
//      pos_correction[PITCH]+=(g_FlyControlDataStruct.AppImuDataStruct.Nx*dt);//位置矫正量
//	     
//}