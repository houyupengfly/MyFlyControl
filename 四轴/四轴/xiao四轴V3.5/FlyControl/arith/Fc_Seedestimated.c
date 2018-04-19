
/*=============================================================================
** ���ߣ�������
** ���ڣ�2017/10/31
** �汾��V1.0
** ������: 
** �������ܣ�
** ���������
** ��� ��1  ����ֵ��
          2  �ı��ȫ�ֱ���
** ���������õ������������汾�ţ���
** ��ע��
** �޸���־�� �����ߣ����ڣ��޸����ݼ�ԭ��
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

float SpeedDealt[3]={0}; //�ٶ�������

float Position_History[2][100]={0};
float Vel_History[2][100]={0};

float z_est[3] = {0};//! z�ᣬ1.����߶�; 2.�ٶ�; 3.���ٶ�
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

	//�����ٶ�
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
	
	//�����Ǽ��ٶȻ����ٶȺ���ѹ���ٶ��ں�
	wcz_spe_fus.fix_kp = 0.8f;
	wcz_spe_fus.in_est_d = wcz_acc;
	wcz_spe_fus.in_obs = wcz_ref_speed;
	wcz_spe_fus.e_limit = 200;
	fix_inte_filter(dT_ms*1e-3f,&wcz_spe_fus);
	
	//�߶�
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
//����������߶ȼ��ں�ϵ��
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
 + ʵ�ֹ��ܣ�
 + ���ò������ܣ�
//==============================================================================*/
void Fc_Position_Estimate(u8 dT_ms)
{    
    float angle;   
    angle = my_sqrt(g_FlyControlDataStruct.App_AngleStruct.Roll*g_FlyControlDataStruct.App_AngleStruct.Roll + g_FlyControlDataStruct.App_AngleStruct.Pitch*g_FlyControlDataStruct.App_AngleStruct.Pitch);
    if(angle>30)	angle=30;	//�޷�
    angle /= 57.29f;	 	//�Ƕ�ת����
   
    //�Թ������ݽ�����ǲ���
    g_FlyControlDataStruct.AppPostionCtrl.Postionx/= my_cos(angle);
    g_FlyControlDataStruct.AppPostionCtrl.Postiony/= my_cos(angle);
    g_FlyControlDataStruct.AppPostionCtrl.Postionx*=my_abs(wcz_hei_fus.out*0.2);
    g_FlyControlDataStruct.AppPostionCtrl.Postiony*=my_abs(wcz_hei_fus.out*0.2);
    //����������λ�� 
    g_FlyControlDataStruct.AppPostionCtrl.Postionx_i+= g_FlyControlDataStruct.AppPostionCtrl.Postionx;
    g_FlyControlDataStruct.AppPostionCtrl.Postiony_i+= g_FlyControlDataStruct.AppPostionCtrl.Postiony;
//    g_FlyControlDataStruct.AppPostionCtrl.Postionx_i*=my_abs(wcz_hei_fus.out*0.02);
//    g_FlyControlDataStruct.AppPostionCtrl.Postiony_i*=my_abs(wcz_hei_fus.out*0.02);
    //���е�ͨ�˲�
    g_FlyControlDataStruct.AppPostionCtrl.Postionx_i_filter += (g_FlyControlDataStruct.AppPostionCtrl.Postionx_i - g_FlyControlDataStruct.AppPostionCtrl.Postionx_i_filter) *0.2;  
    g_FlyControlDataStruct.AppPostionCtrl.Postiony_i_filter += (g_FlyControlDataStruct.AppPostionCtrl.Postiony_i - g_FlyControlDataStruct.AppPostionCtrl.Postiony_i_filter) *0.2;  
//    //��̬�����ںϲ���
//    g_FlyControlDataStruct.AppPostionCtrl.Postionx_i_angle += (600*tan(g_FlyControlDataStruct.App_AngleStruct.Pitch *0.017453) - g_FlyControlDataStruct.AppPostionCtrl.Postionx_i_angle) *0.2;  
//    g_FlyControlDataStruct.AppPostionCtrl.Postiony_i_angle += (600*tan(g_FlyControlDataStruct.App_AngleStruct.Roll  *0.017453) - g_FlyControlDataStruct.AppPostionCtrl.Postiony_i_angle) *0.2;  
//    //λ��������� 
//    g_FlyControlDataStruct.AppPostionCtrl.Postionx_out =  g_FlyControlDataStruct.AppPostionCtrl.Postionx_i_filter - g_FlyControlDataStruct.AppPostionCtrl.Postionx_i_angle;  
//    g_FlyControlDataStruct.AppPostionCtrl.Postiony_out =  g_FlyControlDataStruct.AppPostionCtrl.Postiony_i_filter - g_FlyControlDataStruct.AppPostionCtrl.Postiony_i_angle; 
    //΢����λ���ٶ�
    g_FlyControlDataStruct.AppPostionCtrl.Postionx_speed = ( g_FlyControlDataStruct.AppPostionCtrl.Postionx_i_filter - g_FlyControlDataStruct.AppPostionCtrl.Postionx_out_old)/100;
    g_FlyControlDataStruct.AppPostionCtrl.Postionx_out_old = g_FlyControlDataStruct.AppPostionCtrl.Postionx_i_filter;  
    
    g_FlyControlDataStruct.AppPostionCtrl.Postiony_speed = ( g_FlyControlDataStruct.AppPostionCtrl.Postiony_i_filter - g_FlyControlDataStruct.AppPostionCtrl.Postiony_out_old)/100;  
    g_FlyControlDataStruct.AppPostionCtrl.Postiony_out_old = g_FlyControlDataStruct.AppPostionCtrl.Postiony_i_filter;  
  
    //�ٶȵ�ͨ�˲�  
    g_FlyControlDataStruct.AppPostionCtrl.Postionx_speed_out += ( g_FlyControlDataStruct.AppPostionCtrl.Postionx_speed  - g_FlyControlDataStruct.AppPostionCtrl.Postionx_speed_out) * 0.1f;  
    g_FlyControlDataStruct.AppPostionCtrl.Postiony_speed_out += ( g_FlyControlDataStruct.AppPostionCtrl.Postiony_speed  - g_FlyControlDataStruct.AppPostionCtrl.Postiony_speed_out) * 0.1f;
    
}
/*=============================================================================
 + ʵ�ֹ��ܣ�
 + ���ò������ܣ�
//==============================================================================*/
//void Fc_Alti_Estimatepx4(void)
//{ 
//  	float alt_estimate_loop_time = Fc_Call_timer_cycle(1);
//    
//	float err_alti = 0.0f;	//! ��������ѹ�����
//  	
//  	static float w_z_baro	= 0.8;		//0.05f; ! Z����ѹ��Ȩ��
// 
//  	float e = 0;
//  	float w = 0;
//  	
//  	float corr_pos = 0.0f;	//! �߶�У��ϵ��
//  	float corr_vel = 0.0f;	//! �ٶ�У��ϵ��
//  	float corr_acc = 0.0f;	//! accУ��ϵ��
//  	
//	  //�߶Ȳɼ�����
//  	if(g_FlyControlDataStruct.AppConfigDataStruct.BaroFlag == Baro_State_Ready ) 
//	    {		   	
//  		z_est[2] = g_FlyControlDataStruct.AppImuDataStruct.Nz;
//  		
//  		//! ��ǰ�߶����
//  		err_alti = g_FlyControlDataStruct.AppHightCtrl.BaroHight - z_est[0];
//  		
//  		
//  		//! �������ٶȼ�
//  		e = err_alti;
//  		w = w_z_baro;
//  		corr_acc = e * w * w * w * alt_estimate_loop_time;
//  		z_est[2] += corr_acc;
//  		
//  		
//  		//! �ٶȱ仯��
//  		float deltVel = z_est[2] * alt_estimate_loop_time;//AT
//  		
//  		
//  		//! ���Ƹ߶�
//  		z_est[0] +=  z_est[1] * alt_estimate_loop_time + 0.5 * deltVel * alt_estimate_loop_time; //V0T+1/2AT2
//  		
//		
//  		//! ����λ��
//  		e = err_alti;
//  		w = w_z_baro;
//  		corr_pos = e * w * alt_estimate_loop_time;
//  		z_est[0] += corr_pos;
//  		
//  		
//  		//! �����ٶ�
//  		z_est[1] += deltVel;	//! �õ��ٶ�
//  		e = err_alti;
//  		w = w_z_baro;
//  		corr_vel = e * w * w * alt_estimate_loop_time;
//  		z_est[1] += corr_vel;
//  		

//  		//! \note: ��������
//  		
//  	    NewSinsData.Position[YAW]  = z_est[0];
//  		NewSinsData.Speed[YAW] = z_est[1];
//  		NewSinsData.Acceleration[YAW] = z_est[2];
//		
//   	}
//        
//}
/*=============================================================================
 + ʵ�ֹ��ܣ�
 + ���ò������ܣ�
==============================================================================*/
///*ȷ���Ե�������ϵ��Ϊ��������ϵ*/
//void SpeedEstimated(void)
//{	
//	 
//	 OrigionSinsData.acc_b[0] = g_FlyControlDataStruct.AppImuDataStruct.AccXFilterout;
//	 OrigionSinsData.acc_b[1] = g_FlyControlDataStruct.AppImuDataStruct.AccYFilterout;
//	 OrigionSinsData.acc_b[2] = g_FlyControlDataStruct.AppImuDataStruct.AccZFilterout;
//	
//	//! ת������������ϵ
//	 for(u8 i=0; i<3; i++)
//	 {
//			 for(u8 j=0; j<3; j++)
//		  {
//			  OrigionSinsData.acc_n[i] += DCMgb[j][i]* OrigionSinsData.acc_b[j];
//		  }
//	 }

//	 OrigionSinsData.accNED[0] = OrigionSinsData.acc_n[0]; //��������ϵ��X���ٶ�
//	 OrigionSinsData.accNED[1] = OrigionSinsData.acc_n[1]; //��������ϵ��Y���ٶ�
//	 OrigionSinsData.accNED[2] = OrigionSinsData.acc_n[2]; //��������ϵ��Z���ٶ�
//	
//	 OrigionSinsData.acc_n[0] =0;
//	 OrigionSinsData.acc_n[1] =0;
//	 OrigionSinsData.acc_n[2] =0;
//	 
//	 /*X��  ��ͷ*/
//     OrigionSinsData.accNED[0]*=100;//���ٶ�cm/s^2
//     /*Y��  �Ҳ�*/  
//     OrigionSinsData.accNED[1]*=100;//���ٶ�cm/s^2

//     /*Z��  ����*/
//   
//     OrigionSinsData.accNED[2] =OrigionSinsData.accNED[2]-9.80;//��ȥ�������ٶ�
//     OrigionSinsData.accNED[2]*=100;//���ٶ�cm/s^2

////     /*��������ϵ�¼��ٶȵ�ģ*/
////     Acceleration_Length=my_sqrt(g_FlyControlDataStruct.AppSINS.accNED[2]*g_FlyControlDataStruct.AppSINS.accNED[2] + g_FlyControlDataStruct.AppSINS.accNED[1]*g_FlyControlDataStruct.AppSINS.accNED[1] + g_FlyControlDataStruct.AppSINS.accNED[0]*g_FlyControlDataStruct.AppSINS.accNED[0]);
//                                 
// }
/*=============================================================================
 + ʵ�ֹ��ܣ�
 + ���ò������ܣ�
==============================================================================*/
//void Strapdown_INS_Reset(SINS *Ins,u8 Axis,float Pos_Target,float Vel_Target)
//{
//      u16 Cnt=0;
//      Ins->Position[Axis]=Pos_Target;//λ������
//      Ins->Speed[Axis]=Vel_Target;//�ٶ�����
//      Ins->Acceleration[Axis]=0.0f;//���ٶ�����
//      Ins->Acce_Bias[Axis]=0.0f;
//      for(Cnt=Num-1;Cnt>0;Cnt--)//��ʷλ��ֵ��ȫ��װ��Ϊ��ǰ�۲�ֵ
//      {
//         Ins->Pos_History[Axis][Cnt]=Pos_Target;
//      }
//         Ins->Pos_History[Axis][0]=Pos_Target;
//      for(Cnt=Num-1;Cnt>0;Cnt--)//��ʷ�ٶ�ֵ��ȫ��װ��Ϊ��ǰ�۲�ֵ
//      {
//         Ins->Vel_History[Axis][Cnt]=Vel_Target;
//      }
//         Ins->Vel_History[Axis][0]=Vel_Target;
//      pos_correction[Axis]=0;//��չߵ��ں���
//      acc_correction[Axis]=0;
//      vel_correction[Axis]=0;
//}
 /*=============================================================================
 + ʵ�ֹ��ܣ�
 + ���ò������ܣ�
==============================================================================*/
/*SINS Z����ٶ� λ�� �ٶ� ����*/
//void SINS_Z(float dt)
//{     
//	  static u16 Save_Cnt=0;
//      Save_Cnt++;//���ݴ洢����
//      //�ɹ۲�������ѹ�ƣ��õ�״̬���
//      Altitude_Dealt=g_FlyControlDataStruct.AppHightCtrl.BaroHight -NewSinsData.Position[YAW];//��ѹ��(������)��SINS�������Ĳ��λcm
//      //��·���ַ����������ߵ�
//      acc_correction[YAW] +=Altitude_Dealt* K_ACC_ZER*dt ;//���ٶȽ�����(s)
//      vel_correction[YAW] +=Altitude_Dealt* K_VEL_ZER*dt ;//�ٶȽ�����
//      pos_correction[YAW] +=Altitude_Dealt* K_POS_ZER*dt ;//λ�ý�����
//	
//      //���ٶȼƽ��������  ���������ǹ��������ϸ�����������������
//      NewSinsData.Last_Acceleration[YAW]= NewSinsData.Acceleration[YAW];//��һ�μ��ٶ���
//      NewSinsData.Acceleration[YAW]     = g_FlyControlDataStruct.AppImuDataStruct.Nz + acc_correction[YAW];
//	
//      //�ٶ�������������£����ڸ���λ��,���ڲ���h=0.005,��Խϳ���
//      //������ö����������������΢�ַ��̣��������ø��߽׶Σ���Ϊ���ٶ��źŷ�ƽ��
//	  //���μ��ٶ�ƽ��ֵ����ʱ�����ٶ�
//      SpeedDealt[YAW]=+(NewSinsData.Last_Acceleration[YAW] + NewSinsData.Acceleration[YAW])*dt/2.0;
//	
//      //ԭʼλ�ø��� ���ٶȼ��ٶ�����
//      OrigionSinsData.Position[YAW]+=(NewSinsData.Speed[YAW]+0.5* SpeedDealt[YAW])*dt;//vot +1/2att
//      //λ�ý��������  ԭʼλ�ü�����
//      NewSinsData.Position[YAW] = OrigionSinsData.Position[YAW] + pos_correction[YAW];
//      //ԭʼ�ٶȸ���
//      OrigionSinsData.Speed[YAW] += SpeedDealt[YAW];
//      //�ٶȽ��������
//      NewSinsData.Speed[YAW] = OrigionSinsData.Speed[YAW] + vel_correction[YAW];
//     
//}
//void SINS_XY(float dt)
//{     
//      vel_correction[YAW] +=g_FlyControlDataStruct.AppImuDataStruct.Nx*dt ;//�ٶȽ�����
//      pos_correction[PITCH]+=(g_FlyControlDataStruct.AppImuDataStruct.Nx*dt);//λ�ý�����
//	     
//}