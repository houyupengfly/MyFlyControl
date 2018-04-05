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

#include "Fc_systream_time.h"
#include "Fc_data_Struct.h"


#include "mymath.h"
#include <math.h>
#include "Algorithm_ahrs.h"
#include "Algorithm_math.h"
#include "Algorithm_filter.h"

/*============================ MACROS ========================================*/
/*============================ TYPES =========================================*/


/*============================ GLOBAL VARIABLES ==============================*/
extern  FlyControlDataStruct g_FlyControlDataStruct;
/*============================ LOCAL VARIABLES ===============================*/
/*============================ IMPLEMENTATION ================================*/
/*=============================================================================
 + ʵ�ֹ��ܣ�
 + ���ò������ܣ�
==============================================================================*/

///*=============================================================================
// + ʵ�ֹ��ܣ�
// + ���ò������ܣ�
//==============================================================================*/

void Fc_AHRS_Geteuler(float T)
{		
    static _lf_t err_lf_x;
	static _lf_t err_lf_y;
	static _lf_t err_lf_z;
	static _float_t vec_err_i;
	
	float kp = 0.6,ki = 0;
	
	float q0q1,q0q2,q1q1,q1q3,q2q2,q2q3,q3q3,q1q2,q0q3;//q0q0,
	float w_q,x_q,y_q,z_q;
	float acc_length,q_length;
	_float_t acc_norm;
	_float_t vec_err;
	_float_t d_angle;
	  
    w_q = g_FlyControlDataStruct.App_AngleStruct.w;
    x_q = g_FlyControlDataStruct.App_AngleStruct.x;
    y_q = g_FlyControlDataStruct.App_AngleStruct.y;
    z_q = g_FlyControlDataStruct.App_AngleStruct.z;
						
    q0q1 = w_q * x_q;
    q0q2 = w_q * y_q;
    q1q1 = x_q * x_q;
    q1q3 = x_q * z_q;
    q2q2 = y_q * y_q;
    q2q3 = y_q * z_q;
    q3q3 = z_q * z_q;
    q1q2 = x_q * y_q;
    q0q3 = w_q * z_q;
    

    // ���ٶȼƵĶ�������λ����
    acc_length = my_sqrt(my_pow(g_FlyControlDataStruct.AppImuDataStruct.AccXFilteroutcm) + my_pow(g_FlyControlDataStruct.AppImuDataStruct.AccYFilteroutcm) + my_pow(g_FlyControlDataStruct.AppImuDataStruct.AccZFilteroutcm));
    acc_norm.x = g_FlyControlDataStruct.AppImuDataStruct.AccXFilteroutcm/acc_length;
    acc_norm.y = g_FlyControlDataStruct.AppImuDataStruct.AccYFilteroutcm/acc_length;
    acc_norm.z = g_FlyControlDataStruct.AppImuDataStruct.AccZFilteroutcm/acc_length;
		
		// ���������µ�x������������λ����
    g_FlyControlDataStruct.App_AngleStruct.x_vec.x = 1 - (2*q2q2 + 2*q3q3);
    g_FlyControlDataStruct.App_AngleStruct.x_vec.y = 2*q1q2 - 2*q0q3;
    g_FlyControlDataStruct.App_AngleStruct.x_vec.z = 2*q1q3 + 2*q0q2;
		
		// ���������µ�y������������λ����
    g_FlyControlDataStruct.App_AngleStruct.y_vec.x = 2*q1q2 + 2*q0q3;
    g_FlyControlDataStruct.App_AngleStruct.y_vec.y = 1 - (2*q1q1 + 2*q3q3);
    g_FlyControlDataStruct.App_AngleStruct.y_vec.z = 2*q2q3 - 2*q0q1;
		
    // ���������µ�z������������Ч�����������������ٶ�����������λ����
    g_FlyControlDataStruct.App_AngleStruct.z_vec.x = 2*q1q3 - 2*q0q2;
    g_FlyControlDataStruct.App_AngleStruct.z_vec.y = 2*q2q3 + 2*q0q1;
    g_FlyControlDataStruct.App_AngleStruct.z_vec.z = 1 - (2*q1q1 + 2*q2q2);
		
	// �������������µ��˶����ٶȡ�(����̬�����޹�)
	g_FlyControlDataStruct.App_AngleStruct.a_acc.x = g_FlyControlDataStruct.AppImuDataStruct.AccXFilteroutcm - 980 *g_FlyControlDataStruct.App_AngleStruct.z_vec.x;
	g_FlyControlDataStruct.App_AngleStruct.a_acc.y = g_FlyControlDataStruct.AppImuDataStruct.AccYFilteroutcm - 980 *g_FlyControlDataStruct.App_AngleStruct.z_vec.y;
	g_FlyControlDataStruct.App_AngleStruct.a_acc.z = g_FlyControlDataStruct.AppImuDataStruct.AccZFilteroutcm - 980 *g_FlyControlDataStruct.App_AngleStruct.z_vec.z;
	
	// �������������µ��˶����ٶȡ�(����̬�����޹�)
	g_FlyControlDataStruct.AppImuDataStruct.Nz = g_FlyControlDataStruct.App_AngleStruct.z_vec.x *g_FlyControlDataStruct.App_AngleStruct.a_acc.x + g_FlyControlDataStruct.App_AngleStruct.z_vec.y *g_FlyControlDataStruct.App_AngleStruct.a_acc.y +  g_FlyControlDataStruct.App_AngleStruct.z_vec.z *g_FlyControlDataStruct.App_AngleStruct.a_acc.z;
	
    // ����ֵ���Ч���������Ĳ��������������
    vec_err.x =  (acc_norm.y * g_FlyControlDataStruct.App_AngleStruct.z_vec.z - g_FlyControlDataStruct.App_AngleStruct.z_vec.y * acc_norm.z);
    vec_err.y = -(acc_norm.x * g_FlyControlDataStruct.App_AngleStruct.z_vec.z - g_FlyControlDataStruct.App_AngleStruct.z_vec.x * acc_norm.z);
    vec_err.z = 0;//;
		
	//��ֹƵ��1hz�ĵ�ͨ�޷��˲�
	limit_filter(T,0.2f,&err_lf_x,vec_err.x);
	limit_filter(T,0.2f,&err_lf_y,vec_err.y);
	limit_filter(T,0.2f,&err_lf_z,vec_err.z);
	
	//������
	vec_err_i.x += err_lf_x.out *T *ki;
	vec_err_i.y += err_lf_y.out *T *ki;
	vec_err_i.z += err_lf_z.out *T *ki;
		
    // ����������ת�����ںϾ�������
    d_angle.x = (g_FlyControlDataStruct.AppImuDataStruct.GyroXFilteroutdeg *RAD_PER_DEG + (err_lf_x.out + vec_err_i.x) * kp) * T / 2 ;
    d_angle.y = (g_FlyControlDataStruct.AppImuDataStruct.GyroYFilteroutdeg *RAD_PER_DEG + (err_lf_y.out + vec_err_i.y) * kp) * T / 2 ;
    d_angle.z = (g_FlyControlDataStruct.AppImuDataStruct.GyroZFilteroutdeg *RAD_PER_DEG + (err_lf_z.out + vec_err_i.z) * kp) * T / 2 ;
    
    // ������̬��
    g_FlyControlDataStruct.App_AngleStruct.w = w_q           - x_q*d_angle.x - y_q*d_angle.y - z_q*d_angle.z;
    g_FlyControlDataStruct.App_AngleStruct.x = w_q*d_angle.x + x_q           + y_q*d_angle.z - z_q*d_angle.y;
    g_FlyControlDataStruct.App_AngleStruct.y = w_q*d_angle.y - x_q*d_angle.z + y_q           + z_q*d_angle.x;
    g_FlyControlDataStruct.App_AngleStruct.z = w_q*d_angle.z + x_q*d_angle.y - y_q*d_angle.x + z_q;
		
	q_length = my_sqrt(g_FlyControlDataStruct.App_AngleStruct.w*g_FlyControlDataStruct.App_AngleStruct.w + g_FlyControlDataStruct.App_AngleStruct.x*g_FlyControlDataStruct.App_AngleStruct.x + g_FlyControlDataStruct.App_AngleStruct.y*g_FlyControlDataStruct.App_AngleStruct.y + g_FlyControlDataStruct.App_AngleStruct.z*g_FlyControlDataStruct.App_AngleStruct.z);
   
    g_FlyControlDataStruct.App_AngleStruct.w /= q_length;
    g_FlyControlDataStruct.App_AngleStruct.x /= q_length;
    g_FlyControlDataStruct.App_AngleStruct.y /= q_length;
    g_FlyControlDataStruct.App_AngleStruct.z /= q_length;
	
    g_FlyControlDataStruct.App_AngleStruct.Pitch = asin(2*q1q3 - 2*q0q2)*57.30f;
    g_FlyControlDataStruct.App_AngleStruct.Roll = fast_atan2(2*q2q3 + 2*q0q1, -2*q1q1-2*q2q2 + 1)*57.30f; 
    g_FlyControlDataStruct.App_AngleStruct.Yaw = -fast_atan2(2*q1q2 + 2*q0q3, -2*q2q2-2*q3q3 + 1)*57.30f; 
    
}
    


