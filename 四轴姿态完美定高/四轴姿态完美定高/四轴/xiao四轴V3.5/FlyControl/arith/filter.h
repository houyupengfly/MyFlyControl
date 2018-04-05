#ifndef __FILTER_H
#define __FILTER_H

#include "stm32f10x.h"
#include "mymath.h"
#include "math.h"


typedef struct
{
	float in_est;    //Estimator
	float in_obs;    //Observation
	
	float fix_ki;
	float ei_limit;    
	
	float e;
	float ei;

	float out;
}_inte_fix_filter_st;

typedef struct
{
	float in_est_d;   //Estimator
	float in_obs;    //Observation
	
	float fix_kp;
	float e_limit;

	float e;

	float out;
}_fix_inte_filter_st;

typedef struct
{
	float integration_1;
	float integration_2;
	float out_tmp;
	float out;

}_com_fil_t;

typedef struct
{
	float lpf_1;

	float out;
}_lf_t;

typedef struct
{
	float lpf_1;
	float lpf_2;
	float in_old;
	float out;
}_jldf_t;

typedef struct
{
	u8 cnt;

	s32 lst_pow_sum;
	
	s32 now_out;
	s32 lst_out;
	s32 now_velocity_xdt;
} _steepest_st;

#define LPF_1_(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *6.28f *(t) ) ) ) *( (in) - (out) ))

void inte_fix_filter(float dT,_inte_fix_filter_st *data);
void fix_inte_filter(float dT,_fix_inte_filter_st *data);
	
void steepest_descend(s32 arr[],u8 len,_steepest_st *steepest,u8 step_num,s32 in);

void limit_filter(float T,float hz,_lf_t *data,float in);

void jyoun_limit_deadzone_filter(float T,float hz1,float hz2,_jldf_t *data,float in);//白噪声滤波

void jyoun_filter(float dT,float hz,float ref_value,float exp,float fb,float *out);
//float Moving_Average(u8 item,u8 width_num,float in);
void Moving_Average(float moavarray[],//滤波数组 数组长度：len+1
										u16 len ,//滤波数据长度
										u16 *fil_cnt,//滤波元素号数标记（静态，用作存储）
										float in,//输入
										float *out //输出
										);



void step_filter(float step,float in,float *out);

void fir_arrange_filter(float *arr,u16 len,u8 *fil_cnt,float in,float *arr_out);  //len<=255 len >= 3

void LPF_1(float hz,//截止频率
					float time,//周期
					float in,//输入
					float *out//输出
					);

					
void LPF_1_db(float hz,float time,double in,double *out); //低通滤波，2hz代表0.5秒上升至目标值0.7倍，大约1秒上升到90%

void LPF_I(float raw_a,float raw_b,float time,float in,float *out,float *intera);

float my_deadzone_3(float T,float hz,float x,float ,float zoom,float range_x,float *zoom_adj); 

float LPF_1st(float oldData, float newData, float lpf_factor);

double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na);                   

#endif
