#ifndef __Algorithm_filter_H
#define	__Algorithm_filter_H

#include "basetype.h"
typedef struct
{
	u8 cnt;

	s32 lst_pow_sum;
	
	s32 now_out;
	s32 lst_out;
	s32 now_velocity_xdt;
} _steepest_st;
double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na);
float LPF_1st(float oldData, float newData, float lpf_factor);


typedef struct
{
	float lpf_1;

	float out;
}_lf_t;
//float Moving_Average(u8 item,u8 width_num,float in);
void Moving_Average(float in,float moavarray[],u16 len ,u16 fil_cnt[2],float *out);
float Moving_Median(u8 item,u8 width_num,float in);
float kalmanUpdate(const float gyro_m,const float incAngle);
void steepest_descend(s32 arr[],u8 len,_steepest_st *steepest,u8 step_num,s32 in);
void limit_filter(float T,float hz,_lf_t *data,float in);
#endif
