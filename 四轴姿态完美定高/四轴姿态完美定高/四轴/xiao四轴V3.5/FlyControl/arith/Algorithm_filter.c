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
#include "Algorithm_filter.h"
#include "Algorithm_math.h"
#include "mymath.h"
/*============================ MACROS ========================================*/
#define my_pow(a) ((a)*(a))
/*============================ TYPES =========================================*/

/*============================ GLOBAL VARIABLES ==============================*/
/*============================ STATIC VARIABLES ==============================*/

/*============================ FUNCTION ======================================*/

/*=============================================================================
 + 实现功能：
 + 调用参数功能：
==============================================================================*/
double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na)
{
  double z1,z2;
  short i;
  double OutData;
  
  for(i=nb-1; i>0; i--)
  {
    x[i]=x[i-1];
  }
  
  x[0] = InData;
  
  for(z1=0,i=0; i<nb; i++)
  {
    z1 += x[i]*b[i];
  }
  
  for(i=na-1; i>0; i--)
  {
    y[i]=y[i-1];
  }
  
  for(z2=0,i=1; i<na; i++)
  {
    z2 += y[i]*a[i];
  }
  
  y[0] = z1 - z2; 
  OutData = y[0];
    
  return OutData;
}

/*=============================================================================
 + 实现功能：
 + 调用参数功能：
==============================================================================*/
double KalmanFilter(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_mid ;//= x_last
   static double x_now;
   static double p_mid ;
   static double p_now;
   static double kg;        
   static double x_last,p_last;//原来是在形式参数里的
   x_mid=x_last;          //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q;        //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
   kg=p_mid/(p_mid+R);    //kg为kalman filter，R为噪声
   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
                
   p_now=(1-kg)*p_mid;   //最优值对应的covariance       
   p_last = p_now;       //更新covariance值
   x_last = x_now;       //更新系统状态值
   return x_now;                
}
/*=============================================================================
 + 实现功能：
 + 调用参数功能：
==============================================================================*/
float LPF_1st(float oldData, float newData, float lpf_factor)
{
	return oldData * (1 - lpf_factor) + newData * lpf_factor;
}

void steepest_descend(s32 arr[],u8 len,_steepest_st *steepest,u8 step_num,s32 in)
{	
	u8 updw = 1;//0 dw,1up
	s16 i;
	u8 step_cnt=0;
	u8 step_slope_factor=1;
	u8 on = 1;
	s8 pn = 1;
	//float last = 0;
	float step = 0;
	s32 start_point = 0;
	s32 pow_sum = 0;
	
	steepest->lst_out = steepest->now_out;
	
	if( ++(steepest->cnt) >= len )	
	{
		(steepest->cnt) = 0; //now
	}
	
	//last = arr[ (steepest->cnt) ];
	
	arr[ (steepest->cnt) ] = in;
	
	step = (float)(in - steepest->lst_out)/step_num ;//梯度
	
	if(my_abs(step)<1)//整形数据<1的有效判定
	{
		if(my_abs(step)*step_num<2)
		{
			step = 0;
		}
		else
		{
		  step = (step > 0) ? 1 : -1;
		}
	}
	
	start_point = steepest->lst_out;
	do
	{
		//start_point = steepest->lst_out;
		for(i=0;i<len;i++)
		{
			pow_sum += my_pow(arr[i] - start_point );// /step_num;//除法减小比例**
		}
			
		if(pow_sum - steepest->lst_pow_sum > 0)
		{		
			if(updw==0)
			{
				on = 0;
			}
			updw = 1;//上升了
			pn = (pn == 1 )? -1:1;

		}
		else
		{
			updw = 0; //正在下降
 			if(step_slope_factor<step_num)
 			{
 				step_slope_factor++;
 			}
		}
			
		steepest->lst_pow_sum = pow_sum;		
		pow_sum = 0;
		start_point += pn *step;//调整
		
		if(++step_cnt > step_num)//限制计算次数
		{
			on = 0;
		}
			//////
			if(step_slope_factor>=2)//限制下降次数1次，节省时间，但会增大滞后，若cpu时间充裕可不用。
			{
				on = 0;

			}
			//////
		
	}
	while(on==1);
	
	steepest->now_out = start_point ;//0.5f *(start_point + steepest->lst_out);//
	
	steepest->now_velocity_xdt = steepest->now_out - steepest->lst_out;
}

void LPF_1(float hz,float time,float in,float *out)  
{
	*out += ( 1 / ( 1 + 1 / ( hz *6.28f *time ) ) ) *( in - *out );
}
void limit_filter(float T,float hz,_lf_t *data,float in)
{
	float abs_t;
	LPF_1(hz,T,in,&(data->lpf_1)); 
	abs_t = my_abs(data->lpf_1);
	data->out = LIMIT(in,-abs_t,abs_t);
}
