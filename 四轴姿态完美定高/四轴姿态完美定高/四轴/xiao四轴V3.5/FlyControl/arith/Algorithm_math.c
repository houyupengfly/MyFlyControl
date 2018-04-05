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
#include "Algorithm_math.h"
/*============================ MACROS ========================================*/
#define ONE_PI   (3.14159265)
#define TWO_PI   (2.0 * 3.14159265)
#define ANGLE_UNIT (TWO_PI/10.0)
/*============================ TYPES =========================================*/

/*============================ GLOBAL VARIABLES ==============================*/
/*============================ STATIC VARIABLES ==============================*/

/*============================ FUNCTION ======================================*/

/*=============================================================================
 + 实现功能：
 + 调用参数功能：
==============================================================================*/
float Q_rsqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;
 
	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                      
	i  = 0x5f3759df - ( i >> 1 );               
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration （第一次牛顿迭代）
	return y;
} 
/*=============================================================================
 + 实现功能：
 + 调用参数功能：
==============================================================================*/
void array_astrict(s16 *array,s16 lower,s16 upper)
{
   s16 length = sizeof(array); 
   for(u16 i=0;i<length;i++)
   {
     if(*(array+i)<lower)  *(array+i) = lower;
     else if(*(array+i)>upper)  *(array+i) = upper;
   } 
}

/*=============================================================================
 + 实现功能：
 + 调用参数功能：
==============================================================================*/
void array_assign(u16 *array,s16 value,u16 length)
{
   for(u16 i=0;i<length;i++)
   {
     *(array+i) = value;
   } 
}

/*=============================================================================
 + 实现功能：
 + 调用参数功能：
==============================================================================*/
float data_limit(float data,float toplimit,float lowerlimit)
{
  if(data > toplimit)  data = toplimit;
  else if(data < lowerlimit) data = lowerlimit;
	return data;
}


/*=============================================================================
 + 实现功能：
 + 调用参数功能：
==============================================================================*/
float VariableParameter(float error)
{
	float  result = 0;
	
	if(error < 0)
	{
	   error = -error;
	}
  if(error >0.6f)
	{
	   error = 0.6f;
	}
	result = 1 - 1.667 * error;
	if(result < 0)
	{
	   result = 0;
	}
	return result;
}
/*=============================================================================
 + 实现功能：
 + 调用参数功能：
==============================================================================*/
double Rad(double angle)
{
    return (angle * M_PI / 180.0);
}
/*=============================================================================
 + 实现功能：
 + 调用参数功能：
==============================================================================*/
double Degree(double rad)
{
    return (rad / M_PI * 180.0);
}

/*=============================================================================
 + 实现功能：
 + 调用参数功能：
==============================================================================*/
double constrain(double inputvalue, double limitmin, double limitmax)
{
    if (inputvalue>limitmax) {
        inputvalue=limitmax;
    }
    else
    if (inputvalue<limitmin) {
        inputvalue=limitmin;
    }
		return inputvalue;
}

/*=============================================================================
 + 实现功能：
 + 调用参数功能：
==============================================================================*/
void applyDeadband(double value,double deadband)
{
  if((value  < deadband) && value > -deadband) 
  { value = 0;} 
  else if(value > 0)
        {value -= deadband;
        } else if(value < 0)
                {
                 value += deadband;
                }
}
/*=============================================================================
 + 实现功能：
 + 调用参数功能：
==============================================================================*/
double mx_sin(double rad)
{   
	double sine;
	if (rad < 0)
		sine = rad*(1.27323954f + 0.405284735f * rad);
	else
		sine = rad * (1.27323954f - 0.405284735f * rad);
	if (sine < 0)
		sine = sine*(-0.225f * (sine + 1) + 1);
	else
		sine = sine * (0.225f *( sine - 1) + 1);
	return sine;
}
/*=============================================================================
 + 实现功能：
 + 调用参数功能：
==============================================================================*/
double my_sin(double rad)
{
	s8 flag = 1;

	if (rad >= ONE_PI)
	{
		rad -= ONE_PI;
		flag = -1;
	}
	return mx_sin(rad) * flag;
}
/*=============================================================================
 + 实现功能：
 + 调用参数功能：
==============================================================================*/
float my_cos(double rad)
{
	s8 flag = 1;
	rad += ONE_PI/2.0;

	if (rad >= ONE_PI)
	{
		flag = -1;
		rad -= ONE_PI;
	}
	return my_sin(rad)*flag;
}
/*=============================================================================
 + 实现功能：
 + 调用参数功能：
==============================================================================*/
float my_deathzoom(float x,float zoom)
{
	float t;
	if(x>0)
	{
		t = x - zoom;
		if(t<0)
		{
			t = 0;
		}
	}
	else
	{
		t = x + zoom;
		if(t>0)
		{
			t = 0;
		}
	}
  return (t);
}
/*=============================================================================
 + 实现功能：
 + 调用参数功能：
==============================================================================*/
float my_deathzoom_2(float x,float zoom)
{
	float t;
	
	if( x> -zoom && x < zoom )
	{
		t = 0;
	}
	else
	{
		t = x;
	}
  return (t);
}
/*=============================================================================
 + 实现功能：
 + 调用参数功能：
==============================================================================*/
float my_abs(float f)
{
	if (f >= 0.0f)
	{
		return f;
	}
	return -f;
}

float fast_atan2(float y, float x) 
{
	float x_abs, y_abs, z;
	float alpha, angle, base_angle;
	int index;

	/* don't divide by zero! */
	if ((y == 0.0f) || (x == 0.0f))//if ((y == 0.0f) && (x == 0.0f))
		angle = 0.0f;
	else 
	{
		/* normalize to +/- 45 degree range */
		y_abs = my_abs(y);
		x_abs = my_abs(x);
		//z = (y_abs < x_abs ? y_abs / x_abs : x_abs / y_abs);
		if (y_abs < x_abs)
			z = y_abs / x_abs;
		else
			z = x_abs / y_abs;
		/* when ratio approaches the table resolution, the angle is */
		/*      best approximated with the argument itself...       */
		if (z < TAN_MAP_RES)
			base_angle = z;
		else 
		{
			/* find index and interpolation value */
			alpha = z * (float) TAN_MAP_SIZE - .5f;
			index = (int) alpha;
			alpha -= (float) index;
			/* determine base angle based on quadrant and */
			/* add or subtract table value from base angle based on quadrant */
			base_angle = fast_atan_table[index];
			base_angle += (fast_atan_table[index + 1] - fast_atan_table[index]) * alpha;
		}

		if (x_abs > y_abs) 
		{        /* -45 -> 45 or 135 -> 225 */
			if (x >= 0.0f) 
			{           /* -45 -> 45 */
				if (y >= 0.0f)
					angle = base_angle;   /* 0 -> 45, angle OK */
				else
					angle = -base_angle;  /* -45 -> 0, angle = -angle */
			} 
			else
			{                  /* 135 -> 180 or 180 -> -135 */
				angle = 3.14159265358979323846;

				if (y >= 0.0f)
					angle -= base_angle;  /* 135 -> 180, angle = 180 - angle */
				else
					angle = base_angle - angle;   /* 180 -> -135, angle = angle - 180 */
			}
		} 
		else 
		{                    /* 45 -> 135 or -135 -> -45 */
			if (y >= 0.0f) 
			{           /* 45 -> 135 */
				angle = 1.57079632679489661923;

				if (x >= 0.0f)
					angle -= base_angle;  /* 45 -> 90, angle = 90 - angle */
				else
					angle += base_angle;  /* 90 -> 135, angle = 90 + angle */
			} 
			else
			{                  /* -135 -> -45 */
				angle = -1.57079632679489661923;

				if (x >= 0.0f)
					angle += base_angle;  /* -90 -> -45, angle = -90 + angle */
				else
					angle -= base_angle;  /* -135 -> -90, angle = -90 - angle */
			}
		}
	}
}