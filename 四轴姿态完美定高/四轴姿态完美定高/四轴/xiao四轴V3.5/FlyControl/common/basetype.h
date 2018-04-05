
#ifndef BASETYPE_H
#define BASETYPE_H

#define TRUE  1 
#define FALSE 0


#define TRUE  1 
#define FALSE 0

#define true 1
#define false 0

typedef int bool;

typedef unsigned char 	u8;
typedef unsigned short 	u16;
typedef unsigned int  	u32;

typedef signed char 	s8;
typedef signed short    s16;
typedef signed int      s32;

typedef volatile unsigned char 	 vu8;
typedef volatile unsigned short  vu16;
typedef volatile unsigned int    vu32;

typedef volatile signed char 	 vs8;
typedef volatile signed short  vs16;
typedef volatile signed int    vs32;

#define STATIC 
//#define STATIC static 

typedef enum
{
  FRAME_READY = 0,
  FRAME_HALF  = 1,
  FRAME_EMPTY = 2,
  FRAME_ERROR = 3
}SearchFrameStatusEnum;

#endif

