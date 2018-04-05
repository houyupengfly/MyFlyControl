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
#include "device_spi.h"
#include "stm32f10x.h"
#include "Fc_systream_time.h"
/*============================ MACROS ========================================*/
// Product ID Define
#define PROD_PMW3901             0x49
// Time delay for various operations
#define TIME3901_us_TSWW         45
#define TIME3901_us_TSWR         45
#define TIME3901_us_TSRW_TSRR    20
#define TIME3901_us_TSRAD        35
#define TIME3901_us_TSCLK_NCS_W  35
// Common
#define TIME_us_TBEXIT           1    // should be 500ns
#define TIME_us_TNCS_SCLK        1    // should be 120ns
#define TIME_us_TSCLK_NCS_R      1    // should be 120ns
// Motion read size
#define MOTION_BURST_SIZE        12
#define MOTION_REPORT_SIZE       MOTION_BURST_SIZE
#define MOTION_SINGLE_SIZE       4

/* Bit Define */
#define BIT0 (1 << 0)
#define BIT1 (1 << 1)
#define BIT2 (1 << 2)
#define BIT3 (1 << 3)
#define BIT4 (1 << 4)
#define BIT5 (1 << 5)
#define BIT6 (1 << 6)
#define BIT7 (1 << 7)
// Helper macro
#define BIT_CLEAR(REG,MASK) 	 	 (REG &= ~MASK)
#define BIT_SET(REG,MASK)		     (REG |=  MASK)
#define IS_BIT_CLEAR(REG,MASK) 		((REG & MASK)==0x00)
#define IS_BIT_SET(REG,MASK)   		((REG & MASK)==MASK)

unsigned char ProductID;
bool bStopReadMotion = false;          // Temparory stop reading motion (for Frame Capture)
bool bPower_On = true;                 // first power on flag
bool bRegMapON = false;
bool bFastFrameCapture = false;        // Enable fast frame capture, default should be off to use the default frame capture mode
bool bEnableJitterSuppression = true;  // Enable jitter suppression
s16 x,y;
/*============================ TYPES =========================================*/

union n16Bit
{
    unsigned char c[2];    // c[0] = LO byte, c[1] = HI byte
    unsigned short i;
    signed short si;
};

// Union for 32-bit value
union n32Bit
{
    unsigned char c[4];    // c[0] = bit0-7 byte, c[1] = bit8-15, c[2] = bit16-23, c[3] = bit24-31
    union n16Bit n16[2];   // n16[0] = bit0-15, n16[1] = bit16-31
    unsigned long l;
    signed long sl;
};

// Structure for motion data
union
{
    unsigned char d[MOTION_REPORT_SIZE];    // set to maximum bytes which is burst read
    struct   // Burst reading from register 0x50
    {
        unsigned char motion;        // BYTE 0
        unsigned char observation;   // BYTE 1
        union n16Bit deltaX;         // BYTE 2 & 3
        union n16Bit deltaY;         // BYTE 4 & 5
        unsigned char squal;         // BYTE 6
        unsigned char rd_sum;        // BYTE 7
        unsigned char rd_max;        // BYTE 8
        unsigned char rd_min;        // BYTE 9
        union n16Bit shutter;        // BYTE 10 & 11	// this need to be reversed after reading in byte form, as the first read byte is upper
    } burst;
} MotionData;

/*============================ GLOBAL VARIABLES ==============================*/
/*============================ STATIC VARIABLES ==============================*/


/*============================ FUNCTION ======================================*/
void Register_Write(unsigned char address, unsigned value)
{
    SPI2_CSN_L();
    //  send in the address and value via SPI:
    SPI2_RW(address | 0x80);
    SPI2_RW(value);
    SPI2_CSN_H();
}

// Register Read function
unsigned int Register_Read(unsigned char address)
{
    u8 inByte = 0;                              // incoming byte from the SPI
//  unsigned int result = 0;                      // result to return
    SPI2_CSN_L();
    //  send in the address via SPI:
    SPI2_RW(address & 0x7f);
    Delay_us(35);
    inByte = SPI2_RW(0xFF);                 // byte to read:
    SPI2_CSN_H();
//  result = inByte;
    return(inByte);
}

void RegWrite_tsww(unsigned char address, unsigned value)
{
    Register_Write(address, value);
    Delay_us(TIME3901_us_TSWW);
}

// Function that perform register read and wait for Tsrw/Tsrr microseconds
unsigned char RegRead_tsrw_tsrr(unsigned char address)
{
    unsigned char v = Register_Read(address);
    Delay_us(TIME3901_us_TSRW_TSRR);
    return v;
}

void InitRegisters(void)
{
    unsigned char v, i;
    unsigned char c1, c2;
    RegWrite_tsww(0x7F, 0x00);
    RegWrite_tsww(0x55, 0x01);
    RegWrite_tsww(0x50, 0x07);
    RegWrite_tsww(0x7f, 0x0e);
    // Read back from register 0x47 to confirm value is 0x08
    i = 0;

    while(i < 3)
    {
        RegWrite_tsww(0x43, 0x10);
        v = RegRead_tsrw_tsrr(0x47);

        if(v == 0x08)
            break;
        else
        {
            // Not successful, retry for 3 times, if still not, exit initialization routine
            i++;

            if(i == 3)
            {
                return;
            }
        }
    }

    v = RegRead_tsrw_tsrr(0x67);

    if(IS_BIT_SET(v, BIT7))
    {
        RegWrite_tsww(0x48, 0x04);
    }
    else
    {
        RegWrite_tsww(0x48, 0x02);
    }

    RegWrite_tsww(0x7F, 0x00);
    RegWrite_tsww(0x51, 0x7b);
    RegWrite_tsww(0x50, 0x00);
    RegWrite_tsww(0x55, 0x00);
    RegWrite_tsww(0x7F, 0x0e);
    v = RegRead_tsrw_tsrr(0x73);

    if(v == 0)
    {
        c1 = RegRead_tsrw_tsrr(0x70);

        if(c1 <= 28)
            c1 = c1 + 14;
        else
            c1 = c1 + 11;

        if(c1 > 0x3F)
            c1 = 0x3F;

        c2 = RegRead_tsrw_tsrr(0x71);
        c2 = ((unsigned short)c2 * 45)/100;
        RegWrite_tsww(0x7f, 0x00);
        RegWrite_tsww(0x61, 0xAD);
        RegWrite_tsww(0x51, 0x70);
        RegWrite_tsww(0x7f, 0x0e);
        RegWrite_tsww(0x70, c1);
        RegWrite_tsww(0x71, c2);
    }

    RegWrite_tsww(0x7F, 0x00);
    RegWrite_tsww(0x61, 0xAD);
    RegWrite_tsww(0x7F, 0x03);
    RegWrite_tsww(0x40, 0x00);
    RegWrite_tsww(0x7F, 0x05);
    RegWrite_tsww(0x41, 0xB3);
    RegWrite_tsww(0x43, 0xF1);
    RegWrite_tsww(0x45, 0x14);
    RegWrite_tsww(0x5B, 0x32);
    RegWrite_tsww(0x5F, 0x34);
    RegWrite_tsww(0x7B, 0x08);
    RegWrite_tsww(0x7F, 0x06);
    RegWrite_tsww(0x44, 0x1B);
    RegWrite_tsww(0x40, 0xBF);
    RegWrite_tsww(0x4E, 0x3F);
    RegWrite_tsww(0x7F, 0x08);
    RegWrite_tsww(0x65, 0x20);
    RegWrite_tsww(0x6A, 0x18);
    RegWrite_tsww(0x7F, 0x09);
    RegWrite_tsww(0x4F, 0xAF);
    RegWrite_tsww(0x5F, 0x40);
    RegWrite_tsww(0x48, 0x80);
    RegWrite_tsww(0x49, 0x80);
    RegWrite_tsww(0x57, 0x77);
    RegWrite_tsww(0x60, 0x78);
    RegWrite_tsww(0x61, 0x78);
    RegWrite_tsww(0x62, 0x08);
    RegWrite_tsww(0x63, 0x50);
    RegWrite_tsww(0x7F, 0x0A);
    RegWrite_tsww(0x45, 0x60);
    RegWrite_tsww(0x7F, 0x00);
    RegWrite_tsww(0x4D, 0x11);
    RegWrite_tsww(0x55, 0x80);
    RegWrite_tsww(0x74, 0x21);
    RegWrite_tsww(0x75, 0x1F);
    RegWrite_tsww(0x4A, 0x78);
    RegWrite_tsww(0x4B, 0x78);
    RegWrite_tsww(0x44, 0x08);
    RegWrite_tsww(0x45, 0x50);
    RegWrite_tsww(0x64, 0xFF);
    RegWrite_tsww(0x65, 0x1F);
    RegWrite_tsww(0x7F, 0x14);
    RegWrite_tsww(0x65, 0x67);
    RegWrite_tsww(0x66, 0x08);
    RegWrite_tsww(0x63, 0x70);
    RegWrite_tsww(0x7F, 0x15);
    RegWrite_tsww(0x48, 0x48);
    RegWrite_tsww(0x7F, 0x07);
    RegWrite_tsww(0x41, 0x0D);
    RegWrite_tsww(0x43, 0x14);
    RegWrite_tsww(0x4B, 0x0E);
    RegWrite_tsww(0x45, 0x0F);
    RegWrite_tsww(0x44, 0x42);
    RegWrite_tsww(0x4C, 0x80);
    RegWrite_tsww(0x7F, 0x10);
    RegWrite_tsww(0x5B, 0x02);
    RegWrite_tsww(0x7F, 0x07);
    RegWrite_tsww(0x40, 0x41);
    RegWrite_tsww(0x70, 0x00);
    Delay_ms(10); // delay 10ms
    RegWrite_tsww(0x32, 0x44);
    RegWrite_tsww(0x7F, 0x07);
    RegWrite_tsww(0x40, 0x40);
    RegWrite_tsww(0x7F, 0x06);
    RegWrite_tsww(0x62, 0xF0);
    RegWrite_tsww(0x63, 0x00);
    RegWrite_tsww(0x7F, 0x0D);
    RegWrite_tsww(0x48, 0xC0);
    RegWrite_tsww(0x6F, 0xD5);
    RegWrite_tsww(0x7F, 0x00);
    RegWrite_tsww(0x5B, 0xA0);
    RegWrite_tsww(0x4E, 0xA8);
    RegWrite_tsww(0x5A, 0x50);
    RegWrite_tsww(0x40, 0x80);
}

void Full_Reset()
{
    // Full Reset and Enable back motion read
    bStopReadMotion = false;  // Remove temporary block on motion read

    if(ProductID == PROD_PMW3901)
        InitRegisters();
}

// Function that perform register
void PMW3901_Init(void)
{
    SPI2_CSN_L();
    // Drive NRESET high
    SPI2_CSN_H();
    Delay_ms(100);
    // Write 0x5a to Power_Up_Reset register (0x3a)
    Register_Write(0x3a, 0x5a);
    Delay_ms(100);  // delay 1ms
    // Read the product id to determine which sensor is connected
    ProductID = Register_Read(0x00);

    // Read register 0x02 to 0x06
    for(int i = 0; i < 5; i++)
    {
        Register_Read(0x02+i);
        Delay_us(TIME3901_us_TSRW_TSRR);
    }

    Full_Reset();
    bPower_On = false;
}

void SwapBytes(unsigned char *c1, unsigned char *c2)
{
    char c = *c2;
    *c2 = *c1;
    *c1 = c;
}
void ReadMotion(void)
{
    int i = 0;
    SPI2_CSN_L();
    Delay_us(TIME_us_TNCS_SCLK);                         // 2.5 micro second  measured
    SPI2_RW(0x16);
    Delay_us(TIME3901_us_TSRAD);

    // Read all motion data
    for(i = 0; i < MOTION_BURST_SIZE; i++)
    {
        MotionData.d[i] = SPI2_RW(0x00);
    }

    x=MotionData.d[2]|MotionData.d[3]<<8;
    y=MotionData.d[4]|MotionData.d[5]<<8;
    SPI2_CSN_H();
    Delay_us(TIME_us_TBEXIT);
    // As Arduino is little-endian, we need to swap the shutter bytes
    SwapBytes(&(MotionData.burst.shutter.c[0]),&(MotionData.burst.shutter.c[1]));

    // Check for jitter suppression
    if(bEnableJitterSuppression)
    {   // Squal less than 25        and    Shutter hi byte is 0x1f
        if((MotionData.burst.squal < 25) && (MotionData.burst.shutter.i >= 0x1f00))
        {
            MotionData.burst.deltaX.si = MotionData.burst.deltaY.si = 0;
        }
    }
}

