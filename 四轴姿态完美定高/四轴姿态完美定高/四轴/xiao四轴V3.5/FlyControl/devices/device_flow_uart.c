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
#include "device_flow.h"
#include "device_Uart.h"

#include "Fc_data_Struct.h"
/*============================ MACROS ========================================*/

/*============================ TYPES =========================================*/

/*============================ GLOBAL VARIABLES ==============================*/
extern FlyControlDataStruct g_FlyControlDataStruct;
u8 Flow_SSI,Flow_SSI_CNT,Flow_Err;
/*============================ STATIC VARIABLES ==============================*/

/*============================ FUNCTION ======================================*/
/*----------------------------------------------------------
 + ʵ�ֹ��ܣ����ݷ���
 + ���ò��������յ��ĵ��ֽ�����
----------------------------------------------------------*/
void Fc_Flow_Init(void)
{
    Fc_Usart2_Init(115200);

}
void Flow_Receive_Anl(u8 *data_buf,u8 num)
{
    u8 sum = 0;
    /* ����У���ۼӺ� */
    for(u8 i=0; i<(num-1); i++)
        sum += *(data_buf+i);
    /* �ж�У���ۼӺ� */
    if(!(sum==*(data_buf+num-1)))		return;
    /* �ж�֡ͷ */
    if(!(*(data_buf)==0xAA && *(data_buf+1)==0XAA))		return;
    /* �жϹ����֣���Ҫ��� */

    /* �ش�У���ۼӺ� */
    if(*(data_buf+2)==0Xf1)
    {
        g_FlyControlDataStruct.AppOptflowStruct.x_offest   = ( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        g_FlyControlDataStruct.AppOptflowStruct.y_offest   = ( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        g_FlyControlDataStruct.AppHightCtrl.UltHight       = ( (vs16)(*(data_buf+10)<<8)|*(data_buf+11));
        g_FlyControlDataStruct.AppOptflowStruct.NSS        = ( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        Flow_SSI_CNT++;
    }
}

/*----------------------------------------------------------
 + ʵ�ֹ��ܣ����ݽ��ղ�����
 + ���ò��������յ��ĵ��ֽ�����
----------------------------------------------------------*/
void Flow_Data_Receive_Prepare(u8 data)
{
    /* �ֲ���̬���������ջ��� */
    static u8 RxBuffer[50];
    /* ���ݳ��� *//* ���������±� */
    static u8 _data_len = 0,_data_cnt = 0;
    /* ����״̬ */
    static u8 state = 0;

    /* ֡ͷ1 */
    if(state==0&&data==0XAA)
    {
        state=1;
        RxBuffer[0]=data;
    }
    /* ֡ͷ2 */
    else if(state==1&&data==0XAA)
    {
        state=2;
        RxBuffer[1]=data;
    }
    /* ������ */
    else if(state==2&&data<0XF2)
    {
        state=3;
        RxBuffer[2]=data;
    }
    /* ���� */
    else if(state==3&&data<50)
    {
        state = 4;
        RxBuffer[3]=data;
        _data_len = data;
        _data_cnt = 0;
    }
    /* ���������� */
    else if(state==4&&_data_len>0)
    {
        _data_len--;
        RxBuffer[4+_data_cnt++]=data;
        if(_data_len==0)
            state = 5;
    }
    /* У���ۼӺ� */
    else if(state==5)
    {
        state = 0;
        RxBuffer[4+_data_cnt]=data;
        Flow_Receive_Anl(RxBuffer,_data_cnt+5);
    }
    /* ���д������µȴ�����֡ͷ */
    else
        state = 0;
}

