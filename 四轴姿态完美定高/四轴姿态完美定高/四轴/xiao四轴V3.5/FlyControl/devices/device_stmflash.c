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
#include "device_stmflash.h"
/*============================ MACROS ========================================*/

/*============================ TYPES =========================================*/

/*============================ GLOBAL VARIABLES ==============================*/
/*============================ STATIC VARIABLES ==============================*/

/*============================ FUNCTION ======================================*/
/*=============================================================================
+ ʵ�ֹ��ܣ�
+ ���ò������ܣ�
==============================================================================*/
u16 STMFLASH_ReadHalfWord(u32 faddr)
{
    return *(vu16*)faddr;
}

/*=============================================================================
+ ʵ�ֹ��ܣ�
+ ���ò������ܣ�
==============================================================================*/
void STMFLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)
{
    u16 i;
    for(i=0; i<NumToWrite; i++)
    {
        FLASH_ProgramHalfWord(WriteAddr,pBuffer[i]);
        WriteAddr+=2;//��ַ����2.
    }
}

/*=============================================================================
+ ʵ�ֹ��ܣ�
+ ���ò������ܣ�
==============================================================================*/

void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)
{
    u16 STMFLASH_BUF[STM_SECTOR_SIZE/2];//�����2K�ֽ�
    u32 secpos;	   //������ַ
    u16 secoff;	   //������ƫ�Ƶ�ַ(16λ�ּ���)
    u16 secremain; //������ʣ���ַ(16λ�ּ���)
    u16 i;
    u32 offaddr;   //ȥ��0X08000000��ĵ�ַ
    if(WriteAddr<STM32_FLASH_BASE||(WriteAddr>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))return;//�Ƿ���ַ
    FLASH_Unlock();						//����
    offaddr=WriteAddr;		//ʵ��ƫ�Ƶ�ַ.
    secpos=offaddr/STM_SECTOR_SIZE;			//������ַ  0~127 for STM32F103RBT6
    secoff=(offaddr%STM_SECTOR_SIZE)/2;		//�������ڵ�ƫ��(2���ֽ�Ϊ������λ.)
    secremain=STM_SECTOR_SIZE/2-secoff;		//����ʣ��ռ��С
    if(NumToWrite<=secremain)secremain=NumToWrite;//�����ڸ�������Χ
    while(1)
    {
        STMFLASH_Read(secpos*STM_SECTOR_SIZE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//������������������
        for(i=0; i<secremain; i++) //У������
        {
            if(STMFLASH_BUF[secoff+i]!=0XFFFF)break;//��Ҫ����
        }
        if(i<secremain)//��Ҫ����
        {
            FLASH_ErasePage(secpos*STM_SECTOR_SIZE);//�����������
            for(i=0; i<secremain; i++) //����
            {
                STMFLASH_BUF[i+secoff]=pBuffer[i];
            }
            STMFLASH_Write_NoCheck(secpos*STM_SECTOR_SIZE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//д����������
        } else STMFLASH_Write_NoCheck(WriteAddr,pBuffer,secremain);//д�Ѿ������˵�,ֱ��д������ʣ������.
        if(NumToWrite==secremain)break;//д�������
        else//д��δ����
        {
            secpos++;				//������ַ��1
            secoff=0;				//ƫ��λ��Ϊ0
            pBuffer+=secremain;  	//ָ��ƫ��
            WriteAddr+=secremain;	//д��ַƫ��
            NumToWrite-=secremain;	//�ֽ�(16λ)���ݼ�
            if(NumToWrite>(STM_SECTOR_SIZE/2))secremain=STM_SECTOR_SIZE/2;//��һ����������д����
            else secremain=NumToWrite;//��һ����������д����
        }
    }

    FLASH_Lock();//����
}

/*=============================================================================
+ ʵ�ֹ��ܣ�
+ ���ò������ܣ�
==============================================================================*/

void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead)
{
    u16 i;
    for(i=0; i<NumToRead; i++)
    {
        pBuffer[i]=STMFLASH_ReadHalfWord(ReadAddr);//��ȡ2���ֽ�.
        ReadAddr+=2;//ƫ��2���ֽ�.
    }

}












