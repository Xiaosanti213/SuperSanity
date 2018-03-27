/**
 *
 * @file systick.c
 *
 * ��ʱ���������
 *
 **/
 
 #include "systick.h"
 #include "core_cm3.h"
 #include <stm32f10x_rcc.h>
 
 
 __IO u32 current_time_us = 0;





 /**
 *
 * ���ƣ�systick_init
 *
 * ��������શ�ʱ����ʼ��
 *
 */
void systick_init(void)
{
	if (SysTick_Config(SystemCoreClock / 1000000))
	// Ŀ���ǳ�ʼ��Ϊ1us����һ���жϣ��д�����
	{
		while(1);
		//�������
	}
	
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;//��0
	// �ر���શ�ʱ��
}
 
 
 


 

 /**
 *
 * ���ƣ�delay
 *
 * �����������õ���ʱ����
 *
 */
 void delay(__IO u32 nCount)       
{  
    for(; nCount != 0; nCount--);  
} 








 /**
 *
 * ���ƣ�current_time_count
 *
 * ��������ǰʱ���������ʼ��
 *
 */
void current_time_count(void)  
{  
    current_time_us++;  
} 






 static void os_tick_init(void);
 

 /************************************************************************************
 * 
 * ����: os_tick_init-
 *
 * ����: ����ϵͳ�δ�ʱ�ӳ�ʼ��
 *   
 ************************************************************************************/
 
 void os_tick_init(void)
 {
	 //��ȡϵͳʱ��Ƶ��:72 000 000Hz (��������)
	 SysTick_Config(72000000uL/1000000uL);
 }
 
 
 
 
 
 
 /************************************************************************************
 * 
 * ����: delay_us
 *
 * ����: ����ϵͳ�δ�ʱ����ʱ����
 *   
 ************************************************************************************/
 void delay_us(u32 i)
 {
     u32 temp;
     SysTick->LOAD=9*i;          
     SysTick->CTRL=0X01;//����          
     SysTick->VAL=0;                 
     do
     {
         temp=SysTick->CTRL;            
     }
     while((temp&0x01)&&(!(temp&(1<<16))));      
     SysTick->CTRL=0;  //�ر�   
     SysTick->VAL=0;         
 }
 
 
 
 
 
 
 
 





