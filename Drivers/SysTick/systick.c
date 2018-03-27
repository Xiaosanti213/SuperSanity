/**
 *
 * @file systick.c
 *
 * 定时器相关设置
 *
 **/
 
 #include "systick.h"
 #include "core_cm3.h"
 #include <stm32f10x_rcc.h>
 
 
 __IO u32 current_time_us = 0;





 /**
 *
 * 名称：systick_init
 *
 * 描述：嘀嗒定时器初始化
 *
 */
void systick_init(void)
{
	if (SysTick_Config(SystemCoreClock / 1000000))
	// 目标是初始化为1us产生一次中断，有待测试
	{
		while(1);
		//捕获出错
	}
	
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;//清0
	// 关闭嘀嗒定时器
}
 
 
 


 

 /**
 *
 * 名称：delay
 *
 * 描述：测试用的延时函数
 *
 */
 void delay(__IO u32 nCount)       
{  
    for(; nCount != 0; nCount--);  
} 








 /**
 *
 * 名称：current_time_count
 *
 * 描述：当前时间计数器初始化
 *
 */
void current_time_count(void)  
{  
    current_time_us++;  
} 






 static void os_tick_init(void);
 

 /************************************************************************************
 * 
 * 名称: os_tick_init-
 *
 * 描述: 操作系统滴答时钟初始化
 *   
 ************************************************************************************/
 
 void os_tick_init(void)
 {
	 //获取系统时钟频率:72 000 000Hz (计数速率)
	 SysTick_Config(72000000uL/1000000uL);
 }
 
 
 
 
 
 
 /************************************************************************************
 * 
 * 名称: delay_us
 *
 * 描述: 操作系统滴答时钟延时函数
 *   
 ************************************************************************************/
 void delay_us(u32 i)
 {
     u32 temp;
     SysTick->LOAD=9*i;          
     SysTick->CTRL=0X01;//开启          
     SysTick->VAL=0;                 
     do
     {
         temp=SysTick->CTRL;            
     }
     while((temp&0x01)&&(!(temp&(1<<16))));      
     SysTick->CTRL=0;  //关闭   
     SysTick->VAL=0;         
 }
 
 
 
 
 
 
 
 





