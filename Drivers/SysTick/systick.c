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
	if (SysTick_Config(SystemCoreClock / 1000))
	// 目标是初始化为1ms产生一次中断，有待测试
	{
		while(1);
		//捕获出错
	}
	
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;//置位
	// 开启嘀嗒定时器
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



 
 
 
 
 
 
 /************************************************************************************
 * 
 * 名称: delay_4us
 *
 * 描述: 模拟I2C函数，理论上是4.12us
 *   
 ************************************************************************************/
 void delay_4us(void)
 {
     delay(15);
 }
 
 
 
 
 /************************************************************************************
 * 
 * 名称: delay_1us
 *
 * 描述: 模拟I2C函数，理论上是1.02us
 *   
 ************************************************************************************/
 void delay_1us(void)
 {
   //delay(6);
	 u8 i;
	 /*　
	 	下面的时间是通过安富莱AX-Pro逻辑分析仪测试得到的。
		CPU主频72MHz时，在内部Flash运行, MDK工程不优化
		循环次数为10时，SCL频率 = 205KHz 
		循环次数为7时，SCL频率 = 347KHz， SCL高电平时间1.5us，SCL低电平时间2.87us 
	 	循环次数为5时，SCL频率 = 421KHz， SCL高电平时间1.25us，SCL低电平时间2.375us 
        
    IAR工程编译效率高，不能设置为7
	 */
	 for (i = 0; i < 10; i++);
 } 
 
 
 





