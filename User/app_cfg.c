/**
 * @file app_cfg.c
 *
 * sanity-v1711 操作系统相关配置
 *
 *
 */
 
 
 
 #include <stm32f10x_rcc.h>
 #include <stm32f10x.h>
 #include <stdio.h>
 
 #include "app_cfg.h"
 #include "core_cm3.h"
 #include "misc.h"
 #include "ucos_ii.h" 
 #include "sensors.h"
 
 
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
	 SysTick_Config(72000000uL/OS_TICKS_PER_SEC);
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
     SysTick->CTRL=0X01;          
     SysTick->VAL=0;                 
     do
     {
         temp=SysTick->CTRL;            
     }
     while((temp&0x01)&&(!(temp&(1<<16))));      
     SysTick->CTRL=0;     
     SysTick->VAL=0;         
 }
 
 
 
 
 
 
 
 
 
 /************************************************************************************
 * 
 * 名称: controller_init
 *
 * 描述: 飞控初始化
 *   
 ************************************************************************************/ 
 void controller_init(void)
 {
	 OS_CPU_SR cpu_sr=0;
	 OS_ENTER_CRITICAL();
	 sensors_init();
	 os_tick_init();
	 OS_EXIT_CRITICAL();
 }
 
 
 
 
 
 
 
 
 
 
 
 
 /************************************************************************************
 * 
 * 名称: App_TCBInitHook
 *
 * 描述: 任务控制块初始化回调函数
 *   
 ************************************************************************************/ 
 void App_TCBInitHook(OS_TCB *ptcb)
 {
 
 }
 
 
 
 /************************************************************************************
 * 
 * 名称: App_TaskCreateHook
 *
 * 描述: 任务创建回调函数
 *   
 ************************************************************************************/
 void App_TaskCreateHook(OS_TCB *ptcb)
 {
 
 }
 
 
 
 /************************************************************************************
 * 
 * 名称: App_TaskDelHook
 *
 * 描述: 任务删除回调函数
 *   
 ************************************************************************************/
 void App_TaskDelHook(OS_TCB *ptcb)
 {
 
 }
 
 
 
 
 /************************************************************************************
 * 
 * 名称: App_taskIdleHook
 *
 * 描述: 任务空闲回调函数
 *   
 ************************************************************************************/
 void App_TaskIdleHook(void)
 {
 
 }
 
 
 
 
 /************************************************************************************
 * 
 * 名称: App_TaskReturnHook
 *
 * 描述: 任务返回回调函数
 *   
 ************************************************************************************/
 void App_TaskReturnHook(OS_TCB  *ptcb)
 {
 
 }
 
 
 
 
 
 /************************************************************************************
 * 
 * 名称: App_TaskStatHook
 *
 * 描述: 统计任务回调函数
 *   
 ************************************************************************************/
 void App_TaskStatHook(void)
 {
 
 }
 
 
 
 
 /************************************************************************************
 * 
 * 名称: App_TaskSwHook
 *
 * 描述: 任务切换回调函数
 *   
 ************************************************************************************/
 void App_TaskSwHook(void)
 {
 
 }
 
 
 
 
 /************************************************************************************
 * 
 * 名称: App_TimeTickHook
 *
 * 描述: 系统滴答时钟回调函数
 *   
 ************************************************************************************/
 void App_TimeTickHook(void)
 {
 
 }
	 
 
 
 
 
 
