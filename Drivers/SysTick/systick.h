/**
 *
 * @file systick.h
 *
 * 定时器相关设置函数声明
 *
 **/
 
 #ifndef _SYSTICK_H
 #define _SYSTICK_H
 
 #include "stm32f10x.h"
 
 
 

 void delay(__IO u32 nCount);
 void systick_init(void);
 void current_time_count(void);
 void delay_4us(void);
 void delay_1us(void);
 
 
 #endif
