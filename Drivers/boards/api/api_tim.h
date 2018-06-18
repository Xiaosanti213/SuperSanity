#ifndef _tim_H_
#define _tim_H_
#include "stm32f10x.h"






void sys_tim_init(char clock,int Preiod);//定时器初始化
void timer_nvic_configuration(void);
void TIM4_IRQHandler(void);



// 声明外部可见main.c当中会用到
extern volatile uint16_t anyCnt,anyCnt2,loop100HzCnt,loop200HzCnt;
extern uint8_t  loop200HzFlag,loop500HzFlag,loop50HzFlag,loop600HzFlag,loop100HzFlag,loop10HzFlag,loop20HzFlag;

#endif

