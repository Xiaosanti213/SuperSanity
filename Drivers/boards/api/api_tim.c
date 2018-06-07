/**
 *
 * @file api_tim.c
 *
 * 定时器初始化，定时器中断设置，中断服务函数设置
 *
 **/

#include "api_tim.h"
#include <stdio.h>






volatile uint16_t anyCnt=0,anyCnt2=0;

// 循环标志位
uint8_t  loop500HzFlag,loop200HzFlag,loop50HzFlag,loop600HzFlag,loop100HzFlag,loop20HzFlag,loop10HzFlag;
// 中断计数器
volatile uint16_t loop500Hzcnt,loop200HzCnt,loop50HzCnt , loop600HzCnt,loop100HzCnt, loop20HzCnt , loop10HzCnt=0;




 /************************************************************************************
 * 
 * 函数: TIM4_IRQHandler()
 *
 * 描述：TIM4中断服务函数
 *   
 ************************************************************************************/

void TIM4_IRQHandler(void)		//1ms中断一次
{
    if( TIM_GetITStatus(TIM4 , TIM_IT_Update) != RESET ) 
    {     
					anyCnt++;
					loop200HzCnt++;
					loop100HzCnt++;

					if(++loop50HzCnt * 50 >= (1000))//超过20ms重置计数，置标志位
					{
							loop50HzCnt=0;
							loop50HzFlag=1;
					}
					if(++loop20HzCnt * 20 >=1000 )//超过50ms重置计数，置标志位
					{
							loop20HzCnt=0;
							loop20HzFlag=1;
					}
					if(++loop10HzCnt * 10 >=1000 )//超过10ms重置计数，置标志位
					{
							loop10HzCnt=0;
							loop10HzFlag=1;
					}
          
          TIM_ClearITPendingBit(TIM4 , TIM_FLAG_Update);   //清除中断标志   
    }
}








 /************************************************************************************
 * 
 * 函数: sys_tim_init()
 *
 * 描述：TIM4定时器中断循环初始化
 *   
 ************************************************************************************/
void sys_tim_init(char clock,int Preiod)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  //打开时钟
    
    TIM_DeInit(TIM4);

    TIM_TimeBaseStructure.TIM_Period = Preiod;
    TIM_TimeBaseStructure.TIM_Prescaler = clock-1;//定时1ms
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    
    TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM4,TIM_FLAG_Update);

    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM4,ENABLE);
    
}	







 /************************************************************************************
 * 
 * 函数: timer_nvic_configuration()
 *
 * 描述：TIM4定时器中断配置
 *   
 ************************************************************************************/
void timer_nvic_configuration()
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
  





