/**
 *
 * @file motor.c
 *
 * 对空心杯电机几种飞行模式配置
 *
 **/
 
#include "motor.h"
#include "board_config.h"
#include "stm32f10x_tim.h"
#include "stm32f10x.h"



 
 
static void delay_ms(u16); 
/**
 *
 * 名称: set_mini_motors
 *
 * 描述：空心杯电机直接设置四个电机比较寄存器的值
 *
 */ 
 void set_mini_motors(u16* motor)
 {
	 
	  // 设置4个空心杯电机输出量
	  TIM_SetCompare1(PWM_TIM, motor[0]);
	  TIM_SetCompare2(PWM_TIM, motor[1]);
	  TIM_SetCompare3(PWM_TIM, motor[2]);
		TIM_SetCompare4(PWM_TIM, motor[3]);
 
 }
 
 
 
 /**
 *
 * 名称: set_motors
 *
 * 描述：电机直接设置四个电机比较寄存器的值
 *
 */ 
 void set_motors(u16* motor)
 {
	 
	  // 设置4个空心杯电机输出量
	  TIM_SetCompare1(PWM_TIM, motor[0]);
	  TIM_SetCompare2(PWM_TIM, motor[1]);
	  TIM_SetCompare3(PWM_TIM, motor[2]);
		TIM_SetCompare4(PWM_TIM, motor[3]);
 
 }
 
 
 
 
/**
 *
 * 名称: set_mini_motors
 *
 * 描述：电机初始化延时
 *
 */  
 void init_recog_motors(void)
 {
	  u8 i = 0;
	  u16 motor[4];
	 for (; i < 4; i++)
	 {
			motor[i] = 1050;
	 }
	  // 初始化四个电机
	  set_motors(motor);
	  delay_ms(5000);//延时5s
	 
 }
 
 
 
 /**
 *
 * 名称: delay_ms()
 *
 * 描述：电机
 *
 */  
 void delay_ms(u16 ms)
 {
	 u16 temp = 10000;
	 //根据主频和实验结合确定
		for (; ms; ms--)
	 {
			for (; temp; temp--);
	 }
 }
 
 
 
 
 
 /**
 *
 * 名称: go_disarm()
 *
 * 描述：电机解锁怠速状态
 *
 */ 
 void go_disarm()
 {
		
 }
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 

