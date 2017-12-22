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
#include "sensors.h"

#include <stdio.h>

 
 
static void delay_ms(u16); 
/**
 *
 * 名称: write_mini_motors
 *
 * 描述：空心杯电机直接设置四个电机比较寄存器的值
 *
 */ 
 void write_mini_motors(u16* motor)
 {
	 
	  // 设置4个空心杯电机输出量
	  TIM_SetCompare1(PWM_TIM, motor[0]);
	  TIM_SetCompare2(PWM_TIM, motor[1]);
	  TIM_SetCompare3(PWM_TIM, motor[2]);
		TIM_SetCompare4(PWM_TIM, motor[3]);
 
 }
 
 
 
 /**
 *
 * 名称: write_motors
 *
 * 描述：电机直接设置四个电机比较寄存器的值
 *
 */ 
 void write_motors(u16* motor)
 {
	 
	  // 设置4个电机输出量
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
	  write_motors(motor);
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
 
 
 
 
 
 
 
 /**
 *
 * 名称: mix_table()
 *
 * 描述：将机型姿态与电机对应
 *
 */ 
void mix_table(int16_t* axis_pid, struct sensors_data* sd)//struct不能省略
 {
	 //#define  PIDMIX(X,Y,Z)  sd->rc_command[THROTTLE] + axis_pid[ROLL]*X + axis_pid[PITCH]*Y + axis_pid[YAW]*Z
	 #define  PIDMIX(X,Y,Z)  sd->rc_command[THROTTLE] + sd->rc_command[ROLL]*X + sd->rc_command[PITCH]*Y + sd->rc_command[YAW]*Z
	 // 对于X型四轴
	 sd->motor[0] = PIDMIX(-0,-0,+0);  //右前 1号电机
	 sd->motor[1] = PIDMIX(+0,-0,-0);  //左前 2号电机
	 sd->motor[2] = PIDMIX(+0,+0,+0);  //左后 3号电机
	 sd->motor[3] = PIDMIX(-0,+0,+0);  //右后 4号电机
	 
	 printf("一号：%d\n", sd->motor[0]);
	 printf("二号：%d\n", sd->motor[1]);
	 printf("三号：%d\n", sd->motor[2]);
	 printf("四号：%d\n", sd->motor[3]);
 }
 
 
 
 
 
 
 
 
 
 
 

