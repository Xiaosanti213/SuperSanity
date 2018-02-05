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

 
static int  arm_motors_flag = 0; //默认上锁状态
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
	 if(arm_motors_flag)
	 {
	 	// 设置4个空心杯电机输出量
	  TIM_SetCompare1(PWM_TIM, motor[0]);
	  TIM_SetCompare2(PWM_TIM, motor[1]);
	  TIM_SetCompare3(PWM_TIM, motor[2]);
		TIM_SetCompare4(PWM_TIM, motor[3]);
	 }
	 else//强制输出0
	 {
		TIM_SetCompare1(PWM_TIM, 0);
	  TIM_SetCompare2(PWM_TIM, 0);
	  TIM_SetCompare3(PWM_TIM, 0);
		TIM_SetCompare4(PWM_TIM, 0);
	 }

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
 * 名称: go_arm_check()
 *
 * 描述：电机解锁怠速状态
 *
 */ 
void go_arm_check(u16* rc_command)
{
	static int16_t arming_counter;
	u16 temp;
	// 下面通过四个舵量进行判断 舵量范围
	// 对于空心杯电机来说，已经映射到[0-2000]
	
	if (rc_command[2] > 30) {
  // 前提油门位于最低点，否则直接退出
      arming_counter = 0;
      return;  
   } 
	 temp = rc_command[3];
	 // 取出副翼舵量
	 if (temp > 1970)
	 {
			if(arming_counter < ARM_DELAY)
			{
				arming_counter ++;
				//printf("********************************************\n");
				//printf("当前摇杆状态解锁计数值：%d\n", arming_counter);
			}
			else if(arming_counter == ARM_DELAY)
			{
				arming_counter = 0;
				// 计数器清零
				STATUS_LED_ON;
				// 指示灯点亮
				arm_motors_flag = 1;
				// 可以输出信号
			}
	 }
	 else if(temp < 30)
	 // 如果通讯状况不良，则舵量信号一直是0，保证ARM_MOTORS==0
	 {
			if(arming_counter < DISARM_DELAY)
			{
				arming_counter ++; 
				//printf("********************************************\n");
				//printf("当前摇杆状态解锁计数值：%d\n", arming_counter);
			}
			else if(arming_counter == DISARM_DELAY)
			{
			  arming_counter = 0;
				// 计数器清零
				STATUS_LED_OFF;
				// 上锁状态指示灯熄灭
				arm_motors_flag = 0;
				// 禁止输出信号
			}
	 }
}





 
 
 
 
 
 
 
 /**
 *
 * 名称: mix_table()
 *
 * 描述：将机型姿态与电机对应
 *
 */ 
void mix_table(int16_t* output, sd* s_data)//struct不能省略
 {
	 u8 i=0;
	 //#define  PIDMIX(X,Y,Z)  sd->rc_command[THROTTLE] + axis_pid[ROLL]*X + axis_pid[PITCH]*Y + axis_pid[YAW]*Z
	 #define  PIDMIX(X,Y,Z)  output[0]*X + output[1]*Y + output[2]*Z + output[3];
	 // 对于X型四轴
	 s_data->motor[0] = PIDMIX(-0.02,+0.02,-0.02);  //右前 1号电机 
	 s_data->motor[1] = PIDMIX(+0.02,+0.02,+0.02);  //左前 2号电机
	 s_data->motor[2] = PIDMIX(+0.02,-0.02,-0.02);  //左后 3号电机
	 s_data->motor[3] = PIDMIX(-0.02,-0.02,+0.02);  //右后 4号电机
	 
	 for(i = 0; i<4; i++)
			{
				if(s_data->motor[i]>2000)
					 s_data->motor[i] = 2000;
			  else;
			}
			
	 printf("1st : %d   \n", s_data->motor[0]);
	 printf("2nd : %d   \n", s_data->motor[1]);
	 printf("3rd : %d   \n", s_data->motor[2]);
	 printf("4th : %d   \n", s_data->motor[3]);
	 
}
 
 
 
 
 
 
 
 
 
 
 

