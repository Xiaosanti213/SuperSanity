/**
 *
 * @file motor.c
 *
 * �Կ��ı�������ַ���ģʽ����
 *
 **/
 
#include "motor.h"
#include "board_config.h"
#include "stm32f10x_tim.h"
#include "stm32f10x.h"



 
 
static void delay_ms(u16); 
/**
 *
 * ����: set_mini_motors
 *
 * ���������ı����ֱ�������ĸ�����ȽϼĴ�����ֵ
 *
 */ 
 void set_mini_motors(u16* motor)
 {
	 
	  // ����4�����ı���������
	  TIM_SetCompare1(PWM_TIM, motor[0]);
	  TIM_SetCompare2(PWM_TIM, motor[1]);
	  TIM_SetCompare3(PWM_TIM, motor[2]);
		TIM_SetCompare4(PWM_TIM, motor[3]);
 
 }
 
 
 
 /**
 *
 * ����: set_motors
 *
 * ���������ֱ�������ĸ�����ȽϼĴ�����ֵ
 *
 */ 
 void set_motors(u16* motor)
 {
	 
	  // ����4�����ı���������
	  TIM_SetCompare1(PWM_TIM, motor[0]);
	  TIM_SetCompare2(PWM_TIM, motor[1]);
	  TIM_SetCompare3(PWM_TIM, motor[2]);
		TIM_SetCompare4(PWM_TIM, motor[3]);
 
 }
 
 
 
 
/**
 *
 * ����: set_mini_motors
 *
 * �����������ʼ����ʱ
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
	  // ��ʼ���ĸ����
	  set_motors(motor);
	  delay_ms(5000);//��ʱ5s
	 
 }
 
 
 
 /**
 *
 * ����: delay_ms()
 *
 * ���������
 *
 */  
 void delay_ms(u16 ms)
 {
	 u16 temp = 10000;
	 //������Ƶ��ʵ����ȷ��
		for (; ms; ms--)
	 {
			for (; temp; temp--);
	 }
 }
 
 
 
 
 
 /**
 *
 * ����: go_disarm()
 *
 * �����������������״̬
 *
 */ 
 void go_disarm()
 {
		
 }
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 

