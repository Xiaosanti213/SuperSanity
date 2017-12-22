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
#include "sensors.h"

#include <stdio.h>

 
 
static void delay_ms(u16); 
/**
 *
 * ����: write_mini_motors
 *
 * ���������ı����ֱ�������ĸ�����ȽϼĴ�����ֵ
 *
 */ 
 void write_mini_motors(u16* motor)
 {
	 
	  // ����4�����ı���������
	  TIM_SetCompare1(PWM_TIM, motor[0]);
	  TIM_SetCompare2(PWM_TIM, motor[1]);
	  TIM_SetCompare3(PWM_TIM, motor[2]);
		TIM_SetCompare4(PWM_TIM, motor[3]);
 
 }
 
 
 
 /**
 *
 * ����: write_motors
 *
 * ���������ֱ�������ĸ�����ȽϼĴ�����ֵ
 *
 */ 
 void write_motors(u16* motor)
 {
	 
	  // ����4����������
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
	  write_motors(motor);
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
 
 
 
 
 
 
 
 /**
 *
 * ����: mix_table()
 *
 * ��������������̬������Ӧ
 *
 */ 
void mix_table(int16_t* axis_pid, struct sensors_data* sd)//struct����ʡ��
 {
	 //#define  PIDMIX(X,Y,Z)  sd->rc_command[THROTTLE] + axis_pid[ROLL]*X + axis_pid[PITCH]*Y + axis_pid[YAW]*Z
	 #define  PIDMIX(X,Y,Z)  sd->rc_command[THROTTLE] + sd->rc_command[ROLL]*X + sd->rc_command[PITCH]*Y + sd->rc_command[YAW]*Z
	 // ����X������
	 sd->motor[0] = PIDMIX(-0,-0,+0);  //��ǰ 1�ŵ��
	 sd->motor[1] = PIDMIX(+0,-0,-0);  //��ǰ 2�ŵ��
	 sd->motor[2] = PIDMIX(+0,+0,+0);  //��� 3�ŵ��
	 sd->motor[3] = PIDMIX(-0,+0,+0);  //�Һ� 4�ŵ��
	 
	 printf("һ�ţ�%d\n", sd->motor[0]);
	 printf("���ţ�%d\n", sd->motor[1]);
	 printf("���ţ�%d\n", sd->motor[2]);
	 printf("�ĺţ�%d\n", sd->motor[3]);
 }
 
 
 
 
 
 
 
 
 
 
 

