#ifndef _MOTOR_H
#define _MOTOR_H

#include <stm32f10x.h>
#include "sensors.h" 

//ͨ�����
#define ROLL			0
#define PITCH			1
#define THROTTLE  2
#define YAW				3





void write_mini_motors(u16* motor);// ���ı����
void write_motors(u16* motor);// ��ͨ��ˢ���
void mix_table(int16_t* axis_pid, struct sensors_data* sd);//struct����ʡ��
 
 
 
 
#endif





