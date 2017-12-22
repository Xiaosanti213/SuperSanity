#ifndef _MOTOR_H
#define _MOTOR_H

#include <stm32f10x.h>
#include "sensors.h" 

//通道编号
#define ROLL			0
#define PITCH			1
#define THROTTLE  2
#define YAW				3





void write_mini_motors(u16* motor);// 空心杯电机
void write_motors(u16* motor);// 普通无刷电机
void mix_table(int16_t* axis_pid, struct sensors_data* sd);//struct不能省略
 
 
 
 
#endif





