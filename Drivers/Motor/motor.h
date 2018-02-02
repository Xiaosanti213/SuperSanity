#ifndef _MOTOR_H
#define _MOTOR_H

#include <stm32f10x.h>
#include "sensors.h" 

//通道编号
#define ROLL			0
#define PITCH			1
#define THROTTLE  2
#define YAW				3


//函数调用频率为10*5Hz 相应计算下面时间阈值
#define ARM_DELAY               10    //当前测试状态比较好用的
#define DISARM_DELAY            10




void write_mini_motors(u16* motor);// 空心杯电机
void write_motors(u16* motor);// 普通无刷电机
void mix_table(int16_t*, sd*);
 
void go_arm_check(u16*);

 
 
#endif





