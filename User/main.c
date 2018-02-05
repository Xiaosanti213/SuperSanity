/**
 *
 * @file main.c
 *
 * 主循环
 *
 **/
 
 
#include <stm32f10x.h>
#include "motor.h"
#include "sensors.h"
#include <string.h>
#include <stdio.h> 
#include "attitude_estimate.h"
#include "attitude_control.h"
#include "board_config.h"

#include "ucos_ii.h"
#include "app_cfg.h"



/*
#define TaskStkLength 64
void Task0(void* pdata)
{
	pdata = pdata;
	controller_init();//飞控板初始化
	while(1)
	{
		STATUS_LED1_ON;
		OSTimeDly(OS_TICKS_PER_SEC/4);
		STATUS_LED1_OFF;
		OSTimeDly(OS_TICKS_PER_SEC/4);
	}
}





void Task1(void* pdata)
{
	pdata = pdata;
	while(1)
	{
		STATUS_LED_ON;
		OSTimeDly(OS_TICKS_PER_SEC/3);
		STATUS_LED_OFF;
		OSTimeDly(OS_TICKS_PER_SEC/3);
	}
}





*/





int main()
{
  // 测试操作系统配置	任务调度
	/*OS_STK TaskStk0[TaskStkLength];
	OS_STK TaskStk1[TaskStkLength];
  OSInit();
	OSTaskCreate(Task0, (void*)0, &TaskStk0[TaskStkLength-1],4);
	OSTaskCreate(Task1, (void*)0, &TaskStk1[TaskStkLength-1],5); 
  OSStart(); 
	*/
	
	
	
	//上电之后请保证各个舵程中立
	sd sensors_data;
	sc calib_data;
	int16_t output[4] = {0,0,0,0}; 
	ad attitude_data;
	float reference[4];
	sensors_init();
  sensors_calibration(&calib_data, &sensors_data);//一定要静止水平放置四轴，摇杆中立再上电
	while(1)
	{
		get_sensors_data(&sensors_data, &calib_data);
		attitude_estimate(&attitude_data, &sensors_data);
		set_reference(sensors_data.rc_command, reference); //摇杆舵量数据转化为控制参考
		attitude_control(attitude_data, reference, output);
		
		mix_table(output, &sensors_data); 
		go_arm_check(sensors_data.rc_command);//解锁之前不应有输出，亦不应有舵量
		write_mini_motors(sensors_data.motor);
	}



	return 0;
}




