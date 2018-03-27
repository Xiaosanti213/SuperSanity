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


#include "systick.h"





int main()
{
	
	//上电之后请保证各个舵程中立
	sd sensors_data;
	sc calib_data;
	int16_t output[4] = {0,0,0,0}; 
	ad attitude_data;
	float reference[4];
	
	// 测试定时器输出计数值
	systick_init();  
  
  SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;  
  delay(10000);     
  SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;  
	
	
	
	
	
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




