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



extern __IO u32 current_time_us;

int main()
{
	
	//上电之后请保证各个舵程中立
	sd sensors_data;
	sc calib_data;
	int16_t output[4] = {0,0,0,0}; 
	ad attitude_data;
	float reference[4];
	u16 i,j;
	//static u32 rc_time = 0;
	
	//定时计数开启
	//systick_init();   
	
	//TODO: 传感器初始化和校准会放在传感器读数当中
	sensors_init();
  sensors_calibration(&calib_data, &sensors_data);
	//一定要静止水平放置四轴，摇杆中立再上电，否则可能校准失败
	
	while(1)
	{
		//1 RC时间控制,超出20ms则读取遥控器数据
		//if(current_time_us > rc_time)
		//{
			//1.1 时间控制50Hz
			//rc_time = current_time_us+20000;
			
			//1.2 舵量读取，并用校准值修正
			compute_rc(&sensors_data, &calib_data);

			//1.3 TODO:添加四次读取平滑滤波
			
			//1.4 解锁条件检查
			go_arm_check(sensors_data.rc_command);//解锁之前不应有输出，亦不应有舵量
		//}
		//2 未达到20ms，读取传感器
		//else
		{
			//2.1 参考MWC：读取其他传感器，如GPS等
		}
		//3 读取数据，将校准应用进去，并完成姿态解算
		get_sensors_data(&sensors_data, &calib_data);
		attitude_estimate(&attitude_data, &sensors_data);
		
		
		//4 PID控制
		set_reference(sensors_data.rc_command, reference); //摇杆舵量数据转化为控制参考
		attitude_control(attitude_data, reference, output, sensors_data.rc_command[2]);
		
		//5 输出电机
		mix_table(output, &sensors_data); 
		write_mini_motors(sensors_data.motor);
		
		//6 延时经过测试达到20.01ms
		for(i = 86; i > 0; i--)
		  for(j = 87; j > 0; j--)
			{;}
	}



	return 0;
}



