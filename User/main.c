//test sensor drivers

#include <stm32f10x.h>
#include "motor.h"
#include "sensors.h"
#include <string.h>
#include <stdio.h> 
#include "attitude_estimate.h"
#include "attitude_control.h"

#include "ucos_ii.h"
#include "app_cfg.h"


int main()
{
	
	sd sensors_data;
	u16 output[4] = {0,0,0,0};
	ad attitude_data;
	float reference[4];
	sensors_init();

	while(1)
	{
		get_sensors_data(&sensors_data);
		attitude_estimate(&attitude_data, &sensors_data);
		set_reference(sensors_data.rc_command, reference); //摇杆舵量数据转化为控制参考
		attitude_control(attitude_data, reference, output);
		
		mix_table(output, &sensors_data);
		go_arm_check(sensors_data.rc_command);//校验一下解锁手势
		write_mini_motors(sensors_data.motor);
	}
	//OSInit();
	//os_tick_init();
	
		
	

	
	
}

