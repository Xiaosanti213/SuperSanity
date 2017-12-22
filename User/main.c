//test sensor drivers

#include "stm32f10x.h"

#include "mpu6050.h"
#include "nRF24L01.h"
#include "debug.h" 
#include "app_cfg.h"
#include "motor.h"
#include "ms5611.h" 
#include "hmc5883l.h"
#include "sensors.h"

#include "api_usart.h"
#include "board_config.h" 

#include <string.h>
#include <stdio.h> 


#include "ucos_ii.h"


int main()
{
	u16 i=2000;
	u16 j=2000;
	struct sensors_data sd;
	int16_t axis_pid[3] = {0, 0, 0};
	sensors_init();
	
	
	
	while(1)
	{
	  //get_sensors_data(&sd);
		//nrf_read_to_motors(sd.rc_command);
		get_sensors_data(&sd);
		mix_table(axis_pid, &sd);
		for(; i; i--)
		{
			for(;j; j--);
		}//经过测试，添加之后相响应较好
		
		
		//for(i = 0; i < 4; i++)
		//{
		//  sd.motor[i] = 100;
		//}
		write_mini_motors(sd.motor);
	}
	//OSInit();
	//os_tick_init();
	
		
	

	
	
}

