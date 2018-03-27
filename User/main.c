/**
 *
 * @file main.c
 *
 * ��ѭ��
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
	
	//�ϵ�֮���뱣֤�����������
	sd sensors_data;
	sc calib_data;
	int16_t output[4] = {0,0,0,0}; 
	ad attitude_data;
	float reference[4];
	
	// ���Զ�ʱ���������ֵ
	systick_init();  
  
  SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;  
  delay(10000);     
  SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;  
	
	
	
	
	
	sensors_init();
  sensors_calibration(&calib_data, &sensors_data);//һ��Ҫ��ֹˮƽ�������ᣬҡ���������ϵ�
	while(1)
	{
		get_sensors_data(&sensors_data, &calib_data);
		attitude_estimate(&attitude_data, &sensors_data);
		set_reference(sensors_data.rc_command, reference); //ҡ�˶�������ת��Ϊ���Ʋο�
		attitude_control(attitude_data, reference, output);
		
		mix_table(output, &sensors_data); 
		go_arm_check(sensors_data.rc_command);//����֮ǰ��Ӧ��������಻Ӧ�ж���
		write_mini_motors(sensors_data.motor);
	}



	return 0;
}




