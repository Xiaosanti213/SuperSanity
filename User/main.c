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



extern __IO u32 current_time_us;

int main()
{
	
	//�ϵ�֮���뱣֤�����������
	sd sensors_data;
	sc calib_data;
	int16_t output[4] = {0,0,0,0}; 
	ad attitude_data;
	float reference[4];
	
	static u32 rc_time = 0;
	
	//��ʱ��������
	systick_init();   

	//TODO: ��������ʼ����У׼����ڴ�������������
	sensors_init();
  sensors_calibration(&calib_data, &sensors_data);
	//һ��Ҫ��ֹˮƽ�������ᣬҡ���������ϵ磬�������У׼ʧ��
	
	while(1)
	{
		//1 RCʱ�����,����20ms���ȡң��������
		if(current_time_us > rc_time)
		{
			//1.1 ʱ�����50Hz
			rc_time = current_time_us+20000;
			
			//1.2 ������ȡ������У׼ֵ����
			compute_rc(&sensors_data, &calib_data);

			//1.3 TODO:����Ĵζ�ȡƽ���˲�
			
			//1.4 �����������
			go_arm_check(sensors_data.rc_command);//����֮ǰ��Ӧ��������಻Ӧ�ж���
		}
		//2 δ�ﵽ20ms����ȡ������
		else
		{
			//2.1 �ο�MWC����ȡ����������
		}
		//3 ��ȡ���ݣ���̬����
		get_sensors_data(&sensors_data, &calib_data);
		attitude_estimate(&attitude_data, &sensors_data);
		
		
		//4 PID����
		set_reference(sensors_data.rc_command, reference); //ҡ�˶�������ת��Ϊ���Ʋο�
		attitude_control(attitude_data, reference, output);
		
		//5 ������
		mix_table(output, &sensors_data); 
		write_mini_motors(sensors_data.motor);
	}



	return 0;
}




