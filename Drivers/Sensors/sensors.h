#ifndef _SENSORS_H
#define _SENSORS_H


#include <stm32f10x.h>



struct sensors_data
{
  u16 rc_command[4];//�ṹ�嵱�в��ܸ���ֵ
	u16 motor[4];
	
	float acc[3];
  float gyro[3];
	float temp[1];//��ǰ���ݴ���������ҪMPU6050��MS5611���¶�����
	float mag[3]; 
	float press;
	
};



void sensors_init(void);
void get_sensors_data(struct sensors_data* sd);
void init_recog_motors(void);
void go_disarm(void);
void nrf_read_to_motors(u16* rc_command);







#endif


