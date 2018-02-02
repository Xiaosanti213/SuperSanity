#ifndef _SENSORS_H
#define _SENSORS_H


#include <stm32f10x.h>



typedef struct sensors_data
{
  u16 rc_command[4];//�ṹ�嵱�в��ܸ���ֵ
	u16 motor[4];
	
	float acc[3];
  float gyro[3];
	float temp[1];//��ǰ���ݴ�������ҪMPU6050��MS5611���¶�����
	float mag[3]; 
	float press;
	
}sd;




typedef struct sensors_calib
{
	u8 acc_calib[3];
	u8 gyro_calib[3]; 
}sc; 










void sensors_init(void);
void get_sensors_data(sd*, sc*);
void init_recog_motors(void);
void go_disarm(void);
void nrf_read_to_motors(u16* rc_command);
void sensors_calibration(sc* s_calib, sd* s_data);




#endif



