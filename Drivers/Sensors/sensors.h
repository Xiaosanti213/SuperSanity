#ifndef _SENSORS_H
#define _SENSORS_H


#include <stm32f10x.h>



typedef struct sensors_data
{
  u16 rc_command[4];//结构体当中不能赋初值
	u16 motor[4];
	
	float acc[3];
  float gyro[3];
	float temp[1];//当前数据处理还不需要MPU6050和MS5611的温度数据
	float mag[3]; 
	float press;
	
}sd;




typedef struct sensors_calib
{
	float acc_calib[3];
	float gyro_calib[3];
  float	rc_calib[4]; //油门校准数据始终为0
}sc; 





void sensors_init(void);
void get_sensors_data(sd*, sc*);
void init_recog_motors(void);
void go_disarm(void);
void nrf_read_to_motors(void);
void sensors_calibration(sc* s_calib, sd*);
void delay_approx(u16);
void compute_rc(sd* , sc*);


// 声明外部调用
extern int16_t rccommand[4];





#endif



