/**
 *
 * @file sensors.c
 *
 * 传感器初始化与数据采集
 *
 **/

#include <stm32f10x.h>

#include "board_config.h"
#include "debug.h"
#include "hmc5883l.h"
#include "mpu6050.h"
#include "nRF24L01.h"
#include "ms5611.h"
#include "sensors.h"
#include "app_cfg.h"

#include <stdio.h>
		

	
	


/**
 *
 * 名称：sensors_init
 *
 * 描述：全部传感器设备初始化
 *
 */ 
void sensors_init(void)
{
	debug_usart_init();						// 调试串口	
	status_led_gpio_config();			// 指示灯LED
	
  i2c_mpu6050_init_s();					// 写入配置参数
	i2c_mpu6050_config_mag_s();		// 获取从机数据
  i2c_mpu6050_init_mag_s();
  i2c_ms5611_init_s();
	
	spi_nrf_init();								
	spi_nrf_rx_mode();
	tim_config(); 
	
}




/**
 *
 * 名称：get_sensors_data
 *
 * 描述：读取传感器数据
 *
 */ 
void get_sensors_data(sd* sdata, sc* calib_data)
{
	u8 axis = 0;
	float smooth_factor = 1.5;
	static int16_t acc_filtered[3] = {0,0,0};
	//读取数据
	i2c_mpu6050_read_acc_s(sdata->acc);
	i2c_mpu6050_read_gyro_s(sdata->gyro);
	i2c_mpu6050_read_mag_s(sdata->mag); 
	sdata->press = i2c_ms5611_calculate_s();
	nrf_read_to_motors(sdata->rc_command);
	
	//校准修正
	for(; axis<3; axis--)
  {
		sdata->acc[axis] -= calib_data->acc_calib[axis];
		sdata->gyro[axis] -= calib_data->gyro_calib[axis];
	}
	
	// 迭代加计平滑滤波修正
	for(; axis<3; axis--)
	{
		acc_filtered[axis] -= acc_filtered[axis]/smooth_factor;
		sdata->acc[axis] = acc_filtered[axis] - sdata->acc[axis]/smooth_factor;
	}
}






/**
 *
 * 名称：sensors_calibration
 *
 * 描述：传感器数据校准
 *
 */ 
void sensors_calibration(sc* s_calib, sd* s_data)
{
	u16 acc_sum[3] = {0,0,0};
	u16 gyro_sum[3] = {0,0,0};
	u16 calib_flag = 512;
	u8 axis = 0;
  for(; axis<3 ; axis++)
	{
		s_calib->acc_calib[axis] = 0;
		s_calib->gyro_calib[axis] = 0;
		for(;calib_flag; calib_flag--)
		{
			i2c_mpu6050_read_acc_s(s_data->acc);//读取原生数据
			i2c_mpu6050_read_gyro_s(s_data->gyro);
			acc_sum[axis] += s_data->acc[axis];
			gyro_sum[axis] += s_data->gyro[axis];
		}
		s_calib->acc_calib[axis] = acc_sum[axis]>>9;//512次取平均
		s_calib->gyro_calib[axis] = gyro_sum[axis]>>9;
	}
}






















/**
 *
 * 名称：nrf_read_to_motors
 *
 * 描述：将u8[4]接收到的数据映射到10000~20000上
 *
 */
void nrf_read_to_motors(u16* rc_command)
{
	 u8 rxbuf[8];	
	/*判断接收状态 收到数据*/
	if(spi_nrf_rx_packet(rxbuf))
	{
		//下面这个步骤已经将motor值映射到了1000~2000上
		rc_command[0] = (float)(rxbuf[1]<<8 | rxbuf[0])/4096*1000 + 1000;
		rc_command[1] = (float)(rxbuf[3]<<8 | rxbuf[2])/4096*1000 + 1000;
		rc_command[2] = (float)(rxbuf[5]<<8 | rxbuf[4])/4096*1000 + 1000;
		rc_command[3] = (float)(rxbuf[7]<<8 | rxbuf[6])/4096*1000 + 1000;
		
	 	//printf("\r\n 从机端接收到遥控器杆量数据：\n");
	  //printf("RA右手副翼：%d%s", rc_command[0], "\n");
	  //printf("LE左手升降：%d%s", rc_command[1], "\n");
	  //printf("RT右手油门：%d%s", rc_command[2], "\n");
	  //printf("LR左手方向：%d%s", rc_command[3], "\n");
		
	 // 对于空心杯电机来说：总占空比频率改变5Hz(2000)则需将值映射到0~2000上
	 	rc_command[0] = (rc_command[0] - 1000)*2;
	  rc_command[1] = (rc_command[1] - 1000)*2;
    rc_command[2] = (rc_command[2] - 1000)*2;
	 	rc_command[3] = (rc_command[3] - 1000)*2;	


    //printf("\r\n 从机端接收到遥控器杆量数据：\n");
	  printf("RA右手副翼：%d%s", rc_command[0], "\n");
	  printf("LE左手升降：%d%s", rc_command[1], "\n");
	  printf("RT右手油门：%d%s", rc_command[2], "\n");
	  printf("LR左手方向：%d%s", rc_command[3], "\n");		
	} 
  else 
		//printf("数据接收失败，请检查线路连接...\n");
	;
} 







