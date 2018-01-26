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




//static void nrf_read_to_motors(u16* rc_command);
		


	
	
	


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
	//os_tick_init(); 						// 配置系统滴答时钟
	
	status_led_gpio_config();			// 指示灯LED
	
  i2c_mpu6050_init_s();					// 写入配置参数
	i2c_mpu6050_config_mag_s();		// 获取从机数据
  i2c_mpu6050_init_mag_s();
  i2c_ms5611_init_s();
	
	spi_nrf_init();								// 接收摇杆数据
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
void get_sensors_data(sd* sdata)
{
	
  //if(i2c_mpu6050_check_s() && i2c_ms5611_calculate_s())
	//if(i2c_mpu6050_check_s())
	//{
			i2c_mpu6050_read_acc_s(sdata->acc);
			i2c_mpu6050_read_gyro_s(sdata->gyro);
			i2c_mpu6050_read_temp_s(sdata->temp);
		
			//printf("HMC5883L数据： \n");
	    i2c_mpu6050_read_mag_s(sdata->mag); 
	    //printf("magx: %d%s%d%s%d", sd->mag[0], "magy: ", sd->mag[1], "magz: ", sd->mag[2]);
			sdata->press = i2c_ms5611_calculate_s();
			//printf("MS5611数据： \n");
			//printf("press: %d\n", sd->press);
	//}
			nrf_read_to_motors(sdata->rc_command);
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
		//STATUS_LED_ON;//指示灯表示当前接收到数据
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
	  //printf("RA右手副翼：%d%s", rc_command[0], "\n");
	  //printf("LE左手升降：%d%s", rc_command[1], "\n");
	  //printf("RT右手油门：%d%s", rc_command[2], "\n");
	  //printf("LR左手方向：%d%s", rc_command[3], "\n");		
	} 
  else 
		//printf("数据接收失败，请检查线路连接...\n");
	;
} 







