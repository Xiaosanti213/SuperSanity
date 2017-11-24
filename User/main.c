//test sensor drivers

#include "stm32f10x.h"

#include "mpu6050.h"
#include "nRF24L01.h"
#include "debug.h" 
#include "app_cfg.h"

#include "api_usart.h"
#include "board_config.h" 

#include <string.h>
#include <stdio.h>


#include "ucos_ii.h"


int main()
{
	//char*	 string_to_send = "Hello world";
	//short  data_to_send[] = {1,2,3,4}; 	
	OSInit();
	gpio_clk_config();//������	
	debug_usart_init();	
	os_tick_init();
  status_led_gpio_config();
	
	
	//uint16_t* acc, *gyro, *temp;
	
	
	
	// STATUS_LED_ON;
	
	// ���Ӧ�ö�tim�������ã��������������ֵ�ƽΪ�ͣ���ֹ�߻�
	// tim_config();
	

	
	
	// i2c_mpu6050_init();
	// i2c_mpu6050_check();
	

  // i2c_mpu6050_read_acc( acc);
  // i2c_mpu6050_read_gyro( gyro);
  // i2c_mpu6050_read_temp( temp);
  // i2c_mpu6050_delay();
		
	
	
	spi_nrf_init();
	
	//printf("\r\n���ٶȣ� %8d%8d%8d    ",data_to_send[0],data_to_send[1],data_to_send[2]);
	while(1)
	{
		//usart_debug_send_data(data_to_send, sizeof(data_to_send));
		//printf("\r\n���ٶȣ� %8d%8d%8d		",data_to_send[0],data_to_send[1],data_to_send[2]);	
		STATUS_LED_ON;
	}

}

