//test sensor drivers

#include "stm32f10x.h"

#include "mpu6050.h"
#include "nRF24L01.h"
#include "debug.h" 

#include "api_usart.h"
#include "board_config.h" 

#include <string.h>
#include <stdio.h>


int main()
{

	//char*	 string_to_send = "Hello world";
	short  data_to_send[] = {1,2,3,4}; 

	//uint16_t* acc, *gyro, *temp;
	
	gpio_clk_config();//调试用
	status_led_gpio_config();
	
	// STATUS_LED_ON;
	
	// 这块应该对tim进行配置，如果不输出，保持电平为低，防止走火
	// tim_config();
	

	
	
	// i2c_mpu6050_init();
	// i2c_mpu6050_check();
	

  // i2c_mpu6050_read_acc( acc);
  // i2c_mpu6050_read_gyro( gyro);
  // i2c_mpu6050_read_temp( temp);
  // i2c_mpu6050_delay();
	
	debug_usart_init();
  debug_usart_check();		
	
	
	spi_nrf_init();
	spi_nrf_check();
	
	printf("\r\n加速度： %8d%8d%8d    ",data_to_send[0],data_to_send[1],data_to_send[2]);
	while(1)
	{
		//usart_debug_send_data(data_to_send, sizeof(data_to_send));
		//printf("\r\n加速度： %8d%8d%8d		",data_to_send[0],data_to_send[1],data_to_send[2]);	
	}

}

