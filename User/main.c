//test sensor drivers

#include "stm32f10x.h"

#include "mpu6050.h"
#include "nRF24L01.h"
#include "debug.h" 
#include "app_cfg.h"
#include "motor.h"
#include "ms5611.h" 
#include "hmc5883l.h"

#include "api_usart.h"
#include "board_config.h" 

#include <string.h>
#include <stdio.h> 


#include "ucos_ii.h"


int main()
{
	
	//u8 rxbuf[8];			 
  //u8 status;
	//u16 rx_val[4] = {0, 0, 0, 0};
  //u16 motor[4] = {0, 0, 0, 0};
	
	//int16_t acc[3] = {0, 0, 0};
  //int16_t gyro[3] = {0, 0, 0};
	//float temp[1] = {0.0};
	
	//int32_t press = 0;
	int16_t mag[3] = {0, 0, 0}; 

  
	//OSInit();
	//gpio_clk_config();//调试用	开启全部外设时钟
	debug_usart_init();	
	//os_tick_init();
  //status_led_gpio_config();
	//tim_config();
	
	
	
	 //STATUS_LED_ON;
	
	// 这块应该对tim进行配置，如果不输出，怠速速度
	
	//STATUS_LED_OFF;
  i2c_mpu6050_init();
	i2c_mpu6050_check();
	
  //i2c_ms5611_calculate();

	
	//spi_nrf_init();
	//spi_nrf_rx_mode();
	//printf("\r\n加速度： %8d%8d%8d    ",data_to_send[0],data_to_send[1],data_to_send[2]);
	//while(1)
	//{
		//spi_nrf_rx_packet((uint8_t*)ADC_ConvertedValueLocal);
		//printf("Get Voltage: %f", ADC_ConvertedValueLocal[0]);	
		//usart_debug_send_data(data_to_send, sizeof(data_to_send));
		//printf("\r\n加速度： %8d%8d%8d		",data_to_send[0],data_to_send[1],data_to_send[2]);	
		//STATUS_LED_ON;
		
		
	   /*判断接收状态 收到数据*/
	 //if(spi_nrf_rx_packet(rxbuf))
	//{
		//下面这个步骤已经将motor值映射到了1000~2000上
		//rx_val[0] = (float)(rxbuf[1]<<8 | rxbuf[0])/4096*1000 + 1000;
		//rx_val[1] = (float)(rxbuf[3]<<8 | rxbuf[2])/4096*1000 + 1000;
		//rx_val[2] = (float)(rxbuf[5]<<8 | rxbuf[4])/4096*1000 + 1000;
		//rx_val[3] = (float)(rxbuf[7]<<8 | rxbuf[6])/4096*1000 + 1000;
		
	 //	printf("\r\n 从机端接收到遥控器杆量数据：\n");
	 // printf("RA右手副翼：%d%s", rx_val[0], "\n");
	 // printf("LE左手升降：%d%s", rx_val[1], "\n");
	 // printf("RT右手油门：%d%s", rx_val[2], "\n");
	 // printf("LR左手方向：%d%s", rx_val[3], "\n");
		
	 // 对于空心杯电机来说：总占空比频率不变50Hz(20000)则需将值映射到0~20000上
		//motor[0] = (rx_val[0] - 1000)*200;
		//motor[1] = (rx_val[1] - 1000)*200;
    //motor[2] = (rx_val[2] - 1000)*200;
		//motor[3] = (rx_val[3] - 1000)*200;
		
		
		
	//} 
	
	//if(i2c_mpu6050_check())
	//{
	//	while(1)
	//	{
	//		printf("MPU6050数据：\n");
	//		i2c_mpu6050_read_acc( acc);
	//		printf("accx: %d%s%d%s%d%s", acc[0], " accy: ", acc[1], " accz: ", acc[2], "\n");
	//		i2c_mpu6050_read_gyro( gyro);
	//		printf("gyrox: %d%s%d%s%d%s", gyro[0], " gyroy: ", gyro[1], " gyroz: ", gyro[2], "\n");
	//		i2c_mpu6050_read_temp( temp);
	//		printf("temp: %.2f%s", temp[0], "\n");
		
	//		printf("HMC5883L数据： \n");
			//printf("magx: %d%s%d%s%d", mag[0], "magy: ", mag[1], "magz: ", mag[2]);
	//	}

	//}
	//i2c_ms5611_init();
	
	//press = i2c_ms5611_calculate();
	//while(i2c_ms5611_calculate())
	//{
		//printf("pressure is: %d\n", press); 
	//}
	
		//set_mini_motors(motor);	
		
	//}
	
	i2c_mpu6050_config_mag();
  i2c_mpu6050_init_mag();
	i2c_mpu6050_read_mag(mag);
	
}

