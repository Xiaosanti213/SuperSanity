/**
 *
 * @file ms5611.c
 *
 * 气压计ms5611配置与数据传输
 *
 **/



#include "ms5611.h"
#include "board_config.h" 
#include <stm32f10x_i2c.h>
#include <stdio.h>
#include <stm32f10x.h>
#include "api_i2c.h"
#include "sensors.h"


static void i2c_ms5611_read_calibration_s(uint8_t cx, uint16_t* cx_buffer);
static void i2c_ms5611_read_adc_s(uint8_t config, uint32_t* adc_buffer);
static u8 i2c_ms5611_send_cmd_s(uint8_t cmd); 
static uint8_t i2c_ms5611_receive_data_s(uint8_t* buffer, uint8_t num);






/**
 *  名称: i2c_ms5611_init_s
 *
 *  描述：ms5611设备复位初始化
 *
 */
void i2c_ms5611_init_s(void)
{
	//ms5611_config();																	//ms5611引脚和片上外设配置
	ms5611_i2c_gpio_config_s();
  delay_approx(1000); 														  	//ms5611上电延时
	
	i2c_ms5611_send_cmd_s(MS5611_RESET);	    					//写入传感器复位指令
  delay_approx(1000); 
}






/**
 * 
 * 名称：i2c_ms5611_read_calibration_s
 *
 * 描述：读出PROM 16bit校准字
 *
 */
void i2c_ms5611_read_calibration_s(uint8_t cx, uint16_t* cx_buffer)
{
	uint8_t cx_pre_get[2] = {1, 1}; 															  		// 分别获取低8bit 高8bit
	i2c_ms5611_send_cmd_s(cx);																					// 发送指令	
	i2c_ms5611_receive_data_s(cx_pre_get, 2);
	*cx_buffer = ((u16)cx_pre_get[0] << 8) | ((u16)cx_pre_get[1]);      // 整合到u16当中
}





/**
 * 
 * 名称：i2c_ms5611_read_adc
 *
 * 描述：读出ADC 24bit温度,压力数据
 *
 */
void i2c_ms5611_read_adc_s(uint8_t config, uint32_t* adc_buffer)//形参一个字节，传参需要
{
	u8 adc_pre[3]; 
	i2c_ms5611_send_cmd_s(config);																		// 发送配置指令D1或D2以及OSR
	delay_approx(300);																	      				// 等待转换结束，经过测试，延时最短时间100~300
	i2c_ms5611_send_cmd_s(MS5611_ADC_READ); 													// 发送MS5611_ADC_READ指令
	i2c_ms5611_receive_data_s(adc_pre, 3);
	*adc_buffer = (adc_pre[0] << 16) | (adc_pre[1] << 8) | (adc_pre[2]); 
}






/**
 * 
 * 名称：i2c_ms5611_calculate_s
 *
 * 描述：整合传感器数据，获得气压
 *
 */
int32_t i2c_ms5611_calculate_s(void)
{
	int32_t difference_temp, pressure;
	int64_t offset, sensitivity, temperature;
	u16 coefficient[6];
	u32 digital_pressure, digital_temp;
	
	i2c_ms5611_init_s();
	
		i2c_ms5611_read_calibration_s(PROM_READ_C1, &(coefficient[0]));	//C1: SENS_T1 u16
		i2c_ms5611_read_calibration_s(PROM_READ_C2, &(coefficient[1]));	//C2: OFF_T1 u16						
		i2c_ms5611_read_calibration_s(PROM_READ_C3, &(coefficient[2]));	//C3: TCS u16
		i2c_ms5611_read_calibration_s(PROM_READ_C4, &(coefficient[3]));	//C4: TCO u16
		i2c_ms5611_read_calibration_s(PROM_READ_C5, &(coefficient[4]));	//C5: T_REF u16
		i2c_ms5611_read_calibration_s(PROM_READ_C6, &(coefficient[5]));	//C6: TEMPSENS u16
		
		//printf("MS5611获取内部校准系数:\n");
		//printf("C1: %d\n", coefficient[0]);
		//printf("C2: %d\n", coefficient[1]);
		//printf("C3: %d\n", coefficient[2]);
		//printf("C4: %d\n", coefficient[3]);
		//printf("C5: %d\n", coefficient[4]);
		//printf("C6: %d\n", coefficient[5]);
		
		i2c_ms5611_read_adc_s(MS5611_CONVD1_OSR4096, &digital_pressure);//D1: 压力u32
		i2c_ms5611_read_adc_s(MS5611_CONVD2_OSR4096, &digital_temp);    //D2: 温度u32
		
		
		// MS5611获取ADC压力温度值
		// printf("MS5611获取压力温度值:\n");
		// printf("digital pressure: %d\n", digital_pressure);
		// printf("digital temperature: %d\n", digital_temp);
		
		difference_temp = (int32_t)digital_temp - ((int32_t)coefficient[4] << 8); 									// 强制转化为32bit有符号运算
		
		// dT = D2 - T_REF = D2 - C5 * 2^8
		temperature = 2000 + (difference_temp * (int64_t)coefficient[5] >> 23);  										// 容量足够，>>优先级比* +都要小。10^9量级 32bit可能会溢出，故用int64
		// 实际温度：TEMP = 20C + dT*TEMPSENS = 2000 + dT * C6/2^23
		//printf("MS5611  Temp  (Ces ): %.2f   \n", (float)temperature/100);
		
		offset = ((int64_t)coefficient[1] << 16) + ((int64_t)difference_temp * coefficient[3] >> 7 ); 				// int64_t与先>>7防止溢出
		
		// OFF = OFF_T1 + TCO * dT = C2 * 2^16 + (C4 * dT) / 2^7
		sensitivity = (((int64_t)coefficient[0]) << 15) + ((int64_t)coefficient[2] * difference_temp >> 8); 	// int64_t 与先>>会导致得到0；原理同上计算temperature
		// SENS = SENS_T1 + TCS * dT = C1 * 2^15 + (C3 * dT) / 2^8
		
		pressure = (((int64_t)digital_pressure * sensitivity >> 21) - offset) >> 15; 								// 容量足够 -优先级大于>>，加括号防止警告
		// P = D1 * SENS - OFF = (D1 * SENS / 2^21 - OFF) / 2^15 
    //printf("MS5611  Press (mbar): %.2f  \n", (float)pressure/10);
		
		return pressure;
	
}

 










/**
 *
 *  名称： i2c_ms5611_send_cmd
 *
 *  描述： ms5611发送指令
 *
 */
u8 i2c_ms5611_send_cmd_s(u8 cmd)  
{
	u32 i2c_wait_timeout = I2C_WAIT_TIMEOUT; 
	
	ms5611_i2c_start_s();																//1 产生起始信号	
	
	ms5611_i2c_send_byte_s(MS5611_SLAVE_WRITE_ADDRESS); //2 发送7位从机设备地址，并检查是否收到地址应答
	
	while(ms5611_i2c_wait_ack_s())
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback_s("MS5611", 1);
		}
		i2c_wait_timeout--;
	}
	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	ms5611_i2c_send_byte_s(cmd);												//3 发送一个字节指令
	
	while(ms5611_i2c_wait_ack_s())
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback_s("MS5611", 2);
		}
		i2c_wait_timeout--;
	}

	ms5611_i2c_stop_s(); 																// 4 终止信号
	return SUCCESS;
}








/**
 * 名称: i2c_ms5611_receive_data_s
 *
 * 描述：ms5611接收数据信号
 *
 */

uint8_t i2c_ms5611_receive_data_s(uint8_t* buffer, uint8_t num)
{	
	uint32_t i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	ms5611_i2c_start_s();  																// 1 起始条件
	
	ms5611_i2c_send_byte_s(MS5611_SLAVE_READ_ADDRESS); 		// 2 发送从机设备地址
	while(ms5611_i2c_wait_ack_s())
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback_s("MS5611", 1);
		}
		i2c_wait_timeout--;
	}
	
	
	while(num)																						// 3 循环读取
	{
		if(num == 1)
		{
			*buffer = ms5611_i2c_read_byte_s(0);							// 4 最后一次收到发送Nack，并产生终止信号
			ms5611_i2c_stop_s(); 
		}
		else																  							// 4 接收到数据并发送Ack信号
    {      
      *buffer = ms5611_i2c_read_byte_s(1);
      buffer++; 
     }
		 num--;        

	}
	

	return SUCCESS;																				// 成功读出数据
}
	




