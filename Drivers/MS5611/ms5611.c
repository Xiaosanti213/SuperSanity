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


//static void i2c_ms5611_init(void);
//static u8 i2c_ms5611_check(void);
static void i2c_ms5611_read_calibration(uint8_t cx, uint16_t* cx_buffer);
static void i2c_ms5611_read_adc(uint8_t config, uint32_t* adc_buffer);
static void i2c_ms5611_delay(u16); 


static u8 i2c_ms5611_send_cmd(uint8_t cmd); 
static u8 i2c_ms5611_timeout_usercallback(u8 error_code);
static uint8_t i2c_ms5611_receive_data(uint8_t* buffer, uint8_t num);


#define 			I2C_WAIT_TIMEOUT 			((u32)0x1000)



/**
 *  名称: i2c_ms5611_init
 *
 *  描述：ms5611设备复位初始化
 *
 */
void i2c_ms5611_init(void)
{
	ms5611_config();																	//ms5611引脚和片上外设配置
  i2c_ms5611_delay(1000); 													//ms5611上电延时
	
	i2c_ms5611_send_cmd(MS5611_RESET);	    					//写入传感器复位指令
  i2c_ms5611_delay(1000); 
}





/**
 *  名称: i2c_ms5611_check
 *
 *  描述：ms5611设备检测连接状况
 *
 */
u8 i2c_ms5611_check(void)
{
	return SUCCESS; 
}
	









/**
 * 
 * 名称：i2c_ms5611_read_calibration
 *
 * 描述：读出PROM 16bit校准字
 *
 */
void i2c_ms5611_read_calibration(uint8_t cx, uint16_t* cx_buffer)
{
	uint8_t cx_pre_get[2] = {1, 1}; 																	// 分别获取低8bit 高8bit
	i2c_ms5611_send_cmd(cx);																					// 发送指令	
	i2c_ms5611_receive_data(cx_pre_get, 2);
	*cx_buffer = ((u16)cx_pre_get[0] << 8) | ((u16)cx_pre_get[1]);    // 整合到u16当中
}








/**
 * 
 * 名称：i2c_ms5611_read_adc
 *
 * 描述：读出ADC 24bit温度,压力数据
 *
 */
void i2c_ms5611_read_adc(uint8_t config, uint32_t* adc_buffer)//形参一个字节，传参需要
{
	u8 adc_pre[3]; 
	i2c_ms5611_send_cmd(config);																		// 发送配置指令D1或D2以及OSR
	i2c_ms5611_delay(300);																					// 等待转换结束，经过测试，延时最短时间100~300
	i2c_ms5611_send_cmd(MS5611_ADC_READ); 													// 发送MS5611_ADC_READ指令
	i2c_ms5611_receive_data(adc_pre, 3);
	*adc_buffer = (adc_pre[0] << 16) | (adc_pre[1] << 8) | (adc_pre[2]); 
	//printf("adcx: %d\t\n", *adc_buffer);
}














/**
 * 
 * 名称：i2c_ms5611_calculate
 *
 * 描述：整合传感器数据，获得气压
 *
 */
int32_t i2c_ms5611_calculate(void)
{
	int32_t difference_temp, temperature, pressure;
	int64_t offset, sensitivity;
	u16 coefficient[6];
	u32 digital_pressure, digital_temp;
	
	i2c_ms5611_init();
	if(i2c_ms5611_check())										// 连接状况良好
	{
		i2c_ms5611_read_calibration(PROM_READ_C1, &(coefficient[0]));	//C1: SENS_T1 u16
		i2c_ms5611_read_calibration(PROM_READ_C2, &(coefficient[1]));	//C2: OFF_T1 u16						
		i2c_ms5611_read_calibration(PROM_READ_C3, &(coefficient[2]));	//C3: TCS u16
		i2c_ms5611_read_calibration(PROM_READ_C4, &(coefficient[3]));	//C4: TCO u16
		i2c_ms5611_read_calibration(PROM_READ_C5, &(coefficient[4]));	//C5: T_REF u16
		i2c_ms5611_read_calibration(PROM_READ_C6, &(coefficient[5]));	//C6: TEMPSENS u16
		
		// printf("MS5611获取内部校准系数:\n");
		// printf("C1: %d\n", coefficient[0]);
		// printf("C2: %d\n", coefficient[1]);
		// printf("C3: %d\n", coefficient[2]);
		// printf("C4: %d\n", coefficient[3]);
		// printf("C5: %d\n", coefficient[4]);
		// printf("C6: %d\n", coefficient[5]);
		
		i2c_ms5611_read_adc(MS5611_CONVD1_OSR4096, &digital_pressure);//D1: 压力u32
		i2c_ms5611_read_adc(MS5611_CONVD2_OSR4096, &digital_temp);    //D2: 温度u32
		
		
		// MS5611获取ADC压力温度值
		// printf("MS5611获取压力温度值:\n");
		// printf("digital pressure: %d\n", digital_pressure);
		// printf("digital temperature: %d\n", digital_temp);
		
		difference_temp = (int32_t)digital_temp - ((int32_t)coefficient[4] << 8); 									// 强制转化为32bit有符号运算
		
		// dT = D2 - T_REF = D2 - C5 * 2^8
		temperature = 2000 + (difference_temp * (int64_t)coefficient[5] >> 23);  										// 容量足够，>>优先级比* +都要小。10^9量级 32bit可能会溢出，故用int64
		// 实际温度：TEMP = 20C + dT*TEMPSENS = 2000 + dT * C6/2^23
		printf("ms5611温度：%.2fC\n", (float)temperature/100);
		
		offset = ((int64_t)coefficient[1] << 16) + ((int64_t)difference_temp * coefficient[3] >> 7 ); 				// int64_t与先>>7防止溢出
		
		// OFF = OFF_T1 + TCO * dT = C2 * 2^16 + (C4 * dT) / 2^7
		sensitivity = (((int64_t)coefficient[0]) << 15) + ((int64_t)coefficient[2] * difference_temp >> 8); 	// int64_t 与先>>会导致得到0；原理同上计算temperature
		// SENS = SENS_T1 + TCS * dT = C1 * 2^15 + (C3 * dT) / 2^8
		
		pressure = (((int64_t)digital_pressure * sensitivity >> 21) - offset) >> 15; 								// 容量足够 -优先级大于>>，加括号防止警告
		// P = D1 * SENS - OFF = (D1 * SENS / 2^21 - OFF) / 2^15 
    printf("压力：%.2fmbar\n", (float)pressure/10);
		
		return pressure;
	}
	
	return 0; 												 												 												 						// 失败
}









	
/**
 *
 *  名称： i2c_ms5611_delay
 *
 *  描述： ms5611上电延时
 *
 */
void i2c_ms5611_delay(u16 time_to_delay)
{
	uint16_t i=0,j=0;
  //在初始化之前要延时一段时间，若没有延时，则断电后再上电数据可能会出错
  for(i=0; i < time_to_delay; i++)
  {
    for(j=0; j < time_to_delay; j++)
    {
      ;
    }
  }
}	











/**
 *
 *  名称： i2c_ms5611_send_cmd
 *
 *  描述： ms5611发送指令
 *
 */
u8 i2c_ms5611_send_cmd(u8 cmd)  
{
		u32 i2c_wait_timeout = I2C_WAIT_TIMEOUT; 
	
	//while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY)) // 检测是否正忙
  //{
  //  if((i2c_wait_timeout--) == 0) return i2c_timeout_usercallback(7);
  // }

	
	I2C_GenerateSTART(MS5611_I2C, ENABLE);//1 产生起始信号	
	
	while(!I2C_CheckEvent(MS5611_I2C, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_ms5611_timeout_usercallback(0);
		}
		i2c_wait_timeout--;
	}
	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	I2C_Send7bitAddress(MS5611_I2C, MS5611_SLAVE_ADDRESS, I2C_Direction_Transmitter); //2 发送7位从机设备地址，并检查是否收到地址应答
	
	while(!I2C_CheckEvent(MS5611_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_ms5611_timeout_usercallback(1);
		}
		i2c_wait_timeout--;
	}
	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	I2C_SendData(MS5611_I2C, cmd);		//3 发送一个字节指令
	
	while(!I2C_CheckEvent(MS5611_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_ms5611_timeout_usercallback(2);
		}
		i2c_wait_timeout--;
	}

	I2C_GenerateSTOP(MS5611_I2C, ENABLE); // 4 终止信号
	return SUCCESS;
}








/**
 * 名称: i2c_receive_data
 *
 * 描述：Burst Read Sequence 多字节接收模式
 *
 */

uint8_t i2c_ms5611_receive_data(uint8_t* buffer, uint8_t num)
{	
	uint32_t i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	while(I2C_GetFlagStatus(MPU6050_I2C, I2C_FLAG_BUSY)) // 检测是否正忙  
  {
    if((i2c_wait_timeout--) == 0) return i2c_ms5611_timeout_usercallback(0);
  }
	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	
	I2C_GenerateSTART(MS5611_I2C, ENABLE);  // 1 起始条件
	while(!I2C_CheckEvent(MS5611_I2C, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_ms5611_timeout_usercallback(3);
		}
		i2c_wait_timeout--;
	}
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT; 
	I2C_Send7bitAddress(MS5611_I2C, MS5611_SLAVE_ADDRESS, I2C_Direction_Receiver); // 2 发送从机设备地址
	while(!I2C_CheckEvent(MS5611_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_ms5611_timeout_usercallback(4);
		}
		i2c_wait_timeout--;
	}
	
	
	while(num)// 循环读取
	{
		if(num == 1)
		{
			I2C_AcknowledgeConfig(MS5611_I2C, DISABLE);//最后一次收到数据禁用Ack
			I2C_GenerateSTOP(MS5611_I2C, ENABLE); 
		}
		if(I2C_CheckEvent(MS5611_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))  //注意是接收到数据才读取，逻辑关系与是非关系
    {      
      /* Read a byte from the slave */
      *buffer = I2C_ReceiveData(MS5611_I2C);
			//读取的内容，可使用引用变量或指针，但是保证读出数据

      /* Point to the next location where the byte read will be saved */
      buffer++; 
      
      /* Decrement the read bytes counter */
      num--;        
    }
	}
	I2C_AcknowledgeConfig(MS5611_I2C, ENABLE);
	
	//这块尝试清空标志位
	I2C_Cmd(MPU6050_I2C, DISABLE);
	I2C_Cmd(MPU6050_I2C, ENABLE);
	return SUCCESS;//成功读出数据
}
	






/**
 *
 *  名称： i2c_timeout_usercallback
 *
 *  描述： ms5611发送指令
 *
 */
u8 i2c_ms5611_timeout_usercallback(u8 error_code)
{
	printf("MS5611 等待超时！errorCode =%d\n", error_code); 
	return 0;
}




