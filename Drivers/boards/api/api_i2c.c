/**
 * @file api_i2c.c
 *
 * 软件模拟i2c通信协议
 * 
 */
 

#include <stm32f10x.h>
#include "board_config.h"
#include "api_i2c.h"
#include "app_cfg.h"
#include <stdio.h> 



//static void i2c_delay_s(void);
static void mpu6050_i2c_ack_s(void);
static void mpu6050_i2c_nack_s(void);





/**
 *
 *  名称： i2c_delay
 *
 *  描述： I2C总线延迟
 *
 */
//void i2c_delay_s(void)
//{
//	uint8_t i;

	/*
	  代码来源：秉火F103挑战者　
	 	下面的时间是通过安富莱AX-Pro逻辑分析仪测试得到的。
		CPU主频72MHz时，在内部Flash运行, MDK工程不优化
		循环次数为10时，SCL频率 = 205KHz 
		循环次数为7时，SCL频率 = 347KHz， SCL高电平时间1.5us，SCL低电平时间2.87us 
	 	循环次数为5时，SCL频率 = 421KHz， SCL高电平时间1.25us，SCL低电平时间2.375us 
        
    IAR工程编译效率高，不能设置为7
	*/
//	for (i = 0; i < 10; i++);
//}









/**
 *
 *  名称： mpu6050_i2c_start_s
 *
 *  描述： I2C启动信号
 *
 */
void mpu6050_i2c_start_s(void)
{
	// 当SCL高电平时，SDA出现一个下跳沿表示I2C总线启动信号
	MPU6050_I2C_SDA_1_FUN();
	MPU6050_I2C_SCL_1_FUN();
	delay_us(4);
	MPU6050_I2C_SDA_0_FUN();
	delay_us(4); 							//这样岂不是时钟线又多延时了一会？这个应该是等待数据被接收
	MPU6050_I2C_SCL_0_FUN();
	//delay_us(4);
}






/**
 *
 *  名称： mpu6050_i2c_stop_s
 *
 *  描述： I2C停止信号(外部链接性)
 *
 */
void mpu6050_i2c_stop_s(void)
{
	// 当SCL高电平时，SDA出现一个上跳沿表示I2C总线停止信号
	MPU6050_I2C_SCL_0_FUN();
	MPU6050_I2C_SDA_0_FUN();
	delay_us(4);
	MPU6050_I2C_SCL_1_FUN();
  MPU6050_I2C_SDA_1_FUN();//这个时间貌似要求并不是很严格?
	delay_us(4); 
}





/**
 *
 *  名称： mpu6050_i2c_send_byte
 *
 *  描述： I2C发送一字节数据
 *
 */
void mpu6050_i2c_send_byte_s(uint8_t byte)
{
	uint8_t i;

	/* 先发送字节的高位bit7 */
	for (i = 0; i < 8; i++)
	{		
		if (byte & 0x80)									//逐次取最高位
		{
			MPU6050_I2C_SDA_1_FUN();				//最高位是1则数据线最高位发送1
		}
		else
		{
			MPU6050_I2C_SDA_0_FUN();
		}
		delay_us(1);  										//这块这个貌似不需要，是为了避免干扰吗？
		MPU6050_I2C_SCL_1_FUN();
		delay_us(1);											//时钟线为高，数据采集
		MPU6050_I2C_SCL_0_FUN();  				//时钟线为低，数据切换
		if (i == 7)
		{
			 MPU6050_I2C_SDA_1_FUN(); 			// 释放总线，从机拉低则为应答
		}
		byte <<= 1;											  // 将下一个将要发送的位作为最高位
		delay_us(1); 										  // 保持时钟线低电平一段时间
	}
}






/**
 *
 *  名称： mpu6050_i2c_read_byte_s
 *
 *  描述： I2C接收一字节数据
 *
 */
uint8_t mpu6050_i2c_read_byte_s(u8 ack)
{
	uint8_t i;
	uint8_t value;

	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;  							 // 每次接收到的电平数据左移一位
		MPU6050_I2C_SCL_1_FUN();  	 // 时钟线高电平时读取数据
		delay_us(1);
		if (MPU6050_I2C_SDA_READ_FUN())
		{
			value++;
		}
		MPU6050_I2C_SCL_0_FUN(); 
		delay_us(1); 
	}
	if(ack==0)    								 // 循环结束，根据实参判断等待应答或非应答信号
		mpu6050_i2c_nack_s();  			 // 主机读取结束，形参为0，产生非应答信号
	else
		mpu6050_i2c_ack_s();   			 // 形参为1，产生应答信号
	return value;
}






/**
 *
 *  名称： mpu6050_i2c_wait_ack
 *
 *  描述： 等待应答
 *
 */
uint8_t mpu6050_i2c_wait_ack_s(void)
{
	uint8_t re;

	MPU6050_I2C_SDA_1_FUN();	// CPU释放SDA总线
	delay_us(1);
	MPU6050_I2C_SCL_1_FUN();	// CPU驱动SCL = 1, 此时器件会返回ACK应答
	delay_us(1);
	if (MPU6050_I2C_SDA_READ_FUN())	// 高电平时CPU读取SDA口线状态
	{
		re = 1;    // 接收到该电平信号--没有应答 返回1
	}
	else
	{
		re = 0;
	}
	MPU6050_I2C_SCL_0_FUN(); // 时钟线置准备下一次数据传输
	delay_us(1);
	return re;
}






/**
 *
 *  名称： mpu6050_i2c_ack
 *
 *  描述： 主机发送应答信号
 *
 */
void mpu6050_i2c_ack_s(void)
{
	MPU6050_I2C_SDA_0_FUN();	// CPU驱动SDA = 0  Ack信号
	MPU6050_I2C_SCL_1_FUN();
	delay_us(1);
	MPU6050_I2C_SCL_0_FUN();	// CPU产生1个时钟
	delay_us(1);
	MPU6050_I2C_SDA_1_FUN();
	delay_us(1);
}







/**
 *
 *  名称： mpu6050_i2c_nack_s
 *
 *  描述： 主机发送非应答信号
 *
 */
void mpu6050_i2c_nack_s(void)
{
	MPU6050_I2C_SCL_0_FUN();
	MPU6050_I2C_SDA_1_FUN();	// CPU驱动SDA = 1  Nack信号
	delay_us(1);
	MPU6050_I2C_SCL_1_FUN();	// CPU产生1个时钟 
	delay_us(1);
	MPU6050_I2C_SCL_0_FUN();
	delay_us(1);
}









/**
 * 名称: i2c_timeout_usercallback
 *
 * 描述：等待超时回调函数
 *
 */
u8 i2c_timeout_usercallback_s(char* name, uint8_t error_code)
{
	printf("%s I2C等待超时！error_code =%d\n", name, error_code);
	return 0;
}
