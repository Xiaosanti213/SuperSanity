/**
 * @file api_i2c.h
 *
 * 软件模拟i2c通信协议函数原型
 * 
 */
#ifndef  _API_I2C_H
#define _API_I2C_H
 
 
 #include <stm32f10x.h>
 
 
 
 
 void mpu6050_i2c_start_s(void);
 void mpu6050_i2c_stop_s(void); 
 


 void mpu6050_i2c_send_byte_s(uint8_t byte);
 uint8_t mpu6050_i2c_read_byte_s(u8 ack);
	 
 uint8_t mpu6050_i2c_wait_ack_s(void);
 
 
 
 void ms5611_i2c_start_s(void);
 void ms5611_i2c_stop_s(void);
 
 void ms5611_i2c_send_byte_s(uint8_t byte);
 uint8_t ms5611_i2c_read_byte_s(u8 ack);

 
 uint8_t ms5611_i2c_wait_ack_s(void);
 
 
 #endif
 
 
