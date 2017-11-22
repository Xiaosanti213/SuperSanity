/**
 * @file api_i2c.h
 * 
 * i2c接口函数声明 宏定义
 *
 */
 
 
#ifndef __API_SPI_H
#define __API_SPI_H
	
#include "stm32f10x.h"
	
#define I2C_WAIT_TIMEOUT		100



uint8_t i2c_send_data(I2C_TypeDef* I2Cx, uint8_t device_address, uint8_t byte);
uint8_t i2c_receive_data(I2C_TypeDef* I2Cx, uint8_t device_address, uint8_t byte);
uint8_t i2c_timeout_usercallback(uint8_t error_code);



#endif

