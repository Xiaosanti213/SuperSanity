/**
 * @file api_i2c.c
 *
 * i2c�ӿں�������
 * 
 */
 
#include "stm32f10x.h"
#include "api_i2c.h"
#include "board_config.h"

#include <stdio.h>

 
 
/**
 * ����: i2c_send_data
 *
 * ������7bit��ַģʽ i2c�ӿڷ���һ���ֽ�����
 *
 */
 
 
 
 
/********************************* Defines ************************************/

#define WAIT_FOR_FLAG(flag, value, timeout, errorcode)  \
          while(I2C_GetFlagStatus(MPU6050_I2C, flag) != value) {\
            if((timeout--) == 0) return i2c_timeout_usercallback(errorcode); \
          }\
  
#define CLEAR_ADDR_BIT      I2C_ReadRegister(MPU6050_I2C, I2C_Register_SR1);\
                            I2C_ReadRegister(MPU6050_I2C, I2C_Register_SR2);\
/*******************************************************************************/
       
					
					
	 
uint8_t i2c_send_data(I2C_TypeDef* I2Cx, uint8_t device_address, uint8_t byte)
{
	
	uint8_t i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	I2C_GenerateSTART(I2Cx, ENABLE);
	// ������ʼ�ź�
	while(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT) == ERROR)
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(0);
		}
		i2c_wait_timeout--;
	}// ��ʼ�����ɹ�����
	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	I2C_Send7bitAddress(I2Cx, device_address, I2C_Direction_Transmitter);
	// ����7λ�ӻ��豸��ַ��������Ƿ��յ���ַӦ��
	
	  /* Wait for the start bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_ADDR, SET, i2c_wait_timeout, 9);

  /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
  CLEAR_ADDR_BIT;
  
  /* Wait for address bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_TXE, SET, i2c_wait_timeout, 10);
	
	
	while(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == ERROR)
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(1);
		}
		i2c_wait_timeout--;
	}// �յ���ַӦ��
	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	I2C_SendData(I2Cx, byte);
	while(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR)
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(2);
		}
		i2c_wait_timeout--;
	}
	return 0;
}




/**
 * ����: i2c_receive_data
 *
 * ������7bit��ַģʽ i2c�ӿڽ���һ���ֽ�����
 *
 */
uint8_t i2c_receive_data(I2C_TypeDef* I2Cx, uint8_t device_address, uint8_t byte)
{
	
	uint8_t i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	I2C_GenerateSTART(I2Cx, ENABLE);
	while(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT) == ERROR)
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(3);
		}
		i2c_wait_timeout--;
	}
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	I2C_Send7bitAddress(I2Cx, device_address, I2C_Direction_Receiver);
	while(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == ERROR)
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(4);
		}
		i2c_wait_timeout--;
	}
  
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	byte = I2C_ReceiveData(I2Cx);
	while(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) == ERROR)
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(5);
		}
		i2c_wait_timeout--;
	}
	return byte;
}
	



/**
 * ����: i2c_timeout_usercallback
 *
 * �������ȴ���ʱ�ص�����
 *
 */
uint8_t i2c_timeout_usercallback(uint8_t error_code)
{
	printf("I2C �ȴ���ʱ��errorCode =%d\n", error_code);
	return 0;
}



//int fputc(int ch, FILE* f)
//{
//	USART_SendData(DEBUG_USART, (uint8_t) ch);
//	while (USART_GetFlagStatus(DEBUG_USART, DEBUG_FLAG_TXE) == RESET
	
//}












