/**
 *
 * @file api_usart.c
 *
 * 串口调试
 *
 */
 
 
#include "api_usart.h"

#include "stm32f10x_usart.h"
#include "stm32f10x.h"

#include <stdio.h>




 
 
/**
 *
 * 名称：usart_send_byte
 *
 * 描述：发送一个字节数据
 *
 */
uint8_t usart_send_byte(USART_TypeDef* USARTx, uint8_t Data)
{
	
	uint8_t usart_wait_timeout = USART_WAIT_TIMEOUT;
	USART_SendData(USARTx, Data);
	// 在库函数中，形参Data是uint16_t 但是这块实参用uint8_t初始化没影响
	
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
	{
		if(usart_wait_timeout == 0)
		{
		  usart_timeout_usercallback(0);
		  return ERROR;
		}	
		usart_wait_timeout--;		
	}
	return SUCCESS;
	
}






/**
 *
 * 名称：usart_receive_byte
 *
 * 描述：接收一个字节数据
 *
 */

uint8_t usart_receive_byte(USART_TypeDef* USARTx, uint8_t* Data)
{
	
	uint8_t	usart_wait_timeout = USART_WAIT_TIMEOUT;
	*Data = USART_ReceiveData(USARTx);
	// 在库函数中，形参Data是uint16_t 但是这块实参用uint8_t初始化没影响
	
	while(USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) == RESET)
	{
		if(usart_wait_timeout == 0)
		{
			return usart_timeout_usercallback(1);
		}
			usart_wait_timeout --;
	}
	return SUCCESS;
}









/**
 *
 * 名称：usart_timeout_usercallback
 *
 * 
 * 描述：等待超时回调函数
 *
 **/

uint8_t usart_timeout_usercallback(uint8_t error_code)
{
	printf("usart 等待超时！errorCode =%d\n", error_code);
	return ERROR;
}









