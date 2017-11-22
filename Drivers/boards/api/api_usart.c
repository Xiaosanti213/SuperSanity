/**
 *
 * @file api_usart.c
 *
 * ���ڵ���
 *
 */
 
 
#include "api_usart.h"

#include "stm32f10x_usart.h"
#include "stm32f10x.h"

#include <stdio.h>




 
 
/**
 *
 * ���ƣ�usart_send_byte
 *
 * ����������һ���ֽ�����
 *
 */
uint8_t usart_send_byte(USART_TypeDef* USARTx, uint8_t Data)
{
	
	uint8_t usart_wait_timeout = USART_WAIT_TIMEOUT;
	USART_SendData(USARTx, Data);
	// �ڿ⺯���У��β�Data��uint16_t �������ʵ����uint8_t��ʼ��ûӰ��
	
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
 * ���ƣ�usart_receive_byte
 *
 * ����������һ���ֽ�����
 *
 */

uint8_t usart_receive_byte(USART_TypeDef* USARTx, uint8_t* Data)
{
	
	uint8_t	usart_wait_timeout = USART_WAIT_TIMEOUT;
	*Data = USART_ReceiveData(USARTx);
	// �ڿ⺯���У��β�Data��uint16_t �������ʵ����uint8_t��ʼ��ûӰ��
	
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
 * ���ƣ�usart_timeout_usercallback
 *
 * 
 * �������ȴ���ʱ�ص�����
 *
 **/

uint8_t usart_timeout_usercallback(uint8_t error_code)
{
	printf("usart �ȴ���ʱ��errorCode =%d\n", error_code);
	return ERROR;
}









