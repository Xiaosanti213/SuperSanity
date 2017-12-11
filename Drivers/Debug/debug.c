/**
 *
 * @file debug.c
 *
 * ʹ��usart1���ڽ��е�����¼����
 *
 **/


#include "debug.h"
#include "api_usart.h"
#include "board_config.h"

#include "stm32f10x.h"
#include "stm32f10x_usart.h"

#include <stdio.h> 








/**
 *
 * ����: debug_usart_init
 *
 * ���������ڳ�ʼ��
 *
 */
void debug_usart_init(void)
{
	
	//gpio_clk_config();// ������
	//����GPIO�봮������
	usb_config();
	
	//��ӡһ���ַ������Ƿ��������
	debug_usart_check();
}
 




/**
 *
 * ���ƣ�debug_usart_check
 *
 * ����������У��
 *
 **/
void debug_usart_check(void)
{
	
	char check_data[]  = "Debug-usart1 is working\n" ;
	//usart_debug_send_string(check_data, sizeof(check_data));
	printf("%s", check_data);
}
 
 









/**
 *
 * ���ƣ�usart_debug_send_data
 *
 * ���������ڷ���16bit��������, num��ʾ16bits����
 *
 */
uint8_t usart_debug_send_data(uint8_t *buffer, uint8_t num) 
{
	uint8_t usart_wait_timeout = USART_WAIT_TIMEOUT;
	uint8_t index;
	
	for(index =0; index < num; index++)
	{
		usart_send_byte(DEBUG_USART, buffer[index]);
	}
	while(USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TC) == RESET)
	{
		if(usart_wait_timeout == 0)
		{
			usart_timeout_usercallback(2);
			return ERROR;
		}
		usart_wait_timeout--;
	}
	return SUCCESS;
}










/**
 *
 * ���ƣ�usart_debug_send_string
 *
 * ���������ڷ���8bit�ַ���
 *
 **/
uint8_t usart_debug_send_string(char* buffer_string, uint8_t num)
{
	uint8_t usart_wait_timeout = USART_WAIT_TIMEOUT;
	uint8_t index;
	
	for(index =0; index < num; index++)
	{
		usart_send_byte(DEBUG_USART, buffer_string[index]);
	}
	while(USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TC) == RESET)
	{
		if(usart_wait_timeout == 0)
		{
			usart_timeout_usercallback(2);
			return ERROR;
		}
		usart_wait_timeout--;
	}
	return SUCCESS;
}







/**
 *
 * ���ƣ�usart_debug_receive_buffer
 *
 * ���������ڽ�������, num��ʾ16bits����
 *
 */
uint8_t usart_debug_receive_buffer(uint8_t *buffer, uint8_t num)
{
	uint8_t index;
	
	for(index =0; index < num; index++)
	{
		usart_receive_byte(DEBUG_USART, buffer++);
	}
	return sizeof(*buffer);
}











/**
 *
 * ���ƣ�fputc
 *
 * �������ض���printf������
 *
 **/

int fputc(int ch, FILE *f)
{
		/* ����һ���ֽ����ݵ����� */
		USART_SendData(DEBUG_USART, (uint8_t) ch);
		
		/* �ȴ�������� */
		while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}



/**
 *
 * ���ƣ�fgetc
 *
 * �������ض���scanf getchar������
 *
 **/


int fgetc(FILE *f)
{
		/* �ȴ������������� */
		while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(DEBUG_USART);
}











