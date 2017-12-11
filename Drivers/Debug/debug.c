/**
 *
 * @file debug.c
 *
 * 使用usart1串口进行调试烧录程序
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
 * 名称: debug_usart_init
 *
 * 描述：串口初始化
 *
 */
void debug_usart_init(void)
{
	
	//gpio_clk_config();// 调试用
	//配置GPIO与串口外设
	usb_config();
	
	//打印一串字符串看是否正常输出
	debug_usart_check();
}
 




/**
 *
 * 名称：debug_usart_check
 *
 * 描述：串口校验
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
 * 名称：usart_debug_send_data
 *
 * 描述：串口发送16bit数组数据, num表示16bits个数
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
 * 名称：usart_debug_send_string
 *
 * 描述：串口发送8bit字符串
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
 * 名称：usart_debug_receive_buffer
 *
 * 描述：串口接收数据, num表示16bits个数
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
 * 名称：fputc
 *
 * 描述：重定向printf到串口
 *
 **/

int fputc(int ch, FILE *f)
{
		/* 发送一个字节数据到串口 */
		USART_SendData(DEBUG_USART, (uint8_t) ch);
		
		/* 等待发送完毕 */
		while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}



/**
 *
 * 名称：fgetc
 *
 * 描述：重定向scanf getchar到串口
 *
 **/


int fgetc(FILE *f)
{
		/* 等待串口输入数据 */
		while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(DEBUG_USART);
}











