/**
 *
 * @file debug.c
 *
 * 使用usart1串口进行调试烧录程序
 *
 **/


#include "debug.h"
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
	printf("Debug-usart1 is working\n");
	//显示输出则串口正常工作
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
		/* 库函数发送一个字节数据到串口 */
		USART_SendData(DEBUG_USART, (uint8_t) ch);
		
		/* 等待发送完毕 */
		while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}







