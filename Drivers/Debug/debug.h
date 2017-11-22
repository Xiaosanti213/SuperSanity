/**
 *
 * @file debug
 *
 * 串口调试头文件
 *
 **/
 
 
#ifndef __DEBUG_H
#define __DEBUG_H

#include "stm32f10x.h"


void debug_usart_init(void);
void debug_usart_check(void);
uint8_t usart_debug_send_data(uint8_t *buffer, uint8_t num);
uint8_t usart_debug_send_string(char* buffer_string, uint8_t num);
uint8_t usart_debug_receive_buffer(uint8_t *buffer, uint8_t num);


#endif




