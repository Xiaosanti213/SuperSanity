/**
 *
 * @file api_usart.h
 *
 *
 *
 */
 
 
 
 
 
#ifndef __API_USART_H
#define __API_USART_H



#include "stm32f10x.h"


	
#define				USART_WAIT_TIMEOUT				100






uint8_t usart_send_byte(USART_TypeDef* USARTx, uint8_t Data);
uint8_t usart_receive_byte(USART_TypeDef* USARTx, uint8_t* Data);
uint8_t usart_timeout_usercallback(uint8_t error_code);




#endif



