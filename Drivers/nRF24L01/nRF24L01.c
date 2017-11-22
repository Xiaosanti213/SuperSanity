/**
 * @file nRF24L01.c
 *
 * nRF24L01 无线收发模块操作函数
 * 
 */

#include "nRF24L01.h"

#include "api_spi.h"
#include "board_config.h"

#include "stm32f10x.h"

#include <stdio.h>
#include "api_usart.h"






/**
  * 名称：spi_nrf_init
  *  
  * 描述：nRF24L01设备初始化
  *
  */
void spi_nrf_init(void)
{
	gpio_clk_config(); //调试用
	
	nrf24l01_config(); 
	// 不配置亦不传输
	RC_SPI_CE_LOW_FUN();
	RC_SPI_NSS_HIGH_FUN();
}









/**
  * 名称：spi_nrf_check
  *
  * 描述：检查nRF24L01设备是否连接 
  *
  */
uint8_t spi_nrf_check(void)
{
   
	 uint8_t index;
	 //uint8_t status;
   uint8_t check_data[5] = {0x14, 0x14, 0x14, 0x14, 0x14};
	 uint8_t check_read[5] = {0x07, 0x07, 0x07, 0x07, 0x07};
	 //校验数据
	 
	 spi_send_byte(RC_SPI, TX_ADDR);
	 // 选择准备写入的寄存器
	 spi_nrf_write_buffer(WRITE_REG_NRF, check_data, 5);
	 // 写入寄存器数据
	 
	 spi_send_byte(RC_SPI, TX_ADDR);
	 // 选择准备读出数据的寄存器
	 spi_nrf_read_buffer(READ_REG_NRF, check_read, 5);
	 // 读出刚写入TX_ADDR寄存器中的数组
	 
   for(index = 0; index<5; index++)
	 {
		  printf("\r\n读出TX寄存器数据为：%8d", check_read[index]);
			//break;
	 }
	 if(index == 5)
	 {
	    return SUCCESS;
	 }
	 return ERROR;

}









/**
  * 名称：spi_nrf_reg_write
  *  
  * 描述：配置nRF24L01设备寄存器
  *
  */
  
uint8_t spi_nrf_reg_write(uint8_t reg, uint8_t value)
{
	
	uint8_t status;
	// 开始SPI传输
	RC_SPI_NSS_LOW_FUN() ;
	
	// 返回寄存器状态值
	status = spi_send_byte(RC_SPI, reg);
	spi_send_byte(RC_SPI, value);
	
	RC_SPI_NSS_HIGH_FUN() ;
	return status;
	
}








/**
  * 名称：spi_nrf_reg_read
  *  
  * 描述：配置命令，读取寄存器的值
  *
  */

uint8_t spi_nrf_reg_read(uint8_t reg )
{
	uint8_t reg_val;
	
	RC_SPI_NSS_LOW_FUN();
	
	// 配置寄存器可以得到状态，但是无法返回寄存器中的值
	spi_send_byte(RC_SPI, reg);
	reg_val = spi_send_byte(RC_SPI, READ_REG_NRF);

  RC_SPI_NSS_HIGH_FUN();
	
	return reg_val;

}








/**
  * 名称：spi_nrf_read_buffer
  *  
  * 描述：读取若干字节数据
  *
  */


uint8_t spi_nrf_read_buffer(uint8_t reg, uint8_t* pBuf, uint8_t bytes)
{
	uint8_t status, byte_ctrl; 
	
	RC_SPI_NSS_LOW_FUN();
	
	status = spi_send_byte(RC_SPI, reg);//读出寄存器状态
	
	for (byte_ctrl = 0; byte_ctrl<bytes; byte_ctrl++)
	{
			pBuf[byte_ctrl] = spi_send_byte(RC_SPI, READ_REG_NRF);
	}
	
	RC_SPI_NSS_HIGH_FUN();

  return status;
		
}








/**
  * 名称：spi_nrf_write_buffer
  *  
  * 描述：写入若干字节数据
  *
  */


uint8_t spi_nrf_write_buffer(uint8_t reg, uint8_t* pBuf, uint8_t bytes)
{
	uint8_t byte_ctrl, status; 
	
	RC_SPI_NSS_LOW_FUN();//开始数据传输
	
	status = spi_send_byte(RC_SPI, reg);
	
	for (byte_ctrl = 0; byte_ctrl < bytes; byte_ctrl++)
	{
			spi_send_byte(RC_SPI, pBuf[byte_ctrl]);
	}
	
	RC_SPI_NSS_HIGH_FUN();//使能芯片结束访问

  return status;
		
}









/**
  * 名称：spi_nrf_rx_mode
  *  
  * 描述：初始化NRF24L01为RX模式
  *
  */

void spi_nrf_rx_mode(void)
{
	  RC_SPI_CE_LOW_FUN();
	  //CE为低,进入配置模式 
  
  	spi_nrf_write_buffer(WRITE_REG_NRF+RX_ADDR_P0, (uint8_t *)RX_ADDRESS, RX_ADR_WIDTH);
	  //写RX节点地址
	
	  spi_nrf_reg_write(WRITE_REG_NRF+EN_AA, ENAA_P0);    
	  //使能通道0的自动应答
	 	spi_nrf_reg_write(WRITE_REG_NRF+EN_RXADDR, ERX_P0);
	  //使能通道0的接收地址  	 
  	spi_nrf_reg_write(WRITE_REG_NRF+RF_CH,40);	     
	  //设置RF通信频率40Hz
  	spi_nrf_reg_write(WRITE_REG_NRF+RX_PW_P0,RX_PLOAD_WIDTH);
	  //选择通道0的有效数据宽度 	    
  	spi_nrf_reg_write(WRITE_REG_NRF+RF_SETUP, RF_DR_HIGH | RF_PWR_0dBm);
	  //设置发射参数,0db增益,2Mbps   
  	spi_nrf_reg_write(WRITE_REG_NRF+CONFIG, EN_CRC | CRCO_2Byte | PWR_UP | PRIM_RX);
	  //配置基本工作模式的参数; PWR_UP, 使能CRC校验, 16位CRC编码, 接收模式，开启IRQ全部中断
	
  	RC_SPI_CE_HIGH_FUN(); 
	  //CE为高,进入工作模式(接收) 
}		



/**
  * 名称：spi_nrf_tx_mode
  *  
  * 描述：初始化NRF24L01为TX模式
  *
  */	 
void spi_nrf_tx_mode(void)
{														 
	  RC_SPI_CE_LOW_FUN();    
	
  	spi_nrf_write_buffer(WRITE_REG_NRF+TX_ADDR, (u8*)TX_ADDRESS, TX_ADR_WIDTH);
	  //写TX节点地址 
  	spi_nrf_write_buffer(WRITE_REG_NRF+RX_ADDR_P0, (u8*)RX_ADDRESS, RX_ADR_WIDTH); 
	  //设置pipe0节点地址, 用来接收ACK信号	  

  	spi_nrf_reg_write(WRITE_REG_NRF+EN_AA, ENAA_P0);     
	  //使能通道0的自动应答    
  	spi_nrf_reg_write(WRITE_REG_NRF+EN_RXADDR,ERX_P0); 
	  //使能通道0的接收地址  
  	spi_nrf_reg_write(WRITE_REG_NRF+SETUP_RETR,ARD_WAIT_500US | ARC_RETRANSMIT_10);
	  //设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
  	spi_nrf_reg_write(WRITE_REG_NRF+RF_CH,40);       
	  //设置RF通信频率为40Hz
  	spi_nrf_reg_write(WRITE_REG_NRF+RF_SETUP, RF_DR_HIGH | RF_PWR_0dBm);  
	  //设置发射参数,0db增益,2Mbps 
  	spi_nrf_reg_write(WRITE_REG_NRF+CONFIG, EN_CRC | CRCO_2Byte | PWR_UP );    
	  //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
	
	  RC_SPI_CE_HIGH_FUN(); 
		//CE为高,10us后启动发送
}		  





/**
  * 名称：spi_nrf_tx_packet
  *  
  * 描述：通过nRF24L01发送数据
  *
  */

uint8_t spi_nrf_tx_packet(uint8_t *txbuffer)
{
	uint8_t tx_status;
	
	RC_SPI_CE_LOW_FUN();
	//写入数据配置
	spi_nrf_write_buffer(WRITE_REG_NRF + WR_TX_PAYLOAD, txbuffer, TX_PLOAD_WIDTH);
	//写txbuffer中的数据到WR_TX_PAYLOAD寄存器, txbuffer是一组数据的首地址，自动读取相应32Byte数据
	RC_SPI_CE_HIGH_FUN();
	//启动发送	 
  
	while(RC_SPI_INT_SCAN_FUN()!=RC_SPI_INT_LOW)
	{
		;
	}//等待发送完成
	
	tx_status = spi_nrf_reg_read(READ_REG_NRF + STATUS);  
	//读取状态寄存器的值	
	spi_nrf_reg_write(WRITE_REG_NRF + STATUS, tx_status); 
	//清除TX_DS或MAX_RT中断标志
	
	if(tx_status & MAX_RT)//达到最大重发次数
	{
		spi_nrf_reg_write(FLUSH_TX, 0xff);
		//清除TX FIFO寄存器 
		
		return MAX_RT; 
	}
	
	if(tx_status & TX_DS)
		//发送完成
	{
		return SUCCESS;
	}
	return ERROR;
	//发送失败
}







/**
  * 名称：spi_nrf_rx_packet
  *  
  * 描述：通过nRF24L01接收数据
  *
  */

uint8_t spi_nrf_rx_packet(uint8_t *rxbuf)
{
	uint8_t rx_status;		    
	
	rx_status = spi_nrf_reg_read(STATUS);  
	//读取状态寄存器的值    	 
	spi_nrf_reg_write(WRITE_REG_NRF+STATUS, rx_status); 
	//清除RX_DR中断标志
	
	
	if( rx_status & RX_DR )//接收到数据
	{ 
		spi_nrf_write_buffer(READ_REG_NRF + RD_RX_PAYLOAD, rxbuf, RX_PLOAD_WIDTH);
		//读取数据
		spi_nrf_reg_write(FLUSH_RX, 0xff);
		//清除RX FIFO寄存器 
		return SUCCESS; 
	}	   
	return ERROR;//没收到任何数据
}		
