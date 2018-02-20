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


uint8_t RX_ADDRESS[RX_ADR_WIDTH] = {0x34,0x43,0x10,0x10,0x01};
uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0x34,0x43,0x10,0x10,0x01}; 

uint8_t static spi_nrf_reg_read(uint8_t reg);
uint8_t static spi_nrf_reg_write(uint8_t reg, uint8_t value);

static uint8_t spi_nrf_write_buffer(uint8_t reg, uint8_t* pBuf, uint8_t bytes);
static uint8_t spi_nrf_read_buffer(uint8_t reg, uint8_t* pBuf, uint8_t bytes);

static int nrf_timeout_usercallback(u8 error_code);





/**
  * 名称：spi_nrf_init
  *  
  * 描述：nRF24L01设备初始化
  *
  */
void spi_nrf_init(void)
{	
	nrf24l01_config();	// 不配置亦不传输，CE失能外设，NSS失能SPI
	RC_SPI_NSS_HIGH_FUN();
	printf("Checking nrf... %d\n",spi_nrf_check());
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
   uint8_t check_data[5] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
	 uint8_t read_data[5];
	 //校验数据
	 RC_SPI_CE_LOW_FUN();
	 spi_nrf_write_buffer(WRITE_REG_NRF + TX_ADDR, check_data, 5);
	 //写入发送寄存器
	 spi_nrf_read_buffer(READ_REG_NRF + TX_ADDR, read_data, 5);
	 //读出刚写入TX_ADDR寄存器中的数组
	 RC_SPI_CE_HIGH_FUN();
   for(index = 0; index < 5; index++)
	 {
	    if (read_data[index] != 0xA5)
		  break;
	 }
	 
	 if(index == 5)
	 {
	    return SUCCESS;
	 }
	 return FAILURE;

}









/**
  * 名称：spi_nrf_rx_mode
  *  
  * 描述：初始化NRF24L01为RX模式
  *
  */


void spi_nrf_rx_mode(void)
{
	  //power_off();
	  //power down模式下参数配置
	
	  RC_SPI_CE_LOW_FUN();
	  //CE为低,进入配置模式 
  
  	spi_nrf_write_buffer(WRITE_REG_NRF+RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH);
	  //写RX节点地址
	  spi_nrf_reg_write(WRITE_REG_NRF+EN_AA, 0x01);    
	  //使能通道0的自动应答
	 	spi_nrf_reg_write(WRITE_REG_NRF+EN_RXADDR, 0x01);
	  //使能通道0的接收地址  	 
  	spi_nrf_reg_write(WRITE_REG_NRF+RF_CH,40);	     
	  //设置RF通信频率40Hz
  	spi_nrf_reg_write(WRITE_REG_NRF+RX_PW_P0,RX_PLOAD_WIDTH);
	  //选择通道0的有效数据宽度 	    
  	spi_nrf_reg_write(WRITE_REG_NRF+RF_SETUP, 0x0f);
	  //设置发射参数,0db增益,2Mbps   
  	spi_nrf_reg_write(WRITE_REG_NRF+CONFIG, 0x0f);
	  //配置基本工作模式的参数; PWR_UP, 使能CRC校验, 16位CRC编码, 接收模式，开启IRQ全部中断
	
  	RC_SPI_CE_HIGH_FUN(); 
	  //CE为高,进入工作模式(接收) 
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
  u8 spi_wait_timeout = SPI_WAIT_TIMEOUT;	
	
	RC_SPI_CE_HIGH_FUN();	 //进入接收状态
	
	//设定等待时间限制
	while(RC_SPI_INT_SCAN_FUN())//接收到数据会拉低
	{	
		if(!(spi_wait_timeout--))
			return nrf_timeout_usercallback(0);//没有接收到数据
	}

	RC_SPI_CE_LOW_FUN();  	 //进入待机状态
	
	rx_status = spi_nrf_reg_read(STATUS);  
	//读取状态寄存器的值    	 
	spi_nrf_reg_write(WRITE_REG_NRF+STATUS, rx_status); 
	//清除RX_DR中断标志
	
	
	if( rx_status & RX_DR )//接收到数据
	{ 
		spi_nrf_read_buffer(READ_REG_NRF + RD_RX_PAYLOAD, rxbuf, RX_PLOAD_WIDTH);
		//读取数据
		spi_nrf_reg_write(FLUSH_RX, 0xff);
		//清除RX FIFO寄存器 
		return SUCCESS; 
	}	   
	return FAILURE;//没收到任何数据
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
	
	RC_SPI_NSS_HIGH_FUN();
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
	
	// 配置寄存器可以得到状态，但是无法返回，可调试使用输出
	spi_send_byte(RC_SPI, reg);
	reg_val = spi_send_byte(RC_SPI, 0);

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
	
	status = spi_send_byte(RC_SPI, reg);
	
	for (byte_ctrl = 0; byte_ctrl < bytes; byte_ctrl++)
	{
			pBuf[byte_ctrl] = spi_send_byte(RC_SPI, 0);
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
	
	RC_SPI_NSS_LOW_FUN();
	
	status = spi_send_byte(RC_SPI, reg);
	// 选择寄存器 
	for (byte_ctrl = 0; byte_ctrl<bytes; byte_ctrl++)
	{
		spi_send_byte(RC_SPI, pBuf[byte_ctrl]);
		//按照字节发送数据
	}
	
	RC_SPI_NSS_HIGH_FUN();

  return status;
		
}




/**
  * 名称：nrf_timeout_usercallback
  *  
  * 描述：写入若干字节数据
  *
  */
int nrf_timeout_usercallback(u8 error_code)
{
	printf("nrf 等待超时！errorCode =%d\n", error_code);
	return 0;
}
