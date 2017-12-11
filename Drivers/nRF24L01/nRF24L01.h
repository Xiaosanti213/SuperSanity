/**
 * @file nRF24L01.h
 *
 * nRF24L01 无线收发模块操作指令 寄存器映射 函数声明
 * 
 */

#ifndef _NRF24L01_H
#define _NRF24L01_H

#include "stm32f10x.h"


//NRF24L01寄存器操作命令
#define READ_REG_NRF         0x00  //读配置寄存器,低5位为寄存器地址
#define WRITE_REG_NRF        0x20  //写配置寄存器,低5位为寄存器地址
#define RD_RX_PAYLOAD        0x61  //读RX有效数据,1~32字节
#define WR_TX_PAYLOAD        0xA0  //写TX有效数据,1~32字节
#define FLUSH_TX  		       0xE1  //清除TX FIFO寄存器.发射模式下用
#define FLUSH_RX             0xE2  //清除RX FIFO寄存器.接收模式下用，ACK传输时禁止执行
#define REUSE_TX_PL          0xE3  //PTX设备使用，重新使用上一包数据,CE为高,数据包被不断发送
#define R_RX_PL_WID					 0x60  //读RX FIFO寄存器顶有效数据宽度
#define NOP                  0xFF  //空操作,可以用来读状态寄存器	 

//SPI(NRF24L01)寄存器地址映射
#define CONFIG          0x00  //配置寄存器地址;bit0:1接收模式,0发射模式;bit1:电选择;bit2:CRC模式;bit3:CRC使能;
                              //bit4:中断MAX_RT(达到最大重发次数中断)使能;bit5:中断TX_DS使能;bit6:中断RX_DR使能
#define EN_AA           0x01  //使能自动应答功能  bit0~5,对应通道0~5
#define EN_RXADDR       0x02  //接收地址允许,bit0~5,对应通道0~5
#define SETUP_AW        0x03  //设置地址宽度(所有数据通道):bit1,0:00,3字节;01,4字节;02,5字节;
#define SETUP_RETR      0x04  //建立自动重发;bit3:0,自动重发计数器;bit7:4,自动重发延时 250*x+86us
#define RF_CH           0x05  //RF通道,bit6:0,工作通道频率;
#define RF_SETUP        0x06  //RF寄存器;bit3:传输速率(0:1Mbps,1:2Mbps);bit2:1,发射功率;bit0:低噪声放大器增益
#define STATUS          0x07  //状态寄存器;bit0:TX FIFO满标志;bit3:1,接收数据通道号(最大:6);bit4,达到最多次重发
                              //bit5:数据发送完成中断;bit6:接收数据中断;
#define MAX_TX  	      0x10  //达到最大发送次数中断
#define TX_OK   	      0x20  //TX发送完成中断
#define RX_OK   	      0x40  //接收到数据中断

#define OBSERVE_TX      0x08  //发送检测寄存器,bit7:4,数据包丢失计数器;bit3:0,重发计数器
#define CD              0x09  //载波检测寄存器,bit0,载波检测;
#define RX_ADDR_P0      0x0A  //数据通道0接收地址,最大长度5个字节,低字节在前
#define RX_ADDR_P1      0x0B  //数据通道1接收地址,最大长度5个字节,低字节在前
#define RX_ADDR_P2      0x0C  //数据通道2接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P3      0x0D  //数据通道3接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P4      0x0E  //数据通道4接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P5      0x0F  //数据通道5接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define TX_ADDR         0x10  //发送地址(低字节在前),ShockBurstTM模式下,RX_ADDR_P0与此地址相等
#define RX_PW_P0        0x11  //接收数据通道0有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P1        0x12  //接收数据通道1有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P2        0x13  //接收数据通道2有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P3        0x14  //接收数据通道3有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P4        0x15  //接收数据通道4有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P5        0x16  //接收数据通道5有效数据宽度(1~32字节),设置为0则非法
#define FIFO_STATUS     0x17  //FIFO状态寄存器;bit0,RX FIFO寄存器空标志;bit1,RX FIFO满标志;bit2,3,保留
                              //bit4,TX FIFO空标志;bit5,TX FIFO满标志;bit6,1,循环发送上一数据包.0,不循环;
															
															
															
	
// 寄存器配置:

// CONFIG寄存器配置：bit7:预留 bit6~bit0:
#define MASK_RX_DR			0x01<<6  //屏蔽RX_DR引起的IRQ引脚中断
#define MASK_TX_DS      0x01<<5  //屏蔽TX_DS引起的IRQ引脚中断
#define MASK_MAX_RT     0x01<<4  //屏蔽MAX_RT引起的IRQ引脚中断
#define EN_CRC          0x01<<3  //使能CRC校验，如果EN_AA任意位高，则强制拉高
#define CRCO_2Byte			0x01<<2  //CRC编码过程'0'-1字节 '1'-2字节
#define PWR_UP          0x01<<1  //1: POWER UP, 0: POWER DOWN
#define PRIM_RX         0x01<<0  //RX/TX控制 1:PRX, 0:PTX


// EN_AA寄存器配置： bit7~bit6：预留 bit5~bit0:
#define ENAA_P5         0x01<<5  //pipe5通道自动应答使能
#define ENAA_P4					0x01<<4  //pipe4通道自动应答使能
#define ENAA_P3					0x01<<3  //pipe3通道自动应答使能
#define ENAA_P2         0x01<<2  //pipe2通道自动应答使能
#define ENAA_P1					0x01<<1  //pipe1通道自动应答使能
#define ENAA_P0					0x01<<0  //pipe0通道自动应答使能


// EN_RXADDR寄存器配置： bit7~bit6: 预留 bit5~bit0:
#define ERX_P5					0x01<<5  //EN_RXADDR寄存器配置：使能数据pipe5
#define ERX_P4					0x01<<4  //EN_RXADDR寄存器配置：使能数据pipe4
#define ERX_P3					0x01<<3  //EN_RXADDR寄存器配置：使能数据pipe3
#define ERX_P2					0x01<<2  //EN_RXADDR寄存器配置：使能数据pipe2
#define ERX_P1          0x01<<1  //EN_RXADDR寄存器配置：使能数据pipe1
#define ERX_P0					0x01<<0  //EN_RXADDR寄存器配置：使能数据pipe0



// SETUP_AW寄存器配置： bit7~bit2：预留 bit1~bit0:
#define AW_5BYTES				0x03<<0  //	RX/TX地址域宽度: '11'-5bytes
#define AW_4BYTES       0x02<<0  // RX/TX地址域宽度: '10'-4bytes								
#define AW_3BYTES				0x01<<0  //	RX/TX地址域宽度: '01'-3bytes
#define AW_ILLEGAL      0x00<<0  // RX/TX地址域宽度: '00'-illegal


// SETUP_RETR寄存器配置：bit7~bit4:
#define ARD_WAIT_250US  			0x00<<4  // 自动重传输延时: '0000'-250us
#define ARD_WAIT_500US  			0x01<<4  // 自动重传输延时: '0001'-500us
#define ARD_WAIT_750US  			0x02<<4  // 自动重传输延时: '0010'-750us
#define ARD_WAIT_1000US 			0x03<<4  // 自动重传输延时: '0011'-1000us
#define ARD_WAIT_1250US 			0x04<<4  // 自动重传输延时: '0011'-1250us
#define ARD_WAIT_1500US 			0x05<<4  // 自动重传输延时: '0011'-1500us
#define ARD_WAIT_1750US 			0x06<<4  // 自动重传输延时: '0011'-1750us
#define ARD_WAIT_2000US 			0x07<<4  // 自动重传输延时: '0011'-2000us
#define ARD_WAIT_2250US 			0x08<<4  // 自动重传输延时: '0011'-2250us
#define ARD_WAIT_2500US 			0x09<<4  // 自动重传输延时: '0011'-2500us
#define ARD_WAIT_2750US 			0x0A<<4  // 自动重传输延时: '0011'-2750us
#define ARD_WAIT_3000US 			0x0B<<4  // 自动重传输延时: '0011'-3000us
#define ARD_WAIT_3250US 			0x0C<<4  // 自动重传输延时: '0011'-3250us
#define ARD_WAIT_3500US 			0x0D<<4  // 自动重传输延时: '0011'-3500us
#define ARD_WAIT_3750US 			0x0E<<4  // 自动重传输延时: '0011'-3750us
#define ARD_WAIT_4000US 			0x0F<<4  // 自动重传输延时: '0011'-4000us



// SETUP_RETR寄存器配置：bit3~bit0:
#define ARC_RETRANSMIT_DIS    0x00<<0  // 自动重传输失能
#define ARC_RETRANSMIT_1			0x01<<0  // 自动重传输1次
#define ARC_RETRANSMIT_2			0x02<<0  // 自动重传输2次
#define ARC_RETRANSMIT_3			0x03<<0  // 自动重传输3次
#define ARC_RETRANSMIT_4			0x04<<0  // 自动重传输4次
#define ARC_RETRANSMIT_5			0x05<<0  // 自动重传输5次
#define ARC_RETRANSMIT_6			0x06<<0  // 自动重传输6次
#define ARC_RETRANSMIT_7			0x07<<0  // 自动重传输7次
#define ARC_RETRANSMIT_8			0x08<<0  // 自动重传输8次
#define ARC_RETRANSMIT_9			0x09<<0  // 自动重传输9次
#define ARC_RETRANSMIT_10			0x0A<<0  // 自动重传输10次
#define ARC_RETRANSMIT_11		  0x0B<<0  // 自动重传输11次
#define ARC_RETRANSMIT_12			0x0C<<0  // 自动重传输12次
#define ARC_RETRANSMIT_13			0x0D<<0  // 自动重传输13次
#define ARC_RETRANSMIT_14			0x0E<<0  // 自动重传输14次
#define ARC_RETRANSMIT_15			0x0F<<0  // 自动重传输15次




// RF_SETUP寄存器配置：bit6：预留 bit7，bit5~bit0:
#define CONT_WAVE							0x01<<7  // Enables continuous carrier transmit when high
#define RF_DR_LOW							0x01<<5  // 设置无线数据速率到250kbps 编码参考RF_DR_HIGH
#define PLL_LOCK							0x01<<4  // 强制PLL锁信号，只在测试时使用
#define RF_DR_HIGH						0x01<<3  // 选择高速速率 编码：[RF_DR_LOW, RF_DR_HIGH]
																			 // '00'-1Mbps '01'-2Mbps '10'-250kbos '11'-Reserved
#define RF_PWR_18dBm  				0x00<<0  // RF输出功率大小  '00'- -18dBm
#define RF_PWR_12dBm					0x01<<0	 // RF输出功率大小  '01'- -12dBm 
#define RF_PWR_6dBm						0x02<<0	 // RF输出功率大小  '10'- -6dBm 
#define RF_PWR_0dBm						0x03<<0	 // RF输出功率大小  '11'- 0dBm


// STATUS寄存器配置：bit7：预留 bit6~bit0:
#define RX_DR									0x01<<6  // RX FIFO缓存数据准备好 触发中断，置'1'清除标志位
#define TX_DS									0x01<<5  // TX FIFO缓存数据准备好 触发中断，AUTO_ACK使能则ACK接收到数据该位置高
																			 // 置'1'清除标志位
#define MAX_RT								0x01<<4  // 最大TX重新发送数据中断，置'1'清除标志位		
#define RX_P_NO_PIPE0					0x00<<1  // 可用来读取RX_FIFO的可用数据管道编号 '000'~'101' - PIPE0~PIPE5
#define RX_P_NO_PIPE1					0x01<<1  
#define RX_P_NO_PIPE2					0x02<<1
#define RX_P_NO_PIPE3					0x03<<1
#define RX_P_NO_PIPE4					0x04<<1
#define RX_P_NO_PIPE5					0x05<<1
#define RX_P_NO_PIPE_NOTUSED	0x06<<1  // 未使用
#define RX_P_NO_PIPE_FIFOEMP  0x07<<1  // RX FIFO为空

#define TX_FULL				        0x01<<0  // TX FIFO缓存满标志位 '1'-缓存已满 '0'-TX FIFO中可用位置			



// check返回值宏
#define SUCCESS								  0x01
#define FAILURE									0x00


//nRF24L01发送接收数据宽度定义
#define TX_ADR_WIDTH    5   //5字节的地址宽度
#define RX_ADR_WIDTH    5   //5字节的地址宽度
#define TX_PLOAD_WIDTH  8  //8字节的用户数据宽度
#define RX_PLOAD_WIDTH  8  //8字节的用户数据宽度



//函数声明
void spi_nrf_init(void);
uint8_t spi_nrf_check(void);

void spi_nrf_rx_mode(void);
void spi_nrf_tx_mode(void);

uint8_t spi_nrf_tx_packet(uint8_t *txbuffer);
uint8_t spi_nrf_rx_packet(uint8_t *rxbuf);

															
#endif
