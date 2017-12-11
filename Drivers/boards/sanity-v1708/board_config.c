/**
 * @file board_init.c
 *
 * sanity-v1708 外设初始化函数
 * 
 * 外设引脚配置格式：
 * 			命名：pwm_tim_gpio_config
 *			序列：1 使能时钟
 *						2 声明结构体变量 GPIO_InitTypeDef
 *						3 具体参数配置赋值
 *						4 同一外设不同引脚各自参数配置
 *							可在初始化之前分别配置GPIO_Init() 
 * 外设配置格式：
 *			命名：pwm_tim_config
 *			序列：1~4同上
 *						5 使能外设：TIM_Cmd(PWM_TIM, ENABLE);
 *
 *			
 */
 
 
 
 
 
 
#include "board_config.h"

#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h" 
#include "stm32f10x_i2c.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_spi.h"
 
 
 
 static void rc_spi_config(void);
 
/************************************************************************************
 * 
 * 名称: gpio_clk_config
 *
 * 描述: 使能全部外设gpio时钟
 *   
 ************************************************************************************/
 
  void gpio_clk_config(void)
 {
	 
	 // 使能全部外设GPIO时钟
	 GPIO_CLK_Config_FUN(All_Periph_GPIO_CLK, ENABLE); 
 
 }
 
 
 
/************************************************************************************
 * 
 * 名称: pwm_tim_gpio_config
 *
 * 描述: 配置TIM2对应的PA0~PA3引脚
 *   
 ************************************************************************************/

 static void pwm_tim_gpio_config(void)
{
	
	
	// 配置GPIO初始化结构体
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//开启端口外设与复用IO外设时钟，并没有封装
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	
	
	// 复用推挽输出
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_InitStructure.GPIO_Pin = PWM1_TIM_PIN;
	GPIO_Init(PWM_TIM_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = PWM2_TIM_PIN;
  GPIO_Init(PWM_TIM_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = PWM3_TIM_PIN;
  GPIO_Init(PWM_TIM_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = PWM4_TIM_PIN;
  GPIO_Init(PWM_TIM_PORT, &GPIO_InitStructure);
	
	// 应该置位0
	PWM1_TIM_PIN_LOW_FUN();
	PWM2_TIM_PIN_LOW_FUN();
	PWM3_TIM_PIN_LOW_FUN();
	PWM4_TIM_PIN_LOW_FUN();

}





/************************************************************************************
 * 
 * 名称: pwm_tim_config
 *
 * 描述: 配置TIM2输出的PWM信号的模式，如周期，极性。
 *   
 ************************************************************************************/

 static void pwm_tim_config(void)
{
	
	//占空比配置
	uint16_t CH1_CCR_Val = 20000;
	uint16_t CH2_CCR_Val = 20000;
	uint16_t CH3_CCR_Val = 20000;
	uint16_t CH4_CCR_Val = 20000;
	
	
	
	// 配置TIM2初始化结构体
	
	// 对于通用TIM的输出模式，只需要配置其中两个
	// 1 时基初始化结构体	2 输出比较初始化结构体
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef   TIM_OCBaseInitStructure;
	
	// 使能时钟
	PWM_TIM_APB1Clock_FUN(PWM_TIM_CLK, ENABLE);
	
	
	// 不分频 PCLK1 = 72MHz
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	// 向上计数
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	// 定时器周期（影子寄存器ARR的值）和下面的时钟构成周期
	TIM_TimeBaseInitStructure.TIM_Period = (20000-1);
	// 定时器预分频的值, 配置驱动周期10s，计数一个数时间1/(TIMxCLK/(psc+1)) 20ms
	TIM_TimeBaseInitStructure.TIM_Prescaler = (72-1);
	// TIM_RepetitionCounter只存在与高级定时器当中, 无需设置重复计数器的值
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;	
	TIM_TimeBaseInit(PWM_TIM, &TIM_TimeBaseInitStructure);
	
	

	


	TIM_OCBaseInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	// 使能输出
	TIM_OCBaseInitStructure.TIM_OutputState = TIM_OutputState_Enable ;
	// 设置初始PWM脉冲宽度0
	TIM_OCBaseInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low ;
	// 定时器计数值小于CCR_Val时有效电平为低电平
	
	TIM_OCBaseInitStructure.TIM_Pulse = CH1_CCR_Val;
	PWM_TIM_CH1_Init_FUN(PWM_TIM, &TIM_OCBaseInitStructure);
	
	TIM_OCBaseInitStructure.TIM_Pulse = CH2_CCR_Val;
	PWM_TIM_CH2_Init_FUN(PWM_TIM, &TIM_OCBaseInitStructure);
	
	TIM_OCBaseInitStructure.TIM_Pulse = CH3_CCR_Val;
	PWM_TIM_CH3_Init_FUN(PWM_TIM, &TIM_OCBaseInitStructure);
	
	TIM_OCBaseInitStructure.TIM_Pulse = CH4_CCR_Val;
	PWM_TIM_CH4_Init_FUN(PWM_TIM, &TIM_OCBaseInitStructure);
	
	
	// 单独配置占空比功能使能	
	PWM_TIM_CH1_DUTY_EN_FUN();
	PWM_TIM_CH2_DUTY_EN_FUN();
	PWM_TIM_CH3_DUTY_EN_FUN();
	PWM_TIM_CH4_DUTY_EN_FUN();
	
	// 使能计数器
	TIM_Cmd(PWM_TIM, ENABLE);

}







/************************************************************************************
 * 
 * 名称: rc_spi_gpio_config
 *
 * 描述: 配置gpio对应的PA4~PA7，PB0引脚
 *   
 ************************************************************************************/

 static void rc_spi_gpio_config(void)
{
	
	// 配置GPIO初始化结构体
	GPIO_InitTypeDef GPIO_InitStructure;
		
	//开启端口外设与复用IO外设时钟，并没有封装
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
	// 这个后面这个有点不太确定是否需要加上了
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 

	

	// 复用推挽输出
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
		
	// INT 上拉输入, CE, NSS引脚配置成普通输出模式
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = RC_SPI_NSS_PIN;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = RC_SPI_CE_PIN;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
		
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Pin = RC_SPI_INT_PIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//RC_SPI_NSS_HIGH_FUN();

}





/************************************************************************************
 * 
 * 名称: rc_spi_config
 *
 * 描述: 配置无线模块的spi通信
 *   
 ************************************************************************************/

 static void rc_spi_config(void)
{
	
	// 配置SPI初始化结构体
	SPI_InitTypeDef   RC_SPI_InitStructure;
	
	// 使能时钟
	RC_SPI_APB2Clock_FUN(RC_SPI_CLK, ENABLE);
	
	// 8分频，时钟9M，可以调节
	RC_SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	// SCK信号线第一个上升沿采集数据
	RC_SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	// SCK信号线空闲状态低电平
	RC_SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	// CRC校验多项式
	RC_SPI_InitStructure.SPI_CRCPolynomial = 7;
	// SPI通讯数据帧大小
	RC_SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b ;
	// 双线全双工工作模式
	RC_SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex ;
	// 高位先行
	RC_SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB ;
	// 芯片主机模式
	RC_SPI_InitStructure.SPI_Mode = SPI_Mode_Master ;
	RC_SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;

	SPI_Init(RC_SPI, &RC_SPI_InitStructure);
	// 使能SPI1外设
	SPI_Cmd(RC_SPI, ENABLE);

}





/************************************************************************************
 * 
 * 名称: ms5611_i2c_gpio_config
 *
 * 描述: 配置I2C对应的PB10~PB11引脚
 *   
 ************************************************************************************/

  static void ms5611_i2c_gpio_config(void)
 {
	 
	 GPIO_InitTypeDef GPIO_InitStructure;
	 //开启端口外设时钟，并没有封装
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
	 
	 
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 
	 // 中断引脚是普通输出模式----这个电路图中并没有连接
   GPIO_InitStructure.GPIO_Pin = MS5611_I2C_INT_PIN;
	 GPIO_Init(MS5611_I2C_PORT, &GPIO_InitStructure); 
	 
	 // 其他引脚配置复用模式
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	 GPIO_InitStructure.GPIO_Pin = MS5611_I2C_SCL_PIN;
	 GPIO_Init(MS5611_I2C_PORT, &GPIO_InitStructure); 
	 
	 GPIO_InitStructure.GPIO_Pin = MS5611_I2C_SDA_PIN;
	 GPIO_Init(MS5611_I2C_PORT, &GPIO_InitStructure); 
 
 
 }
 
 
 
 
 
 
 
 
 
 /************************************************************************************
 * 
 * 名称: ms5611_i2c_config
 *
 * 描述: 配置MS5611与主机通过I2C连接
 *   
 ************************************************************************************/

  static void ms5611_i2c_config(void)
 {
	 
	I2C_InitTypeDef  I2C_InitStructure;
	 
	// 使能时钟
  MS5611_I2C_APB1Clock_FUN(RCC_APB1Periph_I2C2, ENABLE);
	 
	// 设置可以发送响应信号
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	// 设置从机地址位数
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	// 设置SCL线高低电平占空比，一般要求不会很严格
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	// 不需要在此处区分主从模式
  I2C_InitStructure.I2C_Mode  = I2C_Mode_I2C;
	// 设置通讯速率100kHz
  I2C_InitStructure.I2C_ClockSpeed = 100000;
	 // 主机地址
  I2C_InitStructure.I2C_OwnAddress1 = 0x0A;

	 
	I2C_Init(MS5611_I2C, &I2C_InitStructure); 
	 
	// 使能I2C1外设
	I2C_Cmd(MS5611_I2C, ENABLE);
 
 
 }
 
 
 
 
 
 
 /************************************************************************************
 * 
 * 名称: debug_usart_gpio_config
 *
 * 描述: 配置USART对应的PA9~PA10引脚
 *   
 ************************************************************************************/

  static void debug_usart_gpio_config(void)
 {
	 
	 GPIO_InitTypeDef GPIO_InitStructure;
	 //开启端口外设时钟，并没有封装
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
	 
	 // TX引脚配置为推挽复用模式
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	 GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_PIN;
	 GPIO_Init(DEBUG_USART_PORT, &GPIO_InitStructure); 
	 
	 // RX引脚配置为浮空输入模式
	 GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_PIN;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	 GPIO_Init(DEBUG_USART_PORT, &GPIO_InitStructure); 
 
 
 }
 
 
 
 
 
 
 
 
 
 /************************************************************************************
 * 
 * 名称: debug_usart_config
 *
 * 描述: 配置USART烧录程序调参
 *   
 ************************************************************************************/

  static void debug_usart_config(void)
 {
	 
	 USART_InitTypeDef  USART_InitStructure;
	 	 
	 // 使能时钟
   DEBUG_USART_APB2Clock_FUN(DEBUG_USART_CLK, ENABLE);
	 
   // 设置波特率
   USART_InitStructure.USART_BaudRate = 115200;
   // 设置硬件流控制RTS和CTS无效
	 USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	 // 设置USART模式
   USART_InitStructure.USART_Mode = (USART_Mode_Rx|USART_Mode_Tx);
   // 设置奇偶校验选择无校验
   USART_InitStructure.USART_Parity = USART_Parity_No;
   // 设置停止位
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   // 设置数据帧字长
   USART_InitStructure.USART_WordLength = USART_WordLength_8b ;
	 
   USART_Init(DEBUG_USART, &USART_InitStructure); 
	 
   // 使能串口外设
   USART_Cmd(DEBUG_USART, ENABLE);
 
 
 }



/************************************************************************************
 * 
 * 名称: mpu6050_i2c_gpio_config
 *
 * 描述: 配置I2C对应的PB5~PB7引脚
 *   
 ************************************************************************************/

  static void mpu6050_i2c_gpio_config(void)
 {
	 
	 GPIO_InitTypeDef GPIO_InitStructure;
	 
	
	RCC_APB2PeriphClockCmd ( RCC_APB2Periph_GPIOB, ENABLE );	 
	 
	 
	 // 中断引脚是普通输出模式
	 GPIO_InitStructure.GPIO_Pin = MPU6050_I2C_INT_PIN;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//中断引脚配置？？IPU
	 GPIO_Init(GPIOB, &GPIO_InitStructure); 
	 
	 // 其他引脚配置复用开漏模式
	 //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	 //GPIO_InitStructure.GPIO_Pin = MPU6050_I2C_SCL_PIN;
	 //GPIO_Init(MPU6050_I2C_PORT, &GPIO_InitStructure); 
	 
	 //GPIO_InitStructure.GPIO_Pin = MPU6050_I2C_SDA_PIN;
	 //GPIO_Init(MPU6050_I2C_PORT, &GPIO_InitStructure); 
	 
	 GPIO_InitStructure.GPIO_Pin = MPU6050_I2C_SCL_PIN;
   //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;	       // 开漏输出
   GPIO_Init(GPIOB, &GPIO_InitStructure);
	
   GPIO_InitStructure.GPIO_Pin = MPU6050_I2C_SDA_PIN;
   //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;	       // 开漏输出
   GPIO_Init(GPIOB, &GPIO_InitStructure);	
 
 
 }
 
 
 
 
 
 
 /************************************************************************************
 * 
 * 名称: mpu6050_i2c_config
 *
 * 描述: 配置与MPU6050传感器I2C连接
 *   
 ************************************************************************************/

  static void mpu6050_i2c_config(void)
 {
	 
	 //I2C_InitTypeDef  I2C_InitStructure;
	 
	 // 使能时钟
   //RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	 
	 // 使能响应信号
	 //I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	 // 设置从机地址位数
	 //I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	 // 设置SCL线高低电平占空比，一般要求不会很严格
   //I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	 // 不需要在此处区分主从模式
   //I2C_InitStructure.I2C_Mode  = I2C_Mode_I2C;
	 // 设置通讯速率100kHz标准模式
   //I2C_InitStructure.I2C_ClockSpeed = 400000;
	 // 主机地址
   //I2C_InitStructure.I2C_OwnAddress1 = 0x0A;

	 
	 //I2C_Init(MPU6050_I2C, &I2C_InitStructure); 
	 
	 // 使能I2C2外设
	 //I2C_Cmd(MPU6050_I2C, ENABLE);
	I2C_InitTypeDef  I2C_InitStructure; 
	 
  RCC_APB1PeriphClockCmd ( RCC_APB1Periph_I2C1, ENABLE );

  /* I2C 配置 */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	
	/* 高电平数据稳定，低电平数据变化 SCL 时钟线的占空比 */
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	
  I2C_InitStructure.I2C_OwnAddress1 =0X0A; 
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable ;
	
	/* I2C的寻址模式 */
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	
	/* 通信速率 */
  I2C_InitStructure.I2C_ClockSpeed = 100000;
  
	/* I2C1 初始化 */
  I2C_Init(I2C1, &I2C_InitStructure);
  
	/* 使能 I2C1 */
  I2C_Cmd(I2C1, ENABLE); 
 
 }
 
 
 
 
 
 
 
 
 /************************************************************************************
 * 
 * 名称: mpu6050_i2c_gpio_config_s
 *
 * 描述: 采用软件模拟i2c方式配置I2C对应的PB5~PB7引脚
 *   
 ************************************************************************************/
 void mpu6050_i2c_gpio_config_s(void)
 {
	 GPIO_InitTypeDef  GPIO_InitStructure;
   RCC_APB2PeriphClockCmd ( RCC_APB2Periph_GPIOB, ENABLE );	 
	 
	 
	 // 中断引脚是普通上拉输入模式
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	 GPIO_Init(GPIOB, &GPIO_InitStructure); 
	 
	 // 其他引脚配置复用开漏模式
	 //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	 //GPIO_InitStructure.GPIO_Pin = MPU6050_I2C_SCL_PIN;
	 //GPIO_Init(MPU6050_I2C_PORT, &GPIO_InitStructure); 
	 
	 //GPIO_InitStructure.GPIO_Pin = MPU6050_I2C_SDA_PIN;
	 //GPIO_Init(MPU6050_I2C_PORT, &GPIO_InitStructure); 
	 
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	       // 普通开漏输出
   GPIO_Init(GPIOB, &GPIO_InitStructure);
	
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	       // 普通开漏输出
   GPIO_Init(GPIOB, &GPIO_InitStructure);	
  }
 
 
 
	
	
/************************************************************************************
 * 
 * 名称: mpu6050_i2c_config_s
 *
 * 描述: 配置软件模拟协议的与MPU6050传感器I2C连接
 *   
 ************************************************************************************/

  static void mpu6050_i2c_config_s(void)
 {
	 
	I2C_InitTypeDef  I2C_InitStructure; 
	 
  RCC_APB1PeriphClockCmd ( RCC_APB1Periph_I2C1, ENABLE );

  /* I2C 配置 */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	
	/* 高电平数据稳定，低电平数据变化 SCL 时钟线的占空比 */
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	
  I2C_InitStructure.I2C_OwnAddress1 =0X0A; 
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable ;
	
	/* I2C的寻址模式 */
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	
	/* 通信速率 */
  I2C_InitStructure.I2C_ClockSpeed = 100000;
  
	/* I2C1 初始化 */
  I2C_Init(I2C1, &I2C_InitStructure);
  
	/* 使能 I2C1 */
  I2C_Cmd(I2C1, ENABLE); 
 
 }
 
 
 
 
 
 
 
 /************************************************************************************
 * 
 * 名称: status_led_gpio_config
 *
 * 描述: 配置状态指示灯LED对应的PB8引脚
 *   
 ************************************************************************************/

  void status_led_gpio_config(void)
 {
	 
	 GPIO_InitTypeDef GPIO_InitStructure;
	 
	 //GPIOB-8时钟开启
	 RCC_APB2PeriphClockCmd ( RCC_APB2Periph_GPIOB, ENABLE );
	 
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 
	 // 引脚是普通推挽输出模式
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Pin = STATUS_LED_PIN;
	 GPIO_Init(STATUS_LED_PORT, &GPIO_InitStructure); 
	 
	 STATUS_LED_ON;
 
 }
 
 
 
 /************************************************************************************
 * 
 * 初始化函数封装
 * 
 ************************************************************************************/
 
 
 void tim_config(void)
 {
		pwm_tim_gpio_config(); 
	  pwm_tim_config();
 }
 
 void mpu6050_config(void)
 {
	 mpu6050_i2c_gpio_config();
	 mpu6050_i2c_config();
 }
 
 void ms5611_config(void)
 {
	 ms5611_i2c_gpio_config(); 
	 ms5611_i2c_config();
 }
 
 void nrf24l01_config(void)
 {
	 rc_spi_gpio_config();
	 rc_spi_config();
 }
 

 void usb_config(void)
 {
	 debug_usart_gpio_config();
	 debug_usart_config();
 }
	 
 
 
 
 
 
 
 
 
 
 
 
 
