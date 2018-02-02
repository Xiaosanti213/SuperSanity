/**
 * @file board_config.h
 *
 * sanity-v1708 内部引脚配置
 * 
 * 封装变量命名规则(项目可选)：
 *			PWM_TIM_CH1_DUTY_EN_FUN
 *			外设名称_总线协议名称_端口_说明用途_引脚/端口/函数
 *
 */
 #ifndef _BOARD_CONFIG_H
 #define _BOARD_CONFIG_H



 #include "stm32f10x.h"


 
 /**
  * 通用端口，引脚配置宏
	*
	*/
 #define						GPIO_CLK_Config_FUN							RCC_APB2PeriphClockCmd
 // 因为同时需要输出，因此也要开启输出GPIO时钟
 #define						All_Periph_GPIO_CLK								(RCC_APB2Periph_AFIO|\
																						RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|\
																								RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD)
 
 
 
 
 
 
 

 /************************************************************************************
 * 
 * 10~13 (PA0~PA3)引脚配置
 *
 * 描述: 复用TIM2通用定时器，输出PWM
 *   
 ************************************************************************************/

 #define						PWM_TIM											TIM2
 
 // 确定外设所在位置APB1，开启对应外设时钟
 #define						PWM_TIM_APB1Clock_FUN					RCC_APB1PeriphClockCmd
 #define						PWM_TIM_CLK										RCC_APB1Periph_TIM2

 
 // PA0~PA3引脚需要重映射到TIM2上
 #define						PWM_TIM_REMAP_FUN()						GPIO_PinRemapConfig(GPIO_FullRemap_TIM2,ENABLE)
 
 // 端口引脚配置
 #define						PWM_TIM_PORT									GPIOA
 #define						PWM1_TIM_PIN									GPIO_Pin_0
 #define						PWM2_TIM_PIN									GPIO_Pin_1
 #define						PWM3_TIM_PIN									GPIO_Pin_2
 #define						PWM4_TIM_PIN									GPIO_Pin_3
 
 // 引脚电平封装
 #define						PWM1_TIM_PIN_LOW_FUN()				GPIO_ResetBits(PWM_TIM_PORT, PWM1_TIM_PIN)
 #define						PWM2_TIM_PIN_LOW_FUN()				GPIO_ResetBits(PWM_TIM_PORT, PWM2_TIM_PIN)
 #define						PWM3_TIM_PIN_LOW_FUN()				GPIO_ResetBits(PWM_TIM_PORT, PWM3_TIM_PIN)
 #define						PWM4_TIM_PIN_LOW_FUN()				GPIO_ResetBits(PWM_TIM_PORT, PWM4_TIM_PIN)
 
 // 初始化时基配置结构体函数
 // #define						PWM_TIM_TimeBaseInit_FUN							TIM_TimeBaseInit
 
 // 初始化TIM2的四个通道CH1~CH4比较输出结构体函数
 #define						PWM_TIM_CH1_Init_FUN  							TIM_OC1Init
 #define						PWM_TIM_CH2_Init_FUN  							TIM_OC2Init
 #define						PWM_TIM_CH3_Init_FUN  							TIM_OC3Init
 #define						PWM_TIM_CH4_Init_FUN  							TIM_OC4Init
 
 // 使能CCR比较寄存器预装载函数,从而可以外部调节占空比
 #define						PWM_TIM_CH1_DUTY_EN_FUN()						TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable)
 #define						PWM_TIM_CH2_DUTY_EN_FUN()						TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable)
 #define						PWM_TIM_CH3_DUTY_EN_FUN()						TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable)
 #define						PWM_TIM_CH4_DUTY_EN_FUN()						TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable)

 // 比较寄存器封装
 #define						PWM_TIM_CH1_CCR									CCR1		
 #define						PWM_TIM_CH2_CCR									CCR2
 #define						PWM_TIM_CH3_CCR									CCR3
 #define						PWM_TIM_CH4_CCR									CCR4
 
 
 
 
 
 
 
 
 
 /************************************************************************************
 * 
 * 14~18 (PA4~PA7,PB0)引脚配置
 *
 * 描述: SPI1连接无线接收模块
 *   
 ************************************************************************************/
 #define						RC_SPI											SPI1
 
 // 确定外设所在位置APB2，开启对应外设时钟
 #define						RC_SPI_APB2Clock_FUN							RCC_APB2PeriphClockCmd
 #define						RC_SPI_CLK										RCC_APB2Periph_SPI1
 
 
 // 端口引脚配置
 #define						RC_SPI_PORT										      GPIOA
 #define						RC_SPI_NSS_PIN									    GPIO_Pin_4
 #define						RC_SPI_CLK_PIN									    GPIO_Pin_5
 #define						RC_SPI_MISO_PIN									    GPIO_Pin_6
 #define						RC_SPI_MOSI_PIN   							    GPIO_Pin_7
 
 #define						RC_SPI_INT_CE_PORT								GPIOB
 #define						RC_SPI_INT_PIN									  GPIO_Pin_0 
 #define						RC_SPI_CE_PIN									    GPIO_Pin_1
 
 // 初始化SPI配置结构体函数
// #define						RC_SPI_Init_FUN									SPI_Init

 // 使能引脚输出高低电平函数
 #define  					RC_SPI_NSS_LOW_FUN()     						GPIO_ResetBits( RC_SPI_PORT, RC_SPI_NSS_PIN )
 #define  					RC_SPI_NSS_HIGH_FUN()    						GPIO_SetBits( RC_SPI_PORT, RC_SPI_NSS_PIN )
 #define						RC_SPI_CE_LOW_FUN()							  	GPIO_ResetBits( RC_SPI_INT_CE_PORT, RC_SPI_CE_PIN)
 #define						RC_SPI_CE_HIGH_FUN()								GPIO_SetBits( RC_SPI_INT_CE_PORT, RC_SPI_CE_PIN)
 #define						RC_SPI_INT_SCAN_FUN()								GPIO_ReadInputDataBit( RC_SPI_INT_CE_PORT, RC_SPI_INT_PIN)
 #define						RC_SPI_INT_LOW											0





/************************************************************************************
 * 
 * 46, 21~22 (PB9~PB11)引脚配置
 *
 * 描述: I2C2连接MS5611模块
 *   
 ************************************************************************************/
 #define						MS5611_I2C										     I2C2//用测试模块
 
 // 确定外设所在位置APB1，开启对应外设时钟
 #define						MS5611_I2C_APB1Clock_FUN						RCC_APB1PeriphClockCmd
 #define						MS5611_I2C_CLK						    			RCC_APB1Periph_I2C2
 
 // PB5~PB7引脚需要重映射到TIM2上
 #define						MS5611_I2C_REMAP_FUN()							GPIO_PinRemapConfig(GPIO_Remap_I2C2,ENABLE) 
 
 // 端口引脚配置
 #define						MS5611_I2C_PORT										GPIOB
 #define						MS5611_I2C_INT_PIN								GPIO_Pin_9
 #define						MS5611_I2C_SCL_PIN								GPIO_Pin_10
 #define						MS5611_I2C_SDA_PIN								GPIO_Pin_11
 
 // 初始化SPI配置结构体函数
// #define						MS5611_I2C_Init_FUN							  	I2C_Init					 


 // 下面是时钟线和地址线的操作封装
 #define MS5611_I2C_SCL_1_FUN()  GPIO_SetBits(MS5611_I2C_PORT, MS5611_I2C_SCL_PIN)		
 #define MS5611_I2C_SCL_0_FUN()  GPIO_ResetBits(MS5611_I2C_PORT, MS5611_I2C_SCL_PIN)		
	
 #define MS5611_I2C_SDA_1_FUN()  GPIO_SetBits(MS5611_I2C_PORT, MS5611_I2C_SDA_PIN)		
 #define MS5611_I2C_SDA_0_FUN()  GPIO_ResetBits(MS5611_I2C_PORT, MS5611_I2C_SDA_PIN)		
	
 #define MS5611_I2C_SDA_READ_FUN()  GPIO_ReadInputDataBit(MS5611_I2C_PORT, MS5611_I2C_SDA_PIN)
 
 
 
 
 



/************************************************************************************
 * 
 * 30~31 (PA9~PA10)引脚配置
 *
 * 描述: 复用串口功能烧录程序调参使用
 *   
 ************************************************************************************/
 
 #define						DEBUG_USART													USART1
 
 // 确定外设所在位置APB2，开启对应外设时钟
 #define						DEBUG_USART_APB2Clock_FUN						RCC_APB2PeriphClockCmd
 #define						DEBUG_USART_CLK											RCC_APB2Periph_USART1
 
 // PA9~PA10引脚需要重映射到TIM2上
 #define						DEBUG_USART_REMAP_FUN()							GPIO_PinRemapConfig(GPIO_Remap_USART1,ENABLE) 
 
 // 端口引脚配置
 #define						DEBUG_USART_PORT										GPIOA
 #define						DEBUG_USART_TX_PIN									GPIO_Pin_9
 #define						DEBUG_USART_RX_PIN									GPIO_Pin_10
 
 // 初始化SPI配置结构体函数
// #define						DEBUG_USART_Init_FUN							  USART_Init	
 
 // 中断服务函数









/************************************************************************************
 * 
 * 41~43 (PB5~PB7)引脚配置
 *
 * 描述: I2C1连接MPU6050模块
 *   
 ************************************************************************************/
 #define						MPU6050_I2C													I2C1
 
 // 确定外设所在位置APB1，开启对应外设时钟
 #define						MPU6050_I2C_APB1Clock_FUN						RCC_APB1PeriphClockCmd
 #define						MPU6050_I2C_CLK											RCC_APB1Periph_I2C1
 
 // PB5~PB7引脚需要重映射到TIM2上
 #define						MPU6050_I2C_REMAP_FUN()							GPIO_PinRemapConfig(GPIO_Remap_I2C1,ENABLE) 
 
 // 端口引脚配置
 #define						MPU6050_I2C_PORT										GPIOB
 #define						MPU6050_I2C_INT_PIN									GPIO_Pin_5
 #define						MPU6050_I2C_SCL_PIN									GPIO_Pin_7
 #define						MPU6050_I2C_SDA_PIN       					GPIO_Pin_6
 
 
 // 初始化SPI配置结构体函数
// #define						MPU6050_I2C_Init_FUN								I2C_Init



 // 下面是时钟线和地址线的操作封装
 #define MPU6050_I2C_SCL_1_FUN()  GPIO_SetBits(MPU6050_I2C_PORT, MPU6050_I2C_SCL_PIN)		
 #define MPU6050_I2C_SCL_0_FUN()  GPIO_ResetBits(MPU6050_I2C_PORT, MPU6050_I2C_SCL_PIN)		
	
 #define MPU6050_I2C_SDA_1_FUN()  GPIO_SetBits(MPU6050_I2C_PORT, MPU6050_I2C_SDA_PIN)		
 #define MPU6050_I2C_SDA_0_FUN()  GPIO_ResetBits(MPU6050_I2C_PORT, MPU6050_I2C_SDA_PIN)		
	
 #define MPU6050_I2C_SDA_READ_FUN()  GPIO_ReadInputDataBit(MPU6050_I2C_PORT, MPU6050_I2C_SDA_PIN)
 
 
 

/************************************************************************************
 * 
 * 45 (PB8)引脚配置
 *
 * 描述: 状态指示灯
 *   
 ************************************************************************************/
 // 确定外设所在位置APB2, 开启对应外设时钟, 已经在上面宏中完成

 
 
 // 端口引脚配置
 #define						STATUS_LED_PORT											GPIOB
 #define						STATUS_LED_PIN											GPIO_Pin_8
 #define						STATUS_LED_ON												GPIO_SetBits(STATUS_LED_PORT, STATUS_LED_PIN)
 #define						STATUS_LED_OFF											GPIO_ResetBits(STATUS_LED_PORT, STATUS_LED_PIN)
  
 #define						STATUS_LED1_PORT										GPIOB
 #define						STATUS_LED1_PIN											GPIO_Pin_7
 #define						STATUS_LED1_ON											GPIO_SetBits(STATUS_LED_PORT, STATUS_LED_PIN)
 #define						STATUS_LED1_OFF											GPIO_ResetBits(STATUS_LED_PORT, STATUS_LED_PIN)


/************************************************************************************
 * 
 * 
 *
 * 描述: 初始化函数声明
 *   
 ************************************************************************************/

  void gpio_clk_config(void);
	void tim_config(void);
  void nrf24l01_config(void);
  void usb_config(void);
  void status_led_gpio_config(void);
	
  void ms5611_i2c_gpio_config_s(void);
	void mpu6050_i2c_gpio_config_s(void);



 
 
 #endif



