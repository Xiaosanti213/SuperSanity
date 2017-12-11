/**
 * @file board_init.c
 *
 * sanity-v1708 �����ʼ������
 * 
 * �����������ø�ʽ��
 * 			������pwm_tim_gpio_config
 *			���У�1 ʹ��ʱ��
 *						2 �����ṹ����� GPIO_InitTypeDef
 *						3 ����������ø�ֵ
 *						4 ͬһ���費ͬ���Ÿ��Բ�������
 *							���ڳ�ʼ��֮ǰ�ֱ�����GPIO_Init() 
 * �������ø�ʽ��
 *			������pwm_tim_config
 *			���У�1~4ͬ��
 *						5 ʹ�����裺TIM_Cmd(PWM_TIM, ENABLE);
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
 * ����: gpio_clk_config
 *
 * ����: ʹ��ȫ������gpioʱ��
 *   
 ************************************************************************************/
 
  void gpio_clk_config(void)
 {
	 
	 // ʹ��ȫ������GPIOʱ��
	 GPIO_CLK_Config_FUN(All_Periph_GPIO_CLK, ENABLE); 
 
 }
 
 
 
/************************************************************************************
 * 
 * ����: pwm_tim_gpio_config
 *
 * ����: ����TIM2��Ӧ��PA0~PA3����
 *   
 ************************************************************************************/

 static void pwm_tim_gpio_config(void)
{
	
	
	// ����GPIO��ʼ���ṹ��
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//�����˿������븴��IO����ʱ�ӣ���û�з�װ
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	
	
	// �����������
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
	
	// Ӧ����λ0
	PWM1_TIM_PIN_LOW_FUN();
	PWM2_TIM_PIN_LOW_FUN();
	PWM3_TIM_PIN_LOW_FUN();
	PWM4_TIM_PIN_LOW_FUN();

}





/************************************************************************************
 * 
 * ����: pwm_tim_config
 *
 * ����: ����TIM2�����PWM�źŵ�ģʽ�������ڣ����ԡ�
 *   
 ************************************************************************************/

 static void pwm_tim_config(void)
{
	
	//ռ�ձ�����
	uint16_t CH1_CCR_Val = 20000;
	uint16_t CH2_CCR_Val = 20000;
	uint16_t CH3_CCR_Val = 20000;
	uint16_t CH4_CCR_Val = 20000;
	
	
	
	// ����TIM2��ʼ���ṹ��
	
	// ����ͨ��TIM�����ģʽ��ֻ��Ҫ������������
	// 1 ʱ����ʼ���ṹ��	2 ����Ƚϳ�ʼ���ṹ��
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef   TIM_OCBaseInitStructure;
	
	// ʹ��ʱ��
	PWM_TIM_APB1Clock_FUN(PWM_TIM_CLK, ENABLE);
	
	
	// ����Ƶ PCLK1 = 72MHz
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	// ���ϼ���
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	// ��ʱ�����ڣ�Ӱ�ӼĴ���ARR��ֵ���������ʱ�ӹ�������
	TIM_TimeBaseInitStructure.TIM_Period = (20000-1);
	// ��ʱ��Ԥ��Ƶ��ֵ, ������������10s������һ����ʱ��1/(TIMxCLK/(psc+1)) 20ms
	TIM_TimeBaseInitStructure.TIM_Prescaler = (72-1);
	// TIM_RepetitionCounterֻ������߼���ʱ������, ���������ظ���������ֵ
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;	
	TIM_TimeBaseInit(PWM_TIM, &TIM_TimeBaseInitStructure);
	
	

	


	TIM_OCBaseInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	// ʹ�����
	TIM_OCBaseInitStructure.TIM_OutputState = TIM_OutputState_Enable ;
	// ���ó�ʼPWM������0
	TIM_OCBaseInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low ;
	// ��ʱ������ֵС��CCR_Valʱ��Ч��ƽΪ�͵�ƽ
	
	TIM_OCBaseInitStructure.TIM_Pulse = CH1_CCR_Val;
	PWM_TIM_CH1_Init_FUN(PWM_TIM, &TIM_OCBaseInitStructure);
	
	TIM_OCBaseInitStructure.TIM_Pulse = CH2_CCR_Val;
	PWM_TIM_CH2_Init_FUN(PWM_TIM, &TIM_OCBaseInitStructure);
	
	TIM_OCBaseInitStructure.TIM_Pulse = CH3_CCR_Val;
	PWM_TIM_CH3_Init_FUN(PWM_TIM, &TIM_OCBaseInitStructure);
	
	TIM_OCBaseInitStructure.TIM_Pulse = CH4_CCR_Val;
	PWM_TIM_CH4_Init_FUN(PWM_TIM, &TIM_OCBaseInitStructure);
	
	
	// ��������ռ�ձȹ���ʹ��	
	PWM_TIM_CH1_DUTY_EN_FUN();
	PWM_TIM_CH2_DUTY_EN_FUN();
	PWM_TIM_CH3_DUTY_EN_FUN();
	PWM_TIM_CH4_DUTY_EN_FUN();
	
	// ʹ�ܼ�����
	TIM_Cmd(PWM_TIM, ENABLE);

}







/************************************************************************************
 * 
 * ����: rc_spi_gpio_config
 *
 * ����: ����gpio��Ӧ��PA4~PA7��PB0����
 *   
 ************************************************************************************/

 static void rc_spi_gpio_config(void)
{
	
	// ����GPIO��ʼ���ṹ��
	GPIO_InitTypeDef GPIO_InitStructure;
		
	//�����˿������븴��IO����ʱ�ӣ���û�з�װ
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
	// �����������е㲻̫ȷ���Ƿ���Ҫ������
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 

	

	// �����������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
		
	// INT ��������, CE, NSS�������ó���ͨ���ģʽ
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
 * ����: rc_spi_config
 *
 * ����: ��������ģ���spiͨ��
 *   
 ************************************************************************************/

 static void rc_spi_config(void)
{
	
	// ����SPI��ʼ���ṹ��
	SPI_InitTypeDef   RC_SPI_InitStructure;
	
	// ʹ��ʱ��
	RC_SPI_APB2Clock_FUN(RC_SPI_CLK, ENABLE);
	
	// 8��Ƶ��ʱ��9M�����Ե���
	RC_SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	// SCK�ź��ߵ�һ�������زɼ�����
	RC_SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	// SCK�ź��߿���״̬�͵�ƽ
	RC_SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	// CRCУ�����ʽ
	RC_SPI_InitStructure.SPI_CRCPolynomial = 7;
	// SPIͨѶ����֡��С
	RC_SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b ;
	// ˫��ȫ˫������ģʽ
	RC_SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex ;
	// ��λ����
	RC_SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB ;
	// оƬ����ģʽ
	RC_SPI_InitStructure.SPI_Mode = SPI_Mode_Master ;
	RC_SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;

	SPI_Init(RC_SPI, &RC_SPI_InitStructure);
	// ʹ��SPI1����
	SPI_Cmd(RC_SPI, ENABLE);

}





/************************************************************************************
 * 
 * ����: ms5611_i2c_gpio_config
 *
 * ����: ����I2C��Ӧ��PB10~PB11����
 *   
 ************************************************************************************/

  static void ms5611_i2c_gpio_config(void)
 {
	 
	 GPIO_InitTypeDef GPIO_InitStructure;
	 //�����˿�����ʱ�ӣ���û�з�װ
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
	 
	 
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 
	 // �ж���������ͨ���ģʽ----�����·ͼ�в�û������
   GPIO_InitStructure.GPIO_Pin = MS5611_I2C_INT_PIN;
	 GPIO_Init(MS5611_I2C_PORT, &GPIO_InitStructure); 
	 
	 // �����������ø���ģʽ
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	 GPIO_InitStructure.GPIO_Pin = MS5611_I2C_SCL_PIN;
	 GPIO_Init(MS5611_I2C_PORT, &GPIO_InitStructure); 
	 
	 GPIO_InitStructure.GPIO_Pin = MS5611_I2C_SDA_PIN;
	 GPIO_Init(MS5611_I2C_PORT, &GPIO_InitStructure); 
 
 
 }
 
 
 
 
 
 
 
 
 
 /************************************************************************************
 * 
 * ����: ms5611_i2c_config
 *
 * ����: ����MS5611������ͨ��I2C����
 *   
 ************************************************************************************/

  static void ms5611_i2c_config(void)
 {
	 
	I2C_InitTypeDef  I2C_InitStructure;
	 
	// ʹ��ʱ��
  MS5611_I2C_APB1Clock_FUN(RCC_APB1Periph_I2C2, ENABLE);
	 
	// ���ÿ��Է�����Ӧ�ź�
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	// ���ôӻ���ַλ��
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	// ����SCL�߸ߵ͵�ƽռ�ձȣ�һ��Ҫ�󲻻���ϸ�
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	// ����Ҫ�ڴ˴���������ģʽ
  I2C_InitStructure.I2C_Mode  = I2C_Mode_I2C;
	// ����ͨѶ����100kHz
  I2C_InitStructure.I2C_ClockSpeed = 100000;
	 // ������ַ
  I2C_InitStructure.I2C_OwnAddress1 = 0x0A;

	 
	I2C_Init(MS5611_I2C, &I2C_InitStructure); 
	 
	// ʹ��I2C1����
	I2C_Cmd(MS5611_I2C, ENABLE);
 
 
 }
 
 
 
 
 
 
 /************************************************************************************
 * 
 * ����: debug_usart_gpio_config
 *
 * ����: ����USART��Ӧ��PA9~PA10����
 *   
 ************************************************************************************/

  static void debug_usart_gpio_config(void)
 {
	 
	 GPIO_InitTypeDef GPIO_InitStructure;
	 //�����˿�����ʱ�ӣ���û�з�װ
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
	 
	 // TX��������Ϊ���츴��ģʽ
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	 GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_PIN;
	 GPIO_Init(DEBUG_USART_PORT, &GPIO_InitStructure); 
	 
	 // RX��������Ϊ��������ģʽ
	 GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_PIN;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	 GPIO_Init(DEBUG_USART_PORT, &GPIO_InitStructure); 
 
 
 }
 
 
 
 
 
 
 
 
 
 /************************************************************************************
 * 
 * ����: debug_usart_config
 *
 * ����: ����USART��¼�������
 *   
 ************************************************************************************/

  static void debug_usart_config(void)
 {
	 
	 USART_InitTypeDef  USART_InitStructure;
	 	 
	 // ʹ��ʱ��
   DEBUG_USART_APB2Clock_FUN(DEBUG_USART_CLK, ENABLE);
	 
   // ���ò�����
   USART_InitStructure.USART_BaudRate = 115200;
   // ����Ӳ��������RTS��CTS��Ч
	 USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	 // ����USARTģʽ
   USART_InitStructure.USART_Mode = (USART_Mode_Rx|USART_Mode_Tx);
   // ������żУ��ѡ����У��
   USART_InitStructure.USART_Parity = USART_Parity_No;
   // ����ֹͣλ
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   // ��������֡�ֳ�
   USART_InitStructure.USART_WordLength = USART_WordLength_8b ;
	 
   USART_Init(DEBUG_USART, &USART_InitStructure); 
	 
   // ʹ�ܴ�������
   USART_Cmd(DEBUG_USART, ENABLE);
 
 
 }



/************************************************************************************
 * 
 * ����: mpu6050_i2c_gpio_config
 *
 * ����: ����I2C��Ӧ��PB5~PB7����
 *   
 ************************************************************************************/

  static void mpu6050_i2c_gpio_config(void)
 {
	 
	 GPIO_InitTypeDef GPIO_InitStructure;
	 
	
	RCC_APB2PeriphClockCmd ( RCC_APB2Periph_GPIOB, ENABLE );	 
	 
	 
	 // �ж���������ͨ���ģʽ
	 GPIO_InitStructure.GPIO_Pin = MPU6050_I2C_INT_PIN;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//�ж��������ã���IPU
	 GPIO_Init(GPIOB, &GPIO_InitStructure); 
	 
	 // �����������ø��ÿ�©ģʽ
	 //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	 //GPIO_InitStructure.GPIO_Pin = MPU6050_I2C_SCL_PIN;
	 //GPIO_Init(MPU6050_I2C_PORT, &GPIO_InitStructure); 
	 
	 //GPIO_InitStructure.GPIO_Pin = MPU6050_I2C_SDA_PIN;
	 //GPIO_Init(MPU6050_I2C_PORT, &GPIO_InitStructure); 
	 
	 GPIO_InitStructure.GPIO_Pin = MPU6050_I2C_SCL_PIN;
   //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;	       // ��©���
   GPIO_Init(GPIOB, &GPIO_InitStructure);
	
   GPIO_InitStructure.GPIO_Pin = MPU6050_I2C_SDA_PIN;
   //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;	       // ��©���
   GPIO_Init(GPIOB, &GPIO_InitStructure);	
 
 
 }
 
 
 
 
 
 
 /************************************************************************************
 * 
 * ����: mpu6050_i2c_config
 *
 * ����: ������MPU6050������I2C����
 *   
 ************************************************************************************/

  static void mpu6050_i2c_config(void)
 {
	 
	 //I2C_InitTypeDef  I2C_InitStructure;
	 
	 // ʹ��ʱ��
   //RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	 
	 // ʹ����Ӧ�ź�
	 //I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	 // ���ôӻ���ַλ��
	 //I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	 // ����SCL�߸ߵ͵�ƽռ�ձȣ�һ��Ҫ�󲻻���ϸ�
   //I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	 // ����Ҫ�ڴ˴���������ģʽ
   //I2C_InitStructure.I2C_Mode  = I2C_Mode_I2C;
	 // ����ͨѶ����100kHz��׼ģʽ
   //I2C_InitStructure.I2C_ClockSpeed = 400000;
	 // ������ַ
   //I2C_InitStructure.I2C_OwnAddress1 = 0x0A;

	 
	 //I2C_Init(MPU6050_I2C, &I2C_InitStructure); 
	 
	 // ʹ��I2C2����
	 //I2C_Cmd(MPU6050_I2C, ENABLE);
	I2C_InitTypeDef  I2C_InitStructure; 
	 
  RCC_APB1PeriphClockCmd ( RCC_APB1Periph_I2C1, ENABLE );

  /* I2C ���� */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	
	/* �ߵ�ƽ�����ȶ����͵�ƽ���ݱ仯 SCL ʱ���ߵ�ռ�ձ� */
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	
  I2C_InitStructure.I2C_OwnAddress1 =0X0A; 
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable ;
	
	/* I2C��Ѱַģʽ */
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	
	/* ͨ������ */
  I2C_InitStructure.I2C_ClockSpeed = 100000;
  
	/* I2C1 ��ʼ�� */
  I2C_Init(I2C1, &I2C_InitStructure);
  
	/* ʹ�� I2C1 */
  I2C_Cmd(I2C1, ENABLE); 
 
 }
 
 
 
 
 
 
 
 
 /************************************************************************************
 * 
 * ����: mpu6050_i2c_gpio_config_s
 *
 * ����: �������ģ��i2c��ʽ����I2C��Ӧ��PB5~PB7����
 *   
 ************************************************************************************/
 void mpu6050_i2c_gpio_config_s(void)
 {
	 GPIO_InitTypeDef  GPIO_InitStructure;
   RCC_APB2PeriphClockCmd ( RCC_APB2Periph_GPIOB, ENABLE );	 
	 
	 
	 // �ж���������ͨ��������ģʽ
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	 GPIO_Init(GPIOB, &GPIO_InitStructure); 
	 
	 // �����������ø��ÿ�©ģʽ
	 //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	 //GPIO_InitStructure.GPIO_Pin = MPU6050_I2C_SCL_PIN;
	 //GPIO_Init(MPU6050_I2C_PORT, &GPIO_InitStructure); 
	 
	 //GPIO_InitStructure.GPIO_Pin = MPU6050_I2C_SDA_PIN;
	 //GPIO_Init(MPU6050_I2C_PORT, &GPIO_InitStructure); 
	 
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	       // ��ͨ��©���
   GPIO_Init(GPIOB, &GPIO_InitStructure);
	
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	       // ��ͨ��©���
   GPIO_Init(GPIOB, &GPIO_InitStructure);	
  }
 
 
 
	
	
/************************************************************************************
 * 
 * ����: mpu6050_i2c_config_s
 *
 * ����: �������ģ��Э�����MPU6050������I2C����
 *   
 ************************************************************************************/

  static void mpu6050_i2c_config_s(void)
 {
	 
	I2C_InitTypeDef  I2C_InitStructure; 
	 
  RCC_APB1PeriphClockCmd ( RCC_APB1Periph_I2C1, ENABLE );

  /* I2C ���� */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	
	/* �ߵ�ƽ�����ȶ����͵�ƽ���ݱ仯 SCL ʱ���ߵ�ռ�ձ� */
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	
  I2C_InitStructure.I2C_OwnAddress1 =0X0A; 
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable ;
	
	/* I2C��Ѱַģʽ */
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	
	/* ͨ������ */
  I2C_InitStructure.I2C_ClockSpeed = 100000;
  
	/* I2C1 ��ʼ�� */
  I2C_Init(I2C1, &I2C_InitStructure);
  
	/* ʹ�� I2C1 */
  I2C_Cmd(I2C1, ENABLE); 
 
 }
 
 
 
 
 
 
 
 /************************************************************************************
 * 
 * ����: status_led_gpio_config
 *
 * ����: ����״ָ̬ʾ��LED��Ӧ��PB8����
 *   
 ************************************************************************************/

  void status_led_gpio_config(void)
 {
	 
	 GPIO_InitTypeDef GPIO_InitStructure;
	 
	 //GPIOB-8ʱ�ӿ���
	 RCC_APB2PeriphClockCmd ( RCC_APB2Periph_GPIOB, ENABLE );
	 
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 
	 // ��������ͨ�������ģʽ
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Pin = STATUS_LED_PIN;
	 GPIO_Init(STATUS_LED_PORT, &GPIO_InitStructure); 
	 
	 STATUS_LED_ON;
 
 }
 
 
 
 /************************************************************************************
 * 
 * ��ʼ��������װ
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
	 
 
 
 
 
 
 
 
 
 
 
 
 
