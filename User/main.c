/**
 *
 * @file main.c
 *
 * ��ѭ��
 *
 **/
 
 
#include <stm32f10x.h>
#include "motor.h"
#include "sensors.h"
#include <string.h>
#include <stdio.h> 
#include "attitude_control.h"
#include "board_config.h"
#include "debug.h"
#include "mpu6050.h"
#include "nRF24L01.h"
#include "stm32f10x_it.h"
#include "systick.h"
#include "api_tim.h"
#include "IMUSO3.h"
#include "IMU.h"
#include "control.h"
#include "Sys_Fun.h"


//check executing time and period in different loop
uint32_t start_time[2],exec_time[2];
uint32_t real_exec_prd[2];	//us , real called period in different loop



int main()
{
	
	//��ʱ��������
	//cycleCounterInit();	    			// �����ǽ�usTicks��ʼ��Ϊ72
  SystemClock_HSE(9);
	
	SysTick_Config(72000L);					// 72000ÿ��1ms�����ж�
 
  
	tim_config(); 								// ��ʼ�����PWM��ʱ�� ������� Ӧ���ϵ�֮��������ʼ����ֹ����Σ��
	
	timer_nvic_configuration();	  // ����TIM4�ж�
	
	
	debug_usart_init();						// ���Դ���	
	status_led_gpio_config();			// ָʾ��LED
	
  i2c_mpu6050_init_s();					// д�����ò���
	//i2c_mpu6050_config_mag_s();	// ���ò���ʼ��������	
  //i2c_mpu6050_init_mag_s();
  //i2c_ms5611_init_s();        // ��ʼ����ѹ��
	
  sys_tim_init(72,1000);		  	// ��ʱ��4��ʼ��1ms�����ڷɿ���ѭ����׼��ʱ
	
	
	spi_nrf_init();							  // ��ʼ���������ջ��豸	
	spi_nrf_rx_mode();						// ��ʼ������ģʽ
	
	
	//ParamSetDefault();						// ����PID����
	
	
	
	while(1)
	{
		
		// crazepony���Ʋ���
		if(loop100HzCnt>=10)
		{
			loop100HzCnt=0; //10msһ��ѭ��
			
			real_exec_prd[0] = micros()-start_time[0];//����ѭ��������ѭ��ʱ��
			start_time[0] = micros();
			
			// ����õ�����ŷ����
			IMUSO3Thread();
			
			if(imuCaliFlag)
      {
          if(IMU_Calibrate())//����1 У׼����
          {
             imuCaliFlag=0;
             imu.caliPass=1;
          }
      }
			
			// �ڻ������ʿ���
			CtrlAttiRate();
			// ������
			CtrlMotor();
		}
		
		// ��ȡң�������� 
		nrf_read_to_motors();
		go_arm_check();
		
		if(loop50HzFlag)
    {
      loop50HzFlag=0; // �ñ�־λ���ٶ������
      real_exec_prd[1]=micros()-start_time[1];
      start_time[1]=micros();

      set_reference();
		  CtrlAttiAng();
			// �⻷���ƽǶ�
		}
	}



	return 0;
}




