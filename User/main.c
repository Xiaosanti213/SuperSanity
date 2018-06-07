/**
 *
 * @file main.c
 *
 * 主循环
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
	
	//定时计数开启
	//cycleCounterInit();	    			// 本质是将usTicks初始化为72
  SystemClock_HSE(9);
	
	SysTick_Config(72000L);					// 72000每隔1ms触发中断
 
  
	tim_config(); 								// 初始化电机PWM定时器 控制输出 应该上电之后立即初始化防止出现危险
	
	timer_nvic_configuration();	  // 配置TIM4中断
	
	
	debug_usart_init();						// 调试串口	
	status_led_gpio_config();			// 指示灯LED
	
  i2c_mpu6050_init_s();					// 写入配置参数
	//i2c_mpu6050_config_mag_s();	// 配置并初始化磁罗盘	
  //i2c_mpu6050_init_mag_s();
  //i2c_ms5611_init_s();        // 初始化气压计
	
  sys_tim_init(72,1000);		  	// 定时器4初始化1ms，用于飞控主循环基准定时
	
	
	spi_nrf_init();							  // 初始化数传接收机设备	
	spi_nrf_rx_mode();						// 初始化接收模式
	
	
	//ParamSetDefault();						// 载入PID参数
	
	
	
	while(1)
	{
		
		// crazepony控制策略
		if(loop100HzCnt>=10)
		{
			loop100HzCnt=0; //10ms一次循环
			
			real_exec_prd[0] = micros()-start_time[0];//更新循环变量和循环时间
			start_time[0] = micros();
			
			// 解算得到三轴欧拉角
			IMUSO3Thread();
			
			if(imuCaliFlag)
      {
          if(IMU_Calibrate())//返回1 校准正常
          {
             imuCaliFlag=0;
             imu.caliPass=1;
          }
      }
			
			// 内环角速率控制
			CtrlAttiRate();
			// 电机输出
			CtrlMotor();
		}
		
		// 读取遥控器舵量 
		nrf_read_to_motors();
		go_arm_check();
		
		if(loop50HzFlag)
    {
      loop50HzFlag=0; // 用标志位减少多次运算
      real_exec_prd[1]=micros()-start_time[1];
      start_time[1]=micros();

      set_reference();
		  CtrlAttiAng();
			// 外环控制角度
		}
	}



	return 0;
}




