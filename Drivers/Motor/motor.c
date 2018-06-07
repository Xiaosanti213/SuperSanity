/**
 *
 * @file motor.c
 *
 * 对空心杯电机几种飞行模式配置
 *
 **/
 
#include "motor.h"
#include "board_config.h"
#include "stm32f10x_tim.h"
#include "stm32f10x.h"
#include "sensors.h"
#include "IMU.h"
#include "nRF24L01.h"
#include <stdio.h>
#include "control.h"


#define CONSTRAIN(x,min,max) {if(x<min) x=min; if(x>max) x=max;}

#define YAW_DB	 70 //偏航死区长度
#define PR_DB		 50 //俯仰滚转死区长度


float reference[4] = {0,0,0,0};
int16_t rccommand[4] = {500, 500, 500, 500};
u8 arm_motors_flag = 0; //默认上锁状态



 
 /**
 *
 * 名称: go_arm_check()
 *
 * 描述：电机解锁
 *
 */ 
void go_arm_check(void)
{
	static int16_t arming_counter;
	u16 temp;
	// 下面通过四个舵量进行判断 舵量范围
	// 对于空心杯电机来说，已经映射到[0-1000]
	
	if (rccommand[2] > 15) {
  // 前提油门位于最低点，否则直接退出
      arming_counter = 0;
      return;  
   } 
	 temp = rccommand[3];
	 // 取出方向舵量
	 if (temp > 970)
	 {
			if(arming_counter < ARM_DELAY)
			{
				arming_counter ++;
				//printf("********************************************\n");
				//printf("当前摇杆状态解锁计数值：%d\n", arming_counter);
			}
			else if(arming_counter == ARM_DELAY)
			{
				arming_counter = 0;
				// 计数器清零
				STATUS_LED_ON;
				// 指示灯点亮
				arm_motors_flag = 1;
				// 可以输出信号
			}
	 }
	 else if(temp < 15)
	 // 如果通讯状况不良，则舵量信号一直是0，保证ARM_MOTORS==0
	 {
			if(arming_counter < DISARM_DELAY)
			{
				arming_counter ++; 
				//printf("********************************************\n");
				//printf("当前摇杆状态解锁计数值：%d\n", arming_counter);
			}
			else if(arming_counter == DISARM_DELAY)
			{
			  arming_counter = 0;
				// 计数器清零
				STATUS_LED_OFF;
				// 上锁状态指示灯熄灭
				arm_motors_flag = 0;
				// 禁止输出信号
			}
	 }
}

 
 
 
 

/**
 *
 * 名称：nrf_read_to_motors
 *
 * 描述：将u8[4]接收到的数据映射到0~1000上
 *
 */
void nrf_read_to_motors(void)
{
	 u8 rxbuf[8];
	 /* 判断接收状态 收到数据 */
	 spi_nrf_rx_packet(rxbuf);
	
		// 将motor值映射到了0~1000上
		rccommand[0] = (float)(rxbuf[1]<<8 | rxbuf[0])/4096*1000 ;
		rccommand[1] = (float)(rxbuf[3]<<8 | rxbuf[2])/4096*1000 ;
		rccommand[2] = (float)(rxbuf[5]<<8 | rxbuf[4])/4096*1000 ;
		rccommand[3] = (float)(rxbuf[7]<<8 | rxbuf[6])/4096*1000 ;
		
		/*
    printf("\r\n 从机端接收到遥控器杆量数据：\n");
	  printf("R--ail: %d%s", rccommand[0], "   \n");
	  printf("L--ele: %d%s", rccommand[1], "   \n");
	  printf("R--thr: %d%s", rccommand[2], "   \n");
	  printf("L--rud: %d%s", rccommand[3], "   \n");		
		*/
		//printf("数据接收失败，请检查线路连接...\n");
} 



 
/**
 *
 * 名称：set_reference
 *
 * 描述：摇杆信号量转化为控制参考值
 *
 */
void set_reference(void)
{

	// 约束舵量不要超出限制
  CONSTRAIN(rccommand[THROTTLE],0,1000);
  CONSTRAIN(rccommand[YAW],0,1000);
  CONSTRAIN(rccommand[PITCH],0,1000);
  CONSTRAIN(rccommand[ROLL],0,1000);
	
	// 设置死区范围并映射成参考信号：三轴-500~500 油门-
	reference[0] = Angle_Max * dbScaleLinear((rccommand[ROLL] - 500),500,PR_DB);       //1副翼2升降3油门4方向
	reference[1] = -Angle_Max * dbScaleLinear((rccommand[PITCH] - 500),500,PR_DB);     //右打，上打为正，pitch角增大，拉杆舵量减小
	reference[2] = YAW_RATE_MAX * dbScaleLinear((rccommand[YAW] - 500),500,YAW_DB);    //偏航速率参考幅度
	reference[3] = rccommand[THROTTLE];        																				  //高度参考
	
}


 

