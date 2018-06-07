/**
 *
 * @file motor.c
 *
 * �Կ��ı�������ַ���ģʽ����
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

#define YAW_DB	 70 //ƫ����������
#define PR_DB		 50 //������ת��������


float reference[4] = {0,0,0,0};
int16_t rccommand[4] = {500, 500, 500, 500};
u8 arm_motors_flag = 0; //Ĭ������״̬



 
 /**
 *
 * ����: go_arm_check()
 *
 * �������������
 *
 */ 
void go_arm_check(void)
{
	static int16_t arming_counter;
	u16 temp;
	// ����ͨ���ĸ����������ж� ������Χ
	// ���ڿ��ı������˵���Ѿ�ӳ�䵽[0-1000]
	
	if (rccommand[2] > 15) {
  // ǰ������λ����͵㣬����ֱ���˳�
      arming_counter = 0;
      return;  
   } 
	 temp = rccommand[3];
	 // ȡ���������
	 if (temp > 970)
	 {
			if(arming_counter < ARM_DELAY)
			{
				arming_counter ++;
				//printf("********************************************\n");
				//printf("��ǰҡ��״̬��������ֵ��%d\n", arming_counter);
			}
			else if(arming_counter == ARM_DELAY)
			{
				arming_counter = 0;
				// ����������
				STATUS_LED_ON;
				// ָʾ�Ƶ���
				arm_motors_flag = 1;
				// ��������ź�
			}
	 }
	 else if(temp < 15)
	 // ���ͨѶ״��������������ź�һֱ��0����֤ARM_MOTORS==0
	 {
			if(arming_counter < DISARM_DELAY)
			{
				arming_counter ++; 
				//printf("********************************************\n");
				//printf("��ǰҡ��״̬��������ֵ��%d\n", arming_counter);
			}
			else if(arming_counter == DISARM_DELAY)
			{
			  arming_counter = 0;
				// ����������
				STATUS_LED_OFF;
				// ����״ָ̬ʾ��Ϩ��
				arm_motors_flag = 0;
				// ��ֹ����ź�
			}
	 }
}

 
 
 
 

/**
 *
 * ���ƣ�nrf_read_to_motors
 *
 * ��������u8[4]���յ�������ӳ�䵽0~1000��
 *
 */
void nrf_read_to_motors(void)
{
	 u8 rxbuf[8];
	 /* �жϽ���״̬ �յ����� */
	 spi_nrf_rx_packet(rxbuf);
	
		// ��motorֵӳ�䵽��0~1000��
		rccommand[0] = (float)(rxbuf[1]<<8 | rxbuf[0])/4096*1000 ;
		rccommand[1] = (float)(rxbuf[3]<<8 | rxbuf[2])/4096*1000 ;
		rccommand[2] = (float)(rxbuf[5]<<8 | rxbuf[4])/4096*1000 ;
		rccommand[3] = (float)(rxbuf[7]<<8 | rxbuf[6])/4096*1000 ;
		
		/*
    printf("\r\n �ӻ��˽��յ�ң�����������ݣ�\n");
	  printf("R--ail: %d%s", rccommand[0], "   \n");
	  printf("L--ele: %d%s", rccommand[1], "   \n");
	  printf("R--thr: %d%s", rccommand[2], "   \n");
	  printf("L--rud: %d%s", rccommand[3], "   \n");		
		*/
		//printf("���ݽ���ʧ�ܣ�������·����...\n");
} 



 
/**
 *
 * ���ƣ�set_reference
 *
 * ������ҡ���ź���ת��Ϊ���Ʋο�ֵ
 *
 */
void set_reference(void)
{

	// Լ��������Ҫ��������
  CONSTRAIN(rccommand[THROTTLE],0,1000);
  CONSTRAIN(rccommand[YAW],0,1000);
  CONSTRAIN(rccommand[PITCH],0,1000);
  CONSTRAIN(rccommand[ROLL],0,1000);
	
	// ����������Χ��ӳ��ɲο��źţ�����-500~500 ����-
	reference[0] = Angle_Max * dbScaleLinear((rccommand[ROLL] - 500),500,PR_DB);       //1����2����3����4����
	reference[1] = -Angle_Max * dbScaleLinear((rccommand[PITCH] - 500),500,PR_DB);     //�Ҵ��ϴ�Ϊ����pitch���������˶�����С
	reference[2] = YAW_RATE_MAX * dbScaleLinear((rccommand[YAW] - 500),500,YAW_DB);    //ƫ�����ʲο�����
	reference[3] = rccommand[THROTTLE];        																				  //�߶Ȳο�
	
}


 

