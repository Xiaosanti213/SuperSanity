/**
 *
 * @file sensors.c
 *
 * ��������ʼ�������ݲɼ�
 *
 **/

#include <stm32f10x.h>

#include "board_config.h"
#include "debug.h"
#include "hmc5883l.h"
#include "mpu6050.h"
#include "nRF24L01.h"
#include "ms5611.h"
#include "sensors.h"
#include "app_cfg.h"


#include <stdio.h>




//static void nrf_read_to_motors(u16* rc_command);
		


	
	
	


/**
 *
 * ���ƣ�sensors_init
 *
 * ������ȫ���������豸��ʼ��
 *
 */ 
void sensors_init(void)
{
	debug_usart_init();						// ���Դ���	
	//os_tick_init(); 						// ����ϵͳ�δ�ʱ��
	
	status_led_gpio_config();			// ָʾ��LED
	
  i2c_mpu6050_init_s();					// д�����ò���
	i2c_mpu6050_config_mag_s();		// ��ȡ�ӻ�����
  i2c_mpu6050_init_mag_s();
  i2c_ms5611_init_s();
	
	spi_nrf_init();								// ����ҡ������
	spi_nrf_rx_mode();
	tim_config(); 
}




/**
 *
 * ���ƣ�get_sensors_data
 *
 * ��������ȡ����������
 *
 */ 
void get_sensors_data(sd* sdata)
{
	
  //if(i2c_mpu6050_check_s() && i2c_ms5611_calculate_s())
	//if(i2c_mpu6050_check_s())
	//{
			i2c_mpu6050_read_acc_s(sdata->acc);
			i2c_mpu6050_read_gyro_s(sdata->gyro);
			i2c_mpu6050_read_temp_s(sdata->temp);
		
			//printf("HMC5883L���ݣ� \n");
	    i2c_mpu6050_read_mag_s(sdata->mag); 
	    //printf("magx: %d%s%d%s%d", sd->mag[0], "magy: ", sd->mag[1], "magz: ", sd->mag[2]);
			sdata->press = i2c_ms5611_calculate_s();
			//printf("MS5611���ݣ� \n");
			//printf("press: %d\n", sd->press);
	//}
			nrf_read_to_motors(sdata->rc_command);
}


/**
 *
 * ���ƣ�nrf_read_to_motors
 *
 * ��������u8[4]���յ�������ӳ�䵽10000~20000��
 *
 */
void nrf_read_to_motors(u16* rc_command)
{
	 u8 rxbuf[8];	
	/*�жϽ���״̬ �յ�����*/
	if(spi_nrf_rx_packet(rxbuf))
	{
		//STATUS_LED_ON;//ָʾ�Ʊ�ʾ��ǰ���յ�����
		//������������Ѿ���motorֵӳ�䵽��1000~2000��
		rc_command[0] = (float)(rxbuf[1]<<8 | rxbuf[0])/4096*1000 + 1000;
		rc_command[1] = (float)(rxbuf[3]<<8 | rxbuf[2])/4096*1000 + 1000;
		rc_command[2] = (float)(rxbuf[5]<<8 | rxbuf[4])/4096*1000 + 1000;
		rc_command[3] = (float)(rxbuf[7]<<8 | rxbuf[6])/4096*1000 + 1000;
		
	 	//printf("\r\n �ӻ��˽��յ�ң�����������ݣ�\n");
	  //printf("RA���ָ���%d%s", rc_command[0], "\n");
	  //printf("LE����������%d%s", rc_command[1], "\n");
	  //printf("RT�������ţ�%d%s", rc_command[2], "\n");
	  //printf("LR���ַ���%d%s", rc_command[3], "\n");
		
	 // ���ڿ��ı������˵����ռ�ձ�Ƶ�ʸı�5Hz(2000)���轫ֵӳ�䵽0~2000��
	 	rc_command[0] = (rc_command[0] - 1000)*2;
	  rc_command[1] = (rc_command[1] - 1000)*2;
    rc_command[2] = (rc_command[2] - 1000)*2;
	 	rc_command[3] = (rc_command[3] - 1000)*2;	


    //printf("\r\n �ӻ��˽��յ�ң�����������ݣ�\n");
	  //printf("RA���ָ���%d%s", rc_command[0], "\n");
	  //printf("LE����������%d%s", rc_command[1], "\n");
	  //printf("RT�������ţ�%d%s", rc_command[2], "\n");
	  //printf("LR���ַ���%d%s", rc_command[3], "\n");		
	} 
  else 
		//printf("���ݽ���ʧ�ܣ�������·����...\n");
	;
} 







