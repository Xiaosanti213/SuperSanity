/**
 * @file mpu6050_s.c
 *
 * mpu6050 ����ģ��I2CЭ��ģ���ȡ����
 * 
 */
 
 
 
#include "stm32f10x.h" 
 
#include "mpu6050.h"
#include "board_config.h"
#include "hmc5883l.h" 
#include "api_i2c.h"

#include <stdio.h>

 
 
static void i2c_mpu6050_delay_s(void); 
static uint8_t i2c_send_data_single_s(uint8_t reg, uint8_t byte);
//static uint8_t i2c_send_data(uint8_t reg, uint8_t* buffer, u8 num);
static uint8_t i2c_receive_data_s(u8 reg, uint8_t* byte_add, uint8_t num);
static uint8_t i2c_timeout_usercallback_s(uint8_t error_code);



#define I2C_WAIT_TIMEOUT		((u32)0x1000)






/**
 *  ����: i2c_mpu6050_init_s
 *
 *  ������mpu6050�豸��ʼ��
 *
 */
void i2c_mpu6050_init_s(void)
{
		
  mpu6050_i2c_gpio_config_s();//����GPIO����ģ��ģʽ
	
  i2c_mpu6050_delay_s();//�ϵ����ʱ��ֹ���ݳ���
	
	i2c_send_data_single_s(MPU6050_RA_PWR_MGMT_1, 0x00);	     																				//�������״̬
	i2c_send_data_single_s(MPU6050_RA_SMPLRT_DIV , MPU6050_SMPLRT_DEV_GY1k);	    								    //�����ǲ�����1kHz
	i2c_send_data_single_s(MPU6050_RA_CONFIG , MPU6050_DLPF_BW_5);	  								    						//�����ⲿ�ź�ͬ��
	i2c_send_data_single_s(MPU6050_RA_ACCEL_CONFIG , MPU6050_ACCEL_FS_2G);	  									  	  //���ü��ٶȼ�����+-2G
	i2c_send_data_single_s(MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_2000);     												//���������ò��Լ죬����+-2000deg/s
  
	i2c_mpu6050_check_s();//�����Ƿ���������
}	




/**
 *  ����: i2c_mpu6050_check_s
 *
 *  ������mpu6050�����豸����
 *
 */

uint8_t i2c_mpu6050_check_s(void)
{
	uint8_t value = 0;
	i2c_receive_data_s(MPU6050_RA_WHO_AM_I, &value, 1);
	//printf("value = %d\n", value);
	if(value == 0x68)
	{
		//printf("MPU6050 is working...\n");
		return SUCCESS;
	}
	return ERROR;
}






/**
 *
 * ���ƣ�i2c_mpu6050_read_acc_s
 *
 * ��������ȡ�Ӽ����� 16348 LSB/g
 *
 */
void i2c_mpu6050_read_acc_s(float* acc)
{
	
	int16_t acc_temp[3];//���м�����ǰ�λ����ʱ����float�洢��ʽ��һ��
	u8 buffer[6];//��Ϊ����������޷�������ν����Ҫ����Ĳ�������
	i2c_receive_data_s(MPU6050_RA_ACCEL_XOUT_H, buffer, 6);
	//��ȡ�Ӽ������׵�ַ
	acc_temp[0] = (buffer[0]<<8) | buffer[1];
	acc_temp[1] = (buffer[2]<<8) | buffer[3];
	acc_temp[2] = (buffer[4]<<8) | buffer[5];
	 
	//ת����g��λ
	acc[0] = (float)acc_temp[0] * 2 / 32768;
	acc[1] = (float)acc_temp[1] * 2 / 32768;
	acc[2] = (float)acc_temp[2] * 2 / 32768;
	printf("MPU6050 accx: %.2f%s%.2f%s%.2f%s", acc[0], " g accy: ", acc[1], " g accz: ", acc[2], " g\n");
	
}







/**
 *
 * ���ƣ�i2c_mpu6050_read_gyro_s
 *
 * ��������ȡ���������� 16.4LSB
 *
 */
void i2c_mpu6050_read_gyro_s(float* gyro)
{
	int16_t gyro_temp[3];
	uint8_t buffer[6];
	i2c_receive_data_s(MPU6050_RA_GYRO_XOUT_H, buffer, 6);
	//��ȡ�����������׵�ַ
	gyro_temp[0] = (buffer[0]<<8) | buffer[1];
	gyro_temp[1] = (buffer[2]<<8) | buffer[3];
	gyro_temp[2] = (buffer[4]<<8) | buffer[5];
		 
	//ת����dps��λ
	gyro[0] = (float)gyro_temp[0] * 2000/ 32768;
	gyro[1] = (float)gyro_temp[1] * 2000/ 32768;
	gyro[2] = (float)gyro_temp[2] * 2000/ 32768;
	
	printf("MPU6050 gyrox: %.2f%s%.2f%s%.2f%s", gyro[0], " dps gyroy: ", gyro[1], " dps gyroz: ", gyro[2], "dps\n");
	
}









/**
 *
 * ���ƣ�i2c_mpu6050_read_temp_s
 *
 * ��������ȡ�¶�����
 *
 */
void i2c_mpu6050_read_temp_s(float* temp)
{
	
	u8 buffer[2];
	int16_t temp_quant;
	i2c_receive_data_s(MPU6050_RA_TEMP_OUT_H, buffer, 2);
	//��ȡ�¶������׵�ַ
	temp_quant = (buffer[0]<<8) | buffer[1];
	//ת���������¶�
	temp[0] = (double)temp_quant/340.0 + 36.53;
	//printf("MPU6050�¶�: %.2f%s", temp[0], "\n");

}









/**
 *
 *  ���ƣ� i2c_mpu6050_delay_s
 *
 *  ������ mpu6050�ϵ���ʱ
 *
 */
void i2c_mpu6050_delay_s(void)
{
	uint16_t i=0,j=0;
  //�ڳ�ʼ��֮ǰҪ��ʱһ��ʱ�䣬��û����ʱ����ϵ�����ϵ����ݿ��ܻ����
  for(i=0; i < 1000;i++)
  {
    for(j=0; j < 1000;j++)
    {
      ;
    }
  }
}







/**
 *
 * ���ƣ�i2c_mpu6050_config_mag_s
 *
 * ��������MPU6050���ó�bypassģʽ�����ô����̲���
 *
 */

void i2c_mpu6050_config_mag_s(void)
{
	 i2c_send_data_single_s(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT);
	 // MPU6050���ó�I2C����ģʽ
	 i2c_send_data_single_s(MPU6050_RA_I2C_MST_CTRL, MPU6050_CLOCK_DIV_258);
	 // MPU6050�ʹӻ�ͨѶ����258KHz��������
	 i2c_send_data_single_s(MPU6050_RA_I2C_SLV4_CTRL, 0x13);
	 // ���ò���������50Hz
	 i2c_send_data_single_s(MPU6050_RA_I2C_MST_DELAY_CTRL, MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_BIT);
	 //���ô������ӳ�ʹ��
	 
	 i2c_send_data_single_s(MPU6050_RA_I2C_SLV0_ADDR, MPU6050_I2C_SLV_RW_WRITE|HMC5883L_ADDRESS);
   // �����̴ӻ���ַ
	 i2c_send_data_single_s(MPU6050_RA_I2C_SLV0_REG, HMC5883L_RA_CONFIG_RA);
   // ���������üĴ���CRA
   i2c_send_data_single_s(MPU6050_RA_I2C_SLV0_DO, (u8)(HMC5883L_MA_AVE_OUTPUT4|HMC5883L_DO_RATE_15));
	 // ���������ò���������4��ƽ��ֵ��������ݸ�������15Hz
	 i2c_send_data_single_s(MPU6050_RA_I2C_SLV0_CTRL, MPU6050_I2C_SLV_EN|0x01);
   // ��д��Ĵ�����ַ����д��1���ֽڲ�����EXT_SEN_R 0-1����һ����MPU6050_RA_USER_CTRL ʹ�����ݴ���
	
	 
	 i2c_send_data_single_s(MPU6050_RA_I2C_SLV0_ADDR, MPU6050_I2C_SLV_RW_WRITE|HMC5883L_ADDRESS);
   // �����̴ӻ���ַ
	 i2c_send_data_single_s(MPU6050_RA_I2C_SLV0_REG, HMC5883L_RA_CONFIG_RB);
   // ���������üĴ���CRB 
   i2c_send_data_single_s(MPU6050_RA_I2C_SLV0_DO, HMC5883L_GN_GAIN_330);
	 // ���������С��HMC5883L_GN_GAIN_230
	 i2c_send_data_single_s(MPU6050_RA_I2C_SLV0_CTRL, MPU6050_I2C_SLV_EN|0x01);
	 
	 
	 i2c_send_data_single_s(MPU6050_RA_I2C_SLV0_ADDR, MPU6050_I2C_SLV_RW_WRITE|HMC5883L_ADDRESS);
   // �����̴ӻ���ַ
	 i2c_send_data_single_s(MPU6050_RA_I2C_SLV0_REG, HMC5883L_RA_MODE_REG);
   // ������ģʽ�Ĵ��� 
   i2c_send_data_single_s(MPU6050_RA_I2C_SLV0_DO, HMC5883L_MD_CONT_MEAS);
	 // ������������ģʽ��HMC5883L_MD_CONT_MEAS
	 i2c_send_data_single_s(MPU6050_RA_I2C_SLV0_CTRL, MPU6050_I2C_SLV_EN|0x01);
}
	







/**
 *
 * ���ƣ�i2c_mpu6050_init_mag_s
 *
 * ��������MPU6050���ó�����ģʽ
 *
 */ 

void i2c_mpu6050_init_mag_s(void)
{
   i2c_send_data_single_s(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT);
	 // MPU6050���ó�I2C����ģʽ
	 i2c_send_data_single_s(MPU6050_RA_I2C_MST_CTRL, MPU6050_CLOCK_DIV_258);
	 // MPU6050�ʹӻ�ͨѶ����258KHz
	 i2c_send_data_single_s(MPU6050_RA_I2C_SLV0_ADDR, MPU6050_I2C_SLV_RW_READ|HMC5883L_ADDRESS);
   // �����̵�ַ
	 i2c_send_data_single_s(MPU6050_RA_I2C_SLV0_REG, HMC5883L_RA_DATAOUT_X_MSB);
	 // ���������üĴ�����ַ
	 i2c_send_data_single_s(MPU6050_RA_I2C_SLV0_CTRL, MPU6050_I2C_SLV_EN|0x06);
	 // ʹ�����ݴ��䣬���ݳ��ȣ�3*2�ֽ�
}

 




/**
 *
 * ���ƣ�i2c_mpu6050_read_mag_s
 *
 * ��������ȡ����������
 *
 */ 
void i2c_mpu6050_read_mag_s(float* mag)
{
	int16_t mag_temp[3];
	uint8_t buffer[6];
	// ʹ�����ݴ���֮�󣬼���ֱ��ͨ����ȡ6050��Ӧ�Ĵ�����ô���������
	i2c_receive_data_s(MPU6050_RA_EXT_SENS_DATA_00, buffer, 6);
	// ��ַMPU6050_RA_EXT_SENS_DATA_00: Reg73~96
	mag_temp[0] = (buffer[0]<<8) | buffer[1];   //magX-MB: 0x03 LB: 0x04
	mag_temp[1] = (buffer[2]<<8) | buffer[3];   //magZ-MB: 0x05 LB: 0x06
	mag_temp[2] = (buffer[4]<<8) | buffer[5];   //magY-MB: 0x07 LB: 0x08

	// ��������Ϊ230
  mag[0] = (float)mag_temp[0] * 4.35;
	mag[1] = (float)mag_temp[1] * 4.35;
	mag[2] = (float)mag_temp[2] * 4.35;
	
	printf("MPU6050 magx: %.2f%s%.2f%s%.2f%s", mag[0], " deg magy: ", mag[1], " deg magz: ", mag[2], "deg\n");
}




 


/**
 * ����: i2c_send_data_single_s
 *
 * ������MPU6050  ����ģ��Single-Byte Write Sequence���ֽڷ���ģʽ
 *
 */
 
uint8_t i2c_send_data_single_s(uint8_t reg, uint8_t byte)
{

	uint32_t i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	mpu6050_i2c_start_s();//1 ������ʼ�ź�	
	
	//i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	mpu6050_i2c_send_byte_s(MPU6050_SLAVE_WRITE_ADDRESS); //2 ����7λ�ӻ��豸��ַ��������Ƿ��յ���ַӦ��
	
	while(mpu6050_i2c_wait_ack_s())
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback_s(1);
		}
		i2c_wait_timeout--;
	}
	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	mpu6050_i2c_send_byte_s(reg);		//3 ����һ���ֽڵ�ַ(MPU6050 Single-Byte Write Sequence)
	
	while(mpu6050_i2c_wait_ack_s())
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback_s(2);
		}
		i2c_wait_timeout--;
	}

	mpu6050_i2c_send_byte_s(byte);		//4 ����һ���ֽ�����
	
	while(mpu6050_i2c_wait_ack_s())
	{
	  if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback_s(3);
		}
		i2c_wait_timeout--;
	}
		
	mpu6050_i2c_stop_s(); //5 ֹͣ�ź�
	
	return SUCCESS;
}





					
/**
 * ����: i2c_send_data_s
 *
 * ������MPU6050  Burst Write Sequence���ֽڷ���ģʽ
 *
 */
 
uint8_t i2c_send_data_s(uint8_t reg, uint8_t* buffer, u8 num)
{
	u8 index;

	uint32_t i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	//while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY)) // ����Ƿ���æ
  //{
  //  if((i2c_wait_timeout--) == 0) return i2c_timeout_usercallback(0);
  // }

	
	mpu6050_i2c_start_s();//1 ������ʼ�ź�	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	mpu6050_i2c_send_byte_s(MPU6050_SLAVE_WRITE_ADDRESS);  //2 ����7λ�ӻ��豸��ַ��������Ƿ��յ���ַӦ��
	
	while(mpu6050_i2c_wait_ack_s())
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback_s(2);
		}
		i2c_wait_timeout--;
	}
	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	mpu6050_i2c_send_byte_s(reg);		//3 ����һ���ֽڵ�ַ(MPU6050 Single-Byte Write Sequence)
	
	while(mpu6050_i2c_wait_ack_s())
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback_s(3);
		}
		i2c_wait_timeout--; 
	}

	for(index = 0; index < num; index++)
	{
		mpu6050_i2c_send_byte_s(*buffer);		//4 ����һ���ֽ�����
		
		while(mpu6050_i2c_wait_ack_s())
		{
		  if(i2c_wait_timeout == 0)
			{
				return i2c_timeout_usercallback_s(4);
			}
			i2c_wait_timeout--;
		}
		buffer++;//������û�н�����һ��ѭ���Ƿ����Σ��
	}
	
	mpu6050_i2c_stop_s(); //5 ֹͣ�ź�
	
	return SUCCESS;
}







/**
 * ����: i2c_receive_data_s
 *
 * ������Burst Read Sequence ���ֽڽ���ģʽ
 *
 */

uint8_t i2c_receive_data_s(u8 reg, uint8_t* buffer, uint8_t num)
{
	uint32_t i2c_wait_timeout = I2C_WAIT_TIMEOUT;

	mpu6050_i2c_start_s();  // 1 ��ʼ����
	
	mpu6050_i2c_send_byte_s(MPU6050_SLAVE_WRITE_ADDRESS); // 2 ���ʹӻ��豸��ַ
	while(mpu6050_i2c_wait_ack_s())
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback_s(2);
		}
		i2c_wait_timeout--;
	}	
	
	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;	
	mpu6050_i2c_send_byte_s(reg);		//3 ����һ���ֽڵ�ַ
	
	while(mpu6050_i2c_wait_ack_s())
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback_s(3);
		}
		i2c_wait_timeout--;
	}
	
	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	mpu6050_i2c_start_s();  // 4 Burst Read Sequenceģʽ��ʼ����

	mpu6050_i2c_send_byte_s(MPU6050_SLAVE_READ_ADDRESS); // 5 ���ʹӻ��豸��ַ
	while(mpu6050_i2c_wait_ack_s())
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback_s(5);
		}
		i2c_wait_timeout--;
	}
	
	while(num)// ѭ����ȡ
	{
		if (num == 1)//��ȡ���һ�ֽ�����
		{
			*buffer = mpu6050_i2c_read_byte_s(0);//���һ������ΪNackģʽ����Nack�ź�
			mpu6050_i2c_stop_s(); 
		}
    else // ��������Ҫint�����ж϶�ȡ
    {      
      /* Read a byte from the slave */
      *buffer = mpu6050_i2c_read_byte_s(1);
			// ����Ӧ���ź�
      /* Point to the next location where the byte read will be saved */
      buffer++; 
      
    }
		num--;        
	}
	
	
	return SUCCESS;//�ɹ���������
}
	



/**
 * ����: i2c_timeout_usercallback
 *
 * �������ȴ���ʱ�ص�����
 *
 */
uint8_t i2c_timeout_usercallback_s(uint8_t error_code)
{
	printf("MPU6050 �ȴ���ʱ��errorCode =%d\n", error_code);
	return 0;
}









