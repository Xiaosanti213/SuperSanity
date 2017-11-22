/**
 * @file mpu6050.c
 *
 * mpu6050 ģ����̬���
 * 
 */
 
#include "stm32f10x.h" 
 
#include "mpu6050.h"
#include "board_config.h"
#include "api_i2c.h"

 
 
 
 
/**
 *  ����: i2c_mpu6050_init
 *
 *  ������mpu6050�豸��ʼ��
 *
 */
void i2c_mpu6050_init(void)
{
	
	gpio_clk_config();//������
	
  tim_config();
	
  i2c_mpu6050_delay();
	
	i2c_mpu6050_write_reg(MPU6050_RA_PWR_MGMT_1, 0x00);	     																				//�������״̬
	i2c_mpu6050_write_reg(MPU6050_RA_SMPLRT_DIV , MPU6050_SMPLRT_DEV_GY1k);	    										//�����ǲ�����1kHz
	i2c_mpu6050_write_reg(MPU6050_RA_CONFIG , MPU6050_EXT_SYNC_ACCEL_YOUT_L);	  										//ͬ���ź�MPU6050_EXT_SYNC_ACCEL_YOUT_L
	i2c_mpu6050_write_reg(MPU6050_RA_ACCEL_CONFIG , MPU6050_ACCEL_FS_2G|MPU6050_DHPF_5);	  				//���ü��ٶȴ�����������2Gģʽ, Digital High Pass Filter��
	i2c_mpu6050_write_reg(MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_2000);     												//���������ò��Լ죬2000deg/s����

}	










/**
 *  ����: i2c_mpu6050_check
 *
 *  ������mpu6050�����豸����
 *
 */

uint8_t i2c_mpu6050_check(void)
{
	uint8_t value;
	value = i2c_mpu6050_read_reg(MPU6050_RA_WHO_AM_I);
	if(value == MPU6050_WHO_AM_I)
	{
		return SUCCESS;
	}
	return ERROR;
}








/**
 *
 * ���ƣ�i2c_mpu6050_write_reg
 *
 * ������д��Ĵ�������
 *
 */
void i2c_mpu6050_write_reg(uint8_t reg, uint8_t value)
{
	
	i2c_send_data(MPU6050_I2C, MPU6050_SLAVE_ADDRESS, reg);
	i2c_send_data(MPU6050_I2C, MPU6050_SLAVE_ADDRESS, value);
	
}








/**
 *
 * ���ƣ�i2c_mpu6050_read_reg
 *
 * �����������Ĵ�������
 *
 */
uint8_t i2c_mpu6050_read_reg(uint8_t reg)
{

	uint8_t value;
	i2c_send_data(MPU6050_I2C, MPU6050_SLAVE_ADDRESS, reg);
	i2c_receive_data(MPU6050_I2C, MPU6050_SLAVE_ADDRESS, value);
	return value;

}




/**
 * 
 * ���ƣ�i2c_mpu6050_write_buffer
 *
 * ������д�뻺���ֵ
 *
 */
void i2c_mpu6050_write_buffer(uint8_t reg, uint8_t* pbuffer, uint8_t num)
{
	uint8_t index;
	for(index = 0; index < num; index++)
	{
		i2c_mpu6050_write_reg(reg, *pbuffer);
		pbuffer++;
	}
}






/**
 * 
 * ���ƣ�i2c_mpu6050_read_buffer
 *
 * ��������ȡ�����ֵ
 *
 */
void i2c_mpu6050_read_buffer(uint8_t reg, uint8_t* pbuffer, uint8_t num)
{
  uint8_t index;
	for(index = 0;index < num; index++)
	{
	  *pbuffer = i2c_mpu6050_read_reg(reg);
		pbuffer++;
	}
}






/**
 *
 * ���ƣ�i2c_mpu6050_read_acc
 *
 * ��������ȡ�Ӽ����� 16348 LSB/g
 *
 */
void i2c_mpu6050_read_acc(uint16_t* acc)
{
	
	uint8_t* buffer;
	i2c_mpu6050_read_buffer(MPU6050_RA_ACCEL_XOUT_H, buffer, 6);
	//��ȡ�Ӽ������׵�ַ
	acc[0] = (buffer[0]<<8) | buffer[1];
	acc[1] = (buffer[2]<<8) | buffer[3];
	acc[2] = (buffer[4]<<8) | buffer[5];
	
}







/**
 *
 * ���ƣ�i2c_mpu6050_read_gyro
 *
 * ��������ȡ���������� 16.4LSB
 *
 */
void i2c_mpu6050_read_gyro(uint16_t* gyro)
{
	
	uint8_t* buffer;
	i2c_mpu6050_read_buffer(MPU6050_RA_GYRO_XOUT_H, buffer, 6);
	//��ȡ�����������׵�ַ
	gyro[0] = (buffer[0]<<8) | buffer[1];
	gyro[1] = (buffer[2]<<8) | buffer[3];
	gyro[2] = (buffer[4]<<8) | buffer[5];
	
}












/**
 *
 * ���ƣ�i2c_mpu6050_read_temp
 *
 * ��������ȡ�¶�����
 *
 */
void i2c_mpu6050_read_temp(uint16_t* temp)
{
	
	uint8_t* buffer;
	uint16_t temp_quant;
	i2c_mpu6050_read_buffer(MPU6050_RA_TEMP_OUT_H, buffer, 2);
	//��ȡ�¶������׵�ַ
	temp_quant = (buffer[0]<<8) | buffer[1];
	//ת���������¶�
	temp[0] = (double)temp_quant/340.0 + 36.53;
	
}









/**
 *
 *  ���ƣ� i2c_mpu6050_delay
 *
 *  ������ mpu6050�ϵ���ʱ
 *
 */
void i2c_mpu6050_delay(void)
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






















