/**
 * @file mpu6050.c
 *
 * mpu6050 模块姿态检测
 * 
 */
 
#include "stm32f10x.h" 
 
#include "mpu6050.h"
#include "board_config.h"
#include "api_i2c.h"

 
 
 
 
/**
 *  名称: i2c_mpu6050_init
 *
 *  描述：mpu6050设备初始化
 *
 */
void i2c_mpu6050_init(void)
{
	
	gpio_clk_config();//调试用
	
  tim_config();
	
  i2c_mpu6050_delay();
	
	i2c_mpu6050_write_reg(MPU6050_RA_PWR_MGMT_1, 0x00);	     																				//解除休眠状态
	i2c_mpu6050_write_reg(MPU6050_RA_SMPLRT_DIV , MPU6050_SMPLRT_DEV_GY1k);	    										//陀螺仪采样率1kHz
	i2c_mpu6050_write_reg(MPU6050_RA_CONFIG , MPU6050_EXT_SYNC_ACCEL_YOUT_L);	  										//同步信号MPU6050_EXT_SYNC_ACCEL_YOUT_L
	i2c_mpu6050_write_reg(MPU6050_RA_ACCEL_CONFIG , MPU6050_ACCEL_FS_2G|MPU6050_DHPF_5);	  				//配置加速度传感器工作在2G模式, Digital High Pass Filter？
	i2c_mpu6050_write_reg(MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_2000);     												//陀螺仪设置不自检，2000deg/s量程

}	










/**
 *  名称: i2c_mpu6050_check
 *
 *  描述：mpu6050检验设备连接
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
 * 名称：i2c_mpu6050_write_reg
 *
 * 描述：写入寄存器参数
 *
 */
void i2c_mpu6050_write_reg(uint8_t reg, uint8_t value)
{
	
	i2c_send_data(MPU6050_I2C, MPU6050_SLAVE_ADDRESS, reg);
	i2c_send_data(MPU6050_I2C, MPU6050_SLAVE_ADDRESS, value);
	
}








/**
 *
 * 名称：i2c_mpu6050_read_reg
 *
 * 描述：读出寄存器参数
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
 * 名称：i2c_mpu6050_write_buffer
 *
 * 描述：写入缓存的值
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
 * 名称：i2c_mpu6050_read_buffer
 *
 * 描述：读取缓存的值
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
 * 名称：i2c_mpu6050_read_acc
 *
 * 描述：读取加计数据 16348 LSB/g
 *
 */
void i2c_mpu6050_read_acc(uint16_t* acc)
{
	
	uint8_t* buffer;
	i2c_mpu6050_read_buffer(MPU6050_RA_ACCEL_XOUT_H, buffer, 6);
	//读取加计数据首地址
	acc[0] = (buffer[0]<<8) | buffer[1];
	acc[1] = (buffer[2]<<8) | buffer[3];
	acc[2] = (buffer[4]<<8) | buffer[5];
	
}







/**
 *
 * 名称：i2c_mpu6050_read_gyro
 *
 * 描述：读取陀螺仪数据 16.4LSB
 *
 */
void i2c_mpu6050_read_gyro(uint16_t* gyro)
{
	
	uint8_t* buffer;
	i2c_mpu6050_read_buffer(MPU6050_RA_GYRO_XOUT_H, buffer, 6);
	//读取陀螺仪数据首地址
	gyro[0] = (buffer[0]<<8) | buffer[1];
	gyro[1] = (buffer[2]<<8) | buffer[3];
	gyro[2] = (buffer[4]<<8) | buffer[5];
	
}












/**
 *
 * 名称：i2c_mpu6050_read_temp
 *
 * 描述：读取温度数据
 *
 */
void i2c_mpu6050_read_temp(uint16_t* temp)
{
	
	uint8_t* buffer;
	uint16_t temp_quant;
	i2c_mpu6050_read_buffer(MPU6050_RA_TEMP_OUT_H, buffer, 2);
	//读取温度数据首地址
	temp_quant = (buffer[0]<<8) | buffer[1];
	//转换成摄氏温度
	temp[0] = (double)temp_quant/340.0 + 36.53;
	
}









/**
 *
 *  名称： i2c_mpu6050_delay
 *
 *  描述： mpu6050上电延时
 *
 */
void i2c_mpu6050_delay(void)
{
	uint16_t i=0,j=0;
  //在初始化之前要延时一段时间，若没有延时，则断电后再上电数据可能会出错
  for(i=0; i < 1000;i++)
  {
    for(j=0; j < 1000;j++)
    {
      ;
    }
  }
}






















