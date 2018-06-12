/**
 * @file mpu6050_s.c
 *
 * mpu6050 软件模拟I2C协议模块获取数据
 * 
 */
 
 
 
#include "stm32f10x.h" 
#include "board_config.h"
#include "mpu6050.h"
#include "hmc5883l.h" 
#include "api_i2c.h"
#include <stdio.h>
#include "sensors.h"

 
 
static void i2c_mpu6050_delay_s(void); 
static uint8_t i2c_send_data_single_s(uint8_t reg, uint8_t byte);
static uint8_t i2c_receive_data_s(u8 reg, uint8_t* byte_add, uint8_t num);






/**
 *  名称: i2c_mpu6050_init_s
 *
 *  描述：mpu6050设备初始化
 *
 */
void i2c_mpu6050_init_s(void)
{
		
  mpu6050_i2c_gpio_config_s();//配置GPIO软件模拟模式
	
  i2c_mpu6050_delay_s();//上电后延时防止数据出错
	
	i2c_send_data_single_s(MPU6050_RA_PWR_MGMT_1, 0x00);	     																				//解除休眠状态
	i2c_send_data_single_s(MPU6050_RA_SMPLRT_DIV , MPU6050_SMPLRT_DEV_GY1k);	    								    //陀螺仪采样率1kHz
	i2c_send_data_single_s(MPU6050_RA_CONFIG , MPU6050_DLPF_BW_5);	  								    						//禁用外部信号同步
	i2c_send_data_single_s(MPU6050_RA_ACCEL_CONFIG , MPU6050_ACCEL_FS_2G);	  									  	  //配置加速度计量程+-2G
	i2c_send_data_single_s(MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_2000);     												//陀螺仪设置不自检，量程+-2000deg/s
  
	i2c_mpu6050_check_s();//测试是否连接正常
}	




/**
 *  名称: i2c_mpu6050_check_s
 *
 *  描述：mpu6050检验设备连接
 *
 */

uint8_t i2c_mpu6050_check_s(void)
{
	uint8_t value = 0;
	i2c_receive_data_s(MPU6050_RA_WHO_AM_I, &value, 1);
	if(value == 0x68)
	{
		//printf("MPU6050 is working...\n");
		return SUCCESS;
	}
	return ERROR;
}






/**
 *
 * 名称：i2c_mpu6050_read_acc_s
 *
 * 描述：读取加计数据 16348 LSB/g
 *
 */
void i2c_mpu6050_read_acc_s(float* acc)
{
	
	int16_t acc_temp[3];//这个大小可以优化的
	u8 buffer[6];
	i2c_receive_data_s(MPU6050_RA_ACCEL_XOUT_H, buffer, 6);//读取加计数据首地址
	
	acc_temp[0] = (buffer[0]<<8) | buffer[1];
	acc_temp[1] = (buffer[2]<<8) | buffer[3];
	acc_temp[2] = (buffer[4]<<8) | buffer[5];
	 
	//转化成g单位
	acc[0] = (float)acc_temp[0] * 2 / 32768;
	acc[1] = (float)acc_temp[1] * 2 / 32768;
	acc[2] = (float)acc_temp[2] * 2 / 32768;
	//printf("MPU6050 Accel ( g  ): %.2f%s%.2f%s%.2f%s", acc[0], "   ", acc[1], "   ", acc[2], "   \n");
	
}







/**
 *
 * 名称：i2c_mpu6050_read_gyro_s
 *
 * 描述：读取陀螺仪数据 16.4LSB
 *
 */
void i2c_mpu6050_read_gyro_s(float* gyro)
{
	int16_t gyro_temp[3];
	uint8_t buffer[6];
	i2c_receive_data_s(MPU6050_RA_GYRO_XOUT_H, buffer, 6);//读取陀螺仪数据首地址
	
	gyro_temp[0] = (buffer[0]<<8) | buffer[1];
	gyro_temp[1] = (buffer[2]<<8) | buffer[3];
	gyro_temp[2] = (buffer[4]<<8) | buffer[5];
		 
	//转化成dps单位
	gyro[0] = (float)gyro_temp[0] * 2000/ 32768;
	gyro[1] = (float)gyro_temp[1] * 2000/ 32768;
	gyro[2] = (float)gyro_temp[2] * 2000/ 32768;
	
	//printf("MPU6050 Gyro  (dps ): %.2f%s%.2f%s%.2f%s", gyro[0], "   ", gyro[1], "    ", gyro[2], "      \n");
	
}









/**
 *
 * 名称：i2c_mpu6050_read_temp_s
 *
 * 描述：读取温度数据
 *
 */
void i2c_mpu6050_read_temp_s(float* temp)
{
	
	u8 buffer[2];
	int16_t temp_quant;
	i2c_receive_data_s(MPU6050_RA_TEMP_OUT_H, buffer, 2); 		//读取温度数据首地址
	temp_quant = (buffer[0]<<8) | buffer[1];					
	temp[0] = (double)temp_quant/340.0 + 36.53;								//转换成摄氏温度
	
}









/**
 *
 *  名称： i2c_mpu6050_delay_s
 *
 *  描述： mpu6050上电延时，防止数据出错
 *
 */
void i2c_mpu6050_delay_s(void)
{
	uint16_t i=0,j=0;

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
 * 名称：i2c_mpu6050_config_mag_s
 *
 * 描述：将MPU6050配置成bypass模式，配置磁罗盘参数
 *
 */

void i2c_mpu6050_config_mag_s(void)
{
	 i2c_send_data_single_s(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT);
	 // MPU6050配置成I2C主机模式
	 i2c_send_data_single_s(MPU6050_RA_I2C_MST_CTRL, MPU6050_CLOCK_DIV_258);
	 // MPU6050和从机通讯速率258KHz最慢的了
	 i2c_send_data_single_s(MPU6050_RA_I2C_SLV4_CTRL, 0x13);
	 // 配置采样磁罗盘50Hz
	 i2c_send_data_single_s(MPU6050_RA_I2C_MST_DELAY_CTRL, MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_BIT);
	 //外置传感器延迟使能
	 
	 i2c_send_data_single_s(MPU6050_RA_I2C_SLV0_ADDR, MPU6050_I2C_SLV_RW_WRITE|HMC5883L_ADDRESS);
   // 磁罗盘从机地址
	 i2c_send_data_single_s(MPU6050_RA_I2C_SLV0_REG, HMC5883L_RA_CONFIG_RA);
   // 磁罗盘配置寄存器CRA
   i2c_send_data_single_s(MPU6050_RA_I2C_SLV0_DO, (u8)(HMC5883L_MA_AVE_OUTPUT4|HMC5883L_DO_RATE_15));
	 // 磁罗盘配置参数：连续4个平均值输出，数据更新速率15Hz
	 i2c_send_data_single_s(MPU6050_RA_I2C_SLV0_CTRL, MPU6050_I2C_SLV_EN|0x01);
   // 先写入寄存器地址，再写入1个字节参数，EXT_SEN_R 0-1构成一个字MPU6050_RA_USER_CTRL 使能数据传输
	
	 
	 i2c_send_data_single_s(MPU6050_RA_I2C_SLV0_ADDR, MPU6050_I2C_SLV_RW_WRITE|HMC5883L_ADDRESS);
   // 磁罗盘从机地址
	 i2c_send_data_single_s(MPU6050_RA_I2C_SLV0_REG, HMC5883L_RA_CONFIG_RB);
   // 磁罗盘配置寄存器CRB 
   i2c_send_data_single_s(MPU6050_RA_I2C_SLV0_DO, HMC5883L_GN_GAIN_230);
	 // 设置增益大小：HMC5883L_GN_GAIN_230
	 i2c_send_data_single_s(MPU6050_RA_I2C_SLV0_CTRL, MPU6050_I2C_SLV_EN|0x01);
	 
	 
	 i2c_send_data_single_s(MPU6050_RA_I2C_SLV0_ADDR, MPU6050_I2C_SLV_RW_WRITE|HMC5883L_ADDRESS);
   // 磁罗盘从机地址
	 i2c_send_data_single_s(MPU6050_RA_I2C_SLV0_REG, HMC5883L_RA_MODE_REG);
   // 磁罗盘模式寄存器 
   i2c_send_data_single_s(MPU6050_RA_I2C_SLV0_DO, HMC5883L_MD_CONT_MEAS);
	 // 设置连续测量模式：HMC5883L_MD_CONT_MEAS
	 i2c_send_data_single_s(MPU6050_RA_I2C_SLV0_CTRL, MPU6050_I2C_SLV_EN|0x01);
}
	







/**
 *
 * 名称：i2c_mpu6050_init_mag_s
 *
 * 描述：将MPU6050配置成主机模式
 *
 */ 

void i2c_mpu6050_init_mag_s(void)
{
   i2c_send_data_single_s(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT);
	 // MPU6050配置成I2C主机模式
	 i2c_send_data_single_s(MPU6050_RA_I2C_MST_CTRL, MPU6050_CLOCK_DIV_258);
	 // MPU6050和从机通讯速率258KHz
	 i2c_send_data_single_s(MPU6050_RA_I2C_SLV0_ADDR, MPU6050_I2C_SLV_RW_READ|HMC5883L_ADDRESS);
   // 磁罗盘地址
	 i2c_send_data_single_s(MPU6050_RA_I2C_SLV0_REG, HMC5883L_RA_DATAOUT_X_MSB);
	 // 磁罗盘配置寄存器地址
	 i2c_send_data_single_s(MPU6050_RA_I2C_SLV0_CTRL, MPU6050_I2C_SLV_EN|0x06);
	 // 使能数据传输，数据长度：3*2字节
}

 




/**
 *
 * 名称：i2c_mpu6050_read_mag_s
 *
 * 描述：读取磁罗盘数据
 *
 */ 
void i2c_mpu6050_read_mag_s(float* mag)
{
	int16_t mag_temp[3];
	u8 buffer[6];
	// 使能数据传输之后，即可直接通过读取6050对应寄存器获得磁罗盘数据
	i2c_receive_data_s(MPU6050_RA_EXT_SENS_DATA_00, buffer, 6);
	// 地址MPU6050_RA_EXT_SENS_DATA_00: Reg73~96
	mag_temp[0] = (buffer[0]<<8) | buffer[1];   //magX-MB: 0x03 LB: 0x04
	mag_temp[1] = (buffer[2]<<8) | buffer[3];   //magZ-MB: 0x05 LB: 0x06
	mag_temp[2] = (buffer[4]<<8) | buffer[5];   //magY-MB: 0x07 LB: 0x08
	
	// 增益设置为230
  mag[0] = (float)mag_temp[0] * 4.35;
	mag[1] = (float)mag_temp[1] * 4.35;
	mag[2] = (float)mag_temp[2] * 4.35;
	
	//printf("MPU6050 Mag   (deg ): %.2f%s%.2f%s%.2f%s", mag[0], "   ", mag[1], "    ", mag[2], "      \n");
}




 


/**
 * 名称: i2c_send_data_single_s
 *
 * 描述：MPU6050  软件模拟Single-Byte Write Sequence单字节发送模式
 *
 */
 
uint8_t i2c_send_data_single_s(uint8_t reg, uint8_t byte)
{

	uint32_t i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	mpu6050_i2c_start_s();																//1 产生起始信号	
		
	mpu6050_i2c_send_byte_s(MPU6050_SLAVE_WRITE_ADDRESS); //2 发送7位从机设备地址，并检查是否收到地址应答
	
	while(mpu6050_i2c_wait_ack_s())
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback_s("MPU6050", 1);
		}
		i2c_wait_timeout--;
	}
	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	mpu6050_i2c_send_byte_s(reg);													//3 发送一个字节地址(MPU6050 Single-Byte Write Sequence)
	
	while(mpu6050_i2c_wait_ack_s())
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback_s("MPU6050", 2);
		}
		i2c_wait_timeout--;
	}

	mpu6050_i2c_send_byte_s(byte);												//4 发送一个字节数据
	
	while(mpu6050_i2c_wait_ack_s())
	{
	  if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback_s("MPU6050", 3);
		}
		i2c_wait_timeout--;
	}
		
	mpu6050_i2c_stop_s(); 																//5 停止信号
	
	return SUCCESS;
}





					
/**
 * 名称: i2c_send_data_s
 *
 * 描述：MPU6050  Burst Write Sequence多字节发送模式
 *
 */
 
uint8_t i2c_send_data_s(uint8_t reg, uint8_t* buffer, u8 num)
{
	u8 index;

	uint32_t i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	mpu6050_i2c_start_s();																 //1 产生起始信号	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	mpu6050_i2c_send_byte_s(MPU6050_SLAVE_WRITE_ADDRESS);  //2 发送7位从机设备地址，并检查是否收到地址应答
	
	while(mpu6050_i2c_wait_ack_s())
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback_s("MPU6050", 1);
		}
		i2c_wait_timeout--;
	}
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	mpu6050_i2c_send_byte_s(reg);													//3 发送一个字节地址(MPU6050 Single-Byte Write Sequence)
	
	while(mpu6050_i2c_wait_ack_s())
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback_s("MPU6050", 2);
		}
		i2c_wait_timeout--; 
	}

	for(index = 0; index < num; index++)
	{
		mpu6050_i2c_send_byte_s(*buffer);										//4 发送若干个字节数据
		
		while(mpu6050_i2c_wait_ack_s())
		{
		  if(i2c_wait_timeout == 0)
			{
				return i2c_timeout_usercallback_s("MPU6050", 3);
			}
			i2c_wait_timeout--;
		}
		buffer++;
	}
	
	mpu6050_i2c_stop_s(); 																//5 停止信号
	
	return SUCCESS;
}







/**
 * 名称: i2c_receive_data_s
 *
 * 描述：Burst Read Sequence 多字节接收模式
 *
 */

uint8_t i2c_receive_data_s(u8 reg, uint8_t* buffer, uint8_t num)
{
	uint32_t i2c_wait_timeout = I2C_WAIT_TIMEOUT;

	mpu6050_i2c_start_s();  															// 1 起始条件
	
	mpu6050_i2c_send_byte_s(MPU6050_SLAVE_WRITE_ADDRESS); // 2 发送从机设备地址
	while(mpu6050_i2c_wait_ack_s())
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback_s("MPU6050", 1);
		}
		i2c_wait_timeout--;
	}	
	
	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;	
	mpu6050_i2c_send_byte_s(reg);													//3 发送一个字节地址
	
	while(mpu6050_i2c_wait_ack_s())
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback_s("MPU6050", 2);
		}
		i2c_wait_timeout--;
	}
	
	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	mpu6050_i2c_start_s();  															// 4 Burst Read Sequence模式起始条件

	mpu6050_i2c_send_byte_s(MPU6050_SLAVE_READ_ADDRESS); 	// 5 发送从机设备地址
	while(mpu6050_i2c_wait_ack_s())
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback_s("MPU6050", 3); 
		}
		i2c_wait_timeout--;
	}
	
	while(num)																						// 6 循环读取
	{
		if (num == 1)																				// 7 读取最后一字节数据
		{
			*buffer = mpu6050_i2c_read_byte_s(0); 						// 7 产生Nack信号
			mpu6050_i2c_stop_s(); 
		}
    else 
    {      
      *buffer = mpu6050_i2c_read_byte_s(1); 						// 7 产生ack信号
      buffer++; 
      
    }
		num--;        
	}
	
	
	return SUCCESS;																				// 成功读出数据
}
	











