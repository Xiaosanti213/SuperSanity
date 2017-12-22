/**
 * @file mpu6050.c
 *
 * mpu6050 模块姿态检测
 * 
 */
 
#include "stm32f10x.h" 
 
#include "mpu6050.h"
#include "board_config.h"
#include "hmc5883l.h" 

#include <stdio.h>

 
 
static void i2c_mpu6050_delay(void); 
static uint8_t i2c_send_data_single(uint8_t reg, uint8_t byte);
//static uint8_t i2c_send_data(uint8_t reg, uint8_t* buffer, u8 num);
static uint8_t i2c_receive_data(u8 reg, uint8_t* byte_add, uint8_t num);
static uint8_t i2c_timeout_usercallback(uint8_t error_code);

#define I2C_WAIT_TIMEOUT		((u32)0x1000)






/**
 *  名称: i2c_mpu6050_init
 *
 *  描述：mpu6050设备初始化
 *
 */
void i2c_mpu6050_init(void)
{
	
	//gpio_clk_config();//调试用
	
  mpu6050_config();//配置GPIO和I2C外设
	
  i2c_mpu6050_delay();//上电后延时防止数据出错
	
	i2c_send_data_single(MPU6050_RA_PWR_MGMT_1, 0x00);	     																				//解除休眠状态
	i2c_send_data_single(MPU6050_RA_SMPLRT_DIV , MPU6050_SMPLRT_DEV_GY1k);	    								    //陀螺仪采样率1kHz
	i2c_send_data_single(MPU6050_RA_CONFIG , MPU6050_EXT_SYNC_ACCEL_YOUT_L);	  								    //同步信号MPU6050_EXT_SYNC_ACCEL_YOUT_L
	i2c_send_data_single(MPU6050_RA_ACCEL_CONFIG , MPU6050_ACCEL_FS_2G|MPU6050_DHPF_5);	  				  //配置加速度计量程+-2G, Digital High Pass Filter
	i2c_send_data_single(MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_2000);     												//陀螺仪设置不自检，量程+-2000deg/s
  
	i2c_mpu6050_check();//测试是否连接正常
}	




/**
 *  名称: i2c_mpu6050_check
 *
 *  描述：mpu6050检验设备连接
 *
 */

uint8_t i2c_mpu6050_check(void)
{
	uint8_t value = 0;
	i2c_receive_data(MPU6050_RA_WHO_AM_I, &value, 1);
	if(value == 0x68)
	{
		printf("MPU6050 is working...\n");
		return SUCCESS;
	}
	return ERROR;
}






/**
 *
 * 名称：i2c_mpu6050_read_acc
 *
 * 描述：读取加计数据 16348 LSB/g
 *
 */
void i2c_mpu6050_read_acc(float* acc)
{
	
	u16 acc_temp[3];//这中间变量是按位计算时候担心float存储方式不一致
	u8 buffer[6];//认为这个数据有无符号无所谓，需要表达的才有意义
	i2c_receive_data(MPU6050_RA_ACCEL_XOUT_H, buffer, 6);
	//读取加计数据首地址
	acc_temp[0] = (buffer[0]<<8) | buffer[1];
	acc_temp[1] = (buffer[2]<<8) | buffer[3];
	acc_temp[2] = (buffer[4]<<8) | buffer[5];
  printf("MPU6050  acc_temp: %d%s%d%s%d%s", acc_temp[0], " acc_tempy: ", acc_temp[1], " acc_tempz: ", acc_temp[2], "\n");
	 
	//转化成g单位
	acc[0] = (float)acc_temp[0] * 2 / 32768;
	acc[1] = (float)acc_temp[1] * 2 / 32768;
	acc[2] = (float)acc_temp[2] * 2 / 32768;
	printf("MPU6050  accx: %.2f%s%.2f%s%.2f%s", acc[0], " g accy: ", acc[1], " g accz: ", acc[2], " g\n");
	
}







/**
 *
 * 名称：i2c_mpu6050_read_gyro
 *
 * 描述：读取陀螺仪数据 16.4LSB
 *
 */
void i2c_mpu6050_read_gyro(float* gyro)
{
	u16 gyro_temp[3];
	uint8_t buffer[6];
	i2c_receive_data(MPU6050_RA_GYRO_XOUT_H, buffer, 6);
	//读取陀螺仪数据首地址
	gyro_temp[0] = (buffer[0]<<8) | buffer[1];
	gyro_temp[1] = (buffer[2]<<8) | buffer[3];
	gyro_temp[2] = (buffer[4]<<8) | buffer[5];
	 
	//转化成dps单位
	gyro[0] = (float)gyro_temp[0] * 2000/ 32768;
	gyro[1] = (float)gyro_temp[1] * 2000/ 32768;
	gyro[2] = (float)gyro_temp[2] * 2000/ 32768;
	
	printf("MPU6050 gyro: %.2f%s%.2f%s%.2f%s", gyro[0], " dps gyroy: ", gyro[1], " dps gyroz: ", gyro[2], "dps\n");
	
}









/**
 *
 * 名称：i2c_mpu6050_read_temp
 *
 * 描述：读取温度数据
 *
 */
void i2c_mpu6050_read_temp(float* temp)
{
	
	uint8_t buffer[2];
	uint16_t temp_quant;
	i2c_receive_data(MPU6050_RA_TEMP_OUT_H, buffer, 2);
	//读取温度数据首地址
	temp_quant = (buffer[0]<<8) | buffer[1];
	//转换成摄氏温度
	temp[0] = (double)temp_quant/340.0 + 36.53;
	//printf("MPU6050温度: %.2f%s", temp[0], "\n");

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







/**
 *
 * 名称：i2c_mpu6050_config_mag
 *
 * 描述：将MPU6050配置成bypass模式，配置磁罗盘参数
 *
 */

void i2c_mpu6050_config_mag(void)
{
	 i2c_send_data_single(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT);
	 // MPU6050配置成I2C主机模式
	 i2c_send_data_single(MPU6050_RA_I2C_MST_CTRL, MPU6050_CLOCK_DIV_258);
	 // MPU6050和从机通讯速率258KHz最慢的了
	 i2c_send_data_single(MPU6050_RA_I2C_SLV4_CTRL, 0x13);
	 // 配置采样磁罗盘50Hz
	 i2c_send_data_single(MPU6050_RA_I2C_MST_DELAY_CTRL, MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_BIT);
	 //外置传感器延迟使能
	 
	 i2c_send_data_single(MPU6050_RA_I2C_SLV0_ADDR, MPU6050_I2C_SLV_RW_WRITE|HMC5883L_ADDRESS);
   // 磁罗盘从机地址
	 i2c_send_data_single(MPU6050_RA_I2C_SLV0_REG, HMC5883L_RA_CONFIG_RA);
   // 磁罗盘配置寄存器CRA
   i2c_send_data_single(MPU6050_RA_I2C_SLV0_DO, (u8)(HMC5883L_MA_AVE_OUTPUT4|HMC5883L_DO_RATE_15));
	 // 磁罗盘配置参数：连续4个平均值输出，数据更新速率15Hz
	 i2c_send_data_single(MPU6050_RA_I2C_SLV0_CTRL, MPU6050_I2C_SLV_EN|0x01);
   // 先写入寄存器地址，再写入1个字节参数，EXT_SEN_R 0-1构成一个字MPU6050_RA_USER_CTRL 使能数据传输
	
	 
	 i2c_send_data_single(MPU6050_RA_I2C_SLV0_ADDR, MPU6050_I2C_SLV_RW_WRITE|HMC5883L_ADDRESS);
   // 磁罗盘从机地址
	 i2c_send_data_single(MPU6050_RA_I2C_SLV0_REG, HMC5883L_RA_CONFIG_RB);
   // 磁罗盘配置寄存器CRB 
   i2c_send_data_single(MPU6050_RA_I2C_SLV0_DO, HMC5883L_GN_GAIN_330);
	 // 设置增益大小：HMC5883L_GN_GAIN_230
	 i2c_send_data_single(MPU6050_RA_I2C_SLV0_CTRL, MPU6050_I2C_SLV_EN|0x01);
	 
	 
	 i2c_send_data_single(MPU6050_RA_I2C_SLV0_ADDR, MPU6050_I2C_SLV_RW_WRITE|HMC5883L_ADDRESS);
   // 磁罗盘从机地址
	 i2c_send_data_single(MPU6050_RA_I2C_SLV0_REG, HMC5883L_RA_MODE_REG);
   // 磁罗盘模式寄存器 
   i2c_send_data_single(MPU6050_RA_I2C_SLV0_DO, HMC5883L_MD_CONT_MEAS);
	 // 设置连续测量模式：HMC5883L_MD_CONT_MEAS
	 i2c_send_data_single(MPU6050_RA_I2C_SLV0_CTRL, MPU6050_I2C_SLV_EN|0x01);
}
	







/**
 *
 * 名称：i2c_mpu6050_init_mag
 *
 * 描述：将MPU6050配置成主机模式
 *
 */ 

void i2c_mpu6050_init_mag(void)
{
   i2c_send_data_single(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT);
	 // MPU6050配置成I2C主机模式
	 i2c_send_data_single(MPU6050_RA_I2C_MST_CTRL, MPU6050_CLOCK_DIV_258);
	 // MPU6050和从机通讯速率258KHz
	 i2c_send_data_single(MPU6050_RA_I2C_SLV0_ADDR, MPU6050_I2C_SLV_RW_READ|HMC5883L_ADDRESS);
   // 磁罗盘地址
	 i2c_send_data_single(MPU6050_RA_I2C_SLV0_REG, HMC5883L_RA_DATAOUT_X_MSB);
	 // 磁罗盘配置寄存器地址
	 i2c_send_data_single(MPU6050_RA_I2C_SLV0_CTRL, MPU6050_I2C_SLV_EN|0x06);
	 // 使能数据传输，数据长度：3*2字节
}

 




/**
 *
 * 名称：i2c_mpu6050_read_mag
 *
 * 描述：读取磁罗盘数据
 *
 */ 
void i2c_mpu6050_read_mag(float* mag)
{
	u16 mag_temp[3];
	uint8_t buffer[6];
	// 使能数据传输之后，即可直接通过读取6050对应寄存器获得磁罗盘数据
	i2c_receive_data(MPU6050_RA_EXT_SENS_DATA_00, buffer, 6);
	// 地址MPU6050_RA_EXT_SENS_DATA_00: Reg73~96
	mag_temp[0] = (buffer[0]<<8) | buffer[1];   //magX-MB: 0x03 LB: 0x04
	mag_temp[1] = (buffer[2]<<8) | buffer[3];   //magZ-MB: 0x05 LB: 0x06
	mag_temp[2] = (buffer[4]<<8) | buffer[5];   //magY-MB: 0x07 LB: 0x08

	// 增益设置为230
  mag[0] = (float)mag_temp[0] * 4.35;
	mag[1] = (float)mag_temp[1] * 4.35;
	mag[2] = (float)mag_temp[2] * 4.35;
	
}




 


/**
 * 名称: i2c_send_data
 *
 * 描述：MPU6050  Single-Byte Write Sequence单字节发送模式
 *
 */
 
uint8_t i2c_send_data_single(uint8_t reg, uint8_t byte)
{
	/* Send STRAT condition */
	uint32_t i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	//while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY)) // 检测是否正忙
  //{
  //  if((i2c_wait_timeout--) == 0) return i2c_timeout_usercallback(7);
  // }

	
	I2C_GenerateSTART(MPU6050_I2C, ENABLE);//1 产生起始信号	
	
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(0);
		}
		i2c_wait_timeout--;
	}
	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	I2C_Send7bitAddress(MPU6050_I2C, MPU6050_SLAVE_ADDRESS, I2C_Direction_Transmitter); //2 发送7位从机设备地址，并检查是否收到地址应答
	
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(1);
		}
		i2c_wait_timeout--;
	}
	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	I2C_SendData(MPU6050_I2C, reg);		//3 发送一个字节地址(MPU6050 Single-Byte Write Sequence)
	
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(2);
		}
		i2c_wait_timeout--;
	}

	I2C_SendData(MPU6050_I2C, byte);		//4 发送一个字节数据
	
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
	  if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(3);
		}
		i2c_wait_timeout--;
	}
		
	I2C_GenerateSTOP (MPU6050_I2C, ENABLE); //5 停止信号
	
	I2C_Cmd(MPU6050_I2C, DISABLE);
	I2C_Cmd(MPU6050_I2C, ENABLE);
	return SUCCESS;
}





					
/**
 * 名称: i2c_send_data
 *
 * 描述：MPU6050  Burst Write Sequence多字节发送模式
 *
 */
 
uint8_t i2c_send_data(uint8_t reg, uint8_t* buffer, u8 num)
{
	u8 index;
	/* Send STRAT condition */
	uint32_t i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	//while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY)) // 检测是否正忙
  //{
  //  if((i2c_wait_timeout--) == 0) return i2c_timeout_usercallback(0);
  // }

	
	I2C_GenerateSTART(MPU6050_I2C, ENABLE);//1 产生起始信号	
	
	while(I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT) == ERROR)
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(1);
		}
		i2c_wait_timeout--;
	}
	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	I2C_Send7bitAddress(MPU6050_I2C, MPU6050_SLAVE_ADDRESS, I2C_Direction_Transmitter); //2 发送7位从机设备地址，并检查是否收到地址应答
	
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(2);
		}
		i2c_wait_timeout--;
	}
	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	I2C_SendData(MPU6050_I2C, reg);		//3 发送一个字节地址(MPU6050 Single-Byte Write Sequence)
	
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(3);
		}
		i2c_wait_timeout--;
	}

	for(index = 0; index < num; index++)
	{
		I2C_SendData(MPU6050_I2C, *buffer);		//4 发送一个字节数据
		while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(4);
		}
		i2c_wait_timeout--;
		buffer++;//这块如果没有进入下一次循环是否会有危险
	}
	
	I2C_GenerateSTOP (MPU6050_I2C, ENABLE); //5 停止信号
	
	return SUCCESS;
}







/**
 * 名称: i2c_receive_data
 *
 * 描述：Burst Read Sequence 多字节接收模式
 *
 */

uint8_t i2c_receive_data(u8 reg, uint8_t* buffer, uint8_t num)
{
	uint32_t i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	
	while(I2C_GetFlagStatus(MPU6050_I2C, I2C_FLAG_BUSY)) // 检测是否正忙  
  {
    if((i2c_wait_timeout--) == 0) return i2c_timeout_usercallback(0);
  }
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	I2C_GenerateSTART(MPU6050_I2C, ENABLE);  // 1 起始条件
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(1);
		}
		i2c_wait_timeout--;
	}
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT; 
	I2C_Send7bitAddress(MPU6050_I2C, MPU6050_SLAVE_ADDRESS, I2C_Direction_Transmitter); // 2 发送从机设备地址
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(2);
		}
		i2c_wait_timeout--;
	}
  /* Clear EV6 by setting again the PE bit */
  I2C_Cmd(MPU6050_I2C, ENABLE);
	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;	
	I2C_SendData(MPU6050_I2C, reg);		//3 发送一个字节地址
	
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(3);
		}
		i2c_wait_timeout--;
	}
	
	
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT;
	I2C_GenerateSTART(MPU6050_I2C, ENABLE);  // 4 Burst Read Sequence模式起始条件
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(4);
		}
		i2c_wait_timeout--;
	}
	
	i2c_wait_timeout = I2C_WAIT_TIMEOUT; 
	I2C_Send7bitAddress(MPU6050_I2C, MPU6050_SLAVE_ADDRESS, I2C_Direction_Receiver); // 5 发送从机设备地址
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{
		if(i2c_wait_timeout == 0)
		{
			return i2c_timeout_usercallback(5);
		}
		i2c_wait_timeout--;
	}
	while(num)// 循环读取
	{
		if (num == 1)//读取最后一字节数据
		{
			I2C_AcknowledgeConfig(MPU6050_I2C, DISABLE);//最后一次收到后发送Nack
			I2C_GenerateSTOP(MPU6050_I2C, ENABLE); 
		}
    if(I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))  
    {      
      /* Read a byte from the slave */
      *buffer = I2C_ReceiveData(MPU6050_I2C);
			//读取的内容，可使用引用变量或指针，但是保证读出数据

      /* Point to the next location where the byte read will be saved */
      buffer++; 
      
      /* Decrement the read bytes counter */
      num--;        
    }
	}
	I2C_AcknowledgeConfig(MPU6050_I2C, ENABLE);
	
	I2C_Cmd(MPU6050_I2C, DISABLE);
	I2C_Cmd(MPU6050_I2C, ENABLE);
	
	return SUCCESS;//成功读出数据
}
	



/**
 * 名称: i2c_timeout_usercallback
 *
 * 描述：等待超时回调函数
 *
 */
uint8_t i2c_timeout_usercallback(uint8_t error_code)
{
	printf("MPU6050 等待超时！errorCode =%d\n", error_code);
	I2C_Cmd(MPU6050_I2C, DISABLE);
	I2C_Cmd(MPU6050_I2C, ENABLE);
	return 0;
}










