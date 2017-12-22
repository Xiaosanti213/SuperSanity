/**
 * @file hmc5883l.h
 *
 * hmc5883l驱动相关宏
 *
 */
 
#ifndef _HMC5883L_H
#define _HMC5883L_H
 
 
#include "stm32f10x.h" 

#define			HMC5883L_ADDRESS							 0x1E
#define     HMC5883L_SLAVE_ADDRESS				(0x1E<<1)   //设备地址确定



// 寄存器地址映射
#define			HMC5883L_RA_CONFIG_RA					0x00  //Read/Write
#define			HMC5883L_RA_CONFIG_RB					0x01
#define			HMC5883L_RA_MODE_REG					0x02
#define			HMC5883L_RA_DATAOUT_X_MSB			0x03  //下面都是只读寄存器
#define			HMC5883L_RA_DATAOUT_X_LSB			0x04
#define			HMC5883L_RA_DATAOUT_Z_MSB			0x05
#define			HMC5883L_RA_DATAOUT_Z_LSB			0x06
#define			HMC5883L_RA_DATAOUT_Y_MSB			0x07
#define			HMC5883L_RA_DATAOUT_Y_LSB			0x08
#define			HMC5883L_RA_STATUS_REG				0x09
#define			HMC5883L_RA_ID_RA							0x0A
#define			HMC5883L_RA_ID_RB							0x0B
#define			HMC5883L_RA_ID_RC							0x0C






// 寄存器配置
// Configuration Register A 配置 复位值0x10
#define			HMC5883L_MA_AVE_OUTPUT1				0x00<<5  //输出数据连续1个取均值(默认)
#define			HMC5883L_MA_AVE_OUTPUT2				0x01<<5	 //连续2个
#define			HMC5883L_MA_AVE_OUTPUT4				0x10<<5	 //连续4个
#define			HMC5883L_MA_AVE_OUTPUT8				0x11<<5	 //连续8个

#define			HMC5883L_DO_RATE_0P75				0x01<<2		//数据输出到3个寄存器速率0.75Hz
#define			HMC5883L_DO_RATE_1P5				0x01<<2		//数据输出到3个寄存器速率1.5Hz
#define			HMC5883L_DO_RATE_3					0x01<<2		//数据输出到3个寄存器速率3Hz
#define			HMC5883L_DO_RATE_7P5				0x01<<2		//数据输出到3个寄存器速率7.5Hz
#define			HMC5883L_DO_RATE_15					0x01<<2		//数据输出到3个寄存器速率15Hz(默认)
#define			HMC5883L_DO_RATE_30					0x01<<2		//数据输出到3个寄存器速率30Hz
#define			HMC5883L_DO_RATE_75					0x01<<2		//数据输出到3个寄存器速率75Hz


#define     HMC5883L_MS_NORMAL					0x00<<0		//常规测量配置(默认)
#define			HMC5883L_MS_POS_BIAS				0x01<<0		//三轴阻抗附加正向电流
#define			HMC5883L_MS_NEG_BIAS				0x02<<0		//三轴阻抗附加负向电流



// Configuration Register B 配置设备增益 复位值0x20
#define			HMC5883L_GN_GAIN_1370			  0x00<<5		//配置增益1370 LSb/Gauss
#define			HMC5883L_GN_GAIN_1090				0x01<<5		//1090 LSb/Gauss
#define			HMC5883L_GN_GAIN_820				0x02<<5		//820 LSb/Gauss
#define			HMC5883L_GN_GAIN_660				0x03<<5		//660 LSb/Gauss
#define			HMC5883L_GN_GAIN_440				0x04<<5		//440 LSb/Gauss
#define			HMC5883L_GN_GAIN_390				0x05<<5		//390 LSb/Gauss
#define			HMC5883L_GN_GAIN_330				0x06<<5		//330 LSb/Gauss
#define			HMC5883L_GN_GAIN_230				0x07<<5		//230 LSb/Gauss


// Mode Register 配置设备运行模式 复位值0x01
#define			HMC5883L_HS_I2C_3400k				0x01<<7		//使能高速I2C 3400kHz
#define			HMC5883L_MD_CONT_MEAS				0x00<<0		//连续测量模式
#define			HMC5883L_MD_SING_MEAS				0x01<<0		//单次采样测量模式
#define			HMC5883L_MD_IDLE_MODE				0x02<<0		//空闲状态




void i2c_mpu6050_config_mag(void);
void i2c_mpu6050_init_mag(void);
void i2c_mpu6050_read_mag(float* mag);

// 软件模拟
void i2c_mpu6050_config_mag_s(void);
void i2c_mpu6050_init_mag_s(void);
void i2c_mpu6050_read_mag_s(float* mag);




#endif
