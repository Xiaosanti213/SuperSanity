#ifndef __MPU6050_H
#define __MPU6050_H
#include "stm32f10x.h"


// Register Address Map 寄存器地址映射

//#define MPU6050_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
//#define MPU6050_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
//#define MPU6050_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
//#define MPU6050_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
//#define MPU6050_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
//#define MPU6050_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
//#define MPU6050_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
//#define MPU6050_RA_XA_OFFS_L_TC     0x07
//#define MPU6050_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
//#define MPU6050_RA_YA_OFFS_L_TC     0x09
//#define MPU6050_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
//#define MPU6050_RA_ZA_OFFS_L_TC     0x0B 

#define MPU6050_RA_SELF_TEST_X			0x0D
#define MPU6050_RA_SELF_TEST_Y			0x0E
#define MPU6050_RA_SELF_TEST_Z			0x0F
#define MPU6050_RA_SELF_TESTA				0x10
#define MPU6050_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL     0x14
#define MPU6050_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL     0x16

//#define MPU6050_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
//#define MPU6050_RA_ZG_OFFS_USRL     0x18

#define MPU6050_RA_SMPLRT_DIV       0x19	
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_RA_FF_THR           0x1D
#define MPU6050_RA_FF_DUR           0x1E
#define MPU6050_RA_MOT_THR          0x1F
#define MPU6050_RA_MOT_DUR          0x20
#define MPU6050_RA_ZRMOT_THR        0x21
#define MPU6050_RA_ZRMOT_DUR        0x22
#define MPU6050_RA_FIFO_EN          0x23
#define MPU6050_RA_I2C_MST_CTRL     0x24
#define MPU6050_RA_I2C_SLV0_ADDR    0x25
#define MPU6050_RA_I2C_SLV0_REG     0x26
#define MPU6050_RA_I2C_SLV0_CTRL    0x27
#define MPU6050_RA_I2C_SLV1_ADDR    0x28
#define MPU6050_RA_I2C_SLV1_REG     0x29
#define MPU6050_RA_I2C_SLV1_CTRL    0x2A
#define MPU6050_RA_I2C_SLV2_ADDR    0x2B
#define MPU6050_RA_I2C_SLV2_REG     0x2C
#define MPU6050_RA_I2C_SLV2_CTRL    0x2D
#define MPU6050_RA_I2C_SLV3_ADDR    0x2E
#define MPU6050_RA_I2C_SLV3_REG     0x2F
#define MPU6050_RA_I2C_SLV3_CTRL    0x30
#define MPU6050_RA_I2C_SLV4_ADDR    0x31
#define MPU6050_RA_I2C_SLV4_REG     0x32
#define MPU6050_RA_I2C_SLV4_DO      0x33
#define MPU6050_RA_I2C_SLV4_CTRL    0x34
#define MPU6050_RA_I2C_SLV4_DI      0x35
#define MPU6050_RA_I2C_MST_STATUS   0x36
#define MPU6050_RA_INT_PIN_CFG      0x37    //中断引脚配置
#define MPU6050_RA_INT_ENABLE       0x38
#define MPU6050_RA_DMP_INT_STATUS   0x39
#define MPU6050_RA_INT_STATUS       0x3A
#define MPU6050_RA_ACCEL_XOUT_H     0x3B		//MPU6050加速度数据寄存器首地址
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H       0x41    //MPU6050温度数据寄存器首地址
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43		//MPU6050陀螺仪数据寄存器首地址
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48
#define MPU6050_RA_EXT_SENS_DATA_00 0x49
#define MPU6050_RA_EXT_SENS_DATA_01 0x4A
#define MPU6050_RA_EXT_SENS_DATA_02 0x4B
#define MPU6050_RA_EXT_SENS_DATA_03 0x4C
#define MPU6050_RA_EXT_SENS_DATA_04 0x4D
#define MPU6050_RA_EXT_SENS_DATA_05 0x4E
#define MPU6050_RA_EXT_SENS_DATA_06 0x4F
#define MPU6050_RA_EXT_SENS_DATA_07 0x50
#define MPU6050_RA_EXT_SENS_DATA_08 0x51
#define MPU6050_RA_EXT_SENS_DATA_09 0x52
#define MPU6050_RA_EXT_SENS_DATA_10 0x53
#define MPU6050_RA_EXT_SENS_DATA_11 0x54
#define MPU6050_RA_EXT_SENS_DATA_12 0x55
#define MPU6050_RA_EXT_SENS_DATA_13 0x56
#define MPU6050_RA_EXT_SENS_DATA_14 0x57
#define MPU6050_RA_EXT_SENS_DATA_15 0x58
#define MPU6050_RA_EXT_SENS_DATA_16 0x59
#define MPU6050_RA_EXT_SENS_DATA_17 0x5A
#define MPU6050_RA_EXT_SENS_DATA_18 0x5B
#define MPU6050_RA_EXT_SENS_DATA_19 0x5C
#define MPU6050_RA_EXT_SENS_DATA_20 0x5D
#define MPU6050_RA_EXT_SENS_DATA_21 0x5E
#define MPU6050_RA_EXT_SENS_DATA_22 0x5F
#define MPU6050_RA_EXT_SENS_DATA_23 0x60
#define MPU6050_RA_MOT_DETECT_STATUS    0x61
#define MPU6050_RA_I2C_SLV0_DO      0x63
#define MPU6050_RA_I2C_SLV1_DO      0x64
#define MPU6050_RA_I2C_SLV2_DO      0x65
#define MPU6050_RA_I2C_SLV3_DO      0x66
#define MPU6050_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU6050_RA_SIGNAL_PATH_RESET    0x68
#define MPU6050_RA_MOT_DETECT_CTRL      0x69
#define MPU6050_RA_USER_CTRL        0x6A
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_PWR_MGMT_2       0x6C
#define MPU6050_RA_BANK_SEL         0x6D
#define MPU6050_RA_MEM_START_ADDR   0x6E
#define MPU6050_RA_MEM_R_W          0x6F
#define MPU6050_RA_DMP_CFG_1        0x70
#define MPU6050_RA_DMP_CFG_2        0x71
#define MPU6050_RA_FIFO_COUNTH      0x72
#define MPU6050_RA_FIFO_COUNTL      0x73
#define MPU6050_RA_FIFO_R_W         0x74
#define MPU6050_RA_WHO_AM_I         0x75




// MPU6050配置参数

// Reg25 (0x19): SMPRT_DIV  bit7~bit0: SMPRT_DIV[7:0] 配置陀螺仪输出速率分频参数
#define MPU6050_SMPLRT_DIV_GY8k      0x00  //陀螺仪输出8000Hz
#define MPU6050_SMPLRT_DIV_GY4k      0x01  //陀螺仪输出4000Hz
#define MPU6050_SMPLRT_DIV_GY2k			 0x03  //陀螺仪输出2000Hz
#define MPU6050_SMPLRT_DEV_GY1k			 0x07  //陀螺仪输出1000Hz



// Reg26 (0x1A): CONFIG  bit5~bit3: EXT_SYNC_SET[2:0] 同步信号 bit2~bit0: bitDLPF_CFG[2:0] 低通滤波
#define MPU6050_EXT_SYNC_DISABLED       0x00<<3 //FSYNC信号 位存储位置
#define MPU6050_EXT_SYNC_TEMP_OUT_L     0x01<<3
#define MPU6050_EXT_SYNC_GYRO_XOUT_L    0x02<<3
#define MPU6050_EXT_SYNC_GYRO_YOUT_L    0x03<<3
#define MPU6050_EXT_SYNC_GYRO_ZOUT_L    0x04<<3
#define MPU6050_EXT_SYNC_ACCEL_XOUT_L   0x05<<3
#define MPU6050_EXT_SYNC_ACCEL_YOUT_L   0x06<<3
#define MPU6050_EXT_SYNC_ACCEL_ZOUT_L   0x07<<3

#define MPU6050_DLPF_BW_256         0x00<<0  //陀螺仪滤波带宽，加计稍有偏差
#define MPU6050_DLPF_BW_188         0x01<<0
#define MPU6050_DLPF_BW_98          0x02<<0
#define MPU6050_DLPF_BW_42          0x03<<0
#define MPU6050_DLPF_BW_20          0x04<<0
#define MPU6050_DLPF_BW_10          0x05<<0
#define MPU6050_DLPF_BW_5           0x06<<0

// Reg27 (0x1B): GYRO_CONFIG  bit7~bit5  XG_ST YG_ST ZG_ST 陀螺仪自检触发 
// bit4~bit3 FS_SEL[1:0] 陀螺仪量程
#define MPU6050_XG_ST 							0x01<<7
#define MPU6050_YG_ST								0x01<<6
#define MPU6050_ZG_ST								0x01<<5

#define MPU6050_GYRO_FS_250         0x00<<3   //±250°/s
#define MPU6050_GYRO_FS_500         0x01<<3
#define MPU6050_GYRO_FS_1000        0x02<<3
#define MPU6050_GYRO_FS_2000        0x03<<3


// Reg28 (0x1C): ACCEL_CONFIG  bit7~bit5  XA_ST YA_ST ZA_ST 加计自检触发  
// bit4~bit3 AFS_SEL[1:0] 加计量程	bit2~bit0 Digital High Pass Filter
#define MPU6050_XA_ST								0x01<<7
#define MPU6050_YA_ST								0x01<<6
#define MPU6050_ZA_ST								0x01<<5

#define MPU6050_ACCEL_FS_2G         0x00<<3		//±2g
#define MPU6050_ACCEL_FS_4G         0x01<<3
#define MPU6050_ACCEL_FS_8G         0x02<<3
#define MPU6050_ACCEL_FS_16G        0x03<<3
  
#define MPU6050_DHPF_RESET          0x00<<0
#define MPU6050_DHPF_5              0x01<<0
#define MPU6050_DHPF_2P5            0x02<<0
#define MPU6050_DHPF_1P25           0x03<<0
#define MPU6050_DHPF_0P63           0x04<<0
#define MPU6050_DHPF_HOLD           0x07<<0


// Reg35 (0x23): FIFO_EN  传感器测量数据载入缓存使能
#define MPU6050_TEMP_FIFO_EN        0x01<<7  //Reg65: TEMP_OUT_H   Reg66: TEMP_OUT_L
#define MPU6050_XG_FIFO_EN          0x01<<6  //Reg67: GYRO_XOUT_H  Reg68: GYRO_XOUT_L
#define MPU6050_YG_FIFO_EN          0x01<<5  //Reg69: GYRO_YOUT_H  Reg70: GYRO_YOUT_L 
#define MPU6050_ZG_FIFO_EN          0x01<<4  //Reg71: GYRO_ZOUT_H  Reg72: GYRO_ZOUT_L
#define MPU6050_ACCEL_FIFO_EN       0x01<<3  //Reg59 ~ Reg64: ACCEL_XOUT_H ACCEL_XOUT_L ACCEL_YOUT_H ACCEL_XOUT_L ACCEL_ZOUT_H ACCEL_XOUT_L
#define MPU6050_SLV2_FIFO_EN        0x01<<2  //Reg73 ~ Reg96: EXT_SENS_DATA associated with Slave2
#define MPU6050_SLV1_FIFO_EN        0x01<<1  //Reg73 ~ Reg96: EXT_SENS_DATA associated with Slave1
#define MPU6050_SLV0_FIFO_EN        0x01<<0  //Reg73 ~ Reg96: EXT_SENS_DATA associated with Slave0


// Reg36 (0x24): I2C_MST_CTRL  MPU6050作为主机的I2C参数配置
#define MPU6050_MULT_MST_EN         0x01<<7 //使能I2C多主机模式
#define MPU6050_WAIT_FOR_ES         0x01<<6 //延时Data Ready Interrupt中断直到从机设备传感器将数据载入EXT_SENS_DATA寄存器
#define MPU6050_SLV_3_FIFO_EN       0x01<<5 //将SLAVE3寄存器中数据写入缓存
#define MPU6050_I2C_MST_P_NSR       0x01<<4 //控制从一个从机到另一个从机

#define MPU6050_CLOCK_DIV_348       0x00<<0  //配置主机时钟速率分频：8MHz / 23 = 348KHz
#define MPU6050_CLOCK_DIV_333       0x01<<0  
#define MPU6050_CLOCK_DIV_320       0x02<<0
#define MPU6050_CLOCK_DIV_308       0x03<<0
#define MPU6050_CLOCK_DIV_296       0x04<<0
#define MPU6050_CLOCK_DIV_286       0x05<<0
#define MPU6050_CLOCK_DIV_276       0x06<<0
#define MPU6050_CLOCK_DIV_267       0x07<<0
#define MPU6050_CLOCK_DIV_258       0x08<<0
#define MPU6050_CLOCK_DIV_500       0x09<<0
#define MPU6050_CLOCK_DIV_471       0x0A<<0
#define MPU6050_CLOCK_DIV_444       0x0B<<0
#define MPU6050_CLOCK_DIV_421       0x0C<<0
#define MPU6050_CLOCK_DIV_400       0x0D<<0
#define MPU6050_CLOCK_DIV_381       0x0E<<0
#define MPU6050_CLOCK_DIV_364       0x0F<<0


// Reg37~Reg39: Slave0  Reg40~Reg42: Slave1  Reg43~45: Slave2  Reg46~Reg48: Slave3
// Reg37(Reg40, 43, 46) bit7: I2C从机传感器设备读写操作
#define MPU6050_I2C_SLV_RW_READ      0x01<<7    //读Slave(1, 2, 3)设备操作
#define MPU6050_I2C_SLV_RW_WRITE		 0x00<<7    //写Slave(1, 2, 3)设备操作

// Reg37(Reg40, 43, 46) bit6~bit0: I2C从机传感器设备地址
//#define MPU6050_I2C_SLV1_ADDR    
//#define MPU6050_I2C_SLV2_ADDR
//#define MPU6050_I2C_SLV3_ADDR 

// Reg39(Reg42, 45, 48) bit7~bit0:
#define MPU6050_I2C_SLV_EN           0x01<<7		//使能数据传输
#define MPU6050_I2C_SLV_BYTE_SW      0x01<<6    //交换高低字节位
#define MPU6050_I2C_SLV_REG_DIS      0x01<<5    //1: Read-Write Only 0:写寄存器地址优先于读写数据
#define MPU6050_I2C_SLV_GRP          0x01<<4		//地址组合指令


// Reg49~Reg53: Slave4
// Reg49 bit7：I2C从机传感器设备读写操作
#define MPU6050_I2C_SLV_RW_READ      0x01<<7    //读Slave4设备操作
#define MPU6050_I2C_SLV_RW_WRITE		 0x00<<7    //写Slave4设备操作

// Reg49: bit6~bit0: I2C从机传感器设备地址
//#define MPU6050_I2C_SLV4_ADDR 

// Reg52: bit7~bit5
#define MPU6050_I2C_SLV4_EN							0x01<<7  //使能数据传输
#define MPU6050_I2C_SLV4_INT_EN		      0x01<<6  //数据传输完毕触发中断
#define MPU6050_I2C_SLV4_REG_DIS		    0x01<<5	 //1: Read-Write Only 0:写寄存器地址优先于读写数据

// Reg52: bit4~bit0 1/(1+I2C_MST_DLY)samples
#define MPU6050_I2C_SLV4_MST_DLY		    0x01<<0  //通信速率分频

// Reg54: I2C_MST_STATUS (Read Only)
#define MPU6050_MST_PASS_THROUGH		    0x01<<7	 //FSYNC引脚输入MPU6050
#define MPU6050_MST_I2C_SLV4_DONE		    0x01<<6  
#define MPU6050_MST_I2C_LOST_ARB		    0x01<<5
#define MPU6050_MST_I2C_SLV4_NACK		    0x01<<4
#define MPU6050_MST_I2C_SLV3_NACK		    0x01<<3
#define MPU6050_MST_I2C_SLV2_NACK		    0x01<<2
#define MPU6050_MST_I2C_SLV1_NACK		    0x01<<1
#define MPU6050_MST_I2C_SLV0_NACK		    0x01<<0

// Reg55: INT_PIN_CFG 中断引脚配置
#define MPU6050_INTMODE_ACTIVEHIGH  0x00<<7  //INT中断引脚高电平有效
#define MPU6050_INTMODE_ACTIVELOW   0x01<<7

#define MPU6050_INTDRV_PUSHPULL     0x00<<6  //推挽输出
#define MPU6050_INTDRV_OPENDRAIN    0x01<<6  //开漏输出

#define MPU6050_INTLATCH_50USPULSE  0x00<<5  //延时50us
#define MPU6050_INTLATCH_WAITCLEAR  0x01<<5  //置高等待清空

#define MPU6050_INTCLEAR_STATUSREAD 0x00<<4  //通过读取INT_STATUS位清空
#define MPU6050_INTCLEAR_ANYREAD    0x01<<4  //通过任何读取操作

#define MPU6050_FSYNCLEVEL_HIGH     0x00<<3  //FSYNC中断引脚高电平有效
#define MPU6050_FSYNCLEVEL_LOW      0x01<<3

#define MPU6050_FSYNCINT_DIS				0x00<<2	 //FSYNC中断引脚失能
#define MPU6050_FSYNCINT_EN 				0x01<<2  //FSYNC中断引脚使能

#define MPU6050_INTCFG_I2C_BYPASS_EN    0x00<<1  //BYPASS失能
#define MPU6050_INTCFG_I2C_BYPASS_DIS   0x01<<1  //该位置1且I2C_MST_EN置0可以实现上位机直接访问从机设备

// Reg56: INT_ENABLE 失能中断源产生中断
//#define MPU6050_INTERRUPT_FF_EN           0x01<<7
#define MPU6050_INTERRUPT_MOT_EN            0x01<<6
//#define MPU6050_INTERRUPT_ZMOT_BIT          0x01<<5
#define MPU6050_INTERRUPT_FIFO_OFLOW _EN 		0x01<<4
#define MPU6050_INTERRUPT_I2C_MST_INT_EN   	0x01<<3
//#define MPU6050_INTERRUPT_PLL_RDY_INT			  0x01<<2
//#define MPU6050_INTERRUPT_DMP_INT	          0x01<<1
#define MPU6050_INTERRUPT_DATA_RDY          0x01<<0

#define MPU6050_DMPINT_5_BIT            5
#define MPU6050_DMPINT_4_BIT            4
#define MPU6050_DMPINT_3_BIT            3
#define MPU6050_DMPINT_2_BIT            2
#define MPU6050_DMPINT_1_BIT            1
#define MPU6050_DMPINT_0_BIT            0

#define MPU6050_MOTION_MOT_XNEG_BIT     7
#define MPU6050_MOTION_MOT_XPOS_BIT     6
#define MPU6050_MOTION_MOT_YNEG_BIT     5
#define MPU6050_MOTION_MOT_YPOS_BIT     4
#define MPU6050_MOTION_MOT_ZNEG_BIT     3
#define MPU6050_MOTION_MOT_ZPOS_BIT     2
#define MPU6050_MOTION_MOT_ZRMOT_BIT    0

// Reg103(0x67): MPU6050_RA_I2C_MST_DELAY_CTRL
#define MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT   0x01<<7
#define MPU6050_DELAYCTRL_I2C_SLV4_DLY_EN_BIT   0x01<<4
#define MPU6050_DELAYCTRL_I2C_SLV3_DLY_EN_BIT   0x01<<3
#define MPU6050_DELAYCTRL_I2C_SLV2_DLY_EN_BIT   0x01<<2
#define MPU6050_DELAYCTRL_I2C_SLV1_DLY_EN_BIT   0x01<<1
#define MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_BIT   0x01<<0

#define MPU6050_PATHRESET_GYRO_RESET_BIT    2
#define MPU6050_PATHRESET_ACCEL_RESET_BIT   1
#define MPU6050_PATHRESET_TEMP_RESET_BIT    0

#define MPU6050_DETECT_ACCEL_ON_DELAY_BIT       5
#define MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH    2
#define MPU6050_DETECT_FF_COUNT_BIT             3
#define MPU6050_DETECT_FF_COUNT_LENGTH          2
#define MPU6050_DETECT_MOT_COUNT_BIT            1
#define MPU6050_DETECT_MOT_COUNT_LENGTH         2

#define MPU6050_DETECT_DECREMENT_RESET  0x0
#define MPU6050_DETECT_DECREMENT_1      0x1
#define MPU6050_DETECT_DECREMENT_2      0x2
#define MPU6050_DETECT_DECREMENT_4      0x3

// Reg106(0x6A): MPU6050_RA_USER_CTRL 
#define MPU6050_USERCTRL_DMP_EN_BIT             0x01<<7
#define MPU6050_USERCTRL_FIFO_EN_BIT            0x01<<6
#define MPU6050_USERCTRL_I2C_MST_EN_BIT         0x01<<5
#define MPU6050_USERCTRL_I2C_IF_DIS_BIT         0x01<<4
#define MPU6050_USERCTRL_DMP_RESET_BIT          0x01<<3
#define MPU6050_USERCTRL_FIFO_RESET_BIT         0x01<<2
#define MPU6050_USERCTRL_I2C_MST_RESET_BIT      0x01<<1
#define MPU6050_USERCTRL_SIG_COND_RESET_BIT     0x01<<0

// Reg107: PWR_MGMT_1 bit7: 复位设备 bit6~bit5, bit2~bit0: 配置电源模式和时钟源 bit3: 失能温传 
#define MPU6050_PWR1_DEVICE_RESET			  0x01<<7	
#define MPU6050_PWR1_SLEEP		          0x01<<6		//进入低电量睡眠模式
#define MPU6050_PWR1_CYCLE		          0x01<<5		//失能睡眠模式且该位置1，设备进入循环模式，睡眠和醒来模式切换，采样加计数据
#define MPU6050_PWR1_TEMP_DIS		        0x01<<3

#define MPU6050_CLOCK_INTERNAL          0x00<<0   //内部8M晶振
#define MPU6050_CLOCK_PLL_XGYRO         0x01<<0		//PLL带有X轴陀螺仪参考
#define MPU6050_CLOCK_PLL_YGYRO         0x02<<0   //PLL带有Y轴陀螺仪参考
#define MPU6050_CLOCK_PLL_ZGYRO         0x03<<0   //PLL带有Z轴陀螺仪参考
#define MPU6050_CLOCK_PLL_EXT32K        0x04<<0   //PLL带有外部32.768kHz参考
#define MPU6050_CLOCK_PLL_EXT19M        0x05<<0   //PLL带有外部19.2MHz参考
#define MPU6050_CLOCK_KEEP_RESET        0x07<<0   //停止时钟，时钟发生器保持复位状态

//Reg108: PWR_MGMT_2  bit7~bit6: LP_WAKE_CTRL[2:0] bit5~bit0: STBY(ACC, GYRO)
#define MPU6050_WAKE_FREQ_1P25          0x00<<6   //低电量模式ACC-ONLY采样频率1.25Hz
#define MPU6050_WAKE_FREQ_2P5           0x01<<6   //低电量模式ACC-ONLY采样频率2.5Hz
#define MPU6050_WAKE_FREQ_5             0x02<<6   //低电量模式ACC-ONLY采样频率5Hz
#define MPU6050_WAKE_FREQ_10            0x03<<6   //低电量模式ACC-ONLY采样频率10Hz

#define MPU6050_PWR2_STBY_XA	          0x01<<5   //X轴加计数据进入standby模式
#define MPU6050_PWR2_STBY_YA	          0x01<<4   //Y轴加计数据进入standby模式
#define MPU6050_PWR2_STBY_ZA	          0x01<<3   //Z轴加计数据进入standby模式
#define MPU6050_PWR2_STBY_XG	          0x01<<2   //X轴陀螺仪数据进入standby模式
#define MPU6050_PWR2_STBY_YG	          0x01<<1   //Y轴陀螺仪数据进入standby模式
#define MPU6050_PWR2_STBY_ZG	          0x01<<0   //Z轴陀螺仪数据进入standby模式


#define MPU6050_BANKSEL_PRFTCH_EN_BIT       6
#define MPU6050_BANKSEL_CFG_USER_BANK_BIT   5
#define MPU6050_BANKSEL_MEM_SEL_BIT         4
#define MPU6050_BANKSEL_MEM_SEL_LENGTH      5

//Reg117: WHO_AM_I bit6~bit0: MPU6050的前6位地址，末尾地址该寄存器中没有体现 
#define MPU6050_WHO_AM_I		        0x68   //该寄存器默认值

#define MPU6050_ADDRESS_AD0_LOW     0x68 // AD0接地，7位从机设备地址0x68
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // AD0接电源，7位从机设备地址0x69
#define MPU6050_SLAVE_ADDRESS       (0x68<<1)    //AD0接地，MPU6050器件读地址0x68

#define MPU6050_SLAVE_READ_ADDRESS  (0x68<<1 |0x01) //软件模拟使用
#define MPU6050_SLAVE_WRITE_ADDRESS (0x68<<1 |0x00) //移位运算符优先于|

#define MPU6050_DMP_MEMORY_BANKS        8
#define MPU6050_DMP_MEMORY_BANK_SIZE    256
#define MPU6050_DMP_MEMORY_CHUNK_SIZE   16

#define MPU6050_DLPF_CFG        0

#define MPU6050_TC_PWR_MODE_BIT     7
#define MPU6050_TC_OFFSET_BIT       6
#define MPU6050_TC_OFFSET_LENGTH    6
#define MPU6050_TC_OTP_BNK_VLD_BIT  0

#define MPU6050_VDDIO_LEVEL_VLOGIC  0
#define MPU6050_VDDIO_LEVEL_VDD     1




void i2c_mpu6050_init(void);
uint8_t i2c_mpu6050_check(void);

void i2c_mpu6050_read_acc(float* acc);
void i2c_mpu6050_read_gyro(float* gyro);
void i2c_mpu6050_read_temp(float* temp);


// 软件模拟模式下
void i2c_mpu6050_init_s(void);
uint8_t i2c_mpu6050_check_s(void);

void i2c_mpu6050_read_acc_s(float* acc);
void i2c_mpu6050_read_gyro_s(float* gyro);
void i2c_mpu6050_read_temp_s(float* temp);



	








#endif  /*__MPU6050*/
