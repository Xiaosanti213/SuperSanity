/**
 * @file ms5611.h
 *
 * ms5611驱动相关宏
 *
 */
 
 
#ifndef _MS5611_H
#define _MS5611_H


#include "stm32f10x.h"

/*
 * The MS5611-01BA has only five commands
 * 1 Reset
 * 2 Read PROM (128 bit if calibration words)
 * 3 D1 conversion
 * 4 D2 conversion
 * 5 Read ADC result
 */

#define				MS5611_LOW_ADDRESS				0x76  //CSB接VCC
#define				MS5611_HIGH_ADDRESS				0x77  //CSB接地

#define       MS5611_SLAVE_ADDRESS			(0x77<<1)

#define				MS5611_SLAVE_READ_ADDRESS (0x77<<1 | 0x01)
#define				MS5611_SLAVE_WRITE_ADDRESS (0x77<<1 | 0x00)



#define				MS5611_RESET							0x1E


// OSR (Over Sampling Ratio) constants
// D1转换：Digital pressure value
#define				MS5611_CONVD1_OSR256			0x40
#define				MS5611_CONVD1_OSR512			0x42
#define				MS5611_CONVD1_OSR1024			0x44
#define				MS5611_CONVD1_OSR2048			0x46
#define				MS5611_CONVD1_OSR4096			0x48

// D2转换：Digital temperature value
#define				MS5611_CONVD2_OSR256			0x50
#define				MS5611_CONVD2_OSR512			0x52
#define				MS5611_CONVD2_OSR1024			0x54
#define				MS5611_CONVD2_OSR2048			0x56
#define				MS5611_CONVD2_OSR4096			0x58


#define				MS5611_ADC_READ						0x00


#define				PROM_READ_PRE							0xA0 //读取系数指令前缀

#define				PROM_READ_C1							(PROM_READ_PRE | (0x01 << 1))//按位或 ||表示结果是1
#define				PROM_READ_C2							(PROM_READ_PRE | (0x02 << 1))
#define				PROM_READ_C3							(PROM_READ_PRE | (0x03 << 1))
#define				PROM_READ_C4							(PROM_READ_PRE | (0x04 << 1))
#define				PROM_READ_C5							(PROM_READ_PRE | (0x05 << 1))
#define				PROM_READ_C6							(PROM_READ_PRE | (0x06 << 1))



// registers of the device
#define MS561101BA_PRESSURE    0x40
#define MS561101BA_TEMPERATURE 0x50
#define MS561101BA_RESET       0x1E



int32_t i2c_ms5611_calculate(void);
void i2c_ms5611_init(void);


int32_t i2c_ms5611_calculate_s(void);
void i2c_ms5611_init_s(void);




#endif
