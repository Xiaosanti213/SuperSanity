/**
 *
 * @file sensors.c
 *
 * 传感器初始化与数据采集
 *
 **/

#include <stm32f10x.h>

#include "board_config.h"
#include "debug.h"
#include "hmc5883l.h"
#include "mpu6050.h"

#include "ms5611.h"
#include "sensors.h"
#include "attitude_estimate.h"
#include <stdio.h>
#include <math.h>









