clear all
close all
clc

%ms5611通过校准值和ADC值计算气压与温度
%coefficient = [40127 36924 23317 23282 33464 28312];
%adc = [9085466, 8569150];

coefficient = [47402 50062 28773 25665 32046 27372];
adc = [8926224, 8392932];
%按照数据手册7/20页表格
%计算温度
dT = adc(2) - coefficient(5) * 2^8;
TEMP = 2000 + dT * coefficient(6)/2^23;

%计算温度补偿的压力
OFF = coefficient(2) * 2^16 + (coefficient(4) * dT)/2^7;
SENS = coefficient(1) * 2^15 + (coefficient(3) * dT)/2^8;
P = (adc(1) * SENS / 2^21 - OFF)/2^15;





%mpu6050 acc ADC值转化成m/s2
acc_adc = [-624, -504, 15624];
acc_adc = acc_adc / 32768 * 2; 
%加速度计,原始数据：有符号16bit，量程为+-2G
acc = acc_adc * 9.8;



%mpu6050 acc ADC值转化成dps
gyro_adc = [-32, 25, -5];
gyro_adc = gyro_adc / 32768 * 2000;
%陀螺仪,原始数据：有符号16bit，量程为2000 degree per second



%hmc5883l磁罗盘

