clear all
close all
clc

%ms5611ͨ��У׼ֵ��ADCֵ������ѹ���¶�
%coefficient = [40127 36924 23317 23282 33464 28312];
%adc = [9085466, 8569150];

coefficient = [47402 50062 28773 25665 32046 27372];
adc = [8926224, 8392932];
%���������ֲ�7/20ҳ���
%�����¶�
dT = adc(2) - coefficient(5) * 2^8;
TEMP = 2000 + dT * coefficient(6)/2^23;

%�����¶Ȳ�����ѹ��
OFF = coefficient(2) * 2^16 + (coefficient(4) * dT)/2^7;
SENS = coefficient(1) * 2^15 + (coefficient(3) * dT)/2^8;
P = (adc(1) * SENS / 2^21 - OFF)/2^15;





%mpu6050 acc ADCֵת����m/s2
acc_adc = [-624, -504, 15624];
acc_adc = acc_adc / 32768 * 2; 
%���ٶȼ�,ԭʼ���ݣ��з���16bit������Ϊ+-2G
acc = acc_adc * 9.8;



%mpu6050 acc ADCֵת����dps
gyro_adc = [-32, 25, -5];
gyro_adc = gyro_adc / 32768 * 2000;
%������,ԭʼ���ݣ��з���16bit������Ϊ2000 degree per second



%hmc5883l������

