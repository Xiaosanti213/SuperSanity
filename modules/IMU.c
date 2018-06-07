/**
 *
 * @file IMUSO3.c
 *
 * mahony姿态解算，参考Crazepony略有改动
 *
 **/
 
 
#include "IMU.h"
#include "filter.h"
#include "stm32f10x_it.h"
#include "mpu6050.h"




imu_t imu= {0};
uint8_t imuCaliFlag=0;




/*
 *  名称: IMU_Init
 *
 *  说明：数据校准滤波初始化设置函数
 *
 */
void IMU_Init(void)
{

  	imu.ready=0; // 数据校准标志位
    imu.caliPass=1;// 数据校准通过
    //filter rate
    LPF2pSetCutoffFreq_1(IMU_SAMPLE_RATE,IMU_FILTER_CUTOFF_FREQ);		//30Hz
    LPF2pSetCutoffFreq_2(IMU_SAMPLE_RATE,IMU_FILTER_CUTOFF_FREQ);
    LPF2pSetCutoffFreq_3(IMU_SAMPLE_RATE,IMU_FILTER_CUTOFF_FREQ);
    LPF2pSetCutoffFreq_4(IMU_SAMPLE_RATE,IMU_FILTER_CUTOFF_FREQ);
    LPF2pSetCutoffFreq_5(IMU_SAMPLE_RATE,IMU_FILTER_CUTOFF_FREQ);
    LPF2pSetCutoffFreq_6(IMU_SAMPLE_RATE,IMU_FILTER_CUTOFF_FREQ);
}





/*
 *  名称: IMU_Init
 *
 *  说明：飞机水平静止1~2s成功返回1
 *
 */
uint8_t IMU_Calibrate(void)
{
	  //3s 校准过程上电之后执行一次，如果有必要之后才会重新执行
    static float accSum[3]= {0,0,0};
    static float gyroSum[3]= {0,0,0};
    static uint16_t cnt=0;
    static uint16_t tPrev=0;
    static uint8_t calibrating=0;
    uint8_t ret=0;
    uint8_t i=0;
    uint16_t dt=0,now=0;

    now=millis(); // 更新计算时间间隔
    dt=now-tPrev;

    if(calibrating==0)
    {
        calibrating=1;
        for(i=0; i<3; i++)
        {
            accSum[i]=0;
            gyroSum[i]=0;
            cnt=0;
            imu.ready=0;
        }

    }

    if(dt>=10)		//10ms
    {
        if(cnt<300) // 累加
        {
            for(i=0; i<3; i++)
            {
                accSum[i]+=imu.accRaw[i];
                gyroSum[i]+=imu.gyroRaw[i];
            }
            cnt++;
            tPrev=now;
        }
        else // 求平均
        {
            for(i=0; i<3; i++)
            {
                imu.accOffset[i]=accSum[i]/(float)cnt;
                imu.gyroOffset[i]=gyroSum[i]/(float)cnt;
            }

            imu.accOffset[2]=imu.accOffset[2] - CONSTANTS_ONE_G;

            calibrating=0;

            imu.ready=1;

            ret=1;
            //计算结果存储到imu.xxxOffset当中
        }
    }

    return ret;
}





/*
 *  名称: IMU_Init
 *
 *  说明：飞机水平静止1~2s成功返回1
 *
 */
#define SENSOR_MAX_G 2.0f		//constant g
#define SENSOR_MAX_W 2000.0f	//deg/s
#define ACC_SCALE  (SENSOR_MAX_G/32768.0f)
#define GYRO_SCALE  (SENSOR_MAX_W/32768.0f)
void ReadIMUSensorHandle(void)
{
    uint8_t i;

    //读取原生数据
    i2c_mpu6050_read_acc_s(imu.accADC);
    i2c_mpu6050_read_gyro_s(imu.gyroADC);
    
	  //单位转换
    for(i=0; i<3; i++)
    {   
			  // 加计：原生数据
        imu.accRaw[i] = (float)imu.accADC[i] * ACC_SCALE * CONSTANTS_ONE_G ;
			  // 
        imu.gyroRaw[i]=(float)imu.gyroADC[i] * GYRO_SCALE * M_PI_F /180.f;		//deg/s
    }

		// 执行滤波
    imu.accb[0]=LPF2pApply_1(imu.accRaw[0]-imu.accOffset[0]);
    imu.accb[1]=LPF2pApply_2(imu.accRaw[1]-imu.accOffset[1]);
    imu.accb[2]=LPF2pApply_3(imu.accRaw[2]-imu.accOffset[2]);
		
		// 这块暂时不太清楚为什么没有直接去掉偏移量而是可能在后面还重新校准陀螺仪
    imu.gyro[0]=LPF2pApply_4(imu.gyroRaw[0]);
    imu.gyro[1]=LPF2pApply_5(imu.gyroRaw[1]);
    imu.gyro[2]=LPF2pApply_6(imu.gyroRaw[2]);

}





/*
 *  名称: IMUCheck
 *
 *  说明：检测IMU是否ready，校准好
 *				需要将四轴放水平
 *
 */
#define ACCZ_ERR_MAX  0.05		//m/s^2
#define CHECK_TIME 5
uint8_t IMUCheck(void)
{
    uint32_t accZSum=0;
    uint8_t i;
    float accZb=0;

    for(i=0; i<CHECK_TIME; i++)
    {
        i2c_mpu6050_read_acc_s(imu.accADC);
        accZSum += imu.accADC[2];
    }
    imu.accRaw[2]= (float)(accZSum /(float)CHECK_TIME) *ACC_SCALE * CONSTANTS_ONE_G ;
    accZb=imu.accRaw[2]-imu.accOffset[2];

    if((accZb > CONSTANTS_ONE_G-ACCZ_ERR_MAX ) && (accZb < CONSTANTS_ONE_G + ACCZ_ERR_MAX))
        imu.caliPass=1;
    else
        imu.caliPass=0;

    return imu.caliPass;

}






/*
 *  名称: eular2DCM
 *
 *  说明：欧拉角计算旋转矩阵
 *
 */
static void eular2DCM(float DCM[3][3],float roll,float pitch,float yaw)
{
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, coszcosy, sinzcosx, coszsinx, sinzsinx;

    cosx = cosf(roll * M_PI_F/180.0f);
    sinx = sinf(roll * M_PI_F/180.0f);
    cosy = cosf(pitch * M_PI_F/180.0f);
    siny = sinf(pitch * M_PI_F/180.0f);
    cosz = cosf(yaw * M_PI_F/180.0f);
    sinz = sinf(yaw * M_PI_F/180.0f);

    coszcosx = cosz * cosx;
    coszcosy = cosz * cosy;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;

    DCM[0][0] = coszcosy;
    DCM[0][1] = cosy * sinz;
    DCM[0][2] = -siny;
    DCM[1][0] = -sinzcosx + (coszsinx * siny);
    DCM[1][1] = coszcosx + (sinzsinx * siny);
    DCM[1][2] = sinx * cosy;
    DCM[2][0] = (sinzsinx) + (coszcosx * siny);
    DCM[2][1] = -(coszsinx) + (sinzcosx * siny);
    DCM[2][2] = cosy * cosx;

}

