/**
 *
 * @file IMUSO3.c
 *
 * mahony姿态解算，参考Crazepony略有改动
 *
 **/

#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include <math.h>
#include "IMU.h"
#include "IMUSO3.h"

#include "stdio.h"


//! Auxiliary variables to reduce number of repeated operations
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	/** quaternion of sensor frame relative to auxiliary frame */
static float dq0 = 0.0f, dq1 = 0.0f, dq2 = 0.0f, dq3 = 0.0f;	/** quaternion of sensor frame relative to auxiliary frame */
static float gyro_bias[3] = {0.0f, 0.0f, 0.0f}; /** bias estimation */
static float q0q0, q0q1, q0q2, q0q3;
static float q1q1, q1q2, q1q3;
static float q2q2, q2q3;
static float q3q3;
static uint8_t bFilterInit = 0; //需要解算静止时候姿态标志位






/**
 *  函数: invSqrt
 *
 *  描述：卡马克快速开方根
 *
 */
static float invSqrt(float number)
{
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );
    return y;
}








/**
 *  函数: NonlinearSO3AHRSinit
 *
 *  描述：静止状态姿态初始化
 *
 */
static void NonlinearSO3AHRSinit(float ax, float ay, float az, float mx, float my, float mz)
{
    float initialRoll, initialPitch;
    float cosRoll, sinRoll, cosPitch, sinPitch;
    float magX, magY;
    float initialHdg, cosHeading, sinHeading;

    initialRoll = atan2(ay, az);
		initialPitch = asin(-ax);
	
    cosRoll = cosf(initialRoll);
    sinRoll = sinf(initialRoll);
    cosPitch = cosf(initialPitch);
    sinPitch = sinf(initialPitch);

    /*
		magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;

    magY = my * cosRoll - mz * sinRoll;

    initialHdg = atan2f(magY, magX);
		*/
		initialHdg = 0;
	
    cosRoll = cosf(initialRoll * 0.5f);
    sinRoll = sinf(initialRoll * 0.5f);

    cosPitch = cosf(initialPitch * 0.5f);
    sinPitch = sinf(initialPitch * 0.5f);

    cosHeading = cosf(initialHdg * 0.5f);
    sinHeading = sinf(initialHdg * 0.5f);

    q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
    q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
    q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
    q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

    // auxillary variables to reduce number of repeated operations, for 1st pass
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;
}








/**
 *  函数: NonlinearSO3AHRSinit
 *
 *  描述：使用的是Mahony互补滤波算法，没有使用Kalman滤波算法
 *        改算法是直接参考pixhawk飞控的算法，可以在Github上看到出处
 *			  https://github.com/hsteinhaus/PX4Firmware/blob/master/src/modules/attitude_estimator_so3/attitude_estimator_so3_main.cpp
 *
 */
static void NonlinearSO3AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float twoKp, float twoKi, float dt)
{
    float recipNorm;
    float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;

    // 初始静止状态姿态解算.初始化一次
    if(bFilterInit == 0) {
        NonlinearSO3AHRSinit(ax,ay,az,mx,my,mz);
        bFilterInit = 1;
    }

    // 融合磁力计数据
    if(!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {
        float hx, hy, hz, bx, bz;
        float halfwx, halfwy, halfwz;

        // 正交化磁力计数据
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

			  // hx hy hz为测量值旋转到地系下的地磁分量
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        hz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);
        
				// 合成地系下水平地磁分量
			  bx = sqrt(hx * hx + hy * hy);
        bz = hz;

		  	// 地理北极矢量投影到机体坐标系下
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // 地系下的地磁北极和地磁分量叉积
        halfex += (mz * halfwy - my * halfwz);
        halfey += (mx * halfwz - mz * halfwx);
        halfez += (my * halfwx - mx * halfwy);
    }

    // 增加一个条件：加速度的模量与G相差不远时。 0.75*G < normAcc < 1.25*G
    // 三轴加计数据不都为0，防止正交化出现NaN
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        float halfvx, halfvy, halfvz;

        //归一化，得到单位加速度
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);

        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 估计的重力在体系下的方向
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;

        // Error is sum of   cross product between estimated direction and measured direction of field vectors
        halfex += -(az * halfvy - ay * halfvz);
        halfey += -(ax * halfvz - az * halfvx);
        halfez += -(ay * halfvx - ax * halfvy);
			
			//printf("\n%.2f, %.2f, %.2f", halfex, halfey, halfez);//检查是否收敛
    }

    // 计算反馈
    if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
        // 积分
        if(twoKi > 0.0f) {
					  // 累加积分反馈
            gyro_bias[0] += twoKi * halfex * dt;	
            gyro_bias[1] += twoKi * halfey * dt;
            gyro_bias[2] += twoKi * halfez * dt;

					  // 补充到陀螺仪上
            gx += gyro_bias[0];
            gy += gyro_bias[1];
            gz += gyro_bias[2];
        }
        else {
            gyro_bias[0] = 0.0f;	// 抗饱和
            gyro_bias[1] = 0.0f;
            gyro_bias[2] = 0.0f;
        }

        // 比例反馈补充到陀螺仪上
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // 角速度带入微分方程更新姿态四元数. q_dot = 0.5*q\otimes omega.
    //! q_k = q_{k-1} + dt*\dot{q}
    //! \dot{q} = 0.5*q \otimes P(\omega)
    dq0 = 0.5f*(-q1 * gx - q2 * gy - q3 * gz);
    dq1 = 0.5f*(q0 * gx + q2 * gz - q3 * gy);
    dq2 = 0.5f*(q0 * gy - q1 * gz + q3 * gx);
    dq3 = 0.5f*(q0 * gz + q1 * gy - q2 * gx);

    q0 += dt*dq0;
    q1 += dt*dq1;
    q2 += dt*dq2;
    q3 += dt*dq3;

    // 正交化
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;
}

#define so3_comp_params_Kp 1.0f
#define so3_comp_params_Ki  0.05f











/**
 *  函数: IMUSO3Thread
 *
 *  描述：main()当中调用
 *        
 *			  
 *
 */
void IMUSO3Thread(void)
{
    //! Time constant
    float dt = 0.01f;		//s
    static uint32_t tPrev=0,startTime=0;	//us
    uint32_t now;
    uint8_t i;

    /* output euler angles */
    float euler[3] = {0.0f, 0.0f, 0.0f};	//rad

    /* Initialization */
    float Rot_matrix[9] = {1.f,  0.0f,  0.0f, 0.0f,  1.f,  0.0f, 0.0f,  0.0f,  1.f };		/**< init: identity matrix */
    float acc[3] = {0.0f, 0.0f, 0.0f};		//m/s^2
    float gyro[3] = {0.0f, 0.0f, 0.0f};		//rad/s
    float mag[3] = {0.0f, 0.0f, 0.0f};
    //need to calc gyro offset before imu start working
    static float gyro_offsets_sum[3]= { 0.0f, 0.0f, 0.0f }; // gyro_offsets[3] = { 0.0f, 0.0f, 0.0f },
    static uint16_t offset_count = 0;

		// 更新时间变量 单位us
    now=micros();
    dt=(tPrev>0)?(now-tPrev)/1000000.0f:0;//us->s
    tPrev=now;

		// 读取传感器测量值
    ReadIMUSensorHandle();

		// 陀螺仪校准(此处禁用，因为在主函数中已经校准过)
    /*
		if(0)//!imu.ready
    {
        if(startTime==0)
            startTime=now;

        gyro_offsets_sum[0] += imu.gyroRaw[0];
        gyro_offsets_sum[1] += imu.gyroRaw[1];
        gyro_offsets_sum[2] += imu.gyroRaw[2];
        offset_count++;

        if(now > startTime + GYRO_CALC_TIME)
        {
            imu.gyroOffset[0] = gyro_offsets_sum[0]/offset_count;
            imu.gyroOffset[1] = gyro_offsets_sum[1]/offset_count;
            imu.gyroOffset[2] = gyro_offsets_sum[2]/offset_count;

            offset_count=0;
            gyro_offsets_sum[0]=0;
            gyro_offsets_sum[1]=0;
            gyro_offsets_sum[2]=0;

            imu.ready = 1;
            startTime=0;

        }
        return;
    }
		*/
		
    gyro[0] = imu.gyro[0];
    gyro[1] = imu.gyro[1];
    gyro[2] = imu.gyro[2];
		

    acc[0] = imu.accb[0];
    acc[1] = imu.accb[1];
    acc[2] = imu.accb[2];
		
		/*
		printf("acc:  %.2f, %.2f, %.2f",acc[0],acc[1],acc[2]);
		printf("gyro:  %.2f, %.2f, %.2f",gyro[0],gyro[1],gyro[2]);
		*/


    // 用校准后传感器值更新姿态四元数
    NonlinearSO3AHRSupdate(gyro[0], gyro[1], gyro[2],
                           acc[0], acc[1], acc[2],
                           mag[0], mag[1], mag[2],
                           so3_comp_params_Kp,
                           so3_comp_params_Ki,
                           dt);

    // 四元数转换成DCM旋转矩阵e->b
    Rot_matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;// 11
    Rot_matrix[1] = 2.f * (q1*q2 + q0*q3);	  // 12
    Rot_matrix[2] = 2.f * (q1*q3 - q0*q2);	  // 13
    Rot_matrix[3] = 2.f * (q1*q2 - q0*q3);	  // 21
    Rot_matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;// 22
    Rot_matrix[5] = 2.f * (q2*q3 + q0*q1);	  // 23
    Rot_matrix[6] = 2.f * (q1*q3 + q0*q2);	  // 31
    Rot_matrix[7] = 2.f * (q2*q3 - q0*q1);	  // 32
    Rot_matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;// 33

    //DCM -> Euler Angle.
    euler[0] = atan2f(Rot_matrix[5], Rot_matrix[8]);	// Roll 
    euler[1] = -asinf(Rot_matrix[2]);									// Pitch
    euler[2] = atan2f(Rot_matrix[1], Rot_matrix[0]);

    //DCM . ground to body 将向量写成矩阵形式表达
    for(i=0; i<9; i++)
    {
        *(&(imu.DCMgb[0][0]) + i)=Rot_matrix[i];
    }
		
		// 解算得到三轴角速率和角度
    imu.rollRad=euler[0];
    imu.pitchRad=euler[1];
    imu.yawRad=euler[2];

		// 角度为单位
    imu.roll=euler[0] * 57.296;
    imu.pitch=euler[1] * 57.296;
    imu.yaw=euler[2] * 57.296;
		
		/*
		printf("roll: %.2f\n", imu.roll);
		printf("pitch: %.2f\n", imu.pitch);
		printf("yaw: %.2f\n", imu.yaw);
		*/
}

