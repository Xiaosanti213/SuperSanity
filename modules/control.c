/**
 *
 * @file control.c
 *
 * 控制算法
 *
 **/



#include <stm32f10x.h>
#include "control.h"
#include "math.h"
#include "mpu6050.h"
#include "imu.h"
#include "stdio.h"
#include "motor.h"
#include "attitude_control.h"
#include "board_config.h"

extern uint32_t micros(void);

uint8_t offLandFlag=0;
u8 ok_to_spin = 0;// 油门较低时，不更新积分量，解锁之后不转



//volatile unsigned char motorLock=1;

int16_t Motor[4]={0};   //定义电机PWM数组，分别对应M1-M4
float rollSp =0,pitchSp =0;		//根据动力分配重新计算得到的期望roll pitch
float Thro=0,Roll=0,Pitch=0,Yaw=0;

#define Moto_PwmMax 999



//----PID结构体实例化----
PID_Typedef pitch_angle_PID;	//pitch角度环的PID
PID_Typedef pitch_rate_PID;		//pitch角速率环的PID

PID_Typedef roll_angle_PID;   //roll角度环的PID
PID_Typedef roll_rate_PID;    //roll角速率环的PID

PID_Typedef yaw_angle_PID;    //yaw角度环的PID 
PID_Typedef yaw_rate_PID;     //yaw角速率环的PID

/*
PID_Typedef	alt_PID;
PID_Typedef alt_vel_PID;



float gyroxGloble = 0;
float gyroyGloble = 0;


uint32_t ctrlPrd=0;
uint8_t headFreeMode=0;
float headHold=0;
*/

#define   ONE_G  9.80665f





/**
 *
 * 函数: ParamSetDefault
 *
 * 作用：PID设置参数
 *
 */
void ParamSetDefault(void)
{

    pitch_angle_PID.P = 3.5;
    pitch_angle_PID.I = 0;//1.0;		//0
    pitch_angle_PID.D = 0;

    pitch_angle_PID.iLimit = 300;	//or 1000

    pitch_rate_PID.P  = 0.7;
    pitch_rate_PID.I  = 0.5; 		//0.5
    pitch_rate_PID.D  = 0.03;

    pitch_rate_PID.iLimit = 300;
////////////////////////////////////////////
    roll_angle_PID.P = 3.5;
    roll_angle_PID.I = 0;//1.0;
    roll_angle_PID.D = 0;
    roll_angle_PID.iLimit = 300;	//or 1000

    roll_rate_PID.P  = 0.7;
    roll_rate_PID.I  = 0.5;; 	//0.5
    roll_rate_PID.D  = 0.03;
    roll_rate_PID.iLimit = 300;
///////////////////////////////////////////
    yaw_angle_PID.P = 1;
    yaw_angle_PID.I = 0.2;
    yaw_angle_PID.D = 0;

    yaw_rate_PID.P  = 5;
    yaw_rate_PID.I  = 0.5;
    yaw_rate_PID.D  = 0;



/*
    alt_PID.P=1.0;
    alt_PID.I=0;
    alt_PID.D=0;

    alt_vel_PID.P=0.1f;
    alt_vel_PID.I=0.02f;
    alt_vel_PID.D=0;

    //should chango to read eeprom cfg. should be 0.
    imu.accOffset[0]=-0.1620515;
    imu.accOffset[1]=0.07422026;
    imu.accOffset[2]=0.7743073;

    imu.gyroOffset[0]=-0.06097556;
    imu.gyroOffset[1]=-0.03780485;
    imu.gyroOffset[2]=0;

*/
//		AttiCtrlParamsFromPIDTable();

}




/**
 *
 * 函数: PID_Postion_Cal
 *
 * 作用：PID函数封装
 *
 */
static void PID_Postion_Cal(PID_Typedef * PID,float target,float measure,int32_t dertT)
{
	float termI=0;
	float dt= dertT/1000000.0;//单位：us->s

	//误差=期望值-测量值
	PID->Error=target-measure;

	PID->Deriv= (PID->Error-PID->PreError)/dt;

	PID->Output=(PID->P * PID->Error) + (PID->I * PID->Integ) + (PID->D * PID->Deriv);    //PID:比例环节+积分环节+微分环节

	PID->PreError=PID->Error;
	//仅用于角度环和角速度环的

	if(arm_motors_flag && ok_to_spin){							//解锁之后油门打到一定量才可以更新积分
		if(fabs(PID->Output) < Thro )		              //比油门还大时不积分
		{
			termI=(PID->Integ) + (PID->Error) * dt;     //积分环节
			if(termI > - PID->iLimit && termI < PID->iLimit && PID->Output > - PID->iLimit && PID->Output < PID->iLimit)       //在-300~300时才进行积分环节
					PID->Integ=termI;
		}
	}else{
		PID->Integ= 0;
	}
}




/**
 *
 * 函数: CtrlAttiAng
 *
 * 作用：外环角度控制
 *
 */
void CtrlAttiAng(void)
{
	
	  //更新计算时间
		static uint32_t tPrev=0;
		float dt=0,t=0;
		t=micros();//单位：us
		dt=(tPrev>0)?(t-tPrev):0;
		tPrev=t;
		
		//计算外环PID
		PID_Postion_Cal(&pitch_angle_PID,reference[PITCH],imu.pitch,dt);
		PID_Postion_Cal(&roll_angle_PID,reference[ROLL],imu.roll,dt);	 
}






/**
 *
 * 函数: CtrlAttiRate
 *
 * 作用：内环角速率控制
 *
 */
void CtrlAttiRate(void)
{
	static uint32_t tPrev=0; 

	// 更新计算时间
	float dt=0,t=0;
	t=micros();
	dt=(tPrev>0)?(t-tPrev):0;
	tPrev=t;
		
	
	// 计算内环角速率PID控制输出
	PID_Postion_Cal(&pitch_rate_PID,pitch_angle_PID.Output,imu.gyro[PITCH]*57.2958,dt);	
	PID_Postion_Cal(&roll_rate_PID,roll_angle_PID.Output,imu.gyro[ROLL]*57.2958,dt);
	PID_Postion_Cal(&yaw_rate_PID,reference[2],imu.gyro[2]*57.2958,dt);
  
	
  // 得到全局变量输出
	Pitch = pitch_rate_PID.Output;
	Roll  = roll_rate_PID.Output;
	Yaw   = yaw_rate_PID.Output;

	/*
  printf("内环角速率PID输出为: \n");
	printf("Roll: %.2f\n", Roll);
	printf("Pitch: %.2f\n", Pitch);
	printf("Yaw: %.2f\n", Yaw);	
	*/
}








/**
 *
 * 函数: dbScaleLinear
 *
 * 作用：可被外部调用
 *
 */
float dbScaleLinear(float x, float x_end, float deadband)
{
	if (x > deadband) {
		return (x - deadband) / (x_end - deadband);

	} else if (x < -deadband) {
		return (x + deadband) / (x_end - deadband);

	} else {
		return 0.0f;
	}
}





/*
//函数名：CtrlAlti()
//输入：无
//输出: 最终结果输出到全局变量thrustZSp
//描述：控制高度，也就是高度悬停控制函数
//only in climb rate mode and landind mode. now we don't work on manual mode
void CtrlAlti(void)
{
	float manThr=0,alt=0,velZ=0;
	float altSp=0;
	float posZVelSp=0;
	float altSpOffset,altSpOffsetMax=0;
	float dt=0,t=0;
	static float tPrev=0,velZPrev=0;
	float posZErr=0,velZErr=0,valZErrD=0;
	float thrustXYSpLen=0,thrustSpLen=0;
	float thrustXYMax=0;
	float minThrust;
	
	//get dt		
	//保证dt运算不能被打断，保持更新，否则dt过大，积分爆满。
	if(tPrev==0){
			tPrev=micros();
			return;
	}else{
			t=micros();
			dt=(t-tPrev) /1000000.0f;
			tPrev=t;
	}
	
	
	//only in climb rate mode and landind mode. now we don't work on manual mode
	//手动模式不使用该高度控制算法
	if(MANUAL == altCtrlMode || !FLY_ENABLE){
		return;
	}
	
	//--------------pos z ctrol---------------//
	//get current alt 
	alt=-nav.z;
	//get desired move rate from stick
	manThr=RC_DATA.THROTTLE / 1000.0f;
	spZMoveRate= -dbScaleLinear(manThr-0.5f,0.5f,ALT_CTRL_Z_DB);	// scale to -1~1 . NED frame
	spZMoveRate = spZMoveRate * ALT_VEL_MAX;	// scale to vel min max

	//get alt setpoint in CLIMB rate mode
	altSp 	=-nav.z;						//only alt is not in ned frame.
	altSp  -= spZMoveRate * dt;	 
	//limit alt setpoint
	altSpOffsetMax=ALT_VEL_MAX / alt_PID.P * 2.0f;
	altSpOffset = altSp-alt; 
	if( altSpOffset > altSpOffsetMax){
		altSp=alt +  altSpOffsetMax;
	}else if( altSpOffset < -altSpOffsetMax){
		altSp=alt - altSpOffsetMax;
	}

	//限高
	if(isAltLimit)
	{
		if(altSp - altLand > ALT_LIMIT)
		{
				altSp=altLand+ALT_LIMIT;
				spZMoveRate=0;
		}
	}
	
	// pid and feedforward control . in ned frame
	posZErr= -(altSp - alt);
	posZVelSp = posZErr * alt_PID.P + spZMoveRate * ALT_FEED_FORWARD;
	//consider landing mode
	if(altCtrlMode==LANDING)
		posZVelSp = LAND_SPEED;
	
	//获取一个预估的Z轴悬停基准值，相关因素有电池电压
	//get hold throttle. give it a estimated value
	if(zIntReset){
		thrustZInt = estimateHoverThru();
		zIntReset = 0;
	}
	
	velZ=nav.vz;	
	velZErr = posZVelSp - velZ;
	valZErrD = (spZMoveRate - velZ) * alt_PID.P - (velZ - velZPrev) / dt;	//spZMoveRate is from manual stick vel control
	velZPrev=velZ;
	
	thrustZSp= velZErr * alt_vel_PID.P + valZErrD * alt_vel_PID.D + thrustZInt;	//in ned frame. thrustZInt contains hover thrust
	
	//限制最小下降油门
	minThrust = estimateMinThru();
	if(altCtrlMode!=LANDING){
		if (-thrustZSp < minThrust){
			thrustZSp = -minThrust; 
		} 
	}
	
	//与动力分配相关	testing
	satXY=0;
	satZ=0;
	thrustXYSp[0]= sinf(RC_DATA.ROOL * M_PI_F /180.0f) ;//目标角度转加速度
	thrustXYSp[1]= sinf(RC_DATA.PITCH * M_PI_F /180.0f) ; 	//归一化
	thrustXYSpLen= sqrtf(thrustXYSp[0] * thrustXYSp[0] + thrustXYSp[1] * thrustXYSp[1]);
	//limit tilt max
	if(thrustXYSpLen >0.01f )
	{
		thrustXYMax=-thrustZSp * tanf(TILT_MAX);
		if(thrustXYSpLen > thrustXYMax)
		{
				float k=thrustXYMax / thrustXYSpLen;
				thrustXYSp[1] *= k;
				thrustXYSp[0] *= k;
				satXY=1;
				thrustXYSpLen= sqrtf(thrustXYSp[0] * thrustXYSp[0] + thrustXYSp[1] * thrustXYSp[1]);
		}
		
	}
	
	
	//limit max thrust!! 
	thrustSpLen=sqrtf(thrustXYSpLen * thrustXYSpLen + thrustZSp * thrustZSp);
	if(thrustSpLen > THR_MAX)
	{
			if(thrustZSp < 0.0f)	//going up
			{
						if (-thrustZSp > THR_MAX) 
						{
								// thrust Z component is too large, limit it 
								thrustXYSp[0] = 0.0f;
								thrustXYSp[1] = 0.0f;
								thrustZSp = -THR_MAX;
								satXY = 1;
								satZ = 1;

							} 
							else {
								float k = 0;
								// preserve thrust Z component and lower XY, keeping altitude is more important than position 
								thrustXYMax = sqrtf(THR_MAX * THR_MAX- thrustZSp * thrustZSp);
								k=thrustXYMax / thrustXYSpLen;
								thrustXYSp[1] *=k;
								thrustXYSp[0] *= k;
								satXY=1;
							}
			}
			else {		//going down
							// Z component is negative, going down, simply limit thrust vector 
							float k = THR_MAX / thrustSpLen;
							thrustZSp *= k;
							thrustXYSp[1] *=k;
							thrustXYSp[0] *= k;
							satXY = 1;
							satZ = 1;
						}
		
	} 
	rollSp= asinf(thrustXYSp[0]) * 180.0f /M_PI_F;
	pitchSp = asinf(thrustXYSp[1]) * 180.0f /M_PI_F;				
	
	
	// if saturation ,don't integral
	if(!satZ )//&& fabs(thrustZSp)<THR_MAX
	{
			thrustZInt += velZErr * alt_vel_PID.I * dt;
			if (thrustZInt > 0.0f)
							thrustZInt = 0.0f;
	}
}
*/




/**
 *
 * 函数: CtrlMotor()
 *
 * 作用：将控制量映射到4个电机的PWM输出 100Hz内环调用
 *
 */
void CtrlMotor(void)
{
		float  cosTilt = imu.accb[2] / ONE_G; //两个倾角余弦相乘
	  
	  
		Thro = reference[3];
	  /*
	  // 补偿推力 当前由于噪声会出现问题
		cosTilt=imu.DCMgb[2][2];
		Thro=Thro/cosTilt;
    */
		
		// 将输出值融合到四个电机 
		Motor[2] = (int16_t)(Thro - Pitch + Roll + Yaw );    //M3  
		Motor[0] = (int16_t)(Thro + Pitch - Roll + Yaw );    //M1
		Motor[3] = (int16_t)(Thro - Pitch - Roll - Yaw );    //M4 
		Motor[1] = (int16_t)(Thro + Pitch + Roll - Yaw );    //M2 
	  
	  // 限定输入不能小于0，大于999
	  if(Motor[0]>=Moto_PwmMax)	Motor[0] = Moto_PwmMax;
    if(Motor[1]>=Moto_PwmMax)	Motor[1] = Moto_PwmMax;
    if(Motor[2]>=Moto_PwmMax)	Motor[2] = Moto_PwmMax;
    if(Motor[3]>=Moto_PwmMax)	Motor[3] = Moto_PwmMax;
	
    if(Motor[0]<=0)	Motor[0] = 0;
    if(Motor[1]<=0)	Motor[1] = 0;
    if(Motor[2]<=0)	Motor[2] = 0;
    if(Motor[3]<=0)	Motor[3] = 0;
		
		
	  printf("电机输出:  \n");
   	printf("Motor-1: %d\n",Motor[0]);
	  printf("Motor-2: %d\n",Motor[1]);
	  printf("Motor-3: %d\n",Motor[2]);
	  printf("Motor-4: %d\n",Motor[3]);
	  
		
	  // 判断是否解锁
   	if(arm_motors_flag && Thro>80)
		{
			ok_to_spin = 1;
			
			TIM2->CCR1 = Motor[0];
			TIM2->CCR2 = Motor[1];
			TIM2->CCR3 = Motor[2];
			TIM2->CCR4 = Motor[3];        //对定时器寄存器赋值
		}
		else                  
		{
			ok_to_spin = 0;
			
			TIM2->CCR1 = 0;
			TIM2->CCR2 = 0;
			TIM2->CCR3 = 0;
			TIM2->CCR4 = 0;
		}
}




