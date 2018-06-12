/**
 *
 * @file attitude_control.c
 *
 * 内环P控制 外环PI控制
 *
 **/



#include "attitude_estimate.h"
#include "attitude_control.h"
#include <stdio.h>
#include <math.h>





/**
 *
 * 名称：attitude_control
 *
 * 描述：内环P控制外环PI控制
 *
 */
void attitude_control(ad attitude_data, float* reference, int16_t* output, u16 thr_command)
{
	float K = 1;						  						  			//内环大增益很快收敛
	float Ko = 2;						  				    				//外环增益
	float Ki = 60;
	//float Kd = 10;															//带上Kd/(3*dT)其中1/dT约100
	//float Ky = 30;															//偏航增益
	//float T = 0.02;														//应该使用当前的执行时间
	//float error_i;
	//static float euler_angle_pre[2] = {0,0};  
	//float p_term;
	//float i_term;
	float p_term_in = 0;
	//float d_term_in = 0;
	static float i_term_in[3] = {0,0,0}; 
	
	float rate_error = 0;												//内环误差
	//static float last_error[2] = {0,0};

	/*
	static float rate_er_delta = 0;			 				//内环误差变化量
	static float rate_er_delta1[2] = {0,0};
	static float rate_er_delta2[2] = {0,0};
	*/
	
	//static float u_o_pre[2] = {0,0};
	float u_o[2];																//外环rp轴输出
	u8 i;
	const u16 output_limit=500; 
	
	//float angle_trim[2] = {0.9, -0.1};          //校准遥控器出错导致
	
	float dT = 0.02;														//单位：s
	
	
	
	//Roll Pitch Yaw轴
	for (i=0; i<3; i++)
	{
		// 滚转俯仰轴
		/**
		//外环 IP控制
		p_term = Kp*(-attitude_data.euler_angle[i]+euler_angle_pre[i]);   //前一次和当前欧拉角做差
		i_term = Kp/tauI*(reference[i]-attitude_data.euler_angle[i])*T;   
		u_o[i] = p_term+i_term+u_o_pre[i];

		//内环
		error_i = u_o[i]-attitude_data.angle_rate[i];
		output[i] = K*error_i; 	
		**/

		//MWC的控制模式
		
		//外环：P
		if(i < 2)
		{
			//reference重新限定在[-25,25]范围内
			if(reference[i]>50)
				reference[i] = 50;
			else if(reference[i]<-50)
				reference[i] = -50;

			u_o[i] = Ko*(reference[i]-attitude_data.euler_angle[i]);//Ko=2响应有点慢
			rate_error = u_o[i] - attitude_data.angle_rate[i];
		}
		else
			rate_error = 4*reference[i]-attitude_data.angle_rate[i];//偏航轴速率差
		
		//内环：PI
		p_term_in = K*rate_error;//这块需要测试一下量级并和MWC比较一下 K取1得到50的量级
		
		
		i_term_in[i] += rate_error*dT*Ki;//0.013*100系数约1.2量级
		
		if(i_term_in[i] > output_limit)//抗积分饱和
			i_term_in[i] = output_limit;
		else if(i_term_in[i] < -output_limit)
			i_term_in[i] = -output_limit;
		else ;
			
		if(thr_command < 100)//油门量很低 积分不起作用
			i_term_in[i] = 0;
		
		
		if(i == 0)
		printf("-------------------------\n");
		printf("P term:%.2f\n", p_term_in);
		printf("I term:%.2f\n", i_term_in[i]);
		
		
		
		/* //暂时不管微分
		rate_er_delta = rate_error-last_error[i];
		d_term_in = (rate_er_delta+rate_er_delta1[i]+rate_er_delta2[i])*Kd;//重新测量时间并更新
		
		last_error[i] = rate_error;									//存储内环误差和误差偏差量 
		rate_er_delta2[i] = rate_er_delta1[i];
		rate_er_delta1[i] = rate_er_delta;
		*/
		
		output[i] = p_term_in+i_term_in[i];
		
		
		
		//更新状态
		//euler_angle_pre[i] = attitude_data.euler_angle[i];
		
		
	}

	
	/*外环输出量->四个电机映射mix_table*/
	//output[2] = Ky*reference[2];
	output[3] = reference[3];//当前没有定高功能

	//printf("Refer  Signal (----): %.2f%s%.2f%s%.2f%s%.2f%s",reference[0], "    ",reference[1], "    ",reference[2], "      ", reference[3], "\n"); 
	//printf("Output Signal (----): %d%s%d%s%d%s%d%s",output[0], "               ",output[1], "               ",output[2], "               ", output[3], "                    \n");
}




/**
 *
 * 名称：set_reference
 *
 * 描述：摇杆信号量转化为控制参考值
 *
 */
void set_reference(const u16* rc_data, float* reference)
{
	short deg_lim = 50;																		//u8 总限制范围: -50~50度
	float deg_per_signal = 2*(float)deg_lim/2000; 					//每单位信号量对应的角度
	u8 axis;
	int16_t tmp, tmp2;
	//u16 rc_commands[3];
	
	// 添加速率曲线映射
	//u8 conf_rcExpo8 = 65;																		
	//u8 conf_rcRate8 = 90;																		
		
	u16 lookupPitchRollRC[6] = {0, 34, 99, 178, 323, 500}; //舵量分5段设计控制点-MATLAB离线设计
		
	for (axis = 0; axis < 2; axis++)//副翼升降拟合控制点
	{
		tmp = rc_data[axis]-1000;
		
		if(tmp < 0)
			tmp = -tmp;
		if(tmp > 1000)
			tmp = 1000;
		tmp2 = tmp/200;//分成五部分
		
		reference[axis] = lookupPitchRollRC[tmp2]+(tmp-(tmp2)*200)*\
		(lookupPitchRollRC[tmp2+1]-lookupPitchRollRC[tmp2])/200;
		reference[axis] /= 10.0;
		
		if(rc_data[axis] < 1000)
			reference[axis] = -reference[axis];
	}
	
	reference[1] = -reference[1];														//抬头为正参考
	//reference[0] = (rc_commands[0]-1000)*deg_per_signal;    //1副翼2升降3油门4方向
	//reference[1] = -(rc_commands[1]-1000)*deg_per_signal;   //右打，上打为正，pitch角增大，拉杆舵量减小
	reference[2] = (rc_data[3]-1000)*deg_per_signal;        //磁罗盘没调通
	reference[3] = rc_data[2];        							    		//高度参考
	
}




