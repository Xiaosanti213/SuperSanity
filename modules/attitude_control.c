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
void attitude_control(ad attitude_data, const float* reference, int16_t* output)
{
	float K = 20;															//内环大增益很快收敛
	float Kp = 14.14;
	float tauI = 0.1414;
	float T = 0.02;														//应该使用当前的执行时间
	float error_i;
	static float euler_angle_pre[2] = {0,0};  
	float p_term, i_term;
	static float u_o_pre[2] = {0,0};
	float u_o[2];
	u8 i;
	const u16 output_limit=10000; 
	
	//Roll Pitch轴
	for (i=0; i<2; i++)
	{
		//外环
		p_term = Kp*(-attitude_data.euler_angle[i]+euler_angle_pre[i]);
		i_term = Kp/tauI*(reference[i]-attitude_data.euler_angle[i])*T;
		u_o[i] = p_term+i_term+u_o_pre[i];

		//内环
		error_i = u_o[i]-attitude_data.angle_rate[i];
		output[i] = K*error_i; 	
		
		//更新状态
		euler_angle_pre[i] = attitude_data.euler_angle[i];
		if(output[i] > output_limit)
			output[i] = output_limit;
		else if(output[i] < -output_limit)
			output[i] = -output_limit;
		else
			u_o_pre[i] = u_o[i];//超出控制限制，则不更新控制输出
	}

	
	/*外环输出量->四个电机映射mix_table*/
	error_i = reference[2]-attitude_data.angle_rate[2];
	output[2] = K*error_i;
	output[3] = reference[3];//当前没有定高功能

	printf("Refer  Signal (----): %.2f%s%.2f%s%.2f%s%.2f%s",reference[0], "    ",reference[1], "    ",reference[2], "      ", reference[3], "\n"); 
	printf("Output Signal (----): %d%s%d%s%d%s%d%s",output[0], "               ",output[1], "               ",output[2], "               ", output[3], "                    \n");
}







/**
 *
 * 名称：set_reference
 *
 * 描述：摇杆信号量转化为控制参考值
 *
 */
void set_reference(const u16* rc_commands, float* reference)
{
	short deg_lim = 40;																		  //u8 总限制范围: -20~20度
	float deg_per_signal = 2*(float)deg_lim/2000; 					//每单位信号量对应的角度

	reference[0] = (rc_commands[0]-1000)*deg_per_signal;    //1副翼2升降3油门4方向
	reference[1] = -(rc_commands[1]-1000)*deg_per_signal;   //右打，上打为正，pitch角增大，拉杆舵量减小
	reference[2] = (rc_commands[3]-1000)*deg_per_signal;    //磁罗盘没调通
	reference[3] = rc_commands[2];        									//高度参考
}




