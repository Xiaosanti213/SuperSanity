/**
 *
 * @file attitude_control.c
 *
 * �ڻ�P���� �⻷PI����
 *
 **/



#include "attitude_estimate.h"
#include "attitude_control.h"
#include <stdio.h>
#include <math.h>


/**
 *
 * ���ƣ�attitude_control
 *
 * �������ڻ�P�����⻷PI����
 *
 */
void attitude_control(ad attitude_data, const float* reference, int16_t* output)
{
	float K = 20;															//�ڻ�������ܿ�����
	float Kp = 14.14;
	float tauI = 0.1414;
	//float T = 0.02;														//Ӧ��ʹ�õ�ǰ��ִ��ʱ��
	//float error_i;
	//static float euler_angle_pre[2] = {0,0};  
	float p_term;
	//float i_term;
	float p_term_in, d_term_in;
	static float i_term_in[2] = {0,0}; 
	float rate_error = 0;
	static float last_error[2] = {0,0};
	static float u_o_pre[2] = {0,0};
	float u_o[2];
	u8 i;
	const u16 output_limit=10000; 
	
	float dT = 0.065;
	
	//Roll Pitch��
	for (i=0; i<2; i++)
	{
		// ��ת������
		/**
		//�⻷ IP����
		p_term = Kp*(-attitude_data.euler_angle[i]+euler_angle_pre[i]);   //ǰһ�κ͵�ǰŷ��������
		i_term = Kp/tauI*(reference[i]-attitude_data.euler_angle[i])*T;   
		u_o[i] = p_term+i_term+u_o_pre[i];

		//�ڻ�
		error_i = u_o[i]-attitude_data.angle_rate[i];
		output[i] = K*error_i; 	
		**/

		//MWC�Ŀ���ģʽ
		//�⻷��P
		u_o[i] = Kp*(reference[i]-attitude_data.euler_angle[i]);
		
		//�ڻ���PID
		rate_error = u_o[i] - attitude_data.angle_rate[i];
		
		p_term_in = K*rate_error;
		i_term_in[i] += rate_error*dT*K/tauI;
		d_term_in = (rate_error-last_error[i])*15.38;//1/dT
		last_error[i] = rate_error; 
		output[i] = rate_error;
		
		//����״̬
		//euler_angle_pre[i] = attitude_data.euler_angle[i];
		if(output[i] > output_limit)
			output[i] = output_limit;
		else if(output[i] < -output_limit)
			output[i] = -output_limit;
		else
			u_o_pre[i] = u_o[i];//�����������ƣ��򲻸��¿������
	}

	
	/*�⻷�����->�ĸ����ӳ��mix_table*/
	output[2] = K*reference[2]/2;
	output[3] = reference[3];//��ǰû�ж��߹���

	//printf("Refer  Signal (----): %.2f%s%.2f%s%.2f%s%.2f%s",reference[0], "    ",reference[1], "    ",reference[2], "      ", reference[3], "\n"); 
	//printf("Output Signal (----): %d%s%d%s%d%s%d%s",output[0], "               ",output[1], "               ",output[2], "               ", output[3], "                    \n");
}




/**
 *
 * ���ƣ�set_reference
 *
 * ������ҡ���ź���ת��Ϊ���Ʋο�ֵ
 *
 */
void set_reference(const u16* rc_commands, float* reference)
{
	short deg_lim = 100;																		//u8 �����Ʒ�Χ: -50~50��
	float deg_per_signal = 2*(float)deg_lim/2000; 					//ÿ��λ�ź�����Ӧ�ĽǶ�

	reference[0] = (rc_commands[0]-1000)*deg_per_signal;    //1����2����3����4����
	reference[1] = -(rc_commands[1]-1000)*deg_per_signal;   //�Ҵ��ϴ�Ϊ����pitch���������˶�����С
	reference[2] = (rc_commands[3]-1000)*deg_per_signal;    //������û��ͨ
	reference[3] = rc_commands[2];        									//�߶Ȳο�
}




