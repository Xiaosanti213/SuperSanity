/**
 *
 * @file attitude_control.c
 *
 * �ڻ�P���� �⻷PI����
 *
 **/



#include "attitude_estimate.h"
#include "attitude_control.h"




/**
 *
 * ���ƣ�attitude_control
 *
 * �������ڻ�P�����⻷PI����
 *
 */
void attitude_control(const ad attitude_data, const float* reference, u16* output)
{
	float K = 40;											//�ڻ�������ܿ�����
	float Kp = 14.14;
	float tauI = 0.1414;
	float T = 0.2;										//Ӧ��ʹ�õ�ǰ��ִ��ʱ��
	float error_o[4] = {0,0,0,0};			//�⻷ƫ��ֵ���������ݼ�¼
	float error_i[4] = {0,0,0,0};
	float u_o; 												//�⻷�������Ϊ�ڻ��ο�
	u8 i = 0;
	float p_term;
	float i_term;
	
	/*�Ƕȣ����ٶ�����->���⻷�����*/
	for(i = 0; i<2; i++)			//��ת ������
	{
		error_o[i] = reference[i]-attitude_data.euler_angle[i];
		p_term = -Kp*attitude_data.euler_angle[i];
		i_term = i_term+Kp/tauI*error_o[i]*T;
		u_o = p_term+i_term;
		error_i[i] = u_o - attitude_data.angle_rate[i];
		output[i] = -K*error_i[i];
	}
	/*�⻷�����->�ĸ����ӳ��mix_table*/
	if(i == 2)
	{
		error_i[i] = reference[i]-attitude_data.angle_rate[i];
		output[i] = -K*error_i[i];
	}	
	else if(i == 3)
	{
		output[i] = reference[i];//��ǰû�ж��߹���
	}
	//���Ӧ�ü��Ͽ����ֱ���
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
	short deg_lim = 20;																		  //u8 �����Ʒ�Χ: -20~20��
	float deg_per_signal = 2*deg_lim/2000; 									//ÿ��λ�ź�����Ӧ�ĽǶ�
	
	reference[0] = (rc_commands[0]-1000)*deg_per_signal;    //1����2����3����4����
	reference[1] = (rc_commands[1]-1000)*deg_per_signal;
	reference[2] = (rc_commands[3]-1000)*deg_per_signal;    //������û��ͨ
	reference[3] = rc_commands[2];        									//�߶Ȳο�
}




