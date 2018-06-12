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
void attitude_control(ad attitude_data, float* reference, int16_t* output, u16 thr_command)
{
	float K = 1;						  						  			//�ڻ�������ܿ�����
	float Ko = 2;						  				    				//�⻷����
	float Ki = 60;
	//float Kd = 10;															//����Kd/(3*dT)����1/dTԼ100
	//float Ky = 30;															//ƫ������
	//float T = 0.02;														//Ӧ��ʹ�õ�ǰ��ִ��ʱ��
	//float error_i;
	//static float euler_angle_pre[2] = {0,0};  
	//float p_term;
	//float i_term;
	float p_term_in = 0;
	//float d_term_in = 0;
	static float i_term_in[3] = {0,0,0}; 
	
	float rate_error = 0;												//�ڻ����
	//static float last_error[2] = {0,0};

	/*
	static float rate_er_delta = 0;			 				//�ڻ����仯��
	static float rate_er_delta1[2] = {0,0};
	static float rate_er_delta2[2] = {0,0};
	*/
	
	//static float u_o_pre[2] = {0,0};
	float u_o[2];																//�⻷rp�����
	u8 i;
	const u16 output_limit=500; 
	
	//float angle_trim[2] = {0.9, -0.1};          //У׼ң����������
	
	float dT = 0.02;														//��λ��s
	
	
	
	//Roll Pitch Yaw��
	for (i=0; i<3; i++)
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
		if(i < 2)
		{
			//reference�����޶���[-25,25]��Χ��
			if(reference[i]>50)
				reference[i] = 50;
			else if(reference[i]<-50)
				reference[i] = -50;

			u_o[i] = Ko*(reference[i]-attitude_data.euler_angle[i]);//Ko=2��Ӧ�е���
			rate_error = u_o[i] - attitude_data.angle_rate[i];
		}
		else
			rate_error = 4*reference[i]-attitude_data.angle_rate[i];//ƫ�������ʲ�
		
		//�ڻ���PI
		p_term_in = K*rate_error;//�����Ҫ����һ����������MWC�Ƚ�һ�� Kȡ1�õ�50������
		
		
		i_term_in[i] += rate_error*dT*Ki;//0.013*100ϵ��Լ1.2����
		
		if(i_term_in[i] > output_limit)//�����ֱ���
			i_term_in[i] = output_limit;
		else if(i_term_in[i] < -output_limit)
			i_term_in[i] = -output_limit;
		else ;
			
		if(thr_command < 100)//�������ܵ� ���ֲ�������
			i_term_in[i] = 0;
		
		
		if(i == 0)
		printf("-------------------------\n");
		printf("P term:%.2f\n", p_term_in);
		printf("I term:%.2f\n", i_term_in[i]);
		
		
		
		/* //��ʱ����΢��
		rate_er_delta = rate_error-last_error[i];
		d_term_in = (rate_er_delta+rate_er_delta1[i]+rate_er_delta2[i])*Kd;//���²���ʱ�䲢����
		
		last_error[i] = rate_error;									//�洢�ڻ��������ƫ���� 
		rate_er_delta2[i] = rate_er_delta1[i];
		rate_er_delta1[i] = rate_er_delta;
		*/
		
		output[i] = p_term_in+i_term_in[i];
		
		
		
		//����״̬
		//euler_angle_pre[i] = attitude_data.euler_angle[i];
		
		
	}

	
	/*�⻷�����->�ĸ����ӳ��mix_table*/
	//output[2] = Ky*reference[2];
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
void set_reference(const u16* rc_data, float* reference)
{
	short deg_lim = 50;																		//u8 �����Ʒ�Χ: -50~50��
	float deg_per_signal = 2*(float)deg_lim/2000; 					//ÿ��λ�ź�����Ӧ�ĽǶ�
	u8 axis;
	int16_t tmp, tmp2;
	//u16 rc_commands[3];
	
	// �����������ӳ��
	//u8 conf_rcExpo8 = 65;																		
	//u8 conf_rcRate8 = 90;																		
		
	u16 lookupPitchRollRC[6] = {0, 34, 99, 178, 323, 500}; //������5����ƿ��Ƶ�-MATLAB�������
		
	for (axis = 0; axis < 2; axis++)//����������Ͽ��Ƶ�
	{
		tmp = rc_data[axis]-1000;
		
		if(tmp < 0)
			tmp = -tmp;
		if(tmp > 1000)
			tmp = 1000;
		tmp2 = tmp/200;//�ֳ��岿��
		
		reference[axis] = lookupPitchRollRC[tmp2]+(tmp-(tmp2)*200)*\
		(lookupPitchRollRC[tmp2+1]-lookupPitchRollRC[tmp2])/200;
		reference[axis] /= 10.0;
		
		if(rc_data[axis] < 1000)
			reference[axis] = -reference[axis];
	}
	
	reference[1] = -reference[1];														//̧ͷΪ���ο�
	//reference[0] = (rc_commands[0]-1000)*deg_per_signal;    //1����2����3����4����
	//reference[1] = -(rc_commands[1]-1000)*deg_per_signal;   //�Ҵ��ϴ�Ϊ����pitch���������˶�����С
	reference[2] = (rc_data[3]-1000)*deg_per_signal;        //������û��ͨ
	reference[3] = rc_data[2];        							    		//�߶Ȳο�
	
}




