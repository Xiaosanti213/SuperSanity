/**
 *
 * @file attitude_estimate.c
 *
 * 加计和陀螺仪加权互补姿态解算
 *
 **/
 
 
 #include "attitude_estimate.h" 
 #include <stm32f10x.h>
 #include <math.h>
 #include <stdio.h>
 
 
 
 static void sensors_data_direction_correct(sd*);
 static void euler_to_rotmatrix(const float* euler_delta, float rot_matrix[3][3]);
 static void matrix_multiply(const float mat[3][3], const float* vec1, float* vec2); 
 static void normalize(float*);
 static float fast_inv_sqrt(float); 
 static float dot_product(const float*, const float*);
 static void cross_product(const float* vec1, const float* vec2, float* cp);
 static void rodrigue_rotation_matrix(float* rot_axis, const float rot_angle, float rot_matrix[3][3]);
 static float atan2_numerical(float y, float x);
 static float abs_c_float_version(float);

 
 
 
 
 
/**
 *
 * 名称：sensors_data_direction_correct
 *
 * 描述：传感器数据方向修正
 *
 */
 void sensors_data_direction_correct(sd* sensors_data)
 {
	 float temp;
	 // 保证三个数据依次是XYZ轴
	 temp = sensors_data->gyro[0];
	 sensors_data->gyro[0] = sensors_data->gyro[1];
	 sensors_data->gyro[1] = temp;
	 
	 temp = -sensors_data->acc[0];
	 sensors_data->acc[0] = -sensors_data->acc[1];
	 sensors_data->acc[1] = temp;
	 return ;
 }
 
 
 
 
 
/**
 *
 * 名称：attitude_estimate
 *
 * 描述：姿态解算
 *
 */
 void attitude_estimate(ad* attitude_data, sd* sensors_data)
 {
	 float euler_delta[3];						  								//三轴分别：roll pitch yaw
	 float rot_matrix[3][3];						  						  //旋转矩阵2维数组
	 static float att_est[3] = {0,0,1};	  	            //只初始化1次
	 float att_gyro[3];
	 float w_gyro2acc = 0.2;                 							  //陀螺仪相对加计比值
	 u8 i = 0;
	 float deltaT = 0.32;								  							//这个如果能通过计算运行循环时间解算就比较好了
	 
	 sensors_data_direction_correct(sensors_data);      //传感器方向对正
     printf_sensors_data_estimate(*sensors_data);       //读出正确取向传感器数据分析
	 for(; i < 3; i++)
	 {
	   euler_delta[i] = sensors_data->gyro[i]*deltaT;   //计算相比较上次解算的旋转欧拉角
	   attitude_data->angle_rate[i] = sensors_data->gyro[i]; 
	 }
	 euler_to_rotmatrix(euler_delta, rot_matrix);   		//欧拉角计算旋转矩阵
	 matrix_multiply(rot_matrix, att_est, att_gyro);		//由前一次估计结果迭代得到当前姿态矢量
	 normalize(sensors_data->acc);				           		//加计矢量正交化
	 

	 for (i = 0; i<3; i++)
	 {
	   att_est[i] = (w_gyro2acc*att_gyro[i]+sensors_data->acc[i])/(1+w_gyro2acc);
	 }
	 attitude_data->euler_angle[0] = atan2_numerical(att_est[1],sqrt(att_est[0]*att_est[0]+att_est[2]*att_est[2]));
	 attitude_data->euler_angle[1] = atan2_numerical(att_est[0], att_est[2]);	 
	 printf("Euler   Angle (deg ): %.2f%s%.2f%s%.2f%s",attitude_data->euler_angle[0], "         ",attitude_data->euler_angle[1], "      ",attitude_data->euler_angle[2], "         \n");
	 printf("Angle   Rate  (dps ): %.2f%s%.2f%s%.2f%s",attitude_data->angle_rate[0], "         ",attitude_data->angle_rate[1], "      ",attitude_data->angle_rate[2], "         \n");

	 return ;
 }
 
 

 
 
/**
 *
 * 名称：euler_to_rotmatrix
 *
 * 描述：欧拉角计算旋转矩阵
 *
 */
 void euler_to_rotmatrix(const float* euler_delta, float rot_matrix[3][3])
 {
	 float phi = DEG_TO_RAD * euler_delta[0]; //封装
	 float theta = DEG_TO_RAD * euler_delta[1];
	 float psi = DEG_TO_RAD * euler_delta[2];
	 
	 rot_matrix[0][0] = cos(theta)*cos(psi);
	 rot_matrix[0][1] = cos(theta)*sin(psi);
	 rot_matrix[0][2] = -sin(theta);
	 
	 rot_matrix[1][0] = sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi);
	 rot_matrix[1][1] = sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi);
	 rot_matrix[1][2] = sin(phi)*cos(theta);
	 
	 rot_matrix[2][0] = cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
	 rot_matrix[2][1] = cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
	 rot_matrix[2][2] = cos(phi)*cos(theta);
	 
	 return;
 }
 
 
 
 
 
 
/**
 *
 * 名称：matrix_multiply
 *
 * 描述：矩阵乘向量
 *
 */ 
 void matrix_multiply(const float mat[3][3], const float* vec1, float* vec2)//
{
	vec2[0] = mat[0][0]*vec1[0]+mat[0][1]*vec1[1]+mat[0][2]*vec1[2];
    vec2[1] = mat[1][0]*vec1[0]+mat[1][1]*vec1[1]+mat[1][2]*vec1[2];
	vec2[2] = mat[2][0]*vec1[0]+mat[2][1]*vec1[1]+mat[2][2]*vec1[2];
	return ;
}
 
 
 

 
 
/**
 *
 * 名称：normalize
 *
 * 描述：向量正交化
 *
 */ 
 void normalize(float* vec)
 {
	 float legth = 0;
	 u8 i; 
	 float inv_norm;
	 for(i = 0; i<3; i++)	 					 //默认三维向量
	 {
			legth = vec[i]*vec[i]+legth;
	 }
	 inv_norm = fast_inv_sqrt(legth);//计算得到模方根分之一
	 for(i = 0; i<3; i++)
	 {
			vec[i] = vec[i]*inv_norm;    
	 }
 }
 
 
 
 
 
 
/**
 *
 * 名称：fast_inv_sqrt
 *
 * 描述：卡马克快速开平方根1/sqrt(num)
 *
 */ 
 float fast_inv_sqrt(float number)
 {
    long i;
	float x2, y;
	const float threehalfs = 1.5f;

	x2 = number * 0.5f;
	y  = number;
	i  = * ( long * ) &y;                       // 按照long的存储方式取float类型
	i  = 0x5f3759df - ( i >> 1 );                 
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 一次迭代
	//	y  = y * ( threehalfs - ( x2 * y * y ) );   // 二次迭代
	return y;
 }
 
 
 
 
 
 
/**
 *
 * 名称：calculate_rot_matrix
 *
 * 描述：两个向量计算旋转矩阵
 *
 */ 
 void calculate_rot_matrix(float* vec1, float* vec2, float rot_matrix[3][3])
 {
	 float rot_axis[3];
	 float rot_angle;
	 normalize(vec1);
	 normalize(vec2);
	 rot_angle = acos(dot_product(vec1, vec2));
	 cross_product(vec1, vec2, rot_axis);//无需正交化，正交化在罗德里格旋转中完成
	 rodrigue_rotation_matrix(rot_axis, rot_angle, rot_matrix);
	 return ;										
 }
 
 
 
 
 
 
/**
 *
 * 名称：dot_product
 *
 * 描述：两个向量点积
 *
 */ 
 float dot_product(const float* vec1, const float* vec2)
 {
	 u8 i;
	 float dp = 0;
	 for (i = 0; i<3; i++)
	 {
			dp = vec1[i]*vec2[i]+dp;
	 }
	 return dp;
 }
 
 
 
 
/**
 *
 * 名称：cross_product
 *
 * 描述：两个向量叉乘 
 *
 */ 
 void cross_product(const float* vec1, const float* vec2, float* cp)
 {
	 cp[0] = vec1[1]*vec2[2]-vec1[2]*vec2[1];
	 cp[1] = vec1[2]*vec2[0]-vec1[0]*vec2[2];
	 cp[2] = vec1[0]*vec2[1]-vec1[1]*vec2[0];
	 return ;
 }
	 
 
 
 
 
 
 
/**
 *
 * 名称：rodrigue_rotation_matrix
 *
 * 描述：罗德里格旋转公式，给定旋转轴与旋转角，计算旋转矩阵
 *
 */ 
void rodrigue_rotation_matrix(float* rot_axis, const float rot_angle, float rot_matrix[3][3])
{	
  // 封装
	float w1 = rot_axis[0];
	float w2 = rot_axis[1];
	float w3 = rot_axis[2];
	float cos_theta = cos(rot_angle);
	float sin_theta = sin(rot_angle);
	
	normalize(rot_axis);
	// 根据公式计算旋转矩阵各个元素
	rot_matrix[0][0] = w1*w1*(1-cos_theta)+cos_theta;
	rot_matrix[0][1] = w1*w2*(1-cos_theta)-w3*sin_theta;
	rot_matrix[0][2] = w1*w3*(1-cos_theta)+w2*sin_theta;
	
	rot_matrix[1][0] = w2*w1*(1-cos_theta)+w3*sin_theta;
	rot_matrix[1][1] = w2*w2*(1-cos_theta)+cos_theta;
	rot_matrix[1][2] = w2*w3*(1-cos_theta)-w1*sin_theta;
	
	rot_matrix[2][0] = w1*w3*(1-cos_theta)-w2*sin_theta;
	rot_matrix[2][1] = w2*w3*(1-cos_theta)+w1*sin_theta;
	rot_matrix[2][2] = w3*w3*(1-cos_theta)+cos_theta;
	
	return ;
}
 
 
 
 
 
 
 
 
/**
 *
 * 名称：rot_matrix_to_euler
 *
 * 描述：旋转矩阵到欧拉角
 *
 */ 
void rot_matrix_to_euler(float R[3][3], float* euler_angle)
{
	
	float theta = atan2_numerical(-R[0][2], 1/fast_inv_sqrt(R[0][0]*R[0][0]+R[0][1]*R[0][1]));
	float psi = atan2_numerical(R[0][1], R[0][0]);
	float phi = atan2_numerical(R[1][2], R[2][2]);
	//当前单位是角度deg
	euler_angle[0] = phi;
	euler_angle[1] = theta;
	euler_angle[2] = psi;
	return ;
}








/**
 *
 * 名称：atan2
 *
 * 描述：按照象限计算反正切
 *
 */ 
float atan2_numerical(float y, float x)     // 防止重名
{
	float z = y/x;
	float a;
  if (abs_c_float_version(y) < abs_c_float_version(x))											// 45度线作为分隔线
  {
     a = 57.3 * z / (1.0f + 0.28f * z * z); // 先按照一四象限计算 
   if (x<0) 
   {
     if (y<0) a -= 180;
     else a += 180;
   }
  } 
  else
  {
   a = 90 - 57.3 * z / (z * z + 0.28f);
   if (y<0) a -= 180;
  }
  return a;																	// 返回角度为单位
}








/**
 *
 * 名称：abs_c_float_version
 *
 * 描述：计算浮点数类型绝对值
 *
 */ 
 float abs_c_float_version(float num)	//在c语言中abs输入输出都是整数，cpp中abs支持float类型
 {
	 if (num<0)
		return -num;
	else
		return num;
 }







/**
 *
 *  名称： printf_sensors_data_estimate
 *
 *  描述： 传感器数据输出，Matlab对接，测试估计情况
 *
 */
void printf_sensors_data_estimate(sd sdata)
{
  printf("MPU6050 Accel ( g  ): %.2f%s%.2f%s%.2f%s", sdata.acc[0], "   ", sdata.acc[1], "   ", sdata.acc[2], "   \n");
  printf("MPU6050 Gyro  (dps ): %.2f%s%.2f%s%.2f%s", sdata.gyro[0], "   ", sdata.gyro[1], "    ", sdata.gyro[2], "      \n");
}




 
 
 
 
