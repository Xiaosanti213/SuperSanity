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
 //static void euler_to_rotmatrix(const float* euler_delta, float rot_matrix[3][3]);
 //static void matrix_multiply(const float mat[3][3], const float* vec1, float* vec2); 
 static void matrix_multiply(const float mat1[3][3], float mat2[3][3]);
 void normalize_mat(float mat[3][3]);
 static void normalize(float*);
 static float fast_inv_sqrt(float); 
 //static float dot_product(const float*, const float*);
 static void cross_product(float* vec1, const float* vec2);
 //static void rodrigue_rotation_matrix(float* rot_axis, const float rot_angle, float rot_matrix[3][3]);
 static void rot_matrix_to_euler(float R[3][3], float* euler_angle);
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
	 //保证三个数据依次是XYZ轴
	 temp = sensors_data->gyro[0];
	 sensors_data->gyro[0] = sensors_data->gyro[1];
	 sensors_data->gyro[1] = -temp;//TODO:后面还要修改一下控制量 
	 
	 temp = -sensors_data->acc[0];
	 sensors_data->acc[0] = -sensors_data->acc[1];
	 sensors_data->acc[1] = temp;
	 
	 //检查有效性
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
	 //float euler_delta[3];						  							
	 static float rot_matrix[3][3] = {1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0};			//旋转矩阵2维数组，初始时刻水平静止
	 float dissym_mat[3][3] = {1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0};		  //角速度反对称矩阵+单位阵
	 //static float att_est[3] = {0,0,1};	  	          
	 //float att_gyro[3];
	 //float w_gyro2acc = 0.2;                 					
	 u8 i,j;
	 //float deltaT = 0.32;								  						
	 float vec_temp[3];																	//取出旋转矩阵中第三列，用于计算误差向量
	 float Kp = 0.01;																		//加计对旋转矩阵比例补偿
	 const float deg2rad = 0.017;
		 
	 // 初始化	 
	 sensors_data_direction_correct(sensors_data);      //传感器方向对正
   //printf_sensors_data_estimate(*sensors_data);     //读出正确取向传感器数据分析
	 /*for(; i < 3; i++)
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
	 */
	 
	 //printf("Euler   Angle (deg ): %.2f%s%.2f%s%.2f%s",attitude_data->euler_angle[0], "         ",attitude_data->euler_angle[1], "      ",attitude_data->euler_angle[2], "         \n");
	 //printf("Angle   Rate  (dps ): %.2f%s%.2f%s%.2f%s",attitude_data->angle_rate[0], "         ",attitude_data->angle_rate[1], "      ",attitude_data->angle_rate[2], "         \n");
	 
	 //加计矢量校正
   normalize(sensors_data->acc);											//k方向参考正交化
   for (i = 0; i < 3; i++)
	 {
		 vec_temp[i] = rot_matrix[i][2];									//取出第三列 
	 }
	 cross_product(vec_temp, sensors_data->acc);	      //计算误差矢量，并将结果返回到第一个矢量当中
   for(i = 0; i < 3; i++)
	 {
		 sensors_data->gyro[i] += Kp * vec_temp[i];
		 //printf("Complemented Angle Rate:\n%.2f\n",sensors_data->gyro[i]);//调试输出经过补偿的角度
	 }
	 
	 dissym_mat[0][1] = -sensors_data->gyro[2]*deg2rad;
	 dissym_mat[0][2] = sensors_data->gyro[1]*deg2rad;
	 dissym_mat[1][2] = -sensors_data->gyro[0]*deg2rad;
	 for (i = 1; i < 3; i++)
			for(j = 0; j < i; j++)
				dissym_mat[i][j] = -dissym_mat[j][i];					//计算角速度反对称矩阵
	 
	 matrix_multiply(dissym_mat, rot_matrix);						//更新姿态旋转矩阵
	 normalize_mat(rot_matrix);													//旋转矩阵列向量正交化并单位化

	 // 下面由旋转矩阵计算欧拉角和三轴角速度
	 rot_matrix_to_euler(rot_matrix, attitude_data->euler_angle);
	 for(i = 0; i < 3; i++)
	 {
	   attitude_data->angle_rate[i] = sensors_data->gyro[i]; 
	 }
	 
	 //printf_sensors_data_estimate(*sensors_data);
	 //printf("Estimated Euler Angle: %.2f%s%.2f%s%.2f%s", attitude_data->euler_angle[0], "   ", attitude_data->euler_angle[1], "   ", attitude_data->euler_angle[2], "   \n");
	 
	 return ;
 }
 
 

 
 
/**
 *
 * 名称：euler_to_rotmatrix
 *
 * 描述：欧拉角计算旋转矩阵
 *
 */
 /*
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
 */
 
 
 
 
 
/**
 *
 * 名称：matrix_multiply
 *
 * 描述：矩阵乘向量
 *
 */ 
 /*
 void matrix_multiply(const float mat[3][3], const float* vec1, float* vec2)//
{
	vec2[0] = mat[0][0]*vec1[0]+mat[0][1]*vec1[1]+mat[0][2]*vec1[2];
  vec2[1] = mat[1][0]*vec1[0]+mat[1][1]*vec1[1]+mat[1][2]*vec1[2];
	vec2[2] = mat[2][0]*vec1[0]+mat[2][1]*vec1[1]+mat[2][2]*vec1[2];
	return ;
}
 */
 
 
 
 
 
 
 
 /**
 *
 * 名称：matrix_multiply
 *
 * 描述：矩阵相乘，计算结果返回到第二个矩阵当中
 *
 */ 
 void matrix_multiply(const float mat1[3][3], float mat2[3][3])
 {
	 float mat_temp[3][3];
	 u8 i, j, k;
	 for(i = 0; i < 3; i++)
		 for(j = 0; j < 3; j++)	
			 for(k = 0; k < 3; k++)
			 {
					mat_temp[i][j] += mat1[i][k]*mat2[k][j];
			 }
		
		for(i = 0; i < 3; i++)
			for(j = 0; j < 3; j++)
			    mat2[i][j] = mat_temp[i][j];
 }
 
 
 
 
 
 /**
 *
 * 名称：normalize_mat
 *
 * 描述：矩阵三轴列向量单位正交化
 *
 */ 
 void normalize_mat(float mat[3][3])
 {
	 // 三轴矢量彼此正交化
	 float error = 0;
	 u8 i,j;
	 float vec_temp[3];
	 for(i = 0; i < 3; i++)
	 {
		 error += mat[i][1]*mat[i][2];
	 }
	 error /= 2;
	 
	 for(i = 0; i < 3; i++)
	 {
		 mat[i][0] -= mat[i][1]*error;
		 mat[i][1] -= mat[i][0]*error;
	 }
	 // 第三列由前两列叉积得到
	 mat[0][2] = mat[1][0]*mat[2][1]-mat[2][0]*mat[1][1];
	 mat[1][2] = mat[2][1]*mat[0][0]-mat[0][1]*mat[2][0];
	 mat[2][2] = mat[0][0]*mat[1][1]-mat[1][0]*mat[0][1];
	 
	 // 三轴矢量各自单位化	 
	 for(i = 0; i < 3; i++)
	 {
		 for(j = 0; j < 3; j++)
		 {
			 vec_temp[j] = mat[j][i];
		 }
		 normalize(vec_temp);//列向量单位化
		 for(j = 0; j < 3; j++)
		 {
			 mat[j][i] = vec_temp[j];
		 }
	 }
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
 /*
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
 */
 
 
 
 
 
/**
 *
 * 名称：dot_product
 *
 * 描述：两个向量点积
 *
 */ 
 /*
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
 */
 
 
 
/**
 *
 * 名称：cross_product
 *
 * 描述：两个向量叉乘 
 *
 */ 
 void cross_product(float* vec1, const float* vec2)
 {
	 float cp[3];
	 u8 i;
	 cp[0] = vec1[1]*vec2[2]-vec1[2]*vec2[1];
	 cp[1] = vec1[2]*vec2[0]-vec1[0]*vec2[2];
	 cp[2] = vec1[0]*vec2[1]-vec1[1]*vec2[0];
	 for(i = 0; i < 3; i++)
	 {
		 vec1[i] = cp[i];		// 叉积结果返回到第一个向量当中
	 }
	 return ;
 }
	 
 
 
 
 
 
 
/**
 *
 * 名称：rodrigue_rotation_matrix
 *
 * 描述：罗德里格旋转公式，给定旋转轴与旋转角，计算旋转矩阵
 *
 */
/* 
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
 */
 
 
 
 
 
 
 
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




 
 
 
 
