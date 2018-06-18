#ifndef _ATTITUDE_CONTROL
#define _ATTITUDE_CONTROL



#include "attitude_estimate.h"
#include <stm32f10x.h>




// 声明由舵量计算得到的参考信号可以被外部调用
extern float reference[4];


void set_reference(void);
void attitude_control(ad attitude_data, const float* reference, int16_t* output);



#endif
