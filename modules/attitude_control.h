#ifndef _ATTITUDE_CONTROL
#define _ATTITUDE_CONTROL



#include "attitude_estimate.h"
#include <stm32f10x.h>




// �����ɶ�������õ��Ĳο��źſ��Ա��ⲿ����
extern float reference[4];


void set_reference(void);
void attitude_control(ad attitude_data, const float* reference, int16_t* output);



#endif
