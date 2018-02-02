#ifndef _ATTITUDE_CONTROL
#define _ATTITUDE_CONTROL



#include "attitude_estimate.h"
#include <stm32f10x.h>








void set_reference(const u16* rc_commands, float* reference);
void attitude_control(ad attitude_data, const float* reference, int16_t* output);



#endif
