//
// Created by YanYuanbin on 22-10-5.
//

#ifndef GIMBAL_ASSIST_H
#define GIMBAL_ASSIST_H

#include "stm32f4xx.h"

extern void angle_Extend(float *result_Angle,float act_Angle);
extern int16_t encoder_Extend(int16_t const *act_Encoder,int16_t torque_Ratio);
extern float encoder_To_Angle(volatile int16_t const *encoder,float encoder_Max);
extern float Radian_Extend(float *act_Radian);

#endif //GIMBAL_ASSIST_H
