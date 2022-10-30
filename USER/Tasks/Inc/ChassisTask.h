//
// Created by YanYuanbin on 22-10-12.
//

#ifndef _CHASSIS_H
#define _CHASSIS_H

#include "stdint.h"
#include "stdbool.h"

#define M_R  	4.8f
#define M_L  	30.f
#define M_LW 	0.f

#define CHANGE_TO_ANGLE 57.295779f




typedef struct
{
	float value;
	float output_last;
	float output;
}Lowpass_Filter_t;



extern float imu_pitgyro;

#endif

