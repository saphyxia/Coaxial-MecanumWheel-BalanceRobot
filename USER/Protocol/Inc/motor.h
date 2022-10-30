//
// Created by YanYuanbin on 22-10-5.
//

#ifndef GIMBAL_MOTOR_H
#define GIMBAL_MOTOR_H

#include "pid.h"
#include "mycan.h"

typedef enum{
		Right_LSpiral,
		Right_RSpiral,
		Left_LSpiral,
		Left_RSpiral,
    MOTOR_NUM,
}MOTOR_USAGE;

typedef enum{
    _6020,
    _3508,
    _2006,
    MOTOR_TYPE_NUM,
}MOTOR_TYPE;

/*  大疆电机封装，继承CAN信息结构体，
    注意：一定要给函数指针初始化！！！否则会导致程序死在溢出报错。
*/
typedef struct
{
    struct{
        uint32_t StdId;
        volatile float angle;
        volatile int16_t current;
        volatile int16_t velocity;
        volatile int16_t encoder;
        volatile uint8_t temperature;
    }Data;

    CAN_TypeDef *CANx;
    MOTOR_TYPE type;
    MOTOR_USAGE usage;
}DJI_MOTOR;

extern DJI_MOTOR Balance[MOTOR_NUM];


extern void get_Motor_Data(uint32_t *canId, uint8_t *rxBuf,DJI_MOTOR* DJI_Motor);

#endif //GIMBAL_MOTOR_H
