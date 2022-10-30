//
// Created by YanYuanbin on 22-10-5.
//

#include "assist.h"
#include "pid.h"

#define PI 3.1415926f

/**
  * @name   encoder_To_Angle
  * @brief  角度转换
  * @param
  *         encoder:编码器角度        encoder_Max:编码器最大值（量程）
  * @retval result_Angle:周角角度
  * @attention
*/
float encoder_To_Angle(volatile int16_t const *encoder,float encoder_Max)
{
    return (float)*encoder / encoder_Max * 360.0f;
}

/**
  * @name   encoder_Extend
  * @brief  编码器码盘拓展，
  * @param
  *         result_Encoder:拓展后的编码器（结果）      act_Encoder:实际编码器的值（参数1）      torque_Ratio:拓展比例
  * @retval none
  * @attention
  *         如果需要上电时编码器值归零，把起始angle_Last赋值为debug看到的数即可。
*/
int16_t encoder_Extend(int16_t const *act_Encoder,int16_t torque_Ratio)
{
    static int16_t angle_Last = 0;
    static int16_t result_Encoder = 0;
    int16_t res1 = 0,res2 = 0;

    if(*act_Encoder < angle_Last){
        res1 = *act_Encoder - angle_Last + 8192;
        res2 = *act_Encoder - angle_Last;
    }else{
        res1 = *act_Encoder - angle_Last - 8192;
        res2 = *act_Encoder - angle_Last;
    }
    angle_Last = *act_Encoder;
    ABS(res1)<ABS(res2) ? (result_Encoder += res1) : (result_Encoder += res2);
    if(ABS(result_Encoder) > 8192*torque_Ratio) result_Encoder = 0;

    return result_Encoder;
}

/**
  * @name   angle_Extend
  * @brief  角度拓展
  * @param
  *         act_Angle:实际角度，通常为陀螺仪返回得到，也可以是用编码器换算。
  * @retval result_Angle:无穷累加的结果（周角角度）
  * @attention
  *         针对正负180临界点类型进行处理
*/
void angle_Extend(float *result_Angle,float act_Angle)
{
    static float angle_Last = 0.f;
    float res1 = 0.f,res2 = 0.f;

    if(act_Angle < angle_Last){
        res1 = act_Angle - angle_Last + 360.0f;
        res2 = act_Angle - angle_Last;
    }else{
        res1 = act_Angle - angle_Last - 360.0f;
        res2 = act_Angle - angle_Last;
    }
    angle_Last = act_Angle;

    if(ABS(res1)<0.01f || ABS(res2)<0.01f)res1 = res2 = 0.f;
    ABS(res1) < ABS(res2) ? (*result_Angle += res1) : (*result_Angle += res2);
}

float Radian_Extend(float *act_Radian)
{
    static float Radian_Last = 0.f;
    static float result_Radian = 0.f;
    float res = 0.f;

    res = *act_Radian - Radian_Last;
    if(res <= -PI)
        res += 2.0f*PI;
    if(res >= PI)
        res -= 2.0f*PI;
    Radian_Last = *act_Radian;
    result_Radian += res;

    return result_Radian;
}


