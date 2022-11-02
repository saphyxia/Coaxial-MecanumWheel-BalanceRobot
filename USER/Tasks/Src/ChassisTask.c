//
// Created by YanYuanbin on 22-10-12.
//
#include "cmsis_os.h"

#include "ChassisTask.h"
#include "InsTask.h"

#include "tim.h"

#include "motor.h"
#include "myusart.h"

#include "pid.h"
#include "kalman.h"

bool IF_START_RUNNING = false;

bool IF_GOAL_SWITCH = false;
uint8_t switch_count =0;

Actline_t Goalline;

/*
 * 出发	(-50,1000)
 * 隔板	(-750,2000) (-750,3000)
 * 取球	
 * 放球
*/
ActPoint_t midTurn[3]=
{
	[0]={.x=-50  ,.y=1000},
	[1]={.x=-750 ,.y=2000},
	[2]={.x=-750 ,.y=3000},
};
ActPoint_t Ballpoint[2][5]=
{
	[0]={
		[0]={.x=75  ,.y=4300},
		[1]={.x=-50 ,.y=4000},
		[2]={.x=-50 ,.y=4000},
		[3]={.x=-50 ,.y=4000},
		[4]={.x=-50 ,.y=4000},
	},
	[1]={
		[0]={.x=-0 ,.y=0},
		[1]={.x=-0 ,.y=0},
		[2]={.x=-0 ,.y=0},
		[3]={.x=-0 ,.y=0},
		[4]={.x=-0 ,.y=0},
	},
};

//PID
PID_TypeDef_t Upright,Speed,Gyro,LLspiral,LRspiral,RLspiral,RRspiral,Posture_X,Posture_Y,Turn;

//一阶低通
Lowpass_Filter_t LLspeed,LRspeed,RLspeed,RRspeed,Pit_gyro;

uint16_t PWM_OUT[5]={0};
float midangle = -4.5f;

float LLout,LRout,RLout,RRout,Upout,speedout,trunout,speed = 0.f,gyro = 0;

//全向移动解算
float Omni[4]={0.f,};
float target[3];

//kalman
extKalman_t Gyro_Kalman;

/* Private function prototypes -----------------------------------------------------*/
static float f_FirstOrder_Lowpass_Filter(Lowpass_Filter_t *std,float value,float k);
static float f_CcltAngleAdd(float angle1, float angle2);
static float f_Posture_Solution(Actline_t *real,Actline_t *goal);
static float f_AngleErr_Solution(float realAngle,float goalAngle);
static void Omni_Motion_Solution(float *result,float *target);

/* USER CODE BEGIN Header_ChassisTask */
/**
* @brief Function implementing the ChasTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ChassisTask */
void ChassisTask(void const * argument)
{
	osDelay(5000);
  /* USER CODE BEGIN ChassisTask */
	TickType_t systick = 0;
	
	PID_Init(&Speed,    0.f, 0.f,   8.f,     0.001f, 0.f,   0.f); //p:0.001
	PID_Init(&Posture_X,0.f, 600.f, 1000.f,  0.f,    0.f,   0.f); // 
	PID_Init(&Posture_Y,0.f, 800.f, 1000.f,  0.f,    0.f,   0.f); //p:8 
	PID_Init(&Turn,     0.f, 500.f, 1000.f,  0.f,    0.f,   0.f); //p:100
	PID_Init(&Upright,  0.f, 0.f,   1200.f,  14.f,   0.f,   0.f); //p:14 
	PID_Init(&Gyro,     0.f, 2000.f,10000.f, 18.f,   0.6f,  200.f); //p:18 i:0.6 d:200
	PID_Init(&LLspiral, 0.f, 2000.f,10000.f, 13.2f,  0.2f,  0.f);
	PID_Init(&LRspiral, 0.f, 2000.f,10000.f, 13.2f,  0.2f,  0.f);
	PID_Init(&RLspiral, 0.f, 2000.f,10000.f, 13.2f,  0.2f,  0.f);
	PID_Init(&RRspiral, 0.f, 2000.f,10000.f, 13.2f,  0.2f,  0.f);
	
	KalmanCreate(&Gyro_Kalman,1,40);

  /* Infinite loop */	
  for(;;)
  {
		systick = osKernelSysTick();
		
//		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,PWM_OUT[0]);
//		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,PWM_OUT[1]);
//		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,PWM_OUT[2]);
//		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,PWM_OUT[3]);
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,PWM_OUT[4]);

		speed = ( - f_FirstOrder_Lowpass_Filter(&LLspeed,Balance[Left_LSpiral].Data.velocity,0.9f) \
							+ f_FirstOrder_Lowpass_Filter(&LRspeed,Balance[Left_RSpiral].Data.velocity,0.9f) \
							- f_FirstOrder_Lowpass_Filter(&RRspeed,Balance[Right_RSpiral].Data.velocity,0.9f) \
							+ f_FirstOrder_Lowpass_Filter(&RLspeed,Balance[Right_LSpiral].Data.velocity,0.9f) )/4.f;
		
		if(IF_GOAL_SWITCH)
		{
			Goalline.point = midTurn[switch_count];
			switch_count++;
			if(switch_count > 2)switch_count=2;
			IF_GOAL_SWITCH = 0;
		}
		
		if(IF_START_RUNNING)
		{
			//路程环
			target[1] = f_PID_Calculate(&Posture_Y,f_Posture_Solution(&Presentline,&Goalline));
		}
		
		//转向环
		trunout  = f_PID_Calculate(&Turn,f_AngleErr_Solution(Presentline.angle,Goalline.angle));
		//速度环
		speedout = f_PID_Calculate(&Speed,speed - target[1]);//p
		//直立环
		Upout    = f_PID_Calculate(&Upright,speedout + midangle - Imu.pit_angle);//pd

//		Imu.pit_gyro = f_FirstOrder_Lowpass_Filter(&Pit_gyro,Imu.pit_gyro,0.9f);
		Imu.pit_gyro = KalmanFilter(&Gyro_Kalman,Imu.pit_gyro);
		
		//角速度环
		gyro = f_PID_Calculate(&Gyro,Upout - Imu.pit_gyro);//pid

		Omni_Motion_Solution(Omni,target);

		//注意极性与整车速度解算一致
		LLout = f_PID_Calculate(&LLspiral,-gyro+trunout+Omni[1] - Balance[Left_LSpiral].Data.velocity);
		LRout = f_PID_Calculate(&LRspiral, gyro-trunout+Omni[0] - Balance[Left_RSpiral].Data.velocity);
		RRout = f_PID_Calculate(&RRspiral,-gyro-trunout+Omni[3] - Balance[Right_RSpiral].Data.velocity);
		RLout = f_PID_Calculate(&RLspiral, gyro+trunout+Omni[2] - Balance[Right_LSpiral].Data.velocity);
		
//		myprintf(Imu.pit_gyro,gyro);
		
		//倒地判断
		if(ABS(Imu.pit_angle) > 30){
			LLout = f_PID_Calculate(&LLspiral,0 - Balance[Left_LSpiral].Data.velocity);
			LRout = f_PID_Calculate(&LRspiral,0 - Balance[Left_RSpiral].Data.velocity);
			RRout = f_PID_Calculate(&RRspiral,0 - Balance[Right_RSpiral].Data.velocity);
			RLout = f_PID_Calculate(&RLspiral,0 - Balance[Right_LSpiral].Data.velocity);
		}

		hcanTxFrame.data[0] = (int16_t)RLout >> 8;
		hcanTxFrame.data[1] = (int16_t)RLout;
		hcanTxFrame.data[2] = (int16_t)RRout >> 8;
		hcanTxFrame.data[3] = (int16_t)RRout;
		hcanTxFrame.data[4] = (int16_t)LLout >> 8;
		hcanTxFrame.data[5] = (int16_t)LLout;
		hcanTxFrame.data[6] = (int16_t)LRout >> 8;
		hcanTxFrame.data[7] = (int16_t)LRout;
		
		USER_CAN_TxMessage(&hcan1,&hcanTxFrame);
		
    osDelayUntil(&systick,1);
  }
  /* USER CODE END ChassisTask */
}

static float f_FirstOrder_Lowpass_Filter(Lowpass_Filter_t *std,float value,float k)
{

	std->output_last = std->output;
	std->value = value;

	std->output = k*std->value + (1.0f-k)*std->output_last;
	
	return std->output;
}

/**
* @name 	f_CcltAngleAdd
* @brief	对-180,180交界处作处理
* @param	angle1:角度1;angle2:角度2;
* @retval
*/
static float f_CcltAngleAdd(float angle1, float angle2)
{
	float result = 0.0f;

	result = angle1 - angle2;

	if (result >  180.0f)  result -= 360.0f;
	else if (result < -180.0f)  result += 360.0f;
	return result;
}


/**
* @name 	f_CcltLineAngle
* @brief	计算两点直线方向角度
* @param	pointStart:起始点；pointEnd:终止点
* @retval
*/
static float f_CcltLineAngle(ActPoint_t pointStart, ActPoint_t pointEnd)
{
	int16_t a = 0;
	int16_t b = 0;
	
	a = pointEnd.y - pointStart.y;
	b = pointEnd.x - pointStart.x;
	//atan2f范围可以包含-180到180  
	return (atan2f(a, b) * CHANGE_TO_ANGLE); //CHANGE_TO_ANGLE:360/(2*pi)，弧度转角度
}
/**
* @name 	f_Posture_Solution
* @brief	定位轮坐标解算
* @param	real:当前点  goal:目标点
* @retval
*/
static float f_Posture_Solution(Actline_t *real,Actline_t *goal)
{
		float result=0;
		int8_t Forward_Switch = 0;

		ActPoint_t midpoint;
	
		midpoint.x =  goal->point.x;
		midpoint.y = (goal->point.y + real->point.y)*0.5f;
	
		if(goal->point.y < real->point.y)
		{
			midpoint.x = 2*real->point.x - midpoint.x;
			midpoint.y = 2*real->point.y - midpoint.y;
			Forward_Switch = -1;
		}
		else
		{
			Forward_Switch = 1;
		}
		
		goal->angle = f_CcltLineAngle(real->point,midpoint);
		
		result = sqrtf(powf((real->point.x-goal->point.x),2.f)+powf((real->point.y-goal->point.y),2.f)) *Forward_Switch;
		
		if(ABS(result)<50.f)
		{
			result = 0.f;
			goal->angle =90.f;
		}
	
		return result;
}

/**
* @name 	f_AngleErr_Solution
* @brief	偏航角误差解算
* @param	realAngle:当前角  goalAngle:目标角
* @retval
*/
static float f_AngleErr_Solution(float realAngle,float goalAngle)
{
	float angleErr =0;
	
	if(f_CcltAngleAdd(goalAngle,realAngle)>=0.f)
	{
		angleErr = sqrtf(f_CcltAngleAdd(goalAngle,realAngle));
	}else
	{
		angleErr = -sqrtf(-f_CcltAngleAdd(goalAngle,realAngle));
	}
	
	return angleErr;
}

static void Omni_Motion_Solution(float *result,float *target)
{
	//全向移动解算
	result[0] = 1.0f / M_R * ( target[0] + target[1] + (M_L + M_LW)/ 2.f * target[2]);
	result[1] = 1.0f / M_R * ( target[0] - target[1] + (M_L + M_LW)/ 2.f * target[2]);
	result[2] = 1.0f / M_R * (-target[0] + target[1] + (M_L + M_LW)/ 2.f * target[2]);
	result[3] = 1.0f / M_R * (-target[0] - target[1] + (M_L + M_LW)/ 2.f * target[2]);
}