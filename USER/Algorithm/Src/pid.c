//
// Created by YanYuanbin on 22-10-11.
//
#include "pid.h"


/**************************PID param Init*********************************/
static void f_PID_param_init(
    PID_TypeDef_t *pid,
    float deadband,
    float maxIntegral,
    float max_out,
    float kp,
    float Ki,
    float Kd)
{
    pid->param.Deadband = deadband;
    pid->param.maxIntegral = maxIntegral;
    pid->param.MaxOut = max_out;

    pid->param.kp = kp;
    pid->param.Ki = Ki;
    pid->param.Kd = Kd;

    pid->ERRORHandler.ERRORCount = 0;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;
	
		pid->Pout = 0;
		pid->Iout = 0;
		pid->Dout = 0;
    pid->Output = 0;
}
/**************************PID param reset*********************************/
static void f_PID_reset(PID_TypeDef_t *pid, float Kp, float Ki, float Kd)
{
    pid->param.kp = Kp;
    pid->param.Ki = Ki;
    pid->param.Kd = Kd;

    if (pid->param.Ki == 0)
        pid->Iout = 0;
}

/***************************PID calculate**********************************/
float f_PID_Calculate(PID_TypeDef_t *pid, float err)
{
    if (pid == NULL)
    {
				return 0; 
    }

		pid->Last_Err = pid->Err;
		if(ABS(err) < pid->param.Deadband) err = 0;
    pid->Err = err;
		
		pid->Integral += pid->Err;
		VAL_Limit(pid->Integral,-pid->param.maxIntegral,pid->param.maxIntegral);
		
		pid->Pout = pid->param.kp * pid->Err;
		pid->Iout = pid->param.Ki * pid->Integral;
		pid->Dout = pid->param.Kd * (pid->Err - pid->Last_Err);
		
		pid->Output = pid->Pout + pid->Iout + pid->Dout;
		VAL_Limit(pid->Output,-pid->param.MaxOut,pid->param.MaxOut);

    return pid->Output;
}

void PID_Init(
    PID_TypeDef_t *pid,
    float deadband,
    float maxIntegral,
    float max_out,
    float kp,
    float Ki,
    float Kd)
{
    pid->PID_reset = f_PID_reset;
    pid->PID_param_init = f_PID_param_init;
    pid->PID_param_init(pid, deadband,maxIntegral, max_out, 
                        kp, Ki, Kd);
}

float f_PID_Delta_Calc(struct _PID_Delta *pid,float err)
{
	if(pid == NULL)
	{
		return 0;
	}
	
	pid->Err[2] = pid->Err[1];
	pid->Err[1] = pid->Err[0];
	pid->Err[0] = err;
	
	pid->Pout = pid->kp * (pid->Err[0] - pid->Err[1]);
	pid->Iout = pid->ki * pid->Err[0];
	pid->Dout = pid->kd * (pid->Err[0] - 2.f*pid->Err[1] + pid->Err[2]);
	
	pid->Dout = pid->Pout + pid->Iout + pid->Dout;
	VAL_Limit(pid->Dout,-pid->MaxOut,pid->MaxOut);
	
	return pid->Dout;
}

void PID_Delta_init(
	struct _PID_Delta *pid,
	float maxOut,
	float kp,
	float ki,
	float kd)
{
	pid->MaxOut = maxOut;
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}

