/*
 * pid.c
 *
 *  Created on: 2024年2月27日
 *      Author: sunqing
 */
#include "Pid.h"
#include "zf_common_headfile.h"

void PidInit(pid_param_t * pid, float kp, float ki, float kd, float imax, float min, float max)
{
    pid->kp        = kp;
    pid->ki        = ki;
    pid->kd        = kd;

    pid->imax      = imax;
		pid->min       = min;
    pid->max       = max;

    pid->out_p     = 0;
    pid->out_i     = 0;
    pid->out_d     = 0;
    pid->out       = 0;
	
    pid->integrator= 0;
    pid->last_error= 0;
    pid->derivator = 0;
}

/*!
  * @brief    pid位置式控制器输出
  *
  * @param    pid     pid参数
  * @param    error   pid输入误差
  *
  * @return   PID输出结果
  *
  */
float PidLocCtrl(pid_param_t * pid, float SetValue, float ActualValue)
{
    float error=0;

    error=SetValue - ActualValue;

    /* 累积误差 */
    pid->integrator += error;
    /* 误差限幅 */
    pid->integrator = func_limit(pid->integrator, pid->imax); //对累积误差的限幅
    /* 微分项计算 */
    pid->derivator = error - pid->last_error;

    pid->out_p = pid->kp * error;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_d = pid->kd * pid->derivator;

    pid->last_error = error;

    pid->out = pid->out_p + pid->out_i + pid->out_d;

    pid->out = func_limit_ab(pid->out, pid->min, pid->max);

    return pid->out;
}

float PidLocCtrlExtDeriv(pid_param_t * pid, float SetValue, float ActualValue, float ExternalDeriv)
{
    float error=0;

    error=SetValue - ActualValue;

    pid->integrator += error;
    pid->integrator = func_limit(pid->integrator, pid->imax);

    pid->derivator = ExternalDeriv;

    pid->out_p = pid->kp * error;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_d = pid->kd * pid->derivator;

    pid->last_error = error;

    pid->out = pid->out_p + pid->out_i + pid->out_d;

    pid->out = func_limit_ab(pid->out, pid->min, pid->max);

    return pid->out;
}


/*!
  * @brief    pid增量式控制器输出  //增量式Pid就是求Pid下一步的输出变化值，它只需要最近三次的误差值，所以可以更符合系统实时的状态
  *
  * @param    pid     pid参数
  * @param    error   pid输入误差
  *
  * @return   PID输出结果   输出结果已经是加上增量的结果了
  *
  */
float PidIncCtrl(pid_param_t * pid, float SetValue, float ActualValue) //第一个形参决定参数，第二个形参决定输入误差，第三个决定期望
{
    float error=0;

    error=SetValue - ActualValue;

    pid->out_p = pid->kp * (error - pid->last_error);
    pid->out_i = pid->ki * error;
    pid->out_d = pid->kd * ((error - pid->last_error) - pid->derivator);

    pid->derivator = error - pid->last_error;
    pid->last_error = error;

    pid->out += pid->out_p + pid->out_i + pid->out_d;

    pid->out = func_limit_ab(pid->out, pid->min, pid->max);

    return pid->out;
}