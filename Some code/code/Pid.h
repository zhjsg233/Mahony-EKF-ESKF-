/*
 * pid.h
 *
 *  Created on: 2024年2月27日
 *      Author: sunqing
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_

typedef struct
{
    float                kp;         //P
    float                ki;         //I
    float                kd;         //D

    float                imax;       //积分限幅
		float                min;
    float                max;       //输出限幅

    float                out_p;  //KP输出
    float                out_i;  //KI输出
    float                out_d;  //KD输出
    float                out;    //pid输出

    float                derivator;  //微分值
    float                integrator; //< 积分值
    float                last_error; //< 上次误差
}pid_param_t;
// 对外子程序
void PidInit(pid_param_t * pid, float kp, float ki, float kd, float imax, float min, float max);
float PidLocCtrl(pid_param_t * pid, float SetValue, float ActualValue);
//增量式PID
float PidIncCtrl(pid_param_t * pid, float SetValue, float ActualValue);
float PidLocCtrlExtDeriv(pid_param_t * pid, float SetValue, float ActualValue, float ExternalDeriv);
//变量声明



#endif /* CODE_PID_H_ */
