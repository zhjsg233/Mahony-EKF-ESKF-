/*********************************************************************************************************************
* 文件名称          balance_pid
* 公司名称          用户工程
* 功能说明          串级PID(自平衡): 外环=俯仰角度环, 内环=俯仰角速度环, 内环输出电流发给左右行进轮
*                   复用 project/code/Pid.c 的 pid_param_t + PidLocCtrl(位置式PID)
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "Pid.h"
#include "imu_ahrs.h"
#include "balance_pid.h"

static pid_param_t angle_pid;   // 外环 角度环: 角度误差(°) -> 目标角速度(°/s)
static pid_param_t rate_pid;    // 内环 角速度环: 角速度误差(°/s) -> 电流(A)

float balance_iq           = 0.0f;   // 当前发给轮子的电流(A)
float balance_target_rate  = 0.0f;   // 目标俯仰角速度(°/s)
float balance_target_angle = 0.0f;   // 目标俯仰角(°)

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     balance_pid_init
// 功能说明     串级PID初始化: 角度环(外环) + 角速度环(内环), 输出清零
// 参数说明     void
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void balance_pid_init(void)
{
    PidInit(&angle_pid, BALANCE_ANGLE_KP, 0.0f, BALANCE_ANGLE_KD, BALANCE_ANGLE_IMAX,
            -BALANCE_ANGLE_MAX, BALANCE_ANGLE_MAX);   // 角度环 PD(无积分), 输出=目标角速度(°/s)
    PidInit(&rate_pid,  BALANCE_RATE_KP, BALANCE_RATE_KI, BALANCE_RATE_KD, BALANCE_RATE_IMAX,
            -BALANCE_IQ_MAX, BALANCE_IQ_MAX);         // 角速度环纯P(当前KI=KD=0), 输出=电流(A)
    balance_iq           = 0.0f;
    balance_target_rate  = 0.0f;
    balance_target_angle = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     balance_pid_update
// 功能说明     串级PID主更新, 500Hz 调用(与 500Hz 控制帧同一位置, 即 tick 0,2,4,6,8)
//              角度环(外环, 100Hz, PD): 目标俯仰角 - 实际俯仰角 -> 目标俯仰角速度
//              角速度环(内环, 500Hz): 目标俯仰角速度 - 实际俯仰率 -> 电流
// 参数说明     void
// 返回参数     void
// 使用示例     balance_pid_update();   // 主循环500Hz控制帧处调用, 随后 Send_Wheel_Val() 发送电流
//-------------------------------------------------------------------------------------------------------------------
void balance_pid_update(void)
{
    static uint8 angle_div = 0;
    float target_rate;

#if BALANCE_ANGLE_LOOP_ENABLE
    // 外环角度环: 按频率分频(500/100 = 每 5 个内环周期跑一次)
    if(++angle_div >= (BALANCE_RATE_LOOP_HZ / BALANCE_ANGLE_LOOP_HZ))
    {
        angle_div = 0;
        target_rate = PidLocCtrl(&angle_pid, balance_target_angle, imu_pitch_deg);
        balance_target_rate = target_rate;              // 实时更新目标角速度(供调参/显示)
    }
    else
    {
        target_rate = balance_target_rate;
    }
#else
    // 内环独立调参模式: 目标角速度直接用 balance_target_rate(先调角速度环)
    target_rate = balance_target_rate;
#endif

    // 内环角速度环(500Hz): 目标俯仰率 - 实际俯仰率 -> 电流
    balance_iq = PidLocCtrl(&rate_pid, target_rate, imu_gyro_dps[1]);

    // 方向修正(接线/安装方向不对时翻转)
    balance_iq *= BALANCE_DIR;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     balance_pid_get_iq
// 功能说明     获取当前 PID 输出的电流(A)
// 参数说明     void
// 返回参数     float 电流(A)
// 使用示例     float iq = balance_pid_get_iq();
//-------------------------------------------------------------------------------------------------------------------
float balance_pid_get_iq(void)
{
    return balance_iq;
}
