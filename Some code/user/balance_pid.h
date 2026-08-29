/*********************************************************************************************************************
* 文件名称          balance_pid
* 公司名称          用户工程
* 功能说明          串级PID(自平衡): 外环=俯仰角度环, 内环=俯仰角速度环, 内环输出电流发给左右行进轮
*                   调参顺序:
*                     1) 先断开外环(ANGLE_LOOP_ENABLE=0), 设 balance_target_rate 给内环目标角速度, 调角速度环
*                     2) 角速度环跟手好之后, 再开外环(ANGLE_LOOP_ENABLE=1)调角度环
*                   约定: pitch 抬头为正(°), 俯仰率 imu_gyro_dps[1] 同号(°/s)
********************************************************************************************************************/

#ifndef _balance_pid_h_
#define _balance_pid_h_

#include "zf_common_typedef.h"

//====================================================================串级PID配置====================================================================
#define BALANCE_ANGLE_LOOP_ENABLE   (1)     // 1=启用角度外环(自平衡); 0=只跑角速度内环(先调内环用)
#define BALANCE_RATE_LOOP_HZ        (500)   // 角速度环频率(内环, 与500Hz控制帧同步)
#define BALANCE_ANGLE_LOOP_HZ       (100)   // 角度环频率(外环, 比内环慢)

// 外环 角度环: 输入 角度误差(°) -> 输出 目标角速度(°/s), PD 控制器
#define BALANCE_ANGLE_KP            (15.0f) // P: 1° 误差 -> 15°/s 目标角速度
#define BALANCE_ANGLE_KD            (10.0f) // D: 对角度误差差分(≈ -俯仰率阻尼), 抑制角度环振荡
#define BALANCE_ANGLE_IMAX          (200.0f)
#define BALANCE_ANGLE_MAX           (1000.0f)// 角度环输出的目标角速度限幅(°/s)

// 内环 角速度环: 输入 角速度误差(°/s) -> 输出 电流(A), 纯 P 控制(KI=KD=0)
#define BALANCE_RATE_KP             (0.2f) // 起步值: 100°/s 误差 -> 2A(会被限幅到20A)
#define BALANCE_RATE_KI             (0.00f)
#define BALANCE_RATE_KD             (0.00f)
#define BALANCE_RATE_IMAX           (50.0f) // (纯P时无效, 保留备用)
#define BALANCE_IQ_MAX              (20.0f)  // 电流输出限幅(A)

#define BALANCE_DIR                 (1)     // 方向修正: 若车向前倾却向后冲, 改为 -1
//====================================================================串级PID配置====================================================================

extern float balance_iq;                    // 当前发给轮子的电流(A)
extern float balance_target_rate;           // 目标俯仰角速度(°/s), 内环调参时直接改它(角度环关闭时生效)
extern float balance_target_angle;          // 目标俯仰角(°), 默认 0(水平)

void balance_pid_init(void);                // 上电初始化(清零PID状态/输出)
void balance_pid_update(void);              // 500Hz 调用(与 500Hz 控制帧同一位置), 内部按频率分频跑角度环
float balance_pid_get_iq(void);             // 获取当前电流(A)

#endif
