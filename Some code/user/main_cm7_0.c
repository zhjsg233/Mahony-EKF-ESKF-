/*********************************************************************************************************************
* 文件名称          main_cm7_0
* 公司名称          用户工程
* 编译平台          IAR 9.40.1
* 芯片平台          CYT4BB
*
* 功能说明          cm7_0 核心主程序
*                   - IMU660RB 上电初始化 + 姿态解算初始化(2s稳定 + 1s零偏/初始对准) + 惯导初始化
*                   - CANFD0 初始化, 500Hz 发送左行进轮电流指令(扭矩模式)
*                   - 从 CANFD0 接收轮子状态, 主循环 500Hz 轮速速度观测
*                   - 主循环用 Justfloat(逐飞上位机)经无线串口上传 滤波后速度/位置 + 速度观测
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "imu_ahrs.h"
#include "Justfloat.h"
#include "LQ_CANFD.h"
#include "balance_pid.h"
#include "Beep.h"
#include <math.h>

#define WHEEL_SPEED_TO_MPS   (18.912f)   // 电机速度 → m/s 转换系数(轮速除以该系数得 m/s)

// 空转(打滑)检测: 轮速加速度超过物理极限 → 本拍不喂速度观测(等效R=∞)
#define SLIP_ACCEL_GATE_MPS2   (20.0f)   // 轮速加速度门限(m/s²): 高于车体能达到的最大加速度, 低于空转加速度
#define SLIP_ACCEL_WIN         (5)       // 加速度计算窗口(拍数): 500Hz下5拍=10ms, 匹配CAN状态100Hz刷新
#define WHEEL_TRACK            (0.3f)    // 轮距(两轮中心距, m): 里程计角速度 = (右-左)/轮距
#define SLIP_YAW_RATE_DIFF_RAD (0.5f)    // 里程计角速度 vs IMU角速度 差值门限(rad/s): 超出视为打滑(轮子与地面不一致)
static float left_mps_ring[SLIP_ACCEL_WIN]  = {0};   // 左右轮速环形缓冲(空转检测差分用)
static float right_mps_ring[SLIP_ACCEL_WIN] = {0};
static uint8  slip_ring_idx = 0;

// 原始轮速分解到世界系(未做杠臂补偿, 显示用)
float raw_world_vx = 0.0f, raw_world_vy = 0.0f;     // v_fwd 按航向分解到世界系 vx/vy(m/s)

// 角速度对比(显示用): 里程计 vs IMU, 单位 rad/s(发送时转 °)
float g_yaw_rate_odom = 0.0f, g_yaw_rate_imu = 0.0f, g_yaw_rate_diff = 0.0f;

uint8 g_slip_detected = 0;      // 打滑标志: 1=检测到打滑(蜂鸣器鸣叫), 0=正常

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     Send_Wheel_Val
// 功能说明     发送左右行进轮电流指令: 扭矩/电流模式, iq = 串级PID输出
//              左轮 -balance_iq, 右轮 +balance_iq: 保证前进时右轮读数>左轮读数,
//              与速度观测公式 (right-left)/2 的符号约定一致
// 参数说明     void
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
static void Send_Wheel_Val(void)
{
    static uint16_t heartbeat = 0;
    heartbeat++;

    g_node0_tx.motor[0].id        = LEFT_WALK_MOTOR;          // 0x55 左行进轮
    g_node0_tx.motor[0].mode      = FOC_MODE_TORQUE_MODE;     // 扭矩/电流模式
    g_node0_tx.motor[0].iq        = 0;//-balance_iq;              // 左轮电流(与右轮反号, 使两轮同向前进)
    g_node0_tx.motor[0].heartbeat = heartbeat;

    g_node0_tx.motor[1].id        = RIGHT_WALK_MOTOR;         // 0x56 右行进轮
    g_node0_tx.motor[1].mode      = FOC_MODE_TORQUE_MODE;
    g_node0_tx.motor[1].iq        =  0;//balance_iq;              // 右轮电流
    g_node0_tx.motor[1].heartbeat = heartbeat;

    CANFD0_TxMotorCtrl(2, 0);                                 // 左右两个行进轮, 立即发送
}

// **************************** 主程序区 ****************************

int main(void)
{
    uint32 last_send_ms = 0;            // justfloat 发送节流(用ISR毫秒tick)
    uint32 last_ctrl_tick = 0;          // 控制帧节流(用 1kHz 采样计数)
    uint32 last_imu_tick_ms = 0;        // 上一拍姿态计算时刻(实际dt用)
    uint32 phase = 0;

    clock_init(SYSTEM_CLOCK_250M);                  // 时钟配置及系统初始化
    debug_init();                                   // 调试串口信息初始化(UART0, 用于速度打印)

    system_delay_ms(100);
    // 1) IMU660RB 初始化(自检失败则重试)
    while(imu660rb_init())
    {
        zf_log(0, "imu660rb init fail, retry");
        system_delay_ms(200);
    }

    system_delay_ms(1000);

    // 2) 姿态解算初始化: 上电零偏 + 加速度初始对准(阻塞约3s, 期间保持板子静止)
    imu_ahrs_init();

    // 3) 惯导初始化: 位置/速度清零
    imu_nav_init();

    // 4) 串级PID初始化(自平衡: 角度环+角速度环)
    balance_pid_init();

    // 5) CANFD0 初始化(控制行进轮)
    CANFD0_Init();

    // 6) Justfloat 初始化(无线串口 UART1, 上位机看角度)
    Justfloat_Init();

    // 6.5) 蜂鸣器初始化(打滑报警)
    Beep_init();
    Beep_Off();

    // 7) 实际dt测量用毫秒定时器(主循环1kHz块量真实间隔)
    timer_init(TC_TIME2_CH0, TIMER_MS);
    timer_start(TC_TIME2_CH0);

    // 8) 1ms PIT 中断 -> pit0_ch0_isr(): 中断做1ms准时SPI采样, 姿态+惯导重计算在主循环做
    pit_ms_init(PIT_CH0, 1);

    while(true)
    {
        // 1kHz 姿态解算 + 惯导(主循环做, ISR 只做1ms准时采样):
        // 用实际经过时间作 dt: 中断1ms采样保证采样准时, dt反映主循环真实间隔, 串口/调度拖累时也不少积分
        if(g_imu_tick_pending)
        {
            float gx, gy, gz, ax, ay, az;
            uint32 now_t;
            float dt;
            g_imu_tick_pending = 0;
            gx = imu_smp_gx;  gy = imu_smp_gy;  gz = imu_smp_gz;   // 取中断采样(1ms准时)
            ax = imu_smp_ax;  ay = imu_smp_ay;  az = imu_smp_az;
            now_t = timer_get(TC_TIME2_CH0);
            dt = (float)(now_t - last_imu_tick_ms) * 0.001f;   // 实际经过时间(秒)
            last_imu_tick_ms = now_t;
            if(dt < 0.0005f || dt > 0.02f) dt = 0.001f;        // 首拍/超长卡住保护
            imu_ahrs_update(gx, gy, gz, ax, ay, az, dt);   // 姿态+速度/位置更新(10态ESKF, 实际dt)
            imu_ahrs_get_euler();
            imu_ahrs_update_count++;
        }

        // 8) CAN bus-off 保护: CAN0 用 PSR 轮询检测; CAN1 用中断标志触发恢复
        CANFD0_PollBusOff();
        // if(g_can1_busoff_flag)
        // {
        //     g_can1_busoff_flag = 0;
        //     CANFD1_BusOffRecover();
        // }

        // 9) 500Hz: 串级PID(角度环100Hz + 角速度环500Hz)算电流 -> 发控制帧 + 500Hz 轮速速度观测
        if(imu_ahrs_update_count != last_ctrl_tick)
        {
            last_ctrl_tick = imu_ahrs_update_count;
            phase = imu_ahrs_update_count % 10;
            if(phase % 2 == 0)
            {
                balance_pid_update();               // 500Hz: 串级PID(角速度环), 内部按频率分频跑角度环
                Send_Wheel_Val();                   // 500Hz: tick 0,2,4,6,8, 发送PID输出电流

                // 前进速度观测按航向分解到世界系 vx/vy, 带杠臂补偿
                float left_mps  = g_node0_lm_status.speed / WHEEL_SPEED_TO_MPS;
                float right_mps = g_node0_rm_status.speed / WHEEL_SPEED_TO_MPS;
                float v_fwd = (right_mps - left_mps) / 2.0f;
                float yaw   = imu_yaw_deg * DEG2RAD;

                // 原始轮速分解到世界系(未杠臂补偿, 显示用): v_fwd 直接按航向投影
                raw_world_vx = v_fwd * cosf(yaw);
                raw_world_vy = v_fwd * sinf(yaw);

                // 杠臂补偿后的观测(显示用): 轮点观测补偿到 IMU 点, 每拍都更新(不随打滑门控冻结)
                // vA_x = vB_x + h*ωy → obs_vx = vx_meas - h*ωy;  obs_vy = vy_meas + h*ωx
                imu_nav_obs_vx = raw_world_vx - NAV_LEVER_ARM_Z * (imu_gyro_dps[1] * DEG2RAD);
                imu_nav_obs_vy = raw_world_vy + NAV_LEVER_ARM_Z * (imu_gyro_dps[0] * DEG2RAD);

                // 空转(打滑)检测: 10ms窗口轮速加速度超门限 → 本拍不喂滤波观测(轮子空转, 轮速≠地面速度)
                float aL = (left_mps  - left_mps_ring[slip_ring_idx])  * (500.0f / SLIP_ACCEL_WIN);
                float aR = (right_mps - right_mps_ring[slip_ring_idx]) * (500.0f / SLIP_ACCEL_WIN);
                left_mps_ring[slip_ring_idx]  = left_mps;
                right_mps_ring[slip_ring_idx] = right_mps;
                slip_ring_idx = (uint8)((slip_ring_idx + 1) % SLIP_ACCEL_WIN);

                // 角速度一致性: 里程计角速度(由左右轮速差/轮距) vs IMU yaw率 差值过大 → 轮子与地面运动不一致(打滑)
                // 注意符号: 直走时右轮读数=+v, 左轮读数=-v(符号相反), 真实 v_L = -left_mps
                // 差速: ω = (v_R - v_L)/L = (right - (-left))/L = (right + left)/L
                // 实测: 该式与 imu_gyro_dps[2] 相差一个负号(电机/编码器方向约定), 故取反使符号一致
                float yaw_rate_odom = -(right_mps + left_mps) / WHEEL_TRACK; // rad/s
                float yaw_rate_imu  = imu_gyro_dps[2] * DEG2RAD;              // rad/s
                float yaw_rate_diff = yaw_rate_odom - yaw_rate_imu;
                g_yaw_rate_odom  = yaw_rate_odom;   // 显示用(rad/s)
                g_yaw_rate_imu   = yaw_rate_imu;
                g_yaw_rate_diff  = yaw_rate_diff;

                if(fabsf(aL) <= SLIP_ACCEL_GATE_MPS2 && fabsf(aR) <= SLIP_ACCEL_GATE_MPS2 &&
                   fabsf(yaw_rate_diff) <= SLIP_YAW_RATE_DIFF_RAD)
                {
#if IMU_AHRS_MODE == 2
                    // 仅 ESKF 模式(2)才有速度/位置惯导: 互补滤波(0)/EKF(1)不推进惯导, 跳过速度观测
                    eskf_vel_update(raw_world_vx,        // vx 观测(世界系)
                                    raw_world_vy,        // vy 观测(世界系)
                                    imu_gyro_dps[0] * DEG2RAD,   // wx 滚转率(rad/s)
                                    imu_gyro_dps[1] * DEG2RAD);  // wy 俯仰率(rad/s)
#endif
                    if(g_slip_detected) { g_slip_detected = 0; /* Beep_Off(); */ }  // 打滑恢复
                }
                else
                {
                    if(!g_slip_detected) { g_slip_detected = 1; /* Beep_On(); */ }   // 打滑触发(暂不蜂鸣)
                }
            }
        }

        // 10) 100Hz 用 Justfloat 上传 三轴姿态角 到逐飞上位机(用ISR毫秒tick节流, 不用定时器)
        if(g_system_tick_ms - last_send_ms >= 10)
        {
            last_send_ms = g_system_tick_ms;
            Justfloat_send_float(imu_roll_deg);                   // 通道1: 横滚角 roll(°)
            Justfloat_send_float(imu_pitch_deg);                  // 通道2: 俯仰角 pitch(°)
            Justfloat_send_float(imu_yaw_deg);                    // 通道3: 航向角 yaw(°)
            Justfloat_Send();                                     // 帧结束标记(+∞)
        }
    }
}

// **************************** 主程序区 ****************************
