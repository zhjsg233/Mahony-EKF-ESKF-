/*********************************************************************************************************************
* 文件名称          imu_ahrs
* 公司名称          用户工程
* 编译平台          IAR 9.40.1
* 芯片平台          CYT4BB
*
* 功能说明          IMU660RB 姿态解算实现(按 IMU_AHRS_MODE 宏选择算法)
*                   - 0 = 曼哈尼(Mahony)互补滤波; 1 = 四元数EKF;
*                   - 2 = ESKF(误差状态卡尔曼, 10态: 姿态+零漂+二维速度/位置, 角速度积分+加速度/轮速观测)
*                   - 均为 6 轴(加速度计 + 陀螺仪, 无磁力计), 航向 yaw 为陀螺积分, 会缓慢漂移
*                   - 1kHz 采样: PIT 1ms 中断做 imu_ahrs_read(SPI采样), 主循环调 imu_ahrs_update(解算)
*                   - 上电静止约1s: 取陀螺均值作零偏, 取加速度均值作初始 roll/pitch
*                   - 安装方向修正: 原始数据经 AXIS_ACC_/AXIS_GYR_* 转换到机体系(NED)后再参与解算
*                   - 注: 1kHz 传感器采样, 1ms 中断调用 imu_ahrs_read -> imu_ahrs_update, 100Hz 上传数据
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "imu_ahrs.h"
#include <math.h>

//------------------------------------------------------配置------------------------------------------------------
#define MAHONY_KP           (0.004f)                 // 互补滤波比例增益(0 = 关闭重力修正, 纯角速度积分测试; 恢复用0.1)
#define MAHONY_KI           (0.001f)                 // 互补滤波积分增益(0 = 关闭重力修正; 恢复用0.001)
#define INTEGRAL_CLAMP      (0.3f)                   // 互补滤波积分反馈饱和保护(rad/s)
#define BIAS_SAMPLES        (1000)                   // 上电标定样本数, 1ms 间隔采样约 1 秒

#define EKF_Q_SCALE         (1e-6f)                  // EKF 过程噪声(四元数), 越大越信任陀螺/动态响应越快
#define EKF_R_SCALE         (100.0f)                 // EKF 量测噪声(加速度, g²), 越大越不信任加速度(已放大10万倍)
#define EKF_P0_SCALE        (0.01f)                  // EKF 初始协方差

#define ACCEL_TRUST_G       (0.2f)                   // 互补滤波冲击保护: |a|-1g 超过该值降低加速度修正权重
#define EKF_R_IMPACT_K      (100.0f)                 // EKF 冲击保护: R_eff = R*(1+K*(|a|-1)^2)

#define ESKF_Q_SCALE        (1e-6f)                  // 旋转误差状态过程噪声(rad²), 越大越信任陀螺
#define ESKF_Q_BIAS         (1e-8f)                  // 零漂随机游走过程噪声((rad/s)²), 越大零漂在线估计越快越活
#define ESKF_P0_SCALE       (0.01f)                  // 初始旋转误差协方差(rad²)
#define ESKF_P0_BIAS        (1e-8f)                  // 初始零漂误差协方差((rad/s)²)
#define ESKF_R_BASE         (1000.0f)                // 加速度观测基础R(g²): 门限内信任加速度的程度(σ=31.6g, 很不信任)
#define ESKF_ACCEL_TRUST_G  (0.3f)                   // |a|门限: 偏离1g超过该值 → 比力含线性加速度, 本拍不用加速度观测
#define ESKF_GYRO_TRUST_DPS (150.0f)                 // 角速度门限: 任一轴超过该值 → 旋转剧烈, 加速度方向受离心/切向污染(|a|可能仍≈1g), 本拍不用
#define ESKF_Q_VEL          (1e-7f)                  // 速度误差状态过程噪声((m/s)²): ≈ 加速度白噪声方差·dt²(0.1·1e-6)
#define ESKF_Q_POS          (0.0f)                   // 位置误差状态过程噪声(m²): 置0, 极小值(≈qa·dt⁴/4)float加到大P上会丢失; 位置方差经 δv→δp 二次积分自然增长
#define ESKF_P0_VEL         (1.0f)                   // 初始速度误差协方差((m/s)², σ=1m/s)
#define ESKF_P0_POS         (1.0f)                   // 初始位置误差协方差(m², σ=1m)
#define ESKF_R_VEL          (1e-4f)                  // 速度观测R((m/s)², σ=1cm/s), 固定不随新息自适应(见 nav_r_vel 说明)
#define ESKF_VEL_INNOV_GATE_MPS (10.0f)              // 速度观测新息门限(m/s): |z-h(x_pred)| 超过该值视为离群(打滑/跳变), 拒绝本次观测
#define ESKF_R_POS          (1e-2f)                  // 位置观测R(m², σ=10cm), 位置观测接口预留

#define CALIB_SETTLE_MS     (2000)                   // 上电后先等待机体稳定再开始零偏标定(毫秒)

//------------------------------------------------------惯导(10态ESKF: 速度/位置)配置------------------------------------------------------
#define NAV_G_MPS2          (9.80665f)               // 重力加速度(m/s²)
// 速度观测噪声方差 R: 固定 ESKF_R_VEL(轮速 1cm/s 精度), 不随新息自适应
//------------------------------------------------------惯导(10态ESKF: 速度/位置)配置------------------------------------------------------
//------------------------------------------------------配置------------------------------------------------------

//------------------------------------------------------模块状态------------------------------------------------------
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;   // 机体->NED 四元数 (w,x,y,z)
static float ex_int = 0.0f, ey_int = 0.0f, ez_int = 0.0f;  // 积分反馈误差
static float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;  // 陀螺零偏(rad/s, 机体系)
static float P[4][4];                                                       // EKF 协方差矩阵(4x4)
static float P_eskf[10][10];                                                // ESKF 误差状态协方差(10x10): [δθ(3), δb(3), δv(2), δp(2)]
static float b_eskf[3] = {0.0f, 0.0f, 0.0f};                                // ESKF 在线估计的残余陀螺零偏(rad/s)
float imu_eskf_bias[3] = {0.0f, 0.0f, 0.0f};                                // 显示用: 在线零偏(rad/s)
float eskf_r_eff = 0.0f;                                                    // ESKF 有效量测R(g², 显示用)

float imu_nav_x = 0.0f, imu_nav_y = 0.0f, imu_nav_vx = 0.0f, imu_nav_vy = 0.0f;  // ESKF 名义状态: 世界系水平位置/速度(由 ESKF 写入, 复用原惯导全局量)
float nav_r_vel = ESKF_R_VEL;                                               // 速度观测噪声方差(固定, σ=1cm/s)
float imu_nav_obs_vx = 0.0f, imu_nav_obs_vy = 0.0f;                        // 杠臂补偿后的观测速度(m/s, 显示用)

float imu_roll_deg = 0.0f, imu_pitch_deg = 0.0f, imu_yaw_deg = 0.0f;
float imu_quat[4]  = {1.0f, 0.0f, 0.0f, 0.0f};
float imu_acc_g[3]  = {0.0f, 0.0f, 0.0f};
float imu_gyro_dps[3] = {0.0f, 0.0f, 0.0f};
volatile uint32 imu_ahrs_update_count = 0;
//------------------------------------------------------模块状态------------------------------------------------------

static void ekf_init(void);          // 前置声明: imu_ahrs_init 中需调用 EKF 协方差初始化
static void eskf_init(void);         // 前置声明: imu_ahrs_init 中需调用 ESKF 协方差初始化
static void eskf_inject(const float de[3], const float db[3],   // 前置声明: 加速度/速度/位置观测统一调用误差注入
                        const float dv[2], const float dp[2]);

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     imu_ahrs_read
// 功能说明     读取 IMU 原始数据(驱动自带函数)并转换到机体坐标(NED)下的物理量:
//              1) 原始陀螺计读数 -> 机体系角速度(rad/s), 并按 AXIS_*_DIR 做安装方向修正
//              2) 原始加速度计数 -> 机体系加速度(g), 并按 AXIS_*_DIR 做安装方向修正
//              同时更新 imu_gyro_dps[3](°/s) 与 imu_acc_g[3](g) 供外部查看
// 参数说明     gx/gy/gz   输出: 机体系角速度(rad/s)
//              ax/ay/az   输出: 机体系加速度(g)
// 返回参数     void
// 使用示例     imu_ahrs_read(&gx, &gy, &gz, &ax, &ay, &az);
//-------------------------------------------------------------------------------------------------------------------
void imu_ahrs_read(float *gx, float *gy, float *gz,
                   float *ax, float *ay, float *az)
{
    float dgx, dgy, dgz;

    imu660rb_get_gyro();                                     // 读取陀螺原始数据(驱动自带函数)
    imu660rb_get_acc();                                      // 读取加速度原始数据

    // 陀螺: 原始(°/s) -> 机体系(°/s 与 rad/s), 标准右手系无需取反
    dgx = AXIS_GYR_X_DIR * imu660rb_gyro_transition(imu660rb_gyro_x);
    dgy = AXIS_GYR_Y_DIR * imu660rb_gyro_transition(imu660rb_gyro_y);
    dgz = AXIS_GYR_Z_DIR * imu660rb_gyro_transition(imu660rb_gyro_z);
    *gx = dgx * DEG2RAD;
    *gy = dgy * DEG2RAD;
    *gz = dgz * DEG2RAD;
    imu_gyro_dps[0] = dgx;  imu_gyro_dps[1] = dgy;  imu_gyro_dps[2] = dgz;

    // 加速度: 比力(原始) -> 重力方向(滤波器需要), 三轴取反
    *ax = AXIS_ACC_X_DIR * imu660rb_acc_transition(imu660rb_acc_x);
    *ay = AXIS_ACC_Y_DIR * imu660rb_acc_transition(imu660rb_acc_y);
    *az = AXIS_ACC_Z_DIR * imu660rb_acc_transition(imu660rb_acc_z);
    imu_acc_g[0] = *ax;  imu_acc_g[1] = *ay;  imu_acc_g[2] = *az;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     imu_ahrs_init
// 功能说明     上电初始化(阻塞约3s, 期间必须保持板子静止):
//              1) 先等待 CALIB_SETTLE_MS(默认2s)让机体稳定, 再进行标定
//              2) 1ms 间隔采样约1s, 同一次循环取机体系陀螺均值(零偏)和机体系加速度均值
//              3) 由重力方向求初始 roll/pitch(yaw=0), 转成四元数作为滤波初值
// 参数说明     void
// 返回参数     void
// 使用示例     imu_ahrs_init();
//-------------------------------------------------------------------------------------------------------------------
void imu_ahrs_init(void)
{
    float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;
    float sum_ax = 0.0f, sum_ay = 0.0f, sum_az = 0.0f;
    float gx, gy, gz, ax, ay, az;
    float roll0, pitch0, cr, sr, cp, sp;
    uint32 i;

    // 1) 等待机体稳定(此时先别动板子), 再开始零偏标定
    system_delay_ms(CALIB_SETTLE_MS);

    // 2) 上电标定: 静止约1秒取均值(已做安装方向修正, 直接累加机体系物理量)
    for(i = 0; i < BIAS_SAMPLES; i ++)
    {
        imu_ahrs_read(&gx, &gy, &gz, &ax, &ay, &az);
        sum_gx += gx;  sum_gy += gy;  sum_gz += gz;          // rad/s
        sum_ax += ax;  sum_ay += ay;  sum_az += az;          // g
        system_delay_ms(1);
    }
    gyro_bias_x = sum_gx / BIAS_SAMPLES;                     // rad/s
    gyro_bias_y = sum_gy / BIAS_SAMPLES;
    gyro_bias_z = sum_gz / BIAS_SAMPLES;

    // 2) 由重力方向求初始 roll/pitch(yaw=0), ZYX 欧拉角转四元数
    ax = sum_ax / BIAS_SAMPLES;
    ay = sum_ay / BIAS_SAMPLES;
    az = sum_az / BIAS_SAMPLES;
    roll0  = atan2f(ay, az);                                 // rad, 右侧下压为正
    pitch0 = atan2f(-ax, sqrtf(ay * ay + az * az));          // rad, 抬头为正

    cr = cosf(roll0  * 0.5f);
    sr = sinf(roll0  * 0.5f);
    cp = cosf(pitch0 * 0.5f);
    sp = sinf(pitch0 * 0.5f);
    q0 = cr * cp;                                            // 初始 yaw = 0 的 ZYX 合成
    q1 = sr * cp;
    q2 = sp * cr;
    q3 = -sp * sr;
    imu_quat[0] = q0;  imu_quat[1] = q1;  imu_quat[2] = q2;  imu_quat[3] = q3;

    ex_int = ey_int = ez_int = 0.0f;
    ekf_init();                                              // EKF 协方差初始化
    eskf_init();                                             // ESKF 协方差初始化
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     mahony_update(内部)
// 功能说明     曼哈尼(Mahony)6轴互补滤波单步更新(NED, 无磁力计)
//              误差取 实测a × 估计v(经典 x-io MahonyAHRS 符号, 收敛方向正确)
//              加速度只修正 roll/pitch; yaw 由陀螺积分, 残差零偏会使其缓慢漂移
// 参数说明     gx/gy/gz   陀螺仪角速度(rad/s, 机体系), 内部自动减去上电标定的零偏
//              ax/ay/az   加速度(g, 机体系)
//              dt         采样间隔(秒), 1kHz 固定 0.001f
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
static void mahony_update(float gx, float gy, float gz,
                          float ax, float ay, float az, float dt)
{
    const float twoKp = 2.0f * MAHONY_KP;
    const float twoKi = 2.0f * MAHONY_KI;
    float norm, vx, vy, vz, ex, ey, ez;
    float halfT = 0.5f * dt;

    // 减去陀螺零偏(rad/s)
    gx -= gyro_bias_x;
    gy -= gyro_bias_y;
    gz -= gyro_bias_z;

    // 归一化加速度; 若模长异常(≈0)则跳过加速度修正, 仅做陀螺积分, 避免姿态被锁死
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if(norm >= 1e-6f)
    {
        ax /= norm;  ay /= norm;  az /= norm;

        // 冲击保护: |a| 偏离 1g 越多, 加速度修正权重越低(冲击/颠簸时的线性加速度不能当倾斜修正用)
        float trust = 1.0f - fabsf(norm - 1.0f) / ACCEL_TRUST_G;
        if(trust < 0.0f) trust = 0.0f;
        if(trust > 1.0f) trust = 1.0f;

        // 估计重力方向在机体系中的分量 v (NED, Z 下)
        vx = 2.0f * (q1 * q3 - q0 * q2);
        vy = 2.0f * (q0 * q1 + q2 * q3);
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        // 误差 = 实测 a × 估计 v
        ex =  ay * vz - az * vy;
        ey =  az * vx - ax * vz;
        ez =  ax * vy - ay * vx;

        // 积分反馈(按可信度累积, 带饱和保护)
        ex_int += twoKi * ex * dt * trust;
        ey_int += twoKi * ey * dt * trust;
        ez_int += twoKi * ez * dt * trust;
        if(ex_int >  INTEGRAL_CLAMP) ex_int =  INTEGRAL_CLAMP;
        if(ex_int < -INTEGRAL_CLAMP) ex_int = -INTEGRAL_CLAMP;
        if(ey_int >  INTEGRAL_CLAMP) ey_int =  INTEGRAL_CLAMP;
        if(ey_int < -INTEGRAL_CLAMP) ey_int = -INTEGRAL_CLAMP;
        if(ez_int >  INTEGRAL_CLAMP) ez_int =  INTEGRAL_CLAMP;
        if(ez_int < -INTEGRAL_CLAMP) ez_int = -INTEGRAL_CLAMP;

        // 比例项按可信度缩放(积分项本身已按可信度累积), 反馈到陀螺
        gx += twoKp * ex * trust + ex_int;
        gy += twoKp * ey * trust + ey_int;
        gz += twoKp * ez * trust + ez_int;
    }

    // 一阶四元数积分: q += 0.5*dt * q ⊗ (0, gx, gy, gz)(无论加速度是否有效都执行)
    q0 += (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 += ( q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 += ( q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 += ( q0 * gz + q1 * gy - q2 * gx) * halfT;

    // 四元数归一化
    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if(norm > 1e-8f)
    {
        q0 /= norm;  q1 /= norm;  q2 /= norm;  q3 /= norm;
    }
    imu_quat[0] = q0;  imu_quat[1] = q1;  imu_quat[2] = q2;  imu_quat[3] = q3;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     ekf_init(内部)
// 功能说明     EKF 协方差矩阵初始化(对角线 = EKF_P0_SCALE)
// 参数说明     void
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
static void ekf_init(void)
{
    uint8 i, j;
    for(i = 0; i < 4; i ++)
        for(j = 0; j < 4; j ++)
            P[i][j] = (i == j) ? EKF_P0_SCALE : 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     ekf_update(内部)
// 功能说明     6 轴姿态 EKF(加速度 + 陀螺, 无磁力计)
//              状态: 四元数 q=(q0,q1,q2,q3)(机体->NED), 4 维
//              预测: q̇ = 0.5*q⊗(0,ω), 离散化 q(k+1)=F*q, F=I+0.5*dt*Ω(ω)
//              量测: z = 加速度(重力方向, 机体系), 量测模型 h(q)=v(q)=第三行旋转矩阵
//              与 Mahony 共用 q0~q3 与零偏, 结果由 imu_ahrs_get_euler 提取
// 参数说明     gx/gy/gz   陀螺仪角速度(rad/s, 机体系), 内部自动减零偏
//              ax/ay/az   加速度(g, 机体系, 已换算为重力方向)
//              dt         采样间隔(秒), 1kHz 固定 0.001f
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
static void ekf_update(float gx, float gy, float gz,
                       float ax, float ay, float az, float dt)
{
    float half_dt = 0.5f * dt;
    float F[4][4], FP[4][4], Pnew[4][4];
    float H[3][4], HP[3][4], S[3][3], S_inv[3][3], PH_T[4][3], K[4][3];
    float q0p, q1p, q2p, q3p, qn;
    float v0, v1, v2, y0, y1, y2, an, dev, r_eff, det;
    uint8 i, j, k;

    // ---- 减零偏 ----
    gx -= gyro_bias_x;  gy -= gyro_bias_y;  gz -= gyro_bias_z;

    // ---- 状态转移矩阵 F = I + 0.5*dt*Ω(ω) ----
    F[0][0] = 1.0f;         F[0][1] = -half_dt*gx;  F[0][2] = -half_dt*gy;  F[0][3] = -half_dt*gz;
    F[1][0] =  half_dt*gx;  F[1][1] = 1.0f;         F[1][2] =  half_dt*gz;  F[1][3] = -half_dt*gy;
    F[2][0] =  half_dt*gy;  F[2][1] = -half_dt*gz;  F[2][2] = 1.0f;         F[2][3] =  half_dt*gx;
    F[3][0] =  half_dt*gz;  F[3][1] =  half_dt*gy;  F[3][2] = -half_dt*gx;  F[3][3] = 1.0f;

    // ---- 预测四元数: q_pred = F * q ----
    q0p = F[0][0]*q0 + F[0][1]*q1 + F[0][2]*q2 + F[0][3]*q3;
    q1p = F[1][0]*q0 + F[1][1]*q1 + F[1][2]*q2 + F[1][3]*q3;
    q2p = F[2][0]*q0 + F[2][1]*q1 + F[2][2]*q2 + F[2][3]*q3;
    q3p = F[3][0]*q0 + F[3][1]*q1 + F[3][2]*q2 + F[3][3]*q3;
    qn = sqrtf(q0p*q0p + q1p*q1p + q2p*q2p + q3p*q3p);
    if(qn > 1e-8f) { q0p/=qn; q1p/=qn; q2p/=qn; q3p/=qn; }

    // ---- 预测协方差: P = F*P*F^T + Q(Q 为对角阵 EKF_Q_SCALE) ----
    for(i = 0; i < 4; i ++)
        for(j = 0; j < 4; j ++)
        {
            FP[i][j] = 0.0f;
            for(k = 0; k < 4; k ++) FP[i][j] += F[i][k]*P[k][j];
        }
    for(i = 0; i < 4; i ++)
        for(j = 0; j < 4; j ++)
        {
            Pnew[i][j] = 0.0f;
            for(k = 0; k < 4; k ++) Pnew[i][j] += FP[i][k]*F[j][k];      // F^T[k][j] = F[j][k]
            if(i == j) Pnew[i][j] += EKF_Q_SCALE;
        }
    for(i = 0; i < 4; i ++)
        for(j = 0; j < 4; j ++) P[i][j] = Pnew[i][j];

    // ---- 归一化加速度(重力方向) ----
    an = sqrtf(ax*ax + ay*ay + az*az);
    if(an < 1e-6f)                          // 加速度无效: 仅用预测结果
    {
        q0=q0p; q1=q1p; q2=q2p; q3=q3p;
        imu_quat[0]=q0; imu_quat[1]=q1; imu_quat[2]=q2; imu_quat[3]=q3;
        return;
    }
    ax/=an;  ay/=an;  az/=an;

    // ---- 冲击保护: |a| 偏离 1g 越大, 量测噪声 R 越大(越不信任加速度), 避免冲击把姿态带乱 ----
    dev   = fabsf(an - 1.0f);
    r_eff = EKF_R_SCALE * (1.0f + EKF_R_IMPACT_K * dev * dev);

    // ---- 量测雅可比 H = ∂v/∂q (3x4), v 为估计重力方向 ----
    H[0][0] = -2*q2p;  H[0][1] =  2*q3p;  H[0][2] = -2*q0p;  H[0][3] =  2*q1p;
    H[1][0] =  2*q1p;  H[1][1] =  2*q0p;  H[1][2] =  2*q3p;  H[1][3] =  2*q2p;
    H[2][0] =  2*q0p;  H[2][1] = -2*q1p;  H[2][2] = -2*q2p;  H[2][3] =  2*q3p;

    // ---- 新息: y = a - v(q_pred) ----
    v0 = 2.0f*(q1p*q3p - q0p*q2p);
    v1 = 2.0f*(q0p*q1p + q2p*q3p);
    v2 = q0p*q0p - q1p*q1p - q2p*q2p + q3p*q3p;
    y0 = ax - v0;  y1 = ay - v1;  y2 = az - v2;

    // ---- S = H*P*H^T + R (3x3), R 为对角阵 EKF_R_SCALE ----
    for(i = 0; i < 3; i ++)
        for(j = 0; j < 4; j ++)
        {
            HP[i][j] = 0.0f;
            for(k = 0; k < 4; k ++) HP[i][j] += H[i][k]*P[k][j];
        }
    for(i = 0; i < 3; i ++)
        for(j = 0; j < 3; j ++)
        {
            S[i][j] = HP[i][0]*H[j][0] + HP[i][1]*H[j][1] + HP[i][2]*H[j][2] + HP[i][3]*H[j][3];
            if(i == j) S[i][j] += r_eff;
        }

    // ---- S^-1 (3x3 伴随/行列式) ----
    det = S[0][0]*(S[1][1]*S[2][2] - S[1][2]*S[2][1])
        - S[0][1]*(S[1][0]*S[2][2] - S[1][2]*S[2][0])
        + S[0][2]*(S[1][0]*S[2][1] - S[1][1]*S[2][0]);
    if(det < 1e-12f && det > -1e-12f)       // 奇异保护: 仅用预测结果
    {
        q0=q0p; q1=q1p; q2=q2p; q3=q3p;
        imu_quat[0]=q0; imu_quat[1]=q1; imu_quat[2]=q2; imu_quat[3]=q3;
        return;
    }
    S_inv[0][0] = ( S[1][1]*S[2][2] - S[1][2]*S[2][1]) / det;
    S_inv[0][1] = (-S[0][1]*S[2][2] + S[0][2]*S[2][1]) / det;
    S_inv[0][2] = ( S[0][1]*S[1][2] - S[0][2]*S[1][1]) / det;
    S_inv[1][0] = (-S[1][0]*S[2][2] + S[1][2]*S[2][0]) / det;
    S_inv[1][1] = ( S[0][0]*S[2][2] - S[0][2]*S[2][0]) / det;
    S_inv[1][2] = (-S[0][0]*S[1][2] + S[0][2]*S[1][0]) / det;
    S_inv[2][0] = ( S[1][0]*S[2][1] - S[1][1]*S[2][0]) / det;
    S_inv[2][1] = (-S[0][0]*S[2][1] + S[0][1]*S[2][0]) / det;
    S_inv[2][2] = ( S[0][0]*S[1][1] - S[0][1]*S[1][0]) / det;

    // ---- 卡尔曼增益: K = P*H^T*S^-1 (4x3) ----
    for(i = 0; i < 4; i ++)
        for(j = 0; j < 3; j ++)
            PH_T[i][j] = P[i][0]*H[j][0] + P[i][1]*H[j][1] + P[i][2]*H[j][2] + P[i][3]*H[j][3];
    for(i = 0; i < 4; i ++)
        for(j = 0; j < 3; j ++)
            K[i][j] = PH_T[i][0]*S_inv[0][j] + PH_T[i][1]*S_inv[1][j] + PH_T[i][2]*S_inv[2][j];

    // ---- 状态更新: q = q_pred + K*y, 并归一化 ----
    q0 = q0p + K[0][0]*y0 + K[0][1]*y1 + K[0][2]*y2;
    q1 = q1p + K[1][0]*y0 + K[1][1]*y1 + K[1][2]*y2;
    q2 = q2p + K[2][0]*y0 + K[2][1]*y1 + K[2][2]*y2;
    q3 = q3p + K[3][0]*y0 + K[3][1]*y1 + K[3][2]*y2;
    qn = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if(qn > 1e-8f) { q0/=qn; q1/=qn; q2/=qn; q3/=qn; }

    // ---- 协方差更新: P = (I - K*H)*P, 再对称化 ----
    for(i = 0; i < 4; i ++)
        for(j = 0; j < 4; j ++)
        {
            Pnew[i][j] = (i == j ? 1.0f : 0.0f);
            for(k = 0; k < 3; k ++) Pnew[i][j] -= K[i][k]*H[k][j];     // I - K*H
        }
    for(i = 0; i < 4; i ++)
        for(j = 0; j < 4; j ++)
        {
            FP[i][j] = 0.0f;
            for(k = 0; k < 4; k ++) FP[i][j] += Pnew[i][k]*P[k][j];
        }
    for(i = 0; i < 4; i ++)
        for(j = 0; j < 4; j ++) P[i][j] = FP[i][j];
    for(i = 0; i < 4; i ++)
        for(j = i + 1; j < 4; j ++)                     // 强制对称, 防数值漂移
        {
            P[i][j] = P[j][i] = 0.5f * (P[i][j] + P[j][i]);
        }

    imu_quat[0] = q0;  imu_quat[1] = q1;  imu_quat[2] = q2;  imu_quat[3] = q3;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     eskf_init(内部)
// 功能说明     ESKF 误差状态协方差初始化(按 θ/b/v/p 分块设对角初值, 并清零名义速度/位置)
// 参数说明     void
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
static void eskf_init(void)
{
    uint8 i, j;
    for(i = 0; i < 10; i ++)
        for(j = 0; j < 10; j ++)
            P_eskf[i][j] = (i == j) ? ((i < 3) ? ESKF_P0_SCALE : (i < 6) ? ESKF_P0_BIAS
                                    : (i < 8) ? ESKF_P0_VEL : ESKF_P0_POS) : 0.0f;
    b_eskf[0] = b_eskf[1] = b_eskf[2] = 0.0f;
    imu_eskf_bias[0] = imu_eskf_bias[1] = imu_eskf_bias[2] = 0.0f;
    eskf_r_eff = 0.0f;
    imu_nav_x = imu_nav_y = imu_nav_vx = imu_nav_vy = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     eskf_inject(内部)
// 功能说明     ESKF 误差状态注入: 把估计出的误差状态注入名义状态(注入后误差状态即归零, 与现 ESKF 一致)
//              1) 姿态误差 δθ(机体系)以右乘小旋转注入当前四元数: q ← normalize(q ⊗ exp(δθ/2))
//              2) 零漂误差 δb 累加到在线零偏 b_eskf 并更新显示拷贝
//              3) 速度误差 δv、位置误差 δp 累加到名义世界系水平速度/位置
//              加速度/速度/位置三个观测更新共用本函数(在各自算好误差状态后调用)
// 参数说明     de  旋转误差(rad, 机体系)
//              db  零漂误差(rad/s, 机体系)
//              dv  速度误差(m/s, 世界系水平)
//              dp  位置误差(m, 世界系水平)
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
static void eskf_inject(const float de[3], const float db[3],
                        const float dv[2], const float dp[2])
{
    float th, ha, ss, dq0, dq1, dq2, dq3, nq0, nq1, nq2, nq3, qn;

    // 姿态误差注入: q ← q ⊗ exp(δθ/2)(Rodrigues 小角度指数映射, 作用于当前 q)
    th = sqrtf(de[0]*de[0] + de[1]*de[1] + de[2]*de[2]);
    if(th > 1e-12f)
    {
        ha  = 0.5f * th;
        ss  = sinf(ha) / th;
        dq0 = cosf(ha);
        dq1 = ss * de[0];
        dq2 = ss * de[1];
        dq3 = ss * de[2];
        nq0 = q0*dq0 - q1*dq1 - q2*dq2 - q3*dq3;
        nq1 = q0*dq1 + q1*dq0 + q2*dq3 - q3*dq2;
        nq2 = q0*dq2 - q1*dq3 + q2*dq0 + q3*dq1;
        nq3 = q0*dq3 + q1*dq2 - q2*dq1 + q3*dq0;
        q0 = nq0;  q1 = nq1;  q2 = nq2;  q3 = nq3;
    }
    qn = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if(qn > 1e-8f) { q0/=qn; q1/=qn; q2/=qn; q3/=qn; }
    imu_quat[0] = q0;  imu_quat[1] = q1;  imu_quat[2] = q2;  imu_quat[3] = q3;

    // 零漂误差注入(rad/s)
    b_eskf[0] += db[0];  b_eskf[1] += db[1];  b_eskf[2] += db[2];
    imu_eskf_bias[0] = b_eskf[0];  imu_eskf_bias[1] = b_eskf[1];  imu_eskf_bias[2] = b_eskf[2];

    // 速度/位置误差注入(m/s, m, 世界系水平)
    imu_nav_vx += dv[0];  imu_nav_vy += dv[1];
    imu_nav_x  += dp[0];  imu_nav_y  += dp[1];
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     eskf_update(内部)
// 功能说明     10 态姿态+惯导 ESKF(误差状态卡尔曼, 角速度积分 + 加速度观测 + 速度/位置积分)
//              名义状态: 四元数 q(机体->NED) + 世界系水平速度/位置(imu_nav_vx/vy, imu_nav_x/y)
//              误差状态: 10 维 [δθ(3旋转误差), δb(3陀螺零漂误差), δv(2速度误差), δp(2位置误差)]
//                        加速度(重力方向)观测修正 roll/pitch 及零漂; 速度/位置由比力积分(INS)推进,
//                        轮速观测(eskf_vel_update)只修正速度/位置(姿态与零漂仅由加速度观测修正)
//              与 Mahony/EKF 共用 q0~q3 与上电静态零偏, 结果由 imu_ahrs_get_euler 提取
// 参数说明     gx/gy/gz   陀螺仪角速度(rad/s, 机体系), 内部自动减零偏
//              ax/ay/az   加速度(g, 机体系, 已换算为重力方向)
//              dt         采样间隔(秒), 1kHz 固定 0.001f
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
static void eskf_update(float gx, float gy, float gz,
                        float ax, float ay, float az, float dt)
{
    float half_dt = 0.5f * dt;
    static float F[10][10], FP[10][10], Pnew[10][10], IKH[10][10]; // static: 1kHz 主循环里减小栈开销
    static float H[3][10], HP[3][10], S[3][3], S_inv[3][3], PH_T[10][3], K[10][3];
    float q0p, q1p, q2p, q3p, qn;
    float v0, v1, v2, y0, y1, y2, an, r_eff, det;
    float de[3], db[3], dv[2], dp[2];
    float R00, R01, R02, R10, R11, R12, fx, fy, fz;
    float M00, M01, M02, M10, M11, M12;
    uint8 i, j, k;

    // ---- 减上电零偏 + ESKF 在线零漂 ----
    gx -= gyro_bias_x;  gy -= gyro_bias_y;  gz -= gyro_bias_z;   // 上电静态零偏(rad/s)
    gx -= b_eskf[0];    gy -= b_eskf[1];    gz -= b_eskf[2];     // ESKF 在线估计的残余零漂(rad/s)

    // ---- 名义四元数预测(角速度积分) ----
    q0p = q0 + (-q1*gx - q2*gy - q3*gz) * half_dt;
    q1p = q1 + ( q0*gx + q2*gz - q3*gy) * half_dt;
    q2p = q2 + ( q0*gy - q1*gz + q3*gx) * half_dt;
    q3p = q3 + ( q0*gz + q1*gy - q2*gx) * half_dt;
    qn = sqrtf(q0p*q0p + q1p*q1p + q2p*q2p + q3p*q3p);
    if(qn > 1e-8f) { q0p/=qn; q1p/=qn; q2p/=qn; q3p/=qn; }

    // ---- 名义速度/位置预测(INS 比力积分, 始终执行, 不受加速度门控影响) ----
    // 比力 f_b = -g·a(机体系, m/s²)转世界系取水平分量积分; 姿态误差对积分的污染由误差状态 δv/δp 吸收
    // 注: 必须在加速度归一化/门控之前用原始 ax/ay/az
    R00 = q0p*q0p + q1p*q1p - q2p*q2p - q3p*q3p;
    R01 = 2.0f*(q1p*q2p - q0p*q3p);
    R02 = 2.0f*(q1p*q3p + q0p*q2p);
    R10 = 2.0f*(q1p*q2p + q0p*q3p);
    R11 = q0p*q0p - q1p*q1p + q2p*q2p - q3p*q3p;
    R12 = 2.0f*(q2p*q3p - q0p*q1p);
    fx = -ax * NAV_G_MPS2;
    fy = -ay * NAV_G_MPS2;
    fz = -az * NAV_G_MPS2;
    imu_nav_vx += (R00*fx + R01*fy + R02*fz) * dt;
    imu_nav_vy += (R10*fx + R11*fy + R12*fz) * dt;
    imu_nav_x  += imu_nav_vx * dt;
    imu_nav_y  += imu_nav_vy * dt;

    // ---- 误差状态协方差预测: P = F*P*F^T + Q (10x10) ----
    // 误差状态 [δθ(3), δb(3), δv(2), δp(2)]:
    //   δθ' = (I-[ω]×dt)δθ − dt·δb,  δb' = δb
    //   δv' = δv − R·[f_b]×·δθ·dt   (姿态误差经比力/重力耦合进水平速度, 产生速度-姿态交叉协方差;
    //                                该耦合现仅用于协方差传播, 速度观测已不向姿态/零漂回馈)
    //   δp' = δp + δv·dt
    for(i = 0; i < 10; i ++)
        for(j = 0; j < 10; j ++) F[i][j] = 0.0f;
    F[0][0] = 1.0f;   F[0][1] =  gz*dt;  F[0][2] = -gy*dt;  F[0][3] = -dt;
    F[1][0] = -gz*dt;  F[1][1] = 1.0f;   F[1][2] =  gx*dt;  F[1][4] = -dt;
    F[2][0] =  gy*dt;  F[2][1] = -gx*dt; F[2][2] = 1.0f;    F[2][5] = -dt;
    F[3][3] = 1.0f;   F[4][4] = 1.0f;   F[5][5] = 1.0f;
    // M = R·[f_b]×, 取世界 x/y 两行(水平静止校验: δv̇_x=-g·δθ_y, δv̇_y=+g·δθ_x)
    M00 =  R01*fz - R02*fy;   M01 = -R00*fz + R02*fx;   M02 =  R00*fy - R01*fx;
    M10 =  R11*fz - R12*fy;   M11 = -R10*fz + R12*fx;   M12 =  R10*fy - R11*fx;
    F[6][0] = -dt*M00;  F[6][1] = -dt*M01;  F[6][2] = -dt*M02;
    F[7][0] = -dt*M10;  F[7][1] = -dt*M11;  F[7][2] = -dt*M12;
    F[6][6] = 1.0f;  F[7][7] = 1.0f;
    F[8][6] = dt;    F[9][7] = dt;
    F[8][8] = 1.0f;  F[9][9] = 1.0f;
    for(i = 0; i < 10; i ++)
        for(j = 0; j < 10; j ++)
        {
            FP[i][j] = 0.0f;
            for(k = 0; k < 10; k ++) FP[i][j] += F[i][k]*P_eskf[k][j];
        }
    for(i = 0; i < 10; i ++)
        for(j = 0; j < 10; j ++)
        {
            Pnew[i][j] = 0.0f;
            for(k = 0; k < 10; k ++) Pnew[i][j] += FP[i][k]*F[j][k];
            if(i == j) Pnew[i][j] += (i < 3) ? ESKF_Q_SCALE : (i < 6) ? ESKF_Q_BIAS
                                   : (i < 8) ? ESKF_Q_VEL : ESKF_Q_POS;
        }
    for(i = 0; i < 10; i ++)
        for(j = 0; j < 10; j ++) P_eskf[i][j] = Pnew[i][j];

    // ---- 加速度观测(重力方向) ----
    an = sqrtf(ax*ax + ay*ay + az*az);
    if(an < 1e-6f)                          // 加速度无效: 仅用名义积分结果
    {
        q0=q0p; q1=q1p; q2=q2p; q3=q3p;
        imu_quat[0]=q0; imu_quat[1]=q1; imu_quat[2]=q2; imu_quat[3]=q3;
        return;
    }
    // ---- 逐拍冲击门限: |a| 偏离 1g 过大 → 比力含线性加速度(非重力), 本拍不信任加速度, 只积分 ----
    if(fabsf(an - 1.0f) > ESKF_ACCEL_TRUST_G)
    {
        q0=q0p; q1=q1p; q2=q2p; q3=q3p;
        imu_quat[0]=q0; imu_quat[1]=q1; imu_quat[2]=q2; imu_quat[3]=q3;
        return;
    }
    // ---- 角速度门限: 旋转剧烈时加速度方向受离心/切向加速度污染(|a|可能仍≈1g, 模长门限拦不住), 本拍不用 ----
    if(fabsf(gx) > ESKF_GYRO_TRUST_DPS * DEG2RAD ||
       fabsf(gy) > ESKF_GYRO_TRUST_DPS * DEG2RAD ||
       fabsf(gz) > ESKF_GYRO_TRUST_DPS * DEG2RAD)
    {
        q0=q0p; q1=q1p; q2=q2p; q3=q3p;
        imu_quat[0]=q0; imu_quat[1]=q1; imu_quat[2]=q2; imu_quat[3]=q3;
        return;
    }
    ax/=an;  ay/=an;  az/=an;

    // ---- 预测观测 v(q_pred) 与新息 y = a - v ----
    v0 = 2.0f*(q1p*q3p - q0p*q2p);
    v1 = 2.0f*(q0p*q1p + q2p*q3p);
    v2 = q0p*q0p - q1p*q1p - q2p*q2p + q3p*q3p;
    y0 = ax - v0;  y1 = ay - v1;  y2 = az - v2;

    // ---- 量测雅可比 H (3x10) = [H_θ(3x3), 0(3x7)] ----
    // 零漂/速度/位置不直接被加速度观测, 靠"姿态误差↔(零漂|速度|位置)"耦合协方差在线修正
    H[0][0] =  2.0f*(q1p*q2p + q0p*q3p);
    H[0][1] = -(q0p*q0p + q1p*q1p - q2p*q2p - q3p*q3p);
    H[0][2] = 0.0f;
    H[1][0] =  q0p*q0p - q1p*q1p + q2p*q2p - q3p*q3p;
    H[1][1] = -2.0f*(q1p*q2p - q0p*q3p);
    H[1][2] = 0.0f;
    H[2][0] =  2.0f*(q2p*q3p - q0p*q1p);
    H[2][1] = -2.0f*(q1p*q3p + q0p*q2p);
    H[2][2] = 0.0f;
    for(j = 3; j < 10; j ++) { H[0][j] = 0.0f;  H[1][j] = 0.0f;  H[2][j] = 0.0f; }

    // ---- R: 门限内恒定小R, 信任加速度、硬纠正 ----
    // 注意: 不能用新息协方差(IAE)——它把"姿态真错了(该硬纠正)"也当成测量噪声吸收,
    //       导致 roll/pitch 被冲击打歪后修不回来。|a|≈1g 说明加速度测的是纯重力, 应强信任。
    r_eff = ESKF_R_BASE;
    eskf_r_eff = r_eff;                             // 显示用

    // ---- S = H*P*H^T + R (3x3) ----
    for(i = 0; i < 3; i ++)
        for(j = 0; j < 10; j ++)
        {
            HP[i][j] = 0.0f;
            for(k = 0; k < 10; k ++) HP[i][j] += H[i][k]*P_eskf[k][j];
        }
    for(i = 0; i < 3; i ++)
        for(j = 0; j < 3; j ++)
        {
            S[i][j] = 0.0f;
            for(k = 0; k < 10; k ++) S[i][j] += HP[i][k]*H[j][k];
            if(i == j) S[i][j] += r_eff;
        }

    // ---- S^-1 (3x3 伴随/行列式) ----
    det = S[0][0]*(S[1][1]*S[2][2] - S[1][2]*S[2][1])
        - S[0][1]*(S[1][0]*S[2][2] - S[1][2]*S[2][0])
        + S[0][2]*(S[1][0]*S[2][1] - S[1][1]*S[2][0]);
    if(det < 1e-12f && det > -1e-12f)       // 奇异保护: 仅用名义积分结果
    {
        q0=q0p; q1=q1p; q2=q2p; q3=q3p;
        imu_quat[0]=q0; imu_quat[1]=q1; imu_quat[2]=q2; imu_quat[3]=q3;
        return;
    }
    S_inv[0][0] = ( S[1][1]*S[2][2] - S[1][2]*S[2][1]) / det;
    S_inv[0][1] = (-S[0][1]*S[2][2] + S[0][2]*S[2][1]) / det;
    S_inv[0][2] = ( S[0][1]*S[1][2] - S[0][2]*S[1][1]) / det;
    S_inv[1][0] = (-S[1][0]*S[2][2] + S[1][2]*S[2][0]) / det;
    S_inv[1][1] = ( S[0][0]*S[2][2] - S[0][2]*S[2][0]) / det;
    S_inv[1][2] = (-S[0][0]*S[1][2] + S[0][2]*S[1][0]) / det;
    S_inv[2][0] = ( S[1][0]*S[2][1] - S[1][1]*S[2][0]) / det;
    S_inv[2][1] = (-S[0][0]*S[2][1] + S[0][1]*S[2][0]) / det;
    S_inv[2][2] = ( S[0][0]*S[1][1] - S[0][1]*S[1][0]) / det;

    // ---- 卡尔曼增益: K = P*H^T*S^-1 (10x3) ----
    for(i = 0; i < 10; i ++)
        for(j = 0; j < 3; j ++)
        {
            PH_T[i][j] = 0.0f;
            for(k = 0; k < 10; k ++) PH_T[i][j] += P_eskf[i][k]*H[j][k];
        }
    for(i = 0; i < 10; i ++)
        for(j = 0; j < 3; j ++)
        {
            K[i][j] = 0.0f;
            for(k = 0; k < 3; k ++) K[i][j] += PH_T[i][k]*S_inv[k][j];
        }

    // ---- 加速度无法观测 yaw 及其零漂: 强制 δθ_z、δb_z 对应 K 行为零, 防污染泄漏 ----
    K[2][0] = 0.0f;  K[2][1] = 0.0f;  K[2][2] = 0.0f;   // δθ_z
    K[5][0] = 0.0f;  K[5][1] = 0.0f;  K[5][2] = 0.0f;   // δb_z
    // 注: 速度观测(eskf_vel_update)只修正速度/位置, 不修正姿态与零漂(内部强制 δθ、δb 全部 K 行置 0)

    // ---- 误差状态估计: δx = K*y ----
    de[0] = K[0][0]*y0 + K[0][1]*y1 + K[0][2]*y2;
    de[1] = K[1][0]*y0 + K[1][1]*y1 + K[1][2]*y2;
    de[2] = K[2][0]*y0 + K[2][1]*y1 + K[2][2]*y2;
    db[0] = K[3][0]*y0 + K[3][1]*y1 + K[3][2]*y2;
    db[1] = K[4][0]*y0 + K[4][1]*y1 + K[4][2]*y2;
    db[2] = K[5][0]*y0 + K[5][1]*y1 + K[5][2]*y2;
    dv[0] = K[6][0]*y0 + K[6][1]*y1 + K[6][2]*y2;
    dv[1] = K[7][0]*y0 + K[7][1]*y1 + K[7][2]*y2;
    dp[0] = K[8][0]*y0 + K[8][1]*y1 + K[8][2]*y2;
    dp[1] = K[9][0]*y0 + K[9][1]*y1 + K[9][2]*y2;

    // ---- 误差注入: 名义四元数先取 q_pred, 再统一注入(旋转+零漂+速度+位置) ----
    q0 = q0p;  q1 = q1p;  q2 = q2p;  q3 = q3p;
    eskf_inject(de, db, dv, dp);

    // ---- 协方差更新: P = (I - K*H)*P (10x10), 再对称化 ----
    // IKH = I − K·H, K(10x3)·H(3x10), 内积维 = 3(量测维, 勿写 10, 会越界读垃圾值 → NaN)
    for(i = 0; i < 10; i ++)
        for(j = 0; j < 10; j ++)
        {
            IKH[i][j] = (i == j ? 1.0f : 0.0f);
            for(k = 0; k < 3; k ++) IKH[i][j] -= K[i][k]*H[k][j];
        }
    for(i = 0; i < 10; i ++)
        for(j = 0; j < 10; j ++)
        {
            Pnew[i][j] = 0.0f;
            for(k = 0; k < 10; k ++) Pnew[i][j] += IKH[i][k]*P_eskf[k][j];
        }
    for(i = 0; i < 10; i ++)
        for(j = 0; j < 10; j ++) P_eskf[i][j] = Pnew[i][j];
    for(i = 0; i < 10; i ++)
        for(j = i + 1; j < 10; j ++)
        {
            P_eskf[i][j] = P_eskf[j][i] = 0.5f * (P_eskf[i][j] + P_eskf[j][i]);
        }

    imu_quat[0] = q0;  imu_quat[1] = q1;  imu_quat[2] = q2;  imu_quat[3] = q3;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     eskf_vel_update
// 功能说明     速度观测(轮速)量测更新: 用世界系水平速度测量修正 10 态 ESKF
//              量测模型: z = [vx, vy](世界系, 需按航向分解), H 只选 δv 两列(索引 6,7)
//              只修正速度 δv 与位置 δp; 姿态/零漂的 K 行强制置 0(姿态与零漂仅由加速度观测修正)
// 参数说明     vx_meas/vy_meas  世界系速度测量(m/s, 已按航向分解)
//              wx/wy            机体角速度(rad/s, 杠臂补偿用)
// 返回参数     void
// 使用示例     eskf_vel_update(v_fwd*cosf(yaw), v_fwd*sinf(yaw), wx, wy);
//-------------------------------------------------------------------------------------------------------------------
void eskf_vel_update(float vx_meas, float vy_meas, float wx, float wy)
{
    static float K[10][2], IKH[10][10], Pnew[10][10];   // static: 500Hz 主循环里减小栈开销
    float y0, y1, S00, S01, S10, S11, det, S00i, S01i, S10i, S11i;
    float vx_pred, vy_pred;
    float de[3], db[3], dv[2], dp[2];
    uint8 i, j, k;

    // 杠臂补偿(NAV_LEVER_ARM_ENABLE=1 时开): 轮速观测点P(0,0,0), IMU B(0,0,-0.1)
    //   v_P = v_B + ω×(r_P-r_B), r_P-r_B = (0,0,+h) → vP_x = vB_x + h*ωy, vP_y = vB_y - h*ωx
#if NAV_LEVER_ARM_ENABLE
    vx_pred = imu_nav_vx + NAV_LEVER_ARM_Z * wy;
    vy_pred = imu_nav_vy - NAV_LEVER_ARM_Z * wx;
#else
    vx_pred = imu_nav_vx;                           // 杠臂补偿关闭: 观测点直接按IMU点处理
    vy_pred = imu_nav_vy;
#endif

    y0 = vx_meas - vx_pred;                             // 新息(含杠臂)
    y1 = vy_meas - vy_pred;

    // 注: imu_nav_obs_vx/vy(杠臂补偿后观测, 显示用)已在 main 每拍计算, 不受打滑门控冻结

    // ---- 新息门限(抗离群): 速度观测与预测速度 2D 模长相差超过门限 → 拒绝本次观测 ----
    // 打滑/轮速跳变时轮速≠地面速度, 更新会把 v/p 带偏; 该门限在主程序打滑检测之外再加一道保险
    if(sqrtf(y0*y0 + y1*y1) > ESKF_VEL_INNOV_GATE_MPS)
    {
        return;
    }

    // ---- 固定 R: ESKF_R_VEL(σ=1cm/s), 不随新息自适应 ----
    // 之前 IAE 自适应把"速度误差偏置"误当噪声, 导致收敛慢/发散; 轮速观测用固定R直接修正
    nav_r_vel = ESKF_R_VEL;
    S00 = P_eskf[6][6] + nav_r_vel;                     // S = H·P⁻·Hᵀ + R, H 只选 δv 两列
    S01 = P_eskf[6][7];
    S10 = P_eskf[7][6];
    S11 = P_eskf[7][7] + nav_r_vel;
    det = S00*S11 - S01*S10;
    if(det < 1e-12f && det > -1e-12f) return;           // 奇异保护
    S00i =  S11/det;  S01i = -S01/det;
    S10i = -S10/det;  S11i =  S00/det;

    // ---- 卡尔曼增益: K = P·Hᵀ·S⁻¹ (10x2) ----
    for(i = 0; i < 10; i ++)
    {
        K[i][0] = P_eskf[i][6]*S00i + P_eskf[i][7]*S10i;
        K[i][1] = P_eskf[i][6]*S01i + P_eskf[i][7]*S11i;
    }

    // ---- 速度观测只修正速度/位置: 强制 δθ、δb 全部 K 行为零, 不修正姿态与零漂(姿态/零漂仅由加速度观测修正) ----
    K[0][0] = 0.0f;  K[0][1] = 0.0f;   // δθ_x
    K[1][0] = 0.0f;  K[1][1] = 0.0f;   // δθ_y
    K[2][0] = 0.0f;  K[2][1] = 0.0f;   // δθ_z
    K[3][0] = 0.0f;  K[3][1] = 0.0f;   // δb_x
    K[4][0] = 0.0f;  K[4][1] = 0.0f;   // δb_y
    K[5][0] = 0.0f;  K[5][1] = 0.0f;   // δb_z

    // ---- 误差状态估计: δx = K*y, 并注入 ----
    de[0] = K[0][0]*y0 + K[0][1]*y1;
    de[1] = K[1][0]*y0 + K[1][1]*y1;
    de[2] = K[2][0]*y0 + K[2][1]*y1;
    db[0] = K[3][0]*y0 + K[3][1]*y1;
    db[1] = K[4][0]*y0 + K[4][1]*y1;
    db[2] = K[5][0]*y0 + K[5][1]*y1;
    dv[0] = K[6][0]*y0 + K[6][1]*y1;
    dv[1] = K[7][0]*y0 + K[7][1]*y1;
    dp[0] = K[8][0]*y0 + K[8][1]*y1;
    dp[1] = K[9][0]*y0 + K[9][1]*y1;
    eskf_inject(de, db, dv, dp);

    // ---- 协方差更新: P = (I - K*H)*P (10x10), 再对称化 ----
    // IKH = I − K·H, K(10x2)·H(2x10), 内积维 = 2(量测维, 勿写 10)
    for(i = 0; i < 10; i ++)
        for(j = 0; j < 10; j ++)
        {
            IKH[i][j] = (i == j ? 1.0f : 0.0f);
            if(j == 6) IKH[i][j] -= K[i][0];
            if(j == 7) IKH[i][j] -= K[i][1];
        }
    for(i = 0; i < 10; i ++)
        for(j = 0; j < 10; j ++)
        {
            Pnew[i][j] = 0.0f;
            for(k = 0; k < 10; k ++) Pnew[i][j] += IKH[i][k]*P_eskf[k][j];
        }
    for(i = 0; i < 10; i ++)
        for(j = 0; j < 10; j ++) P_eskf[i][j] = Pnew[i][j];
    for(i = 0; i < 10; i ++)
        for(j = i + 1; j < 10; j ++)
            P_eskf[i][j] = P_eskf[j][i] = 0.5f * (P_eskf[i][j] + P_eskf[j][i]);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     eskf_pos_update
// 功能说明     位置观测量测更新(预留钩子): 有外部位置测量(GPS/信标/摄像头等)时调用
//              量测模型: z = [x, y](世界系), H 只选 δp 两列(索引 8,9)
//              当前工程暂无位置测量源, 位置由 ESKF 纯航迹推算, 此接口供后续接入
// 参数说明     x_meas/y_meas    世界系位置测量(m)
// 返回参数     void
// 使用示例     eskf_pos_update(gps_x, gps_y);
//-------------------------------------------------------------------------------------------------------------------
void eskf_pos_update(float x_meas, float y_meas)
{
    static float K[10][2], IKH[10][10], Pnew[10][10];
    float y0, y1, S00, S01, S10, S11, det, S00i, S01i, S10i, S11i;
    float de[3], db[3], dv[2], dp[2];
    uint8 i, j, k;

    y0 = x_meas - imu_nav_x;                            // 新息
    y1 = y_meas - imu_nav_y;

    // ---- 固定 R: ESKF_R_POS ----
    S00 = P_eskf[8][8] + ESKF_R_POS;
    S01 = P_eskf[8][9];
    S10 = P_eskf[9][8];
    S11 = P_eskf[9][9] + ESKF_R_POS;
    det = S00*S11 - S01*S10;
    if(det < 1e-12f && det > -1e-12f) return;           // 奇异保护
    S00i =  S11/det;  S01i = -S01/det;
    S10i = -S10/det;  S11i =  S00/det;

    // ---- 卡尔曼增益: K = P·Hᵀ·S⁻¹ (10x2) ----
    for(i = 0; i < 10; i ++)
    {
        K[i][0] = P_eskf[i][8]*S00i + P_eskf[i][9]*S10i;
        K[i][1] = P_eskf[i][8]*S01i + P_eskf[i][9]*S11i;
    }

    // ---- 误差状态估计并注入 ----
    de[0] = K[0][0]*y0 + K[0][1]*y1;
    de[1] = K[1][0]*y0 + K[1][1]*y1;
    de[2] = K[2][0]*y0 + K[2][1]*y1;
    db[0] = K[3][0]*y0 + K[3][1]*y1;
    db[1] = K[4][0]*y0 + K[4][1]*y1;
    db[2] = K[5][0]*y0 + K[5][1]*y1;
    dv[0] = K[6][0]*y0 + K[6][1]*y1;
    dv[1] = K[7][0]*y0 + K[7][1]*y1;
    dp[0] = K[8][0]*y0 + K[8][1]*y1;
    dp[1] = K[9][0]*y0 + K[9][1]*y1;
    eskf_inject(de, db, dv, dp);

    // ---- 协方差更新: P = (I - K*H)*P (10x10), 再对称化 ----
    for(i = 0; i < 10; i ++)
        for(j = 0; j < 10; j ++)
        {
            IKH[i][j] = (i == j ? 1.0f : 0.0f);
            if(j == 8) IKH[i][j] -= K[i][0];
            if(j == 9) IKH[i][j] -= K[i][1];
        }
    for(i = 0; i < 10; i ++)
        for(j = 0; j < 10; j ++)
        {
            Pnew[i][j] = 0.0f;
            for(k = 0; k < 10; k ++) Pnew[i][j] += IKH[i][k]*P_eskf[k][j];
        }
    for(i = 0; i < 10; i ++)
        for(j = 0; j < 10; j ++) P_eskf[i][j] = Pnew[i][j];
    for(i = 0; i < 10; i ++)
        for(j = i + 1; j < 10; j ++)
            P_eskf[i][j] = P_eskf[j][i] = 0.5f * (P_eskf[i][j] + P_eskf[j][i]);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     imu_ahrs_update
// 功能说明     姿态解算入口, 按 IMU_AHRS_MODE 宏选择算法:
//              0 = 互补滤波(Mahony), 1 = 四元数EKF, 2 = ESKF
// 参数说明     gx/gy/gz   陀螺仪角速度(rad/s, 机体系)
//              ax/ay/az   加速度(g, 机体系)
//              dt         采样间隔(秒), 1kHz 固定 0.001f
// 返回参数     void
// 使用示例     imu_ahrs_update(gx, gy, gz, ax, ay, az, 0.001f);
//-------------------------------------------------------------------------------------------------------------------
void imu_ahrs_update(float gx, float gy, float gz,
                     float ax, float ay, float az, float dt)
{
    // IMU_AHRS_MODE 为编译期常量, 编译器会自动只保留一条分支
    if(IMU_AHRS_MODE == 2)
    {
        eskf_update(gx, gy, gz, ax, ay, az, dt);
    }
    else if(IMU_AHRS_MODE == 1)
    {
        ekf_update(gx, gy, gz, ax, ay, az, dt);
    }
    else
    {
        mahony_update(gx, gy, gz, ax, ay, az, dt);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     imu_ahrs_get_euler
// 功能说明     由四元数提取欧拉角(NED, 度), 写入 imu_roll_deg / imu_pitch_deg / imu_yaw_deg
// 参数说明     void
// 返回参数     void
// 使用示例     imu_ahrs_get_euler();  // 之后直接读取 imu_roll_deg 等
//-------------------------------------------------------------------------------------------------------------------
void imu_ahrs_get_euler(void)
{
    float a;

    imu_roll_deg  = RAD2DEG * atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    a = 2.0f * (q0 * q2 - q1 * q3);                             // asinf 参数限幅, 防 ±90° 俯仰溢出
    if(a >  1.0f) a =  1.0f;
    if(a < -1.0f) a = -1.0f;
    imu_pitch_deg = RAD2DEG * asinf(a);
    imu_yaw_deg   = RAD2DEG * atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     imu_nav_init
// 功能说明     惯导(ESKF 速度/位置块)初始化: 清零世界系位置/速度, 重置 [δv, δp] 协方差分块初值
//              (10 态 ESKF 已把速度/位置并入误差状态, 此处只重置这两个分块)
// 参数说明     void
// 返回参数     void
// 使用示例     imu_nav_init();            // 上电或需要重新计程时调用
//-------------------------------------------------------------------------------------------------------------------
void imu_nav_init(void)
{
    uint8 i, j;
    imu_nav_x = imu_nav_y = imu_nav_vx = imu_nav_vy = 0.0f;
    nav_r_vel = ESKF_R_VEL;
    for(i = 6; i < 10; i ++)
        for(j = 0; j < 10; j ++)
            P_eskf[i][j] = P_eskf[j][i] = 0.0f;    // 清速度/位置行与列(含与姿态/零漂的交叉协方差)
    P_eskf[6][6] = P_eskf[7][7] = ESKF_P0_VEL;
    P_eskf[8][8] = P_eskf[9][9] = ESKF_P0_POS;
}
