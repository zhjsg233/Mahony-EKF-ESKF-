/******************************************************************************
* @file    lq_canfd.h
* @brief   轮足机器人 CAN FD通信头文件
* @author  龙邱智能
* @note    1. 关节电机4个（CAN1）+ 行进电机2个（CAN0）
*          2. CANFD0: 500Hz控制帧 + 100Hz状态上报
*          3. CANFD1: 100Hz控制帧 + 100Hz状态上报
* @author  龙邱智能 chiusir
* @version 4.1
* @date    2026.04.24
* @IDE     IAR9.40.1
******************************************************************************/
#ifndef __LQ_CANFD_H__
#define __LQ_CANFD_H__

#include "zf_common_headfile.h"

/*============================================================================
* 电机ID定义
*============================================================================*/
//如果FOC驱动板，开启下面的宏定义；
//如果是控制器，并注释掉下面的宏定义
//配合CANFD.h中的宏定义和main_cm4.c中的宏定义
//FOC需要设置两项：THIS_BOARD_IS_FOC  =1；THIS_MOTOR_ID =电机ID 
//控制板需要设置两项：THIS_BOARD_IS_FOC=0；THIS_MOTOR_ID=CONTROL_BOARD 
 
//FOC驱动板
#define THIS_MOTOR_ID              CONTROL_BOARD

//节点ID，机器狗的
#define LEFT_FRONT_MOTOR           0x51u //左前电机，位置模式，正向转动，转动范围（0-->14）
#define LEFT_BACK_MOTOR            0x52u //左后电机，位置模式，反向转动，转动范围（0-->-14）
#define RIGHT_FRONT_MOTOR          0x53u //右前电机，位置模式，反向转动，转动范围（0-->-14）
#define RIGHT_BACK_MOTOR           0x54u //右后电机，位置模式，正向转动，转动范围（0-->14）
#define LEFT_WALK_MOTOR            0x55u //左行进电机，速度模式
#define RIGHT_WALK_MOTOR           0x56u //右行进电机，速度模式

#define CONTROL_BOARD              0x88u


/*============================================================================
* CAN ID定义
*============================================================================*/
#define CAN_ID_MOTOR_MASTER         0x20u   // 电机控制帧：主控发给FOC
#define CAN_ID_MOTOR_STATUS         0x21u   // 状态请求帧：主控发给FOC
#define CAN_ID_MOTOR_MST_STATUS     0x22u   // 状态上报帧：FOC发给主控
#define CAN_ID_MOTOR_SET_INITIAL_POSE  0x25u   // 设置初始位置


/*============================================================================
* 配置参数
*============================================================================*/

#define JOINT_MOTOR_COUNT     4u      // 关节电机数量
#define WALK_MOTOR_COUNT      2u      // 行进电机数量

#define CANFD0_MIN_INTERVAL_MS        2   // CANFD0 500Hz
#define CANFD1_MIN_INTERVAL_MS        10  // CANFD1 100Hz
#define STATUS_REPORT_INTERVAL_MS      20  // 状态上报 20Hz
#define STATUS_REQUEST_INTERVAL_MS     10  // 请求间隔 100Hz


/*============================================================================
* 电机数量定义
*============================================================================*/
#define MOTOR_MAX                   6u //全部8个电机
// CAN FD 数据长度定义：64字节 = 16个32位数据，对应CAN FD最大帧长度
#define CANFDDLC64B                16u    


/**
 * @brief FOC电机运行模式枚举
 * @note  定义电机不同的工作状态，用于模式切换控制
 */
typedef enum
{
    FOC_MODE_STOP = 0,            // 停止模式：电机无输出，处于待机状态
    FOC_MODE_POSITION_MODE=1,     // 位置模式：闭环控制电机到指定角度
    FOC_MODE_SPEED_MODE=2,        // 速度模式：闭环控制电机以指定速度运行    
    FOC_MODE_CALIBRATION=3,       // 校准模式：电机参数自校准（如电角度零点）
    FOC_MOTOR_MODE_HOMING=4,      // 归零模式：位置归零
    FOC_MODE_MIT_MODE=5,          // MIT模式：特定控制算法模式（如阻抗控制）
    FOC_MODE_TORQUE_MODE=6,       // 扭矩模式：闭环控制输出扭矩（扩展）
    FOC_MODE_VMC_MODE=7,          // 虚拟模型控制
    FOC_MODE_MAX                  // 模式数量上限（用于参数校验）
} foc_run_mode_e;


// 归零状态枚举
typedef enum {
  HOMING_STATE_IDLE = 0,
  HOMING_STATE_MOVING = 1,
  HOMING_STATE_ARRIVED = 2,
  HOMING_STATE_ERROR = 3
} HomingState_t;

// 电机归零配置结构体
typedef struct {
  uint8_t motor_id;
  float balance_position;
  float reduction_ratio;
  HomingState_t state;
  uint32_t timer;
  float current_output_angle;
  float target_motor_angle;
  char name[20];
} MotorHoming_t;


/*============================================================================
* 单电机控制结构体（8字节，packed: 无对齐填充）
*============================================================================*/
#pragma pack(push, 1)
typedef struct {
    uint8_t   id;         // 电机ID
    uint8_t   mode;       // 控制模式
    float iq;
    uint16_t  heartbeat;  // 心跳
} motor_single_ctrl_t;
#pragma pack(pop)

/*============================================================================
* 6电机控制结构体（48字节）
*============================================================================*/
typedef struct {
    motor_single_ctrl_t motor[MOTOR_MAX];  // 8个电机
} motor8_ctrl_t;
typedef struct {
    motor_single_ctrl_t motor[4];  // 4个电机
} motor4_ctrl_t;
typedef struct {
    motor_single_ctrl_t motor[2];  // 2个电机
} motor2_ctrl_t;
/*============================================================================
* 单电机状态结构体（17字节，packed: 无对齐填充）
*============================================================================*/
#pragma pack(push, 1)
typedef struct {
    uint8_t   id;         // 电机ID
    uint8_t   mode;       // 运行模式
    float  angle;        // 实际位置
    float   speed;      // 实际速度
    float    iq;         // Iq电流
    uint8_t   fault;      // 故障码
    // uint16_t  reserve;    // 预留对齐字节用的
    uint16_t  heartbeat;  // 心跳
} motor_single_status_t;
#pragma pack(pop)

typedef struct {
    motor_single_status_t motor[MOTOR_MAX];  // 8个电机
} motor8_status_t;
typedef struct {
    motor_single_status_t motor[2];  // 2个电机
} motor2_status_t;
/*============================================================================
* 电机运行时参数
*============================================================================*/
typedef struct {
    uint8_t   mode;       // 当前运行模式
    int32_t   target;     // 目标值
    int16_t   speed;      // 当前速度
    int16_t   angle;      // 当前位置
    float   iq;         // q轴电流
    uint8_t   fault;      // 故障标志
} motor_runtime_t;

/*============================================================================
* 外部全局变量声明
*============================================================================*/
extern volatile uint32_t g_system_tick_ms;              // 系统tick
extern volatile uint32_t g_can0_busoff_count;           // CAN0 bus-off 累计次数 (PSR轮询)
extern volatile uint8_t  g_can0_busoff_flag;            // CAN0 bus-off 标志
extern uint32_t g_can0_recover_time;                    // CAN0 上次恢复时刻(ms)
extern volatile uint32_t g_can1_busoff_count;           // CAN1 bus-off 累计次数
extern volatile uint8_t  g_can1_busoff_flag;            // CAN1 bus-off 标志

extern motor8_status_t   g_motor4_status;       
extern motor_runtime_t    g_motor_runtime[MOTOR_MAX];   // 运行时参数
extern motor8_ctrl_t     g_node0_tx;                   // CAN0发送缓存
extern motor8_ctrl_t     g_node0_rx;                   // CAN0接收缓存
extern motor_single_ctrl_t g_node0_rx_prt;             // CAN0解析后单电机控制
extern motor_single_status_t g_node0_lm_status;
extern motor_single_status_t g_node0_rm_status;
extern motor_single_status_t g_node1_tx_status; 
extern motor_single_status_t g_node0_rb_status;
extern motor_single_status_t g_node0_rf_status;
extern motor_single_status_t g_node0_lf_status;
extern motor_single_status_t g_node0_lb_status;
extern motor8_ctrl_t     g_node1_tx;                     // CAN1发送缓存
extern motor8_ctrl_t     g_node1_rx;                    // CAN1接收缓存
/*============================================================================
* 初始化函数
*============================================================================*/
void CANFD0_Init(void);
void CANFD1_Init(void);
void CANFD0_BusOffRecover(void);    /* CAN0 Bus-Off 恢复 (PSR轮询触发) */
void CANFD0_PollBusOff(void);       /* CAN0 Bus-Off PSR轮询检测 (主循环调用) */
void CANFD0_PollStatusRequest(void);/* CAN0 状态请求轮询 (0x21请求→0x22应答) */
void CANFD1_BusOffRecover(void);    /* CAN1 Bus-Off 恢复 (中断标志触发, 主循环调用) */

/*============================================================================
* 中断服务函数
*============================================================================*/
void Canfd0InterruptHandler(void);
void Canfd1InterruptHandler(void);

/*============================================================================
* 接收回调函数
*============================================================================*/
void CANFD0_RxMsgCallback(bool bRxFifoMsg, uint8_t u8MsgBufOrRxFifoNum, cy_stc_canfd_msg_t* pstcCanFDmsg);
void CANFD1_RxMsgCallback(bool bRxFifoMsg, uint8_t u8MsgBufOrRxFifoNum, cy_stc_canfd_msg_t* pstcCanFDmsg);








/*============================================================================
 * 基础发送（无时间限制 - 底层）
 *============================================================================*/
void CANFD0_TxMsg(uint32_t id, uint32_t dat[], uint8_t len);
void CANFD1_TxMsg(uint32_t id, uint32_t dat[], uint8_t len);

/*============================================================================
 * 限流发送（统一入口）
 * @param intervalMs: 0=不限流立即发送, 其他=最小间隔ms
 * @return: 1=已发送, 0=被限流跳过
 *============================================================================*/
uint8_t CANFD0_TxLimited(uint32_t id, uint32_t* dat, uint8_t len, uint32_t intervalMs);
uint8_t CANFD1_TxLimited(uint32_t id, uint32_t* dat, uint8_t len, uint32_t intervalMs);

/* 兼容旧接口 */
void CANFD0_TxMsgNB(uint32_t id, uint32_t* dat, uint8_t len);
void CANFD1_TxMsgNB(uint32_t id, uint32_t* dat, uint8_t len);

/*============================================================================
 * 状态上报（内部自带20Hz限流）
 *============================================================================*/
void CANFD0_ReportStatus(uint8_t id, uint8_t mode, int16_t speed,
                         int16_t pos, float iq, uint8_t fault);
void CANFD1_ReportStatus(uint8_t id, uint8_t mode, int16_t speed,
                         int16_t pos, float iq, uint8_t fault);

/*============================================================================
 * 电机控制发送
 * @param intervalMs: 0=不限流
 *============================================================================*/
void CANFD0_TxMotorCtrl(uint8_t count, uint32_t intervalMs); /* 逐个电机发送, count=电机个数 */
void CANFD1_Tx4MotorCtrl(uint32_t intervalMs);               /* ★ 改了签名！*/

/* 兼容旧接口（固定限流频率）*/
void CANFD0_Tx2MotorNB(void);    /* 500Hz */
void CANFD0_Tx2Motor(void);      /* 不限流 */
void CANFD0_Tx8Motor(void);      /* 不限流 */

void CANFD1_Tx4MotorNB(void);    /* 100Hz */
void CANFD1_Tx4Motor(void);      /* 不限流 */

/*============================================================================
 * 应用层
 *============================================================================*/
void CANFD0_Tx2WalkMotor(void);     /* 行进电机 500Hz */
void CANFD1_Tx4JointMotor(void);    /* 关节电机 100Hz */
void CANFD_Stop_All_Motors(void);   /* 紧急停止(不限流) */

#endif /* __LQ_CANFD_H__ */
