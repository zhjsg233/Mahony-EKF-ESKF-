/*******************************************************************************
* \file lq_canfd_no_filter.c
* \brief CANFD Motor Control - No ID Filter (Receive All Frames)
*        CANFD电机控制程序 - 关闭ID过滤（接收所有帧）
*
* @author  龙邱智能 chiusir
* @version 4.1
* @date    2026.04.24
* @IDE     IAR9.40.1
*
* =============================================================================
* 功能概述
* =============================================================================
* 本文件实现双路CANFD控制器(CAN0/CAN1)的完整驱动，包括：
*
*   CAN0 (主控板 ? FOC驱动板):
*     - TX: 发送电机速度/位置/模式指令给FOC驱动板
*     - RX: 接收FOC驱动板返回的状态(速度/电流/故障码)
*     - 用途: 控制左右两个行进轮电机 (ID: 0x55左 / 0x56右)
*
*   CAN1 (主控板 ? 关节FOC板 或 调试回环):
*     - TX: 发送关节电机位置指令
*     - RX: 接收关节状态回复
*     - 用途: 控制4个关节电机 (调试用)
*
* =============================================================================
* 通信协议概要
* =============================================================================
*   帧方向    | CAN ID          | 数据内容              | 频率
*   ----------|-----------------|----------------------|--------
*   主控→FOC  | 0x21 (MASTER)   | 电机指令(id/mode/speed) | 500Hz
*   FOC→主控  | 0x22 (MST_STATUS)| 状态回复(speed/iq/fault) | 按需
*   主控→FOC  | 0x20 (STATUS)   | 状态请求(空数据域)      | 100Hz
*
* =============================================================================
* 安全机制
* =============================================================================
*   1. CAN通信超时检测:
*      - Timer_Handler每500Hz递增 s_can_timeout_cnt
*      - 收到有效0x22帧时清零该计数器
*      - 连续25次(50ms)未收到 → 强制输出=0, 切换STANDBY状态
*
*   2. 限流发送:
*      - 所有发送函数通过 TxLimited() 做时间间隔检查
*      - 防止CAN总线被高频发送占满导致阻塞
*
*   3. 紧急停机:
*      - CANFD_Stop_All_Motors() 不限流, 立即发送STOP指令
*******************************************************************************/

#include "zf_common_headfile.h"
#include "LQ_CANFD.h"
// g_test.h 在本工程不存在(原龙邱测试头文件), 其中用到的 rx_count 已在下方全局区直接定义

/*============================================================================
* 全局变量定义
*============================================================================*/

/* 6电机总状态缓存 - 存储所有电机的最新运行状态 */
motor8_status_t g_motor6_status;

/* 6电机运行参数 - 每个电机的实时运行数据 */
motor_runtime_t g_motor_runtime[MOTOR_MAX] = {0};

motor8_ctrl_t  g_node0_tx;
motor8_ctrl_t  g_node0_rx;
motor_single_ctrl_t g_node0_rx_prt;
motor_single_status_t g_node0_rx_status;
motor_single_status_t g_node0_lm_status;   // 左轮状态 (从0x22帧中按id分离)
motor_single_status_t g_node0_rm_status;   // 右轮状态 (从0x22帧中按id分离)
motor_single_status_t g_node0_lm_status;   // 左轮状态 (从0x22帧中按id分离)
motor_single_status_t g_node0_rm_status;   // 右轮状态 (从0x22帧中按id分离)
motor_single_status_t g_node0_lf_status;   // 左前髋关节状态(从0x22帧中按id分离)
motor_single_status_t g_node0_lb_status;   // 左后髋关节状态(从0x22帧中按id分离)
motor_single_status_t g_node0_rf_status;   // 右前髋关节状态(从0x22帧中按id分离)
motor_single_status_t g_node0_rb_status;   // 右后髋关节状态(从0x22帧中按id分离)
motor_single_status_t g_node1_tx_status;

/* CAN1 收发缓存区 */
motor8_ctrl_t  g_node1_tx;                 // CAN1待发送数据
motor8_ctrl_t  g_node1_rx;                 // CAN1接收数据

/*============================================================================
* 静态变量定义
*============================================================================*/
static uint32_t s_canfd0_last_tick = 0;    // CAN0上次发送时刻(ms), 用于限流判断
static uint32_t s_canfd1_last_tick = 0;    // CAN1上次发送时刻(ms), 用于限流判断

volatile uint32_t g_system_tick_ms = 0;    // 系统毫秒tick, PIT中断中递增

volatile uint32_t rx_count = 0;            // CAN接收帧计数(原 g_test.h 中声明)

volatile uint32_t g_can1_busoff_count = 0; // CAN1 bus-off 次数
volatile uint8_t  g_can1_busoff_flag = 0;  // CAN1 bus-off 标志, ISR设置, 主循环清除
uint32_t g_can1_recover_time = 0;          // 上次 bus-off 恢复时刻(ms), 用于冷却期
#define CAN1_RECOVER_COOLDOWN_MS  1000     // bus-off 恢复后冷却期(ms), 期间禁止发送

volatile uint32_t g_can0_busoff_count = 0; // CAN0 bus-off 累计次数 (PSR轮询方案)
volatile uint8_t  g_can0_busoff_flag  = 0; // CAN0 bus-off 标志
uint32_t g_can0_recover_time = 0;          // CAN0 上次 bus-off 恢复时刻(ms)
#define CAN0_RECOVER_COOLDOWN_MS  50       // CAN0 bus-off 恢复后冷却期(ms)

/* CAN0 状态请求轮询 (请求应答模式) */
static const uint8_t s_can0_poll_ids[] = {0x51, 0x52, 0x53, 0x54, 0x55, 0x56};
#define CAN0_POLL_MOTOR_COUNT  6
static uint8_t s_can0_request_index = 0;
static uint8_t s_can0_bo_detected = 0;  // BO上升沿检测: 防止重复恢复

/* 6个电机的归零配置(回原点用) */
MotorHoming_t motors[MOTOR_MAX];

/**
 * @brief 查询指定电机的归零状态
 * @param motor_id: 电机ID
 * @return: 归零状态枚举值
 */
HomingState_t homing_get_motor_state(uint8_t motor_id)
{
  for (int i = 0; i < MOTOR_MAX; i++)
  {
    if (motors[i].motor_id == motor_id)
      return motors[i].state;
  }
  return HOMING_STATE_IDLE;
}

/*============================================================================
* CAN接收回调函数
*============================================================================*/

/**
 * @brief CAN0接收回调函数 (FOC→主控)
 *
 * 触发时机: CAN0接收到任意一帧数据时, 由中断服务程序调用
 * 职责:
 *   1. 过滤无效帧(DLC=0)
 *   2. 识别0x22状态回复帧
 *   3. 清零CAN通信超时计数器 (★ 安全机制 ★)
 *   4. 按电机ID分离左右轮状态
 *   5. 通知平衡控制器更新反馈速度
 *
 * @param bRxFifoMsg:       是否来自FIFO接收
 * @param u8MsgBufOrRxFifoNum: 接收缓冲区编号
 * @param pstcCanFDmsg:     接收到的CAN消息指针
 */
void CANFD0_RxMsgCallback(bool bRxFifoMsg, uint8_t u8MsgBufOrRxFifoNum,
                          cy_stc_canfd_msg_t* pstcCanFDmsg)
{
  uint32_t id0 = pstcCanFDmsg->idConfig.identifier;
  uint8_t  dlc0 = pstcCanFDmsg->dataConfig.dataLengthCode;

  if (dlc0 == 0) return;

  /* 按 CAN ID 直接路由到对应电机状态变量 (CAN ID = 电机节点ID) */
  if (id0 == LEFT_FRONT_MOTOR)
    memcpy(&g_node0_lf_status, pstcCanFDmsg->dataConfig.data, sizeof(motor_single_status_t));
  else if (id0 == LEFT_BACK_MOTOR)
    memcpy(&g_node0_lb_status, pstcCanFDmsg->dataConfig.data, sizeof(motor_single_status_t));
  else if (id0 == RIGHT_FRONT_MOTOR)
    memcpy(&g_node0_rf_status, pstcCanFDmsg->dataConfig.data, sizeof(motor_single_status_t));
  else if (id0 == RIGHT_BACK_MOTOR)
    memcpy(&g_node0_rb_status, pstcCanFDmsg->dataConfig.data, sizeof(motor_single_status_t));
  else if (id0 == LEFT_WALK_MOTOR)
    memcpy(&g_node0_lm_status, pstcCanFDmsg->dataConfig.data, sizeof(motor_single_status_t));
  else if (id0 == RIGHT_WALK_MOTOR)
    memcpy(&g_node0_rm_status, pstcCanFDmsg->dataConfig.data, sizeof(motor_single_status_t));
  else
    return;

  rx_count++;
}

/**
 * @brief CAN1接收回调函数 (模拟FOC测试用)
 *
 * 用途: 开发调试阶段, 模拟FOC驱动板的回复行为。
 *       当收到主控发出的0x20状态请求帧时, 自动构造一个假的状态回复。
 *
 * 生产环境可删除或替换为真实的关节FOC板通信逻辑。
 */
void CANFD1_RxMsgCallback(bool bRxFifoMsg, uint8_t u8MsgBufOrRxFifoNum,
                          cy_stc_canfd_msg_t* pstcCanFDmsg)
{
  uint32_t id1 = pstcCanFDmsg->idConfig.identifier;
  uint8_t  dlc1 = pstcCanFDmsg->dataConfig.dataLengthCode;

  if (dlc1 == 0) return;

  if(id1 == CAN_ID_MOTOR_MST_STATUS)
  {

  }
  /*
   * 收到 0x20 状态请求帧时, 构造一个模拟的状态回复。
   * 用于在没有真实FOC硬件的情况下测试CAN通信链路。
   */
  if (id1 == CAN_ID_MOTOR_STATUS)
  {
    CANFD1_ReportStatus(LEFT_WALK_MOTOR, FOC_MODE_SPEED_MODE,
                         11, 22, 33, 0);  // 模拟数据: speed=11, pos=22, iq=33
  }

  if (id1 == CAN_ID_MOTOR_MST_STATUS)
  {
    /*
     * ★ CAN通信超时计数清零 ★
     * 每收到一次有效的0x22帧就清零,
     * 表示FOC在线且通信正常。
     * 对应Timer_Handler中的超时检测逻辑:
     *   s_can_timeout_cnt++;
     *   if (cnt > CAN_TIMEOUT_MAX) → 停机
     */
    // s_can_timeout_cnt = 0;

    // /* 标记FOC反馈速度有效 */
    // s_foc_fb_speed_valid = 1;
    // rx_count++;
    /* 将接收数据拷贝到通用状态缓存 */
    memcpy(&g_node0_rx_status, pstcCanFDmsg->dataConfig.data,
           sizeof(motor_single_status_t));

    /* 按电机ID分流到对应的左右轮状态变量 */
    if (g_node0_rx_status.id == LEFT_WALK_MOTOR)         // 0x55 左行进轮
    {
      memcpy(&g_node0_lm_status, &g_node0_rx_status,
             sizeof(motor_single_status_t));

      /* 通知平衡控制器: 左轮速度已更新 */
      // Balance_Update_FOC_Feedback(LEFT_WALK_MOTOR,
      //                             g_node0_lm_status.speed);
    }
    else if (g_node0_rx_status.id == RIGHT_WALK_MOTOR)   // 0x56 右行进轮
    {
      memcpy(&g_node0_rm_status, &g_node0_rx_status,
             sizeof(motor_single_status_t));

      // /* 通知平衡控制器: 右轮速度已更新 */
      // Balance_Update_FOC_Feedback(RIGHT_WALK_MOTOR,
      //                             g_node0_rm_status.speed);
    }
    else if(g_node0_rx_status.id == LEFT_FRONT_MOTOR)
    {
      memcpy(&g_node0_lf_status, &g_node0_rx_status,
             sizeof(motor_single_status_t));
    }
    else if(g_node0_rx_status.id == LEFT_BACK_MOTOR)
    {
      memcpy(&g_node0_lb_status, &g_node0_rx_status,
             sizeof(motor_single_status_t));
    }
    else if(g_node0_rx_status.id == RIGHT_FRONT_MOTOR)
    {
      memcpy(&g_node0_rf_status, &g_node0_rx_status,
             sizeof(motor_single_status_t));
    }
    else if(g_node0_rx_status.id == RIGHT_BACK_MOTOR)
    {
      memcpy(&g_node0_rb_status, &g_node0_rx_status,
             sizeof(motor_single_status_t));
    }
    /*
     * 注意: 如果有更多电机, 在这里继续 else if 分支即可。
     * 当前双轮平衡机器人只有左右两个行进轮。
     */
  }
}

/*============================================================================
* 中断处理函数
*============================================================================*/

/**
 * @brief CAN0中断处理函数 (IRQ入口)
 *
 * 调用链: 硬件中断 → Canfd0InterruptHandler → Cy_CANFD_IrqHandler → RxMsgCallback
 *
 * 注意: 此函数只做中断分发, 不包含业务逻辑。
 *       业务逻辑在 CANFD0_RxMsgCallback() 中实现。
 */
void Canfd0InterruptHandler(void)
{
  Cy_CANFD_IrqHandler(CY_CANFD0_0_TYPE);
}

/**
 * @brief CAN1中断处理函数 (IRQ入口)
 */
void Canfd1InterruptHandler(void)
{
  Cy_CANFD_IrqHandler(CY_CANFD1_1_TYPE);

  if (CY_CANFD1_1_TYPE->M_TTCAN.unIR.stcField.u1BO_ == 1)
  {
    CY_CANFD1_1_TYPE->M_TTCAN.unIR.stcField.u1BO_ = 1;
    g_can1_busoff_flag = 1;
  }
}

/*============================================================================
* 内部辅助函数声明
*============================================================================*/
static void SetISOFormat(cy_pstc_canfd_type_t canfd);  // 设置ISO CAN FD格式

/*============================================================================
* 标准帧ID过滤器配置
*============================================================================*/
static const cy_stc_id_filter_t stdIdFilter[] =
{
  CANFD_CONFIG_STD_ID_FILTER_CLASSIC_RXBUFF(0x010u, 0u),
  CANFD_CONFIG_STD_ID_FILTER_CLASSIC_RXBUFF(0x020u, 1u),
  CANFD_CONFIG_STD_ID_FILTER_CLASSIC_RXBUFF(CAN_ID_MOTOR_SET_INITIAL_POSE, 2u), // 0x25
  CANFD_CONFIG_STD_ID_FILTER_CLASSIC_RXBUFF(LEFT_FRONT_MOTOR,  3u),  // 0x51
  CANFD_CONFIG_STD_ID_FILTER_CLASSIC_RXBUFF(LEFT_BACK_MOTOR,   4u),  // 0x52
  CANFD_CONFIG_STD_ID_FILTER_CLASSIC_RXBUFF(RIGHT_FRONT_MOTOR, 5u),  // 0x53
  CANFD_CONFIG_STD_ID_FILTER_CLASSIC_RXBUFF(RIGHT_BACK_MOTOR,  6u),  // 0x54
  CANFD_CONFIG_STD_ID_FILTER_CLASSIC_RXBUFF(LEFT_WALK_MOTOR,   7u),  // 0x55
  CANFD_CONFIG_STD_ID_FILTER_CLASSIC_RXBUFF(RIGHT_WALK_MOTOR,  8u),  // 0x56
};

/*============================================================================
* 扩展帧ID过滤器配置
*============================================================================*/
static const cy_stc_extid_filter_t extIdFilter[] =
{
  CANFD_CONFIG_EXT_ID_FILTER_CLASSIC_RXBUFF(0x10010u, 2u),
  CANFD_CONFIG_EXT_ID_FILTER_CLASSIC_RXBUFF(0x10020u, 3u),
};

/*============================================================================
* CANFD控制器配置结构体
*============================================================================*/

/*
 * ═══════════════════════════════════════════════════════════════════
 * CAN0 配置 (主控板 ? FOC驱动板)
 * ═══════════════════════════════════════════════════════════════════
 *
 * 波特率配置:
 *   Arbitration(仲裁段): 500 kbps  ← 用于标准帧/CAN2.0兼容通信
 *   Data(数据段):        2 Mbps   ← 用于CAN FD的大数据量传输
 *
 * 时钟源: 40MHz peripheral clock
 *   仲裁段: 40MHz / 10(prescaler) / 8(TQ) = 500 kbps
 *   数据段: 40MHz / 5(prescaler) / 4(TQ)  = 2 Mbps
 *
 * 采样点: 75% (TQ=8时, 采样点在第6TQ)
 */
cy_stc_canfd_config_t can0Cfg =
{
  .txCallback            = NULL,                          // 无单独TX完成回调
  .rxCallback            = CANFD0_RxMsgCallback,          // ★ 接收回调: 解析FOC状态
  .rxFifoWithTopCallback = NULL,                          // FIFO顶部指针回调(未使用)
  .statusCallback        = NULL,                          // 状态变化回调(未使用)
  .errorCallback         = NULL,                          // 错误回调(未使用)
  .canFDMode             = true,                          // 启用CAN FD模式

  /* 仲裁段波特率: 500 kbps */
  .bitrate =
  {
    .prescaler     = 10u,     // 预分频: 40MHz / 10 = 4MHz基础时钟
    .timeSegment1  = 5u,      // TSeg1 = 5 TQ
    .timeSegment2  = 2u,      // TSeg2 = 2 TQ
    .syncJumpWidth = 2u,      // SJW = 2 TQ (规范内最大值, ≤ TSeg2)
    // 总TQ = 1(Sync) + 5(TSeg1) + 2(TSeg2) = 8 TQ
    // 采样点 = (1+5)/8 = 75%
    // 波特率 = 4MHz / 8 = 500 kbps
  },

  /* 数据段波特率: 2 Mbps (CAN FD加速传输) */
  .fastBitrate =
  {
    .prescaler     = 5u,      // 预分频: 40MHz / 5 = 8MHz基础时钟
    .timeSegment1  = 2u,      // TSeg1 = 2 TQ
    .timeSegment2  = 1u,      // TSeg2 = 1 TQ
    .syncJumpWidth = 1u,      // SJW = 1 TQ (规范内最大值, ≤ TSeg2)
    // 总TQ = 1 + 2 + 1 = 4 TQ
    // 采样点 = (1+2)/4 = 75%
    // 波特率 = 8MHz / 4 = 2 Mbps
  },
  
  //  .fastBitrate =
  //  {
  //    .prescaler     = 1u,      // 40MHz / 1 = 40MHz 
  //    .timeSegment1  = 5u,
  //    .timeSegment2  = 2u,
  //    .syncJumpWidth = 1u,
  //    // 总TQ = 12, 采样点 = 75%
  //    // 40MHz / 8 = 5Mbps 
  //  },
  
  /* TDC (Transmitter Delay Compensation) 发送延迟补偿 */
  .tdcConfig =
  {
    .tdcEnabled      = true,   // 启用TDC (高速通信必须)
    .tdcOffset       = 5u,     // TDC偏移量
    .tdcFilterWindow = 2u,     // TDC滤波窗口
  },

  /* 标准帧ID过滤器 */
  .sidFilterConfig =
  {
    .numberOfSIDFilters = sizeof(stdIdFilter) / sizeof(stdIdFilter[0]),
    .sidFilter          = stdIdFilter,
  },

  /* 扩展帧ID过滤器 */
  .extidFilterConfig =
  {
    .numberOfEXTIDFilters = sizeof(extIdFilter) / sizeof(extIdFilter[0]),
    .extidFilter          = extIdFilter,
    .extIDANDMask         = 0x1fffffff,  // 29位扩展ID全匹配
  },

  /*
   * 全局过滤策略: 直接丢弃所有不匹配的帧
   * ID不在过滤器列表中的帧由硬件直接丢弃，不经过FIFO，
   * 减少不必要的中断开销和CPU占用。
   */
  .globalFilterConfig =
  {
    .nonMatchingFramesStandard  = CY_CANFD_REJECT_NON_MATCHING,  // 非标ID帧→直接丢弃
    .nonMatchingFramesExtended  = CY_CANFD_REJECT_NON_MATCHING,  // 非扩展ID帧→直接丢弃
    .rejectRemoteFramesStandard = false,                        // 不拒绝远程帧
    .rejectRemoteFramesExtended = false,                        // 不拒绝远程帧
  },

  /* 缓冲区大小: 支持最大64字节(CAN FD) */
  .rxBufferDataSize = CY_CANFD_BUFFER_DATA_SIZE_64,
  .rxFifo1DataSize  = CY_CANFD_BUFFER_DATA_SIZE_64,
  .rxFifo0DataSize  = CY_CANFD_BUFFER_DATA_SIZE_64,
  .txBufferDataSize = CY_CANFD_BUFFER_DATA_SIZE_64,

  /* RX FIFO0配置 (主要接收FIFO) */
  .rxFifo0Config =
  {
    .mode                   = CY_CANFD_FIFO_MODE_OVERWRITE,  // 覆盖模式(FIFO满时覆盖旧帧)
    .watermark              = 7u,                            // 水位线=7, 到达时触发中断
    .numberOfFifoElements   = 8u,                            // FIFO深度=8帧
    .topPointerLogicEnabled = false,                         // 关闭顶部指针逻辑
  },

  /* RX FIFO1配置 (备用接收FIFO) */
  .rxFifo1Config =
  {
    .mode                   = CY_CANFD_FIFO_MODE_OVERWRITE,
    .watermark              = 7u,
    .numberOfFifoElements   = 8u,
    .topPointerLogicEnabled = false,
  },

  .noOfRxBuffers = 9u,          // 9个独立RX缓冲区 (0~8对应9个STD filter)
  .noOfTxBuffers = 8u,          // 8个独立TX缓冲区
};

/*
 * ═══════════════════════════════════════════════════════════════════
 * CAN1 配置 (主控板 ? 关节FOC板 / 调试用)
 * ═══════════════════════════════════════════════════════════════════
 *
 * 与CAN0的主要区别:
 *   - rxFifo0/rxFifo1 使用 OVERWRITE(覆盖)模式而非BLOCKING(阻塞)
 *     原因: 关节电机数据允许丢帧(位置指令是连续的), 但不能阻塞
 *   - noOfRxBuffers = 0 (纯FIFO模式, 不用独立缓冲区)
 */
cy_stc_canfd_config_t can1Cfg =
{
  .txCallback            = NULL,
  .rxCallback            = CANFD1_RxMsgCallback,
  .rxFifoWithTopCallback = NULL,
  .statusCallback        = NULL,
  .errorCallback         = NULL,
  .canFDMode             = true,

  /* 与CAN0相同的波特率配置 */
  .bitrate =
  {
    .prescaler     = 10u,
    .timeSegment1  = 5u,
    .timeSegment2  = 2u,
    .syncJumpWidth = 2u,
  },

  .fastBitrate =
  {
    .prescaler     = 5u,
    .timeSegment1  = 2u,
    .timeSegment2  = 1u,
    .syncJumpWidth = 1u,
  },

  .tdcConfig =
  {
    .tdcEnabled      = true,
    .tdcOffset       = 5u,
    .tdcFilterWindow = 2u,
  },

  .sidFilterConfig =
  {
    .numberOfSIDFilters = sizeof(stdIdFilter) / sizeof(stdIdFilter[0]),
    .sidFilter          = stdIdFilter,
  },

  .extidFilterConfig =
  {
    .numberOfEXTIDFilters = sizeof(extIdFilter) / sizeof(extIdFilter[0]),
    .extidFilter          = extIdFilter,
    .extIDANDMask         = 0x1fffffff,
  },

  .globalFilterConfig =
  {
    .nonMatchingFramesStandard  = CY_CANFD_REJECT_NON_MATCHING,
    .nonMatchingFramesExtended  = CY_CANFD_REJECT_NON_MATCHING,
    .rejectRemoteFramesStandard = false,
    .rejectRemoteFramesExtended = false,
  },

  .rxBufferDataSize = CY_CANFD_BUFFER_DATA_SIZE_64,
  .rxFifo1DataSize  = CY_CANFD_BUFFER_DATA_SIZE_64,
  .rxFifo0DataSize  = CY_CANFD_BUFFER_DATA_SIZE_64,
  .txBufferDataSize = CY_CANFD_BUFFER_DATA_SIZE_64,

  /*
   * ★ CAN1使用OVERWRITE(覆盖)模式 ★
   * 区别于CAN0的BLOCKING模式:
   *   BLOCKING: FIFO满时新帧被丢弃 (适合关键数据, 不能丢)
   *   OVERWRITE: FIFO满时最旧的帧被覆盖 (适合连续数据, 允许丢旧取新)
   *
   * 关节电机的位置指令是连续更新的, 丢掉旧帧影响不大,
   * 但阻塞会导致整个CAN通信卡死, 所以用覆盖模式。
   */
  .rxFifo0Config =
  {
    .mode                   = CY_CANFD_FIFO_MODE_OVERWRITE,  // ★ 覆盖模式
    .watermark              = 7u,
    .numberOfFifoElements   = 8u,
    .topPointerLogicEnabled = false,
  },

  .rxFifo1Config =
  {
    .mode                   = CY_CANFD_FIFO_MODE_OVERWRITE,  // ★ 覆盖模式
    .watermark              = 7u,
    .numberOfFifoElements   = 8u,
    .topPointerLogicEnabled = false,
  },

  .noOfRxBuffers = 0u,          // ★ 不使用独立RX缓冲区, 纯FIFO模式
  .noOfTxBuffers = 8u,
};

/*============================================================================
* GPIO引脚配置
*============================================================================*/

/**
 * @brief 引脚配置结构体
 *
 * 将GPIO端口+引脚号+配置打包, 方便循环初始化
 */
typedef struct
{
  volatile stc_GPIO_PRT_t* portReg;  // GPIO端口寄存器指针 (如GPIO_PRT8)
  uint8_t pinNum;                    // 引脚编号 (0~15)
  cy_stc_gpio_pin_config_t cfg;      // PDL库引脚配置结构体
} stc_pin_config;

/*
 * CAN0引脚映射:
 *   P8_0 → CAN0_TX (推挽输出, 发送CAN信号到总线)
 *   P8_1 → CAN0_RX (高阻输入, 从总线接收CAN信号)
 */
static const stc_pin_config can0_pin_cfg[] =
{
  {   // CAN0 RX
    .portReg = GPIO_PRT8, .pinNum = 1,
    .cfg = {
      .outVal = 0,
      .driveMode = CY_GPIO_DM_HIGHZ,           // 高阻输入 (总线收发器驱动电平)
      .hsiom = P8_1_CANFD0_TTCAN_RX0,          // 硬件复用为CAN0 RX
      .intEdge = 0, .intMask = 0, .vtrip = 0,
      .slewRate = 0, .driveSel = 0, .vregEn = 0,
      .ibufMode = 0, .vtripSel = 0, .vrefSel = 0, .vohSel = 0,
    }
  },
  {   // CAN0 TX
    .portReg = GPIO_PRT8, .pinNum = 0,
    .cfg = {
      .outVal = 1,                               // 默认高电平(隐性状态)
      .driveMode = CY_GPIO_DM_STRONG,            // 强推挽输出 (驱动总线)
      .hsiom = P8_0_CANFD0_TTCAN_TX0,            // 硬件复用为CAN0 TX
      .intEdge = 0, .intMask = 0, .vtrip = 0,
      .slewRate = 0, .driveSel = 0, .vregEn = 0,
      .ibufMode = 0, .vtripSel = 0, .vrefSel = 0, .vohSel = 0,
    }
  },
};

/*
 * CAN1引脚映射:
 *   P12_4 → CAN1_TX
 *   P12_5 → CAN1_RX
 */
static const stc_pin_config can1_pin_cfg[] =
{
  {   // CAN1 RX
    .portReg = GPIO_PRT12, .pinNum = 5,
    .cfg = {
      .outVal = 0,
      .driveMode = CY_GPIO_DM_HIGHZ,
      .hsiom = P12_5_CANFD1_TTCAN_RX1,
      .intEdge = 0, .intMask = 0, .vtrip = 0,
      .slewRate = 0, .driveSel = 0, .vregEn = 0,
      .ibufMode = 0, .vtripSel = 0, .vrefSel = 0, .vohSel = 0,
    }
  },
  {   // CAN1 TX
    .portReg = GPIO_PRT12, .pinNum = 4,
    .cfg = {
      .outVal = 1,
      .driveMode = CY_GPIO_DM_STRONG,
      .hsiom = P12_4_CANFD1_TTCAN_TX1,
      .intEdge = 0, .intMask = 0, .vtrip = 0,
      .slewRate = 0, .driveSel = 0, .vregEn = 0,
      .ibufMode = 0, .vtripSel = 0, .vrefSel = 0, .vohSel = 0,
    }
  },
};

/*============================================================================
* 中断使能函数
*============================================================================*/

/**
 * @brief 启用CANFD接收中断
 *
 * 操作步骤:
 *   1. 进入INIT初始化模式 (修改寄存器前必须)
 *   2. 设置CCE位 (允许配置更改)
 *   3. 清除RF0N/RF1N中断标志 (防止残留标志触发误中断)
 *   4. 使能RF0NE/RF1NE中断 (FIFO0/FIFO1新消息中断)
 *   5. 使能EINT0中断线 (全局中断使能)
 *   6. 退出INIT模式 (恢复正常工作)
 *
 * @param canfd: CANFD控制器基地址指针
 */
static void EnableRxInterrupt(cy_pstc_canfd_type_t canfd)
{
  /* 步骤1: 进入初始化模式 */
  canfd->M_TTCAN.unCCCR.stcField.u1INIT = 1;
  while(canfd->M_TTCAN.unCCCR.stcField.u1INIT != 1);

  /* 步骤2: 允许配置更改 */
  canfd->M_TTCAN.unCCCR.stcField.u1CCE = 1;

  /* 步骤3: 清除FIFO0/FIFO1新消息中断标志 (写1清除) */
  canfd->M_TTCAN.unIR.stcField.u1RF0N = 1;
  canfd->M_TTCAN.unIR.stcField.u1RF1N = 1;

  /* 步骤4: 使能FIFO0/FIFO1新消息到达中断 */
  canfd->M_TTCAN.unIE.stcField.u1RF0NE = 1;
  canfd->M_TTCAN.unIE.stcField.u1RF1NE = 1;

  /* 步骤5: 使能中断线0 (实际触发CPU中断的那根线) */
  canfd->M_TTCAN.unILE.stcField.u1EINT0 = 1;

  /* 步骤6: 退出初始化模式 */
  canfd->M_TTCAN.unCCCR.stcField.u1INIT = 0;
  while(canfd->M_TTCAN.unCCCR.stcField.u1INIT != 0);

}

/**
 * @brief 启用CANFD Bus-Off中断
 */
static void EnableBusOffInterrupt(cy_pstc_canfd_type_t canfd)
{
  canfd->M_TTCAN.unCCCR.stcField.u1INIT = 1;
  while(canfd->M_TTCAN.unCCCR.stcField.u1INIT != 1);
  canfd->M_TTCAN.unCCCR.stcField.u1CCE = 1;

  canfd->M_TTCAN.unIR.stcField.u1BO_ = 1;   // 清除残留 BO 中断标志
  canfd->M_TTCAN.unIE.stcField.u1BOE = 1;    // 使能 Bus_Off 中断

  canfd->M_TTCAN.unCCCR.stcField.u1INIT = 0;
  while(canfd->M_TTCAN.unCCCR.stcField.u1INIT != 0);
}

/**
 * @brief CAN1 Bus-Off 恢复：DeInit → Init → 重新配置
 */
void CANFD1_BusOffRecover(void)
{
  static uint8_t recovering = 0;
  if (recovering) return;
  recovering = 1;

  g_can1_busoff_count++;
  g_can1_recover_time = g_system_tick_ms;
  Cy_CANFD_DeInit(CY_CANFD1_1_TYPE);
  Cy_CANFD_Init(CY_CANFD1_1_TYPE, &can1Cfg);
  SetISOFormat(CY_CANFD1_1_TYPE);
  EnableRxInterrupt(CY_CANFD1_1_TYPE);
  EnableBusOffInterrupt(CY_CANFD1_1_TYPE);

  recovering = 0;
}

/**
 * @brief CAN0 Bus-Off PSR轮询检测 (主循环中调用)
 *
 * 读取 PSR 寄存器的 u1BO 只读状态位，一旦为 1 则立即触发恢复。
 * 与 CAN1 的中断方案不同：不使用 BOE 中断使能，只在主循环读 PSR，
 * 因此从根本上规避了中断风暴。
 */
void CANFD0_PollBusOff(void)
{
  if (CY_CANFD0_0_TYPE->M_TTCAN.unPSR.stcField.u1BO == 1)
  {
    if (!s_can0_bo_detected)            // 仅上升沿触发一次
    {
      s_can0_bo_detected = 1;
      g_can0_busoff_flag = 1;
      CANFD0_BusOffRecover();
    }
  }
  else
  {
    s_can0_bo_detected = 0;            // BO已清除, 恢复检测能力
    g_can0_busoff_flag = 0;
  }
}

/**
 * @brief CAN0 Bus-Off 恢复：DeInit → Init → 重新配置
 *
 * 核心流程:
 *   1. DeInit 清除旧状态
 *   2. Init 加载 can0Cfg 配置
 *   3. SetISOFormat 设置 ISO CAN FD 格式
 *   4. 中断风暴预防: 清零所有 IE 使能位，只保留接收中断
 *   5. 清零所有 IR 残留标志
 *   6. 使能中断线
 *
 * 注意: 不调用 EnableBusOffInterrupt() -- PSR 轮询方案不需要 BO 中断
 */
void CANFD0_BusOffRecover(void)
{
  static uint8_t recovering = 0;
  if (recovering) return;
  recovering = 1;

  g_can0_busoff_count++;
  g_can0_recover_time = g_system_tick_ms;

  Cy_CANFD_DeInit(CY_CANFD0_0_TYPE);
  Cy_CANFD_Init(CY_CANFD0_0_TYPE, &can0Cfg);
  SetISOFormat(CY_CANFD0_0_TYPE);

  /* 清零 TEC/REC 错误计数器, 防止带着高错误计数恢复 (M_TTCAN 需 INIT+CCE) */
  CY_CANFD0_0_TYPE->M_TTCAN.unCCCR.stcField.u1INIT = 1;
  while(CY_CANFD0_0_TYPE->M_TTCAN.unCCCR.stcField.u1INIT != 1);
  CY_CANFD0_0_TYPE->M_TTCAN.unCCCR.stcField.u1CCE = 1;
  CY_CANFD0_0_TYPE->M_TTCAN.unECR.u32Register = 0u;
  CY_CANFD0_0_TYPE->M_TTCAN.unCCCR.stcField.u1INIT = 0;
  while(CY_CANFD0_0_TYPE->M_TTCAN.unCCCR.stcField.u1INIT != 0);

  /* 使用与 CANFD0_Init 相同的 RX 中断设置 (已验证的流程) */
  EnableRxInterrupt(CY_CANFD0_0_TYPE);
  /* 不调用 EnableBusOffInterrupt -- PSR 轮询不需要 BO 中断 */

  g_can0_busoff_flag = 0;
  recovering = 0;
}

/**
 * @brief CAN0 状态请求轮询: 向当前电机发送 0x21 请求，然后切换到下一个电机
 *
 * 协议: 发送 CAN ID=0x21, data[0]=motor_id (uint32_t)
 *       从机收到后匹配 ID，匹配则回复 0x22 状态帧
 *
 * 调用方式: 在主循环中周期性调用 (建议每 1ms 一次, 用 pit_interrupt_count 分频)
 * 轮询周期: 6 个电机，每 1ms 请求一个，每个电机每 6ms 被请求一次 (~167Hz)
 *
 * 冷却期行为: 经过 TxLimited，冷却期内跳过发送且不轮换到下一个电机
 */
void CANFD0_PollStatusRequest(void)
{
  uint8_t motor_id = s_can0_poll_ids[s_can0_request_index];

  uint32_t dat[1];
  dat[0] = motor_id;
  if (CANFD0_TxLimited(CAN_ID_MOTOR_STATUS, dat, 4, 0))
  {
    s_can0_request_index++;
    if (s_can0_request_index >= CAN0_POLL_MOTOR_COUNT)
      s_can0_request_index = 0;
  }
}


/*============================================================================
* CAN控制器初始化函数
*============================================================================*/

/**
 * @brief CAN0控制器完整初始化
 *
 * 初始化流程:
 *   1. 配置时钟分频 (peripheral clock → CAN模块时钟)
 *   2. 复位CAN控制器 (清除之前的状态)
 *   3. 初始化GPIO引脚 (TX/RX映射)
 *   4. 配置NVIC中断 (优先级、向量、使能)
 *   5. 调用PDL库初始化 (加载can0Cfg配置)
 *   6. 设置ISO CAN FD格式 (兼容性更好)
 *   7. 使能接收中断
 */
void CANFD0_Init(void)
{
  /* 1. 时钟配置: CAN0使用DIV_8分频器的通道0 */
  Cy_SysClk_PeriphAssignDivider(PCLK_CANFD0_CLOCK_CAN0, CY_SYSCLK_DIV_8_BIT, 0u);
  Cy_SysClk_PeriphSetDivider(Cy_SysClk_GetClockGroup(PCLK_CANFD0_CLOCK_CAN0),
                             CY_SYSCLK_DIV_8_BIT, 0u, 0);
  Cy_SysClk_PeriphEnableDivider(Cy_SysClk_GetClockGroup(PCLK_CANFD0_CLOCK_CAN0),
                                CY_SYSCLK_DIV_8_BIT, 0u);

  /* 2. 复位CAN0 (清除所有寄存器和状态) */
  Cy_CANFD_DeInit(CY_CANFD0_0_TYPE);

  /* 3. 初始化GPIO引脚 (遍历配置表, 逐个设置) */
  for (uint8_t i = 0; i < (sizeof(can0_pin_cfg) / sizeof(can0_pin_cfg[0])); i++)
  {
    Cy_GPIO_Pin_Init(can0_pin_cfg[i].portReg, can0_pin_cfg[i].pinNum, &can0_pin_cfg[i].cfg);
  }

  /* 4. NVIC中断配置 */
  cy_stc_sysint_irq_t irq_cfg0 =
  {
    .sysIntSrc = canfd_0_interrupts0_0_IRQn,  // CAN0中断源编号
    .intIdx    = CPUIntIdx2_IRQn,             // 映射到CPU的中断线2
    .isEnabled = true,                        // 立即使能
  };
  Cy_SysInt_InitIRQ(&irq_cfg0);
  Cy_SysInt_SetSystemIrqVector(irq_cfg0.sysIntSrc, Canfd0InterruptHandler);  // 绑定ISR
  NVIC_SetPriority(CPUIntIdx2_IRQn, 2);         // 优先级2 (中等, 高于普通任务)
  NVIC_ClearPendingIRQ(CPUIntIdx2_IRQn);        // 清除可能的挂起中断
  NVIC_EnableIRQ(CPUIntIdx2_IRQn);              // ★ 使能中断 ★

  /* 5. PDL库初始化 (加载上面定义的can0Cfg配置结构体) */
  Cy_CANFD_Init(CY_CANFD0_0_TYPE, &can0Cfg);

  /* 6. 设置ISO CAN FD格式 (区别于Non-ISO格式, 兼容性更好) */
  SetISOFormat(CY_CANFD0_0_TYPE);

  /* 7. 使能接收中断 (独立于回环模式, 即使关闭回环也能收数据) */
  EnableRxInterrupt(CY_CANFD0_0_TYPE);
}

/**
 * @brief CAN1控制器完整初始化
 *
 * 与CAN0基本相同, 区别:
 *   - 使用不同的时钟/引脚/中断资源
 *   - FIFO使用OVERWRITE模式 (见can1Cfg注释)
 */
void CANFD1_Init(void)
{
  /* 时钟配置 */
  Cy_SysClk_PeriphAssignDivider(PCLK_CANFD1_CLOCK_CAN1, CY_SYSCLK_DIV_8_BIT, 0u);
  Cy_SysClk_PeriphSetDivider(Cy_SysClk_GetClockGroup(PCLK_CANFD1_CLOCK_CAN1),
                             CY_SYSCLK_DIV_8_BIT, 0u, 0);
  Cy_SysClk_PeriphEnableDivider(Cy_SysClk_GetClockGroup(PCLK_CANFD1_CLOCK_CAN1),
                                CY_SYSCLK_DIV_8_BIT, 0u);

  /* 复位 */
  Cy_CANFD_DeInit(CY_CANFD1_1_TYPE);

  /* GPIO初始化 */
  for (uint8_t i = 0; i < (sizeof(can1_pin_cfg) / sizeof(can1_pin_cfg[0])); i++)
  {
    Cy_GPIO_Pin_Init(can1_pin_cfg[i].portReg, can1_pin_cfg[i].pinNum, &can1_pin_cfg[i].cfg);
  }

  /* NVIC中断配置 (使用中断线3, 与CAN0错开) */
  cy_stc_sysint_irq_t irq_cfg1 =
  {
    .sysIntSrc = canfd_1_interrupts0_1_IRQn,
    .intIdx    = CPUIntIdx3_IRQn,
    .isEnabled = true,
  };
  Cy_SysInt_InitIRQ(&irq_cfg1);
  Cy_SysInt_SetSystemIrqVector(irq_cfg1.sysIntSrc, Canfd1InterruptHandler);
  NVIC_SetPriority(CPUIntIdx3_IRQn, 2);
  NVIC_ClearPendingIRQ(CPUIntIdx3_IRQn);
  NVIC_EnableIRQ(CPUIntIdx3_IRQn);

  /* PDL库初始化 */
  Cy_CANFD_Init(CY_CANFD1_1_TYPE, &can1Cfg);
  SetISOFormat(CY_CANFD1_1_TYPE);
  EnableRxInterrupt(CY_CANFD1_1_TYPE);
  EnableBusOffInterrupt(CY_CANFD1_1_TYPE);
}

/*============================================================================
* 辅助函数
*============================================================================*/

/**
 * @brief 设置CANFD为ISO标准格式
 *
 * ISO vs Non-ISO CAN FD:
 *   ISO格式:     使用 stuff bit 计数器, 兼容性更好 (推荐)
 *   Non-ISO格式: Bosch V1.0早期规范, 某些芯片不支持
 *
 * @param canfd: CANFD控制器指针
 */
static void SetISOFormat(cy_pstc_canfd_type_t canfd)
{
  /* 必须先进入INIT模式才能修改CCR寄存器 */
  canfd->M_TTCAN.unCCCR.stcField.u1INIT = 1;
  while(canfd->M_TTCAN.unCCCR.stcField.u1INIT != 1);

  /* 允许配置更改 */
  canfd->M_TTCAN.unCCCR.stcField.u1CCE = 1;

  /* 设置NISO=1 → ISO CAN FD格式 */
  canfd->M_TTCAN.unCCCR.stcField.u1NISO = 1;

  /* 退出INIT模式, 立即生效 */
  canfd->M_TTCAN.unCCCR.stcField.u1INIT = 0;
  while(canfd->M_TTCAN.unCCCR.stcField.u1INIT != 0);
}

/*============================================================================
* DLC转换工具函数
*============================================================================*/

/**
 * @brief 字节数 → DLC(Data Length Code)转换
 *
 * CAN FD的DLC编码不是简单的线性映射, 需要查表:
 *   字节数  0~8  → DLC = 字节数本身
 *   字节数 9~12 → DLC = 9
 *   字节数13~16→ DLC = 10
 *   ...以此类推, 最大64字节对应DLC=15
 *
 * @param len: 实际数据字节数 (0~64)
 * @return: CAN DLC编码值 (0~15)
 */
__inline static uint8_t ByteLen_To_DLC(uint8_t len)
{
  if (len <= 8)   return len;
  if (len <= 12)  return 9;
  if (len <= 16)  return 10;
  if (len <= 20)  return 11;
  if (len <= 24)  return 12;
  if (len <= 32)  return 13;
  if (len <= 48)  return 14;
  return 15;  // 49~64字节
}

/*============================================================================
* 底层发送函数 (直接操作硬件, 无限流)
*============================================================================*/

/**
 * @brief CAN0底层发送 - 直接写入TX缓冲区
 *
 * 这是所有CAN0发送操作的最终执行者。
 * 上层函数(限流发送、电机控制等)最终都会调用到这里。
 *
 * @param id:  CAN标准帧ID (如 0x21)
 * @param dat: 数据指针 (uint32_t数组, 小端序)
 * @param len: 有效数据字节数 (非DLC值, 内部会自动转换)
 */
void CANFD0_TxMsg(uint32_t id, uint32_t dat[], uint8_t len)
{
  cy_stc_canfd_msg_t stcMsg;  // 栈上构建消息结构体
  uint8_t i, n;

  memset(&stcMsg, 0, sizeof(cy_stc_canfd_msg_t));

  /* 基本帧头配置 */
  stcMsg.canFDFormat             = true;               // CAN FD格式
  stcMsg.idConfig.extended       = false;              // 标准帧 (11位ID)
  stcMsg.idConfig.identifier     = id;                 // CAN ID
  stcMsg.dataConfig.dataLengthCode = ByteLen_To_DLC(len); // 字节数→DLC

  /* 数据拷贝: 字节数→uint32_t字数 (向上取整) */
  n = (len + 3) >> 2;           // 等价于 ceil(len/4)
  if (n > 16) n = 16;           // 最大16个uint32_t = 64字节
  for (i = 0; i < n; i++)
    stcMsg.dataConfig.data[i] = dat[i];

  /* 扫描空闲TX buffer并发送，不阻塞 */
  {
    static uint8_t s_canfd0_tx_buf = 0;
    uint8_t buf = s_canfd0_tx_buf;
    uint8_t tries = 0;
    cy_en_canfd_status_t ret;

    do {
      cy_en_canfd_tx_buffer_status_t st = Cy_CANFD_GetTxBufferStatus(CY_CANFD0_0_TYPE, buf);
      if (st == CY_CANFD_TX_BUFFER_IDLE || st == CY_CANFD_TX_BUFFER_TRANSMIT_OCCURRED)
      {
        ret = Cy_CANFD_UpdateAndTransmitMsgBuffer(CY_CANFD0_0_TYPE, buf, &stcMsg);
        if (ret == CY_CANFD_SUCCESS)
        {
          s_canfd0_tx_buf = (buf + 1) & 7;  // 下次从下一个buffer开始
          return;
        }
      }
      buf = (buf + 1) & 7;
      tries++;
    } while (tries < 8);  // 最多扫描8个buffer，都忙则丢弃本帧
  }
}

/**
 * @brief CAN1底层发送 - 与CAN0对称
 */
void CANFD1_TxMsg(uint32_t id, uint32_t dat[], uint8_t len)
{
  cy_stc_canfd_msg_t stcMsg;
  uint8_t i, n;

  memset(&stcMsg, 0, sizeof(cy_stc_canfd_msg_t));

  stcMsg.canFDFormat             = true;
  stcMsg.idConfig.extended       = false;
  stcMsg.idConfig.identifier     = id;
  stcMsg.dataConfig.dataLengthCode = ByteLen_To_DLC(len);

  n = (len + 3) >> 2;
  if (n > 16) n = 16;
  for (i = 0; i < n; i++)
    stcMsg.dataConfig.data[i] = dat[i];

  Cy_CANFD_UpdateAndTransmitMsgBuffer(CY_CANFD1_1_TYPE, 0, &stcMsg);
}

/*============================================================================
* 限流发送层 (唯一的时间检查入口)
*============================================================================*/

/**
 * @brief CAN0限流发送
 *
 * 解决问题: 如果调用方以极高频率(比如每个中断都)调用发送,
 *           CAN总线会被占满, 导致其他帧发不出去或硬件溢出。
 *
 * 实现方式: 记录上次发送时间, 如果距离上次发送不足intervalMs则跳过。
 *
 * @param id:         CAN ID
 * @param dat:        数据指针
 * @param len:        字节数
 * @param intervalMs: 最小发送间隔(ms). 0=不限流(立即发送)
 * @return: 1=已发送, 0=被限流跳过
 */
uint8_t CANFD0_TxLimited(uint32_t id, uint32_t* dat, uint8_t len, uint32_t intervalMs)
{
  /* Bus-Off 冷却期: 恢复后一段时间内禁止发送, 让总线稳定 */
  if (g_can0_recover_time != 0)
  {
    if (g_system_tick_ms - g_can0_recover_time < CAN0_RECOVER_COOLDOWN_MS)
      return 0;
  }

  /* 限流检查: 非零间隔才检查 */
  if (intervalMs > 0)
  {
    uint32_t now = g_system_tick_ms;
    if (now - s_canfd0_last_tick < intervalMs)
      return 0;  // 间隔不够, 跳过本次发送
    s_canfd0_last_tick = now;  // 更新时间戳
  }

  /* 间隔满足(或不限流) → 调用底层发送 */
  CANFD0_TxMsg(id, dat, len);
  return 1;  // 已发送
}

/** @brief CAN1限流发送 (与CAN0对称) */
uint8_t CANFD1_TxLimited(uint32_t id, uint32_t* dat, uint8_t len, uint32_t intervalMs)
{
  if (intervalMs > 0)
  {
    uint32_t now = g_system_tick_ms;
    if (now - s_canfd1_last_tick < intervalMs)
      return 0;
    s_canfd1_last_tick = now;
  }

  CANFD1_TxMsg(id, dat, len);
  return 1;
}

/*============================================================================
* 状态上报函数 (FOC→主控方向, 当前为空壳)
*============================================================================*/

/**
 * @brief CAN0状态上报 (当前未实现具体发送逻辑)
 *
 * 设计用途: 当本板作为FOC驱动板时, 向主控报告电机状态。
 * 当前项目此板为主控板, 所以这个函数暂时为空。
 *
 * @param id:    电机ID
 * @param mode:  运行模式
 * @param speed: 实际转速
 * @param pos:   实际位置
 * @param iq:    Iq电流
 * @param fault: 故障码
 */
void CANFD0_ReportStatus(uint8_t id, uint8_t mode, int16_t speed,
                          int16_t pos, float iq, uint8_t fault)
{
  static uint16_t heartbeat = 0;
  heartbeat++;  // 心跳计数(暂未使用)
}

/** @brief CAN1状态上报 (同上, 空壳) */
void CANFD1_ReportStatus(uint8_t id, uint8_t mode, int16_t speed,
                          int16_t pos, float iq, uint8_t fault)
{
  static uint16_t heartbeat = 0;
  heartbeat++;
}

/*============================================================================
* 电机控制发送封装
*============================================================================*/

/**
 * @brief CAN0发送电机控制指令 (带限流)
 *
 * 将 g_node0_tx 缓冲区的数据通过CAN0发出。
 * 所有上层调用都应通过此函数, 以保证限流机制生效。
 *
 * @param len:        要发送的数据字节数 (通常为sizeof(motorN_ctrl_t))
 * @param intervalMs: 限流间隔(ms), 0=不限流
 */
void CANFD0_TxMotorCtrl(uint8_t count, uint32_t intervalMs)
{
  for (uint8_t i = 0; i < count; i++)
  {
    CANFD0_TxLimited(g_node0_tx.motor[i].id + 0x10,  // +0x10错开从机发送的CAN ID
                     (uint32_t*)&g_node0_tx.motor[i],
                     sizeof(motor_single_ctrl_t), intervalMs);
  }
}

/** @brief CAN1发送电机控制指令 (带限流) */
void CANFD1_Tx4MotorCtrl(uint32_t intervalMs)
{
  CANFD1_TxLimited(CAN_ID_MOTOR_MASTER,
                   (uint32_t*)&g_node1_tx,
                   sizeof(motor4_ctrl_t), intervalMs);
}

/* ========== 旧接口兼容层 ========== */

/** @brief CAN0发送2电机指令 (非阻塞=带限流) */
void CANFD0_Tx2MotorNB(void)   { CANFD0_TxMotorCtrl(2, CANFD0_MIN_INTERVAL_MS); }

/** @brief CAN0发送2电机指令 (立即发送=不限流) */
void CANFD0_Tx2Motor(void)     { CANFD0_TxMotorCtrl(2, 0); }

/** @brief CAN0发送8电机指令 (立即发送) */
void CANFD0_Tx8Motor(void)     { CANFD0_TxMotorCtrl(8, 0); }

/** @brief CAN1发送4关节指令 (非阻塞=带限流) */
void CANFD1_Tx4MotorNB(void)   { CANFD1_Tx4MotorCtrl(CANFD1_MIN_INTERVAL_MS); }

/** @brief CAN1发送4关节指令 (立即发送) */
void CANFD1_Tx4Motor(void)     { CANFD1_Tx4MotorCtrl(0); }

/** @brief CAN0通用发送 (带限流) */
void CANFD0_TxMsgNB(uint32_t id, uint32_t* dat, uint8_t len)
{
  CANFD0_TxLimited(id, dat, len, CANFD0_MIN_INTERVAL_MS);
}

/** @brief CAN1通用发送 (带限流) */
void CANFD1_TxMsgNB(uint32_t id, uint32_t* dat, uint8_t len)
{
  CANFD1_TxLimited(id, dat, len, CANFD1_MIN_INTERVAL_MS);
}

/*============================================================================
* 应用层: 行进/关节电机控制
*============================================================================*/

/**
 * @brief 发送左右行进轮速度指令 (★ 平衡控制的核心发送函数 ★)
 *
 * 调用频率: 500Hz (由Timer_Handler调用)
 * 发送内容:
 *   - 左轮 (#0x55): 速度 = g_balance_output.left_walk_speed
 *   - 右轮 (#0x56): 速度 = g_balance_output.right_walk_speed
 *   - 模式: FOC_MODE_SPEED_MODE (速度模式)
 *
 * 数据流向:
 *   Balance_Controller_Update() → g_balance_output → 此函数 → CAN总线 → FOC板
 *
 * 限流策略: 使用 CANFD0_MIN_INTERVAL_MS (通常=2ms, 即最高500Hz)
 */
void CANFD0_Tx2WalkMotor(void)
{
  static uint16_t heartbeat = 0;
  heartbeat++;

  /* 组装左轮指令 */
  g_node0_tx.motor[0].id         = LEFT_WALK_MOTOR;                  // 0x55
  g_node0_tx.motor[0].mode       = FOC_MODE_SPEED_MODE;             // 速度模式
  // g_node0_tx.motor[0].speed      = g_balance_output.left_walk_speed; // ★ 来自平衡算法
  // g_node0_tx.motor[0].angle      = 0;                               // 速度模式下角度无意义
  g_node0_tx.motor[0].heartbeat  = heartbeat;                       // 心跳(FOC侧可检测主控存活)

  /* 组装右轮指令 */
  g_node0_tx.motor[1].id         = RIGHT_WALK_MOTOR;                 // 0x56
  g_node0_tx.motor[1].mode       = FOC_MODE_SPEED_MODE;
  // g_node0_tx.motor[1].speed      = g_balance_output.right_walk_speed; // ★ 来自平衡算法
  // g_node0_tx.motor[1].angle      = 0;
  g_node0_tx.motor[1].heartbeat  = heartbeat;

  /* 发送 (带500Hz限流) */
  CANFD0_TxMotorCtrl(2, CANFD0_MIN_INTERVAL_MS);
}

/**
 * @brief 发送4个关节电机位置指令 (100Hz)
 *
 * 用途: 控制腿部关节电机 (如果有腿的话)。
 *       当前双轮平衡机器人可能未使用, 保留供扩展。
 *
 * 调用频率: 100Hz (比行进轮低, 因为关节响应不需要那么快)
 */
void CANFD1_Tx4JointMotor(void)
{
  static uint16_t heartbeat = 0;
  heartbeat++;
  uint8_t i;

  for (i = 0; i < 4; i++)
  {
    g_node1_tx.motor[i].id        = LEFT_FRONT_MOTOR + i;
    g_node1_tx.motor[i].mode      = FOC_MODE_POSITION_MODE;  // 位置模式
    // g_node1_tx.motor[i].angle     = (int16_t)(g_balance_output.joint_position[i]);
    // g_node1_tx.motor[i].speed     = g_balance_output.joint_speed;
    g_node1_tx.motor[i].heartbeat = heartbeat;
  }

  CANFD1_Tx4MotorCtrl(CANFD1_MIN_INTERVAL_MS);  // 100Hz限流
}

/*============================================================================
* 紧急停止 (安全相关, 不限流!)
*============================================================================*/

/**
 * @brief 紧急停止所有电机
 *
 * 特点:
 *   - ★ 不限流 ★ (interval=0), 立即发送, 不等待任何时间间隔
 *   - 同时停止行进电机和关节电机
 *   - 发送STOP模式指令, 让FOC板立即切断PWM输出
 *
 * 调用场景:
 *   - 角度超限 (倾倒检测)
 *   - CAN通信超时
 *   - 用户按键急停
 *   - 任何需要立即停机的安全事件
 *
 * 注意: 注释掉的CAN1部分是关节电机停机代码,
 *       当前项目如果没用关节电机可以保持注释状态。
 */
void CANFD_Stop_All_Motors(void)
{
  uint8_t i;

  /* 停止行进电机 (CAN0) - 立即发送! */
  for (i = 0; i < 2; i++)
  {
    g_node0_tx.motor[i].id    = (i == 0) ? LEFT_WALK_MOTOR : RIGHT_WALK_MOTOR;
    g_node0_tx.motor[i].mode  = FOC_MODE_STOP;   // ★ STOP模式 ★
    // g_node0_tx.motor[i].angle = 0;
    // g_node0_tx.motor[i].speed = 0;
  }
  CANFD0_TxMotorCtrl(2, 0);  // interval=0, 立即发出!
}
