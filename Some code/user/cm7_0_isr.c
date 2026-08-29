/*********************************************************************************************************************
* CYT4BB Opensource Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cm7_0_isr
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-9      pudding            first version
* 2024-5-14     pudding            新增12个pit周期中断 增加部分注释说明
* 2025-2-4      pudding            优化串口中断逻辑，防止意外干扰导致的卡死问题，优化串口波特率计算逻辑
* 2025-2-4      pudding            新增两个串口接口
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "imu_ahrs.h"
#include "LQ_CANFD.h"


// 1kHz 姿态/惯导 节拍标志: ISR 置位, 主循环清零并做姿态+惯导计算
volatile uint8 g_imu_tick_pending = 0;

// 中断采样IMU原始数据(1ms准时), 主循环取来做ESKF/惯导重计算
volatile float imu_smp_gx = 0, imu_smp_gy = 0, imu_smp_gz = 0;
volatile float imu_smp_ax = 0, imu_smp_ay = 0, imu_smp_az = 0;

// **************************** PIT中断函数 ****************************
void pit0_ch0_isr()                     // 定时器通道 0 中断服务函数
{
    float gx, gy, gz, ax, ay, az;

    pit_isr_flag_clear(PIT_CH0);

    // 中断只做轻量SPI采样(约40µs, 1ms准时, 不受主循环/串口中断拖累);
    // 10态ESKF 重计算放主循环, 避免长中断卡 CAN 接收
    imu_ahrs_read(&gx, &gy, &gz, &ax, &ay, &az);   // 陀螺 rad/s, 加速度 g
    imu_smp_gx = gx;  imu_smp_gy = gy;  imu_smp_gz = gz;
    imu_smp_ax = ax;  imu_smp_ay = ay;  imu_smp_az = az;
    g_system_tick_ms++;                 // 系统毫秒tick(CAN bus-off 恢复冷却计时用)
    g_imu_tick_pending = 1;             // 通知主循环做 1kHz 姿态+惯导
}

void pit0_ch1_isr()                     // 定时器通道 1 中断服务函数
{
    pit_isr_flag_clear(PIT_CH1);

}

void pit0_ch2_isr()                     // 定时器通道 2 中断服务函数
{
    pit_isr_flag_clear(PIT_CH2);

}

void pit0_ch10_isr()                    // 定时器通道 10 中断服务函数
{
    pit_isr_flag_clear(PIT_CH10);

}

void pit0_ch11_isr()                    // 定时器通道 11 中断服务函数
{
    pit_isr_flag_clear(PIT_CH11);

}

void pit0_ch12_isr()                    // 定时器通道 12 中断服务函数
{
    pit_isr_flag_clear(PIT_CH12);

}

void pit0_ch13_isr()                    // 定时器通道 13 中断服务函数
{
    pit_isr_flag_clear(PIT_CH13);

}

void pit0_ch14_isr()                    // 定时器通道 14 中断服务函数
{
    pit_isr_flag_clear(PIT_CH14);

}

void pit0_ch15_isr()                    // 定时器通道 15 中断服务函数
{
    pit_isr_flag_clear(PIT_CH15);

}

void pit0_ch16_isr()                    // 定时器通道 16 中断服务函数
{
    pit_isr_flag_clear(PIT_CH16);

}

void pit0_ch17_isr()                    // 定时器通道 17 中断服务函数
{
    pit_isr_flag_clear(PIT_CH17);

}

void pit0_ch18_isr()                    // 定时器通道 18 中断服务函数
{
    pit_isr_flag_clear(PIT_CH18);

}

void pit0_ch19_isr()                    // 定时器通道 19 中断服务函数
{
    pit_isr_flag_clear(PIT_CH19);

}

void pit0_ch20_isr()                    // 定时器通道 20 中断服务函数
{
    pit_isr_flag_clear(PIT_CH20);

}

void pit0_ch21_isr()                    // 定时器通道 21 中断服务函数
{
    pit_isr_flag_clear(PIT_CH21);
    tsl1401_collect_pit_handler();
}
// **************************** PIT中断函数 ****************************


// **************************** 串口中断函数 ****************************
// 串口0默认为调试串口
void uart0_isr (void)
{
    if(uart_isr_mask(UART_0))            // 串口0接收中断
    {

#if DEBUG_UART_USE_INTERRUPT             // 如果使用 debug 接收中断
        debug_interrupr_handler();       // debug 串口接收处理函数 数据会被 debug 接收函数读取
#endif                                   // 如果修改了 DEBUG_UART_INDEX 未编译 需要放到对应的串口中断去

    }
    else                                 // 串口0发送中断
    {



    }
}

void uart1_isr (void)
{
    if(uart_isr_mask(UART_1))            // 串口1接收中断
    {

        wireless_module_uart_handler();  // 无线模块统一回调函数

    }
    else                                // 串口1发送中断
    {



    }
}

void uart2_isr (void)
{
    if(uart_isr_mask(UART_2))            // 串口2接收中断
    {

        gnss_uart_callback();            // GPS模块回调函数

    }
    else                                // 串口2发送中断
    {



    }
}

void uart3_isr (void)
{
    if(uart_isr_mask(UART_3))            // 串口3接收中断
    {



    }
    else                                // 串口3发送中断
    {



    }
}

void uart4_isr (void)
{
    if(uart_isr_mask(UART_4))            // 串口4接收中断
    {

        uart_receiver_handler();                                                                // 串口接收回调函数

    }
    else                                // 串口4发送中断
    {



    }
}

void uart5_isr (void)
{
    if(uart_isr_mask(UART_5))            // 串口5接收中断
    {



    }
    else                                // 串口5发送中断
    {



    }
}

void uart6_isr (void)
{
    if(uart_isr_mask(UART_6))            // 串口6接收中断
    {



    }
    else                                // 串口6发送中断
    {



    }
}
// **************************** 串口中断函数 ****************************

// **************************** 外部中断函数 ****************************
void gpio_0_exti_isr()                  // 外部 GPIO_0 中断服务函数
{




}

void gpio_1_exti_isr()                  // 外部 GPIO_1 中断服务函数
{
    if(exti_flag_get(P01_0))		// 示例P1_0端口外部中断判断
    {




    }
    if(exti_flag_get(P01_1))
    {



    }
}

void gpio_2_exti_isr()                  // 外部 GPIO_2 中断服务函数
{
    if(exti_flag_get(P02_0))
    {


    }
    if(exti_flag_get(P02_4))
    {


    }

}

void gpio_3_exti_isr()                  // 外部 GPIO_3 中断服务函数
{



}

void gpio_4_exti_isr()                  // 外部 GPIO_4 中断服务函数
{



}

void gpio_5_exti_isr()                  // 外部 GPIO_5 中断服务函数
{



}


void gpio_6_exti_isr()                  // 外部 GPIO_6 中断服务函数
{
    if(exti_flag_get(IMU660RC_INT2_PIN))
    {
        imu660rc_callback();
    }
}

void gpio_7_exti_isr()                  // 外部 GPIO_7 中断服务函数
{



}

void gpio_8_exti_isr()                  // 外部 GPIO_8 中断服务函数
{



}

void gpio_9_exti_isr()                  // 外部 GPIO_9 中断服务函数
{



}

void gpio_10_exti_isr()                  // 外部 GPIO_10 中断服务函数
{



}

void gpio_11_exti_isr()                  // 外部 GPIO_11 中断服务函数
{



}

void gpio_12_exti_isr()                  // 外部 GPIO_12 中断服务函数
{



}

void gpio_13_exti_isr()                  // 外部 GPIO_13 中断服务函数
{



}

void gpio_14_exti_isr()                  // 外部 GPIO_14 中断服务函数
{



}

void gpio_15_exti_isr()                  // 外部 GPIO_15 中断服务函数
{



}

void gpio_16_exti_isr()                  // 外部 GPIO_16 中断服务函数
{



}

void gpio_17_exti_isr()                  // 外部 GPIO_17 中断服务函数
{



}

void gpio_18_exti_isr()                  // 外部 GPIO_18 中断服务函数
{



}

void gpio_19_exti_isr()                  // 外部 GPIO_19 中断服务函数
{



}

void gpio_20_exti_isr()                  // 外部 GPIO_20 中断服务函数
{



}

void gpio_21_exti_isr()                  // 外部 GPIO_21 中断服务函数
{



}

void gpio_22_exti_isr()                  // 外部 GPIO_22 中断服务函数
{



}

void gpio_23_exti_isr()                  // 外部 GPIO_23 中断服务函数
{



}
// **************************** 外部中断函数 ****************************
