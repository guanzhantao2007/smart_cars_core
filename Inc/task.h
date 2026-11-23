#ifndef TASK_H
#define TASK_H

#include "main.h"

/* ============================================================================
 * IMU 总开关（2026-11-23 加）
 *
 *   0 = 不初始化 IMU，Task_Init 跳过它继续往下走（TIM6 / OLED 照常）
 *   1 = 老行为：IMU_Init 失败就 Fault_Report + return，后面全跳过
 *
 * 为什么需要这个开关：IMU_Init 一失败，Task_Init 直接 return，于是
 *   TIM6 没启动、OLED_Init 没跑 → 主循环第一次 OLED_Update 抛 TASK_ERROR
 *   → Error_Handler 把故障标签从 "IMU_Init" 覆盖成 "hal/mx"。
 *   结果是一个坏器件把另外两个好器件一起带哑，还改乱了故障名字。
 *
 * 现在只是"暂时不测姿态"：跳过 IMU_Init 后 s_d.state 停在 UNINIT，
 *   IMU_Update() 第一句判断 state != READY 就 return，不会去读 I2C，
 *   主循环不会被死传感器的 I2C 超时拖慢。
 * 影响面：yaw 恒 0、YawLoop 仍在 YAW_OFF 状态什么都不做（1300 版本来
 *   也没有 yaw 命令入口）。速度环 / 编码器测速 / 调参完全不受影响。
 * 恢复：把下面这行改成 1，重新编译即可。
 * ========================================================================== */
#define TASK_IMU_ENABLE   0U

//创建任务状态枚举
typedef enum {
    TASK_BENGIN = 0U, //任务初始化状态
    TASK_RUNNING=1U,   //任务运行状态
    LINE_STATE =2U,     //循迹状态
    LINE_STOP=3U,      //循迹停止状态
    TASK_STOP=4U,      //任务停止状态
    TASK_ERROR=5U      //任务错误状态
} TaskState_t;


HAL_StatusTypeDef Task_Init(void);
TaskState_t Task1_Run(void);
void Task1_UartIdleCallback(void);
void Task1_UartError(void);
TaskState_t Task2_Run(void);
TaskState_t Task3_Run(void);
TaskState_t Task4_Run(void);
TaskState_t Task5_Run(void);
TaskState_t Task6_Run(void);


#endif
