#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx_hal.h"
#define motor1_start HAL_GPIO_WritePin( EN1_GPIO_Port,EN1_Pin,GPIO_PIN_SET)
#define motor1_stop HAL_GPIO_WritePin( EN1_GPIO_Port,EN1_Pin,GPIO_PIN_RESET)
#define motor2_start HAL_GPIO_WritePin( EN2_GPIO_Port,EN2_Pin,GPIO_PIN_SET)
#define motor2_stop HAL_GPIO_WritePin( EN2_GPIO_Port,EN2_Pin,GPIO_PIN_RESET)
/* ============================================================================
 * PWM 满量程对应的线速度（mm/s）
 *
 * 【含义】占空比 100% 时轮子的空载线速度。speed2pluse() 用它做
 *         mm/s → 占空比 的换算：duty = speed / MOTOR_PWM_FULL_SCALE_MM_S。
 * 【2026-11-22 已实测，不再是占位值】
 *         架空、`oall 100` 稳住 1.2 s 后进平台期（之后 0.3 s 漂移 < 0.05%）：
 *           CH1 1696 / CH2 1642 / CH3 1675 / CH4 1614  → 均值 1657 mm/s
 *         四轮差 ±2.5%，取一个数够用（闭环会自己补差）。
 *         同批实测 25/50/75% 是 335/781/1208，比线性略低 —— 低速段有死区，
 *         约 4~5% 占空比（≈ 70~80 mm/s），按用户要求暂不细测。
 * 【为什么是空载值】这个常量的职责是"1 mm/s 指令 = 多少占空比"这把尺子，
 *         负载带来的速度差由 PID 闭环补。若拿满载值当尺子，闭环会系统性偏高。
 * 【连带影响】改它 → speed2pluse 的换算、pid_tune.c 的 TUNE_TGT_MAX（目标
 *         上限）跟着变。PID 输出和 `o` 命令**不受影响**，它们是占空比百分比。
 * 【前提假设】mm/s 的绝对值还依赖 encoder.c 的 48 mm 轮径、500 PPR、20:1，
 *         那三个数没单独验收过；实际不是这个值的话，1657 要按比例修。
 * 【旧值 100 的教训】FULL=100 时 `tgt 30` 其实是 30% 占空比 ≈ 500 mm/s，
 *         波形上却写着"30 mm/s"。这个标签骗了人一整个阶段，所以这次一并
 *         把 PID 输出改成百分比、只在这里做唯一一次换算。
 * ========================================================================= */
#define MOTOR_PWM_FULL_SCALE_MM_S   1657.0f

//创建电机结构体
typedef struct {
    int motor_id; // 电机编号
    double speed; // 速度指令（毫米/秒），MOTOR_PWM_FULL_SCALE_MM_S 对应满占空比
    int dir;     // 电机方向：1前进，0后退
    float p;
    float i;
    float d;
} Motor;


void motor_init(void);
/* speed → 比较寄存器值。period 由调用方按各自定时器的 ARR 传入，
 * 不再内部硬读 htim1（三个定时器 ARR 不同，硬读必然错一个）。 */
uint16_t speed2pluse(float speed, uint32_t period);
void motor_run(Motor motor);
/* Call periodically; sampling and PID calculations are handled in motor.c. */
void motor_pid_run(Motor motor);

#endif
