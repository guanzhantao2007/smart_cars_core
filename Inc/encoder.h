#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f4xx_hal.h"

#define ENCODER_TIMEBASE_HZ 100U

void encoder_init(void);
void encoder_timebase_tick(void);
/* 返回该 motor_id 的线速度（mm/s），正值表示前进。 */
float encoder_get_speed(int motor_id);

/* 自上电（或上一次清零）以来的累计计数，带方向，单位就是编码器原始计数。
 * 用途只有一个：验收"一圈到底是不是 40000 个计数"。
 * 注意它靠 encoder_get_speed() 顺带累加，所以必须在控制周期里被调用过才有数。 */
int64_t encoder_get_total(int motor_id);

/* 把该 motor_id 的累计计数清零。手转一圈之前先清，转完再读。 */
void encoder_reset_total(int motor_id);

#endif
