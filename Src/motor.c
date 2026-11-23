#include "motor.h"
#include "encoder.h"
#include "tim.h"

#define MOTOR_SPEED_SAMPLE_PERIOD_MS 10U
#define MOTOR_COUNT 4U

typedef struct {
    float integral;
    float previous_measurement;
    uint32_t previous_tick;
    uint8_t initialized;
} MotorPidState;

static MotorPidState motor_pid_states[MOTOR_COUNT];

/* ============================================================================
 * 电机安装极性：1 = 这台电机的"电气正转"与"车头方向"相反
 *
 * 【2026-11-22 实测】架空、逐台 `ch n` + `o 20` 驱动 1.5 s，同时看编码器读数、
 *   由展滔目视确认位置与转向（三轮重复，结果一致）：
 *
 *     编号  位置    指令正转时物理方向      编码器读数
 *      1    左后    往车头  （正）          -320
 *      2    右后    往车尾  （倒）  ← 唯一倒转的一台
 *      3    右前    往车头  （正）          +275
 *      4    左前    往车头  （正）          -240
 *
 *   所以只有 2 号要在表里标 1。原因大概率是这台电机的两根线在驱动器那一侧
 *   接反了（不是软件问题，改这一行等价于把它在软件上接回来）。
 *
 *   ⚠️ 上表最后一列「编码器读数」是**指令正转时**读到的，不能直接抄进
 *   encoder.c 的 forward_count_sign —— 对本表标了 1 的电机（现在只有 2 号），
 *   指令正转已经等于物理往前，读数方向要取反才对。换算过程写在 encoder.c。
 *
 * 【为什么改在这一层】dir 的语义必须是"车头/车尾"，四台一致。如果改到上层
 *   去给 2 号的输出取负号，PID 的正负反馈就会随电机号变化 —— 以后谁写一条
 *   "倒车"或"横移"指令，都要记得绕开 2 号，迟早出错。
 *
 * 【和 encoder.c 的 forward_count_sign 是一对】两个表记的都是"物理安装事实"，
 *   改一个必须同时看另一个：
 *     - 只翻这里不翻编码器 → 轮子转对了，但测速符号反 → 闭环正反馈发散；
 *     - 只翻编码器不翻这里 → 轮子还倒转，测速也反 → 同样发散。
 * ========================================================================= */
static const uint8_t motor_mount_reverse[MOTOR_COUNT] = {0U, 1U, 0U, 0U};

void motor_init(void)
{
    uint32_t index;

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }

    for (index = 0U; index < MOTOR_COUNT; ++index) {
        motor_pid_states[index].integral = 0.0f;
        motor_pid_states[index].previous_measurement = 0.0f;
        motor_pid_states[index].previous_tick = HAL_GetTick();
        motor_pid_states[index].initialized = 0U;
    }
}

uint16_t speed2pluse(float speed, uint32_t period)
{
    if (speed < 0.0f) {
        speed = -speed;
    }
    if (!(speed > 0.0f) || period == 0U) {
        return 0U;
    }
    if (speed >= MOTOR_PWM_FULL_SCALE_MM_S) {
        return (period > UINT16_MAX) ? UINT16_MAX : (uint16_t)period;
    }

    /* 加 0.5f 是四舍五入：float 转整型是截断，直接截会把 0.9 变成 0，
     * 低速时每一拍都少一格，累积成"起步发黏"。 */
    return (uint16_t)((speed * (float)period / MOTOR_PWM_FULL_SCALE_MM_S) + 0.5f);
}


/* ============================================================================
 * 慢衰减（slow decay）驱动
 *
 * 【为什么这么写】AT8236 精简版用户手册 p.5 给了两张表，第二张是 key：
 *
 *     IN1    IN2    功能
 *     PWM    0      正转 PWM，快衰减
 *     1      PWM    正转 PWM，慢衰减
 *     0      PWM    反转 PWM，快衰减
 *     PWM    1      反转 PWM，慢衰减
 *
 *   配合第一张真值表（0/0 → OUT 全高阻 = 滑行；1/1 → 两下管全开 = 刹车）：
 *     - 快衰减 = 一个脚斩波、另一个脚常 0；斩波低电平那一段输出变高阻，
 *       电流经体二极管倒灌回 VM，衰减快 → 电流纹波大、有反向转矩脉动、
 *       低速抖动、有可听噪声。
 *     - 慢衰减 = 一个脚**常 1**、另一个脚斩波；斩波高电平那一段是 1/1 刹车，
 *       两个下管都开，电流在"电机 + 下管"里环流，衰减慢 → 电流平滑、
 *       低速稳、噪声小。代价是常态刹车，靠斩波里那段"非刹车"来出转矩。
 *
 * 【因此占空比是反的】常 1 那一脚写 CCR > ARR 得到恒高；斩波那一脚
 *   写 CCR = period - drive —— drive 越大，斩波脚的低电平越长，驱动越多。
 *   drive = 0 时 CCR = period，斩波脚几乎恒高 → 1/1 → 电机被短接制动。
 *   所以本工程"速度指令 0"= 刹车，不是滑行；AT8236 精简版没引出 nSLEEP，
 *   硬件上也做不到高阻滑行，这一点没有第二种选择。
 * ========================================================================= */
void motor_run(Motor motor)
{
    TIM_HandleTypeDef *timer;
    uint32_t ch_forward;    /* 正转时该恒为高的那一脚（IN1 或 IN2） */
    uint32_t ch_chop;       /* 正转时该斩波的那一脚 */
    uint32_t period;
    uint32_t full;
    uint16_t drive;
    uint8_t forward_like;   /* 1 = 本次要往车头方向转（已含安装极性修正） */

    if (motor.dir != 0 && motor.dir != 1) {
        Error_Handler();
        return;
    }

    switch (motor.motor_id) {
    case 1:
        timer = &htim1;
        ch_forward = TIM_CHANNEL_1;
        ch_chop = TIM_CHANNEL_2;
        break;
    case 2:
        timer = &htim1;
        ch_forward = TIM_CHANNEL_3;
        ch_chop = TIM_CHANNEL_4;
        break;
    case 3:
        timer = &htim9;
        ch_forward = TIM_CHANNEL_1;
        ch_chop = TIM_CHANNEL_2;
        break;
    case 4:
        timer = &htim12;
        ch_forward = TIM_CHANNEL_1;
        ch_chop = TIM_CHANNEL_2;
        break;
    default:
        Error_Handler();
        return;
    }

    period = __HAL_TIM_GET_AUTORELOAD(timer);
    /* CCR 写 ARR+1 就是恒高（PWM1 模式下 CNT 永远小于 CCR）。
     * 16 位寄存器在 ARR=65535 时会溢出成 0，反而变成恒低、方向翻转，
     * 所以这里先夹住。本工程三个 ARR 都远小于 65535，只为防以后改周期。 */
    full = (period >= 0xFFFFU) ? 0xFFFFU : (period + 1U);
    drive = speed2pluse((float)motor.speed, period);

    /* dir 说"往车头"，但这台电机可能装反了：电气上的正转要在这里翻一次，
     * 出了这个函数，全工程只有一种"正转" = 车头方向。 */
    forward_like = (motor.dir == 1) ? 1U : 0U;
    if (motor_mount_reverse[motor.motor_id - 1U] != 0U) {
        forward_like = (forward_like != 0U) ? 0U : 1U;
    }

    if (forward_like != 0U) {
        __HAL_TIM_SET_COMPARE(timer, ch_forward, full);
        __HAL_TIM_SET_COMPARE(timer, ch_chop, period - (uint32_t)drive);
    } else {
        __HAL_TIM_SET_COMPARE(timer, ch_chop, full);
        __HAL_TIM_SET_COMPARE(timer, ch_forward, period - (uint32_t)drive);
    }
}

void motor_pid_run(Motor motor)
{
    uint32_t index;
    uint32_t current_tick;
    uint32_t elapsed_ms;
    float target_speed;
    float dt_seconds;
    float measured_speed;
    float error;
    float derivative;
    float candidate_integral;
    float output;
    float raw_output;
    MotorPidState *state;

    if (motor.motor_id < 1 || motor.motor_id > (int)MOTOR_COUNT ||
        (motor.dir != 0 && motor.dir != 1) ||
        !(motor.speed >= 0.0) ||
        !(motor.p >= 0.0f) ||
        !(motor.i >= 0.0f) ||
        !(motor.d >= 0.0f)) {
        Error_Handler();
        return;
    }

    index = (uint32_t)(motor.motor_id - 1);
    state = &motor_pid_states[index];
    current_tick = HAL_GetTick();
    elapsed_ms = (uint32_t)(current_tick - state->previous_tick);
    if (elapsed_ms < MOTOR_SPEED_SAMPLE_PERIOD_MS) {
        return;
    }

    dt_seconds = (float)elapsed_ms / 1000.0f;
    state->previous_tick = current_tick;

    target_speed = (motor.speed >= MOTOR_PWM_FULL_SCALE_MM_S)
                       ? MOTOR_PWM_FULL_SCALE_MM_S
                       : (float)motor.speed;
    measured_speed = encoder_get_speed(motor.motor_id);
    if (motor.dir == 0) {
        measured_speed = -measured_speed;
    }

    if (state->initialized == 0U) {
        state->previous_measurement = measured_speed;
        state->initialized = 1U;
    }

    if (target_speed == 0.0f) {
        state->integral = 0.0f;
        state->previous_measurement = measured_speed;
        motor.speed = 0.0;
        motor_run(motor);
        return;
    }

    error = target_speed - measured_speed;
    derivative = -(measured_speed - state->previous_measurement) / dt_seconds;
    candidate_integral = state->integral + error * dt_seconds;
    raw_output = motor.p * error +
                 motor.i * candidate_integral +
                 motor.d * derivative;

    if ((raw_output >= 0.0f && raw_output <= MOTOR_PWM_FULL_SCALE_MM_S) ||
        (raw_output > MOTOR_PWM_FULL_SCALE_MM_S && error < 0.0f) ||
        (raw_output < 0.0f && error > 0.0f)) {
        state->integral = candidate_integral;
    }

    output = motor.p * error +
             motor.i * state->integral +
             motor.d * derivative;
    if (output < 0.0f) {
        output = 0.0f;
    } else if (output > MOTOR_PWM_FULL_SCALE_MM_S) {
        output = MOTOR_PWM_FULL_SCALE_MM_S;
    }

    state->previous_measurement = measured_speed;
    motor.speed = output;
    motor_run(motor);
}

