#include "pid.h"

/**
  * @brief  初始化一个 PID 实例
  * @param  out_max 输出限幅，占空比用 100 就对应 0~100%
  * @note   DAlpha 默认 0.3：微分项取 30% 新值 + 70% 旧值，
  *         能把 ±1 个计数的抖动压下去，又不至于让微分完全失灵。
  */
void PID_Init(PID_t *p, float kp, float ki, float kd, float out_max)
{
    p->Kp       = kp;
    p->Ki       = ki;
    p->Kd       = kd;
    p->OutMax   = out_max;
    p->DAlpha   = 0.3f;

    PID_Reset(p);
}

void PID_SetGains(PID_t *p, float kp, float ki, float kd)
{
    p->Kp = kp;
    p->Ki = ki;
    p->Kd = kd;
}

/**
  * @brief  清空内部状态
  * @note   积分和微分状态必须一起清。只清积分的话，
  *         下次上电第一个周期会因为 LastActual 是旧值而打出一个微分尖峰。
  */
void PID_Reset(PID_t *p)
{
    p->ErrorInt   = 0.0f;
    p->LastActual = 0.0f;
    p->DState     = 0.0f;
    p->Out        = 0.0f;
}

/**
  * @brief  执行一次位置式 PID 运算
  * @param  target 目标值（与控制周期同量纲）
  * @param  actual 实际值（编码器增量）
  * @retval 限幅后的输出
  */
float PID_Calc(PID_t *p, float target, float actual)
{
    float err;
    float d_raw;
    float pre_out;

    err = target - actual;

    /* 微分作用于「实际值的变化量」，取负号是标准做法：
       设定值跳变时不会产生微分冲击，只有被控量真在动才有微分作用。 */
    d_raw = -(actual - p->LastActual);
    p->LastActual = actual;

    /* 一阶低通：压制编码器量化噪声 */
    p->DState += p->DAlpha * (d_raw - p->DState);

    /* 先用当前积分值算一次「预输出」，判断是否已经顶到限幅 */
    pre_out = p->Kp * err + p->Ki * p->ErrorInt + p->Kd * p->DState;

    /* 抗积分饱和（条件积分）：
       - 未饱和：正常累加
       - 已顶到上限但误差是负的：说明在往回走，继续累加帮助退出饱和
       - 已顶到下限但误差是正的：同理 */
    if ((pre_out < p->OutMax && pre_out > -p->OutMax) ||
        (pre_out >=  p->OutMax && err < 0.0f) ||
        (pre_out <= -p->OutMax && err > 0.0f))
    {
        p->ErrorInt += err;
    }

    /* 积分钳位：防止长时间堵转后积分堆到天文数字，
       即便条件积分失效也不至于失控。上限按「积分项刚好打满输出」来定。 */
    if (p->Ki > 0.0001f)
    {
        float lim = p->OutMax / p->Ki;
        if (p->ErrorInt >  lim) p->ErrorInt =  lim;
        if (p->ErrorInt < -lim) p->ErrorInt = -lim;
    }
    else
    {
        /* Ki 为 0 时积分项不参与运算，顺手清零，
           避免调试期攒出一个巨大的 ErrorInt，之后一开 Ki 就爆 */
        p->ErrorInt = 0.0f;
    }

    p->Out = p->Kp * err + p->Ki * p->ErrorInt + p->Kd * p->DState;

    if (p->Out >  p->OutMax) p->Out =  p->OutMax;
    if (p->Out < -p->OutMax) p->Out = -p->OutMax;

    return p->Out;
}
