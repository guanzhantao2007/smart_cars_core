/**
  ******************************************************************************
  * @file    pid_tune.c
  * @brief   四电机 PID 在线调参（VOFA+ 文本协议）
  ******************************************************************************
  * 设计要点（改这里之前先看这四条）：
  *
  * 1) 控制环节挂在 TIM7 的 100 Hz 中断里，不挂主循环。
  *    主循环里有 OLED 全屏刷新（I2C 阻塞几十毫秒），周期完全不可控；
  *    而 TIM7 同时是编码器的测速时基，控制周期和测速窗口都是 10 ms，
  *    天然对齐，不需要再算 dt。
  *
  * 2) encoder_get_speed() 是「读一次就更新基准」的一次性差分接口，
  *    一个控制周期只能调一次。所以本文件里测速只在 Tick 里做一次，
  *    波形和显示都用缓存值，绝不再读第二遍。
  *
  * 3) USART2 的 TX 同时被 printf 占用（task.c 的 Task_Init 结尾打了一句）。
  *    顺序上是「先 printf、后 Serial_Init」，所以上电那行文本先发完，
  *    波形流不会和它混。但运行期绝不能再加 printf —— 文本会插进波形帧中间，
  *    VOFA 上表现为随机凹口，很容易被误判成编码器坏了。
  *
 * 4) 数值别参考参照工程（那台两轮车）。它满量程 800 rpm、周期 40 ms；
 *    这里满量程 1657 mm/s（实测，见 motor.h）、周期 10 ms。周期短 4 倍 ⇒
 *    同样的物理效果 Ki 大约要小 4 倍；再加上输出改成了百分比量纲，
 *    那台车的增益数字在这里连量级都对不上，只能当思路参考。
 *
 * 5) 量纲分工（2026-11-22 改）：
 *      s_target / s_speed  = mm/s（物理真值）
 *      s_out / s_open / 增益 = 占空比百分比（0~100）
 *    mm/s → % 的换算有**两处**，缺一处增益就变形 16.57 倍：
 *      a) PID 入口：目标和反馈都要除 TUNE_MM_PER_DUTY（Tick10ms 里闭环分支）
 *      b) 写 PWM 前：输出百分比乘 TUNE_MM_PER_DUTY 换回 mm/s
 *    第一次改方案 A 时只做了 b)、漏了 a)，结果 Kp 等效放大 16.57 倍，
 *    四条速度环集体在极限环边缘，表现为"随机某台跑飞"，极易误判成极性错。
 *    判断依据：**极性错是某一台固定单向顶死；增益过大是几台轮流来回撞限幅。**
 ******************************************************************************
 */

#include "pid_tune.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "pidcmd.h"
#include "serial_async.h"
#include "vofa.h"

#include <stdio.h>

#define TUNE_MOTOR_COUNT   4U
#define TUNE_WAVE_CH       5U      /* 目标 ×1 + 实测 ×4 */

/* 输出限幅：单位是**占空比百分比**，100 = 100% 占空比。
 * 【为什么不直接拿 mm/s 当输出量纲】那样增益会绑死在满量程常数上：改一次
 * MOTOR_PWM_FULL_SCALE_MM_S（这次就从 100 改成了 1657），kp/ki 全部要乘 16.5
 * 重写一遍，改漏一处就是"PID 以为输出满了，PWM 才给一半"。
 * 用百分比，增益的含义永远是"多大速度误差 → 给多少占空比"，换电机、换轮径
 * 都不用动增益。mm/s → 占空比的唯一一次换算放在写 PWM 之前（见 Tick10ms）。
 * 注：pid.c 里 PID_Init 的形参注释本来就是"占空比用 100 就对应 0~100%"，
 * 这里是把那个原意接回来。 */
#define TUNE_OUT_MAX       100.0f

/* 目标上限（mm/s）= 物理满量程，直接取 motor.h 的那个常量，不另抄一个数。 */
#define TUNE_TGT_MAX       MOTOR_PWM_FULL_SCALE_MM_S

/* 1% 占空比 ≈ 多少 mm/s（实测满量程 / 100）。用在三处：
 *   1) PID 入口：目标和反馈都除以它，把 mm/s 归一到百分比量纲；
 *   2) 写 PWM 前把百分比换回 motor_run() 认的 mm/s；
 *   3) 开环时把 `o` 的百分比换算成等效 mm/s 回显到波形 CH0，
 *      否则同一条曲线闭环是 mm/s、开环是 %，两种量纲混看必然误判。 */
#define TUNE_MM_PER_DUTY   (MOTOR_PWM_FULL_SCALE_MM_S / TUNE_OUT_MAX)

/* 测速一阶低通：y += α(x−y)。
 * 参照工程 40 ms 周期用 0.35（时间常数约 90 ms）；这里 10 ms 一拍，
 * 要保住同样的平滑度，α 必须按 dt 折算成 0.10，直接抄 0.35 等于不滤波。
 * 出现低频摆速说明滤过头了（α 调大）；波形全是毛刺说明滤不够（α 调小）。 */
#define TUNE_SPEED_ALPHA   0.10f

/* 波形分频：PID 跑 100 Hz，波形发 50 Hz。
 * 5 列文本约 30 字节，115200 下 2.6 ms；100 Hz 发会占到 26% 的带宽且没必要。 */
#define TUNE_WAVE_DIV      2U

/* 增益量程上限（命令里超出的值会被夹住，不报错）。
 * 量纲：输出是占空比百分比，输入误差也是**占空比百分比**（mm/s 已在 PID
 *       入口除过 TUNE_MM_PER_DUTY）。误差满打满算就是 100。
 *   kp = 1.0 ⇒ 误差 100%（≈整条速度量程）才给满输出 ⇒ 这是偏软的一端。
 *              所以 kp 的有效区大概在 0.5~5，默认 0.5 属于很保守的起步。
 *   ki 是「每拍累加一次误差」，误差 10（%）累 1 秒（100 拍）后 ErrorInt=1000，
 *      ki=0.02 就能给出 20% 输出 —— 起步用 0.01~0.03 量级，别一上手 0.5。 */
#define TUNE_KP_MAX        3.0f
#define TUNE_KI_MAX        0.5f
#define TUNE_KD_MAX        5.0f

/* ==========================================================================
 * 上电默认增益（断电重启也在）。
 * 索引 0~3 = 电机 1~4，**按通道号顺序，不按调参顺序**。
 *
 * 2026-11-22 由实车整定值固化（原来这里是 0.50/0.02 的安全起步值）。
 * 通道号与车轮位置的对应按 motor.c 的 motor_mount_reverse 表：
 *   通道 1（索引 0）左后   Kp 0.750  Ki 0.079
 *   通道 2（索引 1）右后   Kp 0.080  Ki 0.080
 *   通道 3（索引 2）右前   Kp 0.658  Ki 0.062
 *   通道 4（索引 3）左前   Kp 0.825  Ki 0.075
 *
 * 【通道 2 的 Kp 是 0.080，比另外三路小将近 10 倍 —— 这不是笔误就是真调出来的】
 * 三个轮子都在 0.65~0.83，只有它 0.08，量级上很可疑（0.80 漏了个 0 的可能性
 * 很大）。若是笔误，只需把下面 0.080f 改 0.800f 重编；若确实是实测值，
 * 请留意这台在四轮同速时会不会明显拖后腿（同样的误差只给 1/10 的输出）。
 * 调参时可用 `?` 回读核对：#KP=0.080 说明确实是这个值。
 *
 * 量纲：输出/误差都按「占空比百分比 0~100」，所以 Kp=0.75 的物理含义是
 *       「误差 100%（≈整条 1657 mm/s 量程）才给 75% 占空比」。
 * 上下限：kp ≤ TUNE_KP_MAX(3.0)、ki ≤ TUNE_KI_MAX(0.5)，命令行给超了会被
 *       静默夹住 —— 上面四组都在范围内。
 * kd 一律留 0：速度环有测速低通，再加微分只会放大量化噪声。
 * ========================================================================== */
static const float k_kp_default[TUNE_MOTOR_COUNT] = {0.750f, 0.80f, 0.658f, 0.825f};
static const float k_ki_default[TUNE_MOTOR_COUNT] = {0.079f, 0.080f, 0.062f, 0.075f};
static const float k_kd_default[TUNE_MOTOR_COUNT] = {0.000f, 0.000f, 0.000f, 0.000f};

/* 四组 PID 各自独立：四个电机的摩擦力、编码器方向都不一样，
 * 共用一组参数时总有一个轮子拖后腿。 */
static PID_t s_pid[TUNE_MOTOR_COUNT];

/* 下面几个变量在 TIM7 中断里读写、在主循环里写，必须 volatile，
 * 否则编译器会把主循环里的读取优化成常量。单次 32 位读写是原子的，
 * 所以不需要临界区；但「三个增益同时生效」做不到（改到一半来一拍）。 */
static volatile float   s_target[TUNE_MOTOR_COUNT];  /* 目标速度 mm/s，可负 */
static volatile float   s_speed[TUNE_MOTOR_COUNT];   /* 低通后的实测速度 mm/s */
static volatile float   s_out[TUNE_MOTOR_COUNT];     /* PID 输出，占空比 %，0~100 */
static volatile float   s_open[TUNE_MOTOR_COUNT];    /* 开环占空比 %，<0 表示走闭环 */
static volatile uint8_t s_ch;                        /* 当前调哪一组，0~3 */
static volatile uint8_t s_running;                   /* 0 = 停车，1 = 闭环 */
/* 波形闸门：0 = 不发帧（初始化阶段），1 = 正常 50 Hz。
 * 为什么需要它：TIM7 在 encoder_init() 里就启动了，比本文件的 Init 还早，
 * 会跟启动日志抢 USART2->DR，把日志截成半行。详见 PidTune_WaveEnable。 */
static volatile uint8_t s_wave_on;

/* ------------------------------------------------------------------ 小工具 */

static float clampf(float v, float lo, float hi)
{
    if (!(v == v))            /* NaN：非法值一律回落到下限 */
    {
        return lo;
    }
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

static char lower(char c)
{
    if (c >= 'A' && c <= 'Z')
    {
        return (char)(c - 'A' + 'a');
    }
    return c;
}

/** @brief 是不是「命令词到此结束」的分隔符（和 pidcmd.c 那套保持一致） */
static int is_sep(char c)
{
    return (c == '\0' || c == ' ' || c == '\t' ||
            c == ':'  || c == '=' || c == ',');
}

/** @brief 回一行 "#为什么"，行首 '#' 会被 VOFA 当非法数据丢掉，不会画成尖峰 */
static void reply_err(const char *why)
{
    char b[24];
    int  n = sprintf(b, "#%s\r\n", why);

    if (n > 0)
    {
        Serial_SendAsync(b, (uint16_t)n);
    }
}

/** @brief "?" 的尾巴：本工程自己的字段（pidcmd 只管 KP/KI/KD） */
static void report_tail(void)
{
    char b[40];
    int  n = sprintf(b, " TGT=%d CH%u %s\r\n",
                     (int)s_target[s_ch],
                     (unsigned)(s_ch + 1U),
                     (s_running != 0U) ? "RUN" : "STOP");

    if (n > 0)
    {
        Serial_SendAsync(b, (uint16_t)n);
    }
}

/** @brief 把调参模块绑到「当前 ch」这一组的三个增益上（换 ch 后必须重绑） */
static void rebind(void)
{
    PidCmd_Bind(&s_pid[s_ch].Kp, &s_pid[s_ch].Ki, &s_pid[s_ch].Kd,
                TUNE_KP_MAX, TUNE_KI_MAX, TUNE_KD_MAX);
}

/** @brief 全部停车：目标清零、退开环、清积分，但不清增益 */
static void all_stop(void)
{
    uint32_t i;

    for (i = 0U; i < TUNE_MOTOR_COUNT; ++i)
    {
        s_target[i] = 0.0f;
        s_open[i]   = -1.0f;
        PID_Reset(&s_pid[i]);
    }
    s_running = 0U;
}

/* ------------------------------------------------------------------ 命令 */

/**
  * @brief  带编号的增益命令：kp3 0.6 / ki4 0.05 / kd2 0.1
  * @retval 1 = 认这条，0 = 不是本格式（交回给 pidcmd 或报错）
  * @note   为什么单独做一套：四个轮子经常需要不同的参数，而 pidcmd 那套
  *         只认「当前 ch」这一组。有了 kp1~kp4，VOFA 里既可以只放 4 个滑块
  *         走 ch 切换，也可以放 12 个滑块分别给，不用改固件。
  */
static int gain_numbered(const char *s)
{
    char        which;
    char        idx;
    char        tail;
    const char *p;
    float       v;
    float      *dst;
    float       hi;

    if (lower(s[0]) != 'k')
    {
        return 0;
    }

    which = lower(s[1]);
    if (which != 'p' && which != 'i' && which != 'd')
    {
        return 0;
    }

    idx = s[2];
    if (idx < '1' || idx > '4')
    {
        return 0;
    }

    /* 词后必须是分隔符，否则 "kp30" 这种粘连写法会被当成 kp3=0 */
    tail = s[3];
    if (tail != '\0' && tail != ' ' && tail != '\t' &&
        tail != ':' && tail != '=' && tail != ',')
    {
        return 0;
    }

    p = Cmd_SkipWord(s);
    if (Cmd_ParseFloat(p, &v) == 0)
    {
        reply_err("NOVAL");
        return 1;
    }

    if (which == 'p')      { dst = &s_pid[idx - '1'].Kp; hi = TUNE_KP_MAX; }
    else if (which == 'i') { dst = &s_pid[idx - '1'].Ki; hi = TUNE_KI_MAX; }
    else                   { dst = &s_pid[idx - '1'].Kd; hi = TUNE_KD_MAX; }

    *dst = clampf(v, 0.0f, hi);    /* 负增益会让系统反向发散，直接归零 */
    return 1;
}

/**
  * @brief  解析并执行一条命令
  * @note   只在主循环调用：有 sprintf 和浮点乘除，不能进中断。
  *         不认识的命令回一行 #ERR 带原文，免得打错字还以为生效了。
  */
static void cmd_handle(const char *s)
{
    const char *p;
    float       v;
    uint32_t    i;

    while (*s == ' ' || *s == '\t')
    {
        s++;
    }
    if (*s == '\0')
    {
        return;                             /* 空行 */
    }

    /* pid / kp / ki / kd / ? 先交给 pidcmd（改的是当前 ch 那一组） */
    if (PidCmd_Handle(s))
    {
        return;
    }

    /* kp3 0.6 这种带编号的 */
    if (gain_numbered(s))
    {
        return;
    }

    /* ---- ch3 / ch4：连写形式 ----
     * 【为什么单独认一下】VOFA 的按钮绑的是一条固定文本，手敲 "ch3" 比
     * "ch 3" 自然得多；而下面的 Cmd_Match("ch") 要求词后必须是分隔符，
     * 所以 "ch3" 会掉进最后的 #ERR 分支 —— 现象是"按钮按了没反应"，
     * 却又不报错（因为 #ERR 在波形里被 '#' 挡掉了），最难判断。
     * 放在 Cmd_Match("ch") 之前，两者互不干扰："ch" 后面是 '\0'，
     * 这里 s[2] 要求 '1'~'4'，天然匹配不上。 */
    if (lower(s[0]) == 'c' && lower(s[1]) == 'h' &&
        s[2] >= '1' && s[2] <= '4' && is_sep(s[3]))
    {
        s_ch = (uint8_t)(s[2] - '1');
        rebind();
        return;
    }

    /* ---- ch <1~4>：切调哪一路，增益指针要重绑 ---- */
    if (Cmd_Match(s, "ch"))
    {
        p = Cmd_SkipWord(s);
        if (Cmd_ParseFloat(p, &v) == 0)
        {
            reply_err("NOVAL");
            return;
        }
        if (v < 1.0f) { v = 1.0f; }
        if (v > 4.0f) { v = 4.0f; }
        s_ch = (uint8_t)((int)v - 1);
        rebind();
        return;
    }

    /* ---- tgt <mm/s>：当前 ch 的目标，可负（负 = 反转） ---- */
    if (Cmd_Match(s, "tgt") || Cmd_Match(s, "target"))
    {
        p = Cmd_SkipWord(s);
        if (Cmd_ParseFloat(p, &v) == 0)
        {
            reply_err("NOVAL");
            return;
        }
        s_target[s_ch] = clampf(v, -TUNE_TGT_MAX, TUNE_TGT_MAX);
        s_open[s_ch]   = -1.0f;
        /* 改目标就清积分：带着上一段攒的"欠账"起步会先冲一下，
         * 很容易被误判成"这个 Ki 太大"。 */
        PID_Reset(&s_pid[s_ch]);
        s_running = 1U;                     /* 给了目标就开跑，省得忘了 run */
        return;
    }

    /* ---- t <mm/s>：单通道滑块专用目标（tgt 的功能超集里只差两件事）----
     *
     * 【为什么要多这一条，而不是把 tgt 改掉】
     *   tgt 是「敲一次」用的命令，t 是「滑块连续拖」用的命令。滑块拖一格发
     *   一条，一秒能发几十条，两者的正确行为不一样，硬合成一条必然有一边别扭。
     *
     * 与 tgt 的差异（其余完全相同：mm/s 量纲、可负、自动转闭环、自动 run）：
     *
     *   1) **不清积分**。
     *      tgt 每拖一格就 PID_Reset 一次，ErrorInt 永远是 0，于是不管你 Ki
     *      拖到多大，积分项永远贡献 0 —— 表现出来就是"Ki 好像没生效"，
     *      实际上是被自己每格清掉了。调 Ki 时必须用 t。
     *      代价：从上一段速度直接跳到新目标时没有"干净起步"，靠 pid.c 里
     *      的抗饱和和钳位兜着，正常范围内不会冲。
     *
     *   2) **其余三路目标清零**（切到开环 -1，顺手清积分）。
     *      单通道调试最常见的翻车：1 号调到 500 觉得行了，按 ch2 去调 2 号，
     *      结果 1 号还在以 500 mm/s 闷头转 —— 车在地上就直接跑了。
     *      这里替你把上一路收干净，一条命令完成"只让这一路转"。
     *      需要四轮同转时用 all，需要两路对比时用 tgt，两者都没被动过。
     *
     * 【量纲】和 tgt 一样是 mm/s 物理真值，满量程 TUNE_TGT_MAX(1657)。
     *         VOFA 滑块 from/to 直接填 -1657 ~ 1657，不用换算。 */
    if (Cmd_Match(s, "t"))
    {
        p = Cmd_SkipWord(s);
        if (Cmd_ParseFloat(p, &v) == 0)
        {
            reply_err("NOVAL");
            return;
        }

        for (i = 0U; i < TUNE_MOTOR_COUNT; ++i)
        {
            if (i != (uint32_t)s_ch)
            {
                s_target[i] = 0.0f;
                s_open[i]   = -1.0f;
                PID_Reset(&s_pid[i]);
            }
        }

        s_target[s_ch] = clampf(v, -TUNE_TGT_MAX, TUNE_TGT_MAX);
        s_open[s_ch]   = -1.0f;
        s_running      = 1U;
        return;
    }

    /* ---- all <mm/s>：四轮同速，验直线 / 比一致性用 ---- */
    if (Cmd_Match(s, "all") || Cmd_Match(s, "line"))
    {
        p = Cmd_SkipWord(s);
        if (Cmd_ParseFloat(p, &v) != 0)
        {
            v = clampf(v, -TUNE_TGT_MAX, TUNE_TGT_MAX);
        }
        else
        {
            v = 0.0f;                       /* 不带参数就是四轮清零 */
        }
        for (i = 0U; i < TUNE_MOTOR_COUNT; ++i)
        {
            s_target[i] = v;
            s_open[i]   = -1.0f;
            PID_Reset(&s_pid[i]);
        }
        s_running = (v != 0.0f) ? 1U : 0U;
        return;
    }

    /* ---- o <±100>：开环自检，不过 PID。o 0 = 退出开环并停车 ---- */
    if (Cmd_Match(s, "o") || Cmd_Match(s, "open"))
    {
        p = Cmd_SkipWord(s);
        if (Cmd_ParseFloat(p, &v) == 0)
        {
            reply_err("NOVAL");
            return;
        }
        if (v == 0.0f)
        {
            s_open[s_ch]   = -1.0f;         /* 回闭环 */
            s_target[s_ch] = 0.0f;
        }
        else
        {
            s_open[s_ch]   = clampf(v, -TUNE_OUT_MAX, TUNE_OUT_MAX);
            s_target[s_ch] = 0.0f;
        }
        PID_Reset(&s_pid[s_ch]);
        return;
    }

    /* ---- oall <±100>：四轮同时开环，验"同占空比是否同转速" ----
     * 单条 o 一次只能动一路，横向比转速时要来回切 ch，很容易记混哪个看过。 */
    if (Cmd_Match(s, "oall") || Cmd_Match(s, "openall"))
    {
        p = Cmd_SkipWord(s);
        if (Cmd_ParseFloat(p, &v) == 0)
        {
            reply_err("NOVAL");
            return;
        }
        for (i = 0U; i < TUNE_MOTOR_COUNT; ++i)
        {
            if (v == 0.0f)
            {
                s_open[i]   = -1.0f;
                s_target[i] = 0.0f;
            }
            else
            {
                s_open[i]   = clampf(v, -TUNE_OUT_MAX, TUNE_OUT_MAX);
                s_target[i] = 0.0f;
            }
            PID_Reset(&s_pid[i]);
        }
        return;
    }

    /* ---- enc / enc zero：读编码器累计计数 ----
     * 用来验"手转一圈 = 40000 计数"。速度积分看着也能估，但一圈只有
     * 1.5 秒左右、算出来的误差比要验的东西还大，必须直接数计数。 */
    if (Cmd_Match(s, "enc"))
    {
        char b[96];
        int  n;

        if (Cmd_Match(Cmd_SkipWord(s), "zero") || Cmd_Match(Cmd_SkipWord(s), "0"))
        {
            for (i = 0U; i < TUNE_MOTOR_COUNT; ++i)
            {
                encoder_reset_total((int)(i + 1U));
            }
            reply_err("ENCCLEAR");
            return;
        }

        /* 转成 long 打印：AC5 的 sprintf 不支持 %lld，硬写会输出乱码。 */
        n = sprintf(b, "#ENC %ld %ld %ld %ld\r\n",
                    (long)encoder_get_total(1), (long)encoder_get_total(2),
                    (long)encoder_get_total(3), (long)encoder_get_total(4));
        if (n > 0)
        {
            Serial_SendAsync(b, (uint16_t)n);
        }
        return;
    }

    /* ---- run / stop ---- */
    if (Cmd_Match(s, "run"))
    {
        PID_Reset(&s_pid[s_ch]);
        s_running = 1U;
        return;
    }
    if (Cmd_Match(s, "stop"))
    {
        all_stop();
        return;
    }

    /* ---- zero：当前 ch 三个增益清零（安全阀） ---- */
    if (Cmd_Match(s, "zero"))
    {
        s_pid[s_ch].Kp = 0.0f;
        s_pid[s_ch].Ki = 0.0f;
        s_pid[s_ch].Kd = 0.0f;
        PID_Reset(&s_pid[s_ch]);
        return;
    }

    /* 都不认识：把原文带回去，这样"板子到底收到了什么"不用猜 */
    {
        char b[64];
        int  n = sprintf(b, "#ERR %.40s\r\n", s);
        if (n > 0)
        {
            Serial_SendAsync(b, (uint16_t)n);
        }
    }
}

/* ------------------------------------------------------------------ 对外 */

void PidTune_Init(void)
{
    uint32_t i;

    for (i = 0U; i < TUNE_MOTOR_COUNT; ++i)
    {
        PID_Init(&s_pid[i], k_kp_default[i], k_ki_default[i], k_kd_default[i],
                 TUNE_OUT_MAX);
        s_target[i] = 0.0f;
        s_speed[i]  = 0.0f;
        s_out[i]    = 0.0f;
        s_open[i]   = -1.0f;
    }

    s_ch      = 0U;
    s_running = 0U;

    Serial_Init();                 /* 挂 USART2 的收中断，并清溢出标志 */
    rebind();                      /* 绑到 1 号电机 */
    PidCmd_SetReportTail(report_tail);
}

void PidTune_Tick10ms(void)
{
    Motor    m;
    uint32_t i;
    float    raw;

    /* ---------- 测速：每个电机这一拍只能读一次 ---------- */
    for (i = 0U; i < TUNE_MOTOR_COUNT; ++i)
    {
        raw = encoder_get_speed((int)(i + 1U));
        s_speed[i] += TUNE_SPEED_ALPHA * (raw - s_speed[i]);
    }

    /* ---------- 算输出并写 PWM ---------- */
    for (i = 0U; i < TUNE_MOTOR_COUNT; ++i)
    {
        if (s_open[i] >= 0.0f)
        {
            s_out[i] = s_open[i];               /* 开环自检：直接用给定值 */
        }
        else if ((s_running != 0U) && (s_target[i] != 0.0f))
        {
            /* 【量纲必须归一，2026-11-22 修】PID 内部统一用「占空比百分比」，
             * 所以 mm/s 要先除 TUNE_MM_PER_DUTY(16.57)。目标、反馈都要除，
             * 两个一起除等于误差也除了，量纲一致。
             *
             * 漏掉这一步的后果（真踩过）：Kp=0.5 从"误差 100% 给 50% 输出"
             * 变成"误差 200 mm/s 就给满输出"，等效增益凭空放大 16.57 倍。
             * 表现是四台轮流在某台爆掉——其实是整环在极限环里，哪台先炸
             * 只看死区和摩擦的微小差异，跟编码器符号一点关系都没有。
             * 排查时最容易误判成"某某电机符号反了"，一定要先看这个。 */
            s_out[i] = PID_Calc(&s_pid[i],
                                s_target[i] / TUNE_MM_PER_DUTY,
                                s_speed[i]  / TUNE_MM_PER_DUTY);
        }
        else
        {
            s_out[i] = 0.0f;
        }

        /* 正负号 → 方向位。motor_run() 只认「幅值 + 方向」，
         * 所以这里必须自己拆，不能把负数直接塞进 speed。
         * 幅值还要从「占空比 %」换成 mm/s —— 这是全工程唯一一次换算：
         * motor_run() → speed2pluse() 认的是 mm/s，它再除满量程得到占空比，
         * 一进一出正好抵消，所以百分比在这层以上保持一致。 */
        m.motor_id = (int)(i + 1U);
        m.speed    = (double)((s_out[i] >= 0.0f) ? s_out[i] : -s_out[i]) *
                     (double)TUNE_MM_PER_DUTY;
        m.dir      = (s_out[i] >= 0.0f) ? 1 : 0;
        m.p        = s_pid[i].Kp;               /* 仅作记录，motor_run 不用 */
        m.i        = s_pid[i].Ki;
        m.d        = s_pid[i].Kd;
        motor_run(m);
    }

    /* ---------- 波形：每 2 拍一帧 = 50 Hz ----------
     * 列数必须每帧恒定（FireWater 按逗号认列，少发会静默错位）。
     * 传的是「乘 10 后的整数」，vofa.c 会拼成 "305" → 显示 30.5。 */
    {
        static uint8_t div = 0U;
        int32_t        f[TUNE_WAVE_CH];

        if (s_wave_on == 0U)
        {
            /* 初始化阶段：不占串口，让启动日志完整打出来 */
            div = 0U;
        }
        else if (++div >= TUNE_WAVE_DIV)
        {
            div = 0U;

            /* CH0 = 当前 ch 的指令，统一用 mm/s（不是 %）。
             * 闭环：s_target 本来就是 mm/s。
             * 开环：s_open 是占空比 %，必须乘 TUNE_MM_PER_DUTY 换成等效 mm/s，
             *       否则同一条曲线一半是 mm/s 一半是 %，看波形会误判成"目标跳了"。
             * 原来固定发 s_target[s_ch]，开环验证时那一路永远是 0，
             * 看上去像"指令丢了"，实际是指令在 s_open 里。 */
            f[0] = (int32_t)(((s_open[s_ch] >= 0.0f)
                                  ? (s_open[s_ch] * TUNE_MM_PER_DUTY)
                                  : s_target[s_ch]) * 10.0f);
            for (i = 0U; i < TUNE_MOTOR_COUNT; ++i)
            {
                f[1U + i] = (int32_t)(s_speed[i] * 10.0f);
            }

            VOFA_SendFix1(f, (uint8_t)TUNE_WAVE_CH);
        }
    }
}

void PidTune_Process(void)
{
    char line[64];

    /* 一次把积压的行都处理掉，不阻塞 */
    while (Serial_GetLine(line, (uint16_t)sizeof(line)))
    {
        cmd_handle(line);
    }
}

void PidTune_WaveEnable(void)
{
    s_wave_on = 1U;
}
