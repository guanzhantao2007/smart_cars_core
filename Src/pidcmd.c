#include "pidcmd.h"
#include "serial_async.h"
#include <stdio.h>

/* 分隔符只有这一套，三个解析函数共用 —— 见 Cmd_Match 的说明 */
static struct
{
    float *kp;
    float *ki;
    float *kd;
    float  kp_max;
    float  ki_max;
    float  kd_max;
    void (*tail)(void);
} s_cfg;

/* ------------------------------------------------------------------ 对外接口 */

void PidCmd_Bind(float *kp, float *ki, float *kd,
                 float kp_max, float ki_max, float kd_max)
{
    s_cfg.kp = kp;
    s_cfg.ki = ki;
    s_cfg.kd = kd;
    s_cfg.kp_max = kp_max;
    s_cfg.ki_max = ki_max;
    s_cfg.kd_max = kd_max;
}

void PidCmd_SetReportTail(void (*fn)(void))
{
    s_cfg.tail = fn;
}

/* ------------------------------------------------------------------ 解析工具 */

/**
  * @brief  s 是否以完整单词 word 开头（大小写不敏感）
  * @note   必须检查词后面是不是分隔符，否则 "kp0.143" 这种粘连写法会把
  *         "kp0" 当命令、把 ".143" 当参数，静默设进错值。
  *         分隔符含 ':' 是因为 VOFA 的滑条默认发 "控件名:数值"。
  */
int Cmd_Match(const char *s, const char *word)
{
    while (*word != '\0')
    {
        char a = *s++;
        char b = *word++;

        if (a >= 'A' && a <= 'Z')
        {
            a = (char)(a - 'A' + 'a');
        }
        if (a != b)
        {
            return 0;
        }
    }
    return (*s == '\0' || *s == ' ' || *s == '\t' ||
            *s == ':'  || *s == '=' || *s == ',');
}

/** @brief 跳过命令词和紧跟的分隔符，返回参数起点 */
const char *Cmd_SkipWord(const char *s)
{
    while (*s != '\0' && *s != ' ' && *s != '\t' &&
           *s != ':' && *s != '=' && *s != ',')
    {
        s++;
    }
    while (*s == ' ' || *s == '\t' || *s == ':' || *s == '=' || *s == ',')
    {
        s++;
    }
    return s;
}

/**
  * @brief  读一个定点小数
  * @retval 数字后面的第一个字符；不是数字则返回 0
  * @note   不用 atof/strtof：AC5 下会把整套 locale 浮点库拉进镜像，几 KB 起。
  *         也不接受指数写法（1.5e-3）—— 会被读成 1.5 再夹到上限，静默设错值。
  */
const char *Cmd_ParseFloat(const char *s, float *out)
{
    float v    = 0.0f;
    float frac = 0.1f;
    int   neg  = 0;
    int   got  = 0;

    while (*s == ' ' || *s == '\t' || *s == ',' ||
           *s == ':' || *s == '=' || *s == '%')
    {
        s++;
    }

    if (*s == '-' || *s == '+')
    {
        neg = (*s == '-');
        s++;
    }

    while (*s >= '0' && *s <= '9')
    {
        v = v * 10.0f + (float)(*s - '0');
        s++;
        got = 1;
    }

    if (*s == '.')
    {
        s++;
        while (*s >= '0' && *s <= '9')
        {
            v += frac * (float)(*s - '0');
            frac *= 0.1f;
            s++;
            got = 1;
        }
    }

    if (!got || *s == 'e' || *s == 'E')
    {
        return 0;
    }

    *out = neg ? -v : v;
    return s;
}

/* ------------------------------------------------------------------ 内部实现 */

/** @brief 负值归零、超上限夹到上限（负增益会让系统反向发散） */
static float Clampf(float v, float hi)
{
    if (v < 0.0f) { return 0.0f; }
    if (v > hi)   { return hi;   }
    return v;
}

/** @brief 拆成"整数位 + 三位小数"，因为 AC5 的 sprintf 不支持 %f */
static void Split3(float v, int *ip, int *fp)
{
    int i;
    int f;

    if (v < 0.0f) { v = 0.0f; }

    i = (int)v;
    f = (int)((v - (float)i) * 1000.0f + 0.5f);
    if (f >= 1000)            /* 0.9996 四舍五入要进位 */
    {
        i++;
        f -= 1000;
    }

    *ip = i;
    *fp = f;
}

/**
  * @brief 回一行 "#xxx"
  * @note  行首 '#' 是故意的：VOFA 的波形协议把它判为非法数据丢弃，
  *        不会在图上画出一个莫名尖峰。
  *        没这行回显，"命令被丢掉"和"串口根本没通"现象完全一样，没法区分。
  */
static void Cmd_Err(const char *why)
{
    char b[24];
    int  n = sprintf(b, "#%s\r\n", why);

    if (n > 0)
    {
        Serial_SendAsync(b, (uint16_t)n);
    }
}

/** @brief 回读当前增益：KP/KI/KD 由本模块出，其余字段由调用方的回调追加 */
static void Report(void)
{
    char b[64];
    int  i1, f1, i2, f2, i3, f3;
    int  n;

    Split3(*s_cfg.kp, &i1, &f1);
    Split3(*s_cfg.ki, &i2, &f2);
    Split3(*s_cfg.kd, &i3, &f3);

    n = sprintf(b, "#KP=%d.%03d KI=%d.%03d KD=%d.%03d", i1, f1, i2, f2, i3, f3);
    if (n > 0)
    {
        Serial_SendAsync(b, (uint16_t)n);
    }

    if (s_cfg.tail != 0)
    {
        s_cfg.tail();                     /* 回调自己负责行尾的 \r\n */
    }
    else
    {
        Serial_SendAsync("\r\n", 2U);
    }
}

int PidCmd_Handle(const char *line)
{
    const char *s = line;
    const char *p;
    float       v;
    char        which;

    while (*s == ' ' || *s == '\t')
    {
        s++;
    }

    if (*s == '\0')
    {
        return 1;                         /* 空行：算已处理，调用方不用再判 */
    }

    if (s_cfg.kp == 0)
    {
        return 0;                         /* 没绑定就不认账，交给调用方 */
    }

    /* ---- ? / help ---- */
    if (*s == '?' || Cmd_Match(s, "help"))
    {
        Report();
        return 1;
    }

    /* ---- pid <kp> <ki> <kd> ---- */
    if (Cmd_Match(s, "pid"))
    {
        p = Cmd_SkipWord(s);
        if ((p = Cmd_ParseFloat(p, &v)) == 0) { Cmd_Err("NOVAL"); return 1; }
        *s_cfg.kp = Clampf(v, s_cfg.kp_max);

        if ((p = Cmd_ParseFloat(p, &v)) == 0) { Cmd_Err("NOVAL"); return 1; }
        *s_cfg.ki = Clampf(v, s_cfg.ki_max);

        if ((p = Cmd_ParseFloat(p, &v)) == 0) { Cmd_Err("NOVAL"); return 1; }
        *s_cfg.kd = Clampf(v, s_cfg.kd_max);
        return 1;
    }

    /* ---- kp / ki / kd <v> ---- */
    if (Cmd_Match(s, "kp") || Cmd_Match(s, "ki") || Cmd_Match(s, "kd"))
    {
        /* 第二个字母决定写哪个。这里也要归一化，否则 "KP 0.1" 会掉进
           else 分支被当成 Kd —— 改 Kp 却动了 Kd。 */
        which = s[1];
        if (which >= 'A' && which <= 'Z')
        {
            which = (char)(which - 'A' + 'a');
        }

        p = Cmd_SkipWord(s);
        if (Cmd_ParseFloat(p, &v) == 0)
        {
            Cmd_Err("NOVAL");             /* 有命令词没数字：回一行，别静默 */
            return 1;
        }

        if (which == 'p')      { *s_cfg.kp = Clampf(v, s_cfg.kp_max); }
        else if (which == 'i') { *s_cfg.ki = Clampf(v, s_cfg.ki_max); }
        else                   { *s_cfg.kd = Clampf(v, s_cfg.kd_max); }
        return 1;
    }

    return 0;                             /* 不是本模块的命令 */
}
