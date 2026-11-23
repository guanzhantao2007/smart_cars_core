#include "vofa.h"
#include "serial_async.h"
#include <stdio.h>

/*
 * 一列最长占多少字节：
 *   int32 最小值 -2147483648 是 11 个字符，前面再加一个分隔逗号 = 12。
 *   10 列的最坏情况：11 + 9×12 = 119，加 '\n' = 120，加 '\0' = 121。
 *   缓冲取 10×12+2 = 122，够，且留 1 字节余量。
 *
 * 【为什么不做"够用就行"的估算】
 *   如果按"我的转速只有三四位数"开 32 字节，将来某个数（比如误差算出
 *   -1600）刚好超了，sprintf 会静默写越界 —— 而且破坏的可能是紧挨着的
 *   另一个静态变量。这种 bug 表现为"某个无关的功能偶尔不对"，最难查。
 */
#define VOFA_COL_BYTES   12
#define VOFA_BUF_SIZE    (VOFA_MAX_CH * VOFA_COL_BYTES + 2)

/**
  * @brief  把 n 个整数按 FireWater 文本协议发出去（"v0,v1,...,vn\n"）
  * @note   只在中断上下文调用。
  *
  * 【用 %d 不用 %f】
  *   AC5 下浮点格式化会拖进整套软件浮点例程，一次上百微秒；而这个函数
  *   跑在 1ms 节拍中断里（每 40ms 才真正发一帧），没必要为小数位付这个代价。
  *   代价是 1 个单位的台阶 —— rpm 精确到 1，在 150 附近等于 0.67%。
  *   真需要 0.1 rpm 时，不要在调用方乘 10 就完事（那会让"误差"列也跟着
  *   被放大、VOFA 里还得手工配系数），改成在这里统一按通道配缩放更清楚。
  *
  * 【为什么第一列不带逗号】
  *   FireWater 的解析是从行首开始按逗号切，行首多一个逗号会切出一个空字段，
  *   表现为"所有通道整体往前错一位"，图上还照样有曲线。
  */
void VOFA_SendInts(const int32_t *v, uint8_t n)
{
    static char line[VOFA_BUF_SIZE];
    uint16_t    i = 0U;
    uint8_t     k;
    int         w;

    if (v == 0 || n == 0U || n > VOFA_MAX_CH)
    {
        return;
    }

    for (k = 0U; k < n; k++)
    {
        /* 每次都从 line[i] 开始写一列，sprintf 会顺带写一个 '\0'，
           下一列从 i 覆盖它 —— 所以缓冲区长度必须按最坏情况开够。 */
        w = (k == 0U) ? sprintf(&line[i], "%d",  (int)v[k])
                      : sprintf(&line[i], ",%d", (int)v[k]);
        if (w <= 0)
        {
            return;                      /* 格式化失败：这一帧不发，别发半截 */
        }
        i = (uint16_t)(i + (uint16_t)w);
    }

    line[i++] = '\n';

    /* Serial_SendAsync 是"入缓冲即返回"，不等 UART 发完，
       所以在中断里调用不会拖长节拍。真正的阻塞风险在 serial_async.c 那边：
       上一帧没发完时它会丢掉这一整帧（丢帧比等发送完成安全）。 */
    Serial_SendAsync(line, i);
}

/**
  * @brief  把「乘 10 后的整数」写成 "x.y" 形式，返回写入字节数
  * @note   负号单独写，绝对值拆成整数位和一位小数。
  *         不用 sprintf 的 %f（AC5 不支持），也不用 %d 拼完再插小数点——
  *         那样负号位置会算错。
  */
static int put_fix1(char *dst, int32_t v10)
{
    uint32_t a;
    int      n = 0;

    if (v10 < 0)
    {
        dst[n++] = '-';
        a = (uint32_t)(-v10);
    }
    else
    {
        a = (uint32_t)v10;
    }

    /* a/10 = 整数位，a%10 = 唯一的小数位，正好一位，不需要四舍五入 */
    return n + sprintf(&dst[n], "%d.%d", (int)(a / 10U), (int)(a % 10U));
}

/**
  * @brief  按 FireWater 文本协议发 n 列「一位小数」数据
  * @note   格式与 VOFA_SendInts 完全一致（逗号分隔、'\n' 结尾），
  *         只是每个数多了小数点，VOFA 那边不需要改任何配置。
  *         列数必须每帧恒定，理由见 VOFA_SendInts 的说明。
  */
void VOFA_SendFix1(const int32_t *v10, uint8_t n)
{
    static char line[VOFA_BUF_SIZE];
    uint16_t    i = 0U;
    uint8_t     k;
    int         w;

    if (v10 == 0 || n == 0U || n > VOFA_MAX_CH)
    {
        return;
    }

    for (k = 0U; k < n; k++)
    {
        if (k != 0U)
        {
            line[i++] = ',';
        }

        w = put_fix1(&line[i], v10[k]);
        if (w <= 0)
        {
            return;                      /* 格式化失败：这一帧不发，别发半截 */
        }
        i = (uint16_t)(i + (uint16_t)w);
    }

    line[i++] = '\n';
    Serial_SendAsync(line, i);
}
