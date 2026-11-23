#include "serial_async.h"
#include "usart.h"

#include <string.h>

/* ---------------- 发送缓冲 ----------------
 * 【2026-11-22 角度环观测改】96 -> 128 字节。
 * 波形从 11 列扩到 14 列，按 vofa.c 的 put_fix1 实际输出算最坏帧长：
 *   每列 "-3000.0" 7 字节 × 14 = 98，加 13 个逗号 = 111，再加行尾 '\n' = 112。
 * 96 装不下 —— 而 Serial_SendAsync 是**静默截断**（下面 n = min(len, 96)），
 * 截掉的正好是最后一字节 '\n'。后果比"少几列"严重得多：这一帧会和下一帧
 * 粘成一行；下一帧同样超长、同样缺 '\n'，于是行尾永远不出现，VOFA 一直
 * 等行尾，表现为"波形完全不动"。128 覆盖 112，余 16 字节。
 * 注意：128 只覆盖"每列 ≤8 字符"；角度目标列已在 pid_tune.c 里钳到 ±3000，
 * 保证单列 ≤7 字符，靠这层钳位守住 128 这个上界。
 *
 * 2026-11-22 前的原注（历史，仍成立）：96 字节 = 5 列波形最坏 "-100.0"×5 +
 * 4 个逗号 + '\n' = 35 字节，留两倍余量给命令回显。
 * 这里只放「正在发的那一帧」，不做队列：HAL_UART_Transmit_IT 从头到尾
 * 用同一个指针取数据，缓冲区被覆盖就发出乱码，所以必须 s_tx_busy 守着。
 */
#define TX_BUF_SIZE 128U

static uint8_t          s_tx[TX_BUF_SIZE];
static volatile uint8_t s_tx_busy;      /* 1 = 有一帧正在发 */

/* ---------------- 接收环形缓冲 ----------------
 * 128 字节：一行最长 "pid 0.143 0.043 0.000" 约 23 字节，留 5 倍余量。
 * 满了丢新字节——命令可以重发，绝不能在中断里等。
 * 和发送缓冲完全分开：波形是 50 Hz 的连续流，命令是随机来的短文本，
 * 混在一个缓冲里命令字符会插进波形帧中间，VOFA 上就是一个凹口。
 */
#define RX_BUF_SIZE 128U

static volatile uint8_t  s_rx_buf[RX_BUF_SIZE];
static volatile uint16_t s_rx_head;     /* 中断写 */
static volatile uint16_t s_rx_tail;     /* 主循环读 */

/* HAL_UART_Receive_IT 的单字节落点 */
static uint8_t s_rx_byte;

void Serial_Init(void)
{
    s_tx_busy = 0U;
    s_rx_head = 0U;
    s_rx_tail = 0U;

    /* 先清溢出标志，再挂接收：带着 ORE 进中断会立刻触发一次 ErrorCallback，
     * 白白把刚挂上的接收打断一次。 */
    __HAL_UART_CLEAR_OREFLAG(&huart2);

    /* 优先级 5，低于 TIM7（0）：
     * 故意不让每 87 us 一次的串口中断抢占 10 ms 控制节拍。
     * 代价是字节之间可能被拉长几十微秒——UART 只是变慢，不丢数据；
     * 反过来若给它比控制环高的优先级，节拍抖动更难接受。 */
    HAL_NVIC_SetPriority(USART2_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);
}

void Serial_SendAsync(const char *s, uint16_t len)
{
    uint16_t n;

    if (s == 0 || len == 0U)
    {
        return;
    }

    if (s_tx_busy != 0U)
    {
        return;                          /* 上一帧还在发：丢掉这一帧，不阻塞 */
    }

    n = (len > (uint16_t)TX_BUF_SIZE) ? (uint16_t)TX_BUF_SIZE : len;
    memcpy(s_tx, s, n);

    s_tx_busy = 1U;
    if (HAL_UART_Transmit_IT(&huart2, s_tx, n) != HAL_OK)
    {
        s_tx_busy = 0U;                  /* 句柄忙（几乎不会发生）：放弃这一帧 */
    }
}

/** @brief 发送完成：放开标志，允许下一帧 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        s_tx_busy = 0U;
    }
}

/** @brief 收到一个字节：进环形缓冲后立刻重挂，否则只会收到第一个字节 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t next;

    if (huart->Instance != USART2)
    {
        return;
    }

    next = (uint16_t)((s_rx_head + 1U) % RX_BUF_SIZE);
    if (next != s_rx_tail)               /* 满了就丢这个字节 */
    {
        s_rx_buf[s_rx_head] = s_rx_byte;
        s_rx_head = next;
    }

    (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);
}

/**
  * @brief  串口出错后的恢复（最主要就是 ORE 溢出）
  * @note   本工程 stm32f4xx_it.c 里已经有一个 HAL_UART_ErrorCallback（它管
  *         USART1 的上位机帧错误），所以这里不能再重复定义那个回调，
  *         改为由那边转调本函数。
  *         这一段删不得：HAL 收到 ORE 后会把 RxState 置成 READY 并停掉接收
  *         中断，不在这里重挂，接收就永久停死，表现为「命令发出去没反应、
  *         也不报错」，而且拖滑条、重新上电之前都修不好。
  */
void Serial_RxRecover(void)
{
    __HAL_UART_CLEAR_OREFLAG(&huart2);

    /* 强制置回 READY 再重挂：HAL 各版本在错误路径上是否已经清了 RxState
     * 并不一致，若它仍是 BUSY_RX，HAL_UART_Receive_IT() 会直接返回 HAL_BUSY
     * 而不报错 —— 结果就是"以为恢复了，其实还是收不到"。
     * 本工程的 USART2 只做这一种单字节中断接收，没有 DMA，置 READY 是安全的。 */
    huart2.RxState = HAL_UART_STATE_READY;
    (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);
}

/**
  * @brief  紧急发送：绕过 HAL，直接轮询 TXE 往 DR 写字节
  * @note   调用场景只有 Fault_Report()（初始化失败、串口帧错误）。
  *         三条保护：
  *           1) huart2.Instance 为 0 —— USART2 还没 MX_USART2_UART_Init，
  *              此时写寄存器没意义（时钟都没开），直接返回；
  *           2) 每字节有一个计数上限，等不到 TXE 就撤 —— 引脚/时钟配错时
  *              绝不会把故障上报本身变成第二次死循环；
  *           3) 不碰 gState / RxState，所以不会干扰 HAL 后续自己恢复。
  *         2 MHz 次空转 ≈ 十几毫秒，远大于 115200 下一字节的 87 us。
  */
void Serial_EmergencyPuts(const char *s)
{
    if (s == 0)
    {
        return;
    }
    if (huart2.Instance == 0)
    {
        return;                          /* USART2 尚未初始化 */
    }

    while (*s != '\0')
    {
        uint32_t guard = 2000000U;

        while ((USART2->SR & USART_SR_TXE) == 0U)
        {
            if (--guard == 0U)
            {
                return;                  /* 发不出去就撤，绝不在这里死等 */
            }
        }
        USART2->DR = (uint32_t)((uint8_t)(*s));
        ++s;
    }
}

/* 一行最长多少字节。"pid 0.143 0.043 0.000" = 23，留一倍余量。 */
#define LINE_BUF_SIZE 48U

/* 半行超时（ms）：VOFA 控件不保证在命令末尾补换行符，
 * 收到半行后静默 15 ms 就当成完整一行交出去。
 * 15 ms 的取法：115200 下一个字节约 87 us，主循环一轮远小于这个数，
 * 正常分片到达不会空这么久；而人手拖滑条的事件间隔通常 >20 ms，
 * 取小了会把同一次拖动拆成两行，取大了操作有延迟。 */
#define LINE_IDLE_MS 15U

int Serial_GetLine(char *out, uint16_t max)
{
    static char     lb[LINE_BUF_SIZE];
    static uint16_t n = 0U;
    static uint32_t last_rx = 0U;
    uint8_t         c;
    uint16_t        i;
    uint8_t         got = 0U;

    if (out == 0 || max < 2U)
    {
        return 0;
    }

    while (s_rx_tail != s_rx_head)
    {
        c = s_rx_buf[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1U) % RX_BUF_SIZE);
        got = 1U;

        if (c == '\r' || c == '\n')
        {
            if (n == 0U)
            {
                continue;                /* 空行（"\r\n" 的后半截）直接跳过 */
            }

            for (i = 0U; i < n && i < (uint16_t)(max - 1U); i++)
            {
                out[i] = lb[i];
            }
            out[i] = '\0';

            n = 0U;
            return 1;
        }

        if (n < (uint16_t)(LINE_BUF_SIZE - 1U))
        {
            lb[n++] = (char)c;
        }
        else
        {
            n = 0U;                      /* 超长行整行丢掉，重新找行首，避免错位 */
        }
    }

    /* 只有真吃到字节才刷新时刻：主循环空转也会反复调本函数，
       用「本次调用时刻」计时的话永远不会超时，兜底就成了摆设。 */
    if (got != 0U)
    {
        last_rx = HAL_GetTick();
        return 0;
    }

    if (n > 0U && (uint32_t)(HAL_GetTick() - last_rx) >= LINE_IDLE_MS)
    {
        for (i = 0U; i < n && i < (uint16_t)(max - 1U); i++)
        {
            out[i] = lb[i];
        }
        out[i] = '\0';
        n = 0U;
        return 1;
    }

    return 0;
}
