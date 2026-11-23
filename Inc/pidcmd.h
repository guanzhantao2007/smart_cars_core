#ifndef __PIDCMD_H
#define __PIDCMD_H

#include <stdint.h>

/* ============================================================================
 * 串口调 PID 命令模块（FireWater/VOFA 文本命令的下行部分）
 *
 * 认这几条命令，一行一条，\r 或 \n 结尾：
 *   pid <kp> <ki> <kd>   一次设全三个
 *   kp  <v>              只改 Kp（ki / kd 同理）
 *   ?                   回一行 "#KP=0.143 KI=0.043 KD=0.000"
 *
 * 三件事保证它可以整体搬到别的工程：
 *   1. 只绑三个 float 的地址，不绑 PID 结构体 —— 别人的 PID 库叫什么都能用；
 *   2. 不认识的行返回 0，交给调用方处理自己的命令（tgt / run / …）；
 *   3. 不读写任何全局状态，只有一个模块内静态实例。
 *
 * 调用位置：主循环（有 sprintf 和浮点乘法，不能进中断）。
 * ========================================================================= */

/**
  * @brief  绑定要调的那一组增益
  * @param  kp/ki/kd  三个 float 的地址（直接传 &pid.Kp 这种）
  * @param  *_max     各自的量程上限，超出的值会被夹到这里
  * @note   换轮子（或换 PID 实例）后必须重新调用一次，否则还在改旧的那组。
  */
void PidCmd_Bind(float *kp, float *ki, float *kd,
                 float kp_max, float ki_max, float kd_max);

/**
  * @brief  注册"回读时追加字段"的回调，可为 0
  * @note   本模块只负责 KP/KI/KD 三个字段；调用方若还想报目标、通道、
  *         运行状态，写个函数在这里输出 ` TGT=200 CH2\r\n` 即可。
  *         未注册时模块自己补 "\r\n"。
  */
void PidCmd_SetReportTail(void (*fn)(void));

/**
  * @brief  尝试解析并执行一条命令
  * @param  line 一整行，以 '\0' 结尾（换行符已由 Serial_GetLine 去掉）
  * @retval 1 = 这条命令归我，已处理完（含报错）
  * @retval 0 = 不是我的命令，请调用方继续判断
  */
int PidCmd_Handle(const char *line);

/* ---------------------------------------------------------------------------
 * 命令解析工具：本模块内部在用，也开放给调用方解析自己的参数。
 * 三个函数共享同一套分隔符（空格 制表 : = ,），因此不可能再出现
 * "一个改了冒号、另一个没改"导致命令静默失效的老问题。
 * ------------------------------------------------------------------------ */

/** @brief s 是否以完整单词 word 开头（大小写不敏感，词后须是分隔符或结尾） */
int Cmd_Match(const char *s, const char *word);

/** @brief 跳过命令词和紧跟的分隔符，返回参数起点 */
const char *Cmd_SkipWord(const char *s);

/** @brief 读一个定点小数（不接受 1.5e-3），失败返回 0，成功返回数字后面的位置 */
const char *Cmd_ParseFloat(const char *s, float *out);

#endif
