#include "yaw_loop.h"
#include "imu.h"      /* T09：Tick 只读 IMU_GetYawDeg / IMU_IsReady，imu.c 一个字不改 */
#include <string.h>

/* 模块内部唯一实例：不对外暴露，外部只能通过下面几个函数访问。
 * 这样以后 YawLoop_Tick 长出真逻辑时，状态也只在模块内部流转，
 * 不会多出一个全局符号去和别的模块耦合。
 *
 * 【T04 待办，本轮不做】Tick 一旦接进 TIM7 中断，本结构体里"主循环写、
 * 中断读"的字段（target_deg 等）需要加 volatile —— 对齐的 32 位单次写
 * 在 Cortex-M4 上是原子的，不会撕裂，但编译器可能把值缓存进寄存器，
 * 表现为"命令发了，中断里读到的还是旧值"。现在 Tick 是空的、没有任何
 * 读者，加了 volatile 只会让 memset 那行报警，故本轮不加。 */
static YawLoop_t s_yaw;

/* ===== 方向符号：全模块唯一一处定义（T09 建立，以后只改这里）=====
 *
 * 【2026-11-22 主理人实测｜手转车体，俯视基准】顺时针转 → yaw 读数变大。
 * 这跟本文件与 yaw_loop.h 里"MPU6050 正面朝上 ⇒ 逆时针为正"的**理论假设相反**，
 * 以实测为准。旧注释暂不改（属于已验收代码，由 T10 一并修正）。
 *
 * 由此定死：omega_dps 的正方向 = 「使 yaw 增大」的方向 = 俯视顺时针。
 * 这个宏只定义**符号**，不定义幅度（幅度靠 T12 整定）。
 * = +1.0f 是定义而不是可调参数；若哪天实测发现 ω 与 yaw 增大方向相反，
 * 把这里改成 -1.0f 即可，**不要去动 YawLoop_ErrorDeg()**（那是纯函数，已验收）。
 *
 * ⚠【T10 必读｜接差速前先看这段】
 *   YawLoop_DiffDrive() 的注释写"ω>0 = 逆时针 = 右轮快"，与本约定方向相反。
 *   T10 接差速时必须显式处理，二者只能改一处：
 *     要么在调用前对本值取反，要么改 DiffDrive 里 half 前的符号。
 *   漏做这一步的症状：角度环一开就朝反方向猛修，看起来像"Kp 太大"或
 *   "符号接反"，但怎么调增益都救不回来。
 *
 * 【T08 仍待复核】本次只确认了符号方向，未用水平仪标定"转过 90° 读数是否真涨
 * 90°"；"俯视"这一观察基准为口述确认，未写入原始测量记录。 */
#define YAW_OMEGA_SIGN   (1.0f)

/* 【临时值，非整定结论，T12 覆盖】
 * 2026-11-22 由 0.5 提到 1.3，原因是实测：err=170° 时单轮目标只有
 * 94 mm/s（5.7% 占空比），四轮原地转带不动，速度环积分一路顶到 100%
 * 占空比才"窜"起来 —— 现象就是"堵转、突然一下"。提到 1.3 并配套把
 * OutMax 抬到 250 后，同一个 err 给出 ω=221、单轮 245 mm/s（14.8%）。
 *
 * 【为什么必须同时抬 OutMax】
 *   ω 在 Tick 第 ⑦ 步被 ±OutMax 限幅。旧值 0.5/100 下，Kp 超过
 *   100/180=0.59 之后 ω 恒被压成 100，**再加 Kp 输出一个单位都不变**。
 *   这就是"加一点 Kp 解决不了堵转"的根因，不是 Kp 不够大，是天花板太低。
 *
 * 上限自检：1.3 × 180 = 234 < OutMax(250)，**仍然永远不饱和**（余量 6.4%）。
 *   于是"CH13 贴着 ±2340"仍是一条报警线：看到满幅，说明有人又动了 Kp，
 *   或限幅失效，或 err 没走 ErrorDeg 的折回。 */
#define YAW_KP_TEST      (1.3f)

/* ===== 到位门限（T12b，主理人选定"方案 B"）=====
 *
 * 【要解决的现象 —— 2026-11-22 地面实测】
 *   原地转 170° 的 4 秒里，前 2 秒正常；err 掉到 40° 以下后，单轮目标速度
 *   = 1.3 × err × 1.1083 也跟着掉，err<17° 时目标已 <25 mm/s —— 跌破地面
 *   静摩擦死区（实测破死区需 13~17.5% 占空比），轮子不转，而速度环积分
 *   继续往上爬（实测 31.5% → 33.9% → 35.8%，+7%/s），约 9 s 顶到 100%。
 *   板子不会报错，只是在那里一直磨 —— 所以必须由角度环自己喊停。
 *
 * 【为什么门限放这里，而不放在 pid_tune.c 的注入段】
 *   放注入段的话，CH13 仍会显示一个非零 ω，而轮子实际不动 ——
 *   "有输出没动作"正是最容易误判成"电机坏了"的那种波形。放这里则
 *   CH13 与实物同步归 0，所见即所得。
 *
 * 【为什么必须有滞环】
 *   IMU 静止底噪约 0.25°/min（本轮实测），但车身受碰、地面不平会让 yaw
 *   抖零点几度。若 3° 进、3° 出，err 会贴着门限来回翻，车一顿一顿地抽搐，
 *   而每次重启时速度环积分都已清 0，还要重新爬 —— 比不设门限更糟。
 *
 * 【3° / 6° 这两个数是我定的判据，不是实测结论】
 *   精度上限受原地转的机械死区限制，再小也压不下去；等走"行进转"(v≠0)
 *   之后死区大幅下降，这两个数应该跟着收紧。 */
#define YAW_DONE_DEG     (3.0f)
#define YAW_RESUME_DEG   (6.0f)

/**
  * @brief  初始化角度环
  * @note   增益给 0：Kp=Ki=Kd=0 时输出恒为 0，
  *         即使将来有人手滑提前调了 Tick，也不会产生任何动作。
  *         真正的角度增益在 Tick 里用 YAW_KP_TEST，不从这里进。
  *
  *         OutMax 2026-11-22 由 100.0f 提到 250.0f：
  *           100 会把 ω 压平在 6.7% 占空比（单轮 110 mm/s），原地转带不动；
  *           250 对应单轮满幅 277 mm/s（16.7%），才够越过静摩擦。
  *         注意它**不再跟速度环 TUNE_OUT_MAX 一致** —— 两者量纲不同：
  *           这里是 deg/s（角速度），那边是 mm/s（线速度），旧注释说
  *           "与 TUNE_OUT_MAX 一致"只是数值巧合，不是约束。
  */
void YawLoop_Init(void)
{
    memset(&s_yaw, 0, sizeof(s_yaw));

    PID_Init(&s_yaw.pid, 0.0f, 0.0f, 0.0f, 250.0f);

    s_yaw.target_deg   = 0.0f;
    s_yaw.measured_deg = 0.0f;
    s_yaw.omega_dps    = 0.0f;
    s_yaw.state        = (uint8_t)YAW_OFF;
}

/**
  * @brief  设置目标角度
  * @param  target_deg 目标角度，单位 deg
  * @note   只改目标，不动 state：本轮的 YAW_ON 由谁置位留待后续任务决定。
  */
void YawLoop_SetTarget(float target_deg)
{
    s_yaw.target_deg = target_deg;

    /* 换了目标，原来的"已到位"当然作废。不清的话：上一轮停在目标附近
     * 已把 done 置 1，这次 yawt 到别处，Tick 会因为 done==1 直接返回 0，
     * 车纹丝不动 —— 而 CH13 也是 0，看着像"角度环没开"。 */
    s_yaw.done = 0U;
}

/**
  * @brief  读取当前状态
  * @retval YAW_OFF / YAW_ON / YAW_ERROR
  */
YawLoop_State_t YawLoop_GetState(void)
{
    return (YawLoop_State_t)s_yaw.state;
}

/**
  * @brief  角度环周期任务入口（T09：算出 ω，但**不驱动任何电机**）
  * @note   调用点：stm32f4xx_it.c 的 TIM7 回调，`encoder_timebase_tick()` 之后、
  *         `PidTune_Tick10ms()` 之前 —— **顺序不可反**，理由见第 ① 步。
  *
  *         本函数在 TIM7 中断里跑，必须短：只有一次 float 读、一次浮点乘、
  *         几次比较，没有循环、没有 I2C、没有 printf、没有阻塞发送。
  *
  *         本轮只把 ω 算出来存进 s_yaw.omega_dps 供波形 CH13 观测。
  *         接差速（T10）、换真 PID（T12）都不在这里改。
  */
void YawLoop_Tick(void)
{
    float err;
    float omega;

    /* ① 无条件采样本，**必须放在判 state 之前**。
     *    波形 CH9/CH12 现在读的就是这个字段：若关环时就早早 return，
     *    measured_deg 会冻结，CH9 变成一条不再变化的平线 ——
     *    现象酷似"IMU 停止更新"，会把人引到完全错误的方向去查。
     *    IMU_GetYawDeg() 只是返回一个 float，中断里读是安全的
     *    （IMU_Update() 在主循环，32 位对齐单次写不会撕裂）。 */
    s_yaw.measured_deg = IMU_GetYawDeg();

    /* ② 未开环：ω 归 0 后立刻退出。
     *    必须归 0 —— 否则 CH13 会留着上一次的旧值，验收时会误以为
     *    "关了还在算"。这里**不碰 s_yaw.pid**（不清积分、不动 LastActual），
     *    保持 T12 换上真 PID 后状态机干净。 */
    if (s_yaw.state != (uint8_t)YAW_ON)
    {
        s_yaw.omega_dps = 0.0f;
        return;
    }

    /* ③ IMU 未就绪：yaw 是冻结值（imu.c 里 IMU_GetYawDeg 不检查 ready），
     *    拿它算出来的 ω 没有意义，宁可归 0 让波形说"没在算"。 */
    if (IMU_IsReady() == 0U)
    {
        s_yaw.omega_dps = 0.0f;
        return;
    }

    /* ④ 唯一合法的最短路径误差。
     *    **不许**改成 PID_Calc(&s_yaw.pid, target, yaw)：那个函数内部是
     *    err = target - actual（裸减法、无 wrap），target=170、actual=−170 时
     *    会给出 340 而不是 −20，方向正好反着绕一大圈。 */
    err = YawLoop_ErrorDeg(s_yaw.target_deg, s_yaw.measured_deg);

    /* ④.5 到位锁存（带滞环）—— 见文件上方 YAW_DONE_DEG 的整段说明。
     *   命中后就地返回，**不再往下算 ω、也不进限幅**：
     *   ω 归 0 ⇒ 注入段算出的四轮目标全 0 ⇒ 速度环不被调用 ⇒ 轮子停。
     *   （"不被调用"是 pid_tune.c 里 s_target[i] != 0 那道条件保证的，
     *     所以这里给 0 比给一个"很小的值"更彻底，连积分都不会再动。） */
    if (s_yaw.done == 0U)
    {
        if (err > -YAW_DONE_DEG && err < YAW_DONE_DEG)
        {
            s_yaw.done = 1U;
        }
    }
    else
    {
        if (!(err > -YAW_RESUME_DEG && err < YAW_RESUME_DEG))
        {
            s_yaw.done = 0U;
        }
    }

    if (s_yaw.done != 0U)
    {
        s_yaw.omega_dps = 0.0f;
        return;
    }

    /* ⑤ 本轮只用比例项。增益是 YAW_KP_TEST（临时占位，T12 覆盖）。
     *    T12 换真 PID 时注意：走 PID_Calc(&s_yaw.pid, err, 0.0f) 才等价，
     *    它的 D 项会变成"作用于误差"而不是"作用于实际值"，这一点要写进
     *    T12 的设计说明，不能默默换。 */
    omega = YAW_OMEGA_SIGN * YAW_KP_TEST * err;

    /* ⑥ NaN / ±∞ 防护。err 那层已经有防护（坏数据回 0），这里再兜一层：
     *    ω 一旦是 NaN，会一路传给差速分配、再进速度环积分器，之后每拍输出
     *    都是垃圾，而现象酷似"电机/编码器坏了"。 */
    if (!(omega > -1.0e9f && omega < 1.0e9f))
    {
        omega = 0.0f;
    }

    /* ⑦ 限幅取 pid.OutMax（当前 100），不写死 100 ——
     *    T12 若调整 OutMax，这里自动跟着走，不会出现两处限幅打架。 */
    if (omega >  s_yaw.pid.OutMax) { omega =  s_yaw.pid.OutMax; }
    if (omega < -s_yaw.pid.OutMax) { omega = -s_yaw.pid.OutMax; }

    s_yaw.omega_dps = omega;
}

/**
  * @brief  读角度环当前输出 ω
  * @retval deg/s，正 = 使 yaw 增大的方向；关环 / IMU 未就绪时为 0
  * @note   只供观测（波形 CH13）。本轮没有任何控制路径消费这个值。
  */
float YawLoop_GetOmegaDps(void)
{
    return s_yaw.omega_dps;
}

/**
  * @brief  读角度环输出限幅值
  * @retval deg/s，正常应为 100.0（YawLoop_Init 里给的值）
  * @note   纯诊断接口：0.0 说明 YawLoop_Init() 从未被调用过
  *         （s_yaw 落在 BSS 全零区），此时 Tick 算出的 ω 会被限幅压成 0，
  *         CH13 恒 0 且看不出与"误差为 0"的区别。修复点应该在 main.c
  *         上电流程里补 YawLoop_Init()，**不要**改成"OutMax 为 0 就不限幅"。
  */
float YawLoop_GetOutMax(void)
{
    return s_yaw.pid.OutMax;
}

/**
  * @brief  读本拍采到的 yaw
  * @retval deg，与 IMU_GetYawDeg() 同一量纲
  * @note   Tick 每拍无条件刷新，与 state 无关。波形 CH9/CH12 改读这个字段，
  *         目的是让一帧内的 CH9/CH12/CH13 来自**同一个** yaw 样本 ——
  *         否则同一帧里"yaw 没动但误差在跳"，看着像 IMU 抖动。
  */
float YawLoop_GetMeasuredDeg(void)
{
    return s_yaw.measured_deg;
}

/**
  * @brief  开 / 关角度环标志
  * @param  on 非 0 = 开（YAW_ON），0 = 关（YAW_OFF）
  * @note   没有独立的 enable 字段：state 就是唯一真值。
  *         关闭时**不清 target**，方便"关了再开，目标还在"。
  *         YAW_ERROR 本轮无人置位；将来若由 IMU 判定置位，
  *         YawLoop_GetEnable() 会自动变 0（异常不算"开着"），无需同步逻辑。
  */
void YawLoop_SetEnable(uint8_t on)
{
    s_yaw.state = (on != 0U) ? (uint8_t)YAW_ON : (uint8_t)YAW_OFF;

    /* 开或关都把"已到位"清掉。关的时候清：下次 yawon 是一次新的控制过程，
     * 应当按当前误差重新判断；开的时候清：否则 yawon 后第一拍就因 done==1
     * 直接返回 0，现象是"按了 yawon 车不动、CH13 也是 0"，极难查。 */
    s_yaw.done = 0U;
}

/**
  * @brief  读目标角
  * @retval 目标角，单位 deg（可能超出物理可达范围，本轮不夹）
  */
float YawLoop_GetTarget(void)
{
    return s_yaw.target_deg;
}

/**
  * @brief  读开关状态
  * @retval 1 = 已开（YAW_ON），0 = 未开或异常
  */
uint8_t YawLoop_GetEnable(void)
{
    return (s_yaw.state == (uint8_t)YAW_ON) ? 1U : 0U;
}

/**
  * @brief  读"到位"锁存
  * @retval 1 = 已在目标 ±YAW_DONE_DEG 内，ω 已被强制为 0
  * @note   唯一调用点是 pid_tune.c 的注入段：命中就 all_stop()，把四轮目标
  *         清零并 PID_Reset 四个速度环。**不要**在这里顺手清 s_yaw.done ——
  *         清了下一拍 Tick 又会重新置 1，来回翻；锁存的解除只由
  *         |err| > YAW_RESUME_DEG / yawt / yawon 三处触发。
  *         yawoff 时它可能还是 1，这没关系：SetEnable(0) 会清掉。
  */
uint8_t YawLoop_IsDone(void)
{
    return s_yaw.done;
}

/**
  * @brief  最短路径角度误差（纯函数，不读全局、不碰硬件）
  * @param  target 目标角，deg
  * @param  actual 当前角，deg（用 IMU_GetYawDeg() 的值直接传进来）
  * @retval err ∈ (−180, 180]。> 0 = 需要增大 yaw，即使 measured 朝 target 转过去。
  * @note   【符号约定】返回值正 = 需要逆时针转。
  *         这个前提是「MPU6050 正面朝上安装」：imu.c:305 的 s_gz 直接取原始值
  *         除以 LSB，没有取负，按手册右手系（z 轴朝上）绕 z 逆时针为正。
  *         若芯片是翻装的，gz 符号反、yaw 增大对应顺时针，那时"正 = 逆时针"
  *         这句话就要反过来 —— 属 T04 用一个符号常量纠正的事，本函数不动。
  *
  *         【边界约定】err 恰为 ±180 时保留 +180（逆时针）：
  *           +180 不满足 >180 也不满足 <=−180，原样留下；
  *           −180 满足 <=−180，被抬成 +180。
  *         这样 ±180 这个"同一个物理点"只有一个合法返回值，避免误差符号
  *         在 +180/−180 之间来回翻面导致车原地抽搐。代价是目标恰在 ±180
  *         附近时符号仍会跳，滞环留到 T04。
  *
  *         【坏数据】NaN / ±∞ 返回 0.0f，与 imu.c:157 的防护同值同思路。
  *         注意 NaN 参与任何比较都是 false，两个 while 都不会动它，真正
  *         拦住它的是下面这一行判断；返回 0 的语义是"无误差"，调用方无法
  *         区分"正好对准"和"坏数据" —— 纯函数没有错误通道，若要区分就在
  *         调用处问 IMU_IsReady()，不要改这个函数。
  */
float YawLoop_ErrorDeg(float target, float actual)
{
    float err = target - actual;

    /* 1) 先筛坏数据：NaN / ±∞ 一律回 0，绝不让它穿进 PID */
    if (!(err > -1.0e9f && err < 1.0e9f))
    {
        return 0.0f;
    }

    /* 2) 整数粗折：一次性减掉整圈数，把下面的 while 迭代次数压到恒定 ≤2 次。
     *    为什么需要：yawt 不夹范围，若收到病态大值（如 5e8），纯 while 要转
     *    上百万次，会把主循环卡几十毫秒 —— 违反"不许长延时阻塞主循环"。
     *    等价性：只减 360 的整数倍，不改变同余类，最终结果与纯 while 完全一致
     *    （已验证在本工程 ±1e9 的取值范围内 int32_t 不会溢出）。 */
    err -= (float)((int32_t)(err / 360.0f)) * 360.0f;

    /* 3) 精确折回 (−180, 180] */
    while (err > 180.0f)
    {
        err -= 360.0f;
    }
    while (err <= -180.0f)
    {
        err += 360.0f;
    }

    return err;
}

/**
  * @brief  差速分配（纯函数，不读全局、不碰硬件、不做限幅）
  * @param  v_mm_s    整车速度，mm/s，正 = 车头方向
  * @param  omega_dps 角速度，deg/s，正 = 逆时针（右轮快、左轮慢）
  * @param  track_mm  轮距，mm
  * @param  out       输出 {左后, 右后, 右前, 左前}，mm/s
  * @retval 无（坏数据时四路置 0）
  * @note   【为什么必须显式 deg→rad】(mm/s)/mm = 1/s = rad/s，所以 ω 在
  *         v_diff = ω·L 里必须是 rad/s。YawLoop_t.omega_dps 是 deg/s，
  *         漏掉 π/180 这一步，差速只有正确值的 1/57.3 —— 症状是"角度环
  *         几乎不动作、怎么加增益都没用"，而 PID 增益此时完全无辜。
  *         这是本函数最容易被误判成"调参问题"的地方。
  *
  *         【左右为什么不一样】ω>0 = 逆时针 = 车左转，右侧在弯道外圈、
  *         要走的弧更长，所以右轮 +half、左轮 −half。
  *
  *         【为什么坏数据置 0 而不是 return 错误码】纯函数单出口；置 0 的
  *         语义是"停车"，与 YawLoop_ErrorDeg 返回 0 是同一套取舍。
  *
  *         【T08 待实测】"ω>0 = 逆时针 = 右轮快"这条依赖 MPU6050 正面朝上
  *         安装（imu.c:305 的 s_gz 未取负）。实车验证方式：架空、给正 ω，
  *         看右侧两轮是否转得更快。若发现是反的，**只改本函数 half 前的
  *         符号并注释说明**，不要去动已验收的 imu.c。
  */
void YawLoop_DiffDrive(float v_mm_s, float omega_dps, float track_mm, float out[4])
{
    float half = omega_dps * (3.1415926f / 180.0f) * track_mm * 0.5f;

    /* 坏数据一律当"停车"处理：ω 和 L 都会进到 half 里，v 也来自外部，
     * 任一个是 NaN/±∞ 都要拦住 —— NaN 穿进 PID 的积分器就永久是 NaN，
     * 之后每一拍输出都是垃圾，而且现象极像"电机/编码器坏了"。 */
    if (!(half > -1.0e9f && half < 1.0e9f) ||
        !(v_mm_s > -1.0e9f && v_mm_s < 1.0e9f))
    {
        out[0] = 0.0f;
        out[1] = 0.0f;
        out[2] = 0.0f;
        out[3] = 0.0f;
        return;
    }

    out[0] = v_mm_s - half;   /* 左后：弯道内圈，慢 */
    out[1] = v_mm_s + half;   /* 右后：弯道外圈，快 */
    out[2] = v_mm_s + half;   /* 右前：同上 */
    out[3] = v_mm_s - half;   /* 左前：同上 */
}
