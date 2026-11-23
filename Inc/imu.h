#ifndef IMU_H
#define IMU_H

#include <stdint.h>

#define IMU_SAMPLE_HZ          940U
#define IMU_CALIB_SAMPLES      500U
/* 【2026-11-22 P0 改】0 -> 1：打开在线零偏学习。
 * 为什么必须开：yaw 是 gz 的纯积分，零偏只要残差 0.1 dps，1 分钟就漂 6 度。
 *   上电那 500 次静止校准只能吃掉"当次"零偏，温升和上电漂移管不了。
 * 为什么不会把"真转弯"吃成零偏：imu.c 里有两道闸 ——
 *   IMU_MOTION_GYRO_DPS(5 dps) 一旦被超过就进 IMU_COOLDOWN_TICKS(1500 拍
 *   ≈1.6 s) 冷却，冷却期内不学；再加上 IMU_STILL_TICKS(300 拍≈0.32 s) 的
 *   连续静止判据。所以只有"真的停稳了"才修，转弯过程中不会。
 * 【代价，必须知道】静止久了它会慢慢把零偏越修越准，但如果你把车停在
 *   一个本来就在缓慢转的转台上，它会把那个转速当零偏学掉。台架上别转。 */
#define IMU_ONLINE_BIAS_ENABLE 1
#define IMU_STILL_GYRO_DPS     0.6f
#define IMU_STILL_ACCEL_ERR    0.05f
#define IMU_STILL_TICKS        300U
#define IMU_BIAS_ALPHA         0.0002f
#define IMU_MOTION_GYRO_DPS    5.0f
#define IMU_COOLDOWN_TICKS     1500U
#define IMU_SETTLED_TICKS      250U
#define IMU_GAP_MAX_TICKS      120U

typedef enum {
    IMU_STATE_UNINIT = 0,
    IMU_STATE_CALIBRATING,
    IMU_STATE_READY,
    IMU_STATE_ERROR
} IMU_State_t;

typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;       /* 解缠后的航向角，范围为 (-180, 180] */
    float yaw_cont_deg;  /* 多圈累计航向角，不代表车辆直线行驶里程 */
    float gyro_z_dps;
    float accel_norm_g;
    float bias_z_dps;
    float calib_span_dps;
    float temp_c;
    uint32_t tick_ms;
    uint32_t missed_ticks;
    uint32_t read_errors;
    uint8_t state;
} IMU_Data_t;

/* I2C1 初始化后调用；校准期间设备须保持静止，约需 600 ms。 */
uint8_t IMU_Init(void);
/* 在主循环持续调用；有新 1 ms 节拍时才采样。 */
void IMU_Update(void);
/* 由 1 ms 定时器中断调用，需配置 TIM6 并启动其中断。 */
void IMU_Tick1ms(void);
void IMU_GetData(IMU_Data_t *out);//注释：获取IMU数据
float IMU_GetYawDeg(void);//注释：获取航向角
void IMU_GetGyroDps(float *gx, float *gy, float *gz);//注释：获取陀螺仪角速度
void IMU_ZeroYaw(void);//注释：将航向角归零
void IMU_SetBiasLearn(uint8_t en);//注释：设置偏置学习状态
uint8_t IMU_GetBiasLearn(void);//注释：获取偏置学习状态
uint8_t IMU_IsSettled(void);//注释：获取IMU是否稳定
uint8_t IMU_IsReady(void);//注释：获取IMU是否准备好

#endif
