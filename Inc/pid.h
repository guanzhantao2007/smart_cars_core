#ifndef __PID_H
#define __PID_H


typedef struct
{
    float Kp;          
    float Ki;          
    float Kd;         

    float ErrorInt;    /* 积分累加值 */
    float LastActual;  /* 上一次的实际值，用于微分 */
    float DState;      /* 微分项低通滤波状态 */
    float Out;         /* 上一次输出 */

    float OutMax;      /* 输出限幅（对称） */
    float DAlpha;      /* 微分低通系数，0~1，越小越平滑 */
} PID_t;

void  PID_Init(PID_t *p, float kp, float ki, float kd, float out_max);
void  PID_SetGains(PID_t *p, float kp, float ki, float kd);
void  PID_Reset(PID_t *p);
float PID_Calc(PID_t *p, float target, float actual);

#endif
