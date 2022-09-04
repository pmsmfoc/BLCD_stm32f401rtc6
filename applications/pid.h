#ifndef __PID_H
#define __PID_H

#include "stm32f4xx_hal.h"

/* PID相关参数 */

#define  INCR_LOCT_SELECT  1        /* 0：选择位置式 1：增量式控制 */

#if INCR_LOCT_SELECT
#define  S_KP      0.00800f         /* P参数 */
#define  S_KI      0.00025f         /* I参数 */
#define  S_KD      0.00020f         /* D参数 */
#else
#define  S_KP      0.00800f         /* P参数 */
#define  S_KI      0.00025f         /* I参数 */
#define  S_KD      0.00020f         /* D参数 */
#endif

#define SMAPLSE_PID_SPEED  40       /* 采样率 单位ms */

/* 定义位置PID参数相关宏 */
/* PID结构体 */
typedef struct
{
    __IO float  SetPoint;           /* 设定目标 */
    __IO float  ActualValue;        /* 期望值 */
    __IO float  SumError;           /* 误差累计 */
    __IO float  Proportion;         /* 比例常数 P */
    __IO float  Integral;           /* 积分常数 I */
    __IO float  Derivative;         /* 微分常数 D */
    __IO float  Error;              /* Error[-1] */
    __IO float  LastError;          /* Error[-1] */
    __IO float  PrevError;          /* Error[-2] */
    __IO float  IngMin;
    __IO float  IngMax;
    __IO float  OutMin;
    __IO float  OutMax;
} PID_TypeDef;

extern PID_TypeDef  g_speed_pid;    /* 速度PID参数结构体 */
/******************************************************************************************/
/* 外部接口函数 */
void pid_init(void);                                                    /* PID初始化 */
int32_t increment_pid_ctrl(PID_TypeDef *PID,float Feedback_value);      /* PID控制算法 */
#endif
