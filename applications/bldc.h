#ifndef __BLDC_H
#define __BLDC_H

#include "stm32f4xx_hal.h"
#include "pid.h"

/***************************************** 电机状态结构体 **********************************************/
typedef struct 
{
    __IO uint8_t    run_flag;       /* 运行标志 */
    __IO uint8_t    locked_rotor;   /* 堵转标记 */
    __IO uint8_t    step_sta;       /* 本次霍尔状态 */
    __IO uint8_t    hall_single_sta;/* 单个霍尔状态 */
    __IO uint8_t    hall_sta_edge;  /* 单个霍尔状态跳变 */
    __IO uint8_t    step_last;      /* 上次霍尔状态 */
    __IO uint8_t    dir;            /* 电机旋转方向 */
    __IO int32_t    pos;            /* 电机位置 */
    __IO int32_t    speed;          /* 电机速度 */
    __IO int16_t    current;        /* 电机速度 */
    __IO uint16_t   pwm_duty;       /* 电机占空比 */
    __IO uint32_t   hall_keep_t;    /* 霍尔保持时间 */
    __IO uint32_t   hall_pul_num;   /* 霍尔传感器脉冲时间 */
    __IO uint32_t   lock_time;      /* 电机堵转时间 */
    __IO uint32_t   no_single;
    __IO uint32_t   count_j;
    __IO uint64_t   sum_pos;
} _bldc_obj;

extern _bldc_obj g_bldc_motor;

/****************************************** 霍尔传感器接口 ************************************************/

#define HALL1_TIM_CH1_PIN           GPIO_PIN_0     /* U */
#define HALL1_TIM_CH1_GPIO          GPIOB
#define HALL1_U_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)    /* PB口时钟使能 */

#define HALL1_TIM_CH2_PIN           GPIO_PIN_5     /* V */
#define HALL1_TIM_CH2_GPIO          GPIOB
#define HALL1_V_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)    /* PB口时钟使能 */

#define HALL1_TIM_CH3_PIN           GPIO_PIN_4     /* W */
#define HALL1_TIM_CH3_GPIO          GPIOB
#define HALL1_W_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)    /* PB口时钟使能 */

/*************************************** 电机相关系数 *****************************************************/

#define MAX_PWM_DUTY    ((4200 - 1)*0.96)        /* 最大占空比 */

#define H_PWM_L_ON
#ifndef H_PWM_L_ON
#define H_PWM_L_PWM
#endif

#define CCW                         (1)                 /* 逆时针 */
#define CW                          (2)                 /* 顺时针 */
#define HALL_ERROR                  (0xF0)              /* 霍尔错误标志 */
#define RUN                         (1)                 /* 电机运动标志 */
#define STOP                        (0)                 /* 电机停机标志 */

#define SPEED_MAX                   3000                /* 最大转速 */
#define SPEED_MIN                   300                 /* 最小转速 */
#define SPEED_COEFF      (uint32_t)((18000/4)*60)       /*旋转一圈变化4个信号，2对级永磁体特性，NSNS共4级数*/


typedef void(*pctr) (void);
void stop_motor(void);
void start_motor(void);

#define FirstOrderRC_LPF(Yn_1,Xn,a) Yn_1 = (1-a)*Yn_1 + a*Xn;   /* Yn:out;Xn:in;a:系数*/
/***************************************** 函数声明 *************************************************/

void bldc_init(uint16_t arr, uint16_t psc);                     /* BLDC初始化函数 */
uint8_t check_hall_dir(_bldc_obj * obj);                        /* 检测电机旋转方向 */
extern pctr pfunclist_m1[6];                                    /* 六步换相和函数指针数组 */
void bldc_ctrl(int32_t dir,float duty);                         /* bldc控制函数 */
uint8_t uemf_edge(uint8_t val);                                 /* 波形状态检测 */
void hall_gpio_init(void);                                      /* 霍尔接口初始化 */
uint32_t hallsensor_get_state(void);                            /* 获取霍尔状态 */
/**
 * @brief       清除电机状态并关闭电机
 * @param       无
 * @retval      无
 */
void bldc_speed_stop(void);


/* 六步换向 */
void m1_uhvl(void);
void m1_uhwl(void);
void m1_vhwl(void);
void m1_vhul(void);
void m1_whul(void);
void m1_whvl(void);

#endif
