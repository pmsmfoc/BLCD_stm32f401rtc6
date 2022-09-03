#include "bldc_tim.h"
#include "bldc.h"
#include "pid.h"
/******************************************************************************************/
/* 定时器配置句柄 定义 */

/* TIM1 PWM */
TIM_HandleTypeDef g_atimx_handle;           /* TIM1句柄 */
TIM_OC_InitTypeDef g_atimx_oc_chy_handle;   /* 定时器输出句柄 */
extern _bldc_obj g_bldc_motor;
extern PID_TypeDef  g_speed_pid;            /* 速度PID参数结构体 */
/******************************************************************************************/

/**
 * @brief       高级定时器TIM1 PWM 初始化函数
 * @note
  *                            高级定时器的时钟来自APB2。而PCLK2 = 84Mhz，我们设置PPRE2不分频，因此
  *                            高级定时器时钟 = 84Mhz
 *
 * @param       arr: 自动重装值
 * @param       psc: 时钟预分频数
 * @retval      无
 */
void atim_timx_oc_chy_init(uint16_t arr, uint16_t psc)
{
    ATIM_TIMX_PWM_CHY_CLK_ENABLE();                             /* TIMX 时钟使能 */

    
    g_atimx_handle.Instance = ATIM_TIMX_PWM;                    /* 定时器x */
    g_atimx_handle.Init.Prescaler = psc;                        /* 定时器分频数 */
    g_atimx_handle.Init.CounterMode = TIM_COUNTERMODE_UP;       /* 向上计数模式 */
    g_atimx_handle.Init.Period = arr;                           /* 自动重装载值 */
    g_atimx_handle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;   /* 分频因子 */
    g_atimx_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; /* 使能TIMx_ARR进行缓冲 */
    g_atimx_handle.Init.RepetitionCounter = 0;                  /* 开始时不计数 */
    HAL_TIM_PWM_Init(&g_atimx_handle);                          /* 初始化PWM */
    
    g_atimx_oc_chy_handle.OCMode = TIM_OCMODE_PWM1;             /* 模式选择PWM1 */
    g_atimx_oc_chy_handle.Pulse = 0;
    g_atimx_oc_chy_handle.OCPolarity = TIM_OCPOLARITY_HIGH;     /* 输出比较极性为高 */
    g_atimx_oc_chy_handle.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    g_atimx_oc_chy_handle.OCFastMode = TIM_OCFAST_DISABLE;
    g_atimx_oc_chy_handle.OCIdleState = TIM_OCIDLESTATE_RESET;
    g_atimx_oc_chy_handle.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&g_atimx_handle, &g_atimx_oc_chy_handle, ATIM_TIMX_PWM_CH1); /* 配置TIMx通道y */
    HAL_TIM_PWM_ConfigChannel(&g_atimx_handle, &g_atimx_oc_chy_handle, ATIM_TIMX_PWM_CH2); /* 配置TIMx通道y */
    HAL_TIM_PWM_ConfigChannel(&g_atimx_handle, &g_atimx_oc_chy_handle, ATIM_TIMX_PWM_CH3); /* 配置TIMx通道y */
   
    /* 开启定时器通道1输出PWM */
    HAL_TIM_PWM_Start(&g_atimx_handle,TIM_CHANNEL_1);

    /* 开启定时器通道2输出PWM */
    HAL_TIM_PWM_Start(&g_atimx_handle,TIM_CHANNEL_2);

    /* 开启定时器通道3输出PWM */
    HAL_TIM_PWM_Start(&g_atimx_handle,TIM_CHANNEL_3);
    
    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 2, 2);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
               
    HAL_TIM_Base_Start_IT(&g_atimx_handle);                     /* 启动高级定时器 */
}


/**
 * @brief       定时器底层驱动，时钟使能，引脚配置
                                此函数会被HAL_TIM_PWM_Init()调用
 * @param       htim:定时器句柄
 * @retval      无
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == ATIM_TIMX_PWM)
    {
        GPIO_InitTypeDef gpio_init_struct;
        ATIM_TIMX_PWM_CHY_CLK_ENABLE();                             /* 使能定时器时钟 */
        /* 三个上桥臂对应IO时钟使能 */
        ATIM_TIMX_PWM_CH1_GPIO_CLK_ENABLE();                        /* IO时钟使能 */
        ATIM_TIMX_PWM_CH2_GPIO_CLK_ENABLE();                        /* IO时钟使能 */
        ATIM_TIMX_PWM_CH3_GPIO_CLK_ENABLE();                        /* IO时钟使能 */
        /* 三个下桥臂对应IO时钟使能 */
        M1_LOW_SIDE_U_GPIO_CLK_ENABLE();                            /* IO时钟使能 */
        M1_LOW_SIDE_V_GPIO_CLK_ENABLE();                            /* IO时钟使能 */
        M1_LOW_SIDE_W_GPIO_CLK_ENABLE();                            /* IO时钟使能 */

        /* UVW_LOW的IO初始化 */
        gpio_init_struct.Pin = M1_LOW_SIDE_U_PIN;
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Speed = GPIO_SPEED_HIGH;
        gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;                /* 推挽输出模式 */
        HAL_GPIO_Init(M1_LOW_SIDE_U_PORT, &gpio_init_struct);

        gpio_init_struct.Pin = M1_LOW_SIDE_V_PIN;
        HAL_GPIO_Init(M1_LOW_SIDE_V_PORT, &gpio_init_struct);

        gpio_init_struct.Pin = M1_LOW_SIDE_W_PIN;
        HAL_GPIO_Init(M1_LOW_SIDE_W_PORT, &gpio_init_struct);
        
        
        /* 定时器IO初始化 */
        gpio_init_struct.Pin = ATIM_TIMX_PWM_CH1_GPIO_PIN;          /* 通道y的10口 */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                    /* 复用推挽输出 */
        gpio_init_struct.Pull = GPIO_NOPULL;                        /* 上拉 */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* 高速 */
        gpio_init_struct.Alternate = ATIM_TIMX_PWM_CHY_GPIO_AF;     /* 端口复用 */
        HAL_GPIO_Init(ATIM_TIMX_PWM_CH1_GPIO_PORT, &gpio_init_struct);
        
        gpio_init_struct.Pin = ATIM_TIMX_PWM_CH2_GPIO_PIN;          
        HAL_GPIO_Init(ATIM_TIMX_PWM_CH2_GPIO_PORT, &gpio_init_struct);
       
        gpio_init_struct.Pin = ATIM_TIMX_PWM_CH3_GPIO_PIN;         
        HAL_GPIO_Init(ATIM_TIMX_PWM_CH3_GPIO_PORT, &gpio_init_struct);
    }
}

void ATIM_TIMX_PWM_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_atimx_handle);
}


/***********************************************定时器中断回调函数***********************************************/
int32_t temp_pwm1 = 0.0;                    /* 存放PID计算后的期望值 */
int32_t motor_pwm_s = 0;                    /* 存放一阶滤波后的数据 */

/**
 * @brief       定时器中断回调函数
 * @param       无
 * @retval      无
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    uint8_t bldc_dir=0;
    int16_t temp_speed=0;                   /* 临时速度存贮 */
    if(htim->Instance == ATIM_TIMX_PWM)
    {
        /*******************************六步换向*******************************/
        if(g_bldc_motor.run_flag == RUN)
        {
            if(g_bldc_motor.dir == CW)
            {
                g_bldc_motor.step_sta = hallsensor_get_state();
            }
            else
            {
                g_bldc_motor.step_sta = 7 - hallsensor_get_state();
            }
            if((g_bldc_motor.step_sta <= 6)&&(g_bldc_motor.step_sta >= 1))
            {
                pfunclist_m1[g_bldc_motor.step_sta-1]();/*直接驱动换向的函数*/
            }
            else    /*编码器错误、接触不良、断开等情况*/
            {
                stop_motor();
                g_bldc_motor.run_flag = STOP;
            }
            
            /*******************************速度计算*******************************/
            g_bldc_motor.count_j++;                /*计算速度专用计数值*/
            g_bldc_motor.hall_sta_edge = uemf_edge(g_bldc_motor.hall_single_sta);/*检测单个霍尔信号的变化*/
            if(g_bldc_motor.hall_sta_edge == 0)    /*统计单个霍尔信号的高电平时间，当只有一对级的时候，旋转一圈为一个完整脉冲。一高一低相加即旋转一圈所花的时间*/
            {
                /*计算速度*/
                if(g_bldc_motor.dir == CW)
                    temp_speed = (SPEED_COEFF/g_bldc_motor.count_j);
                else
                    temp_speed = -(SPEED_COEFF/g_bldc_motor.count_j);
                FirstOrderRC_LPF(g_bldc_motor.speed,temp_speed,0.2379f);   /*一阶滤波*/
                g_bldc_motor.no_single = 0;
                g_bldc_motor.count_j = 0;
            }
            if(g_bldc_motor.hall_sta_edge == 1)    /* 当采集到下降沿是数据清0 */
            {
                g_bldc_motor.no_single = 0;
                g_bldc_motor.count_j = 0;
            }
            if(g_bldc_motor.hall_sta_edge == 2)    /* 霍尔值一直不变代表未换向 */
            {
                g_bldc_motor.no_single++;          /* 不换相和时间累计 超时则判定速度为0 */
                
                if(g_bldc_motor.no_single > 15000)
                {
                    
                    g_bldc_motor.no_single = 0;
                    g_bldc_motor.speed = 0;        /* 超时换向 判定为停止 速度为0 */
                }
            }
            /******************************* 位置记录*******************************/
            if(g_bldc_motor.step_last != g_bldc_motor.step_sta)
            {
                bldc_dir = check_hall_dir(&g_bldc_motor);
                if(bldc_dir == CCW)
                {
                    g_bldc_motor.pos -= 1;
                }
                else if(bldc_dir == CW)
                {
                    g_bldc_motor.pos += 1;
                }
                g_bldc_motor.step_last = g_bldc_motor.step_sta;
            }
            /******************************* PID控制*******************************/
                temp_pwm1 = increment_pid_ctrl(&g_speed_pid,g_bldc_motor.speed);   /* PID控制算法，输出期望值 */
                FirstOrderRC_LPF(motor_pwm_s,temp_pwm1,0.085);                      /* 一阶滤波 */
                if(motor_pwm_s < 0)                                                 /* 判断正负值 */
                {
                    g_bldc_motor.pwm_duty = -motor_pwm_s;
                }
                else
                {
                   g_bldc_motor.pwm_duty = motor_pwm_s;
                }
        }
    }
}


