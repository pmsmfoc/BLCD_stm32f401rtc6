/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-04     20688       the first version
 */
#include "button.h"
#include "bldc.h"
#include "bldc_tim.h"
#include "pid.h"

button button1;              //声明button1结构体

rt_timer_t button_timer = RT_NULL;

/* @Function:   按键初始化
 * @Parameter:  NULL
 * */
void Button_Init(void)
{
    /*----------------配置PC11 PC12为上拉输入模式--------------*/
    rt_pin_mode(GET_PIN(C,11),PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(GET_PIN(C,12),PIN_MODE_INPUT_PULLUP);
    /*----------初始化按键对象，绑定按键的GPIO电平读取接口，设置有效触发电平----------*/
    button_init(&button1,button1_read_pin,PIN_LOW);
    /*------------注册按键事件---------------*/
    button_attach(&button1,SINGLE_CLICK,button1_Callback_Single_Click_Handler);
    button_attach(&button1,DOUBLE_CLICK,button1_Callback_Double_Click_Handler);
    button_attach(&button1,LONG_PRESS_HOLD,button1_Callback_Long_Press_Hold_Handler);
    /*-----------------启动按键-------------------*/
    button_start(&button1);
    /*----------绑定按键中断服务函数--------------*/
    rt_pin_attach_irq(GET_PIN(C,12),PIN_IRQ_MODE_FALLING,button2_IRQ_Handler,RT_NULL);
    /*----------------使能中断----------------*/
    rt_pin_irq_enable(GET_PIN(C,12),PIN_IRQ_ENABLE);
}


/* @Function:   读取button1的引脚电平
 * @Parameter：   NULL
 * */
uint8_t button1_read_pin(void)
{
    return rt_pin_read(GET_PIN(C,11));
}


/* @Function:   button1的single click回调函数，加速
 * @Parameter：   NULL
 * */
void button1_Callback_Single_Click_Handler(void* paramter)
{
    g_bldc_motor.run_flag = RUN;
    start_motor();
    if(g_bldc_motor.dir == CCW && g_speed_pid.SetPoint == 0)    /* 切换方向调节 */
    {
        g_bldc_motor.dir = CW;
    }
    g_speed_pid.SetPoint += 300;                                /* 电机速度加300 */
    if (g_speed_pid.SetPoint >= SPEED_MAX)                      /* 电机最大转速为3000 */
        g_speed_pid.SetPoint = SPEED_MAX;
    if (g_speed_pid.SetPoint == 0)
        bldc_speed_stop();
}
/* @Function:   button1的double click回调函数，减速
 * @Parameter：   NULL
 * */
void button1_Callback_Double_Click_Handler(void *parameter)
{
    g_bldc_motor.run_flag = RUN;
    start_motor();
    if(g_bldc_motor.dir == CW && g_speed_pid.SetPoint ==0)      /* 切换方向调节 */
    {
        g_bldc_motor.dir = CCW;
    }
    g_speed_pid.SetPoint -=300;                                 /* 电机速度减300 */
    if(g_speed_pid.SetPoint <= -SPEED_MAX)                      /* 电机最大转速3000 */
        g_speed_pid.SetPoint = -SPEED_MAX;
    if(g_speed_pid.SetPoint == 0)
        bldc_speed_stop();
}


void button1_Callback_Long_Press_Hold_Handler(void *parameter)
{
    g_bldc_motor.run_flag = RUN;
    start_motor();
    if(g_bldc_motor.dir == CW)
        g_speed_pid.SetPoint = SPEED_MAX;
    else {
        g_speed_pid.SetPoint = -SPEED_MAX;
    }
}
/* @Function:   button2的中断函数，紧急停机
 * @Parameter：   NULL
 * */
void button2_IRQ_Handler(void *parameter)
{
    bldc_speed_stop();
    rt_kprintf("button2 is clicked ...\r\n");
}
