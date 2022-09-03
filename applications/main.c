/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-08-20     RT-Thread    first version
 */

#include <rtthread.h>

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include "bldc_tim.h"
#include "bldc.h"
#include "pid.h"
#include "uart.h"


int main(void)
{

    bldc_init(4200-1,0);//定时器TIM1 4200-1   50us溢出一次
    bldc_ctrl(CCW,0);                /* 初始无刷电机接口1速度 */

    pid_init();
    //uart4_init();


    g_bldc_motor.dir = CW;
    g_bldc_motor.run_flag = RUN;
    start_motor();//开启pwm输出
    g_speed_pid.SetPoint = 300;



//    uart4_rx_td = rt_thread_create("uart4",
//                            uart4_td_entry,
//                            RT_NULL,
//                            8192,
//                            3,
//                            20);
//    if(uart4_rx_td != RT_NULL)
//        rt_thread_startup(uart4_rx_td);

    while(1)
    {

        rt_kprintf("%d\r\n",g_bldc_motor.speed);
    }
    return RT_EOK;

}
void motor_info(void)
{
    rt_kprintf("motor speed is %d\r\n",g_bldc_motor.speed);
    rt_kprintf("motor position is %d\r\n",g_bldc_motor.pos);
    rt_kprintf("\r\n");
}
MSH_CMD_EXPORT(motor_info,show motor info);

