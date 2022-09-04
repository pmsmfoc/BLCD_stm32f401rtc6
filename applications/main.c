#include <rtthread.h>

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include "bldc_tim.h"
#include "bldc.h"
#include "pid.h"
#include "uart.h"
#include "button.h"

int main(void)
{

    bldc_init(4200-1,0);//定时器TIM1 4200-1   50us溢出一次
    bldc_ctrl(CCW,0);                /* 初始无刷电机接口1速度 */
    pid_init();         //初始化PID
    uart6_init();       //初始化UART6
    Button_Init();      //初始化BUTTON
    start_motor();      //开启pwm输出



    uart6_rx_td = rt_thread_create("uart6",
                            uart6_td_entry,
                            RT_NULL,
                            8192,
                            3,
                            20);
    if(uart6_rx_td != RT_NULL)
        rt_thread_startup(uart6_rx_td);

    button_timer = rt_timer_create("button",
                            button_ticks,
                            RT_NULL,
                            5,
                            RT_TIMER_FLAG_PERIODIC);
    if(button_timer != RT_NULL)
        rt_timer_start(button_timer);

    return RT_EOK;
}
void motor_info(void)
{
    rt_kprintf("motor speed is %d\r\n",g_bldc_motor.speed);
    rt_kprintf("motor position is %d\r\n",g_bldc_motor.pos);
    rt_kprintf("\r\n");
}
MSH_CMD_EXPORT(motor_info,show motor info);

