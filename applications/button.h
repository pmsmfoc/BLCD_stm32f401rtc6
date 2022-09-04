/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-04     20688       the first version
 */
#ifndef APPLICATIONS_BUTTON_H_
#define APPLICATIONS_BUTTON_H_

/*按键功能设计：button1 按一下，电机加速；按两下，电机减速。
               button2 紧急关机按键
 *
 * */
#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>
#include "multi_button.h"
extern rt_timer_t button_timer;
void Button_Init(void);
uint8_t button1_read_pin(void);
void button1_Callback_Single_Click_Handler(void* parameter);
void button1_Callback_Double_Click_Handler(void *parameter);
void button1_Callback_Long_Press_Hold_Handler(void *parameter);
void button2_IRQ_Handler(void *parameter);
#endif /* APPLICATIONS_BUTTON_H_ */
