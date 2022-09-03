/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-08-28     20688       the first version
 */
#ifndef APPLICATIONS_UART_H_
#define APPLICATIONS_UART_H_
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include "bldc.h"
#include "bldc_tim.h"
#include "cJSON.h"
#include "pid.h"
extern rt_device_t uart4_dev;
extern struct rt_semaphore sem1;
extern rt_thread_t uart4_rx_td;
extern rt_uint32_t rx_len;

rt_int8_t uart4_init(void);
rt_err_t uart4_rx_callback(rt_device_t dev,rt_size_t size);
void uart4_td_entry(void *parameter);
#endif /* APPLICATIONS_UART_H_ */
