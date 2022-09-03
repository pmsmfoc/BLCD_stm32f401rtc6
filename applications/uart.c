#include "uart.h"

rt_device_t uart4_dev = RT_NULL;
rt_thread_t uart4_rx_td = RT_NULL;
rt_uint32_t rx_len = 0;


struct rt_semaphore sem1;
extern _bldc_obj g_bldc_motor;
rt_int8_t uart4_init(void)
{
    rt_int8_t ret = 0;
    /*查找设备*/
    uart4_dev = rt_device_find("uart4");
    if(uart4_dev == RT_NULL)
    {
        rt_kprintf("uart4 not find...\r\n");
        return -RT_ERROR;
    }
    /*打开设备*/
    ret = rt_device_open(uart4_dev,RT_DEVICE_OFLAG_RDWR|RT_DEVICE_FLAG_DMA_RX);
    if(ret < 0)
    {
        rt_kprintf("uart4 not open...\r\n");
        return ret;
    }
    /*配置串口参数*/
    struct serial_configure serial_cfg = RT_SERIAL_CONFIG_DEFAULT;
    rt_device_control(uart4_dev,RT_DEVICE_CTRL_CONFIG,(void*)&serial_cfg);
    /*设置uart4的接收回调函数*/
    rt_device_set_rx_indicate(uart4_dev,uart4_rx_callback);
    rt_sem_init(&sem1,"sem1",0,RT_IPC_FLAG_FIFO);
    return RT_EOK;
}
rt_err_t uart4_rx_callback(rt_device_t dev,rt_size_t size)
{
    rt_sem_release(&sem1);
    rx_len = size;
    return RT_EOK;
}

void uart4_td_entry(void *parameter)
{
    //cJSON *Proportion= RT_NULL;
    //cJSON *Integral = RT_NULL;
    //cJSON *Derivative = RT_NULL;
    cJSON RunFlag;
    cJSON Dir;
    cJSON Speed;

    cJSON *cJSON_Buf = RT_NULL;
    char *str = RT_NULL;
    rt_uint8_t len = 0;
    char buffer[64];
    char *dynamic_buf = RT_NULL;
    while(1)
    {
        /*获取信号量*/
        rt_sem_take(&sem1,RT_WAITING_FOREVER);
        /*读取DMA接收到的数据*/
        len = rt_device_read(uart4_dev,0,buffer,rx_len);
        /*申请内存*/
        dynamic_buf = rt_malloc(sizeof(char) * len);
        /*copy数据到申请的内存中*/
        memcpy(dynamic_buf,buffer,len);
        /*解析*/
        cJSON_Buf = cJSON_Parse(dynamic_buf);


        if(cJSON_Buf == RT_NULL)
            rt_kprintf("cJSON prase fail.\r\n");
        else
        {
            /*获取比例系数*/
            //Proportion = cJSON_GetObjectItem(cJSON_Buf,"p");
            /*获取积分系数*/
            //Integral = cJSON_GetObjectItem(cJSON_Buf,"i");
            /*获取微分系数*/
            //Derivative = cJSON_GetObjectItem(cJSON_Buf,"d");


            /*获取启停标志位*/
            if(cJSON_GetObjectItem(cJSON_Buf, "runflag") != RT_NULL)
                RunFlag = *cJSON_GetObjectItem(cJSON_Buf, "runflag");
            else
                RunFlag.valueint = 0;
            /*获取转向标志位*/
            if(cJSON_GetObjectItem(cJSON_Buf, "dir") != RT_NULL)
                Dir = *cJSON_GetObjectItem(cJSON_Buf, "dir");
            else
                Dir.valueint = 1;
            /*获取电机转速*/
            if(cJSON_GetObjectItem(cJSON_Buf, "speed") != RT_NULL)
                Speed = *cJSON_GetObjectItem(cJSON_Buf, "speed");
            else
                Speed.valueint = 0;
            if(Speed.valueint > 4000)
            {
                Speed.valueint = 4000;
            }
            /*写入参数*/
            g_bldc_motor.run_flag = RunFlag.valueint;
            if(g_bldc_motor.run_flag != RUN)
            {
                /*清除电机状态并关闭电机*/
                bldc_speed_stop();
            }
            else
            {
                /*电机为启动状态*/
                if(Dir.valueint != g_bldc_motor.dir)
                {
                    /*转向发生改变*/
                    bldc_speed_stop();
                    g_bldc_motor.dir = Dir.valueint;
                    g_bldc_motor.run_flag = RUN;
                }
                /*电机转速小于100rad/min*/
                if(Speed.valueint <= 100)
                {
                    bldc_speed_stop();
                    g_bldc_motor.run_flag = RUN;
                }
                start_motor();
                if(g_bldc_motor.dir == CW)
                    g_speed_pid.SetPoint = Speed.valueint;
                else
                    g_speed_pid.SetPoint = -Speed.valueint;
            }
            str = cJSON_Print(cJSON_Buf);
            rt_kprintf("%s\r\n",str);
        }
        /*释放内存*/
        cJSON_Delete(cJSON_Buf);
        rt_free(dynamic_buf);
    }
}

