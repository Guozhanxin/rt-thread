/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "sensor.h"
#include "sensor_st_lsm6dsl.h"

/* defined the LED0 pin: PE7 */
#define LED0_PIN    GET_PIN(B, 6)

int main(void)
{
    int count = 1;
    struct rt_sensor_config cfg;
    
    cfg.intf.type = RT_SEN_INTF_I2C;
    cfg.intf.dev_name = "i2c4";
    cfg.intf.user_data = (void *)(0xD7 >> 1);
    cfg.irq_pin.pin = RT_PIN_NONE;
    
    rt_hw_lsm6dsl_init("st", &cfg);
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);

    while (count++)
    {
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }

    return RT_EOK;
}
