/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-02-15     flybreak     the first version
 */

#include <board.h>
#include <sensor.h>

#ifdef BSP_USING_LSM6DSL
#include "sensor_st_lsm6dsl.h"
#endif

int sensor_init(void)
{
    struct rt_sensor_config cfg;

#ifdef BSP_USING_LSM6DSL
    cfg.intf.dev_name = "i2c4";
    cfg.intf.user_data = (void *)LSM6DSL_ADDR_DEFAULT;
    cfg.irq_pin.pin = RT_PIN_NONE;
    rt_hw_lsm6dsl_init("lsm", &cfg);
#endif

    return RT_EOK;
}
INIT_APP_EXPORT(sensor_init);
