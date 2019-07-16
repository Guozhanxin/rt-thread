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
#include "player.h"
#include "drv_isd9160.h"

/* defined the LED0 pin: PE7 */
#define LED0_PIN    GET_PIN(B, 6)

void mplay_finish(void)
{
    rt_kprintf("play finsh!\n");
}

int main(void)
{
    int count = 1;
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);

    iotb_wav_player_init(mplay_finish);
//    isd9160_init();
    rt_thread_mdelay(1000);
    
    while (count++)
    {
        isd9160_loop_once();
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }

    return RT_EOK;
}
