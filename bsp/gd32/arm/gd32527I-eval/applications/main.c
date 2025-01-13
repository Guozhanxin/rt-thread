/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-08-20     BruceOu      first implementation
 * 2023-03-05     yuanzihao    change the LED pins
 */

#include <stdio.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "armv8m_mpu.h"

/* defined the LED1 pin: PE3 */
#define LED1_PIN GET_PIN(E, 3)

int main(void)
{
    int count = 1;

    /* set LED1 pin mode to output */
    rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);

    while (count++)
    {
        rt_pin_write(LED1_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED1_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }

    return RT_EOK;
}
char test_buf[1024];

void test_mpu_write()
{
    volatile uint32_t *temp_addr = (volatile uint32_t *)test_buf;
    rt_kprintf("test_buf:%x\n", *test_buf);
    *test_buf = 0x1;
    rt_kprintf("test_buf:%x\n", *test_buf);

    armv8m_mpu_t *mpu = (armv8m_mpu_t *)0xE000ED90;
    mpu_disable(mpu);
    mpu_select_region(mpu, 0);
    mpu_set_region_base(mpu, (uint32_t)&test_buf, REGION_NON_SHAREABLE, REGION_RO_PRIV_ONLY, REGION_XN);
    mpu_set_region_limit(mpu, (uint32_t)&test_buf[1023], 0, REGION_EN);
    mpu_set_region_attr(mpu, 0, 0); /*device memory*/
    mpu_hfnmiena_disable(mpu);
    mpu_privdefena_enable(mpu);
    mpu_enable(mpu);
    rt_kprintf("%s:mpu setup done\n", __func__);

    rt_kprintf("test_buf:%x\n", *test_buf);
    *temp_addr = 0x2;
    rt_kprintf("test_buf:%x\n", *test_buf);
    rt_kprintf("%s done\n", __func__);
}
MSH_CMD_EXPORT(test_mpu_write, test armv8m mpu write)

void test_mpu_overlap()
{
    volatile uint32_t *temp_addr = (volatile uint32_t *)0x20001000UL;
    rt_kprintf("0x20001000:%x\n", *temp_addr);
    *temp_addr = 0x1;
    rt_kprintf("0x20001000:%x\n", *temp_addr);

    armv8m_mpu_t *mpu = (armv8m_mpu_t *)0xE000ED90;
    mpu_disable(mpu);

    mpu_select_region(mpu, 0);
    mpu_set_region_base(mpu, 0x20000000UL, REGION_NON_SHAREABLE, REGION_RW_PRIV_ONLY, REGION_XN);
    mpu_set_region_limit(mpu, 0x20001FFFUL, 0, REGION_EN);
    mpu_set_region_attr(mpu, 0, 0); /*device memory*/

    mpu_select_region(mpu, 1);
    mpu_set_region_base(mpu, 0x20001000UL, REGION_NON_SHAREABLE, REGION_RW_PRIV_ONLY, REGION_XN);
    mpu_set_region_limit(mpu, 0x20001FFFUL, 1, REGION_EN);
    mpu_set_region_attr(mpu, 0, 1); /*device memory*/

    mpu_hfnmiena_disable(mpu);
    mpu_privdefena_enable(mpu);
    mpu_enable(mpu);
    rt_kprintf("%s:mpu setup done\n", __func__);

    rt_kprintf("0x20001000:%x\n", *temp_addr);
    rt_kprintf("%s done\n", __func__);
}
MSH_CMD_EXPORT(test_mpu_overlap, test armv8m mpu overlap)

void test_mpu_xn()
{
    /* Inject code at 0x20001000 */
typedef void (*test_func_t)(void);
    volatile uint32_t *temp_addr = (volatile uint32_t *)0x20001000UL;
    test_func_t test_f = (test_func_t )0x20001001;
    /*  1000041c <test_func>:
        1000041c:   b500        push    {lr}
        1000041e:   4801        ldr r0, [pc, #4]    ; (10000424 <test_func+0x8>)
        10000420:   4780        blx r0
        10000422:   bd00        pop {pc}
        10000424:   100005eb    andne   r0, r0, fp, ror #11
    */
    *temp_addr++ = 0x4801b500;
    *temp_addr++ = 0xbd004780;
    *temp_addr++ = 0x100005eb;
    test_f();

    armv8m_mpu_t *mpu = (armv8m_mpu_t *)0xE000ED90;
    mpu_disable(mpu);
    mpu_select_region(mpu, 0);
    mpu_set_region_base(mpu, 0x20000100UL, REGION_NON_SHAREABLE, REGION_RO_PRIV_ONLY, REGION_XN);
    mpu_set_region_limit(mpu, 0x20001FFFUL, 0, REGION_EN);
    mpu_set_region_attr(mpu, 0, 0); /*device memory*/
    mpu_hfnmiena_disable(mpu);
    mpu_privdefena_enable(mpu);
    mpu_enable(mpu);
    rt_kprintf("%s:mpu setup done\n", __func__);

    test_f();
}
MSH_CMD_EXPORT(test_mpu_xn, test armv8m xn)