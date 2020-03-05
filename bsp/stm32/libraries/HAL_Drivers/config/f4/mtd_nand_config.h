/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-10-30     SummerGift   first version
 * 2019-01-03     zylx         modify dma support
 */
 
#ifndef __MTD_NAND_CONFIG_H__
#define __MTD_NAND_CONFIG_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

//#if defined(BSP_USING_MTD_NAND)
#ifndef MTD_NAND_CONFIG
#define MTD_NAND_CONFIG                                             \
    {                                                               \
        .name = "nand0",                                            \
        .Instance = FMC_NAND_DEVICE,                                \
    }
#endif /* UART1_CONFIG */
//#endif

#ifdef __cplusplus
}
#endif

#endif
