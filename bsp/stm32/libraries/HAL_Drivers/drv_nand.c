/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-04-10     lizhen9880   the first version
 * 2020-03-05     flybreak     modify to stm32
 */

#include "board.h"
#include "drv_nand.h"
#include "drv_config.h"

//#ifdef RT_USING_NFTL
//    #include <nftl.h>
//#endif

#define DRV_DEBUG
#define LOG_TAG             "drv.nand"
#include <drv_log.h>

#define ECC_SIZE                4

#ifndef STM32_NAND_BANK
    #define STM32_NAND_BANK FMC_NAND_BANK3
#endif

#define STM32_NAND_SET_CMD(_c)    do{*(volatile rt_uint8_t*)(NAND_ADDRESS|NAND_CMD)  = _c;}while(0)
#define STM32_NAND_SET_ADD(_a)    do{*(volatile rt_uint8_t*)(NAND_ADDRESS|NAND_ADDR) = _a;}while(0)
#define STM32_NAND_SET_DAT(_d)    do{*(volatile rt_uint8_t*)(NAND_ADDRESS)           = _d;}while(0)
#define STM32_NAND_GET_DAT(_d)    do{_d = *(volatile rt_uint8_t*)NAND_ADDRESS;            }while(0)

enum
{
#ifdef BSP_USING_UART1
    MTD_NAND_INDEX,
#endif
};

static struct stm32_nand_config nand_config[] =
{
#ifdef BSP_USING_UART1
    MTD_NAND_CONFIG,
#endif
};

static struct stm32_nand nand_obj[sizeof(nand_config) / sizeof(nand_config[0])] = {0};

const struct nand_info nand_info_list[] = NAND_FLASH_INFO_TABLE;

static rt_uint8_t stm32_nand_wait_rb(volatile rt_uint8_t rb)
{
    volatile rt_uint16_t time = 0;
    while (time < 10000)
    {
        time++;
        if (NAND_RB == rb)
        {
            LOG_D("time:%d/%d  R/B:%d\n", time, 10000, rb);
            return 0;
        }
    }
    LOG_D("wait rb timeout\n");
    return 1;
}

static rt_uint8_t stm32_nand_read_status(void)
{
    volatile rt_uint8_t data = 0;
    
    STM32_NAND_SET_CMD(NAND_READSTA);//发送读状态命令
    {
        data++;
        data++;
        data++;
        data++;
        data++; //加延时,防止-O2优化,导致的错误.
    }
    data = *(volatile rt_uint8_t *)NAND_ADDRESS;        //读取状态值
    return data;
}

static rt_uint8_t wait_for_ready(void)
{
    rt_uint8_t status = 0;
    volatile rt_uint32_t time = 0;

    while (1)                       //wait ready
    {
        status = stm32_nand_read_status(); //获取状态值
        if (status & NSTA_READY) break;
        time++;
        if (time >= 0X1FFFF) return NSTA_TIMEOUT; //超时
    }
    return NSTA_READY;//准备好
}

static rt_uint8_t nand_reset(void)
{
    STM32_NAND_SET_CMD(NAND_RESET);//复位NAND

    if (wait_for_ready() == NSTA_READY)
        return 0; //复位成功
    else
        return 1; //复位失败
}

static rt_uint8_t stm32_nand_mode_set(rt_uint8_t mode)
{
    STM32_NAND_SET_CMD(NAND_FEATURE);
    STM32_NAND_SET_ADD(0X01);
    STM32_NAND_SET_DAT(mode);
    STM32_NAND_SET_DAT(0);
    STM32_NAND_SET_DAT(0);
    STM32_NAND_SET_DAT(0);

    if (wait_for_ready() == NSTA_READY)
        return 0; //成功
    else
        return 1; //失败
}

static int stm32_hw_nand_init(struct stm32_nand *device)
{
    FMC_NAND_PCC_TimingTypeDef ComSpaceTiming, AttSpaceTiming;

    device->handle.Instance = FMC_NAND_DEVICE;
    device->handle.Init.NandBank = STM32_NAND_BANK;                        //NAND挂在BANK3上
    device->handle.Init.Waitfeature = FMC_NAND_PCC_WAIT_FEATURE_DISABLE;  //关闭wait 特性
    device->handle.Init.MemoryDataWidth = FMC_NAND_PCC_MEM_BUS_WIDTH_8;   //8位数据宽度
    device->handle.Init.EccComputation = FMC_NAND_ECC_DISABLE;            //不使用ECC
    device->handle.Init.ECCPageSize = FMC_NAND_ECC_PAGE_SIZE_2048BYTE;    //ECC页大小为2k
    device->handle.Init.TCLRSetupTime = 0;                                //设置TCLR(tCLR=CLE到RE的延时)=(TCLR+TSET+2)*THCLK,THCLK=1/180M=5.5ns
    device->handle.Init.TARSetupTime = 1;                                 //设置TAR(tAR=ALE到RE的延时)=(TAR+TSET+2)*THCLK,THCLK=1/180M=5.5n。

    ComSpaceTiming.SetupTime = 2;       //建立时间
    ComSpaceTiming.WaitSetupTime = 3;   //wait 时间
    ComSpaceTiming.HoldSetupTime = 2;   //保持时间
    ComSpaceTiming.HiZSetupTime = 1;    //高阻态时间

    AttSpaceTiming.SetupTime = 2;       //建立时间
    AttSpaceTiming.WaitSetupTime = 3;   //wait 时间
    AttSpaceTiming.HoldSetupTime = 2;   //保持时间
    AttSpaceTiming.HiZSetupTime = 1;    //高阻态时间

    HAL_NAND_Init(&device->handle, &ComSpaceTiming, &AttSpaceTiming);

    nand_reset();                       //复位NAND
//    delay_ms(100);
    wait_for_ready();

    stm32_nand_mode_set(4);                    //设置为MODE4,高速模式
    return 0;
}

static rt_err_t stm32_mtd_nand_getinfo(struct stm32_nand *nand)
{
    rt_uint32_t id;
    int i;

    RT_ASSERT(nand != RT_NULL);

    STM32_NAND_SET_CMD(NAND_READID); //发送读取ID命令
    STM32_NAND_SET_ADD(0X00);
    STM32_NAND_GET_DAT(nand->id[0]);//ID一共有5个字节
    STM32_NAND_GET_DAT(nand->id[1]);
    STM32_NAND_GET_DAT(nand->id[2]);
    STM32_NAND_GET_DAT(nand->id[3]);
    STM32_NAND_GET_DAT(nand->id[4]);

    //镁光的NAND FLASH的ID一共5个字节，但是为了方便我们只取4个字节组成一个32位的ID值
    //根据NAND FLASH的数据手册，只要是镁光的NAND FLASH，那么一个字节ID的第一个字节都是0X2C
    //所以我们就可以抛弃这个0X2C，只取后面四字节的ID值。
    id = ((rt_uint32_t)nand->id[1]) << 24 | ((rt_uint32_t)nand->id[2]) << 16 | ((rt_uint32_t)nand->id[3]) << 8 | nand->id[4];

    for(i = 0; i < sizeof(nand_info_list)/sizeof(struct nand_info); i++)
    {
        if (nand_info_list[i].id == id)
        {
            nand_obj[i].mtd_nand.page_size       = nand_info_list[i].page_size;
            nand_obj[i].mtd_nand.pages_per_block = nand_info_list[i].pages_per_block;
            nand_obj[i].mtd_nand.plane_num       = nand_info_list[i].plane_num;
            nand_obj[i].mtd_nand.oob_size        = nand_info_list[i].oob_size;
            nand_obj[i].mtd_nand.oob_free        = nand_info_list[i].oob_size - ((nand_info_list[i].page_size) * 3 / 256);
            nand_obj[i].mtd_nand.block_start     = 0;
            nand_obj[i].mtd_nand.block_end       = nand_info_list[i].block_total;
            nand_obj[i].mtd_nand.block_total     = nand_info_list[i].block_total;
            LOG_D("NAND ID: 0x%08X", id);
            return RT_EOK;
        }
    }
    if (i == sizeof(nand_info_list)/sizeof(struct nand_info))
    {
        return -RT_ERROR;
    }
    return RT_EOK;
}

static rt_err_t stm32_mtd_nand_readid(struct rt_mtd_nand_device *device)
{
    rt_uint32_t id;
    struct stm32_nand *nand;

    RT_ASSERT(device != RT_NULL);
    nand = rt_container_of(device, struct stm32_nand, mtd_nand);

    STM32_NAND_SET_CMD(NAND_READID); //发送读取ID命令
    STM32_NAND_SET_ADD(0X00);
    STM32_NAND_GET_DAT(nand->id[0]);//ID一共有5个字节
    STM32_NAND_GET_DAT(nand->id[1]);
    STM32_NAND_GET_DAT(nand->id[2]);
    STM32_NAND_GET_DAT(nand->id[3]);
    STM32_NAND_GET_DAT(nand->id[4]);

    //镁光的NAND FLASH的ID一共5个字节，但是为了方便我们只取4个字节组成一个32位的ID值
    //根据NAND FLASH的数据手册，只要是镁光的NAND FLASH，那么一个字节ID的第一个字节都是0X2C
    //所以我们就可以抛弃这个0X2C，只取后面四字节的ID值。
    id = ((rt_uint32_t)nand->id[1]) << 24 | ((rt_uint32_t)nand->id[2]) << 16 | ((rt_uint32_t)nand->id[3]) << 8 | nand->id[4];

    rt_kprintf("\nNAND ID: 0x%08X\n", id);
    return RT_EOK;

}

//static rt_err_t nand_datacorrect(uint32_t generatedEcc, uint32_t readEcc, uint8_t *data)
//{
//#define ECC_MASK28    0x0FFFFFFF          /* 28 valid ECC parity bits. */
//#define ECC_MASK      0x05555555          /* 14 ECC parity bits.       */

//    rt_uint32_t count, bitNum, byteAddr;
//    rt_uint32_t mask;
//    rt_uint32_t syndrome;
//    rt_uint32_t eccP;                            /* 14 even ECC parity bits. */
//    rt_uint32_t eccPn;                           /* 14 odd ECC parity bits.  */

//    syndrome = (generatedEcc ^ readEcc) & ECC_MASK28;

//    if (syndrome == 0)
//        return (RT_MTD_EOK);                  /* No errors in data. */

//    eccPn = syndrome & ECC_MASK;              /* Get 14 odd parity bits.  */
//    eccP  = (syndrome >> 1) & ECC_MASK;       /* Get 14 even parity bits. */

//    if ((eccPn ^ eccP) == ECC_MASK)           /* 1-bit correctable error ? */
//    {
//        bitNum = (eccP & 0x01) |
//                 ((eccP >> 1) & 0x02) |
//                 ((eccP >> 2) & 0x04);
//        LOG_D("ECC bit %d\n", bitNum);
//        byteAddr = ((eccP >> 6) & 0x001) |
//                   ((eccP >> 7) & 0x002) |
//                   ((eccP >> 8) & 0x004) |
//                   ((eccP >> 9) & 0x008) |
//                   ((eccP >> 10) & 0x010) |
//                   ((eccP >> 11) & 0x020) |
//                   ((eccP >> 12) & 0x040) |
//                   ((eccP >> 13) & 0x080) |
//                   ((eccP >> 14) & 0x100) |
//                   ((eccP >> 15) & 0x200) |
//                   ((eccP >> 16) & 0x400) ;

//        data[ byteAddr ] ^= 1 << bitNum;

//        return RT_MTD_EOK;
//    }

//    /* Count number of one's in the syndrome. */
//    count = 0;
//    mask  = 0x00800000;
//    while (mask)
//    {
//        if (syndrome & mask)
//            count++;
//        mask >>= 1;
//    }

//    if (count == 1)           /* Error in the ECC itself. */
//        return RT_MTD_EECC;

//    return -RT_MTD_EECC;       /* Unable to correct data. */

//#undef ECC_MASK
//#undef ECC_MASK24
//}

static rt_err_t stm32_mtd_nand_readpage(struct rt_mtd_nand_device *device,
                                        rt_off_t                   page,
                                        rt_uint8_t                *data,
                                        rt_uint32_t                data_len,
                                        rt_uint8_t                *spare,
                                        rt_uint32_t                spare_len)
{
    rt_uint32_t index;
    rt_uint32_t gecc, recc;
    rt_uint8_t tmp[4];
    rt_err_t result;
    rt_uint32_t i;
    struct stm32_nand *nand;

    RT_ASSERT(device != RT_NULL);
    nand = rt_container_of(device, struct stm32_nand, mtd_nand);
    page = page + device->block_start * device->pages_per_block;
    if (page / device->pages_per_block > device->block_end)
    {
        return -RT_MTD_EIO;
    }

    result = RT_MTD_EOK;
    rt_mutex_take(&nand->lock, RT_WAITING_FOREVER);

    if (data && data_len)
    {
        STM32_NAND_SET_CMD(NAND_AREA_A); //发送地址
        STM32_NAND_SET_ADD((rt_uint8_t)(0 & 0xFF));
        STM32_NAND_SET_ADD((rt_uint8_t)(0 >> 8));
        STM32_NAND_SET_ADD((rt_uint8_t)(page & 0xFF));
        STM32_NAND_SET_ADD((rt_uint8_t)(page >> 8));
        STM32_NAND_SET_ADD((rt_uint8_t)(page >> 16));
        STM32_NAND_SET_CMD(NAND_AREA_TRUE1);
        //下面两行代码是wait R/B引脚变为低电平，其实主要起延时作用的，wait NAND操作R/B引脚。因为我们是通过
        //将STM32的NWAIT引脚(NAND的R/B引脚)配置为普通IO，代码中通过读取NWAIT引脚的电平来判断NAND是否准备
        //就绪的。这个也就是模拟的方法，所以在速度很快的时候有可能NAND还没来得及操作R/B引脚来表示NAND的忙
        //闲状态，就读取了R/B引脚,这个时候肯定会出错的，事实上确实是会出错!
        stm32_nand_wait_rb(0);         //wait RB=0
        //下面2行代码是真正判断NAND是否准备好的
        stm32_nand_wait_rb(1);         //wait RB=1
        
        FMC_NAND_ECC_Enable(nand->handle.Instance, STM32_NAND_BANK);

        for (i = 0; i < data_len; i ++)
        {
            STM32_NAND_GET_DAT(data[i]);
        }
        gecc = FMC_NAND_GetECC(nand->handle.Instance, (uint32_t *)&gecc, STM32_NAND_BANK, 10);

        if (data_len == device->page_size)
        {
            for (index = 0; index < ECC_SIZE; index ++)
            {
                STM32_NAND_GET_DAT(tmp[index]);
            }

            if (spare && spare_len)
            {
                for (i = ECC_SIZE; i < spare_len; i ++)
                {
                    STM32_NAND_GET_DAT(spare[i]);
                }
                rt_memcpy(spare, tmp, ECC_SIZE);
            }

//            recc   = (tmp[3] << 24) | (tmp[2] << 16) | (tmp[1] << 8) | tmp[0];

//            if (recc != 0xFFFFFFFF && gecc != 0)
//                result = nand_datacorrect(gecc, recc, data);

//            if (result != RT_MTD_EOK)
//                LOG_D("page: %d, gecc %X, recc %X>", page, gecc, recc);

            goto _exit;
        }
    }

    if (spare && spare_len)
    {
        STM32_NAND_SET_CMD(NAND_AREA_A); //发送地址
        STM32_NAND_SET_ADD((rt_uint8_t)(device->page_size & 0xFF));
        STM32_NAND_SET_ADD((rt_uint8_t)(device->page_size >> 8));
        STM32_NAND_SET_ADD((rt_uint8_t)(page & 0xFF));
        STM32_NAND_SET_ADD((rt_uint8_t)(page >> 8));
        STM32_NAND_SET_ADD((rt_uint8_t)(page >> 16));
        STM32_NAND_SET_CMD(NAND_AREA_TRUE1);

        //下面两行代码是wait R/B引脚变为低电平，其实主要起延时作用的，wait NAND操作R/B引脚。因为我们是通过
        //将STM32的NWAIT引脚(NAND的R/B引脚)配置为普通IO，代码中通过读取NWAIT引脚的电平来判断NAND是否准备
        //就绪的。这个也就是模拟的方法，所以在速度很快的时候有可能NAND还没来得及操作R/B引脚来表示NAND的忙
        //闲状态，就读取了R/B引脚,这个时候肯定会出错的，事实上确实是会出错!
        stm32_nand_wait_rb(0);         //wait RB=0
        //下面2行代码是真正判断NAND是否准备好的
        stm32_nand_wait_rb(1);         //wait RB=1

        for (i = 0; i < spare_len; i ++)
        {
            STM32_NAND_GET_DAT(spare[i]);
        }

    }
_exit:
    rt_mutex_release(&nand->lock);

    return (result);
}

static rt_err_t stm32_mtd_nand_writepage(struct rt_mtd_nand_device *device,
        rt_off_t                   page,
        const rt_uint8_t          *data,
        rt_uint32_t                data_len,
        const rt_uint8_t          *spare,
        rt_uint32_t                spare_len)
{
    rt_err_t result;
    rt_uint32_t gecc;
    rt_uint32_t i;
    struct stm32_nand *nand;

    RT_ASSERT(device != RT_NULL);
    nand = rt_container_of(device, struct stm32_nand, mtd_nand);

    page = page + device->block_start * device->pages_per_block;
    if (page / device->pages_per_block > device->block_end)
    {
        return -RT_MTD_EIO;
    }

    result = RT_MTD_EOK;
    rt_mutex_take(&nand->lock, RT_WAITING_FOREVER);

    if (data && data_len)
    {
        STM32_NAND_SET_CMD(NAND_WRITE0); //发送地址
        STM32_NAND_SET_ADD((rt_uint8_t)(0 & 0xFF));
        STM32_NAND_SET_ADD((rt_uint8_t)(0 >> 8));
        STM32_NAND_SET_ADD((rt_uint8_t)(page & 0xFF));
        STM32_NAND_SET_ADD((rt_uint8_t)(page >> 8));
        STM32_NAND_SET_ADD((rt_uint8_t)(page >> 16));

        FMC_NAND_ECC_Enable(nand->handle.Instance, STM32_NAND_BANK);

        for (i = 0; i < data_len; i ++)
        {
            STM32_NAND_SET_DAT(data[i]);
        }
        gecc = FMC_NAND_GetECC(nand->handle.Instance, (uint32_t *)&gecc, STM32_NAND_BANK, 10);

        FMC_NAND_ECC_Disable(nand->handle.Instance, STM32_NAND_BANK);

        if (data_len == device->page_size)
        {
            STM32_NAND_SET_DAT((uint8_t)gecc);
            STM32_NAND_SET_DAT((uint8_t)(gecc >> 8));
            STM32_NAND_SET_DAT((uint8_t)(gecc >> 16));
            STM32_NAND_SET_DAT((uint8_t)(gecc >> 24));

            if (spare && spare_len)
            {
                for (i = ECC_SIZE; i < spare_len; i ++)
                {
                    STM32_NAND_SET_DAT(spare[i]);
                }
            }

        }
        STM32_NAND_SET_CMD(NAND_WRITE_TURE1);
        if (wait_for_ready() != NSTA_READY)
        {
            nand_reset();
            result = -RT_MTD_EIO;//失败
        }
        goto _exit;
    }

    if (spare && spare_len)
    {
        STM32_NAND_SET_CMD(NAND_WRITE0); //发送地址
        STM32_NAND_SET_ADD((rt_uint8_t)(device->page_size & 0xFF));
        STM32_NAND_SET_ADD((rt_uint8_t)(device->page_size >> 8));
        STM32_NAND_SET_ADD((rt_uint8_t)(page & 0xFF));
        STM32_NAND_SET_ADD((rt_uint8_t)(page >> 8));
        STM32_NAND_SET_ADD((rt_uint8_t)(page >> 16));

        if (spare && spare_len)
            for (i = ECC_SIZE; i < spare_len; i ++)
            {
                STM32_NAND_SET_DAT(spare[i]);
            }
        STM32_NAND_SET_CMD(NAND_WRITE_TURE1);
        if (wait_for_ready() != NSTA_READY)
        {
            nand_reset();
            result = -RT_MTD_EIO;//失败
        }
    }

_exit:
    rt_mutex_release(&nand->lock);

    return (result);

}

static rt_err_t stm32_mtd_nand_eraseblock(struct rt_mtd_nand_device *device,
        rt_uint32_t block)
{
    unsigned int blockPage;
    rt_err_t result;
    struct stm32_nand *nand;

    RT_ASSERT(device != RT_NULL);
    nand = rt_container_of(device, struct stm32_nand, mtd_nand);

    /* add the start blocks */
    block = block + device->block_start;
    blockPage = (block << 6);
    result = RT_MTD_EOK;

    rt_mutex_take(&nand->lock, RT_WAITING_FOREVER);

    STM32_NAND_SET_CMD(NAND_ERASE0); //发送地址
    STM32_NAND_SET_ADD((rt_uint8_t)blockPage);
    STM32_NAND_SET_ADD((rt_uint8_t)(blockPage >> 8));
    STM32_NAND_SET_ADD((rt_uint8_t)(blockPage >> 16));
    STM32_NAND_SET_CMD(NAND_ERASE1);
    if (wait_for_ready() != NSTA_READY)
    {
        nand_reset();
        result = -RT_MTD_EIO;//失败
    }
    rt_mutex_release(&nand->lock);
    return result;
}

static rt_err_t stm32_mtd_nand_pagecopy(struct rt_mtd_nand_device *device,
                                        rt_off_t                   src_page,
                                        rt_off_t                   dst_page)
{
    rt_err_t result = RT_MTD_EOK;
    rt_uint32_t source_block = 0, dest_block = 0;

    src_page = src_page + device->block_start * device->pages_per_block;
    dst_page = dst_page + device->block_start * device->pages_per_block;
    //判断源页和目的页是否在同一个plane中
    source_block = src_page / device->pages_per_block;
    dest_block = dst_page / device->pages_per_block;
    if ((source_block % 2) != (dest_block % 2))return RT_MTD_ESRC; //不在同一个plane内

    STM32_NAND_SET_CMD(NAND_MOVEDATA_CMD0);//发送命令0X00
    STM32_NAND_SET_ADD((rt_uint8_t)(0 & 0xFF));  //发送源页地址
    STM32_NAND_SET_ADD((rt_uint8_t)(0 >> 8));
    STM32_NAND_SET_ADD((rt_uint8_t)(src_page & 0xFF));
    STM32_NAND_SET_ADD((rt_uint8_t)(src_page >> 8));
    STM32_NAND_SET_ADD((rt_uint8_t)(src_page >> 16));
    STM32_NAND_SET_CMD(NAND_MOVEDATA_CMD1);//发送命令0X35

    //下面两行代码是wait R/B引脚变为低电平，其实主要起延时作用的，wait NAND操作R/B引脚。因为我们是通过
    //将STM32的NWAIT引脚(NAND的R/B引脚)配置为普通IO，代码中通过读取NWAIT引脚的电平来判断NAND是否准备
    //就绪的。这个也就是模拟的方法，所以在速度很快的时候有可能NAND还没来得及操作R/B引脚来表示NAND的忙
    //闲状态，结果我们就读取了R/B引脚,这个时候肯定会出错的，事实上确实是会出错!大家也可以将下面两行
    //代码换成延时函数,只不过这里我们为了效率所以没有用延时函数。
    result = stm32_nand_wait_rb(0);        //wait RB=0
    if (result)return -RT_MTD_EIO;  //超时退出
    //下面2行代码是真正判断NAND是否准备好的
    result = stm32_nand_wait_rb(1);        //wait RB=1
    if (result)return -RT_MTD_EIO;  //超时退出

    STM32_NAND_SET_CMD(NAND_MOVEDATA_CMD2);//发送命令0X85
    STM32_NAND_SET_ADD((rt_uint8_t)(0 & 0xFF)); //发送目的页地址
    STM32_NAND_SET_ADD((rt_uint8_t)(0 >> 8));
    STM32_NAND_SET_ADD((rt_uint8_t)(dst_page & 0xFF));
    STM32_NAND_SET_ADD((rt_uint8_t)(dst_page >> 8));
    STM32_NAND_SET_ADD((rt_uint8_t)(dst_page >> 16));
    STM32_NAND_SET_CMD(NAND_MOVEDATA_CMD3);//发送命令0X10

    if (wait_for_ready() != NSTA_READY)
    {
        nand_reset();
        return -RT_MTD_EIO;//失败
    }

    return RT_MTD_EOK;
}

static rt_err_t stm32_mtd_nand_checkblock(struct rt_mtd_nand_device *device, rt_uint32_t block)
{
    return (RT_MTD_EOK);
}

static rt_err_t stm32_mtd_nand_markbad(struct rt_mtd_nand_device *device, rt_uint32_t block)
{
    return (RT_MTD_EOK);
}

static const struct rt_mtd_nand_driver_ops ops =
{
    stm32_mtd_nand_readid,
    stm32_mtd_nand_readpage,
    stm32_mtd_nand_writepage,
    stm32_mtd_nand_pagecopy,
    stm32_mtd_nand_eraseblock,
    stm32_mtd_nand_checkblock,
    stm32_mtd_nand_markbad,
};

int rt_hw_mtd_nand_init(void)
{
    rt_size_t obj_num = sizeof(nand_obj) / sizeof(struct stm32_nand);
    rt_err_t result = 0;

    for (int i = 0; i < obj_num; i++)
    {
        nand_obj[i].config = &nand_config[i];
        nand_obj[i].mtd_nand.ops = &ops;

        /* nand init */
        if (stm32_hw_nand_init(&nand_obj[i]) != RT_EOK)
        {
            LOG_E("%s init failed", nand_obj[i].config->name);
            result = -RT_ERROR;
            goto __exit;
        }
        else
        {
            LOG_D("%s init success", nand_obj[i].config->name);

            rt_mutex_init(&nand_obj[i].lock, "nand", RT_IPC_FLAG_FIFO);
            
            if (stm32_mtd_nand_getinfo(&nand_obj[i]) == RT_EOK)
            {
                /* register mtd_nand device */
                result = rt_mtd_nand_register_device(nand_obj[i].config->name, &nand_obj[i].mtd_nand);
                RT_ASSERT(result == RT_EOK);
            }
        }
    }

__exit:

    return RT_EOK;
}
INIT_BOARD_EXPORT(rt_hw_mtd_nand_init);
