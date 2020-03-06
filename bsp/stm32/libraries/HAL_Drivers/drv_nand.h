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

#ifndef __DRV_NAND_H__
#define __DRV_NAND_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <board.h>
#include <rtdevice.h>

#define NAND_MAX_PAGE_SIZE      4096        //定义NAND FLASH的最大的PAGE大小（不包括SPARE区），默认4096字节
#define NAND_ECC_SECTOR_SIZE    512         //执行ECC计算的单元大小，默认512字节

#define NAND_RB_PIN             56
#define NAND_RB                 rt_pin_read(NAND_RB_PIN)    //NAND Flash的闲/忙引脚 

#define NAND_ADDRESS            ((rt_uint32_t)0x80000000)   //nand flash的访问地址,接NCE3,地址为:0X8000 0000
#define NAND_CMD                (uint32_t)(0x010000)        //发送命令
#define NAND_ADDR               (uint32_t)(0x020000)        //发送地址

//*((volatile rt_uint8_t *) 0X80000000)
//*((volatile rt_uint8_t *) 0X80010000)
//*((volatile rt_uint8_t *) 0X80020000)

//NAND FLASH命令
#define NAND_READID             0X90        //读ID指令
#define NAND_FEATURE            0XEF        //设置特性指令
#define NAND_RESET              0XFF        //复位NAND
#define NAND_READSTA            0X70        //读状态
#define NAND_AREA_A             0X00
#define NAND_AREA_TRUE1         0X30
#define NAND_WRITE0             0X80
#define NAND_WRITE_TURE1        0X10
#define NAND_ERASE0             0X60
#define NAND_ERASE1             0XD0
#define NAND_MOVEDATA_CMD0      0X00
#define NAND_MOVEDATA_CMD1      0X35
#define NAND_MOVEDATA_CMD2      0X85
#define NAND_MOVEDATA_CMD3      0X10

//NAND FLASH状态
#define NSTA_READY              0X40        //nand已经准备好
#define NSTA_ERROR              0X01        //nand错误
#define NSTA_TIMEOUT            0X02        //超时
#define NSTA_ECC1BITERR         0X03        //ECC 1bit错误
#define NSTA_ECC2BITERR         0X04        //ECC 2bit以上错误

//NAND FLASH型号和对应的ID号
#define MT29F4G08ABADA          0XDC909556  //MT29F4G08ABADA
#define MT29F16G08ABABA         0X48002689  //MT29F16G08ABABA

#ifndef NAND_FLASH_INFO_TABLE
/*
 * | name | id | type_id | capacity_id | capacity | write_mode | erase_gran | erase_gran_cmd |
 */
#define NAND_FLASH_INFO_TABLE                            \
{                                                        \
    {"MT29F4G08", MT29F4G08ABADA, 2048, 64, 2, 64},      \
}
#endif

struct nand_info
{
    char *name;                     /* NAND FLASH Name */
    uint32_t id;             		/* NAND FLASH ID */
    rt_uint16_t page_size;          /* The Page size in the flash */
    rt_uint16_t oob_size;           /* Out of bank size */
    rt_uint16_t plane_num;          /* the number of plane in the NAND Flash */
    rt_uint16_t block_total;
    rt_uint32_t pages_per_block;    /* The number of page a block */
};

int rt_hw_mtd_nand_init(void);

/* stm32 config class */
struct stm32_nand_config
{
    const char *name;
    FMC_NAND_TypeDef *Instance;
};

/* stm32 nand dirver class */
struct stm32_nand
{
    NAND_HandleTypeDef handle;
    struct stm32_nand_config *config;
    struct rt_mtd_nand_device mtd_nand;

    rt_uint8_t id[5];
    struct rt_mutex lock;
    struct rt_completion comp;
};


#ifdef __cplusplus
}
#endif

#endif
