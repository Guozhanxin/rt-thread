/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-10-18     Meco Man     The first version
 */
#include <lvgl.h>
#include <lcd_port.h>

#define MY_DISP_HOR_RES LCD_W

/*A static or global variable to store the buffers*/
static lv_disp_draw_buf_t disp_buf;
rt_device_t lcd_device = 0;
/*Static or global buffer(s). The second buffer is optional*/
static lv_color_t *buf_1;
static lv_color_t *buf_2;

static lv_disp_drv_t disp_drv;  /*Descriptor of a display driver*/

/*Flush the content of the internal buffer the specific area on the display
 *You can use DMA or any hardware acceleration to do this operation in the background but
 *'lv_disp_flush_ready()' has to be called when finished.*/
static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    if (lcd_device != RT_NULL)
    {
        struct rt_device_rect_info rect_info;

        if (area->x1 >= LCD_WIDTH || area->y1 >= LCD_HEIGHT)
        {
            rt_kprintf("error!\n");
            return;
        }
        
        if (area->x2 <= 0 || area->y2 <= 0)
        {
            rt_kprintf("error!\n");
            return;
        }
    
        rect_info.x = area->x1 > 0 ? area->x1 : 0;
        rect_info.y = area->y1 > 0 ? area->y1 : 0;
        
        rect_info.width = area->x2 > LCD_WIDTH ? LCD_WIDTH : area->x2;
        rect_info.height = area->y2 > LCD_HEIGHT ? LCD_HEIGHT : area->y2;
        
        rect_info.width -= rect_info.x;
        rect_info.height -= rect_info.y;

        rt_device_control(lcd_device, RTGRAPHIC_CTRL_RECT_UPDATE, &rect_info);
    }
    /*IMPORTANT!!!
     *Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(disp_drv);
}

void lv_port_disp_init(void)
{
    rt_err_t result;
    lcd_device = rt_device_find("lcd");
    if (lcd_device != 0)
    {
        return;
    }
    result = rt_device_open(lcd_device, 0);
    if (result != RT_EOK)
    {
        return;
    }
    buf_1 = rt_malloc(LCD_WIDTH * LCD_HEIGHT);
    if(!buf_1)
    {
        rt_kprintf("no memory!\n");
        return;
    }
    buf_2 = rt_malloc(LCD_WIDTH * LCD_HEIGHT);
    if(!buf_2)
    {
        rt_kprintf("no memory!\n");
        return;
    }
    
    /*Initialize `disp_buf` with the buffer(s). With only one buffer use NULL instead buf_2 */
    lv_disp_draw_buf_init(&disp_buf, buf_1, buf_2, LCD_WIDTH * LCD_HEIGHT);

    lv_disp_drv_init(&disp_drv); /*Basic initialization*/

    /*Set the resolution of the display*/
    disp_drv.hor_res = LCD_WIDTH;
    disp_drv.ver_res = LCD_HEIGHT;

    /*Set a display buffer*/
    disp_drv.draw_buf = &disp_buf;

    /*Used to copy the buffer's content to the display*/
    disp_drv.flush_cb = disp_flush;

    /*Finally register the driver*/
    lv_disp_drv_register(&disp_drv);
}
