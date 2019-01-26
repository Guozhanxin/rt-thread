
#include "sensor.h"
#include <stdlib.h>
#include <string.h>

#define DBG_ENABLE
#define DBG_LEVEL DBG_INFO
#define DBG_SECTION_NAME  "sensor.test"
#define DBG_COLOR
#include <rtdbg.h>

static rt_sem_t sensor_rx_sem = RT_NULL;

static void sensor_show_data(rt_sensor_t sensor, struct rt_sensor_data *sensor_data)
{
    switch (sensor->info.type)
    {
    case RT_SEN_CLASS_ACCE:
        LOG_I("x:%5d, y:%5d, z:%5d, timestamp:%5d", sensor_data->data.acce.x, sensor_data->data.acce.y, sensor_data->data.acce.z, sensor_data->timestamp);
        break;
    case RT_SEN_CLASS_MAG:
        LOG_I("x:%5d, y:%5d, z:%5d, timestamp:%5d", sensor_data->data.mag.x, sensor_data->data.mag.y, sensor_data->data.mag.z, sensor_data->timestamp);
        break;
    case RT_SEN_CLASS_HUMI:
        LOG_I("humi:%3d.%d%%, timestamp:%5d", sensor_data->data.humi/10, sensor_data->data.humi%10, sensor_data->timestamp);
        break;
    case RT_SEN_CLASS_TEMP:
        LOG_I("temp:%5d, timestamp:%5d", sensor_data->data.temp, sensor_data->timestamp);
        break;
    case RT_SEN_CLASS_BARO:
        LOG_I("press:%5d, timestamp:%5d", sensor_data->data.baro, sensor_data->timestamp);
        break;
    default:
        break;
    }
}

rt_err_t rx_cb(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(sensor_rx_sem);
    return 0;
}

static void sensor_rx_entry(void *parameter)
{
    rt_device_t dev = parameter;
    rt_sensor_t sensor = parameter;
    struct rt_sensor_data *data;

    rt_size_t res, i;

    data = rt_malloc(sizeof(struct rt_sensor_data) * 32);

    while (1)
    {
        rt_sem_take(sensor_rx_sem, RT_WAITING_FOREVER);

        res = 1;
        res = rt_device_read(dev, 0, data, 32);
        for (i = 0; i < res; i++)
        {
            switch (sensor->info.type)
            {
            case RT_SEN_CLASS_ACCE:
                LOG_I("num:%3d, x:%5d, y:%5d, z:%5d, timestamp:%5d", i, data[i].data.acce.x, data[i].data.acce.y, data[i].data.acce.z, data[i].timestamp);
                break;
            case RT_SEN_CLASS_MAG:
                LOG_I("num:%3d, x:%5d, y:%5d, z:%5d, timestamp:%5d", i, data[i].data.mag.x, data[i].data.mag.y, data[i].data.mag.z, data[i].timestamp);
                break;
            case RT_SEN_CLASS_HUMI:
                LOG_I("num:%3d, humi:%3d.%d%%, timestamp:%5d", i, data[i].data.humi/10, data[i].data.humi%10, data[i].timestamp);
                break;
            case RT_SEN_CLASS_BARO:
                LOG_I("num:%3d, baro:%5d, timestamp:%5d", i, data[i].data.baro, data[i].timestamp);
                break;
            default:
                break;
            }
        }
//        rt_device_control(dev, RT_SEN_CTRL_CLEAR_INT, RT_NULL);
    }
}

static void sensor_fifo(int argc, char **argv)
{
    static rt_thread_t tid1 = RT_NULL;
    rt_device_t dev = RT_NULL;
    rt_sensor_t sensor;
    rt_uint8_t reg = 0xFF;

    dev = rt_device_find(argv[1]);
    if (dev == RT_NULL)
    {
        LOG_I("Can't find device:%s", argv[1]);
        return;
    }
    sensor = (rt_sensor_t)dev;

    sensor_rx_sem = rt_sem_create("dsem", 0, RT_IPC_FLAG_FIFO);
    tid1 = rt_thread_create("thread1",
                            sensor_rx_entry, sensor,
                            1024,
                            15, 5);

    if (tid1 != RT_NULL)
        rt_thread_startup(tid1);

    /* int */
    rt_device_set_rx_indicate(dev, rx_cb);
    rt_device_open(dev, RT_DEVICE_FLAG_RDWR);
    rt_device_control(dev, RT_SEN_CTRL_SET_MODE, (void *)RT_SEN_MODE_FIFO);
    rt_device_control(dev, RT_SEN_CTRL_SET_ODR, (void *)20);
    rt_device_control(dev, RT_SEN_CTRL_GET_ID, &reg);
    LOG_I("device id: 0x%x!", reg);

}
#ifdef FINSH_USING_MSH
    MSH_CMD_EXPORT(sensor_fifo, sensor fifo sample function);
#endif

static void sensor_int(int argc, char **argv)
{
    static rt_thread_t tid1 = RT_NULL;
    rt_device_t dev = RT_NULL;
    rt_sensor_t sensor;
    rt_uint8_t reg = 0xFF;

    dev = rt_device_find(argv[1]);
    if (dev == RT_NULL)
    {
        LOG_I("Can't find device:%s", argv[1]);
        return;
    }
    sensor = (rt_sensor_t)dev;

    sensor_rx_sem = rt_sem_create("dsem", 0, RT_IPC_FLAG_FIFO);
    tid1 = rt_thread_create("thread1",
                            sensor_rx_entry, sensor,
                            1024,
                            15, 5);

    if (tid1 != RT_NULL)
        rt_thread_startup(tid1);

    /* int */
    rt_device_set_rx_indicate(dev, rx_cb);
    rt_device_open(dev, RT_DEVICE_FLAG_INT_RX);
    rt_device_control(dev, RT_SEN_CTRL_SET_ODR, (void *)20);
    rt_device_control(dev, RT_SEN_CTRL_GET_ID, &reg);
    LOG_I("device id: 0x%x!", reg);

}
#ifdef FINSH_USING_MSH
    MSH_CMD_EXPORT(sensor_int, sensor polling sample function);
#endif

static void sensor_polling(int argc, char **argv)
{
    uint16_t num = 10;
    rt_device_t dev = RT_NULL;
    rt_sensor_t sensor;
    rt_uint8_t reg = 0xFF;
    struct rt_sensor_data data;
    rt_size_t res;

    dev = rt_device_find(argv[1]);
    if (dev == RT_NULL)
    {
        LOG_I("Can't find device:%s", argv[1]);
        return;
    }
    sensor = (rt_sensor_t)dev;
    /* polling */
    rt_device_open(dev, RT_DEVICE_FLAG_RDWR);
    rt_device_control(dev, RT_SEN_CTRL_GET_ID, &reg);
    LOG_I("device id: 0x%x!", reg);
    rt_device_control(dev, RT_SEN_CTRL_SET_ODR, (void *)100);

    while (num --)
    {
        res = rt_device_read(dev, 0, &data, 1);
        if (res != 1)
        {
            LOG_I("read data failed!size is %d", res);
        }
        else
        {
            sensor_show_data(sensor, &data);
        }
        rt_thread_mdelay(100);
    }
    rt_device_close(dev);
}
#ifdef FINSH_USING_MSH
    MSH_CMD_EXPORT(sensor_polling, sensor polling sample function);
#endif

static void sensor(int argc, char **argv)
{
    static rt_device_t dev = RT_NULL;
    static rt_sensor_t sensor;
    struct rt_sensor_data data;
    rt_size_t res;

    /* If the number of arguments less than 2 */
    if (argc < 2)
    {
        LOG_I("");
        LOG_I("sensor  [OPTION] [PARAM]");
        LOG_I("         probe <dev_name>      Probe sensor by given name");
        LOG_I("         info                  Get sensor info");
        LOG_I("         sr <var>              Set range to var");
        LOG_I("         sm <var>              Set work mode to var");
        LOG_I("         sp <var>              Set power mode to var");
        LOG_I("         sodr <var>            Set output date rate to var");
        LOG_I("         sleep <var>           Set sleep status");
        LOG_I("                               var = 0 means disable, = 1 means enable");
        LOG_I("         read [num]            read [num] times mpu6xxx");
        LOG_I("                               num default 5");
        return ;
    }
    else if (!strcmp(argv[1], "info"))
    {
        struct rt_sensor_info info;
        rt_device_control(dev, RT_SEN_CTRL_GET_INFO, &info);
        LOG_I("vendor :%d", info.vendor);
        LOG_I("model  :%s", info.model);
        LOG_I("unit   :%d", info.unit);
        LOG_I("intf_type :%d", info.intf_type);
        LOG_I("period_min:%d", info.period_min);
    }
    else if (!strcmp(argv[1], "read"))
    {
        uint16_t num = 5;

        if (dev == RT_NULL)
        {
            LOG_I("Please probe mpu6xxx first!");
            return ;
        }
        if (argc == 3)
        {
            num = atoi(argv[2]);
        }

        while (num --)
        {
            res = rt_device_read(dev, 0, &data, 1);
            if (res != 1)
            {
                LOG_I("read data failed!size is %d", res);
            }
            else
            {
                sensor_show_data(sensor, &data);
            }
            rt_thread_mdelay(100);
        }
    }
    else if (argc == 3)
    {
        if (!strcmp(argv[1], "probe"))
        {
            rt_uint8_t reg = 0xFF;
            if (dev)
            {
                rt_device_close(dev);
            }

            dev = rt_device_find(argv[2]);
            if (dev == RT_NULL)
            {
                LOG_I("Can't find device:%s", argv[1]);
                return;
            }
            sensor = (rt_sensor_t)dev;
            rt_device_open(dev, RT_DEVICE_FLAG_RDWR);
            rt_device_control(dev, RT_SEN_CTRL_GET_ID, &reg);
            LOG_I("device id: 0x%x!", reg);

        }
        else if (dev == RT_NULL)
        {
            LOG_I("Please probe sensor first!");
            return ;
        }
        else if (!strcmp(argv[1], "sr"))
        {
            rt_device_control(dev, RT_SEN_CTRL_SET_RANGE, (void *)atoi(argv[2]));
        }
        else if (!strcmp(argv[1], "sm"))
        {
            rt_device_control(dev, RT_SEN_CTRL_SET_MODE, (void *)atoi(argv[2]));
        }
        else if (!strcmp(argv[1], "sp"))
        {
            rt_device_control(dev, RT_SEN_CTRL_SET_POWER, (void *)atoi(argv[2]));
        }
        else if (!strcmp(argv[1], "sodr"))
        {
            rt_device_control(dev, RT_SEN_CTRL_SET_ODR, (void *)atoi(argv[2]));
        }
        else
        {
            LOG_I("Unknown command, please enter 'mpu6xxx' get help information!");
        }
    }
    else
    {
        LOG_I("Unknown command, please enter 'mpu6xxx' get help information!");
    }
}
#ifdef FINSH_USING_MSH
    MSH_CMD_EXPORT(sensor, sensor test function);
#endif
