
#include "sensor.h"

#define DBG_ENABLE
#define DBG_LEVEL DBG_INFO
#define DBG_SECTION_NAME  "SENSOR"
#define DBG_COLOR
#include <rtdbg.h>

/* any sensor callback function */

void rt_sensor_cb(rt_sensor_t sen)
{
    if (sen->parent.rx_indicate == RT_NULL)
    {
        return;
    }
    if (sen->data_len > 0)
    {
        sen->parent.rx_indicate(&sen->parent, sen->data_len / sizeof(struct rt_sensor_data));
    }
    else if (sen->config.mode == RT_SEN_MODE_INT)
    {
        sen->parent.rx_indicate(&sen->parent, 1);
    }
    else if(sen->config.mode == RT_SEN_MODE_FIFO)
    {
        sen->parent.rx_indicate(&sen->parent, sen->info.fifo_max);
    }
}

/* rx interrupt callback function */

static void irq_callback(void *args)
{
    rt_sensor_t sensor = args;

    if (sensor->module)
    {
        rt_uint8_t i;
        for (i = 0; i< sensor->module->sen_num; i++)
        {
            rt_sensor_cb(sensor->module->sen[i]);
        }
    }
    else
    {
        rt_sensor_cb(sensor);
    }
}

static rt_err_t rt_sensor_irq_init(rt_sensor_t sensor)
{
    if (sensor->config.irq_pin.pin == RT_PIN_NONE)
    {
        return -RT_EINVAL;
    }
    
    rt_pin_mode(sensor->config.irq_pin.pin, sensor->config.irq_pin.mode);

    if (sensor->config.irq_pin.mode == PIN_MODE_INPUT_PULLDOWN)
    {
        rt_pin_attach_irq(sensor->config.irq_pin.pin, PIN_IRQ_MODE_RISING, irq_callback, (void *)sensor);
    }
    else if (sensor->config.irq_pin.mode == PIN_MODE_INPUT_PULLUP)
    {
        rt_pin_attach_irq(sensor->config.irq_pin.pin, PIN_IRQ_MODE_FALLING, irq_callback, (void *)sensor);
    }
    else if (sensor->config.irq_pin.mode == PIN_MODE_INPUT)
    {
        rt_pin_attach_irq(sensor->config.irq_pin.pin, PIN_IRQ_MODE_RISING_FALLING, irq_callback, (void *)sensor);
    }
    rt_pin_irq_enable(sensor->config.irq_pin.pin, RT_TRUE);

    LOG_I("interrupt init success");

    return 0;
}

static void rt_sensor_irq_enable(rt_sensor_t sensor)
{
    if (sensor->config.irq_pin.pin != RT_PIN_NONE)
    {
        rt_pin_irq_enable(sensor->config.irq_pin.pin, RT_TRUE);
    }
}

static void rt_sensor_irq_disable(rt_sensor_t sensor)
{
    if (sensor->config.irq_pin.pin != RT_PIN_NONE)
    {
        rt_pin_irq_enable(sensor->config.irq_pin.pin, RT_FALSE);
    }
}

static rt_err_t rt_sensor_open(rt_device_t dev, rt_uint16_t oflag)
{
    rt_sensor_t sensor = (rt_sensor_t)dev;
    RT_ASSERT(dev != RT_NULL);

    if (sensor->module)
        rt_mutex_take(sensor->module->lock, RT_WAITING_FOREVER);
    
    if (oflag & RT_DEVICE_FLAG_INT_RX)
    {
        if (sensor->ops->control(sensor, RT_SEN_CTRL_SET_MODE, (void*)RT_SEN_MODE_INT) == RT_EOK)
        {
            sensor->config.mode = RT_SEN_MODE_INT;
            rt_sensor_irq_init(sensor);
        }
    }
    else if (oflag & RT_SEN_FLAG_FIFO)
    {
        if (sensor->ops->control(sensor, RT_SEN_CTRL_SET_MODE, (void*)RT_SEN_MODE_FIFO) == RT_EOK)
        {
            sensor->config.mode = RT_SEN_MODE_FIFO;
            rt_sensor_irq_init(sensor);
        }
    }
    else
    {
        if (sensor->ops->control(sensor, RT_SEN_CTRL_SET_MODE, (void*)RT_SEN_MODE_POLLING) == RT_EOK)
            sensor->config.mode = RT_SEN_MODE_POLLING;
    }

    if (sensor->ops->control(sensor, RT_SEN_CTRL_SET_POWER, (void*)RT_SEN_POWER_NORMAL) == RT_EOK)
        sensor->config.power = RT_SEN_POWER_NORMAL;

    if (sensor->module)
        rt_mutex_release(sensor->module->lock);

    return RT_EOK;
}

static rt_err_t  rt_sensor_close(rt_device_t dev)
{
    rt_sensor_t sensor = (rt_sensor_t)dev;
    RT_ASSERT(dev != RT_NULL);

    if (sensor->module)
        rt_mutex_take(sensor->module->lock, RT_WAITING_FOREVER);

    if (sensor->ops->control(sensor, RT_SEN_CTRL_SET_POWER, (void*)RT_SEN_POWER_DOWN) == RT_EOK)
        sensor->config.power = RT_SEN_POWER_DOWN;

    rt_sensor_irq_disable(sensor);

    if (sensor->module)
        rt_mutex_release(sensor->module->lock);

    return RT_EOK;
}

static rt_size_t rt_sensor_read(rt_device_t dev, rt_off_t pos, void *buf, rt_size_t len)
{
    rt_sensor_t sensor = (rt_sensor_t)dev;
    rt_size_t result = 0;
    RT_ASSERT(dev != RT_NULL);

    if (buf == NULL || len == 0)
    {
        return 0;
    }
    if (sensor->module)
        rt_mutex_take(sensor->module->lock, RT_WAITING_FOREVER);
    
    if (sensor->data_len > 0)
    {
        if (len > sensor->data_len / sizeof(struct rt_sensor_data))
            len = sensor->data_len / sizeof(struct rt_sensor_data);
        
        rt_memcpy(buf, sensor->data_buf, len * sizeof(struct rt_sensor_data));
        result = len;
        sensor->data_len = 0;
    }
    else
    {
        result = sensor->ops->fetch_data(sensor, buf, len);
    }

    if (sensor->module)
        rt_mutex_release(sensor->module->lock);

    return result;
}

static rt_err_t rt_sensor_control(rt_device_t dev, int cmd, void *args)
{
    rt_sensor_t sensor = (rt_sensor_t)dev;
    rt_err_t result = RT_EOK;
    RT_ASSERT(dev != RT_NULL);

    if (sensor->module)
        rt_mutex_take(sensor->module->lock, RT_WAITING_FOREVER);

    switch (cmd)
    {
    case RT_SEN_CTRL_GET_ID:
        if (args)
        {
            sensor->ops->control(sensor, RT_SEN_CTRL_GET_ID, args);
        }
        break;
    case RT_SEN_CTRL_GET_INFO:
        if (args)
        {
            rt_memcpy(args, &sensor->info, sizeof(struct rt_sensor_info));
        }
        break;
    case RT_SEN_CTRL_SET_RANGE:
        result = sensor->ops->control(sensor, RT_SEN_CTRL_SET_RANGE, args);
        if (result == RT_EOK)
        {
            sensor->config.range = (rt_int32_t)args;
            LOG_D("set range %d", sensor->config.range);
        }
        break;
    case RT_SEN_CTRL_SET_ODR:
        result = sensor->ops->control(sensor, RT_SEN_CTRL_SET_ODR, args);
        if (result == RT_EOK)
        {
            sensor->config.odr = (rt_uint32_t)args & 0xFFFF;
            LOG_D("set odr %d", sensor->config.odr);
        }
        break;
    case RT_SEN_CTRL_SET_MODE:
        result = sensor->ops->control(sensor, RT_SEN_CTRL_SET_MODE, args);
        if (result == RT_EOK)
        {
            sensor->config.mode = (rt_uint32_t)args & 0xFF;
            LOG_D("set work mode code:", sensor->config.mode);
            
            if (sensor->config.mode == RT_SEN_MODE_POLLING)
                rt_sensor_irq_disable(sensor);
            else if(sensor->config.mode == RT_SEN_MODE_INT || sensor->config.mode == RT_SEN_MODE_FIFO)
                rt_sensor_irq_enable(sensor);
        }
        break;
    case RT_SEN_CTRL_SET_POWER:
        result = sensor->ops->control(sensor, RT_SEN_CTRL_SET_POWER, args);
        if (result == RT_EOK)
        {
            sensor->config.power = (rt_uint32_t)args & 0xFF;
            LOG_D("set power mode code:", sensor->config.power);
        }
        break;
    case RT_SEN_CTRL_SELF_TEST:
        result = sensor->ops->control(sensor, RT_SEN_CTRL_SELF_TEST, args);
        break;
    default:
        return -RT_ERROR;
    }

    if (sensor->module)
        rt_mutex_release(sensor->module->lock);

    return result;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops rt_sensor_ops =
{
    RT_NULL,
    rt_sensor_open,
    rt_sensor_close,
    rt_sensor_read,
    RT_NULL,
    rt_sensor_control
};
#endif

int rt_hw_sensor_register(rt_sensor_t sensor,
                           const char              *name,
                           rt_uint32_t              flag,
                           void                    *data)
{
    rt_int8_t result;
    rt_device_t device;
    RT_ASSERT(sensor != RT_NULL);

    device = &sensor->parent;

#ifdef RT_USING_DEVICE_OPS
    device->ops         = &rt_sensor_ops;
#else
    device->init        = RT_NULL;
    device->open        = rt_sensor_open;
    device->close       = rt_sensor_close;
    device->read        = rt_sensor_read;
    device->write       = RT_NULL;
    device->control     = rt_sensor_control;
#endif
    device->type        = RT_Device_Class_Sensor;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;
    device->user_data   = data;

    /* TODO: register a sensor device */
    result = rt_device_register(device, name, flag | RT_DEVICE_FLAG_STANDALONE);
    if (result != RT_EOK)
    {
        LOG_E("rt_sensor register err code: %d", result);
        return result;
    }

    LOG_I("rt_sensor init success");
    return RT_EOK;
}
