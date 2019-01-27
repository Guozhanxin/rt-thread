

#ifndef __SENSOR_H__
#define __SENSOR_H__

#include <rtthread.h>
#include <rtdevice.h>

/* Sensor types */

#define RT_SEN_CLASS_NONE              (0)
#define RT_SEN_CLASS_ACCE              (1)  /* Accelerometer     */
#define RT_SEN_CLASS_GYRO              (2)  /* Gyroscope         */
#define RT_SEN_CLASS_MAG               (3)  /* Magnetometer      */
#define RT_SEN_CLASS_TEMP              (4)  /* Temperature       */
#define RT_SEN_CLASS_HUMI              (5)  /* Relative Humidity */
#define RT_SEN_CLASS_BARO              (6)  /* Barometer         */
#define RT_SEN_CLASS_LIGHT             (7)  /* Ambient light     */
#define RT_SEN_CLASS_PROXIMITY         (8)  /* Proximity         */
#define RT_SEN_CLASS_HR                (9)  /* Heart Rate        */
#define RT_SEN_CLASS_TVOC              (10) /* TVOC Level        */
#define RT_SEN_CLASS_NOISE             (11) /* Noise Loudness    */

#define SEN_ACCE_NAME               "_acc"  /* Accelerometer     */
#define SEN_GYRO_NAME               "_gyr"  /* Gyroscope         */
#define SEN_MAG_NAME                "_mag"  /* Magnetometer      */
#define SEN_TEMP_NAME               "_tem"  /* Temperature       */
#define SEN_HUMI_NAME               "_hum"  /* Relative Humidity */
#define SEN_BARO_NAME               "_ba"   /* Barometer         */
#define SEN_LIGHT_NAME              "_li"   /* Ambient light     */
#define SEN_PROXIMITY_NAME          "_pr"   /* Proximity         */
#define SEN_HR_NAME                 "_hr"   /* Heart Rate        */
#define SEN_TVOC_NAME               "_tv"   /* TVOC Level        */
#define SEN_NOISE_NAME              "_noi"  /* Noise Loudness    */

/* Sensor vendor types */           

#define RT_SEN_VENDOR_UNKNOWN          (0)
#define RT_SEN_VENDOR_STM              (1)  /* STMicroelectronics */
#define RT_SEN_VENDOR_BOSCH            (2)  /* Bosch */
#define RT_SEN_VENDOR_INVENSENSE       (3)  /* Invensense */
#define RT_SEN_VENDOR_SEMTECH          (4)  /* Semtech */

/* Sensor unit types */                

#define  RT_SEN_UNIT_NONE              (0)
#define  RT_SEN_UNIT_MG                (1)  /* Accelerometer     unit: mG         */
#define  RT_SEN_UNIT_UDPS              (2)  /* Gyroscope         unit: udps       */
#define  RT_SEN_UNIT_MGAUSS            (3)  /* Magnetometer      unit: mGauss     */
#define  RT_SEN_UNIT_LUX               (4)  /* Ambient light     unit: lux        */
#define  RT_SEN_UNIT_CM                (5)  /* Distance          unit: cm         */
#define  RT_SEN_UNIT_PA                (6)  /* Barometer         unit: pa         */
#define  RT_SEN_UNIT_PERMILLAGE        (7)  /* Relative Humidity unit: permillage */
#define  RT_SEN_UNIT_DCELSIUS          (8)  /* Temperature       unit: dCelsius   */
#define  RT_SEN_UNIT_HZ                (9)  /* Frequency         unit: HZ         */

/* Sensor communication interface types */

#define  RT_SEN_INTF_I2C               (1 << 0)
#define  RT_SEN_INTF_SPI               (1 << 1)
#define  RT_SEN_INTF_UART              (1 << 2)
#define  RT_SEN_INTF_ONEWIRE           (1 << 3)

/* Sensor power mode types */          

#define  RT_SEN_POWER_NONE             (0)
#define  RT_SEN_POWER_DOWN             (1)  /* power down mode   */
#define  RT_SEN_POWER_NORMAL           (2)  /* normal-power mode */
#define  RT_SEN_POWER_LOW              (3)  /* low-power mode    */
#define  RT_SEN_POWER_HIGH             (4)  /* high-power mode   */

/* Sensor work mode types */

#define  RT_SEN_MODE_NONE              (0)
#define  RT_SEN_MODE_POLLING           (1)  /* One shot only read a data */
#define  RT_SEN_MODE_INT               (2)  /* TODO: One shot interrupt only read a data */
#define  RT_SEN_MODE_FIFO              (3)  /* TODO: One shot interrupt read all fifo data */

/* Sensor control cmd types */

#define  RT_SEN_CTRL_GET_ID            (0)  /* Get device id */
#define  RT_SEN_CTRL_GET_INFO          (1)  /* Get sensor info */
#define  RT_SEN_CTRL_SET_RANGE         (2)  /* Set the measure range of sensor. unit is info of sensor */
#define  RT_SEN_CTRL_SET_ODR           (3)  /* Set output date rate. unit is HZ */
#define  RT_SEN_CTRL_SET_MODE          (4)  /* Set sensor's work mode. ex. RT_SEN_MODE_POLLING,RT_SEN_MODE_INT */
#define  RT_SEN_CTRL_SET_POWER         (5)  /* Set power mode. args type of sensor power mode. ex. RT_SEN_POWER_DOWN,RT_SEN_POWER_NORMAL */
#define  RT_SEN_CTRL_SELF_TEST         (6)  /* Take a self test */

#define  RT_SEN_FLAG_FIFO           0x200   /* FIFO    mode when open */

#define  RT_PIN_NONE                0xFFFF  /* RT PIN NONE   */
#define  RT_SEN_MODULE_MAX             (3)

#ifdef RT_USING_RTC
#define  rt_sen_get_timestamp()   time()
#else
#define  rt_sen_get_timestamp()   rt_tick_get()
#endif

struct rt_sensor_info
{
    rt_uint8_t     type;                    /* The sensor type */
    rt_uint8_t     vendor;                  /* Vendor of sensors */
    const char    *model;                   /* model name of sensor */
    rt_uint8_t     unit;                    /* unit of measurement */
    rt_uint8_t     intf_type;               /* Communication interface type */
    rt_int32_t     range_max;               /* maximum range of this sensor's value. unit is 'unit' */
    rt_int32_t     range_min;               /* minimum range of this sensor's value. unit is 'unit' */
    rt_uint32_t    period_min;              /* Minimum measurement period,unit:ms. zero = not a constant rate */
    rt_uint8_t     fifo_max;
};

struct rt_sensor_intf
{
    char                       *dev_name;   /* The name of the communication device */
    rt_uint8_t                  type;       /* Communication interface type */
    void                       *user_data;  /* Private data for the sensor. ex. i2c addr,spi cs,control I/O */
};

struct rt_sensor_config
{
    struct rt_sensor_intf        intf;    
    struct rt_device_pin_mode    irq_pin;    /* The purpose of this pin is to notification read data */    
    rt_uint8_t                   mode;
    rt_uint8_t                   power;
    rt_uint16_t                  odr;
    rt_int32_t                   range;
//    rt_uint8_t                   fifo_max;
};

struct rt_sensor_device
{
    struct rt_device             parent;    /* The standard device */
                                
    struct rt_sensor_info        info;
    struct rt_sensor_config      config;

    void                        *data_buf;
    rt_uint8_t                   data_len;
    
    struct rt_sensor_ops        *ops;
                                
    struct rt_sensor_module     *module;
};
typedef struct rt_sensor_device *rt_sensor_t;

struct rt_sensor_module
{
    rt_mutex_t                  lock;

    rt_sensor_t                 sen[RT_SEN_MODULE_MAX];
    rt_uint8_t                  sen_num;
};

/* 3-axis Data Type */
struct sensor_3_axis
{
    rt_int32_t x;
    rt_int32_t y;
    rt_int32_t z;
};

struct rt_sensor_data
{
    rt_uint32_t         timestamp;          /* The timestamp when the data was received */
    rt_uint8_t          type;               /* The sensor type of the data */
    union
    {
        struct sensor_3_axis acce;          /* Accelerometer.       unit: mG          */
        struct sensor_3_axis gyro;          /* Gyroscope.           unit: udps        */
        struct sensor_3_axis mag;           /* Magnetometer.        unit: mGauss      */
        rt_int32_t           temp;          /* Temperature.         unit: dCelsius    */
        rt_int32_t           humi;          /* Relative humidity.   unit: permillage  */
        rt_int32_t           baro;          /* Pressure.            unit: pascal (Pa) */
        rt_int32_t           light;         /* Light.               unit: lux         */
        rt_int32_t           proximity;     /* Distance.            unit: centimeters */
        rt_int32_t           hr;            /* Heat rate.           unit: HZ          */
        rt_int32_t           tvoc;          /* TVOC.                unit: permillage  */
        rt_int32_t           noise;         /* Noise Loudness.      unit: HZ          */
    }data;
};

struct rt_sensor_ops
{
    rt_size_t (*fetch_data)(struct rt_sensor_device *sensor, void *buf, rt_size_t len);
    rt_err_t (*control)(struct rt_sensor_device *sensor, int cmd, void *arg);
};
int rt_hw_sensor_register(rt_sensor_t sensor,
                           const char              *name,
                           rt_uint32_t              flag,
                           void                    *data);
#endif
