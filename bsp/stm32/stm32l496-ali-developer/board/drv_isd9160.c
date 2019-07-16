

#include "drv_isd9160.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define AUDIO_CTL_PIN            GET_PIN(D, 5)
#define AUDIO_RST_PIN            GET_PIN(D, 6)
#define ISD9160_I2C_ADDR         (0x15)


static struct rt_i2c_bus_device *i2c_dev;

//#define hal_i2c_master_send(DEV, ADDR, CMD, LEN, ISD9160_I2C_TIMEOUT) rt_i2c_master_send(i2c_dev, ISD9160_I2C_ADDR, RT_NULL, CMD, LEN)
//#define hal_i2c_master_recv(DEV, ADDR, CMD, LEN, ISD9160_I2C_TIMEOUT) rt_i2c_master_recv(i2c_dev, ISD9160_I2C_ADDR, RT_NULL, CMD, LEN)

#define ISD9160_I2C_TIMEOUT             AOS_WAIT_FOREVER
#define ISD9160_ROM_SIZE                0x24400
#define UPG_FRAME_HEAD_SIZE             sizeof(UPG_FRAME_HEAD)
#define UPG_PAYLOAD_HAED_SIZE           sizeof(UPG_PAYLOAD_HEAD)
#define UPG_PAYLOAD_DATA_SIZE           128
#define SLAVE_DATA_MAX                  (UPG_PAYLOAD_HAED_SIZE + UPG_PAYLOAD_DATA_SIZE)
#define UPG_FRAME_MAGIC                 0x18
#define UPG_HINT_GRANU                  5
#define UPG_HINT_DIV                    (100 / UPG_HINT_GRANU)

#define MCU_FLASH_START_ADDR            0x8000000
#define UPG_FLASH_BASE                  ((256 << 10) + MCU_FLASH_START_ADDR)
#define FLASH_MAGIC_HEAD                "9160"
#define FLASH_MAGIC_FILE                0x20180511

#define MAX_PATH_SIZE                   256
#define MAX_VERSION_SIZE                8



typedef enum {
	I2C_CMD_SLPRT = 0,
	I2C_CMD_UPGRADE,
	I2C_CMD_RESPONSE,
	I2C_CMD_RECORD,
	I2C_CMD_PLAYBACK,
	I2C_CMD_AUDIOSTOP,
	
	I2C_CMD_END
} I2C_PROC_CMD;

typedef enum {
	RESP_UPG_ALL_SUCCESS = 0xa8,
	RESP_UPG_ALL_FAILED,
	RESP_UPG_BINSIZE_SUCCESS,
	RESP_UPG_BINSIZE_FAILED,
	RESP_UPG_FRAMEHEAD_SUCCESS,
	RESP_UPG_FRAMEHEAD_FAILED,
	RESP_UPG_PAYLOAD_SUCCESS,
	RESP_UPG_PAYLOAD_FAILED,
	RESP_UPG_PAYLOAD_CRC,
	
	RESP_LB_OPENED,
	RESP_LB_CLOSED,
	RESP_LB_FAILED,
	
	RESP_AUDIO_RECORD,
	RESP_AUDIO_PLAYBACK,
	RESP_AUDIO_STOP,
	
	RESP_FLAG_NORMAL
} RESPONSE_FLAG;

typedef enum {
	BINTYPE_LDROM = 0,
	BINTYPE_APROM,
	BINTYPE_DATAROM,

	BINTYPE_END
} BINTYPE_UPG;

typedef enum {
	SLPRT_HINT_SWVER = 0,
	SLPRT_HINT_VR,

	SLPRT_HINT_END
} SLPRT_HINT_FLAG;

typedef struct {
	uint32_t addr;
	uint32_t size;
} UPG_FLASH_MAP;

#pragma pack(1)
typedef struct {
	uint8_t magic;
	uint8_t payload_size;
} UPG_FRAME_HEAD;

typedef struct {
	uint16_t crc;
	uint32_t offset;
} UPG_PAYLOAD_HEAD;

typedef struct {
	uint32_t magic_head;
	uint32_t file_num;
	uint8_t version[MAX_VERSION_SIZE];
} FLASH_UPG_HEAD;

typedef struct {
	uint32_t magic_file;
	uint32_t bintype;
	uint32_t size;
	uint32_t reserve;
} FLASH_FILE_HEAD;
#pragma pack()

static const UPG_FLASH_MAP g_isd9160_map_table[BINTYPE_END] = {
	{0x00100000, 4  << 10},
	{0x00000000, 60 << 10},
	{0x0000f000, 81 << 10},
};

static const char *g_slprt_hint_matching[] = {
    "isd9160 software version: ",
    "isd9160 Voice Recognition: ",
};
static FLASH_UPG_HEAD g_upg_head;
static FLASH_FILE_HEAD g_file_head[BINTYPE_END];
static char g_fw_ver[MAX_VERSION_SIZE + 1] = {0};
//static aos_mutex_t isd9160_mutex;
static CB_SWVER svfunc = NULL;
static CB_VRCMD vrfunc = NULL;

static inline void hton_4(uint8_t *data, uint32_t value)
{
	data[0] = (value >> (8 * 3)) & 0xff;
	data[1] = (value >> (8 * 2)) & 0xff;
	data[2] = (value >> (8 * 1)) & 0xff;
	data[3] = (value >> (8 * 0)) & 0xff;
}

static inline void ntoh_4(const uint8_t *data, uint32_t *value)
{
	*value = 0;
	*value |= data[0] << (8 * 3);
	*value |= data[1] << (8 * 2);
	*value |= data[2] << (8 * 1);
	*value |= data[3] << (8 * 0);
}

static inline void hton_2(uint8_t *data, uint16_t value)
{
	data[0] = (value >> (8 * 1)) & 0xff;
	data[1] = (value >> (8 * 0)) & 0xff;
}

static int isd9160_slprt_size(uint32_t *size)
{
	int ret = 0;
	uint8_t data = I2C_CMD_SLPRT;
	uint8_t size_data[4] = {0};
    rt_int8_t res = 0;

    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = ISD9160_I2C_ADDR;    /* Slave address */
    msgs[0].flags = RT_I2C_WR;        /* Write flag */
    msgs[0].buf   = &data;             /* Slave register address */
    msgs[0].len   = 1;                /* Number of bytes sent */

    msgs[1].addr  = ISD9160_I2C_ADDR;    /* Slave address */
    msgs[1].flags = RT_I2C_RD;        /* Read flag */
    msgs[1].buf   = size_data;              /* Read data pointer */
    msgs[1].len   = 4;              /* Number of bytes read */

    if (rt_i2c_transfer(i2c_dev, msgs, 2) == 2)
    {
//        res = RT_EOK;
    }
    else
    {
        res = -RT_ERROR;
    }
//    ret = rt_i2c_master_send(i2c_dev, ISD9160_I2C_ADDR, RT_NULL, &data, 1);
////	ret = hal_i2c_master_send(&brd_i2c4_dev, ISD9160_I2C_ADDR, &data, 1, ISD9160_I2C_TIMEOUT);
//	if (ret != 1) {
//		return -1;
//	}
//	ret = rt_i2c_master_recv(i2c_dev, ISD9160_I2C_ADDR, RT_NULL, size_data, 4);
////	ret = hal_i2c_master_recv(&brd_i2c4_dev, ISD9160_I2C_ADDR, size_data, 4, ISD9160_I2C_TIMEOUT);
//	if (ret != 4) {
//		return -1;
//	}
	
	ntoh_4(size_data, size);
	
	return 0;
}

static int isd9160_slprt_data(uint8_t *data, uint32_t size)
{
    if (rt_i2c_master_recv(i2c_dev, ISD9160_I2C_ADDR, RT_NULL, data, size) != size)
        return -1;
    else
        return 0;
}

static int detect_slprt_hint(const char *buf, char **key)
{
	int i;

	if (buf == NULL || key == NULL) {
		rt_kprintf("Parameters is invalid.\n");
		return -1;
	}
	*key = NULL;
	for (i = SLPRT_HINT_SWVER; i < SLPRT_HINT_END; ++i) {
		*key = strstr(buf, g_slprt_hint_matching[i]);
		if (*key) {
			*key += strlen(g_slprt_hint_matching[i]);
			break;
		}
	}
	if (i >= SLPRT_HINT_END) {
		i = -1;
	}

	return i;
}

static void hint_software_version(const char *key)
{
	char *ptemp = NULL;

	strncpy(g_fw_ver, key, MAX_VERSION_SIZE);
	ptemp = strchr(g_fw_ver, '\n');
	if (ptemp) {
		*ptemp = '\0';
	}
	rt_kprintf("%s%s\n", g_slprt_hint_matching[SLPRT_HINT_SWVER], g_fw_ver);
}

static void hint_voice_recognition(const char *key)
{
	char *ptemp = NULL;

	strncpy(g_fw_ver, key, MAX_VERSION_SIZE);
	ptemp = strchr(g_fw_ver, '\n');
	if (ptemp) {
		*ptemp = '\0';
	}
	rt_kprintf("%s%s\n", g_slprt_hint_matching[SLPRT_HINT_SWVER], g_fw_ver);
}


static int handle_slprt(void)
{
	int ret = 0;
	uint32_t size = 0;
	int slprt_num = 0;
	char buf[SLAVE_DATA_MAX] = {0};
	char *str_key = NULL;
	
	while (1) {
		ret = isd9160_slprt_size(&size);
		if (ret != 0) {
			rt_kprintf("isd9160_slprt_size return failed.\n");
			ret = -1;
			break;
		}
		if (size == 0) {
			ret = slprt_num;
			break;
		} else if (size > SLAVE_DATA_MAX) {
			rt_kprintf("isd9160_slprt_size return size is invalid.size=%ld\n", size);
			ret = -1;
			break;
		}
		memset(buf, 0, sizeof(buf));
		ret = isd9160_slprt_data((uint8_t *)buf, size);
		if (ret != 0) {
			rt_kprintf("isd9160_slprt_data return failed.\n");
			ret = -1;
			break;
		}
		ret = detect_slprt_hint(buf, &str_key);
		if (ret < 0) {
			rt_kprintf("slave_print: ");
			rt_kprintf("%s", buf);
		} else {
			switch (ret) {
				case SLPRT_HINT_SWVER:
					hint_software_version(str_key);
					if (svfunc) {
						svfunc(g_fw_ver);
					}
					break;
				case SLPRT_HINT_VR:
					if (vrfunc) {
						vrfunc(g_fw_ver, atoi(str_key));
					}
					break;
				default:
					break;
			}
		}

		++slprt_num;
	}
	
	return ret;
}

static int get_cmdresp(uint8_t cmd, uint8_t *resp)
{
	uint8_t data = cmd;
	int ret = 0;
	
    ret = rt_i2c_master_send(i2c_dev, ISD9160_I2C_ADDR, RT_NULL, &data, 1);
//	ret = hal_i2c_master_send(&brd_i2c4_dev, ISD9160_I2C_ADDR, &data, 1, ISD9160_I2C_TIMEOUT);
	if (ret != 1) {
        rt_kprintf("i2c send failed!\n");
		return -1;
	}
    
	ret = rt_i2c_master_recv(i2c_dev, ISD9160_I2C_ADDR, RT_NULL, resp, 1);
    if (ret == 1)
        return 0;
    else
    {
        rt_kprintf("i2c recv failed!\n");
        return -1;
    }
}


int handle_playback(const char *file_in_sd)
{
	int ret = 0;
	int uret = 0;
	uint8_t resp = RESP_FLAG_NORMAL;

	ret = get_cmdresp(I2C_CMD_PLAYBACK, &resp);
	if (ret != 0 || resp != RESP_AUDIO_PLAYBACK) {
		rt_kprintf("Playback failed, get_cmdresp return %d, resp = 0x%x.\n", ret, resp);
		ret = -1;
		goto END;
	}

//	ret = playback_from_sdcard(file_in_sd);
//	if (ret != 0) {
//		rt_kprintf("playback_from_sdcard return failed.\n");
//		goto END;
//	}

	ret = get_cmdresp(I2C_CMD_AUDIOSTOP, &resp);
	if (ret != 0 || resp != RESP_AUDIO_STOP) {
		rt_kprintf("Playback stop failed, get_cmdresp return %d, resp = 0x%x.\n", ret, resp);
		ret = -1;
		goto END;
	}

END:


	return ret;
}

int isd9160_play(void)
{
	int ret = 0;
	int uret = 0;
	uint8_t resp = RESP_FLAG_NORMAL;

	ret = get_cmdresp(I2C_CMD_PLAYBACK, &resp);
	if (ret != 0 || resp != RESP_AUDIO_PLAYBACK) {
		rt_kprintf("Playback failed, get_cmdresp return %d, resp = 0x%x.\n", ret, resp);
		ret = -1;
	}
    return ret;
}
MSH_CMD_EXPORT(isd9160_play, play);

int isd9160_stop(void)
{
	int ret = 0;
	int uret = 0;
	uint8_t resp = RESP_FLAG_NORMAL;

	ret = get_cmdresp(I2C_CMD_AUDIOSTOP, &resp);
	if (ret != 0 || resp != RESP_AUDIO_STOP) {
		rt_kprintf("Playback stop failed, get_cmdresp return %d, resp = 0x%x.\n", ret, resp);
		ret = -1;
	}
    return ret;
}
MSH_CMD_EXPORT(isd9160_stop, stop);

int register_swver_callback(CB_SWVER pfunc)
{
	svfunc = pfunc;

	return 0;
}

int register_vrcmd_callback(CB_VRCMD pfunc)
{
	vrfunc = pfunc;

	return 0;
}

int isd9160_loop_once(void)
{
	return handle_slprt();
}

void isd9160_reset(void)
{


    rt_pin_write(AUDIO_RST_PIN, 0);
    rt_thread_mdelay(50);
    rt_pin_write(AUDIO_RST_PIN, 1);
//	hal_gpio_output_low(&brd_gpio_table[GPIO_AUDIO_RST]);
//	krhino_task_sleep(krhino_ms_to_ticks(50));
//	hal_gpio_output_high(&brd_gpio_table[GPIO_AUDIO_RST]);
}

int isd9160_init(void)
{
	int ret = 0;

    i2c_dev = (struct rt_i2c_bus_device *)rt_device_find("i2c4");
    rt_pin_mode(AUDIO_RST_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(AUDIO_CTL_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(AUDIO_CTL_PIN, 1);
    rt_thread_mdelay(50);
	isd9160_reset();
	
	return 0;
}

