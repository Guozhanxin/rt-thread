
#include "board.h"
#include "rtdevice.h"
#include "drv_isd9160.h"

#define TX_DMA_FIFO_SIZE 2048
static struct rt_audio_device dev;
static SAI_HandleTypeDef hsai_BlockA2;
static DMA_HandleTypeDef hdma_sai2_a;
static SAI_HandleTypeDef* hsai = &hsai_BlockA2;
static uint8_t buf[TX_DMA_FIFO_SIZE];

static void MX_SAI2_Init(void)
{

  /* USER CODE BEGIN SAI2_Init 0 */

  /* USER CODE END SAI2_Init 0 */

  /* USER CODE BEGIN SAI2_Init 1 */

  /* USER CODE END SAI2_Init 1 */
  hsai_BlockA2.Instance = SAI2_Block_A;
  hsai_BlockA2.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA2.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA2.Init.DataSize = SAI_DATASIZE_16;
  hsai_BlockA2.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA2.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA2.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA2.Init.OutputDrive = SAI_OUTPUTDRIVE_ENABLE;
  hsai_BlockA2.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_1QF;
  hsai_BlockA2.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_16K;
  hsai_BlockA2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA2.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA2.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockA2.FrameInit.FrameLength = 64;
  hsai_BlockA2.FrameInit.ActiveFrameLength = 32;
  hsai_BlockA2.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
  hsai_BlockA2.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA2.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;
  hsai_BlockA2.SlotInit.FirstBitOffset = 0;
  hsai_BlockA2.SlotInit.SlotSize = SAI_SLOTSIZE_32B;
  hsai_BlockA2.SlotInit.SlotNumber = 2;
  hsai_BlockA2.SlotInit.SlotActive = SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_1;
  if (HAL_SAI_Init(&hsai_BlockA2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI2_Init 2 */
    /* Peripheral DMA init*/
    
    hdma_sai2_a.Instance = DMA2_Channel3;
    hdma_sai2_a.Init.Request = DMA_REQUEST_1;
    hdma_sai2_a.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_sai2_a.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_sai2_a.Init.MemInc = DMA_MINC_ENABLE;
    hdma_sai2_a.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_sai2_a.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_sai2_a.Init.Mode = DMA_CIRCULAR;
    hdma_sai2_a.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_sai2_a) != HAL_OK)
    {
      Error_Handler();
    }

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one channel to perform all the requested DMAs. */
    __HAL_LINKDMA(hsai,hdmarx,hdma_sai2_a);

    __HAL_LINKDMA(hsai,hdmatx,hdma_sai2_a);
  /* USER CODE END SAI2_Init 2 */
    HAL_DMA_Init(&hdma_sai2_a);
    __HAL_DMA_ENABLE(&hdma_sai2_a);

    HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0x01, 0);
    HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);

}

void DMA2_Channel3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_sai2_a);
}

/* SAI DMA????? */
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
    rt_audio_tx_complete(&dev);
}

/* SAI DMA????????  */
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
    rt_audio_tx_complete(&dev);
}

rt_err_t audio_getcaps(struct rt_audio_device *audio, struct rt_audio_caps *caps)
{
    rt_err_t result = RT_EOK;
    struct stm32_audio *st_audio = (struct stm32_audio *)audio->parent.user_data;

    switch (caps->main_type)
    {
    case AUDIO_TYPE_QUERY: /* qurey the types of hw_codec device */
    {
        switch (caps->sub_type)
        {
        case AUDIO_TYPE_QUERY:
            caps->udata.mask = AUDIO_TYPE_OUTPUT | AUDIO_TYPE_MIXER;
            break;

        default:
            result = -RT_ERROR;
            break;
        }

        break;
    }

    case AUDIO_TYPE_OUTPUT: /* Provide capabilities of OUTPUT unit */
    {
        switch (caps->sub_type)
        {
        case AUDIO_DSP_PARAM:
//            caps->udata.config.channels     = 1;
//            caps->udata.config.samplefmt    = st_audio->replay_config.samplefmt;
//            caps->udata.config.samplerate   = st_audio->replay_config.samplerate;
            break;

        default:
            result = -RT_ERROR;
            break;
        }

        break;
    }

    case AUDIO_TYPE_MIXER: /* report the Mixer Units */
    {
        switch (caps->sub_type)
        {
        case AUDIO_MIXER_QUERY:
            caps->udata.mask = AUDIO_MIXER_VOLUME | AUDIO_MIXER_LINE;
            break;

        case AUDIO_MIXER_VOLUME:
            caps->udata.value = 100;
            break;

        case AUDIO_MIXER_LINE:
            break;

        default:
            result = -RT_ERROR;
            break;
        }

        break;
    }

    default:
        result = -RT_ERROR;
        break;
    }

    return result;
}

rt_err_t audio_configure(struct rt_audio_device *audio, struct rt_audio_caps *caps)
{
    /* TODO */
    return 0;
}

rt_err_t audio_init(struct rt_audio_device *audio)
{
    MX_SAI2_Init();
    isd9160_init();
//    isd9160_play();
    return 0;
}

rt_err_t audio_start(struct rt_audio_device *audio, int stream)
{
    if (stream == AUDIO_STREAM_REPLAY)
    {
        rt_kprintf("start\n");
        isd9160_play();
        HAL_SAI_Transmit_DMA(&hsai_BlockA2, buf, TX_DMA_FIFO_SIZE);
    }
    return 0;
}

rt_err_t audio_stop(struct rt_audio_device *audio, int stream)
{
    if (stream == AUDIO_STREAM_REPLAY)
    {
        rt_kprintf("stop\n");
        isd9160_stop();
        HAL_SAI_DMAStop(&hsai_BlockA2);

    }
    return 0;
}

/* get page size of codec or private buffer's info */
void audio_buffer_info(struct rt_audio_device *audio, struct rt_audio_buf_info *info)
{
    /**
     *               TX_FIFO
     * +----------------+----------------+
     * |     block1     |     block2     |
     * +----------------+----------------+
     *  \  block_size  /
     */
    info->buffer = buf;
    info->total_size = TX_DMA_FIFO_SIZE;
    info->block_size = TX_DMA_FIFO_SIZE / 2;
    info->block_count = 2;
}

struct rt_audio_ops dev_ops =
{
    audio_getcaps,
    audio_configure,
    audio_init,
    audio_start,
    audio_stop,
    RT_NULL,
    /* get page size of codec or private buffer's info */
    audio_buffer_info
};

int rt_hw_audio_init(void)
{
    dev.ops = &dev_ops;
    
    rt_audio_register(&dev, "sound0", RT_DEVICE_FLAG_WRONLY | RT_DEVICE_FLAG_DMA_TX, RT_NULL);
    
    return 0;
}
INIT_COMPONENT_EXPORT(rt_hw_audio_init);
