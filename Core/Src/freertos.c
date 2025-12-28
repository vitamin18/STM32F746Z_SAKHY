/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "audio_mixing.h"
#include "device.h"
#include "ethernet.h"
#include "fatfs_app.h"
#include "heap_pool.h"
#include "io_control.h"
#include "iwdg.h"
#include "rtc.h"
#include "sai.h"
#include "sdmmc.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum
{
  TASK_ID_0,
  TASK_ID_1,
  TASK_ID_2,
  TASK_ID_3,
  TASK_ID_ALL,
};
/**
 * @brief W5500 operation state.
 */
typedef enum
{
  W5500_CMD_RECEIVE,         /**< Receiving data from W5500 */
  W5500_CMD_TRANSMIT_SAMPLE, /**< Transmitting data sample to W5500 */
  W5500_CMD_TRANSMIT_FIELD,  /**< Transmitting data field to W5500 */
} w5500_cmd_e;

typedef enum
{
  IO_CMD_LOW_BATTERY,
  IO_CMD_SD_CD,
  IO_CMD_PTT,
  IO_CMD_CALL_LOCAL,
  IO_CMD_CALL_DEST,
  IO_CMD_ALARM_LOCAL,
  IO_CMD_ALARM_DEST,
  IO_CMD_RESET_ALARM,
  IO_CMD_SENSOR_LOCAL,
  IO_CMD_SENSOR_DEST,
} io_cmd_e;

typedef enum
{
  SD_INIT,
  SD_DEINIT,
  SD_CREATE_WAV_FILE,
  SD_WRITE_WAV_DATA,
  SD_FILE_WAV_CLOSE,
} sd_cmd_e;

typedef struct
{
  uint8_t active_buf;
  uint32_t write_offset;
} sd_write_state_t;

typedef struct
{
  uint32_t cnt;
} record_t;

typedef enum
{
  FIELD_CMD,
  FIELD_DATA,
  FIELD_SIZE,
} field_size_e;

typedef struct
{
  w5500_cmd_e w5500_cmd;
  io_cmd_e io_cmd;
  uint8_t field;
  uint8_t *data_ptr;
  uint16_t data_len;
} w5500_msg_t;

typedef enum
{
  VOCODER_CMD_BUFFER_0,
  VOCODER_CMD_BUFFER_1
} vocoder_cmd_t;

typedef struct
{
  vocoder_cmd_t cmd;
  int16_t *ptr;
} vocoder_msg_t;

typedef struct
{
  int16_t *ptr;
} audio_msg_t;

typedef struct
{
  io_cmd_e cmd;
  uint8_t field;
} io_msg_t;

typedef struct
{
  sd_cmd_e cmd;
  int16_t *ptr;
} sd_msg_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_TASKS TASK_ID_ALL

#define AUDIO_SAMPLE_SIZE 160
#define AUDIO_BUF_SIZE AUDIO_SAMPLE_SIZE
#define W5500_BUF_SIZE (AUDIO_SAMPLE_SIZE * 2)
#define SD_WRITE_BUF_SIZE 8000  // 8000 = 500 ms
#define RECORD_BLOCK_COUNT 6000 // 3000 sec one block 500 ms

#define HEAP_POOL_AUDIO_ETH_TO_VOCODER_BLOCK_COUNT 20
#define HEAP_POOL_AUDIO_ETH_TO_VOCODER_BLOCK_SIZE (AUDIO_SAMPLE_SIZE * 2)

#define HEAP_POOL_AUDIO_WR_SD_BLOCK_COUNT 20
#define HEAP_POOL_AUDIO_WR_SD_BLOCK_SIZE (AUDIO_SAMPLE_SIZE * 2)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile uint8_t task_alive[NUM_TASKS] = {0, 0, 0, 0};

uint8_t w5500_buf_tx[W5500_BUF_SIZE];
uint8_t w5500_buf_rx[W5500_BUF_SIZE];
/* --- Double buffers --- */
int16_t sai_a_buf[AUDIO_BUF_SIZE * 2] __ALIGNED(4);
int16_t sai_b_buf[AUDIO_BUF_SIZE * 2] __ALIGNED(4);
int16_t silence_buffer[AUDIO_SAMPLE_SIZE] = {0};

__attribute__((aligned(32))) static uint8_t sd_buf[2][SD_WRITE_BUF_SIZE];
static sd_write_state_t sd_state = {.active_buf = 0, .write_offset = 0};

uint8_t heap_pool_audio_eth_to_vocoder_memory[HEAP_POOL_AUDIO_ETH_TO_VOCODER_BLOCK_COUNT *
                                              HEAP_POOL_AUDIO_ETH_TO_VOCODER_BLOCK_SIZE] __ALIGNED(4);
uint8_t heap_pool_audio_eth_to_vocoder_bitmap[(HEAP_POOL_AUDIO_ETH_TO_VOCODER_BLOCK_COUNT - 7) / 8];
heap_pool_t heap_pool_audio_eth_to_vocoder;

uint8_t heap_pool_audio_wr_sd_memory[HEAP_POOL_AUDIO_WR_SD_BLOCK_COUNT * HEAP_POOL_AUDIO_WR_SD_BLOCK_SIZE] __ALIGNED(4);
uint8_t heap_pool_audio_wr_sd_bitmap[(HEAP_POOL_AUDIO_WR_SD_BLOCK_COUNT - 7) / 8];
heap_pool_t heap_pool_audio_wr_sd;

field_t device_local;
field_t device_remote;

record_t record;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for W5500Task */
osThreadId_t W5500TaskHandle;
const osThreadAttr_t W5500Task_attributes = {
  .name = "W5500Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SdCardTask */
osThreadId_t SdCardTaskHandle;
const osThreadAttr_t SdCardTask_attributes = {
  .name = "SdCardTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for AudioMixingTask */
osThreadId_t AudioMixingTaskHandle;
const osThreadAttr_t AudioMixingTask_attributes = {
  .name = "AudioMixingTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ProcessIOTask */
osThreadId_t ProcessIOTaskHandle;
const osThreadAttr_t ProcessIOTask_attributes = {
  .name = "ProcessIOTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for WatchdogTask */
osThreadId_t WatchdogTaskHandle;
const osThreadAttr_t WatchdogTask_attributes = {
  .name = "WatchdogTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for W5500Queue */
osMessageQueueId_t W5500QueueHandle;
const osMessageQueueAttr_t W5500Queue_attributes = {
  .name = "W5500Queue"
};
/* Definitions for RxVocoderToAudioMixQueue */
osMessageQueueId_t RxVocoderToAudioMixQueueHandle;
const osMessageQueueAttr_t RxVocoderToAudioMixQueue_attributes = {
  .name = "RxVocoderToAudioMixQueue"
};
/* Definitions for TxVocoderToAudioMixQueue */
osMessageQueueId_t TxVocoderToAudioMixQueueHandle;
const osMessageQueueAttr_t TxVocoderToAudioMixQueue_attributes = {
  .name = "TxVocoderToAudioMixQueue"
};
/* Definitions for EthToAudioMixQueue */
osMessageQueueId_t EthToAudioMixQueueHandle;
const osMessageQueueAttr_t EthToAudioMixQueue_attributes = {
  .name = "EthToAudioMixQueue"
};
/* Definitions for AudioVocoderToProcessAudioQueue */
osMessageQueueId_t AudioVocoderToProcessAudioQueueHandle;
const osMessageQueueAttr_t AudioVocoderToProcessAudioQueue_attributes = {
  .name = "AudioVocoderToProcessAudioQueue"
};
/* Definitions for AudioWriteSdQueue */
osMessageQueueId_t AudioWriteSdQueueHandle;
const osMessageQueueAttr_t AudioWriteSdQueue_attributes = {
  .name = "AudioWriteSdQueue"
};
/* Definitions for AudioEthToVocoderQueue */
osMessageQueueId_t AudioEthToVocoderQueueHandle;
const osMessageQueueAttr_t AudioEthToVocoderQueue_attributes = {
  .name = "AudioEthToVocoderQueue"
};
/* Definitions for IOQueue */
osMessageQueueId_t IOQueueHandle;
const osMessageQueueAttr_t IOQueue_attributes = {
  .name = "IOQueue"
};
/* Definitions for CallLedTimer */
osTimerId_t CallLedTimerHandle;
const osTimerAttr_t CallLedTimer_attributes = {
  .name = "CallLedTimer"
};
/* Definitions for AudioWrSdBinarySem */
osSemaphoreId_t AudioWrSdBinarySemHandle;
const osSemaphoreAttr_t AudioWrSdBinarySem_attributes = {
  .name = "AudioWrSdBinarySem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void timer_call_start(void);
void timer_call_stop(void);
static uint8_t all_tasks_alive(void);
/* USER CODE END FunctionPrototypes */

void Default_Task(void *argument);
void W5500_Task(void *argument);
void SdCard_Task(void *argument);
void AudioMixing_Task(void *argument);
void ProcessIO_Task(void *argument);
void Watchdog_Task(void *argument);
void CallLedCallback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  heap_pool_init(&heap_pool_audio_eth_to_vocoder, heap_pool_audio_eth_to_vocoder_memory, heap_pool_audio_eth_to_vocoder_bitmap,
                 HEAP_POOL_AUDIO_ETH_TO_VOCODER_BLOCK_SIZE, HEAP_POOL_AUDIO_ETH_TO_VOCODER_BLOCK_COUNT);
  heap_pool_init(&heap_pool_audio_wr_sd, heap_pool_audio_wr_sd_memory, heap_pool_audio_wr_sd_bitmap, HEAP_POOL_AUDIO_WR_SD_BLOCK_SIZE,
                 HEAP_POOL_AUDIO_WR_SD_BLOCK_COUNT);

  //  hp_off();
  //  sp_off();
  mic_ar_set(AR_MED);
  mic_gain_set(GAIN_50DB);
  vocoder_enable(); // vocoder disable
  BCLKR_Set_Low();
  HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t *)sai_a_buf, AUDIO_BUF_SIZE * 2);
  HAL_SAI_Receive_DMA(&hsai_BlockB1, (uint8_t *)sai_b_buf, AUDIO_BUF_SIZE * 2);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of AudioWrSdBinarySem */
  AudioWrSdBinarySemHandle = osSemaphoreNew(1, 1, &AudioWrSdBinarySem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of CallLedTimer */
  CallLedTimerHandle = osTimerNew(CallLedCallback, osTimerPeriodic, NULL, &CallLedTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of W5500Queue */
  W5500QueueHandle = osMessageQueueNew (16, sizeof(w5500_msg_t), &W5500Queue_attributes);

  /* creation of RxVocoderToAudioMixQueue */
  RxVocoderToAudioMixQueueHandle = osMessageQueueNew (16, sizeof(audio_msg_t), &RxVocoderToAudioMixQueue_attributes);

  /* creation of TxVocoderToAudioMixQueue */
  TxVocoderToAudioMixQueueHandle = osMessageQueueNew (16, sizeof(audio_msg_t), &TxVocoderToAudioMixQueue_attributes);

  /* creation of EthToAudioMixQueue */
  EthToAudioMixQueueHandle = osMessageQueueNew (16, sizeof(audio_msg_t), &EthToAudioMixQueue_attributes);

  /* creation of AudioVocoderToProcessAudioQueue */
  AudioVocoderToProcessAudioQueueHandle = osMessageQueueNew (16, sizeof(audio_msg_t), &AudioVocoderToProcessAudioQueue_attributes);

  /* creation of AudioWriteSdQueue */
  AudioWriteSdQueueHandle = osMessageQueueNew (16, sizeof(sd_msg_t), &AudioWriteSdQueue_attributes);

  /* creation of AudioEthToVocoderQueue */
  AudioEthToVocoderQueueHandle = osMessageQueueNew (16, sizeof(audio_msg_t), &AudioEthToVocoderQueue_attributes);

  /* creation of IOQueue */
  IOQueueHandle = osMessageQueueNew (16, sizeof(io_msg_t), &IOQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(Default_Task, NULL, &defaultTask_attributes);

  /* creation of W5500Task */
  W5500TaskHandle = osThreadNew(W5500_Task, NULL, &W5500Task_attributes);

  /* creation of SdCardTask */
  SdCardTaskHandle = osThreadNew(SdCard_Task, NULL, &SdCardTask_attributes);

  /* creation of AudioMixingTask */
  AudioMixingTaskHandle = osThreadNew(AudioMixing_Task, NULL, &AudioMixingTask_attributes);

  /* creation of ProcessIOTask */
  ProcessIOTaskHandle = osThreadNew(ProcessIO_Task, NULL, &ProcessIOTask_attributes);

  /* creation of WatchdogTask */
  WatchdogTaskHandle = osThreadNew(Watchdog_Task, NULL, &WatchdogTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Default_Task */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_Default_Task */
void Default_Task(void *argument)
{
  /* USER CODE BEGIN Default_Task */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END Default_Task */
}

/* USER CODE BEGIN Header_W5500_Task */
/**
 * @brief Function implementing the W5500Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_W5500_Task */
void W5500_Task(void *argument)
{
  /* USER CODE BEGIN W5500_Task */
  osStatus_t status;
  w5500_msg_t w5500_msg;
  io_msg_t io_msg;
  uint8_t *u8_ptr;
  audio_msg_t audio_msg;
  uint16_t len;
  uint8_t sn;
  uint8_t from_pool = 1;

  ethernet_init();
  udp_socket_open();
  /* Infinite loop */
  for (;;)
  {
    status = osMessageQueueGet(W5500QueueHandle, &w5500_msg, NULL, 100);
    if (status == osOK)
    {
      switch (w5500_msg.w5500_cmd)
      {
      case W5500_CMD_RECEIVE:
        u8_ptr = heap_pool_alloc(&heap_pool_audio_eth_to_vocoder);
        if (u8_ptr == NULL)
        {
          u8_ptr = w5500_buf_rx;
          from_pool = 0;
        }
        else
        {
          from_pool = 1;
        }
        len = udp_socket_receive(u8_ptr);
        if (len == AUDIO_SAMPLE_SIZE * 2)
        {
          if (ptt_state_get() == PTT_ACTIVE)
          {
            audio_msg.ptr = (int16_t *)u8_ptr;
            osMessageQueuePut(EthToAudioMixQueueHandle, &audio_msg, 0U, 0U);
          }
          else
          {
            if (from_pool)
            {
              heap_pool_free(&heap_pool_audio_eth_to_vocoder, u8_ptr);
            }
          }
        }
        else if (len == FIELD_SIZE)
        {
          io_msg.cmd = u8_ptr[FIELD_CMD];
          io_msg.field = u8_ptr[FIELD_DATA];
          osMessageQueuePut(IOQueueHandle, &io_msg, 0U, 0U);
          if (from_pool)
          {
            heap_pool_free(&heap_pool_audio_eth_to_vocoder, u8_ptr);
          }
        }
        else
        {
          if (from_pool)
          {
            heap_pool_free(&heap_pool_audio_eth_to_vocoder, u8_ptr);
          }
        }
        break;
      case W5500_CMD_TRANSMIT_SAMPLE: udp_socket_send((uint8_t *)w5500_msg.data_ptr, AUDIO_SAMPLE_SIZE * 2); break;
      case W5500_CMD_TRANSMIT_FIELD:
        w5500_buf_tx[FIELD_CMD] = w5500_msg.io_cmd;
        w5500_buf_tx[FIELD_DATA] = w5500_msg.field;
        udp_socket_send(w5500_buf_tx, FIELD_SIZE);
        break;
      default: break;
      }
    }
    else
    {
      sn = w5500_get_sn_ir(UDP_SOCKET);
      if (sn & SOCKET_INT_RECV)
      {
        w5500_msg_t w5500_msg = {.w5500_cmd = W5500_CMD_RECEIVE};
        osMessageQueuePut(W5500QueueHandle, &w5500_msg, 0U, 0U);
      }
    }
    task_alive[TASK_ID_0] = 1;
  }
  /* USER CODE END W5500_Task */
}

/* USER CODE BEGIN Header_SdCard_Task */
/**
 * @brief Function implementing the SdCardTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_SdCard_Task */
void SdCard_Task(void *argument)
{
  /* USER CODE BEGIN SdCard_Task */
  osStatus_t status = osError;
  uint8_t sd_card_init_flag = 0;
  uint8_t rtc_time_set_flag = 0;
  sd_msg_t sd_msg;
  RTC_TimeTypeDef time = {0};
  RTC_DateTypeDef date = {0};
  /* Infinite loop */
  sd_card_led_off();
  if (sd_cd_state_get() == SD_CD_LOW)
  {
    sd_msg.cmd = SD_INIT;
    osMessageQueuePut(AudioWriteSdQueueHandle, &sd_msg, 0U, 0U);
  }

  for (;;)
  {
    status = osMessageQueueGet(AudioWriteSdQueueHandle, &sd_msg, NULL, 100);
    if (status == osOK)
    {
      switch (sd_msg.cmd)
      {
      case SD_INIT:
        if (!sd_card_init_flag)
        {
          MX_FATFS_Init();
          __HAL_RCC_SDMMC1_FORCE_RESET();
          __HAL_RCC_SDMMC1_RELEASE_RESET();
          if (HAL_SD_Init(&hsd1) != HAL_OK)
          {
            MX_FATFS_DeInit();
            break;
          }
          if (fs_check_fat32() == FR_OK)
          {
            printf("fs_check_fat32 OK\r\n");
            sd_card_led_on();
            sd_card_init_flag = 1;
            printf("sd_card_init_flag = %d\r\n", sd_card_init_flag);
            wav_file_count_set(fatfs32_count_wav_files());
            printf("count_wav_files: %d\r\n", (int)wav_file_count_get());
            if (!rtc_time_set_flag)
            {
              FRESULT res = read_datetime_from_index_dat(&time, &date);
              if (res == FR_OK)
              {
                HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN);
                HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN);
                rtc_time_set_flag = 1;
              }
              else
              {
                // обработка ошибки FatFs
              }
            }
          }
          else
          {
            printf("fs_check_fat32 ERROR\r\n");
            f_mount(NULL, "", 0);
            HAL_SD_DeInit(&hsd1);
            MX_FATFS_DeInit();
          }
        }
        break;
      case SD_DEINIT:
        if (sd_card_init_flag)
        {
          f_mount(NULL, "", 0);
          HAL_SD_DeInit(&hsd1);
          MX_FATFS_DeInit();
          sd_card_init_flag = 0;
          sd_card_led_off();
          printf("sd_card_init_flag = %d\r\n", sd_card_init_flag);
        }
        break;
      case SD_CREATE_WAV_FILE: break;
      case SD_WRITE_WAV_DATA:
        if (sd_card_init_flag)
        {
          wav_file_create();
          if (wav_file_write_data("Music.wav", sd_msg.ptr, SD_WRITE_BUF_SIZE / 2) != FR_OK)
          {
            printf("wav_file_write_data error\r\n");
          }
          else
          {
            record.cnt++;
            if (record.cnt == RECORD_BLOCK_COUNT)
            {
              if (wav_update_header() != FR_OK)
              {
                printf("wav_update_header error: %d\r\n", (int)file_index_get());
              }
              else
              {
                printf("wav_update_header ok: %d\r\n", (int)file_index_get());
              }
              if (wav_file_close() != FR_OK)
              {
                printf("wav_file_close error %d\r\n", (int)file_index_get());
              }
              else
              {
                printf("wav_file_close ok %d\r\n", (int)file_index_get());
              }
              record.cnt = 0;
            }
          }
        }
        break;
      case SD_FILE_WAV_CLOSE:
        if (sd_card_init_flag)
        {
          if (wav_update_header() != FR_OK)
          {
            printf("wav_update_header error: %d\r\n", (int)file_index_get());
          }
          else
          {
            printf("wav_update_header ok: %d\r\n", (int)file_index_get());
          }
          if (wav_file_close() != FR_OK)
          {
            printf("wav_file_close error %d\r\n", (int)file_index_get());
          }
          else
          {
            printf("wav_file_close ok %d\r\n", (int)file_index_get());
          }
          sd_state.active_buf = 0;
          sd_state.write_offset = 0;
          record.cnt = 0;
        }
        break;
      default: break;
      }
    }
    task_alive[TASK_ID_1] = 1;
  }
  /* USER CODE END SdCard_Task */
}

/* USER CODE BEGIN Header_AudioMixing_Task */
/**
 * @brief Function implementing the AudioMixingTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_AudioMixing_Task */
void AudioMixing_Task(void *argument)
{
  /* USER CODE BEGIN AudioMixing_Task */
  osStatus_t statusEth;
  osStatus_t statusRxVocoder;
  osStatus_t statusTxVocoder;
  audio_msg_t vocoder_tx_msg;
  audio_msg_t vocoder_rx_msg;
  audio_msg_t eth_to_audio_mix_msg;
  sd_msg_t sd_msg;
  //  uint8_t *u8_ptr;
  //  audio_msg_t audio_msg;
  //  int16_t *i16_ptr;
  /* Infinite loop */

  for (;;)
  {
    statusTxVocoder = osMessageQueueGet(TxVocoderToAudioMixQueueHandle, &vocoder_tx_msg, NULL, 100);
    if (statusTxVocoder == osOK)
    {
      statusEth = osMessageQueueGet(EthToAudioMixQueueHandle, &eth_to_audio_mix_msg, NULL, 0);

      // Write sd card
      if (dev_addr_get() == DEV_ADDR_A)
      {
        statusRxVocoder = osMessageQueueGet(RxVocoderToAudioMixQueueHandle, &vocoder_rx_msg, NULL, 0);
        if (statusRxVocoder == osOK)
        {
          // mixing of receive and transmit traffic if there is reception and write buffer sd_buf
          if (statusEth == osOK)
          {
            multisamples_mixing(vocoder_tx_msg.ptr, vocoder_rx_msg.ptr, (int16_t *)(&sd_buf[sd_state.active_buf][sd_state.write_offset]),
                                AUDIO_SAMPLE_SIZE);
          }
          // SD buffer write buffer transfer only
          else
          {
            memcpy(&sd_buf[sd_state.active_buf][sd_state.write_offset], vocoder_rx_msg.ptr, AUDIO_SAMPLE_SIZE * sizeof(int16_t));
          }

          sd_state.write_offset += AUDIO_SAMPLE_SIZE * sizeof(int16_t);
          if (sd_state.write_offset >= SD_WRITE_BUF_SIZE)
          {
            sd_msg.cmd = SD_WRITE_WAV_DATA;
            sd_msg.ptr = (int16_t *)sd_buf[sd_state.active_buf];
            osMessageQueuePut(AudioWriteSdQueueHandle, &sd_msg, 0U, 0U);
            sd_state.active_buf ^= 1; /* переключение буфера */
            sd_state.write_offset = 0;
          }
        }
      }

      // if there is Ethernet traffic
      if (statusEth == osOK)
      {
        // check alarm
        if (flag_alarm_button_get() == ALARM_FLAG_STATE_ON)
        {
          copy_int16_array_circular(&pcm_buf_alarm, vocoder_tx_msg.ptr, AUDIO_SAMPLE_SIZE);
          multisamples_mixing(vocoder_tx_msg.ptr, eth_to_audio_mix_msg.ptr, vocoder_tx_msg.ptr, AUDIO_SAMPLE_SIZE);
        }
        // check call
        else if (flag_call_get() == CALL_FLAG_STATE_ON)
        {
          copy_int16_array_circular(&pcm_buf_call, vocoder_tx_msg.ptr, AUDIO_SAMPLE_SIZE);
          multisamples_mixing(vocoder_tx_msg.ptr, eth_to_audio_mix_msg.ptr, vocoder_tx_msg.ptr, AUDIO_SAMPLE_SIZE);
        }
        // voice
        else
        {
          memcpy(vocoder_tx_msg.ptr, eth_to_audio_mix_msg.ptr, AUDIO_SAMPLE_SIZE * sizeof(int16_t));
        }
        heap_pool_free(&heap_pool_audio_eth_to_vocoder, (uint8_t *)eth_to_audio_mix_msg.ptr);
      }
      else
      {
        // check alarm
        if (flag_alarm_button_get() == ALARM_FLAG_STATE_ON)
        {
          copy_int16_array_circular(&pcm_buf_alarm, vocoder_tx_msg.ptr, AUDIO_SAMPLE_SIZE);
        }
        // check call
        else if (flag_call_get() == CALL_FLAG_STATE_ON)
        {
          copy_int16_array_circular(&pcm_buf_call, vocoder_tx_msg.ptr, AUDIO_SAMPLE_SIZE);
        }
        // noise
        else
        {
          memset(vocoder_tx_msg.ptr, 0, AUDIO_SAMPLE_SIZE * sizeof(int16_t));
        }
      }
    }
    task_alive[TASK_ID_2] = 1;
  }
  /* USER CODE END AudioMixing_Task */
}

/* USER CODE BEGIN Header_ProcessIO_Task */
/**
 * @brief Function implementing the ProcessIOTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ProcessIO_Task */
void ProcessIO_Task(void *argument)
{
  /* USER CODE BEGIN ProcessIO_Task */
  osStatus_t status;
  w5500_msg_t w5500_msg;
  io_msg_t io_msg;
  sd_msg_t sd_msg;

  sensor_1_led_off();
  sensor_2_led_off();
  sensor_3_led_off();
  sensor_4_led_off();
  sensor_5_led_off();
  group_led_off();
  read_state_sensors(&field_local);
  process_field_sensor(&field_local, &field_dest);

  if (ptt_state_get() == PTT_ACTIVE)
  {
    vocoder_enable(); // vocoder disable
    hp_on();          // handset speaker amplifier
    mic_on();         // off the microphone amplifier
  }

  /* Infinite loop */
  for (;;)
  {
    status = osMessageQueueGet(IOQueueHandle, &io_msg, NULL, 100);
    if (status == osOK)
    {
      switch (io_msg.cmd)
      {
      case IO_CMD_LOW_BATTERY: break;
      case IO_CMD_SD_CD:
        osDelay(20);
        if (sd_cd_state_get() == SD_CD_LOW)
        {
          sd_msg.cmd = SD_INIT;
          osMessageQueuePut(AudioWriteSdQueueHandle, &sd_msg, 0U, 0U);
        }
        else
        {
          sd_msg.cmd = SD_DEINIT;
          osMessageQueuePut(AudioWriteSdQueueHandle, &sd_msg, 0U, 0U);
        }
        HAL_NVIC_EnableIRQ(EXTI2_IRQn);
        break;
      case IO_CMD_PTT:
        osDelay(20);
        if (ptt_state_get() == PTT_INACTIVE)
        {
          if (flag_alarm_button_get() == ALARM_FLAG_STATE_OFF)
          {
            vocoder_disable(); // vocoder enable
          }
          hp_off();  // off the handset speaker amplifier
          mic_off(); // off the microphone amplifier
          sd_msg.cmd = SD_FILE_WAV_CLOSE;
          osMessageQueuePut(AudioWriteSdQueueHandle, &sd_msg, 0U, 0U);
        }
        else
        {
          vocoder_enable(); // vocoder disable
          hp_on();          // handset speaker amplifier
          mic_on();         // off the microphone amplifier
          flag_call_clear();
          timer_call_stop(); // Timer stop
          call_led_off();    // Led off
        }

        HAL_NVIC_EnableIRQ(EXTI4_IRQn);
        break;
      case IO_CMD_CALL_LOCAL:
        w5500_msg.w5500_cmd = W5500_CMD_TRANSMIT_FIELD;
        w5500_msg.io_cmd = IO_CMD_CALL_DEST;
        osDelay(20);
        if (HAL_GPIO_ReadPin(CALL_BTN_GPIO_Port, CALL_BTN_Pin) == GPIO_PIN_RESET)
        {
          CLEAR_BIT(w5500_msg.field, (1 << FIELD_CALL));
        }
        else
        {
          SET_BIT(w5500_msg.field, (1 << FIELD_CALL));
        }
        osMessageQueuePut(W5500QueueHandle, &w5500_msg, 0U, 0U);
        break;
      case IO_CMD_CALL_DEST:
        field_dest.state = copy_bits(field_dest.state, io_msg.field, MASK_0_BIT_CALL);
        process_field_call(&field_dest);
        break;
      case IO_CMD_ALARM_LOCAL:
        osDelay(20);
        if (HAL_GPIO_ReadPin(ALARM_BTN_GPIO_Port, ALARM_BTN_Pin) == GPIO_PIN_RESET)
        {
          w5500_msg.w5500_cmd = W5500_CMD_TRANSMIT_FIELD;
          w5500_msg.io_cmd = IO_CMD_ALARM_DEST;
          CLEAR_BIT(w5500_msg.field, (1 << FIELD_ALARM));
          osMessageQueuePut(W5500QueueHandle, &w5500_msg, 0U, 0U);
        }
        break;
      case IO_CMD_ALARM_DEST:
        field_dest.state = copy_bits(field_dest.state, io_msg.field, MASK_1_BIT_ALARM);
        process_field_alarm_button(&field_dest);
        break;
      case IO_CMD_RESET_ALARM:
        flag_alarm_sensor_clear();
        process_field_reset_alarm(&field_local, &field_dest);
        break;
      case IO_CMD_SENSOR_LOCAL:
        read_state_sensors(&field_local);
        process_field_sensor(&field_local, &field_dest);
        w5500_msg.w5500_cmd = W5500_CMD_TRANSMIT_FIELD;
        w5500_msg.io_cmd = IO_CMD_SENSOR_DEST;
        w5500_msg.field = field_local.state;
        osMessageQueuePut(W5500QueueHandle, &w5500_msg, 0U, 0U);
        break;
      case IO_CMD_SENSOR_DEST:
        field_dest.state = copy_bits(field_dest.state, io_msg.field, MASK_2_6_BITS_SENSORS);
        read_state_sensors(&field_local);
        process_field_sensor(&field_local, &field_dest);
        break;
      default: break;
      }
    }
    task_alive[TASK_ID_3] = 1;
  }
  /* USER CODE END ProcessIO_Task */
}

/* USER CODE BEGIN Header_Watchdog_Task */
/**
 * @brief Function implementing the WatchdogTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Watchdog_Task */
void Watchdog_Task(void *argument)
{
  /* USER CODE BEGIN Watchdog_Task */
  /* Infinite loop */
  while (!all_tasks_alive())
  {
    osDelay(100);
  }
  MX_IWDG_Init();

  for (;;)
  {
    if (all_tasks_alive())
    {
      HAL_IWDG_Refresh(&hiwdg);
    }
    osDelay(100);
  }
  /* USER CODE END Watchdog_Task */
}

/* CallLedCallback function */
void CallLedCallback(void *argument)
{
  /* USER CODE BEGIN CallLedCallback */
  HAL_GPIO_TogglePin(LED_CALL_GPIO_Port, LED_CALL_Pin);
  /* USER CODE END CallLedCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  io_msg_t io_msg;

  switch (GPIO_Pin)
  {
  case W5500_IRQ_Pin:
    w5500_msg_t w5500_msg = {.w5500_cmd = W5500_CMD_RECEIVE};
    osMessageQueuePut(W5500QueueHandle, &w5500_msg, 0U, 0U);
    break;
  case LOW_BATTERY_Pin:
    io_msg.cmd = IO_CMD_LOW_BATTERY;
    osMessageQueuePut(IOQueueHandle, &io_msg, 0U, 0U);
    break;
  case SD_CD_Pin:
    HAL_NVIC_DisableIRQ(EXTI2_IRQn);
    io_msg.cmd = IO_CMD_SD_CD;
    osMessageQueuePut(IOQueueHandle, &io_msg, 0U, 0U);
    break;
  case PTT_Pin:
    HAL_NVIC_DisableIRQ(EXTI4_IRQn);
    io_msg.cmd = IO_CMD_PTT;
    osMessageQueuePut(IOQueueHandle, &io_msg, 0U, 0U);
    break;
  case CALL_BTN_Pin:
    io_msg.cmd = IO_CMD_CALL_LOCAL;
    osMessageQueuePut(IOQueueHandle, &io_msg, 0U, 0U);
    break;
  case ALARM_BTN_Pin:
    io_msg.cmd = IO_CMD_ALARM_LOCAL;
    osMessageQueuePut(IOQueueHandle, &io_msg, 0U, 0U);
    break;
  case RESET_ALARM_BTN_Pin:
    io_msg.cmd = IO_CMD_RESET_ALARM;
    osMessageQueuePut(IOQueueHandle, &io_msg, 0U, 0U);
    break;
  case SENSOR_1_Pin:
    io_msg.cmd = IO_CMD_SENSOR_LOCAL;
    osMessageQueuePut(IOQueueHandle, &io_msg, 0U, 0U);
    break;
  case SENSOR_2_Pin:
    io_msg.cmd = IO_CMD_SENSOR_LOCAL;
    osMessageQueuePut(IOQueueHandle, &io_msg, 0U, 0U);
    break;
  case SENSOR_3_Pin:
    io_msg.cmd = IO_CMD_SENSOR_LOCAL;
    osMessageQueuePut(IOQueueHandle, &io_msg, 0U, 0U);
    break;
  case SENSOR_4_Pin:
    io_msg.cmd = IO_CMD_SENSOR_LOCAL;
    osMessageQueuePut(IOQueueHandle, &io_msg, 0U, 0U);
    break;
  case SENSOR_5_Pin:
    io_msg.cmd = IO_CMD_SENSOR_LOCAL;
    osMessageQueuePut(IOQueueHandle, &io_msg, 0U, 0U);
    break;
  default: break;
  }
}

/**
 * @brief  Callback: передача половины TX-буфера завершена
 * @param  hsai : указатель на SAI handle
 */
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  audio_msg_t audio_msg = {.ptr = &sai_a_buf[0]};
  osMessageQueuePut(TxVocoderToAudioMixQueueHandle, &audio_msg, 0U, 0U);
}

/**
 * @brief  Callback: передача полного TX-буфера завершена
 * @param  hsai : указатель на SAI handle
 */
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
  audio_msg_t audio_msg = {.ptr = &sai_a_buf[AUDIO_SAMPLE_SIZE]};
  osMessageQueuePut(TxVocoderToAudioMixQueueHandle, &audio_msg, 0U, 0U);
}

/**
 * @brief  Callback: приём половины RX-буфера завершён
 * @param  hsai : указатель на SAI handle
 */
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  if (ptt_state_get() == PTT_ACTIVE)
  {
    w5500_msg_t w5500_msg = {.w5500_cmd = W5500_CMD_TRANSMIT_SAMPLE, .data_ptr = (uint8_t *)sai_b_buf};
    osMessageQueuePut(W5500QueueHandle, &w5500_msg, 0U, 0U);
    if (dev_addr_get() == DEV_ADDR_A) // DEV_ADDR_A
    {
      audio_msg_t audio_msg = {.ptr = sai_b_buf};
      osMessageQueuePut(RxVocoderToAudioMixQueueHandle, &audio_msg, 0U, 0U);
    }
  }
}

/**
 * @brief  Callback: приём полного RX-буфера завершён
 * @param  hsai : указатель на SAI handle
 */
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
  if (ptt_state_get() == PTT_ACTIVE)
  {
    w5500_msg_t msg = {.w5500_cmd = W5500_CMD_TRANSMIT_SAMPLE, .data_ptr = (uint8_t *)(sai_b_buf + AUDIO_SAMPLE_SIZE)};
    osMessageQueuePut(W5500QueueHandle, &msg, 0U, 0U);
    if (dev_addr_get() == DEV_ADDR_A) // DEV_ADDR_A
    {
      audio_msg_t audio_msg = {.ptr = &sai_b_buf[AUDIO_SAMPLE_SIZE]};
      osMessageQueuePut(RxVocoderToAudioMixQueueHandle, &audio_msg, 0U, 0U);
    }
  }
}

void device_delay_ms(uint32_t ms)
{
  osDelay(ms);
}

void ethernet_delay(uint32_t ms)
{
  osDelay(ms);
}

void timer_call_start(void)
{
  osTimerStart(CallLedTimerHandle, 200);
}

void timer_call_stop(void)
{
  osTimerStop(CallLedTimerHandle);
}

static uint8_t all_tasks_alive(void)
{
  for (int i = 0; i < NUM_TASKS; i++)
  {
    if (task_alive[i] == 0)
    {
      return 0; // хотя бы одна задача не обновила heartbeat
    }
  }

  // Все задачи живы → обнуляем флаги для следующего интервала
  for (int i = 0; i < NUM_TASKS; i++)
  {
    task_alive[i] = 0;
  }

  return 1; // все задачи живы
}

/* USER CODE END Application */

