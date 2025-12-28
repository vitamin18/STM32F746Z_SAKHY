/*******************************************************************************
 * Company:     "WSE" LLP
 * Project:     STM32F746Z_FATFS
 * File:        fatfs_app.h
 * Author:      Kostyrev Vitaliy
 * Position:    Embedded Systems Engineer
 *******************************************************************************
 * Version:    V1.0
 * Revision:   R1.0.0
 * *****************************************************************************
 * Description:
 *
 *******************************************************************************
 * Created on:  Sep 19, 2025
 ******************************************************************************/

#ifndef INC_FATFS_APP_H_
#define INC_FATFS_APP_H_

#ifdef __cplusplus
extern "C"
{
#endif

  /*****************************************************************************/
  /******************************* INCLUDES ************************************/
#include "diskio.h"
#include "fatfs.h"
#include "ff.h"
#include "main.h"
#include "string.h"
  /*****************************************************************************/
  /******************************* DEFINES *************************************/

  /*****************************************************************************/
  /******************************* VARIABLES ***********************************/
  typedef struct __attribute__((packed))
  {
    char RIFF[4];           // "RIFF"
    uint32_t ChunkSize;     // 36 + Subchunk2Size
    char WAVE[4];           // "WAVE"
    char fmt[4];            // "fmt "
    uint32_t Subchunk1Size; // 16 for PCM
    uint16_t AudioFormat;   // 1 = PCM
    uint16_t NumChannels;   // 1 = mono
    uint32_t SampleRate;    // e.g. 8000
    uint32_t ByteRate;      // SampleRate * NumChannels * BitsPerSample/8
    uint16_t BlockAlign;    // NumChannels * BitsPerSample/8
    uint16_t BitsPerSample; // 16
    char Subchunk2ID[4];    // "data"
    uint32_t Subchunk2Size; // numSamples * NumChannels * BitsPerSample/8
  } wav_header_t;
  /*****************************************************************************/
  /******************************* METHODS *************************************/
  FRESULT fs_check_fat32(void);
  FRESULT SD_FatFs_Test(void);
  FRESULT wav_write_size(const char *filename, uint32_t data_size);
  FRESULT wav_update_header(void);
  FRESULT wav_file_create(void);
  FRESULT wav_file_write_data(const char *filename, int16_t *pcm_data, uint32_t num_samples);
  FRESULT wav_file_close(void);
  FRESULT get_next_file_index(void);
  uint32_t file_index_get(void);
  FRESULT read_datetime_from_index_dat(RTC_TimeTypeDef *time, RTC_DateTypeDef *date);
  void file_index_set(uint32_t index);
  uint32_t fatfs32_count_wav_files(void);
  void wav_file_count_set(uint32_t count);
  uint32_t wav_file_count_get(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_FATFS_APP_H_ */
/*****************************************************************************/
