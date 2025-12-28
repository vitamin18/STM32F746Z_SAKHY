/*******************************************************************************
 * Company:     "WSE" LLP
 * Project:     STM32F746Z_FATFS
 * File:        fatfs_app.c
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

/******************************* INCLUDES ************************************/
#include "fatfs_app.h"
#include "rtc.h"
#include <stdint.h>
#include <stdio.h>

/*****************************************************************************/
/******************************* DEFINES *************************************/
#define FF_MAX_SS 4096
#define INDEX_FILE "INDEX.dat"

#define WAV_HEADER_SIZE (44U)
#define BYTES_PER_SECOND (16000U) /* 16 bit, mono, 8 kHz */
#define DURATION_SECONDS (180U)

#define MAX_FILES_USER (99999U)
#define MAX_FILES_FAT32 (65534U)

#define RAW_FILE_SIZE (WAV_HEADER_SIZE + (BYTES_PER_SECOND * DURATION_SECONDS))
/*****************************************************************************/
/******************************* VARIABLES ***********************************/
static FATFS fs;                ///< File system object
static BYTE workbuf[FF_MAX_SS]; ///< Working buffer for f_mkfs (sector size)
static FIL file_wav;            ///< File object
static uint32_t file_index = 0;
static uint8_t wav_file_is_open = 0;
static uint32_t g_wav_file_count = 0;

/*****************************************************************************/
/******************************* PROTOTYPES **********************************/
static void create_wav_filename(char *buffer);
/*****************************************************************************/
/******************************* METHODS *************************************/
/**
 * @brief  Ensures the SD card has a FAT32 filesystem and mounts it.
 *
 * This function performs:
 * - Attempts to mount the default logical drive.
 * - If a filesystem exists, checks its type.
 * - If the filesystem is not FAT32 (e.g., FAT12/16) or missing,
 *   formats the card as FAT32 using f_mkfs().
 * - Remounts the filesystem after formatting.
 *
 * @note   Requires FF_FS_FAT32 = 1 and FF_USE_MKFS = 1 in ffconf.h
 * @note   Uses the old-style f_mkfs API:
 *         FRESULT f_mkfs(const TCHAR* path, BYTE opt, DWORD au, void* work, UINT len);
 * @note   Assumes SD is already initialized with HAL_SD_Init() or BSP_SD_Init().
 *
 * @retval FR_OK            FAT32 successfully mounted.
 * @retval FR_NO_FILESYSTEM No filesystem and formatting failed.
 * @retval FR_DISK_ERR      Disk I/O error.
 * @retval FR_NOT_READY     SD card not ready.
 * @retval Other FatFs error codes.
 */
FRESULT fs_check_fat32(void)
{
  FRESULT res;
  FATFS *pfs;
  DWORD free_clusters;

  // 1) Try to mount the filesystem
  res = f_mount(&fs, "", 1);
  if (res == FR_OK)
  {
    // Query filesystem info
    res = f_getfree("", &free_clusters, &pfs);
    if (res != FR_OK) return res;

    // If already FAT32 -> OK
    if (pfs->fs_type == FS_FAT32) return FR_OK;

    // Otherwise: format to FAT32
  }
  else if (res != FR_NO_FILESYSTEM)
  {
    // Other error (disk not ready, etc.)
    return res;
  }

  // 2) Format the card as FAT32
  res = f_mkfs("", FM_FAT32, 0, workbuf, sizeof(workbuf));
  if (res != FR_OK) return res;

  // 3) Remount after formatting
  res = f_mount(&fs, "", 1);
  return res;
}

FRESULT wav_write_header(FIL *file, uint32_t data_size)
{
  wav_header_t header;
  const uint32_t sample_rate = 8000;
  const uint16_t num_channels = 1;
  const uint16_t bits_per_sample = 16;

  memcpy(header.RIFF, "RIFF", 4);
  header.ChunkSize = 36 + data_size;
  memcpy(header.WAVE, "WAVE", 4);
  memcpy(header.fmt, "fmt ", 4);
  header.Subchunk1Size = 16;
  header.AudioFormat = 1;
  header.NumChannels = num_channels;
  header.SampleRate = sample_rate;
  header.BitsPerSample = bits_per_sample;
  header.ByteRate = sample_rate * num_channels * bits_per_sample / 8;
  header.BlockAlign = num_channels * bits_per_sample / 8;
  memcpy(header.Subchunk2ID, "data", 4);
  header.Subchunk2Size = data_size;

  UINT bw;

  return f_write(file, &header, sizeof(wav_header_t), &bw);
}

FRESULT wav_update_header(void)
{
  FRESULT res;
  wav_header_t header;
  UINT bw;

  if (!wav_file_is_open)
  {
    return FR_INVALID_OBJECT; // файл не открыт
  }

  /* Вычисляем размер данных */
  uint32_t data_size = f_size(&file_wav) - sizeof(wav_header_t);

  /* Заполняем заголовок */
  memcpy(header.RIFF, "RIFF", 4);
  header.ChunkSize = 36 + data_size;

  memcpy(header.WAVE, "WAVE", 4);

  memcpy(header.fmt, "fmt ", 4);
  header.Subchunk1Size = 16;
  header.AudioFormat = 1; /* PCM */
  header.NumChannels = 1;
  header.SampleRate = 8000;
  header.BitsPerSample = 16;

  header.ByteRate = header.SampleRate * header.NumChannels * header.BitsPerSample / 8;

  header.BlockAlign = header.NumChannels * header.BitsPerSample / 8;

  memcpy(header.Subchunk2ID, "data", 4);
  header.Subchunk2Size = data_size;

  /* Переходим в начало файла */
  res = f_lseek(&file_wav, 0);
  if (res != FR_OK) return res;

  /* Записываем заголовок */
  res = f_write(&file_wav, &header, sizeof(wav_header_t), &bw);
  if (res != FR_OK) return res;

  if (bw != sizeof(wav_header_t)) return FR_INT_ERR;

  return FR_OK;
}

FRESULT wav_file_create(void)
{
  //  FIL file;
  FRESULT res;
  char filename[40];

  if (wav_file_is_open)
  {
    return FR_OK; // файл уже создан и открыт
  }

  create_wav_filename(filename);

  // Open file
  res = f_open(&file_wav, filename, FA_CREATE_ALWAYS | FA_WRITE);
  if (res != FR_OK)
  {
    return res;
  }
  else
  {
    wav_file_is_open = 1;
  }

  // Write placeholder header
  res = wav_write_header(&file_wav, 0);

  return res;
}

FRESULT wav_file_write_data(const char *filename, int16_t *pcm_data, uint32_t num_samples)
{
  UINT bw;
  FRESULT res;

  if (!wav_file_is_open)
  {
    return FR_INVALID_OBJECT; // файл не открыт
  }

  // Write PCM data
  res = f_write(&file_wav, pcm_data, num_samples * sizeof(int16_t), &bw);

  return res;
}

FRESULT wav_file_close(void)
{
  if (!wav_file_is_open)
  {
    return FR_INVALID_OBJECT; // файл не открыт
  }
  wav_file_is_open = 0;
  return f_close(&file_wav);
}

/**
 * @brief  Test SD card and FatFs functionality.
 *
 * This function performs:
 * - Mounts the filesystem on SD.
 * - Creates and opens a test file.
 * - Writes a known string to the file.
 * - Closes and reopens the file.
 * - Reads the string back and compares.
 *
 * @note   Assumes SD is already initialized with HAL_SD_Init() or BSP_SD_Init().
 * @note   Requires FatFs with FF_FS_READONLY = 0 (write support).
 *
 * @retval FR_OK            SD card and FatFs work correctly.
 * @retval FRESULT error    FatFs error code on failure.
 */
FRESULT SD_FatFs_Test(void)
{
  FRESULT res;
  UINT bw, br;
  FIL file;
  char wtext[] = "SD/FatFs test OK\r\n";
  char rtext[32];

  // 1. Mount filesystem
  res = f_mount(&fs, "0", 1);
  if (res != FR_OK) return res;

  // 2. Create test file
  res = f_open(&file, "test.txt", FA_CREATE_ALWAYS | FA_WRITE);
  if (res != FR_OK) return res;

  // 3. Write test string
  res = f_write(&file, wtext, strlen(wtext), &bw);
  f_close(&file);
  if (res != FR_OK || bw != strlen(wtext)) return res;

  // 4. Reopen file for read
  res = f_open(&file, "test.txt", FA_READ);
  if (res != FR_OK) return res;

  // 5. Read back
  memset(rtext, 0, sizeof(rtext));
  res = f_read(&file, rtext, sizeof(rtext) - 1, &br);
  f_close(&file);
  if (res != FR_OK) return res;

  // 6. Compare
  if (strcmp(wtext, rtext) != 0) return FR_INT_ERR; // mismatch

  return FR_OK;
}

/**
 * @brief  Generate fixed-length filename for WAV recording.
 *         Format: "File XXXXXX.ss-mm-hh.DD-MM-YY.wav"
 *         Always 33 characters + null terminator.
 * @param  buffer     Pointer to string buffer (must be >= 34 bytes)
 * @param  file_index Six-digit sequential file number
 * @retval None
 */
static void create_wav_filename(char *buffer)
{
  if (buffer == NULL) return;

  // Получаем следующий индекс файла
  if (get_next_file_index() != FR_OK)
  {
    snprintf(buffer, 40, "%06d.wav", 0); // на случай ошибки
    return;
  }

  // Формируем имя файла в формате "xxxxxx.wav"
  snprintf(buffer, 40, "%06lu.wav", file_index);
}

FRESULT get_next_file_index(void)
{
  FRESULT res;
  FIL file;
  UINT br, bw;
  uint32_t index = 0;

  /* Пытаемся открыть существующий файл индекса */
  res = f_open(&file, INDEX_FILE, FA_READ | FA_WRITE);
  if (res == FR_OK)
  {
    /* Читаем текущий индекс */
    res = f_read(&file, &index, sizeof(index), &br);
    if (res != FR_OK || br != sizeof(index))
    {
      f_close(&file);
      return res != FR_OK ? res : FR_INT_ERR;
    }

    /* Увеличиваем индекс по кольцу */
    index++;
    if (index > wav_file_count_get()) index = 0;

    /* Перезаписываем файл с новым индексом */
    f_lseek(&file, 0);
  }
  else if (res == FR_NO_FILE)
  {
    /* Файл индекса не существует — создаём новый */
    index = 0;
    res = f_open(&file, INDEX_FILE, FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK) return res;
  }
  else
  {
    /* Другая ошибка FatFs */
    return res;
  }

  /* Записываем обновлённый индекс */
  res = f_write(&file, &index, sizeof(index), &bw);
  f_close(&file);
  if (res != FR_OK || bw != sizeof(index)) return res != FR_OK ? res : FR_INT_ERR;

  /* Сохраняем индекс для использования */
  file_index = index;

  return FR_OK;
}

FRESULT read_datetime_from_index_dat(RTC_TimeTypeDef *time, RTC_DateTypeDef *date)
{
  FILINFO fno;
  FRESULT res;

  if (time == NULL || date == NULL) return FR_INVALID_PARAMETER;

  res = f_stat(INDEX_FILE, &fno);
  if (res != FR_OK) return res;

  /* Декодирование FAT времени */
  uint16_t ftime = fno.ftime;
  uint16_t fdate = fno.fdate;

  time->Hours = (ftime >> 11) & 0x1F;
  time->Minutes = (ftime >> 5) & 0x3F;
  time->Seconds = (ftime & 0x1F) * 2;

  date->Year = (uint8_t)(((fdate >> 9) & 0x7F) + 1980 - 2000);
  date->Month = (fdate >> 5) & 0x0F;
  date->Date = fdate & 0x1F;

  /* Добавление 5 минут */
  time->Minutes += 5;

  /* Перенос минут в часы */
  if (time->Minutes >= 60)
  {
    time->Hours += time->Minutes / 60;
    time->Minutes %= 60;
  }

  /* Перенос часов в дни */
  if (time->Hours >= 24)
  {
    time->Hours %= 24;
    date->Date += 1;

    /* Количество дней в каждом месяце */
    uint8_t days_in_month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

    /* Учёт високосного года для февраля */
    uint16_t full_year = date->Year + 2000;
    if ((date->Month == 2) && ((full_year % 4 == 0 && full_year % 100 != 0) || (full_year % 400 == 0)))
    {
      days_in_month[1] = 29;
    }

    /* Перенос на следующий месяц, если нужно */
    if (date->Date > days_in_month[date->Month - 1])
    {
      date->Date = 1;
      date->Month += 1;

      /* Перенос на следующий год, если месяц > 12 */
      if (date->Month > 12)
      {
        date->Month = 1;
        date->Year += 1;
      }
    }
  }

  return FR_OK;
}

uint32_t file_index_get(void)
{
  return file_index;
}

void file_index_set(uint32_t index)
{
  file_index = index;
}

uint32_t fatfs32_count_wav_files(void)
{
  FATFS *fs;
  DWORD free_clusters;
  FRESULT res;

  /* Получение информации о свободном месте */
  res = f_getfree("", &free_clusters, &fs);
  if (res != FR_OK) return 0;

  /* Размер кластера в байтах */
  uint32_t cluster_size = fs->csize * 512;

  /* Округление размера файла до кластера */
  uint32_t file_size_rounded = ((RAW_FILE_SIZE + cluster_size - 1) / cluster_size) * cluster_size;

  /* Свободное место */
  uint64_t free_bytes = (uint64_t)free_clusters * cluster_size;

  /* Расчёт по свободному месту */
  uint64_t count = free_bytes / file_size_rounded;

  /* Ограничение FAT32 каталога */
  if (count > MAX_FILES_FAT32) count = MAX_FILES_FAT32;

  return (uint32_t)count;
}

void wav_file_count_set(uint32_t count)
{
    g_wav_file_count = count;
}

uint32_t wav_file_count_get(void)
{
    return g_wav_file_count;
}

  /*****************************************************************************/
