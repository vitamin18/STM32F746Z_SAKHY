/*******************************************************************************
 * Project:     STM32F746Z_SAKHY
 * File:        audio_mixing.h
 *******************************************************************************
 * Version:    V1.0
 * Revision:   R1.0.0
 * *****************************************************************************
 * Description:
 *
 *******************************************************************************
 * Created on:  May 1, 2025
 ******************************************************************************/

/******************************* INCLUDES ************************************/
#include "audio_mixing.h"
#include <string.h>

/*****************************************************************************/
/******************************* DEFINES *************************************/
#define PCM_OFFSET 0xFFF
#define PCM_MAX_VALUE 0x1FFF
/*****************************************************************************/
/******************************* VARIABLES ***********************************/

/*****************************************************************************/
/******************************* METHODS *************************************/
/**
 * @brief Mix two PCM samples with non-linear scaling.
 *
 * This function mixes two signed 16-bit PCM samples using a non-linear
 * algorithm that preserves dynamic range and minimizes clipping distortion.
 *
 * The algorithm works by first converting signed samples to an unsigned range
 * (0–65535) using PCM_OFFSET. Then, depending on whether both samples are above
 * or below the midpoint, it applies one of two different mixing formulas.
 * The result is then converted back to signed form.
 *
 * @param a First PCM sample (signed 16-bit)
 * @param b Second PCM sample (signed 16-bit)
 * @return Mixed PCM sample (signed 16-bit)
 */
int16_t mix_pcm_samples(int16_t a, int16_t b)
{
  int16_t res;

  /* Shift samples into unsigned range */
  a += PCM_OFFSET;
  b += PCM_OFFSET;

  if ((a < PCM_OFFSET) || (b < PCM_OFFSET))
  {
    /* Case: both samples below midpoint */
    res = (int16_t)((a * b) / PCM_OFFSET);
  }
  else
  {
    /* Case: both samples above midpoint */
    res = (int16_t)(2 * (a + b) - (a * b) / PCM_OFFSET - PCM_MAX_VALUE);
  }

  /* Prevent overflow beyond PCM_MAX_VALUE */
  if (res > PCM_MAX_VALUE)
  {
    res = PCM_MAX_VALUE;
  }

  /* Convert back to signed range */
  return (int16_t)(res - PCM_OFFSET);
}

/**
 * @brief Mix two PCM buffers into one output buffer.
 *
 * This function mixes two input PCM buffers, each containing a specified number
 * of samples, and stores the mixed output in a third buffer.
 * The mixing is performed sample-by-sample using the function mix_pcm_samples().
 *
 * @param[in]  buf_in_a       Pointer to the first input buffer (signed 16-bit PCM).
 * @param[in]  buf_in_b       Pointer to the second input buffer (signed 16-bit PCM).
 * @param[out] buf_out        Pointer to the output buffer (signed 16-bit PCM).
 * @param[in]  sample_length  Number of PCM samples to process.
 *
 * @note All buffers must contain at least @p sample_length samples.
 * @note The output buffer may overlap with one of the inputs.
 */
void multisamples_mixing(const int16_t *buf_in_a, const int16_t *buf_in_b, int16_t *buf_out, int16_t sample_length)
{
  for (uint16_t i = 0; i < sample_length; i++)
  {
    buf_out[i] = mix_pcm_samples(buf_in_a[i], buf_in_b[i]);
  }
}

/*****************************************************************************/
