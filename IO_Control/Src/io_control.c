/*******************************************************************************
 * Project:     STM32F746Z_SAKHY
 * File:        io_control.c
 *******************************************************************************
 * Version:    V1.0
 * Revision:   R1.0.0
 * *****************************************************************************
 * Description:
 *
 *******************************************************************************
 * Created on:  Sep 30, 2025
 ******************************************************************************/

/******************************* INCLUDES ************************************/
#include "io_control.h"

/*****************************************************************************/
/******************************* DEFINES *************************************/
/*****************************************************************************/
/******************************* VARIABLES ***********************************/
/* Static variable for flags storage */
static uint8_t flags = 0;
/*****************************************************************************/
/******************************* PROTOTYPES **********************************/

/*****************************************************************************/
/******************************* METHODS *************************************/
/**
 * @brief  Enable vocoder power/control line.
 * @note   Sets the PCM_PUI pin to logical high level.
 * @retval None
 */
void vocoder_enable(void)
{
  HAL_GPIO_WritePin(PCM_PUI_GPIO_Port, PCM_PUI_Pin, GPIO_PIN_SET);
}

/**
 * @brief  Disable vocoder power/control line.
 * @note   Sets the PCM_PUI pin to logical low level.
 * @retval None
 */
void vocoder_disable(void)
{
  HAL_GPIO_WritePin(PCM_PUI_GPIO_Port, PCM_PUI_Pin, GPIO_PIN_RESET);
}

/**
 * @brief  Configure PE1 as High-Z (input, no pull).
 * @note   In this mode the external BCLKR signal is passed through.
 * @retval None
 */
void BCLKR_Set_HiZ(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = BCLKR_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BCLKR_CTRL_GPIO_Port, &GPIO_InitStruct);
}

/**
 * @brief  Configure PE1 as output and set logical high level.
 * @note   This mode pulls BCLKR input to +3.3V (attenuator control mode).
 * @retval None
 */
void BCLKR_Set_High(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = BCLKR_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BCLKR_CTRL_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOE, BCLKR_CTRL_Pin, GPIO_PIN_SET);
}

/**
 * @brief  Configure PE1 as output and set logical low level.
 * @note   This mode pulls BCLKR input to GND (16-bit data format select).
 * @retval None
 */
void BCLKR_Set_Low(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = BCLKR_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BCLKR_CTRL_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(BCLKR_CTRL_GPIO_Port, BCLKR_CTRL_Pin, GPIO_PIN_RESET);
}

/**
 * @brief  Get battery status from LOW_BATTERY (LOW BATTERY input).
 * @note   LOW_BATTERY is configured as GPIO input with pull-up.
 * @retval BatteryStatus_t - BATTERY_OK or BATTERY_LOW
 */
battery_status_t battery_get_status(void)
{
  GPIO_PinState pin_state = HAL_GPIO_ReadPin(LOW_BATTERY_GPIO_Port, LOW_BATTERY_Pin);

  if (pin_state == GPIO_PIN_SET)
  {
    return BATTERY_LOW; // 1 → low battery
  }
  else
  {
    return BATTERY_OK; // 0 → safe level
  }
}

/**
 * @brief  Set bit in static flags variable.
 * @param  bit Bit position [0..7]
 * @retval None
 */
void flags_set_bit(uint8_t bit)
{
  if (bit > 7) return;
  flags |= (1U << bit);
}

/**
 * @brief  Reset bit in static flags variable.
 * @param  bit Bit position [0..7]
 * @retval None
 */
void flags_reset_bit(uint8_t bit)
{
  if (bit > 7) return;
  flags &= ~(1U << bit);
}

sd_cd_state_t sd_cd_state_get(void)
{
  GPIO_PinState pin_state = HAL_GPIO_ReadPin(SD_CD_GPIO_Port, SD_CD_Pin);

  if (pin_state == GPIO_PIN_RESET)
  {
    return SD_CD_LOW;
  }
  else
  {
    return SD_CD_HI;
  }
}

/**
 * @brief  Reads the current state of the PTT (Push-To-Talk) input line.
 * @retval ptt_state_t
 *         - PTT_LOW : PTT line is in a low (inactive) state.
 *         - PTT_HI  : PTT line is in a high (active) state.
 *
 * @note   This function reads the GPIO pin associated with the PTT input.
 *         The pin and port must be properly configured before calling this function.
 */
ptt_state_t ptt_state_get(void)
{
  GPIO_PinState pin_state = HAL_GPIO_ReadPin(PTT_GPIO_Port, PTT_Pin);

  if (pin_state == GPIO_PIN_RESET)
  {
    return PTT_ACTIVE;
  }
  else
  {
    return PTT_INACTIVE;
  }
}

/**
 * @brief  Get the state of CALL button (CALL_BTN).
 * @note   CALL_BTN is configured as GPIO input with Pull-Up.
 * @retval call_button_state_t - CALL_BUTTON_PRESSED or CALL_BUTTON_RELEASED
 */
call_button_state_t call_button_get_state(void)
{
  GPIO_PinState pin_state = HAL_GPIO_ReadPin(CALL_BTN_GPIO_Port, CALL_BTN_Pin);

  if (pin_state == GPIO_PIN_RESET)
  {
    return CALL_BUTTON_PRESSED; // 0 → pressed
  }
  else
  {
    return CALL_BUTTON_RELEASED; // 1 → not pressed
  }
}

/**
 * @brief  Get the state of ALARM button (ALARM_BTN).
 * @note   ALARM_BTN is configured as GPIO input with Pull-Up.
 * @retval alarm_button_state_t - ALARM_BUTTON_PRESSED or ALARM_BUTTON_RELEASED
 */
alarm_button_state_t alarm_button_get_state(void)
{
  GPIO_PinState pin_state = HAL_GPIO_ReadPin(ALARM_BTN_GPIO_Port, ALARM_BTN_Pin);

  if (pin_state == GPIO_PIN_RESET)
  {
    return ALARM_BUTTON_PRESSED; // 0 → pressed
  }
  else
  {
    return ALARM_BUTTON_RELEASED; // 1 → not pressed
  }
}

/**
 * @brief  Get the state of RESET_ALARM button (RESET_ALARM_BTN).
 * @note   RESET_ALARM_BTN is configured as GPIO input with Pull-Up.
 * @retval reset_alarm_button_state_t - RESET_BUTTON_PRESSED or RESET_BUTTON_RELEASED
 */
reset_alarm_button_state_t reset_alarm_button_get_state(void)
{
  GPIO_PinState pin_state = HAL_GPIO_ReadPin(RESET_ALARM_BTN_GPIO_Port, RESET_ALARM_BTN_Pin);

  if (pin_state == GPIO_PIN_RESET)
  {
    return RESET_ALARM_BUTTON_PRESSED; // 0 → pressed
  }
  else
  {
    return RESET_ALARM_BUTTON_RELEASED; // 1 → not pressed
  }
}

/**
 * @brief  Get state of selected sensor.
 * @param  channel Sensor channel (SENSOR1…SENSOR5)
 * @retval sensor_state_t - SENSOR_NORMAL or SENSOR_ALARM
 */
sensor_state_t sensor_get_state(sensor_channel_t channel)
{
  GPIO_PinState pin_state = HAL_GPIO_ReadPin(SENSOR_PORT, channel);

  if (pin_state == GPIO_PIN_RESET)
  {
    return SENSOR_ALARM; // 0 → alarm
  }
  else
  {
    return SENSOR_NORMAL; // 1 → normal
  }
}

/**
 * @brief
 */
void group_led_on(void)
{
  HAL_GPIO_WritePin(LED_GROUP_SENSOR_GPIO_Port, LED_GROUP_SENSOR_Pin, GPIO_PIN_RESET); // LED ON
}

/**
 * @brief
 */
void group_led_off(void)
{
  HAL_GPIO_WritePin(LED_GROUP_SENSOR_GPIO_Port, LED_GROUP_SENSOR_Pin, GPIO_PIN_SET); // LED OFF
}

/**
 * @brief
 */
void sensor_1_led_on(void)
{
  HAL_GPIO_WritePin(LED_SENSOR_1_GPIO_Port, LED_SENSOR_1_Pin, GPIO_PIN_RESET); // LED ON
}

/**
 * @brief
 */
void sensor_1_led_off(void)
{
  HAL_GPIO_WritePin(LED_SENSOR_1_GPIO_Port, LED_SENSOR_1_Pin, GPIO_PIN_SET); // LED OFF
}

/**
 * @brief
 */
void sensor_2_led_on(void)
{
  HAL_GPIO_WritePin(LED_SENSOR_2_GPIO_Port, LED_SENSOR_2_Pin, GPIO_PIN_RESET); // LED ON
}

/**
 * @brief
 */
void sensor_2_led_off(void)
{
  HAL_GPIO_WritePin(LED_SENSOR_2_GPIO_Port, LED_SENSOR_2_Pin, GPIO_PIN_SET); // LED OFF
}

/**
 * @brief
 */
void sensor_3_led_on(void)
{
  HAL_GPIO_WritePin(LED_SENSOR_3_GPIO_Port, LED_SENSOR_3_Pin, GPIO_PIN_RESET); // LED ON
}

/**
 * @brief
 */
void sensor_3_led_off(void)
{
  HAL_GPIO_WritePin(LED_SENSOR_3_GPIO_Port, LED_SENSOR_3_Pin, GPIO_PIN_SET); // LED OFF
}

/**
 * @brief
 */
void sensor_4_led_on(void)
{
  HAL_GPIO_WritePin(LED_SENSOR_4_GPIO_Port, LED_SENSOR_4_Pin, GPIO_PIN_RESET); // LED ON
}

/**
 * @brief
 */
void sensor_4_led_off(void)
{
  HAL_GPIO_WritePin(LED_SENSOR_4_GPIO_Port, LED_SENSOR_4_Pin, GPIO_PIN_SET); // LED OFF
}

/**
 * @brief
 */
void sensor_5_led_on(void)
{
  HAL_GPIO_WritePin(LED_SENSOR_5_GPIO_Port, LED_SENSOR_5_Pin, GPIO_PIN_RESET); // LED ON
}

/**
 * @brief
 */
void sensor_5_led_off(void)
{
  HAL_GPIO_WritePin(LED_SENSOR_5_GPIO_Port, LED_SENSOR_5_Pin, GPIO_PIN_SET); // LED OFF
}

/**
 * @brief  Turn ON the ALARM LED.
 * @note   LED ON corresponds to logic 0: alarm active.
 * @retval None
 */
void alarm_led_on(void)
{
  HAL_GPIO_WritePin(LED_ALARM_GPIO_Port, LED_ALARM_Pin, GPIO_PIN_RESET); // 0 → LED ON
}

/**
 * @brief  Turn OFF the ALARM LED.
 * @note   LED OFF corresponds to logic 1: no alarm.
 * @retval None
 */
void alarm_led_off(void)
{
  HAL_GPIO_WritePin(LED_ALARM_GPIO_Port, LED_ALARM_Pin, GPIO_PIN_SET); // 1 → LED OFF
}

/**
 * @brief  Turn ON the CALL LED.
 * @note   LED ON corresponds to logic 0: alarm active.
 * @retval None
 */
void call_led_on(void)
{
  HAL_GPIO_WritePin(LED_CALL_GPIO_Port, LED_CALL_Pin, GPIO_PIN_RESET); // 0 → LED ON
}

/**
 * @brief  Turn OFF the CALL LED.
 * @note   LED OFF corresponds to logic 1: no alarm.
 * @retval None
 */
void call_led_off(void)
{
  HAL_GPIO_WritePin(LED_CALL_GPIO_Port, LED_CALL_Pin, GPIO_PIN_SET); // 1 → LED OFF
}

/**
 * @brief  Turn ON the SD Card Status LED.
 * @note   LED ON corresponds to logic 1: SD card OK.
 * @retval None
 */
void sd_card_led_on(void)
{
  HAL_GPIO_WritePin(LED_SD_CARD_GPIO_Port, LED_SD_CARD_Pin, GPIO_PIN_SET); // 1 → LED ON
}

/**
 * @brief  Turn OFF the SD Card Status LED.
 * @note   LED OFF corresponds to logic 0: SD card absent or error.
 * @retval None
 */
void sd_card_led_off(void)
{
  HAL_GPIO_WritePin(LED_SD_CARD_GPIO_Port, LED_SD_CARD_Pin, GPIO_PIN_RESET); // 0 → LED OFF
}

/**
 * @brief  Turn ON the speaker amplifier.
 * @note   Logic 0 → amplifier enabled
 * @retval None
 */
void sp_on(void)
{
  HAL_GPIO_WritePin(SP_GPIO_Port, SP_Pin, GPIO_PIN_RESET); // 0 → amplifier ON
}

/**
 * @brief  Turn OFF the speaker amplifier.
 * @note   Logic 1 → amplifier disabled
 * @retval None
 */
void sp_off(void)
{
  HAL_GPIO_WritePin(SP_GPIO_Port, SP_Pin, GPIO_PIN_SET); // 1 → amplifier OFF
}

/**
 * @brief  Turn ON the handset speaker amplifier.
 * @note   Logic 0 → amplifier enabled
 * @retval None
 */
void hp_on(void)
{
  HAL_GPIO_WritePin(HP_GPIO_Port, HP_Pin, GPIO_PIN_RESET); // 0 → amplifier ON
}

/**
 * @brief  Turn OFF the handset speaker amplifier.
 * @note   Logic 1 → amplifier disabled
 * @retval None
 */
void hp_off(void)
{
  HAL_GPIO_WritePin(HP_GPIO_Port, HP_Pin, GPIO_PIN_SET); // 1 → amplifier OFF
}

/**
 * @brief  Turn ON the microphone amplifier.
 * @note   MIC = 1 → MIC enabled, only during voice session.
 * @retval None
 */
void mic_on(void)
{
  HAL_GPIO_WritePin(MIC_GPIO_Port, MIC_Pin, GPIO_PIN_SET);
}

/**
 * @brief  Turn OFF the microphone amplifier.
 * @note   MIC = 0 → MIC disabled.
 * @retval None
 */
void mic_off(void)
{
  HAL_GPIO_WritePin(MIC_GPIO_Port, MIC_Pin, GPIO_PIN_RESET);
}

/**
 * @brief  Set the AGC speed for the microphone amplifier.
 * @param  speed Desired AGC speed (AR_FAST, AR_MED, AR_SLOW)
 * @retval None
 */
void mic_ar_set(ar_speed_t speed)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  switch (speed)
  {
  case AR_FAST:
    GPIO_InitStruct.Pin = MIC_A_R_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MIC_A_R_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(MIC_A_R_GPIO_Port, MIC_A_R_Pin, GPIO_PIN_RESET); // 0 → fast
    break;
  case AR_MED:
    GPIO_InitStruct.Pin = MIC_A_R_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MIC_A_R_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(MIC_A_R_GPIO_Port, MIC_A_R_Pin, GPIO_PIN_SET); // 1 → medium
    break;
  case AR_SLOW:
    // High-Z mode
    GPIO_InitStruct.Pin = MIC_A_R_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(MIC_A_R_GPIO_Port, &GPIO_InitStruct);
    break;
  }
}

/**
 * @brief  Set the gain of the microphone amplifier.
 * @param  gain Desired gain (GAIN_40DB, GAIN_50DB, GAIN_60DB)
 * @retval None
 */
void mic_gain_set(mic_gain_t gain)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  switch (gain)
  {
  case GAIN_40DB:
    GPIO_InitStruct.Pin = GAIN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GAIN_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GAIN_GPIO_Port, GAIN_Pin, GPIO_PIN_SET);
    break;
  case GAIN_50DB:
    GPIO_InitStruct.Pin = GAIN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GAIN_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GAIN_GPIO_Port, GAIN_Pin, GPIO_PIN_RESET);
    break;
  case GAIN_60DB:
    // High-Z mode
    GPIO_InitStruct.Pin = GAIN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GAIN_GPIO_Port, &GPIO_InitStruct);
    break;
  }
}
/*****************************************************************************/
