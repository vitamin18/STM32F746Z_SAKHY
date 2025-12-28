/*******************************************************************************
 * Project:     STM32F746Z_SAKHY
 * File:        io_control.h
 *******************************************************************************
 * Version:    V1.0
 * Revision:   R1.0.0
 * *****************************************************************************
 * Description:
 *
 *******************************************************************************
 * Created on:  Sep 30, 2025
 ******************************************************************************/

#ifndef INC_IO_CONTROL_H_
#define INC_IO_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*****************************************************************************/
/******************************* INCLUDES ************************************/
#include "main.h"
  /*****************************************************************************/
  /******************************* DEFINES *************************************/
#define SENSOR_PORT GPIOE
  /*****************************************************************************/
  /******************************* VARIABLES ***********************************/
  /**
   * @brief  Battery status enumeration.
   */
  typedef enum
  {
    BATTERY_OK = 0, /*!< Battery charge is at safe working level */
    BATTERY_LOW = 1 /*!< Battery charge < 5%, recording must be finalized */
  } battery_status_t;

  typedef enum
  {
    SD_CD_LOW = 0,
    SD_CD_HI
  } sd_cd_state_t;

  /**
   * @brief  Enumeration for PTT (Push-To-Talk) line states.
   * @note   Used to represent the logical state of the PTT input signal.
   */
  typedef enum
  {
    PTT_INACTIVE = 0, /**< PTT line is in a low (inactive) state. */
    PTT_ACTIVE = 1    /**< PTT line is in a high (active) state.  */
  } ptt_state_t;

  /**
   * @brief  Call button state definition.
   */
  typedef enum
  {
    CALL_BUTTON_PRESSED = 0, /*!< Logic 0: button is pressed */
    CALL_BUTTON_RELEASED = 1 /*!< Logic 1: button is not pressed */
  } call_button_state_t;

  /**
   * @brief  Alarm button state definition.
   */
  typedef enum
  {
    ALARM_BUTTON_PRESSED = 0, /*!< Logic 0: button is pressed */
    ALARM_BUTTON_RELEASED = 1 /*!< Logic 1: button is not pressed */
  } alarm_button_state_t;

  /**
   * @brief  Reset button state definition.
   */
  typedef enum
  {
    RESET_ALARM_BUTTON_PRESSED = 0, /*!< Logic 0: button is pressed */
    RESET_ALARM_BUTTON_RELEASED = 1 /*!< Logic 1: button is not pressed */
  } reset_alarm_button_state_t;

  /**
   * @brief  Security sensor state definition.
   */
  typedef enum
  {
    SENSOR_ALARM = 0, /*!< Logic 0: sensor triggered (object opened) */
    SENSOR_NORMAL = 1 /*!< Logic 1: sensor closed (no alarm) */
  } sensor_state_t;

  /**
   * @brief  Sensor channel definition (PE11…PE15).
   */
  typedef enum
  {
    SENSOR1 = SENSOR_1_Pin,
    SENSOR2 = SENSOR_2_Pin,
    SENSOR3 = SENSOR_3_Pin,
    SENSOR4 = SENSOR_4_Pin,
    SENSOR5 = SENSOR_5_Pin,
  } sensor_channel_t;

  /**
   * @brief  Automatic gain control (AGC) speed selection.
   */
  typedef enum
  {
    AR_FAST = 0, /*!< Fast AGC (~0.5 sec) */
    AR_MED = 1,  /*!< Medium AGC (~2 sec, default) */
    AR_SLOW = 2  /*!< Slow AGC (~4.5 sec, High-Z) */
  } ar_speed_t;

  /**
   * @brief  Microphone gain selection.
   */
  typedef enum
  {
    GAIN_50DB = 0, /*!< Default gain 50 dB */
    GAIN_40DB = 1, /*!< Gain 40 dB */
    GAIN_60DB = 2  /*!< Gain 60 dB (High-Z) */
  } mic_gain_t;

  /*****************************************************************************/
  /******************************* METHODS *************************************/
  void vocoder_enable(void);
  void vocoder_disable(void);
  sd_cd_state_t sd_cd_state_get(void);
  ptt_state_t ptt_state_get(void);
  void vocoder_enable(void);
  void BCLKR_Set_Low(void);
  battery_status_t battery_get_status(void);

  void group_led_on(void);
  void group_led_off(void);
  void sensor_1_led_on(void);
  void sensor_1_led_off(void);
  void sensor_2_led_on(void);
  void sensor_2_led_off(void);
  void sensor_3_led_on(void);
  void sensor_3_led_off(void);
  void sensor_4_led_on(void);
  void sensor_4_led_off(void);
  void sensor_5_led_on(void);
  void sensor_5_led_off(void);
  void alarm_led_on(void);
  void alarm_led_off(void);
  void call_led_on(void);
  void call_led_off(void);
  void sd_card_led_on(void);
  void sd_card_led_off(void);
  void sp_on(void);
  void sp_off(void);
  void hp_on(void);
  void hp_off(void);
  void mic_on(void);
  void mic_off(void);
  void mic_ar_set(ar_speed_t speed);
  void mic_gain_set(mic_gain_t gain);

#ifdef __cplusplus
}
#endif

#endif /* INC_IO_CONTROL_H_ */
/*****************************************************************************/
