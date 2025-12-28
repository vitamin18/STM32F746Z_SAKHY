/*******************************************************************************
 * Company:     "WSE" LLP
 * Project:     STM32F746Z_SAKHY
 * File:        device.h
 *******************************************************************************
 * Version:    V1.0
 * Revision:   R1.0.0
 * *****************************************************************************
 * Description:
 *
 *******************************************************************************
 * Created on:  Sep 30, 2025
 ******************************************************************************/

#ifndef INC_DEVICE_H_
#define INC_DEVICE_H_

#ifdef __cplusplus
extern "C"
{
#endif

  /*****************************************************************************/
  /******************************* INCLUDES ************************************/
#include "main.h"
  /*****************************************************************************/
  /******************************* DEFINES *************************************/
#define MASK_0_BIT_CALL (0b00000001)       /* bit 0 */
#define MASK_1_BIT_ALARM (0b00000010)      /* bit 1 */
#define MASK_2_6_BITS_SENSORS (0b01111100) /* bits 2–6 */
  /*****************************************************************************/
  /******************************* VARIABLES ***********************************/
  /**
   * @brief  Device address enumeration.
   */
  typedef enum
  {
    DEV_ADDR_A = 0,
    DEV_ADDR_B = 1,
  } dev_addr_e;

  typedef enum
  {
    ALARM_FLAG_STATE_OFF,
    ALARM_FLAG_STATE_ON,
  } alarm_state_e;

  typedef enum
  {
    CALL_FLAG_STATE_OFF,
    CALL_FLAG_STATE_ON,
  } call_state_e;

  typedef enum
  {
    SENSOR_OPEN = 0,
    SENSOR_CLOSE,
  } sensor_state_e;

  typedef enum
  {
    FIELD_STATE_RESET = 0,
    FIELD_STATE_SET,
  } field_state_e;

  typedef enum
  {
    FIELD_CALL = 0,
    FIELD_ALARM,
    FIELD_SENSOR_1,
    FIELD_SENSOR_2,
    FIELD_SENSOR_3,
    FIELD_SENSOR_4,
    FIELD_SENSOR_5,
    FIELD_ALL,
  } field_event_e;

  typedef struct
  {
    uint8_t state;
  } field_t;

  typedef struct
  {
    const int16_t *buffer;
    uint32_t size;
    uint32_t index;
  } pcm_buf_t;

  extern field_t field_local;
  extern field_t field_dest;
  extern pcm_buf_t pcm_buf_alarm;
  extern pcm_buf_t pcm_buf_call;
  /*****************************************************************************/
  /******************************* METHODS *************************************/
  dev_addr_e dev_addr_get(void);
  void copy_int16_array_circular(pcm_buf_t *pcm_buf, int16_t *dst, uint32_t len);
  uint8_t copy_bits(uint8_t dst, uint8_t src, uint8_t mask);
  void flag_alarm_button_set(void);
  void flag_alarm_button_clear(void);
  alarm_state_e flag_alarm_button_get(void);
  void flag_alarm_sensor_set(void);
  void flag_alarm_sensor_clear(void);
  alarm_state_e flag_alarm_sensor_get(void);
  call_state_e flag_call_get(void);
  void flag_call_clear(void);
  void field_event_set(field_t *field, field_event_e event);
  void field_event_clear(field_t *field, field_event_e event);
  field_state_e field_event_get(field_t *field, field_event_e event);
  GPIO_PinState pin_debounce(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

  void read_state_sensors(field_t *field);
  void process_field_sensor(field_t *field_local, field_t *field_dest);
  void process_field_reset_alarm(field_t *field_local, field_t *field_dest);
  void process_field_alarm_button(field_t *field_dest);
  void process_field_call(field_t *field_dest);

#ifdef __cplusplus
}
#endif

#endif /* INC_DEVICE_H_ */
/*****************************************************************************/
