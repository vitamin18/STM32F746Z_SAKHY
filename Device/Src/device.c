/*******************************************************************************
 * Company:     "WSE" LLP
 * Project:     STM32F746Z_SAKHY
 * File:        device.c
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
#include "device.h"
#include "alarm_pcm_buf.h"
#include "call_pcm_buf.h"
#include "io_control.h"
/*****************************************************************************/
/******************************* DEFINES *************************************/
/*****************************************************************************/
/******************************* VARIABLES ***********************************/
static alarm_state_e flag_alarm_sensor = ALARM_FLAG_STATE_OFF;
static alarm_state_e flag_alarm_button = ALARM_FLAG_STATE_OFF;
static call_state_e flag_call = CALL_FLAG_STATE_OFF;
field_t field_local = {.state = 0xFF};
field_t field_dest = {.state = 0xFF};
pcm_buf_t pcm_buf_alarm = {.buffer = alarm_pcm_buf, .size = sizeof(alarm_pcm_buf) / 2, .index = 0};
pcm_buf_t pcm_buf_call = {.buffer = call_pcm_buf, .size = sizeof(call_pcm_buf) / 2, .index = 0};
/*****************************************************************************/
/******************************* PROTOTYPES **********************************/

/*****************************************************************************/
/******************************* METHODS *************************************/
__WEAK void device_delay_ms(uint32_t ms)
{
  (void)ms;
}

__WEAK void timer_call_start(void)
{
}

__WEAK void timer_call_stop(void)
{
}

/**
 * @brief   Get the device address based on the hardware pin state.
 *
 * This function reads the GPIO pin configured for device address selection.
 * If the pin is set (high), it returns DEV_ADDR_B; otherwise, it returns DEV_ADDR_A.
 *
 * @note    The GPIO pin and port must be configured properly before calling this function.
 *
 * @retval  DEV_ADDR_A if the pin is low.
 * @retval  DEV_ADDR_B if the pin is high.
 */
dev_addr_e dev_addr_get(void)
{
  return HAL_GPIO_ReadPin(DEV_ADDR_GPIO_Port, DEV_ADDR_Pin) == GPIO_PIN_SET ? DEV_ADDR_B : DEV_ADDR_A;
}

void copy_int16_array_circular(pcm_buf_t *pcm_buf, int16_t *dst, uint32_t len)
{
  if (pcm_buf == NULL || dst == NULL)
  {
    return;
  }
  if (pcm_buf->size == 0)
  {
    return;
  }

  for (uint32_t i = 0; i < len; i++)
  {
    if (pcm_buf->index >= pcm_buf->size)
    {
      pcm_buf->index = 0;
    }

    dst[i] = pcm_buf->buffer[pcm_buf->index];
    pcm_buf->index++;
  }
}

/**
 * @brief Copies selected bits from src to dst while keeping all other bits unchanged.
 *
 * @param dst      Destination byte (will receive selected bits from src).
 * @param src      Source byte.
 * @param mask     Bit mask specifying which bits must be copied.
 *
 * @return Updated destination byte with copied bits.
 */
uint8_t copy_bits(uint8_t dst, uint8_t src, uint8_t mask)
{
  return (dst & ~mask) | (src & mask);
}

void flag_alarm_button_set(void)
{
  flag_alarm_button = ALARM_FLAG_STATE_ON;
  pcm_buf_alarm.index = 0;
}

void flag_alarm_button_clear(void)
{
  flag_alarm_button = ALARM_FLAG_STATE_OFF;
}

alarm_state_e flag_alarm_button_get(void)
{
  return flag_alarm_button;
}

void flag_alarm_sensor_set(void)
{
  flag_alarm_sensor = ALARM_FLAG_STATE_ON;
}

void flag_alarm_sensor_clear(void)
{
  flag_alarm_sensor = ALARM_FLAG_STATE_OFF;
}

alarm_state_e flag_alarm_sensor_get(void)
{
  return flag_alarm_sensor;
}

void flag_call_set(void)
{
  flag_call = CALL_FLAG_STATE_ON;
  pcm_buf_call.index = 0;
}

void flag_call_clear(void)
{
  flag_call = CALL_FLAG_STATE_OFF;
}

call_state_e flag_call_get(void)
{
  return flag_call;
}

sensor_state_e check_state_sensor(GPIO_TypeDef *gpio, uint16_t pin)
{
  sensor_state_e state = SENSOR_OPEN;

  device_delay_ms(20); // wait debounce

  if (HAL_GPIO_ReadPin(gpio, pin) == GPIO_PIN_SET)
  {
    state = SENSOR_CLOSE;
  }
  else
  {
    state = SENSOR_OPEN;
  }

  return state;
}

void field_event_set(field_t *field, field_event_e event)
{
  if (event == FIELD_ALL)
  {
    field->state = 0x7F; /* All 7 bits set */
  }
  else if (event <= FIELD_SENSOR_5)
  {
    field->state |= (1U << event);
  }
}

void field_event_clear(field_t *field, field_event_e event)
{
  if (event == FIELD_ALL)
  {
    field->state = 0x00; /* All 7 bits set */
  }
  else if (event <= FIELD_SENSOR_5)
  {
    field->state &= ~(1U << event);
  }
}

field_state_e field_event_get(field_t *field, field_event_e event)
{

  if (field->state & (1U << event))
  {
    return FIELD_STATE_SET;
  }

  return FIELD_STATE_RESET;
}

/**
 * @brief Computes the resulting field state for a given event based on local
 *        and destination field event masks.
 *
 * @param field_local  Local field event mask.
 * @param field_dest   Destination field event mask.
 * @param event        Event to evaluate.
 *
 * @return FIELD_STATE_SET if the event bit is set in both masks,
 *         otherwise FIELD_STATE_RESET.
 */
field_state_e field_compute_state(field_event_e field_local, field_event_e field_dest, field_event_e event)
{
  if ((field_local & (1U << event)) & (field_dest & (1U << event)))
  {
    return FIELD_STATE_SET;
  }

  return FIELD_STATE_RESET;
}

void process_group_led(void)
{
  if (HAL_GPIO_ReadPin(LED_SENSOR_1_GPIO_Port, LED_SENSOR_1_Pin) && HAL_GPIO_ReadPin(LED_SENSOR_2_GPIO_Port, LED_SENSOR_2_Pin) &&
      HAL_GPIO_ReadPin(LED_SENSOR_3_GPIO_Port, LED_SENSOR_3_Pin) && HAL_GPIO_ReadPin(LED_SENSOR_4_GPIO_Port, LED_SENSOR_4_Pin) &&
      HAL_GPIO_ReadPin(LED_SENSOR_5_GPIO_Port, LED_SENSOR_5_Pin))
  {
    group_led_on();
  }
  else
  {
    group_led_off();
  }
}

void read_state_sensors(field_t *field)
{
  // SENSOR 1
  if (check_state_sensor(SENSOR_1_GPIO_Port, SENSOR_1_Pin) == SENSOR_OPEN)
  {
    field_event_clear(field, FIELD_SENSOR_1);
  }
  else
  {
    field_event_set(field, FIELD_SENSOR_1);
  }
  // SENSOR 2
  if (check_state_sensor(SENSOR_2_GPIO_Port, SENSOR_2_Pin) == SENSOR_OPEN)
  {
    field_event_clear(field, FIELD_SENSOR_2);
  }
  else
  {
    field_event_set(field, FIELD_SENSOR_2);
  }
  // SENSOR 3
  if (check_state_sensor(SENSOR_3_GPIO_Port, SENSOR_3_Pin) == SENSOR_OPEN)
  {
    field_event_clear(field, FIELD_SENSOR_3);
  }
  else
  {
    field_event_set(field, FIELD_SENSOR_3);
  }
  // SENSOR 4
  if (check_state_sensor(SENSOR_4_GPIO_Port, SENSOR_4_Pin) == SENSOR_OPEN)
  {
    field_event_clear(field, FIELD_SENSOR_4);
  }
  else
  {
    field_event_set(field, FIELD_SENSOR_4);
  }
  // SENSOR 5
  if (check_state_sensor(SENSOR_5_GPIO_Port, SENSOR_5_Pin) == SENSOR_OPEN)
  {
    field_event_clear(field, FIELD_SENSOR_5);
  }
  else
  {
    field_event_set(field, FIELD_SENSOR_5);
  }
}

void process_field_sensor(field_t *field_local, field_t *field_dest)
{
  // SENSOR 1
  if (field_compute_state(field_local->state, field_dest->state, FIELD_SENSOR_1) == FIELD_STATE_RESET)
  {
    flag_alarm_sensor_set(); // Enable alarm
    sensor_1_led_on();       // Led on
    group_led_on();          // Led on
  }
  else
  {
    if (flag_alarm_sensor_get() == ALARM_FLAG_STATE_OFF)
    {
      sensor_1_led_off(); // Led off
    }
  }

  // SENSOR 2
  if (field_compute_state(field_local->state, field_dest->state, FIELD_SENSOR_2) == FIELD_STATE_RESET)
  {
    flag_alarm_sensor_set(); // Enable alarm
    sensor_2_led_on();       // Led on
    group_led_on();          // Led on
  }
  else
  {
    if (flag_alarm_sensor_get() == ALARM_FLAG_STATE_OFF)
    {
      sensor_2_led_off(); // Led off
    }
  }
  // SENSOR 3
  if (field_compute_state(field_local->state, field_dest->state, FIELD_SENSOR_3) == FIELD_STATE_RESET)
  {
    flag_alarm_sensor_set(); // Enable alarm
    sensor_3_led_on();       // Led on
    group_led_on();          // Led on
  }
  else
  {
    if (flag_alarm_sensor_get() == ALARM_FLAG_STATE_OFF)
    {
      sensor_3_led_off(); // Led off
    }
  }
  // SENSOR 4
  if (field_compute_state(field_local->state, field_dest->state, FIELD_SENSOR_4) == FIELD_STATE_RESET)
  {
    flag_alarm_sensor_set(); // Enable alarm
    sensor_4_led_on();       // Led on
    group_led_on();          // Led on
  }
  else
  {
    if (flag_alarm_sensor_get() == ALARM_FLAG_STATE_OFF)
    {
      sensor_4_led_off(); // Led off
    }
  }
  // SENSOR 5
  if (field_compute_state(field_local->state, field_dest->state, FIELD_SENSOR_5) == FIELD_STATE_RESET)
  {
    flag_alarm_sensor_set(); // Enable alarm
    sensor_5_led_on();       // Led on
    group_led_on();          // Led on
  }
  else
  {
    if (flag_alarm_sensor_get() == ALARM_FLAG_STATE_OFF)
    {
      sensor_5_led_off(); // Led off
    }
  }

  process_group_led();
}

void process_field_reset_alarm(field_t *field_local, field_t *field_dest)
{
  // SENSOR 1
  if (field_compute_state(field_local->state, field_dest->state, FIELD_SENSOR_1) != FIELD_STATE_RESET)
  {
    sensor_1_led_off(); // Led off
  }
  // SENSOR 2
  if (field_compute_state(field_local->state, field_dest->state, FIELD_SENSOR_2) != FIELD_STATE_RESET)
  {
    sensor_2_led_off(); // Led off
  }
  // SENSOR 3
  if (field_compute_state(field_local->state, field_dest->state, FIELD_SENSOR_3) != FIELD_STATE_RESET)
  {
    sensor_3_led_off(); // Led off
  }
  // SENSOR 4
  if (field_compute_state(field_local->state, field_dest->state, FIELD_SENSOR_4) != FIELD_STATE_RESET)
  {
    sensor_4_led_off(); // Led off
  }
  // SENSOR 5
  if (field_compute_state(field_local->state, field_dest->state, FIELD_SENSOR_5) != FIELD_STATE_RESET)
  {
    sensor_5_led_off(); // Led off
  }

  process_group_led();

  flag_alarm_button_clear();                // Disable alarm
  alarm_led_off();                          // Led on
  field_event_set(field_dest, FIELD_ALARM); // Clear field ALARM

  if (flag_call_get() == CALL_FLAG_STATE_OFF && ptt_state_get() == PTT_INACTIVE)
  {
    vocoder_disable(); // Vocoder disable
    sp_off();          // Off the speaker amplifier.
    hp_off();          // Off the handset speaker amplifier
  }
  else if (flag_call_get() == CALL_FLAG_STATE_ON && ptt_state_get() == PTT_ACTIVE)
  {
    hp_off(); // Off the handset speaker amplifier
  }
  else if (ptt_state_get() == PTT_ACTIVE)
  {
    sp_off(); // Off the speaker amplifier.
  }
}

void process_field_alarm_button(field_t *field_dest)
{
  if (field_event_get(field_dest, FIELD_ALARM) == FIELD_STATE_RESET)
  {
    flag_alarm_button_set(); // Enable alarm
    alarm_led_on();          // Led on
    vocoder_enable();        // vocoder enable
    sp_on();                 // On the speaker amplifier.
    hp_on();                 // handset speaker amplifier
  }
}

void process_field_call(field_t *field_dest)
{
  if (field_event_get(field_dest, FIELD_CALL) == FIELD_STATE_RESET)
  {
    if (ptt_state_get() == PTT_INACTIVE)
    {
      flag_call_set();    // Enable alarm
      call_led_on();      // Led on
      timer_call_start(); // Timer start
      vocoder_enable();   // vocoder enable
      sp_on();            // On the speaker amplifier.
    }
  }
  else
  {
    flag_call_clear(); // Disable alarm
    call_led_off();    // Led off
    timer_call_stop(); // Timer stop
    if (flag_alarm_button_get() == ALARM_FLAG_STATE_OFF && ptt_state_get() == PTT_INACTIVE)
    {
      vocoder_disable(); // Vocoder disable
      sp_off();          // Off the speaker amplifier.
    }
  }
}

/*****************************************************************************/
