/*******************************************************************************
 * Company:     "WSE" LLP
 * Project:     STM32U5A9ZJT6Q_WSE_6G
 * File:        fifo_queue.h
 * Author:      Kostyrev Vitaliy
 * Position:    Embedded Systems Engineer
 *******************************************************************************
 * Version:    V1.0
 * Revision:   R1.0.0
 * *****************************************************************************
 * Description:
 *
 *******************************************************************************
 * Created on:  Apr 22, 2025
 ******************************************************************************/

#ifndef FIFO_QUEUE_H_
#define FIFO_QUEUE_H_

#ifdef __cplusplus
extern "C"
{
#endif

  /*****************************************************************************/
  /******************************* INCLUDES ************************************/
#include <stddef.h>
#include <stdint.h>
  /*****************************************************************************/
  /******************************* DEFINES *************************************/

  /*****************************************************************************/
  /******************************* VARIABLES ***********************************/
  typedef struct
  {
    void *data;           ///< Pointer to the external buffer [capacity][item_size]
    uint16_t capacity;    ///< Maximum number of elements
    uint16_t item_size;   ///< Size of one item in bytes
    uint16_t write_index; ///< Index to write next item
    uint16_t read_index;  ///< Index to read next item
    uint16_t count;       ///< Current number of stored items
  } fifo_queue_t;
  /*****************************************************************************/
  /******************************* METHODS *************************************/

  void fifo_queue_init(fifo_queue_t *fifo, void *buffer, uint16_t capacity, uint16_t item_size);
  void fifo_queue_clear(fifo_queue_t *fifo);
  uint8_t *fifo_queue_write(fifo_queue_t *fifo);
  uint8_t *fifo_queue_read(fifo_queue_t *fifo);
  void fifo_queue_free(fifo_queue_t *fifo);
  int fifo_queue_is_full(const fifo_queue_t *fifo);
  int fifo_queue_is_empty(const fifo_queue_t *fifo);
  uint16_t fifo_queue_count(const fifo_queue_t *fifo);
  uint16_t fifo_queue_free_space(const fifo_queue_t *fifo);

#ifdef __cplusplus
}
#endif

#endif /* FIFO_QUEUE_H_ */
/*****************************************************************************/
