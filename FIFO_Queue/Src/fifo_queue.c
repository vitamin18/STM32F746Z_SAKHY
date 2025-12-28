/*******************************************************************************
 * Company:     "WSE" LLP
 * Project:     STM32U5A9ZJT6Q_WSE_6G
 * File:        fifo_queue.c
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

/******************************* INCLUDES ************************************/
#include "fifo_queue.h"

/*****************************************************************************/
/******************************* DEFINES *************************************/
/*****************************************************************************/
/******************************* VARIABLES ***********************************/

/*****************************************************************************/
/******************************* PROTOTYPES **********************************/

/*****************************************************************************/
/******************************* METHODS *************************************/
/**
 * @brief Initialize the FIFO queue.
 *
 * @param fifo Pointer to the FIFO structure.
 * @param buffer Pointer to the external memory buffer [capacity][item_size].
 * @param capacity Number of elements in the queue.
 * @param item_size Size of each element in bytes.
 */
void fifo_queue_init(fifo_queue_t *fifo, void *buffer, uint16_t capacity, uint16_t item_size)
{
  fifo->data = buffer;
  fifo->capacity = capacity;
  fifo->item_size = item_size;
  fifo->write_index = 0;
  fifo->read_index = 0;
  fifo->count = 0;
}

/**
 * @brief Clear the FIFO queue and reset all internal indices.
 *
 * This function resets the read and write indices as well as the element count
 * to zero, effectively making the queue empty. The contents of the underlying
 * data buffer are not modified.
 *
 * @param fifo Pointer to the FIFO queue structure.
 */
void fifo_queue_clear(fifo_queue_t *fifo)
{
  if (fifo == NULL)
  {
    return;
  }

  fifo->write_index = 0;
  fifo->read_index = 0;
  fifo->count = 0;
}

/**
 * @brief Reserve space for writing to the queue.
 *
 * Increments write index and count if space is available.
 *
 * @param fifo Pointer to the FIFO structure.
 * @return Pointer to the buffer for writing, or NULL if the queue is full.
 */
uint8_t *fifo_queue_write(fifo_queue_t *fifo)
{
  if (fifo->count >= fifo->capacity)
  {
    return NULL;
  }
  uint8_t *ptr = (uint8_t *)fifo->data + (fifo->write_index * fifo->item_size);
  fifo->write_index = (fifo->write_index + 1) % fifo->capacity;
  fifo->count++;
  return ptr;
}

/**
 * @brief Get pointer to the next item for reading.
 *
 * Does not update read index or count — must call `fifo_queue_free()` after use.
 *
 * @param fifo Pointer to the FIFO structure.
 * @return Pointer to the buffer for reading, or NULL if the queue is empty.
 */
uint8_t *fifo_queue_read(fifo_queue_t *fifo)
{
  if (fifo->count == 0)
  {
    return NULL;
  }
  return (uint8_t *)fifo->data + (fifo->read_index * fifo->item_size);
}

/**
 * @brief Free the current read item after processing.
 *
 * Increments read index and decrements item count.
 * Should be called after reading with `fifo_queue_read()`.
 *
 * @param fifo Pointer to the FIFO structure.
 */
void fifo_queue_free(fifo_queue_t *fifo)
{
  if (fifo->count > 0)
  {
    fifo->read_index = (fifo->read_index + 1) % fifo->capacity;
    fifo->count--;
  }
}

/**
 * @brief Check if the FIFO queue is full.
 *
 * @param fifo Pointer to the FIFO structure.
 * @return 1 if full, 0 otherwise.
 */
int fifo_queue_is_full(const fifo_queue_t *fifo)
{
  return fifo->count >= fifo->capacity;
}

/**
 * @brief Check if the FIFO queue is empty.
 *
 * @param fifo Pointer to the FIFO structure.
 * @return 1 if empty, 0 otherwise.
 */
int fifo_queue_is_empty(const fifo_queue_t *fifo)
{
  return fifo->count == 0;
}

/**
 * @brief Get the number of items currently stored in the queue.
 *
 * @param fifo Pointer to the FIFO structure.
 * @return Number of items in the queue.
 */
uint16_t fifo_queue_count(const fifo_queue_t *fifo)
{
  return fifo->count;
}

/**
 * @brief Get the number of free slots available in the queue.
 *
 * @param fifo Pointer to the FIFO structure.
 * @return Number of free slots.
 */
uint16_t fifo_queue_free_space(const fifo_queue_t *fifo)
{
  return fifo->capacity - fifo->count;
}

/*****************************************************************************/
