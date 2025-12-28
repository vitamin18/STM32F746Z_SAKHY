/*******************************************************************************
 * Company:     "WSE" LLP
 * Project:     STM32U5A9ZJT6Q_WSE_6G
 * File:        heap_pool.h
 * Author:      Kostyrev Vitaliy
 * Position:    Embedded Systems Engineer
 *******************************************************************************
 * Version:    V1.0
 * Revision:   R1.0.0
 * *****************************************************************************
 * Description:
 *
 *******************************************************************************
 * Created on:  Apr 25, 2025
 ******************************************************************************/

#ifndef HEAP_POOL_H_
#define HEAP_POOL_H_

#ifdef __cplusplus
extern "C"
{
#endif

  /*****************************************************************************/
  /******************************* INCLUDES ************************************/
#include <stdint.h>
#include <string.h>
#include <stddef.h>

  /*****************************************************************************/
  /******************************* DEFINES *************************************/

  /*****************************************************************************/
  /******************************* VARIABLES ***********************************/
  typedef struct
  {
    uint8_t *memory;      ///< Memory pool: [block_count * block_size] bytes
    uint8_t *used_bitmap; ///< External bitmap buffer: [(block_count + 7) / 8] bytes
    uint16_t block_size;  ///< Size of one block
    uint16_t block_count; ///< Number of blocks
  } heap_pool_t;
  /*****************************************************************************/
  /******************************* METHODS *************************************/
  void heap_pool_init(heap_pool_t *pool, uint8_t *memory, uint8_t *bitmap, uint16_t block_size, uint16_t block_count);
  void *heap_pool_alloc(heap_pool_t *pool);
  void heap_pool_free(heap_pool_t *pool, void *ptr);
  void heap_pool_free_all(heap_pool_t *pool);

#ifdef __cplusplus
}
#endif

#endif /* HEAP_POOL_H_ */
/*****************************************************************************/
