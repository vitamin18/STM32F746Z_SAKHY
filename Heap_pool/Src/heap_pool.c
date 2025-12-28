/*******************************************************************************
 * Company:     "WSE" LLP
 * Project:     STM32U5A9ZJT6Q_WSE_6G
 * File:        heap_pool.c
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

/******************************* INCLUDES ************************************/
#include "heap_pool.h"

/*****************************************************************************/
/******************************* DEFINES *************************************/
#define BITMAP_GET(bitmap, index) ((bitmap[(index) / 8] >> ((index) % 8)) & 0x01)
#define BITMAP_SET(bitmap, index) (bitmap[(index) / 8] |= (1U << ((index) % 8)))
#define BITMAP_CLR(bitmap, index) (bitmap[(index) / 8] &= ~(1U << ((index) % 8)))

/*****************************************************************************/
/******************************* VARIABLES ***********************************/

/*****************************************************************************/
/******************************* PROTOTYPES **********************************/

/*****************************************************************************/
/******************************* METHODS *************************************/

/**
 * @brief Enter critical section. Can be overridden.
 */
__attribute__((weak)) void heap_pool_lock(void)
{
  // Default: do nothing (non-thread-safe)
}

/**
 * @brief Exit critical section. Can be overridden.
 */
__attribute__((weak)) void heap_pool_unlock(void)
{
  // Default: do nothing (non-thread-safe)
}

/**
 * @brief Initialize the heap pool.
 *
 * @param pool         Pointer to heap structure.
 * @param memory       Pointer to preallocated memory buffer [block_count * block_size].
 * @param bitmap       Pointer to bitmap buffer [(block_count + 7) / 8] bytes.
 * @param block_size   Size of each block in bytes.
 * @param block_count  Number of blocks.
 */
void heap_pool_init(heap_pool_t *pool, uint8_t *memory, uint8_t *bitmap, uint16_t block_size, uint16_t block_count)
{
  pool->memory = memory;
  pool->used_bitmap = bitmap;
  pool->block_size = block_size;
  pool->block_count = block_count;

  uint16_t bitmap_size = (block_count + 7) / 8;
  memset(pool->used_bitmap, 0, bitmap_size);
}

/**
 * @brief Allocate one block from the heap pool.
 *
 * @param pool Pointer to heap structure.
 * @return Pointer to the allocated block, or NULL if none available.
 */
void *heap_pool_alloc(heap_pool_t *pool)
{
  heap_pool_lock();

  for (uint16_t i = 0; i < pool->block_count; ++i)
  {
    if (!BITMAP_GET(pool->used_bitmap, i))
    {
      BITMAP_SET(pool->used_bitmap, i);
      heap_pool_unlock();
      return pool->memory + i * pool->block_size;
    }
  }

  heap_pool_unlock();
  return NULL;
}

/**
 * @brief Free a previously allocated block.
 *
 * @param pool Pointer to heap structure.
 * @param ptr  Pointer to block to be freed.
 */
void heap_pool_free(heap_pool_t *pool, void *ptr)
{
  heap_pool_lock();

  uintptr_t offset = (uint8_t *)ptr - pool->memory;

  if (offset % pool->block_size == 0)
  {
    uint16_t index = offset / pool->block_size;
    if (index < pool->block_count)
    {
      BITMAP_CLR(pool->used_bitmap, index);
    }
  }

  heap_pool_unlock();
}

/**
 * @brief Free all allocated blocks in the pool.
 *
 * @param pool Pointer to heap structure.
 */
void heap_pool_free_all(heap_pool_t *pool)
{
  heap_pool_lock();
  uint16_t bitmap_size = (pool->block_count + 7) / 8;
  memset(pool->used_bitmap, 0, bitmap_size);
  heap_pool_unlock();
}

/**
 * @brief Get the number of free blocks in the heap pool.
 *
 * @param pool Pointer to the heap structure.
 * @return Number of free (available) blocks.
 */
uint16_t heap_pool_free_count(const heap_pool_t *pool)
{
  uint16_t free_count = 0;

  heap_pool_lock();

  for (uint16_t i = 0; i < pool->block_count; ++i)
  {
    if (!BITMAP_GET(pool->used_bitmap, i))
    {
      free_count++;
    }
  }

  heap_pool_unlock();

  return free_count;
}
/*****************************************************************************/
