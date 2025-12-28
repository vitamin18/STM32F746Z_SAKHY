/*******************************************************************************
 * Company:     "WSE" LLP
 * Project:     STM32F746Z_SAKHY
 * File:        w5500_port.c
 * Author:      Kostyrev Vitaliy
 * Position:    Embedded Systems Engineer
 *******************************************************************************
 * Version:    V1.0
 * Revision:   R1.0.0
 * *****************************************************************************
 * Description:
 *
 *******************************************************************************
 * Created on:  May 10, 2025
 ******************************************************************************/

/******************************* INCLUDES ************************************/
#include "w5500_port.h"
#include "spi.h"

/*****************************************************************************/
/******************************* DEFINES *************************************/
/*****************************************************************************/
/******************************* VARIABLES ***********************************/
/*****************************************************************************/
/******************************* PROTOTYPES **********************************/
/*****************************************************************************/
/******************************* METHODS *************************************/
/**
 * @brief Enter critical section for W5500 access.
 *
 * This function can be overridden to disable interrupts or perform
 * other protection before accessing W5500 shared resources.
 */
__WEAK void w5500_critcical_enter(void)
{
  // Default implementation: do nothing
}

/**
 * @brief Exit critical section for W5500 access.
 *
 * This function can be overridden to re-enable interrupts or
 * finalize protection after accessing W5500 shared resources.
 */
__WEAK void w5500_critcical_exit(void)
{
  // Default implementation: do nothing
}

/**
 * @brief Set W5500 chip select (CS) pin low.
 *
 * Activates the W5500 SPI interface by pulling the CS pin low.
 */
void w5500_spi_cs_low(void)
{
  HAL_GPIO_WritePin(W5500_NSS_GPIO_Port, W5500_NSS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Set W5500 chip select (CS) pin high.
 *
 * Deactivates the W5500 SPI interface by releasing the CS pin.
 */
void w5500_spi_cs_hi(void)
{
  HAL_GPIO_WritePin(W5500_NSS_GPIO_Port, W5500_NSS_Pin, GPIO_PIN_SET);
}

/**
 * @brief Read data from W5500 via SPI.
 *
 * @param data Pointer to the buffer where received data will be stored.
 * @param len  Number of bytes to receive.
 */
void w5500_spi_read(uint8_t *data, uint16_t len)
{
  HAL_SPI_Receive(&hspi1, data, len, 100);
}

/**
 * @brief Write data to W5500 via SPI.
 *
 * @param data Pointer to the buffer containing data to be sent.
 * @param len  Number of bytes to transmit.
 */
void w5500_spi_write(uint8_t *data, uint16_t len)
{
  HAL_SPI_Transmit(&hspi1, data, len, 100);
}
/*****************************************************************************/
