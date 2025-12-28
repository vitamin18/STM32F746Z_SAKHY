/*******************************************************************************
 * Company:     "WSE" LLP
 * Project:     intercom
 * File:        w5500.c
 * Author:      Kostyrev Vitaliy
 * Position:    Embedded Systems Engineer
 *******************************************************************************
 * Version:    V1.0
 * Revision:   R1.0.0
 * *****************************************************************************
 * Description:
 *
 *******************************************************************************
 * Created on:  Apr 1, 2024
 ******************************************************************************/

/******************************* INCLUDES ************************************/
#include "w5500.h"
#include "w5500_port.h"

/*****************************************************************************/
/******************************* DEFINES *************************************/
#define W5500_SPI_VDM_OP 0x00
#define W5500_SPI_FDM_OP_LEN1 0x01
#define W5500_SPI_FDM_OP_LEN2 0x02
#define W5500_SPI_FDM_OP_LEN4 0x03
/*****************************************************************************/
/******************************* VARIABLES ***********************************/
static uint8_t _DNS_[4];         // DNS server ip address
static w5500_dhcp_mode_t _DHCP_; // DHCP mode
/*****************************************************************************/
/******************************* METHODS *************************************/
/**
 * @brief It reads 1 byte value from a register
 * @param addr Register address
 * @return The value of register
 */
uint8_t w5500_read(uint32_t addr)
{
  uint8_t ret;
  uint8_t data[3];

  w5500_critcical_enter();
  w5500_spi_cs_low();

  addr |= (W5500_SPI_READ | W5500_SPI_VDM_OP);

  data[0] = (addr & 0x00FF0000) >> 16;
  data[1] = (addr & 0x0000FF00) >> 8;
  data[2] = (addr & 0x000000FF) >> 0;

  w5500_spi_write(data, 3);
  w5500_spi_read(&ret, 1);

  w5500_spi_cs_hi();
  w5500_critcical_exit();

  return ret;
}

/**
 * @brief It writes 1 byte value to a register
 * @param addr Register address
 * @param wb Write data
 */
void w5500_write(uint32_t addr, uint8_t wb)
{
  uint8_t data[4];

  w5500_critcical_enter();
  w5500_spi_cs_low();

  addr |= (W5500_SPI_WRITE | W5500_SPI_VDM_OP);

  data[0] = (addr & 0x00FF0000) >> 16;
  data[1] = (addr & 0x0000FF00) >> 8;
  data[2] = (addr & 0x000000FF) >> 0;
  data[3] = wb;
  w5500_spi_write(data, 4);

  w5500_spi_cs_hi();
  w5500_critcical_exit();
}

/**
 * @brief It reads sequence data from registers
 * @param addr Register address
 * @param p_buf Pointer buffer to read data
 * @param len Data length
 */
void w5500_read_buf(uint32_t addr, uint8_t *p_buf, uint16_t len)
{
  uint8_t data[3];

  w5500_critcical_enter();
  w5500_spi_cs_low();

  addr |= (W5500_SPI_READ | W5500_SPI_VDM_OP);

  data[0] = (addr & 0x00FF0000) >> 16;
  data[1] = (addr & 0x0000FF00) >> 8;
  data[2] = (addr & 0x000000FF) >> 0;

  w5500_spi_write(data, 3);
  w5500_spi_read(p_buf, len);

  w5500_spi_cs_hi();
  w5500_critcical_exit();
}

/**
 * @brief It writes sequence data to registers
 * @param addr Register address
 * @param p_buf Pointer buffer to write data
 * @param len Data length
 */
void w5500_write_buf(uint32_t addr, uint8_t *p_buf, uint16_t len)
{
  uint8_t data[4];

  w5500_critcical_enter();
  w5500_spi_cs_low();

  addr |= (W5500_SPI_WRITE | W5500_SPI_VDM_OP);

  data[0] = (addr & 0x00FF0000) >> 16;
  data[1] = (addr & 0x0000FF00) >> 8;
  data[2] = (addr & 0x000000FF) >> 0;
  w5500_spi_write(data, 3);
  w5500_spi_write(p_buf, len);

  w5500_spi_cs_hi();
  w5500_critcical_exit();
}

/**
 * @brief Get Sn_TX_FSR register
 * @param sn Socket number. It should be 0 - 7
 * @return Value of Sn_TX_FSR
 */
uint16_t w5500_get_sn_tx_fsr(uint8_t sn)
{
  uint16_t val = 0, val1 = 0;

  do
  {
    val1 = w5500_read(W5500_SOCKETn_REG_TX_FSR_ADDR(sn));
    val1 = (val1 << 8) + w5500_read(W5500_OFFSET_INC(W5500_SOCKETn_REG_TX_FSR_ADDR(sn), 1));
    if (val1 != 0)
    {
      val = w5500_read(W5500_SOCKETn_REG_TX_FSR_ADDR(sn));
      val = (val << 8) + w5500_read(W5500_OFFSET_INC(W5500_SOCKETn_REG_TX_FSR_ADDR(sn), 1));
    }
  } while (val != val1);

  return val;
}

/**
 * @brief Get Sn_RX_RSR register
 * @param sn Socket number. It should be 0 - 7
 * @return Value of @ref Sn_RX_RSR
 */
uint16_t w5500_get_sn_rx_rsr(uint8_t sn)
{
  uint16_t val = 0, val1 = 0;

  do
  {
    val1 = w5500_read(W5500_SOCKETn_REG_RX_RSR_ADDR(sn));
    val1 = (val1 << 8) + w5500_read(W5500_OFFSET_INC(W5500_SOCKETn_REG_RX_RSR_ADDR(sn), 1));
    if (val1 != 0)
    {
      val = w5500_read(W5500_SOCKETn_REG_RX_RSR_ADDR(sn));
      val = (val << 8) + w5500_read(W5500_OFFSET_INC(W5500_SOCKETn_REG_RX_RSR_ADDR(sn), 1));
    }
  } while (val != val1);
  return val;
}

/**
 * @brief It copies data to internal TX memory
 * @param sn Socket number. It should be 0 - 7
 * @param data Pointer buffer to write data
 * @param len Data length
 */
void w5500_send_data(uint8_t sn, uint8_t *data, uint16_t len)
{
  uint16_t ptr = 0;
  uint32_t addr = 0;

  if (len == 0) return;

  ptr = w5500_get_tx_wr(sn);
  addr = ((uint32_t)ptr << 8) + (W5500_TXBUF_BLOCK(sn) << 3);

  w5500_write_buf(addr, data, len);

  ptr += len;
  w5500_set_tx_wr(sn, ptr);
}

/**
 * @brief It copies data to your buffer from internal RX memory
 * @param sn Socket number. It should be 0 - 7
 * @param data Pointer buffer to read data
 * @param len Data length
 */
void w5500_recv_data(uint8_t sn, uint8_t *data, uint16_t len)
{
  uint16_t ptr = 0;
  uint32_t addr = 0;

  if (len == 0) return;

  ptr = w5500_get_rx_rd(sn);
  addr = ((uint32_t)ptr << 8) + (W5500_RXBUF_BLOCK(sn) << 3);

  w5500_read_buf(addr, data, len);
  ptr += len;

  w5500_set_rx_rd(sn, ptr);
}

/**
 * @brief It discard the received data in RX memory
 * @param sn Socket number. It should be 0 - 7
 * @param len Data length
 */
void w5500_recv_ignore(uint8_t sn, uint16_t len)
{
  uint16_t ptr = 0;

  ptr = w5500_get_rx_rd(sn);
  ptr += len;
  w5500_set_rx_rd(sn, ptr);
}

///* Common register I/O function *///

/**
 * @brief Set Mode Register
 * @param data The value to be set
 */
void w5500_set_mr(uint8_t data)
{
  w5500_write(W5500_COMMON_REG_MR_ADDR, data);
}

/**
 * @brief Get Mode Register
 * @return The value of Mode register
 */
uint8_t w5500_get_mr(void)
{
  return w5500_read(W5500_COMMON_REG_MR_ADDR);
}

/**
 * @brief Set gateway IP address
 * @param data Pointer variable to set gateway IP address. It should be allocated 4 bytes
 */
void w5500_set_gar(uint8_t *data)
{
  w5500_write_buf(W5500_COMMON_REG_GAR_ADDR, data, 4);
}

/**
 * @brief Get gateway IP address
 * @param data Pointer variable to get gateway IP address. It should be allocated 4 bytes
 */
void w5500_get_gar(uint8_t *data)
{
  w5500_read_buf(W5500_COMMON_REG_GAR_ADDR, data, 4);
}

/**
 * @brief Set subnet mask address
 * @param data Pointer variable to set subnet mask address. It should be allocated 4 bytes
 */
void w5500_set_subr(uint8_t *data)
{
  w5500_write_buf(W5500_COMMON_REG_SUBR_ADDR, data, 4);
}

/**
 * @brief Get subnet mask address
 * @param data Pointer variable to get subnet mask address. It should be allocated 4 bytes
 */
void w5500_get_subr(uint8_t *data)
{
  w5500_read_buf(W5500_COMMON_REG_SUBR_ADDR, data, 4);
}

/**
 * @brief Set local MAC address
 * @param data Pointer variable to set local MAC address. It should be allocated 6 bytes
 */
void w5500_set_shar(uint8_t *data)
{
  w5500_write_buf(W5500_COMMON_REG_SHAR_ADDR, data, 6);
}

/**
 * @brief Get local MAC address
 * @param data Pointer variable to set local MAC address. It should be allocated 6 bytes
 */
void w5500_get_shar(uint8_t *data)
{
  w5500_read_buf(W5500_COMMON_REG_SHAR_ADDR, data, 6);
}

/**
 * @brief Set local IP address
 * @param data Pointer variable to set local IP address. It should be allocated 4 bytes
 */
void w5500_set_sipr(uint8_t *data)
{
  w5500_write_buf(W5500_COMMON_REG_SIPR_ADDR, data, 4);
}

/**
 * @brief Get local IP address
 * @param data Pointer variable to set local IP address. It should be allocated 4 bytes
 */
void w5500_get_sipr(uint8_t *data)
{
  w5500_read_buf(W5500_COMMON_REG_SIPR_ADDR, data, 4);
}

/**
 * @brief Set INTLEVEL register
 * @param data Value to set INTLEVEL register
 */
void w5500_set_intlevel(uint16_t data)
{
  w5500_write(W5500_COMMON_REG_INTLEVEL_ADDR, (uint8_t)(data >> 8));
  w5500_write(W5500_OFFSET_INC(W5500_COMMON_REG_INTLEVEL_ADDR, 1), (uint8_t)data);
}

/**
 * @brief Get INTLEVEL register
 * @return Value of INTLEVEL register
 */
uint16_t w5500_get_intlevel(void)
{
  return ((uint16_t)(w5500_read(W5500_COMMON_REG_INTLEVEL_ADDR) << 8)) + w5500_read(W5500_OFFSET_INC(W5500_COMMON_REG_INTLEVEL_ADDR, 1));
}

/**
 * @brief Set IR register
 * @param data Value to set IR register
 */
void w5500_set_ir(uint8_t data)
{
  w5500_write(W5500_COMMON_REG_IR_ADDR, (data & 0xF0));
}

/**
 * @brief Get IR register
 * @return Value of IR register
 */
uint8_t w5500_get_ir(void)
{
  return (w5500_read(W5500_COMMON_REG_IR_ADDR) & 0xF0);
}

/**
 * @brief Set IMR register
 * @param data Value to set IMR register
 */
void w5500_set_imr(uint8_t data)
{
  w5500_write(W5500_COMMON_REG_IMR_ADDR, data);
}

/**
 * @brief Get IMR register
 * @return Value of IMR register
 */
uint8_t w5500_get_imr(void)
{
  return w5500_read(W5500_COMMON_REG_IMR_ADDR);
}

/**
 * @brief Set SIR register
 * @param data Value to set SIR register
 */
void w5500_set_sir(uint8_t data)
{
  w5500_write(W5500_COMMON_REG_SIR_ADDR, data);
}

/**
 * @brief Get SIR register
 * @return Value of SIR register
 */
uint8_t w5500_get_sir(void)
{
  return w5500_read(W5500_COMMON_REG_SIR_ADDR);
}

/**
 * @brief Set SIMR register
 * @param data Value to set SIMR register
 */
void w5500_set_simr(uint8_t data)
{
  w5500_write(W5500_COMMON_REG_SIMR_ADDR, data);
}

/**
 * @brief Get SIMR register
 * @return Value of SIMR register
 */
uint8_t w5500_get_simr(void)
{
  return w5500_read(W5500_COMMON_REG_SIMR_ADDR);
}

/**
 * @brief Set RTR register
 * @param data Value to set RTR register
 */
void w5500_set_rtr(uint16_t data)
{
  w5500_write(W5500_COMMON_REG_RTR_ADDR, (uint8_t)(data >> 8));
  w5500_write(W5500_OFFSET_INC(W5500_COMMON_REG_RTR_ADDR, 1), (uint8_t)data);
}

/**
 * @brief Get RTR register
 * @return Value of RTR register
 */
uint16_t w5500_get_rtr(void)
{
  return ((uint16_t)(w5500_read(W5500_COMMON_REG_RTR_ADDR) << 8)) + w5500_read(W5500_OFFSET_INC(W5500_COMMON_REG_RTR_ADDR, 1));
}

/**
 * @brief Set RCR register
 * @param data Value to set RCR register
 */
void w5500_set_rcr(uint8_t data)
{
  w5500_write(W5500_COMMON_REG_RCR_ADDR, data);
}

/**
 * @brief Get RCR register
 * @return Value of RCR register
 */
uint8_t w5500_get_rcr(void)
{
  return w5500_read(W5500_COMMON_REG_RCR_ADDR);
}

/**
 * @brief Set PTIMER register
 * @param data Value to set PTIMER register
 */
void w5500_set_ptimer(uint8_t data)
{
  w5500_write(W5500_COMMON_REG_PTIMER_ADDR, data);
}

/**
 * @brief Get PTIMER register
 * @return Value of PTIMER register
 */
uint8_t w5500_get_ptimer(void)
{
  return w5500_read(W5500_COMMON_REG_PTIMER_ADDR);
}

/**
 * @brief Set PMAGIC register
 * @param data Value to set PMAGIC register
 */
void w5500_set_pmagic(uint8_t data)
{
  w5500_write(W5500_COMMON_REG_PMAGIC_ADDR, data);
}

/**
 * @brief Get PMAGIC register
 * @return Value of PMAGIC register
 */
uint8_t w5500_get_pmagic(void)
{
  return w5500_read(W5500_COMMON_REG_PMAGIC_ADDR);
}

/**
 * @brief
 * @param data
 */
void w5500_set_phar(uint8_t *data)
{
  w5500_write_buf(W5500_COMMON_REG_PHAR_ADDR, data, 6);
}

/**
 * @brief Set PHAR address
 * @param data Pointer variable to set PPP destination MAC register address. It should be allocated 6 bytes
 */
void w5500_get_phar(uint8_t *data)
{
  w5500_read_buf(W5500_COMMON_REG_PHAR_ADDR, data, 6);
}

/**
 * @brief Get PHAR address
 * @param data Pointer variable to PPP destination MAC register address. It should be allocated 6 bytes
 */
void w5500_set_psid(uint16_t data)
{
  w5500_write(W5500_COMMON_REG_PSID_ADDR, (uint8_t)(data >> 8));
  w5500_write(W5500_OFFSET_INC(W5500_COMMON_REG_PSID_ADDR, 1), (uint8_t)data);
}

/**
 * @brief Set PSID register
 * @return Value to set PSID register
 */
uint16_t w5500_get_psid(void)
{
  return ((uint16_t)(w5500_read(W5500_COMMON_REG_PSID_ADDR) << 8)) + w5500_read(W5500_OFFSET_INC(W5500_COMMON_REG_PSID_ADDR, 1));
}

/**
 * @brief Set PMRU register
 * @param data Value to set PMRU register
 */
void w5500_set_pmru(uint16_t data)
{
  w5500_write(W5500_COMMON_REG_PMRU_ADDR, (uint8_t)(data >> 8));
  w5500_write(W5500_OFFSET_INC(W5500_COMMON_REG_PMRU_ADDR, 1), (uint8_t)data);
}

/**
 * @brief Get PMRU register
 * @return Value of PMRU register
 */
uint16_t w5500_get_pmru(void)
{
  return ((uint16_t)(w5500_read(W5500_COMMON_REG_PMRU_ADDR) << 8)) + w5500_read(W5500_OFFSET_INC(W5500_COMMON_REG_PMRU_ADDR, 1));
}

/**
 * @brief  Get unreachable IP address
 * @return Pointer variable to get unreachable IP address. It should be allocated 4 bytes
 */
uint8_t w5500_get_uipr(void)
{
  return w5500_read(W5500_COMMON_REG_UIPR_ADDR);
}

/**
 * @brief Get UPORTR register
 * @return Value of UPORTR register
 */
uint8_t w5500_get_uportr(void)
{
  return w5500_read(W5500_COMMON_REG_UPORTR_ADDR);
}

/**
 * @brief Set PHYCFGR register
 * @param data Value to set PHYCFGR register
 */
void w5500_set_phycfgr(uint8_t data)
{
  w5500_write(W5500_COMMON_REG_PHYCFGR_ADDR, data);
}

/**
 * @brief Get PHYCFGR register
 * @return Value of PHYCFGR register
 */
uint8_t w5500_get_phycfgr(void)
{
  return w5500_read(W5500_COMMON_REG_PHYCFGR_ADDR);
}

/**
 * @brief Get VERSIONR register
 * @return Value of VERSIONR register
 */
uint8_t w5500_get_version(void)
{
  return w5500_read(W5500_COMMON_REG_VERSIONR_ADDR);
}

///* Socket N register I/O function *///

/**
 * @brief Set Sn_MR register
 * @param sn Socket number. It should be 0 - 7
 * @param data Value to set Sn_MR register
 */
void w5500_set_sn_mr(uint8_t sn, uint8_t data)
{
  w5500_write(W5500_SOCKETn_REG_MR_ADDR(sn), data);
}

/**
 * @brief Get Sn_MR register
 * @param sn Socket number. It should be 0 - 7
 * @return Value of Sn_MR register
 */
uint8_t w5500_get_sn_mr(uint8_t sn)
{
  return w5500_read(W5500_SOCKETn_REG_MR_ADDR(sn));
}

/**
 * @brief Set Sn_CR register
 * @param sn Socket number. It should be 0 - 7
 * @param data Value to set Sn_CR register
 */
void w5500_set_sn_cr(uint8_t sn, uint8_t data)
{
  w5500_write(W5500_SOCKETn_REG_CR_ADDR(sn), data);
}

/**
 * @brief Get Sn_CR register
 * @param sn Socket number. It should be 0 - 7
 * @return Value of Sn_CR register
 */
uint8_t w5500_get_sn_cr(uint8_t sn)
{
  return w5500_read(W5500_SOCKETn_REG_CR_ADDR(sn));
}

/**
 * @brief Set Sn_IR register
 * @param sn Socket number. It should be 0 - 7
 * @param data Value to set Sn_IR register
 */
void w5500_set_sn_ir(uint8_t sn, uint8_t data)
{
  w5500_write(W5500_SOCKETn_REG_IR_ADDR(sn), data);
}

/**
 * @brief Get Sn_IR register
 * @param sn Socket number. It should be 0 - 7
 * @return Value of Sn_IR register
 */
uint8_t w5500_get_sn_ir(uint8_t sn)
{
  return w5500_read(W5500_SOCKETn_REG_IR_ADDR(sn));
}

/**
 * @brief Set Sn_IMR register
 * @param sn Socket number. It should be 0 - 7
 * @param data Value to set Sn_IMR register
 */
void w5500_set_sn_imr(uint8_t sn, uint8_t data)
{
  w5500_write(W5500_SOCKETn_REG_IMR_ADDR(sn), data);
}

/**
 * @brief Get Sn_IMR register
 * @param sn Socket number. It should be 0 - 7
 * @return Value of Sn_IMR register
 */
uint8_t w5500_get_sn_imr(uint8_t sn)
{
  return w5500_read(W5500_SOCKETn_REG_IMR_ADDR(sn));
}

/**
 * @brief Get Sn_SR register
 * @param sn Socket number. It should be 0 - 7
 * @return Value of Sn_SR register
 */
uint8_t w5500_get_sn_sr(uint8_t sn)
{
  return w5500_read(W5500_SOCKETn_REG_SR_ADDR(sn));
}

/**
 * @brief Set Sn_PORT register
 * @param sn Socket number. It should be 0 - 7
 * @param data Value to set Sn_PORT register
 */
void w5500_set_sn_port(uint8_t sn, uint16_t data)
{
  w5500_write(W5500_SOCKETn_REG_PORT_ADDR(sn), (uint8_t)(data >> 8));
  w5500_write(W5500_OFFSET_INC(W5500_SOCKETn_REG_PORT_ADDR(sn), 1), (uint8_t)data);
}

/**
 * @brief Get Sn_PORT register
 * @param sn Socket number. It should be 0 - 7
 * @return Value of Sn_PORT register
 */
uint16_t w5500_get_sn_port(uint8_t sn)
{
  return ((uint16_t)(w5500_read(W5500_SOCKETn_REG_PORT_ADDR(sn)) << 8)) + w5500_read(W5500_OFFSET_INC(W5500_SOCKETn_REG_PORT_ADDR(sn), 1));
}

/**
 * @brief Set Sn_DHAR register
 * @param sn Socket number. It should be 0 - 7
 * @param data Value to set Sn_DHAR register
 */
void w5500_set_sn_dhar(uint8_t sn, uint8_t *data)
{
  w5500_write_buf(W5500_SOCKETn_REG_DHAR_ADDR(sn), data, 6);
}

/**
 * @brief Get Sn_DHAR register
 * @param sn Socket number. It should be 0 - 7
 * @param data Value of Sn_DHAR register
 */
void w5500_get_sn_dhar(uint8_t sn, uint8_t *data)
{
  w5500_read_buf(W5500_SOCKETn_REG_DHAR_ADDR(sn), data, 6);
}

/**
 * @brief Set Sn_DIPR register
 * @param sn Socket number. It should be 0 - 7
 * @param data Value to set Sn_DIPR register
 */
void w5500_set_sn_dipr(uint8_t sn, uint8_t *data)
{
  w5500_write_buf(W5500_SOCKETn_REG_DIPR_ADDR(sn), data, 4);
}

/**
 * @brief Get Sn_DIPR register
 * @param sn Socket number. It should be 0 - 7
 * @param data Value of Sn_DIPR register
 */
void w5500_get_sn_dipr(uint8_t sn, uint8_t *data)
{
  w5500_read_buf(W5500_SOCKETn_REG_DIPR_ADDR(sn), data, 4);
}

/**
 * @brief Set Sn_DPORT register
 * @param sn Socket number. It should be 0 - 7
 * @param data Value to set Sn_DPORT register
 */
void w5500_set_sn_dport(uint8_t sn, uint16_t data)
{
  w5500_write(W5500_SOCKETn_REG_DPORT_ADDR(sn), (uint8_t)(data >> 8));
  w5500_write(W5500_OFFSET_INC(W5500_SOCKETn_REG_DPORT_ADDR(sn), 1), (uint8_t)data);
}

/**
 * @brief Get Sn_DPORT register
 * @param sn Socket number. It should be 0 - 7
 * @return Value of Sn_DPORT register
 */
uint16_t w5500_get_sn_dport(uint8_t sn)
{
  return ((uint16_t)(w5500_read(W5500_SOCKETn_REG_DPORT_ADDR(sn)) << 8)) +
         w5500_read(W5500_OFFSET_INC(W5500_SOCKETn_REG_DPORT_ADDR(sn), 1));
}

/**
 * @brief Set Sn_MSSR register
 * @param sn Socket number. It should be 0 - 7
 * @param data Value to set Sn_MSSR register
 */
void w5500_set_sn_mssr(uint8_t sn, uint16_t data)
{
  w5500_write(W5500_SOCKETn_REG_MSSR_ADDR(sn), (uint8_t)(data >> 8));
  w5500_write(W5500_OFFSET_INC(W5500_SOCKETn_REG_MSSR_ADDR(sn), 1), (uint8_t)data);
}

/**
 * @brief Get Sn_MSSR register
 * @param sn Socket number. It should be 0 - 7
 * @return Value of Sn_MSSR register
 */
uint16_t w5500_get_sn_mssr(uint8_t sn)
{
  return ((uint16_t)(w5500_read(W5500_SOCKETn_REG_MSSR_ADDR(sn)) << 8)) + w5500_read(W5500_OFFSET_INC(W5500_SOCKETn_REG_MSSR_ADDR(sn), 1));
}

/**
 * @brief Set Sn_TOS register
 * @param sn Socket number. It should be 0 - 7
 * @param data Value to set Sn_TOS register
 */
void w5500_set_sn_tos(uint8_t sn, uint8_t data)
{
  w5500_write(W5500_SOCKETn_REG_TOS_ADDR(sn), data);
}

/**
 * @brief Get Sn_TOS register
 * @param sn Socket number. It should be 0 - 7
 * @return Value of Sn_TOS register
 */
uint8_t w5500_get_sn_tos(uint8_t sn)
{
  return w5500_read(W5500_SOCKETn_REG_TOS_ADDR(sn));
}

/**
 * @brief Set Sn_TTL register
 * @param sn Socket number. It should be 0 - 7
 * @param data Value to set Sn_TTL register
 */
void w5500_set_sn_ttl(uint8_t sn, uint8_t data)
{
  w5500_write(W5500_SOCKETn_REG_TTL_ADDR(sn), data);
}

/**
 * @brief Get Sn_TTL register
 * @param sn Socket number. It should be 0 - 7
 * @return  Value of Sn_TTL register
 */
uint8_t w5500_get_sn_ttl(uint8_t sn)
{
  return w5500_read(W5500_SOCKETn_REG_TTL_ADDR(sn));
}

/**
 * @brief Set Sn_RXBUF_SIZE register
 * @param sn Socket number. It should be 0 - 7
 * @param data Value to set Sn_RXBUF_SIZE register
 */
void w5500_set_sn_rxbuf_size(uint8_t sn, uint8_t data)
{
  w5500_write(W5500_SOCKETn_REG_RXBUF_SIZE_ADDR(sn), data);
}

/**
 * @brief Get Sn_RXBUF_SIZE register
 * @param sn Socket number. It should be 0 - 7
 * @return Value of Sn_RXBUF_SIZE register
 */
uint8_t w5500_get_sn_rxbuf_size(uint8_t sn)
{
  return w5500_read(W5500_SOCKETn_REG_RXBUF_SIZE_ADDR(sn));
}

/**
 * @brief Set Sn_TXBUF_SIZE register
 * @param sn Socket number. It should be 0 - 7
 * @param data Value to set Sn_TXBUF_SIZE register
 */
void w5500_set_sn_txbuf_size(uint8_t sn, uint8_t data)
{
  w5500_write(W5500_SOCKETn_REG_TXBUF_SIZE_ADDR(sn), data);
}

/**
 * @brief Get Sn_TXBUF_SIZE register
 * @param sn Socket number. It should be 0 - 7
 * @return Value of Sn_TXBUF_SIZE register
 */
uint8_t w5500_get_sn_txbuf_size(uint8_t sn)
{
  return w5500_read(W5500_SOCKETn_REG_TXBUF_SIZE_ADDR(sn));
}

/**
 * @brief Get Sn_TX_RD register
 * @param sn Socket number. It should be 0 - 7
 * @return Value of Sn_TX_RD register
 */
uint16_t w5500_get_tx_rd(uint8_t sn)
{
  return ((uint16_t)(w5500_read(W5500_SOCKETn_REG_TX_RD_ADDR(sn)) << 8)) +
         w5500_read(W5500_OFFSET_INC(W5500_SOCKETn_REG_TX_RD_ADDR(sn), 1));
}

/**
 * @brief Set Sn_TX_WR register
 * @param sn Socket number. It should be 0 - 7
 * @param data Value to set Sn_TX_WR register
 */
void w5500_set_tx_wr(uint8_t sn, uint16_t data)
{
  w5500_write(W5500_SOCKETn_REG_TX_WR_ADDR(sn), (uint8_t)(data >> 8));
  w5500_write(W5500_OFFSET_INC(W5500_SOCKETn_REG_TX_WR_ADDR(sn), 1), (uint8_t)data);
}

/**
 * @brief Get Sn_TX_WR register
 * @param sn Socket number. It should be 0 - 7
 * @return Value of Sn_TX_WR register
 */
uint16_t w5500_get_tx_wr(uint8_t sn)
{
  return ((uint16_t)(w5500_read(W5500_SOCKETn_REG_TX_WR_ADDR(sn)) << 8)) +
         w5500_read(W5500_OFFSET_INC(W5500_SOCKETn_REG_TX_WR_ADDR(sn), 1));
}

/**
 * @brief Set Sn_RX_RD register
 * @param sn Socket number. It should be 0 - 7
 * @param data Value to set Sn_RX_RD register
 */
void w5500_set_rx_rd(uint8_t sn, uint16_t data)
{
  w5500_write(W5500_SOCKETn_REG_RX_RD_ADDR(sn), (uint8_t)(data >> 8));
  w5500_write(W5500_OFFSET_INC(W5500_SOCKETn_REG_RX_RD_ADDR(sn), 1), (uint8_t)data);
}

/**
 * @brief Get Sn_RX_RD register
 * @param sn Socket number. It should be 0 - 7
 * @return Value of Sn_RX_RD register
 */
uint16_t w5500_get_rx_rd(uint8_t sn)
{
  return ((uint16_t)(w5500_read(W5500_SOCKETn_REG_RX_RD_ADDR(sn)) << 8)) +
         w5500_read(W5500_OFFSET_INC(W5500_SOCKETn_REG_RX_RD_ADDR(sn), 1));
}

/**
 * @brief Get Sn_RX_WR register
 * @param sn Socket number. It should be 0 - 7
 * @return Value of Sn_RX_WR register
 */
uint16_t w5500_get_rx_wr(uint8_t sn)
{
  return ((uint16_t)(w5500_read(W5500_SOCKETn_REG_RX_WR_ADDR(sn)) << 8)) +
         w5500_read(W5500_OFFSET_INC(W5500_SOCKETn_REG_RX_WR_ADDR(sn), 1));
}

/**
 * @brief Set Sn_FRAG register
 * @param sn Socket number. It should be 0 - 7
 * @param data Value to set Sn_FRAG register
 */
void w5500_set_frag(uint8_t sn, uint16_t data)
{
  w5500_write(W5500_SOCKETn_REG_FRAG_ADDR(sn), (uint8_t)(data >> 8));
  w5500_write(W5500_OFFSET_INC(W5500_SOCKETn_REG_FRAG_ADDR(sn), 1), (uint8_t)data);
}

/**
 * @brief Get Sn_FRAG register
 * @param sn Socket number. It should be 0 - 7
 * @return Value of Sn_FRAG
 */
uint16_t w5500_get_frag(uint8_t sn)
{
  return ((uint16_t)(w5500_read(W5500_SOCKETn_REG_FRAG_ADDR(sn)) << 8)) + w5500_read(W5500_OFFSET_INC(W5500_SOCKETn_REG_FRAG_ADDR(sn), 1));
}

/**
 * @brief Set Sn_KPALVTR register
 * @param sn Socket number. It should be 0 - 7
 * @param data Value to set Sn_KPALVTR register
 */
void w5500_set_sn_kpalvtr(uint8_t sn, uint8_t data)
{
  w5500_write(W5500_SOCKETn_REG_KPALVTR_ADDR(sn), data);
}

/**
 * @brief Get Sn_KPALVTR register
 * @param sn Socket number. It should be 0 - 7
 * @return Value of Sn_KPALVTR
 */
uint8_t w5500_get_sn_kpalvtr(uint8_t sn)
{
  return w5500_read(W5500_SOCKETn_REG_KPALVTR_ADDR(sn));
}

/**
 *
 * @brief Gets the max buffer size of socket sn passed as parameter
 * @param sn Socket number. It should be 0 - 7
 * @return Value of Socket n RX max buffer size
 */
uint16_t w5500_get_sn_rx_max(uint8_t sn)
{
  return (((uint16_t)w5500_get_sn_rxbuf_size(sn)) << 10);
}

/**
 * @brief Gets the max buffer size of socket sn passed as parameters
 * @param sn Socket number. It should be 0 - 7
 * @return Value of Socket n TX max buffer size
 */
uint16_t w5500_get_sn_tx_max(uint8_t sn)
{
  return (((uint16_t)w5500_get_sn_txbuf_size(sn)) << 10);
}

///* Configuration functions *///
/**
 * @brief Reset W5500 by softly
 */
void w5500_sw_reset(void)
{
  uint8_t gw[4], sn[4], sip[4];
  uint8_t mac[6];

  w5500_get_shar(mac); // get Source Hardware Address Register
  w5500_get_gar(gw);   // get Gateway IP Address Register
  w5500_get_subr(sn);  // get Subnet Mask Register
  w5500_get_sipr(sip); // get Source IP Address Register

  w5500_set_mr(MR_RST); // reset chip
  w5500_get_mr();       // for delay

  w5500_set_shar(mac); // set Source Hardware Address Register
  w5500_set_gar(gw);   // set Gateway IP Address Register
  w5500_set_subr(sn);  // set Subnet Mask Register
  w5500_set_sipr(sip); // set Source IP Address Register
}

/**
 * @brief Initializes W5500 with socket buffer size
 * @param txsize Socket tx buffer sizes
 * @param rxsize Socket rx buffer sizes
 */
void w5500_init(uint8_t *txsize, uint8_t *rxsize)
{
  w5500_sw_reset();

  for (uint8_t i = 0; i < W5500_SOCK_NUM; i++)
  {
    w5500_set_sn_txbuf_size(i, txsize[i]);
    w5500_set_sn_rxbuf_size(i, rxsize[i]);
  }
}

/**
 * @brief Clear Interrupt of W5500
 * @param intr w5500_interupt_t value operated OR. It can type-cast to uint16_t
 */
void w5500_clear_interrupt(w5500_interrupt_t intr)
{
  uint8_t ir = (uint8_t)intr;
  uint8_t sir = (uint8_t)((uint16_t)intr >> 8);

  w5500_set_ir(ir);
  for (ir = 0; ir < 8; ir++)
  {
    if (sir & (0x01 << ir))
    {
      w5500_set_sn_ir(ir, 0xff);
    }
  }
}

/**
 * @brief Get Interrupt of W5500
 * @return w5500_interupt_t value operated OR. It can type-cast to uint16_t.
 */
w5500_interrupt_t w5500_get_interrupt(void)
{
  uint8_t ir = 0;
  uint8_t sir = 0;
  uint16_t ret = 0;

  ir = w5500_get_ir();
  sir = w5500_get_sir();

  ret = sir;
  ret = (ret << 8) + ir;

  return (w5500_interrupt_t)ret;
}

/**
 * @brief Mask or Unmask Interrupt of W5500
 * @param intr w5500_interupt_t value operated OR. It can type-cast to uint16_t
 */
void w5500_set_interrupt_mask(w5500_interrupt_t intr)
{
  uint8_t imr = (uint8_t)intr;
  uint8_t simr = (uint8_t)((uint16_t)intr >> 8);

  w5500_set_imr(imr);
  w5500_set_simr(simr);
}

/**
 * @brief Get Interrupt mask of W5500
 * @return The operated OR value of w5500_interupt_t. It can type-cast to uint16_t
 */
w5500_interrupt_t w5500_get_interrupt_mask(void)
{
  uint8_t imr = 0;
  uint8_t simr = 0;
  uint16_t ret = 0;

  imr = w5500_get_imr();
  simr = w5500_get_simr();

  ret = simr;
  ret = (ret << 8) + imr;

  return (w5500_interrupt_t)ret;
}

/**
 * @brief Get the link status of phy in W5500
 * @return PHY_LINK_ON or PHY_LINK_OFF
 */
int8_t w5500_get_phylink(void)
{
  int8_t link = PHY_LINK_OFF;

  if (w5500_get_phycfgr() & PHYCFGR_LNK_ON)
  {
    link = PHY_LINK_ON;
  }

  return link;
}

/**
 * @brief
 * @return
 */
int8_t w5500_get_phymode(void)
{
  int8_t mode = 0;

  if ((w5500_get_phycfgr() & PHYCFGR_OPMDC_ALLA) == PHYCFGR_OPMDC_PDOWN)
  {
    mode = PHY_POWER_DOWN;
  }
  else
  {
    mode = PHY_POWER_NORM;
  }

  return mode;
}

/**
 * @brief Reset phy
 */
void w5500_phy_reset(void)
{
  uint8_t phy = 0;

  phy = w5500_get_phycfgr();
  phy &= PHYCFGR_RST;
  w5500_set_phycfgr(phy);
  phy = w5500_get_phycfgr();
  phy |= ~PHYCFGR_RST;
  w5500_set_phycfgr(phy);
}

/**
 * @brief Set the phy information for W5500 without power mode
 * @param phyconf w5500_phyconf_t
 */
void w5500_set_phyconf(w5500_phyconf_t *phyconf)
{
  uint8_t phy = 0;
  if (phyconf->confby == PHY_CONFBY_SW)
  {
    phy |= PHYCFGR_OPMD;
  }
  else
  {
    phy &= ~PHYCFGR_OPMD;
  }
  if (phyconf->mode == PHY_MODE_AUTONEGO)
  {
    phy |= PHYCFGR_OPMDC_ALLA;
  }
  else
  {
    if (phyconf->duplex == PHY_DUPLEX_FULL)
    {
      if (phyconf->speed == PHY_SPEED_100)
      {
        phy |= PHYCFGR_OPMDC_100F;
      }
      else
      {
        phy |= PHYCFGR_OPMDC_10F;
      }
    }
    else
    {
      if (phyconf->speed == PHY_SPEED_100)
      {
        phy |= PHYCFGR_OPMDC_100H;
      }
      else
      {
        phy |= PHYCFGR_OPMDC_10H;
      }
    }
  }
  w5500_set_phycfgr(phy);
  w5500_phy_reset();
}

/**
 * @brief Get phy configuration information
 * @param phyconf w5500_phyconf_t
 */
void w5500_get_phyconf(w5500_phyconf_t *phyconf)
{
  uint8_t phy = 0;

  phy = w5500_get_phycfgr();
  phyconf->confby = (phy & PHYCFGR_OPMD) ? PHY_CONFBY_SW : PHY_CONFBY_HW;
  switch (phy & PHYCFGR_OPMDC_ALLA)
  {
  case PHYCFGR_OPMDC_ALLA:
  case PHYCFGR_OPMDC_100FA: phyconf->mode = PHY_MODE_AUTONEGO; break;
  default: phyconf->mode = PHY_MODE_MANUAL; break;
  }
  switch (phy & PHYCFGR_OPMDC_ALLA)
  {
  case PHYCFGR_OPMDC_100FA:
  case PHYCFGR_OPMDC_100F:
  case PHYCFGR_OPMDC_100H: phyconf->speed = PHY_SPEED_100; break;
  default: phyconf->speed = PHY_SPEED_10; break;
  }
  switch (phy & PHYCFGR_OPMDC_ALLA)
  {
  case PHYCFGR_OPMDC_100FA:
  case PHYCFGR_OPMDC_100F:
  case PHYCFGR_OPMDC_10F: phyconf->duplex = PHY_DUPLEX_FULL; break;
  default: phyconf->duplex = PHY_DUPLEX_HALF; break;
  }
}

/**
 * @brief Get phy status
 * @param phyconf w5500_phyconf_t
 */
void w5500_get_phystat(w5500_phyconf_t *phyconf)
{
  uint8_t phy = 0;

  phy = w5500_get_phycfgr();
  phyconf->duplex = (phy & PHYCFGR_DPX_FULL) ? PHY_DUPLEX_FULL : PHY_DUPLEX_HALF;
  phyconf->speed = (phy & PHYCFGR_SPD_100) ? PHY_SPEED_100 : PHY_SPEED_10;
}

/**
 * @brief Set the power mode of phy inside W5500
 * @param pmode Setting value of power down mode
 * @return Execution result
 */
int8_t w5500_set_phypmode(uint8_t pmode)
{
  uint8_t phy = 0;

  phy = w5500_get_phycfgr();
  if ((phy & PHYCFGR_OPMD) == 0)
  {
    return -1;
  }
  phy &= ~PHYCFGR_OPMDC_ALLA;
  if (pmode == PHY_POWER_DOWN)
  {
    phy |= PHYCFGR_OPMDC_PDOWN;
  }
  else
  {
    phy |= PHYCFGR_OPMDC_ALLA;
  }
  w5500_set_phycfgr(phy);
  w5500_phy_reset();
  phy = w5500_get_phycfgr();
  if (pmode == PHY_POWER_DOWN)
  {
    if (phy & PHYCFGR_OPMDC_PDOWN)
    {
      return 0;
    }
  }
  else
  {
    if (phy & PHYCFGR_OPMDC_ALLA)
    {
      return 0;
    }
  }
  return -1;
}

/**
 * @brief Set the network information for W5500
 * @param pnetinfo w5500_net_info_t
 */
void w5500_set_netinfo(w5500_net_info_t *pnetinfo)
{
  w5500_set_shar(pnetinfo->mac);
  w5500_set_gar(pnetinfo->gw);
  w5500_set_subr(pnetinfo->sn);
  w5500_set_sipr(pnetinfo->ip);
  _DNS_[0] = pnetinfo->dns[0];
  _DNS_[1] = pnetinfo->dns[1];
  _DNS_[2] = pnetinfo->dns[2];
  _DNS_[3] = pnetinfo->dns[3];
  _DHCP_ = pnetinfo->dhcp;
}

/**
 * @brief Get the network information for W5500
 * @param pnetinfo w5500_net_info_t
 */
void w5500_get_netinfo(w5500_net_info_t *pnetinfo)
{
  w5500_get_shar(pnetinfo->mac);
  w5500_get_gar(pnetinfo->gw);
  w5500_get_subr(pnetinfo->sn);
  w5500_get_sipr(pnetinfo->ip);
  pnetinfo->dns[0] = _DNS_[0];
  pnetinfo->dns[1] = _DNS_[1];
  pnetinfo->dns[2] = _DNS_[2];
  pnetinfo->dns[3] = _DNS_[3];
  pnetinfo->dhcp = _DHCP_;
}

/**
 * @brief Set the network mode such WOL, PPPoE, Ping Block, and etc
 * @param netmode w5500_netmode_t
 * @return
 */
int8_t w5500_set_netmode(w5500_netmode_t netmode)
{
  uint8_t net = 0;

  if (netmode & ~(NM_WAKEONLAN | NM_PPPOE | NM_PINGBLOCK))
  {
    return -1;
  }
  net = w5500_get_mr();
  net |= (uint8_t)netmode;
  w5500_set_mr(net);

  return 0;
}

/**
 * @brief Get the network mode such WOL, PPPoE, Ping Block, and etc
 * @return Value of network mode. Refer to w5500_netmode_t
 */
w5500_netmode_t w5500_get_netmode(void)
{
  return (w5500_netmode_t)w5500_get_mr();
}

/**
 * @brief Set retry time value(@ref _RTR_) and retry count
 * @param nettime w5500_net_timeout_t
 */
void w5500_set_timeout(w5500_net_timeout_t *nettime)
{
  w5500_set_rcr(nettime->retry_cnt);
  w5500_set_rtr(nettime->time_100us);
}

/**
 * @brief Get retry time value(@ref _RTR_) and retry count
 * @param nettime w5500_net_timeout_t
 */
void w5500__get_timeout(w5500_net_timeout_t *nettime)
{
  nettime->retry_cnt = w5500_get_rcr();
  nettime->time_100us = w5500_get_rtr();
}
/*****************************************************************************/
