/*******************************************************************************
 * Company:     "WSE" LLP
 * Project:     intercom
 * File:        udp.c
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
#include "ethernet.h"
#include "device.h"
#include <string.h>

/*****************************************************************************/
/******************************* DEFINES *************************************/
#define MAX_DHCP_RET 3
#define DHCP_SOCKET 1
#define UDP_PORT 50000

#define DATA_BUF_SIZE 2048
/*****************************************************************************/
/******************************* VARIABLES ***********************************/
uint8_t ip_dest[4] = {192, 168, 0, 100};
w5500_net_info_t w5500_net_info = {
    .ip = {192, 168, 0, 100},
    .gw = {192, 168, 0, 1},
    .mac = {0x02, 0x00, 0x5E, 0x10, 0x00, 0x01},
    .sn = {255, 255, 255, 0},
};
uint8_t mac_group[6];
uint8_t buf_rx[2048];
uint8_t tx_buf_size[W5500_SOCK_NUM] = {2, 2, 2, 2, 2, 2, 2, 2}; // All sockets with 16 kB TX buffer size
uint8_t rx_buf_size[W5500_SOCK_NUM] = {2, 2, 2, 2, 2, 2, 2, 2}; // All sockets with 16 kB RX buffer size
/*****************************************************************************/
/***************************** PROTOTYPES ************************************/
/*****************************************************************************/
/******************************* METHODS *************************************/
__WEAK void ethernet_delay(uint32_t ms)
{
  (void)ms;
}
/**
 * @brief Initializes the Ethernet communication using the W5500 module.
 * @param net Pointer to network information structure.
 * @param mul_ip Pointer to multicast IP address.
 * @param mul_port Multicast port number.
 * @param port Local port number.
 */
void ethernet_init(void)
{
  do
  {
    // Reset hw w5500
    SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR5); // W5500 pin reset to low state
    ethernet_delay(100);
    SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS5); // W5500 pin reset to high state
    ethernet_delay(2000);
  } while (w5500_get_phylink() != PHY_LINK_ON);

  // Perform software reset and initialize W5500 module with buffer sizes
  w5500_init(tx_buf_size, rx_buf_size);

  // Set MAC address and configure EXTI interrupt priority for W5500 interrupt pin
  if (dev_addr_get() == DEV_ADDR_A) // DEV_ADDR_A
  {
    w5500_net_info.ip[3] = 101;
    w5500_net_info.mac[5] = 101;
    ip_dest[3] = 102;
  }
  else // DEV_ADDR_B
  {
    w5500_net_info.ip[3] = 102;
    w5500_net_info.mac[5] = 102;
    ip_dest[3] = 101;
  }
  w5500_set_shar(w5500_net_info.mac);
  // Configure network settings
  w5500_set_sipr(w5500_net_info.ip);
  w5500_set_gar(w5500_net_info.gw);
  w5500_set_subr(w5500_net_info.sn);
}

/**
 * @brief Callback function for receiving data from a multicast socket.
 *
 * This weakly linked function is intended to be overridden by the user
 * to handle data received from a multicast socket. The default implementation
 * does nothing and should be replaced in the user's code with an appropriate
 * handler.
 *
 * @param buf Pointer to the buffer containing the received data.
 * @param len Length of the received data in bytes.
 */
__WEAK void multicast_socket_receive_cb(uint8_t *buf, uint16_t len)
{
  // Suppress compiler warnings for unused parameters.
  (void)buf;
  (void)len;

  // Default implementation does nothing.
  // User should provide their own implementation to handle the received data.
}

/**
 * @brief Opens the multicast socket.
 *
 * This function opens the multicast socket identified by MULTICAST_SOCKET,
 * configures it for UDP communication with multicast IP and port, sets up
 * necessary configurations including multicast MAC address, socket mode,
 * and interrupt masks.
 */
void udp_socket_open(void)
{
  int32_t ret;

  if (w5500_get_sn_sr(UDP_SOCKET) == SOCK_CLOSED)
  {
    if ((ret = socket(UDP_SOCKET, Sn_MR_UDP, UDP_PORT, 0x00)) != UDP_SOCKET)
    {
    }
    else
    {
      w5500_set_simr(1 << UDP_SOCKET);
      w5500_set_sn_imr(UDP_SOCKET, SOCKET_INT_RECV);
      w5500_set_sn_ir(UDP_SOCKET, SOCKET_INT_RECV);
    }
  }
}

/**
 * @brief Closes the multicast socket.
 *
 * This function closes the multicast socket identified by MULTICAST_SOCKET.
 * It terminates any active connections or listening on the socket.
 */
void multicast_socket_close(void)
{
  close(UDP_SOCKET);
}

/**
 * @brief Sends data over a multicast socket.
 *
 * This function sends data to a predefined multicast IP and port
 * if DHCP is enabled in the network configuration.
 *
 * @param data Pointer to the data to be sent.
 * @param len Length of the data to be sent.
 */
void udp_socket_send(uint8_t *data, uint16_t len)
{
  if (w5500_get_phylink() == PHY_LINK_ON)
  {
    // Check if DHCP is enabled in the network info
    if (w5500_get_sn_sr(UDP_SOCKET) == SOCK_UDP)
    {
      // Send data to the multicast IP and port
      sendto(UDP_SOCKET, data, len, ip_dest, UDP_PORT);
    }
  }
}

/**
 * @brief
 * @param data
 * @param len
 */
uint16_t udp_socket_receive(uint8_t *data)
{
  uint16_t pack_len = 0;
  uint16_t size = 0;
  uint8_t destip[4];
  uint16_t destport;

  if ((size = w5500_get_sn_rx_rsr(UDP_SOCKET)) > 0)
  {
    if (size > DATA_BUF_SIZE)
    {
      size = DATA_BUF_SIZE;
    }
    pack_len = recvfrom(UDP_SOCKET, data, size, destip, (uint16_t *)&destport);
  }

  w5500_set_sn_ir(UDP_SOCKET, SOCKET_INT_RECV);

  return pack_len;
}

/**
 * @brief
 * @param sn
 * @param port
 * @return
 */
int32_t udps_socket_process(uint8_t sn, uint16_t port)
{
  int32_t ret;
  //  uint16_t size = 0;
  //  uint16_t pack_len;
  //  uint8_t destip[4];
  //  uint16_t destport;

  switch (w5500_get_sn_sr(sn))
  {
  case SOCK_UDP: break;
  case SOCK_CLOSED:

    if ((ret = socket(sn, Sn_MR_UDP, port, 0x00)) != sn)
    {
      return ret;
    }
    //    w5500_set_simr(1 << sn);
    //    w5500_set_sn_imr(sn, SOCKET_INT_RECV);
    //    w5500_set_sn_ir(sn, SOCKET_INT_RECV);

    break;
  default:
    if ((ret = close(sn)) != SOCK_OK)
    {
      return ret;
    }

    break;
  }
  return 1;
}

/*****************************************************************************/
