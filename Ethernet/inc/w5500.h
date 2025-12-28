/*******************************************************************************
 * Company:     "WSE" LLP
 * Project:     intercom
 * File:        w5500.h
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
#ifndef INC_W5500_H_
#define INC_W5500_H_

#ifdef __cplusplus
extern "C"
{
#endif
/******************************* INCLUDES ************************************/
#include <stdint.h>
/*****************************************************************************/
/******************************* DEFINES *************************************/
#define W5500_CHIP_VERSION 0x04

#define W5500_IO_BASE 0x00000000

#define W5500_SPI_READ (0x00 << 2)  ///< SPI interface Read operation in Control Phase
#define W5500_SPI_WRITE (0x01 << 2) ///< SPI interface Write operation in Control Phase

#define W5500_CREG_BLOCK 0x00            ///< Common register block
#define W5500_SREG_BLOCK(N) (1 + 4 * N)  ///< Socket N register block
#define W5500_TXBUF_BLOCK(N) (2 + 4 * N) ///< Socket N Tx buffer address block
#define W5500_RXBUF_BLOCK(N) (3 + 4 * N) /// Socket N Rx buffer address block

#define W5500_OFFSET_INC(ADDR, N) (ADDR + (N << 8)) ///< Increase offset address

#define W5500_SOCK_NUM 8 ///< Number of sockets supported

  ///*----------------------------- W5500 Common Registers -----------------------------*///

#define W5500_COMMON_REG_MR_ADDR (W5500_IO_BASE + (0x0000 << 8) + (W5500_CREG_BLOCK << 3))
#define W5500_COMMON_REG_GAR_ADDR (W5500_IO_BASE + (0x0001 << 8) + (W5500_CREG_BLOCK << 3))
#define W5500_COMMON_REG_SUBR_ADDR (W5500_IO_BASE + (0x0005 << 8) + (W5500_CREG_BLOCK << 3))
#define W5500_COMMON_REG_SHAR_ADDR (W5500_IO_BASE + (0x0009 << 8) + (W5500_CREG_BLOCK << 3))
#define W5500_COMMON_REG_SIPR_ADDR (W5500_IO_BASE + (0x000F << 8) + (W5500_CREG_BLOCK << 3))
#define W5500_COMMON_REG_INTLEVEL_ADDR (W5500_IO_BASE + (0x0013 << 8) + (W5500_CREG_BLOCK << 3))
#define W5500_COMMON_REG_IR_ADDR (W5500_IO_BASE + (0x0015 << 8) + (W5500_CREG_BLOCK << 3))
#define W5500_COMMON_REG_IMR_ADDR (W5500_IO_BASE + (0x0016 << 8) + (W5500_CREG_BLOCK << 3))
#define W5500_COMMON_REG_SIR_ADDR (W5500_IO_BASE + (0x0017 << 8) + (W5500_CREG_BLOCK << 3))
#define W5500_COMMON_REG_SIMR_ADDR (W5500_IO_BASE + (0x0018 << 8) + (W5500_CREG_BLOCK << 3))
#define W5500_COMMON_REG_RTR_ADDR (W5500_IO_BASE + (0x0019 << 8) + (W5500_CREG_BLOCK << 3))
#define W5500_COMMON_REG_RCR_ADDR (W5500_IO_BASE + (0x001B << 8) + (W5500_CREG_BLOCK << 3))
#define W5500_COMMON_REG_PTIMER_ADDR (W5500_IO_BASE + (0x001C << 8) + (W5500_CREG_BLOCK << 3))
#define W5500_COMMON_REG_PMAGIC_ADDR (W5500_IO_BASE + (0x001D << 8) + (W5500_CREG_BLOCK << 3))
#define W5500_COMMON_REG_PHAR_ADDR (W5500_IO_BASE + (0x001E << 8) + (W5500_CREG_BLOCK << 3))
#define W5500_COMMON_REG_PSID_ADDR (W5500_IO_BASE + (0x0024 << 8) + (W5500_CREG_BLOCK << 3))
#define W5500_COMMON_REG_PMRU_ADDR (W5500_IO_BASE + (0x0026 << 8) + (W5500_CREG_BLOCK << 3))
#define W5500_COMMON_REG_UIPR_ADDR (W5500_IO_BASE + (0x0028 << 8) + (W5500_CREG_BLOCK << 3))
#define W5500_COMMON_REG_UPORTR_ADDR (W5500_IO_BASE + (0x002C << 8) + (W5500_CREG_BLOCK << 3))
#define W5500_COMMON_REG_PHYCFGR_ADDR (W5500_IO_BASE + (0x002E << 8) + (W5500_CREG_BLOCK << 3))
#define W5500_COMMON_REG_VERSIONR_ADDR (W5500_IO_BASE + (0x0039 << 8) + (W5500_CREG_BLOCK << 3))

  ///*----------------------------- W5500 Socket Registers -----------------------------*///

#define W5500_SOCKETn_REG_MR_ADDR(N) (W5500_IO_BASE + (0x0000 << 8) + (W5500_SREG_BLOCK(N) << 3))
#define W5500_SOCKETn_REG_CR_ADDR(N) (W5500_IO_BASE + (0x0001 << 8) + (W5500_SREG_BLOCK(N) << 3))
#define W5500_SOCKETn_REG_IR_ADDR(N) (W5500_IO_BASE + (0x0002 << 8) + (W5500_SREG_BLOCK(N) << 3))
#define W5500_SOCKETn_REG_SR_ADDR(N) (W5500_IO_BASE + (0x0003 << 8) + (W5500_SREG_BLOCK(N) << 3))
#define W5500_SOCKETn_REG_PORT_ADDR(N) (W5500_IO_BASE + (0x0004 << 8) + (W5500_SREG_BLOCK(N) << 3))
#define W5500_SOCKETn_REG_DHAR_ADDR(N) (W5500_IO_BASE + (0x0006 << 8) + (W5500_SREG_BLOCK(N) << 3))
#define W5500_SOCKETn_REG_DIPR_ADDR(N) (W5500_IO_BASE + (0x000C << 8) + (W5500_SREG_BLOCK(N) << 3))
#define W5500_SOCKETn_REG_DPORT_ADDR(N) (W5500_IO_BASE + (0x0010 << 8) + (W5500_SREG_BLOCK(N) << 3))
#define W5500_SOCKETn_REG_MSSR_ADDR(N) (W5500_IO_BASE + (0x0012 << 8) + (W5500_SREG_BLOCK(N) << 3))
#define W5500_SOCKETn_REG_TOS_ADDR(N) (W5500_IO_BASE + (0x0015 << 8) + (W5500_SREG_BLOCK(N) << 3))
#define W5500_SOCKETn_REG_TTL_ADDR(N) (W5500_IO_BASE + (0x0016 << 8) + (W5500_SREG_BLOCK(N) << 3))
#define W5500_SOCKETn_REG_RXBUF_SIZE_ADDR(N) (W5500_IO_BASE + (0x001E << 8) + (W5500_SREG_BLOCK(N) << 3))
#define W5500_SOCKETn_REG_TXBUF_SIZE_ADDR(N) (W5500_IO_BASE + (0x001F << 8) + (W5500_SREG_BLOCK(N) << 3))
#define W5500_SOCKETn_REG_TX_FSR_ADDR(N) (W5500_IO_BASE + (0x0020 << 8) + (W5500_SREG_BLOCK(N) << 3))
#define W5500_SOCKETn_REG_TX_RD_ADDR(N) (W5500_IO_BASE + (0x0022 << 8) + (W5500_SREG_BLOCK(N) << 3))
#define W5500_SOCKETn_REG_TX_WR_ADDR(N) (W5500_IO_BASE + (0x0024 << 8) + (W5500_SREG_BLOCK(N) << 3))
#define W5500_SOCKETn_REG_RX_RSR_ADDR(N) (W5500_IO_BASE + (0x0026 << 8) + (W5500_SREG_BLOCK(N) << 3))
#define W5500_SOCKETn_REG_RX_RD_ADDR(N) (W5500_IO_BASE + (0x0028 << 8) + (W5500_SREG_BLOCK(N) << 3))
#define W5500_SOCKETn_REG_RX_WR_ADDR(N) (W5500_IO_BASE + (0x002A << 8) + (W5500_SREG_BLOCK(N) << 3))
#define W5500_SOCKETn_REG_IMR_ADDR(N) (W5500_IO_BASE + (0x002C << 8) + (W5500_SREG_BLOCK(N) << 3))
#define W5500_SOCKETn_REG_FRAG_ADDR(N) (W5500_IO_BASE + (0x002D << 8) + (W5500_SREG_BLOCK(N) << 3))
#define W5500_SOCKETn_REG_KPALVTR_ADDR(N) (W5500_IO_BASE + (0x002F << 8) + (W5500_SREG_BLOCK(N) << 3))

///*----------------------------- W5500 Register values  -----------------------------*///
///* Common Registers MR (Mode Register) *///
#define MR_RST 0x80
#define MR_WOL 0x20
#define MR_PB 0x10
#define MR_PPPOE 0x08
#define MR_FARP 0x02

///* Common Registers IR (Interrupt Register) *///
#define IR_CONFLICT 0x80
#define IR_UNREACH 0x40
#define IR_PPPoE 0x20
#define IR_MP 0x10

///* Common Registers IMR (Interrupt Mask Register) *///
#define IM_IR7 0x80
#define IM_IR6 0x40
#define IM_IR5 0x20
#define IM_IR4 0x10

///* Common Registers PHYCFGR (W5500 PHY Configuration Register) *///
#define PHYCFGR_RST ~(1 << 7) ///< For PHY reset, must operate AND mask.
#define PHYCFGR_OPMD (1 << 6) ///< Configre PHY with OPMDC value
#define PHYCFGR_OPMDC_ALLA (7 << 3)
#define PHYCFGR_OPMDC_PDOWN (6 << 3)
#define PHYCFGR_OPMDC_NA (5 << 3)
#define PHYCFGR_OPMDC_100FA (4 << 3)
#define PHYCFGR_OPMDC_100F (3 << 3)
#define PHYCFGR_OPMDC_100H (2 << 3)
#define PHYCFGR_OPMDC_10F (1 << 3)
#define PHYCFGR_OPMDC_10H (0 << 3)
#define PHYCFGR_DPX_FULL (1 << 2)
#define PHYCFGR_DPX_HALF (0 << 2)
#define PHYCFGR_SPD_100 (1 << 1)
#define PHYCFGR_SPD_10 (0 << 1)
#define PHYCFGR_LNK_ON (1 << 0)
#define PHYCFGR_LNK_OFF (0 << 0)

///* Socket Registers  Sn_MR (Socket n Mode Register) *///
///* Sn_MR Default values *///
#define Sn_MR_MULTI 0x80
#define Sn_MR_BCASTB 0x40
#define Sn_MR_ND 0x20
#define Sn_MR_UCASTB 0x10
#define Sn_MR_MACRAW 0x04
#define Sn_MR_IPRAW 0x03 ///< IP LAYER RAW SOCK
#define Sn_MR_UDP 0x02
#define Sn_MR_TCP 0x01
#define Sn_MR_CLOSE 0x00

///* Sn_MR values used with Sn_MR_MACRAW *///
#define Sn_MR_MFEN Sn_MR_MULTI
#define Sn_MR_MMB Sn_MR_ND
#define Sn_MR_MIP6B Sn_MR_UCASTB

///* Sn_MR value used with Sn_MR_UDP & Sn_MR_MULTI *///
#define Sn_MR_MC Sn_MR_ND

///* Socket Registers  Sn_CR (Socket n Command Register)) *///
#define Sn_CR_OPEN 0x01
#define Sn_CR_LISTEN 0x02
#define Sn_CR_CONNECT 0x04
#define Sn_CR_DISCON 0x08
#define Sn_CR_CLOSE 0x10
#define Sn_CR_SEND 0x20
#define Sn_CR_SEND_MAC 0x21
#define Sn_CR_SEND_KEEP 0x22
#define Sn_CR_RECV 0x40

///* Socket Registers  Sn_IR (Socket n Interrupt Register) *///
#define Sn_IR_SENDOK 0x10
#define Sn_IR_TIMEOUT 0x08
#define Sn_IR_RECV 0x04
#define Sn_IR_DISCON 0x02
#define Sn_IR_CON 0x01

///* Socket Registers  Sn_SR (Socket n Status Register) *///
#define SOCK_CLOSED 0x00
#define SOCK_INIT 0x13
#define SOCK_LISTEN 0x14
#define SOCK_SYNSENT 0x15
#define SOCK_SYNRECV 0x16
#define SOCK_ESTABLISHED 0x17
#define SOCK_FIN_WAIT 0x18
#define SOCK_CLOSING 0x1A
#define SOCK_TIME_WAIT 0x1B
#define SOCK_CLOSE_WAIT 0x1C
#define SOCK_LAST_ACK 0x1D
#define SOCK_UDP 0x22
#define SOCK_IPRAW 0x32 ///*< IP raw mode socket */
#define SOCK_MACRAW 0x42

/* IP PROTOCOL */
#define IPPROTO_IP 0    ///< Dummy for IP
#define IPPROTO_ICMP 1  ///< Control message protocol
#define IPPROTO_IGMP 2  ///< Internet group management protocol
#define IPPROTO_GGP 3   ///< Gateway^2 (deprecated)
#define IPPROTO_TCP 6   ///< TCP
#define IPPROTO_PUP 12  ///< PUP
#define IPPROTO_UDP 17  ///< UDP
#define IPPROTO_IDP 22  ///< XNS idp
#define IPPROTO_ND 77   ///< UNOFFICIAL net disk protocol
#define IPPROTO_RAW 255 ///< Raw IP packet

/* PHY Configuration  */
#define PHY_CONFBY_HW 0     ///< Configured PHY operation mode by HW pin
#define PHY_CONFBY_SW 1     ///< Configured PHY operation mode by SW register
#define PHY_MODE_MANUAL 0   ///< Configured PHY operation mode with user setting.
#define PHY_MODE_AUTONEGO 1 ///< Configured PHY operation mode with auto-negotiation
#define PHY_SPEED_10 0      ///< Link Speed 10
#define PHY_SPEED_100 1     ///< Link Speed 100
#define PHY_DUPLEX_HALF 0   ///< Link Half-Duplex
#define PHY_DUPLEX_FULL 1   ///< Link Full-Duplex
#define PHY_LINK_OFF 0      ///< Link Off
#define PHY_LINK_ON 1       ///< Link On
#define PHY_POWER_NORM 0    ///< PHY power normal mode
#define PHY_POWER_DOWN 1    ///< PHY power down mode

  /*****************************************************************************/
  /******************************* VARIABLES ***********************************/
  typedef enum
  {
    NETINFO_STATIC = 1, ///< Static IP configuration by manually.
    NETINFO_DHCP        ///< Dynamic IP configruation from a DHCP sever
  } w5500_dhcp_mode_t;

  typedef struct
  {
    uint8_t mac[6];         ///< Source Mac Address
    uint8_t ip[4];          ///< Source IP Address
    uint8_t sn[4];          ///< Subnet Mask
    uint8_t gw[4];          ///< Gateway IP Address
    uint8_t dns[4];         ///< DNS server IP Address
    w5500_dhcp_mode_t dhcp; ///< 1 - Static, 2 - DHCP
  } w5500_net_info_t;

  typedef enum
  {
    NM_FORCEARP = (1 << 1),  ///< Force to APP send whenever udp data is sent. Valid only in W5500
    NM_WAKEONLAN = (1 << 5), ///< Wake On Lan
    NM_PINGBLOCK = (1 << 4), ///< Block ping-request
    NM_PPPOE = (1 << 3),     ///< PPPoE mode
  } w5500_netmode_t;

  typedef struct
  {
    uint8_t retry_cnt;   ///< retry count
    uint16_t time_100us; ///< time unit 100us
  } w5500_net_timeout_t;

  typedef enum
  {
    IK_WOL = (1 << 4),              ///< Wake On Lan by receiving the magic packet
    IK_PPPOE_TERMINATED = (1 << 5), ///< PPPoE Disconnected
    IK_DEST_UNREACH = (1 << 6),     ///< Destination IP & Port Unreachable
    IK_IP_CONFLICT = (1 << 7),      ///< IP conflict occurred
    IK_SOCK_0 = (1 << 8),           ///< Socket 0 interrupt
    IK_SOCK_1 = (1 << 9),           ///< Socket 1 interrupt
    IK_SOCK_2 = (1 << 10),          ///< Socket 2 interrupt
    IK_SOCK_3 = (1 << 11),          ///< Socket 3 interrupt
    IK_SOCK_4 = (1 << 12),          ///< Socket 4 interrupt
    IK_SOCK_5 = (1 << 13),          ///< Socket 5 interrupt
    IK_SOCK_6 = (1 << 14),          ///< Socket 6 interrupt
    IK_SOCK_7 = (1 << 15),          ///< Socket 7 interrupt
  } w5500_interrupt_t;

  typedef enum
  {
    SOCKET_INT_CON = (1 << 0),
    SOCKET_INT_DISCON = (1 << 1),
    SOCKET_INT_RECV = (1 << 2),
    SOCKET_INT_TIMEOUT = (1 << 3),
    SOCKET_INT_SEND_OK = (1 << 4),
  } socket_interrupt;

  typedef struct
  {
    uint8_t confby; ///< set by @ref PHY_CONFBY_HW or @ref PHY_CONFBY_SW
    uint8_t mode;   ///< set by @ref PHY_MODE_MANUAL or @ref PHY_MODE_AUTONEGO
    uint8_t speed;  ///< set by @ref PHY_SPEED_10 or @ref PHY_SPEED_100
    uint8_t duplex; ///< set by @ref PHY_DUPLEX_HALF @ref PHY_DUPLEX_FULL
    // uint8_t power;  ///< set by @ref PHY_POWER_NORM or @ref PHY_POWER_DOWN
    // uint8_t link;   ///< Valid only in CW_GET_PHYSTATUS. set by @ref PHY_LINK_ON or PHY_DUPLEX_OFF
  } w5500_phyconf_t;
  /*****************************************************************************/
  /******************************* METHODS *************************************/
  ///* Basic I/O function *///
  uint8_t w5500_read(uint32_t addr);
  void w5500_write(uint32_t addr, uint8_t wb);
  void w5500_read_buf(uint32_t addr, uint8_t *p_buf, uint16_t len);
  void w5500_write_buf(uint32_t addr, uint8_t *p_buf, uint16_t len);
  void w5500_send_data(uint8_t sn, uint8_t *data, uint16_t len);
  void w5500_recv_data(uint8_t sn, uint8_t *data, uint16_t len);
  void w5500_recv_ignore(uint8_t sn, uint16_t len);

  ///* Common register I/O function *///
  void w5500_set_mr(uint8_t data);
  uint8_t w5500_get_mr(void);
  void w5500_set_gar(uint8_t *data);
  void w5500_get_gar(uint8_t *data);
  void w5500_set_subr(uint8_t *data);
  void w5500_get_subr(uint8_t *data);
  void w5500_set_shar(uint8_t *data);
  void w5500_get_shar(uint8_t *data);
  void w5500_set_sipr(uint8_t *data);
  void w5500_get_sipr(uint8_t *data);
  void w5500_set_intlevel(uint16_t data);
  uint16_t w5500_get_intlevel(void);
  void w5500_set_ir(uint8_t data);
  uint8_t w5500_get_ir(void);
  void w5500_set_imr(uint8_t data);
  uint8_t w5500_get_imr(void);
  void w5500_set_sir(uint8_t data);
  uint8_t w5500_get_sir(void);
  void w5500_set_simr(uint8_t data);
  uint8_t w5500_get_simr(void);
  void w5500_set_rtr(uint16_t data);
  uint16_t w5500_get_rtr(void);
  void w5500_set_rcr(uint8_t data);
  uint8_t w5500_get_rcr(void);
  void w5500_set_ptimer(uint8_t data);
  uint8_t w5500_get_ptimer(void);
  void w5500_set_pmagic(uint8_t data);
  uint8_t w5500_get_pmagic(void);
  void w5500_set_phar(uint8_t *data);
  void w5500_get_phar(uint8_t *data);
  void w5500_set_psid(uint16_t data);
  uint16_t w5500_get_psid(void);
  void w5500_set_pmru(uint16_t data);
  uint16_t w5500_get_pmru(void);
  uint8_t w5500_get_uipr(void);
  uint8_t w5500_get_uportr(void);
  void w5500_set_phycfgr(uint8_t data);
  uint8_t w5500_get_phycfgr(void);
  uint8_t w5500_get_version(void);

  ///* Socket N register I/O function *///
  void w5500_set_sn_mr(uint8_t sn, uint8_t data);
  uint8_t w5500_get_sn_mr(uint8_t sn);
  void w5500_set_sn_cr(uint8_t sn, uint8_t data);
  uint8_t w5500_get_sn_cr(uint8_t sn);
  void w5500_set_sn_ir(uint8_t sn, uint8_t data);
  uint8_t w5500_get_sn_ir(uint8_t sn);
  void w5500_set_sn_imr(uint8_t sn, uint8_t data);
  uint8_t w5500_get_sn_imr(uint8_t sn);
  uint8_t w5500_get_sn_sr(uint8_t sn);
  void w5500_set_sn_port(uint8_t sn, uint16_t data);
  uint16_t w5500_get_sn_port(uint8_t sn);
  void w5500_set_sn_dhar(uint8_t sn, uint8_t *data);
  void w5500_get_sn_dhar(uint8_t sn, uint8_t *data);
  void w5500_set_sn_dipr(uint8_t sn, uint8_t *data);
  void w5500_get_sn_dipr(uint8_t sn, uint8_t *data);
  void w5500_set_sn_dport(uint8_t sn, uint16_t data);
  uint16_t w5500_get_sn_dport(uint8_t sn);
  void w5500_set_sn_mssr(uint8_t sn, uint16_t data);
  uint16_t w5500_get_sn_mssr(uint8_t sn);
  void w5500_set_sn_tos(uint8_t sn, uint8_t data);
  uint8_t w5500_get_sn_tos(uint8_t sn);
  void w5500_set_sn_ttl(uint8_t sn, uint8_t data);
  uint8_t w5500_get_sn_ttl(uint8_t sn);
  void w5500_set_sn_rxbuf_size(uint8_t sn, uint8_t data);
  uint8_t w5500_get_sn_rxbuf_size(uint8_t sn);
  void w5500_set_sn_txbuf_size(uint8_t sn, uint8_t data);
  uint8_t w5500_get_sn_txbuf_size(uint8_t sn);
  uint16_t w5500_get_sn_tx_fsr(uint8_t sn);
  uint16_t w5500_get_tx_rd(uint8_t sn);
  void w5500_set_tx_wr(uint8_t sn, uint16_t data);
  uint16_t w5500_get_tx_wr(uint8_t sn);
  uint16_t w5500_get_sn_rx_rsr(uint8_t sn);
  void w5500_set_rx_rd(uint8_t sn, uint16_t data);
  uint16_t w5500_get_rx_rd(uint8_t sn);
  uint16_t w5500_get_rx_wr(uint8_t sn);
  void w5500_set_frag(uint8_t sn, uint16_t data);
  uint16_t w5500_get_frag(uint8_t sn);
  void w5500_set_sn_kpalvtr(uint8_t sn, uint8_t data);
  uint8_t w5500_get_sn_kpalvtr(uint8_t sn);
  uint16_t w5500_get_sn_rx_max(uint8_t sn);
  uint16_t w5500_get_sn_tx_max(uint8_t sn);

  ///* Configuration functions *///
  void w5500_sw_reset(void);
  void w5500_init(uint8_t *txsize, uint8_t *rxsize);
  void w5500_clear_interrupt(w5500_interrupt_t intr);
  w5500_interrupt_t w5500_get_interrupt(void);
  void w5500_set_interrupt_mask(w5500_interrupt_t intr);
  w5500_interrupt_t w5500_get_interrupt_mask(void);
  int8_t w5500_get_phylink(void);
  int8_t w5500_get_phymode(void);
  void w5500_phy_reset(void);
  void w5500_set_phyconf(w5500_phyconf_t *phyconf);
  void w5500_get_phyconf(w5500_phyconf_t *phyconf);
  void w5500_get_phystat(w5500_phyconf_t *phyconf);
  int8_t w5500_set_phypmode(uint8_t pmode);
  void w5500_set_netinfo(w5500_net_info_t *pnetinfo);
  void w5500_get_netinfo(w5500_net_info_t *pnetinfo);
  int8_t w5500_set_netmode(w5500_netmode_t netmode);
  w5500_netmode_t w5500_get_netmode(void);
  void w5500_set_timeout(w5500_net_timeout_t *nettime);
  void w5500__get_timeout(w5500_net_timeout_t *nettime);

#ifdef __cplusplus
}
#endif

#endif /* INC_W5500_H_ */
/*****************************************************************************/
