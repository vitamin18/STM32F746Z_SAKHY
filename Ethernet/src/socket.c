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
#include "socket.h"

/*****************************************************************************/
/******************************* DEFINES *************************************/
#define DEFAULT_PORT 1900

#define CHECK_SOCKNUM()                                                                                                                    \
  do                                                                                                                                       \
  {                                                                                                                                        \
    if (sn > W5500_SOCK_NUM) return SOCKERR_SOCKNUM;                                                                                       \
  } while (0);

#define CHECK_SOCKMODE(mode)                                                                                                               \
  do                                                                                                                                       \
  {                                                                                                                                        \
    if ((w5500_get_sn_mr(sn) & 0x0F) != mode) return SOCKERR_SOCKMODE;                                                                     \
  } while (0);

#define CHECK_SOCKINIT()                                                                                                                   \
  do                                                                                                                                       \
  {                                                                                                                                        \
    if ((w5500_get_sn_sr(sn) != SOCK_INIT)) return SOCKERR_SOCKINIT;                                                                       \
  } while (0);

#define CHECK_SOCKDATA()                                                                                                                   \
  do                                                                                                                                       \
  {                                                                                                                                        \
    if (len == 0) return SOCKERR_DATALEN;                                                                                                  \
  } while (0);                                                                                                                             \
/*****************************************************************************/
/******************************* VARIABLES ***********************************/
static uint16_t sock_io_mode = 0;
static uint16_t sock_is_sending = 0;
static uint16_t sock_remained_size[W5500_SOCK_NUM] = {
    0,
    0,
};
uint8_t sock_pack_info[W5500_SOCK_NUM] = {
    0,
};
/*****************************************************************************/
/******************************* METHODS *************************************/
/**
 * @brief
 * @param sn
 * @param protocol
 * @param port
 * @param flag
 * @return
 */
int8_t socket(uint8_t sn, uint8_t protocol, uint16_t port, uint8_t flag)
{
  CHECK_SOCKNUM();
  switch (protocol)
  {
  case Sn_MR_TCP:
    uint32_t taddr;
    w5500_get_sipr((uint8_t *)&taddr);
    if (taddr == 0) return SOCKERR_SOCKINIT;
    break;
  case Sn_MR_UDP:
  case Sn_MR_MACRAW:
  case Sn_MR_IPRAW: break;
  default: return SOCKERR_SOCKMODE;
  }
  if ((flag & 0x04) != 0)
  {
    return SOCKERR_SOCKFLAG;
  }

  if (flag != 0)
  {
    switch (protocol)
    {
    case Sn_MR_TCP:
      if ((flag & (SF_TCP_NODELAY | SF_IO_NONBLOCK)) == 0)
      {
        return SOCKERR_SOCKFLAG;
      }
      break;
    case Sn_MR_UDP:
      if (flag & SF_IGMP_VER2)
      {
        if ((flag & SF_MULTI_ENABLE) == 0) return SOCKERR_SOCKFLAG;
      }
      if (flag & SF_UNI_BLOCK)
      {
        if ((flag & SF_MULTI_ENABLE) == 0) return SOCKERR_SOCKFLAG;
      }
      break;
    default: break;
    }
  }

  close(sn);
  w5500_set_sn_mr(sn, (protocol | (flag & 0xF0)));

  if (!port)
  {
    port = DEFAULT_PORT;
  }

  w5500_set_sn_port(sn, port);
  w5500_set_sn_cr(sn, Sn_CR_OPEN);

  while (w5500_get_sn_cr(sn));
  sock_io_mode &= ~(1 << sn);
  sock_io_mode |= ((flag & SF_IO_NONBLOCK) << sn);
  sock_is_sending &= ~(1 << sn);
  sock_remained_size[sn] = 0;
  sock_pack_info[sn] = PACK_COMPLETED;

  while (w5500_get_sn_sr(sn) == SOCK_CLOSED);
  return (int8_t)sn;
}

/**
 * @brief
 * @param sn
 * @return
 */
int8_t close(uint8_t sn)
{
  CHECK_SOCKNUM();

  w5500_set_sn_cr(sn, Sn_CR_CLOSE);
  /* wait to process the command... */
  while (w5500_get_sn_cr(sn));
  /* clear all interrupt of the socket. */
  w5500_set_sn_ir(sn, 0xFF);
  sock_io_mode &= ~(1 << sn);
  sock_is_sending &= ~(1 << sn);
  sock_remained_size[sn] = 0;
  sock_pack_info[sn] = 0;
  while (w5500_get_sn_sr(sn) != SOCK_CLOSED);
  return SOCK_OK;
}

/**
 * @brief
 * @param sn
 * @return
 */
int8_t listen(uint8_t sn)
{
  CHECK_SOCKNUM();
  CHECK_SOCKMODE(Sn_MR_TCP);
  CHECK_SOCKINIT();
  w5500_set_sn_cr(sn, Sn_CR_LISTEN);
  while (w5500_get_sn_cr(sn));
  while (w5500_get_sn_sr(sn) != SOCK_LISTEN)
  {
    close(sn);
    return SOCKERR_SOCKCLOSED;
  }
  return SOCK_OK;
}

/**
 * @brief
 * @param sn
 * @return
 */
int8_t disconnect(uint8_t sn)
{
  CHECK_SOCKNUM();
  CHECK_SOCKMODE(Sn_MR_TCP);
  w5500_set_sn_cr(sn, Sn_CR_DISCON);
  /* wait to process the command... */
  while (w5500_get_sn_cr(sn));
  sock_is_sending &= ~(1 << sn);
  if (sock_io_mode & (1 << sn)) return SOCK_BUSY;
  while (w5500_get_sn_sr(sn) != SOCK_CLOSED)
  {
    if (w5500_get_sn_ir(sn) & Sn_IR_TIMEOUT)
    {
      close(sn);
      return SOCKERR_TIMEOUT;
    }
  }
  return SOCK_OK;
}

/**
 * @brief
 * @param sn
 * @param buf
 * @param len
 * @param addr
 * @param port
 * @return
 */
int32_t sendto(uint8_t sn, uint8_t *buf, uint16_t len, uint8_t *addr, uint16_t port)
{
  uint8_t tmp = 0;
  uint16_t freesize = 0;
  uint32_t taddr;

  CHECK_SOCKNUM();

  switch (w5500_get_sn_mr(sn) & 0x0F)
  {
  case Sn_MR_UDP:
  case Sn_MR_MACRAW:
  case Sn_MR_IPRAW: break;
  default: return SOCKERR_SOCKMODE;
  }

  CHECK_SOCKDATA();

  taddr = ((uint32_t)addr[0]) & 0x000000FF;
  taddr = (taddr << 8) + ((uint32_t)addr[1] & 0x000000FF);
  taddr = (taddr << 8) + ((uint32_t)addr[2] & 0x000000FF);
  taddr = (taddr << 8) + ((uint32_t)addr[3] & 0x000000FF);

  if ((taddr == 0) && ((w5500_get_sn_mr(sn) & Sn_MR_MACRAW) != Sn_MR_MACRAW))
  {
    return SOCKERR_IPINVALID;
  }
  if ((port == 0) && ((w5500_get_sn_mr(sn) & Sn_MR_MACRAW) != Sn_MR_MACRAW))
  {
    return SOCKERR_PORTZERO;
  }
  tmp = w5500_get_sn_sr(sn);

  if ((tmp != SOCK_MACRAW) && (tmp != SOCK_UDP) && (tmp != SOCK_IPRAW))
  {
    return SOCKERR_SOCKSTATUS;
  }

  w5500_set_sn_dipr(sn, addr);
  w5500_set_sn_dport(sn, port);
  freesize = w5500_get_sn_tx_max(sn);
  if (len > freesize)
  {
    len = freesize; // check size not to exceed MAX size.
  }
  while (1)
  {
    freesize = w5500_get_sn_tx_fsr(sn);
    if (w5500_get_sn_sr(sn) == SOCK_CLOSED)
    {
      return SOCKERR_SOCKCLOSED;
    }
    if ((sock_io_mode & (1 << sn)) && (len > freesize))
    {
      return SOCK_BUSY;
    }
    if (len <= freesize)
    {
      break;
    }
  };

  w5500_send_data(sn, buf, len);

  w5500_set_sn_cr(sn, Sn_CR_SEND);
  /* wait to process the command... */
  while (w5500_get_sn_cr(sn));
  while (1)
  {
    tmp = w5500_get_sn_ir(sn);
    if (tmp & Sn_IR_SENDOK)
    {
      w5500_set_sn_ir(sn, Sn_IR_SENDOK);
      break;
    }
    else if (tmp & Sn_IR_TIMEOUT)
    {
      w5500_set_sn_ir(sn, Sn_IR_TIMEOUT);
      return SOCKERR_TIMEOUT;
    }
  }

  return (int32_t)len;
}

/**
 * @brief
 * @param sn
 * @param buf
 * @param len
 * @return
 */
int32_t send(uint8_t sn, uint8_t *buf, uint16_t len)
{
  uint8_t tmp = 0;
  uint16_t freesize = 0;

  CHECK_SOCKNUM();
  CHECK_SOCKMODE(Sn_MR_TCP);
  CHECK_SOCKDATA();
  tmp = w5500_get_sn_sr(sn);
  if (tmp != SOCK_ESTABLISHED && tmp != SOCK_CLOSE_WAIT)
  {
    return SOCKERR_SOCKSTATUS;
  }

  if (sock_is_sending & (1 << sn))
  {
    tmp = w5500_get_sn_ir(sn);
    if (tmp & Sn_IR_SENDOK)
    {
      w5500_set_sn_ir(sn, Sn_IR_SENDOK);
      sock_is_sending &= ~(1 << sn);
    }
    else if (tmp & Sn_IR_TIMEOUT)
    {
      close(sn);
      return SOCKERR_TIMEOUT;
    }
    else
      return SOCK_BUSY;
  }
  freesize = w5500_get_sn_tx_max(sn);
  if (len > freesize)
  {
    len = freesize; // check size not to exceed MAX size.
  }
  while (1)
  {
    freesize = w5500_get_sn_tx_fsr(sn);
    tmp = w5500_get_sn_sr(sn);
    if ((tmp != SOCK_ESTABLISHED) && (tmp != SOCK_CLOSE_WAIT))
    {
      close(sn);
      return SOCKERR_SOCKSTATUS;
    }
    if ((sock_io_mode & (1 << sn)) && (len > freesize)) return SOCK_BUSY;
    if (len <= freesize) break;
  }
  w5500_send_data(sn, buf, len);
  w5500_set_sn_cr(sn, Sn_CR_SEND);
  /* wait to process the command... */
  while (w5500_get_sn_cr(sn));
  sock_is_sending |= (1 << sn);

  return (int32_t)len;
}

/**
 * @brief
 * @param sn
 * @param buf
 * @param len
 * @return
 */
int32_t recv(uint8_t sn, uint8_t *buf, uint16_t len)
{
  uint8_t tmp = 0;
  uint16_t recvsize = 0;

  CHECK_SOCKNUM();
  CHECK_SOCKMODE(Sn_MR_TCP);
  CHECK_SOCKDATA();

  recvsize = w5500_get_sn_rx_max(sn);
  if (recvsize < len)
  {
    len = recvsize;
  }

  while (1)
  {
    recvsize = w5500_get_sn_rx_rsr(sn);
    tmp = w5500_get_sn_sr(sn);
    if (tmp != SOCK_ESTABLISHED)
    {
      if (tmp == SOCK_CLOSE_WAIT)
      {
        if (recvsize != 0)
          break;
        else if (w5500_get_sn_tx_fsr(sn) == w5500_get_sn_tx_max(sn))
        {
          close(sn);
          return SOCKERR_SOCKSTATUS;
        }
      }
      else
      {
        close(sn);
        return SOCKERR_SOCKSTATUS;
      }
    }
    if ((sock_io_mode & (1 << sn)) && (recvsize == 0)) return SOCK_BUSY;
    if (recvsize != 0) break;
  };

  if (recvsize < len)
  {
    len = recvsize;
  }
  w5500_recv_data(sn, buf, len);
  w5500_set_sn_cr(sn, Sn_CR_RECV);
  while (w5500_get_sn_cr(sn));

  return (int32_t)len;
}

/**
 * @brief
 * @param sn
 * @param buf
 * @param len
 * @param addr
 * @param port
 * @return
 */
int32_t recvfrom(uint8_t sn, uint8_t *buf, uint16_t len, uint8_t *addr, uint16_t *port)
{
  uint8_t mr;

  uint8_t head[8];
  uint16_t pack_len = 0;

  CHECK_SOCKNUM();

  switch ((mr = w5500_get_sn_mr(sn)) & 0x0F)
  {
  case Sn_MR_UDP:
  case Sn_MR_IPRAW:
  case Sn_MR_MACRAW: break;
  default: return SOCKERR_SOCKMODE;
  }

  CHECK_SOCKDATA();

  if (sock_remained_size[sn] == 0)
  {
    while (1)
    {
      pack_len = w5500_get_sn_rx_rsr(sn);
      if (w5500_get_sn_sr(sn) == SOCK_CLOSED) return SOCKERR_SOCKCLOSED;
      if ((sock_io_mode & (1 << sn)) && (pack_len == 0)) return SOCK_BUSY;
      if (pack_len != 0) break;
    };
  }
  switch (mr & 0x07)
  {
  case Sn_MR_UDP:
    if (sock_remained_size[sn] == 0)
    {
      w5500_recv_data(sn, head, 8);
      w5500_set_sn_cr(sn, Sn_CR_RECV);
      while (w5500_get_sn_cr(sn));
      addr[0] = head[0];
      addr[1] = head[1];
      addr[2] = head[2];
      addr[3] = head[3];
      *port = head[4];
      *port = (*port << 8) + head[5];
      sock_remained_size[sn] = head[6];
      sock_remained_size[sn] = (sock_remained_size[sn] << 8) + head[7];
      sock_pack_info[sn] = PACK_FIRST;
    }
    if (len < sock_remained_size[sn])
    {
      pack_len = len;
    }
    else
    {
      pack_len = sock_remained_size[sn];
    }
    len = pack_len;
    //
    // Need to packet length check (default 1472)
    //
    w5500_recv_data(sn, buf, pack_len); // data copy.
    break;
  case Sn_MR_MACRAW:
    if (sock_remained_size[sn] == 0)
    {
      w5500_recv_data(sn, head, 2);
      w5500_set_sn_cr(sn, Sn_CR_RECV);
      while (w5500_get_sn_cr(sn));
      // read peer's IP address, port number & packet length
      sock_remained_size[sn] = head[0];
      sock_remained_size[sn] = (sock_remained_size[sn] << 8) + head[1] - 2;
      if (sock_remained_size[sn] > 1514)
      {
        close(sn);
        return SOCKFATAL_PACKLEN;
      }
      sock_pack_info[sn] = PACK_FIRST;
    }
    if (len < sock_remained_size[sn])
    {
      pack_len = len;
    }
    else
    {
      pack_len = sock_remained_size[sn];
    }
    w5500_recv_data(sn, buf, pack_len);
    break;
  case Sn_MR_IPRAW:
    if (sock_remained_size[sn] == 0)
    {
      w5500_recv_data(sn, head, 6);
      w5500_set_sn_cr(sn, Sn_CR_RECV);
      while (w5500_get_sn_cr(sn));
      addr[0] = head[0];
      addr[1] = head[1];
      addr[2] = head[2];
      addr[3] = head[3];
      sock_remained_size[sn] = head[4];
      sock_remained_size[sn] = (sock_remained_size[sn] << 8) + head[5];
      sock_pack_info[sn] = PACK_FIRST;
    }
    //
    // Need to packet length check
    //
    if (len < sock_remained_size[sn])
    {
      pack_len = len;
    }
    else
    {
      pack_len = sock_remained_size[sn];
    }
    w5500_recv_data(sn, buf, pack_len); // data copy.
    break;
  default:
    w5500_recv_ignore(sn, pack_len); // data copy.
    sock_remained_size[sn] = pack_len;
    break;
  }
  w5500_set_sn_cr(sn, Sn_CR_RECV);
  /* wait to process the command... */
  while (w5500_get_sn_cr(sn));
  sock_remained_size[sn] -= pack_len;
  if (sock_remained_size[sn] != 0)
  {
    sock_pack_info[sn] |= PACK_REMAINED;
  }
  else
  {
    sock_pack_info[sn] = PACK_COMPLETED;
  }

  return (int32_t)pack_len;
}
/*****************************************************************************/
