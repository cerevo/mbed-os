/**
 * @file UART_TZ10xx.c
 * @brief TZ10xx UART driver
 * @date $Date:: 2014-11-21 14:30:34 +0900 #$
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */
#include "Driver_UART.h"
#include "TZ10xx.h"
#include "PMU_TZ10xx.h"

#include "RTE_Device.h"
#include "RTE_Components.h"

#define ARM_UART_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 16)   /* driver version */

#define UART_TX_BUFFER_SIZE 64
#define UART_RX_BUFFER_SIZE 64

typedef struct {
  uint8_t *sp;
  uint8_t *ep;
  uint8_t *wp;
  uint8_t *rp;
} UART_Buffer;

typedef struct {
  bool init;
  UART_Buffer tx_buffer;
  UART_Buffer rx_buffer;
  ARM_POWER_STATE power_state;
  ARM_UART_SignalEvent_t cb_event;
  uint32_t event_mask;
	uint32_t rtsctsen;
} UART_INFO;

typedef struct {
  uart_Type *dev; /* Pointer to UART peripheral */
  IRQn_Type irq; /* IRQ number */
  bool hw_flow; /* Hardware flow control */
  PMU_CD pmu_cd;
  PMU_MODULE pmu_module;
  PMU_IO_FUNC pmu_func_rxd;
  PMU_IO_FUNC pmu_func_txd;
  PMU_IO_FUNC pmu_func_rts;
  PMU_IO_FUNC pmu_func_cts;
  uint8_t *txbuf_base;
  uint32_t txbuf_size;
  uint8_t *rxbuf_base;
  uint32_t rxbuf_size;
  UART_INFO *info;
} const UART_RESOURCES;


#if RTE_UART0
static uint8_t uart0_tx_buffer[UART_TX_BUFFER_SIZE];
static uint8_t uart0_rx_buffer[UART_RX_BUFFER_SIZE];

static UART_INFO UART0_Info;

static UART_RESOURCES UART0_Resources = {
  uart0, /* dev */
  UART0_IRQn, /* irq */
#if (RTE_UART0_HW_FLOW == 1)
  true, /* hw_flow */
#else
  false, /* hw_flow */
#endif
  PMU_CD_UART0, /* pmu_cd */
  PMU_MODULE_UART0, /* pmu_module */
  PMU_IO_FUNC_UA0_RXD, /* pmu_func_rxd */
  PMU_IO_FUNC_UA0_TXD, /* pmu_func_txd */
  PMU_IO_FUNC_UA0_RTS_N, /* pmu_func_rts */
  PMU_IO_FUNC_UA0_CTS_N, /* pmu_func_cts */
  uart0_tx_buffer, /* txbuf_base */
  UART_TX_BUFFER_SIZE, /* txbuf_size */
  uart0_rx_buffer, /* rxbuf_base */
  UART_RX_BUFFER_SIZE, /* rxbuf_size */
  &UART0_Info /* info */
};
#endif /* RTE_UART0 */

#if RTE_UART1
static uint8_t uart1_tx_buffer[UART_TX_BUFFER_SIZE];
static uint8_t uart1_rx_buffer[UART_RX_BUFFER_SIZE];

static UART_INFO UART1_Info;

static UART_RESOURCES UART1_Resources = {
  uart1, /* dev */
  UART1_IRQn, /* irq */
#if (RTE_UART1_HW_FLOW == 1)
  true, /* hw_flow */
#else
  false, /* hw_flow */
#endif
  PMU_CD_UART1, /* pmu_cd */
  PMU_MODULE_UART1, /* pmu_module */
  PMU_IO_FUNC_UA1_RXD, /* pmu_func_rxd */
  PMU_IO_FUNC_UA1_TXD, /* pmu_func_txd */
  PMU_IO_FUNC_UA1_RTS_N, /* pmu_func_rts */
  PMU_IO_FUNC_UA1_CTS_N, /* pmu_func_cts */
  uart1_tx_buffer, /* txbuf_base */
  UART_TX_BUFFER_SIZE, /* txbuf_size */
  uart1_rx_buffer, /* rxbuf_base */
  UART_RX_BUFFER_SIZE, /* rxbuf_size */
  &UART1_Info /* info */
};
#endif /* RTE_UART1 */

#if RTE_UART2
static uint8_t uart2_tx_buffer[UART_TX_BUFFER_SIZE];
static uint8_t uart2_rx_buffer[UART_RX_BUFFER_SIZE];

static UART_INFO UART2_Info;

static UART_RESOURCES UART2_Resources = {
  uart2, /* dev */
  UART2_IRQn, /* irq */
#if (RTE_UART2_HW_FLOW == 1)
  true, /* hw_flow */
#else
  false, /* hw_flow */
#endif
  PMU_CD_UART2, /* pmu_cd */
  PMU_MODULE_UART2, /* pmu_module */
  PMU_IO_FUNC_UA2_RXD, /* pmu_func_rxd */
  PMU_IO_FUNC_UA2_TXD, /* pmu_func_txd */
  PMU_IO_FUNC_UA2_RTS_N, /* pmu_func_rts */
  PMU_IO_FUNC_UA2_CTS_N, /* pmu_func_cts */
  uart2_tx_buffer, /* txbuf_base */
  UART_TX_BUFFER_SIZE, /* txbuf_size */
  uart2_rx_buffer, /* rxbuf_base */
  UART_RX_BUFFER_SIZE, /* rxbuf_size */
  &UART2_Info /* info */
};
#endif /* RTE_UART2 */

static ARM_UART_STATUS UART_PowerControl(UART_RESOURCES *rsc, ARM_POWER_STATE state);
static bool UART_TxDone(UART_RESOURCES *rsc);
static ARM_UART_STATUS UART_FlushTxBuffer(UART_RESOURCES *rsc);
static ARM_UART_STATUS UART_FlushRxBuffer(UART_RESOURCES *rsc);

/**
  @brief       Get driver version.
  @return      \ref ARM_DRV_VERSION
*/
static ARM_DRIVER_VERSION UARTX_GetVersion(void)
{
  static const ARM_DRIVER_VERSION driver_version = {
    ARM_UART_API_VERSION,
    ARM_UART_DRV_VERSION
  };
  return driver_version;
}

/**
  @brief       Get driver capabilities
  @return      \ref ARM_UART_CAPABILITIES
*/
static ARM_UART_CAPABILITIES UARTX_GetCapabilities(void)
{
  static const ARM_UART_CAPABILITIES capabilities = {
    16, /* tx_buffer_size */
    16, /* rx_buffer_size */
    1,  /* rx_timeout_event*/
    1,  /* cts */
    1,  /* cts_event */
    0,  /* dsr */
    0,  /* dsr_event */
    0,  /* dcd */
    0,  /* dcd_event */
    0,  /* ri */
    0,  /* ri_event */
    0   /* reserved */
  };
  return capabilities;
}

static void null_callback(ARM_UART_EVENT e)
{
  /* DO NOTHING */
}

static void UART_InitBuffer(UART_Buffer *buf, uint8_t *mem, uint32_t size)
{
  buf->sp = mem;
  buf->ep = mem + size;
  buf->wp = mem;
  buf->rp = mem;
}

/**
  @fn          ARM_UART_STATUS ARM_UART_Initialize (ARM_UART_SignalEvent_t cb_event,
                                                    uint32_t               event_mask)
  @brief       Initialize UART Interface.
  @param[in]   cb_event    Pointer to \ref ARM_UART_SignalEvent
  @param[in]   event_mask  Events that are reported through callbacks
  @return      execution status \ref ARM_UART_STATUS
*/
static ARM_UART_STATUS UART_Initialize(UART_RESOURCES *rsc,
                                       ARM_UART_SignalEvent_t cb_event,        
                                       uint32_t               event_mask)
{
  if (rsc->info->init) {
    /* already initialized */
    return ARM_UART_ERROR;
  }
  
  UART_InitBuffer(&rsc->info->tx_buffer, rsc->txbuf_base, rsc->txbuf_size);
  UART_InitBuffer(&rsc->info->rx_buffer, rsc->rxbuf_base, rsc->rxbuf_size);
  
  /* callback setting */
  if (cb_event == 0) {
    rsc->info->cb_event = null_callback;
  } else {
    rsc->info->cb_event = cb_event;
  }
  rsc->info->event_mask = event_mask;

  rsc->info->rtsctsen = 0;
  
  /* Enable UART IRQ */
  NVIC_ClearPendingIRQ(rsc->irq);
  NVIC_EnableIRQ(rsc->irq);
  
  rsc->info->init = true;
  
  UART_PowerControl(rsc, ARM_POWER_LOW);
  
  return ARM_UART_OK;
}

/**
  @brief       De-initialize UART Interface.
  @return      execution status \ref ARM_UART_STATUS
*/
static ARM_UART_STATUS UART_Uninitialize(UART_RESOURCES *rsc)
{
  /* Disable UART IRQ */
  NVIC_ClearPendingIRQ(rsc->irq);
  NVIC_DisableIRQ(rsc->irq);
  
 if (rsc->info->power_state != ARM_POWER_OFF) {
    /* discard rx buffer */
    UART_FlushRxBuffer(rsc);
    /* stop tx/rx */
    rsc->dev->UARTCR &= ~0x301u;
     /* (LOW or FULL) -> OFF */
    rsc->dev->UARTIMSC = 0u;
    Driver_PMU.EnableModule(rsc->pmu_module, 0);
    /* transit input pins to standby */
    Driver_PMU.StandbyInputBuffer(rsc->pmu_func_rxd, 1);
    if (rsc->hw_flow) {
      Driver_PMU.StandbyInputBuffer(rsc->pmu_func_cts, 1);
    }
	rsc->info->power_state = ARM_POWER_OFF;
  }
   
  rsc->info->init = false;
  
  return ARM_UART_OK;
}

/**
  @brief       Control  UART Interface Power.
  @param[in]   state    Power state
  @return      execution status \ref ARM_UART_STATUS
*/
static ARM_UART_STATUS UART_PowerControl(UART_RESOURCES *rsc, ARM_POWER_STATE state)
{
  if (!rsc->info->init) {
    return ARM_UART_ERROR;
  }
  
  if (state == rsc->info->power_state) {
    return ARM_UART_OK;
  }
  
  if ((uint32_t)state > ARM_POWER_FULL) {
    return ARM_UART_ERROR;
  }
  
  if (state != ARM_POWER_OFF) {
    /* transit input pins to active */
    Driver_PMU.StandbyInputBuffer(rsc->pmu_func_rxd, 0);
    if (rsc->hw_flow) {
      Driver_PMU.StandbyInputBuffer(rsc->pmu_func_cts, 0);
    }
    Driver_PMU.EnableModule(rsc->pmu_module, 1);
  }
  
  if (state != ARM_POWER_FULL) {
    /* FULL -> (OFF or LOW) */
    /* wait for finish tx */
    while (!UART_TxDone(rsc)) {
      /* DO NOTHING */
    }
    /* discard rx buffer */
    UART_FlushRxBuffer(rsc);
    /* stop tx/rx */
    rsc->dev->UARTCR &= ~0x301u;
    /* RTS disable */
    rsc->dev->UARTCR_b.RTSEn = 0;
    rsc->dev->UARTCR_b.RTS = 0;
	} else {
    /* (OFF or LOW) -> FULL */
    uint32_t imsc = 0u;
    uint32_t event_mask = rsc->info->event_mask;
    /* enable interrupts */
    if (event_mask & (1u << ARM_UART_EVENT_RX_OVERRUN)) {
      imsc |= 1u << 10;
    }
    if (event_mask & (1u << ARM_UART_EVENT_RX_BREAK)) {
      imsc |= 1u << 9;
    }
    if (event_mask & (1u << ARM_UART_EVENT_RX_PARITY_ERROR)) {
      imsc |= 1u << 8;
    }
    if (event_mask & (1u << ARM_UART_EVENT_RX_FRAMING_ERROR)) {
      imsc |= 1u << 7;
    }
    if (event_mask & (1u << ARM_UART_EVENT_RX_TIMEOUT)) {
      imsc |= 1u << 6;
    }
    if (event_mask & (1u << ARM_UART_EVENT_TX_THRESHOLD)) {
      imsc |= 1u << 5;
    }
    if (event_mask & (1u << ARM_UART_EVENT_RX_THRESHOLD)) {
      imsc |= 1u << 4;
    }
    if (event_mask & (1u << ARM_UART_EVENT_CTS)) {
      imsc |= 1u << 1;
    }
    rsc->dev->UARTIMSC = imsc;
    
    /* set RTSEn */
    rsc->dev->UARTCR_b.RTSEn = rsc->info->rtsctsen & 0x1;
    /* start tx/rx */
    rsc->dev->UARTCR |= 0x301u;
  }
  
  if (state == ARM_POWER_OFF) {
    /* (LOW or FULL) -> OFF */
    rsc->dev->UARTIMSC = 0u;
    Driver_PMU.EnableModule(rsc->pmu_module, 0);
    /* transit input pins to standby */
    Driver_PMU.StandbyInputBuffer(rsc->pmu_func_rxd, 1);
    if (rsc->hw_flow) {
      Driver_PMU.StandbyInputBuffer(rsc->pmu_func_cts, 1);
    }
		rsc->info->rtsctsen = 0;
  }
  
  rsc->info->power_state = state;
  return ARM_UART_OK;
}

/**
  @brief       Configure UART Interface.
  @param[in]   baudrate      Requested baudrate in bits/s
  @param[in]   data_bits     Number of data bits
  @param[in]   parity        \ref ARM_UART_PARITY
  @param[in]   stop_bits     \ref ARM_UART_STOP_BITS
  @param[in]   flow_control  \ref ARM_UART_FLOW_CONTROL
  @return      execution status \ref ARM_UART_STATUS
*/
static ARM_UART_STATUS UART_Configure(UART_RESOURCES *rsc,
                                      uint32_t              baudrate,            
                                      uint8_t               data_bits,
                                      ARM_UART_PARITY       parity,
                                      ARM_UART_STOP_BITS    stop_bits,
                                      ARM_UART_FLOW_CONTROL flow_control)
{
  uint32_t freq_uartclk;
  uint32_t q;
  uint32_t ibrd;
  uint32_t fbrd;
  
  uint32_t wle;
  uint32_t pen;
  uint32_t eps;
  uint32_t sps;
  uint32_t stp2;
  uint32_t rtsctsen;
  
  uint32_t power_full;
  
  switch (rsc->info->power_state) {
  case ARM_POWER_LOW:
    power_full = 0;
    break;
  case ARM_POWER_FULL:
    power_full = 1;
    break;
  case ARM_POWER_OFF:
  default:
    return ARM_UART_ERROR;
  }
  
  /* baudrate */
  freq_uartclk = Driver_PMU.GetFrequency(rsc->pmu_cd);
  q = ((freq_uartclk << 3) / baudrate + 1) >> 1;
  ibrd = q >> 6;
  fbrd = q & 0x3f;
  if ((ibrd == 0) || (ibrd > 0xffff)) {
    return ARM_UART_ERROR_BAUDRATE;
  }
  
  /* data_bits */
  if ((data_bits < 5) || (data_bits > 8)) {
    return ARM_UART_ERROR_DATA_BITS;
  }
  wle = data_bits - 5;
  
  /* parity */
  switch (parity) {
  case ARM_UART_PARITY_NONE:
    pen = 0;
    eps = 0;
    sps = 0;
    break;
  case ARM_UART_PARITY_ODD:
    pen = 1;
    eps = 0;
    sps = 0;
    break;
  case ARM_UART_PARITY_EVEN:
    pen = 1;
    eps = 1;
    sps = 0;
    break;
  case ARM_UART_PARITY_MARK:
    pen = 1;
    eps = 0;
    sps = 1;
    break;
  case ARM_UART_PARITY_SPACE:
    pen = 1;
    eps = 1;
    sps = 1;
    break;
  default:
    return ARM_UART_ERROR_PARITY;
  }
  
  /* stop_bits */
  if (stop_bits == ARM_UART_STOP_BITS_1) {
    stp2 = 0;
  } else if (stop_bits == ARM_UART_STOP_BITS_2) {
    stp2 = 1;
  } else {
    return ARM_UART_ERROR_STOP_BITS;
  }
  
  /* flow_control */
  switch (flow_control) {
  case ARM_UART_FLOW_CONTROL_NONE:
    rtsctsen = 0;
    break;
  case ARM_UART_FLOW_CONTROL_RTS_CTS:
    if (rsc->hw_flow) {
      rtsctsen = 3;
    } else {
      return ARM_UART_ERROR_FLOW_CONTROL;
    }
    break;
  default:
    return ARM_UART_ERROR_FLOW_CONTROL;
  }
  
  if (power_full) {
    UART_PowerControl(rsc, ARM_POWER_LOW);
  }
  
  rsc->dev->UARTIBRD = ibrd;
  rsc->dev->UARTFBRD = fbrd;
  rsc->dev->UARTLCR_H =
    (sps << 7) | (wle << 5) | (1u << 4) | (stp2 << 3) | (eps << 2) | (pen << 1);
  /* set CTSEn */
	rsc->dev->UARTCR = ((rtsctsen&0x2) << 14); 
  rsc->info->rtsctsen = rtsctsen;
  
  if (power_full) {
    UART_PowerControl(rsc, ARM_POWER_FULL);
  }
  
  return ARM_UART_OK;
}


/**
  @brief       Write data to UART transmitter.
  @param[in]   data  Pointer to buffer with data to write to UART transmitter
  @param[in]   size  Data buffer size in bytes
  @return      number of data bytes written or execution status
                 - value >= 0: number of data bytes written
                 - value < 0: error occurred, -value is execution status as defined with \ref ARM_UART_STATUS 
*/
static int32_t UART_WriteData(UART_RESOURCES *rsc, const uint8_t *data, uint32_t size)
{
  uint32_t i;
  
  if (rsc->info->power_state != ARM_POWER_FULL) {
    return -ARM_UART_ERROR;
  }
  
  // TODO: DMA
  for (i = 0; i < size; i++) {
    if (rsc->dev->UARTFR_b.TXFF != 0) {
      break;
    }
    rsc->dev->UARTDR = data[i];
  }
  return i;
}

/**
  @brief       Read data from UART receiver.
  @param[out]  data  Pointer to buffer for data read from UART receiver
  @param[in]   size  Data buffer size in bytes
  @return      number of data bytes read or execution status
                 - value >= 0: number of data bytes read
                 - value < 0: error occurred, -value is execution status as defined with \ref ARM_UART_STATUS 
*/
static int32_t UART_ReadData(UART_RESOURCES *rsc, uint8_t *data, uint32_t size)
{
  uint32_t i;

  if (rsc->info->power_state != ARM_POWER_FULL) {
    return -ARM_UART_ERROR;
  }
  
  // TODO: DMA
  for (i = 0; i < size; i++) {
    if (rsc->dev->UARTFR_b.RXFE != 0) {
      break;
    }
    data[i] = (uint8_t)rsc->dev->UARTDR;
  }
  return i;
}

/**
  @brief       Check available data in UART receiver.
  @return      number of data bytes in receive buffer or execution status
                 - value >= 0: number of data bytes in receive buffer
                 - value < 0: error occurred, -value is execution status as defined with \ref ARM_UART_STATUS 
*/
static int32_t UART_DataAvailable(UART_RESOURCES *rsc)
{
  if (rsc->info->power_state != ARM_POWER_FULL) {
    return -ARM_UART_ERROR;
  }
  
  if (rsc->dev->UARTFR_b.RXFF) {
    /* FIFO full */
    return 16;
  } else if (rsc->dev->UARTFR_b.RXFE) {
    /* FIFO empty */
    return 0;
  } else {
    /* FIFO is neither full nor empty; at least 1 byte is available */
    return 1;
  }
}

/**
  @brief       Check if UART transmission is completed.
  @return      transmitter completion status
                 - \b true transmission done
                 - \b false transmission in progress
*/
static bool UART_TxDone(UART_RESOURCES *rsc)
{
  if (rsc->info->power_state != ARM_POWER_FULL) {
    return true;
  }
  return !rsc->dev->UARTFR_b.BUSY;
}

/**
  @brief       Set transmit threshold for UART_TX_THRESHOLD event.
  @param[in]   level  Number of character in transmit buffer
  @return      execution status \ref ARM_UART_STATUS
*/
static ARM_UART_STATUS UART_SetTxThreshold(UART_RESOURCES *rsc, uint32_t level)
{
  uint32_t txiflsel;
  
  if (rsc->info->power_state == ARM_POWER_OFF) {
    return ARM_UART_ERROR;
  }
  
  switch (level) {
  case 2:
    txiflsel = 0;
    break;
  case 4:
    txiflsel = 1;
    break;
  case 8:
    txiflsel = 2;
    break;
  case 12:
    txiflsel = 3;
    break;
  case 14:
    txiflsel = 4;
    break;
  default:
    return ARM_UART_ERROR;
  }
  
  rsc->dev->UARTIFLS_b.TXIFLSEL = txiflsel;
  
  return ARM_UART_OK;
}

/**
  @brief       Set receive threshold for UART_RX_THRESHOLD event.
  @param[in]   level  Number of character in receive buffer
  @return      execution status \ref ARM_UART_STATUS
*/
static ARM_UART_STATUS UART_SetRxThreshold(UART_RESOURCES *rsc, uint32_t level)
{
  uint32_t rxiflsel;
  
  if (rsc->info->power_state == ARM_POWER_OFF) {
    return ARM_UART_ERROR;
  }
  
  switch (level) {
  case 2:
    rxiflsel = 0;
    break;
  case 4:
    rxiflsel = 1;
    break;
  case 8:
    rxiflsel = 2;
    break;
  case 12:
    rxiflsel = 3;
    break;
  case 14:
    rxiflsel = 4;
    break;
  default:
    return ARM_UART_ERROR;
  }
  
  rsc->dev->UARTIFLS_b.RXIFLSEL = rxiflsel;
  
  return ARM_UART_OK;
}

/**
  @brief       Flush UART transmit buffer.
  @return      execution status \ref ARM_UART_STATUS
*/
static ARM_UART_STATUS UART_FlushTxBuffer(UART_RESOURCES *rsc)
{
  volatile uint32_t dummy;
  
  if (rsc->info->power_state != ARM_POWER_FULL) {
    return ARM_UART_ERROR;
  }
  
  rsc->dev->UARTTCR_b.TESTFIFO = 1;
  while (rsc->dev->UARTFR_b.TXFE == 0) {
    dummy = rsc->dev->UARTTDR;
  }
  rsc->dev->UARTTCR_b.TESTFIFO = 0;
  while (rsc->dev->UARTFR_b.BUSY) {
    /* DO NOTHING */
  }
  
  return ARM_UART_OK;
}

/**
  @brief       Flush UART receive buffer.
  @return      execution status \ref ARM_UART_STATUS
*/
static ARM_UART_STATUS UART_FlushRxBuffer(UART_RESOURCES *rsc)
{
  volatile uint32_t dummy;
  
  if (rsc->info->power_state != ARM_POWER_FULL) {
    return ARM_UART_ERROR;
  }
  
  while (rsc->dev->UARTFR_b.RXFE == 0) {
    dummy = rsc->dev->UARTDR;
  }
  
  return ARM_UART_OK;
}

/**
  @brief       Get UART Communication errors.
  @return      \ref ARM_UART_COM_ERROR
*/
static ARM_UART_COM_ERROR UART_GetComError(UART_RESOURCES *rsc)
{
  ARM_UART_COM_ERROR com_error = {0, 0, 0, 0};
  
  if (rsc->info->power_state == ARM_POWER_FULL) {
    com_error.rx_break         = rsc->dev->UARTRSR_ECR_b.BE;
    com_error.rx_framing_error = rsc->dev->UARTRSR_ECR_b.FE;
    com_error.rx_overrun       = rsc->dev->UARTRSR_ECR_b.OE;
    com_error.rx_parity_error  = rsc->dev->UARTRSR_ECR_b.PE;
  }
  
  return com_error;
}

/**
  @brief       Suspend transmission and put UART Transmission line in a break state.
  @return      execution status \ref ARM_UART_STATUS
*/
static ARM_UART_STATUS UART_SetComBreak(UART_RESOURCES *rsc)
{
  if (rsc->info->power_state != ARM_POWER_FULL) {
    return ARM_UART_ERROR;
  }
  
  rsc->dev->UARTLCR_H |= 1u;
  
  return ARM_UART_OK;
}

/**
  @brief       Resume transmission and put UART Transmission line in a non-break state.
  @return      execution status \ref ARM_UART_STATUS
*/
static ARM_UART_STATUS UART_ClearComBreak(UART_RESOURCES *rsc)
{
  if (rsc->info->power_state != ARM_POWER_FULL) {
    return ARM_UART_ERROR;
  }
  
  rsc->dev->UARTLCR_H &= ~1u;
  
  return ARM_UART_OK;
}

/**
  @brief       Set UART Modem Control line state.
  @param[in]   control  \ref ARM_UART_MODEM_CONTROL
  @return      execution status \ref ARM_UART_STATUS
*/
static ARM_UART_STATUS UART_SetModemControl(UART_RESOURCES *rsc, ARM_UART_MODEM_CONTROL control)
{
  if (rsc->info->power_state != ARM_POWER_FULL) {
    return ARM_UART_ERROR;
  }
  
  switch (control) {
  case ARM_UART_RTS_CLEAR:
    rsc->dev->UARTCR_b.RTS = 0;
    break;
  case ARM_UART_RTS_SET:
    rsc->dev->UARTCR_b.RTS = 1;
    break;
  case ARM_UART_DTR_CLEAR:
  case ARM_UART_DTR_SET:
    /* TZ10xx doesn't support DTR */
  default:
    return ARM_UART_ERROR;
  }
  
  return ARM_UART_OK;
}

/**
  @brief       TZ1000 series doesn't support this function.
  @return      modem status \ref ARM_UART_MODEM_STATUS
*/
static ARM_UART_MODEM_STATUS UART_GetModemStatus(UART_RESOURCES *rsc)
{
  ARM_UART_MODEM_STATUS status = {
    0, /* cts */
    0, /* dsr */
    0, /* dcd */
    0  /* ri */
  };
  
  if (rsc->info->power_state == ARM_POWER_FULL) {
    status.cts = rsc->dev->UARTFR_b.CTS;
  }
  
  return status;
}

static void UART_IRQHandler(UART_RESOURCES *rsc) {
  uint32_t mis = rsc->dev->UARTMIS;
  uint32_t event_mask = rsc->info->event_mask;
  rsc->dev->UARTICR = mis;
  
  if ((mis & (1u << 10)) != 0) {
    /* Receive buffer overrun detected */
    if ((event_mask & (1u << ARM_UART_EVENT_RX_OVERRUN)) != 0) {
      rsc->info->cb_event(ARM_UART_EVENT_RX_OVERRUN);
    }
  }
  if ((mis & (1u << 9)) != 0) {
    /* Break detected on receive */
    if ((event_mask & (1u << ARM_UART_EVENT_RX_BREAK)) != 0) {
      rsc->info->cb_event(ARM_UART_EVENT_RX_BREAK);
    }
  }
  if ((mis & (1u << 8)) != 0) {
    /* Parity error detected on receive */
    if ((event_mask & (1u << ARM_UART_EVENT_RX_PARITY_ERROR)) != 0) {
      rsc->info->cb_event(ARM_UART_EVENT_RX_PARITY_ERROR);
    }
  }
  if ((mis & (1u << 7)) != 0) {
    /* Framing error detected on receive */
    if ((event_mask & (1u << ARM_UART_EVENT_RX_FRAMING_ERROR)) != 0) {
      rsc->info->cb_event(ARM_UART_EVENT_RX_FRAMING_ERROR);
    }
  }
  if ((mis & (1u << 6)) != 0) {
    /* Receive character timeout */
    if ((event_mask & (1u << ARM_UART_EVENT_RX_TIMEOUT)) != 0) {
      rsc->info->cb_event(ARM_UART_EVENT_RX_TIMEOUT);
    }
  }
  if ((mis & (1u << 5)) != 0) {
    /* Transmit buffer threshold reached */
    if ((event_mask & (1u << ARM_UART_EVENT_TX_THRESHOLD)) != 0) {
      rsc->info->cb_event(ARM_UART_EVENT_TX_THRESHOLD);
    }
  }
  if ((mis & (1u << 4)) != 0) {
    /* Receive buffer threshold reached */
    if ((event_mask & (1u << ARM_UART_EVENT_RX_THRESHOLD)) != 0) {
      rsc->info->cb_event(ARM_UART_EVENT_RX_THRESHOLD);
    }
  }
  if ((mis & (1u << 1)) != 0) {
    /* CTS state changed */
    if ((event_mask & (1u << ARM_UART_EVENT_CTS)) != 0) {
      rsc->info->cb_event(ARM_UART_EVENT_CTS);
    }
  }
}


#if RTE_UART0
static ARM_UART_STATUS UART0_Initialize(ARM_UART_SignalEvent_t cb_event,        
                                        uint32_t               event_mask)
{
  return UART_Initialize(&UART0_Resources, cb_event, event_mask);
}

static ARM_UART_STATUS UART0_Uninitialize(void)
{
  return UART_Uninitialize(&UART0_Resources);
}

static ARM_UART_STATUS UART0_PowerControl(ARM_POWER_STATE state)
{
  return UART_PowerControl(&UART0_Resources, state);
}

static ARM_UART_STATUS UART0_Configure(uint32_t              baudrate,            
                                       uint8_t               data_bits,
                                       ARM_UART_PARITY       parity,
                                       ARM_UART_STOP_BITS    stop_bits,
                                       ARM_UART_FLOW_CONTROL flow_control)
{
  return UART_Configure(&UART0_Resources, baudrate, data_bits, parity, stop_bits, flow_control);
}

static int32_t UART0_WriteData(const uint8_t *data, uint32_t size)
{
  return UART_WriteData(&UART0_Resources, data, size);
}

static int32_t UART0_ReadData(uint8_t *data, uint32_t size)
{
  return UART_ReadData(&UART0_Resources, data, size);
}

static int32_t UART0_DataAvailable(void)
{
  return UART_DataAvailable(&UART0_Resources);
}

static bool UART0_TxDone(void)
{
  return UART_TxDone(&UART0_Resources);
}

static ARM_UART_STATUS UART0_SetTxThreshold(uint32_t level)
{
  return UART_SetTxThreshold(&UART0_Resources, level);
}

static ARM_UART_STATUS UART0_SetRxThreshold(uint32_t level)
{
  return UART_SetRxThreshold(&UART0_Resources, level);
}

static ARM_UART_STATUS UART0_FlushTxBuffer(void)
{
  return UART_FlushTxBuffer(&UART0_Resources);
}

static ARM_UART_STATUS UART0_FlushRxBuffer(void)
{
  return UART_FlushRxBuffer(&UART0_Resources);
}

static ARM_UART_COM_ERROR UART0_GetComError(void)
{
  return UART_GetComError(&UART0_Resources);
}

static ARM_UART_STATUS UART0_SetComBreak(void)
{
  return UART_SetComBreak(&UART0_Resources);
}

static ARM_UART_STATUS UART0_ClearComBreak(void)
{
  return UART_ClearComBreak(&UART0_Resources);
}

static ARM_UART_STATUS UART0_SetModemControl(ARM_UART_MODEM_CONTROL control)
{
  return UART_SetModemControl(&UART0_Resources, control);
}

static ARM_UART_MODEM_STATUS UART0_GetModemStatus(void)
{
  return UART_GetModemStatus(&UART0_Resources);
}

void UART0_IRQHandler(void)
{
  UART_IRQHandler(&UART0_Resources);
}

ARM_DRIVER_UART Driver_UART0 = {
  UARTX_GetVersion,
  UARTX_GetCapabilities,
  UART0_Initialize,
  UART0_Uninitialize,
  UART0_PowerControl,
  UART0_Configure,
  UART0_WriteData,
  UART0_ReadData,
  UART0_DataAvailable,
  UART0_TxDone,
  UART0_SetTxThreshold,
  UART0_SetRxThreshold,
  UART0_FlushTxBuffer,
  UART0_FlushRxBuffer,
  UART0_GetComError,
  UART0_SetComBreak,
  UART0_ClearComBreak,
  UART0_SetModemControl,
  UART0_GetModemStatus
};
#endif /* ARM_UART0 */

#if RTE_UART1
static ARM_UART_STATUS UART1_Initialize(ARM_UART_SignalEvent_t cb_event,        
                                        uint32_t               event_mask)
{
  return UART_Initialize(&UART1_Resources, cb_event, event_mask);
}

static ARM_UART_STATUS UART1_Uninitialize(void)
{
  return UART_Uninitialize(&UART1_Resources);
}

static ARM_UART_STATUS UART1_PowerControl(ARM_POWER_STATE state)
{
  return UART_PowerControl(&UART1_Resources, state);
}

static ARM_UART_STATUS UART1_Configure(uint32_t              baudrate,            
                                       uint8_t               data_bits,
                                       ARM_UART_PARITY       parity,
                                       ARM_UART_STOP_BITS    stop_bits,
                                       ARM_UART_FLOW_CONTROL flow_control)
{
  return UART_Configure(&UART1_Resources, baudrate, data_bits, parity, stop_bits, flow_control);
}

static int32_t UART1_WriteData(const uint8_t *data, uint32_t size)
{
  return UART_WriteData(&UART1_Resources, data, size);
}

static int32_t UART1_ReadData(uint8_t *data, uint32_t size)
{
  return UART_ReadData(&UART1_Resources, data, size);
}

static int32_t UART1_DataAvailable(void)
{
  return UART_DataAvailable(&UART1_Resources);
}

static bool UART1_TxDone(void)
{
  return UART_TxDone(&UART1_Resources);
}

static ARM_UART_STATUS UART1_SetTxThreshold(uint32_t level)
{
  return UART_SetTxThreshold(&UART1_Resources, level);
}

static ARM_UART_STATUS UART1_SetRxThreshold(uint32_t level)
{
  return UART_SetRxThreshold(&UART1_Resources, level);
}

static ARM_UART_STATUS UART1_FlushTxBuffer(void)
{
  return UART_FlushTxBuffer(&UART1_Resources);
}

static ARM_UART_STATUS UART1_FlushRxBuffer(void)
{
  return UART_FlushRxBuffer(&UART1_Resources);
}

static ARM_UART_COM_ERROR UART1_GetComError(void)
{
  return UART_GetComError(&UART1_Resources);
}

static ARM_UART_STATUS UART1_SetComBreak(void)
{
  return UART_SetComBreak(&UART1_Resources);
}

static ARM_UART_STATUS UART1_ClearComBreak(void)
{
  return UART_ClearComBreak(&UART1_Resources);
}

static ARM_UART_STATUS UART1_SetModemControl(ARM_UART_MODEM_CONTROL control)
{
  return UART_SetModemControl(&UART1_Resources, control);
}

static ARM_UART_MODEM_STATUS UART1_GetModemStatus(void)
{
  return UART_GetModemStatus(&UART1_Resources);
}

void UART1_IRQHandler(void)
{
  UART_IRQHandler(&UART1_Resources);
}

ARM_DRIVER_UART Driver_UART1 = {
  UARTX_GetVersion,
  UARTX_GetCapabilities,
  UART1_Initialize,
  UART1_Uninitialize,
  UART1_PowerControl,
  UART1_Configure,
  UART1_WriteData,
  UART1_ReadData,
  UART1_DataAvailable,
  UART1_TxDone,
  UART1_SetTxThreshold,
  UART1_SetRxThreshold,
  UART1_FlushTxBuffer,
  UART1_FlushRxBuffer,
  UART1_GetComError,
  UART1_SetComBreak,
  UART1_ClearComBreak,
  UART1_SetModemControl,
  UART1_GetModemStatus
};
#endif /* ARM_UART1 */

#if RTE_UART2
static ARM_UART_STATUS UART2_Initialize(ARM_UART_SignalEvent_t cb_event,        
                                        uint32_t               event_mask)
{
  return UART_Initialize(&UART2_Resources, cb_event, event_mask);
}

static ARM_UART_STATUS UART2_Uninitialize(void)
{
  return UART_Uninitialize(&UART2_Resources);
}

static ARM_UART_STATUS UART2_PowerControl(ARM_POWER_STATE state)
{
  return UART_PowerControl(&UART2_Resources, state);
}

static ARM_UART_STATUS UART2_Configure(uint32_t              baudrate,            
                                       uint8_t               data_bits,
                                       ARM_UART_PARITY       parity,
                                       ARM_UART_STOP_BITS    stop_bits,
                                       ARM_UART_FLOW_CONTROL flow_control)
{
  return UART_Configure(&UART2_Resources, baudrate, data_bits, parity, stop_bits, flow_control);
}

static int32_t UART2_WriteData(const uint8_t *data, uint32_t size)
{
  return UART_WriteData(&UART2_Resources, data, size);
}

static int32_t UART2_ReadData(uint8_t *data, uint32_t size)
{
  return UART_ReadData(&UART2_Resources, data, size);
}

static int32_t UART2_DataAvailable(void)
{
  return UART_DataAvailable(&UART2_Resources);
}

static bool UART2_TxDone(void)
{
  return UART_TxDone(&UART2_Resources);
}

static ARM_UART_STATUS UART2_SetTxThreshold(uint32_t level)
{
  return UART_SetTxThreshold(&UART2_Resources, level);
}

static ARM_UART_STATUS UART2_SetRxThreshold(uint32_t level)
{
  return UART_SetRxThreshold(&UART2_Resources, level);
}

static ARM_UART_STATUS UART2_FlushTxBuffer(void)
{
  return UART_FlushTxBuffer(&UART2_Resources);
}

static ARM_UART_STATUS UART2_FlushRxBuffer(void)
{
  return UART_FlushRxBuffer(&UART2_Resources);
}

static ARM_UART_COM_ERROR UART2_GetComError(void)
{
  return UART_GetComError(&UART2_Resources);
}

static ARM_UART_STATUS UART2_SetComBreak(void)
{
  return UART_SetComBreak(&UART2_Resources);
}

static ARM_UART_STATUS UART2_ClearComBreak(void)
{
  return UART_ClearComBreak(&UART2_Resources);
}

static ARM_UART_STATUS UART2_SetModemControl(ARM_UART_MODEM_CONTROL control)
{
  return UART_SetModemControl(&UART2_Resources, control);
}

static ARM_UART_MODEM_STATUS UART2_GetModemStatus(void)
{
  return UART_GetModemStatus(&UART2_Resources);
}

void UART2_IRQHandler(void)
{
  UART_IRQHandler(&UART2_Resources);
}

ARM_DRIVER_UART Driver_UART2 = {
  UARTX_GetVersion,
  UARTX_GetCapabilities,
  UART2_Initialize,
  UART2_Uninitialize,
  UART2_PowerControl,
  UART2_Configure,
  UART2_WriteData,
  UART2_ReadData,
  UART2_DataAvailable,
  UART2_TxDone,
  UART2_SetTxThreshold,
  UART2_SetRxThreshold,
  UART2_FlushTxBuffer,
  UART2_FlushRxBuffer,
  UART2_GetComError,
  UART2_SetComBreak,
  UART2_ClearComBreak,
  UART2_SetModemControl,
  UART2_GetModemStatus
};
#endif /* ARM_UART2 */
