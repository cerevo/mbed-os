/* mbed Microcontroller Library
 * Copyright 2017, Cerevo Inc. 
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "mbed_assert.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "serial_api.h"
#include "cmsis.h"
#include "pinmap.h"
#include "gpio_api.h"
#include "mbed_error.h"
#include "PeripheralPins.h"
#include "pmu.h"
#include "RTE_Device.h"
#include "Driver_UART.h"
#include "PMU_TZ10xx.h"

/******************************************************************************
 ****************************INITIALIZATION************************************
 ******************************************************************************
 */
#if (RTE_BLE)
#define UART_NUM    2
#else
#define UART_NUM    3
#endif

#define UART_RX_INTERRUPT_MASK  ((1u << 4) | (1u << 6))
#define UART_TX_INTERRUPT_MASK  ((1u << 5))
#define UART_FIFO_ENABLE_MASK   ((1u << 4))

static uart_irq_handler irq_handler = NULL;

struct serial_global_data_s {
    uint32_t serial_irq_id;
    gpio_t sw_rts, sw_cts;
    uint8_t count, rx_irq_set_flow, rx_irq_set_api, tx_irq_set_api;
};

static struct serial_global_data_s uart_data[UART_NUM];

int stdio_uart_inited = 0;
serial_t stdio_uart;

void uart0_handler(ARM_UART_EVENT e);
void uart1_handler(ARM_UART_EVENT e);

#if (!RTE_BLE)
void uart2_handler(ARM_UART_EVENT e);
#endif

void uart_handler(ARM_UART_EVENT e, uint32_t index);

#if (RTE_BLE)
static ARM_UART_SignalEvent_t hander_tbl[UART_NUM] = {
    NULL,
    NULL
};
#else
static ARM_UART_SignalEvent_t hander_tbl[UART_NUM] = {
    NULL,
    NULL,
    NULL
};
#endif

/* Flag variable to control tx and rx */
bool is_enable_tx = false;
bool is_enable_rx = false;

/* PMU Driver interface */
extern TZ10XX_DRIVER_PMU Driver_PMU;

/* UART CMSIS interface */
extern ARM_DRIVER_UART Driver_UART0;
extern ARM_DRIVER_UART Driver_UART1;

#if (!RTE_BLE)
extern ARM_DRIVER_UART Driver_UART2;
#endif

/* Internal APIs */
static void serial_disable_interrupt(serial_t *obj)
{
    UARTName uart_name;
    uart_name = (UARTName) obj->index;

    switch (uart_name) {
        case UART_0:
            uart0->UARTIMSC = 0u;
            break;
        case UART_1:
            uart1->UARTIMSC = 0u;
            break;
#if (!RTE_BLE)
        case UART_2:
            uart2->UARTIMSC = 0u;
            break;
#endif
        default:
            error("UART is not available");
            break;
    }
}

/* Internal APIs */
static void serial_clear_tx_interrupt(serial_t *obj)
{
    UARTName uart_name;
    uart_name = (UARTName) obj->index;

    switch (uart_name) {
        case UART_0:
            uart0->UARTICR_b.TXIC = 1u;
            break;
        case UART_1:
            uart1->UARTICR_b.TXIC = 1u;
            break;
#if (!RTE_BLE)
        case UART_2:
            uart2->UARTICR_b.TXIC = 1u;
            break;
#endif
        default:
            error("UART is not available");
            break;
    }
}

static void serial_clear_rx_interrupt(serial_t *obj)
{
    UARTName uart_name;
    uart_name = (UARTName) obj->index;

    switch (uart_name) {
        case UART_0:
            uart0->UARTICR_b.RXIC = 1u;
            break;
        case UART_1:
            uart1->UARTICR_b.RXIC = 1u;
            break;
#if (!RTE_BLE)
        case UART_2:
            uart2->UARTICR_b.RXIC = 1u;
            break;
#endif
        default:
            error("UART is not available");
            break;
    }
}

static void serial_disable_fifo(serial_t *obj)
{
    UARTName uart_name;
    uart_name = (UARTName) obj->index;

    switch (uart_name) {
        case UART_0:
            uart0->UARTLCR_H &= ~UART_FIFO_ENABLE_MASK;
            break;
        case UART_1:
            uart1->UARTLCR_H &= ~UART_FIFO_ENABLE_MASK;
            break;
#if (!RTE_BLE)
        case UART_2:
            uart2->UARTLCR_H &= ~UART_FIFO_ENABLE_MASK;
            break;
#endif
        default:
            error("UART is not available");
            break;
    }
}

/** Initialize the UART peripheral module.
 *  @remarks
 *    Wrapper from CMSIS UART Driver:
 *      - UART_Initialize()
 *      - UART_Configure()
 *
 *  @param  obj         The Serial object to initialize
 *  @param  tx          The pin name of Serial Tx
 *  @param  rx          The pin name of Serial Rx
 *
 *  @returns
 *    NONE.
 */
void serial_init(serial_t *obj, PinName tx, PinName rx)
{
    int is_stdio_uart = 0;
    PMU_CSM pmu_csm = PMU_CSM_UART0;
    PMU_CD  pmu_cd  = PMU_CD_UART0;

    /* determine the UART to use */
    UARTName uart_tx = (UARTName)pinmap_peripheral(tx, PinMap_UART_TX);
    UARTName uart_rx = (UARTName)pinmap_peripheral(rx, PinMap_UART_RX);
    UARTName uart_name = (UARTName)pinmap_merge(uart_tx, uart_rx);
    if ((int)uart_name == NC) {
        error("Serial pinout mapping failed");
    }

    obj->index = uart_name;

    uart_data[obj->index].tx_irq_set_api = 0;

    switch (uart_name) {
        case UART_0:
            obj->uart = &Driver_UART0;
            pmu_csm = PMU_CSM_UART0;
            pmu_cd = PMU_CD_UART0;
            hander_tbl[obj->index] = uart0_handler;
            break;
        case UART_1:
            obj->uart = &Driver_UART1;
            pmu_csm = PMU_CSM_UART1;
            pmu_cd = PMU_CD_UART1;
            hander_tbl[obj->index]  = uart1_handler;
            break;
#if (!RTE_BLE)
        case UART_2:
            obj->uart = &Driver_UART2;
            pmu_csm = PMU_CSM_UART2;
            pmu_cd = PMU_CD_UART2;
            hander_tbl[obj->index]  = uart2_handler;
            break;
#endif
        default:
            error("UART is not available");
            break;
    }

    /* Setup PMU */
    PmuInit();
    Driver_PMU.SelectClockSource(pmu_csm, PMU_CLOCK_SOURCE_OSC12M);
    Driver_PMU.SetPrescaler(pmu_cd, 1);

    /* 
     * There is callback function in default setting
     * But callback function does nothing
     */
    obj->uart->Initialize(hander_tbl[obj->index],
                          (1 << ARM_UART_EVENT_RX_THRESHOLD)
                          |   (1 << ARM_UART_EVENT_RX_TIMEOUT)
                          |   (1 << ARM_UART_EVENT_TX_THRESHOLD)
                         );

    /* Configure UART to default settings */
    obj->baudrate = 9600;
    obj->data_bits = 8;
    obj->parity = ARM_UART_PARITY_NONE;
    obj->stop_bits = ARM_UART_STOP_BITS_1;
    obj->flow_control = ARM_UART_FLOW_CONTROL_NONE;
    obj->uart->Configure( obj->baudrate, obj->data_bits, obj->parity, obj->stop_bits, obj->flow_control );
    obj->uart->PowerControl(ARM_POWER_FULL);

    /* Disable FIFO for both mbed SDK and mbed OS */
    /* Disable UART FIFO */
    serial_disable_fifo(obj);
    /* Disable Interrupt */
    serial_disable_interrupt(obj);
    /* Set to RX/TX threshold value
     * Interrupt occurs when Rx/Tx fifo size reaches threshold value
     */
    obj->uart->SetRxThreshold(2);
    obj->uart->SetTxThreshold(2);

    /* pinout the chosen uart */
    pinmap_pinout(tx, PinMap_UART_TX);
    pinmap_pinout(rx, PinMap_UART_RX);

    /* Check if tx or rx NC to disbale tx, rx functions */
    if (tx != NC) {
        is_enable_tx = true;
    } else {
        is_enable_tx = false;
    }
    if (rx != NC) {
        is_enable_rx = true;
    } else {
        is_enable_rx = false;
    }

    is_stdio_uart = (uart_name == STDIO_UART) ? (1) : (0);

    if (is_stdio_uart) {
        stdio_uart_inited = 1;
        memcpy(&stdio_uart, obj, sizeof(serial_t));
    }
}

/** Uninitialize the Serial object.
 *  @remarks
 *    Wrapper from CMSIS UART Driver:
 *      - UART_Uninitialize()
 *
 *  @param  obj         The SERIAL object to de-initialize
 *
 *  @returns
 *    NONE.
 */
void serial_free(serial_t *obj)
{
#if (!RTE_BLE)
    MBED_ASSERT( (obj->index == UART_0) ||  (obj->index == UART_1)  || (obj->index == UART_2));
#else
    MBED_ASSERT( (obj->index == UART_0) ||  (obj->index == UART_1));
#endif
    MBED_ASSERT( obj->uart != NULL );

    obj->uart->Uninitialize();
    obj->uart = NULL;
}

/** Set baud rate for UART.
 *  @remarks
 *    Wrapper from CMSIS UART Driver:
 *      - UART_Configure()
 *
 *  @param  obj         The Serial object to set baudrate
 *  @param  baudrate    Value of new baudrate
 *
 *  @returns
 *    NONE.
 */
void serial_baud(serial_t *obj, int baudrate)
{
    ARM_UART_STATUS sts;

#if (!RTE_BLE)
    MBED_ASSERT( (obj->index == UART_0) ||  (obj->index == UART_1)  || (obj->index == UART_2));
#else
    MBED_ASSERT( (obj->index == UART_0) ||  (obj->index == UART_1));
#endif
    MBED_ASSERT( obj->uart != NULL );
    /* Make sure tx buffer is empty before changing port configuration */
    while (!serial_writable(obj));
    /* Save value of new baudrate */
    obj->baudrate = baudrate;
    /* Disable FIFO for both mbed SDK and mbed OS */
    /* Set new baudrate */
    obj->uart->PowerControl(ARM_POWER_LOW);
    sts = obj->uart->Configure( obj->baudrate, obj->data_bits, obj->parity, obj->stop_bits, obj->flow_control );
    obj->uart->PowerControl(ARM_POWER_FULL);
    if (sts != ARM_UART_OK) {
        error("UART changing baudrate error");
    }

    /* Disable UART FIFO */
    serial_disable_fifo(obj);
    /* Disable Interrupt */
    serial_disable_interrupt(obj);
}

/** Set new format for Serial object.
 *  @remarks
 *    Wrapper from CMSIS UART Driver:
 *      - UART_Configure()
 *
 *  @param  obj         The Serial object to set format
 *  @param  data_bits   Value of new data bits
 *  @param  parity      Value of new parity
 *  @param  stop_bits   Value of new stop bits
 *
 *  @returns
 *    NONE.
 */
void serial_format(serial_t *obj, int data_bits, SerialParity parity, int stop_bits)
{
#if (!RTE_BLE)
    MBED_ASSERT( (obj->index == UART_0) ||  (obj->index == UART_1)  || (obj->index == UART_2));
#else
    MBED_ASSERT( (obj->index == UART_0) ||  (obj->index == UART_1));
#endif
    MBED_ASSERT( obj->uart != NULL );
    MBED_ASSERT( (data_bits == 5) || (data_bits == 6) || (data_bits ==7 || (data_bits == 8)));

    /* 
     * ParityForced1 = ARM_UART_PARITY_MARK (PEN=1, EPS=0, SPS=1)
     * ParityForced0 = ARM_UART_PARITY_SPACE (PEN=1, EPS=1, SPS=1)
     */
    /* Save new databit, parity, baudrate */
    obj->data_bits = data_bits;
    obj->parity = (ARM_UART_PARITY)parity;    /*enum SerialParity is same as ARM_UART_PARITY*/
    obj->stop_bits = (ARM_UART_STOP_BITS)stop_bits;
    /* Set new databit, parity, baudrate */
    obj->uart->Configure( obj->baudrate, obj->data_bits, obj->parity, obj->stop_bits, obj->flow_control );
    /* Disable FIFO for both mbed SDK and mbed OS */
    /* Disable UART FIFO */
    serial_disable_fifo(obj);
    /* Disable Interrupt */
    serial_disable_interrupt(obj);
}

/******************************************************************************
 * INTERRUPTS HANDLING
 ******************************************************************************/
void uart_handler(ARM_UART_EVENT e, uint32_t index)
{
    if (hander_tbl[index] == NULL) {
        /* UART handler is disable, do nothing */
        return;
    }

    SerialIrq irq_type;

    switch (e) {
        case ARM_UART_EVENT_RX_THRESHOLD:
        case ARM_UART_EVENT_RX_TIMEOUT:
            irq_type = RxIrq;
            break;
        case ARM_UART_EVENT_TX_THRESHOLD:
            irq_type = TxIrq;
            break;
        default:
            return;
    }

    if (uart_data[index].serial_irq_id != 0) {
        irq_handler(uart_data[index].serial_irq_id, irq_type);
    }
}

void uart0_handler(ARM_UART_EVENT e)
{
    uart_handler(e, 0);
}

void uart1_handler(ARM_UART_EVENT e)
{
    uart_handler(e, 1);
}

#if (!RTE_BLE)
void uart2_handler(ARM_UART_EVENT e)
{
    uart_handler(e, 2);
}
#endif

/** Set handler for all serial interrupts.
 *  @remarks
 *    This function only updates value Serial object, no hardware configuration
 *
 *  @param  obj         The Serial object to set IRQ handler
 *  @param  handler     Interrupt handler callback function
 *  @param  id          UNIX ID of interrupt serial
 *
 *  @returns
 *    NONE.
 */
void serial_irq_handler(serial_t *obj, uart_irq_handler handler, uint32_t id)
{
    irq_handler = handler;
    uart_data[obj->index].serial_irq_id = id;
}

/** Set up IRQ for serial based on IRQ type.
 *  @remarks
 *    Wrapper from CMSIS UART Driver:
 *      - UART_Initialize()
 *      - UART_Uninitialize()
 *
 *  @param  obj         The Serial object to set IRQ handler
 *  @param  irq         IRQ type
 *  @param  enable      Enable or disable IRQ
 *
 *  @returns
 *    NONE.
 */
void serial_irq_set(serial_t *obj, SerialIrq irq, uint32_t enable)
{
    if (!uart_data[obj->index].rx_irq_set_api) {
        serial_clear_rx_interrupt(obj);
    }

    if (!uart_data[obj->index].tx_irq_set_api) {
        serial_clear_tx_interrupt(obj);
    }

    if (RxIrq == irq) {
        
        uart_data[obj->index].rx_irq_set_api = enable;
        
        switch (obj->index) {
            case 0:
                if (enable) {
                    uart0->UARTIMSC |= UART_RX_INTERRUPT_MASK;
                } else {
                    uart0->UARTIMSC &= ~UART_RX_INTERRUPT_MASK;
                }
                break;
            case 1:
                if (enable) {
                    uart1->UARTIMSC |= UART_RX_INTERRUPT_MASK;
                } else {
                    uart1->UARTIMSC &= ~UART_RX_INTERRUPT_MASK;
                }                break;
#if (!RTE_BLE)
            case 2:
                if (enable) {
                    uart2->UARTIMSC |= UART_RX_INTERRUPT_MASK;
                } else {
                    uart2->UARTIMSC &= ~UART_RX_INTERRUPT_MASK;
                }                break;
#endif  
            default:
                error("Uart index error");
                break;
        }

    } 
    
    if (TxIrq == irq) {

        uart_data[obj->index].tx_irq_set_api = enable;

        switch (obj->index) {
            case 0:
                if (enable) {
                    uart0->UARTIMSC |= UART_TX_INTERRUPT_MASK;
                } else {
                    uart0->UARTIMSC &= ~UART_TX_INTERRUPT_MASK;
                }
                break;
            case 1:
                if (enable) {
                    uart1->UARTIMSC |= UART_TX_INTERRUPT_MASK;
                } else {
                    uart1->UARTIMSC &= ~UART_TX_INTERRUPT_MASK;
                }                break;
#if (!RTE_BLE)
            case 2:
                if (enable) {
                    uart2->UARTIMSC |= UART_TX_INTERRUPT_MASK;
                } else {
                    uart2->UARTIMSC &= ~UART_TX_INTERRUPT_MASK;
                }                break;
#endif  
            default:
                error("Uart index error");
                break;
        }
    }
}

#if (DEVICE_SERIAL_FC)
static void serial_flow_irq_set(serial_t *obj, uint32_t enable)
{
    uart_data[obj->index].rx_irq_set_flow = enable;
    serial_irq_set(obj, RxIrq, enable);
    /* Disable interrupt */
    switch (obj->index) {
        case 0:
            /* Save interrupt mask for next enable */
            obj->interrupt_mask = uart0->UARTIMSC;
            uart0->UARTIMSC = 0;
            break;
        case 1:
            /* Save interrupt mask for next enable */
            obj->interrupt_mask = uart1->UARTIMSC;
            uart1->UARTIMSC = 0;
            break;
        case 2:
            /* Save interrupt mask for next enable */
            obj->interrupt_mask = uart2->UARTIMSC;
            uart2->UARTIMSC = 0;
            break;
        default:
            break;
    }
}
#endif

/******************************************************************************
 * READ/WRITE
 ******************************************************************************/
/** Get a char from Serial object.
 *  @remarks
 *    Wrapper from CMSIS UART Driver:
 *      - UART_ReadData ()
 *
 *  @param  obj         The Serial object to get character
 *
 *  @returns
 *    UART data.
 */
int serial_getc(serial_t *obj)
{
    /* Check if rx enable or not */
    if (is_enable_rx == false) {
        return 0;
    }
    uint8_t buf;

    while (serial_readable(obj) == 0) {
        /* DO NOTHING */
    }

    if (obj->uart->ReadData(&buf, 1) == 1) {
        return buf;
    } else {
        return 0;
    }
}

/** Put a char to Serial object.
 *  @remarks
 *    Wrapper from CMSIS UART Driver:
 *      - UART_WriteData ()
 *
 *  @param  obj         The Serial object to put char
 *  @param  c           Value of char
 *
 *  @returns
 *    NONE.
 */
void serial_putc(serial_t *obj, int c)
{
    /* Check if rx enable or not */
    if (is_enable_tx == false) {
        return;
    }

    uint8_t buf = (uint8_t)c;

    while (!serial_writable(obj));

    obj->uart->WriteData(&buf, 1);

    /* wait for finish tx */
    while (!serial_writable(obj));
}

/** Check ready to read data from Serial object.
 *  @remarks
 *    Wrapper from CMSIS UART Driver:
 *      - UART_DataAvailable ()
 *
 *  @param  obj         The Serial object to check
 *
 *  @returns
 *    Available data in UART.
 */
int serial_readable(serial_t *obj)
{
    return obj->uart->DataAvailable();
}

/** Check writable of UART.
 *
 *  @param  obj         The Serial object to check
 *
 *  @returns
 *    Status of UART.
 */
int serial_writable(serial_t *obj)
{
    UARTName uart_name;
    uart_name = (UARTName) obj->index;
    int status;

    switch (uart_name) {
        case UART_0:
            status = (uart0->UARTFR_b.TXFF == 0) ? 1 : 0;
            break;
        case UART_1:
            status = (uart1->UARTFR_b.TXFF == 0) ? 1 : 0;
            break;
#if (!RTE_BLE)
        case UART_2:
            status = (uart2->UARTFR_b.TXFF == 0) ? 1 : 0;
            break;
#endif
        default:
            status = 0;
            break;
    }

    return status;
}

/** Clear transmission and reception of Serial object.
 *  @remarks
 *    Wrapper from CMSIS UART Driver:
 *      - UART_FlushTxBuffer()
 *      - UART_FlushRxBuffer()
 *
 *  @param  obj         The Serial object to clear
 *
 *  @returns
 *    NONE.
 */
void serial_clear(serial_t *obj)
{
    obj->uart->FlushTxBuffer();
    obj->uart->FlushRxBuffer();
}

/** Set pinout for Tx pin.
 *
 *  @param  tx          TX pin name to set pinout
 *
 *  @returns
 *    NONE.
 */
void serial_pinout_tx(PinName tx)
{
    pinmap_pinout(tx, PinMap_UART_TX);
}

/** Break UART transmission.
 *
 *  @param  obj         The Serial object to initialize to break
 *
 *  @returns
 *    NONE.
 */
void serial_break_set(serial_t *obj)
{
    obj->uart->SetComBreak();
}

/** Switch from break transmission to normal transmission.
 *  @remarks
 *    Wrapper from CMSIS UART Driver:
 *      - UART_ClearComBreak ()
 *
 *  @param  obj         The Serial object to clear break transmission
 *
 *  @returns
 *    NONE.
 */
void serial_break_clear(serial_t *obj)
{
    obj->uart->ClearComBreak();
}

#if (DEVICE_SERIAL_FC)
/** Set flow control to Serial object.
 *  @remarks
 *    Wrapper from CMSIS UART Driver:
 *      - UART_Configure ()
 *
 *  @param  obj         The Serial object to put char
 *  @param  type        Control flow type
 *  @param  rxflow      UART RTS pin name
 *  @param  txflow      UART CTS pin name
 *
 *  @returns
 *    NONE
 */
void serial_set_flow_control(serial_t *obj, FlowControl type, PinName rxflow, PinName txflow)
{
    if (FlowControlNone == type) {
        /* Enable interrupt */
        switch (obj->index) {
            case 0:
                uart0->UARTIMSC = obj->interrupt_mask;
                break;
            case 1:
                uart1->UARTIMSC = obj->interrupt_mask;
                break;
#if (!RTE_BLE)
            case 2:
                uart2->UARTIMSC = obj->interrupt_mask;
                break;
#endif
            default:
                break;
        }

        /* Disable hardware flow control */
        obj->flow_control = ARM_UART_FLOW_CONTROL_NONE;
        obj->uart->Configure( obj->baudrate, obj->data_bits, obj->parity, obj->stop_bits, obj->flow_control );
        return;
    }

    /* Check to see HW flow control support */
    UARTName uart_cts = (UARTName)pinmap_peripheral(txflow, PinMap_UART_CTS);
    UARTName uart_rts = (UARTName)pinmap_peripheral(rxflow, PinMap_UART_RTS);
    UARTName uart_name = (UARTName)pinmap_merge(uart_cts, uart_rts);

    if ( uart_name == (UARTName)NC ) {
        return;    /* No harware control support */
    }

    /* Disable UART interrupt */
    serial_flow_irq_set(obj, 0);

    if ((FlowControlCTS == type) || (FlowControlRTSCTS== type)) {
        /* Enable the pin for CTS function */
        pinmap_pinout(txflow, PinMap_UART_CTS);
    }

    if ((FlowControlRTS == type) || (FlowControlRTSCTS== type))  {
        /* Enable the pin for RTS function */
        pinmap_pinout(rxflow, PinMap_UART_RTS);
    }

    /* Support hardware control flow only */
    obj->flow_control = ARM_UART_FLOW_CONTROL_RTS_CTS;
    obj->uart->Configure( obj->baudrate, obj->data_bits, obj->parity, obj->stop_bits, obj->flow_control );
}
#endif
