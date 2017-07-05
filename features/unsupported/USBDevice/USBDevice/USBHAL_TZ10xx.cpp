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

#if defined(TARGET_BLUENINJA_CDP_TZ01B)
#include "USBHAL.h"

#ifdef __cplusplus
extern "C" {
#endif 

/* RTEDriver */
#include "RTE_Device.h"
#include "PMU_TZ10xx.h"
/* usb driver, usb define */
#include "USBD_TZ10xx.h"
#include "usb_desc.h"
#include "usb_device.h"
#include "ip_config.h"
/* misc */
#include "usb_misc.h"
#include <string.h>
#include "usbdebug.h"
#include "pmu.h"

#ifdef __cplusplus
}
#endif

#if RTE_USB2FS_DEBUG
#define USBD_DEBUG
#endif

#ifdef USBD_DEBUG
#define _logd(fmt, ...) printf(fmt"%s", __VA_ARGS__)
#define logd(...) _logd(__VA_ARGS__, "")
#else
#define logd(...)
#endif

#define TRACE_LOG(ERR_CODE)                                                 \
do {                                                                        \
    if(ERR_CODE != 0) { /* ERR_CODE != ARM_USBD_OK or ERR_CODE != PMU_OK */ \
        logd("Func: %s error %d line %d\r\n", __func__, ERR_CODE, __LINE__);\
    }                                                                       \
} while(0);

#define USBD_ASSERT(expr)                                                   \
do {                                                                        \
    if(expr) {                                                              \
        logd("Error in %s line %d\r\n", __FILE__, __LINE__);                \
    }                                                                       \
} while(0);

/* USB device interrupt event */
enum USBDISREvent {
    USBD_EVENT_NONE = 0,
    USBD_EVENT_EP0SETUP,
    USBD_EVENT_EP0IN,
    USBD_EVENT_EP0OUT,
    USBD_EVENT_EP_XFER_COMPLETED,
    USBD_EVENT_SOF,
    USBD_EVENT_BUSRESET,
    USBD_EVENT_SUSPEND_STATE_CHANGED
};
/* Enpoint status */
enum EPStatus {
    NOT_CONFIGURED = 0,
    IDLE = 1,
    READ_PENDING = 2,
    WRITE_PENDING = 3,
    READ_COMPLETE = 4,
    WRITE_COMPLETE = 5,
    STALLED = 6,
    FAILED_INVALID = 7
};

typedef struct {
    EPStatus status;                /* Status of endpoint */
    uint32_t total_xferred;         /* Accumulate byte sent */
    uint32_t data_length;           /* Length of data being read or write */
    uint32_t max_packet;            /* The length is read or written */
    uint8_t *data_buf;              /* Endpoint buffer pointer */
} ep_state_t;

/* USB descriptors for EP0 open */
static const USB_edesc_t USBS_desc_EP0out = {
    .bLength          = USB_EDESC_SIZE,
    .bDescriptorType  = USB_DESC_ENDPOINT,
    .bEndpointAddress = USB_EP_DIR_OUT | 0,
    .bmAttributes     = USB_EP_CTRL,
    .wMaxPacketSize   = {MAX_PACKET_SIZE_EP0, 0},
    .bInterval        = 0
};

static const USB_edesc_t USBS_desc_EP0in = {
    .bLength          = USB_EDESC_SIZE,
    .bDescriptorType  = USB_DESC_ENDPOINT,
    .bEndpointAddress = USB_EP_DIR_IN | 0,
    .bmAttributes     = USB_EP_CTRL,
    .wMaxPacketSize   = {MAX_PACKET_SIZE_EP0, 0},
    .bInterval        = 0
};

static USBDISREvent g_usb_isr_event;
static uint32_t g_usb_isr_event_prams;
static uint8_t g_ep0_setup_data[8];
static void (*g_isr_ptr)() = NULL;
static bool g_usb_is_locked;
static ep_state_t ep_state[NUMBER_OF_ENDPOINTS];
/* Data buffer for all enpoints */
static uint8_t ep_data_buf[NUMBER_OF_ENDPOINTS][64] TZ1K_ALIGNED((4));
/* USBD driver if */
static void USBD_EndpointEvent_callback(uint8_t ep_addr, ARM_USBD_EP_EVENT ep_event);
static void USBD_DeviceEvent_callback(ARM_USBD_EVENT event);
static void USBD_ExecuteISREvent(USBDISREvent event, uint32_t params);
static ARM_USBD_STATUS USBD_EndpointTransfer(uint8_t ep_addr, uint8_t * data, uint32_t  num);
static EP_STATUS USBD_InternEndpointRead(uint8_t ep, uint32_t maxSize);
static void USBD_EP0InternWrite();
static bool USBD_Lock(void);
static bool USBD_Unlock(void);
static inline bool is_aligned(const void *pointer, size_t size)
{
    return ((uintptr_t)pointer % size == 0);
}

USBHAL * USBHAL::instance;

USBHAL::USBHAL(void)
{
    epCallback[0] = &USBHAL::EP1_OUT_callback;
    epCallback[1] = &USBHAL::EP1_IN_callback;
    epCallback[2] = &USBHAL::EP2_OUT_callback;
    epCallback[3] = &USBHAL::EP2_IN_callback;
    epCallback[4] = &USBHAL::EP3_OUT_callback;
    epCallback[5] = &USBHAL::EP3_IN_callback;
    g_isr_ptr = &USBHAL::_usbisr;
    instance =  this;
    memset(ep_state, 0, sizeof(ep_state));
    memset(g_ep0_setup_data, 0, 8);
    g_usb_is_locked = true;
    g_usb_isr_event_prams = 0;
    g_usb_isr_event = USBD_EVENT_NONE;
    for (int i=0 ; i<NUMBER_OF_ENDPOINTS ; i++) {
        ep_state[i].data_buf = ep_data_buf[i];
    }
    PmuInit();
    TRACE_LOG(Driver_PMU.SelectClockSource(PMU_CSM_USB, PMU_CLOCK_SOURCE_PLL));
    TRACE_LOG(Driver_PMU.SetPrescaler(PMU_CD_USBB, 1));   /* 1=48MHz or 2=24MHz */
    TRACE_LOG(Driver_PMU.SetPrescaler(PMU_CD_USBI, 1));   /* 1=48MHz only */
    TRACE_LOG(Driver_USBD0.Initialize(USBD_DeviceEvent_callback, USBD_EndpointEvent_callback));
    TRACE_LOG(Driver_USBD0.PowerControl(ARM_POWER_FULL));
}

USBHAL::~USBHAL(void)
{
    TRACE_LOG(Driver_USBD0.Uninitialize());
    memset(ep_state, 0, sizeof(ep_state));
    memset(g_ep0_setup_data, 0, 8);
    g_isr_ptr = NULL;
}

void USBHAL::connect(void)
{
    TRACE_LOG(Driver_USBD0.DeviceConnect());
    ARM_USBD_STATE usbd_state = Driver_USBD0.DeviceGetState();
    if(usbd_state.active) {
        USBD_Unlock();
    } else {
        disconnect();
    }
}

void USBHAL::disconnect(void)
{
    ARM_USBD_STATE usbd_state = Driver_USBD0.DeviceGetState();
    if(usbd_state.connected) {
        USBD_Lock();
        TRACE_LOG(Driver_USBD0.DeviceDisconnect());
        TRACE_LOG(Driver_USBD0.PowerControl(ARM_POWER_OFF));
        NVIC_ClearPendingIRQ(USB_IRQn);
        USBD_Lock();
    }
}

void USBHAL::configureDevice(void)
{
    TRACE_LOG(Driver_USBD0.DeviceConfigure(true));
}

void USBHAL::unconfigureDevice(void)
{
    TRACE_LOG(Driver_USBD0.DeviceConfigure(false));
}

void USBHAL::setAddress(uint8_t address)
{
    TRACE_LOG(Driver_USBD0.DeviceSetAddress(address, ARM_USBD_SET_ADDRESS_SETUP));
}

bool USBHAL::realiseEndpoint(uint8_t endpoint, uint32_t maxPacket, uint32_t flags)
{
    ARM_USB_ENDPOINT_TYPE type;

    if (endpoint >= NUMBER_OF_ENDPOINTS) {
        return false;
    }

    if (endpoint == EP0IN || endpoint == EP0OUT) {
        return false;
    }

    if ((maxPacket > MAX_PACKET_SIZE_EPBULK) || (maxPacket > MAX_PACKET_SIZE_EPINT)) {
        return false;
    }

    ep_state[endpoint].max_packet = 0;

    switch (endpoint) {
        case EPBULK_OUT:
        case EPBULK_IN:
            type = ARM_USB_ENDPOINT_BULK;
            break;
        case EPINT_OUT:
        case EPINT_IN:
            type = ARM_USB_ENDPOINT_INTERRUPT;
            break;
        default:
            /* TZ10xx do not support Isochronous endpoint */
            return false;
    }

    if(flags & ISOCHRONOUS) {
        /* TZ10xx do not support Isochronous endpoint */
        return false;
    } else if (flags & RATE_FEEDBACK_MODE) {
        type = ARM_USB_ENDPOINT_INTERRUPT;
    }

    if(ARM_USBD_OK != Driver_USBD0.EndpointUnconfigure(USBD_EP_TO_ADDR(endpoint))) {
        return false;
    }

    if(ARM_USBD_OK != Driver_USBD0.EndpointConfigure(USBD_EP_TO_ADDR(endpoint), type, maxPacket)) {
        return false;
    }

    ep_state[endpoint].max_packet = maxPacket;
    ep_state[endpoint].status = IDLE;

    return true;
}

void USBHAL::EP0setup(uint8_t *buffer)
{
    if (buffer) {
        memcpy(buffer, g_ep0_setup_data, 8);
    }
}

void USBHAL::EP0readStage(void)
{
    /* Not need */
}

void USBHAL::EP0read(void)
{
    if(WRITE_PENDING == ep_state[EP0IN].status){
        /* Status phase */
        (void)USBD_InternEndpointRead(0, 0);
    } else {
        /* Data phase */
        (void)USBD_InternEndpointRead(0, MAX_PACKET_SIZE_EP0);
    }
}

uint32_t USBHAL::EP0getReadResult(uint8_t *buffer)
{
    uint32_t size = 0;

    if(buffer == NULL) {
        return 0;
    }

    endpointReadResult(0, buffer, &size);
    return size;
}

void USBHAL::EP0write(uint8_t *buffer, uint32_t size)
{
    USBD_ASSERT(size > MAX_PACKET_SIZE_EP0);

    ep_state[EP0IN].status = WRITE_PENDING;
    if (buffer && !is_aligned(buffer,4)) {
        /* Copy unaligned data to write-buffer before sending */
        memcpy(ep_state[EP0IN].data_buf, buffer, size);
        TRACE_LOG(USBD_EndpointTransfer(USB_ep_addr(0, 1), ep_state[EP0IN].data_buf, size));
    } else {
        TRACE_LOG(USBD_EndpointTransfer(USB_ep_addr(0, 1), buffer, size));
    }
    /* We suppose the writing process is success */
    ep_state[EP0IN].total_xferred = 0;
    ep_state[EP0IN].data_length = size;
}

void USBHAL::EP0stall(void)
{
    TRACE_LOG(Driver_USBD0.EndpointStall(USB_ep_addr(0,1), true));
    TRACE_LOG(Driver_USBD0.EndpointStall(USB_ep_addr(0,0), true));
}

EP_STATUS USBHAL::endpointRead(uint8_t endpoint, uint32_t maximumSize)
{
    return USBD_InternEndpointRead(endpoint, maximumSize);
}

EP_STATUS USBHAL::endpointReadResult(uint8_t endpoint, uint8_t *data, uint32_t *bytesRead)
{
    if (endpoint >= NUMBER_OF_ENDPOINTS) {
        return EP_INVALID;
    }

    if (!data || !bytesRead) {
        return EP_INVALID;
    }

    switch (ep_state[endpoint].status) {
        case READ_PENDING:
            return EP_PENDING;
        case READ_COMPLETE:
            memcpy(data, ep_state[endpoint].data_buf, ep_state[endpoint].total_xferred);
            *bytesRead = ep_state[endpoint].total_xferred;
            ep_state[endpoint].status = IDLE;
            return EP_COMPLETED;
        case STALLED:
            return EP_STALLED;
        default:
            ep_state[endpoint].status = IDLE;
            return EP_INVALID;
    }
}

EP_STATUS USBHAL::endpointWrite(uint8_t endpoint, uint8_t *data, uint32_t size)
{
    ARM_USBD_STATUS status;

    if (endpoint >= NUMBER_OF_ENDPOINTS || endpoint <= EP0IN) {
        return EP_INVALID;
    }

    if ((size > ep_state[endpoint].max_packet) || !data) {
        return EP_INVALID;
    }

    if(ep_state[endpoint].status == STALLED) {
        return EP_STALLED;
    }

    memcpy(ep_state[endpoint].data_buf, data, size);

    ep_state[endpoint].status = WRITE_PENDING;

    status = USBD_EndpointTransfer(USBD_EP_TO_ADDR(endpoint), ep_state[endpoint].data_buf, size);

    if (status != ARM_USBD_OK) {
        ep_state[endpoint].status = IDLE;
        return EP_INVALID;
    }

    ep_state[endpoint].total_xferred = 0;
    ep_state[endpoint].data_length = size;

    return EP_PENDING;
}

EP_STATUS USBHAL::endpointWriteResult(uint8_t endpoint)
{
    if (endpoint >= NUMBER_OF_ENDPOINTS  || endpoint <= EP0IN) {
        return EP_INVALID;
    }

    switch (ep_state[endpoint].status) {
        case WRITE_PENDING:
            return EP_PENDING;
        case WRITE_COMPLETE:
            ep_state[endpoint].status = IDLE;
            return EP_COMPLETED;
        case STALLED:
            return EP_STALLED;
        default:
            ep_state[endpoint].status = IDLE;
            return EP_INVALID;
    }
}

void USBHAL::stallEndpoint(uint8_t endpoint)
{
    USBD_ASSERT(endpoint < NUMBER_OF_ENDPOINTS);
    USBD_ASSERT((endpoint != EP0OUT) && (endpoint != EP0IN));
    TRACE_LOG(Driver_USBD0.EndpointStall(USBD_EP_TO_ADDR(endpoint), true));
    ep_state[endpoint].status = STALLED;
}

void USBHAL::unstallEndpoint(uint8_t endpoint)
{
    USBD_ASSERT(endpoint < NUMBER_OF_ENDPOINTS);
    USBD_ASSERT((endpoint != EP0OUT) && (endpoint != EP0IN));
    TRACE_LOG(Driver_USBD0.EndpointStall(USBD_EP_TO_ADDR(endpoint), false));
    ep_state[endpoint].status = IDLE;
}

bool USBHAL::getEndpointStallState(uint8_t endpoint)
{
    if (endpoint >= NUMBER_OF_ENDPOINTS) {
        return false;
    }

    if(ep_state[endpoint].status == STALLED) {
        return true;
    }

    return false;
}

void USBHAL::remoteWakeup(void)
{
    TRACE_LOG(Driver_USBD0.DeviceRemoteWakeup());
}

void USBHAL::_usbisr(void) {
    instance->usbisr();
}


void USBHAL::usbisr(void)
{
    switch (g_usb_isr_event) {
        case USBD_EVENT_EP0SETUP:
            this->EP0setupCallback();
            break;
        case USBD_EVENT_EP0IN:
            this->EP0in();
            break;
        case USBD_EVENT_EP0OUT:
            this->EP0out();
            break;
        case USBD_EVENT_BUSRESET:
            this->busReset();
            break;
        case USBD_EVENT_EP_XFER_COMPLETED:
            if (epCallback[g_usb_isr_event_prams - 2] && instance) {
               (instance->*(epCallback[g_usb_isr_event_prams - 2]))();
            }
            break;
        case USBD_EVENT_SOF:
            this->SOF(g_usb_isr_event_prams);
            break;
        case USBD_EVENT_SUSPEND_STATE_CHANGED:
            this->suspendStateChanged(g_usb_isr_event_prams);
            break;
        default:
            break;
    }
    g_usb_isr_event = USBD_EVENT_NONE;
}

static void USBD_EndpointEvent_callback(uint8_t ep_addr, ARM_USBD_EP_EVENT ep_event)
{
    uint8_t ep_dir = (((ep_addr) & ARM_USB_ENDPOINT_DIRECTION_MASK) ? 1 : 0);
    uint8_t ep_num = ((ep_addr) & ARM_USB_ENDPOINT_NUMBER_MASK);
    uint8_t ep_index = USBD_ADDR_TO_EP(ep_addr);
    uint8_t ep_xferred;

    switch(ep_event)
    {
        case ARM_USBD_EP_EVENT_SETUP:
        case ARM_USBD_EP_EVENT_OUT:
            if (0 != ep_dir) {
                return; /* Direction mismatch; */
            }
            break;
        case ARM_USBD_EP_EVENT_IN:
            if (1 != ep_dir) {
                return; /* Direction mismatch */
            }
            break;
        case (ARM_USBD_EP_EVENT)(-1):/* Error occured */
            {
                uint32_t param;
                Driver_USBD0.GetError(&param);
            }
            return;
        default:
            return; /* Unknown event */
    }

    if (ARM_USBD_EP_EVENT_SETUP != ep_event)
    {
        ep_xferred = Driver_USBD0.EndpointTransferGetResult(ep_addr);
    }

    if (0 == ep_num) {
        switch(ep_event) {
            case ARM_USBD_EP_EVENT_SETUP:
                memset(g_ep0_setup_data, 0, 8);
                TRACE_LOG(Driver_USBD0.ReadSetupPacket(g_ep0_setup_data));
                USBD_ExecuteISREvent(USBD_EVENT_EP0SETUP, 0);
                break;
            case ARM_USBD_EP_EVENT_OUT:
                if (ep_state[EP0OUT].status == READ_PENDING) {
                    ep_state[EP0OUT].status = READ_COMPLETE;
                    ep_state[EP0OUT].total_xferred = ep_xferred;

                    if (ep_xferred == 0) {
                        /* Read status after write */
                        if(ep_state[EP0IN].status == WRITE_PENDING) {
                            ep_state[EP0IN].status = WRITE_COMPLETE;
                        }
                        break;
                    }
                }
                USBD_ExecuteISREvent(USBD_EVENT_EP0OUT, 0);
                break;
            case ARM_USBD_EP_EVENT_IN:
                if (ep_state[EP0IN].status == WRITE_PENDING) {
                    ep_state[EP0IN].total_xferred += ep_xferred;
                    if (ep_xferred == 0) {
                        break;
                    } else if(ep_state[EP0IN].total_xferred < ep_state[EP0IN].data_length) {
                        USBD_EP0InternWrite();
                        break;
                    }
                }
                USBD_ExecuteISREvent(USBD_EVENT_EP0IN, 0);
                break;
            default:
                break;
        }
    } else {
        switch(ep_event) {
            case ARM_USBD_EP_EVENT_SETUP:
                return;
            case ARM_USBD_EP_EVENT_OUT:
                if (ep_state[ep_index].status == READ_PENDING) {
                    ep_state[ep_index].status = READ_COMPLETE;
                    ep_state[ep_index].total_xferred = ep_xferred;
                } else {
                    ep_state[ep_index].status = FAILED_INVALID;
                }
                break;
            case ARM_USBD_EP_EVENT_IN:
                if (ep_state[ep_index].status == WRITE_PENDING) {
                    ep_state[ep_index].total_xferred += ep_xferred;
                    if(ep_state[ep_index].total_xferred < ep_state[ep_index].data_length) {
                        /* lost data */
                        ep_state[ep_index].status = FAILED_INVALID;
                    } else {
                        ep_state[ep_index].status = WRITE_COMPLETE;
                    }
                } else {
                    ep_state[ep_index].status = FAILED_INVALID;
                }
                break;
            default:
                break;
        }

        if (ep_state[ep_index].status != FAILED_INVALID) {
            USBD_ExecuteISREvent(USBD_EVENT_EP_XFER_COMPLETED, ep_index);
        }
    }
}

static void USBD_DeviceEvent_callback(ARM_USBD_EVENT event)
{
    switch (event) {
        case ARM_USBD_EVENT_RESET:
            TRACE_LOG(Driver_USBD0.EndpointUnconfigure (USB_ep_addr(0,0)));
            TRACE_LOG(Driver_USBD0.EndpointUnconfigure (USB_ep_addr(0,1)));
            TRACE_LOG(Driver_USBD0.EndpointConfigure (USBS_desc_EP0out.bEndpointAddress, (ARM_USB_ENDPOINT_TYPE)USB_EP_GETTYPE(USBS_desc_EP0out.bmAttributes), *(uint16_t *)USBS_desc_EP0out.wMaxPacketSize));
            TRACE_LOG(Driver_USBD0.EndpointConfigure (USBS_desc_EP0in.bEndpointAddress, (ARM_USB_ENDPOINT_TYPE)USB_EP_GETTYPE(USBS_desc_EP0in.bmAttributes), *(uint16_t *)USBS_desc_EP0in.wMaxPacketSize));
            USBD_ExecuteISREvent(USBD_EVENT_BUSRESET, 0);
            break;
        case ARM_USBD_EVENT_SUSPEND:
            USBD_ExecuteISREvent(USBD_EVENT_SUSPEND_STATE_CHANGED, 1);
            break;
        case ARM_USBD_EVENT_RESUME:
            USBD_ExecuteISREvent(USBD_EVENT_SUSPEND_STATE_CHANGED, 0);
            break;
        case ARM_USBD_EVENT_REMOTE_WAKEUP:
            break;
        case (ARM_USBD_EVENT)(-1): /* Error occured */
            {
                uint32_t param;
                Driver_USBD0.GetError(&param);
            }
            break;
        case (ARM_USBD_EVENT)(-2): /* SOF Received */
            uint16_t frame;
            frame = Driver_USBD0.GetFrameNumber();
            USBD_ExecuteISREvent(USBD_EVENT_SOF, frame);
            break;
        default:
            break;
    }
}

static void USBD_ExecuteISREvent(USBDISREvent event, uint32_t params)
{
    g_usb_isr_event = event;
    g_usb_isr_event_prams = params;
    g_isr_ptr();
}

static ARM_USBD_STATUS USBD_EndpointTransfer(uint8_t ep_addr, uint8_t* data, uint32_t  num)
{
    ARM_USBD_STATUS status;
    uint8_t ep_dir = (((ep_addr) & ARM_USB_ENDPOINT_DIRECTION_MASK) ? 1 : 0);

    if (ep_dir) {
        status = Driver_USBD0.EndpointWriteStart(ep_addr, data, num);
    } else {
        status = Driver_USBD0.EndpointReadStart(ep_addr, data, num);
    }

    return status;
}

static EP_STATUS USBD_InternEndpointRead(uint8_t ep, uint32_t maxSize)
{
    ARM_USBD_STATUS status;

    if (ep >= NUMBER_OF_ENDPOINTS) {
        return EP_INVALID;
    }

    if(ep_state[ep].status == STALLED) {
        return EP_STALLED;
    }
    
    ep_state[ep].status = READ_PENDING;

    status = USBD_EndpointTransfer(USBD_EP_TO_ADDR(ep), ep_state[ep].data_buf, maxSize);

    if (status == ARM_USBD_OK) {
        return EP_PENDING;
    } else {
        return EP_INVALID;
    }
}

static bool USBD_Lock(void)
{
    bool before = g_usb_is_locked;
    NVIC_DisableIRQ(USB_IRQn);
    g_usb_is_locked = true;
    return before;
}

static bool USBD_Unlock(void)
{
    bool before = g_usb_is_locked;
    g_usb_is_locked = false;
    NVIC_EnableIRQ(USB_IRQn);
    return before;
}

static void USBD_EP0InternWrite()
{
    uint32_t size = ep_state[EP0IN].data_length - ep_state[EP0IN].total_xferred;
    TRACE_LOG(USBD_EndpointTransfer(USB_ep_addr(0, 1), &ep_state[EP0IN].data_buf[ep_state[EP0IN].total_xferred - 1], size));
}
#endif
