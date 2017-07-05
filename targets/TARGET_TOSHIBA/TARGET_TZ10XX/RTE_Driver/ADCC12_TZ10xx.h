/**
 * @file ADCC12_TZ10xx.h
 * @brief a header file for TZ10xx ADCC12 driver
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */

#ifndef ADCC12_TZ10XX_H
#define ADCC12_TZ10XX_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "RTE_Device.h"
#include "Driver_Common.h"

#define ARM_DRIVER_VERSION_MAJOR_MINOR(major,minor) (((major) << 8) | (minor))

#ifdef __cplusplus
extern "C" {
#endif

/* API version */
#define TZ1000_ADCC12_API_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 27)

#define ADCC12_MAX_CH 4 /* Channel num */

typedef enum _ADCC12_STATUS {
    ADCC12_OK,
    ADCC12_ERROR,
    ADCC12_ERROR_PARAM,
    ADCC12_ERROR_FIFO_EMPTY
} ADCC12_STATUS;

/* ADCC12 Channel */
typedef enum _ADCC12_CHANNEL {
    ADCC12_CHANNEL_0 = (1<<0),
    ADCC12_CHANNEL_1 = (1<<1),
    ADCC12_CHANNEL_2 = (1<<2),
    ADCC12_CHANNEL_3 = (1<<3),

    ADCC12_CHANNEL_MAX
} ADCC12_CHANNEL;

typedef enum _ADCC12_SAMPLING_PERIOD {
    ADCC12_SAMPLING_PERIOD_1MS = 1,
    ADCC12_SAMPLING_PERIOD_2MS,
    ADCC12_SAMPLING_PERIOD_4MS,
    ADCC12_SAMPLING_PERIOD_8MS,
    ADCC12_SAMPLING_PERIOD_16MS,
    ADCC12_SAMPLING_PERIOD_32MS,
    ADCC12_SAMPLING_PERIOD_64MS,
    ADCC12_SAMPLING_PERIOD_1S,

    ADCC12_SAMPLING_PERIOD_MAX
} ADCC12_SAMPLING_PERIOD;


/* Interrupt Enable */
typedef enum _ADCC12_EVENT {
    ADCC12_EVENT_CONV_END    = (1<<0),     /* Conversion End */
    ADCC12_EVENT_SCAN_END    = (1<<1),     /* Scan End (for One-Time, Cyclic Mode) */
    ADCC12_EVENT_CMPINT_0    = (1<<4),     /* Compare 0 */
    ADCC12_EVENT_CMPINT_1    = (1<<5)      /* Compare 1 */
} ADCC12_EVENT;

typedef enum _ADCC12_CHANNEL_EVENT {
    ADCC12_CHANNEL_EVENT_CONV_END      = (1<<0),    /* Convert end */
    ADCC12_CHANNEL_EVENT_WATERMARK     = (1<<1),    /* Watermark */
    ADCC12_CHANNEL_EVENT_FDATA_LOST    = (1<<2),    /* First Data Lost */
    ADCC12_CHANNEL_EVENT_LDATA_LOST    = (1<<3),    /* Last Data Lost */
    ADCC12_CHANNEL_EVENT_FIFO_FULL     = (1<<4),    /* FIFO Full */
    ADCC12_CHANNEL_EVENT_FIFO_UNDERRUN = (1<<5),    /* FIFO Under Run */
    ADCC12_CHANNEL_EVENT_DMA_COMPLETE  = (1<<6)     /* DMA transfer complete */
} ADCC12_CHANNEL_EVENT;

/* Scan Mode */
typedef enum _ADCC12_SCAN_MODE {
    ADCC12_SCAN_MODE_SINGLE  = 0,
    ADCC12_SCAN_MODE_CYCLIC,
    ADCC12_SCAN_MODE_DISABLE
} ADCC12_SCAN_MODE;

/* Comparetion */
typedef enum _ADCC12_CMP_DATA_SELECT {
    ADCC12_CMP_DATA_0 = 0,
    ADCC12_CMP_DATA_1
} ADCC12_CMP_DATA_SELECT;

typedef enum _ADCC12_CMP_CAUSE_SELECT {
    ADCC12_CMP_DATA_LT_CONV = 0,
    ADCC12_CMP_DATA_GT_CONV,
    ADCC12_CMP_NO_COMPARISON,

    ADCC12_CMP_DATA_MAX
} ADCC12_CMP_CAUSE_SELECT;

/* FIFO Mode */
typedef enum _ADCC12_FIFO_MODE {
    ADCC12_FIFO_MODE_STREAM = 0,
    ADCC12_FIFO_MODE_STOP
} ADCC12_FIFO_MODE;

typedef enum _ADCC12_DOWN_SAMPLING {
    ADCC12_DOWN_SAMPLING_1_1 = 0,    /* Not Down Sampling */
    ADCC12_DOWN_SAMPLING_1_2,        /* Every 2nd */
    ADCC12_DOWN_SAMPLING_1_3,        /* Every 3rd */
    ADCC12_DOWN_SAMPLING_1_4,        /* Every 4th */
    ADCC12_DOWN_SAMPLING_1_5,        /* Every 5th */
    ADCC12_DOWN_SAMPLING_1_6,        /* Every 6th */
    ADCC12_DOWN_SAMPLING_1_7,        /* Every 7th */
    ADCC12_DOWN_SAMPLING_1_8,        /* Every 8th */

    ADCC12_DOWN_SAMPLING_MAX
} ADCC12_DOWN_SAMPLING;

typedef enum _ADCC12_FIFO_WATERMARK {
    ADCC12_WATERMARK_1 = 0, /* 1st */
    ADCC12_WATERMARK_2,     /* 2nd */
    ADCC12_WATERMARK_3,     /* 3rd */
    ADCC12_WATERMARK_4,     /* 4th */
    ADCC12_WATERMARK_5,     /* 5th */
    ADCC12_WATERMARK_6,     /* 6th */
    ADCC12_WATERMARK_7,     /* 7th */
    ADCC12_WATERMARK_8,     /* 8th */

    ADCC12_WATERMARK_MAX
} ADCC12_FIFO_WATERMARK;

typedef enum _ADCC12_DATA_FORMAT {
    ADCC12_UNSIGNED = 0,
    ADCC12_SIGNED = 1
} ADCC12_DATA_FORMAT;

typedef void (*ADCC12_SignalEvent_t)(ADCC12_EVENT event);
typedef void (*ADCC12_ChannelEvent_t)(ADCC12_CHANNEL ch, ADCC12_CHANNEL_EVENT event);


typedef struct _TZ10XX_DRIVER_ADCC12 {
    ARM_DRIVER_VERSION     (*GetVersion)              (void);
    ADCC12_STATUS       (*Initialize)              (ADCC12_SignalEvent_t cb_event,
                                                    uint32_t             event_mask);
    ADCC12_STATUS       (*Uninitialize)            (void);
    ADCC12_STATUS       (*PowerControl)            (ARM_POWER_STATE state);
    ADCC12_STATUS       (*SetScanMode)             (ADCC12_SCAN_MODE scan_mode, uint32_t ch);
    ADCC12_STATUS       (*SetFIFOOverwrite)        (ADCC12_FIFO_MODE fifo_mode);
    ADCC12_STATUS       (*SetComparison)           (ADCC12_CMP_DATA_SELECT cmp_data, int32_t cmp_data_val,
                                                    ADCC12_CMP_CAUSE_SELECT cmp_cause, ADCC12_CHANNEL ch);
    ADCC12_STATUS       (*SetSamplingPeriod)       (ADCC12_SAMPLING_PERIOD sampling_period);
    ADCC12_STATUS       (*SetDataFormat)           (ADCC12_CHANNEL ch, ADCC12_DATA_FORMAT data_fmt);
    ADCC12_STATUS       (*SetOffset)               (ADCC12_CHANNEL ch, int8_t offset);
    ADCC12_STATUS       (*UseDMA)                  (ADCC12_CHANNEL ch, bool dma_en, uint16_t *data_area, uint32_t num_of_data);
    ADCC12_STATUS       (*SetDecimation)           (ADCC12_CHANNEL ch, ADCC12_DOWN_SAMPLING down_spl);
    ADCC12_STATUS       (*SetWatermark)            (ADCC12_CHANNEL ch, ADCC12_FIFO_WATERMARK watermark);
    ADCC12_STATUS       (*SetChannelEvent)         (ADCC12_CHANNEL ch, ADCC12_ChannelEvent_t cb_event,
                                                    uint32_t chevent_mask);
    ADCC12_STATUS       (*Start)                   (void);
    ADCC12_STATUS       (*Stop)                    (void);
    ADCC12_STATUS       (*ReadData)                (ADCC12_CHANNEL ch, uint16_t* adc);
    ADCC12_STATUS       (*GetNumberOfData)         (ADCC12_CHANNEL ch, uint32_t* num_of_data);
    ADCC12_STATUS       (*GetAreaInformation)      (ADCC12_CHANNEL ch, uint16_t** data_area, uint32_t* num_of_data);

} const TZ10XX_DRIVER_ADCC12;


#ifdef __cplusplus
}
#endif

#endif /* ADCC12_TZ10XX_H */
