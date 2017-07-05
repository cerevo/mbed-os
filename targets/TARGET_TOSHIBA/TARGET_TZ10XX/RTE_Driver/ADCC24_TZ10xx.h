/**
 * @file ADCC24_TZ10xx.h
 * @brief a header file for TZ10xx ADCC24 driver
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */

#ifndef ADCC24_TZ10XX_H
#define ADCC24_TZ10XX_H

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
#define TZ1000_ADCC24_API_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 27)

#define ADCC24_MAX_CH 3 /* Channel num */

typedef enum _ADCC24_STATUS {
    ADCC24_OK,
    ADCC24_ERROR,
    ADCC24_ERROR_PARAM,
    ADCC24_ERROR_FIFO_EMPTY
} ADCC24_STATUS;

/* ADCC24 Channel */
typedef enum _ADCC24_CHANNEL {
    ADCC24_CHANNEL_0 = (1<<0),
    ADCC24_CHANNEL_1 = (1<<1),
    ADCC24_CHANNEL_2 = (1<<2),
} ADCC24_CHANNEL;

typedef enum _ADCC24_SAMPLING_PERIOD {
    ADCC24_SAMPLING_PERIOD_1MS = 1,
    ADCC24_SAMPLING_PERIOD_2MS,
    ADCC24_SAMPLING_PERIOD_4MS,
    ADCC24_SAMPLING_PERIOD_8MS,
    ADCC24_SAMPLING_PERIOD_16MS,
    ADCC24_SAMPLING_PERIOD_32MS,
    ADCC24_SAMPLING_PERIOD_64MS,
    ADCC24_SAMPLING_PERIOD_1S,

    ADCC24_SAMPLING_PERIOD_MAX
} ADCC24_SAMPLING_PERIOD;

/* Interrupt Enable */
typedef enum _ADCC24_EVENT {
    ADCC24_EVENT_CONV_END    = (1<<0),    /* Conversion End */
    ADCC24_EVENT_SCAN_END    = (1<<1),    /* Scan End (for One-Time, Cyclic Mode)*/
    ADCC24_EVENT_CMPINT_0    = (1<<4),    /* Compare 0 */
    ADCC24_EVENT_CMPINT_1    = (1<<5)     /* Compare 1 */
} ADCC24_EVENT;

typedef enum _ADCC24_CHANNEL_EVENT {
    ADCC24_CHANNEL_EVENT_CONV_END      = (1<<0),    /* Convert end */
    ADCC24_CHANNEL_EVENT_WATERMARK     = (1<<1),    /* Watermark */
    ADCC24_CHANNEL_EVENT_FDATA_LOST    = (1<<2),    /* First Data Lost */
    ADCC24_CHANNEL_EVENT_LDATA_LOST    = (1<<3),    /* Last Data Lost */
    ADCC24_CHANNEL_EVENT_FIFO_FULL     = (1<<4),    /* FIFO Full */
    ADCC24_CHANNEL_EVENT_FIFO_UNDERRUN = (1<<5),    /* FIFO Under Run */
    ADCC24_CHANNEL_EVENT_DMA_COMPLETE  = (1<<6)     /* DMA transfer complete */
} ADCC24_CHANNEL_EVENT;

/* Scan Mode */
typedef enum _ADCC24_SCAN_MODE {
    ADCC24_SCAN_MODE_SINGLE  = 0,
    ADCC24_SCAN_MODE_CYCLIC,
    ADCC24_SCAN_MODE_DISABLE
} ADCC24_SCAN_MODE;

/* Comparetion */
typedef enum _ADCC24_CMP_DATA_SELECT {
    ADCC24_CMP_DATA_0 = 0,
    ADCC24_CMP_DATA_1
} ADCC24_CMP_DATA_SELECT;

typedef enum _ADCC24_CMP_CAUSE_SELECT {
    ADCC24_CMP_DATA_LT_CONV = 0,
    ADCC24_CMP_DATA_GT_CONV,
    ADCC24_CMP_NO_COMPARISON,

    ADCC24_CMP_DATA_MAX
} ADCC24_CMP_CAUSE_SELECT;

/* FIFO Mode */
typedef enum _ADCC24_FIFO_MODE {
    ADCC24_FIFO_MODE_STREAM = 0,
    ADCC24_FIFO_MODE_STOP
} ADCC24_FIFO_MODE;

typedef enum _ADCC24_DOWN_SAMPLING {
    ADCC24_DOWN_SAMPLING_1_1 = 0,    /* Not Down Sampling */
    ADCC24_DOWN_SAMPLING_1_2,        /* Every 2nd */
    ADCC24_DOWN_SAMPLING_1_3,        /* Every 3rd */
    ADCC24_DOWN_SAMPLING_1_4,        /* Every 4th */
    ADCC24_DOWN_SAMPLING_1_5,        /* Every 5th */
    ADCC24_DOWN_SAMPLING_1_6,        /* Every 6th */
    ADCC24_DOWN_SAMPLING_1_7,        /* Every 7th */
    ADCC24_DOWN_SAMPLING_1_8,        /* Every 8th */

    ADCC24_DOWN_SAMPLING_MAX
} ADCC24_DOWN_SAMPLING;

typedef enum _ADCC24_FIFO_WATERMARK {
    ADCC24_WATERMARK_1 = 0, /* 1st */
    ADCC24_WATERMARK_2,     /* 2nd */
    ADCC24_WATERMARK_3,     /* 3rd */
    ADCC24_WATERMARK_4,     /* 4th */
    ADCC24_WATERMARK_5,     /* 5th */
    ADCC24_WATERMARK_6,     /* 6th */
    ADCC24_WATERMARK_7,     /* 7th */
    ADCC24_WATERMARK_8,     /* 8th */
    ADCC24_WATERMARK_9,     /* 9th */
    ADCC24_WATERMARK_10,    /* 10th */
    ADCC24_WATERMARK_11,    /* 11th */
    ADCC24_WATERMARK_12,    /* 12th */
    ADCC24_WATERMARK_13,    /* 13th */
    ADCC24_WATERMARK_14,    /* 14th */
    ADCC24_WATERMARK_15,    /* 15th */
    ADCC24_WATERMARK_16,    /* 16th */

    ADCC24_WATERMARK_MAX
} ADCC24_FIFO_WATERMARK;

typedef enum _ADCC24_DATA_FORMAT {
    ADCC24_UNSIGNED = 0,
    ADCC24_SIGNED = 1
} ADCC24_DATA_FORMAT;


typedef enum _ADCC24_AMP_CHANNEL {
    ADCC24_AMP_CHANNEL_0 = 0,   /* Cyclick 0ch */
    ADCC24_AMP_CHANNEL_1,       /* Cyclick 1ch */
    ADCC24_AMP_CHANNEL_2,       /* Cyclick 2ch */
    ADCC24_AMP_CHANNEL_SINGLE,  /* Single */

    ADCC24_AMP_CHANNEL_MAX
} ADCC24_AMP_CHANNEL;

typedef enum _ADCC24_AMP_GAIN {
    ADCC24_AMP_GAIN_X1 = 0,     /* x1 */
    ADCC24_AMP_GAIN_X2,         /* x2 */
    ADCC24_AMP_GAIN_X4,         /* x4 */

    ADCC24_AMP_GAIN_MAX
} ADCC24_AMP_GAIN;

typedef enum _ADCC24_ADC_CYCLE {
    ADCC24_AMP_CYCLE_0 = 0,     /* 4130 cycle */
    ADCC24_AMP_CYCLE_1,         /* 1058 cycle */
    ADCC24_AMP_CYCLE_2,         /* 546 cycle */
    ADCC24_AMP_CYCLE_3,         /* 290 cycle */

    ADCC24_AMP_CYCLE_MAX
} ADCC24_ADC_CYCLE;

typedef enum _ADCC24_AMP_CAPACITOR {
    ADCC24_AMP_CAPACITOR_3PF = 0,
    ADCC24_AMP_CAPACITOR_6PF,
    ADCC24_AMP_CAPACITOR_9PF,
    ADCC24_AMP_CAPACITOR_12PF,
    ADCC24_AMP_CAPACITOR_15PF,
    ADCC24_AMP_CAPACITOR_18PF,
    ADCC24_AMP_CAPACITOR_21PF,
    ADCC24_AMP_CAPACITOR_24PF,

    ADCC24_AMP_CAPACITOR_MAX
} ADCC24_AMP_CAPACITOR;

typedef enum _ADCC24_AMP_RESISTOR {
    ADCC24_AMP_RESISTOR_10K = 0,
    ADCC24_AMP_RESISTOR_20K,
    ADCC24_AMP_RESISTOR_40K,
    ADCC24_AMP_RESISTOR_80K,
    ADCC24_AMP_RESISTOR_160K,
    ADCC24_AMP_RESISTOR_320K,
    ADCC24_AMP_RESISTOR_640K,
    ADCC24_AMP_RESISTOR_1M,

    ADCC24_AMP_RESISTOR_MAX
} ADCC24_AMP_RESISTOR;

typedef void (*ADCC24_SignalEvent_t)(ADCC24_EVENT event);
typedef void (*ADCC24_ChannelEvent_t)(ADCC24_CHANNEL ch, ADCC24_CHANNEL_EVENT event);

typedef struct _TZ10XX_DRIVER_ADCC24 {
    ARM_DRIVER_VERSION     (*GetVersion)              (void);
    ADCC24_STATUS       (*Initialize)              (ADCC24_SignalEvent_t cb_event,
                                                    uint32_t             event_mask);
    ADCC24_STATUS       (*Uninitialize)            (void);
    ADCC24_STATUS       (*PowerControl)            (ARM_POWER_STATE state);
    ADCC24_STATUS       (*SetScanMode)             (ADCC24_SCAN_MODE scan_mode, uint32_t ch);
    ADCC24_STATUS       (*SetFIFOOverwrite)        (ADCC24_FIFO_MODE fifo_mode);
    ADCC24_STATUS       (*SetComparison)           (ADCC24_CMP_DATA_SELECT cmp_data, int32_t cmp_data_val,
                                                    ADCC24_CMP_CAUSE_SELECT cmp_cause, ADCC24_CHANNEL ch);
    ADCC24_STATUS       (*SetSamplingPeriod)       (ADCC24_SAMPLING_PERIOD sampling_period);
    ADCC24_STATUS       (*SetDataFormat)           (ADCC24_CHANNEL ch, ADCC24_DATA_FORMAT data_fmt);
    ADCC24_STATUS       (*SetOffset)               (ADCC24_CHANNEL ch, int8_t offset);
    ADCC24_STATUS       (*UseDMA)                  (ADCC24_CHANNEL ch, bool dma_en, uint32_t *data_area, uint32_t num_of_data);
    ADCC24_STATUS       (*SetDecimation)           (ADCC24_CHANNEL ch, ADCC24_DOWN_SAMPLING down_spl);
    ADCC24_STATUS       (*SetWatermark)            (ADCC24_CHANNEL ch, ADCC24_FIFO_WATERMARK watermark);
    ADCC24_STATUS       (*SetChannelEvent)         (ADCC24_CHANNEL ch, ADCC24_ChannelEvent_t cb_event,
                                                    uint32_t chevent_mask);
    ADCC24_STATUS       (*Start)                   (void);
    ADCC24_STATUS       (*Stop)                    (void);
    ADCC24_STATUS       (*ReadData)                (ADCC24_CHANNEL ch, uint32_t* adc);
    ADCC24_STATUS       (*GetNumberOfData)         (ADCC24_CHANNEL ch, uint32_t* num_of_data);
    ADCC24_STATUS       (*GetAreaInformation)      (ADCC24_CHANNEL ch, uint32_t** data_area,
                                                    uint32_t* num_of_data);
    ADCC24_STATUS       (*UseVoltageAMP)           (ADCC24_AMP_CHANNEL ch, ADCC24_AMP_GAIN gain,
                                                    ADCC24_ADC_CYCLE cycle);
    ADCC24_STATUS       (*UseConversionAMP)        (ADCC24_AMP_CHANNEL ch, ADCC24_AMP_CAPACITOR cf,
                                                    ADCC24_AMP_RESISTOR rf, ADCC24_ADC_CYCLE cycle);
    ADCC24_STATUS       (*BypassAMP)               (ADCC24_AMP_CHANNEL ch);

} const TZ10XX_DRIVER_ADCC24;


#ifdef __cplusplus
}
#endif

#endif /* ADCC24_TZ10XX_H */
