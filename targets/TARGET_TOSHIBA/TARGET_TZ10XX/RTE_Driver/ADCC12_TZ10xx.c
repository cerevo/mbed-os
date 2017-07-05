/**
 * @file ADCC12_TZ10xx.c
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

#include "TZ10xx.h"
#include "ADCC12_TZ10xx.h"
#include "ADCC_TZ10xx_common.h"
#include "ADCC12_TZ10xx_internal.h"
#include "SDMAC_TZ10xx.h"
#include "PMU_TZ10xx.h"

#define TZ1000_ADCC12_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 37)   /* driver version */

extern TZ10XX_DRIVER_SDMAC Driver_SDMAC;
static TZ10XX_DRIVER_SDMAC *dma = &Driver_SDMAC;
extern TZ10XX_DRIVER_PMU Driver_PMU;
static TZ10XX_DRIVER_PMU *pmu = &Driver_PMU;

#define ADCC12_SET_IRQ        0x00000033
#define ADCC12_SET_CHIRQ      0x0000007F
#define ADCC12_CHECK_CHIRQ    0x000F0000 /* check ch irq in ADCINTSTS */
#define ADCC12_DATA_SHIFT     0x4
#define ADCC12_CH_ERR         0xFFFF

#define ADCC12_DATA           0x00000FFF
#define ADCC12_SINGLE_MODE    0
#define ADCC12_CYCLIC_MODE    2

/* for Sampling period */
#define ADCC12_SPLT_CTRCTL_0    0x0
#define ADCC12_SPLT_CTRCTL_1_4  0x1
#define ADCC12_SPLT_CTRCTL_1_64 0x3
#define ADCC12_SPLT_PER_1MS     0x0f9f
#define ADCC12_SPLT_PER_2MS     0x1f3f
#define ADCC12_SPLT_PER_4MS     0x3e7f
#define ADCC12_SPLT_PER_8MS     0x7cff
#define ADCC12_SPLT_PER_16MS    0xf9ff
#define ADCC12_SPLT_PER_32MS    0x7cff
#define ADCC12_SPLT_PER_64MS    0xf9ff
#define ADCC12_SPLT_PER_1S      0xffff

/* Initial value */
#define ADCC12_SPLT_PER       0xffff
#define ADCC12_SPLT_CTRCTL    0x0
#define ADCC12_USE_CH         0x1  /* set ch0:0x0, ch1:0x1, ch2:0x2 */
#define ADCC12_PUP_TO_CHSET   0x1
#define ADCC12_CHSET_TO_START 0x1
#define ADCC12_CMP_DATA       0x0

#define ADCC12_ERROR_STS ADCC12_ERROR

#define ADCC12_ALL_CHANNEL    (ADCC12_CHANNEL_0|ADCC12_CHANNEL_1|ADCC12_CHANNEL_2|ADCC12_CHANNEL_3)

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
  TZ1000_ADCC12_API_VERSION,
  TZ1000_ADCC12_DRV_VERSION
};

/* ADC channel setting information */
typedef struct _ADCC12_CHANEL_INFO {
    bool enable;
    ADCC12_ChannelEvent_t ch_callback;
    uint32_t              ch_mask;
    uint32_t              ch_set;
} ADCC12_CHANEL_INFO_t;

/* ADC setting information. */
typedef struct _ADCC12_INFO {
    ARM_POWER_STATE       power;
    bool                  init;
    bool                  ch_en;
    ADCC12_SCAN_MODE      scan_m;
    ADCC12_SignalEvent_t  cb;
    uint32_t              mask;
    ADCC12_CHANEL_INFO_t  ch_info[ADCC12_MAX_CH];
} ADCC12_INFO;

static ADCC12_INFO ADCC_Info = {
    ARM_POWER_OFF, /* power */
    false          /* init */
};

#define DMA_BUFF_0   0
#define DMA_BUFF_1   1
#define DMA_BUFF_NUM 2
typedef struct _ADCC12_DMA_INFO {
    uint16_t *buf;
    bool     enable;
    uint32_t dma_ch;
    int      addr_idx;
    uint16_t *use_buf[DMA_BUFF_NUM];
    uint32_t data_num;
    uint32_t transferred_num;
    uint32_t waterm;
} ADCC12_DMA_INFO;

static ADCC12_DMA_INFO dma_info[ADCC12_MAX_CH];
typedef void (* adcc12_DMA_Handler_t)(uint32_t ch, uint32_t event);

/* ADCC12 Resource type */
typedef struct {
    adcc12_Type *dev;          /* Pointer to ADCC12 peripheral */
    IRQn_Type irq;             /* IRQ number */
    adcc12_ch_Type *ch_conf[ADCC12_MAX_CH]; /* each channel configration */
} const ADCC12_RESOURCES;

static ADCC12_RESOURCES ADCC12_Resources = {
    adcc12,        /* dev */
    ADCC12_IRQn,   /* irq */
    ADCC12_ch_BASE /* ch_conf */
};

adcc12_data_Type *conv_data12[ADCC12_MAX_CH] = ADCC12_data_BASE;

static ADCC12_RESOURCES *rsc = &ADCC12_Resources;
static bool g_SetPeriod_flg = false;
static uint32_t g_timeout = 0;

/* Internal Function */
static void adcc12_ADCInfoInitialize(void);
static uint32_t adcc12_ChangeChNum(ADCC12_CHANNEL ch);
static int adcc12_IsStateIdle(void);
static void adcc12_SetInitConfig(void);
static uint32_t getChFifoSts(uint32_t ch_idx);
void  adcc12_DMA_IRQHandler(ADCC12_CHANNEL ch, uint32_t event);
void  adcc12_DMA_IRQHandler0(uint32_t ch, uint32_t event);
void  adcc12_DMA_IRQHandler1(uint32_t ch, uint32_t event);
void  adcc12_DMA_IRQHandler2(uint32_t ch, uint32_t event);
void  adcc12_DMA_IRQHandler3(uint32_t ch, uint32_t event);
static uint32_t adcc12_getDmaMsize(uint32_t watermark);

/* External Function */
static ADCC12_STATUS ADCC12_PowerControl(ARM_POWER_STATE state);
static ADCC12_STATUS ADCC12_Stop(void);
static ADCC12_STATUS ADCC12_GetNumberOfData(ADCC12_CHANNEL ch, uint32_t* num_of_data);
static ADCC12_STATUS ADCC12_ReadData(ADCC12_CHANNEL ch, uint16_t* adc);

/**
 * @brief       Get driver version.
 * @param       None.
 * @return      \ref ARM_DRV_VERSION
 */
static ARM_DRIVER_VERSION ADCC12_GetVersion(void)
{
  return DriverVersion;
}


/**
 * @brief       Initialize driver.
 * @param[in]   cb_event   Callback function.
 * @param[in]   event_mask Interrupt mask.
 * @return      \ref ADCC12_STATUS
 */
static ADCC12_STATUS ADCC12_Initialize(ADCC12_SignalEvent_t cb_event,
                                       uint32_t             event_mask)
{
    if (ADCC_Info.init) {
        /* already initialized */
        return ADCC12_ERROR;
    }

    /* Check reserved bit in mask */
    if(~ADCC12_SET_IRQ & event_mask) {
        return ADCC12_ERROR_PARAM;
    }
    if((ADCC12_SET_IRQ & event_mask) && (cb_event == NULL)) {
        return ADCC12_ERROR_PARAM;
    }

    /* Initialize ADCInfo */
    adcc12_ADCInfoInitialize();
    g_SetPeriod_flg = false;

    g_timeout = pmu->GetFrequency(PMU_CD_MPIER);

    /* Power on */
    if(ADCC12_PowerControl(ARM_POWER_LOW) == ADCC12_OK) {
        /* Set callback */
        ADCC_Info.cb = cb_event;
        ADCC_Info.mask = event_mask;

        /* Set clock & Reset off controller */
        pmu->EnableModule(PMU_MODULE_ADCC12, 1);

        /* Initialize reg */
        adcc12_SetInitConfig();
        /* Set interrupt enable */
        rsc->dev->ADCINTEN = event_mask;

        ADCC_Info.init = true;
        /* Clear and Enable ADCC IRQ */
        NVIC_ClearPendingIRQ(rsc->irq);
        NVIC_EnableIRQ(rsc->irq);

        return ADCC12_OK;
    }
    return ADCC12_ERROR;
}


/**
 * @brief       Uninitialize driver.
 * @param       None.
 * @return      \ref ADCC12_STATUS
 */
static ADCC12_STATUS ADCC12_Uninitialize(void)
{
    int i;

    if(!(adcc12_IsStateIdle())) {
        return ADCC12_ERROR_STS;
    }

    /* Clear and Disable ADCC IRQ */
    NVIC_DisableIRQ(rsc->irq);
    NVIC_ClearPendingIRQ(rsc->irq);

    /* Reset on controller & Stop clock */
    pmu->EnableModule(PMU_MODULE_ADCC12, 0);
    for(i = 0; i < ADCC12_MAX_CH; i++) {
        dma_info[i].enable = false;
    }

    adcc12_ADCInfoInitialize();
    g_SetPeriod_flg = false;

    /* Power Off */
    if(ADCC12_PowerControl(ARM_POWER_OFF) == ADCC12_OK) {
        ADCC_Info.init = false;
    } else {
        return ADCC12_ERROR;
    }

    return ADCC12_OK;
}


/**
 * @brief       ADCC12 PowerControl.
 * @param[in]   state \ref ARM_POWER_STATE
 * @return      \ref ADCC12_STATUS
 */
static ADCC12_STATUS ADCC12_PowerControl(ARM_POWER_STATE state)
{
    ADCC12_STATUS ret = ADCC12_OK;
    ARM_POWER_STATE prev_power = ADCC_Info.power;

    switch (state) {
        case ARM_POWER_OFF:
            pmu->SetPowerDomainState(PMU_PD_ADCC12, PMU_PD_MODE_OFF);
            ADCC_Info.power = ARM_POWER_OFF;
            break;

        case ARM_POWER_LOW:
            if(pmu->SetPowerDomainState(PMU_PD_ADCC12, PMU_PD_MODE_ON) == PMU_OK) {
                ADCC_Info.power = ARM_POWER_LOW;
            } else {
                ret = ADCC12_ERROR;
            }
            break;

        case ARM_POWER_FULL:
            if(pmu->SetPowerDomainState(PMU_PD_ADCC12, PMU_PD_MODE_ON) == PMU_OK) {
                ADCC_Info.power = ARM_POWER_FULL;
            } else {
                ret = ADCC12_ERROR;
            }
            break;

        default:
            ret = ADCC12_ERROR_PARAM;
            break;
    }

    if( ret == ADCC12_OK && prev_power == ARM_POWER_OFF && state != ARM_POWER_OFF ){
        /* Initialize reg */
        adcc12_SetInitConfig();
        /* Set interrupt enable */
        rsc->dev->ADCINTEN = ADCC_Info.mask;
    }

    return ret;
}

static void adcc12_set_ch_flag(ADCC12_INFO *info, uint32_t ch, uint32_t en_dis) 
{
    if(ch & ADCC12_CHANNEL_0) {
        info->ch_info[0].ch_set = en_dis;
        rsc->dev->CH0_MODE_b.ChEn = en_dis;
    }
    if(ch & ADCC12_CHANNEL_1) {
        info->ch_info[1].ch_set = en_dis;
        rsc->dev->CH1_MODE_b.ChEn = en_dis;
    }
    if(ch & ADCC12_CHANNEL_2) {
        info->ch_info[2].ch_set = en_dis;
        rsc->dev->CH2_MODE_b.ChEn = en_dis;
    }
    if(ch & ADCC12_CHANNEL_3) {
        info->ch_info[3].ch_set = en_dis;
        rsc->dev->CH3_MODE_b.ChEn = en_dis;
    }
}

/**
 * @brief       Set Scan Mode.
 * @param[in]   scan_mode \ref ADCC12_SCAN_MODE
 * @param[in]   ch        Using channel
 * @return      \ref ADCC12_STATUS
 */
static ADCC12_STATUS ADCC12_SetScanMode(ADCC12_SCAN_MODE scan_mode, uint32_t ch)
{
    uint32_t set_mode, i;
    bool ch_en = false;

    /* Check convert state */
    if(!(adcc12_IsStateIdle())) {
        return ADCC12_ERROR_STS;
    }

    /* Check channel */
    if(!(ch & ADCC12_ALL_CHANNEL) || (ch & ~ADCC12_ALL_CHANNEL)) {
        return ADCC12_ERROR_PARAM;
    }

    if(scan_mode == ADCC12_SCAN_MODE_SINGLE) {
        uint32_t ch_idx = adcc12_ChangeChNum((ADCC12_CHANNEL)ch);
        if(ch_idx == ADCC12_CH_ERR) {
            return ADCC12_ERROR_PARAM;
        }
        set_mode = ADCC12_SINGLE_MODE;
        adcc12_set_ch_flag(&ADCC_Info, ~ch, ADCC_DISABLE);
        ADCC_Info.ch_info[ch_idx].ch_set = ADCC_ENABLE;
        rsc->dev->MODESEL_b.ChSel = ch_idx;
    } else if (scan_mode == ADCC12_SCAN_MODE_CYCLIC) {
        set_mode = ADCC12_CYCLIC_MODE;
        if(ADCC_Info.scan_m == ADCC12_SCAN_MODE_SINGLE) {
            adcc12_set_ch_flag(&ADCC_Info, ADCC12_ALL_CHANNEL, ADCC_DISABLE);
        }
        adcc12_set_ch_flag(&ADCC_Info, ch, ADCC_ENABLE);
    } else if(scan_mode == ADCC12_SCAN_MODE_DISABLE) {
        adcc12_set_ch_flag(&ADCC_Info, ch, ADCC_DISABLE);
        set_mode = (ADCC_Info.scan_m == ADCC12_SCAN_MODE_SINGLE) ? ADCC12_SINGLE_MODE : ADCC12_CYCLIC_MODE;
    } else {
        return ADCC12_ERROR_PARAM;
    }

    for(i = 0;i < ADCC12_MAX_CH;i++) {
        if(ADCC_Info.ch_info[i].ch_set == ADCC_ENABLE) {
            ch_en = true;
            break;
        }
    }

    if(ch_en == true) {
        /* Set Conversion mode */
        rsc->dev->MODESEL_b.ConvMd = set_mode;
        ADCC_Info.scan_m = scan_mode;
        ADCC_Info.ch_en = true;
    } else {
        ADCC_Info.ch_en = false;
    }

    return ADCC12_OK;
}


/**
 * @brief       Set FIFO overwrite.
 * @param[in]   fifo_mode \ref ADCC12_FIFO_MODE
 * @return      \ref ADCC12_STATUS
 */
static ADCC12_STATUS ADCC12_SetFIFOOverwrite(ADCC12_FIFO_MODE fifo_mode)
{
    /* Check param */
    if((ADCC12_FIFO_MODE_STREAM != fifo_mode) &&
       (ADCC12_FIFO_MODE_STOP != fifo_mode)) {
            return ADCC12_ERROR_PARAM;
    }

    /* Check convert state */
    if(adcc12_IsStateIdle()) {
        /* Set FIFO mode */
        rsc->dev->MODESEL_b.FIFOMd = fifo_mode;
    } else {
        return ADCC12_ERROR_STS;
    }

    return ADCC12_OK;
}


/**
 * @brief       Set compare data.
 * @param[in]   cmp_data     \ref ADCC12_CMP_DATA_SELECT
 * @param[in]   cmp_data_val Setting value
 * @param[in]   cmp_cause    \ref ADCC12_CMP_CAUSE_SELECT
 * @param[in]   ch           \ref ADCC12_CHANNEL
 * @return      ADCC12_STATUS
 */
static ADCC12_STATUS ADCC12_SetComparison(ADCC12_CMP_DATA_SELECT cmp_data, int32_t cmp_data_val,
                                          ADCC12_CMP_CAUSE_SELECT cmp_cause, ADCC12_CHANNEL ch)
{
    uint32_t ch_idx = adcc12_ChangeChNum(ch);

    if(ch_idx == ADCC12_CH_ERR) {
        return ADCC12_ERROR_PARAM;
    }

    /* param check */
    if(cmp_cause >= ADCC12_CMP_DATA_MAX) {
        return ADCC12_ERROR_PARAM;
    }
    if(cmp_data_val & ~ADCC12_DATA) {
        return ADCC12_ERROR_PARAM;
    }

    /* Check convert state */
    if(adcc12_IsStateIdle()) {
        if(cmp_data == ADCC12_CMP_DATA_0) {
            if(cmp_cause == ADCC12_CMP_NO_COMPARISON) {
                rsc->dev->CMPDATA_0_b.CmpEn = ADCC_DISABLE;
                return ADCC12_OK;
            }

            rsc->dev->CMPDATA_0_b.CmpData = cmp_data_val & ADCC12_DATA;
            rsc->dev->CMPDATA_0_b.CmpCh   = ch_idx;
            rsc->dev->CMPDATA_0_b.CmpEn   = ADCC_ENABLE;
            rsc->dev->CMPDATA_0_b.CmpSel  = cmp_cause;
        } 
        else if(cmp_data == ADCC12_CMP_DATA_1) {
            if(cmp_cause == ADCC12_CMP_NO_COMPARISON) {
                rsc->dev->CMPDATA_1_b.CmpEn = ADCC_DISABLE;
                return ADCC12_OK;
            }

            rsc->dev->CMPDATA_1_b.CmpData = cmp_data_val & ADCC12_DATA;
            rsc->dev->CMPDATA_1_b.CmpCh   = ch_idx;
            rsc->dev->CMPDATA_1_b.CmpEn   = ADCC_ENABLE;
            rsc->dev->CMPDATA_1_b.CmpSel  = cmp_cause;
        } else {
            return ADCC12_ERROR_PARAM;
        }
    } else {
        return ADCC12_ERROR_STS;
    }

    return ADCC12_OK;
}


/**
 * @brief       Set Sampling Period.
 * @param[in]   ch              \ref ADCC12_CHANNEL
 * @param[in]   sampling_period \ref ADCC12_SAMPLING_PERIOD
 * @return      \ref ADCC12_STATUS
 */
static ADCC12_STATUS ADCC12_SetSamplingPeriod(ADCC12_SAMPLING_PERIOD sampling_period)
{
    ADCC12_STATUS ret = ADCC12_OK;

    /* Check convert state */
    if(adcc12_IsStateIdle()) {
        switch(sampling_period) {
            case ADCC12_SAMPLING_PERIOD_1MS:
                rsc->dev->SPLT_CTRCTL_b.SrcSel = ADCC12_SPLT_CTRCTL_0;
                rsc->dev->SPLT_PER             = ADCC12_SPLT_PER_1MS;
                break;

            case ADCC12_SAMPLING_PERIOD_2MS:
                rsc->dev->SPLT_CTRCTL_b.SrcSel = ADCC12_SPLT_CTRCTL_0;
                rsc->dev->SPLT_PER             = ADCC12_SPLT_PER_2MS;
                break;

            case ADCC12_SAMPLING_PERIOD_4MS:
                rsc->dev->SPLT_CTRCTL_b.SrcSel = ADCC12_SPLT_CTRCTL_0;
                rsc->dev->SPLT_PER             = ADCC12_SPLT_PER_4MS;
                break;

            case ADCC12_SAMPLING_PERIOD_8MS:
                rsc->dev->SPLT_CTRCTL_b.SrcSel = ADCC12_SPLT_CTRCTL_0;
                rsc->dev->SPLT_PER             = ADCC12_SPLT_PER_8MS;
                break;

            case ADCC12_SAMPLING_PERIOD_16MS:
                rsc->dev->SPLT_CTRCTL_b.SrcSel = ADCC12_SPLT_CTRCTL_0;
                rsc->dev->SPLT_PER             = ADCC12_SPLT_PER_16MS;
                break;

            case ADCC12_SAMPLING_PERIOD_32MS:
                rsc->dev->SPLT_CTRCTL_b.SrcSel = ADCC12_SPLT_CTRCTL_1_4;
                rsc->dev->SPLT_PER             = ADCC12_SPLT_PER_32MS;
                break;

            case ADCC12_SAMPLING_PERIOD_64MS:
                rsc->dev->SPLT_CTRCTL_b.SrcSel = ADCC12_SPLT_CTRCTL_1_4;
                rsc->dev->SPLT_PER             = ADCC12_SPLT_PER_64MS;
                break;

            case ADCC12_SAMPLING_PERIOD_1S:
                rsc->dev->SPLT_CTRCTL_b.SrcSel = ADCC12_SPLT_CTRCTL_1_64;
                rsc->dev->SPLT_PER             = ADCC12_SPLT_PER_1S;
                break;

            default:
                ret = ADCC12_ERROR_PARAM;
                break;
        }
    } else {
        ret = ADCC12_ERROR_STS;
    }

    if(ret == ADCC12_OK) {
        g_SetPeriod_flg = true;
    }
    return ret;
}


/**
 * @brief       Set Data Format.
 * @param[in]   ch       \ref ADCC12_CHANNEL    
 * @param[in]   data_fmt \ref ADCC12_DATA_FORMAT
 * @return      \ref ADCC12_STATUS
 */
static ADCC12_STATUS ADCC12_SetDataFormat(ADCC12_CHANNEL ch, ADCC12_DATA_FORMAT data_fmt)
{
    uint32_t ch_idx = adcc12_ChangeChNum(ch);

    /* Check param */
    if(ch_idx == ADCC12_CH_ERR) {
        return ADCC12_ERROR_PARAM;
    }
    if((ADCC12_UNSIGNED != data_fmt) && (ADCC12_SIGNED != data_fmt)) {
            return ADCC12_ERROR_PARAM;
    }

    /* Check convert state */
    if(adcc12_IsStateIdle()) {
        rsc->ch_conf[ch_idx]->CH_MODE_b.DtFmt = data_fmt;
    } else {
        return ADCC12_ERROR_STS;
    }
    return ADCC12_OK;
}


/**
 * @brief       Set Offset.
 * @param[in]   ch     \ref ADCC12_CHANNEL
 * @param[in]   offset setting offset
 * @return      \ref ADCC12_STATUS
 */
static ADCC12_STATUS ADCC12_SetOffset(ADCC12_CHANNEL ch, int8_t offset)
{
    uint32_t ch_idx = adcc12_ChangeChNum(ch);

    if(ch_idx == ADCC12_CH_ERR) {
        return ADCC12_ERROR_PARAM;
    }

    /* Check convert state */
    if(adcc12_IsStateIdle()) {
        /* Set offset enable. */
        rsc->ch_conf[ch_idx]->CH_OFFSET_b.Offset = offset;
        rsc->ch_conf[ch_idx]->CH_MODE_b.OfstEn = ADCC_ENABLE;
    } else {
        return ADCC12_ERROR_STS;
    }

    return ADCC12_OK;
}


/**
 * @brief       Set using DMA.
 * @param[in]   ch     \ref ADCC12_CHANNEL
 * @param[in]   dma_en set DMA use or not use
 * @return      \ref ADCC12_STATUS
 */
static ADCC12_STATUS ADCC12_UseDMA(ADCC12_CHANNEL ch, bool dma_en, uint16_t *data_area, uint32_t num_of_data)
{
    uint32_t ch_idx = adcc12_ChangeChNum(ch);

    if(ch_idx == ADCC12_CH_ERR) {
        return ADCC12_ERROR_PARAM;
    }
    if((true != dma_en) && (false != dma_en)) {
        return ADCC12_ERROR_PARAM;
    }
    if((data_area == NULL) || (num_of_data == 0)) {
        return ADCC12_ERROR_PARAM;
    }

    /* Check convert state */
    if(adcc12_IsStateIdle()) {
        rsc->ch_conf[ch_idx]->CH_MODE_b.DMAEn = dma_en;
    } else {
        return ADCC12_ERROR_STS;
    }

    /* Set DMA Info */
    dma_info[ch_idx].buf = data_area;
    dma_info[ch_idx].enable = dma_en;
    dma_info[ch_idx].use_buf[DMA_BUFF_0] = data_area;
    dma_info[ch_idx].use_buf[DMA_BUFF_1] = &data_area[num_of_data/2];
    dma_info[ch_idx].addr_idx = DMA_BUFF_0;
    dma_info[ch_idx].data_num = (num_of_data/2);

    return ADCC12_OK;
}


/**
 * @brief       Set Down Sampling.
 * @param[in]   ch         \ref ADCC12_CHANNEL
 * @param[in]   down_spl   \ref ADCC12_DOWN_SAMPLING
 * @return      \ref ADCC12_STATUS
 */
static ADCC12_STATUS ADCC12_SetDecimation(ADCC12_CHANNEL ch, ADCC12_DOWN_SAMPLING down_spl)
{
    uint32_t ch_idx = adcc12_ChangeChNum(ch);

    if(ch_idx == ADCC12_CH_ERR) {
        return ADCC12_ERROR_PARAM;
    }
    if(ADCC12_DOWN_SAMPLING_MAX <= down_spl) {
        return ADCC12_ERROR_PARAM;
    }

    /* Check convert state */
    if(adcc12_IsStateIdle()) {
        rsc->ch_conf[ch_idx]->CH_MODE_b.DownSpl = down_spl;
    } else {
        return ADCC12_ERROR_STS;
    }

    return ADCC12_OK;
}


/**
 * @brief       Set fifo Watermark.
 * @param[in]   ch        \ref ADCC12_CHANNEL
 * @param[in]   watermark \ref ADCC12_FIFO_WATERMARK
 * @return      \ref ADCC12_STATUS
 */
static ADCC12_STATUS ADCC12_SetWatermark(ADCC12_CHANNEL ch, ADCC12_FIFO_WATERMARK watermark)
{
    uint32_t ch_idx = adcc12_ChangeChNum(ch);

    if(ch_idx == ADCC12_CH_ERR) {
        return ADCC12_ERROR_PARAM;
    }
    if(ADCC12_WATERMARK_MAX <= watermark) {
        return ADCC12_ERROR_PARAM;
    }

    /* Check convert state */
    if(adcc12_IsStateIdle()) {
        rsc->ch_conf[ch_idx]->CH_MODE_b.WaterMark = watermark;
        dma_info[ch_idx].waterm = watermark;
    } else {
        return ADCC12_ERROR_STS;
    }

    return ADCC12_OK;
}


/**
 * @brief       Set Channel Interrupt Event.
 * @param[in]   ch       \ref ADCC12_CHANNEL
 * @param[in]   cb_event Callback function
 * @return      \ref ADCC12_STATUS
 */
static ADCC12_STATUS ADCC12_SetChannelEvent(ADCC12_CHANNEL ch, ADCC12_ChannelEvent_t cb_event,
                                            uint32_t chevent_mask)
{
    uint32_t ch_idx = adcc12_ChangeChNum(ch);

    if(ch_idx == ADCC12_CH_ERR) {
        return ADCC12_ERROR_PARAM;
    }

    /* Check reserved bit in mask */
    if(~ADCC12_SET_CHIRQ & chevent_mask) {
        return ADCC12_ERROR_PARAM;
    }

    if((ADCC12_SET_CHIRQ & chevent_mask) && (cb_event == NULL)) {
        return ADCC12_ERROR_PARAM;
    }

    /* Check convert state */
    if(adcc12_IsStateIdle()) {
        /* Set callback */
        ADCC_Info.ch_info[ch_idx].ch_callback = cb_event;
        /* Set interrupt enable */
        rsc->ch_conf[ch_idx]->CH_INTEN = chevent_mask;
        ADCC_Info.ch_info[ch_idx].ch_mask = chevent_mask;
    } else {
        return ADCC12_ERROR_STS;
    }

    return ADCC12_OK;
}


/**
 * @brief       Start sampling.
 * @param       None       
 * @return      \ref ADCC12_STATUS
 */
static ADCC12_STATUS ADCC12_Start(void)
{
    uint32_t i;
    SDMAC_STATUS err;
    SDMAC_MSIZE dma_msize;
    adcc12_DMA_Handler_t DMA_Handler[] = 
    {
        adcc12_DMA_IRQHandler0,
        adcc12_DMA_IRQHandler1,
        adcc12_DMA_IRQHandler2,
        adcc12_DMA_IRQHandler3
    };
    
    /* Check initialized */
    if(ADCC_Info.init != true) {
        return ADCC12_ERROR;
    }

    if(ADCC_Info.ch_en != true) {
        return ADCC12_ERROR;
    }

    if((ADCC12_SCAN_MODE_CYCLIC == ADCC_Info.scan_m) &&
       (false == g_SetPeriod_flg)) {
        return ADCC12_ERROR;
    }

    /* Check convert state */
    if(adcc12_IsStateIdle()) {
        /* Start DMA */
        for(i = 0; i < ADCC12_MAX_CH; i++) {
            if(dma_info[i].enable == true) {
                dma_info[i].dma_ch = dma->ChannelAcquire(DMA_Handler[i], (SDMAC_SRC)(SDMAC_SRC_ADC12_CH0+i),
                                                          SDMAC_DST_MEMORY, SDMAC_FIFO_MODE_UTILIZATION,
                                                          SDMAC_PRIORITY_LOWEST);

                dma_msize = (SDMAC_MSIZE)adcc12_getDmaMsize(dma_info[i].waterm);
                /* width * msize <= 16 */
                err = dma->ChannelTransferData(dma_info[i].dma_ch, (const uint8_t*)&conv_data12[i]->CH_DATA[0],
                                               (uint8_t *)dma_info[i].use_buf[dma_info[i].addr_idx], dma_info[i].data_num*sizeof(uint16_t),
                                               SDMAC_CTL(SDMAC_WIDTH_2, SDMAC_INC_NO_CHANGE, dma_msize,
                                               SDMAC_WIDTH_2, SDMAC_INC_INCREMENT, dma_msize));
                if (err) {
                    return ADCC12_ERROR;
                }
            }
        }
        
        /* ADC Start */
        rsc->dev->ADCCTL_b.Start = ADCC_ENABLE;
        for (i = 0; i < g_timeout; i++) {
            if(rsc->dev->ADCCTL_b.Start != 0) {
                break;
            }
        }
        if(i == g_timeout) {
            return ADCC12_ERROR;
        }

    } else {
        return ADCC12_ERROR_STS;
    }

    return ADCC12_OK;
}


/**
 * @brief       Stop sampling.
 * @param       None
 * @return      \ref ADCC12_STATUS
 */
static ADCC12_STATUS ADCC12_Stop(void)
{
    int num, j;
    uint16_t buf;
    uint32_t i;

    /* Stop DMA */
    for(i = 0; i < ADCC12_MAX_CH; i++) {
        if(dma_info[i].enable == true) {
            /* Get transfered data. */
            dma->ChannelAbortTransfer(dma_info[i].dma_ch);
            dma->ChannelCountBytesTransferred(dma_info[i].dma_ch, &dma_info[i].transferred_num);
            dma->ChannelUninitialize(dma_info[i].dma_ch);
            dma->ChannelRelease(dma_info[i].dma_ch);
        }
    }

    rsc->dev->ADCCTL_b.Stop = ADCC_ENABLE;
    /* Check status. */
    for (i = 0; i < g_timeout; i++)
    {
        if((rsc->dev->ADCCTL_b.Stop == ADCC_DISABLE) && 
           (rsc->dev->ADCCTL_b.Start == ADCC_DISABLE)) {
            break;
        }
    }

    if(i == g_timeout) {
        return ADCC12_ERROR;
    }

    for(i = 0; i < ADCC12_MAX_CH; i++) {
        /* trush FIFO data */
        num = getChFifoSts(i);
        for(j = 0; j < num; j++) {
            ADCC12_ReadData((ADCC12_CHANNEL)(1 << i), &buf);
        }
    }

    return ADCC12_OK;
}


/**
 * @brief       Read sampling data.
 * @param[in]   ch   \ref ADCC12_CHANNEL
 * @param[out]  *adc Set converted data buffer
 * @return      \ref ADCC12_STATUS
 */
static ADCC12_STATUS ADCC12_ReadData(ADCC12_CHANNEL ch, uint16_t* adc)
{
    uint32_t num;
    uint32_t ch_idx = adcc12_ChangeChNum(ch);

    if( (ch_idx == ADCC12_CH_ERR) || (adc == NULL) ) {
        return ADCC12_ERROR_PARAM;
    }

    /* Check DMA status */
    if(!(adcc12_IsStateIdle()) && (rsc->ch_conf[ch_idx]->CH_MODE_b.DMAEn == ADCC_ENABLE)) {
        return ADCC12_ERROR_STS;
    }

    num = getChFifoSts(ch_idx);

    /* FIFO empty */
    if(num == 0) {
        *adc = 0;
        return ADCC12_ERROR_FIFO_EMPTY;
    }
   
    *adc = conv_data12[ch_idx]->CH_DATA[0];

    return ADCC12_OK;
}


/**
 * @brief       Get number of FIFO data.
 * @param[in]   ch           \ref ADCC12_CHANNEL
 * @param[out]  *num_of_data Number of data in FIFO
 * @return      \ref ADCC12_STATUS
 */
static ADCC12_STATUS ADCC12_GetNumberOfData(ADCC12_CHANNEL ch, uint32_t* num_of_data)
{
    uint32_t ch_idx = adcc12_ChangeChNum(ch);

    if( (ch_idx == ADCC12_CH_ERR) || (num_of_data == NULL) ) {
        return ADCC12_ERROR_PARAM;
    }

    *num_of_data = getChFifoSts(ch_idx);

    return ADCC12_OK;
}


/**
 * @brief       Get converted data information.
 * @param[in]   ch           \ref ADCC12_CHANNEL
 * @param[out]  **data_area  set converted data buffer
 * @param[out]  *num_of_data Number of data
 * @return      \ref ADCC12_STATUS
 */
static ADCC12_STATUS ADCC12_GetAreaInformation(ADCC12_CHANNEL ch,
                                                   uint16_t** data_area, uint32_t* num_of_data)
{
    uint32_t ch_idx = adcc12_ChangeChNum(ch);

    if( (ch_idx == ADCC12_CH_ERR) || (data_area == NULL) || (num_of_data == NULL) ) {
        return ADCC12_ERROR_PARAM;
    }

    if(adcc12_IsStateIdle()) {
        /* ADC is stopped, return current DMA buffer. */
        *data_area = dma_info[ch_idx].use_buf[dma_info[ch_idx].addr_idx];
        if(dma_info[ch_idx].data_num == dma_info[ch_idx].transferred_num) {
            /* If DMA transmission is completed, set data size 0. */
            *num_of_data = 0;
        } else {
            *num_of_data = dma_info[ch_idx].transferred_num;
        }
    } else {
        *data_area = dma_info[ch_idx].use_buf[!dma_info[ch_idx].addr_idx];
        *num_of_data = dma_info[ch_idx].data_num;
    }
    return ADCC12_OK;
}

/**
 * @brief       ADCC12 irq handler.
 * @param       None
 * @return      \ref ADCC12_STATUS
 */
void ADCC12_IRQHandler(void) {
    uint32_t irq = rsc->dev->ADCINTSTS;
    uint32_t ch_irq[ADCC12_MAX_CH];
    uint32_t masked_irq;
    int i;


    for(i = 0; i < ADCC12_MAX_CH; i++) {
        ch_irq[i] = rsc->ch_conf[i]->CH_INTSTS;
    }
    
    if((ADCC_Info.cb != NULL) && (irq & ADCC_Info.mask)){
        /* Common interrupt */
        masked_irq = irq & ADCC_Info.mask;
        if(masked_irq & ADCC12_EVENT_CONV_END) {
            /* Conversion end */
            ADCC_Info.cb(ADCC12_EVENT_CONV_END);
        }
        if(masked_irq & ADCC12_EVENT_SCAN_END) {
            /* Scan end */
            ADCC_Info.cb(ADCC12_EVENT_SCAN_END);
        }
        if(masked_irq & ADCC12_EVENT_CMPINT_0) {
            /* Compare0 */
            ADCC_Info.cb(ADCC12_EVENT_CMPINT_0);
        }
        if(masked_irq & ADCC12_EVENT_CMPINT_1) {
            /* Compare1 */
            ADCC_Info.cb(ADCC12_EVENT_CMPINT_1);
        }
    }


    /* Each channel interrupt */
    if (irq & ADCC12_CHECK_CHIRQ) {
        for(i = 0; i < ADCC12_MAX_CH; i++) {
            masked_irq = ch_irq[i] & ADCC_Info.ch_info[i].ch_mask;
            if((masked_irq == 0) || (ADCC_Info.ch_info[i].ch_callback == NULL)) {
                continue;
            }

            /* Conversion end */
            if(masked_irq & ADCC12_CHANNEL_EVENT_CONV_END) {
                ADCC_Info.ch_info[i].ch_callback((ADCC12_CHANNEL)(1<<i),
                                                 ADCC12_CHANNEL_EVENT_CONV_END);
            }
            /* FIFO watermark */
            if(masked_irq & ADCC12_CHANNEL_EVENT_WATERMARK) {
                ADCC_Info.ch_info[i].ch_callback((ADCC12_CHANNEL)(1<<i),
                                                 ADCC12_CHANNEL_EVENT_WATERMARK);
            }
            /* First data lost */
            if(masked_irq & ADCC12_CHANNEL_EVENT_FDATA_LOST) {
                ADCC_Info.ch_info[i].ch_callback((ADCC12_CHANNEL)(1<<i),
                                                 ADCC12_CHANNEL_EVENT_FDATA_LOST);
            }
            /* Last data lost */
            if(masked_irq & ADCC12_CHANNEL_EVENT_LDATA_LOST) {
                ADCC_Info.ch_info[i].ch_callback((ADCC12_CHANNEL)(1<<i),
                                                 ADCC12_CHANNEL_EVENT_LDATA_LOST);
            }
            /* FIFO full */
            if(masked_irq & ADCC12_CHANNEL_EVENT_FIFO_FULL) {
                ADCC_Info.ch_info[i].ch_callback((ADCC12_CHANNEL)(1<<i),
                                                 ADCC12_CHANNEL_EVENT_FIFO_FULL);
            }
            /* FIFO underrun */
            if(masked_irq & ADCC12_CHANNEL_EVENT_FIFO_UNDERRUN) {
                ADCC_Info.ch_info[i].ch_callback((ADCC12_CHANNEL)(1<<i),
                                                 ADCC12_CHANNEL_EVENT_FIFO_UNDERRUN);
            }
        }
    }

    /* Clear IRQ */
    rsc->dev->ADCINTSTS = ADCC12_SET_IRQ;
    for(i = 0; i < ADCC12_MAX_CH; i++) {
        /* Write 1 for Clear ch IRQ */
        rsc->ch_conf[i]->CH_INTSTS = ch_irq[i];
    }

}

void  adcc12_DMA_IRQHandler0(uint32_t ch, uint32_t event)
{
    adcc12_DMA_IRQHandler(ADCC12_CHANNEL_0, event);
}

void  adcc12_DMA_IRQHandler1(uint32_t ch, uint32_t event)
{
    adcc12_DMA_IRQHandler(ADCC12_CHANNEL_1, event);
}

void  adcc12_DMA_IRQHandler2(uint32_t ch, uint32_t event)
{
    adcc12_DMA_IRQHandler(ADCC12_CHANNEL_2, event);
}

void  adcc12_DMA_IRQHandler3(uint32_t ch, uint32_t event)
{
    adcc12_DMA_IRQHandler(ADCC12_CHANNEL_3, event);
}

void  adcc12_DMA_IRQHandler(ADCC12_CHANNEL ch, uint32_t event)
{
    uint32_t ch_irq;
    SDMAC_MSIZE dma_msize;
    uint32_t ch_idx = adcc12_ChangeChNum(ch);

    if(event & SDMAC_EVENT_TRANSFER) {
        /* set addr */
        dma_info[ch_idx].addr_idx = !dma_info[ch_idx].addr_idx;

        
        dma_msize = (SDMAC_MSIZE)adcc12_getDmaMsize(dma_info[ch_idx].waterm);
        /* width * msize <= 16 */
        dma->ChannelTransferData(dma_info[ch_idx].dma_ch, (const uint8_t*)&conv_data12[ch_idx]->CH_DATA[0],
                                       (uint8_t *)dma_info[ch_idx].use_buf[dma_info[ch_idx].addr_idx], dma_info[ch_idx].data_num*sizeof(uint16_t),
                                       SDMAC_CTL(SDMAC_WIDTH_2, SDMAC_INC_NO_CHANGE, dma_msize,
                                       SDMAC_WIDTH_2, SDMAC_INC_INCREMENT, dma_msize));
        ch_irq = rsc->ch_conf[ch_idx]->CH_INTSTS;
        rsc->ch_conf[ch_idx]->CH_INTSTS = ch_irq;
        
        /* callback */
        if(ADCC_Info.ch_info[ch_idx].ch_mask & ADCC12_CHANNEL_EVENT_DMA_COMPLETE) {
            ADCC_Info.ch_info[ch_idx].ch_callback(ch, ADCC12_CHANNEL_EVENT_DMA_COMPLETE);
        }
    }

    if(event & SDMAC_EVENT_ERROR) {
        ;
    }

}

static void adcc12_ADCInfoInitialize(void)
{
    int i;

    ADCC_Info.power = ARM_POWER_OFF;
    ADCC_Info.ch_en = false;
    ADCC_Info.scan_m = ADCC12_SCAN_MODE_SINGLE;
    ADCC_Info.mask = 0x00000000;
    ADCC_Info.cb = NULL;

    for (i = 0; i < ADCC12_MAX_CH; i++) {
        ADCC_Info.ch_info[i].enable = 0;
        ADCC_Info.ch_info[i].ch_callback = NULL;
        ADCC_Info.ch_info[i].ch_mask = 0x00000000;
        ADCC_Info.ch_info[i].ch_set = ADCC_DISABLE;
        
        dma_info[i].enable = false;
        dma_info[i].use_buf[DMA_BUFF_0] = NULL;
        dma_info[i].use_buf[DMA_BUFF_1] = NULL;
        dma_info[i].dma_ch = 0;
        dma_info[i].addr_idx = DMA_BUFF_0;
        dma_info[i].data_num = 0;
    }
}

static uint32_t adcc12_ChangeChNum(ADCC12_CHANNEL ch) {
    uint32_t ret;

    switch(ch) {
        case ADCC12_CHANNEL_0:
            ret = 0x0;
            break;
        case ADCC12_CHANNEL_1:
            ret = 0x1;
            break;
        case ADCC12_CHANNEL_2:
            ret = 0x2;
            break;
        case ADCC12_CHANNEL_3:
            ret = 0x3;
            break;
        default:
            ret = ADCC12_CH_ERR;
            break;
    }
    return ret;
}

static int adcc12_IsStateIdle(void)
{
    return !(rsc->dev->ADCCTL_b.Start);
}

static void adcc12_SetInitConfig(void)
{
    int i;
    /* Set ADC interrupt enable */
    rsc->dev->ADCINTEN = 0x0;

    /* Set channel interrupt enable */
    for(i = 0; i < ADCC12_MAX_CH; i++) {
        rsc->ch_conf[i]->CH_INTEN = 0;
    }

    /* Set sampling period */
    rsc->dev->SPLT_PER = ADCC12_SPLT_PER;
    rsc->dev->SPLT_CTRCTL = ADCC12_SPLT_CTRCTL;
    /* Set MODESEL */
    rsc->dev->MODESEL_b.ConvMd = ADCC12_SCAN_MODE_SINGLE;
    rsc->dev->MODESEL_b.FIFOMd = ADCC12_FIFO_MODE_STREAM;
    rsc->dev->MODESEL_b.ChSel = ADCC12_USE_CH;
    rsc->dev->MODESEL_b.PUpToChSet = ADCC12_PUP_TO_CHSET;
    rsc->dev->MODESEL_b.ChSetToStart = ADCC12_CHSET_TO_START;

    /* Set CMPDATA_0 */
    rsc->dev->CMPDATA_0_b.CmpSel = ADCC12_CMP_DATA_LT_CONV;
    rsc->dev->CMPDATA_0_b.CmpEn = ADCC_ENABLE;
    rsc->dev->CMPDATA_0_b.CmpCh = ADCC12_USE_CH;
    rsc->dev->CMPDATA_0_b.CmpData = ADCC12_CMP_DATA;
    /* Set CMPDATA_1 */
    rsc->dev->CMPDATA_1_b.CmpSel = ADCC12_CMP_DATA_LT_CONV;
    rsc->dev->CMPDATA_1_b.CmpEn = ADCC_ENABLE;
    rsc->dev->CMPDATA_1_b.CmpCh = ADCC12_USE_CH;
    rsc->dev->CMPDATA_1_b.CmpData = ADCC12_CMP_DATA;

    /* Set CHx_MODE */
    for(i = 0; i < ADCC12_MAX_CH; i++) {
        rsc->ch_conf[i]->CH_MODE_b.DtFmt = ADCC12_UNSIGNED;
        rsc->ch_conf[i]->CH_MODE_b.OfstEn = ADCC_DISABLE;
        rsc->ch_conf[i]->CH_MODE_b.DMAEn = ADCC_DISABLE;
        rsc->ch_conf[i]->CH_MODE_b.DownSpl = ADCC12_DOWN_SAMPLING_1_1;
        rsc->ch_conf[i]->CH_MODE_b.WaterMark = ADCC12_WATERMARK_1;
    }
}

static uint32_t adcc12_getDmaMsize(uint32_t watermark) {
    uint32_t msize;
    
    switch(watermark) {
        case ADCC12_WATERMARK_1:
        case ADCC12_WATERMARK_2:
        case ADCC12_WATERMARK_3:
            msize = SDMAC_MSIZE_1;
            break;

        case ADCC12_WATERMARK_4:
        case ADCC12_WATERMARK_5:
        case ADCC12_WATERMARK_6:
        case ADCC12_WATERMARK_7:
            msize = SDMAC_MSIZE_4;
            break;

        case ADCC12_WATERMARK_8:
            msize = SDMAC_MSIZE_8;
            break;

        default:
            msize = SDMAC_MSIZE_1;
            break;
    }
    
    return msize;
}

static uint32_t getChFifoSts(uint32_t ch_idx) {
    return rsc->ch_conf[ch_idx]->CH_FIFOSTS;
}


/* ADCC12 Driver Control Block */
TZ10XX_DRIVER_ADCC12 Driver_ADCC12 = {
    ADCC12_GetVersion,
    ADCC12_Initialize,
    ADCC12_Uninitialize,
    ADCC12_PowerControl,
    ADCC12_SetScanMode,
    ADCC12_SetFIFOOverwrite,
    ADCC12_SetComparison,
    ADCC12_SetSamplingPeriod,
    ADCC12_SetDataFormat,
    ADCC12_SetOffset,
    ADCC12_UseDMA,
    ADCC12_SetDecimation,
    ADCC12_SetWatermark,
    ADCC12_SetChannelEvent,
    ADCC12_Start,
    ADCC12_Stop,
    ADCC12_ReadData,
    ADCC12_GetNumberOfData,
    ADCC12_GetAreaInformation
};
