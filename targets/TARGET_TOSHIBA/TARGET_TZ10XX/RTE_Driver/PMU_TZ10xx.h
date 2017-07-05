/**
 * @file PMU_TZ10xx.h
 * @brief a header file for TZ10xx PMU driver
 * @date $Date:: 2015-02-09 18:59:22 +0900 #$* @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */

#ifndef PMU_TZ10XX_H
#define PMU_TZ10XX_H

#include "Driver_Common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* API version */
#define PMU_API_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 22)

/**
 * @brief status of executed operation
 */
typedef enum _PMU_STATUS {
  PMU_OK    = 0, /**< Operation succeeded */
  PMU_ERROR = 1, /**< Unspecified error */
  PMU_ERROR_IGNORED = 2, /**< API call has been ignored */
} PMU_STATUS;

/**
 * @brief PMU driver event
 */
typedef enum _PMU_EVENT {
  PMU_EVENT_CLOCK_SOURCE_STATE_CHANGED
} PMU_EVENT;

/**
 * @brief clock source
 */
typedef enum _PMU_CLOCK_SOURCE {
  PMU_CLOCK_SOURCE_OSC32K,   /**< external 32.768kHz */
  PMU_CLOCK_SOURCE_SIOSC32K, /**< internal 32.768kHz */
  PMU_CLOCK_SOURCE_SIOSC4M,  /**< internal 4MHz */
  PMU_CLOCK_SOURCE_ADPLL,    /**< All Digital PLL */
  PMU_CLOCK_SOURCE_OSC12M,   /**< external 12MHz */
  PMU_CLOCK_SOURCE_PLL,      /**< PLL */
  PMU_CLOCK_SOURCE_32K,      /**< 32.768kHz */
  PMU_CLOCK_SOURCE_INVALID
} PMU_CLOCK_SOURCE;

/**
 * @brief clock source state
 */
typedef enum _PMU_CLOCK_SOURCE_STATE {
  PMU_CLOCK_SOURCE_STATE_STOPPED,   /**< clock is stopped */
  PMU_CLOCK_SOURCE_STATE_PREPARING, /**< clock is running but unstable */
  PMU_CLOCK_SOURCE_STATE_RUNNING,   /**< clock is running */
} PMU_CLOCK_SOURCE_STATE;


/**
 * @brief clock source selector
 */
typedef enum _PMU_CSM {
  PMU_CSM_MAIN,   /**< CSM_MAIN */
  PMU_CSM_RTC,    /**< CSM_RTC */
  PMU_CSM_CPUTRC, /**< CSM_CPUTRC */
  PMU_CSM_CPUST,  /**< CSM_CPUST */
  PMU_CSM_PMULV,  /**< CSM_PMULV */
  PMU_CSM_UART0,  /**< CSM_UART0 */
  PMU_CSM_UART1,  /**< CSM_UART1 */
  PMU_CSM_UART2,  /**< CSM_UART2 */
  PMU_CSM_USB,    /**< CSM_USB */
  PMU_CSM_ADCC12, /**< CSM_ADCC12 */
  PMU_CSM_ADCC24, /**< CSM_ADCC24 */
  PMU_CSM_INVALID
} PMU_CSM;

/**
 * @brief ClockDomain
 */
typedef enum _PMU_CD {
  PMU_CD_MPIER,   /**< CD_MPIER */
  PMU_CD_PPIER0,  /**< CD_PPIER0 */
  PMU_CD_PPIER1,  /**< CD_PPIER1 */
  PMU_CD_PPIER2,  /**< CD_PPIER2 */
  PMU_CD_SPIC,    /**< CD_SPIC */
  PMU_CD_USBB,    /**< CD_USBB */
  PMU_CD_RTC,     /**< CD_RTC */
  PMU_CD_CPUTRC,  /**< CD_CPUTRC */
  PMU_CD_CPUST,   /**< CD_CPUST */
  PMU_CD_PMULV,   /**< CD_PMULV */
  PMU_CD_UART0,   /**< CD_UART0 */
  PMU_CD_UART1,   /**< CD_UART1 */
  PMU_CD_UART2,   /**< CD_UART2 */
  PMU_CD_USBI,    /**< CD_USBI */
  PMU_CD_ADCC12, /**< CD_ADCC12 */
  PMU_CD_ADCC24, /**< CD_ADCC24 */
  PMU_CD_INVALID
} PMU_CD;

/**
 * @brief PowerDomain
 */
typedef enum _PMU_PD {
  PMU_PD_CPU,     /**< POWER_CPU */
  PMU_PD_USB,     /**< POWER_USB */
  PMU_PD_FLASH,   /**< POWER_FLASH */
  PMU_PD_SRAM0,   /**< POWER_SRAM0 */
  PMU_PD_SRAM1,   /**< POWER_SRAM1 */
  PMU_PD_SRAM2,   /**< POWER_SRAM2 */
  PMU_PD_DMAC,    /**< POWER_DMAC */
  PMU_PD_ENCRYPT, /**< POWER_ENCRYPT */
  PMU_PD_PPIER1,  /**< POWER_PPIER1 */
  PMU_PD_ADCC12,  /**< POWER_ADCC12 */
  PMU_PD_ADCC24,  /**< POWER_ADCC24 */
  PMU_PD_MAIN,    /**< POWER_MAIN */
  PMU_PD_EFUSE,   /**< POWER_EFUSE */
  PMU_PD_PLL,     /**< POWER_PLL */
  PMU_PD_ADPLL,   /**< POWER_ADPLL */
  PMU_PD_AON_PM,  /**< POWER_AlwaysON (PM domain) */
  PMU_PD_AON_PP1  /**< POWER_AlwaysON (PP1 domain) */
} PMU_PD;

/**
 * @brief state of PowerDomain
 */
typedef enum _PMU_PD_MODE {
  PMU_PD_MODE_ON,        /**< ON */
  PMU_PD_MODE_OFF,       /**< OFF */
  PMU_PD_MODE_RETENTION, /**< RETENTION */
  PMU_PD_MODE_WAIT       /**< WAIT */
} PMU_PD_MODE;

/**
 * @brief state of VoltageMode
 */
typedef enum _PMU_VOLTAGE_MODE {
  PMU_VOLTAGE_MODE_A, /**< VoltageMode A */
  PMU_VOLTAGE_MODE_B, /**< VoltageMode B */
  PMU_VOLTAGE_MODE_C, /**< VoltageMode C */
  PMU_VOLTAGE_MODE_D  /**< VoltageMode D */
} PMU_VOLTAGE_MODE;

/**
 * @brief PowerMode
 */
typedef enum _PMU_POWER_MODE {
  PMU_POWER_MODE_ACTIVE,         /**< ACTIVE */
  PMU_POWER_MODE_SLEEP0,         /**< SLEEP0 with SLEEPING */
  PMU_POWER_MODE_SLEEP1,         /**< SLEEP1 with SLEEPING */
  PMU_POWER_MODE_SLEEP2,         /**< SLEEP2 with SLEEPING */
  PMU_POWER_MODE_WAIT,           /**< WAIT */
  PMU_POWER_MODE_WAIT_RETENTION, /**< WAIT_RETENTION */
  PMU_POWER_MODE_RETENTION,      /**< RETENTION */
  PMU_POWER_MODE_RTC,            /**< RTC */
  PMU_POWER_MODE_STOP,           /**< STOP */
  PMU_POWER_MODE_SLEEPDEEP0,     /**< SLEEP0 with SLEEPDEEP */
  PMU_POWER_MODE_SLEEPDEEP1,     /**< SLEEP1 with SLEEPDEEP */
  PMU_POWER_MODE_SLEEPDEEP2      /**< SLEEP2 with SLEEPDEEP */
} PMU_POWER_MODE;

/**
 * @brief internal sensor
 */
typedef enum _PMU_SENSOR {
  PMU_SENSOR_ACC,  /**< accelerometer */
  PMU_SENSOR_GYRO, /**< gyrometer */
  PMU_SENSOR_MAG   /**< magnetometer */
} PMU_SENSOR;

#define PMU_WAKEUP_FACTOR_GPIO_0   (1u <<  0)
#define PMU_WAKEUP_FACTOR_GPIO_1   (1u <<  1)
#define PMU_WAKEUP_FACTOR_GPIO_2   (1u <<  2)
#define PMU_WAKEUP_FACTOR_GPIO_3   (1u <<  3)
#define PMU_WAKEUP_FACTOR_GPIO_4   (1u <<  4)
#define PMU_WAKEUP_FACTOR_GPIO_5   (1u <<  5)
#define PMU_WAKEUP_FACTOR_GPIO_6   (1u <<  6)
#define PMU_WAKEUP_FACTOR_GPIO_7   (1u <<  7)
#define PMU_WAKEUP_FACTOR_GPIO_24  (1u <<  8)
#define PMU_WAKEUP_FACTOR_GPIO_25  (1u <<  9)
#define PMU_WAKEUP_FACTOR_GPIO_26  (1u << 10)
#define PMU_WAKEUP_FACTOR_GPIO_27  (1u << 11)
#define PMU_WAKEUP_FACTOR_GPIO_30  (1u << 12)
#define PMU_WAKEUP_FACTOR_BROWNOUT (1u << 15)
#define PMU_WAKEUP_FACTOR_RTC      (1u << 16)
#define PMU_WAKEUP_FACTOR_DEBUG    (1u << 17)

/**
 * @brief behavior when brown out is detected
 */
typedef enum _PMU_BROWN_OUT_MODE {
  PMU_BROWN_OUT_IGNORE,
  PMU_BROWN_OUT_RESET,
  PMU_BROWN_OUT_NOTIFY_EXCEPT_RTC_STOP,
  PMU_BROWN_OUT_NOTIFY
} PMU_BROWN_OUT_MODE;

/**
 * @brief wakeup condition
 */
typedef enum _PMU_WAKEUP_EVENT {
  PMU_WAKEUP_EVENT_DISABLE,     /* disabled */
  PMU_WAKEUP_EVENT_LEVEL_HIGH,  /* high level */
  PMU_WAKEUP_EVENT_LEVEL_LOW,   /* low level */
  PMU_WAKEUP_EVENT_EDGE_POS,    /* rising edge */
  PMU_WAKEUP_EVENT_EDGE_NEG,    /* falling edge */
  PMU_WAKEUP_EVENT_EDGE_BOTH,   /* both edges */
} PMU_WAKEUP_EVENT;

#define PMU_RESET_BY_SYS_RESET_N (1u << 0)
#define PMU_RESET_BY_WDT         (1u << 4)
#define PMU_RESET_BY_LOCKUP      (1u << 8)
#define PMU_RESET_BY_SYSRESETREQ (1u << 12)
#define PMU_RESET_BY_BOR         (1u << 16)
#define PMU_RESET_BY_RTC         (1u << 20)
#define PMU_RESET_BY_STOP        (1u << 24)

/**
 * @brief hardware module
 */
typedef enum _PMU_MODULE {
  PMU_MODULE_MPIER,
  PMU_MODULE_SDMAC,
  PMU_MODULE_AESA,
  PMU_MODULE_RNG,
  PMU_MODULE_SRAMC,
  PMU_MODULE_SPIC,
  PMU_MODULE_UART0,
  PMU_MODULE_UART1,
  PMU_MODULE_UART2,
  PMU_MODULE_I2C0,
  PMU_MODULE_I2C1,
  PMU_MODULE_I2C2,
  PMU_MODULE_SPIM0,
  PMU_MODULE_SPIM1,
  PMU_MODULE_SPIM2,
  PMU_MODULE_SPIM3,
  PMU_MODULE_WDT,
  PMU_MODULE_ADVTMR,
  PMU_MODULE_TMR,
  PMU_MODULE_GCONF,
  PMU_MODULE_GPIO0,
  PMU_MODULE_GPIO1,
  PMU_MODULE_GPIO2,
  PMU_MODULE_GPIO3,
  PMU_MODULE_ADCC12,
  PMU_MODULE_ADCC24,
  PMU_MODULE_RTCLV,
  PMU_MODULE_EVM,
  PMU_MODULE_USB2FS
} PMU_MODULE;

typedef enum _PMU_IO_FUNC {
  PMU_IO_FUNC_GPIO_0,
  PMU_IO_FUNC_GPIO_1,
  PMU_IO_FUNC_GPIO_2,
  PMU_IO_FUNC_GPIO_3,
  PMU_IO_FUNC_GPIO_4,
  PMU_IO_FUNC_GPIO_5,
  PMU_IO_FUNC_GPIO_6,
  PMU_IO_FUNC_GPIO_7,
  PMU_IO_FUNC_GPIO_8,
  PMU_IO_FUNC_GPIO_9,
  PMU_IO_FUNC_GPIO_10,
  PMU_IO_FUNC_GPIO_11,
  PMU_IO_FUNC_GPIO_12,
  PMU_IO_FUNC_GPIO_13,
  PMU_IO_FUNC_GPIO_14,
  PMU_IO_FUNC_GPIO_15,
  PMU_IO_FUNC_GPIO_16,
  PMU_IO_FUNC_GPIO_17,
  PMU_IO_FUNC_GPIO_18,
  PMU_IO_FUNC_GPIO_19,
  PMU_IO_FUNC_GPIO_20,
  PMU_IO_FUNC_GPIO_21,
  PMU_IO_FUNC_GPIO_22,
  PMU_IO_FUNC_GPIO_23,
  PMU_IO_FUNC_GPIO_24,
  PMU_IO_FUNC_GPIO_25,
  PMU_IO_FUNC_GPIO_26,
  PMU_IO_FUNC_GPIO_27,
  PMU_IO_FUNC_GPIO_28,
  PMU_IO_FUNC_GPIO_29,
  PMU_IO_FUNC_GPIO_30,
  PMU_IO_FUNC_GPIO_31,
  PMU_IO_FUNC_I2C0_DATA,
  PMU_IO_FUNC_I2C0_CLK,
  PMU_IO_FUNC_I2C1_DATA,
  PMU_IO_FUNC_I2C1_CLK,
  PMU_IO_FUNC_I2C2_DATA,
  PMU_IO_FUNC_I2C2_CLK,
  PMU_IO_FUNC_UA0_RXD,
  PMU_IO_FUNC_UA0_TXD,
  PMU_IO_FUNC_UA0_RTS_N,
  PMU_IO_FUNC_UA0_CTS_N,
  PMU_IO_FUNC_UA1_RXD,
  PMU_IO_FUNC_UA1_TXD,
  PMU_IO_FUNC_UA1_RTS_N,
  PMU_IO_FUNC_UA1_CTS_N,
  PMU_IO_FUNC_UA2_RXD,
  PMU_IO_FUNC_UA2_TXD,
  PMU_IO_FUNC_UA2_RTS_N,
  PMU_IO_FUNC_UA2_CTS_N,
  PMU_IO_FUNC_SPIM0_CS_N,
  PMU_IO_FUNC_SPIM0_CLK,
  PMU_IO_FUNC_SPIM0_MOSI,
  PMU_IO_FUNC_SPIM0_MISO,
  PMU_IO_FUNC_SPIM1_CS_N,
  PMU_IO_FUNC_SPIM1_CLK,
  PMU_IO_FUNC_SPIM1_MOSI,
  PMU_IO_FUNC_SPIM1_MISO,
  PMU_IO_FUNC_SPIM2_CS_N,
  PMU_IO_FUNC_SPIM2_CLK,
  PMU_IO_FUNC_SPIM2_MOSI,
  PMU_IO_FUNC_SPIM2_MISO,
  PMU_IO_FUNC_SPIM3_CS_N,
  PMU_IO_FUNC_SPIM3_CLK,
  PMU_IO_FUNC_SPIM3_MOSI,
  PMU_IO_FUNC_SPIM3_MISO,
  PMU_IO_FUNC_PWM0,
  PMU_IO_FUNC_PWM1,
  PMU_IO_FUNC_PWM2,
  PMU_IO_FUNC_PWM3,
  PMU_IO_FUNC_CAPTURE0,
  PMU_IO_FUNC_CAPTURE1,
  PMU_IO_FUNC_CAPTURE2,
  PMU_IO_FUNC_CAPTURE3,
  PMU_IO_FUNC_SPIC_CS_N,
  PMU_IO_FUNC_SPIC_CLK,
  PMU_IO_FUNC_SPIC_MOSI,
  PMU_IO_FUNC_SPIC_MISO,
  PMU_IO_FUNC_SPIC_IO2,
  PMU_IO_FUNC_SPIC_IO3,
  PMU_IO_FUNC_TRACECLK,
  PMU_IO_FUNC_TRACEDATA0,
  PMU_IO_FUNC_TRACEDATA1,
  PMU_IO_FUNC_TRACEDATA2,
  PMU_IO_FUNC_TRACEDATA3,
  PMU_IO_FUNC_ADC24_SYNC,
  PMU_IO_FUNC_DBG_TCK,
  PMU_IO_FUNC_DBG_TDO,
  PMU_IO_FUNC_DBG_TDI,
  PMU_IO_FUNC_DBG_TMS,
  PMU_IO_FUNC_DBG_TRST_N,
  PMU_IO_FUNC_INVALID
} PMU_IO_FUNC;

typedef enum _PMU_IO_PIN {
  PMU_IO_PIN_GPIO_0,
  PMU_IO_PIN_GPIO_1,
  PMU_IO_PIN_GPIO_2,
  PMU_IO_PIN_GPIO_3,
  PMU_IO_PIN_GPIO_4,
  PMU_IO_PIN_GPIO_5,
  PMU_IO_PIN_GPIO_6,
  PMU_IO_PIN_GPIO_7,
  PMU_IO_PIN_GPIO_8,
  PMU_IO_PIN_GPIO_9,
  PMU_IO_PIN_GPIO_10,
  PMU_IO_PIN_GPIO_11,
  PMU_IO_PIN_GPIO_12,
  PMU_IO_PIN_GPIO_13,
  PMU_IO_PIN_GPIO_14,
  PMU_IO_PIN_GPIO_15,
  PMU_IO_PIN_GPIO_24,
  PMU_IO_PIN_GPIO_25,
  PMU_IO_PIN_GPIO_26,
  PMU_IO_PIN_GPIO_27,
  PMU_IO_PIN_GPIO_28,
  PMU_IO_PIN_GPIO_29,
  PMU_IO_PIN_GPIO_30,
  PMU_IO_PIN_GPIO_31,
  PMU_IO_PIN_I2C0_DATA,
  PMU_IO_PIN_I2C0_CLK,
  PMU_IO_PIN_I2C1_DATA,
  PMU_IO_PIN_I2C1_CLK,
  PMU_IO_PIN_I2C2_DATA,
  PMU_IO_PIN_I2C2_CLK,
  PMU_IO_PIN_UA0_RXD,
  PMU_IO_PIN_UA0_TXD,
  PMU_IO_PIN_UA1_RXD,
  PMU_IO_PIN_UA1_TXD,
  PMU_IO_PIN_UA1_RTS_N,
  PMU_IO_PIN_UA1_CTS_N,
  PMU_IO_PIN_UA2_RXD,
  PMU_IO_PIN_UA2_TXD,
  PMU_IO_PIN_UA2_RTS_N,
  PMU_IO_PIN_UA2_CTS_N,
  PMU_IO_PIN_SPIM0_CS_N,
  PMU_IO_PIN_SPIM0_CLK,
  PMU_IO_PIN_SPIM0_MOSI,
  PMU_IO_PIN_SPIM0_MISO,
  PMU_IO_PIN_SPIM1_CS_N,
  PMU_IO_PIN_SPIM1_CLK,
  PMU_IO_PIN_SPIM1_MOSI,
  PMU_IO_PIN_SPIM1_MISO,
  PMU_IO_PIN_SPIM2_CS_N,
  PMU_IO_PIN_SPIM2_CLK,
  PMU_IO_PIN_SPIM2_MOSI,
  PMU_IO_PIN_SPIM2_MISO,
  PMU_IO_PIN_SPIM3_CS_N,
  PMU_IO_PIN_SPIM3_CLK,
  PMU_IO_PIN_SPIM3_MOSI,
  PMU_IO_PIN_SPIM3_MISO,
  PMU_IO_PIN_SPIC_CS_N,
  PMU_IO_PIN_SPIC_CLK,
  PMU_IO_PIN_SPIC_MOSI,
  PMU_IO_PIN_SPIC_MISO,
  PMU_IO_PIN_SPIC_IO2,
  PMU_IO_PIN_SPIC_IO3,
  PMU_IO_PIN_ADC24_SYNC,
  PMU_IO_PIN_DBG,
  PMU_IO_PIN_INVALID,
} PMU_IO_PIN;

typedef enum _PMU_IO_DRIVE_CAPABILITY {
  PMU_DRIVE_CAPABILITY_2MA, /**< 2mA */
  PMU_DRIVE_CAPABILITY_4MA, /**< 4mA */
  PMU_DRIVE_CAPABILITY_5MA, /**< 5mA */
  PMU_DRIVE_CAPABILITY_7MA, /**< 7mA */
} PMU_IO_DRIVE_CAPABILITY;

typedef enum _PMU_IO_RESISTOR {
  PMU_IO_RESISTOR_PULLDOWN,
  PMU_IO_RESISTOR_NONE,
  PMU_IO_RESISTOR_PULLUP,
} PMU_IO_RESISTOR;

typedef void (*PMU_SignalEvent_t)(PMU_EVENT);

typedef struct _TZ10XX_DRIVER_PMU {
  ARM_DRIVER_VERSION (*GetVersion)(void);
    /**< Pointer to \ref PMU_GetVersion */
  PMU_STATUS (*Initialize)(PMU_SignalEvent_t cb_event);
    /**< Pointer to \ref PMU_Initialize */
  PMU_STATUS (*Uninitialize)(void);
    /**< Pointer to \ref PMU_Uninitialize */
  
  PMU_STATUS (*StartClockSource)(PMU_CLOCK_SOURCE source);
    /**< Pointer to \ref PMU_StartClockSource */
  PMU_STATUS (*StopClockSource)(PMU_CLOCK_SOURCE source);
    /**< Pointer to \ref PMU_StopClockSource */
  PMU_CLOCK_SOURCE_STATE (*GetClockSourceState)(
    PMU_CLOCK_SOURCE source);
    /**< Pointer to \ref PMU_GetClockSourceState */
  PMU_STATUS (*SetPLLFrequency)(uint32_t freq);
    /**< Pointer to \ref PMU_SetPLLFrequency */
  PMU_STATUS (*SetPrescaler)(
    PMU_CD clockDomain, uint32_t divider);
    /**< Pointer to \ref PMU_SetPrescaler */
  uint32_t (*GetPrescaler)(PMU_CD clockDomain);
    /**< Pointer to \ref PMU_GetPrescaler */
  PMU_STATUS (*SelectClockSource)(
    PMU_CSM csm, PMU_CLOCK_SOURCE source);
    /**< Pointer to \ref PMU_SelectClockSource */
  uint32_t (*GetFrequency)(PMU_CD clockDomain);
    /**< Pointer to \ref PMU_GetFrequency */
  PMU_STATUS (*SetPowerDomainState)(
    PMU_PD powerDomain, PMU_PD_MODE mode);
    /**< Pointer to \ref PMU_SetPowerDomainMode */
  PMU_STATUS (*SetPowerDomainStateLowPowerMode)(
    PMU_PD powerDomain,
    PMU_PD_MODE modeWait,
    PMU_PD_MODE modeWaitRetention,
    PMU_PD_MODE modeRetention);
    /**< Pointer to \ref PMU_SetPowerDomainLowPowerMode */
  PMU_STATUS (*SetVoltageMode)(PMU_VOLTAGE_MODE mode);
    /**< Pointer to \ref PMU_SetVoltageMode */
  PMU_STATUS (*SetPowerMode)(PMU_POWER_MODE mode);
    /**< Pointer to \ref PMU_SetPowerMode */
  PMU_STATUS (*EnableSensor)(
    PMU_SENSOR sensor, uint32_t enable);
    /**< Pointer to \ref PMU_EnableSensor */
  
  PMU_STATUS (*ConfigureWakeup)(uint32_t factor, PMU_WAKEUP_EVENT event);
    /**< Pointer to \ref PMU_ConfigureWakeup */
  uint32_t (*GetWakeupFactor)(void);
    /**< Pointer to \ref PMU_GetWakeupFactor */
  PMU_STATUS (*SetBrownOutMode)(PMU_BROWN_OUT_MODE mode);
    /**< Pointer to \ref PMU_SetBrownOutMode */
  void (*BrownOutReset)(void);
    /**< Pointer to \ref PMU_BrownOutReset */
  uint32_t (*GetResetFactor)(void);
    /**< Pointer to \ref PMU_GetResetFactor */
  PMU_STATUS (*EnableModule)(PMU_MODULE module, uint32_t enable);
  PMU_STATUS (*EnableModuleOnSleep)(uint32_t module, uint32_t enable);
  PMU_STATUS (*ResetOutputBuffer)(PMU_PD domain, uint32_t enable);
  PMU_STATUS (*RetainOutputBuffer)(PMU_PD domain, uint32_t enable);
  PMU_STATUS (*StandbyInputBuffer)(PMU_IO_FUNC func, uint32_t enable);
  PMU_STATUS (*ConfigureIOCell)
    (PMU_IO_FUNC func,
     PMU_IO_DRIVE_CAPABILITY capability,
     PMU_IO_RESISTOR pullupdown);
  PMU_STATUS (*SetCpuLowFrequencyOnSleep)(bool enable);
  PMU_STATUS (*Enable32kOut)(bool enable);
  PMU_STATUS (*EnableWakeup)(uint32_t factor, bool enable);
  PMU_VOLTAGE_MODE (*GetVoltageMode)(void);
} const TZ10XX_DRIVER_PMU;

extern TZ10XX_DRIVER_PMU Driver_PMU;

#ifdef __cplusplus
}
#endif

#endif /* PMU_TZ10XX_H */
