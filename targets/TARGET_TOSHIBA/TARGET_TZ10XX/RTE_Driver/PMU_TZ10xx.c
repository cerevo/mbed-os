/**
 * @file PMU_TZ10xx.c
 * @brief TZ10xx PMU driver
 * @date $Date:: 2016-02-04 16:56:31 +0900 #$
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */
#include "PMU_TZ10xx.h"
#include "PMU_TZ10xx_AES.h"
#include "RTE_Device.h"
#include "TZ10xx.h"

/* clock sources used internally */
#define PMU_CLOCK_SOURCE_OSC12M3 (PMU_CLOCK_SOURCE_INVALID + 1) /**< external 12MHz / 3 */
#define PMU_CLOCK_SOURCE_PLL3    (PMU_CLOCK_SOURCE_INVALID + 2) /**< PLL / 3 */
#define PMU_CLOCK_SOURCE_PLL9    (PMU_CLOCK_SOURCE_INVALID + 3) /**< PLL / 9 */

typedef struct {
  bool init;
  PMU_SignalEvent_t cb_event;
  /* ClockSourceState of each clock source */
  PMU_CLOCK_SOURCE_STATE clock_state[PMU_CLOCK_SOURCE_INVALID];
  /* PLL frequency in Hz */
  uint32_t pll_freq;
  /* reset factor */
  uint32_t reset_factor;
  /* IRQ_STATUS of last wake-up */
  uint32_t wakeup_status;
  uint8_t module_enabled[PMU_MODULE_USB2FS + 1];
} PMU_INFO;

typedef struct {
  PMU_INFO *info;
} const PMU_RESOURCES;

static PMU_INFO PMU_Info;

static PMU_RESOURCES PMU_Resources = {
  &PMU_Info
};

#define PMU_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 40)   /* driver version */

#if (RTE_GPIO_0_ID != 1)
  #define IOMUX_GPIO_0_FMODE 0
#else
  #define IOMUX_GPIO_0_FMODE 1
#endif

#if (RTE_GPIO_1_ID != 1)
  #define IOMUX_GPIO_1_FMODE 0
#else
  #define IOMUX_GPIO_1_FMODE 1
#endif

#if (RTE_GPIO_2_ID != 1)
  #define IOMUX_GPIO_2_FMODE 0
#else
  #define IOMUX_GPIO_2_FMODE 1
#endif

#if (RTE_GPIO_3_ID != 1)
  #define IOMUX_GPIO_3_FMODE 0
#else
  #define IOMUX_GPIO_3_FMODE 1
#endif

#if (RTE_GPIO_4_ID != 1)
  #define IOMUX_GPIO_4_FMODE 0
#else
  #define IOMUX_GPIO_4_FMODE 1
#endif

#if (RTE_GPIO_5_ID != 1)
  #define IOMUX_GPIO_5_FMODE 0
#else
  #define IOMUX_GPIO_5_FMODE 1
#endif

#if (RTE_GPIO_6_ID != 1)
  #define IOMUX_GPIO_6_FMODE 0
#else
  #define IOMUX_GPIO_6_FMODE 1
#endif

#if (RTE_GPIO_7_ID != 1)
  #define IOMUX_GPIO_7_FMODE 0
#else
  #define IOMUX_GPIO_7_FMODE 1
#endif

#if (RTE_GPIO_8_ID != 1) && (RTE_PWM0_ID != 1) && (RTE_TRACEDATA0_ID != 1)
  #define IOMUX_GPIO_8_FMODE 0
#elif (RTE_GPIO_8_ID == 1) && (RTE_PWM0_ID != 1) && (RTE_TRACEDATA0_ID != 1)
  #define IOMUX_GPIO_8_FMODE 1
#elif (RTE_GPIO_8_ID != 1) && (RTE_PWM0_ID == 1) && (RTE_TRACEDATA0_ID != 1)
  #define IOMUX_GPIO_8_FMODE 2
#elif (RTE_GPIO_8_ID != 1) && (RTE_PWM0_ID != 1) && (RTE_TRACEDATA0_ID == 1)
  #define IOMUX_GPIO_8_FMODE 3
#else
  #error Conflicted IO pin in IOMUX_GPIO_8_FMODE
#endif

#if (RTE_GPIO_9_ID != 1) && (RTE_PWM1_ID != 1) && (RTE_TRACEDATA1_ID != 1)
  #define IOMUX_GPIO_9_FMODE 0
#elif (RTE_GPIO_9_ID == 1) && (RTE_PWM1_ID != 1) && (RTE_TRACEDATA1_ID != 1)
  #define IOMUX_GPIO_9_FMODE 1
#elif (RTE_GPIO_9_ID != 1) && (RTE_PWM1_ID == 1) && (RTE_TRACEDATA1_ID != 1)
  #define IOMUX_GPIO_9_FMODE 2
#elif (RTE_GPIO_9_ID != 1) && (RTE_PWM1_ID != 1) && (RTE_TRACEDATA1_ID == 1)
  #define IOMUX_GPIO_9_FMODE 3
#else
  #error Conflicted IO pin in IOMUX_GPIO_9_FMODE
#endif

#if (RTE_GPIO_10_ID != 1) && (RTE_PWM2_ID != 1) && (RTE_TRACEDATA2_ID != 1)
  #define IOMUX_GPIO_10_FMODE 0
#elif (RTE_GPIO_10_ID == 1) && (RTE_PWM2_ID != 1) && (RTE_TRACEDATA2_ID != 1)
  #define IOMUX_GPIO_10_FMODE 1
#elif (RTE_GPIO_10_ID != 1) && (RTE_PWM2_ID == 1) && (RTE_TRACEDATA2_ID != 1)
  #define IOMUX_GPIO_10_FMODE 2
#elif (RTE_GPIO_10_ID != 1) && (RTE_PWM2_ID != 1) && (RTE_TRACEDATA2_ID == 1)
  #define IOMUX_GPIO_10_FMODE 3
#else
  #error Conflicted IO pin in IOMUX_GPIO_10_FMODE
#endif

#if (RTE_GPIO_11_ID != 1) && (RTE_PWM3_ID != 1) && (RTE_TRACEDATA3_ID != 1)
  #define IOMUX_GPIO_11_FMODE 0
#elif (RTE_GPIO_11_ID == 1) && (RTE_PWM3_ID != 1) && (RTE_TRACEDATA3_ID != 1)
  #define IOMUX_GPIO_11_FMODE 1
#elif (RTE_GPIO_11_ID != 1) && (RTE_PWM3_ID == 1) && (RTE_TRACEDATA3_ID != 1)
  #define IOMUX_GPIO_11_FMODE 2
#elif (RTE_GPIO_11_ID != 1) && (RTE_PWM3_ID != 1) && (RTE_TRACEDATA3_ID == 1)
  #define IOMUX_GPIO_11_FMODE 3
#else
  #error Conflicted IO pin in IOMUX_GPIO_11_FMODE
#endif

#if (RTE_GPIO_12_ID != 1) && (RTE_SPIM3_CS_N_ID != 2) && (RTE_CAPTURE0_ID != 1)
  #define IOMUX_GPIO_12_FMODE 0
#elif (RTE_GPIO_12_ID == 1) && (RTE_SPIM3_CS_N_ID != 2) && (RTE_CAPTURE0_ID != 1)
  #define IOMUX_GPIO_12_FMODE 1
#elif (RTE_GPIO_12_ID != 1) && (RTE_SPIM3_CS_N_ID == 2) && (RTE_CAPTURE0_ID != 1)
  #define IOMUX_GPIO_12_FMODE 2
#elif (RTE_GPIO_12_ID != 1) && (RTE_SPIM3_CS_N_ID != 2) && (RTE_CAPTURE0_ID == 1)
  #define IOMUX_GPIO_12_FMODE 3
#else
  #error Conflicted IO pin in IOMUX_GPIO_12_FMODE
#endif

#if (RTE_GPIO_13_ID != 1) && (RTE_SPIM3_CLK_ID != 2) && (RTE_CAPTURE1_ID != 1)
  #define IOMUX_GPIO_13_FMODE 0
#elif (RTE_GPIO_13_ID == 1) && (RTE_SPIM3_CLK_ID != 2) && (RTE_CAPTURE1_ID != 1)
  #define IOMUX_GPIO_13_FMODE 1
#elif (RTE_GPIO_13_ID != 1) && (RTE_SPIM3_CLK_ID == 2) && (RTE_CAPTURE1_ID != 1)
  #define IOMUX_GPIO_13_FMODE 2
#elif (RTE_GPIO_13_ID != 1) && (RTE_SPIM3_CLK_ID != 2) && (RTE_CAPTURE1_ID == 1)
  #define IOMUX_GPIO_13_FMODE 3
#else
  #error Conflicted IO pin in IOMUX_GPIO_13_FMODE
#endif

#if (RTE_GPIO_14_ID != 1) && (RTE_SPIM3_MOSI_ID != 2) && (RTE_UA0_RTS_N_ID != 1)
  #define IOMUX_GPIO_14_FMODE 0
#elif (RTE_GPIO_14_ID == 1) && (RTE_SPIM3_MOSI_ID != 2) && (RTE_UA0_RTS_N_ID != 1)
  #define IOMUX_GPIO_14_FMODE 1
#elif (RTE_GPIO_14_ID != 1) && (RTE_SPIM3_MOSI_ID == 2) && (RTE_UA0_RTS_N_ID != 1)
  #define IOMUX_GPIO_14_FMODE 2
#elif (RTE_GPIO_14_ID != 1) && (RTE_SPIM3_MOSI_ID != 2) && (RTE_UA0_RTS_N_ID == 1)
  #define IOMUX_GPIO_14_FMODE 3
#else
  #error Conflicted IO pin in IOMUX_GPIO_14_FMODE
#endif

#if (RTE_GPIO_15_ID != 1) && (RTE_SPIM3_MISO_ID != 2) && (RTE_UA0_CTS_N_ID != 1)
  #define IOMUX_GPIO_15_FMODE 0
#elif (RTE_GPIO_15_ID == 1) && (RTE_SPIM3_MISO_ID != 2) && (RTE_UA0_CTS_N_ID != 1)
  #define IOMUX_GPIO_15_FMODE 1
#elif (RTE_GPIO_15_ID != 1) && (RTE_SPIM3_MISO_ID == 2) && (RTE_UA0_CTS_N_ID != 1)
  #define IOMUX_GPIO_15_FMODE 2
#elif (RTE_GPIO_15_ID != 1) && (RTE_SPIM3_MISO_ID != 2) && (RTE_UA0_CTS_N_ID == 1)
  #define IOMUX_GPIO_15_FMODE 3
#else
  #error Conflicted IO pin in IOMUX_GPIO_15_FMODE
#endif

#if (RTE_GPIO_24_ID != 1)
  #define IOMUX_GPIO_24_FMODE 0
#else
  #define IOMUX_GPIO_24_FMODE 1
#endif

#if (RTE_GPIO_25_ID != 1)
  #define IOMUX_GPIO_25_FMODE 0
#else
  #define IOMUX_GPIO_25_FMODE 1
#endif

#if (RTE_GPIO_26_ID != 1)
  #define IOMUX_GPIO_26_FMODE 0
#else
  #define IOMUX_GPIO_26_FMODE 1
#endif

#if (RTE_GPIO_27_ID != 1) && (RTE_PWM3_ID != 2)
  #define IOMUX_GPIO_27_FMODE 0
#elif (RTE_GPIO_27_ID == 1) && (RTE_PWM3_ID != 2)
  #define IOMUX_GPIO_27_FMODE 1
#elif (RTE_GPIO_27_ID != 1) && (RTE_PWM3_ID == 2)
  #define IOMUX_GPIO_27_FMODE 3
#else
  #error Conflicted IO pin in IOMUX_GPIO_27_FMODE
#endif

#if (RTE_GPIO_28_ID != 1)
  #define IOMUX_GPIO_28_FMODE 0
#else
  #define IOMUX_GPIO_28_FMODE 1
#endif

#if (RTE_GPIO_29_ID != 1)
  #define IOMUX_GPIO_29_FMODE 0
#else
  #define IOMUX_GPIO_29_FMODE 1
#endif

#if (RTE_GPIO_30_ID != 1)
  #define IOMUX_GPIO_30_FMODE 0
#else
  #define IOMUX_GPIO_30_FMODE 1
#endif

#if (RTE_GPIO_31_ID != 1)
  #define IOMUX_GPIO_31_FMODE 0
#else
  #define IOMUX_GPIO_31_FMODE 1
#endif

#if (RTE_I2C0_DATA_ID != 1) && (RTE_CAPTURE2_ID != 1) && (RTE_GPIO_16_ID != 1)
  #define IOMUX_I2C0_DATA_FMODE 0
#elif (RTE_I2C0_DATA_ID == 1) && (RTE_CAPTURE2_ID != 1) && (RTE_GPIO_16_ID != 1)
  #define IOMUX_I2C0_DATA_FMODE 1
#elif (RTE_I2C0_DATA_ID != 1) && (RTE_CAPTURE2_ID == 1) && (RTE_GPIO_16_ID != 1)
  #define IOMUX_I2C0_DATA_FMODE 2
#elif (RTE_I2C0_DATA_ID != 1) && (RTE_CAPTURE2_ID != 1) && (RTE_GPIO_16_ID == 1)
  #define IOMUX_I2C0_DATA_FMODE 3
#else
  #error Conflicted IO pin in IOMUX_I2C0_DATA_FMODE
#endif

#if (RTE_I2C0_CLK_ID != 1) && (RTE_CAPTURE3_ID != 1) && (RTE_GPIO_17_ID != 1)
  #define IOMUX_I2C0_CLK_FMODE 0
#elif (RTE_I2C0_CLK_ID == 1) && (RTE_CAPTURE3_ID != 1) && (RTE_GPIO_17_ID != 1)
  #define IOMUX_I2C0_CLK_FMODE 1
#elif (RTE_I2C0_CLK_ID != 1) && (RTE_CAPTURE3_ID == 1) && (RTE_GPIO_17_ID != 1)
  #define IOMUX_I2C0_CLK_FMODE 2
#elif (RTE_I2C0_CLK_ID != 1) && (RTE_CAPTURE3_ID != 1) && (RTE_GPIO_17_ID == 1)
  #define IOMUX_I2C0_CLK_FMODE 3
#else
  #error Conflicted IO pin in IOMUX_I2C0_CLK_FMODE
#endif

#if (RTE_I2C1_DATA_ID != 1) && (RTE_UA0_RTS_N_ID != 2) && (RTE_GPIO_14_ID != 2)
  #define IOMUX_I2C1_DATA_FMODE 0
#elif (RTE_I2C1_DATA_ID == 1) && (RTE_UA0_RTS_N_ID != 2) && (RTE_GPIO_14_ID != 2)
  #define IOMUX_I2C1_DATA_FMODE 1
#elif (RTE_I2C1_DATA_ID != 1) && (RTE_UA0_RTS_N_ID == 2) && (RTE_GPIO_14_ID != 2)
  #define IOMUX_I2C1_DATA_FMODE 2
#elif (RTE_I2C1_DATA_ID != 1) && (RTE_UA0_RTS_N_ID != 2) && (RTE_GPIO_14_ID == 2)
  #define IOMUX_I2C1_DATA_FMODE 3
#else
  #error Conflicted IO pin in IOMUX_I2C1_DATA_FMODE
#endif

#if (RTE_I2C1_CLK_ID != 1) && (RTE_UA0_CTS_N_ID != 2) && (RTE_GPIO_15_ID != 2)
  #define IOMUX_I2C1_CLK_FMODE 0
#elif (RTE_I2C1_CLK_ID == 1) && (RTE_UA0_CTS_N_ID != 2) && (RTE_GPIO_15_ID != 2)
  #define IOMUX_I2C1_CLK_FMODE 1
#elif (RTE_I2C1_CLK_ID != 1) && (RTE_UA0_CTS_N_ID == 2) && (RTE_GPIO_15_ID != 2)
  #define IOMUX_I2C1_CLK_FMODE 2
#elif (RTE_I2C1_CLK_ID != 1) && (RTE_UA0_CTS_N_ID != 2) && (RTE_GPIO_15_ID == 2)
  #define IOMUX_I2C1_CLK_FMODE 3
#else
  #error Conflicted IO pin in IOMUX_I2C1_CLK_FMODE
#endif

#if (RTE_I2C2_DATA_ID != 1) && (RTE_I2C2_CLK_ID != 1)
  #define IOMUX_I2C2_FMODE 0
#elif (RTE_I2C2_DATA_ID == 1) && (RTE_I2C2_CLK_ID == 1)
  #define IOMUX_I2C2_FMODE 1
#else
  #error Conflicted IO pin in IOMUX_I2C2_CLK_FMODE
#endif

#if (RTE_UA0_RXD_ID != 1) && (RTE_CAPTURE0_ID != 2) && (RTE_GPIO_20_ID != 1)
  #define IOMUX_UA0_RXD_FMODE 0
#elif (RTE_UA0_RXD_ID == 1) && (RTE_CAPTURE0_ID != 2) && (RTE_GPIO_20_ID != 1)
  #define IOMUX_UA0_RXD_FMODE 1
#elif (RTE_UA0_RXD_ID != 1) && (RTE_CAPTURE0_ID == 2) && (RTE_GPIO_20_ID != 1)
  #define IOMUX_UA0_RXD_FMODE 2
#elif (RTE_UA0_RXD_ID != 1) && (RTE_CAPTURE0_ID != 2) && (RTE_GPIO_20_ID == 1)
  #define IOMUX_UA0_RXD_FMODE 3
#else
  #error Conflicted IO pin in IOMUX_UA0_RXD_FMODE
#endif

#if (RTE_UA0_TXD_ID != 1) && (RTE_CAPTURE1_ID != 2) && (RTE_GPIO_21_ID != 1)
  #define IOMUX_UA0_TXD_FMODE 0
#elif (RTE_UA0_TXD_ID == 1) && (RTE_CAPTURE1_ID != 2) && (RTE_GPIO_21_ID != 1)
  #define IOMUX_UA0_TXD_FMODE 1
#elif (RTE_UA0_TXD_ID != 1) && (RTE_CAPTURE1_ID == 2) && (RTE_GPIO_21_ID != 1)
  #define IOMUX_UA0_TXD_FMODE 2
#elif (RTE_UA0_TXD_ID != 1) && (RTE_CAPTURE1_ID != 2) && (RTE_GPIO_21_ID == 1)
  #define IOMUX_UA0_TXD_FMODE 3
#else
  #error Conflicted IO pin in IOMUX_UA0_TXD_FMODE
#endif

#if (RTE_UA1_RXD_ID != 1) && (RTE_I2C0_CLK_ID != 2) && (RTE_GPIO_12_ID != 2)
  #define IOMUX_UA1_RXD_FMODE 0
#elif (RTE_UA1_RXD_ID == 1) && (RTE_I2C0_CLK_ID != 2) && (RTE_GPIO_12_ID != 2)
  #define IOMUX_UA1_RXD_FMODE 1
#elif (RTE_UA1_RXD_ID != 1) && (RTE_I2C0_CLK_ID == 2) && (RTE_GPIO_12_ID != 2)
  #define IOMUX_UA1_RXD_FMODE 2
#elif (RTE_UA1_RXD_ID != 1) && (RTE_I2C0_CLK_ID != 2) && (RTE_GPIO_12_ID == 2)
  #define IOMUX_UA1_RXD_FMODE 3
#else
  #error Conflicted IO pin in IOMUX_UA1_RXD_FMODE
#endif

#if (RTE_UA1_TXD_ID != 1) && (RTE_I2C0_DATA_ID != 2) && (RTE_GPIO_13_ID != 2)
  #define IOMUX_UA1_TXD_FMODE 0
#elif (RTE_UA1_TXD_ID == 1) && (RTE_I2C0_DATA_ID != 2) && (RTE_GPIO_13_ID != 2)
  #define IOMUX_UA1_TXD_FMODE 1
#elif (RTE_UA1_TXD_ID != 1) && (RTE_I2C0_DATA_ID == 2) && (RTE_GPIO_13_ID != 2)
  #define IOMUX_UA1_TXD_FMODE 2
#elif (RTE_UA1_TXD_ID != 1) && (RTE_I2C0_DATA_ID != 2) && (RTE_GPIO_13_ID == 2)
  #define IOMUX_UA1_TXD_FMODE 3
#else
  #error Conflicted IO pin in IOMUX_UA1_TXD_FMODE
#endif

#if (RTE_UA1_RTS_N_ID != 1) && (RTE_I2C2_DATA_ID != 2) && (RTE_GPIO_22_ID != 1)
  #define IOMUX_UA1_RTS_N_FMODE 0
#elif (RTE_UA1_RTS_N_ID == 1) && (RTE_I2C2_DATA_ID != 2) && (RTE_GPIO_22_ID != 1)
  #define IOMUX_UA1_RTS_N_FMODE 1
#elif (RTE_UA1_RTS_N_ID != 1) && (RTE_I2C2_DATA_ID == 2) && (RTE_GPIO_22_ID != 1)
  #define IOMUX_UA1_RTS_N_FMODE 2
#elif (RTE_UA1_RTS_N_ID != 1) && (RTE_I2C2_DATA_ID != 2) && (RTE_GPIO_22_ID == 1)
  #define IOMUX_UA1_RTS_N_FMODE 3
#else
  #error Conflicted IO pin in IOMUX_UA1_RTS_N_FMODE
#endif

#if (RTE_UA1_CTS_N_ID != 1) && (RTE_I2C2_CLK_ID != 2) && (RTE_GPIO_23_ID != 1)
  #define IOMUX_UA1_CTS_N_FMODE 0
#elif (RTE_UA1_CTS_N_ID == 1) && (RTE_I2C2_CLK_ID != 2) && (RTE_GPIO_23_ID != 1)
  #define IOMUX_UA1_CTS_N_FMODE 1
#elif (RTE_UA1_CTS_N_ID != 1) && (RTE_I2C2_CLK_ID == 2) && (RTE_GPIO_23_ID != 1)
  #define IOMUX_UA1_CTS_N_FMODE 2
#elif (RTE_UA1_CTS_N_ID != 1) && (RTE_I2C2_CLK_ID != 2) && (RTE_GPIO_23_ID == 1)
  #define IOMUX_UA1_CTS_N_FMODE 3
#else
  #error Conflicted IO pin in IOMUX_UA1_CTS_N_FMODE
#endif

#if (RTE_UA2_RXD_ID != 1) && (RTE_GPIO_18_ID != 1)
  #define IOMUX_UA2_RXD_FMODE 0
#elif (RTE_UA2_RXD_ID == 1) && (RTE_GPIO_18_ID != 1)
  #define IOMUX_UA2_RXD_FMODE 1
#elif (RTE_UA2_RXD_ID != 1) && (RTE_GPIO_18_ID == 1)
  #define IOMUX_UA2_RXD_FMODE 3
#else
  #error Conflicted IO pin in IOMUX_UA2_RXD_FMODE
#endif

#if (RTE_UA2_TXD_ID != 1) && (RTE_GPIO_19_ID != 1)
  #define IOMUX_UA2_TXD_FMODE 0
#elif (RTE_UA2_TXD_ID == 1) && (RTE_GPIO_19_ID != 1)
  #define IOMUX_UA2_TXD_FMODE 1
#elif (RTE_UA2_TXD_ID != 1) && (RTE_GPIO_19_ID == 1)
  #define IOMUX_UA2_TXD_FMODE 3
#else
  #error Conflicted IO pin in IOMUX_UA2_TXD_FMODE
#endif

#if (RTE_UA2_RTS_N_ID != 1) && (RTE_GPIO_10_ID != 2)
  #define IOMUX_UA2_RTS_N_FMODE 0
#elif (RTE_UA2_RTS_N_ID == 1) && (RTE_GPIO_10_ID != 2)
  #define IOMUX_UA2_RTS_N_FMODE 1
#elif (RTE_UA2_RTS_N_ID != 1) && (RTE_GPIO_10_ID == 2)
  #define IOMUX_UA2_RTS_N_FMODE 3
#else
  #error Conflicted IO pin in IOMUX_UA2_RTS_N_FMODE
#endif

#if (RTE_UA2_CTS_N_ID != 1) && (RTE_GPIO_11_ID != 2)
  #define IOMUX_UA2_CTS_N_FMODE 0
#elif (RTE_UA2_CTS_N_ID == 1) && (RTE_GPIO_11_ID != 2)
  #define IOMUX_UA2_CTS_N_FMODE 1
#elif (RTE_UA2_CTS_N_ID != 1) && (RTE_GPIO_11_ID == 2)
  #define IOMUX_UA2_CTS_N_FMODE 3
#else
  #error Conflicted IO pin in IOMUX_UA2_CTS_N_FMODE
#endif

#if (RTE_SPIM0_CS_N_ID != 1) && (RTE_GPIO_16_ID != 2) && (RTE_PWM0_ID != 2)
  #define IOMUX_SPIM0_CS_N_FMODE 0
#elif (RTE_SPIM0_CS_N_ID == 1) && (RTE_GPIO_16_ID != 2) && (RTE_PWM0_ID != 2)
  #define IOMUX_SPIM0_CS_N_FMODE 1
#elif (RTE_SPIM0_CS_N_ID != 1) && (RTE_GPIO_16_ID == 2) && (RTE_PWM0_ID != 2)
  #define IOMUX_SPIM0_CS_N_FMODE 2
#elif (RTE_SPIM0_CS_N_ID != 1) && (RTE_GPIO_16_ID != 2) && (RTE_PWM0_ID == 2)
  #define IOMUX_SPIM0_CS_N_FMODE 3
#else
  #error Conflicted IO pin in IOMUX_SPIM0_CS_N_FMODE
#endif

#if (RTE_SPIM0_CLK_ID != 1) && (RTE_GPIO_17_ID != 2) && (RTE_PWM1_ID != 2)
  #define IOMUX_SPIM0_CLK_FMODE 0
#elif (RTE_SPIM0_CLK_ID == 1) && (RTE_GPIO_17_ID != 2) && (RTE_PWM1_ID != 2)
  #define IOMUX_SPIM0_CLK_FMODE 1
#elif (RTE_SPIM0_CLK_ID != 1) && (RTE_GPIO_17_ID == 2) && (RTE_PWM1_ID != 2)
  #define IOMUX_SPIM0_CLK_FMODE 2
#elif (RTE_SPIM0_CLK_ID != 1) && (RTE_GPIO_17_ID != 2) && (RTE_PWM1_ID == 2)
  #define IOMUX_SPIM0_CLK_FMODE 3
#else
  #error Conflicted IO pin in IOMUX_SPIM0_CLK_FMODE
#endif

#if (RTE_SPIM0_MOSI_ID != 1) && (RTE_GPIO_18_ID != 2) && (RTE_PWM2_ID != 2)
  #define IOMUX_SPIM0_MOSI_FMODE 0
#elif (RTE_SPIM0_MOSI_ID == 1) && (RTE_GPIO_18_ID != 2) && (RTE_PWM2_ID != 2)
  #define IOMUX_SPIM0_MOSI_FMODE 1
#elif (RTE_SPIM0_MOSI_ID != 1) && (RTE_GPIO_18_ID == 2) && (RTE_PWM2_ID != 2)
  #define IOMUX_SPIM0_MOSI_FMODE 2
#elif (RTE_SPIM0_MOSI_ID != 1) && (RTE_GPIO_18_ID != 2) && (RTE_PWM2_ID == 2)
  #define IOMUX_SPIM0_MOSI_FMODE 3
#else
  #error Conflicted IO pin in IOMUX_SPIM0_MOSI_FMODE
#endif

#if (RTE_SPIM0_MISO_ID != 1) && (RTE_GPIO_19_ID != 2) && (RTE_TRACECLK_ID != 1)
  #define IOMUX_SPIM0_MISO_FMODE 0
#elif (RTE_SPIM0_MISO_ID == 1) && (RTE_GPIO_19_ID != 2) && (RTE_TRACECLK_ID != 1)
  #define IOMUX_SPIM0_MISO_FMODE 1
#elif (RTE_SPIM0_MISO_ID != 1) && (RTE_GPIO_19_ID == 2) && (RTE_TRACECLK_ID != 1)
  #define IOMUX_SPIM0_MISO_FMODE 2
#elif (RTE_SPIM0_MISO_ID != 1) && (RTE_GPIO_19_ID != 2) && (RTE_TRACECLK_ID == 1)
  #define IOMUX_SPIM0_MISO_FMODE 3
#else
  #error Conflicted IO pin in IOMUX_SPIM0_MISO_FMODE
#endif

#if (RTE_SPIM1_CS_N_ID != 1) && (RTE_GPIO_20_ID != 2) && (RTE_UA0_RXD_ID != 2)
  #define IOMUX_SPIM1_CS_N_FMODE 0
#elif (RTE_SPIM1_CS_N_ID == 1) && (RTE_GPIO_20_ID != 2) && (RTE_UA0_RXD_ID != 2)
  #define IOMUX_SPIM1_CS_N_FMODE 1
#elif (RTE_SPIM1_CS_N_ID != 1) && (RTE_GPIO_20_ID == 2) && (RTE_UA0_RXD_ID != 2)
  #define IOMUX_SPIM1_CS_N_FMODE 2
#elif (RTE_SPIM1_CS_N_ID != 1) && (RTE_GPIO_20_ID != 2) && (RTE_UA0_RXD_ID == 2)
  #define IOMUX_SPIM1_CS_N_FMODE 3
#else
  #error Conflicted IO pin in IOMUX_SPIM1_CS_N_FMODE
#endif

#if (RTE_SPIM1_CLK_ID != 1) && (RTE_GPIO_21_ID != 2) && (RTE_UA0_TXD_ID != 2)
  #define IOMUX_SPIM1_CLK_FMODE 0
#elif (RTE_SPIM1_CLK_ID == 1) && (RTE_GPIO_21_ID != 2) && (RTE_UA0_TXD_ID != 2)
  #define IOMUX_SPIM1_CLK_FMODE 1
#elif (RTE_SPIM1_CLK_ID != 1) && (RTE_GPIO_21_ID == 2) && (RTE_UA0_TXD_ID != 2)
  #define IOMUX_SPIM1_CLK_FMODE 2
#elif (RTE_SPIM1_CLK_ID != 1) && (RTE_GPIO_21_ID != 2) && (RTE_UA0_TXD_ID == 2)
  #define IOMUX_SPIM1_CLK_FMODE 3
#else
  #error Conflicted IO pin in IOMUX_SPIM1_CLK_FMODE
#endif

#if (RTE_SPIM1_MOSI_ID != 1) && (RTE_GPIO_22_ID != 2)
  #define IOMUX_SPIM1_MOSI_FMODE 0
#elif (RTE_SPIM1_MOSI_ID == 1) && (RTE_GPIO_22_ID != 2)
  #define IOMUX_SPIM1_MOSI_FMODE 1
#elif (RTE_SPIM1_MOSI_ID != 1) && (RTE_GPIO_22_ID == 2)
  #define IOMUX_SPIM1_MOSI_FMODE 2
#else
  #error Conflicted IO pin in IOMUX_SPIM1_MOSI_FMODE
#endif

#if (RTE_SPIM1_MISO_ID != 1) && (RTE_GPIO_23_ID != 2)
  #define IOMUX_SPIM1_MISO_FMODE 0
#elif (RTE_SPIM1_MISO_ID == 1) && (RTE_GPIO_23_ID != 2)
  #define IOMUX_SPIM1_MISO_FMODE 1
#elif (RTE_SPIM1_MISO_ID != 1) && (RTE_GPIO_23_ID == 2)
  #define IOMUX_SPIM1_MISO_FMODE 2
#else
  #error Conflicted IO pin in IOMUX_SPIM1_MISO_FMODE
#endif

#if (RTE_SPIM2_CS_N_ID != 1) && (RTE_SPIM2_CLK_ID != 1) && (RTE_SPIM2_MOSI_ID != 1) && (RTE_SPIM2_MISO_ID != 1)
  #define IOMUX_SPIM2_FMODE 0
#elif (RTE_SPIM2_CS_N_ID == 1) && (RTE_SPIM2_CLK_ID == 1) && (RTE_SPIM2_MOSI_ID == 1) && (RTE_SPIM2_MISO_ID == 1)
  #define IOMUX_SPIM2_FMODE 1
#else
  #error Conflicted IO pin in IOMUX_SPIM2_FMODE
#endif

#if (RTE_SPIM3_CS_N_ID != 1) && (RTE_SPIM3_CLK_ID != 1) && (RTE_SPIM3_MOSI_ID != 1) && (RTE_SPIM3_MISO_ID != 1)
  #define IOMUX_SPIM3_FMODE 0
#elif (RTE_SPIM3_CS_N_ID == 1) && (RTE_SPIM3_CLK_ID == 1) && (RTE_SPIM3_MOSI_ID == 1) && (RTE_SPIM3_MISO_ID == 1)
  #define IOMUX_SPIM3_FMODE 1
#else
  #error Conflicted IO pin in IOMUX_SPIM3_FMODE
#endif

#if (RTE_SPIC_CS_N_ID == 1)
  #define IOMUX_SPIC_FMODE 0
#else
  #define IOMUX_SPIC_FMODE 1
#endif

#if (RTE_ADC24_SYNC_ID != 1) && (RTE_GPIO_8_ID != 2)
  #define IOMUX_ADC24_SYNC_FMODE 0
#elif (RTE_ADC24_SYNC_ID == 1) && (RTE_GPIO_8_ID != 2)
  #define IOMUX_ADC24_SYNC_FMODE 1
#elif (RTE_ADC24_SYNC_ID != 1) && (RTE_GPIO_8_ID == 2)
  #define IOMUX_ADC24_SYNC_FMODE 3
#else
  #error Conflicted IO pin in IOMUX_ADC24_SYNC_FMODE
#endif

#define PMU_WAKEUP_FACTOR_MASK         (PMU_WAKEUP_FACTOR_GPIO_0 | PMU_WAKEUP_FACTOR_GPIO_1 | PMU_WAKEUP_FACTOR_GPIO_2 | PMU_WAKEUP_FACTOR_GPIO_3 | PMU_WAKEUP_FACTOR_GPIO_4 | PMU_WAKEUP_FACTOR_GPIO_5 | PMU_WAKEUP_FACTOR_GPIO_6 | PMU_WAKEUP_FACTOR_GPIO_7 | PMU_WAKEUP_FACTOR_GPIO_24 | PMU_WAKEUP_FACTOR_GPIO_25 | PMU_WAKEUP_FACTOR_GPIO_26 | PMU_WAKEUP_FACTOR_GPIO_27 | PMU_WAKEUP_FACTOR_GPIO_30 | PMU_WAKEUP_FACTOR_BROWNOUT | PMU_WAKEUP_FACTOR_RTC | PMU_WAKEUP_FACTOR_DEBUG)

/* bitband alias */
#define BB_PMULV_WAKEUP_EN(_x)          BITBAND_VALUE(&pmulv->WAKEUP_EN,_x)
#define BB_PMULV_DCG_PMULV_LV0CLK()     BITBAND_VALUE(&pmulv->DCG_PM_0,31)
#define BB_PMULV_VOLTAGE_START()        BITBAND_VALUE(&pmulv->MOVE_VOLTAGE_START,0)
#define BB_PMULV_SPIC_ON_SPIFRSTN()     BITBAND_VALUE(&pmulv->SRST_ON_PF,3)
#define BB_PMULV_SPIC_OFF_SPIFRSTN()    BITBAND_VALUE(&pmulv->SRST_OFF_PF,3)
#define BB_PMULV_CG_ON_PA_32K()         BITBAND_VALUE(&pmulv->CG_ON_PA,8)
#define BB_PMULV_CG_OFF_PA_32K()        BITBAND_VALUE(&pmulv->CG_OFF_PA,8)
#define BB_PMULV_CG_OFF_REFCLK_PMULV()  BITBAND_VALUE(&pmulv->CG_OFF_REFCLK,0)
#define BB_PMULV_CG_OFF_REFCLK_ADPLL()  BITBAND_VALUE(&pmulv->CG_OFF_REFCLK,16)
#define BB_PMULV_CONFIG_ADPLL_EN()      BITBAND_VALUE(&pmulv->CONFIG_ADPLL_0,0)
#define BB_PMULV_CONFIG_ADPLL_NTWMODE() BITBAND_VALUE(&pmulv->CONFIG_ADPLL_1,0)
#define BB_PMULV_OSC12M_EN()            BITBAND_VALUE(&pmulv->CONFIG_OSC12M,0)
#define BB_PMULV_OSC32K_BOOST_DISABLE() BITBAND_VALUE(&pmulv->CONFIG_OSC32K,2)
#define BB_PMULV_OSC32K_EN()            BITBAND_VALUE(&pmulv->CONFIG_OSC32K,0)
#define BB_PMULV_SIOSC32K_EN()          BITBAND_VALUE(&pmulv->CONFIG_SiOSC32K,0)

static PMU_CSM Cd2Csm(PMU_CD clockDomain);
static PMU_CLOCK_SOURCE_STATE PMU_GetClockSourceState(PMU_CLOCK_SOURCE source);
static PMU_CLOCK_SOURCE_STATE PMU_GetClockSourceStateInternal(uint32_t source);
static PMU_STATUS PMU_SetPrescaler(PMU_CD clockDomain, uint32_t divider);
static uint32_t PMU_GetClockSourceSelector(PMU_CSM csm);
static uint32_t PMU_GetFrequency(PMU_CD clockDomain);
static PMU_STATUS PMU_SelectClockSourceInternal(PMU_CSM csm, uint32_t source);

/**
 * @brief Wait a while by busy loop
 * @param[in] usec Wait time in microseconds
 */
static void usleep(uint32_t usec)
{
  int32_t t, dt;
  if (SystemCoreClock >= 4000000) {
    t = ((SystemCoreClock + 999999) / 1000000) * usec;
    dt = 8;
  } else {
    t = usec * 100;
    dt = (8 * 100 * 1000000) / SystemCoreClock;
  }
  /* It assumes 8 cycles per loop */
  while (t > 0) {
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    t -= dt;
  }
}

/**
 * @brief Internally used in PMU_SetPowerDomainState
 * @param[in] powerDomain power domain
 * @return bit of POWERDOMAIN_CTRL_MODE register
 */
static uint32_t GetPowerDomainCtrlBit(PMU_PD powerDomain)
{
  uint32_t n;
  static const int8_t tbl[] = {
    -1,
    PMU_PD_ENCRYPT,
    PMU_PD_SRAM0,
    PMU_PD_SRAM1,
    PMU_PD_SRAM2,
    PMU_PD_FLASH,
    -1,
    PMU_PD_DMAC,
    -1,
    PMU_PD_ADCC12,
    PMU_PD_ADCC24,
    PMU_PD_PPIER1
  };
  
  for (n = 0; n < 12; n++) {
    if (powerDomain == tbl[n]) {
      break;
    }
  }
  if (n == 12) {
    /* error */
    return 0xffffffff;
  } else {
    return n << 1;
  }
}

static void _SystemCoreClockUpdate(void)
{
  SystemCoreClock = PMU_GetFrequency(PMU_CD_MPIER);
}

/**
 * @brief Initialize IO multiplex configuration according to RTE_Device.h.
 * @note gconf must be available before calling it.
 */
static void InitIomux(void)
{
  gconf->FMODE_CFG0 =
        (IOMUX_GPIO_0_FMODE)
      | (IOMUX_GPIO_1_FMODE << 2)
      | (IOMUX_GPIO_2_FMODE << 4)
      | (IOMUX_GPIO_3_FMODE << 6)
      | (IOMUX_GPIO_4_FMODE << 8)
      | (IOMUX_GPIO_5_FMODE << 10)
      | (IOMUX_GPIO_6_FMODE << 12)
      | (IOMUX_GPIO_7_FMODE << 14)
      | (IOMUX_GPIO_8_FMODE << 16)
      | (IOMUX_GPIO_9_FMODE << 18)
      | (IOMUX_GPIO_10_FMODE << 20)
      | (IOMUX_GPIO_11_FMODE << 22)
      | (IOMUX_GPIO_12_FMODE << 24)
      | (IOMUX_GPIO_13_FMODE << 26)
      | (IOMUX_GPIO_14_FMODE << 28)
      | ((uint32_t)IOMUX_GPIO_15_FMODE << 30);
  
  gconf->FMODE_CFG1 =
        (IOMUX_GPIO_24_FMODE << 16)
      | (IOMUX_GPIO_25_FMODE << 18)
      | (IOMUX_GPIO_26_FMODE << 20)
      | (IOMUX_GPIO_27_FMODE << 22)
      | (IOMUX_GPIO_28_FMODE << 24)
      | (IOMUX_GPIO_29_FMODE << 26)
      | (IOMUX_GPIO_30_FMODE << 28)
      | ((uint32_t)IOMUX_GPIO_31_FMODE << 30);

  gconf->FMODE_CFG2 =
        (IOMUX_I2C0_DATA_FMODE)
      | (IOMUX_I2C0_CLK_FMODE  << 2)
      | (IOMUX_I2C1_DATA_FMODE << 4)
      | (IOMUX_I2C1_CLK_FMODE  << 6)
      | (IOMUX_I2C2_FMODE      << 8);

  gconf->FMODE_CFG3 =
        (IOMUX_UA0_RXD_FMODE)
      | (IOMUX_UA0_TXD_FMODE   << 2)
      | (IOMUX_UA1_RXD_FMODE   << 8)
      | (IOMUX_UA1_TXD_FMODE   << 10)
      | (IOMUX_UA1_RTS_N_FMODE << 12)
      | (IOMUX_UA1_CTS_N_FMODE << 14)
      | (IOMUX_UA2_RXD_FMODE   << 16)
      | (IOMUX_UA2_TXD_FMODE   << 18)
      | (IOMUX_UA2_RTS_N_FMODE << 20)
      | (IOMUX_UA2_CTS_N_FMODE << 22);

  gconf->FMODE_CFG4 =
        (IOMUX_SPIM0_CS_N_FMODE)
      | (IOMUX_SPIM0_CLK_FMODE  << 2)
      | (IOMUX_SPIM0_MOSI_FMODE << 4)
      | (IOMUX_SPIM0_MISO_FMODE << 6)
      | (IOMUX_SPIM1_CS_N_FMODE << 8)
      | (IOMUX_SPIM1_CLK_FMODE  << 10)
      | (IOMUX_SPIM1_MOSI_FMODE << 12)
      | (IOMUX_SPIM1_MISO_FMODE << 14)
      | (IOMUX_SPIM2_FMODE      << 16)
      | (IOMUX_SPIM3_FMODE      << 24);

  gconf->FMODE_CFG5 = IOMUX_SPIC_FMODE;
  
  gconf->FMODE_CFG6 = IOMUX_ADC24_SYNC_FMODE;
}

#define TABLE_FUNC2PIN_ROW(index, data1, data2) \
(((index) == 1) ? (data1) : (((index) == 2) ? (data2) : PMU_IO_PIN_INVALID))

/**
 * @brief Convert PMU_IO_FUNC -> PMU_IO_PIN
 * @param[in] func PMU_IO_FUNC
 * @return PMU_IO_PIN of PMU_IO_PIN_INVALID when a pin is not assigned for func
 */
static PMU_IO_PIN Func2Pin(PMU_IO_FUNC func)
{
  static const uint8_t tbl[] = {
    TABLE_FUNC2PIN_ROW(RTE_GPIO_0_ID,     PMU_IO_PIN_GPIO_0,     PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_1_ID,     PMU_IO_PIN_GPIO_1,     PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_2_ID,     PMU_IO_PIN_GPIO_2,     PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_3_ID,     PMU_IO_PIN_GPIO_3,     PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_4_ID,     PMU_IO_PIN_GPIO_4,     PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_5_ID,     PMU_IO_PIN_GPIO_5,     PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_6_ID,     PMU_IO_PIN_GPIO_6,     PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_7_ID,     PMU_IO_PIN_GPIO_7,     PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_8_ID,     PMU_IO_PIN_GPIO_8,     PMU_IO_PIN_ADC24_SYNC),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_9_ID,     PMU_IO_PIN_GPIO_9,     PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_10_ID,    PMU_IO_PIN_GPIO_10,    PMU_IO_PIN_UA2_RTS_N),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_11_ID,    PMU_IO_PIN_GPIO_11,    PMU_IO_PIN_UA2_CTS_N),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_12_ID,    PMU_IO_PIN_GPIO_12,    PMU_IO_PIN_UA1_RXD),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_13_ID,    PMU_IO_PIN_GPIO_13,    PMU_IO_PIN_UA1_TXD),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_14_ID,    PMU_IO_PIN_GPIO_14,    PMU_IO_PIN_I2C1_DATA),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_15_ID,    PMU_IO_PIN_GPIO_15,    PMU_IO_PIN_I2C1_CLK),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_16_ID,    PMU_IO_PIN_I2C0_DATA,  PMU_IO_PIN_SPIM0_CS_N),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_17_ID,    PMU_IO_PIN_I2C0_CLK,   PMU_IO_PIN_SPIM0_CLK),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_18_ID,    PMU_IO_PIN_UA2_RXD,    PMU_IO_PIN_SPIM0_MOSI),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_19_ID,    PMU_IO_PIN_UA2_TXD,    PMU_IO_PIN_SPIM0_MISO),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_20_ID,    PMU_IO_PIN_UA0_RXD,    PMU_IO_PIN_SPIM1_CS_N),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_21_ID,    PMU_IO_PIN_UA0_TXD,    PMU_IO_PIN_SPIM1_CLK),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_22_ID,    PMU_IO_PIN_UA1_RTS_N,  PMU_IO_PIN_SPIM1_MOSI),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_23_ID,    PMU_IO_PIN_UA1_CTS_N,  PMU_IO_PIN_SPIM1_MISO),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_24_ID,    PMU_IO_PIN_GPIO_24,    PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_25_ID,    PMU_IO_PIN_GPIO_25,    PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_26_ID,    PMU_IO_PIN_GPIO_26,    PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_27_ID,    PMU_IO_PIN_GPIO_27,    PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_28_ID,    PMU_IO_PIN_GPIO_28,    PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_29_ID,    PMU_IO_PIN_GPIO_29,    PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_30_ID,    PMU_IO_PIN_GPIO_30,    PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_GPIO_31_ID,    PMU_IO_PIN_GPIO_31,    PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_I2C0_DATA_ID,  PMU_IO_PIN_I2C0_DATA,  PMU_IO_PIN_UA1_TXD),
    TABLE_FUNC2PIN_ROW(RTE_I2C0_CLK_ID,   PMU_IO_PIN_I2C0_CLK,   PMU_IO_PIN_UA1_RXD),
    TABLE_FUNC2PIN_ROW(RTE_I2C1_DATA_ID,  PMU_IO_PIN_I2C1_DATA,  PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_I2C1_CLK_ID,   PMU_IO_PIN_I2C1_CLK,   PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_I2C2_DATA_ID,  PMU_IO_PIN_I2C2_DATA,  PMU_IO_PIN_UA1_RTS_N),
    TABLE_FUNC2PIN_ROW(RTE_I2C2_CLK_ID,   PMU_IO_PIN_I2C2_CLK,   PMU_IO_PIN_UA1_CTS_N),
    TABLE_FUNC2PIN_ROW(RTE_UA0_RXD_ID,    PMU_IO_PIN_UA0_RXD,    PMU_IO_PIN_SPIM1_CS_N),
    TABLE_FUNC2PIN_ROW(RTE_UA0_TXD_ID,    PMU_IO_PIN_UA0_TXD,    PMU_IO_PIN_SPIM1_CLK),
    TABLE_FUNC2PIN_ROW(RTE_UA0_RTS_N_ID,  PMU_IO_PIN_GPIO_14,    PMU_IO_PIN_I2C1_DATA),
    TABLE_FUNC2PIN_ROW(RTE_UA0_CTS_N_ID,  PMU_IO_PIN_GPIO_15,    PMU_IO_PIN_I2C1_CLK),
    TABLE_FUNC2PIN_ROW(RTE_UA1_RXD_ID,    PMU_IO_PIN_UA1_RXD,    PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_UA1_TXD_ID,    PMU_IO_PIN_UA1_TXD,    PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_UA1_RTS_N_ID,  PMU_IO_PIN_UA1_RTS_N,  PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_UA1_CTS_N_ID,  PMU_IO_PIN_UA1_CTS_N,  PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_UA2_RXD_ID,    PMU_IO_PIN_UA2_RXD,    PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_UA2_TXD_ID,    PMU_IO_PIN_UA2_TXD,    PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_UA2_RTS_N_ID,  PMU_IO_PIN_UA2_RTS_N,  PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_UA2_CTS_N_ID,  PMU_IO_PIN_UA2_CTS_N,  PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_SPIM0_CS_N_ID, PMU_IO_PIN_SPIM0_CS_N, PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_SPIM0_CLK_ID,  PMU_IO_PIN_SPIM0_CLK,  PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_SPIM0_MOSI_ID, PMU_IO_PIN_SPIM0_MOSI, PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_SPIM0_MISO_ID, PMU_IO_PIN_SPIM0_MISO, PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_SPIM1_CS_N_ID, PMU_IO_PIN_SPIM1_CS_N, PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_SPIM1_CLK_ID,  PMU_IO_PIN_SPIM1_CLK,  PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_SPIM1_MOSI_ID, PMU_IO_PIN_SPIM1_MOSI, PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_SPIM1_MISO_ID, PMU_IO_PIN_SPIM1_MISO, PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_SPIM2_CS_N_ID, PMU_IO_PIN_SPIM2_CS_N, PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_SPIM2_CLK_ID,  PMU_IO_PIN_SPIM2_CLK,  PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_SPIM2_MOSI_ID, PMU_IO_PIN_SPIM2_MOSI, PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_SPIM2_MISO_ID, PMU_IO_PIN_SPIM2_MISO, PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_SPIM3_CS_N_ID, PMU_IO_PIN_SPIM3_CS_N, PMU_IO_PIN_GPIO_12),
    TABLE_FUNC2PIN_ROW(RTE_SPIM3_CLK_ID,  PMU_IO_PIN_SPIM3_CLK,  PMU_IO_PIN_GPIO_13),
    TABLE_FUNC2PIN_ROW(RTE_SPIM3_MOSI_ID, PMU_IO_PIN_SPIM3_MOSI, PMU_IO_PIN_GPIO_14),
    TABLE_FUNC2PIN_ROW(RTE_SPIM3_MISO_ID, PMU_IO_PIN_SPIM3_MISO, PMU_IO_PIN_GPIO_15),
    TABLE_FUNC2PIN_ROW(RTE_PWM0_ID,       PMU_IO_PIN_GPIO_8,     PMU_IO_PIN_SPIM0_CS_N),
    TABLE_FUNC2PIN_ROW(RTE_PWM1_ID,       PMU_IO_PIN_GPIO_9,     PMU_IO_PIN_SPIM0_CLK),
    TABLE_FUNC2PIN_ROW(RTE_PWM2_ID,       PMU_IO_PIN_GPIO_10,    PMU_IO_PIN_SPIM0_MOSI),
    TABLE_FUNC2PIN_ROW(RTE_PWM3_ID,       PMU_IO_PIN_GPIO_11,    PMU_IO_PIN_GPIO_27),
    TABLE_FUNC2PIN_ROW(RTE_CAPTURE0_ID,   PMU_IO_PIN_GPIO_12,    PMU_IO_PIN_UA0_RXD),
    TABLE_FUNC2PIN_ROW(RTE_CAPTURE1_ID,   PMU_IO_PIN_GPIO_13,    PMU_IO_PIN_UA0_TXD),
    TABLE_FUNC2PIN_ROW(RTE_CAPTURE2_ID,   PMU_IO_PIN_I2C0_DATA,  PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_CAPTURE3_ID,   PMU_IO_PIN_I2C0_CLK,   PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_SPIC_CS_N_ID,  PMU_IO_PIN_SPIC_CS_N,  PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_SPIC_CLK_ID,   PMU_IO_PIN_SPIC_CLK,   PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_SPIC_MOSI_ID,  PMU_IO_PIN_SPIC_MOSI,  PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_SPIC_MISO_ID,  PMU_IO_PIN_SPIC_MISO,  PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_SPIC_IO2_ID,   PMU_IO_PIN_SPIC_IO2,   PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_SPIC_IO3_ID,   PMU_IO_PIN_SPIC_IO3,   PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_TRACECLK_ID,   PMU_IO_PIN_SPIM0_MISO, PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_TRACEDATA0_ID, PMU_IO_PIN_GPIO_8,     PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_TRACEDATA1_ID, PMU_IO_PIN_GPIO_9,     PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_TRACEDATA2_ID, PMU_IO_PIN_GPIO_10,    PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_TRACEDATA3_ID, PMU_IO_PIN_GPIO_11,    PMU_IO_PIN_INVALID),
    TABLE_FUNC2PIN_ROW(RTE_ADC24_SYNC_ID, PMU_IO_PIN_ADC24_SYNC, PMU_IO_PIN_INVALID),
    PMU_IO_PIN_DBG,
    PMU_IO_PIN_DBG,
    PMU_IO_PIN_DBG,
    PMU_IO_PIN_DBG,
    PMU_IO_PIN_DBG,
  };
  
  if ((uint32_t)func >= PMU_IO_FUNC_INVALID) {
    return PMU_IO_PIN_INVALID;
  } else {
    return (PMU_IO_PIN)tbl[func];
  }
}

/**
 * @brief Negate bus reset
 */
static void PMU_BusResetOff(void)
{
  uint8_t *en = PMU_Resources.info->module_enabled;
  
  if (en[PMU_MODULE_USB2FS]) {
    /* USB - AHB Bus Matrix Bridge */
    pmulv->CG_OFF_PU_b.CG_mpierclk_h2hdnu_hclk = 1;
    pmulv->CG_OFF_PU_b.CG_mpierclk_h2hupu_hclk = 1;
    pmulv->SRST_OFF_PU_b.SRST_asyncrst_h2hdnu_hrstn = 1;
    pmulv->SRST_OFF_PU_b.SRST_asyncrst_h2hupu_hrstn = 1;
  }
  
  if (en[PMU_MODULE_AESA] || en[PMU_MODULE_RNG]
      || en[PMU_MODULE_EVM] || en[PMU_MODULE_SRAMC]) {
    /* AESA, RNG, EVM, SRAMC - AHB Bus Matrix Bridge */
    pmulv->CG_OFF_PM_0_b.CG_mpierclk_h2pm_hclk = 1;
    pmulv->SRST_OFF_PM_0_b.SRST_asyncrst_h2pm_hrstn = 1;
  }
  
  if (en[PMU_MODULE_I2C2]
      || en[PMU_MODULE_SPIM2]  || en[PMU_MODULE_SPIM3]
      || en[PMU_MODULE_WDT]    || en[PMU_MODULE_ADVTMR]
      || en[PMU_MODULE_TMR]    || en[PMU_MODULE_GCONF]
      || en[PMU_MODULE_GPIO0]  || en[PMU_MODULE_GPIO1]
      || en[PMU_MODULE_GPIO2]  || en[PMU_MODULE_GPIO3]
      || en[PMU_MODULE_ADCC12] || en[PMU_MODULE_ADCC24]) {
    /* APB0 Bus */
    pmulv->CG_OFF_PM_1_b.CG_mpierclk_h2hp0_hclk = 1;
    pmulv->CG_OFF_PM_1_b.CG_ppier0clk_h2pp0_hclk = 1;
    pmulv->SRST_OFF_PM_1_b.SRST_asyncrst_h2hp0_hrstn = 1;
    pmulv->SRST_OFF_PM_1_b.SRST_asyncrst_h2pp0_hrstn = 1;
  }
  
  if (en[PMU_MODULE_UART0] || en[PMU_MODULE_UART1]
      || en[PMU_MODULE_I2C0] || en[PMU_MODULE_I2C1]
      || en[PMU_MODULE_SPIM0] || en[PMU_MODULE_SPIM1]) {
    /* APB1 Bus */
    pmulv->CG_OFF_PM_2_b.CG_mpierclk_h2hp1_hclk = 1;
    pmulv->CG_OFF_PP1_b.CG_ppier1clk_h2pp1_hclk = 1;
    pmulv->SRST_OFF_PM_2_b.SRST_asyncrst_h2hp1_hrstn = 1;
    pmulv->SRST_OFF_PP1_b.SRST_asyncrst_h2pp1_hrstn = 1;
  }
  
  if (en[PMU_MODULE_UART2]) {
    /* APB2 Bus */
    pmulv->CG_OFF_PM_2_b.CG_mpierclk_h2hp2_hclk = 1;
    pmulv->CG_OFF_PM_2_b.CG_ppier2clk_h2pp2_hclk = 1;
    pmulv->SRST_OFF_PM_2_b.SRST_asyncrst_h2hp2_hrstn = 1;
    pmulv->SRST_OFF_PM_2_b.SRST_asyncrst_h2pp2_hrstn = 1;
  }
}

/**
 * @brief Assert bus reset
 */
static void PMU_BusResetOn(void)
{
  uint8_t *en = PMU_Resources.info->module_enabled;
  
  if (0) {
    /* USB - AHB Bus Matrix Bridge */
    pmulv->SRST_ON_PU_b.SRST_asyncrst_h2hupu_hrstn = 1;
    pmulv->SRST_ON_PU_b.SRST_asyncrst_h2hdnu_hrstn = 1;
    pmulv->CG_ON_PU_b.CG_mpierclk_h2hupu_hclk = 1;
    pmulv->CG_ON_PU_b.CG_mpierclk_h2hdnu_hclk = 1;
  }
  
  if (!(en[PMU_MODULE_AESA] || en[PMU_MODULE_RNG]
        || en[PMU_MODULE_EVM] || en[PMU_MODULE_SRAMC])) {
    /* AESA, RNG, EVM, SRAMC - AHB Bus Matrix Bridge */
    pmulv->SRST_ON_PM_0_b.SRST_asyncrst_h2pm_hrstn = 1;
    pmulv->CG_ON_PM_0_b.CG_mpierclk_h2pm_hclk = 1;
  }
  
  if (!(en[PMU_MODULE_I2C2]
        || en[PMU_MODULE_SPIM2]  || en[PMU_MODULE_SPIM3]
        || en[PMU_MODULE_WDT]    || en[PMU_MODULE_ADVTMR]
        || en[PMU_MODULE_TMR]    || en[PMU_MODULE_GCONF]
        || en[PMU_MODULE_GPIO0]  || en[PMU_MODULE_GPIO1]
        || en[PMU_MODULE_GPIO2]  || en[PMU_MODULE_GPIO3]
        || en[PMU_MODULE_ADCC12] || en[PMU_MODULE_ADCC24])) {
    /* APB0 Bus */
    pmulv->SRST_ON_PM_1_b.SRST_asyncrst_h2pp0_hrstn = 1;
    pmulv->SRST_ON_PM_1_b.SRST_asyncrst_h2hp0_hrstn = 1;
    pmulv->CG_ON_PM_1_b.CG_ppier0clk_h2pp0_hclk = 1;
    pmulv->CG_ON_PM_1_b.CG_mpierclk_h2hp0_hclk = 1;
  }
  
  if (!(en[PMU_MODULE_UART0] || en[PMU_MODULE_UART1]
        || en[PMU_MODULE_I2C0] || en[PMU_MODULE_I2C1]
        || en[PMU_MODULE_SPIM0] || en[PMU_MODULE_SPIM1])) {
    /* APB1 Bus */
    pmulv->SRST_ON_PP1_b.SRST_asyncrst_h2pp1_hrstn = 1;
    pmulv->SRST_ON_PM_2_b.SRST_asyncrst_h2hp1_hrstn = 1;
    pmulv->CG_ON_PP1_b.CG_ppier1clk_h2pp1_hclk = 1;
    pmulv->CG_ON_PM_2_b.CG_mpierclk_h2hp1_hclk = 1;
  }
  
  if (!en[PMU_MODULE_UART2]) {
    /* APB2 Bus */
    pmulv->SRST_ON_PM_2_b.SRST_asyncrst_h2pp2_hrstn = 1;
    pmulv->SRST_ON_PM_2_b.SRST_asyncrst_h2hp2_hrstn = 1;
    pmulv->CG_ON_PM_2_b.CG_ppier2clk_h2pp2_hclk = 1;
    pmulv->CG_ON_PM_2_b.CG_mpierclk_h2hp2_hclk = 1;
  }
}

/**
 * @brief Negate module reset
 * @param[in] module kind of module
 * @return \ref PMU_STATUS
 */
static PMU_STATUS PMU_EnableModuleOn(PMU_MODULE module)
{
  if ((uint32_t)module > PMU_MODULE_USB2FS) {
    return PMU_ERROR;
  }
  PMU_Resources.info->module_enabled[module] = 1;
  PMU_BusResetOff();

  switch (module) {
  case PMU_MODULE_MPIER:
    /* DO NOTHING */
    break;
  case PMU_MODULE_SDMAC:
    pmulv->CG_OFF_POWERDOMAIN = 0x81;
    pmulv->CG_OFF_PD = 0x1;
    pmulv->SRST_OFF_POWERDOMAIN = 0x81;
    pmulv->SRST_OFF_PD = 0x1;
    break;
  case PMU_MODULE_AESA:
#ifdef PMU_TZ10XX_AES_H
    PMU_ENABLE_MODULE_AES();
#endif
    break;
  case PMU_MODULE_RNG:
    pmulv->CG_OFF_PE_b.CG_mpierclk_rng_coreclk = 1;
    pmulv->CG_OFF_PE_b.CG_mpierclk_rng_busclk = 1;
    pmulv->SRST_OFF_PE_b.SRST_asyncrst_rng_rstn = 1;
    break;
  case PMU_MODULE_SRAMC:
    pmulv->CG_OFF_PM_0_b.CG_mpierclk_sramc_pclk = 1;
    pmulv->CG_OFF_PM_0_b.CG_mpierclk_sramc_s2hclk = 1;
    pmulv->CG_OFF_PM_0_b.CG_mpierclk_sramc_s1hclk = 1;
    pmulv->CG_OFF_PM_0_b.CG_mpierclk_sramc_s0hclk = 1;
    pmulv->CG_OFF_PM_0_b.CG_mpierclk_sramc_hclk = 1;
    pmulv->SRST_OFF_PM_0_b.SRST_asyncrst_sramc_hrstn = 1;
    pmulv->SRST_OFF_PM_0_b.SRST_asyncrst_sramc_prstn = 1;
    break;
  case PMU_MODULE_SPIC:
    pmulv->CG_OFF_PF = 0x7;
    pmulv->SRST_OFF_PF = 0xF;
    break;
  case PMU_MODULE_UART0:
    pmulv->CG_OFF_POWERDOMAIN = 0x801;
    pmulv->CG_OFF_PM_2 = 0x10000;
    pmulv->CG_OFF_PP1 = 0x3001;
    pmulv->SRST_OFF_POWERDOMAIN = 0x801;
    pmulv->SRST_OFF_PM_2 = 0x10000;
    pmulv->SRST_OFF_PP1 = 0x3001;
    gconf->OE_CTRL_b.UART0_OE = 1;
    break;
  case PMU_MODULE_UART1:
    pmulv->CG_OFF_POWERDOMAIN = 0x801;
    pmulv->CG_OFF_PM_2 = 0x10000;
    pmulv->CG_OFF_PP1 = 0xc001;
    pmulv->SRST_OFF_POWERDOMAIN = 0x801;
    pmulv->SRST_OFF_PM_2 = 0x10000;
    pmulv->SRST_OFF_PP1 = 0xc001;
    gconf->OE_CTRL_b.UART1_OE = 1;
    break;
  case PMU_MODULE_UART2:
    pmulv->CG_OFF_POWERDOMAIN = 0x1;
    pmulv->CG_OFF_PM_1 = 0xc0000003;
    pmulv->SRST_OFF_POWERDOMAIN = 0x1;
    pmulv->SRST_OFF_PM_1 = 0xc0000003;
    gconf->OE_CTRL_b.UART2_OE = 1;
    break;
  case PMU_MODULE_I2C0:
    pmulv->CG_OFF_POWERDOMAIN = 0x801;
    pmulv->CG_OFF_PM_2 = 0x10000;
    pmulv->CG_OFF_PP1 = 0x31;
    pmulv->SRST_OFF_POWERDOMAIN = 0x801;
    pmulv->SRST_OFF_PM_2 = 0x10000;
    pmulv->SRST_OFF_PP1 = 0x31;
    break;
  case PMU_MODULE_I2C1:
    pmulv->CG_OFF_POWERDOMAIN = 0x801;
    pmulv->CG_OFF_PM_2 = 0x10000;
    pmulv->CG_OFF_PP1 = 0xc1;
    pmulv->SRST_OFF_POWERDOMAIN = 0x801;
    pmulv->SRST_OFF_PM_2 = 0x10000;
    pmulv->SRST_OFF_PP1 = 0xc1;
    break;
  case PMU_MODULE_I2C2:
    pmulv->CG_OFF_POWERDOMAIN = 0x801;
    pmulv->CG_OFF_PM_1 = 0x3000003;
    pmulv->SRST_OFF_POWERDOMAIN = 0x801;
    pmulv->SRST_OFF_PM_1 = 0x3000003;
    break;
  case PMU_MODULE_SPIM0:
    pmulv->CG_OFF_POWERDOMAIN = 0x801;
    pmulv->CG_OFF_PM_2 = 0x10000;
    pmulv->CG_OFF_PP1 = 0x301;
    pmulv->SRST_OFF_POWERDOMAIN = 0x801;
    pmulv->SRST_OFF_PM_2 = 0x10000;
    pmulv->SRST_OFF_PP1 = 0x301;
    gconf->OE_CTRL_b.SPIM0_OE = 1;
    break;
  case PMU_MODULE_SPIM1:
    pmulv->CG_OFF_POWERDOMAIN = 0x801;
    pmulv->CG_OFF_PM_2 = 0x10000;
    pmulv->CG_OFF_PP1 = 0xc01;
    pmulv->SRST_OFF_POWERDOMAIN = 0x801;
    pmulv->SRST_OFF_PM_2 = 0x10000;
    pmulv->SRST_OFF_PP1 = 0xc01;
    gconf->OE_CTRL_b.SPIM1_OE = 1;
    break;
  case PMU_MODULE_SPIM2:
    pmulv->CG_OFF_POWERDOMAIN = 0x1;
    pmulv->CG_OFF_PM_1 = 0xc000003;
    pmulv->SRST_OFF_POWERDOMAIN = 0x1;
    pmulv->SRST_OFF_PM_1 = 0xc000003;
    gconf->OE_CTRL_b.SPIM2_OE = 1;
    break;
  case PMU_MODULE_SPIM3:
    pmulv->CG_OFF_POWERDOMAIN = 0x1;
    pmulv->CG_OFF_PM_1 = 0x30000003;
    pmulv->SRST_OFF_POWERDOMAIN = 0x1;
    pmulv->SRST_OFF_PM_1 = 0x30000003;
    gconf->OE_CTRL_b.SPIM3_OE = 1;
    break;
  case PMU_MODULE_WDT:
    pmulv->CG_OFF_POWERDOMAIN = 0x1;
    pmulv->CG_OFF_PM_1 = 0x13;
    pmulv->CG_OFF_PM_1 = 0x20;
    pmulv->SRST_OFF_POWERDOMAIN = 0x1;
    pmulv->SRST_OFF_PM_1 = 0x13;
    break;
  case PMU_MODULE_ADVTMR:
    pmulv->CG_OFF_POWERDOMAIN = 0x1;
    pmulv->CG_OFF_PM_1 = 0x83;
    pmulv->CG_OFF_PM_1_b.CG_ppier0clk_advtmr_ch0_timclk = 1;
    pmulv->CG_OFF_PM_1_b.CG_ppier0clk_advtmr_ch1_timclk = 1;
    pmulv->CG_OFF_PM_1_b.CG_ppier0clk_advtmr_ch2_timclk = 1;
    pmulv->CG_OFF_PM_1_b.CG_ppier0clk_advtmr_ch3_timclk = 1;
    pmulv->SRST_OFF_POWERDOMAIN = 0x1;
    pmulv->SRST_OFF_PM_1 = 0x83;
    break;
  case PMU_MODULE_TMR:
    pmulv->CG_OFF_POWERDOMAIN = 0x1;
    pmulv->CG_OFF_PM_1 = 0x80003;
    pmulv->CG_OFF_PM_1_b.CG_ppier0clk_tmr_ch0_timclk = 1;
    pmulv->CG_OFF_PM_1_b.CG_ppier0clk_tmr_ch1_timclk = 1;
    pmulv->SRST_OFF_POWERDOMAIN = 0x1;
    pmulv->SRST_OFF_PM_1 = 0x80003;
    break;
  case PMU_MODULE_GCONF:
    pmulv->CG_OFF_PM_2_b.CG_ppier0clk_gconf_pclk = 1;
    pmulv->SRST_OFF_PM_2_b.SRST_asyncrst_gconf_prstn = 1;
    break;
  case PMU_MODULE_GPIO0:
    pmulv->CG_OFF_POWERDOMAIN = 0x1;
    pmulv->CG_OFF_PM_2 = 0x10;
    pmulv->SRST_OFF_POWERDOMAIN = 0x1;
    pmulv->SRST_OFF_PM_2 = 0x10;
    break;
  case PMU_MODULE_GPIO1:
    pmulv->CG_OFF_POWERDOMAIN = 0x1;
    pmulv->CG_OFF_PM_2 = 0x20;
    pmulv->SRST_OFF_POWERDOMAIN = 0x1;
    pmulv->SRST_OFF_PM_2 = 0x20;
    break;
  case PMU_MODULE_GPIO2:
    pmulv->CG_OFF_POWERDOMAIN = 0x1;
    pmulv->CG_OFF_PM_2 = 0x40;
    pmulv->SRST_OFF_POWERDOMAIN = 0x1;
    pmulv->SRST_OFF_PM_2 = 0x40;
    break;
  case PMU_MODULE_GPIO3:
    pmulv->CG_OFF_POWERDOMAIN = 0x1;
    pmulv->CG_OFF_PM_2 = 0x80;
    pmulv->SRST_OFF_POWERDOMAIN = 0x1;
    pmulv->SRST_OFF_PM_2 = 0x80;
    break;
  case PMU_MODULE_ADCC12:
    pmulv->CG_OFF_POWERDOMAIN_b.CG_PA12 = 1;
    pmulv->SRST_OFF_POWERDOMAIN_b.SRST_PA12 = 1;
    break;
  case PMU_MODULE_ADCC24:
    pmulv->CG_OFF_POWERDOMAIN_b.CG_PA24 = 1;
    pmulv->SRST_OFF_POWERDOMAIN_b.SRST_PA24 = 1;
    gconf->OE_CTRL_b.ADC24_OE = 1;
    break;
  case PMU_MODULE_RTCLV:
    pmulv->CG_OFF_POWERDOMAIN_b.CG_PM = 1;
    pmulv->SRST_OFF_POWERDOMAIN_b.SRST_PM = 1;
    pmulv->CG_OFF_PA_b.CG_rtcclk_rtchv_rtcclk = 1;
    pmulv->SRST_OFF_PA_b.SRST_asyncrst_rtchv_rtcrstn = 1;
    pmulv->CG_OFF_PM_1_b.CG_ppier0clk_rtclv_pclk = 1;
    // tentative
    // pmulv->CG_OFF_PM_1_b.CG_rtcclk_rtclv_busclk = 1;
    pmulv->SRST_OFF_PM_1_b.SRST_asyncrst_rtclv_prstn = 1;
    pmulv->SRST_OFF_PM_1_b.SRST_asyncrst_rtclv_busrstn = 1;
    break;
  case PMU_MODULE_EVM:
    pmulv->CG_OFF_PM_0_b.CG_mpierclk_evm_pclk = 1;
    pmulv->SRST_OFF_PM_0_b.SRST_asyncrst_evm_prstn = 1;
    break;
  case PMU_MODULE_USB2FS:
    pmulv->CG_OFF_POWERDOMAIN_b.CG_PU = 1;
    pmulv->CG_OFF_PU_b.CG_mpierclk_h2hdnu_hclk = 1;
    pmulv->CG_OFF_PU_b.CG_mpierclk_h2hupu_hclk = 1;
    pmulv->CG_OFF_PU_b.CG_usbbclk_usb2fs_hclk = 1;
    pmulv->CG_OFF_PU_b.CG_usbiclk_usb2fs_usbclk = 1;
    pmulv->SRST_OFF_PU_b.SRST_asyncrst_usb2fs_usbrstn = 1;
    usleep(1); /* wait 12 cycles */
    pmulv->SRST_OFF_PU_b.SRST_asyncrst_h2hdnu_hrstn = 1;
    pmulv->SRST_OFF_PU_b.SRST_asyncrst_h2hupu_hrstn = 1;
    pmulv->SRST_OFF_PU_b.SRST_asyncrst_usb2fs_hrstn = 1;
    usleep(1); /* wait 6 cycles */
    break;
  default:
    return PMU_ERROR;
  }
  
  return PMU_OK;
}

/**
 * @brief Assert module reset
 * @param[in] module kind of module
 * @return \ref PMU_STATUS
 */
static PMU_STATUS PMU_EnableModuleOff(PMU_MODULE module)
{
  switch (module) {
  case PMU_MODULE_MPIER:
    /* cannot stop mpier */
    return PMU_ERROR;
  case PMU_MODULE_SDMAC:
    pmulv->SRST_ON_PD = 0x1;
    pmulv->CG_ON_PD = 0x1;
    break;
  case PMU_MODULE_AESA:
#ifdef PMU_TZ10XX_AES_H
    PMU_DISABLE_MODULE_AES();
#endif
    break;
  case PMU_MODULE_RNG:
    pmulv->SRST_ON_PE_b.SRST_asyncrst_rng_rstn = 1;
    pmulv->CG_ON_PE_b.CG_mpierclk_rng_coreclk = 1;
    pmulv->CG_ON_PE_b.CG_mpierclk_rng_busclk = 1;
    break;
  case PMU_MODULE_SRAMC:
    pmulv->SRST_ON_PM_0_b.SRST_asyncrst_sramc_hrstn = 1;
    pmulv->SRST_ON_PM_0_b.SRST_asyncrst_sramc_prstn = 1;
    pmulv->CG_ON_PM_0_b.CG_mpierclk_sramc_pclk = 1;
    pmulv->CG_ON_PM_0_b.CG_mpierclk_sramc_s2hclk = 1;
    pmulv->CG_ON_PM_0_b.CG_mpierclk_sramc_s1hclk = 1;
    pmulv->CG_ON_PM_0_b.CG_mpierclk_sramc_s0hclk = 1;
    pmulv->CG_ON_PM_0_b.CG_mpierclk_sramc_hclk = 1;
    break;
  case PMU_MODULE_SPIC:
    pmulv->SRST_ON_PF = 0xF;
    pmulv->CG_ON_PF = 0x7;
    break;
  case PMU_MODULE_UART0:
    pmulv->SRST_ON_PP1 = 0x3000;
    pmulv->CG_ON_PP1 = 0x3000;
    gconf->OE_CTRL_b.UART0_OE = 0;
    break;
  case PMU_MODULE_UART1:
    pmulv->SRST_ON_PP1 = 0xc000;
    pmulv->CG_ON_PP1 = 0xc000;
    gconf->OE_CTRL_b.UART1_OE = 0;
    break;
  case PMU_MODULE_UART2:
    pmulv->SRST_ON_PM_2 = 0xc00;
    pmulv->CG_ON_PM_2 = 0xc00;
    gconf->OE_CTRL_b.UART2_OE = 0;
    break;
  case PMU_MODULE_I2C0:
    pmulv->SRST_ON_PP1 = 0x30;
    pmulv->CG_ON_PP1 = 0x30;
    break;
  case PMU_MODULE_I2C1:
    pmulv->SRST_ON_PP1 = 0xc0;
    pmulv->CG_ON_PP1 = 0xc0;
    break;
  case PMU_MODULE_I2C2:
    pmulv->SRST_ON_PM_1 = 0x30000000;
    pmulv->CG_ON_PM_1 = 0x30000000;
    break;
  case PMU_MODULE_SPIM0:
    pmulv->SRST_ON_PP1 = 0x300;
    pmulv->CG_ON_PP1 = 0x300;
    gconf->OE_CTRL_b.SPIM0_OE = 0;
    break;
  case PMU_MODULE_SPIM1:
    pmulv->SRST_ON_PP1 = 0xc00;
    pmulv->CG_ON_PP1 = 0xc00;
    gconf->OE_CTRL_b.SPIM1_OE = 0;
    break;
  case PMU_MODULE_SPIM2:
    pmulv->SRST_ON_PM_1 = 0xc000000;
    pmulv->CG_ON_PM_1 = 0xc000000;
    gconf->OE_CTRL_b.SPIM2_OE = 0;
    break;
  case PMU_MODULE_SPIM3:
    pmulv->SRST_ON_PM_1 = 0x30000000;
    pmulv->CG_ON_PM_1 = 0x30000000;
    gconf->OE_CTRL_b.SPIM3_OE = 0;
    break;
  case PMU_MODULE_WDT:
    pmulv->SRST_ON_PM_1 = 0x10;
    pmulv->CG_ON_PM_1 = 0x20;
    pmulv->CG_ON_PM_1 = 0x10;
    break;
  case PMU_MODULE_ADVTMR:
    pmulv->SRST_ON_PM_1 = 0x80;
    pmulv->CG_ON_PM_1_b.CG_ppier0clk_advtmr_ch0_timclk = 1;
    pmulv->CG_ON_PM_1_b.CG_ppier0clk_advtmr_ch1_timclk = 1;
    pmulv->CG_ON_PM_1_b.CG_ppier0clk_advtmr_ch2_timclk = 1;
    pmulv->CG_ON_PM_1_b.CG_ppier0clk_advtmr_ch3_timclk = 1;
    pmulv->CG_ON_PM_1 = 0x80;
    break;
  case PMU_MODULE_TMR:
    pmulv->SRST_ON_PM_1 = 0x80000;
    pmulv->CG_ON_PM_1_b.CG_ppier0clk_tmr_ch0_timclk = 1;
    pmulv->CG_ON_PM_1_b.CG_ppier0clk_tmr_ch1_timclk = 1;
    pmulv->CG_ON_PM_1 = 0x80000;
    break;
  case PMU_MODULE_GCONF:
    pmulv->SRST_ON_PM_2_b.SRST_asyncrst_gconf_prstn = 1;
    pmulv->CG_ON_PM_2_b.CG_ppier0clk_gconf_pclk = 1;
    break;
  case PMU_MODULE_GPIO0:
    pmulv->SRST_ON_PM_2 = 0x10;
    pmulv->CG_ON_PM_2 = 0x10;
    break;
  case PMU_MODULE_GPIO1:
    pmulv->SRST_ON_PM_2 = 0x20;
    pmulv->CG_ON_PM_2 = 0x20;
    break;
  case PMU_MODULE_GPIO2:
    pmulv->SRST_ON_PM_2 = 0x40;
    pmulv->CG_ON_PM_2 = 0x40;
    break;
  case PMU_MODULE_GPIO3:
    pmulv->SRST_ON_PM_2 = 0x80;
    pmulv->CG_ON_PM_2 = 0x80;
    break;
  case PMU_MODULE_ADCC12:
    pmulv->SRST_ON_POWERDOMAIN_b.SRST_PA12 = 1;
    pmulv->CG_ON_POWERDOMAIN_b.CG_PA12 = 1;
    break;
  case PMU_MODULE_ADCC24:
    pmulv->SRST_ON_POWERDOMAIN_b.SRST_PA24 = 1;
    pmulv->CG_ON_POWERDOMAIN_b.CG_PA24 = 1;
    gconf->OE_CTRL_b.ADC24_OE = 0;
    break;
  case PMU_MODULE_RTCLV:
    pmulv->SRST_ON_PM_1_b.SRST_asyncrst_rtclv_prstn = 1;
    pmulv->SRST_ON_PM_1_b.SRST_asyncrst_rtclv_busrstn = 1;
    pmulv->CG_ON_PM_1_b.CG_ppier0clk_rtclv_pclk = 1;
    // tentative
    // pmulv->CG_ON_PM_1_b.CG_rtcclk_rtclv_busclk = 1;
    break;
  case PMU_MODULE_EVM:
    pmulv->SRST_ON_PM_0_b.SRST_asyncrst_evm_prstn = 1;
    pmulv->CG_ON_PM_0_b.CG_mpierclk_evm_pclk = 1;
    break;
  case PMU_MODULE_USB2FS:
    pmulv->SRST_ON_PU_b.SRST_asyncrst_h2hdnu_hrstn = 1;
    pmulv->SRST_ON_PU_b.SRST_asyncrst_h2hupu_hrstn = 1;
    pmulv->SRST_ON_PU_b.SRST_asyncrst_usb2fs_hrstn = 1;
    pmulv->SRST_ON_PU_b.SRST_asyncrst_usb2fs_usbrstn = 1;
    pmulv->CG_ON_PU_b.CG_mpierclk_h2hdnu_hclk = 1;
    pmulv->CG_ON_PU_b.CG_mpierclk_h2hupu_hclk = 1;
    pmulv->CG_ON_PU_b.CG_usbbclk_usb2fs_hclk = 1;
    pmulv->CG_ON_PU_b.CG_usbiclk_usb2fs_usbclk = 1;
    pmulv->CG_ON_POWERDOMAIN_b.CG_PU = 1;
    break;
  default:
    return PMU_ERROR;
  }
  
  PMU_Resources.info->module_enabled[module] = 0;
  PMU_BusResetOn();
  
  return PMU_OK;
}

/**
 * @brief enable or disable a clock of a module 
 * @param[in] module kind of module
 * @param[in] enable
 * @return \ref PMU_STATUS
 */
static PMU_STATUS PMU_EnableModule(PMU_MODULE module, uint32_t enable)
{
  PMU_RESOURCES *rsc = &PMU_Resources;

  if (!rsc->info->init) {
    return PMU_ERROR;
  }
  
  if (enable == 0) {
    return PMU_EnableModuleOff(module);
  } else if (enable == 1) {
    return PMU_EnableModuleOn(module);
  } else {
    return PMU_ERROR;
  }
}

/**
 * @brief configure additional clock gating module in SLEEP mode
 * @param[in] module kind of module
 * @param[in] enable
 * @return \ref PMU_STATUS
 */
static PMU_STATUS PMU_EnableModuleOnSleep(uint32_t module, uint32_t enable)
{
  PMU_RESOURCES *rsc = &PMU_Resources;
  uint32_t bit;

  if (!rsc->info->init) {
    return PMU_ERROR;
  }
  
  bit = (uint32_t)module;
  
  if (bit > 27) {
    return PMU_ERROR;
  }
  
  if (enable > 1) {
    return PMU_ERROR;
  }
  
  if (enable) {
    pmulv->POWERMODE_SLEEP_CG_ON |= 1u << bit;
  } else {
    pmulv->POWERMODE_SLEEP_CG_ON &= ~(1u << bit);
  }    
  
  return PMU_OK;
}

/**
 * @brief reset I/O output buffer
 * @param[in] func
 * @param[in] enable
 * @return \ref PMU_STATUS
 */
static PMU_STATUS PMU_ResetOutputBuffer(PMU_PD domain, uint32_t enable)
{
  PMU_RESOURCES *rsc = &PMU_Resources;

  if (!rsc->info->init) {
    return PMU_ERROR;
  }
  
  if (enable > 1) {
    return PMU_ERROR;
  }
  
  switch (domain) {
  case PMU_PD_AON_PM:
    pmulv->CTRL_IO_AON_0 = 1 - enable;
    break;
  case PMU_PD_AON_PP1:
    pmulv->CTRL_IO_AON_4 = 1 - enable;
    break;
  default:
    return PMU_ERROR;
  }
  
  return PMU_OK;
}

/**
 * @brief retain IO output buffer
 * @param[in] func
 * @param[in] enable
 * @return \ref PMU_STATUS
 */
static PMU_STATUS PMU_RetainOutputBuffer(PMU_PD domain, uint32_t enable)
{
  PMU_RESOURCES *rsc = &PMU_Resources;

  if (!rsc->info->init) {
    return PMU_ERROR;
  }
  
  if (enable > 1) {
    return PMU_ERROR;
  }
  
  switch (domain) {
  case PMU_PD_AON_PM:
    pmulv->CTRL_IO_AON_1 = enable;
    break;
  case PMU_PD_AON_PP1:
    pmulv->CTRL_IO_AON_5 = enable;
    break;
  default:
    return PMU_ERROR;
  }
  
  return PMU_OK;
}

/**
 * @brief standby I/O input buffer
 * @param[in] func
 * @param[in] enable
 * @return \ref PMU_STATUS
 */
static PMU_STATUS PMU_StandbyInputBuffer(PMU_IO_FUNC func, uint32_t standby)
{
  PMU_RESOURCES *rsc = &PMU_Resources;
  PMU_IO_PIN pin;
  __IO uint32_t *reg;
  
  static const struct {
    uint8_t reg; /* CTRL_IO_AON_* */
    uint8_t bit;
  } tbl[] = {
    {0, 99}, /* GPIO_0 */
    {3,  1}, /* GPIO_1 */
    {3,  2}, /* GPIO_2 */
    {3,  3}, /* GPIO_3 */
    {3,  4}, /* GPIO_4 */
    {3,  5}, /* GPIO_5 */
    {3,  6}, /* GPIO_6 */
    {3,  7}, /* GPIO_7 */
    {3,  8}, /* GPIO_8 */
    {3,  9}, /* GPIO_9 */
    {3, 10}, /* GPIO_10 */
    {3, 11}, /* GPIO_11 */
    {3, 12}, /* GPIO_12 */
    {3, 13}, /* GPIO_13 */
    {3, 14}, /* GPIO_14 */
    {3, 15}, /* GPIO_15 */
    {3, 24}, /* GPIO_24 */
    {3, 25}, /* GPIO_25 */
    {3, 26}, /* GPIO_26 */
    {3, 27}, /* GPIO_27 */
    {3, 28}, /* GPIO_28 */
    {3, 29}, /* GPIO_29 */
    {3, 30}, /* GPIO_30 */
    {3, 31}, /* GPIO_31 */
    {6, 12}, /* I2C0_DATA */
    {6, 13}, /* I2C0_CLK */
    {6, 14}, /* I2C1_DATA */
    {6, 15}, /* I2C1_CLK */
    {2,  0}, /* I2C2_DATA */
    {2,  0}, /* I2C2_CLK */
    {6,  0}, /* UA0_RXD */
    {6,  0}, /* UA0_TXD */
    {6,  8}, /* UA1_RXD */
    {6,  9}, /* UA1_TXD */
    {6, 10}, /* UA1_RTS_N */
    {6, 11}, /* UA1_CTS_N */
    {2,  4}, /* UA2_RXD */
    {2,  4}, /* UA2_TXD */
    {2,  5}, /* UA2_RTS_N */
    {2,  6}, /* UA2_CTS_N */
    {6, 16}, /* SPIM0_CS_N */
    {6, 17}, /* SPIM0_CLK */
    {6, 18}, /* SPIM0_MOSI */
    {6, 19}, /* SPIM0_MISO */
    {6, 20}, /* SPIM1_CS_N */
    {6, 21}, /* SPIM1_CLK */
    {6, 22}, /* SPIM1_MOSI */
    {6, 23}, /* SPIM1_MISO */
    {2,  1}, /* SPIM2_CS_N */
    {2,  1}, /* SPIM2_CLK */
    {2,  1}, /* SPIM2_MOSI */
    {2,  1}, /* SPIM2_MISO */
    {2,  2}, /* SPIM3_CS_N */
    {2,  2}, /* SPIM3_CLK */
    {2,  2}, /* SPIM3_MOSI */
    {2,  2}, /* SPIM3_MISO */
    {0, 99}, /* SPIC_CS_N */
    {0, 99}, /* SPIC_CLK */
    {0, 99}, /* SPIC_MOSI */
    {0, 99}, /* SPIC_MISO */
    {0, 99}, /* SPIC_IO2 */
    {0, 99}, /* SPIC_IO3 */
    {2,  8}, /* ADC24_SYNC */
    {0, 99}, /* DBG */
  };
  
  if (!rsc->info->init) {
    return PMU_ERROR;
  }
  
  pin = Func2Pin(func);
  if (pin == PMU_IO_PIN_INVALID) {
    /* pin is invalid */
    return PMU_ERROR;
  }
  
  if (tbl[pin].bit == 99) {
    /* standby not supported */
    return PMU_ERROR;
  }
  
  reg = &pmulv->CTRL_IO_AON_0 + tbl[pin].reg;
  if (standby) {
    *reg &= ~(1u << tbl[pin].bit);
  } else {
    *reg |= 1u << tbl[pin].bit;
  }
  
  return PMU_OK;
}

/**
 * @brief IO configuration
 * @param[in] func
 * @param[in] capability
 * @param[in] pullupdown
 * @return \ref PMU_STATUS
 */
static PMU_STATUS PMU_ConfigureIOCell(
    PMU_IO_FUNC func,
    PMU_IO_DRIVE_CAPABILITY capability,
    PMU_IO_RESISTOR pullupdown)
{
  PMU_RESOURCES *rsc = &PMU_Resources;
  PMU_IO_PIN pin;
  __IO uint32_t *reg;
  uint32_t t;
  
  static const struct {
    uint8_t reg; /* gconf->IO_CFG_* */
    uint8_t bit;
  } tbl[] = {
    {0, 99}, /* GPIO_0 */
    {0,  4}, /* GPIO_1 */
    {0,  8}, /* GPIO_2 */
    {0, 12}, /* GPIO_3 */
    {0, 16}, /* GPIO_4 */
    {0, 20}, /* GPIO_5 */
    {0, 24}, /* GPIO_6 */
    {0, 28}, /* GPIO_7 */
    {1,  0}, /* GPIO_8 */
    {1,  4}, /* GPIO_9 */
    {1,  8}, /* GPIO_10 */
    {1, 12}, /* GPIO_11 */
    {1, 16}, /* GPIO_12 */
    {1, 20}, /* GPIO_13 */
    {1, 24}, /* GPIO_14 */
    {1, 28}, /* GPIO_15 */
    {3,  0}, /* GPIO_24 */
    {3,  4}, /* GPIO_25 */
    {3,  8}, /* GPIO_26 */
    {3, 12}, /* GPIO_27 */
    {3, 16}, /* GPIO_28 */
    {3, 20}, /* GPIO_29 */
    {3, 24}, /* GPIO_30 */
    {3, 28}, /* GPIO_31 */
    {4,  0}, /* I2C0_DATA */
    {4,  4}, /* I2C0_CLK */
    {4,  8}, /* I2C1_DATA */
    {4, 12}, /* I2C1_CLK */
    {4, 16}, /* I2C2_DATA */
    {4, 20}, /* I2C2_CLK */
    {5,  0}, /* UA0_RXD */
    {5,  4}, /* UA0_TXD */
    {5, 16}, /* UA1_RXD */
    {5, 20}, /* UA1_TXD */
    {5, 24}, /* UA1_RTS_N */
    {5, 28}, /* UA1_CTS_N */
    {6,  0}, /* UA2_RXD */
    {6,  4}, /* UA2_TXD */
    {6,  8}, /* UA2_RTS_N */
    {6, 12}, /* UA2_CTS_N */
    {7,  0}, /* SPIM0_CS_N */
    {7,  4}, /* SPIM0_CLK */
    {7,  8}, /* SPIM0_MOSI */
    {7, 12}, /* SPIM0_MISO */
    {7, 16}, /* SPIM1_CS_N */
    {7, 20}, /* SPIM1_CLK */
    {7, 24}, /* SPIM1_MOSI */
    {7, 28}, /* SPIM1_MISO */
    {8,  0}, /* SPIM2_CS_N */
    {8,  4}, /* SPIM2_CLK */
    {8,  8}, /* SPIM2_MOSI */
    {8, 12}, /* SPIM2_MISO */
    {8, 16}, /* SPIM3_CS_N */
    {8, 20}, /* SPIM3_CLK */
    {8, 24}, /* SPIM3_MOSI */
    {8, 28}, /* SPIM3_MISO */
    {9,  0}, /* SPIC_CS_N */
    {9,  4}, /* SPIC_CLK */
    {9,  8}, /* SPIC_MOSI */
    {9, 12}, /* SPIC_MISO */
    {9, 16}, /* SPIC_IO2 */
    {9, 20}, /* SPIC_IO3 */
    {10, 0}, /* ADC24_SYNC */
    {11, 0}, /* DBG */
  };
  
  if (!rsc->info->init) {
    return PMU_ERROR;
  }
  
  pin = Func2Pin(func);
  if (pin == PMU_IO_PIN_INVALID) {
    /* pin is invalid */
    return PMU_ERROR;
  }
  
  if (tbl[pin].bit == 99) {
    /* IO cell configuration not supported */
    return PMU_ERROR;
  }
  
  if ((uint32_t)capability > PMU_DRIVE_CAPABILITY_7MA) {
    return PMU_ERROR;
  }
  if ((uint32_t)pullupdown > PMU_IO_RESISTOR_PULLUP) {
    return PMU_ERROR;
  }
  
  reg = &gconf->IO_CFG0 + tbl[pin].reg;
  t = *reg;
  t &= ~(15u << tbl[pin].bit);
  t |= (((uint32_t)capability << 2) + (uint32_t)pullupdown) << tbl[pin].bit;
  *reg = t;
  
  return PMU_OK;
}

/**
 * @brief Trim SiOSC4M clock source.
 * @param mode Voltage Mode
 */
static void PMU_TrimSIOSC4M(PMU_VOLTAGE_MODE mode)
{
  uint32_t t;
  
  pmulv->SELECT_EFUSE_b.SEL_EFUSE_SiOSC4M = 1;
  pmulv->CONFIG_SiOSC4M_b.SiOSC4M_CTRIM_LAT = 1;
  switch (mode) {
  case PMU_VOLTAGE_MODE_A:
    t = pmulv->EFUSE_SiOSC4M_b.EFUSE_SiOSC4M_CTRIM_MODEA;
    break;
  case PMU_VOLTAGE_MODE_B:
    t = pmulv->EFUSE_SiOSC4M_b.EFUSE_SiOSC4M_CTRIM_MODEB;
    break;
  case PMU_VOLTAGE_MODE_C:
    t = pmulv->EFUSE_SiOSC4M_b.EFUSE_SiOSC4M_CTRIM_MODEC;
    break;
  case PMU_VOLTAGE_MODE_D:
  default:
    t = pmulv->EFUSE_SiOSC4M_b.EFUSE_SiOSC4M_CTRIM_MODED;
    break;
  }
  pmulv->OVERRIDE_EFUSE_SiOSC4M_b.OVERRIDE_EFUSE_SiOSC4M_CTRIM = t;
  pmulv->CONFIG_SiOSC4M_b.SiOSC4M_CTRIM_LAT = 0;
}

/**
 * @brief Prevent RTC IRQ Handler and wait until storing in RTC control register.
 *
 * @return previous status of IRQ mask
 */
static uint32_t PMU_PreventRTCirq(void)
{
  uint32_t prevented_irq;
  PMU_RESOURCES *rsc = &PMU_Resources;

  prevented_irq = __get_PRIMASK();
  __disable_irq();
  if (rsc->info->module_enabled[PMU_MODULE_RTCLV]) {
    if (pmulv->PRESCAL_MAIN_b.PSSEL_CD_PPIER0) {
      while (0 != rtclv->RTC_CTRL_WIP) {
        /* DO NOTHING */
      }
    }
  }
  return prevented_irq;
}

/**
 * @brief Prevent RTC IRQ Handler and wait until writing value into RTC control register.
 *
 * @param[in] Specify IRQ mask obtained from PMU_PreventRTCirq
 */
static void PMU_RestoreRTCirq(uint32_t prevented_irq)
{
  if (!prevented_irq) {
    __enable_irq();
  }
}

static void PMU_WaitUntilDoneRtc(PMU_INFO *info)
{
  if (info->module_enabled[PMU_MODULE_RTCLV]) {
    if (pmulv->PRESCAL_MAIN_b.PSSEL_CD_PPIER0) {
      while (0 != rtclv->RTC_CTRL_WIP) {
        /* DO NOTHING */
      }
    }
  }
}

static uint32_t PMU_ExpandEventSettings(uint32_t iev)
{
  iev = ((iev << 12) | iev) & 0x000F000F;
  iev = ((iev <<  6) | iev) & 0x03030303;
  iev = ((iev <<  3) | iev) & 0x11111111;
  return iev;
}

static uint32_t PMU_ShrinkWakeupSettings(uint32_t is)
{
  is = ((is >>  3) | is) & 0x03030303;
  is = ((is >>  6) | is) & 0x000F000F;
  is = ((is >> 12) | is) & 0x000000FF;
  return is;
}

static uint32_t PMU_MakeWmaskForEdge(uint32_t is, uint32_t iev)
{
  uint32_t msk;

  msk = ((is >> 1) & ~iev) | (is & iev);
  return PMU_ShrinkWakeupSettings(msk);
}

static uint32_t PMU_MakeMaskForDetectionSingleEdge(uint32_t is, uint32_t ibe)
{
  uint32_t both_edge;
  both_edge = ((is >> 1) & is) & 0x11111111;
  if (both_edge) {
    both_edge = PMU_ShrinkWakeupSettings(both_edge);
    both_edge &= ~ibe;
  }
  return both_edge;
}

void PMU_MakeWakeupMasks(uint32_t is, gpio_Type* g, uint32_t *r_um, uint32_t *r_sngl, uint32_t *r_iev)
{
  uint32_t ibe, sngl, iev, um;
  uint32_t gdir, gis;

  ibe  = g->GPIOIBE;
  sngl = PMU_MakeMaskForDetectionSingleEdge(is, ibe);
  iev  = g->GPIOIEV;
  iev  = PMU_ExpandEventSettings(iev);
  um   = PMU_MakeWmaskForEdge(is, iev);
  um  |= ibe;
  gdir = g->GPIODIR;
  gis  = g->GPIOIS;
  um  &= ~(gdir | gis);

  *r_um   = um;
  *r_sngl = sngl;
  *r_iev  = iev;
}

//////////////////////////////////////////////////////////////////////////////



/**
 * @brief Get driver version.
 * @return \ref ARM_DRV_VERSION
 */
static ARM_DRIVER_VERSION PMU_GetVersion(void)
{
  static const ARM_DRIVER_VERSION driver_version = {
    PMU_API_VERSION,
    PMU_DRV_VERSION
  };
  return driver_version;
}

/**
 * @brief Initialize PMU driver interface.
 * @param[in] cb_event
 * @return \ref PMU_STATUS
 */
static PMU_STATUS PMU_Initialize(PMU_SignalEvent_t cb_event)
{
  PMU_RESOURCES *rsc = &PMU_Resources;
  PMU_INFO      *info;

  info = rsc->info;
  if (info->init) {
    return PMU_ERROR;
  }

  /* Initialize driver internal state */
  info->init = true;
  info->cb_event = cb_event;

  if (0 == pmulv->CONFIG_OSC32K_b.OSC32K_BOOST_DISABLE) {
    info->clock_state[PMU_CLOCK_SOURCE_OSC32K] = PMU_CLOCK_SOURCE_STATE_STOPPED;
  } else {
    info->clock_state[PMU_CLOCK_SOURCE_OSC32K] = PMU_CLOCK_SOURCE_STATE_RUNNING;
  }
  if (0 == pmulv->CONFIG_SiOSC32K_b.SiOSC32K_EN) {
    info->clock_state[PMU_CLOCK_SOURCE_SIOSC32K] = PMU_CLOCK_SOURCE_STATE_STOPPED;
  } else {
    info->clock_state[PMU_CLOCK_SOURCE_SIOSC32K] = PMU_CLOCK_SOURCE_STATE_RUNNING;
  }
  info->clock_state[PMU_CLOCK_SOURCE_SIOSC4M]  = PMU_CLOCK_SOURCE_STATE_RUNNING;
  info->clock_state[PMU_CLOCK_SOURCE_ADPLL]    = PMU_CLOCK_SOURCE_STATE_STOPPED;
  info->clock_state[PMU_CLOCK_SOURCE_OSC12M]   = PMU_CLOCK_SOURCE_STATE_RUNNING;
  info->clock_state[PMU_CLOCK_SOURCE_PLL]      = PMU_CLOCK_SOURCE_STATE_RUNNING;
  info->clock_state[PMU_CLOCK_SOURCE_32K]      = PMU_CLOCK_SOURCE_STATE_STOPPED;
  
  info->pll_freq = 48000000;
  
  info->module_enabled[PMU_MODULE_MPIER] = 1;
  info->module_enabled[PMU_MODULE_SPIC] = 1;
  
  /* Turn off POWER_EFUSE */
  pmulv->ISO_EFUSE_b.INISOEN_EFUSE = 1;
  pmulv->ISO_EFUSE_b.OUTISOEN_EFUSE = 1;
  pmulv->PSW_EFUSE_b.PSW_EFUSE_VDDCS = 0;
  pmulv->PSW_EFUSE_b.PSW_EFUSE_VDDCW = 0;
  
  /* Enable dynamic clock gating */
  pmulv->DCG_POWERDOMAIN = 0xfbfu;
  /* workaround for eratta E25 */
  pmulv->DCG_PM_2_b.DCG_ppier0clk_gconf_pclk = 0;

  /* Initialize iomux setting */
  PMU_SetPrescaler(PMU_CD_PPIER0,    4);
  PMU_EnableModule(PMU_MODULE_GCONF, 1);
  InitIomux();

  _SystemCoreClockUpdate();

  /* Trim clock sources */
  /* OSC32K */
  if (PMU_CLOCK_SOURCE_STATE_STOPPED == info->clock_state[PMU_CLOCK_SOURCE_OSC32K]) {
    pmulv->SELECT_EFUSE_b.SEL_EFUSE_OSC32K = 0;
    pmulv->OVERRIDE_EFUSE_OSC32K           = 1;
  }
  /* SIOSC32K */
  if (PMU_CLOCK_SOURCE_STATE_STOPPED == info->clock_state[PMU_CLOCK_SOURCE_SIOSC32K]) {
    pmulv->SELECT_EFUSE_b.SEL_EFUSE_SiOSC32K = 0;
    pmulv->OVERRIDE_EFUSE_SiOSC32K           = 1;
  }
  /* SIOSC4M */
  PMU_TrimSIOSC4M((PMU_VOLTAGE_MODE)pmulv->MOVE_VOLTAGE_START_b.VMSTATUS);

  /* Read and clear reset factor */
  info->reset_factor = pmulv->STATUS_LVRST;
  pmulv->STATUS_LVRST = 0xffffffffu;

  /* set DCDC_EXTCLKMODE and DCDC_SSFAST */
  pmulv->CONFIG_DCDC_LVREG_1 |= (1u << 31) | (1u << 18);
  /* set WAITTIME_PSW_PWON and WAITTIME_PSW_PWOFF */
  pmulv->WAITTIME_PSW         = 0x00090009;
  pmulv->WAITTIME_DVSCTL_b.WAITTIME_CHGTIME = 0;

  /* Enable PMU_WKUP IRQ */
  NVIC_ClearPendingIRQ(PMU_WKUP_IRQn);
  NVIC_EnableIRQ(PMU_WKUP_IRQn);

#if (RTE_WORKAROUND_51 == 1)
  /* workaround for E51 */
  pmulv->SELECT_EFUSE_b.SEL_EFUSE_LDOF = 1;
  *((volatile uint32_t*)0x400005b0)    = 2;
#endif

  return PMU_OK;
}

/**
 * @brief Do nothing.
 * @return \ref PMU_STATUS
 */
static PMU_STATUS PMU_Uninitialize(void)
{
  return PMU_OK;
}

/**
 * @brief Start clock source.
 * @param[in] source Clock source
 * @return \ref PMU_STATUS
 */
static PMU_STATUS PMU_StartClockSource(PMU_CLOCK_SOURCE source)
{
  PMU_RESOURCES *rsc = &PMU_Resources;
  PMU_INFO      *info;

  info = rsc->info;
  if (!info->init) {
    return PMU_ERROR;
  }
  
  if ((uint32_t)source > PMU_CLOCK_SOURCE_PLL) {
    return PMU_ERROR;
  }
  if (info->clock_state[source] != PMU_CLOCK_SOURCE_STATE_STOPPED) {
    return PMU_OK;
  }

  if (source == PMU_CLOCK_SOURCE_OSC12M) {
    pmulv->CONFIG_OSC12M_b.OSC12M_EN = 1;
    usleep(1000);
    info->clock_state[PMU_CLOCK_SOURCE_OSC12M] = PMU_CLOCK_SOURCE_STATE_RUNNING;
  } else if (source == PMU_CLOCK_SOURCE_OSC32K) {
    pmulv->CONFIG_OSC32K_b.OSC32K_EN = 1;
    usleep(1000000); /* FIXME: rewrite it to non-blocking function */
    pmulv->CONFIG_OSC32K_b.OSC32K_BOOST_DISABLE = 1;
    usleep(2000);
    info->clock_state[PMU_CLOCK_SOURCE_OSC32K] = PMU_CLOCK_SOURCE_STATE_RUNNING;
  } else if (source == PMU_CLOCK_SOURCE_SIOSC32K) {
    pmulv->CONFIG_SiOSC32K_b.SiOSC32K_EN = 1;
    usleep(500);
    info->clock_state[PMU_CLOCK_SOURCE_SIOSC32K] = PMU_CLOCK_SOURCE_STATE_RUNNING;
  } else if (source == PMU_CLOCK_SOURCE_PLL) {
    if (PMU_GetClockSourceState(PMU_CLOCK_SOURCE_OSC12M) != PMU_CLOCK_SOURCE_STATE_RUNNING) {
      return PMU_ERROR;
    }
    /* Turn on PPLL */
    pmulv->PSW_PLL_b.PSW_PLL_VDDCW = 1;
    pmulv->PSW_PLL_b.PSW_PLL_VDDCS = 1;
    pmulv->ISO_PLL_b.INISOEN_PLL = 0;
    pmulv->ISO_PLL_b.OUTISOEN_PLL = 0;

    pmulv->CONFIG_PLL_0_b.PLL_BP = 1;
    usleep(100);
    pmulv->CONFIG_PLL_0_b.PLL_BP = 0;
    usleep(100);
    info->clock_state[PMU_CLOCK_SOURCE_PLL] = PMU_CLOCK_SOURCE_STATE_RUNNING;
  } else if (source == PMU_CLOCK_SOURCE_ADPLL) {
    if (PMU_GetClockSourceState(PMU_CLOCK_SOURCE_32K) != PMU_CLOCK_SOURCE_STATE_RUNNING) {
      return PMU_ERROR;
    }
    /* Start reference clock for ADPLL */
    pmulv->CG_OFF_PA_b.CG_32KOSC_pmulv               = 1;
    pmulv->CG_OFF_REFCLK_b.CG_ref32kclk_adpll_refclk = 1;
    /* Turn on PADPLL */
    pmulv->PSW_ADPLL_b.PSW_ADPLL_VDDCW = 1;
    pmulv->PSW_ADPLL_b.PSW_ADPLL_VDDCS = 1;
    usleep(150);
    pmulv->ISO_ADPLL_b.INISOEN_ADPLL = 0;
    pmulv->ISO_ADPLL_b.OUTISOEN_ADPLL = 0;
    
    /* Start ADPLL */
    pmulv->CONFIG_ADPLL_0_b.ADPLL_ENPLL = 1;
    pmulv->CONFIG_ADPLL_1_b.ADPLL_NTWOUT_LATEN = 1;
    usleep(8000);
    info->clock_state[PMU_CLOCK_SOURCE_ADPLL] = PMU_CLOCK_SOURCE_STATE_RUNNING;
  } else {
    return PMU_ERROR;
  }
  
  return PMU_OK;
}

/**
 * @brief Stop clock source.
 * @param[in] source Clock source
 * @return \ref PMU_STATUS
 */
static PMU_STATUS PMU_StopClockSource(PMU_CLOCK_SOURCE source)
{
  PMU_RESOURCES *rsc = &PMU_Resources;
  
  if (!rsc->info->init) {
    return PMU_ERROR;
  }
  if ((uint32_t)source > PMU_CLOCK_SOURCE_PLL) {
    return PMU_ERROR;
  }
  if (rsc->info->clock_state[source] == PMU_CLOCK_SOURCE_STATE_STOPPED) {
    return PMU_OK;
  }
  
  if (source == PMU_CLOCK_SOURCE_OSC12M) {
    rsc->info->clock_state[PMU_CLOCK_SOURCE_OSC12M] = PMU_CLOCK_SOURCE_STATE_STOPPED;
    pmulv->CONFIG_OSC12M_b.OSC12M_EN = 0;
  } else if (source == PMU_CLOCK_SOURCE_OSC32K) {
    rsc->info->clock_state[PMU_CLOCK_SOURCE_OSC32K] = PMU_CLOCK_SOURCE_STATE_STOPPED;
    pmulv->CONFIG_OSC32K_b.OSC32K_BOOST_DISABLE = 0;
    pmulv->CONFIG_OSC32K_b.OSC32K_EN = 0;
  } else if (source == PMU_CLOCK_SOURCE_SIOSC32K) {
    rsc->info->clock_state[PMU_CLOCK_SOURCE_SIOSC32K] = PMU_CLOCK_SOURCE_STATE_STOPPED;
    pmulv->CONFIG_SiOSC32K_b.SiOSC32K_EN = 0;
  } else if (source == PMU_CLOCK_SOURCE_PLL) {
    rsc->info->clock_state[PMU_CLOCK_SOURCE_PLL] = PMU_CLOCK_SOURCE_STATE_STOPPED;
    pmulv->CONFIG_PLL_0_b.PLL_BP = 1;
    /* Turn off PPLL */
    pmulv->ISO_PLL_b.INISOEN_PLL = 1;
    pmulv->ISO_PLL_b.OUTISOEN_PLL = 1;
    pmulv->PSW_PLL_b.PSW_PLL_VDDCS = 0;
    pmulv->PSW_PLL_b.PSW_PLL_VDDCW = 0;
  } else if (source == PMU_CLOCK_SOURCE_ADPLL) {
    rsc->info->clock_state[PMU_CLOCK_SOURCE_ADPLL] = PMU_CLOCK_SOURCE_STATE_STOPPED;
    /* Stop ADPLL */
    pmulv->CONFIG_ADPLL_0_b.ADPLL_ENPLL = 0;
    /* Turn off PADPLL */
    pmulv->ISO_ADPLL_b.INISOEN_ADPLL = 1;
    pmulv->ISO_ADPLL_b.OUTISOEN_ADPLL = 1;
    pmulv->PSW_ADPLL_b.PSW_ADPLL_VDDCS = 0;
    pmulv->PSW_ADPLL_b.PSW_ADPLL_VDDCW = 0;
    /* Stop reference clock for ADPLL */
    pmulv->CG_ON_REFCLK_b.CG_ref32kclk_adpll_refclk = 1;
  } else {
    return PMU_ERROR;
  }
  
  return PMU_OK;
}

/**
 * @brief Get state of clock source
 * @param[in] source Clock source
 * @return \ref PMU_STATUS
 */
static PMU_CLOCK_SOURCE_STATE PMU_GetClockSourceState(PMU_CLOCK_SOURCE source)
{
  PMU_RESOURCES *rsc = &PMU_Resources;
  
  if (!rsc->info->init) {
    return PMU_CLOCK_SOURCE_STATE_STOPPED;
  }
  
  if ((uint32_t)source >= PMU_CLOCK_SOURCE_INVALID) {
    return PMU_CLOCK_SOURCE_STATE_STOPPED;
  }
  
  if (source == PMU_CLOCK_SOURCE_32K) {
    if (pmulv->CSM_RTC_b.CSMSEL_RTC != 0) {
      source = PMU_CLOCK_SOURCE_SIOSC32K;
    } else {
      source = PMU_CLOCK_SOURCE_OSC32K;
    }
  }
  
  return rsc->info->clock_state[source];
}

static PMU_CLOCK_SOURCE_STATE PMU_GetClockSourceStateInternal(uint32_t source)
{
  PMU_CLOCK_SOURCE s;
  switch (source) {
  case PMU_CLOCK_SOURCE_OSC12M3:
    s = PMU_CLOCK_SOURCE_OSC12M;
    break;
  case PMU_CLOCK_SOURCE_PLL3:
  case PMU_CLOCK_SOURCE_PLL9:
    s = PMU_CLOCK_SOURCE_PLL;
    break;
  default:
    s = (PMU_CLOCK_SOURCE)source;
    break;
  }
  return PMU_GetClockSourceState(s);
}

/**
 * @brief Set PLL frequency
 * @param[in] freq Frequency (24000000, 36000000 or 48000000)
 * @return \ref PMU_STATUS
 */
static PMU_STATUS PMU_SetPLLFrequency(uint32_t freq)
{
  PMU_RESOURCES *rsc = &PMU_Resources;
  uint32_t nd;
  PMU_INFO      *info;

  info = rsc->info;
  if (!info->init) {
    return PMU_ERROR;
  }

  if (freq == info->pll_freq) {
    return PMU_OK;
  }

  switch (freq) {
  case 24000000:
    nd = 0x91;
    break;
  case 36000000:
    nd = 0x92;
    break;
  case 48000000:
    nd = 0x93;
    break;
  default:
    return PMU_ERROR;
  }
  
  /* PLL should be stopped */
  if (info->clock_state[PMU_CLOCK_SOURCE_PLL] != PMU_CLOCK_SOURCE_STATE_STOPPED) {
    return PMU_ERROR;
  }
  
  pmulv->CONFIG_PLL_0_b.PLL_SWEN = 1;
  pmulv->CONFIG_PLL_0_b.PLL_BP = 1;
  pmulv->CONFIG_PLL_1_b.PLL_ND = nd;
  pmulv->CONFIG_PLL_0_b.PLL_SWEN = 0;

  info->pll_freq = freq;
  return PMU_OK;
}

/**
 * @brief Set prescaler of a clock domain
 * @param[in] clockDomain clock domain
 * @param[in] prescaler frequency divider (0 means clock-gated)
 * @return \ref PMU_STATUS
 */
static PMU_STATUS PMU_SetPrescaler(PMU_CD clockDomain, uint32_t divider)
{
  PMU_RESOURCES *rsc = &PMU_Resources;
  uint32_t n;
  uint32_t prevented_irq;

  if (!rsc->info->init) {
    return PMU_ERROR;
  }

  if (divider <= 10) {
    n = divider;
  } else if (divider == 12) {
    n = 11;
  } else if (divider == 18) {
    n = 12;
  } else if (divider == 24) {
    n = 13;
  } else if (divider == 36) {
    n = 14;
  } else if (divider == 48) {
    n = 15;
  } else {
    return PMU_ERROR;
  }
  
  switch (clockDomain) {
  case PMU_CD_MPIER:
  case PMU_CD_PPIER0:
  case PMU_CD_PPIER1:
  case PMU_CD_PPIER2:
  case PMU_CD_SPIC:
  case PMU_CD_CPUST:
  case PMU_CD_UART0: 
  case PMU_CD_UART1: 
  case PMU_CD_UART2: 
    break;
  case PMU_CD_USBB:  
  case PMU_CD_USBI:
    if (n > 2) {
      return PMU_ERROR;
    }
    break;
  case PMU_CD_ADCC12:
    switch (PMU_GetClockSourceSelector(PMU_CSM_ADCC12)) {
    case PMU_CLOCK_SOURCE_SIOSC4M:
      if ((n != 0) && (n != 1) && (n != 2) && (n != 4)) {
        return PMU_ERROR;
      }
      break;
    case PMU_CLOCK_SOURCE_OSC12M:
    case PMU_CLOCK_SOURCE_OSC12M3:
      if (n == 0) {
        /* DO NOTHING */
      } else if ((n == 1) || (n == 2) || (n == 4) || (n == 6)) {
        PMU_SelectClockSourceInternal(PMU_CSM_ADCC12, PMU_CLOCK_SOURCE_OSC12M);
      } else if (n == 11) {
        /* divider == 12 */
        PMU_SelectClockSourceInternal(PMU_CSM_ADCC12, PMU_CLOCK_SOURCE_OSC12M);
      } else if (n == 3) {
        /* OSC12M / 3 -> OSC12M3 / 1 */
        PMU_SelectClockSourceInternal(PMU_CSM_ADCC12, PMU_CLOCK_SOURCE_OSC12M3);
        n = 1;
      } else {
        return PMU_ERROR;
      }
      break;
    case PMU_CLOCK_SOURCE_PLL:
    case PMU_CLOCK_SOURCE_PLL3:
    case PMU_CLOCK_SOURCE_PLL9:
      if (n == 0) {
        /* DO NOTHING */
      } else if ((n == 2) || (n == 4) || (n == 6) || (n == 8) || (n >= 11)) {
        /* divider = {2, 4, 6, 8, 12, 18, 24, 36, 48} */
        PMU_SelectClockSourceInternal(PMU_CSM_ADCC12, PMU_CLOCK_SOURCE_PLL);
      } else if (n == 3) {
        /* PLL / 3 -> PLL3 / 1 */
        PMU_SelectClockSourceInternal(PMU_CSM_ADCC12, PMU_CLOCK_SOURCE_PLL3);
        n = 1;
      } else if (n == 9) {
        /* PLL / 9 -> PLL9 / 1 */
        PMU_SelectClockSourceInternal(PMU_CSM_ADCC12, PMU_CLOCK_SOURCE_PLL9);
        n = 1;
      } else {
        return PMU_ERROR;
      }
      break;
    case PMU_CLOCK_SOURCE_ADPLL:
      if ((n != 0) && (n != 4) && (n != 6) && (n != 8) && (n != 11) && (n != 13) && (n != 15)) {
        /* divider != {0, 4, 6, 8, 12, 24, 48} */
        return PMU_ERROR;
      }
      break;
    default:
      return PMU_ERROR;
    }
    break;
  case PMU_CD_ADCC24:
    switch (PMU_GetClockSourceSelector(PMU_CSM_ADCC24)) {
    case PMU_CLOCK_SOURCE_SIOSC4M:
      if (n > 2) {
        return PMU_ERROR;
      }
      break;
    case PMU_CLOCK_SOURCE_OSC12M:
    case PMU_CLOCK_SOURCE_OSC12M3:
      if (n == 0) {
        /* DO NOTHING */
      } else if (n == 3) {
        /* OSC12M / 3 -> OSC12M3 / 1 */
        PMU_SelectClockSourceInternal(PMU_CSM_ADCC24, PMU_CLOCK_SOURCE_OSC12M3);
        n = 1;
      } else {
        return PMU_ERROR;
      }
      break;
    case PMU_CLOCK_SOURCE_PLL:
    case PMU_CLOCK_SOURCE_PLL9:
      if (n == 0) {
        /* DO NOTHING */
      } else if ((n == 6) || (n == 11)) {
        /* divider = {6, 12} */
        PMU_SelectClockSourceInternal(PMU_CSM_ADCC24, PMU_CLOCK_SOURCE_PLL);
      } else if (n == 9) {
        /* PLL / 9 -> PLL9 / 1 */
        PMU_SelectClockSourceInternal(PMU_CSM_ADCC24, PMU_CLOCK_SOURCE_PLL9);
        n = 1;
      } else {
        return PMU_ERROR;
      }
      break;
    case PMU_CLOCK_SOURCE_ADPLL:
      if ((n != 0) && (n != 6) && (n != 11)) {
        /* divider != {0, 6, 12} */
        return PMU_ERROR;
      }
      break;
    default:
      return PMU_ERROR;
    }
    break;
  case PMU_CD_RTC:
  case PMU_CD_CPUTRC:
  case PMU_CD_PMULV:
  default:
    return PMU_ERROR;
  }
  
  switch (clockDomain) {
  case PMU_CD_MPIER:
    prevented_irq = PMU_PreventRTCirq();
    pmulv->PRESCAL_MAIN_b.PSSEL_CD_MPIER = n;
    PMU_RestoreRTCirq(prevented_irq);
    break;
  case PMU_CD_PPIER0:
    prevented_irq = PMU_PreventRTCirq();
    pmulv->PRESCAL_MAIN_b.PSSEL_CD_PPIER0 = n;
    PMU_RestoreRTCirq(prevented_irq);
    break;
  case PMU_CD_PPIER1:
    pmulv->PRESCAL_MAIN_b.PSSEL_CD_PPIER1 = n;
    break;
  case PMU_CD_PPIER2:
    pmulv->PRESCAL_MAIN_b.PSSEL_CD_PPIER2 = n;
    break;
  case PMU_CD_SPIC:
    pmulv->PRESCAL_MAIN_b.PSSEL_CD_SPIC = n;
    break;
  case PMU_CD_USBB:  
    pmulv->PRESCAL_MAIN_b.PSSEL_CD_USBB = n;
    break;
  case PMU_CD_CPUST:
    pmulv->PRESCAL_CPUST_b.PSSEL_CD_CPUST = n;
    break;
  case PMU_CD_UART0: 
    pmulv->PRESCAL_UART0_b.PSSEL_CD_UART0 = n;
    break;
  case PMU_CD_UART1: 
    pmulv->PRESCAL_UART1_b.PSSEL_CD_UART1 = n;
    break;
  case PMU_CD_UART2: 
    pmulv->PRESCAL_UART2_b.PSSEL_CD_UART2 = n;
    break;
  case PMU_CD_USBI:
    pmulv->PRESCAL_USBI_b.PSSEL_CD_USBI = n;
    break;
  case PMU_CD_ADCC12:
    pmulv->PRESCAL_ADCC12A_b.PSSEL_CD_ADCC12A = n;
    break;
  case PMU_CD_ADCC24:
    pmulv->PRESCAL_ADCC24A_b.PSSEL_CD_ADCC24A = n;
    break;
  case PMU_CD_RTC:
  case PMU_CD_CPUTRC:
  case PMU_CD_PMULV:
  default:
    return PMU_ERROR;
  }
  
  if (clockDomain == PMU_CD_MPIER) {
    _SystemCoreClockUpdate();
  }
  
  return PMU_OK;
}

/**
 * @brief Get prescaler of a clock domain
 * @param[in] clockDomain clock domain
 * @return prescaler
 */
static uint32_t PMU_GetPrescaler(PMU_CD clockDomain)
{
  PMU_RESOURCES *rsc = &PMU_Resources;
  uint32_t n;
  uint32_t src;
  
  if (!rsc->info->init) {
    return 0;
  }
  
  switch (clockDomain) {
  case PMU_CD_MPIER:
    n = pmulv->PRESCAL_MAIN_b.PSSEL_CD_MPIER;
    break;
  case PMU_CD_PPIER0:
    n = pmulv->PRESCAL_MAIN_b.PSSEL_CD_PPIER0;
    break;
  case PMU_CD_PPIER1:
    n = pmulv->PRESCAL_MAIN_b.PSSEL_CD_PPIER1;
    break;
  case PMU_CD_PPIER2:
    n = pmulv->PRESCAL_MAIN_b.PSSEL_CD_PPIER2;
    break;
  case PMU_CD_SPIC:
    n = pmulv->PRESCAL_MAIN_b.PSSEL_CD_SPIC;
    break;
  case PMU_CD_USBB:  
    n = pmulv->PRESCAL_MAIN_b.PSSEL_CD_USBB;
    break;
  case PMU_CD_CPUST:
    n = pmulv->PRESCAL_CPUST_b.PSSEL_CD_CPUST;
    break;
  case PMU_CD_UART0: 
    n = pmulv->PRESCAL_UART0_b.PSSEL_CD_UART0;
    break;
  case PMU_CD_UART1: 
    n = pmulv->PRESCAL_UART1_b.PSSEL_CD_UART1;
    break;
  case PMU_CD_UART2: 
    n = pmulv->PRESCAL_UART2_b.PSSEL_CD_UART2;
    break;
  case PMU_CD_USBI:
    n = pmulv->PRESCAL_USBI_b.PSSEL_CD_USBI;
    break;
  case PMU_CD_ADCC12:
    n = pmulv->PRESCAL_ADCC12A_b.PSSEL_CD_ADCC12A;
    if (n == 1) {
      src = PMU_GetClockSourceSelector(PMU_CSM_ADCC12);
      /* OSC12M3, PLL3 and PLL9 are treated as OSC12M/3, PLL/3 and PLL/9 */
      if (src == PMU_CLOCK_SOURCE_OSC12M3) {
        n = 3;
      } else if (src == PMU_CLOCK_SOURCE_PLL3) {
        n = 3;
      } else if (src == PMU_CLOCK_SOURCE_PLL9) {
        n = 9;
      }
    }
    break;
  case PMU_CD_ADCC24:
    n = pmulv->PRESCAL_ADCC24A_b.PSSEL_CD_ADCC24A;
    if (n == 1) {
      src = PMU_GetClockSourceSelector(PMU_CSM_ADCC24);
      if (src == PMU_CLOCK_SOURCE_OSC12M3) {
        n = 3;
      } else if (src == PMU_CLOCK_SOURCE_PLL9) {
        n = 9;
      }
    }
    break;
  case PMU_CD_RTC:
  case PMU_CD_CPUTRC:
  case PMU_CD_PMULV:
    /* CD_RTC, CD_CPUTRC and CD_PMULV don't have a prescaler */
    n = 1;
    break;
  default:
    n = 0;
    break;
  }
  
  if (n > 10) {
    static const uint8_t tbl[] = {12, 18, 24, 36, 48};
    n = tbl[n - 11];
  }

  return n;
}

static PMU_STATUS PMU_SelectClockSourceInternal(PMU_CSM csm, uint32_t source)
{
  PMU_RESOURCES *rsc = &PMU_Resources;
  uint32_t n, cg_pmu;
  uint32_t prevented_irq;

  if (!rsc->info->init) {
    return PMU_ERROR;
  }
  switch (csm) {
  case PMU_CSM_MAIN:
  case PMU_CSM_UART0:
  case PMU_CSM_UART1:
  case PMU_CSM_UART2:
    switch (source) {
    case PMU_CLOCK_SOURCE_SIOSC4M:
      n = 0;
      break;
    case PMU_CLOCK_SOURCE_OSC12M:
      n = 1;
      break;
    case PMU_CLOCK_SOURCE_PLL:
      n = 2;
      break;
    case PMU_CLOCK_SOURCE_ADPLL:
      n = 3;
      break;
    case PMU_CLOCK_SOURCE_32K:
      n = 4;
      break;
    default:
      return PMU_ERROR;
    }
    break;
  case PMU_CSM_RTC:
    switch (source) {
    case PMU_CLOCK_SOURCE_OSC32K:
      n = 0;
      break;
    case PMU_CLOCK_SOURCE_SIOSC32K:
      n = 1;
      break;
    default:
      return PMU_ERROR;
    }
    break;
  case PMU_CSM_CPUTRC:
    switch (source) {
    case PMU_CLOCK_SOURCE_SIOSC4M:
      n = 0;
      break;
    case PMU_CLOCK_SOURCE_OSC12M:
      n = 1;
      break;
    default:
      return PMU_ERROR;
    }
    break;
  case PMU_CSM_ADCC12:
  case PMU_CSM_ADCC24:
    switch (source) {
    case PMU_CLOCK_SOURCE_SIOSC4M:
      n = 0;
      break;
    case PMU_CLOCK_SOURCE_OSC12M:
      n = 1;
      break;
    case PMU_CLOCK_SOURCE_PLL:
      n = 2;
      break;
    case PMU_CLOCK_SOURCE_ADPLL:
      n = 3;
      break;
    case PMU_CLOCK_SOURCE_OSC12M3:
      n = 5;
      break;
    case PMU_CLOCK_SOURCE_PLL3:
      if (source == PMU_CSM_ADCC12) {
        n = 6;
      } else {
        /* PMU_CSM_ADCC24 */
        return PMU_ERROR;
      }
      break;
    case PMU_CLOCK_SOURCE_PLL9:
      n = 7;
      break;
    default:
      return PMU_ERROR;
    }
    break;
  case PMU_CSM_CPUST:
    switch (source) {
    case PMU_CLOCK_SOURCE_SIOSC4M:
      n = 0;
      break;
    case PMU_CLOCK_SOURCE_OSC12M:
      n = 1;
      break;
    case PMU_CLOCK_SOURCE_32K:
      n = 4;
      break;
    default:
      return PMU_ERROR;
    }
    break;
  case PMU_CSM_PMULV:
    if (source == PMU_CLOCK_SOURCE_SIOSC4M) {
      n = 0;
    } else {
      return PMU_ERROR;
    }
    break;
  case PMU_CSM_USB:
    switch (source) {
    case PMU_CLOCK_SOURCE_PLL:
      n = 0;
      break;
    case PMU_CLOCK_SOURCE_ADPLL:
      n = 1;
      break;
    default:
      return PMU_ERROR;
    }
    break;
  default:
    return PMU_ERROR;
  }

  if (source == PMU_CLOCK_SOURCE_32K) {
    pmulv->CG_OFF_PA_b.CG_32KOSC_pmulv               = 1;
    pmulv->CG_OFF_REFCLK_b.CG_ref32kclk_pmulv_refclk = 1;
  }

  switch (csm) {
  case PMU_CSM_MAIN:
    prevented_irq = PMU_PreventRTCirq();
    pmulv->CSM_MAIN = n;
    PMU_RestoreRTCirq(prevented_irq);
    /* update SystemCoreClock */
    _SystemCoreClockUpdate();
    break;
  case PMU_CSM_RTC:
    cg_pmu = pmulv->CG_OFF_PA_b.CG_32KOSC_pmulv;
    pmulv->CG_ON_PA_b.CG_rtcclk_rtchv_rtcclk  = 1;
    pmulv->CG_ON_PA_b.CG_32KOSC_pmulv         = 1;
    pmulv->CSM_RTC                            = n;
    pmulv->CG_OFF_PA_b.CG_rtcclk_rtchv_rtcclk = 1;
    pmulv->CG_OFF_PA_b.CG_32KOSC_pmulv        = cg_pmu;
    break;
  case PMU_CSM_CPUTRC:
    pmulv->CSM_CPUTRC = n;
    break;
  case PMU_CSM_CPUST:
    pmulv->CSM_CPUST = n;
    break;
  case PMU_CSM_PMULV:
    /* DO NOTHING */
    break;
  case PMU_CSM_UART0:
    pmulv->CSM_UART0 = n;
    break;
  case PMU_CSM_UART1:
    pmulv->CSM_UART1 = n;
    break;
  case PMU_CSM_UART2:
    pmulv->CSM_UART2 = n;
    break;
  case PMU_CSM_USB:
    pmulv->CSM_USBI  = n;
    break;
  case PMU_CSM_ADCC12:
    PMU_SetPrescaler(PMU_CD_ADCC12, 0);
    pmulv->CSM_ADCC12A = n;
    break;
  case PMU_CSM_ADCC24:
    PMU_SetPrescaler(PMU_CD_ADCC24, 0);
    pmulv->CSM_ADCC24A = n;
    break;
  default:
    /* DO NOTHING */
    break;
  }

  return PMU_OK;
}

/**
 * @brief Set clock source selector
 * @param[in] csm Clock source selector
 * @param[in] source Clock source
 * @return \ref PMU_STATUS
 */
static PMU_STATUS PMU_SelectClockSource(PMU_CSM csm, PMU_CLOCK_SOURCE source)
{
  return PMU_SelectClockSourceInternal(csm, source);
}

/**
 * @brief Get clock source selector
 * @param[in] csm Clock source selector
 * @return \ref PMU_CLOCK_SOURCE
 */
static uint32_t PMU_GetClockSourceSelector(PMU_CSM csm)
{
  static const uint32_t tbl[] = {
    PMU_CLOCK_SOURCE_SIOSC4M,
    PMU_CLOCK_SOURCE_OSC12M,
    PMU_CLOCK_SOURCE_PLL,
    PMU_CLOCK_SOURCE_ADPLL,
    PMU_CLOCK_SOURCE_32K,
    PMU_CLOCK_SOURCE_OSC12M3,
    PMU_CLOCK_SOURCE_PLL3, 
    PMU_CLOCK_SOURCE_PLL9
  };
  static const PMU_CLOCK_SOURCE tbl_rtc[] = {
    PMU_CLOCK_SOURCE_OSC32K,
    PMU_CLOCK_SOURCE_SIOSC32K
  };
  static const PMU_CLOCK_SOURCE tbl_usb[] = {
    PMU_CLOCK_SOURCE_PLL,
    PMU_CLOCK_SOURCE_ADPLL
  };
  switch (csm) {
  case PMU_CSM_MAIN:
    return tbl[pmulv->CSM_MAIN_b.CSMSEL_MAIN];
  case PMU_CSM_RTC:
    return tbl_rtc[pmulv->CSM_RTC_b.CSMSEL_RTC];
  case PMU_CSM_CPUTRC:
    return tbl[pmulv->CSM_CPUTRC_b.CSMSEL_CPUTRC];
  case PMU_CSM_CPUST:
    return tbl[pmulv->CSM_CPUST_b.CSMSEL_CPUST];
  case PMU_CSM_PMULV:
    return PMU_CLOCK_SOURCE_SIOSC4M;
  case PMU_CSM_UART0:
    return tbl[pmulv->CSM_UART0_b.CSMSEL_UART0];
  case PMU_CSM_UART1:
    return tbl[pmulv->CSM_UART1_b.CSMSEL_UART1];
  case PMU_CSM_UART2:
    return tbl[pmulv->CSM_UART2_b.CSMSEL_UART2];
  case PMU_CSM_USB:
    return tbl_usb[pmulv->CSM_USBI_b.CSMSEL_USBI];
  case PMU_CSM_ADCC12:
    return tbl[pmulv->CSM_ADCC12A_b.CSMSEL_ADCC12A];
  case PMU_CSM_ADCC24:
    return tbl[pmulv->CSM_ADCC24A_b.CSMSEL_ADCC24A];
  default:
    return PMU_CLOCK_SOURCE_INVALID;
  }
}

static PMU_CSM Cd2Csm(PMU_CD clockDomain)
{
  static const PMU_CSM tbl[] = {
    PMU_CSM_MAIN,
    PMU_CSM_MAIN,
    PMU_CSM_MAIN,
    PMU_CSM_MAIN,
    PMU_CSM_MAIN,
    PMU_CSM_MAIN,
    PMU_CSM_RTC,
    PMU_CSM_CPUTRC,
    PMU_CSM_CPUST,
    PMU_CSM_PMULV,
    PMU_CSM_UART0,
    PMU_CSM_UART1,
    PMU_CSM_UART2,
    PMU_CSM_USB,
    PMU_CSM_ADCC12,
    PMU_CSM_ADCC24
  };
  
  if ((uint32_t) clockDomain >= PMU_CD_INVALID) {
    return PMU_CSM_INVALID;
  } else {
    return tbl[clockDomain];
  }
}

/**
 * @brief Get frequency of a clock domain
 * @param[in] clockDomain clock domain
 * @return frequency (Hz)
 */
static uint32_t PMU_GetFrequency(PMU_CD clockDomain)
{
  PMU_RESOURCES *rsc = &PMU_Resources;
  PMU_CSM csm;
  uint32_t source;
  uint32_t freq = 0;
  uint32_t divisor;
  
  if (!rsc->info->init) {
    return 0;
  }
  
  if ((uint32_t)clockDomain >= PMU_CD_INVALID) {
    return 0;
  }
  
  /* TODO: check clock gating */
  divisor = PMU_GetPrescaler(clockDomain);
  if (divisor == 0) {
    return 0;
  }
  if ((clockDomain == PMU_CD_PPIER0) || (clockDomain == PMU_CD_PPIER1) || (clockDomain == PMU_CD_PPIER2) || (clockDomain == PMU_CD_USBB)) {
    divisor *= PMU_GetPrescaler(PMU_CD_MPIER);
  }

  csm = Cd2Csm(clockDomain);
  source = PMU_GetClockSourceSelector(csm);
  
  if (PMU_GetClockSourceStateInternal(source) != PMU_CLOCK_SOURCE_STATE_RUNNING) {
    return 0;
  } else {
    switch (source) {
    case PMU_CLOCK_SOURCE_OSC32K:
    case PMU_CLOCK_SOURCE_SIOSC32K:
    case PMU_CLOCK_SOURCE_32K:
      freq = 32768; /* 32.768kHz */
      break;
    case PMU_CLOCK_SOURCE_SIOSC4M:
      freq = 4000000; /* 4MHz */
      break;
    case PMU_CLOCK_SOURCE_ADPLL:
      freq = 47972352; /* 47.972352MHz = 32.768kHz * 1464 */
      break;
    case PMU_CLOCK_SOURCE_OSC12M:
    case PMU_CLOCK_SOURCE_OSC12M3:
      freq = 12000000; /* 12MHz */
      break;
    case PMU_CLOCK_SOURCE_PLL:
    case PMU_CLOCK_SOURCE_PLL3:
    case PMU_CLOCK_SOURCE_PLL9:
      freq = rsc->info->pll_freq;
      break;
    default:
      freq = 0;
      break;
    }  
  }
  return freq / divisor;
}

/**
 * @brief Set PowerDomain mode
 * @param[in] powerDomain PowerDomain
 * @param[in] mode PowerDomain mode (ON/OFF/RETENTION/WAIT)
 * @return \ref PMU_STATUS
 */
static PMU_STATUS PMU_SetPowerDomainState(PMU_PD powerDomain, PMU_PD_MODE mode)
{
  PMU_RESOURCES *rsc = &PMU_Resources;
  uint32_t n;
  uint32_t ctrl_value;
  PMU_INFO      *info;

  info = rsc->info;
  if (!info->init) {
    return PMU_ERROR;
  }
  
  if ((uint32_t)mode > PMU_PD_MODE_WAIT) {
    return PMU_ERROR;
  }
  if (mode == PMU_PD_MODE_WAIT) {
    if ((powerDomain < PMU_PD_SRAM0) || (powerDomain > PMU_PD_SRAM2)) {
      return PMU_ERROR;
    }
  }

  n = GetPowerDomainCtrlBit(powerDomain);
  if (n == 0xffffffff) {
    return PMU_ERROR;
  }

  if ((PMU_PD_FLASH == powerDomain) && (mode == PMU_PD_MODE_OFF) &&
      (info->module_enabled[PMU_MODULE_SPIC])) {
    PMU_EnableModuleOff(PMU_MODULE_SPIC);
  }

  while (BB_PMULV_VOLTAGE_START() != 0) {
    /* DO NOTHING */
  }
  pmulv->POWERDOMAIN_CTRL_MODE =
    (pmulv->POWERDOMAIN_CTRL_MODE & ~(3u << n)) | (mode << n);
  ctrl_value = 1u << (n >> 1);
  pmulv->POWERDOMAIN_CTRL = ctrl_value;
  while ((pmulv->POWERDOMAIN_CTRL & ctrl_value) != 0) {
    /* DO NOTHING */
  }

  return PMU_OK;
}

/**
 * @brief Set PowerDomain mode in low power consumption mode
 * @param[in] powerDomain PowerDomain
 * @param[in] modeWait PowerDomain mode in WAIT mode
 * @param[in] modeWaitRetention PowerDomain mode in WAIT_RETENTION mode
 * @param[in] modeRetention PowerDomain mode in RETENTION mode
 * @return \ref PMU_STATUS
 */
static PMU_STATUS PMU_SetPowerDomainStateLowPowerMode(
  PMU_PD powerDomain,
  PMU_PD_MODE modeWait,
  PMU_PD_MODE modeWaitRetention,
  PMU_PD_MODE modeRetention)
{
  PMU_RESOURCES *rsc = &PMU_Resources;
  uint32_t n;
  
  if (!rsc->info->init) {
    return PMU_ERROR;
  }
  
  n = GetPowerDomainCtrlBit(powerDomain);
  if (n == 0xffffffff) {
    return PMU_ERROR;
  }
  
  if ((powerDomain >= PMU_PD_SRAM0) && (powerDomain <= PMU_PD_SRAM2)) {
    if (((uint32_t)modeWait > PMU_PD_MODE_WAIT)
        || ((uint32_t)modeWaitRetention > PMU_PD_MODE_WAIT)
        || ((uint32_t)modeRetention > PMU_PD_MODE_WAIT)) {
      return PMU_ERROR;
    }
    pmulv->POWERDOMAIN_CTRL_MODE_FOR_WAIT =
      (pmulv->POWERDOMAIN_CTRL_MODE_FOR_WAIT & ~(3u << n)) | (modeWait << n);
  } else {
    if (((uint32_t)modeWaitRetention > PMU_PD_MODE_RETENTION)
        || ((uint32_t)modeRetention > PMU_PD_MODE_RETENTION)) {
      return PMU_ERROR;
    }
  }
  
  pmulv->POWERDOMAIN_CTRL_MODE_FOR_WRET =
    (pmulv->POWERDOMAIN_CTRL_MODE_FOR_WRET & ~(3u << n)) | (modeWaitRetention << n);
  pmulv->POWERDOMAIN_CTRL_MODE_FOR_RET =
    (pmulv->POWERDOMAIN_CTRL_MODE_FOR_RET & ~(3u << n)) | (modeRetention << n);
  
  return PMU_OK;
}

/**
 * @brief Change VoltageMode
 * @param[in] mode VoltageMode
 * @return \ref PMU_STATUS
 */
static PMU_STATUS PMU_SetVoltageMode(PMU_VOLTAGE_MODE mode)
{
  PMU_RESOURCES *rsc = &PMU_Resources;
  uint32_t voltage_mode_val;

  if (!rsc->info->init) {
    return PMU_ERROR;
  }

  voltage_mode_val = (uint32_t)mode;
  if (voltage_mode_val > 3) {
    return PMU_ERROR;
  }

  PMU_TrimSIOSC4M(mode);
  while (pmulv->POWERDOMAIN_CTRL != 0) {
    /* DO NOTHING */
  }
  pmulv->MOVE_POWER_VOLTAGE_MODE_b.VOLTAGE_MODE = voltage_mode_val;
  pmulv->MOVE_POWER_VOLTAGE_MODE_b.POWER_MODE = 0;
  BB_PMULV_VOLTAGE_START() = 1;
  while (BB_PMULV_VOLTAGE_START() != 0) {
    /* DO NOTHING */
  }

  return PMU_OK;
}

/**
 * @brief Change PowerMode
 * @param[in] mode PowerMode
 * @return \ref PMU_STATUS
 */
static PMU_STATUS PMU_SetPowerMode(PMU_POWER_MODE mode)
{
  PMU_RESOURCES *rsc = &PMU_Resources;
  PMU_INFO      *info;
  uint32_t power_mode_val;
  uint32_t sleepdeep;
  uint32_t e26_powermode_sleep_cg_on_backup;
  uint32_t prevented_irq;
  uint32_t fast_mode;
  uint32_t spifrstn, csm_main;
  uint32_t csm_cpust;
  uint32_t csm_uart0, csm_uart1, csm_uart2;
  uint32_t pm1_latch;
  uint32_t cg_off_pa;
  uint32_t wmsk, sngl, iev, val;
  uint32_t is0,is1,um0,um3,sngl0,sngl3,iev0,iev3;
  uint32_t wk, num;
  uint32_t g0dat, g3dat;
  uint32_t iser[3];
  PMU_STATUS ret;

  info = rsc->info;
  if (!info->init) {
    return PMU_ERROR;
  }

  switch (mode) {
  case PMU_POWER_MODE_ACTIVE:
    __WFI();
    return PMU_OK;
  case PMU_POWER_MODE_SLEEP0:
    power_mode_val = 1;
    sleepdeep = 0;
    break;
  case PMU_POWER_MODE_SLEEP1:
    power_mode_val = 2;
    sleepdeep = 0;
    break;
  case PMU_POWER_MODE_SLEEP2:
    power_mode_val = 3;
    sleepdeep = 0;
    break;
  case PMU_POWER_MODE_WAIT:
    power_mode_val = 4;
    sleepdeep = 1;
    break;
  case PMU_POWER_MODE_WAIT_RETENTION:
    power_mode_val = 5;
    sleepdeep = 1;
    break;
  case PMU_POWER_MODE_RETENTION:
    power_mode_val = 7;
    sleepdeep = 1;
    break;
  case PMU_POWER_MODE_RTC:
    power_mode_val = 8;
    sleepdeep = 1;
    break;
  case PMU_POWER_MODE_STOP:
    power_mode_val = 9;
    sleepdeep = 1;
    break;
  case PMU_POWER_MODE_SLEEPDEEP0:
    power_mode_val = 1;
    sleepdeep = 1;
    mode = PMU_POWER_MODE_SLEEP0;
    break;
  case PMU_POWER_MODE_SLEEPDEEP1:
    power_mode_val = 2;
    sleepdeep = 1;
    mode = PMU_POWER_MODE_SLEEP1;
    break;
  case PMU_POWER_MODE_SLEEPDEEP2:
    power_mode_val = 2;
    sleepdeep = 1;
    mode = PMU_POWER_MODE_SLEEP2;
    break;
  default:
    return PMU_ERROR;
  }

  prevented_irq = __get_PRIMASK();
  __disable_irq();

  ret = PMU_OK;
  /* workaround for eratta E26 */
  if (mode == PMU_POWER_MODE_SLEEP0) {
    e26_powermode_sleep_cg_on_backup = pmulv->POWERMODE_SLEEP_CG_ON;
    pmulv->POWERMODE_SLEEP_CG_ON = 0;
  }

  fast_mode = 0;
  pmulv->MOVE_POWER_VOLTAGE_MODE_b.FAST_MODE  = fast_mode;
  pmulv->MOVE_POWER_VOLTAGE_MODE_b.POWER_MODE = power_mode_val;
  if (sleepdeep) {
    SCB->SCR |= (1u << 2);
  } else {
    SCB->SCR &= ~(1u << 2);
  }
  wmsk = 0;
  sngl = 0;
  iev  = 0;
  if (mode >= PMU_POWER_MODE_SLEEP2) {
    /* make wakeup interrupt masks for gpio handler */
    if (pmulv->PRESCAL_MAIN_b.PSSEL_CD_PPIER0) {
      if (info->module_enabled[PMU_MODULE_GPIO0]) {
        is0   = pmulv->IRQ_SETTING_0;
        PMU_MakeWakeupMasks(is0, gpio0, &um0, &sngl0, &iev0);
        wmsk = um0;
        sngl = sngl0;
        iev  = iev0;
      }
      if (info->module_enabled[PMU_MODULE_GPIO3]) {
        is1  = pmulv->IRQ_SETTING_1;
        // adjust gpio 30 position
        is1   = ((is1 & 0x00030000) << 8) | (is1 & 0x00003333);
        PMU_MakeWakeupMasks(is1, gpio3, &um3, &sngl3, &iev3);
        wmsk |= um3   << 24;
        sngl |= sngl3 << 24;
        iev  |= iev3  << 24;
      }
    }
  }

  if (mode <= PMU_POWER_MODE_SLEEP2) {
    /* SLEEP0, SLEEP1, SLEEP2 */
    usleep(1);
  } else {
    /* disable non-wakeup interrupts. */
    iser[0]  = NVIC->ISER[0];
    iser[1]  = NVIC->ISER[1];
    iser[2]  = NVIC->ISER[2];
    NVIC->ICER[0] = 0xFFFFFFFF;
    NVIC->ICER[1] = 0xFFFFFFFF;
    NVIC->ICER[2] = 0x00000FFF;

    /* WAIT, WAIT-RET, RETENTION, RTC, STOP */
    while (0 != pmulv->POWERDOMAIN_CTRL) {
      /* DO NOTHING */
    }
    while (0 != BB_PMULV_VOLTAGE_START()) {
      /* DO NOTHING */
    }
    pmulv->POWERDOMAIN_CTRL_MODE = pmulv->POWERDOMAIN_CTRL_STATUS;

    if (mode == PMU_POWER_MODE_WAIT_RETENTION) {
      pm1_latch = pmulv->CTRL_IO_AON_1;
      pmulv->CTRL_IO_AON_1 = 1;
    } else if (mode == PMU_POWER_MODE_RETENTION) {
      /* SPIC I/F reset on  */
      spifrstn = BB_PMULV_SPIC_OFF_SPIFRSTN();
      BB_PMULV_SPIC_ON_SPIFRSTN() = 1;
    }
    if (mode >= PMU_POWER_MODE_RETENTION) {
      /* RETENTION, RTC, STOP */
      /* Changes clock sources to SIOSC4M if it needs */
      csm_main = pmulv->CSM_MAIN;
      if ((4 == csm_main) || (3 == csm_main)) {
        pmulv->CSM_MAIN = 0;
      }
      csm_cpust = pmulv->CSM_CPUST;
      if ((4 == csm_cpust) || (3 == csm_cpust)) {
        pmulv->CSM_CPUST = 0;
      }
      csm_uart0 = pmulv->CSM_UART0;
      if ((4 == csm_uart0) || (3 == csm_uart0)) {
        pmulv->CSM_UART0 = 0;
      }
      csm_uart1 = pmulv->CSM_UART1;
      if ((4 == csm_uart1) || (3 == csm_uart1)) {
        pmulv->CSM_UART1 = 0;
      }
      csm_uart2 = pmulv->CSM_UART2;
      if ((4 == csm_uart2) || (3 == csm_uart2)) {
        pmulv->CSM_UART2 = 0;
      }
      /* Stop ADPLL */
      BB_PMULV_CONFIG_ADPLL_EN() = 0;
      pmulv->CG_ON_REFCLK        = 0x00010001;

      PMU_WaitUntilDoneRtc(info);

      /* Stop 32K clock source */
      cg_off_pa = pmulv->CG_OFF_PA;
      BB_PMULV_CG_ON_PA_32K() = 1;
      if (mode == PMU_POWER_MODE_STOP) {
        pmulv->CG_ON_PA                 = 0x00010101;
        BB_PMULV_OSC32K_BOOST_DISABLE() = 0;
        BB_PMULV_OSC32K_EN()            = 0;
        BB_PMULV_SIOSC32K_EN()          = 0;
      }
    }
    /* workaround for E36 */
    BB_PMULV_DCG_PMULV_LV0CLK() = 0;
  }
  if ((mode >= PMU_POWER_MODE_SLEEP2) && (mode <= PMU_POWER_MODE_WAIT_RETENTION)) {
    /* SLEEP2, WAIT, WAIT-RET */
    PMU_WaitUntilDoneRtc(info);
  }

  if (mode >= PMU_POWER_MODE_SLEEP1) {
    /* SLEEP1, SLEEP2, WAIT, WAIT-RET, RETENTION, RTC, STOP */
    while (mpier->BUS_BUSY != 0) {
      /* DO NOTHING */
    }
  }
  __WFI();
  if (sngl) {
    g0dat = gpio0->GPIODATA[255];
    g3dat = gpio3->GPIODATA[ 79];
    val   = g0dat | (g3dat << 24);
  }
  if (mode >= PMU_POWER_MODE_RTC) {
    /* The power mode transition has been cancelled. */
    ret = PMU_ERROR_IGNORED;
    /* Revert clock source settings if it needs. */
    if (cg_off_pa) {
      /* Start 32K clock source */
      if (mode == PMU_POWER_MODE_STOP) {
        if (info->clock_state[PMU_CLOCK_SOURCE_OSC32K] == PMU_CLOCK_SOURCE_STATE_RUNNING) {
          BB_PMULV_OSC32K_EN()            = 1;
          usleep(1000000);
          BB_PMULV_OSC32K_BOOST_DISABLE() = 1;
          usleep(2000);
        }
        if (info->clock_state[PMU_CLOCK_SOURCE_SIOSC32K] == PMU_CLOCK_SOURCE_STATE_RUNNING) {
          pmulv->CONFIG_SiOSC32K_b.SiOSC32K_EN = 1;
          usleep(500);
        }
      }
      pmulv->CG_OFF_PA = cg_off_pa;
    }
  }
  if (mode >= PMU_POWER_MODE_WAIT) {
    /* When the power mode transition has been cancelled, any clock source is running. */
    if (0 == pmulv->PSW_PLL) {
      info->clock_state[PMU_CLOCK_SOURCE_PLL]    = PMU_CLOCK_SOURCE_STATE_STOPPED;
    } else {
      ret = PMU_ERROR_IGNORED;
    }
    if (0 == BB_PMULV_OSC12M_EN()) {
      info->clock_state[PMU_CLOCK_SOURCE_OSC12M] = PMU_CLOCK_SOURCE_STATE_STOPPED;
    } else {
      ret = PMU_ERROR_IGNORED;
    }
    if (0 == pmulv->PSW_ADPLL) {
      info->clock_state[PMU_CLOCK_SOURCE_ADPLL]  = PMU_CLOCK_SOURCE_STATE_STOPPED;
    } else {
      ret = PMU_ERROR_IGNORED;
    }
    if (ret == PMU_OK) {
      SystemCoreClock = 4000000;
    } else if (mode >= PMU_POWER_MODE_RETENTION) {
      /* Revert clock source settings if it needs. */
      if (info->clock_state[PMU_CLOCK_SOURCE_ADPLL] == PMU_CLOCK_SOURCE_STATE_RUNNING) {
        BB_PMULV_CG_OFF_REFCLK_ADPLL()  = 1;
        /* Start ADPLL */
        BB_PMULV_CONFIG_ADPLL_EN()      = 1;
        BB_PMULV_CONFIG_ADPLL_NTWMODE() = 1;
        usleep(100);
        BB_PMULV_CONFIG_ADPLL_NTWMODE() = 0;
        usleep(2000);
      }
      if ((4 == csm_cpust) || (4 == csm_uart0) ||
          (4 == csm_uart1) || (4 == csm_uart2) || (4 == csm_main)) {
        BB_PMULV_CG_OFF_REFCLK_PMULV() = 1;
      }
      pmulv->CSM_CPUST = csm_cpust;
      pmulv->CSM_UART0 = csm_uart0;
      pmulv->CSM_UART1 = csm_uart1;
      pmulv->CSM_UART2 = csm_uart2;
      pmulv->CSM_MAIN  = csm_main;
    }
    while (pmulv->STATUS_LVPWR != 0) {
      /* DO NOTHING */
    }
  }
  /* restore states */
  if (mode == PMU_POWER_MODE_SLEEP0) {
    pmulv->POWERMODE_SLEEP_CG_ON = e26_powermode_sleep_cg_on_backup;
  }
  pmulv->MOVE_POWER_VOLTAGE_MODE_b.POWER_MODE = 0;
  SCB->SCR &= ~(1u << 2);
  if (mode >= PMU_POWER_MODE_WAIT) {
    /* workaround for E36 */
    BB_PMULV_DCG_PMULV_LV0CLK() = 1;
    if (mode == PMU_POWER_MODE_WAIT_RETENTION) {
      pmulv->CTRL_IO_AON_1 = pm1_latch;
    } else if (mode == PMU_POWER_MODE_RETENTION) {
      BB_PMULV_SPIC_OFF_SPIFRSTN() = spifrstn;
    }
    if (fast_mode) {
      /* WAIT, WAIT-RET, RETENTION, RTC, STOP */
      PMU_TrimSIOSC4M(PMU_VOLTAGE_MODE_D);
    }
    /* restore enabled interrupt settings */
    NVIC->ISER[0] = iser[0];
    NVIC->ISER[1] = iser[1];
    NVIC->ISER[2] = iser[2];
  }

  /* forward wakeup events to GPIO events. */
  wk  = pmulv->WAKEUP_STATUS;
  info->wakeup_status = wk;
  wk  = (wk & 0xFF) | ((wk & 0xF00) << 16) | ((wk & 0x1000) << 18);
  wk &= wmsk;
  if (wk & sngl) {
    wk &= ~((val ^ iev) & sngl);  /* deassert not target edge */
  }
  while (wk) {
    num  = 31 - __CLZ(wk);
    wk  -= 1 << num;
    NVIC_SetPendingIRQ((IRQn_Type)num);
  }

  if (!prevented_irq) {
    __enable_irq();
  }

  return ret;
}

/**
 * @brief Turn on/off on-chip sensor power
 * @param[in] sensor kind of sensor
 * @param[in] enable 0:off 1:on
 * @return \ref PMU_STATUS
 */
static PMU_STATUS PMU_EnableSensor(PMU_SENSOR sensor, uint32_t enable)
{
  PMU_RESOURCES *rsc = &PMU_Resources;
  
  if (!rsc->info->init) {
    return PMU_ERROR;
  }
  
  if (enable == 1) {
    /* power on */
    switch (sensor) {
    case PMU_SENSOR_ACC:
      pmulv->PSW_HARDMACRO_b.PSW_ACC_VDDCW = 1;
      usleep(110);
      pmulv->PSW_HARDMACRO_b.PSW_ACC_VDDCS = 1;
      break;
    case PMU_SENSOR_GYRO:
      pmulv->PSW_HARDMACRO_b.PSW_GYRO_VDDCW = 1;
      usleep(70);
      pmulv->PSW_HARDMACRO_b.PSW_GYRO_VDDCS = 1;
      break;
    case PMU_SENSOR_MAG:
      pmulv->PSW_HARDMACRO_b.PSW_MAG_VDDCW = 1;
      usleep(70);
      pmulv->PSW_HARDMACRO_b.PSW_MAG_VDDCS = 1;
      break;
    default:
      return PMU_ERROR;
    }
  } else if (enable == 0) {
    /* power off */
    switch (sensor) {
    case PMU_SENSOR_ACC:
      pmulv->PSW_HARDMACRO_b.PSW_ACC_VDDCS = 0;
      pmulv->PSW_HARDMACRO_b.PSW_ACC_VDDCW = 0;
      break;
    case PMU_SENSOR_GYRO:
      pmulv->PSW_HARDMACRO_b.PSW_GYRO_VDDCS = 0;
      pmulv->PSW_HARDMACRO_b.PSW_GYRO_VDDCW = 0;
      break;
    case PMU_SENSOR_MAG:
      pmulv->PSW_HARDMACRO_b.PSW_MAG_VDDCS = 0;
      pmulv->PSW_HARDMACRO_b.PSW_MAG_VDDCW = 0;
      break;
    default:
      return PMU_ERROR;
    }
  } else {
    return PMU_ERROR;
  }
  
  return PMU_OK;
}

/**
 * @brief Configure WAKEUP condition
 * @param[in] factor wakeup factor
 * @param[in] interrupt condition
 * @return \ref PMU_STATUS
 */
static PMU_STATUS PMU_ConfigureWakeup(uint32_t factor, PMU_WAKEUP_EVENT event)
{
  static const uint8_t e[6] = {0, 8, 4, 1, 2, 3};
  PMU_RESOURCES *rsc;
  uint32_t setting;
  uint32_t z, sft, msk, st;

  rsc = &PMU_Resources;
  if (!rsc->info->init) {
    return PMU_ERROR;
  }

  if ((uint32_t)PMU_WAKEUP_EVENT_EDGE_BOTH < (uint32_t)event) {
    return PMU_ERROR;
  }
  if ((0 == factor) || (0 != (factor & ~PMU_WAKEUP_FACTOR_MASK))) {
    return PMU_ERROR;
  }
  z = __CLZ(factor);
  if (0 != (factor - (0x80000000 >> z))) {
    return PMU_ERROR;
  }

  setting = e[event];
  if (factor <= PMU_WAKEUP_FACTOR_GPIO_7) {
    sft = (31 - z) * 4;
    msk = 0xFF << sft;
    st  = pmulv->IRQ_SETTING_0 & ~msk;

    pmulv->IRQ_SETTING_0 = st;
    pmulv->IRQ_STATUS    = factor;
    pmulv->IRQ_SETTING_0 = st | (setting << sft);
  } else if (factor <= PMU_WAKEUP_FACTOR_GPIO_30) {
    sft = (23 - z) * 4;
    msk = 0xFF << sft;
    st  = pmulv->IRQ_SETTING_1 & ~msk;

    pmulv->IRQ_SETTING_1 = st;
    pmulv->IRQ_STATUS    = factor;
    pmulv->IRQ_SETTING_1 = st | (setting << sft);
  } else {
    if (setting > 1) {
      return PMU_ERROR;
    }
    sft = (21 - z) * 4;
    msk = 0xFF << sft;
    st  = pmulv->IRQ_SETTING_1 & ~msk;

    pmulv->IRQ_SETTING_1 = st;
    pmulv->IRQ_STATUS    = factor;
    pmulv->IRQ_SETTING_1 = st | (setting << sft);
  }
  return PMU_OK;
}

/**
 * @brief Get WAKEUP factor
 * @return bitwise-OR of PMU_WAKEUP_FACTOR_*
 */
static uint32_t PMU_GetWakeupFactor(void)
{
  PMU_RESOURCES *rsc = &PMU_Resources;
  
  if (!rsc->info->init) {
    return 0;
  }
  
  return rsc->info->wakeup_status;
}

/**
 * @brief Configure behevior when BrownOut occurs
 * @param[in] mode behevior when BrownOut occurs
 * @return \ref PMU_STATUS
 */
static PMU_STATUS PMU_SetBrownOutMode(PMU_BROWN_OUT_MODE mode)
{
  PMU_RESOURCES *rsc = &PMU_Resources;
  uint32_t brownoutmode;
  
  if (!rsc->info->init) {
    return PMU_ERROR;
  }
  
  switch (mode) {
  case PMU_BROWN_OUT_IGNORE:
    brownoutmode = 0;
    break;
  case PMU_BROWN_OUT_RESET:
    brownoutmode = (1u << 31);
    break;
  case PMU_BROWN_OUT_NOTIFY_EXCEPT_RTC_STOP:
    brownoutmode = (1u << 31) | (1u << 5) | (1u << 0);
    break;
  case PMU_BROWN_OUT_NOTIFY:
    brownoutmode = (1u << 31) | (1u << 0);
    break;
  default:
    return PMU_ERROR;
  }
  pmulv->BROWNOUTMODE = brownoutmode;
  
  return PMU_OK;
}

/**
 * @brief Issue BrownOut reset
 */
static void PMU_BrownOutReset(void)
{
  PMU_RESOURCES *rsc = &PMU_Resources;
  
  if (!rsc->info->init) {
    return;
  }
  
  pmulv->BROWNOUTRESET_b.BOR = 1;
}

/**
 * @brief Get reset factor
 * @return reset factor
 */
static uint32_t PMU_GetResetFactor(void)
{
  PMU_RESOURCES *rsc = &PMU_Resources;
  return rsc->info->reset_factor;
}

/**
 * @brief Enable or disable slowing down CPU when SLEEP mode
 * @param[in] enable
 * @return \ref PMU_STATUS
 */
static PMU_STATUS PMU_SetCpuLowFrequencyOnSleep(bool enable)
{
  PMU_RESOURCES *rsc = &PMU_Resources;
  
  if (!rsc->info->init) {
    return PMU_ERROR;
  }
  pmulv->POWERMODE_SLEEP_PRESCAL = (enable != false);
  return PMU_OK;
}

/**
 * @brief Enable or disable OSC32KOUT
 * @param[in] enable
 * @return \ref PMU_STATUS
 */
static PMU_STATUS PMU_Enable32kOut(bool enable)
{
  PMU_RESOURCES *rsc = &PMU_Resources;
  
  if (!rsc->info->init) {
    return PMU_ERROR;
  }
  
  if (enable) {
    pmulv->CG_OFF_PA_b.CG_CLK32K_OUT = 1;
  } else {
    pmulv->CG_ON_PA_b.CG_CLK32K_OUT = 1;
  }
  return PMU_OK;
}

/**
 * @brief Enables or disables WAKEUP event
 * @param[in] factor wakeup factor
 * @param[in] enable specifies enable or disable for wakeup event.
 * @return \ref PMU_STATUS
 */
static PMU_STATUS PMU_EnableWakeup(uint32_t factor, bool enable)
{
  PMU_RESOURCES *rsc;
  uint32_t pos;

  rsc = &PMU_Resources;
  if (!rsc->info->init) {
    return PMU_ERROR;
  }

  if ((0 == factor) || (0 != (factor & ~PMU_WAKEUP_FACTOR_MASK))) {
    return PMU_ERROR;
  }
  pos = 31 - __CLZ(factor);
  if (0 != (factor - (1 << pos))) {
    return PMU_ERROR;
  }

  BB_PMULV_WAKEUP_EN(pos) = enable;

  return PMU_OK;
}

/**
 * @brief Gets current VoltageMode
 * @return \ref PMU_VOLTAGE_MODE
 */
static PMU_VOLTAGE_MODE PMU_GetVoltageMode(void)
{
  return (PMU_VOLTAGE_MODE)pmulv->MOVE_VOLTAGE_START_b.VMSTATUS;
}

void PMU_WKUP_IRQHandler(void)
{
  PMU_INFO         *info;
  PMU_RESOURCES *rsc = &PMU_Resources;
  uint32_t status;
  uint32_t s0, s1, m0, m1;

  info = rsc->info;
  info->wakeup_status = 0;
  while (0 != (status = pmulv->WAKEUP_STATUS)) {
    info->wakeup_status |= status;
    m0 = 0;
    if (status & 0x000000FF) {
      m0  = status;
      m0  = ((m0 & 0xF0) << 12) | (m0 & 0xF);
      m0  = ((m0 << 6) | m0) & 0x03030303;
      m0  = ((m0 << 3) | m0) & 0x11111111;
      m0  = 0x88888888 - m0;
      m0 ^= 0x88888888;

      s0 = pmulv->IRQ_SETTING_0;

      pmulv->IRQ_SETTING_0 = s0 & ~m0;
    }

    m1 = 0;;
    if (status & 0x00031F00) {
      m1 = status;
      m1  = ((m1 & 0x38000) >> 2) | (m1 & 0x1F00);
      m1  = ((m1 & 0xF000) << 4) | ((m1 & 0xF00) >> 8);
      m1  = ((m1 << 6) | m1) & 0x03030303;
      m1  = ((m1 << 3) | m1) & 0x11111111;
      m1  = 0x88888888 - m1;
      m1 ^= 0x88888888;

      s1 = pmulv->IRQ_SETTING_1;
      pmulv->IRQ_SETTING_1 = s1 & ~m1;
    }

    pmulv->IRQ_STATUS = status;

    if (m0) {
      pmulv->IRQ_SETTING_0 = s0;
    }
    if (m1) {
      pmulv->IRQ_SETTING_1 = s1;
    }
  }
}

TZ10XX_DRIVER_PMU Driver_PMU = {
  PMU_GetVersion,
  PMU_Initialize,
  PMU_Uninitialize,
  PMU_StartClockSource,
  PMU_StopClockSource,
  PMU_GetClockSourceState,
  PMU_SetPLLFrequency,
  PMU_SetPrescaler,
  PMU_GetPrescaler,
  PMU_SelectClockSource,
  PMU_GetFrequency,
  PMU_SetPowerDomainState,
  PMU_SetPowerDomainStateLowPowerMode,
  PMU_SetVoltageMode,
  PMU_SetPowerMode,
  PMU_EnableSensor,
  PMU_ConfigureWakeup,
  PMU_GetWakeupFactor,
  PMU_SetBrownOutMode,
  PMU_BrownOutReset,
  PMU_GetResetFactor,
  PMU_EnableModule,
  PMU_EnableModuleOnSleep,
  PMU_ResetOutputBuffer,
  PMU_RetainOutputBuffer,
  PMU_StandbyInputBuffer,
  PMU_ConfigureIOCell,
  PMU_SetCpuLowFrequencyOnSleep,
  PMU_Enable32kOut,
  PMU_EnableWakeup,
  PMU_GetVoltageMode,
};
