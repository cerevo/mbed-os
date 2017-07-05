/**
 * @file WDT_TZ10xx.c
 * @brief TZ10xx WDT driver
 * @date $Date:: 2014-07-22 15:32:30 +0900 #$
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */
#include "WDT_TZ10xx.h"
#include "TZ10xx.h"
#include "PMU_TZ10xx.h"

#if (RTE_WDT)

/* Driver Version */
#define TZ1000_WDT_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0)   /* driver version 0.1 */

/* API version */
#define TZ1000_WDT_API_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0)

/* Macros */
#define WDT_ENABLE 1
#define WDT_DISABLE 0
#define WDT_LOAD_VAL_MAX 0xFFFFFFFF
#define WDT_LOAD_VAL_MIN 0x00000001
#define WDT_LOCK_REG_LOCK 0x1
#define WDT_LOCK_REG_UNLOCK 0x1ACCE551 
#define WDT_INTR_MASK 0x1 /* Timeout Intr is the 0th bit of WDT_WDOGRIS register */
#define WDT_CTRL_REG_MASK 0x3 /* Bit-0: Intr enable, Bit-1: Reset enable */
#define WDT_DEFAULT_TIMEOUT 60 /* seconds */

/* WDT runtime resources */
typedef struct _WDT_INFO {
  bool init;
  ARM_POWER_STATE power_state;
  WDT_SignalEvent_t cb_event;
  uint32_t timeout;
  uint32_t load;
} WDT_INFO;

/* WDT resources */
typedef struct _WDT_RESOURCE {
  wdt_Type *wdtmr;
  WDT_INFO *info;
  IRQn_Type irq; /* IRQ number */  
  PMU_CD clockDomain;
} WDT_RESOURCE;


static WDT_INFO WDT_INFORMATION;
static WDT_RESOURCE WDT_RES = {
    wdt,
    &WDT_INFORMATION,
    WDT_IRQn,    
    PMU_CD_PPIER0,
};

/* Local Functions */
void WDT_ResetOn(void)
{
    Driver_PMU.EnableModule(PMU_MODULE_WDT, WDT_DISABLE);
    return;
}

void WDT_ResetOff(void)
{
    Driver_PMU.EnableModule(PMU_MODULE_WDT, WDT_ENABLE);
    return;
}


/* Driver API  Functions */

/**
  @brief        Get Driver Version.
  @return      \ref ARM_DRV_VERSION
*/
static ARM_DRIVER_VERSION WDT_GetVersion(void)
{
  static const ARM_DRIVER_VERSION DrvVersion = {
    TZ1000_WDT_API_VERSION,
    TZ1000_WDT_DRV_VERSION
  };
  
  return DrvVersion;
}

static WDT_CAPABILITIES WDT_GetCapabilities(void)
{
  static const WDT_CAPABILITIES capabilities = {
    1,  /* settimeout enabled */
    1,  /* system reset request signal enabled */
    1,  /* keep alive ping reply */
    0   /* reserved */
  };
  
  return capabilities;
}

/**
  @brief        Power control of WDT
  @return      \ref WDT_STATUS
*/
static WDT_STATUS WDT_PowerControl(ARM_POWER_STATE state)
{
    WDT_RESOURCE *rsc = &WDT_RES;

    /* Argument check */
    if(state > ARM_POWER_FULL)
    {
        return WDT_ERROR;
    }
    
    if (!rsc->info->init) 
    {
        return WDT_ERROR;
    }
    
    if (state == rsc->info->power_state)
    {
        return WDT_OK;
    }

    switch (state) 
    {
        case ARM_POWER_OFF:
            WDT_ResetOn();
            rsc->info->power_state = ARM_POWER_OFF;
            break;
          
        case ARM_POWER_LOW:
        case ARM_POWER_FULL:
            WDT_ResetOff();
            rsc->info->power_state = state;
            break;
          
        default:
            return WDT_ERROR;
    }
    
    return WDT_OK;
}

/**
  @brief        Initialise WDT
  @return      \ref WDT_STATUS
*/
static WDT_STATUS WDT_Initialize(WDT_SignalEvent_t cb_event)
{
    WDT_RESOURCE *rsc = &WDT_RES;
    WDT_STATUS sts;
    uint32_t clk_freq = Driver_PMU.GetFrequency(rsc->clockDomain);    

    /* Check callback */
    if(cb_event == NULL)
    {
        return WDT_CALLBACK_ERROR;
    }

    /* Check init status */
    if(rsc->info->init)
    {
        return WDT_ERROR;
    }

    /* Initialise run-time resources */
    rsc->info->timeout = WDT_DEFAULT_TIMEOUT;
    rsc->info->load = ((clk_freq/2) * WDT_DEFAULT_TIMEOUT) - 1;
    rsc->info->cb_event = cb_event;
    
      
    /* Enable WDT IRQ */
    NVIC_ClearPendingIRQ(rsc->irq);
    NVIC_EnableIRQ(rsc->irq);

    rsc->info->init = true;  

    sts = WDT_PowerControl(ARM_POWER_LOW);
    if(sts != WDT_OK)
    {
        return sts;
    }
    
    return WDT_OK;
}

/**
  @brief        Uninitialise WDT
  @return      \ref WDT_STATUS
*/
static WDT_STATUS WDT_Uninitialize(void)
{
    WDT_RESOURCE *rsc = &WDT_RES;
    WDT_STATUS sts;
    
    /* Check init status */
    if(!rsc->info->init)
    {
        return WDT_OK; /* Uninitialize already done */
    }
    
    /* Disable WDT IRQ */
    NVIC_ClearPendingIRQ(rsc->irq);
    NVIC_DisableIRQ(rsc->irq);
  
    sts = WDT_PowerControl(ARM_POWER_OFF);
    if(sts != WDT_OK)
    {
        return sts;
    }
    
    rsc->info->init = false;
  
    return WDT_OK;
}

/**
  @brief        Set WDT Interrupt Timeout
  @return      \ref WDT_STATUS
*/
static WDT_STATUS WDT_SetTimeoutInterval(uint32_t timeout)
{
    WDT_RESOURCE *rsc = &WDT_RES;
    uint32_t clk_freq = Driver_PMU.GetFrequency(rsc->clockDomain);
    uint32_t max_timeout;
    uint64_t max_load = (uint64_t)WDT_LOAD_VAL_MAX + 1 ;

    /* Check for init status */
    if (!rsc->info->init) 
    {
        return WDT_ERROR;
    }

    /* Cannot call in ARM_POWER_OFF */
    if(rsc->info->power_state == ARM_POWER_OFF)
    {
        return WDT_ERROR;    
    }

    /* Timeout cannot be set as 0 */
    if(timeout == 0)
    {
        return WDT_ARGUMENT_ERROR;
    }

    /* Formula to calculate Timeout: timeout = (load_val + 1) x effective_clock_period
     *  where, effective_clock_period = 1/clk_freq
     */
     
    /* Calculate max timeout that can be set */
    max_timeout = (uint32_t)(max_load/clk_freq);

    /* Check if timeout set by user is greater than max_timeout */
    if(timeout > max_timeout)
    {
        return WDT_ARGUMENT_ERROR;
    }

    /* Save in resource struct */
    rsc->info->timeout = timeout;

    return WDT_OK;
}

/**
  @brief        Get WDT Timeout value
  @return      \ref WDT_STATUS
*/
static WDT_STATUS WDT_GetTimeoutInterval(uint32_t *timeout)
{
    WDT_RESOURCE *rsc = &WDT_RES;

    /* Check arguments */
    if(NULL == timeout)
    {
        return WDT_ARGUMENT_ERROR;
    }
    
    /* Check for init status */
    if (!rsc->info->init) 
    {
        return WDT_ERROR;
    }

    /* Cannot call in ARM_POWER_OFF */
    if(rsc->info->power_state == ARM_POWER_OFF)
    {
        return WDT_ERROR;    
    }
    
    /* Return stored timeout value */
    *timeout = rsc->info->timeout;

    return WDT_OK;
}

/**
  @brief        Get time left for WDT Timeout
  @return      \ref WDT_STATUS
*/
static WDT_STATUS WDT_GetTimeLeft(uint32_t *timeleft)
{
    WDT_RESOURCE *rsc = &WDT_RES;
    uint64_t read_val = 0;
    uint32_t clk_freq = Driver_PMU.GetFrequency(rsc->clockDomain);
    uint32_t ctrl_reg_val = 0;

    /* Check arguments */
    if(NULL == timeleft)
    {
        return WDT_ARGUMENT_ERROR;
    }
    
    /* Can only use in ARM_POWER_FULL */
    if(rsc->info->power_state != ARM_POWER_FULL)
    {
        return WDT_ERROR;
    }

    /* Should be called only after WDT is started, otherwise return error */
    rsc->wdtmr->WDOGLOCK = WDT_LOCK_REG_UNLOCK;
    ctrl_reg_val = rsc->wdtmr->WDOGCONTROL;
    rsc->wdtmr->WDOGLOCK = WDT_LOCK_REG_LOCK;

    /* Intr Enable and Reset Enable are only set in StartWDT */
    if(!(ctrl_reg_val & WDT_CTRL_REG_MASK))
    {
        return WDT_ERROR;
    }
    
    /* Read the current value from Current Value Register, convert to 'seconds' and return */
    rsc->wdtmr->WDOGLOCK = WDT_LOCK_REG_UNLOCK;
    read_val = rsc->wdtmr->WDOGVALUE;

    /* If Timeout interrupt has NOT been raised then,
     * time_left = Current Value (WDOGVALUE) + Load Value (WDOGLOAD)
     * else, time_left = Current Value (WDOGVALUE)
     */
    if(!(rsc->wdtmr->WDOGRIS & WDT_INTR_MASK)  )
    {
        read_val += rsc->info->load + 1;
    }
    rsc->wdtmr->WDOGLOCK = WDT_LOCK_REG_LOCK;

    /* Convert to seconds */
    *timeleft = (uint32_t)(read_val /clk_freq);

    return WDT_OK;    
}

/**
  @brief        Start WDT
  @return      \ref WDT_STATUS
*/
static WDT_STATUS WDT_Start(void)
{
    WDT_RESOURCE *rsc = &WDT_RES;
    uint32_t intr_enable = 1;
    uint32_t sys_reset_enable = 1;
    uint32_t clk_freq = Driver_PMU.GetFrequency(rsc->clockDomain);
    
    /* Can only start in ARM_POWER_FULL */
    if(rsc->info->power_state != ARM_POWER_FULL)
    {
        return WDT_ERROR;
    }
    
    /* WDT runs the counter twice, when the first period ends it raises an interrupt
     * and starts the countdown again, when the second period ends if earlier interrupt is NOT yet cleared
     * WDT raises the System Reset signal
     * Hence, load_val should be halved (to account for two countdowns)
     */
    rsc->info->load = ((clk_freq/2) * rsc->info->timeout) - 1;

    /* Steps:
     * 1) Unlock Lock Register 
     * 2) Write Timeout value to Load Regigster
     * 3) Enable Interrupt in Control Register
     * 4) Enable System Reset Request in Control Register
     * 5) Lock the Lock Register
     */
    rsc->wdtmr->WDOGLOCK = WDT_LOCK_REG_UNLOCK;
    rsc->wdtmr->WDOGLOAD = rsc->info->load;
    rsc->wdtmr->WDOGCONTROL = ((sys_reset_enable << 1) | (intr_enable));
    rsc->wdtmr->WDOGLOCK = WDT_LOCK_REG_LOCK;

    return WDT_OK;
}

/**
  @brief        Stop WDT
  @return      \ref WDT_STATUS
*/
static WDT_STATUS WDT_Stop(void)
{
    WDT_RESOURCE *rsc = &WDT_RES;
    uint32_t intr_enable = 0; /* disable */
    uint32_t sys_reset_enable = 0; /* disable */

    /* Can only stop in ARM_POWER_FULL */	
    if(rsc->info->power_state != ARM_POWER_FULL)
    {
        return WDT_ERROR;
    }
    
    /* Disable Interrupt in Control Register, counter will stop immediately 
     * Additionally disabling System Reset Request also, can be enabled while re-starting
     */
    rsc->wdtmr->WDOGLOCK = WDT_LOCK_REG_UNLOCK;
    rsc->wdtmr->WDOGCONTROL = ((sys_reset_enable << 1) | (intr_enable));
    rsc->wdtmr->WDOGLOCK = WDT_LOCK_REG_LOCK;

    return WDT_OK;
}


/**
  @brief        Clear WDT Timeout Interrupt
  @return      \ref WDT_STATUS
*/
static WDT_STATUS WDT_KeepAlive(void)
{
    WDT_RESOURCE *rsc = &WDT_RES;
    
    /* Can use only in ARM_POWER_FULL */
    if(rsc->info->power_state != ARM_POWER_FULL)
    {
        return WDT_ERROR;
    }
    
    rsc->wdtmr->WDOGLOCK = WDT_LOCK_REG_UNLOCK;
    /* If called after Timeout Interrupt then clear Interrupt and enable future interrupts (was disabled in IRQ Handler) */
    if(rsc->wdtmr->WDOGRIS & WDT_INTR_MASK )
    {
        rsc->wdtmr->WDOGINTCLR = 0x1; /* write any value to clear interrupt */
        /* Enable WDT IRQ */
        NVIC_ClearPendingIRQ(rsc->irq);
        NVIC_EnableIRQ(rsc->irq);
    }
    else /* Called before Interrupt occured, so just kick the WDT */
    {
        rsc->wdtmr->WDOGLOAD = rsc->info->load;
    }
    rsc->wdtmr->WDOGLOCK = WDT_LOCK_REG_LOCK;

    return WDT_OK;
}

/**
  @brief        WDT IRQ Handler
  @return      \ref None
*/
void WDT_IRQHandler(void)
{
    WDT_RESOURCE *rsc = &WDT_RES;

    /* Intimate the Callback function */
    rsc->info->cb_event(WDT_EVENT_TIMEOUT);

    /* Disable WDT IRQ */
    NVIC_ClearPendingIRQ(rsc->irq);
    NVIC_DisableIRQ(rsc->irq);    

    return;
}

/* WDT Driver Control Block */
TZ10XX_DRIVER_WDT Driver_WDT = {
    WDT_GetVersion,
    WDT_GetCapabilities,
    WDT_Initialize,
    WDT_Uninitialize,
    WDT_PowerControl,
    WDT_SetTimeoutInterval,
    WDT_GetTimeoutInterval,
    WDT_GetTimeLeft,
    WDT_Start,
    WDT_Stop,
    WDT_KeepAlive
};
#endif

