/**
 * @file SRAMC_TZ10xx.c
 * @brief a header file for TZ10xx SRAMC driver
 * @date $Date:: 2014-10-20 14:51:25 +0900 #$
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */

#include "SRAMC_TZ10xx.h"
#include "RTE_Device.h"
#include "TZ10xx.h"

#define SRAMC_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 12)   /* driver version */

#define  SRAMC_STATUS_STOP   0
#define  SRAMC_STATUS_TRANS  1
#define  SRAMC_STATUS_WAIT  	2

/* SRAMC Initialize status */
typedef enum _SRAMC_INIT_STATUS {
  SRAMC_UN_INITIALIZED    = 0,     
  SRAMC_INITIALIZED= 1,  
} SRAMC_INIT_STATUS;

/* SRAMC transfer flag */
typedef enum _SRAMC_FLAG {
  SRAMC_FLAG_START   	 = 1,     
  SRAMC_FLAG_END						= 2,  
} SRAMC_FLAG;

 /* SRAMC Information  */
typedef struct {
	SRAMC_INIT_STATUS       init;    // Initialize State
	ARM_POWER_STATE        power;                // Power mode
	volatile int32_t 							flag;
  SRAMC_SignalEvent_t     	 cb_event;         // Event Callback
} SRAMC_INFO;

/* SRAMC Resources definition */
typedef struct {
  SRAMC_INFO         *info;                 //< SRAMC information
} const SRAMC_RESOURCES;

static SRAMC_INFO sramc_info = {
	SRAMC_UN_INITIALIZED,
	ARM_POWER_OFF,
};

static SRAMC_RESOURCES sramc_res = {	
	&sramc_info
};

/**
  \fn          static void SRAMC_SetNVICEnable(void)
  \brief       set NVIC enable.
\return      void
*/
static void SRAMC_SetNVICEnable(void)
{
	NVIC_ClearPendingIRQ(SRAM_M2M_IRQn);
	NVIC_EnableIRQ(SRAM_M2M_IRQn);
	NVIC_ClearPendingIRQ(SRAM_ERR_IRQn);
	NVIC_EnableIRQ(SRAM_ERR_IRQn);
	return;
}
/**
  \fn          static void SRAMC_SetNVICDisable(void)
  \brief       set NVIC disable.
\return      void
*/
static void SRAMC_SetNVICDisable(void)
{
	NVIC_DisableIRQ(SRAM_M2M_IRQn);
	NVIC_ClearPendingIRQ(SRAM_M2M_IRQn);
	NVIC_DisableIRQ(SRAM_ERR_IRQn);
	NVIC_ClearPendingIRQ(SRAM_ERR_IRQn);
	return;
}

/**
  \fn          static void SRAMC_StopTransfer (void)
  \brief       write stop to register.
	\return      void
*/
static void SRAMC_StopTransfer (void)
{
	volatile uint32_t tmp;
	
	sramc->M2MINTEN_b.EN = 0;
	sramc->ERRINTEN_b.EN = 0;
	sramc_res.info->flag = SRAMC_FLAG_END;
	
	sramc->M2MSTOP_b.STOP = 1;
	while(1) {
		tmp = sramc->M2MSTATUS_b.STATUS;
		if(SRAMC_STATUS_TRANS != tmp) break; 
	}  
	 
	if(SRAMC_STATUS_WAIT == tmp) {
		sramc->M2MINTCLR_b.CLR = 1; /* clear interrupt */
		while(1) {
			tmp = sramc->M2MSTATUS_b.STATUS;
			if(SRAMC_STATUS_STOP == tmp) break; 
		}  
	}
	
	return ;	
}

	/**
  \fn          static SRAMC_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRV_VERSION
*/
static ARM_DRIVER_VERSION SRAMC_GetVersion(void)
{
  static const ARM_DRIVER_VERSION driver_version = {
    SRAMC_API_VERSION,
    SRAMC_DRV_VERSION
  };
  return driver_version;
}

/**
  \fn          static SRAMC_CAPABILITIES SRAMC_GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref SRAMC_CAPABILITIES
*/
static SRAMC_CAPABILITIES SRAMC_GetCapabilities(void)
{
  SRAMC_CAPABILITIES capabilities = {0};
	return capabilities;
}

/**
  \fn          static SRAMC_STATUS SRAMC_PowerControl (ARM_POWER_STATE  state)
  \brief       Controls SRAMC Interface Power.
  \param[in]   state    Power state
  \return      \ref SRAMC_STATUS
*/
static SRAMC_STATUS SRAMC_PowerControl (ARM_POWER_STATE  state)
{
 if(SRAMC_UN_INITIALIZED == sramc_res.info->init) return SRAMC_ERROR;
	
 switch (state) {
    case ARM_POWER_OFF:
    case ARM_POWER_LOW:
			if(ARM_POWER_FULL == sramc_res.info->power) {
				SRAMC_StopTransfer ();	
			}
			break;
    case ARM_POWER_FULL:
			break;
 		default:
			return SRAMC_ERROR;
	}
 	sramc_res.info->power = state;
  return SRAMC_OK;
}

/**
  \fn          static SRAMC_STATUS SRAMC_Initialize (SRAMC_SignalEvent_t cb_event)
  \brief       Initialize SRAMC Interface.
  \param[in]   cb_event     Pointer to SRAMC_SignalEvent
	\return      \ref SRAMC_STATUS
*/
static SRAMC_STATUS SRAMC_Initialize (SRAMC_SignalEvent_t cb_event)
{
	if (SRAMC_UN_INITIALIZED != sramc_res.info->init) return SRAMC_ERROR;

	sramc_res.info->init = SRAMC_INITIALIZED;
	
	// Clear param
	sramc_res.info->flag = SRAMC_FLAG_END;
	sramc_res.info->cb_event = cb_event;
	SRAMC_SetNVICEnable(); 
	
	return SRAMC_PowerControl(ARM_POWER_LOW);
}

/**
  \fn          static SRAMC_STATUS SRAMC_Uninitialize (void)
  \brief       De-initialize SRAMC Interface.
  \return      \ref SRAMC_STATUS
*/
static SRAMC_STATUS SRAMC_Uninitialize (void)
{
	if(SRAMC_UN_INITIALIZED != sramc_res.info->init) {
		SRAMC_PowerControl(ARM_POWER_OFF);
	}

  // Clear param
	SRAMC_SetNVICDisable(); 
	sramc_res.info->cb_event = NULL;
	sramc_res.info->flag = SRAMC_FLAG_END;
	
	sramc_res.info->init = SRAMC_UN_INITIALIZED;

  return SRAMC_OK;
}

/**
  \fn          static SRAMC_STATUS SRAMC_StartTransfer (uint8_t * dst, const uint8_t *src, uint32_t size)
  \brief       start translation from SRAM to SRAM .
	\param[out]   dst    pointer to dst
	\param[in]   src    pointer to src 
 	\param[in]   size    data size
 \return      \ref SRAMC_STATUS
*/
static SRAMC_STATUS SRAMC_StartTransfer (uint8_t * dst, const uint8_t *src, uint32_t size)
{
	volatile uint32_t tmp = 0;
	
	if(SRAMC_UN_INITIALIZED == sramc_res.info->init)	return SRAMC_ERROR;
	if (ARM_POWER_FULL != sramc_res.info->power) return SRAMC_ERROR;
	if (0x3ffff < size) return SRAMC_ERROR;	

	tmp = sramc->M2MSTATUS_b.STATUS;
	if(SRAMC_STATUS_STOP != tmp) return SRAMC_ERROR; 

	sramc->M2MDSTADDR_b.ADDR = (uint32_t)dst >> 2;
	sramc->M2MSRCADDR_b.ADDR = (uint32_t)src >> 2;
	sramc->M2MSIZE_b.SIZE = size >> 2;

	sramc->M2MINTEN_b.EN = 1;
	sramc->ERRINTEN_b.EN = 0;
	sramc->ERRCLR_b.CLR = 0xf; 
	sramc_res.info->flag = SRAMC_FLAG_START;
	sramc->M2MSTART_b.START = 1;
	
	{
		volatile uint32_t i = 0;
		while(i<4) {
			tmp = sramc->M2MSTATUS_b.STATUS;
			if(SRAMC_STATUS_STOP != tmp) {
				break;
			}
			tmp = sramc->ERRSTATUS_b.STATUS;
			if(tmp&0x8) {
				sramc_res.info->flag = SRAMC_FLAG_END;
				return SRAMC_ERROR;
			}
			if(SRAMC_FLAG_END == sramc_res.info->flag) {
				return SRAMC_OK;
			}
			i++;
		}
	}	
	sramc->ERRCLR_b.CLR = 0xf;
	sramc->ERRINTEN_b.EN = 1;
	tmp = sramc->M2MSTATUS_b.STATUS;
	if((SRAMC_STATUS_STOP == tmp) && (SRAMC_FLAG_START == sramc_res.info->flag)) {
		sramc->M2MSIZE_b.SIZE = 0;
		sramc->M2MSTART_b.START = 1;
	}
	return SRAMC_OK;
}

/**
  \fn          static SRAMC_STATUS SRAMC_AbortTransfer (void)
  \brief       stop translation from SRAM to SRAM.
 \return      \ref SRAMC_STATUS
*/
static SRAMC_STATUS SRAMC_AbortTransfer (void)
{
	if(SRAMC_UN_INITIALIZED == sramc_res.info->init)	return SRAMC_ERROR;
	if (ARM_POWER_FULL != sramc_res.info->power) return SRAMC_ERROR;

	SRAMC_StopTransfer();
	
	return SRAMC_OK;	
}

/**
  \fn          void SRAM_M2M_IRQHandler (void)
  \brief       Interrupt handler.
  \return      void 
*/
void SRAM_M2M_IRQHandler(void)
{
	volatile uint32_t tmp;
	
	sramc->M2MINTCLR_b.CLR = 1;
	while(1) {
		tmp = sramc->M2MSTATUS_b.STATUS;
		if(SRAMC_STATUS_STOP == tmp) break; 
	}  
	if(SRAMC_FLAG_START == sramc_res.info->flag) {
		sramc->M2MINTEN_b.EN = 0;
		sramc_res.info->flag = SRAMC_FLAG_END;
		if (NULL != sramc_res.info->cb_event) {
 	    sramc_res.info->cb_event(SRAMC_EVENT_TRANSFER);
		} 
	}
}

/**
  \fn          void SRAM_ERR_IRQHandler (void)
  \brief       ErrorInterrupt handler.
  \return      void 
*/
void SRAM_ERR_IRQHandler(void)
{
	if(SRAMC_FLAG_START == sramc_res.info->flag) {
		sramc->M2MINTEN_b.EN = 0;
		sramc_res.info->flag = SRAMC_FLAG_END;
		if (NULL != sramc_res.info->cb_event) {
			sramc_res.info->cb_event(SRAMC_EVENT_ERROR);
		}	
	}
	sramc->ERRCLR_b.CLR = 0xf; 
}

/* SRAMC Driver Control Block */
TZ10XX_DRIVER_SRAMC Driver_SRAMC = {
	SRAMC_GetVersion,
	SRAMC_GetCapabilities,
	SRAMC_Initialize,
	SRAMC_Uninitialize,
	SRAMC_PowerControl ,
	SRAMC_StartTransfer,
	SRAMC_AbortTransfer
};
 

